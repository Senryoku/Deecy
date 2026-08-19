const Signature: u32 = 0x43444247;
const Version: u32 = 3;

const Entry = struct {
    path: []const u8,
    name: []const u8,
    product_name: []const u8,
    product_id: []const u8,
    image_size: struct { width: u32, height: u32 },
    image: ?[]const u8,
};

map: std.StringHashMap(Entry),

_path: []const u8,
_mutex: std.Io.Mutex = .init,
_arena: std.heap.ArenaAllocator,

pub fn create(allocator: std.mem.Allocator) !*@This() {
    var r = try allocator.create(@This());
    r.* = .{
        .map = undefined,
        ._path = undefined,
        ._arena = std.heap.ArenaAllocator.init(allocator),
    };
    r.map = std.StringHashMap(Entry).init(r._arena.allocator());
    r._path = try get_path(r._arena.allocator());
    return r;
}

pub fn destroy(self: *@This(), allocator: std.mem.Allocator) void {
    self._arena.deinit();
    allocator.destroy(self);
}

fn get_path(allocator: std.mem.Allocator) ![]const u8 {
    return try std.fs.path.join(allocator, &[_][]const u8{ host_paths.get_userdata_path(), "game_info_cache" });
}

pub fn load(self: *@This(), io: std.Io) !void {
    const data = try host_paths.root().readFileAlloc(io, self._path, self._arena.allocator(), .unlimited);
    defer self._arena.allocator().free(data);
    try self.deserialize(data);
}

/// Thread safe
pub fn get(self: *@This(), io: std.Io, path: []const u8) ?Entry {
    self._mutex.lock(io) catch return null;
    defer self._mutex.unlock(io);
    return self.map.get(path);
}

/// Thread safe
/// Copies all inputs (including image) to owned memory.
pub fn add(self: *@This(), io: std.Io, path: []const u8, product_name: []const u8, product_id: []const u8, image_size: struct { width: u32, height: u32 }, image: ?[]const u8) !void {
    try self._mutex.lock(io);
    defer self._mutex.unlock(io);
    const arena_alloc = self._arena.allocator();
    const duped_path = try arena_alloc.dupe(u8, path);
    try self.map.put(duped_path, .{
        .path = duped_path,
        .name = try arena_alloc.dupe(u8, std.fs.path.basename(path)),
        .product_name = try arena_alloc.dupe(u8, product_name),
        .product_id = try arena_alloc.dupe(u8, product_id),
        .image_size = .{ .width = image_size.width, .height = image_size.height },
        .image = if (image) |i| try arena_alloc.dupe(u8, i) else null,
    });
}

pub fn save_to_disk(self: *@This(), io: std.Io) !void {
    var file = try host_paths.root().createFile(io, self._path, .{});
    defer file.close(io);

    var allocating_writer = std.Io.Writer.Allocating.init(self._arena.allocator());
    defer allocating_writer.deinit();
    var uncompressed_writer = &allocating_writer.writer;

    try uncompressed_writer.writeInt(u32, self.map.count(), .little);
    var it = self.map.iterator();
    while (it.next()) |entry| {
        const value = entry.value_ptr.*;
        try write_string(uncompressed_writer, value.path);
        try write_string(uncompressed_writer, value.name);
        try write_string(uncompressed_writer, value.product_name);
        try write_string(uncompressed_writer, value.product_id);
        try uncompressed_writer.writeInt(u32, value.image_size.width, .little);
        try uncompressed_writer.writeInt(u32, value.image_size.height, .little);
        if (value.image) |image| try uncompressed_writer.writeAll(image);
    }
    try uncompressed_writer.flush();

    var uncompressed = allocating_writer.toArrayList();
    defer uncompressed.deinit(self._arena.allocator());

    const compressed = try lz4.Standard.compress(self._arena.allocator(), uncompressed.items);
    defer self._arena.allocator().free(compressed);

    const buffer = try self._arena.allocator().alloc(u8, 4 * 1024);
    defer self._arena.allocator().free(buffer);
    var file_writer = file.writer(io, buffer);
    var writer = &file_writer.interface;

    try writer.writeInt(u32, Signature, .little);
    try writer.writeInt(u32, Version, .little);
    try writer.writeInt(u32, @intCast(uncompressed.items.len), .little);
    try writer.writeAll(compressed);

    try writer.flush();
}

fn deserialize(self: *@This(), data: []const u8) !void {
    var header_reader = std.Io.Reader.fixed(data);

    const signature = try header_reader.takeInt(u32, .little);
    if (signature != Signature) return error.InvalidSignature;
    const version = try header_reader.takeInt(u32, .little);
    if (version != Version) return error.IncompatibleVersion;

    const uncompressed_size = try header_reader.takeInt(u32, .little);

    const decompressed = try lz4.Standard.decompress(self._arena.allocator(), data[header_reader.seek..], uncompressed_size);
    errdefer self._arena.allocator().free(decompressed); // Kept alive in success path: Entries will refer to it.

    if (decompressed.len != uncompressed_size) return error.UnexpectedDecompressedSize;

    var reader = std.Io.Reader.fixed(decompressed);

    const count = try reader.takeInt(u32, .little);
    try self.map.ensureTotalCapacity(count);

    for (0..count) |_| {
        const path = try read_string(&reader);
        const name = try read_string(&reader);
        const product_name = try read_string(&reader);
        const product_id = try read_string(&reader);
        const image_size_width = try reader.takeInt(u32, .little);
        const image_size_height = try reader.takeInt(u32, .little);
        const image_byte_size = image_size_width * image_size_height * 4;
        if (reader.buffer[reader.seek..].len < image_byte_size) return error.EndOfStream;
        const image = if (image_size_width > 0 and image_size_height > 0) reader.buffer[reader.seek..][0..image_byte_size] else null;
        reader.toss(image_byte_size);
        self.map.putAssumeCapacity(path, .{
            .path = path,
            .name = name,
            .product_name = product_name,
            .product_id = product_id,

            .image_size = .{ .width = image_size_width, .height = image_size_height },
            .image = image,
        });
    }
}

fn read_string(reader: *std.Io.Reader) ![]const u8 {
    const size = try reader.takeInt(u32, .little);
    if (reader.buffer[reader.seek..].len < size) return error.EndOfStream;
    const string = reader.buffer[reader.seek..][0..size];
    reader.toss(size);
    return string;
}

fn write_string(writer: *std.Io.Writer, string: []const u8) !void {
    try writer.writeInt(u32, @intCast(string.len), .little);
    try writer.writeAll(string);
}

const std = @import("std");
const lz4 = @import("lz4");
const host_paths = @import("../host_paths.zig");
