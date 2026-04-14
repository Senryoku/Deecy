const std = @import("std");

const HostPaths = @import("dreamcast").HostPaths;
const Deecy = @import("deecy.zig");

const log = std.log.scoped(.pipeline_cache);

const CacheDir = ".pipeline_cache/";

fn cache_file_name(key: []const u8, buffer: []u8) ![]const u8 {
    return try std.fmt.bufPrint(buffer, "{x}", .{std.hash.Wyhash.hash(0, key)});
}

pub fn load_pipeline_cache(key_ptr: [*]const u8, key_size: usize, value_ptr: ?[*]u8, value_size: usize, userdata: ?*anyopaque) callconv(.c) usize {
    const maybe_app: ?*Deecy = @ptrCast(@alignCast(userdata));
    if (maybe_app) |app| {
        const key: []const u8 = @ptrCast(key_ptr[0..key_size]);
        return load_pipeline_cache_impl(app._allocator, key, value_ptr, value_size) catch |err| {
            switch (err) {
                error.FileNotFound => {}, // Cache not found, not an error case.
                else => log.err("Failed to load pipeline cache: {t}", .{err}),
            }
            return 0;
        };
    }
    return 0;
}

fn load_pipeline_cache_impl(allocator: std.mem.Allocator, key: []const u8, value_ptr: ?[*]u8, value_size: usize) !usize {
    log.debug("load_pipeline_cache: key.len: {d}, value_size: {d}", .{ key.len, value_size });

    var threaded: std.Io.Threaded = .init_single_threaded;
    const io = threaded.io();

    var name_buf: [64]u8 = undefined;
    const hex_name = try cache_file_name(key, &name_buf);

    const userdata_path = HostPaths.get_userdata_path(io);
    const path = try std.fs.path.join(allocator, &.{ userdata_path, CacheDir, hex_name });
    defer allocator.free(path);

    const file = try std.Io.Dir.cwd().openFile(io, path, .{});
    defer file.close(io);

    const size = (try file.stat(io)).size;

    // Only requesting the size.
    if (value_ptr == null or value_size == 0) return @intCast(size);

    if (size > value_size) return error.ValueBufferTooSmall;

    const buffer: []u8 = @ptrCast(value_ptr.?[0..value_size]);
    return try file.readPositionalAll(io, buffer, 0);
}

pub fn store_pipeline_cache(key_ptr: [*]const u8, key_size: usize, value_ptr: [*]const u8, value_size: usize, userdata: ?*anyopaque) callconv(.c) void {
    const maybe_app: ?*Deecy = @ptrCast(@alignCast(userdata));
    if (maybe_app) |app| {
        const key: []const u8 = @ptrCast(key_ptr[0..key_size]);
        const value: []const u8 = @ptrCast(value_ptr[0..value_size]);
        store_pipeline_cache_impl(app._allocator, key, value) catch |err| {
            log.err("Failed to store pipeline cache: {t}", .{err});
        };
    }
}

fn store_pipeline_cache_impl(allocator: std.mem.Allocator, key: []const u8, value: []const u8) !void {
    log.debug("store_pipeline_cache: key.len: {d}, value.len: {d}", .{ key.len, value.len });

    var threaded: std.Io.Threaded = .init_single_threaded;
    const io = threaded.io();

    var name_buf: [64]u8 = undefined;
    const hex_name = try cache_file_name(key, &name_buf);

    const path = try std.fs.path.join(allocator, &.{ HostPaths.get_userdata_path(io), CacheDir, hex_name });
    defer allocator.free(path);

    if (std.fs.path.dirname(path)) |dir| try std.Io.Dir.cwd().createDirPath(io, dir);

    try std.Io.Dir.cwd().writeFile(io, .{
        .sub_path = path,
        .data = value,
        .flags = .{ .truncate = true },
    });
}

pub fn clear(allocator: std.mem.Allocator) !void {
    var threaded: std.Io.Threaded = .init_single_threaded;
    const io = threaded.io();

    const userdata_path = HostPaths.get_userdata_path(io);
    const dir = try std.fs.path.join(allocator, &.{ userdata_path, CacheDir });
    defer allocator.free(dir);
    log.warn("Deleting '{s}'", .{dir});
    try std.Io.Dir.cwd().deleteTree(io, dir);
    try std.Io.Dir.cwd().createDirPath(io, dir);
}
