const std = @import("std");
const builtin = @import("builtin");
const log = std.log.scoped(.memory_mapped_file);

file: std.Io.File,
views: std.ArrayList(std.Io.File.MemoryMap) = .empty,
allocator: std.mem.Allocator,
io: std.Io,

pub fn init(allocator: std.mem.Allocator, io: std.Io, filepath: []const u8) !@This() {
    return .{
        .file = std.Io.Dir.cwd().openFile(io, filepath, .{}) catch |err| {
            log.err("Error opening file '{s}': {t}", .{ filepath, err });
            return err;
        },
        .allocator = allocator,
        .io = io,
    };
}

pub fn deinit(self: *@This()) void {
    for (self.views.items) |*view|
        view.destroy(self.io);
    self.views.deinit(self.allocator);
    self.file.close(self.io);
}

pub fn create_full_view(self: *@This()) ![]const u8 {
    return try self.create_view(0, try self.file_size());
}

/// FIXME: After switching to using zig 0.16.0 std.Io.File.MemoryMap, I don't think this works properly outside of creating a full view anymore.
///        Removing the only external use was easier than figuring it out. Fix before using :D
fn create_view(self: *@This(), offset: u64, size: u64) ![]const u8 {
    const alignment = std.heap.pageSize();
    const aligned_offset = std.mem.alignBackward(u64, offset, alignment);
    const adjustment = offset - aligned_offset;
    const adjusted_size = size + adjustment;
    const map = try std.Io.File.MemoryMap.create(self.io, self.file, .{
        .len = adjusted_size,
        .protection = .{ .read = true, .write = false },
        // NOTE : Don't read-ahead on Linux.
        .populate = false,
        .offset = aligned_offset,
    });
    try self.views.append(self.allocator, map);
    return map.memory[adjustment..][0..size];
}

pub fn file_size(self: *const @This()) !u64 {
    return (try self.file.stat(self.io)).size;
}
