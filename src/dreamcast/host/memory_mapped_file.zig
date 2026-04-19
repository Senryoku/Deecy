const std = @import("std");
const builtin = @import("builtin");
const log = std.log.scoped(.memory_mapped_file);

file: std.Io.File,
/// The view into the file might be page aligned, this is the real file size.
size: usize,
map: std.Io.File.MemoryMap,

pub fn init(io: std.Io, filepath: []const u8) !@This() {
    const file = std.Io.Dir.cwd().openFile(io, filepath, .{}) catch |err| {
        log.err("Error opening file '{s}': {t}", .{ filepath, err });
        return err;
    };
    const size = (try file.stat(io)).size;
    const map = try std.Io.File.MemoryMap.create(io, file, .{
        .len = size,
        .protection = .{ .read = true, .write = false },
        // NOTE: Don't read-ahead on Linux. Changing the default is incompatible with Windows.
        .populate = builtin.os.tag != .linux,
        .offset = 0,
    });
    return .{
        .file = file,
        .size = size,
        .map = map,
    };
}

pub fn deinit(self: *@This(), io: std.Io) void {
    self.map.destroy(io);
    self.file.close(io);
}

pub inline fn view(self: @This()) []align(std.heap.page_size_min) const u8 {
    return self.map.memory;
}
