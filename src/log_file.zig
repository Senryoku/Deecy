const std = @import("std");
const HostPaths = @import("dreamcast").HostPaths;

pub var writer: *std.Io.Writer = undefined;

var file: std.Io.File = undefined;
var file_writer: std.Io.File.Writer = undefined;
var buffer: [128]u8 = undefined;
var mutex: std.Io.Mutex = .init;
var _opened: bool = false;

/// Noop if already open
pub fn open(allocator: std.mem.Allocator) !void {
    if (!opened()) {
        const io = std.Options.debug_io;
        mutex.lockUncancelable(io);
        defer mutex.unlock(io);
        const path = try get_path(allocator);
        defer allocator.free(path);
        file = try std.Io.Dir.cwd().createFile(io, path, .{});
        file_writer = file.writer(io, &buffer);
        writer = &file_writer.interface;
        _opened = true;
    }
}

/// Noop if not open
pub fn close() void {
    if (opened()) {
        const io = std.Options.debug_io;
        mutex.lockUncancelable(io);
        defer mutex.unlock(io);
        _opened = false;
        writer.flush() catch {};
        file.close(io);
    }
}

pub fn opened() bool {
    return _opened;
}

/// Noop if disabled
pub fn lock() void {
    const io = std.Options.debug_io;
    if (opened()) mutex.lockUncancelable(io);
}

/// Noop if disabled
pub fn unlock() void {
    const io = std.Options.debug_io;
    if (opened()) mutex.unlock(io);
}

pub fn get_path(allocator: std.mem.Allocator) ![]const u8 {
    return std.fs.path.join(allocator, &.{ HostPaths.get_userdata_path(), "deecy.log" });
}
