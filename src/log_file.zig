const std = @import("std");
const host_paths = @import("host_paths.zig");

pub var writer: *std.Io.Writer = undefined;

var file: std.Io.File = undefined;
var file_writer: std.Io.File.Writer = undefined;
var buffer: [128]u8 = undefined;
var mutex: std.Io.Mutex = .init;
var _opened: bool = false;

var _io: std.Io = undefined;

/// Noop if already open
pub fn open(io: std.Io) !void {
    if (!opened()) {
        _io = io;

        mutex.lockUncancelable(io);
        defer mutex.unlock(io);
        file = try host_paths.userdata().createFile(io, "deecy.log", .{});
        file_writer = file.writer(io, &buffer);
        writer = &file_writer.interface;
        _opened = true;
    }
}

/// Noop if not open
pub fn close() void {
    if (opened()) {
        mutex.lockUncancelable(_io);
        defer mutex.unlock(_io);
        _opened = false;
        writer.flush() catch {};
        file.close(_io);
    }
}

pub fn opened() bool {
    return _opened;
}

/// Noop if disabled
pub fn lock() void {
    if (opened()) mutex.lockUncancelable(_io);
}

/// Noop if disabled
pub fn unlock() void {
    if (opened()) mutex.unlock(_io);
}

/// Caller owns the returned memory.
pub fn path(allocator: std.mem.Allocator) ![]const u8 {
    return std.fs.path.join(allocator, &.{ host_paths.get_userdata_path(), "deecy.log" });
}
