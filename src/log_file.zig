const std = @import("std");
const HostPaths = @import("dreamcast").HostPaths;

pub var writer: *std.Io.Writer = undefined;

var file: std.fs.File = undefined;
var file_writer: std.fs.File.Writer = undefined;
var buffer: [128]u8 = undefined;
var mutex: std.Thread.Mutex = .{};
var _opened: bool = false;

/// Noop if already open
pub fn open(allocator: std.mem.Allocator) !void {
    if (!opened()) {
        mutex.lock();
        defer mutex.unlock();
        const path = try get_path(allocator);
        defer allocator.free(path);
        file = try std.fs.cwd().createFile(path, .{});
        file_writer = file.writer(&buffer);
        writer = &file_writer.interface;
        _opened = true;
    }
}

/// Noop if not open
pub fn close() void {
    if (opened()) {
        mutex.lock();
        defer mutex.unlock();
        _opened = false;
        writer.flush() catch {};
        file.close();
    }
}

pub fn opened() bool {
    return _opened;
}

/// Noop if disabled
pub fn lock() void {
    if (opened()) mutex.lock();
}

/// Noop if disabled
pub fn unlock() void {
    if (opened()) mutex.unlock();
}

pub fn get_path(allocator: std.mem.Allocator) ![]const u8 {
    return std.fs.path.join(allocator, &.{ HostPaths.get_userdata_path(), "deecy.log" });
}
