const std = @import("std");
const HostPaths = @import("dreamcast").HostPaths;

pub var writer: *std.Io.Writer = undefined;

var file: std.fs.File = undefined;
var file_writer: std.fs.File.Writer = undefined;
var buffer: [128]u8 = undefined;
var mutex: std.Thread.Mutex = .{};
var _enabled: bool = false;

pub fn open(allocator: std.mem.Allocator) !void {
    if (!enabled()) {
        mutex.lock();
        defer mutex.unlock();
        const path = try get_path(allocator);
        defer allocator.free(path);
        file = try std.fs.cwd().createFile(path, .{});
        file_writer = file.writer(&buffer);
        writer = &file_writer.interface;
        _enabled = true;
    }
}

pub fn close() void {
    if (enabled()) {
        mutex.lock();
        defer mutex.unlock();
        _enabled = false;
        writer.flush() catch {};
        file.close();
    }
}

pub fn enabled() bool {
    return _enabled;
}

pub fn lock() void {
    if (enabled()) mutex.lock();
}

pub fn unlock() void {
    if (enabled()) mutex.unlock();
}

pub fn get_path(allocator: std.mem.Allocator) ![]const u8 {
    return std.fs.path.join(allocator, &.{ HostPaths.get_userdata_path(), "deecy.log" });
}
