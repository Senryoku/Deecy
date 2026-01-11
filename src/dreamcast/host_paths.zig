const std = @import("std");
pub const path_config = @import("path_config");

var data_path: []const u8 = "";
var userdata_path: []const u8 = "";

var data_path_buffer: [std.fs.max_path_bytes]u8 = @splat(0);
var userdata_path_buffer: [std.fs.max_path_bytes]u8 = @splat(0);

var _mutex = if (path_config.use_appdata_dir) std.Thread.Mutex{} else void;

fn generate_appdata_path(buf: []u8, path: []const u8) []const u8 {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();
    const app_data_dir = std.fs.getAppDataDir(allocator, "Deecy") catch |err|
        std.debug.panic("Failed to get appdata dir: {t}\n", .{err});
    const p = std.fs.path.resolve(allocator, &[_][]const u8{ app_data_dir, path }) catch |err|
        std.debug.panic("Failed to resolve paths: {t}\n", .{err});
    @memcpy(buf[0..p.len], p);
    return buf[0..p.len];
}

pub fn get_data_path() []const u8 {
    if (path_config.use_appdata_dir) {
        _mutex.lock();
        defer _mutex.unlock();
        if (data_path.len == 0)
            data_path = generate_appdata_path(&data_path_buffer, path_config.data_path);
        return data_path;
    }
    return path_config.data_path;
}

pub fn get_userdata_path() []const u8 {
    if (path_config.use_appdata_dir) {
        _mutex.lock();
        defer _mutex.unlock();
        if (userdata_path.len == 0)
            userdata_path = generate_appdata_path(&userdata_path_buffer, path_config.userdata_path);
        return userdata_path;
    }
    return path_config.userdata_path;
}
