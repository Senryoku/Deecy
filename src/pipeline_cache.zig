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
    var name_buf: [64]u8 = undefined;
    const hex_name = try cache_file_name(key, &name_buf);

    const userdata_path = HostPaths.get_userdata_path();
    const path = try std.fs.path.join(allocator, &.{ userdata_path, CacheDir, hex_name });
    defer allocator.free(path);

    const file = try std.fs.cwd().openFile(path, .{});
    defer file.close();

    const size = try file.getEndPos();

    if (value_ptr == null) return @intCast(size);

    const buffer: []u8 = @ptrCast(value_ptr.?[0..value_size]);
    return try file.readAll(buffer);
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
    var name_buf: [64]u8 = undefined;
    const hex_name = try cache_file_name(key, &name_buf);

    const userdata_path = HostPaths.get_userdata_path();
    const dir = try std.fs.path.join(allocator, &.{ userdata_path, CacheDir });
    defer allocator.free(dir);
    try std.fs.cwd().makePath(dir);

    const path = try std.fs.path.join(allocator, &.{ dir, hex_name });
    defer allocator.free(path);

    const file = try std.fs.cwd().createFile(path, .{});
    defer file.close();
    try file.writeAll(value);
}

pub fn clear(allocator: std.mem.Allocator) !void {
    const userdata_path = HostPaths.get_userdata_path();
    const dir = try std.fs.path.join(allocator, &.{ userdata_path, CacheDir });
    defer allocator.free(dir);
    log.warn("Deleting {s}", .{dir});
    try std.fs.cwd().deleteTree(dir);
    try std.fs.cwd().makePath(dir);
}
