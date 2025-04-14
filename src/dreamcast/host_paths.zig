const std = @import("std");
pub const dc_config = @import("dc_config");

var data_path: []const u8 = "";
var userdata_path: []const u8 = "";

var data_path_buffer: [std.fs.max_path_bytes]u8 = @splat(0);
var userdata_path_buffer: [std.fs.max_path_bytes]u8 = @splat(0);

pub fn get_data_path() []const u8 {
    if (dc_config.use_appdata_dir) {
        if (data_path.len == 0) {
            const app_data_dir = std.fs.getAppDataDir(std.heap.c_allocator, "Deecy") catch |err| {
                std.debug.panic("Failed to get appdata dir: {s}\n", .{@errorName(err)});
            };
            defer std.heap.c_allocator.free(app_data_dir);
            const p = std.fs.path.resolve(std.heap.c_allocator, &[_][]const u8{ app_data_dir, dc_config.data_path }) catch |err| {
                std.debug.panic("Failed to resolve paths: {s}\n", .{@errorName(err)});
            };
            defer std.heap.c_allocator.free(p);
            @memcpy(data_path_buffer[0..p.len], p);
            data_path = data_path_buffer[0..p.len];
            std.debug.print("data_path: {s}\n", .{data_path});
        }
        return data_path;
    } else {
        return dc_config.data_path;
    }
}

pub fn get_userdata_path() []const u8 {
    if (dc_config.use_appdata_dir) {
        if (userdata_path.len == 0) {
            const app_data_dir = std.fs.getAppDataDir(std.heap.c_allocator, "Deecy") catch |err| {
                std.debug.panic("Failed to get appdata dir: {s}\n", .{@errorName(err)});
            };
            defer std.heap.c_allocator.free(app_data_dir);
            const p = std.fs.path.resolve(std.heap.c_allocator, &[_][]const u8{ app_data_dir, dc_config.userdata_path }) catch |err| {
                std.debug.panic("Failed to resolve paths: {s}\n", .{@errorName(err)});
            };
            defer std.heap.c_allocator.free(p);
            @memcpy(userdata_path_buffer[0..p.len], p);
            userdata_path = userdata_path_buffer[0..p.len];
            std.debug.print("userdata_path: {s}\n", .{userdata_path});
        }
        return userdata_path;
    } else {
        return dc_config.userdata_path;
    }
}
