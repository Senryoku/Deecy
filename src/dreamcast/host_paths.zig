const std = @import("std");
pub const path_config = @import("path_config");
const known_folders = @import("known-folders");

var data_path: []const u8 = "";
var userdata_path: []const u8 = "";

pub fn init(io: std.Io, allocator: std.mem.Allocator, environ: std.process.Environ.Map) !void {
    if (path_config.use_appdata_dir) {
        const app_data_dir = try known_folders.getPath(io, allocator, environ, .local_configuration) orelse {
            std.log.warn("No known 'local_configuration' folder.", .{});
            return error.MissingKnownFolder;
        };
        defer allocator.free(app_data_dir);

        const deecy_folder = try std.fs.path.join(allocator, &[_][]const u8{ app_data_dir, "Deecy" });
        defer allocator.free(deecy_folder);
        std.log.info("Using App data folder as '{s}'", .{deecy_folder});

        data_path = try std.fs.path.resolve(allocator, &[_][]const u8{ deecy_folder, path_config.data_path });
        userdata_path = try std.fs.path.resolve(allocator, &[_][]const u8{ deecy_folder, path_config.userdata_path });

        try std.Io.Dir.cwd().createDirPath(io, deecy_folder);
        try std.Io.Dir.cwd().createDirPath(io, data_path);
        try std.Io.Dir.cwd().createDirPath(io, userdata_path);
    }
}

pub fn deinit(allocator: std.mem.Allocator) void {
    if (path_config.use_appdata_dir) {
        allocator.free(data_path);
        allocator.free(userdata_path);
    }
}

/// Replaces invalid characters with underscores
pub fn safe_path(path: []u8) void {
    for (path) |*c| {
        switch (c.*) {
            '0'...'9', 'A'...'Z', 'a'...'z', '.', '[', ']', '{', '}', '-', '/' => {},
            else => c.* = '_',
        }
    }
}

pub fn get_data_path() []const u8 {
    if (path_config.use_appdata_dir)
        return data_path;
    return path_config.data_path;
}

pub fn get_userdata_path() []const u8 {
    if (path_config.use_appdata_dir)
        return userdata_path;
    return path_config.userdata_path;
}

/// Caller owns the returned memory
pub fn userdata_game_directory(allocator: std.mem.Allocator, product_name: []const u8, product_id: []const u8) ![]const u8 {
    const folder_name = try std.fmt.allocPrint(allocator, "{s}[{s}]", .{ product_name, product_id });
    safe_path(folder_name);
    defer allocator.free(folder_name);
    const path = try std.fs.path.join(allocator, &[_][]const u8{ get_userdata_path(), folder_name });
    return path;
}
