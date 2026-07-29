var data_path: []const u8 = "";
var userdata_path: []const u8 = "";

var root_dir: std.Io.Dir = undefined;
var data_dir: std.Io.Dir = undefined;
var userdata_dir: std.Io.Dir = undefined;

pub fn init(io: std.Io, allocator: std.mem.Allocator, environ: std.process.Environ.Map) !void {
    var path_buffer: [std.fs.max_path_bytes]u8 = undefined;

    const root_path = if (path_config.use_appdata_dir) dir: {
        const app_data_dir = try known_folders.getPath(io, allocator, environ, .local_configuration) orelse {
            std.log.warn("No known 'local_configuration' folder.", .{});
            return error.MissingKnownFolder;
        };
        defer allocator.free(app_data_dir);

        const deecy_folder = try std.fs.path.join(allocator, &[_][]const u8{ app_data_dir, "Deecy" });
        defer allocator.free(deecy_folder);

        break :dir deecy_folder;
    } else dir: {
        const path_len = try std.process.executableDirPath(io, &path_buffer);
        break :dir path_buffer[0..path_len];
    };
    std.log.info("Using directory '{s}'", .{root_path});

    root_dir = try std.Io.Dir.openDirAbsolute(io, root_path, .{});
    data_path = try std.fs.path.resolve(allocator, &[_][]const u8{ root_path, path_config.data_path });
    userdata_path = try std.fs.path.resolve(allocator, &[_][]const u8{ root_path, path_config.userdata_path });

    try ensure_dir(io, root_path);
    try ensure_dir(io, data_path);
    try ensure_dir(io, userdata_path);

    root_dir = try std.Io.Dir.openDirAbsolute(io, root_path, .{});
    data_dir = try std.Io.Dir.openDirAbsolute(io, get_data_path(), .{});
    userdata_dir = try std.Io.Dir.openDirAbsolute(io, get_userdata_path(), .{});
}

/// Make sure `path` exists by calling createDirAbsolute. Does not return an error if `path` already exists.
fn ensure_dir(io: std.Io, path: []const u8) !void {
    std.Io.Dir.createDirAbsolute(io, path, .default_dir) catch |err| switch (err) {
        error.PathAlreadyExists => {},
        else => |e| return e,
    };
}

pub fn deinit(io: std.Io, allocator: std.mem.Allocator) void {
    userdata_dir.close(io);
    data_dir.close(io);
    root_dir.close(io);

    allocator.free(data_path);
    allocator.free(userdata_path);
}

/// Replaces invalid characters with underscores in place.
pub fn safe_path(path: []u8) void {
    for (path) |*c| {
        switch (c.*) {
            '0'...'9', 'A'...'Z', 'a'...'z', '.', '[', ']', '{', '}', '-', '/' => {},
            else => c.* = '_',
        }
    }
}

/// Absolute path to data folder.
pub fn get_data_path() []const u8 {
    return data_path;
}

/// Absolute path to userdata folder.
pub fn get_userdata_path() []const u8 {
    return userdata_path;
}

/// Root directory of saved files.
///   Executable directory if not using appdata.
pub fn root() std.Io.Dir {
    return root_dir;
}

pub fn data() std.Io.Dir {
    return data_dir;
}

pub fn userdata() std.Io.Dir {
    return userdata_dir;
}

/// Caller owns the returned memory.
pub fn game_directory_path(allocator: std.mem.Allocator, uid: ProductUID) ![]const u8 {
    const folder_name = try std.fmt.allocPrint(allocator, "{s}[{s}]", .{ uid.name, uid.id });
    safe_path(folder_name);
    defer allocator.free(folder_name);
    const path = try std.fs.path.join(allocator, &[_][]const u8{ get_userdata_path(), folder_name });
    return path;
}

/// Caller should `close` the returned directory.
pub fn game_directory(io: std.Io, allocator: std.mem.Allocator, uid: ProductUID) !std.Io.Dir {
    const path = try game_directory_path(allocator, uid);
    defer allocator.free(path);
    try ensure_dir(io, path);
    return std.Io.Dir.openDirAbsolute(io, path, .{});
}

const std = @import("std");
pub const path_config = @import("path_config");
const known_folders = @import("known-folders");
const ProductUID = @import("ProductUID.zig");
