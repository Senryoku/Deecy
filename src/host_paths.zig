var context: ?struct {
    data_path: []const u8,
    userdata_path: []const u8,

    root_dir: std.Io.Dir,
    data_dir: std.Io.Dir,
    userdata_dir: std.Io.Dir,
} = null;

pub fn init(io: std.Io, allocator: std.mem.Allocator, environ: *const std.process.Environ.Map) !void {
    std.debug.assert(context == null);
    var path_buffer: [std.fs.max_path_bytes]u8 = undefined;

    const root_path = if (path_config.use_appdata_dir) dir: {
        const app_data_dir = try known_folders.getPath(io, allocator, environ, .local_configuration) orelse {
            std.log.warn("No known 'local_configuration' folder.", .{});
            return error.MissingKnownFolder;
        };
        defer allocator.free(app_data_dir);

        var fixed_allocator = std.heap.FixedBufferAllocator.init(&path_buffer);
        break :dir try std.fs.path.join(fixed_allocator.allocator(), &[_][]const u8{ app_data_dir, "Deecy" });
    } else dir: {
        const path_len = try std.process.executableDirPath(io, &path_buffer);
        break :dir path_buffer[0..path_len];
    };
    std.log.info("Using directory '{s}'", .{root_path});

    const data_path = try std.fs.path.resolve(allocator, &[_][]const u8{ root_path, path_config.data_path });
    const userdata_path = try std.fs.path.resolve(allocator, &[_][]const u8{ root_path, path_config.userdata_path });

    try ensure_dir(io, root_path);
    try ensure_dir(io, data_path);
    try ensure_dir(io, userdata_path);

    context = .{
        .root_dir = try std.Io.Dir.openDirAbsolute(io, root_path, .{}),
        .data_dir = try std.Io.Dir.openDirAbsolute(io, data_path, .{}),
        .userdata_dir = try std.Io.Dir.openDirAbsolute(io, userdata_path, .{}),
        .data_path = data_path,
        .userdata_path = userdata_path,
    };
}

/// Make sure `path` exists by calling createDirAbsolute. Does not return an error if `path` already exists.
fn ensure_dir(io: std.Io, path: []const u8) !void {
    std.Io.Dir.createDirAbsolute(io, path, .default_dir) catch |err| switch (err) {
        error.PathAlreadyExists => {},
        else => |e| return e,
    };
}

pub fn deinit(io: std.Io, allocator: std.mem.Allocator) void {
    if (context) |ctx| {
        ctx.userdata_dir.close(io);
        ctx.data_dir.close(io);
        ctx.root_dir.close(io);
        allocator.free(ctx.userdata_path);
        allocator.free(ctx.data_path);
    }
    context = null;
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

/// Replaces invalid characters with underscores in place.
pub fn safe_filename(path: []u8) void {
    for (path) |*c| {
        switch (c.*) {
            '0'...'9', 'A'...'Z', 'a'...'z', '.', '[', ']', '{', '}', '-' => {},
            else => c.* = '_',
        }
    }
}

/// Absolute path to data folder.
pub fn get_data_path() []const u8 {
    return context.?.data_path;
}

/// Absolute path to userdata folder.
pub fn get_userdata_path() []const u8 {
    return context.?.userdata_path;
}

/// Root directory of saved files.
///   Executable directory if not using appdata.
pub fn root() std.Io.Dir {
    return context.?.root_dir;
}

pub fn data() std.Io.Dir {
    return context.?.data_dir;
}

pub fn userdata() std.Io.Dir {
    return context.?.userdata_dir;
}

/// Caller owns the returned memory.
pub fn game_directory_path(allocator: std.mem.Allocator, uid: ProductUID) ![]const u8 {
    const folder_name = try std.fmt.allocPrint(allocator, "{s}[{s}]", .{ uid.name, uid.id });
    defer allocator.free(folder_name);
    safe_filename(folder_name);
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
