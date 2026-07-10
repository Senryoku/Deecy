//! Game specific Settings
rendering: @import("renderer.zig").GameSettings = .{},

pub fn save(self: @This(), io: std.Io, allocator: std.mem.Allocator, product_name: []const u8, product_id: []const u8) !void {
    const file_path = try path(allocator, product_name, product_id);
    defer allocator.free(file_path);

    if (std.fs.path.dirname(file_path)) |dir| try std.Io.Dir.cwd().createDirPath(io, dir);

    const file = try std.Io.Dir.cwd().createFile(io, file_path, .{});
    defer file.close(io);
    const buffer = try allocator.alloc(u8, 8192);
    defer allocator.free(buffer);
    var writer = file.writer(io, buffer);
    try std.zon.stringify.serialize(self, .{}, &writer.interface);
    try writer.end();
}

pub fn load(io: std.Io, allocator: std.mem.Allocator, product_name: []const u8, product_id: []const u8) !@This() {
    const file_path = try path(allocator, product_name, product_id);
    defer allocator.free(file_path);

    const settings_str = std.Io.Dir.cwd().readFileAllocOptions(io, file_path, allocator, .limited(8 * 1024 * 1024), .@"8", 0) catch |err| {
        switch (err) {
            error.FileNotFound => {
                if (Default.get(product_name, product_id)) |builtin| {
                    if (builtin.settings) |settings|
                        return helpers.to_complete(@This(), settings);
                }
                return .{};
            },
            else => return err,
        }
    };
    defer allocator.free(settings_str);

    const zon = std.zon.parse.fromSlice(Partial(@This()), allocator, settings_str, null, .{ .ignore_unknown_fields = true, .free_on_error = true }) catch |err| {
        log.err("Failed to parse game settings file '{s}': {t}.", .{ file_path, err });
        return .{};
    };
    return helpers.to_complete(@This(), zon);
}

/// Caller owns the returned memory
fn path(allocator: std.mem.Allocator, product_name: []const u8, product_id: []const u8) ![]const u8 {
    const game_dir = try HostPaths.userdata_game_directory(allocator, product_name, product_id);
    defer allocator.free(game_dir);
    return try std.fs.path.join(allocator, &[_][]const u8{ game_dir, "settings.zon" });
}

const std = @import("std");
const log = std.log.scoped(.game_settings);

const helpers = @import("helpers");
const Partial = helpers.Partial;

const HostPaths = @import("dreamcast").HostPaths;
const Default = @import("default_game_settings.zig");
