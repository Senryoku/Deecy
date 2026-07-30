//! Game specific Settings
region: ?Deecy.Configuration.Region = null,
video_cable: ?Deecy.Configuration.VideoCable = null,
bios_emulation: ?Deecy.Configuration.BiosEmulation = null,
rendering: @import("renderer.zig").GameSettings = .{},

pub fn save(self: @This(), io: std.Io, allocator: std.mem.Allocator, product_uid: Default.ProductUID) !void {
    const dir = try host_paths.game_directory(io, allocator, product_uid);
    defer dir.close(io);

    const file = try dir.createFile(io, SettingsFileName, .{});
    defer file.close(io);
    const buffer = try allocator.alloc(u8, 8192);
    defer allocator.free(buffer);
    var writer = file.writer(io, buffer);
    try std.zon.stringify.serialize(self, .{}, &writer.interface);
    try writer.end();
}

pub fn load(io: std.Io, allocator: std.mem.Allocator, product_uid: Default.ProductUID) !@This() {
    const dir = try host_paths.game_directory(io, allocator, product_uid);
    defer dir.close(io);

    const settings_str = dir.readFileAllocOptions(io, SettingsFileName, allocator, .limited(8 * 1024 * 1024), .@"8", 0) catch |err| {
        switch (err) {
            error.FileNotFound => {
                if (Default.get(product_uid)) |builtin| {
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
        log.err("Failed to parse game settings file for {f}: {t}.", .{ product_uid, err });
        return .{};
    };
    return helpers.to_complete(@This(), zon);
}

const SettingsFileName = "settings.zon";

const std = @import("std");
const log = std.log.scoped(.game_settings);

const helpers = @import("helpers");
const Partial = helpers.Partial;

const Deecy = @import("deecy.zig");
const host_paths = Deecy.host_paths;
const Default = @import("default_game_settings.zig");
