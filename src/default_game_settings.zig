const Game = struct {
    /// Nice name
    name: []const u8,
    /// Title from IP.BIN
    product_name: []const u8,
    /// Version from IP.BIN
    product_id: []const u8,
};

const BuiltinCheat = struct {
    name: []const u8,
    enabled: bool = false,
    actions: []const cheats.Action,

    pub fn dupe(self: @This(), allocator: std.mem.Allocator) !cheats.Cheat {
        return .{
            .name = try allocator.dupe(u8, self.name),
            .enabled = self.enabled,
            .actions = try allocator.dupe(cheats.Action, self.actions),
        };
    }
};

const Builtin = struct {
    game: Game,
    settings: ?Partial(GameSettings) = null,
    cheats: []const BuiltinCheat = &.{},
};

const Builtins: []const Builtin = if (builtin.mode == .Debug) &.{} else @import("default_game_settings.zon");

pub fn get(product_name: []const u8, product_id: []const u8) ?Builtin {
    if (builtin.mode == .Debug) {
        // Load from disc at runtime in debug mode
        var threaded: std.Io.Threaded = .init_single_threaded;
        const io = threaded.io();
        const file = std.Io.Dir.cwd().readFileAllocOptions(io, "./src/default_game_settings.zon", std.heap.page_allocator, .unlimited, .@"8", 0) catch |err| {
            std.log.err("Failed to read default_game_settings.zon: {}", .{err});
            return null;
        };
        defer std.heap.page_allocator.free(file);
        const zon = std.zon.parse.fromSliceAlloc([]const Builtin, std.heap.page_allocator, file, null, .{ .ignore_unknown_fields = true, .free_on_error = true }) catch |err| {
            std.log.err("Failed to parse default_game_settings.zon: {}", .{err});
            return null;
        };
        defer std.zon.parse.free(std.heap.page_allocator, zon);
        for (zon) |entry| {
            if (std.mem.eql(u8, entry.game.product_name, product_name) and std.mem.eql(u8, entry.game.product_id, product_id))
                return entry;
        }
        return null;
    }

    for (Builtins) |entry| {
        if (std.mem.eql(u8, entry.game.product_name, product_name) and std.mem.eql(u8, entry.game.product_id, product_id)) {
            return entry;
        }
    }
    return null;
}

const std = @import("std");
const builtin = @import("builtin");
const cheats = @import("cheats.zig");
const GameSettings = @import("GameSettings.zig");
const Partial = @import("helpers").Partial;
const HostPaths = @import("dreamcast").HostPaths;
