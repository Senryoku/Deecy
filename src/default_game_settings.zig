const Game = struct {
    /// Nice name
    name: []const u8,
    versions: []const ProductUID,
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

const Builtins: []const Builtin = @import("default_game_settings.zon");

pub fn get(uid: ProductUID) ?Builtin {
    for (Builtins) |entry| {
        for (entry.game.versions) |version| {
            if (uid.eql(version))
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
pub const ProductUID = @import("dreamcast").ProductUID;
