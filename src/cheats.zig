const std = @import("std");
const log = std.log.scoped(.cheats);

const HostPaths = @import("dreamcast").HostPaths;

pub const Value = union(enum) { u8: u8, u16: u16, u32: u32, u64: u64 };
pub const Action = struct { address: u32, value: Value };

pub const Cheat = struct {
    name: []const u8,
    enabled: bool = false,
    actions: []Action,

    pub fn deinit(self: @This(), allocator: std.mem.Allocator) void {
        allocator.free(self.name);
        allocator.free(self.actions);
    }
};

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
    actions: []const Action,

    pub fn dupe(self: @This(), allocator: std.mem.Allocator) !Cheat {
        return .{
            .name = try allocator.dupe(u8, self.name),
            .enabled = self.enabled,
            .actions = try allocator.dupe(Action, self.actions),
        };
    }
};

const Builtin: []const struct { game: Game, cheats: []const BuiltinCheat } = @import("cheats.zon");

pub fn get_builtin_cheats(product_name: []const u8, product_id: []const u8) ?[]const BuiltinCheat {
    for (Builtin) |entry| {
        if (std.mem.eql(u8, entry.game.product_name, product_name) and std.mem.eql(u8, entry.game.product_id, product_id)) {
            return entry.cheats;
        }
    }
    return null;
}

/// Caller owns the returned memory
pub fn path(allocator: std.mem.Allocator, product_id: ?[]const u8, product_name: ?[]const u8) ![]const u8 {
    const game_dir = try HostPaths.userdata_game_directory(allocator, product_id, product_name);
    defer allocator.free(game_dir);
    return try std.fs.path.join(allocator, &[_][]const u8{ game_dir, "cheats.zon" });
}

pub fn save(allocator: std.mem.Allocator, product_id: ?[]const u8, product_name: ?[]const u8, cheats: []const Cheat) !void {
    const cheat_path = try path(allocator, product_id, product_name);
    defer allocator.free(cheat_path);

    if (std.fs.path.dirname(cheat_path)) |dir| try std.fs.cwd().makePath(dir);

    const file = try std.fs.cwd().createFile(cheat_path, .{});
    defer file.close();
    const buffer = try allocator.alloc(u8, 8192);
    defer allocator.free(buffer);
    var writer = file.writer(buffer);
    try std.zon.stringify.serialize(cheats, .{}, &writer.interface);
    try writer.end();
}

/// Caller owns the returned memory
pub fn load(allocator: std.mem.Allocator, product_id: ?[]const u8, product_name: ?[]const u8) !?[]Cheat {
    const cheat_path = try path(allocator, product_id, product_name);
    defer allocator.free(cheat_path);

    const file = std.fs.cwd().openFile(cheat_path, .{}) catch |err| {
        switch (err) {
            error.FileNotFound => {
                if (product_id == null or product_name == null) return null;
                // Load default cheats.
                if (get_builtin_cheats(product_id.?, product_name.?)) |builtin_cheats| {
                    var cheat_list: std.ArrayList(Cheat) = .empty;
                    errdefer {
                        for (cheat_list.items) |cheat| cheat.deinit(allocator);
                        cheat_list.deinit(allocator);
                    }
                    for (builtin_cheats) |cheat|
                        try cheat_list.append(allocator, try cheat.dupe(allocator));
                    const slice = try cheat_list.toOwnedSlice(allocator);

                    try save(allocator, product_id, product_name, slice);

                    return slice;
                }
                return null;
            },
            else => return err,
        }
    };
    defer file.close();

    const cheats_str = try file.readToEndAllocOptions(allocator, 1024 * 1024, null, .@"8", 0);
    defer allocator.free(cheats_str);

    const zon = std.zon.parse.fromSlice([]Cheat, allocator, cheats_str, null, .{ .ignore_unknown_fields = true, .free_on_error = true }) catch |err| {
        log.err("Failed to parse cheats file '{s}': {t}.", .{ cheat_path, err });
        return &.{};
    };
    return zon;
}
