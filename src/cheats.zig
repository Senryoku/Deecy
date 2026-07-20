const std = @import("std");
const log = std.log.scoped(.cheats);

const HostPaths = @import("dreamcast").HostPaths;
const Default = @import("default_game_settings.zig");
const codebreaker = @import("codebreaker.zig");

pub const Condition = enum {
    @"=",
    @"!=",
    @"<",
    @">",

    pub fn from_codebreaker(cb: codebreaker.Condition) Condition {
        return switch (cb) {
            .Equal => .@"=",
            .Different => .@"!=",
            .LessThan => .@"<",
            .GreaterThan => .@">",
        };
    }
};

pub const Value = union(enum) { u8: u8, u16: u16, u32: u32, u64: u64 };
pub const Action = union(enum) {
    Write: struct { address: u32 = 0x0C000000, value: Value = .{ .u32 = 0 } },
    Condition: struct { condition: Condition = .@"=", count: u8 = 1, address: u32 = 0x0C000000, value: Value = .{ .u32 = 0 } },

    pub fn address_ptr(self: *@This()) *u32 {
        return switch (self.*) {
            .Write => |*w| &w.address,
            .Condition => |*c| &c.address,
        };
    }

    pub fn value_ptr(self: *@This()) *Value {
        return switch (self.*) {
            .Write => |*w| &w.value,
            .Condition => |*c| &c.value,
        };
    }
};

pub const Cheat = struct {
    name: []const u8,
    enabled: bool = false,
    actions: []Action,

    pub fn deinit(self: @This(), allocator: std.mem.Allocator) void {
        allocator.free(self.name);
        allocator.free(self.actions);
    }
};

/// Caller owns the returned memory
pub fn path(allocator: std.mem.Allocator, product_name: []const u8, product_id: []const u8) ![]const u8 {
    const game_dir = try HostPaths.userdata_game_directory(allocator, product_name, product_id);
    defer allocator.free(game_dir);
    return try std.fs.path.join(allocator, &[_][]const u8{ game_dir, "cheats.zon" });
}

pub fn save(allocator: std.mem.Allocator, io: std.Io, product_name: []const u8, product_id: []const u8, cheats: []const Cheat) !void {
    const cheat_path = try path(allocator, product_name, product_id);
    defer allocator.free(cheat_path);

    if (std.fs.path.dirname(cheat_path)) |dir| try std.Io.Dir.cwd().createDirPath(io, dir);

    const file = try std.Io.Dir.cwd().createFile(io, cheat_path, .{});
    defer file.close(io);
    const buffer = try allocator.alloc(u8, 8192);
    defer allocator.free(buffer);
    var writer = file.writer(io, buffer);
    try std.zon.stringify.serialize(cheats, .{}, &writer.interface);
    try writer.end();
}

/// Caller owns the returned memory
pub fn load(allocator: std.mem.Allocator, io: std.Io, product_name: []const u8, product_id: []const u8) !?[]Cheat {
    const cheat_path = try path(allocator, product_name, product_id);
    defer allocator.free(cheat_path);

    const cheats_str = std.Io.Dir.cwd().readFileAllocOptions(io, cheat_path, allocator, .limited(8 * 1024 * 1024), .@"8", 0) catch |err| {
        switch (err) {
            error.FileNotFound => {
                // Load default cheats.
                if (Default.get(product_name, product_id)) |builtin| {
                    var cheat_list: std.ArrayList(Cheat) = .empty;
                    errdefer {
                        for (cheat_list.items) |cheat| cheat.deinit(allocator);
                        cheat_list.deinit(allocator);
                    }
                    for (builtin.cheats) |cheat|
                        try cheat_list.append(allocator, try cheat.dupe(allocator));
                    const slice = try cheat_list.toOwnedSlice(allocator);

                    try save(allocator, io, product_name, product_id, slice);

                    return slice;
                }
                return null;
            },
            else => return err,
        }
    };
    defer allocator.free(cheats_str);

    const zon = std.zon.parse.fromSliceAlloc([]Cheat, allocator, cheats_str, null, .{ .ignore_unknown_fields = true, .free_on_error = true }) catch |err| {
        log.err("Failed to parse cheats file '{s}': {t}.", .{ cheat_path, err });
        return &.{};
    };
    return zon;
}
