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

pub fn save(allocator: std.mem.Allocator, io: std.Io, product_uid: Default.ProductUID, cheats: []const Cheat) !void {
    const dir = try host_paths.game_directory(io, allocator, product_uid);
    defer dir.close(io);

    const file = try dir.createFile(io, CheatsFileName, .{});
    defer file.close(io);
    const buffer = try allocator.alloc(u8, 8192);
    defer allocator.free(buffer);
    var writer = file.writer(io, buffer);
    try std.zon.stringify.serialize(cheats, .{}, &writer.interface);
    try writer.end();
}

/// Caller owns the returned memory
pub fn load(allocator: std.mem.Allocator, io: std.Io, uid: Default.ProductUID) !?[]Cheat {
    const dir = try host_paths.game_directory(io, allocator, uid);
    defer dir.close(io);

    const cheats_str = dir.readFileAllocOptions(io, CheatsFileName, allocator, .limited(8 * 1024 * 1024), .@"8", 0) catch |err| {
        switch (err) {
            error.FileNotFound => {
                // Load default cheats.
                if (Default.get(uid)) |builtin| {
                    var cheat_list: std.ArrayList(Cheat) = .empty;
                    errdefer {
                        for (cheat_list.items) |cheat| cheat.deinit(allocator);
                        cheat_list.deinit(allocator);
                    }
                    for (builtin.cheats) |cheat|
                        try cheat_list.append(allocator, try cheat.dupe(allocator));
                    const slice = try cheat_list.toOwnedSlice(allocator);

                    try save(allocator, io, uid, slice);

                    return slice;
                }
                return null;
            },
            else => return err,
        }
    };
    defer allocator.free(cheats_str);

    var diagnostics: std.zon.parse.Diagnostics = undefined;
    defer helpers.free(allocator, diagnostics);
    const zon = std.zon.parse.fromSlice([]Cheat, .{ .gpa = allocator, .arena = allocator, .source = cheats_str, .diagnostics = &diagnostics, .ignore_unknown_fields = true }) catch |err| {
        log.err("Failed to parse cheats file for {f}: {t}.", .{ uid, err });
        switch (err) {
            error.ParseZon => diagnostics.log(CheatsFileName),
            else => {},
        }
        return &.{};
    };
    return zon;
}

const CheatsFileName = "cheats.zon";

const std = @import("std");
const log = std.log.scoped(.cheats);
const helpers = @import("helpers");

const host_paths = @import("host_paths.zig");
const Default = @import("default_game_settings.zig");
const codebreaker = @import("codebreaker.zig");
