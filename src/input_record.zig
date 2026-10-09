//! Input record

game_id: GameID = .{},
initial_rtc: u32 = 0,
ports: [4]union(Device) {
    none,
    controller: struct { inputs: std.ArrayList(Entry(ControllerState)) = .empty },
} = @splat(.none),

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    for (&self.ports) |*p| switch (p.*) {
        .none => {},
        .controller => |*c| c.inputs.deinit(allocator),
    };
}

pub fn set_game(self: *@This(), product_id: ProductUID) void {
    self.game_id.name = @splat(0);
    self.game_id.id = @splat(0);
    @memcpy(self.game_id.name[0..product_id.name.len], product_id.name);
    @memcpy(self.game_id.id[0..product_id.id.len], product_id.id);
}

pub fn add(self: *@This(), allocator: std.mem.Allocator, port: u8, cycle: u64, state: ControllerState) !void {
    switch (self.ports[port]) {
        .controller => |*c| try c.inputs.append(allocator, .{ .cycle = cycle, .input = state }),
        else => return error.InvalidDevice,
    }
}

const ControllerState = extern struct {
    buttons: maple.Controller.Buttons,
    axis: [6]u8,
};

fn Entry(comptime T: type) type {
    return extern struct {
        cycle: u64,
        input: T,
    };
}

const Device = enum(u8) {
    none = 0,
    controller = 1,
};

const Header = extern struct {
    tag: [8]u8 = Tag,
    version: u32 = 0,
    flags: u32,
    deecy_version: [8]u8 = padded(comptime_config.version, 8),
    deecy_commit: [8]u8 = padded(comptime_config.git_commit, 8),
    game_id: GameID,
    initial_rtc: u32,
    ports: [4]Device,
    _reserved: [8]u8 = @splat(0),

    const Tag = "DEECYMOV".*;

    fn padded(comptime str: []const u8, comptime len: usize) [len]u8 {
        const padding: [len - @min(str.len, len)]u8 = @splat(0);
        return (str[0..@min(str.len, len)] ++ padding).*;
    }
};

const GameID = extern struct {
    name: [16]u8 = @splat(0),
    id: [16]u8 = @splat(0),
};

pub fn serialize(self: *const @This(), writer: *std.Io.Writer) !void {
    try writer.writeStruct(Header{
        .flags = 0,
        .game_id = self.game_id,
        .initial_rtc = self.initial_rtc,
        .ports = .{
            std.meta.activeTag(self.ports[0]),
            std.meta.activeTag(self.ports[1]),
            std.meta.activeTag(self.ports[2]),
            std.meta.activeTag(self.ports[3]),
        },
    }, .little);
    for (self.ports) |p| switch (p) {
        .none => {},
        .controller => |c| {
            try writer.writeInt(u64, c.inputs.items.len, .little);
            try writer.writeAll(std.mem.sliceAsBytes(c.inputs.items));
        },
    };
}

pub fn deserialize(allocator: std.mem.Allocator, reader: *std.Io.Reader) !@This() {
    const header = try reader.takeStruct(Header, .little);
    if (!std.mem.eql(u8, &header.tag, &Header.Tag))
        return error.InvalidDCM;

    var r: @This() = .{};
    errdefer r.deinit(allocator);

    log.info("Loading DCM v{d} for '{s}' ({s}). Created with Deecy v{s} ({s}).", .{ header.version, header.game_id.name, header.game_id.id, header.deecy_version, header.deecy_commit });
    r.game_id = header.game_id;
    r.initial_rtc = header.initial_rtc;

    for (header.ports, 0..) |port, idx| {
        switch (port) {
            .none => {},
            .controller => {
                r.ports[idx] = .{ .controller = .{} };
                const count = try reader.takeInt(u64, .little);
                try r.ports[idx].controller.inputs.ensureTotalCapacity(allocator, count);
                for (0..count) |_| {
                    try r.ports[idx].controller.inputs.append(allocator, try reader.takeStruct(Entry(ControllerState), .little));
                }
            },
        }
    }
    return r;
}

const std = @import("std");
const log = std.log.scoped(.input_record);
const maple = @import("dreamcast").Maple;
const comptime_config = @import("config");

const ProductUID = @import("ProductUID.zig");
