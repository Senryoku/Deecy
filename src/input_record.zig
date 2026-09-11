//! Input record
inputs: std.ArrayList(Entry) = .empty,

pub fn add(self: *@This(), allocator: std.mem.Allocator, port: u8, cycle: u64, state: ControllerState) !void {
    _ = port;
    try self.inputs.append(allocator, .{ .cycle = cycle, .input = state });
}

const ControllerState = extern struct {
    buttons: maple.Controller.Buttons,
    axis: [6]u8,
};

const Entry = extern struct {
    cycle: u64,
    input: ControllerState,
};

const Header = packed struct {};

const std = @import("std");
const maple = @import("dreamcast").Maple;
