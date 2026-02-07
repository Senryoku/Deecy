const std = @import("std");

const common = @import("../maple.zig");
const FunctionCodesMask = common.FunctionCodesMask;
const DeviceInfoPayload = common.DeviceInfoPayload;

const log = std.log.scoped(.maple);

pub const Capabilities = FunctionCodesMask.Pointing;

subcapabilities: [3]u32 = .{ (FunctionDefinition{}).as_u32(), 0, 0 },

absolute_positions: [2]f64 = @splat(0),

button_status: Buttons = .DefaultState,
axes_delta: [3]u16 = @splat(RelativeOrigin),

pub fn get_identity(self: *const @This()) DeviceInfoPayload {
    return .{
        .FunctionCodesMask = Capabilities,
        .SubFunctionCodesMasks = self.subcapabilities,
        // FIXME: None of the following values are correct.
        .DescriptionString = "Dreamcast Mouse               ".*,
        .StandbyConsumption = 0x01AE,
        .MaximumConsumption = 0x01F4,
    };
}

pub fn get_condition(self: *@This(), function: u32) [6]u32 {
    _ = function;
    var data: MouseData = .{};
    data.buttons = self.button_status;
    data.ac[0] = self.axes_delta[0];
    data.ac[1] = self.axes_delta[1];
    data.ac[2] = self.axes_delta[2];

    self.axes_delta = @splat(RelativeOrigin);

    const status = @as([*]const u32, @ptrCast(@alignCast(&data)))[0..5].*;
    return .{
        Capabilities.as_u32(),
        status[0],
        status[1],
        status[2],
        status[3],
        status[4],
    };
}

const RelativeOrigin = 0x200;
const RelativeMax = 0x3FF;

pub fn move_to(self: *@This(), x: f64, y: f64) void {
    // Only Mouse is implemented: 8bits relative coordinates
    std.debug.assert(@as(FunctionDefinition, @bitCast(self.subcapabilities[0])).category == .Mouse);

    const x_diff = self.absolute_positions[0] - x;
    const y_diff = self.absolute_positions[1] - y;
    self.absolute_positions[0] = x;
    self.absolute_positions[1] = y;

    self.axes_delta[0] = @intFromFloat(std.math.clamp(@as(f64, @floatFromInt(self.axes_delta[0])) - x_diff, 0.0, RelativeMax));
    self.axes_delta[1] = @intFromFloat(std.math.clamp(@as(f64, @floatFromInt(self.axes_delta[1])) - y_diff, 0.0, RelativeMax));
}

pub fn scroll(self: *@This(), y: f64) void {
    self.axes_delta[2] = @intFromFloat(std.math.clamp(@as(f64, @floatFromInt(self.axes_delta[2])) - y, 0.0, RelativeMax));
}

pub const Buttons = packed struct {
    c: bool = false,
    b: bool = false, // Right button: Cancel
    a: bool = false, // Left button: OK
    w: bool = false, // Wheel button: center
    u: bool = false, // Up
    d: bool = false, // Down
    l: bool = false, // Left
    r: bool = false, // Right

    pub const DefaultConfig = @This(){ .b = true, .a = true, .w = true };
    pub const DefaultState = @This(){ .c = true, .b = true, .a = true, .w = true, .u = true, .d = true, .l = true, .r = true };
};

pub const FunctionDefinition = packed struct(u32) {
    category: enum(u4) {
        // Coordinate axis accuracy : All 8-bit accuracy... At least according to the docs. In practice it seems to be 10-bit.
        // Coordinate systems : Relative coordinate systems
        Mouse = 0,
        // Coordinate axis accuracy : All 10-bit accuracy
        // Coordinate systems : Absolute coordinate systems
        Tablet = 1,
        _,
    } = .Mouse,
    _: u4 = 0,
    operating_button_definition: Buttons = .DefaultConfig,
    /// Indicates whether each analog coordinate axis is used or unused.
    analog_coordinate_axis_definition: packed struct {
        ac1: bool = true, // X axis data
        ac2: bool = true, // Y axis data
        ac3: bool = true, // Z axis data (wheel)
        ac4: bool = false,
        ac5: bool = false,
        ac6: bool = false,
        ac7: bool = false,
        ac8: bool = false,
    } = .{},
    /// Reserved
    fd: u8 = 0,

    pub const Default = @This(){};

    pub fn as_u32(self: @This()) u32 {
        return @bitCast(self);
    }
};

pub const MouseData = extern struct {
    buttons: Buttons = .{},
    op: packed struct { wire: enum(u1) { Connected = 0, Disconnected = 1 } = .Connected, battery: enum(u1) { NoProblem = 0, BatteryLow = 1 } = .NoProblem, _: u6 = 0 } = .{},
    /// Coordinate data overflow, one flag per axis
    aov: u8 = 0,
    reserved: u8 = 0,
    ac: [8]u16 align(1) = @splat(RelativeOrigin), // Default for relative coordinates
};
