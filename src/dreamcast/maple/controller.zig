const common = @import("../maple.zig");
const FunctionCodesMask = common.FunctionCodesMask;
const DeviceInfoPayload = common.DeviceInfoPayload;

pub const InputCapabilities = packed struct(u32) {
    _0: u8 = 0,

    analogRtrigger: u1 = 0,
    analogLtrigger: u1 = 0,
    analogHorizontal: u1 = 0,
    analogVertical: u1 = 0,
    analogHorizontal2: u1 = 0,
    analogVertical2: u1 = 0,

    _1: u2 = 0,

    z: u1 = 0,
    y: u1 = 0,
    x: u1 = 0,
    d: u1 = 0,
    up2: u1 = 0,
    down2: u1 = 0,
    left2: u1 = 0,
    right2: u1 = 0,

    c: u1 = 0,
    b: u1 = 0,
    a: u1 = 0,
    start: u1 = 0,
    up: u1 = 0,
    down: u1 = 0,
    left: u1 = 0,
    right: u1 = 0,

    pub const Standard = @This(){
        .b = 1,
        .a = 1,
        .start = 1,
        .up = 1,
        .down = 1,
        .left = 1,
        .right = 1,
        .y = 1,
        .x = 1,
        .analogRtrigger = 1,
        .analogLtrigger = 1,
        .analogHorizontal = 1,
        .analogVertical = 1,
    };

    pub fn as_u32(self: @This()) u32 {
        return @bitCast(self);
    }
};

/// NOTE: 0 = Pressed!
pub const Buttons = packed struct(u16) {
    _0: u1 = 1,

    b: u1 = 1,
    a: u1 = 1,
    start: u1 = 1,
    up: u1 = 1,
    down: u1 = 1,
    left: u1 = 1,
    right: u1 = 1,

    _1: u1 = 1,

    y: u1 = 1,
    x: u1 = 1,

    _2: u5 = 0b11111,

    pub fn as_u16(self: @This()) u16 {
        return @bitCast(self);
    }
};

pub const DualStickControllerCapabilities: InputCapabilities = .{
    .b = 1,
    .a = 1,
    .start = 1,
    .up = 1,
    .down = 1,
    .left = 1,
    .right = 1,
    .y = 1,
    .x = 1,
    .analogRtrigger = 1,
    .analogLtrigger = 1,
    .analogHorizontal = 1,
    .analogVertical = 1,
    .analogHorizontal2 = 1,
    .analogVertical2 = 1,
};

pub const Capabilities = FunctionCodesMask.Controller;

subcapabilities: [3]u32 = .{ InputCapabilities.Standard.as_u32(), 0, 0 },

buttons: Buttons = .{},
axis: [6]u8 = @splat(0x80),
pub fn press_buttons(self: *@This(), buttons: Buttons) void {
    self.buttons = @bitCast(self.buttons.as_u16() & buttons.as_u16());
}
pub fn release_buttons(self: *@This(), buttons: Buttons) void {
    self.buttons = @bitCast(self.buttons.as_u16() | ~buttons.as_u16());
}

pub fn get_identity(self: *const @This()) DeviceInfoPayload {
    return .{
        .FunctionCodesMask = Capabilities,
        .SubFunctionCodesMasks = self.subcapabilities,
        .DescriptionString = "Dreamcast Controller          ".*, // NOTE: dc-arm7wrestler checks for this, maybe some games do too?
        .StandbyConsumption = 0x01AE,
        .MaximumConsumption = 0x01F4,
    };
}

pub fn get_condition(self: *const @This(), function: u32) [3]u32 {
    _ = function;
    var r = [3]u32{ 0xFFFFFFFF, 0x8080FFFF, 0x80808080 };
    r[0] = Capabilities.as_u32();
    r[1] = self.buttons.as_u16();
    for (0..6) |i| {
        @as([*]u8, @ptrCast(&r))[6 + i] = self.axis[i];
    }
    return r;
}
