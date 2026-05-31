//! Emulated Microphone
pub const Capabilities = FunctionCodesMask.AudioInput;
subcapabilities: [3]u32 = .{ DreamEyeMicrophoneCapabilities.Default.as_u32(), 0, 0 },

pub fn init() @This() {
    return .{};
}

pub fn deinit(_: *@This()) void {}

pub fn reset(_: *@This()) void {}

pub fn get_identity(self: *const @This()) DeviceInfoPayload {
    return .{
        .FunctionCodesMask = Capabilities,
        .SubFunctionCodesMasks = self.subcapabilities,
        .DescriptionString = "Dreamcast Microphone          ".*,
        .StandbyConsumption = 0x01AE,
        .MaximumConsumption = 0x01F4,
    };
}

/// Returns the response header: Command and payload length
pub fn audio_input_command(self: *@This(), command: []const u32) struct { common.Command, u8 } {
    _ = self;
    if (command.len == 0) return .{ .UnknownCommand, 0 };

    const bytes = std.mem.asBytes(&command[0]);
    const subcommand: Subcommand = @enumFromInt(bytes[0]);
    const dt = bytes[1..4];
    log.debug("Audio Input Command: {}, {X}", .{ subcommand, dt });
    switch (subcommand) {
        else => {
            log.warn("Unimplemented Audio Input Command: {}, {X}", .{ subcommand, dt });
            return .{ .UnknownCommand, 0 };
        },
    }
}

// The DreamEye includes a microphone, I don't know if the standalone microphone has the same capabilities.
pub const DreamEyeMicrophoneCapabilities = packed struct(u32) {
    _: u26 = 0,

    unused: u1 = 1,
    volume_mode: u1 = 1,
    extu_bit: u1 = 1,
    amp_gain: u1 = 1,
    basic_control: u1 = 1,
    get_sampling_data: u1 = 1,

    pub const Default = @This(){};

    pub fn as_u32(self: @This()) u32 {
        return @bitCast(self);
    }
};

pub const Subcommand = enum(u8) {
    /// Response: Command Unknown
    Undefined = 0x00,
    /// Response: Data Transfer
    GetSamplingData = 0x01,
    /// Response: Device Reply
    BasicControl = 0x02,
    /// Response: Device Reply
    AmpGain = 0x03,
    /// Response: Device Reply
    ExtuBit = 0x04,
    /// Response: Device Reply
    VolumeMode = 0x05,
    /// Response: -
    Prohibited = 0x06,
    /// Response: Data Transfer
    TestMode = 0xFC,
    /// Response: -
    UndefinedFE = 0xFE,
    UndefinedFF = 0xFF,
    /// Undefined
    /// Response: Undefined
    _,
};

const std = @import("std");
const log = std.log.scoped(.microphone);
const common = @import("../maple.zig");
const FunctionCodesMask = common.FunctionCodesMask;
const DeviceInfoPayload = common.DeviceInfoPayload;
