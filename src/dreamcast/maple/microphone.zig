//! Emulated Microphone
pub const Capabilities = FunctionCodesMask.AudioInput;
subcapabilities: [3]u32 = .{ DreamEyeMicrophoneCapabilities.Default.as_u32(), 0, 0 },

// Internal state
// NOTE: This is not serialized, but also not really used.
amp_gain: u8 = 0x80,
control: BasicControl = .{},
sample_expansion: ExtuBit = .Low00,

// Callbacks
context: ?*anyopaque,
on_start_sampling: ?*const fn (?*anyopaque) void,
on_stop_sampling: ?*const fn (?*anyopaque) void,
on_get_samples: ?*const fn (?*anyopaque) []i16,

pub fn init(context: ?*anyopaque, start_sampling: ?*const fn (?*anyopaque) void, stop_sampling: ?*const fn (?*anyopaque) void, get_samples: ?*const fn (?*anyopaque) []i16) @This() {
    return .{
        .context = context,
        .on_start_sampling = start_sampling,
        .on_stop_sampling = stop_sampling,
        .on_get_samples = get_samples,
    };
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
pub fn audio_input_command(self: *@This(), command: []const u32, dest: [*]u32) struct { common.Command, u8 } {
    if (command.len == 0) return .{ .UnknownCommand, 0 };

    const bytes = std.mem.asBytes(&command[0]);
    const subcommand: Subcommand = @enumFromInt(bytes[0]);
    const dt = bytes[1..4];
    log.debug("Audio Input Command: {}, {X}", .{ subcommand, dt });
    switch (subcommand) {
        .GetSamplingData => {
            var sample_header = SamplingData{
                .status = .{
                    .frequency = .@"11.025 kHz",
                    .uLaw = .@"14bit Linear",
                    .sbfov = .Normal,
                    .sample_expansion = @truncate(@intFromEnum(self.sample_expansion)),
                    .sampling = self.control.sampling,
                },
                .amp = self.amp_gain,
                .sample_count = 0,
            };
            self.amp_gain = dt[0];
            if (self.on_get_samples) |callback| {
                const samples = callback(self.context);
                sample_header.sample_count = @intCast(samples.len);
                const sample_dest: [*]i16 = @ptrCast(dest[2..]);
                for (samples, 0..) |sample, i| sample_dest[i] = sample;
            }
            log.debug("Get Sampling Data command: {any}", .{sample_header});
            dest[0] = Capabilities.as_u32(); // "Function type"
            dest[1] = @bitCast(sample_header); // "FT4 data"
            return .{ .DataTransfer, 2 + sample_header.sample_count / 2 };
        },
        .BasicControl => {
            const control: BasicControl = @bitCast(dt[0]);
            log.info("Basic Control: {any}", .{control});
            if (control.sampling != self.control.sampling) {
                if (control.sampling) {
                    if (self.on_start_sampling) |callback| callback(self.context);
                } else {
                    if (self.on_stop_sampling) |callback| callback(self.context);
                }
            }
            self.control = control;
            return .{ .Acknowledge, 0 };
        },
        .AmpGain => {
            self.amp_gain = dt[0];
            return .{ .Acknowledge, 0 };
        },
        .ExtuBit => {
            const extu_bit: ExtuBit = @enumFromInt(dt[0]);
            switch (extu_bit) {
                else => {
                    self.sample_expansion = extu_bit;
                    return .{ .Acknowledge, 0 };
                },
                _ => return .{ .UnknownCommand, 0 },
            }
        },
        .VolumeMode => {
            const mode: enum(u8) { @"+30dB" = 0, @"+12dB" = 1, _ } = @enumFromInt(dt[0]);
            log.warn("Unimplemented Volume Mode command: {}", .{mode});
            return .{ .Acknowledge, 0 };
        },
        .TestMode => {
            log.warn("Unimplemented Test Mode command.", .{});
            // NOTE: Correct implementation should return DataTransfer with test result payload.
            //   return .{ .DataTransfer, ... };
            return .{ .Acknowledge, 0 };
        },
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
    /// Used by host to request sampling data and set AMP gain (DT1).
    /// Response: Data Transfer
    GetSamplingData = 0x01,
    /// Sets the following functions.
    ///   Start/stop sound sampling
    ///   Select sampling frequency
    ///   Select sampling bit count
    /// Response: Device Reply
    BasicControl = 0x02,
    /// Sets the sound input AMP gain value (DT1).
    /// Response: Device Reply
    AmpGain = 0x03,
    /// Sampling data can be expanded in devices belonging to FT4.
    /// The method used or expanding sampling data is specified using this command.
    /// Response: Device Reply
    ExtuBit = 0x04,
    /// Changes the amplification reference.
    /// Response: Device Reply
    VolumeMode = 0x05,
    /// Response: -
    Prohibited = 0x06,
    /// Test mode for debugging. For information on the format, see the specifications for each
    /// device. The command is designed to be used only for debugging. It may not be used in
    /// an application.
    /// Response: Data Transfer
    TestMode = 0xFC,
    /// Response: -
    UndefinedFE = 0xFE,
    UndefinedFF = 0xFF,
    /// Undefined
    /// Response: Undefined
    _,
};

pub const BasicControl = packed struct(u8) {
    quantization: u2 = 0,
    sampling_frequency: u2 = 0,
    _: u3 = 0,
    sampling: bool = false,
};

pub const ExtuBit = enum(u8) {
    /// Lower 2 bits are set at "00"
    Low00 = 0,
    /// LSB 2 bits are copied to the 2 lowest bits of the expanded sample
    LSB = 1,
    /// Lower 2 bits are set at "10"
    Low10 = 2,
    /// Prohibited
    _,
};

pub const SamplingData = packed struct(u32) {
    status: packed struct(u8) {
        /// Sampling frequency setting
        frequency: enum(u1) { @"11.025 kHz" = 0, @"8.0 kHz" = 1 },
        /// CODEC conversion
        uLaw: enum(u1) {
            // Signed 14-bit samples extended to 16-bit
            @"14bit Linear" = 0,
            @"8bit u-Law Codec" = 1,
        },
        /// Sampling operation start
        sampling: bool,
        /// Linear sampling data expansion method flag ("14lsb")
        sample_expansion: u2,
        _: u1 = 0,
        /// Sampling data buffer overflow condition
        sbfov: enum(u1) { Normal = 0, Overflow = 1 },
        /// Expansion bit for new functions. When this bit is "1", the connected peripheral
        /// incorporates another function besides the basic functions. "DT2" comprises
        /// the status data for the new function. Details are still pending.
        ex_bit: bool = false,
    },
    amp: u8,
    ext: u8 = 0,
    sample_count: u8,
};

const std = @import("std");
const log = std.log.scoped(.microphone);
const common = @import("../maple.zig");
const FunctionCodesMask = common.FunctionCodesMask;
const DeviceInfoPayload = common.DeviceInfoPayload;
