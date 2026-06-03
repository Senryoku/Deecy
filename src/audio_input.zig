//! Audio capture for the emulated microphone.
//! NOTE: Currently a single emulated microphone at a time is supported.
//!       Multiple microphones will just read from the same buffer and starve.

device: *zaudio.Device,
buffer: []i16,
write_offset: u32 = 0,
read_offset: u32 = 0,
/// Small buffer handed over to the DC get_samples callback.
tmp_buffer: []i16,

pub fn create(allocator: std.mem.Allocator) !*@This() {
    const self = try allocator.create(@This());
    var audio_device_config = zaudio.Device.Config.init(.capture);
    audio_device_config.sample_rate = 11025;
    audio_device_config.data_callback = input_callback;
    audio_device_config.user_data = self;
    audio_device_config.capture.format = .signed16;
    audio_device_config.capture.channels = 1;
    const device = try zaudio.Device.create(null, audio_device_config);
    errdefer device.destroy();
    self.* = .{
        .device = device,
        .buffer = try allocator.alloc(i16, 32 * 1024),
        .tmp_buffer = try allocator.alloc(i16, 255),
    };
    @memset(self.buffer, 0);
    return self;
}

pub fn destroy(self: *@This(), allocator: std.mem.Allocator) void {
    self.device.stop() catch |err| log.err("Failed to stop audio input device: {t}", .{err});
    self.device.destroy();
    allocator.free(self.buffer);
    allocator.free(self.tmp_buffer);
    allocator.destroy(self);
}

/// Idempotent
pub fn start(self: *@This()) !void {
    if (!self.device.isStarted()) {
        self.write_offset = 0;
        self.read_offset = 0;
        try self.device.start();
    }
}

pub fn get_samples(self: *@This()) ![]i16 {
    try self.start(); // The game should take care of that, but this might not hold after loading a save state.
    for (0..self.tmp_buffer.len) |i| {
        if (self.read_offset == self.write_offset) return self.tmp_buffer[0..i];
        self.tmp_buffer[i] = self.buffer[self.read_offset];
        self.read_offset += 1;
        if (self.read_offset >= self.buffer.len) self.read_offset = 0;
    }
    return self.tmp_buffer;
}

fn input_callback(device: *zaudio.Device, _: ?*anyopaque, input: ?*const anyopaque, frame_count: u32) callconv(.c) void {
    const self: *@This() = @ptrCast(@alignCast(device.getUserData()));
    const samples: [*]const i16 = @ptrCast(@alignCast(input));
    for (0..frame_count) |i| {
        self.buffer[self.write_offset] = samples[i];
        self.write_offset +%= 1;
        if (self.write_offset >= self.buffer.len) self.write_offset = 0;
    }
}

const std = @import("std");
const log = std.log.scoped(.deecy);
const zaudio = @import("zaudio");
