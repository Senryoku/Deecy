const std = @import("std");
const log = std.log.scoped(.maple);
const termcolor = @import("termcolor");

const common = @import("../maple.zig");
const FunctionCodesMask = common.FunctionCodesMask;
const DeviceInfoPayload = common.DeviceInfoPayload;

const VibrationFunctionDefinition = packed struct(u32) {
    /// VN: Vibration source number
    ///   Indicates the number of vibration sources
    ///   The 4 upper bits are fixed at '0', and the number of vibration sources is represented by the 4 lower bits.
    ///   The number of vibration sources is 1～15 ('1h'～'Fh'). '0' setting is not permitted.
    vibration_source_number: u8,
    /// SE: Number of vibration sources that can be concurrently selected.
    ///   Indicates the number of vibration sources which can be concurrently specified to generate vibration.
    ///   The 4 upper bits are fixed at '0', and the number of vibration sources is represented by the 4 lower bits.
    ///   The number of vibration sources is 1～15 ('1h'～'Fh').
    ///   '0' setting is not permitted. The settings must conform to SE <= VN.
    concurrent_sources: u8,
    _reserved: u16 = 0,
};

const Capabilities = FunctionCodesMask.Vibration;
const Subcapabilities: [3]u32 = .{
    @bitCast(VibrationFunctionDefinition{
        .vibration_source_number = 1,
        .concurrent_sources = 1,
    }),
    0,
    0,
};

const Identity: DeviceInfoPayload = .{
    .FunctionCodesMask = Capabilities,
    .SubFunctionCodesMasks = Subcapabilities,
    .RegionCode = 0xFF,
    .DescriptionString = "Puru Puru Pack                 ".*,
    .StandbyConsumption = 0x00CB,
    .MaximumConsumption = 0x0640,
};

const VibrationSourceSettings = packed struct(u32) {
    /// Vibration source vibration axis.
    ///   Indicates the axis (direction) the vibration source vibrates along.
    vd: enum(u2) { None = 0, X = 1, Y = 2, Z = 3 },
    /// Vibration source position.
    ///    Indicates the position where the vibration source is installed.
    vp: enum(u2) { Front = 0, Back = 1, Left = 2, Right = 3 },
    /// Vibration source No.
    ///   Indicates the number of vibration sources.
    ///   The number of vibration sources is 1～15 ('1h'～'Fh').
    ///   '0h' is not permitted.
    vn: u4,
    /// Vibration attribute flag
    ///   Information following this attribute changes according.
    ///   The 3 kinds of VA settings are '0000','0001','1111'. All others are reserved.
    ///     VA='0000': fm0 and fm1 represents the minimum and maximum vibration frequency values, respectively.
    ///     VA='0001': fm0 represents a fixed frequency 0.5～128Hz (00h～FFh). (fm1 is not used and should be 0)
    ///     VA='1111': the vibration frequency cannot be specified. (fm0 and fm1 are not used and should be 0)
    va: enum(u4) { MinMax = 0b0000, Fixed = 0b0001, None = 0b1111, _ },
    /// Arbitrary vibration waveform flag
    ///   Indicates if the arbitrary vibration waveform can be selected.
    owf: bool,
    /// Vibration source direction setting flag
    ///   Indicates if + directions and - directions are settable.
    ///   If +/- settings are not permitted, the setting is specified as + direction. Even if - direction is specified, it is treated as + direction.
    pd: bool,
    /// Vibration source continuous vibration flag
    ///   Indicates if a specified vibration can continue until the next setting command.
    cv: bool,
    /// Setting of variable vibration intensity
    ///   Indicates if the intensity of the vibration source is variable. (Fixed or up to 8 levels)
    pf: bool,
    fm0: u8,
    fm1: u8,
};

const VibrationConfiguration = packed struct(u32) {
    /// Continuous vibration setting bits
    cnt: bool = false,
    _res: u3 = 0,
    /// Vibration source No
    vn: u4 = 0,
    /// Backward direction (- direction) intensity setting bit
    mpow: u3 = 0,
    /// Divergent vibration setting bit
    exh: bool = false,
    /// Forward direction (+ direction) intensity setting bit
    ppow: u3 = 0,
    /// Convergent vibration setting bit
    inh: bool = false,
    /// Vibration frequency setting bit
    freq: u8 = 0,
    /// Vibration inclination period setting bit
    ///   Specifies the vibration inclination period.
    ///   Is specified when either convergence or divergence is used. When convergence and divergence are not used, the arbitrary value "00h" is used.
    ///   1 convergent (or 1 divergent) vibration is completed in the period specified in Inc.
    ///   Specifying Inc="00h" when convergence or divergence are selected results in an error.
    inc: u8 = 0,

    pub fn frequency(self: @This()) f32 {
        const freq: f32 = @floatFromInt(self.freq);
        return (freq + 1.0) / 2.0;
    }

    pub fn intensity_change(self: @This(), freq: f32) f32 {
        if (self.cnt) return 0;
        const abs = (1.0 / 7.0) * @as(f32, @floatFromInt(self.inc)) * 1.0 / freq;
        if (self.inh) return -abs;
        if (self.exh) return abs;
        return 0;
    }
};

pub const Callback = struct {
    function: ?*const fn (*anyopaque, f32, f32) void,
    context: *anyopaque,

    pub fn call(self: @This(), power: f32, change: f32) void {
        if (self.function != null)
            self.function.?(self.context, power, change);
    }
};

const Settings = VibrationSourceSettings{
    .vd = .None,
    .vp = .Front,
    .vn = 1,
    .va = .MinMax,
    .owf = false,
    .pd = true,
    .cv = true,
    .pf = true,
    .fm0 = 0x07,
    .fm1 = 0x3B,
};

callback: ?Callback = null,
vibration_source: VibrationConfiguration = .{},

positive_peak: f32 = 0.0,
negative_peak: f32 = 0.0,

pub fn init(callback: Callback) @This() {
    return .{ .callback = callback };
}

pub fn get_identity(_: *const @This()) DeviceInfoPayload {
    return Identity;
}

/// Writes Media Info to dest. Returns the payload size in 32-bit words.
pub fn get_media_info(_: *const @This(), dest: [*]u8, function: u32, vibration_source: u8) u8 {
    log.debug("VibrationPack.get_media_info for function: {f}, vibration_source: {d}", .{ FunctionCodesMask.from_u32(function), vibration_source });
    switch (function) {
        FunctionCodesMask.Vibration.as_u32() => {
            @memcpy(dest[0..@sizeOf(VibrationSourceSettings)], std.mem.asBytes(&Settings));
            return @sizeOf(VibrationSourceSettings) / 4;
        },
        else => log.err(termcolor.red("Unimplemented VibrationPack.get_media_info for function: {f}"), .{FunctionCodesMask.from_u32(function)}),
    }
    return 0;
}

/// Returns payload size in 32-bit words
pub fn block_read(self: *const @This(), dest: [*]u8, function: u32, vn: u8, block_num: u16, phase: u8) u8 {
    _ = self;
    _ = dest;
    _ = block_num;
    _ = phase;
    // In response to the function, this command requests the data of the specified
    // vibration source (VN). It is used to read the settings for both current
    // arbitrary waveforms in the vibration source and auto-stop time.
    // In vibration functions, Phase='00h' ,Block No. ='0000h' are fixed values. VN
    // for each vibration source is '01h'～'0Fh' for Vibration Source -1～Vibration
    // Source -15, respectively.
    log.warn(termcolor.yellow("VibrationPack.block_read for function: {f}, source: {d}"), .{ FunctionCodesMask.from_u32(function), vn });
    switch (function) {
        FunctionCodesMask.Vibration.as_u32() => {},
        else => log.err("Unimplemented VibrationPack.block_read for function: {f}", .{FunctionCodesMask.from_u32(function)}),
    }
    return 0;
}

/// Returns payload size in 32-bit words
pub fn block_write(self: *@This(), function: u32, vn: u8, phase: u8, block_num: u16, data: []const u32) u8 {
    _ = self;
    _ = data; // Arbitrary waveform data
    _ = block_num;
    _ = phase;
    // In response to the vibration function, this command records (writes) data in
    // the specified vibration source. It is used to specify arbitrary waveforms and
    // vibration auto-stop time.
    // In vibration functions, Phase='00h' ,Block No. ='0000h' are fixed values. VN
    // for each vibration source is '01h'～'0Fh' for Vibration Source -1～Vibration
    // Source -15, respectively.
    log.warn(termcolor.yellow("VibrationPack.block_write for function: {f}, source: {d}"), .{ FunctionCodesMask.from_u32(function), vn });
    switch (function) {
        FunctionCodesMask.Vibration.as_u32() => {},
        else => log.err("Unimplemented VibrationPack.block_write for function: {f}", .{FunctionCodesMask.from_u32(function)}),
    }
    return 0;
}

pub fn get_condition(self: *const @This(), function: u32) [1]u32 {
    log.debug("VibrationPack.get_condition for function: {f}", .{FunctionCodesMask.from_u32(function)});
    switch (function) {
        FunctionCodesMask.Vibration.as_u32() => return .{@bitCast(self.vibration_source)},
        else => log.err("Unimplemented VibrationPack.get_condition for function: {f}", .{FunctionCodesMask.from_u32(function)}),
    }
    return .{0};
}

pub fn set_condition(self: *@This(), function: u32, data: []const u32) void {
    // NOTE: Multiple vibration sources can be set at once (one 32bits VibrationConfiguration each), but only one is supported (and advertised).
    switch (function) {
        FunctionCodesMask.Vibration.as_u32() => {
            const value: VibrationConfiguration = @bitCast(data[0]);

            log.debug("[VibrationPack.set_condition] Vibration settings: {any}", .{value});
            if (value.vn != 1) log.warn(termcolor.yellow("Only vibration source 1 is supported: {any}"), .{value});

            self.vibration_source = value;

            // "When no axis direction is specified (when VD='00'), the + peak value and the - peak value cannot
            //  be specified concurrently. Concurrent configuration results in an error."
            if (value.ppow != 0) {
                self.positive_peak = @floatFromInt(value.ppow);
            } else if (value.mpow != 0) {
                self.negative_peak = @floatFromInt(value.mpow);
            } else {
                self.positive_peak = 0.0;
                self.negative_peak = 0.0;
            }

            const frequency = switch (Settings.va) {
                .MinMax => value.frequency(),
                .Fixed => (@as(f32, @floatFromInt(Settings.fm0)) + 1.0) / 2.0,
                else => 0.0,
            };

            const power = (self.positive_peak + self.negative_peak) / 14.0;
            if (self.callback) |c| c.call(power, value.intensity_change(frequency));
        },
        else => log.err("Unimplemented VibrationPack.set_condition for function: {f}", .{FunctionCodesMask.from_u32(function)}),
    }
}

pub fn serialize(_: @This(), _: anytype) !usize {
    return 0;
}
