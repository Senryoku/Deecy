const std = @import("std");

const PixelFormat = @import("holly.zig").TexturePixelFormat;

pub const Color16 = packed union {
    value: u16,
    argb1555: packed struct(u16) {
        b: u5,
        g: u5,
        r: u5,
        a: u1,
    },
    rgb565: packed struct(u16) {
        b: u5,
        g: u6,
        r: u5,
    },
    argb4444: packed struct(u16) {
        b: u4,
        g: u4,
        r: u4,
        a: u4,
    },

    pub inline fn bgra(self: @This(), format: PixelFormat, twiddled: bool) [4]u8 {
        // See 3.6.3 Color Data Extension (p149)
        if (twiddled) {
            return self.bgra_twiddled(format);
        } else {
            return self.bgra_non_twiddled(format);
        }
    }

    pub inline fn bgra_non_twiddled(self: @This(), format: PixelFormat) [4]u8 {
        switch (format) {
            .ARGB1555 => {
                return .{
                    @as(u8, self.argb1555.b) << 3,
                    @as(u8, self.argb1555.g) << 3,
                    @as(u8, self.argb1555.r) << 3,
                    @as(u8, self.argb1555.a) * 0xFF,
                };
            },
            .RGB565 => {
                return .{
                    @as(u8, self.rgb565.b) << 3,
                    @as(u8, self.rgb565.g) << 2,
                    @as(u8, self.rgb565.r) << 3,
                    255,
                };
            },
            .ARGB4444 => {
                return .{
                    @as(u8, self.argb4444.b) << 4,
                    @as(u8, self.argb4444.g) << 4,
                    @as(u8, self.argb4444.r) << 4,
                    @as(u8, self.argb4444.a) << 4,
                };
            },
            else => std.debug.panic("Invalid 16-bits pixel format: {any}", .{format}),
        }
    }

    pub inline fn bgra_twiddled(self: @This(), format: PixelFormat) [4]u8 {
        switch (format) {
            .ARGB1555 => {
                return .{
                    @as(u8, self.argb1555.b) << 3 | @as(u8, self.argb1555.b) >> 2,
                    @as(u8, self.argb1555.g) << 3 | @as(u8, self.argb1555.g) >> 2,
                    @as(u8, self.argb1555.r) << 3 | @as(u8, self.argb1555.r) >> 2,
                    @as(u8, self.argb1555.a) * 0xFF,
                };
            },
            .RGB565 => {
                return .{
                    @as(u8, self.rgb565.b) << 3 | @as(u8, self.rgb565.b) >> 2,
                    @as(u8, self.rgb565.g) << 2 | @as(u8, self.rgb565.g) >> 4,
                    @as(u8, self.rgb565.r) << 3 | @as(u8, self.rgb565.r) >> 2,
                    255,
                };
            },
            .ARGB4444 => {
                return .{
                    @as(u8, self.argb4444.b) << 4 | @as(u8, self.argb4444.b),
                    @as(u8, self.argb4444.g) << 4 | @as(u8, self.argb4444.g),
                    @as(u8, self.argb4444.r) << 4 | @as(u8, self.argb4444.r),
                    @as(u8, self.argb4444.a) << 4 | @as(u8, self.argb4444.a),
                };
            },
            else => std.debug.panic("Invalid 16-bits pixel format: {any}", .{format}),
        }
    }
};

pub const YUV422 = packed struct(u32) {
    u: u8,
    y0: u8,
    v: u8,
    y1: u8,
};

// /!\ Components in the order used by Holly
pub const RGBA = packed struct(u32) {
    a: u8,
    b: u8,
    g: u8,
    r: u8,
};

pub const PackedColor = packed struct(u32) {
    b: u8 = 0,
    g: u8 = 0,
    r: u8 = 0,
    a: u8 = 0,

    pub fn with_alpha(self: @This(), use_alpha: bool) @This() {
        return .{ .r = self.r, .g = self.g, .b = self.b, .a = if (use_alpha) self.a else 255 };
    }
};

pub const fARGB = packed struct {
    a: f32 = 0,
    r: f32 = 0,
    g: f32 = 0,
    b: f32 = 0,

    pub fn to_packed(self: @This(), use_alpha: bool) PackedColor {
        return .{
            .r = std.math.lossyCast(u8, self.r * 255.0),
            .g = std.math.lossyCast(u8, self.g * 255.0),
            .b = std.math.lossyCast(u8, self.b * 255.0),
            .a = if (use_alpha) std.math.lossyCast(u8, self.a * 255.0) else 255,
        };
    }

    pub fn apply_intensity(self: @This(), intensity: f32, use_alpha: bool) PackedColor {
        // "Convert the Face Color alpha values specified in the Global
        // Parameters into 8-bit integers (0 to 255). Multiply the RGB
        // values by the corresponding Face Color R/G/B value, and
        // convert the result into an 8-bit integer (0 to 255). Combine
        // each 8-bit value thus obtained into a 32-bit value and store it
        // in the ISP/TSP Parameters."
        const clampled = std.math.clamp(intensity, 0.0, 1.0);
        return (@This(){
            .r = clampled * self.r,
            .g = clampled * self.g,
            .b = clampled * self.b,
            .a = self.a,
        }).to_packed(use_alpha);
    }
};

pub const fRGBA = packed struct {
    r: f32 = 0,
    g: f32 = 0,
    b: f32 = 0,
    a: f32 = 0,

    pub fn from_packed(color: PackedColor, use_alpha: bool) @This() {
        return .{
            .r = @as(f32, @floatFromInt(color.r)) / 255.0,
            .g = @as(f32, @floatFromInt(color.g)) / 255.0,
            .b = @as(f32, @floatFromInt(color.b)) / 255.0,
            .a = if (use_alpha) @as(f32, @floatFromInt(color.a)) / 255.0 else 1.0,
        };
    }

    pub fn to_packed(self: @This(), use_alpha: bool) PackedColor {
        return .{
            .r = std.math.lossyCast(u8, self.r * 255.0),
            .g = std.math.lossyCast(u8, self.g * 255.0),
            .b = std.math.lossyCast(u8, self.b * 255.0),
            .a = if (use_alpha) std.math.lossyCast(u8, self.a * 255.0) else 255,
        };
    }

    pub fn clamped(self: @This()) @This() {
        return .{
            .r = std.math.clamp(self.r, 0.0, 1.0),
            .g = std.math.clamp(self.g, 0.0, 1.0),
            .b = std.math.clamp(self.b, 0.0, 1.0),
            .a = std.math.clamp(self.a, 0.0, 1.0),
        };
    }
};

// Expects u and v to already be shifted, then, per the documentation:
//   R = Y + (11/8) × (V-128)
//   G = Y - 0.25 × (11/8) × (U-128) - 0.5 × (11/8) × (V-128)
//   B = Y + 1.25 × (11/8) × (U-128)
//   α= 255
pub inline fn _yuv(y: f32, u: f32, v: f32) RGBA {
    return .{
        .r = @intFromFloat(std.math.clamp(y + (11.0 / 8.0) * v, 0.0, 255.0)),
        .g = @intFromFloat(std.math.clamp(y - 0.25 * (11.0 / 8.0) * u - 0.5 * (11.0 / 8.0) * v, 0.0, 255.0)),
        .b = @intFromFloat(std.math.clamp(y + 1.25 * (11.0 / 8.0) * u, 0.0, 255.0)),
        .a = 255,
    };
}

pub inline fn yuv_to_rgba(yuv: YUV422) [2]RGBA {
    const v = @as(f32, @floatFromInt(yuv.v)) - 128.0;
    const u = @as(f32, @floatFromInt(yuv.u)) - 128.0;
    const y0: f32 = @floatFromInt(yuv.y0);
    const y1: f32 = @floatFromInt(yuv.y1);
    return .{ _yuv(y0, u, v), _yuv(y1, u, v) };
}
