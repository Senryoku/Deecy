const std = @import("std");

pub const Color16 = packed union {
    value: u16,
    arbg1555: packed struct(u16) {
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
    b: u8,
    g: u8,
    r: u8,
    a: u8,
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

    pub fn apply_intensity(self: @This(), intensity: f32, use_alpha: bool) @This() {
        const clampled = @min(1.0, @max(0.0, intensity));
        return .{
            .r = clampled * self.r,
            .g = clampled * self.g,
            .b = clampled * self.b,
            .a = if (use_alpha) self.a else 1.0,
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

pub fn yuv_to_rgba(yuv: YUV422) [2]RGBA {
    const v = @as(f32, @floatFromInt(yuv.v)) - 128.0;
    const u = @as(f32, @floatFromInt(yuv.u)) - 128.0;
    const y0: f32 = @floatFromInt(yuv.y0);
    const y1: f32 = @floatFromInt(yuv.y1);
    return .{ _yuv(y0, u, v), _yuv(y1, u, v) };
}
