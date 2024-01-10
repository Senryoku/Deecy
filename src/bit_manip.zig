pub inline fn zero_extend(d: anytype) u32 {
    return @intCast(d);
}

pub inline fn sign_extension_u8(d: u8) i32 {
    if ((d & 0x80) == 0) {
        return @bitCast(0x000000FF & zero_extend(d));
    } else {
        return @bitCast(0xFFFFFF00 | zero_extend(d));
    }
}

pub inline fn sign_extension_u12(d: u12) i32 {
    if ((d & 0x800) == 0) {
        return @bitCast(0x00000FFF & zero_extend(d));
    } else {
        return @bitCast(0xFFFFF000 | zero_extend(d));
    }
}

pub inline fn sign_extension_u16(d: u16) i32 {
    if ((d & 0x8000) == 0) {
        return @bitCast(0x0000FFFF & zero_extend(d));
    } else {
        return @bitCast(0xFFFF0000 | zero_extend(d));
    }
}

pub inline fn as_i32(val: u32) i32 {
    return @bitCast(val);
}
