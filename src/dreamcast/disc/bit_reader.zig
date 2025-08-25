const std = @import("std");

reader: *std.Io.Reader,
bits: u8 = 0,
count: u8 = 0,

const LowBitMasks = [9]u8{
    0b00000000,
    0b00000001,
    0b00000011,
    0b00000111,
    0b00001111,
    0b00011111,
    0b00111111,
    0b01111111,
    0b11111111,
};
const InvertedLowBitMasks = [8]u8{
    0b11111111,
    0b01111111,
    0b00111111,
    0b00011111,
    0b00001111,
    0b00000111,
    0b00000011,
    0b00000001,
};

pub fn init(reader: *std.Io.Reader) @This() {
    return .{ .reader = reader };
}

pub inline fn readBitsNoEof(self: *@This(), comptime T: type, num: u32) !T {
    std.debug.assert(self.count <= 8);
    std.debug.assert(num <= @bitSizeOf(T));

    const U = if (@bitSizeOf(T) < 8) u8 else std.meta.Int(.unsigned, @bitSizeOf(T));

    if (num <= self.count) return @intCast(self.removeBits(@intCast(num)));

    var bits_left: u32 = num - self.count;
    std.debug.assert(bits_left > 0 and bits_left <= @bitSizeOf(T));
    var out: U = self.flush();

    if (bits_left >= 8) {
        if (U == u8) {
            // Only possible case: num == 8 on an empty buffer
            std.debug.assert(num == 8);
            std.debug.assert(bits_left == 8);
            std.debug.assert(self.bits == 0);
            std.debug.assert(self.count == 0);
            return @intCast(try self.reader.takeByte());
        } else {
            const full_bytes_left = bits_left / 8;
            std.debug.assert(full_bytes_left <= @sizeOf(T));

            for (0..full_bytes_left) |_| {
                out <<= 8;
                out |= try self.reader.takeByte();
            }

            bits_left %= 8;
        }
    }
    if (bits_left == 0) return @intCast(out);

    std.debug.assert(bits_left > 0 and bits_left < 8);

    const final_byte = try self.reader.takeByte();
    const keep = 8 - bits_left;

    out <<= @intCast(bits_left);
    out |= final_byte >> @intCast(keep);
    self.bits = final_byte & LowBitMasks[keep];

    self.count = @intCast(keep);
    return @intCast(out);
}

inline fn removeBits(self: *@This(), num: u8) u8 {
    std.debug.assert(self.count <= 8);
    std.debug.assert(num <= 8);
    std.debug.assert(num <= self.count);

    const keep = self.count - num;
    const bits = self.bits >> @intCast(keep);
    self.bits &= LowBitMasks[keep];

    self.count = keep;
    return bits;
}

inline fn flush(self: *@This()) u8 {
    const bits = self.bits;
    self.bits = 0;
    self.count = 0;
    return bits;
}

pub inline fn alignToByte(self: *@This()) void {
    self.bits = 0;
    self.count = 0;
}

pub inline fn readUnary(self: *@This()) !u32 {
    std.debug.assert(self.count <= 8);
    // Also accounts for the self.count == 0 case.
    if (self.bits == 0) return self.count + try self.readUnaryFromEmptyBuffer();
    std.debug.assert(self.count > 0);
    const clz = @clz(self.bits);
    std.debug.assert(clz < 8);
    const r = clz - (8 - self.count);
    // Discard those bits and the 1
    self.count = 7 - clz;
    self.bits &= LowBitMasks[self.count];
    return r;
}

inline fn readUnaryFromEmptyBuffer(self: *@This()) !u32 {
    var unary_integer: u32 = 0;
    var bits = try self.reader.takeByte();
    while (bits == 0) { // <=> clz == 8
        unary_integer += 8;
        bits = try self.reader.takeByte();
    }
    const clz = @clz(bits);
    std.debug.assert(clz < 8);
    // Discard those bits and the 1
    self.count = 7 - clz;
    self.bits = bits & LowBitMasks[self.count];
    return unary_integer + clz;
}
