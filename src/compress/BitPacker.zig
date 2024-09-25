const std = @import("std");

fn BitMasks(comptime T: type) [@bitSizeOf(T) + 1]T {
    var masks: [@bitSizeOf(T) + 1]T = undefined;
    masks[0] = 0;
    for (1..@bitSizeOf(T)) |i| {
        masks[i] = ~@as(T, 0) >> @intCast(@bitSizeOf(T) - i);
    }
    masks[@bitSizeOf(T)] = ~@as(T, 0);
    return masks;
}

// initial_bit_size: Determine the initial expected max values.
// reserved_bits: Skip this number of bits in each array item. Reduces the packer efficiency to produce valid values for your target encoding.
pub fn BitPacker(comptime _UnderlyingType: type, comptime _ValueType: type, comptime initial_bit_size: u8, comptime reserved_bits: u8) type {
    return struct {
        arr: std.ArrayList(UnderlyingType),

        size: usize = 0,
        size_since_reset: usize = 0,
        value_size: u8 = initial_bit_size,
        bit: u8 = reserved_bits, // Current bit position

        const Self = @This();
        pub const UnderlyingType = _UnderlyingType;
        pub const ValueType = _ValueType;
        pub const InitialBitSize = initial_bit_size;
        pub const ReservedBits = reserved_bits;

        pub fn bitsPerItem() u8 {
            return @bitSizeOf(UnderlyingType) - reserved_bits;
        }

        pub fn init(allocator: std.mem.Allocator) !@This() {
            var r = @This(){
                .arr = std.ArrayList(UnderlyingType).init(allocator),
            };
            try r.arr.append(0);
            return r;
        }

        // Initial capacity in number of values (approximate upper bound).
        pub fn initCapacity(allocator: std.mem.Allocator, initial_capacity: usize) !@This() {
            const bit_count = @min(@bitSizeOf(ValueType), @as(u7, @intFromFloat(@log2(@as(f32, @floatFromInt(initial_capacity)) + 1))));
            const capacity_in_underlying_type: u32 = @intCast(1 + (bit_count * initial_capacity + @bitSizeOf(UnderlyingType)) / @bitSizeOf(UnderlyingType));
            var r = @This(){
                .arr = try std.ArrayList(UnderlyingType).initCapacity(allocator, capacity_in_underlying_type),
            };
            r.arr.appendAssumeCapacity(0);
            return r;
        }

        // FIXME: This should return a const version of @This()
        pub fn fromSlice(allocator: std.mem.Allocator, data: []const UnderlyingType, size: usize) !@This() {
            return @This(){
                .arr = std.ArrayList(UnderlyingType).fromOwnedSlice(allocator, @constCast(data)),
                .size = size,
                .size_since_reset = 0,
                .value_size = 0, // FIXME: This is intended to be read-only. Passing a wrong value on purpose here.
            };
        }

        pub fn unpack(self: @This(), allocator: std.mem.Allocator) ![]const ValueType {
            var r = try std.ArrayList(ValueType).initCapacity(allocator, self.size);
            var it = self.iterator();
            while (it.next()) |v| {
                r.appendAssumeCapacity(v);
            }
            return r.toOwnedSlice();
        }

        pub fn unpackWithReset(self: @This(), allocator: std.mem.Allocator, sentinel_token: ValueType) ![]const ValueType {
            var r = try std.ArrayList(ValueType).initCapacity(allocator, self.size);
            var it = self.iterator();
            while (it.next()) |v| {
                r.appendAssumeCapacity(v);
                if (v == sentinel_token) {
                    it.resetValueSize();
                }
            }
            return r.toOwnedSlice();
        }

        pub fn deinit(self: *@This()) void {
            self.arr.deinit();
        }

        pub fn resetValueSize(self: *@This()) void {
            self.value_size = initial_bit_size;
            self.size_since_reset = 0;
        }

        pub fn append(self: *@This(), value: ValueType) !void {
            // We're being conservative here. Prefer using appendAssumeCapacity directly.
            if (self.bit + self.value_size + 1 >= @bitSizeOf(UnderlyingType))
                try self.arr.ensureTotalCapacity(self.arr.items.len + 1);
            self.appendAssumeCapacity(value);
        }

        pub fn appendAssumeCapacity(self: *@This(), value: ValueType) void {
            if (self.value_size < @bitSizeOf(ValueType) and self.size_since_reset + (comptime std.math.pow(usize, 2, @max(0, initial_bit_size - 1))) >= (@as(u32, 1) << @intCast(self.value_size)) - 1) {
                self.value_size += 1;
            }

            std.debug.assert(self.value_size <= @bitSizeOf(ValueType));
            std.debug.assert(self.value_size == @bitSizeOf(ValueType) or (value >> @intCast(self.value_size)) == 0);

            // We know a value cannot span more than two underlying items.
            if (comptime (@bitSizeOf(ValueType) <= @bitSizeOf(UnderlyingType) - reserved_bits)) {
                const available_bits = @bitSizeOf(UnderlyingType) - self.bit;
                if (self.value_size <= available_bits) {
                    self.arr.items[self.arr.items.len - 1] |= @as(UnderlyingType, @intCast(value)) << @truncate(@bitSizeOf(UnderlyingType) - (self.value_size + self.bit));
                    self.bit += self.value_size;
                } else {
                    if (available_bits > 0)
                        self.arr.items[self.arr.items.len - 1] |= value >> @intCast(self.value_size - available_bits);
                    const shifted = @as(UnderlyingType, @intCast(value)) << @intCast(@bitSizeOf(UnderlyingType) - (self.value_size - available_bits));
                    self.arr.appendAssumeCapacity(shifted >> reserved_bits);
                    self.bit = reserved_bits + self.value_size - available_bits;
                }
            } else {
                var remaining_bits = self.value_size;
                while (remaining_bits > 0) {
                    if (self.bit == @bitSizeOf(UnderlyingType)) {
                        self.arr.appendAssumeCapacity(0);
                        self.bit = reserved_bits;
                    }

                    const to_write = @min(remaining_bits, @bitSizeOf(UnderlyingType) - self.bit);

                    var shifted: ValueType = value << @intCast(@bitSizeOf(ValueType) - remaining_bits); // "Mask" high bits
                    shifted >>= @intCast(self.bit + (@bitSizeOf(ValueType) - @bitSizeOf(UnderlyingType)));
                    self.arr.items[self.arr.items.len - 1] |= @intCast(shifted);

                    remaining_bits -= to_write;

                    self.bit += to_write;
                }
            }

            self.size += 1;
            self.size_since_reset += 1;
        }

        const Iterator = struct {
            bp: *const Self,

            index: usize,
            index_since_reset: usize,
            arr_index: usize,
            bit: u8,
            value_size: u8,

            pub fn resetValueSize(self: *@This()) void {
                self.value_size = initial_bit_size;
                self.index_since_reset = 0;
            }

            pub fn next(self: *@This()) ?ValueType {
                if (self.index >= self.bp.size) return null;

                if (self.value_size < @bitSizeOf(ValueType) and self.index_since_reset + (comptime std.math.pow(usize, 2, @max(0, initial_bit_size - 1))) >= (@as(u32, 1) << @intCast(self.value_size)) - 1) {
                    self.value_size += 1;
                }

                self.index += 1;
                self.index_since_reset += 1;

                if (@bitSizeOf(UnderlyingType) - reserved_bits > @bitSizeOf(ValueType)) {
                    // Here, we now we'll extract the value from at most 2 underlying items.
                    const available_bits = (@bitSizeOf(UnderlyingType) - self.bit);
                    if (self.value_size <= available_bits) { // Single item case
                        const output: ValueType = @truncate((self.bp.arr.items[self.arr_index] >> @intCast(available_bits - self.value_size)) & (comptime BitMasks(ValueType))[self.value_size]);
                        if (available_bits == self.value_size) {
                            self.arr_index += 1;
                            self.bit = reserved_bits;
                        } else self.bit += self.value_size;
                        return output;
                    } else {
                        const next_bits = self.value_size - available_bits;
                        var output: ValueType = @truncate((self.bp.arr.items[self.arr_index] & (comptime BitMasks(ValueType))[available_bits]) << @intCast(next_bits));
                        output |= @truncate((self.bp.arr.items[self.arr_index + 1] >> @intCast(@bitSizeOf(UnderlyingType) - (reserved_bits + next_bits))) & (comptime BitMasks(ValueType))[next_bits]);
                        self.arr_index += 1;
                        self.bit = reserved_bits + next_bits;
                        return output;
                    }
                } else {
                    var output: ValueType = 0;
                    var remaining_bits = self.value_size;
                    while (remaining_bits > 0) {
                        if (self.bit == @bitSizeOf(UnderlyingType)) {
                            self.arr_index += 1;
                            self.bit = reserved_bits;
                        }

                        const to_output = @min(remaining_bits, @bitSizeOf(UnderlyingType) - self.bit);
                        var shifted = self.bp.arr.items[self.arr_index] << @intCast(self.bit); // "Mask" upper bits
                        shifted >>= @intCast(@bitSizeOf(UnderlyingType) - to_output); // "Mask" lower bits
                        output |= @as(ValueType, @intCast(shifted)) << @intCast(remaining_bits - to_output);
                        remaining_bits -= to_output;
                        self.bit += to_output;
                    }
                    return output;
                }
            }
        };

        pub fn iterator(self: *const @This()) Iterator {
            return Iterator{
                .bp = self,
                .index = 0,
                .index_since_reset = 0,
                .arr_index = 0,
                .bit = reserved_bits,
                .value_size = initial_bit_size,
            };
        }
    };
}

// NOTE: (/FIXME) 'expected' and 'actual' are flipped in all these expectEqual calls. Type isn't coerced correctly otherwise, and I'd rather have a weird error message than writing a workaround for that right now.

test "basics" {
    var bp = try BitPacker(u16, u16, 3, 0).init(std.testing.allocator);
    defer bp.deinit();

    try bp.append(2);
    try std.testing.expectEqual(bp.arr.items[0], 2 << @intCast(16 - 3));
    try std.testing.expectEqual(bp.bit, 3);

    try bp.append(3);
    try std.testing.expectEqual(bp.arr.items[0], (2 << @intCast(16 - 3)) | (3 << @intCast(16 - 3 * 2)));
    try std.testing.expectEqual(bp.bit, 6);

    var it = bp.iterator();
    try std.testing.expectEqual(it.next(), 2);
    try std.testing.expectEqual(it.next(), 3);
}

test "reserved bits" {
    const reserved_bits = 1;
    var bp = try BitPacker(u16, u16, 3, reserved_bits).init(std.testing.allocator);
    defer bp.deinit();

    try bp.append(2);
    try std.testing.expectEqual(bp.arr.items[0], 2 << @intCast(16 - 3 - reserved_bits));
    try std.testing.expectEqual(bp.bit, 3 + reserved_bits);

    try bp.append(3);
    try std.testing.expectEqual(bp.arr.items[0], (2 << @intCast(16 - 3 - reserved_bits)) | (3 << @intCast(16 - 3 * 2 - reserved_bits)));
    try std.testing.expectEqual(bp.bit, 6 + reserved_bits);

    var it = bp.iterator();
    try std.testing.expectEqual(it.next(), 2);
    try std.testing.expectEqual(it.next(), 3);
}

test "incrementing bit size" {
    var bp = try BitPacker(u16, u16, 3, 0).init(std.testing.allocator);
    defer bp.deinit();

    for (0..64) |v| {
        try bp.append(@intCast(v));
    }

    var it = bp.iterator();
    for (0..64) |v| {
        try std.testing.expectEqual(it.next(), @as(u16, @intCast(v)));
    }
}

test "incrementing bit size and reserved bits" {
    var bp = try BitPacker(u16, u16, 3, 1).init(std.testing.allocator);
    defer bp.deinit();

    for (0..64) |v| {
        try bp.append(@intCast(v));
    }

    // Make sure the reserved bit is unset
    for (bp.arr.items) |v| {
        try std.testing.expectEqual(v >> 15, 0);
    }

    var it = bp.iterator();
    for (0..64) |v| {
        try std.testing.expectEqual(it.next(), @as(u16, @intCast(v)));
    }
}

test "initial_bit_size(2) and reserved_bits(0)" {
    var bp = try BitPacker(u16, u16, 2, 0).init(std.testing.allocator);
    defer bp.deinit();

    for (0..32768) |v| {
        try bp.append(@intCast(v));
    }

    var it = bp.iterator();
    for (0..32768) |v| {
        try std.testing.expectEqual(it.next(), @as(u16, @intCast(v)));
    }
}

test "initial_bit_size(9) and reserved_bits(1)" {
    var bp = try BitPacker(u16, u16, 9, 1).init(std.testing.allocator);
    defer bp.deinit();

    for (0..32768) |v| {
        try bp.append(@intCast(v));
    }

    // Make sure the reserved bit is unset
    for (bp.arr.items) |v| {
        try std.testing.expectEqual(v >> 15, 0);
    }

    var it = bp.iterator();
    for (0..32768) |v| {
        try std.testing.expectEqual(it.next(), @as(u16, @intCast(v)));
    }
}

test "underlying_type(u16), value_type(u32), initial_bit_size(9) and reserved_bits(1)" {
    var bp = try BitPacker(u16, u32, 9, 1).init(std.testing.allocator);
    defer bp.deinit();

    for (0..131072) |v| {
        try bp.append(@intCast(v));
    }

    // Make sure the reserved bit is unset
    for (bp.arr.items) |v| {
        try std.testing.expectEqual(v >> 15, 0);
    }

    var it = bp.iterator();
    for (0..131072) |v| {
        try std.testing.expectEqual(it.next(), @as(u32, @intCast(v)));
    }

    const unpacked = try bp.unpack(std.testing.allocator);
    defer std.testing.allocator.free(unpacked);

    for (unpacked) |v| {
        try std.testing.expectEqual(v, @as(u32, @intCast(v)));
    }
}
