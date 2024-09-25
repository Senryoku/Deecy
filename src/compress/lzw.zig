const std = @import("std");

const Context = @import("Context.zig");
const bp = @import("./BitPacker.zig");

pub const BitPacker = bp.BitPacker(u64, u20, 9, 0);

pub fn compress(data: []const u8, allocator: std.mem.Allocator) !BitPacker {
    if (data.len == 0) return BitPacker.init(allocator);

    const first_allocated_token: BitPacker.ValueType = comptime std.math.maxInt(u8) + 1;
    var next_value: BitPacker.ValueType = first_allocated_token;
    var context = try Context.Context(BitPacker.ValueType).initCapacity(allocator, @as(usize, @min(std.math.maxInt(BitPacker.ValueType), data.len)));
    defer context.deinit();

    // Max string length in dictionary based on its first character.
    var max_length: [256]u16 = undefined;
    @memset(&max_length, 1);

    var output = try BitPacker.initCapacity(allocator, data.len);

    var i: usize = 0;
    while (true) {
        const d = data[i..];
        var curr_len: usize = 1;
        var prev_value: BitPacker.ValueType = (@as(BitPacker.ValueType, @intCast(d[0])));

        var max: usize = @min(d.len, max_length[d[0]]);
        // Binary search for the largest string in context
        while (max > curr_len + 1) {
            const mid: usize = curr_len + (max - curr_len) / 2;
            if (context.get(d[0..mid])) |value| {
                curr_len = mid;
                prev_value = value;
            } else {
                max = mid - 1;
            }
        }

        if (curr_len < max) {
            if (context.get(d[0..max])) |value| {
                prev_value = value;
                curr_len = max;
            }
        }

        output.appendAssumeCapacity(prev_value);
        i += curr_len;

        if (curr_len >= d.len - 1) break;

        if (next_value != std.math.maxInt(BitPacker.ValueType)) {
            const str = d[0 .. curr_len + 1];
            max_length[str[0]] = @max(max_length[str[0]], @as(u16, @intCast(str.len)));
            context.putAssumeCapacityNoClobber(str, next_value);
            next_value += 1;
        } else {
            // Restart compression from here with a fresh context
            context.clearRetainingCapacity();
            @memset(&max_length, 1);
            // Insert special token signifying a context reset for decompression.
            output.appendAssumeCapacity(std.math.maxInt(BitPacker.ValueType));

            output.resetValueSize();

            next_value = first_allocated_token;
        }
    }

    // Handle the (potentially) last unencoded byte.
    if (i < data.len) {
        std.debug.assert(i == data.len - 1);
        output.appendAssumeCapacity(@intCast(data[i]));
    }

    return output;
}

// Caller owns the returned ArrayList
pub fn decompress(data: []const u8, token_count: usize, expected_size: usize, allocator: std.mem.Allocator) !std.ArrayList(u8) {
    const packed_data = try BitPacker.fromSlice(allocator, @as([*]const BitPacker.UnderlyingType, @alignCast(@ptrCast(data.ptr)))[0 .. data.len / @sizeOf(BitPacker.UnderlyingType)], token_count);

    const unpacked_data = try packed_data.unpackWithReset(allocator, std.math.maxInt(BitPacker.ValueType));
    defer allocator.free(unpacked_data);

    if (unpacked_data.len != token_count)
        return error.UnexpectedTokenCount;

    const decompressed = try decompressImpl(BitPacker.ValueType, 0, std.math.maxInt(BitPacker.ValueType), unpacked_data, expected_size, allocator);

    if (decompressed.items.len != expected_size)
        return error.UnexpectedDecompressedSize;

    return decompressed;
}

pub fn decompressImpl(comptime TokenType: type, comptime reserved_codepoints: TokenType, comptime sentinel_token: TokenType, data: []const TokenType, expected_output_size: usize, allocator: std.mem.Allocator) !std.ArrayList(u8) {
    if (data.len == 0) return std.ArrayList(u8).init(allocator);

    const first_allocated_token: TokenType = comptime std.math.maxInt(u8) + 1 + reserved_codepoints;
    var next_value: TokenType = first_allocated_token;
    var context = try std.ArrayList(?[]const u8).initCapacity(allocator, @min(std.math.maxInt(TokenType), first_allocated_token + data.len));
    defer context.deinit();
    context.appendNTimesAssumeCapacity(null, context.capacity);

    var output = try std.ArrayList(u8).initCapacity(allocator, expected_output_size);
    output.appendAssumeCapacity(@intCast(data[0] - reserved_codepoints));

    context.items[data[0]] = output.items[0..1];

    var prev_start: usize = 0;
    var i: usize = 1;
    while (i < data.len) {
        const v = data[i];
        const new_start = output.items.len;

        if (v != sentinel_token) {
            if (v < first_allocated_token) {
                output.appendAssumeCapacity(@intCast(v - reserved_codepoints));
            } else {
                if (context.items[v]) |str| {
                    output.appendSliceAssumeCapacity(str);
                } else {
                    // I think the only case where this might happen, is repeating characters.
                    // For example, 'aaaa' will be encoded as [129, 288, 129], with 288 representing 'aa'.
                    // However the decoder will never have encountered 'aa' before.
                    output.appendSliceAssumeCapacity(output.items[prev_start..new_start]);
                    output.appendAssumeCapacity(output.items[prev_start]);
                }
            }

            //                          equivalent to concat(prev_str, str[0])
            context.items[next_value] = output.items[prev_start .. new_start + 1];
            next_value += 1;
        } else {
            i += 1; // Skip special reset token
            if (i >= data.len) break;
            // Reinitialize state
            next_value = first_allocated_token;
            context.clearRetainingCapacity();
            context.appendNTimesAssumeCapacity(null, context.capacity);
            try std.testing.expect(data[i] < first_allocated_token);
            output.appendAssumeCapacity(@intCast(data[i] - reserved_codepoints));
        }

        prev_start = new_start;
        i += 1;
    }

    return output;
}

fn testRound(str: []const u8) !void {
    var compressed = try compress(str, std.testing.allocator);
    defer compressed.deinit();
    const decompressed = try decompress(std.mem.sliceAsBytes(compressed.arr.items), compressed.size, str.len, std.testing.allocator);
    defer decompressed.deinit();
    try std.testing.expectEqualSlices(u8, str, decompressed.items);
}

test "basic" {
    try testRound("");
    try testRound("a");
    try testRound("aa");
    try testRound("aaaa");
    try testRound("aaaaaa");
    try testRound("aabbaaccaaccaavvbbaaxaaaavwaa");
    try testRound("1212121");
    try testRound("3737373");
    try testRound("3333737370");
    try testRound("3333737370000000000000000000000");
    try testRound("12121212");
    try testRound("37373737");
    try testRound("33337373737");
    try testRound("3333737373700000000000000000000");
    try testRound("3333773737373777777373773737373");
    try testRound("\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}\u{0}");
}

test "fuzzing" {
    // Doesn't ensure that the string is valid UTF-8, but it should not matter.
    // Note: In the future, use std.testing.random_seed. See https://github.com/ziglang/zig/issues/17609.
    const seed = std.crypto.random.int(u64);
    errdefer std.debug.print("\nFuzzing Test FAILED\n\tSeed: {d}\n", .{seed});
    var rng = std.rand.DefaultPrng.init(seed);
    for (0..1) |_| {
        const length = rng.random().intRangeAtMost(usize, 0, 10_000_000); // Up to ~10MB
        const str = try std.testing.allocator.alloc(u8, length);
        defer std.testing.allocator.free(str);
        rng.fill(str);
        try testRound(str);
    }
}

fn testFile(path: []const u8) !void {
    const str = try std.fs.cwd().readFileAlloc(std.testing.allocator, path, 1e8);
    defer std.testing.allocator.free(str);
    try testRound(str);
}

test "files" {
    try testFile("logs/test_save_uncompressed.bin");
}
