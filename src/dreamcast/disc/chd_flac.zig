const std = @import("std");
const builtin = @import("builtin");

// Adapted/Stripped down from https://github.com/Senryoku/zflac to decompress FLAC frames from CHD files.

const log = std.log.scoped(.chd_flac);
const log_frame = std.log.scoped(.chd_flac_frame);
const log_subframe = std.log.scoped(.chd_flac_subframe);
const log_residual = std.log.scoped(.chd_flac_residual);

const SampleRate = enum(u4) {
    StoredInMetadata = 0b0000, // Sample rate only stored in the streaminfo metadata block
    @"88.2kHz" = 0b0001, // 88.2 kHz
    @"176.4kHz" = 0b0010, // 176.4 kHz
    @"192kHz" = 0b0011, // 192 kHz
    @"8kHz" = 0b0100, // 8 kHz
    @"16kHz" = 0b0101, // 16 kHz
    @"22.05kHz" = 0b0110, // 22.05 kHz
    @"24kHz" = 0b0111, // 24 kHz
    @"32kHz" = 0b1000, // 32 kHz
    @"44.1kHz" = 0b1001, // 44.1 kHz
    @"48kHz" = 0b1010, // 48 kHz
    @"96kHz" = 0b1011, // 96 kHz
    Uncommon8b = 0b1100, // Uncommon sample rate in kHz, stored as an 8-bit number
    Uncommon16b = 0b1101, // Uncommon sample rate in Hz, stored as a 16-bit number
    Uncommon16bx10 = 0b1110, // Uncommon sample rate in Hz divided by 10, stored as a 16-bit number
    Forbidden = 0b1111, // Forbidden

    pub fn hz(self: @This()) u24 {
        return switch (self) {
            .@"88.2kHz" => 88200,
            .@"176.4kHz" => 176400,
            .@"192kHz" => 192000,
            .@"8kHz" => 8000,
            .@"16kHz" => 16000,
            .@"22.05kHz" => 22050,
            .@"24kHz" => 24000,
            .@"32kHz" => 32000,
            .@"44.1kHz" => 44100,
            .@"48kHz" => 48000,
            .@"96kHz" => 96000,
            .StoredInMetadata, .Uncommon8b, .Uncommon16b, .Uncommon16bx10, .Forbidden => unreachable,
        };
    }
};

const Channels = enum(u4) {
    Mono = 0b0000, // 1 channel: mono
    LR = 0b0001, // 2 channels: left, right
    LRC = 0b0010, // 3 channels: left, right, center
    @"4Channels" = 0b0011, // 4 channels: front left, front right, back left, back right
    @"5Channels" = 0b0100, // 5 channels: front left, front right, front center, back/surround left, back/surround right
    @"6Channels" = 0b0101, // 6 channels: front left, front right, front center, LFE, back/surround left, back/surround right
    @"7Channels" = 0b0110, // 7 channels: front left, front right, front center, LFE, back center, side left, side right
    @"8Channels" = 0b0111, // 8 channels: front left, front right, front center, LFE, back left, back right, side left, side right
    LRLeftSideStereo = 0b1000, // 2 channels: left, right; stored as left-side stereo
    LRSideRightStereo = 0b1001, // 2 channels: left, right; stored as side-right stereo
    LRMidSideStereo = 0b1010, // 2 channels: left, right; stored as mid-side stereo
    _, // Reserved

    pub fn count(self: @This()) u4 {
        return switch (self) {
            .Mono => 1,
            .LR => 2,
            .LRC => 3,
            .@"4Channels" => 4,
            .@"5Channels" => 5,
            .@"6Channels" => 6,
            .@"7Channels" => 7,
            .@"8Channels" => 8,
            .LRLeftSideStereo => 2,
            .LRSideRightStereo => 2,
            .LRMidSideStereo => 2,
            _ => 0,
        };
    }
};

const BitDepth = enum(u3) {
    StoredInMetadata = 0b000, // Bit depth only stored in the streaminfo metadata block
    @"8bps" = 0b001, // 8 bits per sample
    @"12bps" = 0b010, // 12 bits per sample
    Reserved = 0b011, // Reserved
    @"16bps" = 0b100, // 16 bits per sample
    @"20bps" = 0b101, // 20 bits per sample
    @"24bps" = 0b110, // 24 bits per sample
    @"32bps" = 0b111, // 32 bits per sample

    pub fn bps(self: @This()) u6 {
        return switch (self) {
            .@"8bps" => 8,
            .@"12bps" => 12,
            .@"16bps" => 16,
            .@"20bps" => 20,
            .@"24bps" => 24,
            .@"32bps" => 32,
            else => unreachable,
        };
    }
};

const BlockSize = enum(u4) {
    Reserved = 0b0000,
    @"192" = 0b0001,
    Uncommon8b = 0b0110,
    Uncommon16b = 0b0111,
    _,

    pub fn value(self: @This()) u16 {
        return switch (@intFromEnum(self)) {
            0b0001 => 192,
            0b0010...0b0101 => |b| 144 * std.math.pow(u16, 2, b),
            0b1000...0b1111 => |b| std.math.pow(u16, 2, b),
            else => unreachable,
        };
    }
};

const FrameHeader = packed struct(u32) {
    zero: u1,
    bit_depth: BitDepth,
    channels: Channels,
    sample_rate: SampleRate,
    block_size: BlockSize,
    blocking_strategy: enum(u1) { Fixed = 0, Variable = 1 },
    frame_sync: u15,
};

const SubframeHeader = packed struct {
    zero: u1,
    // 0b000000             Constant subframe
    // 0b000001             Verbatim subframe
    // 0b000010 - 0b000111  Reserved
    // 0b001000 - 0b001100  Subframe with a fixed predictor of order v-8; i.e., 0, 1, 2, 3 or 4
    // 0b001101 - 0b011111  Reserved
    // 0b100000 - 0b111111  Subframe with a linear predictor of order v-31; i.e., 1 through 32 (inclusive)
    subframe_type: u6,
    wasted_bit_flag: u1,
};

pub fn BitReader(comptime Reader: type) type {
    return struct {
        reader: Reader,
        bits: u8 = 0,
        count: u4 = 0,

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
                    return @intCast(try self.reader.readByte());
                } else {
                    const full_bytes_left = bits_left / 8;
                    std.debug.assert(full_bytes_left <= @sizeOf(T));

                    for (0..full_bytes_left) |_| {
                        out <<= 8;
                        out |= try self.reader.readByte();
                    }

                    bits_left %= 8;
                }
            }
            if (bits_left == 0) return @intCast(out);

            std.debug.assert(bits_left > 0 and bits_left < 8);

            const final_byte = try self.reader.readByte();
            const keep = 8 - bits_left;

            out <<= @intCast(bits_left);
            out |= final_byte >> @intCast(keep);
            self.bits = final_byte & LowBitMasks[keep];

            self.count = @intCast(keep);
            return @intCast(out);
        }

        inline fn removeBits(self: *@This(), num: u4) u8 {
            if (num == 8) return self.flush();

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
            // Also accounts for the self.count == 0 case.
            if (self.bits == 0) return self.count + try self.readUnaryFromEmptyBuffer();
            std.debug.assert(self.count > 0 and self.count <= 8);
            const clz = @clz(self.bits) - (8 - self.count);
            std.debug.assert(clz < 8);
            // Discard those bits and the 1
            self.count = self.count - 1 - clz;
            self.bits &= LowBitMasks[self.count];
            return clz;
        }

        inline fn readUnaryFromEmptyBuffer(self: *@This()) !u32 {
            var unary_integer: u32 = 0;
            var bits = try self.reader.readByte();
            while (bits == 0) { // <=> clz == 8
                unary_integer += 8;
                bits = try self.reader.readByte();
            }
            const clz = @clz(bits);
            std.debug.assert(clz < 8);
            // Discard those bits and the 1
            self.count = 8 - 1 - clz;
            self.bits = bits & LowBitMasks[self.count];
            return unary_integer + clz;
        }
    };
}

pub fn bitReader(reader: anytype) BitReader(@TypeOf(reader)) {
    return .{ .reader = reader };
}

/// Reads a signed integer with a runtime known bit depth
inline fn read_signed_integer(comptime T: type, bit_reader: anytype, bit_depth: u6) !T {
    std.debug.assert(bit_depth > 0 and bit_depth <= @bitSizeOf(T));
    const ContainerType = std.meta.Int(.unsigned, @bitSizeOf(T));
    var r = try bit_reader.readBitsNoEof(ContainerType, bit_depth);
    // Sign extend from bit_depth to container_type size
    const shift = @bitSizeOf(ContainerType) - @as(usize, bit_depth);
    r <<= @intCast(shift);
    return @as(T, @bitCast(r)) >> @intCast(shift);
}

inline fn read_unencoded_sample(comptime SampleType: type, bit_reader: anytype, wasted_bits: u6, bits_per_sample: u6) !SampleType {
    const InterType = std.meta.Int(.signed, try std.math.ceilPowerOfTwo(u32, @bitSizeOf(SampleType) + 1));
    return @intCast(try read_signed_integer(InterType, bit_reader, bits_per_sample - wasted_bits));
}

fn read_coded_number(reader: anytype) !u64 {
    const first_byte = try reader.readByte();
    const byte_count = @clz(first_byte ^ 0xFF);
    if (first_byte == 0xFF or byte_count == 1) return error.InvalidCodedNumber;
    if (byte_count == 0) return first_byte;
    var coded_number: u64 = (first_byte & (@as(u8, 0x7F) >> @intCast(byte_count)));
    for (0..byte_count - 1) |_| {
        coded_number <<= 6;
        coded_number |= (try reader.readByte()) & 0x3F;
    }
    return coded_number;
}

fn decode_residuals(comptime ResidualType: type, residuals: []ResidualType, block_size: u16, order: u6, bit_reader: anytype) !void {
    std.debug.assert(residuals.len >= block_size - order);

    const coding_method = try bit_reader.readBitsNoEof(u2, 2);
    if (coding_method >= 0b10) return error.InvalidResidualCodingMethod;
    const partition_order = try bit_reader.readBitsNoEof(u4, 4);

    log_residual.debug("    Residual decoding: Coding method: {s}, Partition order: {d}", .{ if (coding_method == 0b00) "Rice" else "Rice2", partition_order });

    var partition_start_idx: u32 = 0;
    for (0..std.math.pow(u32, 2, partition_order)) |partition| {
        var count = (block_size >> partition_order);
        if (partition == 0) count -= order;
        switch (coding_method) {
            inline 0b00, 0b01 => |comptime_coding_method| try decode_residual_partition(ResidualType, @enumFromInt(comptime_coding_method), residuals[partition_start_idx..][0..count], bit_reader),
            else => unreachable,
        }
        partition_start_idx += count;
    }
}

fn decode_residual_partition(comptime ResidualType: type, comptime coding_method: enum(u2) { Rice = 0, Rice2 = 1 }, residuals: []ResidualType, bit_reader: anytype) !void {
    const UnsignedResidualType = std.meta.Int(.unsigned, @bitSizeOf(ResidualType));
    const RiceParameterType = switch (coding_method) {
        .Rice => u4,
        .Rice2 => u5,
    };

    const rice_parameter = try bit_reader.readBitsNoEof(RiceParameterType, @bitSizeOf(RiceParameterType));
    log_residual.debug("      Partition: Rice parameter {d}", .{rice_parameter});

    switch (rice_parameter) {
        std.math.maxInt(RiceParameterType) => { // No rice parameter
            const bit_depth: u5 = try bit_reader.readBitsNoEof(u5, 5);
            if (bit_depth == 0) {
                @memset(residuals, 0);
            } else {
                for (0..residuals.len) |i|
                    residuals[i] = try read_signed_integer(ResidualType, bit_reader, bit_depth);
            }
        },
        inline else => |comptime_rice_parameter| {
            if (comptime_rice_parameter >= @bitSizeOf(UnsignedResidualType)) unreachable;
            for (0..residuals.len) |i| {
                const quotient: UnsignedResidualType = @intCast(try bit_reader.readUnary());
                const remainder = try bit_reader.readBitsNoEof(UnsignedResidualType, comptime_rice_parameter);
                const zigzag_encoded: UnsignedResidualType = (quotient << @intCast(comptime_rice_parameter)) + remainder;
                const residual: ResidualType = @bitCast((zigzag_encoded >> 1) ^ @as(UnsignedResidualType, @bitCast(-@as(ResidualType, @intCast(zigzag_encoded & 1)))));
                residuals[i] = residual;
            }
        },
    }
}

pub fn decode_frames(comptime SampleType: type, allocator: std.mem.Allocator, reader: anytype, output: []SampleType, num_samples: u32, expected_block_size: u16, expected_channel_count: u4, expected_bits_per_sample: u6) !void {
    // Larger type for intermediate computations
    const InterType = switch (SampleType) {
        i8 => i16,
        i16 => i32,
        i24, i32 => i64,
        else => @compileError("Unsupported sample type: " ++ @typeName(SampleType)),
    };

    // AFAIK these are fixed for CHDs. Keeping the distinction around just in case.
    const channel_count = expected_channel_count;
    const bits_per_sample = expected_bits_per_sample;

    var samples_working_buffer = try allocator.alloc(InterType, expected_block_size);
    defer allocator.free(samples_working_buffer);

    var frame_sample_offset: usize = 0;
    while (frame_sample_offset < num_samples) {
        const frame_header: FrameHeader = @bitCast(try reader.readInt(u32, .big));
        if (frame_header.frame_sync != (0xFFF8 >> 1))
            return error.InvalidFrameHeader; // NOTE: We could try to return normally when valid_total_sample_count is false here. The CRC check should catch if this was the wrong decision.

        const coded_number = try read_coded_number(&reader);

        const block_size: u16 = switch (frame_header.block_size) {
            .Reserved => return error.InvalidFrameHeader,
            .Uncommon8b => @as(u16, try reader.readInt(u8, .big)) + 1,
            .Uncommon16b => bs: {
                const ubs = try reader.readInt(u16, .big);
                if (ubs == std.math.maxInt(u16)) return error.InvalidFrameHeader;
                break :bs ubs + 1;
            },
            else => |b| b.value(),
        };

        const frame_sample_rate: u24 = switch (frame_header.sample_rate) {
            .StoredInMetadata => return error.InvalidFrameHeader,
            .Uncommon8b => try reader.readInt(u8, .big),
            .Uncommon16b => try reader.readInt(u16, .big),
            .Uncommon16bx10 => 10 * @as(u24, try reader.readInt(u16, .big)),
            .Forbidden => return error.InvalidFrameHeader,
            else => |sr| sr.hz(),
        };

        if (block_size != expected_block_size) return error.UnexpectedBlockSize;
        if (frame_header.channels.count() != expected_channel_count) return error.UnexpectedChannelCount;
        const frame_bit_depth = switch (frame_header.bit_depth) {
            .StoredInMetadata => return error.InvalidFrameHeader,
            else => |bd| bd.bps(),
        };
        if (frame_bit_depth != expected_bits_per_sample) return error.UnexpectedBitsPerSample;

        const frame_header_crc = try reader.readInt(u8, .big);
        // TODO: Check CRC
        // Finally, an 8-bit CRC follows the frame/sample number, an uncommon block size, or an uncommon sample rate (depending on whether the latter two are stored).
        // This CRC is initialized with 0 and has the polynomial x^8 + x^2 + x^1 + x^0. This CRC covers the whole frame header before the CRC, including the sync code.

        log_frame.debug("Frame: First sample: {d}, Sample Rate: {t} ({d}Hz), Channels: {t} ({d}), Bit Depth: {t} ({d}), Block Size: {s} ({d}), CRC: {X:0>2}, Coded Number ({s}): {d}", .{
            frame_sample_offset, frame_header.sample_rate, frame_sample_rate, frame_header.channels,
            channel_count,       frame_header.bit_depth,   bits_per_sample,   std.enums.tagName(BlockSize, frame_header.block_size) orelse "Common",
            block_size,          frame_header_crc,
            switch (frame_header.blocking_strategy) {
                .Fixed => "Frame number",
                .Variable => "Sample number",
            },
            coded_number,
        });

        var bit_reader = bitReader(reader);

        for (0..channel_count) |channel| {
            const subframe_header: SubframeHeader = .{
                .zero = try bit_reader.readBitsNoEof(u1, 1),
                .subframe_type = try bit_reader.readBitsNoEof(u6, 6),
                .wasted_bit_flag = try bit_reader.readBitsNoEof(u1, 1),
            };
            if (subframe_header.zero != 0) return error.InvalidSubframeHeader;

            const wasted_bits: u6 = if (subframe_header.wasted_bit_flag == 1) @intCast(try bit_reader.readUnary() + 1) else 0;

            // "When stereo decorrelation is used, the side channel will have one extra bit of bit depth"
            const unencoded_samples_bit_depth = switch (frame_header.channels) {
                .LRLeftSideStereo => if (channel == 1) bits_per_sample + 1 else bits_per_sample,
                .LRSideRightStereo => if (channel == 0) bits_per_sample + 1 else bits_per_sample,
                .LRMidSideStereo => if (channel == 1) bits_per_sample + 1 else bits_per_sample,
                else => bits_per_sample,
            };

            const subframe_samples = output[frame_sample_offset..][channel .. @as(usize, channel_count) * block_size];
            switch (subframe_header.subframe_type) {
                0b000000 => { // Constant subframe
                    log_subframe.debug("Subframe #{d}: Constant, {d} wasted bits", .{ channel, wasted_bits });
                    const sample = try read_unencoded_sample(SampleType, &bit_reader, wasted_bits, bits_per_sample);
                    if (channel_count == 1) {
                        @memset(subframe_samples, sample);
                    } else {
                        for (0..block_size) |i|
                            subframe_samples[channel_count * i] = sample;
                    }
                },
                0b000001 => { // Verbatim subframe
                    log_subframe.debug("Subframe #{d}: Verbatim subframe, {d} wasted bits", .{ channel, wasted_bits });
                    for (0..block_size) |i|
                        subframe_samples[channel_count * i] = try read_unencoded_sample(SampleType, &bit_reader, wasted_bits, unencoded_samples_bit_depth);
                },
                0b001000...0b001100 => |t| { // Subframe with a fixed predictor of order v-8; i.e., 0, 1, 2, 3 or 4
                    if (samples_working_buffer.len < block_size)
                        samples_working_buffer = try allocator.realloc(samples_working_buffer, block_size);

                    const order: u3 = @intCast(t & 0b000111);
                    if (order > 4) return error.InvalidSubframeHeader;
                    // Unencoded warm-up samples (n = subframe's bits per sample * LPC order).
                    for (0..order) |i|
                        samples_working_buffer[i] = try read_unencoded_sample(SampleType, &bit_reader, wasted_bits, unencoded_samples_bit_depth);

                    log_subframe.debug("Subframe #{d}: Fixed predictor of order {d}, {d} wasted bits", .{ channel, order, wasted_bits });
                    log_subframe.debug("  Warmup Samples: {any}", .{samples_working_buffer[0..order]});

                    try decode_residuals(InterType, samples_working_buffer[order..], block_size, order, &bit_reader);

                    for (order..block_size) |i| {
                        samples_working_buffer[i] += switch (order) {
                            0 => 0, // Just the residuals
                            1 => 1 * samples_working_buffer[i - 1],
                            2 => 2 * samples_working_buffer[i - 1] - 1 * samples_working_buffer[i - 2],
                            3 => 3 * samples_working_buffer[i - 1] - 3 * samples_working_buffer[i - 2] + 1 * samples_working_buffer[i - 3],
                            4 => 4 * samples_working_buffer[i - 1] - 6 * samples_working_buffer[i - 2] + 4 * samples_working_buffer[i - 3] - samples_working_buffer[i - 4],
                            else => unreachable,
                        };
                    }

                    // Interleave
                    for (0..block_size) |i| {
                        if (samples_working_buffer[i] < std.math.minInt(SampleType) or samples_working_buffer[i] > std.math.maxInt(SampleType))
                            return error.FLPCOverflow;
                        subframe_samples[channel_count * i] = @intCast(samples_working_buffer[i]);
                    }
                },
                0b100000...0b111111 => |t| { // Subframe with a linear predictor of order v-31; i.e., 1 through 32 (inclusive)
                    if (samples_working_buffer.len < block_size)
                        samples_working_buffer = try allocator.realloc(samples_working_buffer, block_size);

                    const order: u6 = @intCast(t - 31);
                    // Unencoded warm-up samples (n = subframe's bits per sample * LPC order).
                    for (0..order) |i|
                        samples_working_buffer[i] = try read_unencoded_sample(SampleType, &bit_reader, wasted_bits, unencoded_samples_bit_depth);
                    // (Predictor coefficient precision in bits)-1 (Note: 0b1111 is forbidden).
                    const coefficient_precision = (try bit_reader.readBitsNoEof(u4, 4)) + 1;
                    // Prediction right shift needed in bits.
                    const prediction_shift_right = try bit_reader.readBitsNoEof(u5, 5);
                    // Predictor coefficients (n = predictor coefficient precision * LPC order).
                    var predictor_coefficient: [32]InterType align(32) = @splat(0);
                    for (0..order) |i| // Stored in reverse order to match the layout of samples in memory (samples_working_buffer).
                        predictor_coefficient[order - 1 - i] = try read_unencoded_sample(InterType, &bit_reader, 0, coefficient_precision);

                    log_subframe.debug("Subframe #{d}: Linear predictor of order {d}, {d} bits coefficients, {d} bits right shift, {d} wasted bits", .{ channel, order, coefficient_precision, prediction_shift_right, wasted_bits });
                    log_subframe.debug("  Warmup Samples: {any}", .{samples_working_buffer[0..order]});
                    log_subframe.debug("  Predictor Coefficients: {any}", .{predictor_coefficient[0..order]});

                    try decode_residuals(InterType, samples_working_buffer[order..], block_size, order, &bit_reader);

                    switch (order) {
                        0 => {}, // Just the residuals
                        33...63 => unreachable,
                        inline else => |comptime_order| {
                            for (comptime_order..block_size) |i| {
                                var prediction: InterType = 0;
                                inline for (0..comptime_order) |o|
                                    prediction += samples_working_buffer[i - comptime_order + o] * predictor_coefficient[o];
                                samples_working_buffer[i] += prediction >> @intCast(prediction_shift_right);
                            }
                        },
                    }
                    // Interleave
                    for (0..block_size) |i| {
                        if (samples_working_buffer[i] < std.math.minInt(SampleType) or samples_working_buffer[i] > std.math.maxInt(SampleType))
                            return error.LPCOverflow;
                        subframe_samples[channel_count * i] = @intCast(samples_working_buffer[i]);
                    }
                },
                0b000010...0b000111, 0b001101...0b011111 => return error.InvalidSubframeHeader, // Reserved
            }

            if (wasted_bits > 0) {
                for (0..block_size) |i|
                    subframe_samples[channel_count * i] <<= @intCast(wasted_bits);
            }
        }
        // NOTE: Last subframe is padded with zero bits to be byte aligned.
        bit_reader.alignToByte();

        // Frame CRC
        _ = try reader.readInt(u16, .big);

        // Stereo decorrelation
        switch (frame_header.channels) {
            .LRLeftSideStereo => {
                for (0..block_size) |i| {
                    const idx = frame_sample_offset + channel_count * i;
                    output[idx + 1] = output[idx] - output[idx + 1];
                }
            },
            .LRSideRightStereo => {
                for (0..block_size) |i| {
                    const idx = frame_sample_offset + channel_count * i;
                    output[idx] += output[idx + 1];
                }
            },
            .LRMidSideStereo => {
                for (0..block_size) |i| {
                    const idx = frame_sample_offset + channel_count * i;
                    var mid = @as(InterType, output[idx]) << 1;
                    const side = output[idx + 1];
                    mid += side & 1;
                    output[idx] = @intCast((mid + side) >> 1);
                    output[idx + 1] = @intCast((mid - side) >> 1);
                }
            },
            else => {},
        }

        frame_sample_offset += @as(usize, @intCast(channel_count)) * block_size;
    }
}
