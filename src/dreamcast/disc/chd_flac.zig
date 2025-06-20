const std = @import("std");
const builtin = @import("builtin");

// Adapted/Stripped down from https://github.com/Senryoku/zflac to decompress FLAC frames from CHD files.
// (Exact commit: c494f37e76192fea694de2f30c0dd7ea2ce6558c, 20/06/2025)

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

const FrameHeader = packed struct(u32) {
    zero: u1,
    bit_depth: BitDepth,
    channels: Channels,
    sample_rate: SampleRate,
    block_size: u4,
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

const low_bit_mask = [_]u8{
    0b00000000,
    0b00000001,
    0b00000011,
    0b00000111,
    0b00001111,
    0b00011111,
    0b00111111,
    0b01111111,
};

inline fn read_unary_integer_from_empty_buffer(bit_reader: anytype) !u32 {
    std.debug.assert(bit_reader.count == 0);
    if (try bit_reader.readBitsNoEof(u1, 1) == 0) { // Force fetching the next byte
        std.debug.assert(bit_reader.count == 7);
        var unary_integer: u32 = 1;
        const buffered_bits = (bit_reader.bits << @intCast(8 - bit_reader.count)) | low_bit_mask[8 - bit_reader.count];
        const clz = @clz(buffered_bits);
        unary_integer += clz;
        if (clz == bit_reader.count) {
            bit_reader.alignToByte(); // Discard those 0 bits
            while (try bit_reader.readBitsNoEof(u1, 1) == 0) unary_integer += 1; // Revert to simple version
        } else {
            _ = try bit_reader.readBitsNoEof(u8, clz + 1); // Discard those bits and the 1
        }
        return unary_integer;
    }
    return 0;
}

inline fn read_unary_integer(bit_reader: anytype) !u32 {
    if (false) {
        // Easy to read and portable version.
        var unary_integer: u32 = 0;
        while (try bit_reader.readBitsNoEof(u1, 1) == 0) unary_integer += 1;
        return unary_integer;
    } else {
        // Faster version relying on the internals of std.io.bitReader
        if (bit_reader.count == 0)
            return try read_unary_integer_from_empty_buffer(bit_reader);
        const buffered_bits = (bit_reader.bits << @intCast(8 - bit_reader.count)) | low_bit_mask[8 - bit_reader.count];
        const clz = @clz(buffered_bits);
        var unary_integer: u32 = clz;
        if (clz == bit_reader.count) {
            bit_reader.alignToByte(); // Discard those 0 bits
            if (false) {
                // Revert to simple version immediately
                while (try bit_reader.readBitsNoEof(u1, 1) == 0) unary_integer += 1;
            } else {
                // Inline a second round before reverting (NOTE: a recursive call would prevent inlining and seems to be slower)
                unary_integer += try read_unary_integer_from_empty_buffer(bit_reader);
            }
            return unary_integer;
        } else {
            _ = try bit_reader.readBitsNoEof(u8, clz + 1); // Discard those bits and the 1
            return unary_integer;
        }
    }
}

/// Reads a signed integer with a runtime known bit depth
inline fn read_signed_integer(comptime T: type, bit_reader: anytype, bit_depth: u6) !T {
    if (bit_depth == 0) return 0;
    const container_type = switch (@bitSizeOf(T)) {
        8 => u8,
        16 => u16,
        32 => u32,
        64 => u64,
        else => @compileError("Unsupported container type: " ++ @typeName(T)),
    };

    var r = try bit_reader.readBitsNoEof(container_type, bit_depth);
    // Sign extend from bit_depth to container_type size
    if ((@as(container_type, 1) << @intCast(bit_depth - 1)) & r != 0) r |= @as(container_type, @truncate(0xFFFFFFFFFFFFFFFF)) << @intCast(bit_depth);
    return @bitCast(r);
}

inline fn read_unencoded_sample(comptime SampleType: type, bit_reader: anytype, wasted_bits: u6, bits_per_sample: u6) !SampleType {
    const InterType = switch (SampleType) {
        i8 => i16,
        i16 => i32,
        i24 => i32,
        i32 => i64,
        else => @compileError("Unsupported sample type: " ++ @typeName(SampleType)),
    };
    if (wasted_bits > 0) {
        return @intCast(try read_signed_integer(InterType, bit_reader, bits_per_sample - wasted_bits));
    } else {
        return @intCast(try read_signed_integer(InterType, bit_reader, bits_per_sample));
    }
}

fn decode_residuals(comptime ResidualType: type, residuals: []ResidualType, block_size: u16, order: u6, bit_reader: anytype) !void {
    std.debug.assert(residuals.len >= block_size);

    const coding_method = try bit_reader.readBitsNoEof(u2, 2);
    if (coding_method >= 0b10) return error.InvalidResidualCodingMethod;
    const partition_order = try bit_reader.readBitsNoEof(u4, 4);

    log_residual.debug("    Residual decoding. coding_method: {d}, partition_order: {d}", .{ coding_method, partition_order });

    var partition_start_idx: u32 = 0;
    for (0..std.math.pow(u32, 2, partition_order)) |partition| {
        var count = (block_size >> partition_order);
        if (partition == 0) count -= order;
        const rice_parameter: u5 = try bit_reader.readBitsNoEof(u5, switch (coding_method) {
            0b00 => 4,
            0b01 => 5,
            else => unreachable,
        });
        log_residual.debug("      partition[{d}]: rice_parameter: {d}", .{ partition, rice_parameter });
        if ((coding_method == 0b00 and rice_parameter == 0b1111) or (coding_method == 0b01 and rice_parameter == 0b11111)) {
            // No rice parameter
            const bit_depth: u5 = try bit_reader.readBitsNoEof(u5, 5);
            if (bit_depth == 0) {
                @memset(residuals[partition_start_idx..][0..count], 0);
            } else {
                for (0..count) |i|
                    residuals[partition_start_idx + i] = try read_signed_integer(ResidualType, bit_reader, bit_depth);
            }
        } else {
            const UnsignedResidualType = std.meta.Int(.unsigned, @bitSizeOf(ResidualType));
            for (0..count) |i| {
                const quotient: UnsignedResidualType = @intCast(try read_unary_integer(bit_reader));
                const remainder = try bit_reader.readBitsNoEof(UnsignedResidualType, rice_parameter);
                const zigzag_encoded: UnsignedResidualType = (quotient << @intCast(rice_parameter)) + remainder;
                const residual: ResidualType = @bitCast((zigzag_encoded >> 1) ^ @as(UnsignedResidualType, @bitCast(-@as(ResidualType, @intCast(zigzag_encoded & 1)))));
                residuals[partition_start_idx + i] = residual;
            }
        }
        partition_start_idx += count;
    }
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

pub fn decode_frames(comptime SampleType: type, allocator: std.mem.Allocator, reader: anytype, output: []SampleType, num_samples: u32, expected_block_size: u16) !void {
    // Larger type for intermediate computations
    const InterType = switch (SampleType) {
        i8 => i16,
        i16 => i32,
        i24, i32 => i64,
        else => @compileError("Unsupported sample type: " ++ @typeName(SampleType)),
    };

    var first_frame = true;
    var channel_count: u4 = 2;
    var bits_per_sample: u6 = 16;

    var residuals = try allocator.alloc(InterType, expected_block_size);
    defer allocator.free(residuals);

    var frame_sample_offset: usize = 0;
    while (frame_sample_offset < num_samples) {
        const frame_header: FrameHeader = @bitCast(try reader.readInt(u32, .big));
        log_frame.debug("frame header: {any} (frame_sample_offset: {d})", .{ frame_header, frame_sample_offset });

        const coded_number = try read_coded_number(&reader);
        log_frame.debug("  Coded number: {d}", .{coded_number});

        var bit_reader = std.io.bitReader(.big, reader);

        const block_size: u16 = switch (frame_header.block_size) {
            0b0000 => return error.InvalidFrameHeader, // Reserved
            0b0001 => 192,
            0b0010...0b0101 => |b| 144 * std.math.pow(u16, 2, b),
            // Uncommon block size
            0b0110 => @as(u16, try reader.readInt(u8, .big)) + 1,
            0b0111 => bs: {
                const ubs = try reader.readInt(u16, .big);
                if (ubs == std.math.maxInt(u16)) return error.InvalidFrameHeader;
                break :bs ubs + 1;
            },
            0b1000...0b1111 => |b| std.math.pow(u16, 2, b),
        };
        log_frame.debug("  block_size: {d}", .{block_size});

        const frame_sample_rate: u24 = switch (frame_header.sample_rate) {
            .StoredInMetadata => return error.InvalidFrameHeader,
            .Uncommon8b => try reader.readInt(u8, .big),
            .Uncommon16b => try reader.readInt(u16, .big),
            .Uncommon16bx10 => 10 * @as(u24, try reader.readInt(u16, .big)),
            .Forbidden => return error.InvalidFrameHeader,
            else => |sr| sr.hz(),
        };
        log_frame.debug("  frame_sample_rate: {d}", .{frame_sample_rate});

        if (first_frame) {
            channel_count = frame_header.channels.count();
            bits_per_sample = switch (frame_header.bit_depth) {
                .StoredInMetadata => return error.InvalidFrameHeader,
                else => |bd| bd.bps(),
            };

            first_frame = false;
        }
        // Block size of 1 not allowed except for the last frame.
        if (block_size == 1 and frame_sample_offset + channel_count * block_size < num_samples) return error.InvalidFrameHeader;

        // Frame header CRC
        _ = try reader.readInt(u8, .big);

        for (0..channel_count) |channel| {
            const subframe_header: SubframeHeader = .{
                .zero = try bit_reader.readBitsNoEof(u1, 1),
                .subframe_type = try bit_reader.readBitsNoEof(u6, 6),
                .wasted_bit_flag = try bit_reader.readBitsNoEof(u1, 1),
            };
            log_subframe.debug("  subframe_header[{d}]: {any} (first sample: {d})", .{ channel, subframe_header, frame_sample_offset + channel });
            if (subframe_header.zero != 0)
                return error.InvalidSubframeHeader;

            var wasted_bits: u6 = 0;
            if (subframe_header.wasted_bit_flag == 1) {
                wasted_bits = @intCast(try read_unary_integer(&bit_reader) + 1);
                log_subframe.debug("  wasted_bits: {d}", .{wasted_bits});
            }

            const unencoded_samples_bit_depth = switch (frame_header.channels) {
                .LRLeftSideStereo => if (channel == 1) bits_per_sample + 1 else bits_per_sample,
                .LRSideRightStereo => if (channel == 0) bits_per_sample + 1 else bits_per_sample,
                .LRMidSideStereo => if (channel == 1) bits_per_sample + 1 else bits_per_sample,
                else => bits_per_sample,
            };

            switch (subframe_header.subframe_type) {
                0b000000 => { // Constant subframe
                    const sample = try read_unencoded_sample(SampleType, &bit_reader, wasted_bits, bits_per_sample);
                    if (channel_count == 1) {
                        @memset(output[frame_sample_offset..][0..block_size], sample);
                    } else {
                        for (0..block_size) |i| {
                            output[frame_sample_offset + channel_count * i + channel] = sample;
                        }
                    }
                },
                0b000001 => { // Verbatim subframe
                    for (0..block_size) |i| {
                        output[frame_sample_offset + channel_count * i + channel] = try read_unencoded_sample(SampleType, &bit_reader, wasted_bits, unencoded_samples_bit_depth);
                        log_subframe.debug("    sample: {d}", .{output[frame_sample_offset + channel_count * i + channel]});
                    }
                },
                0b001000...0b001100 => |t| { // Subframe with a fixed predictor of order v-8; i.e., 0, 1, 2, 3 or 4
                    const order: u3 = @intCast(t & 0b000111);
                    log_subframe.debug("  Subframe with a fixed predictor of order {d}", .{order});
                    if (order > 4) return error.InvalidSubframeHeader;

                    for (0..order) |i| {
                        output[frame_sample_offset + channel_count * i + channel] = try read_unencoded_sample(SampleType, &bit_reader, wasted_bits, unencoded_samples_bit_depth);
                        log_subframe.debug("    warmup_sample: {d}", .{output[frame_sample_offset + channel_count * i + channel]});
                    }

                    if (residuals.len < block_size)
                        residuals = try allocator.realloc(residuals, block_size);

                    try decode_residuals(InterType, residuals, block_size, order, &bit_reader);

                    for (0..block_size - order) |i| {
                        const idx = frame_sample_offset + channel_count * (order + i) + channel;
                        output[idx] = @intCast(switch (order) {
                            0 => residuals[i],
                            1 => residuals[i] + @as(InterType, 1) * output[idx - channel_count * 1],
                            2 => residuals[i] + @as(InterType, 2) * output[idx - channel_count * 1] - @as(InterType, 1) * output[idx - channel_count * 2],
                            3 => residuals[i] + @as(InterType, 3) * output[idx - channel_count * 1] - @as(InterType, 3) * output[idx - channel_count * 2] + @as(InterType, 1) * output[idx - channel_count * 3],
                            4 => residuals[i] + @as(InterType, 4) * output[idx - channel_count * 1] - @as(InterType, 6) * output[idx - channel_count * 2] + @as(InterType, 4) * output[idx - channel_count * 3] - output[idx - channel_count * 4],
                            else => unreachable,
                        });
                    }
                },
                0b100000...0b111111 => |t| { // Subframe with a linear predictor of order v-31; i.e., 1 through 32 (inclusive)
                    const order: u6 = @intCast(t - 31);
                    log_subframe.debug("  Subframe with a linear predictor of order: {d}", .{order});
                    // Unencoded warm-up samples (n = subframe's bits per sample * LPC order).
                    for (0..order) |i| {
                        output[frame_sample_offset + channel_count * i + channel] = try read_unencoded_sample(SampleType, &bit_reader, wasted_bits, unencoded_samples_bit_depth);
                        log_subframe.debug("    warmup_sample: {d}", .{output[frame_sample_offset + channel_count * i + channel]});
                    }
                    // (Predictor coefficient precision in bits)-1 (Note: 0b1111 is forbidden).
                    const coefficient_precision = (try bit_reader.readBitsNoEof(u4, 4)) + 1;
                    log_subframe.debug("    coefficient_precision: {d}", .{coefficient_precision});
                    // Prediction right shift needed in bits.
                    const coefficient_shift_right = try bit_reader.readBitsNoEof(u5, 5);
                    log_subframe.debug("    coefficient_shift_right: {d}", .{coefficient_shift_right});

                    // Predictor coefficients (n = predictor coefficient precision * LPC order).
                    var predictor_coefficient: [32]SampleType = undefined;
                    for (0..order) |i| {
                        predictor_coefficient[i] = try read_unencoded_sample(SampleType, &bit_reader, 0, coefficient_precision);
                        log_subframe.debug("    predictor_coefficient[{d}]: {d}", .{ i, predictor_coefficient[i] });
                    }

                    if (residuals.len < block_size)
                        residuals = try allocator.realloc(residuals, block_size);

                    try decode_residuals(InterType, residuals, block_size, order, &bit_reader);

                    for (0..block_size - order) |i| {
                        const idx = frame_sample_offset + channel_count * (order + i) + channel;
                        var predicted_without_shift: InterType = 0;
                        for (0..order) |o| {
                            predicted_without_shift += @as(InterType, @intCast(output[idx - channel_count * (1 + o)])) * predictor_coefficient[o];
                        }
                        const predicted = predicted_without_shift >> @intCast(coefficient_shift_right);
                        output[idx] = @intCast(predicted + residuals[i]);
                    }
                },
                0b000010...0b000111, 0b001101...0b011111 => return error.InvalidSubframeHeader, // Reserved
            }

            if (wasted_bits > 0) {
                for (0..block_size) |i| {
                    const idx = frame_sample_offset + channel_count * i + channel;
                    output[idx] <<= @intCast(wasted_bits);
                }
            }
        }
        // NOTE: Last subframe is padded with zero bits to be byte aligned.
        bit_reader.alignToByte();

        // Frame CRC
        _ = try reader.readInt(u16, .big);

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
