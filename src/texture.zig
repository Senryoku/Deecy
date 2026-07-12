const std = @import("std");
const log = std.log.scoped(.renderer);

const HollyModule = @import("dreamcast").HollyModule;
const Colors = HollyModule.Colors;
const Color16 = Colors.Color16;
const YUV422 = Colors.YUV422;

// First 1024 values of the Moser de Bruijin sequence, Textures on the dreamcast are limited to 1024*1024 pixels.
const moser_de_bruijin_sequence: [1024]u32 = moser: {
    @setEvalBranchQuota(1024);
    var table: [1024]u32 = undefined;
    table[0] = 0;
    for (1..table.len) |idx| {
        table[idx] = (table[idx - 1] + 0xAAAAAAAB) & 0x55555555;
    }
    break :moser table;
};

// Returns the indice of the z-order curve for the given coordinates.
inline fn zorder_curve(x: u32, y: u32) u32 {
    return (moser_de_bruijin_sequence[x] << 1) | moser_de_bruijin_sequence[y];
}

pub inline fn to_twiddled_index(i: u32, w: u32) u32 {
    return zorder_curve(i % w, i / w);
}

// Extract from the documentation:
//   Twiddled format textures can be either square or rectangular. The relationship between the texture data storage address and the UV coordinates is shown below.
//     <Squares> The bits of the storage address are configured so that each bit of the UV coordinates alternate, starting from the low-order end. The least significant bit is bit 0 of the V coordinate (V0).
//               Example: …… U4 V4 U3 V3 U2 V2 U1 V1 U0 V0
//     <Rectangles> The bits of the storage address are configured so that each bit of the UV coordinates alternate, starting from the low-order end. The least significant bit is bit 0 of the V coordinate (V0).
//                  Any extra bits for one coordinate are positioned in order at the high end.
//                  Example: …… V5 V4 U3 V3 U2 V2 U1 V1 U0 V0
// The following function attempt to implement the general case - rectangles.
inline fn untwiddle(u: u32, v: u32, w: u32, h: u32) u32 {
    // This can probably be made more efficient, but it makes sense to me.
    if (h <= w) {
        // Operate in a single square, assuming that, if w != h, then w > h and w is a multiple of h.
        var r = zorder_curve(@intCast(u % h), @intCast(v));
        // Shift square by square. This corresponds to the extra bits for one coordinates described in the documentation.
        r += (u / h) * h * h;
        return r;
    } else {
        // Same thing, but vertically.
        var r = zorder_curve(@intCast(u), @intCast(v % w));
        r += (v / w) * w * w;
        return r;
    }
}

/// Converts a 2*2 texel block from YUV422 to BGRA
inline fn decode_yuv_block(halfwords: []align(1) const u16) [4][4]u8 {
    const texels_0_1: YUV422 = @bitCast(@as(u32, halfwords[2]) << 16 | @as(u32, halfwords[0]));
    const texels_2_3: YUV422 = @bitCast(@as(u32, halfwords[3]) << 16 | @as(u32, halfwords[1]));
    const colors_0 = Colors.yuv_to_rgba(texels_0_1);
    const colors_1 = Colors.yuv_to_rgba(texels_2_3);
    return .{ colors_0[0].bgra(), colors_0[1].bgra(), colors_1[0].bgra(), colors_1[1].bgra() };
}

pub fn decode_tex(dest_bgra: [*][4]u8, pixel_format: HollyModule.TexturePixelFormat, texture: []const u8, u_size: u32, v_size: u32, twiddled: bool) void {
    std.debug.assert(u_size >= 8 and u_size % 8 == 0);
    std.debug.assert(v_size >= 8 and v_size % 8 == 0);
    switch (pixel_format) {
        inline .ARGB1555, .RGB565, .ARGB4444 => |format| {
            const texels = std.mem.bytesAsSlice(Color16, texture[0..]);
            for (0..v_size) |y| {
                for (0..u_size) |x| {
                    const pixel_index: usize = y * u_size + x;
                    const texel_index: usize = if (twiddled) untwiddle(@intCast(x), @intCast(y), u_size, v_size) else pixel_index;
                    dest_bgra[pixel_index] = texels[texel_index].bgra(@enumFromInt(@intFromEnum(format)), twiddled);
                }
            }
        },
        inline .Palette4BPP, .Palette8BPP => |format| {
            std.debug.assert(twiddled);
            for (0..v_size) |v| {
                for (0..u_size) |u| {
                    const pixel_idx = v * u_size + u;
                    const texel_idx = untwiddle(@intCast(u), @intCast(v), u_size, v_size);
                    const data: u8 = if (format == .Palette4BPP) ((texture[texel_idx >> 1] >> @intCast(4 * (texel_idx & 0x1))) & 0xF) else texture[texel_idx];
                    @as([*]u32, @ptrCast(@alignCast(&dest_bgra[0])))[pixel_idx] = data;
                }
            }
        },
        .YUV422 => {
            if (twiddled) {
                for (0..v_size / 2) |v| {
                    for (0..u_size / 2) |u| {
                        const pixel_idx = 2 * v * u_size + 2 * u;
                        const texel_idx = untwiddle(@intCast(u), @intCast(v), u_size / 2, v_size / 2);
                        const halfwords = std.mem.bytesAsSlice(u16, texture[8 * texel_idx ..])[0..4];
                        const bgra = decode_yuv_block(halfwords);
                        dest_bgra[pixel_idx] = bgra[0];
                        dest_bgra[pixel_idx + 1] = bgra[1];
                        dest_bgra[pixel_idx + u_size] = bgra[2];
                        dest_bgra[pixel_idx + u_size + 1] = bgra[3];
                    }
                }
            } else {
                const texels = std.mem.bytesAsSlice(YUV422, texture);
                for (0..v_size) |v| {
                    for (0..u_size / 2) |u| {
                        const pixel_idx = v * u_size + 2 * u;
                        const colors = Colors.yuv_to_rgba(texels[pixel_idx / 2]);
                        dest_bgra[pixel_idx] = colors[0].bgra();
                        dest_bgra[pixel_idx + 1] = colors[1].bgra();
                    }
                }
            }
        },
        .BumpMap => {
            const texels = std.mem.bytesAsSlice([2]u8, texture[0..]);
            for (0..v_size) |y| {
                for (0..u_size) |x| {
                    const pixel_index: usize = y * u_size + x;
                    const texel_index: usize = if (twiddled) untwiddle(@intCast(x), @intCast(y), u_size, v_size) else pixel_index;
                    dest_bgra[pixel_index] = .{ texels[texel_index][0], texels[texel_index][1], 0, 255 };
                }
            }
        },
        else => std.debug.panic("Unsupported pixel format {t}", .{pixel_format}),
    }
}

pub fn decode_vq(dest_bgra: [*][4]u8, pixel_format: HollyModule.TexturePixelFormat, code_book: []const u8, indices: []const u8, u_size: u32, v_size: u32, twiddled: bool) void {
    std.debug.assert(u_size >= 8 and u_size % 8 == 0);
    std.debug.assert(v_size >= 8 and v_size % 8 == 0);
    std.debug.assert(code_book.len >= 8 * 256);
    std.debug.assert(indices.len >= u_size * v_size / 4);
    switch (pixel_format) {
        .YUV422 => return decode_vq_yuv(dest_bgra, code_book, indices, u_size, v_size, twiddled),
        inline .ARGB1555, .RGB565, .ARGB4444 => |pf| decode_vq_rgb(dest_bgra, pf, code_book, indices, u_size, v_size, twiddled),
        else => return log.err("Unsupported pixel format for VQ textures: {t}", .{pixel_format}),
    }
}

fn decode_vq_rgb(dest_bgra: [*][4]u8, comptime pixel_format: HollyModule.TexturePixelFormat, code_book: []const u8, indices: []const u8, u_size: u32, v_size: u32, twiddled: bool) void {
    const texels = std.mem.bytesAsSlice([4]Color16, code_book);
    if (twiddled) {
        // FIXME: It's not an efficient way to run through the texture, but it's already hard enough to wrap my head around the multiple levels of twiddling.
        for (0..v_size / 2) |v| {
            for (0..u_size / 2) |u| {
                const index = indices[untwiddle(@intCast(u), @intCast(v), u_size / 2, v_size / 2)];
                const block_index = (2 * v * u_size + 2 * u); // Macro 2*2 Block
                for (0..4) |tidx| {
                    const pixel_index = (tidx & 1) * u_size + (tidx >> 1);
                    dest_bgra[block_index + pixel_index] = texels[index][tidx].bgra(pixel_format, true);
                }
            }
        }
    } else {
        // NOTE: This isn't officially supported, but does work in hardware. See https://github.com/pcercuei/bloom/commit/2b214a94e9ed45c2f0a9c9507c8c1af43c4fcc3f for a example use case.
        for (0..v_size) |v| {
            for (0..u_size / 4) |u| {
                const index = indices[v * u_size / 4 + u];
                const block_index = (v * u_size + 4 * u); // Macro 4*1 Block
                for (0..4) |tidx|
                    dest_bgra[block_index + tidx] = texels[index][tidx].bgra(pixel_format, true);
            }
        }
    }
}

fn decode_vq_yuv(dest_bgra: [*][4]u8, code_book: []const u8, indices: []const u8, u_size: u32, v_size: u32, twiddled: bool) void {
    const texels = std.mem.bytesAsSlice(u16, code_book);
    if (twiddled) {
        for (0..v_size / 2) |v| {
            for (0..u_size / 2) |u| {
                const index: u32 = indices[untwiddle(@intCast(u), @intCast(v), u_size / 2, v_size / 2)];
                const block_index = (2 * v * u_size + 2 * u); // Macro 2*2 Block
                const bgra = decode_yuv_block(texels[4 * index ..][0..4]);
                dest_bgra[block_index] = bgra[0];
                dest_bgra[block_index + 1] = bgra[1];
                dest_bgra[block_index + u_size] = bgra[2];
                dest_bgra[block_index + u_size + 1] = bgra[3];
            }
        }
    } else {
        for (0..v_size) |v| {
            for (0..u_size / 4) |u| {
                const index: u32 = indices[v * u_size / 4 + u];
                const block_index = (v * u_size + 4 * u);
                const block = texels[4 * index ..][0..4];
                const bgra = decode_yuv_block(&[_]u16{
                    block[0], block[2], block[1], block[3],
                });
                inline for (0..4) |tidx|
                    dest_bgra[block_index + tidx] = bgra[tidx];
            }
        }
    }
}

pub fn vq_mipmap_offset(u_size: u32) u32 {
    return switch (u_size) {
        8 => 0x6,
        16 => 0x16,
        32 => 0x56,
        64 => 0x156,
        128 => 0x556,
        256 => 0x1556,
        512 => 0x5556,
        1024 => 0x15556,
        else => std.debug.panic("Invalid u_size for vq_compressed mip mapped texture: {d}", .{u_size}),
    };
}
pub fn palette_mipmap_offset(u_size: u32) u32 {
    return switch (u_size) {
        8 => 0x18,
        16 => 0x58,
        32 => 0x158,
        64 => 0x558,
        128 => 0x1558,
        256 => 0x5558,
        512 => 0x15558,
        1024 => 0x55558,
        else => std.debug.panic("Invalid u_size for paletted mip mapped texture: {d}", .{u_size}),
    };
}
pub fn mipmap_offset(u_size: u32) u32 {
    return switch (u_size) {
        8 => 0x30,
        16 => 0xB0,
        32 => 0x2B0,
        64 => 0xAB0,
        128 => 0x2AB0,
        256 => 0xAAB0,
        512 => 0x2AAB0,
        1024 => 0xAAAB0,
        else => std.debug.panic("Invalid u_size for mip mapped texture: {d}", .{u_size}),
    };
}
