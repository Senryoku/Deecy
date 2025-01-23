const std = @import("std");

const renderer_log = std.log.scoped(.renderer);

const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;

const termcolor = @import("termcolor");

const Colors = @import("colors.zig");
const PackedColor = Colors.PackedColor;
const fRGBA = Colors.fRGBA;
const fARGB = Colors.fARGB;
const Color16 = Colors.Color16;
const YUV422 = Colors.YUV422;

const Dreamcast = @import("dreamcast.zig").Dreamcast;
const HollyModule = @import("holly.zig");

const MipMap = @import("mipmap.zig");

pub const ExperimentalFBWriteBack = false;

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
pub inline fn zorder_curve(x: u32, y: u32) u32 {
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
pub inline fn untwiddle(u: u32, v: u32, w: u32, h: u32) u32 {
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

pub fn decode_tex(dest_bgra: [*][4]u8, pixel_format: HollyModule.TexturePixelFormat, texture: []const u8, u_size: u32, v_size: u32, twiddled: bool) void {
    std.debug.assert(u_size >= 8 and u_size % 8 == 0);
    std.debug.assert(v_size >= 8 and v_size % 8 == 0);
    switch (pixel_format) {
        inline .ARGB1555, .RGB565, .ARGB4444 => |format| {
            const texels = std.mem.bytesAsSlice(Colors.Color16, texture[0..]);
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
                    @as([*]u32, @alignCast(@ptrCast(&dest_bgra[0])))[pixel_idx] = data;
                }
            }
        },
        .YUV422 => {
            if (twiddled) {
                for (0..v_size / 2) |v| {
                    for (0..u_size / 2) |u| {
                        const pixel_idx = 2 * v * u_size + 2 * u;
                        const texel_idx = untwiddle(@intCast(u), @intCast(v), u_size / 2, v_size / 2);
                        const halfwords = @as([*]const u16, @alignCast(@ptrCast(&texture[8 * texel_idx])))[0..4];
                        const texels_0_1: YUV422 = @bitCast(@as(u32, halfwords[2]) << 16 | @as(u32, halfwords[0]));
                        const texels_2_3: YUV422 = @bitCast(@as(u32, halfwords[3]) << 16 | @as(u32, halfwords[1]));
                        const colors_0 = Colors.yuv_to_rgba(texels_0_1);
                        dest_bgra[pixel_idx] = .{ colors_0[0].b, colors_0[0].g, colors_0[0].r, colors_0[0].a };
                        dest_bgra[pixel_idx + 1] = .{ colors_0[1].b, colors_0[1].g, colors_0[1].r, colors_0[1].a };
                        const colors_1 = Colors.yuv_to_rgba(texels_2_3);
                        dest_bgra[pixel_idx + u_size] = .{ colors_1[0].b, colors_1[0].g, colors_1[0].r, colors_1[0].a };
                        dest_bgra[pixel_idx + u_size + 1] = .{ colors_1[1].b, colors_1[1].g, colors_1[1].r, colors_1[1].a };
                    }
                }
            } else {
                const texels = std.mem.bytesAsSlice(YUV422, texture);
                for (0..v_size) |v| {
                    for (0..u_size / 2) |u| {
                        const pixel_idx = v * u_size + 2 * u;
                        const colors = Colors.yuv_to_rgba(texels[pixel_idx / 2]);
                        dest_bgra[pixel_idx] = .{ colors[0].b, colors[0].g, colors[0].r, colors[0].a };
                        dest_bgra[pixel_idx + 1] = .{ colors[1].b, colors[1].g, colors[1].r, colors[1].a };
                    }
                }
            }
        },
        .BumpMap => {
            renderer_log.err(termcolor.red("Unsupported pixel format {any}"), .{pixel_format});
            for (0..v_size) |v| {
                for (0..u_size) |u| {
                    const pixel_idx = v * u_size + u;
                    const texel_idx = 2 * pixel_idx;
                    const texels: [2]u8 = @as([*]const u8, @alignCast(@ptrCast(&texture[texel_idx])))[0..2].*;
                    dest_bgra[pixel_idx] = .{ texels[0], texels[1], 0, 0 };
                }
            }
        },
        else => {
            renderer_log.err(termcolor.red("Unsupported pixel format {any}"), .{pixel_format});
            @panic("Unsupported pixel format");
        },
    }
}

pub fn decode_vq(dest_bgra: [*][4]u8, pixel_format: HollyModule.TexturePixelFormat, code_book: []const u8, indices: []const u8, u_size: u32, v_size: u32) void {
    std.debug.assert(u_size >= 8 and u_size % 8 == 0);
    std.debug.assert(v_size >= 8 and v_size % 8 == 0);
    std.debug.assert(code_book.len >= 8 * 256);
    std.debug.assert(indices.len >= u_size * v_size / 4);
    std.debug.assert(pixel_format == .ARGB1555 or pixel_format == .RGB565 or pixel_format == .ARGB4444);
    const texels = std.mem.bytesAsSlice([4]Color16, code_book);
    // FIXME: It's not an efficient way to run through the texture, but it's already hard enough to wrap my head around the multiple levels of twiddling.
    for (0..v_size / 2) |v| {
        for (0..u_size / 2) |u| {
            const index = indices[untwiddle(@intCast(u), @intCast(v), u_size / 2, v_size / 2)];
            for (0..4) |tidx| {
                //                  Macro 2*2 Block            Pixel within the block
                const pixel_index = (2 * v * u_size + 2 * u) + u_size * (tidx & 1) + (tidx >> 1);
                dest_bgra[pixel_index] = texels[index][tidx].bgra(pixel_format, true);
            }
        }
    }
}

const PaletteInstructions = packed struct(u16) {
    palette: bool, // Texture uses 4bpp or 8bpp palette
    filtered: bool, // Should be filtered manually in the shader.
    selector: u6,
    _: u8 = 0,
};

const ShadingInstructions = packed struct(u32) {
    textured: u1 = 0,
    mode: HollyModule.TextureShadingInstruction = .Decal,
    ignore_alpha: u1 = 0,
    tex_u_size: u3,
    tex_v_size: u3,
    src_blend_factor: HollyModule.AlphaInstruction = .One,
    dst_blend_factor: HollyModule.AlphaInstruction = .Zero,
    depth_compare: HollyModule.DepthCompareMode,
    fog_control: HollyModule.FogControl,
    offset_bit: u1,
    shadow_bit: u1,
    gouraud_bit: u1,
    volume_bit: u1,
    mipmap_bit: u1,
    _: u6 = 0,
};

fn sampler_index(mag_filter: wgpu.FilterMode, min_filter: wgpu.FilterMode, mipmap_filter: wgpu.MipmapFilterMode, address_mode_u: wgpu.AddressMode, address_mode_v: wgpu.AddressMode) u8 {
    return @as(u8, @truncate(@intFromEnum(address_mode_v))) << 5 |
        @as(u8, @truncate(@intFromEnum(address_mode_u))) << 3 |
        @as(u8, @truncate(@intFromEnum(mipmap_filter))) << 2 |
        @as(u8, @truncate(@intFromEnum(min_filter))) << 1 |
        @as(u8, @truncate(@intFromEnum(mag_filter)));
}

const TextureIndex = u16;
const InvalidTextureIndex = std.math.maxInt(TextureIndex);

const VertexTextureInfo = packed struct(u64) {
    index: TextureIndex,
    palette: PaletteInstructions,
    shading: ShadingInstructions,

    pub fn invalid() VertexTextureInfo {
        return .{ .index = InvalidTextureIndex, .palette = @bitCast(@as(u16, 0)), .shading = @bitCast(@as(u32, 0)) };
    }
};

const Uniforms = extern struct {
    depth_min: f32,
    depth_max: f32,
    fpu_shad_scale: f32,
    fog_density: f32, // Should be a f16?
    pt_alpha_ref: f32,
    fog_col_pal: fRGBA align(16),
    fog_col_vert: fRGBA align(16),
    fog_lut: [0x80]u32 align(16), // actually 2 * 8bits per entry.
};

const Vertex = packed struct {
    x: f32,
    y: f32,
    z: f32,
    primitive_index: u32,
    base_color: PackedColor,
    offset_color: PackedColor = .{},
    area1_base_color: PackedColor = .{},
    area1_offset_color: PackedColor = .{},
    u: f32 = 0.0,
    v: f32 = 0.0,
    area1_u: f32 = 0.0,
    area1_v: f32 = 0.0,

    // Allow constructing a vertex with the optional fields correctly initialized.
    pub fn undef() Vertex {
        return .{
            .x = -0.0,
            .y = -0.0,
            .z = -0.0,
            .primitive_index = 0xFFFFFFFF,
            .base_color = .{ .r = 255, .g = 0, .b = 0, .a = 255 },
        };
    }
};

const StripMetadata = packed struct(u128) {
    area0_instructions: VertexTextureInfo,
    area1_instructions: VertexTextureInfo = VertexTextureInfo.invalid(),
};

const wgsl_vs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/position_clip.wgsl") ++ @embedFile("./shaders/vs.wgsl");
const wgsl_fs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/fragment_color.wgsl") ++ @embedFile("./shaders/fs.wgsl");
const wgsl_translucent_fs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/oit_structs.wgsl") ++ @embedFile("./shaders/fragment_color.wgsl") ++ @embedFile("./shaders/oit_draw_fs.wgsl");
const wgsl_modvol_translucent_fs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/oit_structs.wgsl") ++ @embedFile("./shaders/modifier_volume_translucent_fs.wgsl");
const wgsl_modvol_merge_cs = @embedFile("./shaders/oit_structs.wgsl") ++ @embedFile("./shaders/modifier_volume_translucent_merge.wgsl");
const wgsl_blend_cs = @embedFile("./shaders/oit_structs.wgsl") ++ @embedFile("./shaders/oit_blend_cs.wgsl");
const wgsl_modifier_volume_vs = @embedFile("./shaders/position_clip.wgsl") ++ @embedFile("./shaders/modifier_volume_vs.wgsl");
const wgsl_modifier_volume_fs = @embedFile("./shaders/modifier_volume_fs.wgsl");
const wgsl_modifier_volume_apply_fs = @embedFile("./shaders/modifier_volume_apply_fs.wgsl");
const blit_vs = @embedFile("./shaders/blit_vs.wgsl");
const blit_fs = @embedFile("./shaders/blit_fs.wgsl");

const TextureStatus = enum {
    Invalid, // Has never been written to.
    Outdated, // Has been modified in VRAM, but a version of it is still in use. Kept around longer for potential re-use. Will reclaim it if necessary.
    Unused, // Wasn't used in the last frame. Will reclaim it if necessary.
    Used, // Was used in the last frame.
};

const TextureMetadata = struct {
    status: TextureStatus = .Invalid,
    age: u32 = 0, // Unused for this many frames.
    control_word: HollyModule.TextureControlWord = .{},
    tsp_instruction: HollyModule.TSPInstructionWord = @bitCast(@as(u32, 0)), // For debugging
    index: TextureIndex = InvalidTextureIndex,
    usage: u32 = 0, // Current usage for this frame
    size: [2]u16 = .{ 0, 0 },
    start_address: u32 = 0,
    end_address: u32 = 0,
    hash: u64 = 0,

    fn masked_tcw(control_word: HollyModule.TextureControlWord) u32 {
        return switch (control_word.pixel_format) {
            .Palette4BPP, .Palette8BPP => control_word.masked() & 0xF81F_FFFF, // Ignore all palette selector bits
            else => control_word.masked(),
        };
    }

    // NOTE: In most cases, the address should be enough, but Soul Calibur mixes multiple different pixel formats in the same texture.
    //       Do handle this, we'll treat then as different textures and upload an additional copy of the texture for each pixel format used.
    //       This is pretty wasteful, but I hope this will be okay.
    //       Ignores the palette selector bits for the cache. The shader can use the same texture data by recombining it with
    //       the palette selector carried by the strip metadata.
    pub fn match(self: @This(), control_word: HollyModule.TextureControlWord) bool {
        return masked_tcw(self.control_word) == masked_tcw(control_word);
    }
};

const DrawCall = struct {
    sampler: u8,
    user_clip: ?HollyModule.UserTileClipInfo,
    start_index: u32 = 0,
    index_count: u32 = 0,

    indices: std.ArrayList(u32), // Temporary storage before uploading them to the GPU.

    pub fn init(allocator: std.mem.Allocator, sampler: u8, user_clip: ?HollyModule.UserTileClipInfo) DrawCall {
        return .{
            .sampler = sampler,
            .user_clip = user_clip,
            .indices = std.ArrayList(u32).init(allocator),
        };
    }

    pub fn deinit(self: *DrawCall) void {
        self.indices.deinit();
    }
};

fn translate_blend_factor(factor: HollyModule.AlphaInstruction) wgpu.BlendFactor {
    return switch (factor) {
        .Zero => .zero,
        .One => .one,
        .SourceAlpha => .src_alpha,
        .InverseSourceAlpha => .one_minus_src_alpha,
        .DestAlpha => .dst_alpha,
        .InverseDestAlpha => .one_minus_dst_alpha,
        else => @panic("Invalid blend factor"),
    };
}

fn translate_src_blend_factor(factor: HollyModule.AlphaInstruction) wgpu.BlendFactor {
    return switch (factor) {
        .OtherColor => .dst,
        .InverseOtherColor => .one_minus_dst,
        else => translate_blend_factor(factor),
    };
}

fn translate_dst_blend_factor(factor: HollyModule.AlphaInstruction) wgpu.BlendFactor {
    return switch (factor) {
        .OtherColor => .src,
        .InverseOtherColor => .one_minus_src,
        else => translate_blend_factor(factor),
    };
}

fn translate_depth_compare_mode(mode: HollyModule.DepthCompareMode) wgpu.CompareFunction {
    return switch (mode) {
        .Never => .never,
        .Less => .less,
        .Equal => .equal,
        .LessEqual => .less_equal,
        .Greater => .greater,
        .NotEqual => .not_equal,
        .GreaterEqual => .greater_equal,
        .Always => .always,
    };
}

const PipelineKey = struct {
    src_blend_factor: wgpu.BlendFactor,
    dst_blend_factor: wgpu.BlendFactor,
    depth_compare: wgpu.CompareFunction,
    depth_write_enabled: bool,

    pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        try writer.print("PipelineKey{{ .src_blend_factor = {s}, .dst_blend_factor = {s}, .depth_compare = {s}, .depth_write_enabled = {} }}", .{
            @tagName(self.src_blend_factor),
            @tagName(self.dst_blend_factor),
            @tagName(self.depth_compare),
            self.depth_write_enabled,
        });
    }
};

const DrawCallKey = struct {
    sampler: u8,
    user_clip: ?HollyModule.UserTileClipInfo,
};

const PipelineMetadata = struct {
    draw_calls: std.AutoArrayHashMap(DrawCallKey, DrawCall),

    pub fn init(allocator: std.mem.Allocator) PipelineMetadata {
        return .{
            .draw_calls = std.AutoArrayHashMap(DrawCallKey, DrawCall).init(allocator),
        };
    }

    fn deinit(self: *@This()) void {
        self.draw_calls.deinit();
    }
};

const PassMetadata = struct {
    pass_type: HollyModule.ListType,
    pipelines: std.AutoArrayHashMap(PipelineKey, PipelineMetadata),

    pub fn init(allocator: std.mem.Allocator, pass_type: HollyModule.ListType) PassMetadata {
        return .{
            .pass_type = pass_type,
            .pipelines = std.AutoArrayHashMap(PipelineKey, PipelineMetadata).init(allocator),
        };
    }

    fn deinit(self: *@This()) void {
        for (self.pipelines.values()) |*pipeline| {
            pipeline.deinit();
        }
        self.pipelines.deinit();
    }

    pub fn reset(self: *@This()) void {
        self.pipelines.clearRetainingCapacity();
    }
};

fn gen_sprite_vertices(sprite: HollyModule.VertexParameter) [4]Vertex {
    var r: [4]Vertex = .{Vertex.undef()} ** 4;

    // B --- C
    // |  \  |
    // A --- D
    // Pushing the vertices in CCW order: A, D, B, C

    switch (sprite) {
        inline .SpriteType0, .SpriteType1 => |v| {
            r[0].x = v.ax;
            r[0].y = v.ay;
            r[0].z = v.az;

            r[1].x = v.dx;
            r[1].y = v.dy;

            r[2].x = v.bx;
            r[2].y = v.by;
            r[2].z = v.bz;

            r[3].x = v.cx;
            r[3].y = v.cy;
            r[3].z = v.cz;
        },
        else => @panic("Not a Sprite"),
    }
    if (sprite == .SpriteType1) {
        const v = sprite.SpriteType1;
        r[0].u = v.auv.u_as_f32();
        r[0].v = v.auv.v_as_f32();
        r[2].u = v.buv.u_as_f32();
        r[2].v = v.buv.v_as_f32();
        r[3].u = v.cuv.u_as_f32();
        r[3].v = v.cuv.v_as_f32();
    }

    // dz have to be deduced from the plane equation
    const ab = [3]f32{
        r[2].x - r[0].x,
        r[2].y - r[0].y,
        r[2].z - r[0].z,
    };
    const ac = [3]f32{
        r[3].x - r[0].x,
        r[3].y - r[0].y,
        r[3].z - r[0].z,
    };
    const normal = [3]f32{
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    };
    const plane_equation_coeff = [4]f32{
        normal[0],
        normal[1],
        normal[2],
        -(normal[0] * r[0].x + normal[1] * r[0].y + normal[2] * r[0].z),
    };
    const dz = (-plane_equation_coeff[0] * r[1].x - plane_equation_coeff[1] * r[1].y - plane_equation_coeff[3]) / plane_equation_coeff[2];
    // Same thing, texture coordinates have to be deduced from other vertices.
    const du = r[0].u + r[3].u - r[2].u;
    const dv = r[0].v + r[3].v - r[2].v;
    r[1].z = dz;
    r[1].u = du;
    r[1].v = dv;

    return r;
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
        else => {
            renderer_log.err(termcolor.red("Invalid u_size for vq_compressed mip mapped texture"), .{});
            @panic("Invalid u_size for vq_compressed mip mapped texture");
        },
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
        else => {
            renderer_log.err(termcolor.red("Invalid u_size for paletted mip mapped texture"), .{});
            @panic("Invalid u_size for paletted mip mapped texture");
        },
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
        else => {
            renderer_log.err(termcolor.red("Invalid u_size for mip mapped texture"), .{});
            @panic("Invalid u_size for mip mapped texture");
        },
    };
}

const VertexAttributes = [_]wgpu.VertexAttribute{
    .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
    .{ .format = .uint32, .offset = @offsetOf(Vertex, "primitive_index"), .shader_location = 1 },
    .{ .format = .uint32, .offset = @offsetOf(Vertex, "base_color"), .shader_location = 2 },
    .{ .format = .uint32, .offset = @offsetOf(Vertex, "offset_color"), .shader_location = 3 },
    .{ .format = .uint32, .offset = @offsetOf(Vertex, "area1_base_color"), .shader_location = 4 },
    .{ .format = .uint32, .offset = @offsetOf(Vertex, "area1_offset_color"), .shader_location = 5 },
    .{ .format = .float32x2, .offset = @offsetOf(Vertex, "u"), .shader_location = 6 },
    .{ .format = .float32x2, .offset = @offsetOf(Vertex, "area1_u"), .shader_location = 7 },
};
const VertexBufferLayout = [_]wgpu.VertexBufferLayout{.{
    .array_stride = @sizeOf(Vertex),
    .attribute_count = VertexAttributes.len,
    .attributes = &VertexAttributes,
}};

const ModifierVolumeVertexAttributes = [_]wgpu.VertexAttribute{
    .{ .format = .float32x4, .offset = 0, .shader_location = 0 },
};
const ModifierVolumeVertexBufferLayout = [_]wgpu.VertexBufferLayout{.{
    .array_stride = 4 * @sizeOf(f32),
    .attribute_count = ModifierVolumeVertexAttributes.len,
    .attributes = &ModifierVolumeVertexAttributes,
}};

const BlitBindGroupLayout = [_]wgpu.BindGroupLayoutEntry{
    zgpu.textureEntry(0, .{ .fragment = true }, .float, .tvdim_2d, false),
    zgpu.samplerEntry(1, .{ .fragment = true }, .filtering),
};

pub const Renderer = struct {
    pub const MaxTextures: [8]u16 = .{ 512, 512, 512, 512, 256, 128, 32, 8 }; // Max texture count for each size. FIXME: Not sure what are good values.

    pub const DisplayMode = enum { Center, Fit, Stretch };

    pub const Resolution = struct { width: u32, height: u32 };
    pub const NativeResolution: Resolution = .{ .width = 640, .height = 480 };

    const OITHorizontalSlices = 4; // Divides the OIT pass into multiple passes to limit memory usage.
    const MaxFragmentsPerPixel = 24;
    const OITLinkedListNodeSize = 5 * 4;
    const VolumeLinkedListNodeSize = 3 * 4;
    const VolumePixelSize = 4 + 4 + 4 * 4 * 2; // See Volumes in oit_structs.zig

    const FirstVertex: u32 = 4; // The 4 first vertices are reserved for the background.
    const FirstIndex: u32 = 5; // The 5 first indices are reserved for the background.

    const DepthClearValue = 0.0;
    const DepthCompareFunction: wgpu.CompareFunction = .greater;

    render_start: bool = false,
    on_render_start_param_base: u32 = 0,
    ta_lists: HollyModule.TALists,

    // That's too much for the higher texture sizes, but that probably doesn't matter.
    texture_metadata: [8][512]TextureMetadata = [_][512]TextureMetadata{[_]TextureMetadata{.{}} ** 512} ** 8,

    framebuffer_resize_bind_group: zgpu.BindGroupHandle,

    blit_pipeline: zgpu.RenderPipelineHandle = .{},
    blit_bind_group: zgpu.BindGroupHandle = undefined,
    blit_vertex_buffer: zgpu.BufferHandle,
    blit_index_buffer: zgpu.BufferHandle,
    blit_to_window_vertex_buffer: zgpu.BufferHandle,

    opaque_pipelines: std.AutoHashMap(PipelineKey, zgpu.RenderPipelineHandle),
    closed_modifier_volume_pipeline: zgpu.RenderPipelineHandle = .{},
    shift_stencil_buffer_modifier_volume_pipeline: zgpu.RenderPipelineHandle = .{},
    open_modifier_volume_pipeline: zgpu.RenderPipelineHandle = .{},
    modifier_volume_apply_pipeline: zgpu.RenderPipelineHandle = .{},
    translucent_pipeline: zgpu.RenderPipelineHandle = .{},
    translucent_modvol_pipeline: zgpu.RenderPipelineHandle = .{},
    translucent_modvol_merge_pipeline: zgpu.ComputePipelineHandle = .{},
    blend_pipeline: zgpu.ComputePipelineHandle = .{},

    translucent_modvol_bind_group_layout: zgpu.BindGroupLayoutHandle,
    translucent_bind_group_layout: zgpu.BindGroupLayoutHandle,
    translucent_modvol_merge_bind_group_layout: zgpu.BindGroupLayoutHandle,
    blend_bind_group_layout: zgpu.BindGroupLayoutHandle,

    opaque_pipeline_layout: zgpu.PipelineLayoutHandle,
    opaque_vertex_shader_module: wgpu.ShaderModule,
    opaque_fragment_shader_module: wgpu.ShaderModule,

    bind_group: zgpu.BindGroupHandle = undefined,
    modifier_volume_bind_group: zgpu.BindGroupHandle = undefined,
    modifier_volume_apply_bind_group: zgpu.BindGroupHandle = undefined,
    translucent_bind_group: zgpu.BindGroupHandle = undefined,
    translucent_modvol_bind_group: zgpu.BindGroupHandle = undefined,
    translucent_modvol_merge_bind_group: zgpu.BindGroupHandle = undefined,
    blend_bind_group: zgpu.BindGroupHandle = undefined,

    vertex_buffer: zgpu.BufferHandle,
    index_buffer: zgpu.BufferHandle,
    modifier_volume_vertex_buffer: zgpu.BufferHandle,

    strips_metadata_buffer: zgpu.BufferHandle,

    list_heads_buffer: zgpu.BufferHandle = undefined,
    linked_list_buffer: zgpu.BufferHandle = undefined,

    modvol_list_heads_buffer: zgpu.BufferHandle = undefined,
    modvol_linked_list_buffer: zgpu.BufferHandle = undefined,
    modvol_volumes_buffer: zgpu.BufferHandle = undefined,

    texture_arrays: [8]zgpu.TextureHandle,
    texture_array_views: [8]zgpu.TextureViewHandle,
    palette_buffer: zgpu.BufferHandle,

    display_mode: DisplayMode = .Center,
    resolution: Resolution = .{ .width = 2 * NativeResolution.width, .height = 2 * NativeResolution.height },

    // Intermediate texture to upload framebuffer from VRAM (and maybe downsample and read back from at some point?)
    framebuffer_texture: zgpu.TextureHandle,
    framebuffer_texture_view: zgpu.TextureViewHandle,
    framebuffer_copy_buffer: if (ExperimentalFBWriteBack) zgpu.BufferHandle else void, // Intermediate buffer used to read pixels back to main RAM.
    // Framebuffer at window resolution to draw on
    resized_framebuffer_texture: zgpu.TextureHandle = undefined,
    resized_framebuffer_texture_view: zgpu.TextureViewHandle = undefined,
    resized_framebuffer_area1_texture: zgpu.TextureHandle = undefined,
    resized_framebuffer_area1_texture_view: zgpu.TextureViewHandle = undefined,

    // NOTE: This should not be needed, but WGPU doesn't handle reading from a storage texture yet.
    resized_framebuffer_copy_texture: zgpu.TextureHandle = undefined,
    resized_framebuffer_copy_texture_view: zgpu.TextureViewHandle = undefined,

    samplers: [256]zgpu.SamplerHandle,
    sampler_bind_groups: [256]zgpu.BindGroupHandle, // FIXME: Use a single one? (Dynamic uniform)

    depth_texture: zgpu.TextureHandle = undefined,
    depth_texture_view: zgpu.TextureViewHandle = undefined,
    depth_only_texture_view: zgpu.TextureViewHandle = undefined,

    opaque_pass: PassMetadata,
    punchthrough_pass: PassMetadata,
    translucent_pass: PassMetadata,

    min_depth: f32 = std.math.floatMax(f32),
    max_depth: f32 = 0.0,
    pt_alpha_ref: f32 = 1.0,
    fpu_shad_scale: f32 = 1.0,
    fog_col_pal: fRGBA = .{},
    fog_col_vert: fRGBA = .{},
    fog_density: f32 = 0,
    fog_lut: [0x80]u32 = [_]u32{0} ** 0x80,
    global_clip: struct { x: struct { min: u16, max: u16 }, y: struct { min: u16, max: u16 } } = .{ .x = .{ .min = 0, .max = 0 }, .y = .{ .min = 0, .max = 0 } },

    vertices: std.ArrayList(Vertex) = undefined, // Just here to avoid repeated allocations.
    strips_metadata: std.ArrayList(StripMetadata) = undefined, // Just here to avoid repeated allocations.
    modifier_volume_vertices: std.ArrayList([4]f32) = undefined,

    _scratch_pad: []u8 align(4), // Used to avoid temporary allocations before GPU uploads for example. 4 * 1024 * 1024, since this is the maximum texture size supported by the DC.

    _gctx: *zgpu.GraphicsContext,
    _allocator: std.mem.Allocator,

    pub fn create(allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) !*Renderer {
        const start = std.time.milliTimestamp();
        defer renderer_log.info("Renderer initialized in {d}ms", .{std.time.milliTimestamp() - start});

        // Writes to texture all rely on that.
        std.debug.assert(zgpu.GraphicsContext.swapchain_format == .bgra8_unorm);

        const framebuffer_texture = gctx.createTexture(.{
            .usage = .{ .render_attachment = true, .texture_binding = true, .copy_dst = true, .copy_src = true },
            .size = .{
                .width = NativeResolution.width,
                .height = NativeResolution.height,
                .depth_or_array_layers = 1,
            },
            .format = zgpu.GraphicsContext.swapchain_format,
            .mip_level_count = 1,
        });
        const framebuffer_texture_view = gctx.createTextureView(framebuffer_texture, .{});

        const framebuffer_copy_buffer = if (ExperimentalFBWriteBack) gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .map_read = true },
            .size = 4 * NativeResolution.width * NativeResolution.height,
        }) else {};

        const bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .vertex = true, .fragment = true }, .uniform, true, 0),
            zgpu.textureEntry(1, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(2, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(3, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(4, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(5, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(6, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(7, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(8, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.bufferEntry(9, .{ .vertex = true }, .read_only_storage, false, 0),
            zgpu.bufferEntry(10, .{ .fragment = true }, .read_only_storage, false, 0),
        });
        defer gctx.releaseResource(bind_group_layout);
        const sampler_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.samplerEntry(0, .{ .fragment = true }, .filtering),
        });
        defer gctx.releaseResource(sampler_bind_group_layout);

        const blit_bind_group_layout = gctx.createBindGroupLayout(&BlitBindGroupLayout);
        defer gctx.releaseResource(blit_bind_group_layout);

        const blit_vs_module = zgpu.createWgslShaderModule(gctx.device, blit_vs, "blit_vs");
        defer blit_vs_module.release();

        const blit_vertex_attributes = [_]wgpu.VertexAttribute{
            .{ .format = .float32x2, .offset = 0, .shader_location = 0 },
            .{ .format = .float32x2, .offset = 2 * @sizeOf(f32), .shader_location = 1 },
        };
        const blit_vertex_buffers = [_]wgpu.VertexBufferLayout{.{
            .array_stride = 4 * @sizeOf(f32),
            .attribute_count = blit_vertex_attributes.len,
            .attributes = &blit_vertex_attributes,
        }};

        const blit_vertex_data = [_]f32{
            // x    y     u    v
            -1.0, 1.0,  0.0, 0.0,
            1.0,  1.0,  1.0, 0.0,
            1.0,  -1.0, 1.0, 1.0,
            -1.0, -1.0, 0.0, 1.0,
        };

        const blit_vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = blit_vertex_data.len * 4 * @sizeOf(f32),
        });
        gctx.queue.writeBuffer(gctx.lookupResource(blit_vertex_buffer).?, 0, f32, blit_vertex_data[0..]);

        const blit_to_window_vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = blit_vertex_data.len * 4 * @sizeOf(f32),
        });

        // Create an index buffer.
        const blit_index_data = [_]u32{ 0, 3, 1, 2 };
        const blit_index_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .index = true },
            .size = blit_index_data.len * @sizeOf(u32),
        });
        gctx.queue.writeBuffer(gctx.lookupResource(blit_index_buffer).?, 0, u32, blit_index_data[0..]);

        var samplers: [256]zgpu.SamplerHandle = undefined;
        var sampler_bind_groups: [256]zgpu.BindGroupHandle = undefined;

        for ([_]wgpu.FilterMode{ .nearest, .linear }) |mag_filter| {
            for ([_]wgpu.FilterMode{ .nearest, .linear }) |min_filter| {
                for ([_]wgpu.MipmapFilterMode{ .nearest, .linear }) |mipmap_filter| {
                    for ([_]wgpu.AddressMode{ .repeat, .mirror_repeat, .clamp_to_edge }) |u_addr_mode| {
                        for ([_]wgpu.AddressMode{ .repeat, .mirror_repeat, .clamp_to_edge }) |v_addr_mode| {
                            const index = sampler_index(mag_filter, min_filter, mipmap_filter, u_addr_mode, v_addr_mode);
                            samplers[index] = gctx.createSampler(.{
                                .mag_filter = mag_filter,
                                .min_filter = min_filter,
                                .mipmap_filter = mipmap_filter,
                                .address_mode_u = u_addr_mode,
                                .address_mode_v = v_addr_mode,
                            });
                            sampler_bind_groups[index] = gctx.createBindGroup(sampler_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
                                .{ .binding = 0, .sampler_handle = samplers[index] },
                            });
                        }
                    }
                }
            }
        }

        const framebuffer_resize_bind_group = gctx.createBindGroup(blit_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = framebuffer_texture_view },
            .{ .binding = 1, .sampler_handle = samplers[sampler_index(.linear, .linear, .linear, .clamp_to_edge, .clamp_to_edge)] },
        });

        var texture_arrays: [8]zgpu.TextureHandle = undefined;
        var texture_array_views: [8]zgpu.TextureViewHandle = undefined;
        for (0..8) |i| {
            texture_arrays[i] = gctx.createTexture(.{
                .usage = .{ .texture_binding = true, .storage_binding = true, .copy_dst = true },
                .size = .{
                    .width = @as(u32, 8) << @intCast(i),
                    .height = @as(u32, 8) << @intCast(i),
                    .depth_or_array_layers = Renderer.MaxTextures[i],
                },
                .format = .bgra8_unorm,
                .mip_level_count = @intCast(4 + i),
            });
            texture_array_views[i] = gctx.createTextureView(texture_arrays[i], .{});
        }
        const palette_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .storage = true },
            .size = 4 * 1024,
        });

        const StripMetadataSize = 4 * 4096 * @sizeOf(StripMetadata); // FIXME: Arbitrary size for testing
        const strips_metadata_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .storage = true },
            .size = StripMetadataSize,
        });

        const bind_group = gctx.createBindGroup(bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(Uniforms) },
            .{ .binding = 1, .texture_view_handle = texture_array_views[0] },
            .{ .binding = 2, .texture_view_handle = texture_array_views[1] },
            .{ .binding = 3, .texture_view_handle = texture_array_views[2] },
            .{ .binding = 4, .texture_view_handle = texture_array_views[3] },
            .{ .binding = 5, .texture_view_handle = texture_array_views[4] },
            .{ .binding = 6, .texture_view_handle = texture_array_views[5] },
            .{ .binding = 7, .texture_view_handle = texture_array_views[6] },
            .{ .binding = 8, .texture_view_handle = texture_array_views[7] },
            .{ .binding = 9, .buffer_handle = strips_metadata_buffer, .offset = 0, .size = StripMetadataSize },
            .{ .binding = 10, .buffer_handle = palette_buffer, .offset = 0, .size = 4 * 1024 },
        });

        const vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = 64 * 4096 * @sizeOf(Vertex), // FIXME: Arbitrary size for testing
        });

        const index_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .index = true },
            .size = 64 * 16384 * @sizeOf(u32), // FIXME: Arbitrary size for testing
        });

        const modifier_volume_vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = 64 * 4096 * @sizeOf([4]f32), // FIXME: Arbitrary size for testing
        });

        const opaque_vertex_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_vs, "vs");

        // Translucent pipeline

        const translucent_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_translucent_fs, "fs");
        defer translucent_fragment_shader_module.release();

        const translucent_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .fragment = true }, .uniform, true, 0),
            zgpu.bufferEntry(1, .{ .fragment = true }, .storage, false, 0),
            zgpu.bufferEntry(2, .{ .fragment = true }, .storage, false, 0),
            zgpu.textureEntry(3, .{ .fragment = true }, .depth, .tvdim_2d, false),
        });

        const translucent_modvol_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .fragment = true }, .uniform, true, 0),
            zgpu.bufferEntry(1, .{ .fragment = true }, .storage, false, 0),
            zgpu.bufferEntry(2, .{ .fragment = true }, .storage, false, 0),
            zgpu.bufferEntry(3, .{ .fragment = true }, .uniform, true, 0),
        });

        const modifier_volume_vertex_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modifier_volume_vs, "modvol_vs");
        defer modifier_volume_vertex_shader_module.release();
        const modifier_volume_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .vertex = true }, .uniform, true, 0),
        });
        defer gctx.releaseResource(modifier_volume_group_layout);

        const translucent_modvol_merge_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .compute = true }, .uniform, true, 0),
            zgpu.bufferEntry(1, .{ .compute = true }, .storage, false, 0),
            zgpu.bufferEntry(2, .{ .compute = true }, .storage, false, 0),
            zgpu.bufferEntry(3, .{ .compute = true }, .storage, false, 0),
        });

        const color_targets = [_]wgpu.ColorTargetState{.{
            .format = zgpu.GraphicsContext.swapchain_format,
            .write_mask = .{}, // We won't write to the color attachment
        }};

        const translucent_pipeline_layout = gctx.createPipelineLayout(&.{
            bind_group_layout,
            sampler_bind_group_layout,
            translucent_bind_group_layout,
        });
        defer gctx.releaseResource(translucent_pipeline_layout);
        const translucent_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = wgpu.VertexState{
                .module = opaque_vertex_shader_module,
                .entry_point = "main",
                .buffer_count = VertexBufferLayout.len,
                .buffers = &VertexBufferLayout,
            },
            .primitive = wgpu.PrimitiveState{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_strip,
                .strip_index_format = .uint32,
            },
            .depth_stencil = null, // FIXME: Use opaque depth here rather than sampling it manually in the shader?
            .fragment = &wgpu.FragmentState{
                .module = translucent_fragment_shader_module,
                .entry_point = "main",
                .target_count = color_targets.len,
                .targets = &color_targets,
            },
        };

        // Translucent fragment blending pipeline

        const blend_compute_module = zgpu.createWgslShaderModule(gctx.device, wgsl_blend_cs, null);
        defer blend_compute_module.release();

        const blend_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .compute = true }, .uniform, true, 0),
            zgpu.bufferEntry(1, .{ .compute = true }, .storage, false, 0),
            zgpu.bufferEntry(2, .{ .compute = true }, .storage, false, 0),
            zgpu.textureEntry(3, .{ .compute = true }, .float, .tvdim_2d, false),
            zgpu.storageTextureEntry(4, .{ .compute = true }, .write_only, .bgra8_unorm, .tvdim_2d),
            zgpu.bufferEntry(5, .{ .compute = true }, .storage, false, 0),
        });

        const blend_pipeline_layout = gctx.createPipelineLayout(&.{blend_bind_group_layout});
        defer gctx.releaseResource(blend_pipeline_layout);
        const blend_pipeline_descriptor = wgpu.ComputePipelineDescriptor{
            .compute = .{
                .module = blend_compute_module,
                .entry_point = "main",
            },
        };

        // Modifier Volumes
        // Implemented using a stencil buffer and the shadow volume algorithm.
        // This first pipeline takes the previous depth buffer and the modifier volume to generate the stencil buffer.

        const modifier_volume_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modifier_volume_fs, "fs");
        defer modifier_volume_fragment_shader_module.release();

        const modifier_volume_pipeline_layout = gctx.createPipelineLayout(&.{modifier_volume_group_layout});

        const closed_modifier_volume_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = wgpu.VertexState{
                .module = modifier_volume_vertex_shader_module,
                .entry_point = "main",
                .buffer_count = ModifierVolumeVertexBufferLayout.len,
                .buffers = &ModifierVolumeVertexBufferLayout,
            },
            .primitive = wgpu.PrimitiveState{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_list,
            },
            .depth_stencil = &wgpu.DepthStencilState{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = false,
                .depth_compare = DepthCompareFunction,
                .stencil_read_mask = 0x01,
                .stencil_write_mask = 0x01,
                .stencil_front = .{
                    .compare = .always,
                    .fail_op = .keep,
                    .pass_op = .increment_wrap,
                    .depth_fail_op = .keep,
                },
                .stencil_back = .{
                    .compare = .always,
                    .fail_op = .keep,
                    .pass_op = .increment_wrap,
                    .depth_fail_op = .keep,
                },
            },
            .fragment = &wgpu.FragmentState{
                .module = modifier_volume_fragment_shader_module,
                .entry_point = "main",
                .target_count = 0,
                .targets = null,
            },
        };

        const shift_stencil_buffer_modifier_volume_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = wgpu.VertexState{
                .module = modifier_volume_vertex_shader_module,
                .entry_point = "main",
                .buffer_count = ModifierVolumeVertexBufferLayout.len,
                .buffers = &ModifierVolumeVertexBufferLayout,
            },
            .primitive = wgpu.PrimitiveState{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_list,
            },
            .depth_stencil = &wgpu.DepthStencilState{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = false,
                .depth_compare = .always,
                .stencil_read_mask = 0x03,
                .stencil_write_mask = 0x03,
                .stencil_front = .{
                    .compare = .equal,
                    .fail_op = .keep,
                    .pass_op = .increment_wrap, // 0x01 -> 0x02
                    .depth_fail_op = .keep,
                },
                .stencil_back = .{
                    .compare = .equal,
                    .fail_op = .keep,
                    .pass_op = .increment_wrap,
                    .depth_fail_op = .keep,
                },
            },
            .fragment = &wgpu.FragmentState{
                .module = modifier_volume_fragment_shader_module,
                .entry_point = "main",
                .target_count = 0,
                .targets = null,
            },
        };

        const open_modifier_volume_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = wgpu.VertexState{
                .module = modifier_volume_vertex_shader_module,
                .entry_point = "main",
                .buffer_count = ModifierVolumeVertexBufferLayout.len,
                .buffers = &ModifierVolumeVertexBufferLayout,
            },
            .primitive = wgpu.PrimitiveState{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_list,
            },
            .depth_stencil = &wgpu.DepthStencilState{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = false,
                .depth_compare = DepthCompareFunction,
                .stencil_read_mask = 0x02,
                .stencil_write_mask = 0x03,
                .stencil_front = .{
                    .compare = .not_equal, // Only run if not already marked as area 1.
                    .fail_op = .keep, // Action performed on samples that fail the stencil test
                    .pass_op = .replace, // Action performed on samples that pass both the depth and stencil tests.
                    .depth_fail_op = .keep, // Action performed on samples that pass the stencil test and fail the depth test.
                },
                .stencil_back = .{ // Same as front, I don't think the orientation matters for the hardward and games are not required to properly distinguish between front facing and back facing triangles.
                    .compare = .not_equal,
                    .fail_op = .keep,
                    .pass_op = .replace,
                    .depth_fail_op = .keep,
                },
            },
            .fragment = &wgpu.FragmentState{
                .module = modifier_volume_fragment_shader_module,
                .entry_point = "main",
                .target_count = 0,
                .targets = null,
            },
        };

        const modifier_volume_bind_group = gctx.createBindGroup(modifier_volume_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = 2 * @sizeOf(f32) },
        });

        var renderer = try allocator.create(Renderer);
        renderer.* = .{
            .blit_vertex_buffer = blit_vertex_buffer,
            .blit_index_buffer = blit_index_buffer,
            .blit_to_window_vertex_buffer = blit_to_window_vertex_buffer,

            .framebuffer_resize_bind_group = framebuffer_resize_bind_group,

            .framebuffer_texture = framebuffer_texture,
            .framebuffer_texture_view = framebuffer_texture_view,
            .framebuffer_copy_buffer = framebuffer_copy_buffer,

            .opaque_pipelines = std.AutoHashMap(PipelineKey, zgpu.RenderPipelineHandle).init(allocator),

            .translucent_bind_group_layout = translucent_bind_group_layout,
            .translucent_modvol_bind_group_layout = translucent_modvol_bind_group_layout,
            .translucent_modvol_merge_bind_group_layout = translucent_modvol_merge_bind_group_layout,

            .blend_bind_group_layout = blend_bind_group_layout,

            .opaque_pipeline_layout = gctx.createPipelineLayout(&.{ bind_group_layout, sampler_bind_group_layout }),
            .opaque_vertex_shader_module = opaque_vertex_shader_module,
            .opaque_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "fs"),

            .bind_group = bind_group,
            .modifier_volume_bind_group = modifier_volume_bind_group,
            .vertex_buffer = vertex_buffer,
            .index_buffer = index_buffer,
            .strips_metadata_buffer = strips_metadata_buffer,
            .modifier_volume_vertex_buffer = modifier_volume_vertex_buffer,

            .texture_arrays = texture_arrays,
            .texture_array_views = texture_array_views,
            .palette_buffer = palette_buffer,

            .samplers = samplers,
            .sampler_bind_groups = sampler_bind_groups,

            .vertices = try std.ArrayList(Vertex).initCapacity(allocator, 4096),
            .strips_metadata = try std.ArrayList(StripMetadata).initCapacity(allocator, 4096),
            .modifier_volume_vertices = try std.ArrayList([4]f32).initCapacity(allocator, 4096),

            .opaque_pass = PassMetadata.init(allocator, .Opaque),
            .punchthrough_pass = PassMetadata.init(allocator, .PunchThrough),
            .translucent_pass = PassMetadata.init(allocator, .Translucent),

            .ta_lists = HollyModule.TALists.init(allocator),

            ._scratch_pad = try allocator.allocWithOptions(u8, 4 * 1024 * 1024, 4, null),

            ._gctx = gctx,
            ._allocator = allocator,
        };

        MipMap.init(allocator, gctx);
        // Blit pipeline
        {
            const blit_fs_module = zgpu.createWgslShaderModule(gctx.device, blit_fs, "blit_fs");
            defer blit_fs_module.release();
            const blit_pipeline_layout = gctx.createPipelineLayout(&.{blit_bind_group_layout});
            defer gctx.releaseResource(blit_pipeline_layout);

            const blit_color_targets = [_]wgpu.ColorTargetState{.{
                .format = zgpu.GraphicsContext.swapchain_format,
            }};

            const pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = wgpu.VertexState{
                    .module = blit_vs_module,
                    .entry_point = "main",
                    .buffer_count = blit_vertex_buffers.len,
                    .buffers = &blit_vertex_buffers,
                },
                .primitive = wgpu.PrimitiveState{
                    .front_face = .ccw,
                    .cull_mode = .none,
                    .topology = .triangle_strip,
                    .strip_index_format = .uint32,
                },
                .depth_stencil = null,
                .fragment = &wgpu.FragmentState{
                    .module = blit_fs_module,
                    .entry_point = "main",
                    .target_count = blit_color_targets.len,
                    .targets = &blit_color_targets,
                },
            };
            gctx.createRenderPipelineAsync(allocator, blit_pipeline_layout, pipeline_descriptor, &renderer.blit_pipeline);
        }

        gctx.createComputePipelineAsync(allocator, blend_pipeline_layout, blend_pipeline_descriptor, &renderer.blend_pipeline);
        gctx.createRenderPipelineAsync(allocator, translucent_pipeline_layout, translucent_pipeline_descriptor, &renderer.translucent_pipeline);

        gctx.createRenderPipelineAsync(allocator, modifier_volume_pipeline_layout, closed_modifier_volume_pipeline_descriptor, &renderer.closed_modifier_volume_pipeline);
        gctx.createRenderPipelineAsync(allocator, modifier_volume_pipeline_layout, shift_stencil_buffer_modifier_volume_pipeline_descriptor, &renderer.shift_stencil_buffer_modifier_volume_pipeline);
        gctx.createRenderPipelineAsync(allocator, modifier_volume_pipeline_layout, open_modifier_volume_pipeline_descriptor, &renderer.open_modifier_volume_pipeline);

        // Modifier Volume Apply pipeline - Use the stencil from the previous pass to apply modifier volume effects.
        {
            const mv_apply_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modifier_volume_apply_fs, "fs");
            defer mv_apply_fragment_shader_module.release();

            const mv_apply_bind_group_layout = gctx.createBindGroupLayout(&.{
                zgpu.textureEntry(0, .{ .fragment = true }, .float, .tvdim_2d, false),
            });
            const mv_apply_pipeline_layout = gctx.createPipelineLayout(&.{
                mv_apply_bind_group_layout,
            });

            const mv_apply_color_targets = [_]wgpu.ColorTargetState{.{
                .format = zgpu.GraphicsContext.swapchain_format,
            }};

            const mv_apply_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = wgpu.VertexState{
                    .module = blit_vs_module,
                    .entry_point = "main",
                    .buffer_count = blit_vertex_buffers.len,
                    .buffers = &blit_vertex_buffers,
                },
                .primitive = wgpu.PrimitiveState{
                    .front_face = .ccw,
                    .cull_mode = .none,
                    .topology = .triangle_strip,
                    .strip_index_format = .uint32,
                },
                .depth_stencil = &wgpu.DepthStencilState{
                    .format = .depth32_float_stencil8,
                    .depth_write_enabled = false,
                    .depth_compare = .always,
                    .stencil_read_mask = 0x2,
                    .stencil_front = .{
                        .compare = .equal,
                        .fail_op = .keep,
                        .depth_fail_op = .keep,
                        .pass_op = .keep,
                    },
                    .stencil_back = .{
                        .compare = .equal,
                        .fail_op = .keep,
                        .depth_fail_op = .keep,
                        .pass_op = .keep,
                    },
                },
                .fragment = &wgpu.FragmentState{
                    .module = mv_apply_fragment_shader_module,
                    .entry_point = "main",
                    .target_count = mv_apply_color_targets.len,
                    .targets = &mv_apply_color_targets,
                },
            };
            gctx.createRenderPipelineAsync(allocator, mv_apply_pipeline_layout, mv_apply_pipeline_descriptor, &renderer.modifier_volume_apply_pipeline);
        }
        // Translucent modifier volume pipeline
        {
            const translucent_modvol_fs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modvol_translucent_fs, "translucent_modvol_fs");
            defer translucent_modvol_fs_module.release();

            const translucent_modvol_pipeline_layout = gctx.createPipelineLayout(&.{
                modifier_volume_group_layout,
                translucent_modvol_bind_group_layout,
            });

            const translucent_modvol_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = wgpu.VertexState{
                    .module = modifier_volume_vertex_shader_module,
                    .entry_point = "main",
                    .buffer_count = ModifierVolumeVertexBufferLayout.len,
                    .buffers = &ModifierVolumeVertexBufferLayout,
                },
                .primitive = wgpu.PrimitiveState{
                    .front_face = .ccw,
                    .cull_mode = .none,
                    .topology = .triangle_list,
                },
                .depth_stencil = &wgpu.DepthStencilState{
                    .format = .depth32_float_stencil8,
                    .depth_write_enabled = false,
                    .depth_compare = DepthCompareFunction,
                    .stencil_read_mask = 0x00,
                    .stencil_write_mask = 0x00,
                    .stencil_front = .{
                        .compare = .always,
                        .fail_op = .keep,
                        .pass_op = .keep,
                        .depth_fail_op = .keep,
                    },
                    .stencil_back = .{
                        .compare = .always,
                        .fail_op = .keep,
                        .pass_op = .keep,
                        .depth_fail_op = .keep,
                    },
                },
                .fragment = &wgpu.FragmentState{
                    .module = translucent_modvol_fs_module,
                    .entry_point = "main",
                    .target_count = 0,
                    .targets = null,
                },
            };
            gctx.createRenderPipelineAsync(allocator, translucent_modvol_pipeline_layout, translucent_modvol_pipeline_descriptor, &renderer.translucent_modvol_pipeline);
        }
        // Translucent modifier volume merge pipeline
        {
            const translucent_modvol_merge_pipeline_layout = gctx.createPipelineLayout(&.{translucent_modvol_merge_bind_group_layout});
            const translucent_modvol_merge_compute_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modvol_merge_cs, null);
            defer translucent_modvol_merge_compute_module.release();
            const translucent_modvol_merge_pipeline_descriptor = wgpu.ComputePipelineDescriptor{
                .compute = .{
                    .module = translucent_modvol_merge_compute_module,
                    .entry_point = "main",
                },
            };
            gctx.createComputePipelineAsync(allocator, translucent_modvol_merge_pipeline_layout, translucent_modvol_merge_pipeline_descriptor, &renderer.translucent_modvol_merge_pipeline);
        }

        // Ensure capacity for opaque pipelines: Async creation needs pointer stability.
        try renderer.opaque_pipelines.ensureTotalCapacity(std.meta.fields(wgpu.BlendFactor).len * std.meta.fields(wgpu.BlendFactor).len * std.meta.fields(wgpu.CompareFunction).len * 2);

        // Asyncronously create some common opaque pipelines ahead of time
        _ = try renderer.get_or_put_opaque_pipeline(.{ .src_blend_factor = .one, .dst_blend_factor = .zero, .depth_compare = .always, .depth_write_enabled = false }, .Async); // Background
        _ = try renderer.get_or_put_opaque_pipeline(.{ .src_blend_factor = .one, .dst_blend_factor = .zero, .depth_compare = .greater_equal, .depth_write_enabled = true }, .Async);
        _ = try renderer.get_or_put_opaque_pipeline(.{ .src_blend_factor = .src_alpha, .dst_blend_factor = .one_minus_src_alpha, .depth_compare = .greater_equal, .depth_write_enabled = true }, .Async);

        renderer.on_inner_resolution_change();

        return renderer;
    }

    pub fn destroy(self: *Renderer) void {
        // Wait for async pipeline creation to finish (prevents crashing on exit).
        while (self._gctx.lookupResource(self.closed_modifier_volume_pipeline) == null or
            self._gctx.lookupResource(self.shift_stencil_buffer_modifier_volume_pipeline) == null or
            self._gctx.lookupResource(self.open_modifier_volume_pipeline) == null or
            self._gctx.lookupResource(self.modifier_volume_apply_pipeline) == null or
            self._gctx.lookupResource(self.translucent_pipeline) == null or
            self._gctx.lookupResource(self.translucent_modvol_pipeline) == null or
            self._gctx.lookupResource(self.translucent_modvol_merge_pipeline) == null or
            self._gctx.lookupResource(self.blend_pipeline) == null)
        {
            self._gctx.device.tick();
        }
        var async_pipeline_creation = true;
        while (async_pipeline_creation) {
            async_pipeline_creation = false;
            self._gctx.device.tick();
            var it = self.opaque_pipelines.iterator();
            while (it.next()) |pipeline| {
                if (self._gctx.lookupResource(pipeline.value_ptr.*) == null) {
                    async_pipeline_creation = true;
                }
            }
        }

        self.deinit_screen_textures();

        self._allocator.free(self._scratch_pad);

        self.translucent_pass.deinit();
        self.punchthrough_pass.deinit();
        self.opaque_pass.deinit();

        self.modifier_volume_vertices.deinit();
        self.strips_metadata.deinit();
        self.vertices.deinit();

        for (self.sampler_bind_groups) |sampler_bind_group| {
            self._gctx.releaseResource(sampler_bind_group);
        }
        for (self.samplers) |sampler| {
            self._gctx.releaseResource(sampler);
        }

        for (self.texture_array_views) |view| {
            self._gctx.releaseResource(view);
        }
        for (self.texture_arrays) |array| {
            self._gctx.releaseResource(array);
        }

        self._gctx.releaseResource(self.modifier_volume_vertex_buffer);
        self._gctx.releaseResource(self.strips_metadata_buffer);
        self._gctx.releaseResource(self.index_buffer);
        self._gctx.releaseResource(self.vertex_buffer);

        self.opaque_vertex_shader_module.release();
        self.opaque_fragment_shader_module.release();
        self._gctx.releaseResource(self.opaque_pipeline_layout);

        self._gctx.releaseResource(self.blend_bind_group_layout);
        self._gctx.releaseResource(self.blend_pipeline);

        self._gctx.releaseResource(self.translucent_modvol_merge_bind_group_layout);
        self._gctx.releaseResource(self.translucent_modvol_merge_pipeline);
        self._gctx.releaseResource(self.translucent_modvol_bind_group_layout);
        self._gctx.releaseResource(self.translucent_modvol_pipeline);
        self._gctx.releaseResource(self.translucent_bind_group_layout);
        self._gctx.releaseResource(self.translucent_pipeline);

        self._gctx.releaseResource(self.modifier_volume_apply_pipeline);
        self._gctx.releaseResource(self.open_modifier_volume_pipeline);
        self._gctx.releaseResource(self.shift_stencil_buffer_modifier_volume_pipeline);
        self._gctx.releaseResource(self.closed_modifier_volume_pipeline);

        var opaque_pipelines = self.opaque_pipelines.iterator();
        while (opaque_pipelines.next()) |opaque_pipeline| {
            self._gctx.releaseResource(opaque_pipeline.value_ptr.*);
        }
        self.opaque_pipelines.deinit();

        self._gctx.releaseResource(self.framebuffer_texture_view);
        self._gctx.releaseResource(self.framebuffer_texture);
        if (ExperimentalFBWriteBack)
            self._gctx.releaseResource(self.framebuffer_copy_buffer);

        self._gctx.releaseResource(self.framebuffer_resize_bind_group);

        self._gctx.releaseResource(self.blit_to_window_vertex_buffer);
        self._gctx.releaseResource(self.blit_index_buffer);
        self._gctx.releaseResource(self.blit_vertex_buffer);
        // self._gctx.releaseResource(self.blit_pipeline);

        self._allocator.destroy(self);
    }

    pub fn reset(self: *@This()) void {
        self.render_start = false;
        self.texture_metadata = [_][512]TextureMetadata{[_]TextureMetadata{.{}} ** 512} ** 8;

        self.ta_lists.clearRetainingCapacity();

        self.translucent_pass.reset();
        self.punchthrough_pass.reset();
        self.opaque_pass.reset();

        self.modifier_volume_vertices.clearRetainingCapacity();
        self.strips_metadata.clearRetainingCapacity();
        self.vertices.clearRetainingCapacity();
    }

    pub fn on_render_start(self: *@This(), dc: *Dreamcast) void {
        if (self.render_start) {
            renderer_log.warn(termcolor.yellow("Woops! Skipped a frame."), .{});
        }

        self.on_render_start_param_base = dc.gpu.read_register(u32, .PARAM_BASE);

        // Clear the previous used TA lists and swap it with the one submitted by the game.
        // NOTE: Clearing the lists here means the game cannot render lists more than once (e.i. starting a render without
        //       writing to LIST_INIT). No idea if there are games that actually do that, but just in case, emit a warning.
        self.ta_lists.clearRetainingCapacity();
        const list_idx: u4 = @truncate(self.on_render_start_param_base >> 20);
        std.mem.swap(HollyModule.TALists, &dc.gpu._ta_lists[list_idx], &self.ta_lists);
        if (self.ta_lists.opaque_list.vertex_strips.items.len == 0 and self.ta_lists.punchthrough_list.vertex_strips.items.len == 0 and self.ta_lists.translucent_list.vertex_strips.items.len == 0) {
            renderer_log.warn(termcolor.yellow("on_render_start: Empty TA lists submitted. Is the game trying to reuse the previous TA lists?"), .{});
        }

        self.render_start = true;
    }

    // FIXME: This is way too slow.
    fn texture_hash(gpu: *const HollyModule.Holly, start: u32, end: u32) u64 {
        return std.hash.CityHash64.hash(gpu.vram[start & 0xFFFFFFC .. end & 0xFFFFFFC]);
    }

    // For external use only (e.g. Debug UI)
    pub fn get_texture_view(self: *const Renderer, control_word: HollyModule.TextureControlWord, tsp_instruction: HollyModule.TSPInstructionWord) ?struct { size_index: u3, index: u32 } {
        const size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);
        for (self.texture_metadata[size_index][0..Renderer.MaxTextures[size_index]], 0..) |*entry, idx| {
            if (entry.status != .Invalid and entry.status != .Outdated and entry.match(control_word)) {
                return .{ .size_index = size_index, .index = @intCast(idx) };
            }
        }
        return null;
    }

    fn get_texture_index(self: *Renderer, gpu: *const HollyModule.Holly, size_index: u3, control_word: HollyModule.TextureControlWord) ?TextureIndex {
        for (self.texture_metadata[size_index][0..Renderer.MaxTextures[size_index]], 0..) |*entry, idx| {
            if (entry.status != .Invalid and entry.match(control_word)) {
                if (entry.usage == 0) {
                    // Texture appears to have changed in memory. Mark as outdated.
                    if (texture_hash(gpu, entry.start_address, entry.end_address) != entry.hash) {
                        entry.status = .Outdated;
                        entry.usage = 0xFFFFFFFF; // Do not check it again.
                        continue; // Not valid anymore, ignore it.
                    } else if (entry.status == .Outdated) { // It became valid again!
                        entry.status = .Used;
                    }
                } else if (entry.status == .Outdated) continue;
                return @intCast(idx);
            }
        }
        return null;
    }

    inline fn bgra_scratch_pad(self: *Renderer) [*][4]u8 {
        return @as([*][4]u8, @ptrCast(self._scratch_pad.ptr));
    }

    fn upload_texture(self: *Renderer, gpu: *const HollyModule.Holly, tsp_instruction: HollyModule.TSPInstructionWord, texture_control_word: HollyModule.TextureControlWord) TextureIndex {
        renderer_log.debug("[Upload] tsp_instruction: {any}", .{tsp_instruction});
        renderer_log.debug("[Upload] texture_control_word: {any}", .{texture_control_word});

        const texture_control_register = gpu.read_register(HollyModule.TEXT_CONTROL, .TEXT_CONTROL);

        const is_paletted = texture_control_word.pixel_format == .Palette4BPP or texture_control_word.pixel_format == .Palette8BPP;

        const scan_order = if (is_paletted) 0 else texture_control_word.scan_order;
        const stride_select = if (is_paletted) 0 else texture_control_word.stride_select;

        const twiddled = scan_order == 0;
        const size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);

        // NOTE: This is used by stride textures. Stride textures actual size can be smaller than their allocated size, but UV calculation are still done with it.
        const alloc_u_size = (@as(u16, 8) << tsp_instruction.texture_u_size);
        const alloc_v_size = (@as(u16, 8) << tsp_instruction.texture_v_size);

        var u_size: u32 = if (scan_order == 1 and stride_select == 1) @as(u16, 32) * texture_control_register.stride else alloc_u_size;
        const v_size: u32 = if (scan_order == 0 and texture_control_word.mip_mapped == 1) u_size else alloc_v_size;

        if (u_size > alloc_u_size) {
            // FIXME: This can happen with stride textures. The documentation seems to contradict itself:
            //        - Texture Control Word, Stride Select: "When this bit is 1, the U size of the texture is specified by the TEXT_CONTROL register (i.e., the U Size bit is ignored).""
            //        - TSP Instruction Word, U Size:        "The value that is specified here must be greater than the U size of the stride texture.""
            //   I don't know if the U Size "must be greater" or if it is "ignored". Virtua Tennis 2 runs into this issue.
            //   Ignoring the U Size is more problematic for us as we use it to select the correct texture array, so I'll clamp the stride size for now.
            renderer_log.err(termcolor.red("Texture U size ({d}) is greater than the allocated size ({d})\n") ++ termcolor.grey("  TEXT_CONTROL:    {any}\n  TSP Instruction: {any}\n  Texture Control: {any})"), .{ u_size, alloc_u_size, texture_control_register, tsp_instruction, texture_control_word });
            u_size = alloc_u_size;
            // Virtua Tennis 2 uses a stride value of 640 (texture_control_register.stride = 20) when the issue occurs. I suspect it tries to sample the framebuffer.
        }

        var addr: u32 = 8 * @as(u32, texture_control_word.address); // given in units of 64-bits.
        var vq_index_addr = addr + 8 * 256;

        if (texture_control_word.mip_mapped == 1) {
            // We only want the highest mip level and we'll compute the others ourself.
            // See DreamcastDevBoxSystemArchitecture.pdf p.148
            if (texture_control_word.vq_compressed == 1) {
                vq_index_addr += vq_mipmap_offset(u_size);
            } else if (is_paletted) {
                const val: u32 = palette_mipmap_offset(u_size);
                addr += if (texture_control_word.pixel_format == .Palette4BPP) val / 2 else val;
            } else {
                addr += mipmap_offset(u_size);
            }
        }

        if (texture_control_word.vq_compressed == 1) {
            std.debug.assert(twiddled);
            decode_vq(self.bgra_scratch_pad(), texture_control_word.pixel_format, gpu.vram[addr..], gpu.vram[vq_index_addr..], u_size, v_size);
        } else {
            decode_tex(self.bgra_scratch_pad(), texture_control_word.pixel_format, gpu.vram[addr..], u_size, v_size, twiddled);
        }

        // Search for an available texture index.
        var texture_index: TextureIndex = InvalidTextureIndex;
        for (0..Renderer.MaxTextures[size_index]) |i| {
            // Prefer slots that have never been used to keep textures in the cache for as long as possible.
            if (self.texture_metadata[size_index][i].status == .Invalid) {
                texture_index = @as(TextureIndex, @intCast(i));
                break;
            }
        }
        if (texture_index == InvalidTextureIndex) {
            // Search for the last recently used texture, preferring an outdated one.
            for (self.texture_metadata[size_index][0..Renderer.MaxTextures[size_index]], 0..) |*entry, idx| {
                if (entry.status == .Outdated and (texture_index == InvalidTextureIndex or self.texture_metadata[size_index][texture_index].status == .Unused or (self.texture_metadata[size_index][texture_index].status == .Outdated and self.texture_metadata[size_index][texture_index].age < entry.age))) {
                    texture_index = @as(TextureIndex, @intCast(idx));
                } else if (entry.status == .Unused and (texture_index == InvalidTextureIndex or self.texture_metadata[size_index][texture_index].age < entry.age)) {
                    texture_index = @as(TextureIndex, @intCast(idx));
                }
            }
        }

        if (texture_index == InvalidTextureIndex) {
            renderer_log.err(termcolor.red("Out of textures slot (size index: {d}, {d}x{d})"), .{ size_index, u_size, v_size });
            return 0;
        }

        const end_address = if (texture_control_word.vq_compressed == 1)
            vq_index_addr + u_size * v_size / 4
        else
            addr + switch (texture_control_word.pixel_format) {
                .Palette4BPP => u_size * v_size / 2,
                .Palette8BPP => u_size * v_size,
                else => 2 * u_size * v_size,
            };

        self.texture_metadata[size_index][texture_index] = .{
            .status = .Used,
            .control_word = texture_control_word,
            .tsp_instruction = tsp_instruction,
            .index = texture_index,
            .usage = 0,
            // NOTE: This is used for UV calculation in the shaders.
            //       In the case of stride textures, we still need to use the power of two allocation size for UV calculation, not the actual texture size.
            .size = .{ alloc_u_size, alloc_v_size },
            .start_address = addr,
            .end_address = end_address,
            .hash = texture_hash(gpu, addr, end_address),
        };

        // Fill with repeating texture data when v_size != u_size to avoid wrapping artifacts.
        const repeat_vertically = v_size <= u_size;
        const copies = if (repeat_vertically) u_size / v_size else v_size / u_size;
        var tex_source = [2][]u8{ self._scratch_pad[0 .. 4 * u_size * v_size], self._scratch_pad[0 .. 4 * u_size * v_size] };
        // Flip the repetition depending on wrapping settings
        const clamp_u = tsp_instruction.clamp_uv & 0b10 != 0;
        const clamp_v = tsp_instruction.clamp_uv & 0b01 != 0;
        const flip_u = tsp_instruction.flip_uv & 0b10 != 0 and !clamp_u;
        const flip_v = tsp_instruction.flip_uv & 0b01 != 0 and !clamp_v;
        if (copies > 1) {
            if ((clamp_u or flip_u) and !repeat_vertically) {
                tex_source[1] = self._scratch_pad[4 * u_size * v_size .. 2 * 4 * u_size * v_size];
                for (0..v_size) |v| {
                    for (0..u_size) |u| {
                        const src = tex_source[0][4 * (u_size * v + (u_size - u - 1)) ..];
                        const dst = tex_source[1][4 * (u_size * v + u) ..];
                        @memcpy(dst[0..4], src[0..4]);
                    }
                }
            } else if ((clamp_v or flip_v) and repeat_vertically) {
                tex_source[1] = self._scratch_pad[4 * u_size * v_size .. 2 * 4 * u_size * v_size];
                for (0..v_size) |v| {
                    @memcpy(tex_source[1][4 * u_size * v ..][0 .. 4 * u_size], tex_source[0][4 * u_size * (v_size - v - 1) ..][0 .. 4 * u_size]);
                }
            }
        }

        for (0..copies) |part| {
            self._gctx.queue.writeTexture(
                .{
                    .texture = self._gctx.lookupResource(self.texture_arrays[size_index]).?,
                    .origin = .{
                        .x = if (repeat_vertically) 0 else @intCast(u_size * part),
                        .y = if (repeat_vertically) @intCast(v_size * part) else 0,
                        .z = @intCast(texture_index),
                    },
                },
                .{
                    .bytes_per_row = u_size * 4,
                    .rows_per_image = v_size,
                },
                .{ .width = u_size, .height = v_size, .depth_or_array_layers = 1 },
                u8,
                tex_source[part % 2],
            );
        }

        if (texture_control_word.mip_mapped == 1)
            MipMap.generate_mipmaps(self._gctx, self.texture_arrays[size_index], texture_index);

        return texture_index;
    }

    fn reset_texture_usage(self: *Renderer) void {
        for (0..Renderer.MaxTextures.len) |j| {
            for (0..Renderer.MaxTextures[j]) |i| {
                self.texture_metadata[j][i].usage = 0;
            }
        }
    }

    fn check_texture_usage(self: *Renderer) void {
        for (0..Renderer.MaxTextures.len) |j| {
            for (0..Renderer.MaxTextures[j]) |i| {
                if (self.texture_metadata[j][i].status != .Invalid) {
                    if (self.texture_metadata[j][i].status == .Outdated) {
                        self.texture_metadata[j][i].age += 1;
                    } else {
                        if (self.texture_metadata[j][i].usage == 0) {
                            self.texture_metadata[j][i].status = .Unused;
                            self.texture_metadata[j][i].age += 1;
                        } else {
                            self.texture_metadata[j][i].status = .Used;
                            self.texture_metadata[j][i].age = 0;
                        }
                    }
                }
            }
        }
    }

    pub fn update_framebuffer_texture(self: *@This(), holly: *const HollyModule.Holly) void {
        const SPG_CONTROL = holly.read_register(HollyModule.SPG_CONTROL, .SPG_CONTROL);
        const FB_R_CTRL = holly.read_register(HollyModule.FB_R_CTRL, .FB_R_CTRL);
        const FB_R_SOF1 = holly.read_register(u32, .FB_R_SOF1);
        const FB_R_SOF2 = holly.read_register(u32, .FB_R_SOF2);
        const FB_R_SIZE = holly.read_register(HollyModule.FB_R_SIZE, .FB_R_SIZE);

        const vram = holly.vram;

        const line_size: u32 = 4 * (@as(u32, FB_R_SIZE.x_size) + 1); // From 32-bit units to bytes.
        const field_size: u32 = @as(u32, FB_R_SIZE.y_size) + 1; // Number of lines

        const bytes_per_pixels: u32 = switch (FB_R_CTRL.format) {
            0, 1 => 2,
            2 => 3,
            3 => 4,
        };

        const interlaced = SPG_CONTROL.interlace == 1;
        const x_size: u32 = @min(NativeResolution.width, line_size / bytes_per_pixels);
        const y_size: u32 = @min(NativeResolution.height, if (interlaced) field_size * 2 else field_size);
        // FB_R_SIZE.modulus: "Set this value to 0x001 in order to link the last pixel data on a line with the first pixel data on a line."
        const line_padding = 4 * if (FB_R_SIZE.modulus > 0) @as(u32, FB_R_SIZE.modulus) - 1 else 0; // From 32-bit units to bytes.

        @memset(self._scratch_pad, 0); // TODO: Fill using VO_BORDER_COL?

        for (0..y_size) |y| {
            const line_in_field = if (interlaced) y / 2 else y;
            const addr = (if (interlaced and (y % 2) == 1) FB_R_SOF2 else FB_R_SOF1) + line_in_field * (line_size + line_padding);
            for (0..x_size) |x| {
                const pixel_idx = x_size * y + x;
                const pixel_addr = addr + bytes_per_pixels * x;
                switch (FB_R_CTRL.format) {
                    0x0 => { // 0555 RGB 16 bit
                        const pixel = std.mem.bytesAsValue(Color16, vram[pixel_addr..]);
                        self._scratch_pad[pixel_idx * 4 + 0] = (@as(u8, pixel.argb1555.b) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 1] = (@as(u8, pixel.argb1555.g) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 2] = (@as(u8, pixel.argb1555.r) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 3] = 255;
                    },
                    0x1 => { // 565 RGB
                        const pixel = std.mem.bytesAsValue(Color16, vram[pixel_addr..]);
                        self._scratch_pad[pixel_idx * 4 + 0] = (@as(u8, pixel.rgb565.b) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 1] = (@as(u8, pixel.rgb565.g) << 2) | (FB_R_CTRL.concat & 0b11);
                        self._scratch_pad[pixel_idx * 4 + 2] = (@as(u8, pixel.rgb565.r) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 3] = 255;
                    },
                    0x2 => { // 888 RGB 24 bit packed
                        const pixel = vram[pixel_addr .. pixel_addr + 3];
                        self._scratch_pad[pixel_idx * 4 + 0] = pixel[2];
                        self._scratch_pad[pixel_idx * 4 + 1] = pixel[1];
                        self._scratch_pad[pixel_idx * 4 + 2] = pixel[0];
                        self._scratch_pad[pixel_idx * 4 + 3] = 255;
                    },
                    0x3 => { // 0888 RGB 32 bit
                        const pixel = vram[pixel_addr .. pixel_addr + 3];
                        self._scratch_pad[pixel_idx * 4 + 0] = pixel[0];
                        self._scratch_pad[pixel_idx * 4 + 1] = pixel[1];
                        self._scratch_pad[pixel_idx * 4 + 2] = pixel[2];
                        self._scratch_pad[pixel_idx * 4 + 3] = 255;
                    },
                }
            }
        }
        self._gctx.queue.writeTexture(
            .{
                .texture = self._gctx.lookupResource(self.framebuffer_texture).?,
                .origin = .{},
            },
            .{
                .bytes_per_row = 4 * x_size,
                .rows_per_image = y_size,
            },
            .{ .width = x_size, .height = y_size, .depth_or_array_layers = 1 },
            u8,
            self._scratch_pad,
        );
    }

    // Pulls 3 vertices from the address pointed by ISP_BACKGND_T and places them at the front of the vertex buffer.
    pub fn update_background(self: *@This(), gpu: *const HollyModule.Holly) !void {
        const tags = gpu.read_register(HollyModule.ISP_BACKGND_T, .ISP_BACKGND_T);
        const param_base: u32 = self.on_render_start_param_base;
        const addr = param_base + 4 * @as(u32, tags.tag_address);
        const isp_tsp_instruction = std.mem.bytesAsValue(HollyModule.ISPTSPInstructionWord, gpu.vram[addr..]);
        const tsp_instruction = std.mem.bytesAsValue(HollyModule.TSPInstructionWord, gpu.vram[addr + 4 ..]).*;
        const texture_control = std.mem.bytesAsValue(HollyModule.TextureControlWord, gpu.vram[addr + 8 ..]).*;
        const texture_size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);

        // FIXME: I don't understand. In the boot menu for example, this depth value is 0.0,
        //        which doesn't make sense. The vertices z position looks more inline with what
        //        I understand of the render pipeline.
        const depth = gpu.read_register(f32, .ISP_BACKGND_D);
        _ = depth;

        // Offset into the strip pointed by ISP_BACKGND_T indicated by tag_offset.
        const parameter_volume_mode = gpu.read_register(HollyModule.FPU_SHAD_SCALE, .FPU_SHAD_SCALE).enable and tags.shadow == 1;
        const skipped_vertex_byte_size: u32 = @as(u32, 4) * (if (parameter_volume_mode) 3 + tags.skip else 3 + 2 * tags.skip);
        const start = addr + 12 + tags.tag_offset * skipped_vertex_byte_size;

        var vertices: [4]Vertex = undefined;

        var vertex_byte_size: u32 = 4 * (3 + 1);
        var tex_idx: TextureIndex = 0;

        // The unused fields seems to be absent.
        if (isp_tsp_instruction.texture == 1) {
            tex_idx = self.get_texture_index(gpu, texture_size_index, texture_control) orelse self.upload_texture(gpu, tsp_instruction, texture_control);
            self.texture_metadata[texture_size_index][tex_idx].usage += 1;

            if (isp_tsp_instruction.uv_16bit == 1) {
                vertex_byte_size += 4 * 1;
            } else {
                vertex_byte_size += 4 * 2;
            }
        }
        if (isp_tsp_instruction.offset == 1) {
            vertex_byte_size += 4 * 1;
        }

        const use_alpha = tsp_instruction.use_alpha == 1;

        const tex = VertexTextureInfo{
            .index = tex_idx,
            .palette = .{
                .palette = texture_control.pixel_format == .Palette4BPP or texture_control.pixel_format == .Palette8BPP,
                .filtered = tsp_instruction.filter_mode != .Point,
                .selector = @truncate(texture_control.palette_selector() >> 4),
            },
            .shading = .{
                .textured = isp_tsp_instruction.texture,
                .mode = tsp_instruction.texture_shading_instruction,
                .ignore_alpha = tsp_instruction.ignore_texture_alpha,
                .tex_u_size = tsp_instruction.texture_u_size,
                .tex_v_size = tsp_instruction.texture_v_size,
                .depth_compare = isp_tsp_instruction.depth_compare_mode,
                .fog_control = tsp_instruction.fog_control,
                .offset_bit = isp_tsp_instruction.offset,
                .shadow_bit = 0,
                .gouraud_bit = isp_tsp_instruction.gouraud,
                .volume_bit = 0,
                .mipmap_bit = 0,
            },
        };

        for (0..3) |i| {
            const vp = @as([*]const u32, @alignCast(@ptrCast(&gpu.vram[start + i * vertex_byte_size])));
            var u: f32 = 0;
            var v: f32 = 0;
            var base_color: PackedColor = @bitCast(vp[3]);
            var offset_color: PackedColor = .{ .b = 0, .g = 0, .r = 0, .a = 0 };
            if (isp_tsp_instruction.texture == 1) {
                if (isp_tsp_instruction.uv_16bit == 1) {
                    u = @as(f32, @bitCast(vp[3] >> 16));
                    v = @as(f32, @bitCast(vp[3] & 0xFFFF));
                    base_color = @bitCast(vp[4]);
                    if (isp_tsp_instruction.offset == 1)
                        offset_color = @bitCast(vp[5]);
                } else {
                    u = @bitCast(vp[3]);
                    v = @bitCast(vp[4]);
                    base_color = @bitCast(vp[5]);
                    if (isp_tsp_instruction.offset == 1)
                        offset_color = @bitCast(vp[6]);
                }
            }
            if (!use_alpha) base_color.a = 255;

            vertices[i] = Vertex{
                .primitive_index = @intCast(self.strips_metadata.items.len),
                .x = @bitCast(vp[0]),
                .y = @bitCast(vp[1]),
                .z = @bitCast(vp[2]),
                .base_color = base_color,
                .offset_color = offset_color,
                .u = u,
                .v = v,
            };
            // NOTE: We draw the background with depth test disabled, but it might get clipped if for some reason it's drawn in front and we don't consider it for the max_depth.
            //       So, even if we're not using the min_depth right now (where it is most likely to matter), I'd rather be safe :)
            self.min_depth = @min(self.min_depth, vertices[i].z);
            self.max_depth = @max(self.max_depth, vertices[i].z);
        }

        // FIXME: In Crazy Taxi (and 2) and Soulcalibur, the vertices coordinates make no sense, even if the data format seems right:
        //     40000000 -       2.00
        //     43820000 -     260.00
        //     3727C5AC -       0.00
        //     FF000000 - (Packed color)
        //     43160000 -     150.00
        //     40000000 -       2.00
        //     3727C5AC -       0.00
        //     FF000000 - (Packed color)
        //     440C0000 -     560.00
        //     43820000 -     260.00
        //     3727C5AC -       0.00
        //     FF000000 - (Packed color)
        // There's obviously something I don't understand here.
        // Overriding the coordinates to cover the screen for now, I'm tired of seeing the broken framebuffer (Oh yeah, that's another bug.)
        // NOTE: MetalliC's comment about background rendering:
        //       "it's a bit brainfuck, and I don't think it was documented anywhere how exactly PVR2 background rendered...
        //        in short - it takes 3 vertices and calculate interpolation, UV, shading etc coefficients as for triangle rendering. but iterate these coefficients to fill the whole screen"
        //  Example of an effect that my current "solution" will fail to render correctly (Naomi example, but there might be some in the DC library too): https://youtu.be/gtIwGUG9iZk?t=127
        const screen_width: f32 = 640.0; // FIXME: Hack within a hack, hardcording the screen size too.
        const screen_height: f32 = 480.0;
        vertices[0].x = 0.0;
        vertices[0].y = 0.0;
        vertices[1].x = screen_width;
        vertices[1].y = 0.0;
        vertices[2].x = 0.0;
        vertices[2].y = screen_height;

        vertices[3] = Vertex{
            .primitive_index = vertices[0].primitive_index,
            .x = vertices[1].x,
            .y = vertices[2].y,
            .z = vertices[2].z,
            // NOTE: I have no idea how the color is computed, looking at the boot menu, this seems right.
            .base_color = vertices[2].base_color,
            .u = vertices[2].u,
            .v = vertices[1].v,
        };

        const indices = [_]u32{ 0, 1, 2, 3, 0xFFFFFFFF };

        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, 0, Vertex, &vertices);
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.index_buffer).?, 0, u32, &indices);

        try self.strips_metadata.append(.{
            .area0_instructions = tex,
        });

        std.debug.assert(FirstVertex == vertices.len);
        std.debug.assert(FirstIndex == indices.len);
    }

    pub fn update_palette(self: *@This(), gpu: *const HollyModule.Holly) !void {
        // TODO: Check if the palette has changed (palette hash) instead of updating unconditionally?

        const palette_ram = gpu.get_palette();
        const palette_ctrl_ram: u2 = @truncate(gpu.read_register(u32, .PAL_RAM_CTRL) & 0b11);

        for (0..palette_ram.len) |i| {
            self.bgra_scratch_pad()[i] = switch (palette_ctrl_ram) {
                // ARGB1555, RGB565, ARGB4444. These happen to match the values of TexturePixelFormat.
                0x0, 0x1, 0x2 => (Color16{ .value = @truncate(palette_ram[i]) }).bgra(@enumFromInt(palette_ctrl_ram), true),
                // ARGB8888
                0x3 => @bitCast(palette_ram[i]),
            };
        }
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.palette_buffer).?, 0, u8, self._scratch_pad[0 .. 4 * palette_ram.len]);
    }

    pub fn update(self: *Renderer, gpu: *const HollyModule.Holly) !void {
        const ta_lists = self.ta_lists;

        self.vertices.clearRetainingCapacity();
        self.strips_metadata.clearRetainingCapacity();

        self.reset_texture_usage();
        defer self.check_texture_usage();

        self.min_depth = std.math.floatMax(f32);
        self.max_depth = 0.0;

        self.pt_alpha_ref = @as(f32, @floatFromInt(gpu.read_register(u8, .PT_ALPHA_REF))) / 255.0;

        self.fpu_shad_scale = gpu.read_register(HollyModule.FPU_SHAD_SCALE, .FPU_SHAD_SCALE).get_factor();

        const col_pal = gpu.read_register(PackedColor, .FOG_COL_RAM);
        const col_vert = gpu.read_register(PackedColor, .FOG_COL_VERT);

        self.fog_col_pal = fRGBA.from_packed(col_pal, true);
        self.fog_col_vert = fRGBA.from_packed(col_vert, true);
        const fog_density = gpu.read_register(u16, .FOG_DENSITY);
        const fog_density_mantissa = (fog_density >> 8) & 0xFF;
        const fog_density_exponent: i8 = @bitCast(@as(u8, @truncate(fog_density & 0xFF)));
        self.fog_density = @as(f32, @floatFromInt(fog_density_mantissa)) / 128.0 * std.math.pow(f32, 2.0, @floatFromInt(fog_density_exponent));
        for (0..0x80) |i| {
            self.fog_lut[i] = gpu.get_fog_table()[i] & 0x0000FFFF;
        }

        const x_clip = gpu.read_register(HollyModule.FB_CLIP, .FB_X_CLIP);
        const y_clip = gpu.read_register(HollyModule.FB_CLIP, .FB_Y_CLIP);
        self.global_clip.x.min = x_clip.min;
        self.global_clip.x.max = x_clip.max;
        self.global_clip.y.min = y_clip.min;
        self.global_clip.y.max = y_clip.max;

        try self.update_background(gpu);
        try self.update_palette(gpu);

        for ([3]*PassMetadata{ &self.opaque_pass, &self.punchthrough_pass, &self.translucent_pass }) |pass| {
            // NOTE/FIXME: We're never purging the draw calls list. Right now we can only have at most one draw call per sampler type (i.e. 3 * 3 * 2 = 18),
            //             which is okay, I think. However this might become problematic down the line.
            //             We're saving a lot of allocations this way, but there's probably a better way to do it.
            var it = pass.pipelines.iterator();
            while (it.next()) |pipeline| {
                for (pipeline.value_ptr.*.draw_calls.values()) |*draw_call| {
                    draw_call.start_index = 0;
                    draw_call.index_count = 0;
                }
            }
        }

        inline for (.{ HollyModule.ListType.Opaque, HollyModule.ListType.PunchThrough, HollyModule.ListType.Translucent }) |list_type| {
            // Parameters specific to a polygon type
            var face_color: fARGB = undefined; // In Intensity Mode 2, the face color is the one of the previous Intensity Mode 1 Polygon
            var face_offset_color: fARGB = undefined;
            var area1_face_color: fARGB = undefined;
            var area1_face_offset_color: fARGB = undefined;
            const display_list: *const HollyModule.DisplayList = @constCast(&ta_lists).get_list(list_type);

            for (0..display_list.vertex_strips.items.len) |idx| {
                const start: u32 = @intCast(self.vertices.items.len);
                const polygon = display_list.vertex_strips.items[idx].polygon;

                // Generic Parameters
                const parameter_control_word = polygon.control_word();
                const isp_tsp_instruction = polygon.isp_tsp_instruction();
                const tsp_instruction = polygon.tsp_instruction();
                const texture_control = polygon.texture_control();
                const area1_tsp_instruction = polygon.area1_tsp_instruction();
                const area1_texture_control = polygon.area1_texture_control();

                if (parameter_control_word.obj_control.col_type == .IntensityMode1) {
                    switch (polygon) {
                        .PolygonType1 => |p| {
                            face_color = p.face_color;
                        },
                        .PolygonType2 => |p| {
                            face_color = p.face_color;
                            face_offset_color = p.face_offset_color;
                        },
                        .PolygonType4 => |p| {
                            // NOTE: In the case of Polygon Type 4 (Intensity, with Two Volumes), the Face Color is used in both the Base Color and the Offset Color.
                            face_color = p.face_color_0;
                            face_offset_color = p.face_color_0;
                            area1_face_color = p.face_color_1;
                            area1_face_offset_color = p.face_color_1;
                        },
                        else => {},
                    }
                }

                var sprite_base_color: PackedColor = undefined;
                var sprite_offset_color: PackedColor = undefined;
                if (polygon == .Sprite) {
                    sprite_base_color = polygon.Sprite.base_color;
                    sprite_offset_color = polygon.Sprite.offset_color;
                }

                var tex_idx: TextureIndex = 0;
                var tex_idx_area_1: TextureIndex = InvalidTextureIndex;
                const textured = parameter_control_word.obj_control.texture == 1;
                if (textured) {
                    const texture_size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);
                    tex_idx = self.get_texture_index(gpu, texture_size_index, texture_control) orelse self.upload_texture(gpu, tsp_instruction, texture_control);
                    self.texture_metadata[texture_size_index][tex_idx].usage += 1;
                }
                if (area1_texture_control) |tc| {
                    const texture_size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);
                    tex_idx_area_1 = self.get_texture_index(gpu, texture_size_index, tc) orelse self.upload_texture(gpu, area1_tsp_instruction.?, tc);
                    self.texture_metadata[texture_size_index][tex_idx_area_1].usage += 1;
                }

                const use_alpha = tsp_instruction.use_alpha == 1;
                const use_offset = isp_tsp_instruction.offset == 1; // FIXME: I did not find a way to validate what I'm doing with the offset color yet.

                const clamp_u = tsp_instruction.clamp_uv & 0b10 != 0;
                const clamp_v = tsp_instruction.clamp_uv & 0b01 != 0;

                const flip_u = tsp_instruction.flip_uv & 0b10 != 0 and !clamp_u;
                const flip_v = tsp_instruction.flip_uv & 0b01 != 0 and !clamp_v;

                const u_addr_mode = if (clamp_u) wgpu.AddressMode.clamp_to_edge else if (flip_u) wgpu.AddressMode.mirror_repeat else wgpu.AddressMode.repeat;
                const v_addr_mode = if (clamp_v) wgpu.AddressMode.clamp_to_edge else if (flip_v) wgpu.AddressMode.mirror_repeat else wgpu.AddressMode.repeat;

                // TODO: Add support for mipmapping (Tri-linear filtering) (And figure out what Pass A and Pass B means!).
                // Force nearest filtering when using palette textures (we'll be sampling indices into the palette). Filtering will have to be done in the shader.
                const filter_mode: wgpu.FilterMode = if (texture_control.pixel_format == .Palette4BPP or texture_control.pixel_format == .Palette8BPP) .nearest else if (tsp_instruction.filter_mode == .Point) .nearest else .linear;

                const sampler = if (textured) sampler_index(filter_mode, filter_mode, .linear, u_addr_mode, v_addr_mode) else sampler_index(.linear, .linear, .linear, .clamp_to_edge, .clamp_to_edge);

                const area0_instructions: VertexTextureInfo = .{
                    .index = tex_idx,
                    .palette = .{
                        .palette = texture_control.pixel_format == .Palette4BPP or texture_control.pixel_format == .Palette8BPP,
                        .filtered = tsp_instruction.filter_mode != .Point,
                        .selector = @truncate(texture_control.palette_selector() >> 4),
                    },
                    .shading = .{
                        .textured = parameter_control_word.obj_control.texture,
                        .mode = tsp_instruction.texture_shading_instruction,
                        .ignore_alpha = tsp_instruction.ignore_texture_alpha,
                        .tex_u_size = tsp_instruction.texture_u_size,
                        .tex_v_size = tsp_instruction.texture_v_size,
                        .src_blend_factor = tsp_instruction.src_alpha_instr,
                        .dst_blend_factor = tsp_instruction.dst_alpha_instr,
                        .depth_compare = isp_tsp_instruction.depth_compare_mode,
                        .fog_control = tsp_instruction.fog_control,
                        .offset_bit = isp_tsp_instruction.offset,
                        .shadow_bit = parameter_control_word.obj_control.shadow,
                        .gouraud_bit = isp_tsp_instruction.gouraud,
                        .volume_bit = parameter_control_word.obj_control.volume,
                        .mipmap_bit = texture_control.mip_mapped,
                    },
                };

                const area1_instructions: VertexTextureInfo = if (area1_tsp_instruction) |atspi| .{
                    .index = tex_idx_area_1,
                    .palette = .{
                        .palette = if (area1_texture_control) |a| a.pixel_format == .Palette4BPP or a.pixel_format == .Palette8BPP else false,
                        .filtered = atspi.filter_mode != .Point,
                        .selector = if (area1_texture_control) |a| @truncate(a.palette_selector() >> 4) else 0,
                    },
                    .shading = .{
                        .textured = parameter_control_word.obj_control.texture,
                        .mode = atspi.texture_shading_instruction,
                        .ignore_alpha = atspi.ignore_texture_alpha,
                        .tex_u_size = atspi.texture_u_size,
                        .tex_v_size = atspi.texture_v_size,
                        .src_blend_factor = atspi.src_alpha_instr,
                        .dst_blend_factor = atspi.dst_alpha_instr,
                        .depth_compare = isp_tsp_instruction.depth_compare_mode,
                        .fog_control = atspi.fog_control,
                        .offset_bit = isp_tsp_instruction.offset,
                        .shadow_bit = parameter_control_word.obj_control.shadow,
                        .gouraud_bit = isp_tsp_instruction.gouraud,
                        .volume_bit = parameter_control_word.obj_control.volume,
                        .mipmap_bit = if (area1_texture_control) |a| a.mip_mapped else 0,
                    },
                } else VertexTextureInfo.invalid();

                const first_vertex = display_list.vertex_strips.items[idx].vertex_parameter_index;
                const last_vertex = display_list.vertex_strips.items[idx].vertex_parameter_index + display_list.vertex_strips.items[idx].vertex_parameter_count;

                const primitive_index: u32 = @intCast(self.strips_metadata.items.len);
                try self.strips_metadata.append(.{
                    .area0_instructions = area0_instructions,
                    .area1_instructions = area1_instructions,
                });

                for (display_list.vertex_parameters.items[first_vertex..last_vertex]) |vertex| {
                    switch (vertex) {
                        // Packed Color, Non-Textured
                        .Type0 => |v| {
                            // Sanity checks.
                            std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                            std.debug.assert(!textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color.with_alpha(use_alpha),
                            });
                        },
                        // Non-Textured, Floating Color
                        .Type1 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .FloatingColor);
                            std.debug.assert(!textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color.to_packed(use_alpha),
                            });
                        },
                        // Non-Textured, Intensity
                        .Type2 => |v| {
                            std.debug.assert(!textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = face_color.apply_intensity(v.base_intensity, use_alpha),
                            });
                        },
                        // Packed Color, Textured 32bit UV
                        .Type3 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color.with_alpha(use_alpha),
                                .offset_color = if (use_offset) v.offset_color.with_alpha(true) else .{},
                                .u = v.u,
                                .v = v.v,
                            });
                        },
                        // Packed Color, Textured 16bit UV
                        .Type4 => |v| {
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color.with_alpha(use_alpha),
                                .offset_color = if (use_offset) v.offset_color.with_alpha(true) else .{},
                                .u = v.uv.u_as_f32(),
                                .v = v.uv.v_as_f32(),
                            });
                        },
                        // Floating Color, Textured
                        .Type5 => |v| {
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color.to_packed(use_alpha),
                                .offset_color = if (use_offset) v.offset_color.to_packed(true) else .{},
                                .u = v.u,
                                .v = v.v,
                            });
                        },
                        // Floating Color, Textured 16bit UV
                        .Type6 => |v| {
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color.to_packed(use_alpha),
                                .offset_color = if (use_offset) v.offset_color.to_packed(true) else .{},
                                .u = v.uv.u_as_f32(),
                                .v = v.uv.v_as_f32(),
                            });
                        },
                        // Intensity
                        .Type7 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .IntensityMode1 or parameter_control_word.obj_control.col_type == .IntensityMode2);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = face_color.apply_intensity(v.base_intensity, use_alpha),
                                .offset_color = if (use_offset) face_offset_color.apply_intensity(v.offset_intensity, true) else .{},
                                .u = v.u,
                                .v = v.v,
                            });
                        },
                        // Intensity, 16bit UV
                        .Type8 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .IntensityMode1 or parameter_control_word.obj_control.col_type == .IntensityMode2);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = face_color.apply_intensity(v.base_intensity, use_alpha),
                                .offset_color = if (use_offset) face_offset_color.apply_intensity(v.offset_intensity, true) else .{},
                                .u = v.uv.u_as_f32(),
                                .v = v.uv.v_as_f32(),
                            });
                        },
                        // Non-Textured, Packed Color, with Two Volumes
                        .Type9 => |v| {
                            std.debug.assert(area1_tsp_instruction != null);
                            std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                            std.debug.assert(!textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color_0.with_alpha(use_alpha),
                                .area1_base_color = v.base_color_1.with_alpha(use_alpha),
                            });
                        },
                        // Non-Textured, Intensity, with Two Volumes
                        .Type10 => |v| {
                            std.debug.assert(area1_tsp_instruction != null);
                            std.debug.assert(!textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = face_color.apply_intensity(v.base_intensity_0, use_alpha),
                                .area1_base_color = area1_face_color.apply_intensity(v.base_intensity_1, use_alpha),
                            });
                        },
                        // Textured, Packed Color, with Two Volumes
                        .Type11 => |v| {
                            std.debug.assert(area1_tsp_instruction != null);
                            std.debug.assert(area1_texture_control != null);
                            std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color_0.with_alpha(use_alpha),
                                .offset_color = if (use_offset) v.offset_color_0.with_alpha(true) else .{},
                                .u = v.u0,
                                .v = v.v0,
                                .area1_base_color = v.base_color_1.with_alpha(use_alpha),
                                .area1_offset_color = if (use_offset) v.offset_color_1.with_alpha(true) else .{},
                                .area1_u = v.u1,
                                .area1_v = v.v1,
                            });
                        },
                        // Textured, Packed Color, 16bit UV, with Two Volumes
                        .Type12 => |v| {
                            std.debug.assert(area1_tsp_instruction != null);
                            std.debug.assert(area1_texture_control != null);
                            std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = v.base_color_0.with_alpha(use_alpha),
                                .offset_color = if (use_offset) v.offset_color_0.with_alpha(true) else .{},
                                .u = v.uv_0.u_as_f32(),
                                .v = v.uv_0.v_as_f32(),
                                .area1_base_color = v.base_color_1.with_alpha(use_alpha),
                                .area1_offset_color = if (use_offset) v.offset_color_1.with_alpha(true) else .{},
                                .area1_u = v.uv_1.u_as_f32(),
                                .area1_v = v.uv_1.v_as_f32(),
                            });
                        },
                        // Textured, Intensity, with Two Volumes
                        .Type13 => |v| {
                            std.debug.assert(area1_tsp_instruction != null);
                            std.debug.assert(area1_texture_control != null);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = face_color.apply_intensity(v.base_intensity_0, use_alpha),
                                .offset_color = if (use_offset) face_offset_color.apply_intensity(v.offset_intensity_0, true) else .{},
                                .u = v.u0,
                                .v = v.v0,
                                .area1_base_color = area1_face_color.apply_intensity(v.base_intensity_1, use_alpha),
                                .area1_offset_color = if (use_offset) area1_face_offset_color.apply_intensity(v.offset_intensity_1, true) else .{},
                                .area1_u = v.u1,
                                .area1_v = v.v1,
                            });
                        },
                        // Textured, Intensity, with Two Volumes
                        .Type14 => |v| {
                            std.debug.assert(area1_tsp_instruction != null);
                            std.debug.assert(area1_texture_control != null);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .primitive_index = primitive_index,
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = face_color.apply_intensity(v.base_intensity_0, use_alpha),
                                .offset_color = if (use_offset) face_offset_color.apply_intensity(v.offset_intensity_0, true) else .{},
                                .u = v.uv_0.u_as_f32(),
                                .v = v.uv_0.v_as_f32(),
                                .area1_base_color = area1_face_color.apply_intensity(v.base_intensity_1, use_alpha),
                                .area1_offset_color = if (use_offset) area1_face_offset_color.apply_intensity(v.offset_intensity_1, true) else .{},
                                .area1_u = v.uv_1.u_as_f32(),
                                .area1_v = v.uv_1.v_as_f32(),
                            });
                        },
                        .SpriteType0, .SpriteType1 => {
                            var vs = gen_sprite_vertices(vertex);
                            for (&vs) |*v| {
                                v.primitive_index = primitive_index;
                                v.base_color = sprite_base_color.with_alpha(use_alpha);
                                if (use_offset)
                                    v.offset_color = sprite_offset_color;
                                self.min_depth = @min(self.min_depth, v.z);
                                self.max_depth = @max(self.max_depth, v.z);

                                try self.vertices.append(v.*);
                            }
                        },
                    }

                    self.min_depth = @min(self.min_depth, self.vertices.getLast().z);
                    self.max_depth = @max(self.max_depth, self.vertices.getLast().z);
                }

                // Triangle Strips
                if (self.vertices.items.len - start < 3) {
                    renderer_log.err("Not enough vertices in strip: {d} vertices.", .{self.vertices.items.len - start});
                } else {
                    // "In the case of a flat-shaded polygon, the Shading Color data (the Base Color, Offset Color, and Bump Map parameters) become valid starting with the third vertex after the start of the strip." - Thanks MetalliC for pointing that out!
                    if (isp_tsp_instruction.gouraud == 0) {
                        // WebGPU uses the parameters of the first vertex by default (you can specify first or either (implementation dependent), but not force last),
                        // while the DC used the last (3rd) of each triangle. This shifts the concerned parameters.
                        for (start..self.vertices.items.len - 2) |i| {
                            self.vertices.items[i].base_color = self.vertices.items[i + 2].base_color;
                            self.vertices.items[i].offset_color = self.vertices.items[i + 2].offset_color;
                            // TODO: Bump map parameters?
                        }
                    }

                    const pipeline_key = PipelineKey{
                        .src_blend_factor = translate_src_blend_factor(tsp_instruction.src_alpha_instr),
                        .dst_blend_factor = translate_dst_blend_factor(tsp_instruction.dst_alpha_instr),
                        .depth_compare = translate_depth_compare_mode(isp_tsp_instruction.depth_compare_mode),
                        .depth_write_enabled = isp_tsp_instruction.z_write_disable == 0,
                    };

                    const pass = switch (list_type) {
                        .Opaque => &self.opaque_pass,
                        .PunchThrough => &self.punchthrough_pass,
                        .Translucent => &self.translucent_pass,
                        else => @compileError("Invalid list type"),
                    };

                    var pipeline = pass.pipelines.getPtr(pipeline_key) orelse put: {
                        try pass.pipelines.put(pipeline_key, PipelineMetadata.init(self._allocator));
                        break :put pass.pipelines.getPtr(pipeline_key).?;
                    };

                    const draw_call_key = DrawCallKey{ .sampler = sampler, .user_clip = display_list.vertex_strips.items[idx].user_clip };

                    var draw_call = pipeline.draw_calls.getPtr(draw_call_key);
                    if (draw_call == null) {
                        try pipeline.draw_calls.put(draw_call_key, DrawCall.init(
                            self._allocator,
                            sampler,
                            display_list.vertex_strips.items[idx].user_clip,
                        ));
                        draw_call = pipeline.draw_calls.getPtr(draw_call_key);
                    }

                    for (start..self.vertices.items.len) |i| {
                        try draw_call.?.indices.append(@intCast(FirstVertex + i));
                    }
                    try draw_call.?.indices.append(std.math.maxInt(u32)); // Primitive Restart: Ends the current triangle strip.
                }
            }
        }

        // Send everything to the GPU
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.strips_metadata_buffer).?, 0, StripMetadata, self.strips_metadata.items);
        if (self.vertices.items.len > 0) {
            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, FirstVertex * @sizeOf(Vertex), Vertex, self.vertices.items);

            var index = FirstIndex;
            for ([3]*PassMetadata{ &self.opaque_pass, &self.punchthrough_pass, &self.translucent_pass }) |pass| {
                var it = pass.pipelines.iterator();
                while (it.next()) |entry| {
                    for (entry.value_ptr.*.draw_calls.values()) |*draw_call| {
                        draw_call.start_index = index;
                        draw_call.index_count = @intCast(draw_call.indices.items.len);
                        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.index_buffer).?, index * @sizeOf(u32), u32, draw_call.indices.items);
                        index += @intCast(draw_call.indices.items.len);

                        draw_call.indices.clearRetainingCapacity();
                    }
                }
            }
        }

        // Modifier volumes
        self.modifier_volume_vertices.clearRetainingCapacity();

        for (ta_lists.volume_triangles.items) |triangle| {
            try self.modifier_volume_vertices.append(.{ triangle.ax, triangle.ay, triangle.az, 1.0 });
            try self.modifier_volume_vertices.append(.{ triangle.bx, triangle.by, triangle.bz, 1.0 });
            try self.modifier_volume_vertices.append(.{ triangle.cx, triangle.cy, triangle.cz, 1.0 });
            self.min_depth = @min(self.min_depth, triangle.az);
            self.max_depth = @max(self.max_depth, triangle.az);
            self.min_depth = @min(self.min_depth, triangle.bz);
            self.max_depth = @max(self.max_depth, triangle.bz);
            self.min_depth = @min(self.min_depth, triangle.cz);
            self.max_depth = @max(self.max_depth, triangle.cz);
        }

        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.modifier_volume_vertex_buffer).?, 0, [4]f32, self.modifier_volume_vertices.items);
    }

    fn convert_clipping(self: *Renderer, user_clip: ?HollyModule.UserTileClipInfo) HollyModule.UserTileClipInfo {
        const factor = @divTrunc(self.resolution.width, NativeResolution.width);
        const x = factor * self.global_clip.x.min;
        const y = factor * self.global_clip.y.min;
        const width = @min(factor * (self.global_clip.x.max - self.global_clip.x.min), self.resolution.width);
        const height = @min(factor * (self.global_clip.y.max - self.global_clip.y.min), self.resolution.height);

        if (user_clip) |uc| {
            // FIXME: Handle other usages.
            //        Use Stencil for OutsideEnabled
            if (uc.usage == .InsideEnabled) {
                return .{
                    .usage = .InsideEnabled,
                    .x = @max(factor * uc.x, x),
                    .y = @max(factor * uc.y, y),
                    .width = @min(factor * uc.width, width),
                    .height = @min(factor * uc.height, height),
                };
            }
        }
        return .{
            .usage = .InsideEnabled,
            .x = x,
            .y = y,
            .width = width,
            .height = height,
        };
    }

    // Convert Framebuffer from native 640*480 to window resolution
    pub fn blit_framebuffer(self: *Renderer) void {
        const gctx = self._gctx;

        if (gctx.lookupResource(self.blit_pipeline)) |pipeline| {
            const commands = commands: {
                const encoder = gctx.device.createCommandEncoder(null);
                defer encoder.release();

                const blit_vb_info = gctx.lookupResourceInfo(self.blit_vertex_buffer).?;
                const blit_ib_info = gctx.lookupResourceInfo(self.blit_index_buffer).?;

                const framebuffer_resize_bind_group = gctx.lookupResource(self.framebuffer_resize_bind_group).?;

                const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                    .view = gctx.lookupResource(self.resized_framebuffer_texture_view).?,
                    .load_op = .clear,
                    .store_op = .store,
                }};
                const render_pass_info = wgpu.RenderPassDescriptor{
                    .label = "Blit Framebuffer",
                    .color_attachment_count = color_attachments.len,
                    .color_attachments = &color_attachments,
                };

                {
                    const pass = encoder.beginRenderPass(render_pass_info);
                    defer {
                        pass.end();
                        pass.release();
                    }

                    pass.setVertexBuffer(0, blit_vb_info.gpuobj.?, 0, blit_vb_info.size);
                    pass.setIndexBuffer(blit_ib_info.gpuobj.?, .uint32, 0, blit_ib_info.size);

                    pass.setPipeline(pipeline);

                    pass.setBindGroup(0, framebuffer_resize_bind_group, &.{});
                    pass.drawIndexed(4, 1, 0, 0, 0);
                }

                break :commands encoder.finish(null);
            };
            defer commands.release();

            gctx.submit(&.{commands});
        }
    }

    pub fn render(self: *Renderer, _: *const HollyModule.Holly) !void {
        const gctx = self._gctx;

        const ta_lists = self.ta_lists;

        const commands = commands: {
            const encoder = gctx.device.createCommandEncoder(null);
            defer encoder.release();

            const vb_info = gctx.lookupResourceInfo(self.vertex_buffer).?;
            const ib_info = gctx.lookupResourceInfo(self.index_buffer).?;

            const uniform_mem = gctx.uniformsAllocate(Uniforms, 1);
            uniform_mem.slice[0].depth_min = self.min_depth;
            uniform_mem.slice[0].depth_max = self.max_depth;
            uniform_mem.slice[0].pt_alpha_ref = self.pt_alpha_ref;
            uniform_mem.slice[0].fpu_shad_scale = self.fpu_shad_scale;
            uniform_mem.slice[0].fog_col_pal = self.fog_col_pal;
            uniform_mem.slice[0].fog_col_vert = self.fog_col_vert;
            uniform_mem.slice[0].fog_density = self.fog_density;
            uniform_mem.slice[0].fog_lut = self.fog_lut;

            const bind_group = gctx.lookupResource(self.bind_group).?;
            const depth_view = gctx.lookupResource(self.depth_texture_view).?;

            skip_opaque: {
                const color_attachments = [_]wgpu.RenderPassColorAttachment{
                    .{
                        .view = gctx.lookupResource(self.resized_framebuffer_texture_view).?,
                        .load_op = .load, // NOTE: I don't know if some games mixes direct writes to the framebuffer with renders using the PVR, but if this is the case, we'll want to load here.
                        .store_op = .store,
                    },
                    .{
                        .view = gctx.lookupResource(self.resized_framebuffer_area1_texture_view).?,
                        .load_op = .clear,
                        .store_op = .store,
                    },
                };
                const depth_attachment = wgpu.RenderPassDepthStencilAttachment{
                    .view = depth_view,
                    .depth_load_op = .clear,
                    .depth_store_op = .store,
                    .depth_clear_value = DepthClearValue,
                    .stencil_load_op = .clear,
                    .stencil_store_op = .discard,
                    .stencil_clear_value = 0,
                    .stencil_read_only = .false,
                };
                const render_pass_info = wgpu.RenderPassDescriptor{
                    .label = "Opaque pass",
                    .color_attachment_count = color_attachments.len,
                    .color_attachments = &color_attachments,
                    .depth_stencil_attachment = &depth_attachment,
                };
                const pass = encoder.beginRenderPass(render_pass_info);
                defer {
                    pass.end();
                    pass.release();
                }

                pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                pass.setBindGroup(0, bind_group, &.{uniform_mem.offset});

                // Draw background
                const background_pipeline = try self.get_or_put_opaque_pipeline(.{
                    .src_blend_factor = .one,
                    .dst_blend_factor = .zero,
                    .depth_compare = .always,
                    .depth_write_enabled = false,
                }, .Async);
                const bg_pipeline = gctx.lookupResource(background_pipeline) orelse break :skip_opaque;
                pass.setPipeline(bg_pipeline);
                pass.setBindGroup(1, gctx.lookupResource(self.sampler_bind_groups[sampler_index(.linear, .linear, .linear, .clamp_to_edge, .clamp_to_edge)]).?, &.{});
                pass.drawIndexed(FirstIndex, 1, 0, 0, 0);

                // Opaque and PunchThrough geometry
                inline for ([2]*const PassMetadata{ &self.opaque_pass, &self.punchthrough_pass }) |metadata| {
                    var it = metadata.pipelines.iterator();
                    while (it.next()) |entry| {
                        // FIXME: We should also check if at least one of the draw calls is not empty (we're keeping them around even if they are empty right now).
                        if (entry.value_ptr.*.draw_calls.count() > 0) {
                            const pl = try self.get_or_put_opaque_pipeline(entry.key_ptr.*, .Async);
                            const pipeline = gctx.lookupResource(pl) orelse break;
                            pass.setPipeline(pipeline);

                            for (entry.value_ptr.*.draw_calls.values()) |draw_call| {
                                if (draw_call.index_count > 0) {
                                    const clip = self.convert_clipping(draw_call.user_clip);
                                    pass.setScissorRect(clip.x, clip.y, clip.width, clip.height);

                                    pass.setBindGroup(1, gctx.lookupResource(self.sampler_bind_groups[draw_call.sampler]).?, &.{});
                                    pass.drawIndexed(draw_call.index_count, 1, draw_call.start_index, 0, 0);
                                }
                            }
                        }
                    }
                }
            }

            // FIXME: WGPU doesn't support reading from storage textures... This is a bad workaround.
            encoder.copyTextureToTexture(
                .{ .texture = gctx.lookupResource(self.resized_framebuffer_texture).? },
                .{ .texture = gctx.lookupResource(self.resized_framebuffer_copy_texture).? },
                .{ .width = self.resolution.width, .height = self.resolution.height },
            );

            if (ta_lists.opaque_modifier_volumes.items.len > 0) skip_mv: {
                const closed_modifier_volume_pipeline = gctx.lookupResource(self.closed_modifier_volume_pipeline) orelse break :skip_mv;
                const shift_stencil_buffer_modifier_volume_pipeline = gctx.lookupResource(self.shift_stencil_buffer_modifier_volume_pipeline) orelse break :skip_mv;
                const modifier_volume_apply_pipeline = gctx.lookupResource(self.modifier_volume_apply_pipeline) orelse break :skip_mv;
                const open_modifier_volume_pipeline = gctx.lookupResource(self.open_modifier_volume_pipeline) orelse break :skip_mv;

                // Write to stencil buffer
                {
                    const modifier_volume_bind_group = gctx.lookupResource(self.modifier_volume_bind_group).?;
                    const vs_uniform_mem = gctx.uniformsAllocate(struct { min_depth: f32, max_depth: f32 }, 1);
                    vs_uniform_mem.slice[0].min_depth = self.min_depth;
                    vs_uniform_mem.slice[0].max_depth = self.max_depth;

                    const modifier_volume_vb_info = gctx.lookupResourceInfo(self.modifier_volume_vertex_buffer).?;

                    const depth_attachment = wgpu.RenderPassDepthStencilAttachment{
                        .view = depth_view,
                        .depth_load_op = .load,
                        .depth_store_op = .store,
                        .depth_clear_value = DepthClearValue,
                        .depth_read_only = .false,
                        .stencil_load_op = .clear,
                        .stencil_store_op = .store,
                        .stencil_clear_value = 0,
                        .stencil_read_only = .false,
                    };
                    const render_pass_info = wgpu.RenderPassDescriptor{
                        .label = "Modifier Volume Stencil",
                        .color_attachment_count = 0,
                        .color_attachments = null,
                        .depth_stencil_attachment = &depth_attachment,
                    };

                    const pass = encoder.beginRenderPass(render_pass_info);
                    defer {
                        pass.end();
                        pass.release();
                    }

                    pass.setVertexBuffer(0, modifier_volume_vb_info.gpuobj.?, 0, modifier_volume_vb_info.size);
                    pass.setBindGroup(0, modifier_volume_bind_group, &.{vs_uniform_mem.offset});

                    // Close volume pass.
                    // Counts triangles passing the depth test: If odd, the volume intersects the depth buffer.
                    for (ta_lists.opaque_modifier_volumes.items) |volume| {
                        if (volume.closed) {
                            pass.setStencilReference(0x00);
                            pass.setPipeline(closed_modifier_volume_pipeline);
                            pass.draw(3 * volume.triangle_count, 1, 3 * volume.first_triangle_index, 0);

                            pass.setStencilReference(0x01);
                            pass.setPipeline(shift_stencil_buffer_modifier_volume_pipeline);
                            // NOTE: We could draw a single fullscreen quad here.
                            pass.draw(3 * volume.triangle_count, 1, 3 * volume.first_triangle_index, 0);
                        }
                    }
                    // Open "volume" pass.
                    // Triangle passing the depth test immediately set the stencil buffer.
                    pass.setStencilReference(0x02);
                    pass.setPipeline(open_modifier_volume_pipeline);
                    for (ta_lists.opaque_modifier_volumes.items) |volume| {
                        if (!volume.closed)
                            pass.draw(3 * volume.triangle_count, 1, 3 * volume.first_triangle_index, 0);
                    }
                }
                // Copy 'area 1' colors where the stencil buffer is non-zero.
                {
                    const blit_vb_info = gctx.lookupResourceInfo(self.blit_vertex_buffer).?;
                    const blit_ib_info = gctx.lookupResourceInfo(self.blit_index_buffer).?;
                    const mva_bind_group = gctx.lookupResource(self.modifier_volume_apply_bind_group).?;
                    const dest_view = gctx.lookupResource(self.resized_framebuffer_texture_view).?;

                    const depth_attachment = wgpu.RenderPassDepthStencilAttachment{
                        .view = depth_view,
                        .depth_load_op = .undef,
                        .depth_store_op = .undef,
                        .depth_clear_value = DepthClearValue,
                        .depth_read_only = .true,
                        .stencil_load_op = .undef,
                        .stencil_store_op = .undef,
                        .stencil_clear_value = 0,
                        .stencil_read_only = .true,
                    };
                    const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                        .view = dest_view,
                        .load_op = .load,
                        .store_op = .store,
                    }};
                    const render_pass_info = wgpu.RenderPassDescriptor{
                        .label = "Modifier Volume Apply",
                        .color_attachment_count = color_attachments.len,
                        .color_attachments = &color_attachments,
                        .depth_stencil_attachment = &depth_attachment,
                    };

                    const pass = encoder.beginRenderPass(render_pass_info);
                    defer {
                        pass.end();
                        pass.release();
                    }

                    pass.setVertexBuffer(0, blit_vb_info.gpuobj.?, 0, blit_vb_info.size);
                    pass.setIndexBuffer(blit_ib_info.gpuobj.?, .uint32, 0, blit_ib_info.size);
                    pass.setPipeline(modifier_volume_apply_pipeline);
                    pass.setBindGroup(0, mva_bind_group, &.{});

                    pass.setStencilReference(0x02);
                    pass.drawIndexed(4, 1, 0, 0, 0);
                }

                // Copy the result to resized_framebuffer_copy_texture_view too.
                encoder.copyTextureToTexture(
                    .{ .texture = gctx.lookupResource(self.resized_framebuffer_texture).? },
                    .{ .texture = gctx.lookupResource(self.resized_framebuffer_copy_texture).? },
                    .{ .width = self.resolution.width, .height = self.resolution.height },
                );
            }

            // Generate all translucent fragments
            const translucent_bind_group = gctx.lookupResource(self.translucent_bind_group).?;
            const blend_bind_group = gctx.lookupResource(self.blend_bind_group).?;

            const oit_color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                .view = gctx.lookupResource(self.resized_framebuffer_texture_view).?,
                .load_op = .load,
                .store_op = .store,
            }};
            const oit_render_pass_info = wgpu.RenderPassDescriptor{
                .label = "Translucent Pass",
                .color_attachment_count = oit_color_attachments.len,
                .color_attachments = &oit_color_attachments,
                .depth_stencil_attachment = null, // TODO: Use the depth buffer rather than discarding the fragments manually?
                // NOTE: We could need to sample it in the fragment shader to correctly implement "Pre-sort"
                //       mode where the depth_compare mode can be set by the user (it is written, but unused).
            };

            const slice_size = self.resolution.height / OITHorizontalSlices;
            for (0..OITHorizontalSlices) |i| {
                const start_y: u32 = @as(u32, @intCast(i)) * slice_size;

                const oit_uniform_mem = gctx.uniformsAllocate(struct { max_fragments: u32, target_width: u32, start_y: u32 }, 1);
                oit_uniform_mem.slice[0].target_width = self.resolution.width;
                oit_uniform_mem.slice[0].start_y = start_y;

                // Render Translucent Modifier Volumes
                if (ta_lists.translucent_modifier_volumes.items.len > 0) skip_tmv: {
                    const translucent_modvol_pipeline = gctx.lookupResource(self.translucent_modvol_pipeline) orelse break :skip_tmv;
                    const translucent_modvol_merge_pipeline = gctx.lookupResource(self.translucent_modvol_merge_pipeline) orelse break :skip_tmv;

                    oit_uniform_mem.slice[0].max_fragments = @intCast(self.get_max_storage_buffer_binding_size() / VolumeLinkedListNodeSize);

                    const modifier_volume_bind_group = gctx.lookupResource(self.modifier_volume_bind_group).?;
                    const translucent_modvol_bind_group = gctx.lookupResource(self.translucent_modvol_bind_group).?;
                    const translucent_modvol_merge_bind_group = gctx.lookupResource(self.translucent_modvol_merge_bind_group).?;
                    const vs_uniform_mem = gctx.uniformsAllocate(struct { min_depth: f32, max_depth: f32 }, 1);
                    vs_uniform_mem.slice[0].min_depth = self.min_depth;
                    vs_uniform_mem.slice[0].max_depth = self.max_depth;

                    const modifier_volume_vb_info = gctx.lookupResourceInfo(self.modifier_volume_vertex_buffer).?;

                    const depth_attachment = wgpu.RenderPassDepthStencilAttachment{
                        .view = depth_view,
                        .depth_read_only = .true,
                        .stencil_read_only = .true,
                    };
                    const render_pass_info = wgpu.RenderPassDescriptor{
                        .label = "Translucent Modifier Volumes",
                        .color_attachment_count = 0,
                        .color_attachments = null,
                        .depth_stencil_attachment = &depth_attachment,
                    };

                    {
                        const pass = encoder.beginRenderPass(render_pass_info);
                        defer {
                            pass.end();
                            pass.release();
                        }
                        pass.setVertexBuffer(0, modifier_volume_vb_info.gpuobj.?, 0, modifier_volume_vb_info.size);
                        pass.setBindGroup(0, modifier_volume_bind_group, &.{vs_uniform_mem.offset});
                        pass.setPipeline(translucent_modvol_pipeline);
                        pass.setScissorRect(0, start_y, self.resolution.width, slice_size);

                        // Close volume pass.
                        var volume_index: u32 = 0;
                        for (ta_lists.translucent_modifier_volumes.items) |volume| {
                            if (volume.closed) {
                                const oit_fs_uniform_mem = gctx.uniformsAllocate(struct { volume_index: u32 }, 1);
                                oit_fs_uniform_mem.slice[0].volume_index = volume_index;
                                pass.setBindGroup(1, translucent_modvol_bind_group, &.{ oit_uniform_mem.offset, oit_fs_uniform_mem.offset });
                                volume_index += 1;

                                pass.draw(3 * volume.triangle_count, 1, 3 * volume.first_triangle_index, 0);
                            } else {
                                renderer_log.warn(termcolor.yellow("TODO: Unhandled Open Translucent Modifier Volume!"), .{});
                                // TODO: Almost the same thing, but the compute shader is really simple: Take the smallest
                                //       depth value and add a volume from it to "infinity" (1.0+ depth). Or find a more efficient way :)
                            }
                        }
                    }

                    {
                        const pass = encoder.beginComputePass(.{ .label = "Merge Modifier Volumes", .timestamp_write_count = 0, .timestamp_writes = null });
                        defer {
                            pass.end();
                            pass.release();
                        }
                        const num_groups = [2]u32{ @divExact(self.resolution.width, 8), @divExact(self.resolution.height, OITHorizontalSlices * 8) };
                        pass.setPipeline(translucent_modvol_merge_pipeline);

                        pass.setBindGroup(0, translucent_modvol_merge_bind_group, &.{oit_uniform_mem.offset});
                        pass.dispatchWorkgroups(num_groups[0], num_groups[1], 1);
                    }
                }

                oit_uniform_mem.slice[0].max_fragments = @intCast(self.get_max_storage_buffer_binding_size() / OITLinkedListNodeSize);

                skip: {
                    const pipeline = gctx.lookupResource(self.translucent_pipeline) orelse break :skip;

                    const pass = encoder.beginRenderPass(oit_render_pass_info);
                    defer {
                        pass.end();
                        pass.release();
                    }

                    pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                    pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                    pass.setPipeline(pipeline);

                    pass.setBindGroup(0, bind_group, &.{uniform_mem.offset});
                    pass.setBindGroup(2, translucent_bind_group, &.{oit_uniform_mem.offset});

                    var it = self.translucent_pass.pipelines.iterator();
                    while (it.next()) |entry| {
                        if (entry.value_ptr.*.draw_calls.count() > 0) {
                            for (entry.value_ptr.*.draw_calls.values()) |draw_call| {
                                if (draw_call.index_count > 0) {
                                    var clip = self.convert_clipping(draw_call.user_clip);
                                    const min_max_y = @min(clip.y + clip.height, start_y + slice_size);
                                    clip.y = @max(clip.y, start_y);
                                    clip.height = if (min_max_y > clip.y) min_max_y - clip.y else 0;
                                    if (clip.height > 0 and clip.width > 0) {
                                        pass.setScissorRect(clip.x, clip.y, clip.width, clip.height);

                                        pass.setBindGroup(1, gctx.lookupResource(self.sampler_bind_groups[draw_call.sampler]).?, &.{});
                                        pass.drawIndexed(draw_call.index_count, 1, draw_call.start_index, 0, 0);
                                    }
                                }
                            }
                        }
                    }
                }

                // Blend the results of the translucent pass
                skip_blend: {
                    const pipeline = gctx.lookupResource(self.blend_pipeline) orelse break :skip_blend;

                    const pass = encoder.beginComputePass(null);
                    defer {
                        pass.end();
                        pass.release();
                    }
                    const num_groups = [2]u32{ @divExact(self.resolution.width, 8), @divExact(self.resolution.height, OITHorizontalSlices * 8) };
                    pass.setPipeline(pipeline);

                    pass.setBindGroup(0, blend_bind_group, &.{oit_uniform_mem.offset});

                    pass.dispatchWorkgroups(num_groups[0], num_groups[1], 1);
                }
            }

            // Blit to framebuffer texture
            skip_blit: {
                const pipeline = gctx.lookupResource(self.blit_pipeline) orelse break :skip_blit;

                const blit_vb_info = gctx.lookupResourceInfo(self.blit_vertex_buffer).?;
                const blit_ib_info = gctx.lookupResourceInfo(self.blit_index_buffer).?;
                const blit_bind_group = gctx.lookupResource(self.blit_bind_group).?;

                const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                    .view = gctx.lookupResource(self.framebuffer_texture_view),
                    .load_op = .clear,
                    .store_op = .store,
                }};
                const render_pass_info = wgpu.RenderPassDescriptor{
                    .label = "Framebuffer Blit",
                    .color_attachment_count = color_attachments.len,
                    .color_attachments = &color_attachments,
                };

                const pass = encoder.beginRenderPass(render_pass_info);
                defer {
                    pass.end();
                    pass.release();
                }

                pass.setVertexBuffer(0, blit_vb_info.gpuobj.?, 0, blit_vb_info.size);
                pass.setIndexBuffer(blit_ib_info.gpuobj.?, .uint32, 0, blit_ib_info.size);

                pass.setPipeline(pipeline);

                pass.setBindGroup(0, blit_bind_group, &.{});
                pass.drawIndexed(4, 1, 0, 0, 0);
            }

            if (ExperimentalFBWriteBack)
                encoder.copyTextureToBuffer(
                    .{
                        .texture = gctx.lookupResource(self.framebuffer_texture).?,
                        .mip_level = 0,
                        .origin = .{},
                        .aspect = .all,
                    },
                    .{
                        .layout = .{
                            .offset = 0,
                            .bytes_per_row = 4 * NativeResolution.width,
                            .rows_per_image = NativeResolution.height,
                        },
                        .buffer = gctx.lookupResource(self.framebuffer_copy_buffer).?,
                    },
                    .{
                        .width = NativeResolution.width,
                        .height = NativeResolution.height,
                        .depth_or_array_layers = 1,
                    },
                );

            break :commands encoder.finish(null);
        };
        defer commands.release();

        gctx.submit(&.{commands});
    }

    pub fn draw(self: *Renderer) void {
        if (self._gctx.lookupResource(self.blit_pipeline)) |pipeline| {
            const back_buffer_view = self._gctx.swapchain.getCurrentTextureView();
            defer back_buffer_view.release();

            // NOTE: This does not change - expect for the back_buffer_view - and could be recorded once and for all.
            const commands = commands: {
                const encoder = self._gctx.device.createCommandEncoder(null);
                defer encoder.release();

                // Blit to screen
                {
                    const vb_info = self._gctx.lookupResourceInfo(self.blit_to_window_vertex_buffer).?;
                    const ib_info = self._gctx.lookupResourceInfo(self.blit_index_buffer).?;
                    const blit_bind_group = self._gctx.lookupResource(self.blit_bind_group).?;

                    const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                        .view = back_buffer_view,
                        .load_op = .clear,
                        .store_op = .store,
                    }};
                    const render_pass_info = wgpu.RenderPassDescriptor{
                        .label = "Final Blit",
                        .color_attachment_count = color_attachments.len,
                        .color_attachments = &color_attachments,
                    };

                    const pass = encoder.beginRenderPass(render_pass_info);
                    defer {
                        pass.end();
                        pass.release();
                    }

                    pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                    pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                    pass.setPipeline(pipeline);

                    pass.setBindGroup(0, blit_bind_group, &.{});
                    pass.drawIndexed(4, 1, 0, 0, 0);
                }

                break :commands encoder.finish(null);
            };
            defer commands.release();

            self._gctx.submit(&.{commands});
        }
    }

    fn get_or_put_opaque_pipeline(self: *Renderer, key: PipelineKey, sync: enum { Sync, Async }) !zgpu.RenderPipelineHandle {
        if (self.opaque_pipelines.get(key)) |pl|
            return pl;

        renderer_log.info("Creating Pipeline: {any}", .{key});
        const start = std.time.milliTimestamp();

        const color_targets = [_]wgpu.ColorTargetState{
            .{
                .format = zgpu.GraphicsContext.swapchain_format,
                .blend = &wgpu.BlendState{
                    .color = .{ .operation = .add, .src_factor = key.src_blend_factor, .dst_factor = key.dst_blend_factor },
                    .alpha = .{ .operation = .add, .src_factor = .one, .dst_factor = .zero }, // FIXME: Not sure about this.
                },
            },
            .{
                .format = zgpu.GraphicsContext.swapchain_format,
                .blend = &wgpu.BlendState{
                    .color = .{ .operation = .add, .src_factor = key.src_blend_factor, .dst_factor = key.dst_blend_factor },
                    .alpha = .{ .operation = .add, .src_factor = .one, .dst_factor = .zero },
                },
            },
        };

        const pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = wgpu.VertexState{
                .module = self.opaque_vertex_shader_module,
                .entry_point = "main",
                .buffer_count = VertexBufferLayout.len,
                .buffers = &VertexBufferLayout,
            },
            .primitive = wgpu.PrimitiveState{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_strip,
                .strip_index_format = .uint32,
            },
            .depth_stencil = &wgpu.DepthStencilState{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = key.depth_write_enabled,
                .depth_compare = key.depth_compare,
            },
            .fragment = &wgpu.FragmentState{
                .module = self.opaque_fragment_shader_module,
                .entry_point = "main",
                .target_count = color_targets.len,
                .targets = &color_targets,
            },
        };

        switch (sync) {
            .Async => {
                // Experiment: Asynchronous pipeline creation
                self.opaque_pipelines.putAssumeCapacityNoClobber(key, .{});
                const ptr = self.opaque_pipelines.getPtr(key).?;
                self._gctx.createRenderPipelineAsync(self._allocator, self.opaque_pipeline_layout, pipeline_descriptor, ptr);
                return ptr.*;
            },
            else => {
                defer renderer_log.info("Pipeline created in {d}ms", .{std.time.milliTimestamp() - start});
                const pl = self._gctx.createRenderPipeline(self.opaque_pipeline_layout, pipeline_descriptor);

                if (!self._gctx.isResourceValid(pl)) {
                    renderer_log.err("Error creating pipeline.", .{});
                    renderer_log.err("{any}", .{pipeline_descriptor});
                }

                self.opaque_pipelines.putAssumeCapacityNoClobber(key, pl);

                return pl;
            },
        }
    }

    fn deinit_screen_textures(self: *@This()) void {
        self._gctx.releaseResource(self.depth_texture_view);
        self._gctx.destroyResource(self.depth_texture);

        self._gctx.releaseResource(self.resized_framebuffer_texture_view);
        self._gctx.destroyResource(self.resized_framebuffer_texture);

        self._gctx.releaseResource(self.resized_framebuffer_copy_texture_view);
        self._gctx.destroyResource(self.resized_framebuffer_copy_texture);

        self._gctx.releaseResource(self.blit_bind_group);

        self._gctx.releaseResource(self.list_heads_buffer);
        self._gctx.releaseResource(self.linked_list_buffer);

        self._gctx.releaseResource(self.modvol_list_heads_buffer);
        self._gctx.releaseResource(self.modvol_linked_list_buffer);
        self._gctx.releaseResource(self.modvol_volumes_buffer);

        self._gctx.releaseResource(self.translucent_bind_group);
        self._gctx.releaseResource(self.translucent_modvol_bind_group);
        self._gctx.releaseResource(self.translucent_modvol_merge_bind_group);

        self._gctx.releaseResource(self.blend_bind_group);
    }

    // Creates all resources that depends on the render size
    pub fn on_inner_resolution_change(self: *@This()) void {
        self.deinit_screen_textures();

        // Create a new depth texture to match the new render size.
        const depth = create_depth_texture(self._gctx, self.resolution);
        self.depth_texture = depth.texture;
        self.depth_texture_view = depth.view;
        self.depth_only_texture_view = depth.depth_only_view;

        // Same thing for our screen size framebuffer.
        const resized_framebuffer = create_resized_framebuffer_texture(self._gctx, self.resolution, true, false);
        self.resized_framebuffer_texture = resized_framebuffer.texture;
        self.resized_framebuffer_texture_view = resized_framebuffer.view;

        const resized_framebuffer_area1 = create_resized_framebuffer_texture(self._gctx, self.resolution, false, false);
        self.resized_framebuffer_area1_texture = resized_framebuffer_area1.texture;
        self.resized_framebuffer_area1_texture_view = resized_framebuffer_area1.view;

        const resized_framebuffer_copy = create_resized_framebuffer_texture(self._gctx, self.resolution, false, true);
        self.resized_framebuffer_copy_texture = resized_framebuffer_copy.texture;
        self.resized_framebuffer_copy_texture_view = resized_framebuffer_copy.view;

        const blit_bind_group_layout = self._gctx.createBindGroupLayout(&BlitBindGroupLayout);
        defer self._gctx.releaseResource(blit_bind_group_layout);

        self.blit_bind_group = self._gctx.createBindGroup(blit_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = resized_framebuffer.view },
            .{ .binding = 1, .sampler_handle = self.samplers[sampler_index(.linear, .linear, .linear, .clamp_to_edge, .clamp_to_edge)] },
        });

        const mv_apply_bind_group_layout = self._gctx.createBindGroupLayout(&.{
            zgpu.textureEntry(0, .{ .fragment = true }, .float, .tvdim_2d, false),
        });
        defer self._gctx.releaseResource(mv_apply_bind_group_layout);

        self.modifier_volume_apply_bind_group = self._gctx.createBindGroup(mv_apply_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = self.resized_framebuffer_area1_texture_view },
        });

        self.create_oit_buffers();
        self.create_translucent_bind_group();
        self.create_translucent_modvol_bind_group();
        self.create_translucent_modvol_merge_bind_group();
        self.create_blend_bind_group();

        self.update_blit_to_screen_vertex_buffer();
    }

    pub fn update_blit_to_screen_vertex_buffer(self: *@This()) void {
        const iw: f32 = @floatFromInt(self.resolution.width);
        const ih: f32 = @floatFromInt(self.resolution.height);
        const tw: f32 = @floatFromInt(self._gctx.swapchain_descriptor.width);
        const th: f32 = @floatFromInt(self._gctx.swapchain_descriptor.height);
        const ias = iw / ih;
        const tas = tw / th;
        var displayMode = self.display_mode;
        if (displayMode == .Center and (tw < iw or th < ih))
            displayMode = .Fit;
        const blit_vertex_data = switch (displayMode) {
            .Center => [_]f32{
                // x    y     u    v
                -@min(1.0, iw / tw), @min(1.0, ih / th),  0.0, 0.0,
                @min(1.0, iw / tw),  @min(1.0, ih / th),  1.0, 0.0,
                @min(1.0, iw / tw),  -@min(1.0, ih / th), 1.0, 1.0,
                -@min(1.0, iw / tw), -@min(1.0, ih / th), 0.0, 1.0,
            },
            .Fit => if (tas < ias) [_]f32{
                // x    y     u    v
                -1.0, tas / ias,  0.0, 0.0,
                1.0,  tas / ias,  1.0, 0.0,
                1.0,  -tas / ias, 1.0, 1.0,
                -1.0, -tas / ias, 0.0, 1.0,
            } else [_]f32{
                // x    y     u    v
                -ias / tas, 1.0,  0.0, 0.0,
                ias / tas,  1.0,  1.0, 0.0,
                ias / tas,  -1.0, 1.0, 1.0,
                -ias / tas, -1.0, 0.0, 1.0,
            },
            .Stretch => [_]f32{
                // x    y     u    v
                -1.0, 1.0,  0.0, 0.0,
                1.0,  1.0,  1.0, 0.0,
                1.0,  -1.0, 1.0, 1.0,
                -1.0, -1.0, 0.0, 1.0,
            },
        };
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.blit_to_window_vertex_buffer).?, 0, f32, blit_vertex_data[0..]);
    }

    fn create_depth_texture(gctx: *zgpu.GraphicsContext, resolution: Resolution) struct {
        texture: zgpu.TextureHandle,
        view: zgpu.TextureViewHandle,
        depth_only_view: zgpu.TextureViewHandle,
    } {
        const texture = gctx.createTexture(.{
            .usage = .{
                .render_attachment = true,
                .texture_binding = true,
            },
            .dimension = .tdim_2d,
            .size = .{
                .width = resolution.width,
                .height = resolution.height,
                .depth_or_array_layers = 1,
            },
            .format = .depth32_float_stencil8,
            .mip_level_count = 1,
            .sample_count = 1,
        });
        const view = gctx.createTextureView(texture, .{});
        const depth_only_view = gctx.createTextureView(texture, .{ .aspect = .depth_only });
        return .{ .texture = texture, .view = view, .depth_only_view = depth_only_view };
    }

    fn create_resized_framebuffer_texture(gctx: *zgpu.GraphicsContext, resolution: Resolution, copy_src: bool, copy_dst: bool) struct {
        texture: zgpu.TextureHandle,
        view: zgpu.TextureViewHandle,
    } {
        const resized_framebuffer_texture = gctx.createTexture(.{
            .usage = .{
                .render_attachment = true,
                .texture_binding = true,
                .storage_binding = true,
                .copy_src = copy_src, // FIXME: This should not be needed (wgpu doesn't support reading from storage textures...)
                .copy_dst = copy_dst,
            },
            .size = .{
                .width = resolution.width,
                .height = resolution.height,
                .depth_or_array_layers = 1,
            },
            .format = zgpu.GraphicsContext.swapchain_format,
            .mip_level_count = 1,
        });
        const resized_framebuffer_texture_view = gctx.createTextureView(resized_framebuffer_texture, .{});

        return .{ .texture = resized_framebuffer_texture, .view = resized_framebuffer_texture_view };
    }

    fn create_oit_buffers(self: *@This()) void {
        {
            self.list_heads_buffer = self._gctx.createBuffer(.{
                .usage = .{ .storage = true },
                .size = self.get_linked_list_heads_size(),
                .mapped_at_creation = .true,
            });
            self._gctx.lookupResource(self.list_heads_buffer).?.setLabel("OIT List Heads Buffer");
            const init_buffer = self._gctx.lookupResourceInfo(self.list_heads_buffer).?.gpuobj.?;
            const mapped = init_buffer.getMappedRange(u32, 0, self.get_linked_list_heads_size() / @sizeOf(u32));
            @memset(mapped.?, 0xFFFFFFFF); // Set heads to invalid (or 'end-of-list')
            mapped.?[0] = 0; // Set fragment count to 0
            init_buffer.unmap();

            self.linked_list_buffer = self._gctx.createBuffer(.{
                .usage = .{ .copy_dst = true, .storage = true },
                .size = self.get_max_storage_buffer_binding_size(),
            });
            self._gctx.lookupResource(self.linked_list_buffer).?.setLabel("OIT Linked List Buffer");
        }

        // Mod Vol
        {
            self.modvol_list_heads_buffer = self._gctx.createBuffer(.{
                .usage = .{ .storage = true },
                .size = self.get_linked_list_heads_size(),
                .mapped_at_creation = .true,
            });
            self._gctx.lookupResource(self.modvol_list_heads_buffer).?.setLabel("ModVol List Heads Buffer");
            {
                const init_buffer = self._gctx.lookupResourceInfo(self.modvol_list_heads_buffer).?.gpuobj.?;
                const mapped = init_buffer.getMappedRange(u32, 0, self.get_linked_list_heads_size() / @sizeOf(u32));
                @memset(mapped.?, 0xFFFFFFFF); // Set heads to invalid (or 'end-of-list')
                mapped.?[0] = 0; // Set fragment count to 0
                init_buffer.unmap();
            }
            self.modvol_linked_list_buffer = self._gctx.createBuffer(.{
                .usage = .{ .copy_dst = true, .storage = true },
                .size = self.get_modvol_linked_list_size(),
            });
            self._gctx.lookupResource(self.modvol_linked_list_buffer).?.setLabel("ModVol Fragments Buffer");

            self.modvol_volumes_buffer = self._gctx.createBuffer(.{
                .usage = .{ .copy_dst = true, .storage = true },
                .size = self.get_modvol_volumes_size(),
                .mapped_at_creation = .true,
            });
            self._gctx.lookupResource(self.modvol_volumes_buffer).?.setLabel("ModVol Volumes Buffer");

            {
                const init_buffer = self._gctx.lookupResourceInfo(self.modvol_volumes_buffer).?.gpuobj.?;
                const mapped = init_buffer.getMappedRange(u8, 0, self.get_modvol_volumes_size());
                @memset(mapped.?, 0); // Set all counts to 0
                init_buffer.unmap();
            }
        }
    }

    fn create_translucent_bind_group(self: *@This()) void {
        self.translucent_bind_group = self._gctx.createBindGroup(self.translucent_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = 3 * @sizeOf(u32) },
            .{ .binding = 1, .buffer_handle = self.list_heads_buffer, .offset = 0, .size = self.get_linked_list_heads_size() },
            .{ .binding = 2, .buffer_handle = self.linked_list_buffer, .offset = 0, .size = self.get_max_storage_buffer_binding_size() },
            .{ .binding = 3, .texture_view_handle = self.depth_only_texture_view },
        });
    }

    fn create_translucent_modvol_bind_group(self: *@This()) void {
        self.translucent_modvol_bind_group = self._gctx.createBindGroup(self.translucent_modvol_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = 3 * @sizeOf(u32) },
            .{ .binding = 1, .buffer_handle = self.modvol_list_heads_buffer, .offset = 0, .size = self.get_linked_list_heads_size() },
            .{ .binding = 2, .buffer_handle = self.modvol_linked_list_buffer, .offset = 0, .size = self.get_modvol_linked_list_size() },
            .{ .binding = 3, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = 1 * @sizeOf(u32) },
        });
    }
    fn create_translucent_modvol_merge_bind_group(self: *@This()) void {
        self.translucent_modvol_merge_bind_group = self._gctx.createBindGroup(self.translucent_modvol_merge_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = 3 * @sizeOf(u32) },
            .{ .binding = 1, .buffer_handle = self.modvol_list_heads_buffer, .offset = 0, .size = self.get_linked_list_heads_size() },
            .{ .binding = 2, .buffer_handle = self.modvol_linked_list_buffer, .offset = 0, .size = self.get_modvol_linked_list_size() },
            .{ .binding = 3, .buffer_handle = self.modvol_volumes_buffer, .offset = 0, .size = self.get_modvol_volumes_size() },
        });
    }

    fn create_blend_bind_group(self: *@This()) void {
        self.blend_bind_group = self._gctx.createBindGroup(self.blend_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = 3 * @sizeOf(u32) },
            .{ .binding = 1, .buffer_handle = self.list_heads_buffer, .offset = 0, .size = self.get_linked_list_heads_size() },
            .{ .binding = 2, .buffer_handle = self.linked_list_buffer, .offset = 0, .size = self.get_max_storage_buffer_binding_size() },
            .{ .binding = 3, .texture_view_handle = self.resized_framebuffer_copy_texture_view },
            .{ .binding = 4, .texture_view_handle = self.resized_framebuffer_texture_view },
            .{ .binding = 5, .buffer_handle = self.modvol_volumes_buffer, .offset = 0, .size = self.get_modvol_volumes_size() },
        });
    }

    fn get_linked_list_heads_size(self: *const @This()) u64 {
        return (1 + self.resolution.width * self.resolution.height / OITHorizontalSlices) * @sizeOf(u32);
    }

    fn get_modvol_linked_list_size(self: *const @This()) u64 {
        return @min(
            VolumeLinkedListNodeSize * (1 + self.resolution.width * self.resolution.height / OITHorizontalSlices) * MaxFragmentsPerPixel,
            self.get_max_storage_buffer_binding_size(),
        );
    }

    fn get_modvol_volumes_size(self: *const @This()) u64 {
        return VolumePixelSize * (1 + self.resolution.width * self.resolution.height / OITHorizontalSlices);
    }

    fn get_max_storage_buffer_binding_size(self: *const @This()) u64 {
        var r: zgpu.wgpu.SupportedLimits = .{};
        if (!self._gctx.device.getLimits(&r)) {
            renderer_log.err("get_max_storage_buffer_binding_size: Failed to get device limits.", .{});
            return 134217728; // Min WebGPU spec.
        }
        return r.limits.max_storage_buffer_binding_size;
    }
};
