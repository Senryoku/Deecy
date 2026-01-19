const std = @import("std");
const termcolor = @import("termcolor");

const log = std.log.scoped(.renderer);
const Once = @import("helpers").Once;

const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;

const DreamcastModule = @import("dreamcast");
const Dreamcast = DreamcastModule.Dreamcast;
const HollyModule = DreamcastModule.HollyModule;
const Colors = HollyModule.Colors;
const PackedColor = Colors.PackedColor;
const fRGBA = Colors.fRGBA;
const fARGB = Colors.fARGB;
const Color16 = Colors.Color16;
const YUV422 = Colors.YUV422;

const MipMap = @import("mipmap.zig");

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
                        const halfwords = @as([*]const u16, @ptrCast(@alignCast(&texture[8 * texel_idx])))[0..4];
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
            const texels = std.mem.bytesAsSlice([2]u8, texture[0..]);
            for (0..v_size) |y| {
                for (0..u_size) |x| {
                    const pixel_index: usize = y * u_size + x;
                    const texel_index: usize = if (twiddled) untwiddle(@intCast(x), @intCast(y), u_size, v_size) else pixel_index;
                    dest_bgra[pixel_index] = .{ texels[texel_index][0], texels[texel_index][1], 0, 255 };
                }
            }
        },
        else => std.debug.panic(termcolor.red("Unsupported pixel format {t}"), .{pixel_format}),
    }
}

pub fn decode_vq(dest_bgra: [*][4]u8, pixel_format: HollyModule.TexturePixelFormat, code_book: []const u8, indices: []const u8, u_size: u32, v_size: u32, twiddled: bool) void {
    std.debug.assert(u_size >= 8 and u_size % 8 == 0);
    std.debug.assert(v_size >= 8 and v_size % 8 == 0);
    std.debug.assert(code_book.len >= 8 * 256);
    std.debug.assert(indices.len >= u_size * v_size / 4);
    std.debug.assert(pixel_format == .ARGB1555 or pixel_format == .RGB565 or pixel_format == .ARGB4444);
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
    bump_mapping_bit: u1,
    color_clamp: u1,
    _: u4 = 0,
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

    pub const invalid = @This(){
        .index = InvalidTextureIndex,
        .palette = @bitCast(@as(u16, 0)),
        .shading = @bitCast(@as(u32, 0)),
    };
};

const Uniforms = extern struct {
    depth_min: f32,
    depth_max: f32,
    framebuffer_width: f32,
    framebuffer_height: f32,
    fpu_shad_scale: f32,
    fog_density: f32, // Should be a f16?
    pt_alpha_ref: f32,
    fog_col_pal: PackedColor,
    fog_col_vert: PackedColor,
    fog_clamp_min: PackedColor,
    fog_clamp_max: PackedColor,
    _padding: u32 = 0,
    fog_lut: [0x80]u32 align(16), // actually 2 * 8bits per entry.
};

const ModifierVolumeUniforms = extern struct {
    min_depth: f32,
    max_depth: f32,
    framebuffer_width: f32,
    framebuffer_height: f32,
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
    pub const undef = @This(){
        .x = -0.0,
        .y = -0.0,
        .z = -0.0,
        .primitive_index = 0xFFFFFFFF,
        .base_color = .{ .r = 255, .g = 0, .b = 0, .a = 255 },
    };
};

const StripMetadata = packed struct(u128) {
    area0_instructions: VertexTextureInfo,
    area1_instructions: VertexTextureInfo = .invalid,
};

const wgsl_vs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/position_clip.wgsl") ++ @embedFile("./shaders/vs.wgsl");
const wgsl_fs = "const Opaque = true;\n" ++ @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/fragment_color.wgsl") ++ @embedFile("./shaders/fs.wgsl");
const wgsl_presort_fs = "const Opaque = false;\n" ++ @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/fragment_color.wgsl") ++ @embedFile("./shaders/fs.wgsl");
const wgsl_translucent_fs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/oit_structs.wgsl") ++ @embedFile("./shaders/fragment_color.wgsl") ++ @embedFile("./shaders/oit_draw_fs.wgsl");
const wgsl_modvol_translucent_fs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/morton.wgsl") ++ @embedFile("./shaders/tmv_structs.wgsl") ++ @embedFile("./shaders/modifier_volume_translucent_fs.wgsl");
const wgsl_modvol_merge_cs = @embedFile("./shaders/morton.wgsl") ++ @embedFile("./shaders/tmv_structs.wgsl") ++ @embedFile("./shaders/modifier_volume_translucent_merge.wgsl");
const wgsl_blend_cs = @embedFile("./shaders/oit_structs.wgsl") ++ @embedFile("./shaders/morton.wgsl") ++ @embedFile("./shaders/tmv_structs.wgsl") ++ @embedFile("./shaders/oit_blend_cs.wgsl");
const wgsl_modifier_volume_vs = @embedFile("./shaders/position_clip.wgsl") ++ @embedFile("./shaders/modifier_volume_vs.wgsl");
const wgsl_modifier_volume_fs = @embedFile("./shaders/modifier_volume_fs.wgsl");
const wgsl_modifier_volume_apply_fs = @embedFile("./shaders/modifier_volume_apply_fs.wgsl");
const blit_vs = @embedFile("./shaders/blit_vs.wgsl");
const blit_fs = @embedFile("./shaders/blit_fs.wgsl");
const blit_opaque_fs = @embedFile("./shaders/blit_opaque_fs.wgsl");

const TextureStatus = enum {
    Invalid, // Has never been written to.
    Outdated, // Has been modified in VRAM, but a version of it is still in use. Kept around longer for potential re-use. Will reclaim it if necessary.
    Unused, // Wasn't used in the last frame. Will reclaim it if necessary.
    Used, // Was used in the last (or current) frame.
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

    indices: std.ArrayList(u32) = .empty, // Temporary storage before uploading them to the GPU.

    pub fn init(sampler: u8, user_clip: ?HollyModule.UserTileClipInfo) DrawCall {
        return .{
            .sampler = sampler,
            .user_clip = user_clip,
        };
    }

    pub fn deinit(self: *DrawCall, allocator: std.mem.Allocator) void {
        self.indices.deinit(allocator);
    }
};

const SortedDrawCall = struct {
    pipeline_key: PipelineKey,
    sampler: u8,
    user_clip: ?HollyModule.UserTileClipInfo,
    start_index: u32 = 0,
    index_count: u32 = 0,
};

fn translate_blend_factor(factor: HollyModule.AlphaInstruction) wgpu.BlendFactor {
    return switch (factor) {
        .Zero => .zero,
        .One => .one,
        .SourceAlpha => .src_alpha,
        .InverseSourceAlpha => .one_minus_src_alpha,
        .DestAlpha => .dst_alpha,
        .InverseDestAlpha => .one_minus_dst_alpha,
        else => std.debug.panic("Invalid blend factor: {t}", .{factor}),
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
    translucent: bool,
    src_blend_factor: wgpu.BlendFactor,
    dst_blend_factor: wgpu.BlendFactor,
    depth_compare: wgpu.CompareFunction,
    depth_write_enabled: bool,
    culling_mode: HollyModule.CullingMode,

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        try writer.print("[Translucent: {}, Blend SRC: {t}, DST: {t}, Depth Compare: {t}, Depth Write: {}, Culling Mode: {t}]", .{
            self.translucent,
            self.src_blend_factor,
            self.dst_blend_factor,
            self.depth_compare,
            self.depth_write_enabled,
            self.culling_mode,
        });
    }
};

const BackgroundPipelineKey = PipelineKey{
    .translucent = false,
    .src_blend_factor = .one,
    .dst_blend_factor = .zero,
    .depth_compare = .always,
    .depth_write_enabled = false,
    .culling_mode = .None,
};

const DrawCallKey = struct {
    sampler: u8,
    user_clip: ?HollyModule.UserTileClipInfo,
};

const PipelineMetadata = struct {
    draw_calls: std.AutoArrayHashMap(DrawCallKey, DrawCall),

    pub fn init(allocator: std.mem.Allocator) PipelineMetadata {
        return .{ .draw_calls = .init(allocator) };
    }

    fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        for (self.draw_calls.values()) |*draw_call| draw_call.deinit(allocator);
        self.draw_calls.deinit();
    }
};

const PassMetadata = struct {
    pass_type: HollyModule.ListType,
    steps: std.ArrayList(std.AutoArrayHashMap(PipelineKey, PipelineMetadata)) = .empty,

    pub fn init(pass_type: HollyModule.ListType) PassMetadata {
        return .{
            .pass_type = pass_type,
        };
    }

    fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        self.reset(allocator);
        self.steps.deinit(allocator);
    }

    pub fn reset(self: *@This(), allocator: std.mem.Allocator) void {
        for (self.steps.items) |*step| {
            for (step.values()) |*pipeline|
                pipeline.deinit(allocator);
            step.deinit();
        }
        self.steps.clearRetainingCapacity();
    }
};

const RenderPass = struct {
    z_clear: bool = true,
    pre_sort: bool = false,

    opaque_list_pointer: HollyModule.RegionArrayDataConfiguration.ListPointer = .Empty,
    opaque_modifier_volume_pointer: HollyModule.RegionArrayDataConfiguration.ListPointer = .Empty,
    translucent_list_pointer: HollyModule.RegionArrayDataConfiguration.ListPointer = .Empty,
    translucent_modifier_volume_pointer: HollyModule.RegionArrayDataConfiguration.ListPointer = .Empty,
    punchthrough_list_pointer: HollyModule.RegionArrayDataConfiguration.ListPointer = .Empty,

    opaque_pass: PassMetadata,
    punchthrough_pass: PassMetadata,
    translucent_pass: PassMetadata,
    pre_sorted_translucent_pass: std.ArrayList(SortedDrawCall) = .empty,

    pub fn init() RenderPass {
        return .{
            .opaque_pass = .init(.Opaque),
            .punchthrough_pass = .init(.PunchThrough),
            .translucent_pass = .init(.Translucent),
        };
    }

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        self.opaque_pass.deinit(allocator);
        self.punchthrough_pass.deinit(allocator);
        self.translucent_pass.deinit(allocator);
        self.pre_sorted_translucent_pass.deinit(allocator);
    }

    pub fn clearRetainingCapacity(self: *@This(), allocator: std.mem.Allocator) void {
        self.opaque_pass.reset(allocator);
        self.punchthrough_pass.reset(allocator);
        self.translucent_pass.reset(allocator);
        self.pre_sorted_translucent_pass.clearRetainingCapacity();
    }
};

fn gen_sprite_vertices(sprite: HollyModule.VertexParameter) [4]Vertex {
    var r: [4]Vertex = @splat(Vertex.undef);

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
    const dz = if (r[0].z == r[2].z and r[0].z == r[3].z) r[0].z else pe: {
        // dz has to be deduced from the plane equation
        const ab = @Vector(3, f32){
            r[2].x - r[0].x,
            r[2].y - r[0].y,
            r[2].z - r[0].z,
        };
        const ac = @Vector(3, f32){
            r[3].x - r[0].x,
            r[3].y - r[0].y,
            r[3].z - r[0].z,
        };
        const normal = @Vector(3, f32){
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        };
        const plane_equation_coeff = @Vector(4, f32){
            normal[0],
            normal[1],
            normal[2],
            -(normal[0] * r[0].x + normal[1] * r[0].y + normal[2] * r[0].z),
        };
        break :pe (-plane_equation_coeff[0] * r[1].x - plane_equation_coeff[1] * r[1].y - plane_equation_coeff[3]) / plane_equation_coeff[2];
    };
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
        else => std.debug.panic(termcolor.red("Invalid u_size for vq_compressed mip mapped texture"), .{}),
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
            log.err(termcolor.red("Invalid u_size for paletted mip mapped texture"), .{});
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
            log.err(termcolor.red("Invalid u_size for mip mapped texture"), .{});
            @panic("Invalid u_size for mip mapped texture");
        },
    };
}

const VertexAttributes = [_]wgpu.VertexAttribute{
    .{ .shader_location = 0, .format = .float32x3, .offset = @offsetOf(Vertex, "x") },
    .{ .shader_location = 1, .format = .uint32, .offset = @offsetOf(Vertex, "primitive_index") },
    .{ .shader_location = 2, .format = .uint32, .offset = @offsetOf(Vertex, "base_color") },
    .{ .shader_location = 3, .format = .uint32, .offset = @offsetOf(Vertex, "offset_color") },
    .{ .shader_location = 4, .format = .uint32, .offset = @offsetOf(Vertex, "area1_base_color") },
    .{ .shader_location = 5, .format = .uint32, .offset = @offsetOf(Vertex, "area1_offset_color") },
    .{ .shader_location = 6, .format = .float32x2, .offset = @offsetOf(Vertex, "u") },
    .{ .shader_location = 7, .format = .float32x2, .offset = @offsetOf(Vertex, "area1_u") },
};
const VertexBufferLayout = [_]wgpu.VertexBufferLayout{.{
    .array_stride = @sizeOf(Vertex),
    .attribute_count = VertexAttributes.len,
    .attributes = &VertexAttributes,
}};

const ModifierVolumeVertexAttributes = [_]wgpu.VertexAttribute{
    .{ .shader_location = 0, .format = .float32x4, .offset = 0 },
};
const ModifierVolumeVertexBufferLayout = [_]wgpu.VertexBufferLayout{.{
    .array_stride = 4 * @sizeOf(f32),
    .attribute_count = ModifierVolumeVertexAttributes.len,
    .attributes = &ModifierVolumeVertexAttributes,
}};

const BlitBindGroupLayout = [_]wgpu.BindGroupLayoutEntry{
    zgpu.textureEntry(0, .{ .fragment = true }, .float, .tvdim_2d, false),
    zgpu.samplerEntry(1, .{ .fragment = true }, .filtering),
    zgpu.bufferEntry(2, .{ .vertex = true }, .uniform, true, 0),
};

const ModifierVolumeApplyBindGroupLayout = [_]wgpu.BindGroupLayoutEntry{
    zgpu.textureEntry(0, .{ .fragment = true }, .float, .tvdim_2d, false),
    zgpu.bufferEntry(2, .{ .vertex = true }, .uniform, true, 0),
};

const TextureAndView = struct {
    texture: zgpu.TextureHandle = .nil,
    view: zgpu.TextureViewHandle = .nil,

    pub const Resources = struct { texture: wgpu.Texture, view: wgpu.TextureView };
    pub inline fn lookup(self: @This(), gctx: *zgpu.GraphicsContext) Resources {
        return .{
            .texture = gctx.lookupResource(self.texture).?,
            .view = gctx.lookupResource(self.view).?,
        };
    }
    pub fn release(self: @This(), gctx: *zgpu.GraphicsContext) void {
        gctx.releaseResource(self.view);
        gctx.destroyResource(self.texture);
    }
};

pub const Filter = enum { Nearest, Linear };

pub const Renderer = struct {
    // Initial max texture count for each size (8x8 to 1024x1024).
    // FIXME: Not sure what are good values.
    //        As a good stress test, Fatal Fury: Mark of the Wolves can use more than 1024 16x16 textures in a single frame right in the character select screen with the current system.
    //        In matches, I've also seen it exceed 2048 8x8 textures (!) and the current limit of 8 1024x1024 textures by using a lot of 16x1024. This might be the limit of the current solution.
    pub const InitialTextureSlots: [8]u16 = .{ 2048, 2048, 512, 512, 256, 128, 32, 8 };
    const StripMetadataSize = 16 * 4096 * @sizeOf(StripMetadata); // FIXME: Arbitrary size for testing

    pub const DisplayMode = enum { Center, Fit, Stretch };

    pub const Resolution = struct { width: u32, height: u32 };
    pub const NativeResolution: Resolution = .{ .width = 640, .height = 480 };

    pub const Configuration = struct {
        pub const TextureFilter = enum { @"Application Driven", @"Force Nearest", @"Force Linear" };

        internal_resolution_factor: u32 = 2,
        display_mode: DisplayMode = .Center,
        scaling_filter: Filter = .Linear,
        texture_filter: TextureFilter = .@"Application Driven",
    };

    const MaxFragmentsPerPixel = 24;
    const OITLinkedListNodeSize = 5 * 4;
    const MaxVolumeFragmentsPerPixel = 16;
    const VolumeFragmentSize = 2 * @sizeOf(u32);
    const MaxVolumesPerPixel = 8;
    const VolumePixelSize = MaxVolumesPerPixel * @sizeOf(f32) * 2; // See Volumes in tmv_structs.zig

    const OITUniforms = packed struct { max_fragments: u32, target_width: u32, start_y: u32 };
    const OITTMVUniforms = packed struct { square_size: u32, pixels_per_slice: u32, target_width: u32, start_y: u32 };
    const BlitUniforms = extern struct { min: [2]f32, max: [2]f32 };

    const FirstVertex: u32 = 4; // The 4 first vertices are reserved for the background.
    const FirstIndex: u32 = 5; // The 5 first indices are reserved for the background.

    const DepthClearValue = 0.0;
    const DepthCompareFunction: wgpu.CompareFunction = .greater;

    /// Write the framebuffer back to a texture or guest VRAM (depending on ExperimentalRenderToVRAM) after each render.
    ExperimentalFramebufferEmulation: bool = false,
    /// Allow rendering to a texture.
    ExperimentalRenderToTexture: bool = true,
    /// When rendering to a texture or framebuffer, copy the result to guest VRAM. Necessary for some effects in Grandia II or Tony Hawk 2 for example.
    ExperimentalRenderToVRAM: bool = true,
    ExperimentalClampSpritesUVs: bool = true,
    ExperimentalRenderOnEmulationThread: bool = false,

    DebugDisableTextureCacheAcrossFrames: bool = false,

    render_request: bool = false,
    on_render_start_param_base: u32 = 0,
    render_passes: std.ArrayList(RenderPass),
    ta_lists: std.ArrayList(HollyModule.TALists),

    texture_metadata: [8][]TextureMetadata = @splat(&.{}),

    framebuffer_resize_bind_group: zgpu.BindGroupHandle = .nil,

    blit_pipeline: zgpu.RenderPipelineHandle = .{},
    blit_opaque_pipeline: zgpu.RenderPipelineHandle = .{},
    blit_bind_group: zgpu.BindGroupHandle = .nil,
    blit_bind_group_render_to_texture: zgpu.BindGroupHandle = .nil,
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
    pre_sort_fragment_shader_module: wgpu.ShaderModule,

    textures_bind_group: zgpu.BindGroupHandle = .nil,
    modifier_volume_bind_group: zgpu.BindGroupHandle = .nil,
    modifier_volume_apply_bind_group: zgpu.BindGroupHandle = .nil,
    translucent_bind_group: zgpu.BindGroupHandle = .nil,
    translucent_modvol_bind_group: zgpu.BindGroupHandle = .nil,
    translucent_modvol_merge_bind_group: zgpu.BindGroupHandle = .nil,
    blend_bind_group: zgpu.BindGroupHandle = .nil,
    blend_bind_group_render_to_texture: zgpu.BindGroupHandle = .nil,

    vertex_buffer: zgpu.BufferHandle,
    index_buffer: zgpu.BufferHandle,
    modifier_volume_vertex_buffer: zgpu.BufferHandle,

    strips_metadata_buffer: zgpu.BufferHandle,

    oit_horizontal_slices: u32 = 4, // Divides the OIT pass into multiple passes to limit memory usage.
    list_heads_buffer: zgpu.BufferHandle = .nil,
    linked_list_buffer: zgpu.BufferHandle = .nil,

    translucent_modvol_fragment_counts_buffer: zgpu.BufferHandle = .nil,
    translucent_modvol_fragment_list_buffer: zgpu.BufferHandle = .nil,
    translucent_modvol_volumes_buffer: zgpu.BufferHandle = .nil,

    texture_arrays: [8]TextureAndView,
    palette_buffer: zgpu.BufferHandle,

    resolution: Resolution,
    texture_filter: Configuration.TextureFilter,

    /// Intermediate texture to upload framebuffer from VRAM at native resolution
    framebuffer: TextureAndView,
    /// Target for rendering to a texture (will be read back to guest VRAM using framebuffer_copy_buffer)
    render_to_texture_target: TextureAndView,
    resized_render_to_texture_target: TextureAndView = .{},
    /// Intermediate buffer used to read pixels back to main RAM.
    framebuffer_copy_buffer: zgpu.BufferHandle,
    /// Framebuffer at target resolution to draw on
    resized_framebuffer: TextureAndView = .{},
    resized_framebuffer_area1: TextureAndView = .{},

    // NOTE: This should not be needed, but WGPU doesn't handle reading from a storage texture yet.
    resized_framebuffer_copy: TextureAndView = .{},

    depth: struct {
        texture: zgpu.TextureHandle,
        view: zgpu.TextureViewHandle,
        depth_only_view: zgpu.TextureViewHandle,
    } = undefined,

    samplers: [256]zgpu.SamplerHandle,
    sampler_bind_groups: [256]zgpu.BindGroupHandle, // FIXME: Use a single one? (Dynamic uniform)

    min_depth: f32 = std.math.floatMax(f32),
    max_depth: f32 = 0.0,
    pt_alpha_ref: f32 = 1.0,
    fpu_shad_scale: f32 = 1.0,
    fog_col_pal: PackedColor = .{},
    fog_col_vert: PackedColor = .{},
    fog_clamp_min: PackedColor = .{},
    fog_clamp_max: PackedColor = .{},
    fog_density: f32 = 0,
    fog_lut: [0x80]u32 = @splat(0),
    output_resolution: Resolution = .{ .width = 640, .height = 480 },
    render_size: Resolution = .{ .width = 640, .height = 480 },
    global_clip: struct { x: struct { min: u16, max: u16 }, y: struct { min: u16, max: u16 } } = .{ .x = .{ .min = 0, .max = 0 }, .y = .{ .min = 0, .max = 0 } },
    fb_r_ctrl: HollyModule.FB_R_CTRL = undefined,
    write_back_parameters: HollyModule.Holly.WritebackParameters = undefined,
    spg_control: HollyModule.SPG_CONTROL = undefined,
    vo_startx: HollyModule.VO_STARTX = undefined,
    vo_starty: HollyModule.VO_STARTY = undefined,
    palette_bgra: []u32,

    // Some memory to avoid repeated allocations accross frames.
    vertices: std.ArrayList(Vertex),
    strips_metadata: std.ArrayList(StripMetadata),
    modifier_volume_vertices: std.ArrayList([4]f32),

    last_frame_timestamp: i64,
    last_n_frametimes: struct {
        pub const MaxCount = 60;

        count: usize = 0,
        position: usize = 0,
        times: [MaxCount]i64 = @splat(0),

        pub fn push(self: *@This(), time: i64) void {
            self.times[self.position] = time;
            self.position = (self.position + 1) % self.times.len;
            if (self.count < self.times.len)
                self.count += 1;
        }

        pub fn sum(self: *const @This()) i64 {
            const first = if (self.position > self.count) self.position - self.count else self.position + self.times.len - self.count;
            var s: i64 = 0;
            for (0..self.count) |i| {
                s += self.times[(first + i) % self.times.len];
            }
            return s;
        }
    } = .{},

    _scratch_pad: []u8 align(4), // Used to avoid temporary allocations before GPU uploads for example. 4 * 1024 * 1024, since this is the maximum texture size supported by the DC.

    _gctx: *zgpu.GraphicsContext,
    _gctx_queue_mutex: *std.Thread.Mutex,
    _allocator: std.mem.Allocator,

    fn create_textures_bind_group_layout(gctx: *zgpu.GraphicsContext) zgpu.BindGroupLayoutHandle {
        return gctx.createBindGroupLayout(&.{
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
        }, .{ .label = "TexturesBindGroupLayout" });
    }

    fn create_textures_bind_group(self: *@This()) void {
        const bind_group_layout = create_textures_bind_group_layout(self._gctx);
        defer self._gctx.releaseResource(bind_group_layout);

        if (self.textures_bind_group.id != 0) {
            self._gctx.releaseResource(self.textures_bind_group);
            self.textures_bind_group = .nil;
        }

        self.textures_bind_group = self._gctx.createBindGroup(bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(Uniforms) },
            .{ .binding = 1, .texture_view_handle = self.texture_arrays[0].view },
            .{ .binding = 2, .texture_view_handle = self.texture_arrays[1].view },
            .{ .binding = 3, .texture_view_handle = self.texture_arrays[2].view },
            .{ .binding = 4, .texture_view_handle = self.texture_arrays[3].view },
            .{ .binding = 5, .texture_view_handle = self.texture_arrays[4].view },
            .{ .binding = 6, .texture_view_handle = self.texture_arrays[5].view },
            .{ .binding = 7, .texture_view_handle = self.texture_arrays[6].view },
            .{ .binding = 8, .texture_view_handle = self.texture_arrays[7].view },
            .{ .binding = 9, .buffer_handle = self.strips_metadata_buffer, .offset = 0, .size = StripMetadataSize },
            .{ .binding = 10, .buffer_handle = self.palette_buffer, .offset = 0, .size = 4 * 1024 },
        });
    }

    pub fn create(allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext, gctx_queue_mutex: *std.Thread.Mutex, config: Configuration) !*Renderer {
        const start = std.time.milliTimestamp();
        defer log.info("Renderer initialized in {d}ms", .{std.time.milliTimestamp() - start});

        // Writes to texture all rely on that.
        std.debug.assert(zgpu.GraphicsContext.surface_texture_format == .bgra8_unorm);

        const framebuffer_texture = gctx.createTexture(.{
            .usage = .{ .render_attachment = true, .texture_binding = true, .copy_dst = true, .copy_src = true },
            .size = .{
                .width = NativeResolution.width,
                .height = NativeResolution.height,
                .depth_or_array_layers = 1,
            },
            .format = zgpu.GraphicsContext.surface_texture_format,
            .mip_level_count = 1,
            .label = .init("Framebuffer Texture"),
        });
        const framebuffer_texture_view = gctx.createTextureView(framebuffer_texture, .{});

        const render_to_texture_target = gctx.createTexture(.{
            .usage = .{ .render_attachment = true, .texture_binding = true, .copy_dst = true, .copy_src = true },
            .size = .{
                .width = NativeResolution.width,
                .height = NativeResolution.height,
                .depth_or_array_layers = 1,
            },
            .format = zgpu.GraphicsContext.surface_texture_format,
            .mip_level_count = 1,
            .label = .init("Render to Texture Target"),
        });

        const framebuffer_copy_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .map_read = true },
            .size = 4 * NativeResolution.width * NativeResolution.height,
            .label = .init("Framebuffer Copy Buffer"),
        });

        const textures_bind_group_layout = create_textures_bind_group_layout(gctx);
        defer gctx.releaseResource(textures_bind_group_layout);
        const sampler_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.samplerEntry(0, .{ .fragment = true }, .filtering),
        }, .{ .label = "Sampler Bind Group Layout" });
        defer gctx.releaseResource(sampler_bind_group_layout);

        const blit_bind_group_layout = gctx.createBindGroupLayout(&BlitBindGroupLayout, .{ .label = "Blit Bind Group Layout" });
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

        var texture_arrays: [8]TextureAndView = undefined;
        for (0..8) |i|
            texture_arrays[i] = create_texture_cache_array(gctx, i, InitialTextureSlots[i]);

        const palette_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .storage = true },
            .size = 4 * 1024,
        });

        const strips_metadata_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .storage = true },
            .size = StripMetadataSize,
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
        }, .{ .label = "TranslucentBindGroupLayout" });

        const translucent_modvol_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .fragment = true }, .uniform, true, 0),
            zgpu.bufferEntry(1, .{ .fragment = true }, .storage, false, 0),
            zgpu.bufferEntry(2, .{ .fragment = true }, .storage, false, 0),
            zgpu.bufferEntry(3, .{ .fragment = true }, .uniform, true, 0),
            zgpu.textureEntry(4, .{ .fragment = true }, .depth, .tvdim_2d, false),
        }, .{ .label = "TranslucentModVolBindGroupLayout" });

        const modifier_volume_vertex_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modifier_volume_vs, "modvol_vs");
        defer modifier_volume_vertex_shader_module.release();
        const modifier_volume_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .vertex = true }, .uniform, true, 0),
        }, .{ .label = "ModifierVolumeBindGroupLayout" });
        defer gctx.releaseResource(modifier_volume_group_layout);

        const translucent_modvol_merge_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .compute = true }, .uniform, true, 0),
            zgpu.bufferEntry(1, .{ .compute = true }, .storage, false, 0),
            zgpu.bufferEntry(2, .{ .compute = true }, .storage, false, 0),
            zgpu.bufferEntry(3, .{ .compute = true }, .storage, false, 0),
        }, .{ .label = "TranslucentModVolMergeBindGroupLayout" });

        const color_targets = [_]wgpu.ColorTargetState{.{
            .format = zgpu.GraphicsContext.surface_texture_format,
            .write_mask = .{}, // We won't write to the color attachment
        }};

        const translucent_pipeline_layout = gctx.createPipelineLayout(&.{
            textures_bind_group_layout,
            sampler_bind_group_layout,
            translucent_bind_group_layout,
        });
        defer gctx.releaseResource(translucent_pipeline_layout);
        const translucent_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = .{
                .module = opaque_vertex_shader_module,
                .entry_point = .init("main"),
                .buffer_count = VertexBufferLayout.len,
                .buffers = &VertexBufferLayout,
            },
            .primitive = .{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_strip,
                .strip_index_format = .uint32,
            },
            .depth_stencil = null, // FIXME: Use opaque depth here rather than sampling it manually in the shader?
            .fragment = &.{
                .module = translucent_fragment_shader_module,
                .entry_point = .init("main"),
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
        }, .{ .label = "BlendBindGroupLayout" });

        const blend_pipeline_layout = gctx.createPipelineLayout(&.{blend_bind_group_layout});
        defer gctx.releaseResource(blend_pipeline_layout);
        const blend_pipeline_descriptor = wgpu.ComputePipelineDescriptor{
            .compute = .{
                .module = blend_compute_module,
                .entry_point = .init("main"),
            },
        };

        // Modifier Volumes
        // Implemented using a stencil buffer and the shadow volume algorithm.
        // This first pipeline takes the previous depth buffer and the modifier volume to generate the stencil buffer.

        const modifier_volume_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modifier_volume_fs, "fs");
        defer modifier_volume_fragment_shader_module.release();

        const modifier_volume_pipeline_layout = gctx.createPipelineLayout(&.{modifier_volume_group_layout});

        const closed_modifier_volume_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = .{
                .module = modifier_volume_vertex_shader_module,
                .entry_point = .init("main"),
                .buffer_count = ModifierVolumeVertexBufferLayout.len,
                .buffers = &ModifierVolumeVertexBufferLayout,
            },
            .primitive = .{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_list,
            },
            .depth_stencil = &.{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = .false,
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
            .fragment = &.{
                .module = modifier_volume_fragment_shader_module,
                .entry_point = .init("main"),
                .target_count = 0,
                .targets = null,
            },
        };

        const shift_stencil_buffer_modifier_volume_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = .{
                .module = modifier_volume_vertex_shader_module,
                .entry_point = .init("main"),
                .buffer_count = ModifierVolumeVertexBufferLayout.len,
                .buffers = &ModifierVolumeVertexBufferLayout,
            },
            .primitive = .{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_list,
            },
            .depth_stencil = &.{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = .false,
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
            .fragment = &.{
                .module = modifier_volume_fragment_shader_module,
                .entry_point = .init("main"),
                .target_count = 0,
                .targets = null,
            },
        };

        const open_modifier_volume_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = .{
                .module = modifier_volume_vertex_shader_module,
                .entry_point = .init("main"),
                .buffer_count = ModifierVolumeVertexBufferLayout.len,
                .buffers = &ModifierVolumeVertexBufferLayout,
            },
            .primitive = .{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_list,
            },
            .depth_stencil = &.{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = .false,
                .depth_compare = DepthCompareFunction,
                .stencil_read_mask = 0x02,
                .stencil_write_mask = 0x03,
                .stencil_front = .{
                    .compare = .not_equal, // Only run if not already marked as area 1.
                    .fail_op = .keep, // Action performed on samples that fail the stencil test
                    .pass_op = .replace, // Action performed on samples that pass both the depth and stencil tests.
                    .depth_fail_op = .keep, // Action performed on samples that pass the stencil test and fail the depth test.
                },
                .stencil_back = .{ // Same as front, I don't think the orientation matters for the hardware and games are not required to properly distinguish between front facing and back facing triangles.
                    .compare = .not_equal,
                    .fail_op = .keep,
                    .pass_op = .replace,
                    .depth_fail_op = .keep,
                },
            },
            .fragment = &.{
                .module = modifier_volume_fragment_shader_module,
                .entry_point = .init("main"),
                .target_count = 0,
                .targets = null,
            },
        };

        const modifier_volume_bind_group = gctx.createBindGroup(modifier_volume_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(ModifierVolumeUniforms) },
        });

        var renderer = try allocator.create(Renderer);
        renderer.* = .{
            .resolution = .{ .width = config.internal_resolution_factor * NativeResolution.width, .height = config.internal_resolution_factor * NativeResolution.height },
            .texture_filter = config.texture_filter,

            .blit_vertex_buffer = blit_vertex_buffer,
            .blit_index_buffer = blit_index_buffer,
            .blit_to_window_vertex_buffer = blit_to_window_vertex_buffer,

            .framebuffer = .{ .texture = framebuffer_texture, .view = framebuffer_texture_view },
            .render_to_texture_target = .{ .texture = render_to_texture_target, .view = gctx.createTextureView(render_to_texture_target, .{}) },
            .framebuffer_copy_buffer = framebuffer_copy_buffer,

            .opaque_pipelines = .init(allocator),

            .translucent_bind_group_layout = translucent_bind_group_layout,
            .translucent_modvol_bind_group_layout = translucent_modvol_bind_group_layout,
            .translucent_modvol_merge_bind_group_layout = translucent_modvol_merge_bind_group_layout,

            .blend_bind_group_layout = blend_bind_group_layout,

            .opaque_pipeline_layout = gctx.createPipelineLayout(&.{ textures_bind_group_layout, sampler_bind_group_layout }),
            .opaque_vertex_shader_module = opaque_vertex_shader_module,
            .opaque_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "fs"),
            .pre_sort_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_presort_fs, "fs"),

            .modifier_volume_bind_group = modifier_volume_bind_group,
            .vertex_buffer = vertex_buffer,
            .index_buffer = index_buffer,
            .strips_metadata_buffer = strips_metadata_buffer,
            .modifier_volume_vertex_buffer = modifier_volume_vertex_buffer,

            .texture_arrays = texture_arrays,
            .palette_buffer = palette_buffer,

            .samplers = samplers,
            .sampler_bind_groups = sampler_bind_groups,

            .vertices = try .initCapacity(allocator, 4096),
            .strips_metadata = try .initCapacity(allocator, 4096),
            .modifier_volume_vertices = try .initCapacity(allocator, 4096),

            .last_frame_timestamp = std.time.microTimestamp(),

            .render_passes = .empty,
            .ta_lists = .empty,

            .palette_bgra = try allocator.alloc(u32, 1024),
            ._scratch_pad = try allocator.allocWithOptions(u8, 4 * 1024 * 1024, .@"4", null),

            ._gctx = gctx,
            ._gctx_queue_mutex = gctx_queue_mutex,
            ._allocator = allocator,
        };
        try renderer.ta_lists.append(allocator, .init());

        renderer.create_textures_bind_group();

        MipMap.init(allocator, gctx);
        // Blit pipeline
        {
            const blit_fs_module = zgpu.createWgslShaderModule(gctx.device, blit_fs, "blit_fs");
            defer blit_fs_module.release();
            const blit_pipeline_layout = gctx.createPipelineLayout(&.{blit_bind_group_layout});
            defer gctx.releaseResource(blit_pipeline_layout);

            const blit_color_targets = [_]wgpu.ColorTargetState{.{
                .format = zgpu.GraphicsContext.surface_texture_format,
            }};

            const pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = .{
                    .module = blit_vs_module,
                    .entry_point = .init("main"),
                    .buffer_count = blit_vertex_buffers.len,
                    .buffers = &blit_vertex_buffers,
                },
                .primitive = .{
                    .front_face = .ccw,
                    .cull_mode = .none,
                    .topology = .triangle_strip,
                    .strip_index_format = .uint32,
                },
                .depth_stencil = null,
                .fragment = &.{
                    .module = blit_fs_module,
                    .entry_point = .init("main"),
                    .target_count = blit_color_targets.len,
                    .targets = &blit_color_targets,
                },
            };
            _ = try gctx.createRenderPipelineAsync(allocator, blit_pipeline_layout, pipeline_descriptor, &renderer.blit_pipeline);

            const blit_opaque_fs_module = zgpu.createWgslShaderModule(gctx.device, blit_opaque_fs, "blit_opaque_fs");
            defer blit_opaque_fs_module.release();
            const opaque_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = .{
                    .module = blit_vs_module,
                    .entry_point = .init("main"),
                    .buffer_count = blit_vertex_buffers.len,
                    .buffers = &blit_vertex_buffers,
                },
                .primitive = .{
                    .front_face = .ccw,
                    .cull_mode = .none,
                    .topology = .triangle_strip,
                    .strip_index_format = .uint32,
                },
                .depth_stencil = null,
                .fragment = &.{
                    .module = blit_opaque_fs_module,
                    .entry_point = .init("main"),
                    .target_count = blit_color_targets.len,
                    .targets = &blit_color_targets,
                },
            };
            _ = try gctx.createRenderPipelineAsync(allocator, blit_pipeline_layout, opaque_pipeline_descriptor, &renderer.blit_opaque_pipeline);
        }

        _ = try gctx.createComputePipelineAsync(allocator, blend_pipeline_layout, blend_pipeline_descriptor, &renderer.blend_pipeline);
        _ = try gctx.createRenderPipelineAsync(allocator, translucent_pipeline_layout, translucent_pipeline_descriptor, &renderer.translucent_pipeline);

        _ = try gctx.createRenderPipelineAsync(allocator, modifier_volume_pipeline_layout, closed_modifier_volume_pipeline_descriptor, &renderer.closed_modifier_volume_pipeline);
        _ = try gctx.createRenderPipelineAsync(allocator, modifier_volume_pipeline_layout, shift_stencil_buffer_modifier_volume_pipeline_descriptor, &renderer.shift_stencil_buffer_modifier_volume_pipeline);
        _ = try gctx.createRenderPipelineAsync(allocator, modifier_volume_pipeline_layout, open_modifier_volume_pipeline_descriptor, &renderer.open_modifier_volume_pipeline);

        // Modifier Volume Apply pipeline - Use the stencil from the previous pass to apply modifier volume effects.
        {
            const mv_apply_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modifier_volume_apply_fs, "fs");
            defer mv_apply_fragment_shader_module.release();

            const mv_apply_bind_group_layout = gctx.createBindGroupLayout(&ModifierVolumeApplyBindGroupLayout, .{ .label = "ModifierVolumeApplyBindGroupLayout" });
            const mv_apply_pipeline_layout = gctx.createPipelineLayout(&.{mv_apply_bind_group_layout});

            const mv_apply_color_targets = [_]wgpu.ColorTargetState{.{
                .format = zgpu.GraphicsContext.surface_texture_format,
            }};

            const mv_apply_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = .{
                    .module = blit_vs_module,
                    .entry_point = .init("main"),
                    .buffer_count = blit_vertex_buffers.len,
                    .buffers = &blit_vertex_buffers,
                },
                .primitive = .{
                    .front_face = .ccw,
                    .cull_mode = .none,
                    .topology = .triangle_strip,
                    .strip_index_format = .uint32,
                },
                .depth_stencil = &.{
                    .format = .depth32_float_stencil8,
                    .depth_write_enabled = .false,
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
                .fragment = &.{
                    .module = mv_apply_fragment_shader_module,
                    .entry_point = .init("main"),
                    .target_count = mv_apply_color_targets.len,
                    .targets = &mv_apply_color_targets,
                },
            };
            _ = try gctx.createRenderPipelineAsync(allocator, mv_apply_pipeline_layout, mv_apply_pipeline_descriptor, &renderer.modifier_volume_apply_pipeline);
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
                .vertex = .{
                    .module = modifier_volume_vertex_shader_module,
                    .entry_point = .init("main"),
                    .buffer_count = ModifierVolumeVertexBufferLayout.len,
                    .buffers = &ModifierVolumeVertexBufferLayout,
                },
                .primitive = .{
                    .front_face = .ccw,
                    .cull_mode = .none,
                    .topology = .triangle_list,
                },
                .depth_stencil = &.{
                    .format = .depth32_float_stencil8,
                    .depth_write_enabled = .false,
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
                .fragment = &.{
                    .module = translucent_modvol_fs_module,
                    .entry_point = .init("main"),
                    .target_count = 0,
                    .targets = null,
                },
            };
            _ = try gctx.createRenderPipelineAsync(allocator, translucent_modvol_pipeline_layout, translucent_modvol_pipeline_descriptor, &renderer.translucent_modvol_pipeline);
        }
        // Translucent modifier volume merge pipeline
        {
            const translucent_modvol_merge_pipeline_layout = gctx.createPipelineLayout(&.{translucent_modvol_merge_bind_group_layout});
            const translucent_modvol_merge_compute_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modvol_merge_cs, null);
            defer translucent_modvol_merge_compute_module.release();
            const translucent_modvol_merge_pipeline_descriptor = wgpu.ComputePipelineDescriptor{
                .compute = .{
                    .module = translucent_modvol_merge_compute_module,
                    .entry_point = .init("main"),
                },
            };
            _ = try gctx.createComputePipelineAsync(allocator, translucent_modvol_merge_pipeline_layout, translucent_modvol_merge_pipeline_descriptor, &renderer.translucent_modvol_merge_pipeline);
        }

        // Ensure capacity for pipelines: Async creation needs pointer stability.
        try renderer.opaque_pipelines.ensureTotalCapacity(4 * 2 * std.meta.fields(wgpu.BlendFactor).len * std.meta.fields(wgpu.BlendFactor).len * std.meta.fields(wgpu.CompareFunction).len * 2);

        // Asynchronously create some common pipelines ahead of time
        _ = renderer.get_or_put_pipeline(BackgroundPipelineKey, .Async);
        _ = renderer.get_or_put_pipeline(.{ .translucent = false, .src_blend_factor = .one, .dst_blend_factor = .zero, .depth_compare = .greater_equal, .depth_write_enabled = true, .culling_mode = .Small }, .Async);
        _ = renderer.get_or_put_pipeline(.{ .translucent = false, .src_blend_factor = .src_alpha, .dst_blend_factor = .one_minus_src_alpha, .depth_compare = .greater_equal, .depth_write_enabled = true, .culling_mode = .Small }, .Async); // Punchthrough

        for (0..renderer.texture_metadata.len) |i| {
            renderer.texture_metadata[i] = try allocator.alloc(TextureMetadata, InitialTextureSlots[i]);
            for (renderer.texture_metadata[i]) |*tm| tm.* = .{};
        }

        renderer.on_inner_resolution_change(config.scaling_filter);

        return renderer;
    }

    pub fn destroy(self: *@This()) void {
        for (self.ta_lists.items) |*list| list.deinit(self._allocator);
        self.ta_lists.deinit(self._allocator);
        for (self.render_passes.items) |*pass| pass.deinit(self._allocator);
        self.render_passes.deinit(self._allocator);

        // Wait for async pipeline creation to finish (prevents crashing on exit).
        var async_pipeline_creation = true;
        while (async_pipeline_creation) {
            async_pipeline_creation = false;
            var it = self.opaque_pipelines.iterator();
            while (it.next()) |pipeline| {
                if (self._gctx.lookupResource(pipeline.value_ptr.*) == null) {
                    async_pipeline_creation = true;
                }
            }
        }

        self.deinit_screen_textures();

        self._allocator.free(self._scratch_pad);
        self._allocator.free(self.palette_bgra);

        self.modifier_volume_vertices.deinit(self._allocator);
        self.strips_metadata.deinit(self._allocator);
        self.vertices.deinit(self._allocator);

        for (self.sampler_bind_groups) |sampler_bind_group| {
            self._gctx.releaseResource(sampler_bind_group);
        }
        for (self.samplers) |sampler| {
            self._gctx.releaseResource(sampler);
        }

        for (self.texture_arrays) |tv| {
            tv.release(self._gctx);
        }

        self._gctx.releaseResource(self.modifier_volume_vertex_buffer);
        self._gctx.releaseResource(self.strips_metadata_buffer);
        self._gctx.releaseResource(self.index_buffer);
        self._gctx.releaseResource(self.vertex_buffer);

        self.opaque_vertex_shader_module.release();
        self.opaque_fragment_shader_module.release();
        self.pre_sort_fragment_shader_module.release();
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

        self.framebuffer.release(self._gctx);
        self.render_to_texture_target.release(self._gctx);
        self._gctx.releaseResource(self.framebuffer_copy_buffer);

        self._gctx.releaseResource(self.framebuffer_resize_bind_group);

        self._gctx.releaseResource(self.blit_to_window_vertex_buffer);
        self._gctx.releaseResource(self.blit_index_buffer);
        self._gctx.releaseResource(self.blit_vertex_buffer);
        // self._gctx.releaseResource(self.blit_pipeline);

        for (self.texture_metadata) |arr| self._allocator.free(arr);

        self._allocator.destroy(self);
    }

    pub fn reset(self: *@This()) void {
        self.render_request = false;
        self.last_frame_timestamp = std.time.microTimestamp();
        self.last_n_frametimes = .{};
        for (self.texture_metadata) |arr| {
            for (arr) |*tex| tex.* = .{};
        }
        for (self.ta_lists.items) |*list| list.clearRetainingCapacity();
        for (self.render_passes.items) |*pass| pass.clearRetainingCapacity(self._allocator);

        self.modifier_volume_vertices.clearRetainingCapacity();
        self.strips_metadata.clearRetainingCapacity();
        self.vertices.clearRetainingCapacity();
    }

    pub fn on_render_start(self: *@This(), dc: *Dreamcast) void {
        const render_to_texture = dc.gpu.render_to_texture();
        if (render_to_texture and !self.ExperimentalRenderToTexture) {
            log.warn("Render to Texture is disabled.", .{});
            return;
        }

        // NOTE: Here we also rely on this lock to protect access to `ta_lists` and `render_passes`.
        self._gctx_queue_mutex.lock();
        defer self._gctx_queue_mutex.unlock();

        if (self.render_request and !render_to_texture)
            log.warn(termcolor.yellow("Woops! Skipped a frame."), .{});
        if (self.render_request and render_to_texture) {
            log.warn(termcolor.yellow("Render to a texture with a pending render request. Force enabling Render on Emulation Thread."), .{});
            // NOTE: This has a greater chance of happening with the frame limiter enabled. Render requests can easily be delayed by a frame.
            // We cannot simply wait on the main thread here, as we could end up in a deadlock (On exit for example: main thread joining on the emulation thread, while the emulation thread is waiting on the main thread to finish rendering).
            // So for now we'll guarantee order of operations by rendering everything in the emulation thread. FIXME: If exiting is the only problematic case, and we're careful, waiting might be a better option.
            self.render_request = false; // Cancel the render request, it will be invalid after this render-to-texture anyway.
            self.ExperimentalRenderOnEmulationThread = true;
        }

        self.on_render_start_param_base = dc.gpu.read_register(u32, .PARAM_BASE);
        self.spg_control = dc.gpu.read_register(HollyModule.SPG_CONTROL, .SPG_CONTROL);
        self.vo_startx = dc.gpu.read_register(HollyModule.VO_STARTX, .VO_STARTX);
        self.vo_starty = dc.gpu.read_register(HollyModule.VO_STARTY, .VO_STARTY);

        // Clear the previous used TA lists and swap it with the one submitted by the game.
        // NOTE: Clearing the lists here means the game cannot render lists more than once (i.e. starting a render without
        //       writing to LIST_INIT). No idea if there are games that actually do that, but just in case, emit a warning.
        for (self.ta_lists.items) |*list| list.clearRetainingCapacity();
        const list_idx: u4 = @truncate(self.on_render_start_param_base >> 20);
        std.mem.swap(std.ArrayList(HollyModule.TALists), &dc.gpu._ta_lists[list_idx], &self.ta_lists);
        if (self.ta_lists.items[0].opaque_list.vertex_strips.items.len == 0 and self.ta_lists.items[0].punchthrough_list.vertex_strips.items.len == 0 and self.ta_lists.items[0].translucent_list.vertex_strips.items.len == 0) {
            log.warn(termcolor.yellow("on_render_start: Empty TA lists submitted. Is the game trying to reuse the previous TA lists?"), .{});
        }

        const header_type = dc.gpu.get_region_header_type();

        var region_count: u32 = 0;
        for (0..self.ta_lists.items.len) |region_array_idx| {
            var region_config = dc.gpu.get_region_array_data_config(region_array_idx);
            if (region_config.empty()) break;

            switch (header_type) {
                .Type1 => log.debug("[{t}] ({d}) {f}", .{ header_type, region_array_idx, region_config }),
                .Type2 => log.debug("[{t}] ({d}) {f}", .{ header_type, region_array_idx, std.fmt.alt(region_config, .formatType1) }),
            }

            if (self.render_passes.items.len <= region_array_idx) self.render_passes.append(self._allocator, .init()) catch @panic("Out of memory");

            self.render_passes.items[region_array_idx].z_clear = region_config.settings.z_clear == .Clear;
            self.render_passes.items[region_array_idx].pre_sort = region_config.settings.pre_sort;
            self.render_passes.items[region_array_idx].opaque_list_pointer = region_config.opaque_list_pointer;
            self.render_passes.items[region_array_idx].opaque_modifier_volume_pointer = region_config.opaque_modifier_volume_pointer;
            self.render_passes.items[region_array_idx].translucent_list_pointer = region_config.translucent_list_pointer;
            self.render_passes.items[region_array_idx].translucent_modifier_volume_pointer = region_config.translucent_modifier_volume_pointer;
            self.render_passes.items[region_array_idx].punchthrough_list_pointer = region_config.punch_through_list_pointer;
            region_count += 1;

            if (region_config.settings.last_region) break;
        }

        if (region_count != self.ta_lists.items.len)
            log.warn(termcolor.yellow("Expected {d} regions, found {d}"), .{ self.ta_lists.items.len, region_count });

        if (region_count < self.render_passes.items.len) {
            for (region_count..self.render_passes.items.len) |i|
                self.render_passes.items[i].deinit(self._allocator);
            self.render_passes.shrinkRetainingCapacity(region_count);
        }

        self.update_registers(&dc.gpu);
        self.update_palette(&dc.gpu);

        // Process and render immediately when rendering to a texture. Decouples it from the host refresh rate, and some games require the result to be visible in guest VRAM ASAP.
        // Othersize, let the main thread process the list asynchronously unless explicitly disabled.
        self.render_request = !self.ExperimentalRenderOnEmulationThread and !render_to_texture;

        if (!self.render_request) {
            self.update(&dc.gpu) catch |err| return log.err(termcolor.red("Failed to update renderer: {t}"), .{err});
            self.render(&dc.gpu, render_to_texture) catch |err| return log.err(termcolor.red("Failed to render: {t}"), .{err});
        }
    }

    // FIXME: This is way too slow.
    fn texture_hash(gpu: *const HollyModule.Holly, start: u32, end: u32) u64 {
        return std.hash.CityHash64.hash(gpu.vram[start & 0xFFFFFFC .. end & 0xFFFFFFC]);
    }

    /// For external use only (e.g. Debug UI)
    pub fn get_texture_view(self: *const @This(), control_word: HollyModule.TextureControlWord, tsp_instruction: HollyModule.TSPInstructionWord) ?struct { size_index: u3, index: u32 } {
        const size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);
        for (self.texture_metadata[size_index], 0..) |*entry, idx| {
            if (entry.status != .Invalid and entry.status != .Outdated and entry.match(control_word)) {
                return .{ .size_index = size_index, .index = @intCast(idx) };
            }
        }
        return null;
    }

    /// For internal use only (tracks texture usage in addition to returning the texture index)
    fn get_texture_index(self: *@This(), gpu: *const HollyModule.Holly, size_index: u3, control_word: HollyModule.TextureControlWord) ?TextureIndex {
        for (self.texture_metadata[size_index], 0..) |*entry, idx| {
            if (entry.status != .Invalid and entry.match(control_word)) {
                // This texture has not been encountered yet this frame. Check if it is still valid.
                if (entry.usage == 0) {
                    // Texture appears to have changed in memory. Mark as outdated.
                    if (texture_hash(gpu, entry.start_address, entry.end_address) != entry.hash) {
                        if (entry.status != .Outdated) {
                            entry.status = .Outdated;
                            entry.age = 0;
                        }
                        entry.usage = 0xFFFFFFFF; // Do not check it again.
                        continue; // Not valid anymore, ignore it.
                    }
                    entry.status = .Used;
                    entry.age = 0;
                }
                if (entry.status != .Used) continue;
                entry.usage +|= 1;
                return @intCast(idx);
            }
        }
        return null;
    }

    /// Search for the least recently used texture, preferring an outdated one.
    fn get_lru_texture_index(self: *@This(), size_index: u3) TextureIndex {
        var texture_index: TextureIndex = InvalidTextureIndex;
        for (self.texture_metadata[size_index], 0..) |*entry, idx| {
            if (texture_index == InvalidTextureIndex) {
                if (entry.status == .Outdated or entry.status == .Unused) texture_index = @intCast(idx);
            } else {
                const current = self.texture_metadata[size_index][texture_index];
                if (entry.status == .Outdated and (current.status == .Unused or (current.status == .Outdated and current.age < entry.age))) {
                    texture_index = @intCast(idx);
                } else if (entry.status == .Unused and current.age < entry.age) {
                    texture_index = @intCast(idx);
                }
            }
        }
        return texture_index;
    }

    inline fn bgra_scratch_pad(self: *@This()) [*][4]u8 {
        return @as([*][4]u8, @ptrCast(self._scratch_pad.ptr));
    }

    /// Assumes _gctx_queue_mutex is locked.
    fn upload_texture(self: *@This(), gpu: *const HollyModule.Holly, tsp_instruction: HollyModule.TSPInstructionWord, texture_control_word: HollyModule.TextureControlWord) TextureIndex {
        log.debug("[Upload] tsp_instruction: {any}", .{tsp_instruction});
        log.debug("[Upload] texture_control_word: {any}", .{texture_control_word});

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
            log.err(termcolor.red("Texture U size ({d}) is greater than the allocated size ({d})\n") ++ termcolor.grey("  TEXT_CONTROL:    {any}\n  TSP Instruction: {any}\n  Texture Control: {any})"), .{ u_size, alloc_u_size, texture_control_register, tsp_instruction, texture_control_word });
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
            decode_vq(self.bgra_scratch_pad(), texture_control_word.pixel_format, gpu.vram[addr..], gpu.vram[vq_index_addr..], u_size, v_size, twiddled);
        } else {
            decode_tex(self.bgra_scratch_pad(), texture_control_word.pixel_format, gpu.vram[addr..], u_size, v_size, twiddled);
        }

        // Search for an available texture index.
        var texture_index: TextureIndex = InvalidTextureIndex;
        for (0..self.texture_metadata[size_index].len) |i| {
            // Prefer slots that have never been used to keep textures in the cache for as long as possible.
            if (self.texture_metadata[size_index][i].status == .Invalid) {
                texture_index = @as(TextureIndex, @intCast(i));
                break;
            }
        }
        if (texture_index == InvalidTextureIndex)
            texture_index = self.get_lru_texture_index(size_index);

        if (texture_index == InvalidTextureIndex) {
            const slot_count = self.texture_metadata[size_index].len;
            if (slot_count < 2048) { // 2048 is the hard limit requested on context creation.
                const new_count = 2 * slot_count;
                log.warn(termcolor.yellow("Out of textures slot (size index: {d}, {d}x{d}). Increasing from {d} to {d}."), .{ size_index, u_size, v_size, slot_count, new_count });
                self.increase_texture_slot_count(size_index, new_count);
                texture_index = @intCast(slot_count);
            } else {
                log.err(termcolor.red("Out of textures slot (size index: {d}, {d}x{d}, slot count: {d})."), .{ size_index, u_size, v_size, slot_count });
                return 0;
            }
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
            .usage = 1, // Used this frame.
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
        if (copies > 1) {
            const clamp = tsp_instruction.clamp_uv;
            const flip = tsp_instruction.final_flip_uv();
            if ((clamp.u or flip.u) and !repeat_vertically) {
                tex_source[1] = self._scratch_pad[4 * u_size * v_size .. 2 * 4 * u_size * v_size];
                for (0..v_size) |v| {
                    for (0..u_size) |u| {
                        const src = tex_source[0][4 * (u_size * v + (u_size - u - 1)) ..];
                        const dst = tex_source[1][4 * (u_size * v + u) ..];
                        @memcpy(dst[0..4], src[0..4]);
                    }
                }
            } else if ((clamp.v or flip.v) and repeat_vertically) {
                tex_source[1] = self._scratch_pad[4 * u_size * v_size .. 2 * 4 * u_size * v_size];
                for (0..v_size) |v| {
                    @memcpy(tex_source[1][4 * u_size * v ..][0 .. 4 * u_size], tex_source[0][4 * u_size * (v_size - v - 1) ..][0 .. 4 * u_size]);
                }
            }
        }

        for (0..copies) |part| {
            self._gctx.queue.writeTexture(
                .{
                    .texture = self._gctx.lookupResource(self.texture_arrays[size_index].texture).?,
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
            MipMap.generate_mipmaps(self._gctx, self.texture_arrays[size_index].texture, texture_index);

        return texture_index;
    }

    fn check_texture_usage(self: *@This()) void {
        for (self.texture_metadata) |arr| {
            for (arr) |*tm| {
                switch (tm.status) {
                    .Used => {
                        if (tm.usage == 0) {
                            tm.status = .Unused;
                            tm.age = 0;
                        }
                    },
                    .Unused => tm.age +|= 1,
                    .Outdated => tm.age +|= 1,
                    .Invalid => {},
                }
                tm.usage = 0;
            }
        }
    }

    /// Uploads framebuffer from guest VRAM to host texture.
    /// Locks gctx_queue_mutex.
    pub fn update_framebuffer_texture(self: *@This(), holly: *const HollyModule.Holly) void {
        const SPG_CONTROL = self.spg_control;
        const FB_R_CTRL = holly.read_register(HollyModule.FB_R_CTRL, .FB_R_CTRL);
        const FB_R_SOF1 = holly.read_register(u32, .FB_R_SOF1);
        const FB_R_SOF2 = holly.read_register(u32, .FB_R_SOF2);
        const FB_R_SIZE = holly.read_register(HollyModule.FB_R_SIZE, .FB_R_SIZE);

        const line_size: u32 = 4 * (@as(u32, FB_R_SIZE.x_size) + 1); // From 32-bit units to bytes.
        const field_size: u32 = @as(u32, FB_R_SIZE.y_size) + 1; // Number of lines

        const bytes_per_pixels: u32 = switch (FB_R_CTRL.format) {
            .RGB555, .RGB565 => 2,
            .RGB888 => 3,
            .RGB0888_32bit => 4,
        };

        const interlaced = SPG_CONTROL.interlace;
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
                const pixel_addr: u32 = @intCast(addr + bytes_per_pixels * x);
                switch (FB_R_CTRL.format) {
                    .RGB555 => { // 0555 RGB 16 bit
                        const pixel = holly.read_vram(Color16, pixel_addr);
                        self._scratch_pad[pixel_idx * 4 + 0] = (@as(u8, pixel.argb1555.b) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 1] = (@as(u8, pixel.argb1555.g) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 2] = (@as(u8, pixel.argb1555.r) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 3] = 255;
                    },
                    .RGB565 => { // 565 RGB
                        const pixel = holly.read_vram(Color16, pixel_addr);
                        self._scratch_pad[pixel_idx * 4 + 0] = (@as(u8, pixel.rgb565.b) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 1] = (@as(u8, pixel.rgb565.g) << 2) | (FB_R_CTRL.concat & 0b11);
                        self._scratch_pad[pixel_idx * 4 + 2] = (@as(u8, pixel.rgb565.r) << 3) | FB_R_CTRL.concat;
                        self._scratch_pad[pixel_idx * 4 + 3] = 255;
                    },
                    .RGB888 => { // 888 RGB 24 bit packed
                        self._scratch_pad[pixel_idx * 4 + 0] = holly.read_vram(u8, pixel_addr + 2);
                        self._scratch_pad[pixel_idx * 4 + 1] = holly.read_vram(u8, pixel_addr + 1);
                        self._scratch_pad[pixel_idx * 4 + 2] = holly.read_vram(u8, pixel_addr + 0);
                        self._scratch_pad[pixel_idx * 4 + 3] = 255;
                    },
                    .RGB0888_32bit => { // 0888 RGB 32 bit
                        self._scratch_pad[pixel_idx * 4 + 0] = holly.read_vram(u8, pixel_addr + 0);
                        self._scratch_pad[pixel_idx * 4 + 1] = holly.read_vram(u8, pixel_addr + 1);
                        self._scratch_pad[pixel_idx * 4 + 2] = holly.read_vram(u8, pixel_addr + 2);
                        self._scratch_pad[pixel_idx * 4 + 3] = 255;
                    },
                }
            }
        }

        self._gctx_queue_mutex.lock();
        defer self._gctx_queue_mutex.unlock();
        self._gctx.queue.writeTexture(
            .{
                .texture = self._gctx.lookupResource(self.framebuffer.texture).?,
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

    /// Assumes gctx_queue_mutex is locked: Modifies parameters used in rendering.
    fn update_registers(self: *@This(), gpu: *const HollyModule.Holly) void {
        self.pt_alpha_ref = @as(f32, @floatFromInt(gpu.read_register(u8, .PT_ALPHA_REF))) / 255.0;

        self.fpu_shad_scale = gpu.read_register(HollyModule.FPU_SHAD_SCALE, .FPU_SHAD_SCALE).get_factor();

        self.fog_col_pal = gpu.read_register(PackedColor, .FOG_COL_RAM);
        self.fog_col_pal.a = 0; // Reserved
        self.fog_col_vert = gpu.read_register(PackedColor, .FOG_COL_VERT);
        self.fog_col_vert.a = 0; // Reserved
        self.fog_clamp_min = gpu.read_register(PackedColor, .FOG_CLAMP_MIN);
        self.fog_clamp_max = gpu.read_register(PackedColor, .FOG_CLAMP_MAX);

        const fog_density = gpu.read_register(u16, .FOG_DENSITY);
        const fog_density_mantissa = (fog_density >> 8) & 0xFF;
        const fog_density_exponent: i8 = @bitCast(@as(u8, @truncate(fog_density & 0xFF)));
        self.fog_density = @as(f32, @floatFromInt(fog_density_mantissa)) / 128.0 * std.math.pow(f32, 2.0, @floatFromInt(fog_density_exponent));
        for (0..0x80) |i| {
            self.fog_lut[i] = gpu.get_fog_table()[i] & 0x0000FFFF;
        }

        self.fb_r_ctrl = gpu.read_register(HollyModule.FB_R_CTRL, .FB_R_CTRL);
        self.write_back_parameters = gpu.get_write_back_parameters();
        // I suspect I'll have to come back to this: Printing some information to help detect unusual configurations.
        if (self.fb_r_ctrl.line_double) // Not handled: Emit a warning.
            if (Once(@src())) log.warn(termcolor.yellow("FB_R_CTRL.line_double is set: {any}"), .{self.fb_r_ctrl});

        const render_to_texture = gpu.render_to_texture();

        const ta_glob_tile_clip = gpu.read_register(HollyModule.TA_GLOB_TILE_CLIP, .TA_GLOB_TILE_CLIP);
        const ta_clip = ta_glob_tile_clip.pixel_size();

        // NOTE: The scaler isn't currently emulated, only the global_clip is adjusted.
        //       Rendering should be done as it is currently, but then filtered and scaled up/down.
        //         - Horizontal: Always scaled down by a factor of 2 (blend 2 neighboring pixels).
        //         - Vertical: Any value over 0x0400 (scaling down) enable Y filtering (blends 3 scanlines),
        //                     then scaled by value / 0x0400.
        const scale_x: u16 = if (self.write_back_parameters.scaler_ctl.horizontal_scaling_enable) 2 else 1;
        const scale_y: u16 = switch (self.write_back_parameters.scaler_ctl.vertical_scale_factor) {
            // NOTE: Technically, this is a floating point number, but I expect only 0x0400 for normal operation or 0x0800 for "flicker-free interlace mode type B".
            //       I'm afraid of rounding errors, keeping everything integer is simpler for now, we'll see if full support is needed later on.
            0x0400, 0x401 => 1,
            0x0800 => 2,
            else => sy: {
                if (Once(@src())) log.warn("Unusual vertical factor: {x}.", .{self.write_back_parameters.scaler_ctl.vertical_scale_factor});
                break :sy 1;
            },
        };

        // Clip values are pre-scaling, convert them to pixel coordinates.
        self.global_clip.x.min = scale_x * self.write_back_parameters.x_clip.min;
        self.global_clip.x.max = scale_y * @min(@as(u16, self.write_back_parameters.x_clip.max) + 1, ta_clip.x);
        self.global_clip.y.min = scale_x * self.write_back_parameters.y_clip.min;
        self.global_clip.y.max = scale_y * @min(@as(u16, self.write_back_parameters.y_clip.max) + 1, ta_clip.y);

        self.render_size = .{
            .width = self.global_clip.x.max,
            .height = self.global_clip.y.max,
        };

        if (!render_to_texture) {
            // vclk_div == 0 for halved pixel clock (480i or 240p); vclk_div == 1 for VGA mode (480p)
            const vga = self.fb_r_ctrl.vclk_div == 1;
            const spg_vblank = gpu.read_register(HollyModule.SPG_VBLANK, .SPG_VBLANK);
            self.output_resolution = .{
                .width = 640,
                .height = if (vga) 480 else @intCast(@abs(@as(i32, spg_vblank.vbend) - @as(i32, spg_vblank.vbstart))),
            };
            if (self.write_back_parameters.video_out_ctrl.pixel_double) self.output_resolution.width /= 2;
            // TODO: Same thing for line double?
            // FIXME: Because scaling isn't implemented (scaling up/down after rendering), output_resolution must be scaled as well.
            self.output_resolution.width *= scale_x;
            self.output_resolution.height *= scale_y;
        }
    }

    /// Assumes gctx_queue_mutex is locked: Modifies parameters used in rendering.
    pub fn update_palette(self: *@This(), gpu: *const HollyModule.Holly) void {
        // TODO: Check if the palette has changed (palette hash) instead of updating unconditionally?
        const palette_ram = gpu.get_palette();
        const palette_ctrl_ram: u2 = @truncate(gpu.read_register(u32, .PAL_RAM_CTRL) & 0b11);

        for (0..palette_ram.len) |i| {
            self.palette_bgra[i] = switch (palette_ctrl_ram) {
                // ARGB1555, RGB565, ARGB4444. These happen to match the values of TexturePixelFormat.
                0x0, 0x1, 0x2 => std.mem.bytesToValue(u32, &(Color16{ .value = @truncate(palette_ram[i]) }).bgra(@enumFromInt(palette_ctrl_ram), true)),
                // ARGB8888
                0x3 => @bitCast(palette_ram[i]),
            };
        }
    }

    // Assumes _gctx_queue_mutex is locked.
    pub fn upload_palette(self: *const @This()) void {
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.palette_buffer).?, 0, u8, std.mem.sliceAsBytes(self.palette_bgra));
    }

    // Pulls 3 vertices from the address pointed by ISP_BACKGND_T and places them at the front of the vertex buffer.
    // Assumes _gctx_queue_mutex is locked.
    pub fn update_background(self: *@This(), gpu: *const HollyModule.Holly) !void {
        const tags = gpu.read_register(HollyModule.ISP_BACKGND_T, .ISP_BACKGND_T);
        const param_base: u32 = self.on_render_start_param_base;
        const addr = param_base + 4 * @as(u32, tags.tag_address);
        const isp_tsp_instruction = gpu.read_vram(HollyModule.ISPTSPInstructionWord, addr);
        const tsp_instruction = gpu.read_vram(HollyModule.TSPInstructionWord, addr + 4);
        const texture_control = gpu.read_vram(HollyModule.TextureControlWord, addr + 8);
        const texture_size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);

        // FIXME: I don't understand. In the boot menu for example, this depth value is 0.0,
        //        which doesn't make sense. The vertices z position looks more inline with what
        //        I understand of the render pipeline.
        const depth = gpu.read_register(f32, .ISP_BACKGND_D);
        _ = depth;

        // Offset into the strip pointed by ISP_BACKGND_T indicated by tag_offset.
        const parameter_volume_mode = gpu.read_register(HollyModule.FPU_SHAD_SCALE, .FPU_SHAD_SCALE).enable and tags.shadow == 1;
        const tag_skip: u32 = tags.skip;
        const skipped_vertex_byte_size: u32 = 4 * (if (parameter_volume_mode) 3 + tag_skip else 3 + 2 * tag_skip);
        const start = addr + 12 + tags.tag_offset * skipped_vertex_byte_size;

        var vertex_byte_size: u32 = 4 * (3 + 1);
        var tex_idx: TextureIndex = 0;

        // The unused fields seems to be absent.
        if (isp_tsp_instruction.texture == 1) {
            tex_idx = self.get_texture_index(gpu, texture_size_index, texture_control) orelse self.upload_texture(gpu, tsp_instruction, texture_control);

            if (isp_tsp_instruction.uv_16bit == 1) {
                vertex_byte_size += 4 * 1;
            } else {
                vertex_byte_size += 4 * 2;
            }
        }
        if (isp_tsp_instruction.offset == 1) {
            vertex_byte_size += 4 * 1;
        }

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
                .bump_mapping_bit = 0,
                .color_clamp = tsp_instruction.color_clamp,
            },
        };

        const use_alpha = tsp_instruction.use_alpha == 1;
        var vertices: [4]Vertex = undefined;
        for (0..3) |i| {
            const vp = [_]u32{
                gpu.read_vram(u32, @intCast(start + i * vertex_byte_size + 0)),
                gpu.read_vram(u32, @intCast(start + i * vertex_byte_size + 4)),
                gpu.read_vram(u32, @intCast(start + i * vertex_byte_size + 8)),
                gpu.read_vram(u32, @intCast(start + i * vertex_byte_size + 12)),
                gpu.read_vram(u32, @intCast(start + i * vertex_byte_size + 16)),
                gpu.read_vram(u32, @intCast(start + i * vertex_byte_size + 20)),
                gpu.read_vram(u32, @intCast(start + i * vertex_byte_size + 24)),
            };
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

            vertices[i] = .{
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

        vertices[3] = .{
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

        try self.strips_metadata.append(self._allocator, .{ .area0_instructions = tex });

        std.debug.assert(FirstVertex == vertices.len);
        std.debug.assert(FirstIndex == indices.len);
    }

    /// Updates and transfers indices and vertices buffer.
    /// Assumes gctx_queue_mutex is locked.
    /// NOTE: Locking gctx_queue_mutex is necessary because of this function will upload textures and buffers to the GPU.
    ///       But! It incidently also protect concurrent access to `ta_lists` and `render_passes`.
    pub fn update(self: *@This(), gpu: *const HollyModule.Holly) !void {
        self.vertices.clearRetainingCapacity();
        self.strips_metadata.clearRetainingCapacity();

        if (self.DebugDisableTextureCacheAcrossFrames) {
            for (self.texture_metadata) |arr| {
                for (arr) |*tm| tm.status = .Invalid;
            }
        }

        defer self.check_texture_usage();

        self.min_depth = std.math.floatMax(f32);
        self.max_depth = 0.0;

        try self.update_background(gpu);
        self.upload_palette();

        var modifier_volumes_offset: usize = 0;

        const index_buffer = self._gctx.lookupResource(self.index_buffer).?;
        var index_buffer_pointer = FirstIndex;

        var pre_sorted_indices: std.ArrayList(u32) = .empty;
        defer pre_sorted_indices.deinit(self._allocator);

        for (self.render_passes.items, 0..) |*render_pass, pass_idx| {
            if (self.ta_lists.items.len <= pass_idx) break;
            const ta_lists = &self.ta_lists.items[pass_idx];

            render_pass.clearRetainingCapacity(self._allocator);

            pre_sorted_indices.clearRetainingCapacity();
            var pre_sorted_index_offset: u32 = 0;

            inline for ([_]HollyModule.ListType{ .Opaque, .PunchThrough, .Translucent }) |list_type| {
                const display_list: *const HollyModule.DisplayList = @constCast(ta_lists).get_list(list_type);

                var current_depth_compare_function: ?wgpu.CompareFunction = null;
                var current_step: *std.AutoArrayHashMap(PipelineKey, PipelineMetadata) = undefined;

                const pass = switch (list_type) {
                    .Opaque => &render_pass.opaque_pass,
                    .PunchThrough => &render_pass.punchthrough_pass,
                    .Translucent => &render_pass.translucent_pass,
                    else => @compileError("Invalid list type"),
                };

                for (0..display_list.vertex_strips.items.len) |idx| {
                    const strip_first_vertex_index: usize = self.vertices.items.len;
                    const global_parameters = display_list.vertex_strips.items[idx].global_parameters;
                    const polygon = global_parameters.polygon;

                    // Generic Parameters
                    const parameter_control_word = polygon.control_word();
                    const isp_tsp_instruction = polygon.isp_tsp_instruction();
                    var tsp_instruction = polygon.tsp_instruction();
                    const texture_control = polygon.texture_control();
                    var area1_tsp_instruction = polygon.area1_tsp_instruction();
                    const area1_texture_control = polygon.area1_texture_control();

                    switch (self.texture_filter) {
                        .@"Application Driven" => {},
                        .@"Force Nearest" => {
                            tsp_instruction.filter_mode = .Point;
                            if (area1_tsp_instruction) |*a1ti| a1ti.filter_mode = .Point;
                        },
                        .@"Force Linear" => {
                            tsp_instruction.filter_mode = .Bilinear;
                            if (area1_tsp_instruction) |*a1ti| a1ti.filter_mode = .Bilinear;
                        },
                    }

                    if (tsp_instruction.src_select != 0 or tsp_instruction.dst_select != 0) {
                        // NOTE: Ideas on how to suppport the Secondary Accumulation Buffer:
                        //        - Pass the SRC/DST select bit of each fragment all the way to the blend compute shader (Not ideal already).
                        //        - Sort fragments with dst=1 separately.
                        //        - When blending, if a fragment with src=1 is encountered, blend all dst=1 fragments with a lower index and use the result as the source. Remove the used fragments.
                        if (Once(@src()))
                            log.warn(termcolor.yellow("Secondary Accumulation Buffer usage detected in {t} pass (src_select={d}, dst_select={d}). This is not implemented, these polygons will be skipped."), .{ list_type, tsp_instruction.src_select, tsp_instruction.dst_select });
                        continue;
                    }

                    const textured = parameter_control_word.obj_control.texture == 1;
                    const use_alpha = tsp_instruction.use_alpha == 1;
                    const use_offset = isp_tsp_instruction.offset == 1;

                    const first_vertex = display_list.vertex_strips.items[idx].vertex_parameter_index;
                    const last_vertex = display_list.vertex_strips.items[idx].vertex_parameter_index + display_list.vertex_strips.items[idx].vertex_parameter_count;
                    const primitive_index: u32 = @intCast(self.strips_metadata.items.len);

                    const strip_start = self.vertices.items.len;
                    for (display_list.vertex_parameters.items[first_vertex..last_vertex]) |vertex| {
                        switch (vertex) {
                            // Packed Color, Non-Textured
                            .Type0 => |v| {
                                // Sanity checks.
                                std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                                std.debug.assert(!textured);
                                try self.vertices.append(self._allocator, .{
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
                                try self.vertices.append(self._allocator, .{
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
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = global_parameters.face_base_color.apply_intensity(v.base_intensity, use_alpha),
                                });
                            },
                            // Packed Color, Textured 32bit UV
                            .Type3 => |v| {
                                std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                                std.debug.assert(textured);
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = v.base_color.with_alpha(use_alpha),
                                    .offset_color = if (use_offset) v.offset_color.with_alpha(true) else .zero,
                                    .u = v.u,
                                    .v = v.v,
                                });
                            },
                            // Packed Color, Textured 16bit UV
                            .Type4 => |v| {
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = v.base_color.with_alpha(use_alpha),
                                    .offset_color = if (use_offset) v.offset_color.with_alpha(true) else .zero,
                                    .u = v.uv.u_as_f32(),
                                    .v = v.uv.v_as_f32(),
                                });
                            },
                            // Floating Color, Textured
                            .Type5 => |v| {
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = v.base_color.to_packed(use_alpha),
                                    .offset_color = if (use_offset) v.offset_color.to_packed(true) else .zero,
                                    .u = v.u,
                                    .v = v.v,
                                });
                            },
                            // Floating Color, Textured 16bit UV
                            .Type6 => |v| {
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = v.base_color.to_packed(use_alpha),
                                    .offset_color = if (use_offset) v.offset_color.to_packed(true) else .zero,
                                    .u = v.uv.u_as_f32(),
                                    .v = v.uv.v_as_f32(),
                                });
                            },
                            // Intensity
                            .Type7 => |v| {
                                std.debug.assert(parameter_control_word.obj_control.col_type == .IntensityMode1 or parameter_control_word.obj_control.col_type == .IntensityMode2);
                                std.debug.assert(textured);
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = global_parameters.face_base_color.apply_intensity(v.base_intensity, use_alpha),
                                    .offset_color = if (use_offset) global_parameters.face_offset_color.apply_intensity(v.offset_intensity, true) else .zero,
                                    .u = v.u,
                                    .v = v.v,
                                });
                            },
                            // Intensity, 16bit UV
                            .Type8 => |v| {
                                std.debug.assert(parameter_control_word.obj_control.col_type == .IntensityMode1 or parameter_control_word.obj_control.col_type == .IntensityMode2);
                                std.debug.assert(textured);
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = global_parameters.face_base_color.apply_intensity(v.base_intensity, use_alpha),
                                    .offset_color = if (use_offset) global_parameters.face_offset_color.apply_intensity(v.offset_intensity, true) else .zero,
                                    .u = v.uv.u_as_f32(),
                                    .v = v.uv.v_as_f32(),
                                });
                            },
                            // Non-Textured, Packed Color, with Two Volumes
                            .Type9 => |v| {
                                std.debug.assert(area1_tsp_instruction != null);
                                std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                                std.debug.assert(!textured);
                                try self.vertices.append(self._allocator, .{
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
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = global_parameters.face_base_color.apply_intensity(v.base_intensity_0, use_alpha),
                                    .area1_base_color = global_parameters.area1_face_base_color.apply_intensity(v.base_intensity_1, use_alpha),
                                });
                            },
                            // Textured, Packed Color, with Two Volumes
                            .Type11 => |v| {
                                std.debug.assert(area1_tsp_instruction != null);
                                std.debug.assert(area1_texture_control != null);
                                std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                                std.debug.assert(textured);
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = v.base_color_0.with_alpha(use_alpha),
                                    .offset_color = if (use_offset) v.offset_color_0.with_alpha(true) else .zero,
                                    .u = v.u0,
                                    .v = v.v0,
                                    .area1_base_color = v.base_color_1.with_alpha(use_alpha),
                                    .area1_offset_color = if (use_offset) v.offset_color_1.with_alpha(true) else .zero,
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
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = v.base_color_0.with_alpha(use_alpha),
                                    .offset_color = if (use_offset) v.offset_color_0.with_alpha(true) else .zero,
                                    .u = v.uv_0.u_as_f32(),
                                    .v = v.uv_0.v_as_f32(),
                                    .area1_base_color = v.base_color_1.with_alpha(use_alpha),
                                    .area1_offset_color = if (use_offset) v.offset_color_1.with_alpha(true) else .zero,
                                    .area1_u = v.uv_1.u_as_f32(),
                                    .area1_v = v.uv_1.v_as_f32(),
                                });
                            },
                            // Textured, Intensity, with Two Volumes
                            .Type13 => |v| {
                                std.debug.assert(area1_tsp_instruction != null);
                                std.debug.assert(area1_texture_control != null);
                                std.debug.assert(textured);
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = global_parameters.face_base_color.apply_intensity(v.base_intensity_0, use_alpha),
                                    .offset_color = if (use_offset) global_parameters.face_offset_color.apply_intensity(v.offset_intensity_0, true) else .zero,
                                    .u = v.u0,
                                    .v = v.v0,
                                    .area1_base_color = global_parameters.area1_face_base_color.apply_intensity(v.base_intensity_1, use_alpha),
                                    // "In the case of Polygon Type 4 (Intensity, with Two Volumes), the Face Color is used in both the Base Color and the Offset Color."
                                    .area1_offset_color = if (use_offset) global_parameters.area1_face_base_color.apply_intensity(v.offset_intensity_1, true) else .zero,
                                    .area1_u = v.u1,
                                    .area1_v = v.v1,
                                });
                            },
                            // Textured, Intensity, with Two Volumes
                            .Type14 => |v| {
                                std.debug.assert(area1_tsp_instruction != null);
                                std.debug.assert(area1_texture_control != null);
                                std.debug.assert(textured);
                                try self.vertices.append(self._allocator, .{
                                    .primitive_index = primitive_index,
                                    .x = v.x,
                                    .y = v.y,
                                    .z = v.z,
                                    .base_color = global_parameters.face_base_color.apply_intensity(v.base_intensity_0, use_alpha),
                                    .offset_color = if (use_offset) global_parameters.face_offset_color.apply_intensity(v.offset_intensity_0, true) else .zero,
                                    .u = v.uv_0.u_as_f32(),
                                    .v = v.uv_0.v_as_f32(),
                                    .area1_base_color = global_parameters.area1_face_base_color.apply_intensity(v.base_intensity_1, use_alpha),
                                    // "In the case of Polygon Type 4 (Intensity, with Two Volumes), the Face Color is used in both the Base Color and the Offset Color."
                                    .area1_offset_color = if (use_offset) global_parameters.area1_face_base_color.apply_intensity(v.offset_intensity_1, true) else .zero,
                                    .area1_u = v.uv_1.u_as_f32(),
                                    .area1_v = v.uv_1.v_as_f32(),
                                });
                            },
                            .SpriteType0, .SpriteType1 => {
                                var vs = gen_sprite_vertices(vertex);
                                for (&vs) |*v| {
                                    v.primitive_index = primitive_index;
                                    v.base_color = global_parameters.sprite_face_base_color.with_alpha(use_alpha);
                                    if (use_offset)
                                        v.offset_color = global_parameters.sprite_face_offset_color;
                                    self.min_depth = @min(self.min_depth, v.z);
                                    self.max_depth = @max(self.max_depth, v.z);

                                    try self.vertices.append(self._allocator, v.*);
                                }
                            },
                        }

                        self.min_depth = @min(self.min_depth, self.vertices.getLast().z);
                        self.max_depth = @max(self.max_depth, self.vertices.getLast().z);
                    }

                    if (self.ExperimentalClampSpritesUVs and self.resolution.width != NativeResolution.width and textured) {
                        // Here "Sprite" means "Screen Aligned Square", basically (not strictly a sprite in the TA sense). 4 vertices with the same depth, with UVs in [0..1].
                        const vertices = self.vertices.items[strip_start..];
                        if (vertices.len == 4 and (vertices[0].z == vertices[1].z and vertices[0].z == vertices[2].z and vertices[0].z == vertices[3].z)) {
                            const min_uv = .{ .u = @min(vertices[0].u, vertices[1].u, vertices[2].u, vertices[3].u), .v = @min(vertices[0].v, vertices[1].v, vertices[2].v, vertices[3].v) };
                            const max_uv = .{ .u = @max(vertices[0].u, vertices[1].u, vertices[2].u, vertices[3].u), .v = @max(vertices[0].v, vertices[1].v, vertices[2].v, vertices[3].v) };
                            if (min_uv.u >= 0.0 and max_uv.u <= 1.0 and min_uv.v >= 0.0 and max_uv.v <= 1.0) {
                                const min = .{ std.math.lossyCast(u32, @round(@min(vertices[0].x, vertices[1].x, vertices[2].x, vertices[3].x))), std.math.lossyCast(u32, @round(@min(vertices[0].y, vertices[1].y, vertices[2].y, vertices[3].y))) };
                                const max = .{ std.math.lossyCast(u32, @round(@max(vertices[0].x, vertices[1].x, vertices[2].x, vertices[3].x))), std.math.lossyCast(u32, @round(@max(vertices[0].y, vertices[1].y, vertices[2].y, vertices[3].y))) };
                                const texture_size = .{ @as(f32, @floatFromInt(tsp_instruction.get_u_size())), @as(f32, @floatFromInt(tsp_instruction.get_v_size())) };

                                const min_texel = .{ std.math.lossyCast(u32, texture_size[0] * min_uv.u), std.math.lossyCast(u32, texture_size[1] * min_uv.v) };
                                const max_texel = .{ std.math.lossyCast(u32, texture_size[0] * max_uv.u), std.math.lossyCast(u32, texture_size[1] * max_uv.v) };
                                const texel_width = max_texel[0] - min_texel[0];
                                const texel_height = max_texel[1] - min_texel[1];

                                const width: u32 = max[0] - min[0];
                                const height: u32 = max[1] - min[1];
                                const one_actually = 0.99609375; // Sonic Adventure 2 title screen special case.
                                // Sprite screen size perfectly matches the texture size.
                                if (width == tsp_instruction.get_u_size() and height == tsp_instruction.get_v_size()) {
                                    for (self.vertices.items[strip_start..]) |*v| {
                                        if (v.u == one_actually) v.u = 1.0;
                                        if (v.v == one_actually) v.v = 1.0;
                                        tsp_instruction.clamp_uv = .{ .u = true, .v = true }; // Force UV clamping on to avoid wrapping errors after upscaling. (NOTE: UVs are in [0,1], so the value of flip_uv is irrelevant.)
                                    }
                                } else if (width == texel_width and height == texel_height) {
                                    // Example: Text in the Crazy Taxi menu.
                                    // I don't have a good fix for this case yet.
                                }
                            }
                        }
                    }

                    var tex_idx: TextureIndex = 0;
                    var tex_idx_area_1: TextureIndex = InvalidTextureIndex;
                    if (textured) {
                        const texture_size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);
                        tex_idx = self.get_texture_index(gpu, texture_size_index, texture_control) orelse self.upload_texture(gpu, tsp_instruction, texture_control);
                    }
                    if (area1_texture_control) |tc| {
                        const texture_size_index = @max(tsp_instruction.texture_u_size, tsp_instruction.texture_v_size);
                        tex_idx_area_1 = self.get_texture_index(gpu, texture_size_index, tc) orelse self.upload_texture(gpu, area1_tsp_instruction.?, tc);
                    }

                    const clamp = tsp_instruction.clamp_uv;
                    const flip = tsp_instruction.final_flip_uv();
                    const u_addr_mode: wgpu.AddressMode = if (clamp.u) .clamp_to_edge else if (flip.u) .mirror_repeat else .repeat;
                    const v_addr_mode: wgpu.AddressMode = if (clamp.v) .clamp_to_edge else if (flip.v) .mirror_repeat else .repeat;

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
                            .bump_mapping_bit = if (texture_control.pixel_format == .BumpMap) 1 else 0,
                            .color_clamp = tsp_instruction.color_clamp,
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
                            .mipmap_bit = if (area1_texture_control) |tc| tc.mip_mapped else 0,
                            .bump_mapping_bit = if (area1_texture_control) |tc| (if (tc.pixel_format == .BumpMap) 1 else 0) else 0,
                            .color_clamp = atspi.color_clamp,
                        },
                    } else .invalid;

                    try self.strips_metadata.append(self._allocator, .{
                        .area0_instructions = area0_instructions,
                        .area1_instructions = area1_instructions,
                    });

                    // Triangle Strips
                    if (self.vertices.items.len - strip_first_vertex_index < 3) {
                        log.err("Not enough vertices in strip: {d} vertices.", .{self.vertices.items.len - strip_first_vertex_index});
                    } else {
                        // "In the case of a flat-shaded polygon, the Shading Color data (the Base Color, Offset Color, and Bump Map parameters) become valid starting with the third vertex after the start of the strip." - Thanks MetalliC for pointing that out!
                        if (isp_tsp_instruction.gouraud == 0) {
                            // WebGPU uses the parameters of the first vertex by default (you can specify first or either (implementation dependent), but not force last),
                            // while the DC used the last (3rd) of each triangle. This shifts the concerned parameters.
                            for (strip_first_vertex_index..self.vertices.items.len - 2) |i| {
                                self.vertices.items[i].base_color = self.vertices.items[i + 2].base_color;
                                self.vertices.items[i].offset_color = self.vertices.items[i + 2].offset_color;
                                self.vertices.items[i].area1_base_color = self.vertices.items[i + 2].area1_base_color;
                                self.vertices.items[i].area1_offset_color = self.vertices.items[i + 2].area1_offset_color;
                                // TODO: Bump map parameters?
                            }
                        }

                        var pipeline_key = PipelineKey{
                            .translucent = list_type == .Translucent,
                            .src_blend_factor = translate_src_blend_factor(tsp_instruction.src_alpha_instr),
                            .dst_blend_factor = translate_dst_blend_factor(tsp_instruction.dst_alpha_instr),
                            .depth_compare = translate_depth_compare_mode(isp_tsp_instruction.depth_compare_mode),
                            .depth_write_enabled = isp_tsp_instruction.z_write_disable == 0,
                            .culling_mode = isp_tsp_instruction.culling_mode,
                        };

                        switch (list_type) {
                            .Opaque => {
                                if (pipeline_key.src_blend_factor != .one or pipeline_key.dst_blend_factor != .zero) {
                                    log.debug("Unexpected blend mode for opaque list: src={t}, dst={t}", .{ pipeline_key.src_blend_factor, pipeline_key.dst_blend_factor });
                                    pipeline_key.src_blend_factor = .one;
                                    pipeline_key.dst_blend_factor = .zero;
                                }
                            },
                            .PunchThrough => {
                                if (pipeline_key.src_blend_factor != .src_alpha or pipeline_key.dst_blend_factor != .one_minus_src_alpha) {
                                    log.debug("Unexpected blend mode for punch through list: src={t}, dst={t}", .{ pipeline_key.src_blend_factor, pipeline_key.dst_blend_factor });
                                    pipeline_key.src_blend_factor = .src_alpha;
                                    pipeline_key.dst_blend_factor = .one_minus_src_alpha;
                                }
                            },
                            else => {},
                        }

                        {
                            if (list_type == .Translucent and render_pass.pre_sort) {
                                // In this case, we need to fully preserve the order of the draw calls. Batching only when they draw calls are stricly following each others.
                                const prev_draw_call = if (render_pass.pre_sorted_translucent_pass.items.len == 0) null else &render_pass.pre_sorted_translucent_pass.items[render_pass.pre_sorted_translucent_pass.items.len - 1];
                                if (prev_draw_call == null or
                                    !std.meta.eql(prev_draw_call.?.pipeline_key, pipeline_key) or
                                    !std.meta.eql(prev_draw_call.?.user_clip, display_list.vertex_strips.items[idx].user_clip) or
                                    prev_draw_call.?.sampler != sampler)
                                {
                                    if (prev_draw_call) |draw_call| {
                                        draw_call.start_index = index_buffer_pointer + pre_sorted_index_offset;
                                        draw_call.index_count = @intCast(pre_sorted_indices.items.len - pre_sorted_index_offset);
                                        pre_sorted_index_offset += draw_call.index_count;
                                    }

                                    try render_pass.pre_sorted_translucent_pass.append(self._allocator, .{
                                        .pipeline_key = pipeline_key,
                                        .sampler = sampler,
                                        .user_clip = display_list.vertex_strips.items[idx].user_clip,
                                    });
                                }
                                for (strip_first_vertex_index..self.vertices.items.len) |i|
                                    try pre_sorted_indices.append(self._allocator, @intCast(FirstVertex + i));
                                try pre_sorted_indices.append(self._allocator, std.math.maxInt(u32)); // Primitive Restart: Ends the current triangle strip.
                            } else {
                                // Draw calls are batched together as much as possible by PipelineKeys (and then by DrawCallKey).
                                // This works well in most cases and reduces host draw calls by a lot, but it fails in some
                                // edge cases. To alleviate those, changes in depth compare function are treated as a "barrier",
                                // guaranteeing correct ordering between draws before and after the change (spliting them into
                                // two distinct 'steps'). As failure cases I found involved the use of 'always' or 'never' compare
                                // functions, this seem to be enough, while keeping most of the benefit of batching.
                                if (current_depth_compare_function == null) { // Initialisation
                                    current_depth_compare_function = pipeline_key.depth_compare;
                                    try pass.steps.append(self._allocator, .init(self._allocator));
                                    current_step = &pass.steps.items[0];
                                } else if (current_depth_compare_function != pipeline_key.depth_compare) { // Next Step
                                    current_depth_compare_function = pipeline_key.depth_compare;
                                    try pass.steps.append(self._allocator, .init(self._allocator));
                                    current_step = &pass.steps.items[pass.steps.items.len - 1];
                                }

                                var pipeline = current_step.getPtr(pipeline_key) orelse put: {
                                    try current_step.put(pipeline_key, .init(self._allocator));
                                    break :put current_step.getPtr(pipeline_key).?;
                                };

                                const draw_call_key = DrawCallKey{ .sampler = sampler, .user_clip = display_list.vertex_strips.items[idx].user_clip };

                                var draw_call = pipeline.draw_calls.getPtr(draw_call_key);
                                if (draw_call == null) {
                                    try pipeline.draw_calls.put(draw_call_key, .init(
                                        sampler,
                                        display_list.vertex_strips.items[idx].user_clip,
                                    ));
                                    draw_call = pipeline.draw_calls.getPtr(draw_call_key);
                                }
                                for (strip_first_vertex_index..self.vertices.items.len) |i|
                                    try draw_call.?.indices.append(self._allocator, @intCast(FirstVertex + i));
                                try draw_call.?.indices.append(self._allocator, std.math.maxInt(u32)); // Primitive Restart: Ends the current triangle strip.
                            }
                        }
                    }
                }
            }

            if (pre_sorted_indices.items.len > 0) {
                // Finalize the last pre-sorted draw call
                const draw_call = &render_pass.pre_sorted_translucent_pass.items[render_pass.pre_sorted_translucent_pass.items.len - 1];
                draw_call.start_index = index_buffer_pointer + pre_sorted_index_offset;
                draw_call.index_count = @intCast(pre_sorted_indices.items.len - pre_sorted_index_offset);
                // And send all indices to the GPU
                self._gctx.queue.writeBuffer(index_buffer, @sizeOf(u32) * index_buffer_pointer, u32, pre_sorted_indices.items);
                index_buffer_pointer += @intCast(pre_sorted_indices.items.len);
            }

            if (self.vertices.items.len > 0) {
                for ([3]*PassMetadata{ &render_pass.opaque_pass, &render_pass.punchthrough_pass, &render_pass.translucent_pass }) |pass| {
                    for (pass.steps.items) |*step| {
                        var it = step.iterator();
                        while (it.next()) |entry| {
                            for (entry.value_ptr.*.draw_calls.values()) |*draw_call| {
                                draw_call.start_index = index_buffer_pointer;
                                draw_call.index_count = @intCast(draw_call.indices.items.len);
                                self._gctx.queue.writeBuffer(index_buffer, @sizeOf(u32) * index_buffer_pointer, u32, draw_call.indices.items);
                                index_buffer_pointer += @intCast(draw_call.indices.items.len);

                                draw_call.indices.clearRetainingCapacity();
                            }
                        }
                    }
                }
            }

            // Modifier volumes
            self.modifier_volume_vertices.clearRetainingCapacity();

            for (ta_lists.volume_triangles.items) |triangle| {
                try self.modifier_volume_vertices.append(self._allocator, .{ triangle.ax, triangle.ay, triangle.az, 1.0 });
                try self.modifier_volume_vertices.append(self._allocator, .{ triangle.bx, triangle.by, triangle.bz, 1.0 });
                try self.modifier_volume_vertices.append(self._allocator, .{ triangle.cx, triangle.cy, triangle.cz, 1.0 });
                self.min_depth = @min(self.min_depth, triangle.az);
                self.max_depth = @max(self.max_depth, triangle.az);
                self.min_depth = @min(self.min_depth, triangle.bz);
                self.max_depth = @max(self.max_depth, triangle.bz);
                self.min_depth = @min(self.min_depth, triangle.cz);
                self.max_depth = @max(self.max_depth, triangle.cz);
            }

            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.modifier_volume_vertex_buffer).?, modifier_volumes_offset, [4]f32, self.modifier_volume_vertices.items);
            modifier_volumes_offset += @sizeOf([4]f32) * self.modifier_volume_vertices.items.len;
        }
        // Send everything else to the GPU
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.strips_metadata_buffer).?, 0, StripMetadata, self.strips_metadata.items);
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, @sizeOf(Vertex) * FirstVertex, Vertex, self.vertices.items);
    }

    fn convert_clipping(self: *@This(), user_clip: ?HollyModule.UserTileClipInfo) HollyModule.UserTileClipInfo {
        const factor = @divTrunc(self.resolution.width, NativeResolution.width);
        const x = factor * self.global_clip.x.min;
        const y = factor * self.global_clip.y.min;
        const width = @min(factor * (self.global_clip.x.max - self.global_clip.x.min), self.resolution.width);
        const height = @min(factor * (self.global_clip.y.max - self.global_clip.y.min), self.resolution.height);

        if (user_clip) |uc| {
            // FIXME: Handle other usages.
            //        Use Stencil for OutsideEnabled
            if (uc.usage == .InsideEnabled) {
                const scaled_x = @max(factor *| uc.x, x);
                const scaled_y = @max(factor *| uc.y, y);
                return .{
                    .usage = .InsideEnabled,
                    .x = scaled_x,
                    .y = scaled_y,
                    .width = @min(@min(factor *| uc.width, width), self.resolution.width -| scaled_x),
                    .height = @min(@min(factor *| uc.height, height), self.resolution.height -| scaled_y),
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

    /// Convert framebuffer from internal resolution to upscaled resolution, copying from framebuffer to resized_framebuffer.
    /// Locks _gctx_queue_mutex.
    pub fn blit_framebuffer(self: *const @This()) void {
        const gctx = self._gctx;
        self._gctx_queue_mutex.lock();
        defer self._gctx_queue_mutex.unlock();

        if (gctx.lookupResource(self.blit_pipeline)) |pipeline| {
            const commands = commands: {
                const encoder = gctx.device.createCommandEncoder(null);
                defer encoder.release();

                const blit_vb_info = gctx.lookupResourceInfo(self.blit_vertex_buffer).?;
                const blit_ib_info = gctx.lookupResourceInfo(self.blit_index_buffer).?;

                const framebuffer_resize_bind_group = gctx.lookupResource(self.framebuffer_resize_bind_group).?;

                {
                    const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                        .view = gctx.lookupResource(self.resized_framebuffer.view).?,
                        .load_op = .clear,
                        .store_op = .store,
                    }};
                    const pass = encoder.beginRenderPass(.{
                        .label = .init("Blit Framebuffer"),
                        .color_attachment_count = color_attachments.len,
                        .color_attachments = &color_attachments,
                    });
                    defer {
                        pass.end();
                        pass.release();
                    }

                    const blit_uniform_mem = self._gctx.uniformsAllocate(BlitUniforms, 1);
                    blit_uniform_mem.slice[0] = .{
                        .min = .{ 0, 0 },
                        .max = .{ 1.0, 1.0 },
                    };

                    pass.setVertexBuffer(0, blit_vb_info.gpuobj.?, 0, blit_vb_info.size);
                    pass.setIndexBuffer(blit_ib_info.gpuobj.?, .uint32, 0, blit_ib_info.size);

                    pass.setPipeline(pipeline);

                    pass.setBindGroup(0, framebuffer_resize_bind_group, &.{blit_uniform_mem.offset});
                    pass.drawIndexed(4, 1, 0, 0, 0);
                }

                break :commands encoder.finish(null);
            };
            defer commands.release();

            gctx.submit(&.{commands});
        }
    }

    /// Assumes gctx_queue_mutex is locked.
    /// NOTE: Locking gctx_queue_mutex is required for submitting commands, mapping the framebuffer, but also (sadly) uniform allocation.
    ///       Also protects concurrent access to `ta_lists` and `render_passes`.
    pub fn render(self: *@This(), holly: *HollyModule.Holly, render_to_texture: bool) !void {
        const gctx = self._gctx;
        const target_size = Resolution{ .width = 640, .height = 480 };

        // We might only use a fraction of the target (240p games and RTT). The following is used as an optimization: Only copy and run computed shaders on the relevant portion of the target.
        const render_area: Resolution = .{
            .width = @min(self.render_size.width * self.resolution.width / NativeResolution.width, self.resolution.width), // Assumes resolution is an integer multiplier of the NativeResolution.
            .height = @min(self.render_size.height * self.resolution.height / NativeResolution.height, self.resolution.height),
        };

        if (render_to_texture) {
            log.info("Rendering to texture! [{d},{d}] to [{d},{d}]", .{ self.global_clip.x.min, self.global_clip.y.min, self.global_clip.x.max, self.global_clip.y.max });
            if (!self.ExperimentalRenderToTexture) {
                log.warn("ExperimentalRenderToTexture is disabled.", .{});
                return;
            }
        }

        const Target = struct { native: TextureAndView.Resources, resized: TextureAndView.Resources };
        const target: Target = if (render_to_texture) .{
            .native = self.render_to_texture_target.lookup(gctx),
            .resized = self.resized_render_to_texture_target.lookup(gctx),
        } else .{
            .native = self.framebuffer.lookup(gctx),
            .resized = self.resized_framebuffer.lookup(gctx),
        };

        const commands = commands: {
            const encoder = gctx.device.createCommandEncoder(null);
            defer encoder.release();

            const vb_info = gctx.lookupResourceInfo(self.vertex_buffer).?;
            const ib_info = gctx.lookupResourceInfo(self.index_buffer).?;

            const uniform_mem = gctx.uniformsAllocate(Uniforms, 1);
            uniform_mem.slice[0] = .{
                .depth_min = self.min_depth,
                .depth_max = self.max_depth,
                .framebuffer_width = @floatFromInt(target_size.width),
                .framebuffer_height = @floatFromInt(target_size.height),
                .fpu_shad_scale = self.fpu_shad_scale,
                .fog_density = self.fog_density,
                .pt_alpha_ref = self.pt_alpha_ref,
                .fog_col_pal = self.fog_col_pal,
                .fog_col_vert = self.fog_col_vert,
                .fog_clamp_min = self.fog_clamp_min,
                .fog_clamp_max = self.fog_clamp_max,
                .fog_lut = self.fog_lut,
            };

            const textures_bind_group = gctx.lookupResource(self.textures_bind_group).?;
            const depth_view = gctx.lookupResource(self.depth.view).?;

            // Background
            if (gctx.lookupResource(self.get_or_put_pipeline(BackgroundPipelineKey, .Async))) |background_pipeline| {
                const color_attachments = [_]wgpu.RenderPassColorAttachment{
                    .{
                        .view = target.resized.view,
                        .load_op = .load,
                        .store_op = .store,
                    },
                    .{
                        .view = gctx.lookupResource(self.resized_framebuffer_area1.view).?,
                        .load_op = .clear,
                        .store_op = .store,
                    },
                };
                const pass = encoder.beginRenderPass(.{
                    .label = .init("Background"),
                    .color_attachment_count = color_attachments.len,
                    .color_attachments = &color_attachments,
                    .depth_stencil_attachment = &.{
                        .view = depth_view,
                        .depth_load_op = .clear,
                        .depth_store_op = .store,
                        .depth_clear_value = DepthClearValue,
                        .stencil_load_op = .clear,
                        .stencil_store_op = .discard,
                        .stencil_clear_value = 0,
                        .stencil_read_only = .false,
                    },
                });
                defer {
                    pass.end();
                    pass.release();
                }

                pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                pass.setBindGroup(0, textures_bind_group, &.{uniform_mem.offset});

                pass.setPipeline(background_pipeline);
                pass.setBindGroup(1, gctx.lookupResource(self.sampler_bind_groups[sampler_index(.linear, .linear, .linear, .clamp_to_edge, .clamp_to_edge)]).?, &.{});
                pass.drawIndexed(FirstIndex, 1, 0, 0, 0);
            }

            for (self.render_passes.items, 0..) |render_pass, pass_idx| {
                if (self.ta_lists.items.len <= pass_idx) break;
                const ta_lists = &self.ta_lists.items[pass_idx];

                if (!render_pass.opaque_list_pointer.empty) {
                    {
                        const color_attachments = [_]wgpu.RenderPassColorAttachment{
                            .{
                                .view = target.resized.view,
                                .load_op = .load,
                                .store_op = .store,
                            },
                            .{
                                .view = gctx.lookupResource(self.resized_framebuffer_area1.view).?,
                                .load_op = .clear,
                                .store_op = .store,
                            },
                        };
                        const pass = encoder.beginRenderPass(.{
                            .label = .init("Opaque pass"),
                            .color_attachment_count = color_attachments.len,
                            .color_attachments = &color_attachments,
                            .depth_stencil_attachment = &.{
                                .view = depth_view,
                                .depth_load_op = if (render_pass.z_clear) .clear else .load,
                                .depth_store_op = .store,
                                .depth_clear_value = DepthClearValue,
                                .stencil_load_op = .clear,
                                .stencil_store_op = .discard,
                                .stencil_clear_value = 0,
                                .stencil_read_only = .false,
                            },
                        });
                        defer {
                            pass.end();
                            pass.release();
                        }

                        pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                        pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                        pass.setBindGroup(0, textures_bind_group, &.{uniform_mem.offset});

                        // Opaque and PunchThrough geometry
                        // FIXME: PunchThrough should be drawn last? Is there a case where it matters with this setup?
                        inline for (.{ &render_pass.opaque_pass, &render_pass.punchthrough_pass }) |metadata| {
                            for (metadata.steps.items) |step| {
                                var it = step.iterator();
                                while (it.next()) |entry| {
                                    if (entry.value_ptr.*.draw_calls.count() > 0) {
                                        const pl = self.get_or_put_pipeline(entry.key_ptr.*, .Async);
                                        const pipeline = gctx.lookupResource(pl) orelse continue;
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
                    }

                    // FIXME: WGPU doesn't support reading from storage textures... This is a bad workaround.
                    encoder.copyTextureToTexture(
                        .{ .texture = target.resized.texture },
                        .{ .texture = gctx.lookupResource(self.resized_framebuffer_copy.texture).? },
                        .{ .width = render_area.width, .height = render_area.height },
                    );

                    if (!render_pass.opaque_modifier_volume_pointer.empty and ta_lists.opaque_modifier_volumes.items.len > 0) skip_mv: {
                        const closed_modifier_volume_pipeline = gctx.lookupResource(self.closed_modifier_volume_pipeline) orelse break :skip_mv;
                        const shift_stencil_buffer_modifier_volume_pipeline = gctx.lookupResource(self.shift_stencil_buffer_modifier_volume_pipeline) orelse break :skip_mv;
                        const modifier_volume_apply_pipeline = gctx.lookupResource(self.modifier_volume_apply_pipeline) orelse break :skip_mv;
                        const open_modifier_volume_pipeline = gctx.lookupResource(self.open_modifier_volume_pipeline) orelse break :skip_mv;

                        // Write to stencil buffer
                        {
                            const modifier_volume_bind_group = gctx.lookupResource(self.modifier_volume_bind_group).?;
                            const vs_uniform_mem = gctx.uniformsAllocate(ModifierVolumeUniforms, 1);
                            vs_uniform_mem.slice[0] = .{
                                .min_depth = self.min_depth,
                                .max_depth = self.max_depth,
                                .framebuffer_width = @floatFromInt(target_size.width),
                                .framebuffer_height = @floatFromInt(target_size.height),
                            };

                            const pass = encoder.beginRenderPass(.{
                                .label = .init("Modifier Volume Stencil"),
                                .color_attachment_count = 0,
                                .color_attachments = null,
                                .depth_stencil_attachment = &.{
                                    .view = depth_view,
                                    .depth_load_op = .load,
                                    .depth_store_op = .store,
                                    .depth_clear_value = DepthClearValue,
                                    .depth_read_only = .false,
                                    .stencil_load_op = .clear,
                                    .stencil_store_op = .store,
                                    .stencil_clear_value = 0,
                                    .stencil_read_only = .false,
                                },
                            });
                            defer {
                                pass.end();
                                pass.release();
                            }

                            const modifier_volume_vb_info = gctx.lookupResourceInfo(self.modifier_volume_vertex_buffer).?;
                            pass.setVertexBuffer(0, modifier_volume_vb_info.gpuobj.?, 0, modifier_volume_vb_info.size);
                            pass.setBindGroup(0, modifier_volume_bind_group, &.{vs_uniform_mem.offset});

                            // Close volume pass.
                            // Counts triangles passing the depth test: If odd, the volume intersects the depth buffer.
                            for (ta_lists.opaque_modifier_volumes.items) |volume| {
                                if (volume.instructions.volume_instruction == .OutsideLastPolygon and Once(@src())) log.warn(termcolor.yellow("TODO: Unimplemented Exclusion Modifier Volume."), .{});
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

                            const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                                .view = target.resized.view,
                                .load_op = .load,
                                .store_op = .store,
                            }};
                            const pass = encoder.beginRenderPass(.{
                                .label = .init("Modifier Volume Apply"),
                                .color_attachment_count = color_attachments.len,
                                .color_attachments = &color_attachments,
                                .depth_stencil_attachment = &.{
                                    .view = depth_view,
                                    .depth_load_op = .undefined,
                                    .depth_store_op = .undefined,
                                    .depth_clear_value = DepthClearValue,
                                    .depth_read_only = .true,
                                    .stencil_load_op = .undefined,
                                    .stencil_store_op = .undefined,
                                    .stencil_clear_value = 0,
                                    .stencil_read_only = .true,
                                },
                            });
                            defer {
                                pass.end();
                                pass.release();
                            }

                            const blit_uniform_mem = gctx.uniformsAllocate(BlitUniforms, 1);
                            blit_uniform_mem.slice[0] = .{
                                .min = .{ 0, 0 },
                                .max = .{ 1.0, 1.0 },
                            };

                            pass.setPipeline(modifier_volume_apply_pipeline);
                            pass.setVertexBuffer(0, blit_vb_info.gpuobj.?, 0, blit_vb_info.size);
                            pass.setIndexBuffer(blit_ib_info.gpuobj.?, .uint32, 0, blit_ib_info.size);
                            pass.setBindGroup(0, mva_bind_group, &.{blit_uniform_mem.offset});

                            pass.setStencilReference(0x02);
                            pass.drawIndexed(4, 1, 0, 0, 0);
                        }

                        // Copy the result to resized_framebuffer_copy_texture_view too.
                        encoder.copyTextureToTexture(
                            .{ .texture = target.resized.texture },
                            .{ .texture = gctx.lookupResource(self.resized_framebuffer_copy.texture).? },
                            .{ .width = render_area.width, .height = render_area.height },
                        );
                    }
                }

                if (!render_pass.translucent_list_pointer.empty) {
                    if (render_pass.pre_sort) {
                        // Disable PT discards for pre-sorted translucent polygons (This uses the same pipelines)
                        const pre_sort_uniform_mem = gctx.uniformsAllocate(Uniforms, 1);
                        pre_sort_uniform_mem.slice[0] = uniform_mem.slice[0];
                        pre_sort_uniform_mem.slice[0].pt_alpha_ref = -1.0;

                        const color_attachments = [_]wgpu.RenderPassColorAttachment{
                            .{
                                .view = target.resized.view,
                                .load_op = .load,
                                .store_op = .store,
                            },
                            .{
                                .view = gctx.lookupResource(self.resized_framebuffer_area1.view).?,
                                .load_op = .clear,
                                .store_op = .store,
                            },
                        };
                        const pass = encoder.beginRenderPass(.{
                            .label = .init("Presorted Translucent pass"),
                            .color_attachment_count = color_attachments.len,
                            .color_attachments = &color_attachments,
                            .depth_stencil_attachment = &.{
                                .view = depth_view,
                                .depth_load_op = .load,
                                .depth_store_op = .store,
                                .stencil_load_op = .clear,
                                .stencil_store_op = .discard,
                                .stencil_clear_value = 0,
                                .stencil_read_only = .false,
                            },
                        });
                        defer {
                            pass.end();
                            pass.release();
                        }

                        pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                        pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                        pass.setBindGroup(0, textures_bind_group, &.{pre_sort_uniform_mem.offset});

                        var current_pipeline: ?PipelineKey = null;
                        var current_sampler: ?u8 = null;
                        for (render_pass.pre_sorted_translucent_pass.items) |draw_call| {
                            if (current_pipeline == null or !std.meta.eql(draw_call.pipeline_key, current_pipeline.?)) {
                                const pl = self.get_or_put_pipeline(draw_call.pipeline_key, .Async);
                                const pipeline = gctx.lookupResource(pl) orelse continue;
                                pass.setPipeline(pipeline);
                                current_pipeline = draw_call.pipeline_key;
                            }

                            if (draw_call.index_count > 0) {
                                const clip = self.convert_clipping(draw_call.user_clip);
                                pass.setScissorRect(clip.x, clip.y, clip.width, clip.height);

                                if (current_sampler == null or draw_call.sampler != current_sampler.?) {
                                    pass.setBindGroup(1, gctx.lookupResource(self.sampler_bind_groups[draw_call.sampler]).?, &.{});
                                    current_sampler = draw_call.sampler;
                                }
                                pass.drawIndexed(draw_call.index_count, 1, draw_call.start_index, 0, 0);
                            }
                        }

                        if (ta_lists.translucent_modifier_volumes.items.len > 0 and Once(@src()))
                            log.err(termcolor.red("Translucent modifier volumes cannot be used in Pre-sort mode."), .{});
                    } else if (render_pass.translucent_pass.steps.items.len > 0) skip_tmv: {
                        // Generate all translucent fragments
                        const translucent_bind_group = gctx.lookupResource(self.translucent_bind_group).?;
                        const blend_bind_group = if (render_to_texture) gctx.lookupResource(self.blend_bind_group_render_to_texture).? else gctx.lookupResource(self.blend_bind_group).?;

                        const translucent_modvol_pipeline = gctx.lookupResource(self.translucent_modvol_pipeline) orelse break :skip_tmv;
                        const translucent_modvol_merge_pipeline = gctx.lookupResource(self.translucent_modvol_merge_pipeline) orelse break :skip_tmv;

                        const modifier_volume_bind_group = gctx.lookupResource(self.modifier_volume_bind_group).?;
                        const translucent_modvol_bind_group = gctx.lookupResource(self.translucent_modvol_bind_group).?;
                        const translucent_modvol_merge_bind_group = gctx.lookupResource(self.translucent_modvol_merge_bind_group).?;

                        const modifier_volume_vb_info = gctx.lookupResourceInfo(self.modifier_volume_vertex_buffer).?;
                        const vs_mv_uniform_mem = gctx.uniformsAllocate(ModifierVolumeUniforms, 1);
                        vs_mv_uniform_mem.slice[0] = .{
                            .min_depth = self.min_depth,
                            .max_depth = self.max_depth,
                            .framebuffer_width = @floatFromInt(target_size.width),
                            .framebuffer_height = @floatFromInt(target_size.height),
                        };

                        const slice_size = render_area.height / self.oit_horizontal_slices;
                        for (0..self.oit_horizontal_slices) |i| {
                            const start_y: u32 = @as(u32, @intCast(i)) * slice_size;

                            // Render Translucent Modifier Volumes
                            if (ta_lists.translucent_modifier_volumes.items.len > 0) {
                                const oit_mv_uniform_mem = gctx.uniformsAllocate(OITTMVUniforms, 1);
                                oit_mv_uniform_mem.slice[0] = .{
                                    .square_size = @intCast(self.translucent_modvol_dimensions().square_size),
                                    .pixels_per_slice = @intCast(self.translucent_modvol_dimensions().pixels_per_slice),
                                    .target_width = render_area.width,
                                    .start_y = start_y,
                                };

                                {
                                    const pass = encoder.beginRenderPass(.{
                                        .label = .init("Translucent Modifier Volumes"),
                                        .color_attachment_count = 0,
                                        .color_attachments = null,
                                        .depth_stencil_attachment = &.{
                                            .view = depth_view,
                                            .depth_read_only = .true,
                                            .stencil_read_only = .true,
                                        },
                                    });
                                    defer {
                                        pass.end();
                                        pass.release();
                                    }
                                    pass.setPipeline(translucent_modvol_pipeline);
                                    pass.setVertexBuffer(0, modifier_volume_vb_info.gpuobj.?, 0, modifier_volume_vb_info.size);
                                    pass.setBindGroup(0, modifier_volume_bind_group, &.{vs_mv_uniform_mem.offset});
                                    pass.setScissorRect(0, start_y, render_area.width, slice_size);

                                    // Close volume pass.
                                    var volume_index: u32 = 0;
                                    for (ta_lists.translucent_modifier_volumes.items) |volume| {
                                        if (volume.instructions.volume_instruction == .OutsideLastPolygon) if (Once(@src())) log.warn(termcolor.yellow("TODO: Unimplemented Exclusion Translucent Modifier Volume."), .{});
                                        if (volume.closed) {
                                            const oit_fs_uniform_mem = gctx.uniformsAllocate(struct { volume_index: u32 }, 1);
                                            oit_fs_uniform_mem.slice[0] = .{ .volume_index = volume_index };
                                            pass.setBindGroup(1, translucent_modvol_bind_group, &.{ oit_mv_uniform_mem.offset, oit_fs_uniform_mem.offset });
                                            volume_index += 1;

                                            pass.draw(3 * volume.triangle_count, 1, 3 * volume.first_triangle_index, 0);
                                        } else {
                                            log.warn(termcolor.yellow("TODO: Unhandled Open Translucent Modifier Volume!"), .{});
                                            // TODO: Almost the same thing, but the compute shader is really simple: Take the smallest
                                            //       depth value and add a volume from it to "infinity" (1.0+ depth). Or find a more efficient way :)
                                        }
                                    }
                                }

                                {
                                    const pass = encoder.beginComputePass(.{ .label = .init("Merge Modifier Volumes"), .timestamp_writes = null });
                                    defer {
                                        pass.end();
                                        pass.release();
                                    }
                                    const group_size = .{ 8, 8 };
                                    const num_groups = .{
                                        (@min(render_area.width, self.resolution.width) - 1) / group_size[0] + 1,
                                        (@min(render_area.height, self.resolution.height) - 1) / (self.oit_horizontal_slices * group_size[1]) + 1,
                                    };
                                    pass.setPipeline(translucent_modvol_merge_pipeline);

                                    pass.setBindGroup(0, translucent_modvol_merge_bind_group, &.{oit_mv_uniform_mem.offset});
                                    pass.dispatchWorkgroups(num_groups[0], num_groups[1], 1);
                                }
                            }

                            const oit_uniform_mem = gctx.uniformsAllocate(OITUniforms, 1);
                            oit_uniform_mem.slice[0] = .{
                                .max_fragments = @intCast(self.get_fragments_list_size() / OITLinkedListNodeSize),
                                .target_width = render_area.width,
                                .start_y = start_y,
                            };

                            if (gctx.lookupResource(self.translucent_pipeline)) |pipeline| {
                                const oit_color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                                    .view = target.resized.view,
                                    .load_op = .load,
                                    .store_op = .store,
                                }};
                                const pass = encoder.beginRenderPass(.{
                                    .label = .init("Translucent Pass"),
                                    .color_attachment_count = oit_color_attachments.len,
                                    .color_attachments = &oit_color_attachments,
                                    .depth_stencil_attachment = null, // TODO: Use the depth buffer rather than discarding the fragments manually?
                                });
                                defer {
                                    pass.end();
                                    pass.release();
                                }

                                pass.setPipeline(pipeline);

                                pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                                pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                                pass.setBindGroup(0, textures_bind_group, &.{uniform_mem.offset});
                                pass.setBindGroup(2, translucent_bind_group, &.{oit_uniform_mem.offset});

                                for (render_pass.translucent_pass.steps.items) |step| {
                                    var it = step.iterator();
                                    while (it.next()) |entry| {
                                        if (entry.value_ptr.draw_calls.count() > 0) {
                                            for (entry.value_ptr.draw_calls.values()) |draw_call| {
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
                            }

                            // Blend the results of the translucent pass
                            if (gctx.lookupResource(self.blend_pipeline)) |pipeline| {
                                const pass = encoder.beginComputePass(null);
                                defer {
                                    pass.end();
                                    pass.release();
                                }
                                const group_size = .{ 8, 4 };
                                const num_groups = .{
                                    (@min(render_area.width, self.resolution.width) - 1) / group_size[0] + 1,
                                    (@min(render_area.height, self.resolution.height) - 1) / (self.oit_horizontal_slices * group_size[1]) + 1,
                                };
                                pass.setPipeline(pipeline);
                                pass.setBindGroup(0, blend_bind_group, &.{oit_uniform_mem.offset});
                                pass.dispatchWorkgroups(num_groups[0], num_groups[1], 1);
                            }
                        }
                    }
                }
            }

            // Blit to native resolution framebuffer texture
            //   opaque_blit: Don't copy the alpha channel if write back is in a format that doesn't support it.
            //   Rendering to a texture normally involves converting to the specified "packmode". When "RenderToVRAM" is disabled,
            //   this conversion is skipped entirely. This is an issue in Virtua Tennis 2 where the replay are visibly darker when avoiding the roundtrip via guest VRAM.
            //   Loosing the alpha channel is enough in this case, but *technically* a proper fix would be to perform the conversion in a compute shader and back to BGRA.
            //   This seems a bit pointless when avoiding writing to guest VRAM is only an optimization.
            const opaque_blit = switch (self.write_back_parameters.w_ctrl.fb_packmode) {
                .KRGB0555, .RGB565, .RGB888, .KRGB0888 => true,
                .ARGB4444, .ARGB1555, .ARGB8888, .Reserved => false,
            };
            if (gctx.lookupResource(if (opaque_blit) self.blit_opaque_pipeline else self.blit_pipeline)) |pipeline| {
                const blit_vb_info = gctx.lookupResourceInfo(self.blit_vertex_buffer).?;
                const blit_ib_info = gctx.lookupResourceInfo(self.blit_index_buffer).?;
                const blit_bind_group = gctx.lookupResource(if (render_to_texture) self.blit_bind_group_render_to_texture else self.blit_bind_group).?;

                const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                    .view = target.native.view,
                    .load_op = .clear,
                    .store_op = .store,
                }};
                const pass = encoder.beginRenderPass(.{
                    .label = .init("Framebuffer Blit"),
                    .color_attachment_count = color_attachments.len,
                    .color_attachments = &color_attachments,
                });
                defer {
                    pass.end();
                    pass.release();
                }

                const blit_uniform_mem = gctx.uniformsAllocate(BlitUniforms, 1);
                blit_uniform_mem.slice[0] = .{
                    .min = .{ 0, 0 },
                    .max = .{ 1.0, 1.0 },
                };

                pass.setVertexBuffer(0, blit_vb_info.gpuobj.?, 0, blit_vb_info.size);
                pass.setIndexBuffer(blit_ib_info.gpuobj.?, .uint32, 0, blit_ib_info.size);

                pass.setPipeline(pipeline);

                pass.setBindGroup(0, blit_bind_group, &.{blit_uniform_mem.offset});
                pass.drawIndexed(4, 1, 0, 0, 0);
            }

            if (self.ExperimentalFramebufferEmulation or render_to_texture) {
                // FIXME: Could technically go up to 1024 when rendering to a texture... Don't know if any game actualy does it.
                //        Using the framebuffer size allow reusing some textures and buffers from the regular pipeline.
                const width: u32 = @min(self.global_clip.x.max, NativeResolution.width);
                const height: u32 = @min(self.global_clip.y.max, NativeResolution.height);
                // Skips the CPU writeback and copy directly to a host texture slot.
                if (!self.ExperimentalRenderToVRAM) {
                    // TODO: How is the minimum value of the global_clip used exactly? (Find a game where it isn't just [0, 0].)
                    if (self.global_clip.x.min != 0 or self.global_clip.y.min != 0)
                        log.warn("Render to texture with unusual global_clip:  [{d},{d}] to [{d},{d}]", .{ self.global_clip.x.min, self.global_clip.y.min, self.global_clip.x.max, self.global_clip.y.max });
                    const u_size: u3 = @intCast(std.math.log2(try std.math.ceilPowerOfTwo(u32, self.global_clip.x.max >> 3)));
                    const v_size: u3 = @intCast(std.math.log2(try std.math.ceilPowerOfTwo(u32, self.global_clip.y.max >> 3)));
                    const size_index: u3 = @max(u_size, v_size);
                    const interlaced = self.write_back_parameters.scaler_ctl.interlace;
                    const field = if (interlaced) self.write_back_parameters.scaler_ctl.field_select else 0;
                    const FB_W_SOF = if (field == 0) self.write_back_parameters.fb_w_sof1 else self.write_back_parameters.fb_w_sof2;
                    const addr = FB_W_SOF & HollyModule.Holly.VRAMMask;
                    const pixel_size: u32 = 2;
                    const end_address = addr + pixel_size * width * height;

                    var texture_index: TextureIndex = InvalidTextureIndex;
                    for (0..self.texture_metadata[size_index].len) |i| {
                        // Prefer slots that have never been used to keep textures in the cache for as long as possible.
                        if (self.texture_metadata[size_index][i].status == .Invalid) {
                            if (texture_index == InvalidTextureIndex)
                                texture_index = @as(TextureIndex, @intCast(i));
                        } else {
                            // Replace texture slot previously used by this mechanism.
                            if (self.texture_metadata[size_index][i].start_address == addr) {
                                texture_index = @as(TextureIndex, @intCast(i));
                                break;
                            }
                        }
                    }
                    if (texture_index == InvalidTextureIndex)
                        texture_index = self.get_lru_texture_index(size_index);

                    if (texture_index != InvalidTextureIndex) {
                        encoder.copyTextureToTexture(
                            .{
                                .texture = target.native.texture,
                                .mip_level = 0,
                                .origin = .{},
                                .aspect = .all,
                            },
                            .{
                                .texture = self._gctx.lookupResource(self.texture_arrays[size_index].texture).?,
                                .mip_level = 0,
                                .origin = .{ .z = @intCast(texture_index) },
                                .aspect = .all,
                            },
                            .{
                                .width = width,
                                .height = height,
                                .depth_or_array_layers = 1,
                            },
                        );
                        // FIXME: All of these settings are those used by Virtual Tennis 2 during replay.
                        //        I need to find a better way to find them at runtime.
                        //        (Wait the next frame, look for this specific address, and update them 'just in time'?
                        //          Add a flag to the slot to bypass all settings except the address?).
                        const texture_control_word = HollyModule.TextureControlWord{
                            .address = @intCast(addr >> 3),
                            .stride_select = 1,
                            .scan_order = 1,
                            .pixel_format = switch (self.write_back_parameters.w_ctrl.fb_packmode) {
                                .KRGB0555 => .ARGB1555,
                                .RGB565 => .RGB565,
                                .ARGB4444 => .ARGB4444,
                                .ARGB1555 => .ARGB1555,
                                else => .BumpMap, // Other modes can't be used to write to a texture map.
                            },
                            .vq_compressed = 0,
                            .mip_mapped = 0,
                        };
                        const tsp_instruction = HollyModule.TSPInstructionWord{
                            .texture_v_size = v_size,
                            .texture_u_size = u_size,
                            .texture_shading_instruction = .ModulateAlpha,
                            .mipmap_d_adjust = 0,
                            .supersample_texture = 0,
                            .filter_mode = .Point,
                            .clamp_uv = .{ .u = false, .v = false },
                            .flip_uv = .{ .u = false, .v = false },
                            .ignore_texture_alpha = 0,
                            .use_alpha = 1,
                            .color_clamp = 0,
                            .fog_control = .LookUpTable,
                            .dst_select = 0,
                            .src_select = 0,
                            .dst_alpha_instr = .InverseSourceAlpha,
                            .src_alpha_instr = .SourceAlpha,
                        };
                        self.texture_metadata[size_index][texture_index] = .{
                            .status = .Used,
                            .control_word = texture_control_word,
                            .tsp_instruction = tsp_instruction,
                            .index = texture_index,
                            .usage = 1,
                            .size = .{ u_size, v_size },
                            .start_address = addr,
                            .end_address = end_address,
                            .hash = texture_hash(holly, addr, end_address),
                        };
                    }
                } else {
                    // Otherwise copy to a buffer we'll read right after.
                    encoder.copyTextureToBuffer(
                        .{
                            .texture = target.native.texture,
                            .mip_level = 0,
                            .origin = .{},
                            .aspect = .all,
                        },
                        .{
                            .layout = .{
                                .offset = 0,
                                .bytes_per_row = 4 * width,
                                .rows_per_image = height,
                            },
                            .buffer = gctx.lookupResource(self.framebuffer_copy_buffer).?,
                        },
                        .{
                            .width = width,
                            .height = height,
                            .depth_or_array_layers = 1,
                        },
                    );
                }
            }

            break :commands encoder.finish(null);
        };
        defer commands.release();

        gctx.submit(&.{commands});

        // Read the result and copy it to guest VRAM
        if ((self.ExperimentalFramebufferEmulation or render_to_texture) and self.ExperimentalRenderToVRAM) {
            const static = struct {
                var result: zgpu.wgpu.MapAsyncStatus = .@"error";
                fn signal_fb_mapped(status: zgpu.wgpu.MapAsyncStatus, message: zgpu.wgpu.StringView.C, _: ?*anyopaque, _: ?*anyopaque) callconv(.c) void {
                    result = status;
                    if (status != .success)
                        log.err(termcolor.red("Failed to map framebuffer: {t} ({s})"), .{ status, message.data[0..message.length] });
                }
            };

            const width: u32 = @min(self.global_clip.x.max, NativeResolution.width);
            const height: u32 = @min(self.global_clip.y.max, NativeResolution.height);

            const copy_buffer = gctx.lookupResource(self.framebuffer_copy_buffer).?;
            const future = copy_buffer.mapAsync(.{ .read = true }, 0, 4 * width * height, .{ .callback = &static.signal_fb_mapped, .mode = .wait_any_only });
            // Wait for mapping to be available. There's no synchronous way to do that AFAIK.
            // It needs to be unmapped before the next frame.
            var wait_info = [_]wgpu.FutureWaitInfo{.{ .future = future }};
            if (gctx.instance.waitAny(&wait_info, std.math.maxInt(u64)) == .success) {
                if (static.result == .success) {
                    defer copy_buffer.unmap();
                    const mapped_pixels = copy_buffer.getConstMappedRange(u8, 0, 4 * width * height);
                    if (mapped_pixels) |pixels| {
                        holly.write_framebuffer(self.write_back_parameters, .{ .width = width, .height = height }, pixels);
                    } else log.err(termcolor.red("Failed to map framebuffer"), .{});
                } // Error logged in the callback
            } else log.err("Failed to wait for framebuffer mapping to be available.", .{});
        }

        if (!render_to_texture) {
            const now = std.time.microTimestamp();
            self.last_n_frametimes.push(now - self.last_frame_timestamp);
            self.last_frame_timestamp = now;
        }
    }

    /// Blit the last rendered frame to the window surface.
    /// Locks _gctx_queue_mutex.
    pub fn draw(self: *const @This(), display_mode: DisplayMode, window_width: u32, window_height: u32) void {
        self._gctx_queue_mutex.lock();
        defer self._gctx_queue_mutex.unlock();

        self.update_blit_to_screen_vertex_buffer(window_width, window_height, display_mode);

        if (self._gctx.lookupResource(self.blit_pipeline)) |pipeline| {
            var surface: zgpu.wgpu.SurfaceTexture = undefined;
            self._gctx.surface.getCurrentTexture(&surface);
            defer surface.texture.?.release();
            const back_buffer_view = surface.texture.?.createView(.{});
            defer back_buffer_view.release();

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
                    const pass = encoder.beginRenderPass(.{
                        .label = .init("Final Blit"),
                        .color_attachment_count = color_attachments.len,
                        .color_attachments = &color_attachments,
                    });
                    defer {
                        pass.end();
                        pass.release();
                    }

                    const width: f32 = @floatFromInt(self.output_resolution.width);
                    const height: f32 = @floatFromInt(self.output_resolution.height);

                    const blit_uniform_mem = self._gctx.uniformsAllocate(BlitUniforms, 1);
                    blit_uniform_mem.slice[0] = .{
                        .min = .{ 0, 0 },
                        .max = .{
                            width / NativeResolution.width,
                            height / NativeResolution.height,
                        },
                    };

                    pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                    pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                    pass.setPipeline(pipeline);

                    pass.setBindGroup(0, blit_bind_group, &.{blit_uniform_mem.offset});
                    pass.drawIndexed(4, 1, 0, 0, 0);
                }

                break :commands encoder.finish(null);
            };
            defer commands.release();

            self._gctx.submit(&.{commands});
        }
    }

    fn get_or_put_pipeline(self: *@This(), pipeline_key: PipelineKey, sync: enum { Sync, Async }) zgpu.RenderPipelineHandle {
        var key = pipeline_key;
        // Deduplicate culling modes None and Small: They resolve to the same pipeline for the renderer.
        if (key.culling_mode == .Small) key.culling_mode = .None;

        if (self.opaque_pipelines.get(key)) |pl|
            return pl;

        log.info("Creating Pipeline: {f}", .{key});
        const start = std.time.milliTimestamp();

        const color_targets = [_]wgpu.ColorTargetState{
            .{
                .format = zgpu.GraphicsContext.surface_texture_format,
                .blend = &.{
                    .color = .{ .operation = .add, .src_factor = key.src_blend_factor, .dst_factor = key.dst_blend_factor },
                    .alpha = .{ .operation = .add, .src_factor = key.src_blend_factor, .dst_factor = key.dst_blend_factor }, // FIXME: Not sure about this.
                },
            },
            .{
                .format = zgpu.GraphicsContext.surface_texture_format,
                .blend = &.{
                    .color = .{ .operation = .add, .src_factor = key.src_blend_factor, .dst_factor = key.dst_blend_factor },
                    .alpha = .{ .operation = .add, .src_factor = key.src_blend_factor, .dst_factor = key.dst_blend_factor },
                },
            },
        };

        const pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = .{
                .module = self.opaque_vertex_shader_module,
                .entry_point = .init("main"),
                .buffer_count = VertexBufferLayout.len,
                .buffers = &VertexBufferLayout,
            },
            .primitive = .{
                .front_face = .ccw,
                .cull_mode = switch (key.culling_mode) {
                    .None => .none,
                    .Small => .none,
                    .Positive => .back,
                    .Negative => .front,
                },
                .topology = .triangle_strip,
                .strip_index_format = .uint32,
            },
            .depth_stencil = &.{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = if (key.depth_write_enabled) .true else .false,
                .depth_compare = key.depth_compare,
            },
            .fragment = &.{
                .module = if (key.translucent) self.pre_sort_fragment_shader_module else self.opaque_fragment_shader_module,
                .entry_point = .init("main"),
                .target_count = color_targets.len,
                .targets = &color_targets,
            },
        };

        switch (sync) {
            .Async => {
                // Experiment: Asynchronous pipeline creation
                self.opaque_pipelines.putAssumeCapacityNoClobber(key, .{});
                const ptr = self.opaque_pipelines.getPtr(key).?;
                _ = self._gctx.createRenderPipelineAsync(self._allocator, self.opaque_pipeline_layout, pipeline_descriptor, ptr) catch |err| std.debug.panic("Failed to create pipeline: {t}", .{err});
                return ptr.*;
            },
            else => {
                defer log.info("Pipeline created in {d}ms", .{std.time.milliTimestamp() - start});
                const pl = self._gctx.createRenderPipeline(self.opaque_pipeline_layout, pipeline_descriptor);

                if (!self._gctx.isResourceValid(pl)) {
                    log.err("Error creating pipeline.", .{});
                    log.err("{any}", .{pipeline_descriptor});
                }

                self.opaque_pipelines.putAssumeCapacityNoClobber(key, pl);

                return pl;
            },
        }
    }

    fn deinit_screen_textures(self: *@This()) void {
        self._gctx.releaseResource(self.depth.depth_only_view);
        self._gctx.releaseResource(self.depth.view);
        self._gctx.destroyResource(self.depth.texture);

        self.resized_framebuffer.release(self._gctx);
        self.resized_framebuffer_copy.release(self._gctx);
        self.resized_render_to_texture_target.release(self._gctx);

        self._gctx.releaseResource(self.blit_bind_group);
        self._gctx.releaseResource(self.blit_bind_group_render_to_texture);

        self._gctx.releaseResource(self.list_heads_buffer);
        self._gctx.releaseResource(self.linked_list_buffer);

        self._gctx.releaseResource(self.translucent_modvol_fragment_counts_buffer);
        self._gctx.releaseResource(self.translucent_modvol_fragment_list_buffer);
        self._gctx.releaseResource(self.translucent_modvol_volumes_buffer);

        self._gctx.releaseResource(self.translucent_bind_group);
        self._gctx.releaseResource(self.translucent_modvol_bind_group);
        self._gctx.releaseResource(self.translucent_modvol_merge_bind_group);

        self._gctx.releaseResource(self.blend_bind_group);
        self._gctx.releaseResource(self.blend_bind_group_render_to_texture);
    }

    /// Creates all resources that depends on the render size
    pub fn on_inner_resolution_change(self: *@This(), scaling_filter: Filter) void {
        self.deinit_screen_textures();

        // This is currently the largest buffer whose size is dependent on the resolution. Make sure we can allocate it.
        if (self.translucent_modvol_dimensions().fragment_list_buffer_size > self.get_max_storage_buffer_binding_size()) {
            self.oit_horizontal_slices = 1;
            // Vertical resolution has to be divisible by 8 * horizontal slices because of the compute shaders workgroup sizes.
            while (self.translucent_modvol_dimensions().fragment_list_buffer_size > self.get_max_storage_buffer_binding_size() or self.resolution.height % (8 * self.oit_horizontal_slices) != 0) {
                self.oit_horizontal_slices += 1;
            }
        }

        // Create a new depth texture to match the new render size.
        const depth = create_depth_texture(self._gctx, self.resolution);
        self.depth.texture = depth.texture;
        self.depth.view = depth.view;
        self.depth.depth_only_view = depth.depth_only_view;

        // Same thing for our screen size framebuffer.
        self.resized_framebuffer = create_resized_framebuffer_texture(self._gctx, self.resolution, true, false);
        self.resized_framebuffer_area1 = create_resized_framebuffer_texture(self._gctx, self.resolution, false, false);
        self.resized_framebuffer_copy = create_resized_framebuffer_texture(self._gctx, self.resolution, false, true);
        // Intermediate buffer used when rendering to a texture. We don't want to override resized_framebuffer since it is used for blitting.
        self.resized_render_to_texture_target = create_resized_framebuffer_texture(self._gctx, self.resolution, true, false);

        self.create_blit_bind_groups(scaling_filter);

        const mv_apply_bind_group_layout = self._gctx.createBindGroupLayout(&ModifierVolumeApplyBindGroupLayout, .{ .label = "ModifierVolumeApplyBindGroupLayout" });
        defer self._gctx.releaseResource(mv_apply_bind_group_layout);

        self.modifier_volume_apply_bind_group = self._gctx.createBindGroup(mv_apply_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = self.resized_framebuffer_area1.view },
            .{ .binding = 2, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(BlitUniforms) },
        });

        self.create_oit_buffers();
        self.create_translucent_bind_group();
        self.create_translucent_modvol_bind_group();
        self.create_translucent_modvol_merge_bind_group();
        self.create_blend_bind_groups();
    }

    /// Returns the rendered frame as RGBA pixels.
    /// Locks self._gctx_queue_mutex.
    /// Caller owns the returned memory and should call `deinit` on the returned Image.
    pub fn capture(self: *const @This(), allocator: std.mem.Allocator) !@import("image.zig") {
        self._gctx_queue_mutex.lock();
        defer self._gctx_queue_mutex.unlock();

        const static = struct {
            var result: zgpu.wgpu.MapAsyncStatus = .@"error";
            fn signal_mapped(status: zgpu.wgpu.MapAsyncStatus, message: zgpu.wgpu.StringView.C, _: ?*anyopaque, _: ?*anyopaque) callconv(.c) void {
                result = status;
                if (status != .success)
                    log.err(termcolor.red("Failed to map buffer: {t} ({s})"), .{ status, message.data[0..message.length] });
            }
        };

        // Allocates a temporary buffer to copy the framebuffer to.
        const tmp_buffer_handle = self._gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .map_read = true },
            .size = 4 * self.resolution.width * self.resolution.height,
        });
        defer self._gctx.releaseResource(tmp_buffer_handle);
        const tmp_buffer = self._gctx.lookupResource(tmp_buffer_handle).?;

        const commands = commands: {
            const encoder = self._gctx.device.createCommandEncoder(null);
            defer encoder.release();
            encoder.copyTextureToBuffer(
                .{
                    .texture = self.resized_framebuffer.lookup(self._gctx).texture,
                    .mip_level = 0,
                    .origin = .{},
                    .aspect = .all,
                },
                .{
                    .layout = .{
                        .offset = 0,
                        .bytes_per_row = 4 * self.resolution.width,
                        .rows_per_image = self.resolution.height,
                    },
                    .buffer = tmp_buffer,
                },
                .{
                    .width = self.resolution.width,
                    .height = self.resolution.height,
                    .depth_or_array_layers = 1,
                },
            );
            break :commands encoder.finish(null);
        };
        defer commands.release();
        self._gctx.submit(&.{commands});

        const future = tmp_buffer.mapAsync(.{ .read = true }, 0, 4 * self.resolution.width * self.resolution.height, .{ .callback = &static.signal_mapped, .mode = .wait_any_only });
        var wait_info = [_]zgpu.wgpu.FutureWaitInfo{.{ .future = future }};
        if (self._gctx.instance.waitAny(&wait_info, std.math.maxInt(u64)) == .success) {
            if (static.result == .success) {
                defer tmp_buffer.unmap();
                const mapped_pixels = tmp_buffer.getConstMappedRange(u8, 0, 4 * self.resolution.width * self.resolution.height);
                if (mapped_pixels) |pixels| {
                    return .{
                        .width = self.resolution.width,
                        .height = self.resolution.height,
                        .bgra = try allocator.dupe(u8, pixels),
                    };
                } else return error.MapFailed;
            } else return error.MapFailed;
        } else return error.WaitFailed;
    }

    // Assumes gctx_queue_mutex is locked.
    pub fn update_blit_to_screen_vertex_buffer(self: *const @This(), width: u32, height: u32, display_mode: DisplayMode) void {
        const iw: f32 = @floatFromInt(self.resolution.width);
        const ih: f32 = @floatFromInt(self.resolution.height);
        const tw: f32 = @floatFromInt(width);
        const th: f32 = @floatFromInt(height);
        const ias = iw / ih;
        const tas = tw / th;
        const actual_dm = if (display_mode == .Center and (tw < iw or th < ih))
            .Fit
        else
            display_mode;
        var blit_vertex_data = switch (actual_dm) {
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
        const defaults =
            if (self.spg_control.PAL == 1)
                if (self.spg_control.interlace) HollyModule.VideoModes.PALInterlace else HollyModule.VideoModes.PAL
            else if (self.spg_control.NTSC == 1)
                if (self.spg_control.interlace) HollyModule.VideoModes.NTSCInterlace else HollyModule.VideoModes.NTSC
            else
                HollyModule.VideoModes.VGA;
        const pixel_shifts: [2]f32 = .{
            @as(f32, @floatFromInt(self.vo_startx.horizontal_start_position)) - @as(f32, @floatFromInt(defaults.vo_startx.horizontal_start_position)),
            @as(f32, @floatFromInt(self.vo_starty.vertical_start_position_on_field1)) - @as(f32, @floatFromInt(defaults.vo_starty.vertical_start_position_on_field1)),
        };
        const native_horizontal_resolution: f32 = if (self.write_back_parameters.video_out_ctrl.pixel_double) 320 else 640;
        const native_vertical_resolution: f32 = if ((self.spg_control.NTSC == 1 or self.spg_control.PAL == 1) and !self.spg_control.interlace) 240 else 480;
        for (0..4) |i| {
            blit_vertex_data[i * 4 + 0] += 2.0 * pixel_shifts[0] / native_horizontal_resolution;
            blit_vertex_data[i * 4 + 1] -= 2.0 * pixel_shifts[1] / native_vertical_resolution; // Coordinates are [-1, 1], with -1 being up.
        }
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
            .label = .init("Depth Texture"),
        });
        const view = gctx.createTextureView(texture, .{});
        const depth_only_view = gctx.createTextureView(texture, .{ .aspect = .depth_only });
        return .{ .texture = texture, .view = view, .depth_only_view = depth_only_view };
    }

    fn create_texture_cache_array(gctx: *zgpu.GraphicsContext, size_index: u64, layer_count: u64) TextureAndView {
        const pixel_size = @as(u32, 8) << @intCast(size_index);
        const texture = gctx.createTexture(.{
            .usage = .{
                .texture_binding = true,
                .storage_binding = true,
                .copy_dst = true,
                .copy_src = true, // Necessary for dynamically increasing the slot count.
            },
            .size = .{
                .width = pixel_size,
                .height = pixel_size,
                .depth_or_array_layers = @intCast(layer_count),
            },
            .format = .bgra8_unorm,
            .mip_level_count = @intCast(4 + size_index),
            .label = .init("Texture Cache Array"),
        });
        const view = gctx.createTextureView(texture, .{});
        return .{ .texture = texture, .view = view };
    }

    fn create_resized_framebuffer_texture(gctx: *zgpu.GraphicsContext, resolution: Resolution, copy_src: bool, copy_dst: bool) TextureAndView {
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
            .format = zgpu.GraphicsContext.surface_texture_format,
            .mip_level_count = 1,
            .label = .init("Resized Framebuffer Texture"),
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
                .label = .init("OIT List Heads Buffer"),
            });
            const init_buffer = self._gctx.lookupResourceInfo(self.list_heads_buffer).?.gpuobj.?;
            const mapped = init_buffer.getMappedRange(u32, 0, self.get_linked_list_heads_size() / @sizeOf(u32));
            @memset(mapped.?, 0xFFFFFFFF); // Set heads to invalid (or 'end-of-list')
            mapped.?[0] = 0; // Set fragment count to 0
            init_buffer.unmap();

            self.linked_list_buffer = self._gctx.createBuffer(.{
                .usage = .{ .copy_dst = true, .storage = true },
                .size = self.get_fragments_list_size(),
            });
            self._gctx.lookupResource(self.linked_list_buffer).?.setLabel("OIT Linked List Buffer");
        }

        // Mod Vol
        {
            self.translucent_modvol_fragment_counts_buffer = self._gctx.createBuffer(.{
                .usage = .{ .storage = true },
                .size = self.translucent_modvol_dimensions().fragment_counts_buffer_size,
                .mapped_at_creation = .true,
                .label = .init("Translucent ModVol Fragment Counts Buffer"),
            });
            {
                const init_buffer = self._gctx.lookupResourceInfo(self.translucent_modvol_fragment_counts_buffer).?.gpuobj.?;
                const mapped = init_buffer.getMappedRange(u32, 0, self.translucent_modvol_dimensions().pixels_per_slice);
                @memset(mapped.?, 0); // Set all fragment counts to 0
                init_buffer.unmap();
            }

            self.translucent_modvol_fragment_list_buffer = self._gctx.createBuffer(.{
                .usage = .{ .copy_dst = true, .storage = true },
                .size = self.translucent_modvol_dimensions().fragment_list_buffer_size,
                .label = .init("Translucent ModVol Fragment List Buffer"),
            });

            self.translucent_modvol_volumes_buffer = self._gctx.createBuffer(.{
                .usage = .{ .copy_dst = true, .storage = true },
                .size = self.translucent_modvol_dimensions().volumes_buffer_size,
                .mapped_at_creation = .true,
                .label = .init("Translucent ModVol Volumes Buffer"),
            });
            {
                const init_buffer = self._gctx.lookupResourceInfo(self.translucent_modvol_volumes_buffer).?.gpuobj.?;
                const mapped = init_buffer.getMappedRange(u8, 0, self.translucent_modvol_dimensions().volumes_buffer_size);
                @memset(mapped.?, 0); // Set all counts to 0
                init_buffer.unmap();
            }
        }
    }

    pub fn set_scaling_filter(self: *@This(), scaling_filter: Filter) void {
        self.create_blit_bind_groups(scaling_filter);
    }

    fn create_blit_bind_groups(self: *@This(), scaling_filter: Filter) void {
        if (self.framebuffer_resize_bind_group.id != 0) self._gctx.releaseResource(self.framebuffer_resize_bind_group);
        if (self.blit_bind_group.id != 0) self._gctx.releaseResource(self.blit_bind_group);
        if (self.blit_bind_group_render_to_texture.id != 0) self._gctx.releaseResource(self.blit_bind_group_render_to_texture);

        const blit_bind_group_layout = self._gctx.createBindGroupLayout(&BlitBindGroupLayout, .{ .label = "BlitBindGroupLayout" });
        defer self._gctx.releaseResource(blit_bind_group_layout);

        const sampler = switch (scaling_filter) {
            .Nearest => self.samplers[sampler_index(.nearest, .nearest, .nearest, .clamp_to_edge, .clamp_to_edge)],
            .Linear => self.samplers[sampler_index(.linear, .linear, .linear, .clamp_to_edge, .clamp_to_edge)],
        };

        self.framebuffer_resize_bind_group = self._gctx.createBindGroup(blit_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = self.framebuffer.view },
            .{ .binding = 1, .sampler_handle = sampler },
            .{ .binding = 2, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(BlitUniforms) },
        });

        self.blit_bind_group = self._gctx.createBindGroup(blit_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = self.resized_framebuffer.view },
            .{ .binding = 1, .sampler_handle = sampler },
            .{ .binding = 2, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(BlitUniforms) },
        });
        self.blit_bind_group_render_to_texture = self._gctx.createBindGroup(blit_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = self.resized_render_to_texture_target.view },
            .{ .binding = 1, .sampler_handle = sampler },
            .{ .binding = 2, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(BlitUniforms) },
        });
    }

    fn create_translucent_bind_group(self: *@This()) void {
        self.translucent_bind_group = self._gctx.createBindGroup(self.translucent_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(OITUniforms) },
            .{ .binding = 1, .buffer_handle = self.list_heads_buffer, .offset = 0, .size = self.get_linked_list_heads_size() },
            .{ .binding = 2, .buffer_handle = self.linked_list_buffer, .offset = 0, .size = self.get_fragments_list_size() },
            .{ .binding = 3, .texture_view_handle = self.depth.depth_only_view },
        });
    }

    fn create_translucent_modvol_bind_group(self: *@This()) void {
        self.translucent_modvol_bind_group = self._gctx.createBindGroup(self.translucent_modvol_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(OITTMVUniforms) },
            .{ .binding = 1, .buffer_handle = self.translucent_modvol_fragment_counts_buffer, .offset = 0, .size = self.translucent_modvol_dimensions().fragment_counts_buffer_size },
            .{ .binding = 2, .buffer_handle = self.translucent_modvol_fragment_list_buffer, .offset = 0, .size = self.translucent_modvol_dimensions().fragment_list_buffer_size },
            .{ .binding = 3, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = 1 * @sizeOf(u32) },
            .{ .binding = 4, .texture_view_handle = self.depth.depth_only_view },
        });
    }
    fn create_translucent_modvol_merge_bind_group(self: *@This()) void {
        self.translucent_modvol_merge_bind_group = self._gctx.createBindGroup(self.translucent_modvol_merge_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(OITTMVUniforms) },
            .{ .binding = 1, .buffer_handle = self.translucent_modvol_fragment_counts_buffer, .offset = 0, .size = self.translucent_modvol_dimensions().fragment_counts_buffer_size },
            .{ .binding = 2, .buffer_handle = self.translucent_modvol_fragment_list_buffer, .offset = 0, .size = self.translucent_modvol_dimensions().fragment_list_buffer_size },
            .{ .binding = 3, .buffer_handle = self.translucent_modvol_volumes_buffer, .offset = 0, .size = self.translucent_modvol_dimensions().volumes_buffer_size },
        });
    }

    fn create_blend_bind_groups(self: *@This()) void {
        self.blend_bind_group = self._gctx.createBindGroup(self.blend_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(OITUniforms) },
            .{ .binding = 1, .buffer_handle = self.list_heads_buffer, .offset = 0, .size = self.get_linked_list_heads_size() },
            .{ .binding = 2, .buffer_handle = self.linked_list_buffer, .offset = 0, .size = self.get_fragments_list_size() },
            .{ .binding = 3, .texture_view_handle = self.resized_framebuffer_copy.view },
            .{ .binding = 4, .texture_view_handle = self.resized_framebuffer.view },
            .{ .binding = 5, .buffer_handle = self.translucent_modvol_volumes_buffer, .offset = 0, .size = self.translucent_modvol_dimensions().volumes_buffer_size },
        });
        self.blend_bind_group_render_to_texture = self._gctx.createBindGroup(self.blend_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(OITUniforms) },
            .{ .binding = 1, .buffer_handle = self.list_heads_buffer, .offset = 0, .size = self.get_linked_list_heads_size() },
            .{ .binding = 2, .buffer_handle = self.linked_list_buffer, .offset = 0, .size = self.get_fragments_list_size() },
            .{ .binding = 3, .texture_view_handle = self.resized_framebuffer_copy.view },
            .{ .binding = 4, .texture_view_handle = self.resized_render_to_texture_target.view },
            .{ .binding = 5, .buffer_handle = self.translucent_modvol_volumes_buffer, .offset = 0, .size = self.translucent_modvol_dimensions().volumes_buffer_size },
        });
    }

    fn get_linked_list_heads_size(self: *const @This()) u64 {
        return (1 + self.resolution.width * self.resolution.height / self.oit_horizontal_slices) * @sizeOf(u32);
    }

    fn get_fragments_list_size(self: *const @This()) u64 {
        return self.get_max_storage_buffer_binding_size();
    }

    inline fn translucent_modvol_dimensions(self: *const @This()) struct { square_size: u64, square_count: u64, pixels_per_slice: u64, fragment_counts_buffer_size: u64, fragment_list_buffer_size: u64, volumes_buffer_size: u64 } {
        std.debug.assert(self.resolution.width >= self.resolution.height / self.oit_horizontal_slices); // This is dealt with on the Zig size, but the shaders assumes slices are wider than they are tall.
        const square_size = std.math.ceilPowerOfTwo(u64, @min(self.resolution.width, self.resolution.height / self.oit_horizontal_slices)) catch unreachable;
        const large_side = @max(self.resolution.width, self.resolution.height / self.oit_horizontal_slices);
        const square_count = if (large_side % square_size == 0) large_side / square_size else 1 + large_side / square_size;
        return .{
            .square_size = square_size,
            .square_count = square_count,
            .pixels_per_slice = square_size * square_size * square_count,
            .fragment_counts_buffer_size = @sizeOf(u32) * square_count * square_size * square_size,
            .fragment_list_buffer_size = VolumeFragmentSize * square_count * square_size * square_size * MaxVolumeFragmentsPerPixel,
            .volumes_buffer_size = VolumePixelSize * (self.resolution.width * self.resolution.height / self.oit_horizontal_slices),
        };
    }

    fn get_max_storage_buffer_binding_size(self: *const @This()) u64 {
        var limits: zgpu.wgpu.Limits = .{};
        if (!self._gctx.device.getLimits(&limits)) {
            log.err("get_max_storage_buffer_binding_size: Failed to get device limits.", .{});
            return 134217728; // Min WebGPU spec.
        }
        return limits.max_storage_buffer_binding_size;
    }

    /// Assumes gctx_queue_mutex is locked.
    fn increase_texture_slot_count(self: *@This(), size_index: u32, new_count: u64) void {
        const slot_count = self.texture_metadata[size_index].len;
        std.debug.assert(new_count > slot_count);

        const size = @as(u32, 8) << @intCast(size_index);
        const mip_level_count = 4 + @as(u32, size_index);
        const arr = create_texture_cache_array(self._gctx, size_index, new_count);

        // Copy previous textures
        const commands = commands: {
            const encoder = self._gctx.device.createCommandEncoder(null);
            defer encoder.release();
            const source = self.texture_arrays[size_index].lookup(self._gctx).texture;
            const destination = arr.lookup(self._gctx).texture;
            for (0..mip_level_count) |mip_level_u64| {
                const mip_level: u32 = @intCast(mip_level_u64);
                encoder.copyTextureToTexture(
                    .{
                        .texture = source,
                        .mip_level = mip_level,
                        .origin = .{},
                        .aspect = .all,
                    },
                    .{
                        .texture = destination,
                        .mip_level = mip_level,
                        .origin = .{},
                        .aspect = .all,
                    },
                    .{
                        .width = size / std.math.pow(u32, 2, mip_level),
                        .height = size / std.math.pow(u32, 2, mip_level),
                        .depth_or_array_layers = @intCast(slot_count),
                    },
                );
            }
            break :commands encoder.finish(null);
        };
        defer commands.release();

        self._gctx.submit(&.{commands});

        const prev_texture = self.texture_arrays[size_index];
        self.texture_arrays[size_index] = arr;
        defer prev_texture.release(self._gctx);

        const tmp = self._allocator.alloc(TextureMetadata, new_count) catch std.debug.panic("Out of memory.", .{});
        for (0..slot_count) |i| tmp[i] = self.texture_metadata[size_index][i];
        for (slot_count..new_count) |i| tmp[i] = .{};
        const prev_metadata = self.texture_metadata[size_index];
        self.texture_metadata[size_index] = tmp;
        defer self._allocator.free(prev_metadata);

        self.create_textures_bind_group();
    }
};
