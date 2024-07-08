const std = @import("std");

const renderer_log = std.log.scoped(.renderer);

const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;

const common = @import("common.zig");
const termcolor = @import("termcolor");

const HollyModule = @import("holly.zig");

// First 1024 values of the Moser de Bruijin sequence, Textures on the dreamcast are limited to 1024*1024 pixels.
var moser_de_bruijin_sequence: [1024]u32 = .{0} ** 1024;

// Returns the indice of the z-order curve for the given coordinates.
pub fn zorder_curve(x: u32, y: u32) u32 {
    return (moser_de_bruijin_sequence[x] << 1) | moser_de_bruijin_sequence[y];
}

pub fn to_twiddled_index(i: u32, w: u32) u32 {
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
pub fn untwiddle(u: u32, v: u32, w: u32, h: u32) u32 {
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

fn uv16(val: u16) f32 {
    return @bitCast(@as(u32, val) << 16);
}

// FIXME: This is way too slow.
fn texture_hash(gpu: *HollyModule.Holly, start: u32, end: u32) u64 {
    return std.hash.CityHash64.hash(gpu.vram[start & 0xFFFFFFC .. end & 0xFFFFFFC]);
}

const fRGBA = packed struct {
    r: f32 = 0,
    g: f32 = 0,
    b: f32 = 0,
    a: f32 = 0,

    pub fn fromPacked(color: HollyModule.PackedColor) fRGBA {
        return .{
            .r = @as(f32, @floatFromInt(color.r)) / 255.0,
            .g = @as(f32, @floatFromInt(color.g)) / 255.0,
            .b = @as(f32, @floatFromInt(color.b)) / 255.0,
            .a = @as(f32, @floatFromInt(color.a)) / 255.0,
        };
    }
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
    _: u8 = 0,
};

fn sampler_index(mag_filter: wgpu.FilterMode, min_filter: wgpu.FilterMode, mipmap_filter: wgpu.MipmapFilterMode, address_mode_u: wgpu.AddressMode, address_mode_v: wgpu.AddressMode) u8 {
    return @as(u8, @truncate(@intFromEnum(address_mode_v))) << 5 |
        @as(u8, @truncate(@intFromEnum(address_mode_u))) << 3 |
        @as(u8, @truncate(@intFromEnum(mipmap_filter))) << 2 |
        @as(u8, @truncate(@intFromEnum(min_filter))) << 1 |
        @as(u8, @truncate(@intFromEnum(mag_filter)));
}

const TextureIndex = u32;
const InvalidTextureIndex = std.math.maxInt(TextureIndex);

const VertexTextureInfo = packed struct {
    index: TextureIndex,
    shading: ShadingInstructions,
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
    base_color: packed struct {
        r: f32,
        g: f32,
        b: f32,
        a: f32,
    },
    offset_color: packed struct {
        r: f32 = 0,
        g: f32 = 0,
        b: f32 = 0,
        a: f32 = 0,
    } = .{},
    u: f32 = 0.0,
    v: f32 = 0.0,
    tex: VertexTextureInfo,
};

const wgsl_vs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/vs.wgsl");
const wgsl_fs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/fragment_color.wgsl") ++ @embedFile("./shaders/fs.wgsl");
const wgsl_translucent_fs = @embedFile("./shaders/uniforms.wgsl") ++ @embedFile("./shaders/fragment_color.wgsl") ++ @embedFile("./shaders/oit_draw_fs.wgsl");
const wgsl_blend_cs = @embedFile("./shaders/oit_blend_cs.wgsl");
const wgsl_modifier_volume_vs = @embedFile("./shaders/modifier_volume_vs.wgsl");
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
    palette_hash: u32 = 0,
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

// NOTE: Holly assumes 1/z depth values, we re-invert them in the vertex shader,
//       so we also need to invert the compare modes; this is not a mistake.
fn translate_depth_compare_mode(mode: HollyModule.DepthCompareMode) wgpu.CompareFunction {
    return switch (mode) {
        .Never => .never,
        .Less => .greater,
        .Equal => .equal,
        .LessEqual => .greater_equal,
        .Greater => .less,
        .NotEqual => .not_equal,
        .GreaterEqual => .less_equal,
        .Always => .always,
    };
}

const PipelineKey = struct {
    src_blend_factor: wgpu.BlendFactor,
    dst_blend_factor: wgpu.BlendFactor,
    depth_compare: wgpu.CompareFunction,
    depth_write_enabled: bool,
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
};

fn gen_sprite_vertices(sprite: HollyModule.VertexParameter) [4]Vertex {
    var r: [4]Vertex = undefined;

    // B --- C
    // |  \  |
    // A --- D
    // Pushing the vertices in CCW order: A, D, B, C

    switch (sprite) {
        .SpriteType0 => |v| {
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
        .SpriteType1 => |v| {
            r[0].x = v.ax;
            r[0].y = v.ay;
            r[0].z = v.az;
            r[0].u = uv16(v.auv.u);
            r[0].v = uv16(v.auv.v);

            r[1].x = v.dx;
            r[1].y = v.dy;

            r[2].x = v.bx;
            r[2].y = v.by;
            r[2].z = v.bz;
            r[2].u = uv16(v.buv.u);
            r[2].v = uv16(v.buv.v);

            r[3].x = v.cx;
            r[3].y = v.cy;
            r[3].z = v.cz;
            r[3].u = uv16(v.cuv.u);
            r[3].v = uv16(v.cuv.v);
        },
        else => {
            @panic("That's not a sprite. Dafuq are you doing?");
        },
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

const vertex_attributes = [_]wgpu.VertexAttribute{
    .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
    .{ .format = .float32x4, .offset = @offsetOf(Vertex, "base_color"), .shader_location = 1 },
    .{ .format = .float32x4, .offset = @offsetOf(Vertex, "offset_color"), .shader_location = 2 },
    .{ .format = .float32x2, .offset = @offsetOf(Vertex, "u"), .shader_location = 3 },
    .{ .format = .uint32x2, .offset = @offsetOf(Vertex, "tex"), .shader_location = 4 },
};
const vertex_buffers = [_]wgpu.VertexBufferLayout{.{
    .array_stride = @sizeOf(Vertex),
    .attribute_count = vertex_attributes.len,
    .attributes = &vertex_attributes,
}};

const modifier_volume_vertex_attributes = [_]wgpu.VertexAttribute{
    .{ .format = .float32x4, .offset = 0, .shader_location = 0 },
};
const modifier_volume_vertex_buffers = [_]wgpu.VertexBufferLayout{.{
    .array_stride = 4 * @sizeOf(f32),
    .attribute_count = modifier_volume_vertex_attributes.len,
    .attributes = &modifier_volume_vertex_attributes,
}};

pub const Renderer = struct {
    pub const MaxTextures: [8]u16 = .{ 256, 256, 256, 256, 256, 128, 32, 4 }; // Max texture count for each size. FIXME: Not sure what are good values.

    pub const DisplayMode = enum { Center, Fit, Stretch };

    pub const Resolution = struct { width: u32, height: u32 };
    pub const NativeResolution: Resolution = .{ .width = 640, .height = 480 };

    const FirstVertex: u32 = 4; // The 4 first vertices are reserved for the background.
    const FirstIndex: u32 = 5; // The 5 first indices are reserved for the background.

    // That's too much for the higher texture sizes, but that probably doesn't matter.
    texture_metadata: [8][256]TextureMetadata = [_][256]TextureMetadata{[_]TextureMetadata{.{}} ** 256} ** 8,

    framebuffer_resize_bind_group: zgpu.BindGroupHandle,

    blit_pipeline: zgpu.RenderPipelineHandle,
    blit_bind_group: zgpu.BindGroupHandle = undefined,
    blit_vertex_buffer: zgpu.BufferHandle,
    blit_index_buffer: zgpu.BufferHandle,
    blit_to_window_vertex_buffer: zgpu.BufferHandle,

    opaque_pipelines: std.AutoHashMap(PipelineKey, zgpu.RenderPipelineHandle),
    modifier_volume_pipeline: zgpu.RenderPipelineHandle,
    modifier_volume_apply_pipeline: zgpu.RenderPipelineHandle,
    translucent_pipeline: zgpu.RenderPipelineHandle,
    blend_pipeline: zgpu.ComputePipelineHandle,

    bind_group_layout: zgpu.BindGroupLayoutHandle,
    translucent_bind_group_layout: zgpu.BindGroupLayoutHandle,
    blend_bind_group_layout: zgpu.BindGroupLayoutHandle,
    sampler_bind_group_layout: zgpu.BindGroupLayoutHandle,

    pipeline_layout: zgpu.PipelineLayoutHandle,
    blit_vertex_shader_module: wgpu.ShaderModule,
    opaque_vertex_shader_module: wgpu.ShaderModule,
    opaque_fragment_shader_module: wgpu.ShaderModule,

    bind_group: zgpu.BindGroupHandle = undefined,
    modifier_volume_bind_group: zgpu.BindGroupHandle = undefined,
    modifier_volume_apply_bind_group: zgpu.BindGroupHandle = undefined,
    translucent_bind_group: zgpu.BindGroupHandle = undefined,
    blend_bind_group: zgpu.BindGroupHandle = undefined,

    vertex_buffer: zgpu.BufferHandle,
    index_buffer: zgpu.BufferHandle,
    modifier_volume_vertex_buffer: zgpu.BufferHandle,

    list_heads_buffer: zgpu.BufferHandle = undefined,
    init_list_heads_buffer: zgpu.BufferHandle = undefined,
    linked_list_buffer: zgpu.BufferHandle = undefined,

    texture_arrays: [8]zgpu.TextureHandle,
    texture_array_views: [8]zgpu.TextureViewHandle,

    display_mode: DisplayMode = .Center,
    resolution: Resolution = .{ .width = 2 * NativeResolution.width, .height = 2 * NativeResolution.height },

    // Intermediate texture to upload framebuffer from VRAM (and maybe downsample and read back from at some point?)
    framebuffer_texture: zgpu.TextureHandle,
    framebuffer_texture_view: zgpu.TextureViewHandle,
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

    passes: [5]PassMetadata = undefined,

    read_framebuffer_enabled: bool = false,
    min_depth: f32 = std.math.floatMax(f32),
    max_depth: f32 = 0.0,
    pt_alpha_ref: f32 = 1.0,
    fpu_shad_scale: f32 = 1.0,
    fog_col_pal: fRGBA = .{},
    fog_col_vert: fRGBA = .{},
    fog_density: f32 = 0,
    fog_lut: [0x80]u32 = [_]u32{0} ** 0x80,

    vertices: std.ArrayList(Vertex) = undefined, // Just here to avoid repeated allocations.
    modifier_volume_vertices: std.ArrayList([4]f32) = undefined,
    opaque_modifier_volumes: std.ArrayList(HollyModule.ModifierVolume) = undefined,
    _scratch_pad: []u8, // Used to avoid temporary allocations before GPU uploads for example. 4 * 1024 * 1024, since this is the maximum texture size supported by the DC.

    _gctx: *zgpu.GraphicsContext,
    _allocator: std.mem.Allocator,

    fn create_blit_bind_group_layout(gctx: *zgpu.GraphicsContext) zgpu.BindGroupLayoutHandle {
        return gctx.createBindGroupLayout(&.{
            zgpu.textureEntry(0, .{ .fragment = true }, .float, .tvdim_2d, false),
            zgpu.samplerEntry(1, .{ .fragment = true }, .filtering),
        });
    }

    fn get_or_put_opaque_pipeline(self: *Renderer, key: PipelineKey) !zgpu.RenderPipelineHandle {
        if (self.opaque_pipelines.get(key)) |pl|
            return pl;

        renderer_log.info("Creating Pipeline: {any}", .{key});

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
                .buffer_count = vertex_buffers.len,
                .buffers = &vertex_buffers,
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

        const pl = self._gctx.createRenderPipeline(self.pipeline_layout, pipeline_descriptor);

        if (!self._gctx.isResourceValid(pl)) {
            renderer_log.err("Error creating pipeline.", .{});
            renderer_log.err("{any}", .{pipeline_descriptor});
        }

        try self.opaque_pipelines.putNoClobber(key, pl);

        return pl;
    }

    pub fn init(allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) !Renderer {
        // Write to texture all rely on that.
        std.debug.assert(zgpu.GraphicsContext.swapchain_format == .bgra8_unorm);

        // FIXME: Make this comptime?
        moser_de_bruijin_sequence[0] = 0;
        for (1..moser_de_bruijin_sequence.len) |idx| {
            moser_de_bruijin_sequence[idx] = (moser_de_bruijin_sequence[idx - 1] + 0xAAAAAAAB) & 0x55555555;
        }

        const framebuffer_texture = gctx.createTexture(.{
            .usage = .{ .texture_binding = true, .copy_dst = true },
            .size = .{
                .width = NativeResolution.width,
                .height = NativeResolution.height,
                .depth_or_array_layers = 1,
            },
            .format = zgpu.GraphicsContext.swapchain_format,
            .mip_level_count = 1, // std.math.log2_int(u32, @max(1024, 1024)) + 1,
        });
        const framebuffer_texture_view = gctx.createTextureView(framebuffer_texture, .{});

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
        });
        const sampler_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.samplerEntry(0, .{ .fragment = true }, .filtering),
        });
        const pipeline_layout = gctx.createPipelineLayout(&.{ bind_group_layout, sampler_bind_group_layout });

        const blit_bind_group_layout = create_blit_bind_group_layout(gctx);
        defer gctx.releaseResource(blit_bind_group_layout);
        const blit_pipeline_layout = gctx.createPipelineLayout(&.{blit_bind_group_layout});
        defer gctx.releaseResource(blit_pipeline_layout);

        const blit_vs_module = zgpu.createWgslShaderModule(gctx.device, blit_vs, "vs");
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

        const blit_pipeline = blit_pl: {
            const blit_fs_module = zgpu.createWgslShaderModule(gctx.device, blit_fs, "fs");
            defer blit_fs_module.release();

            const color_targets = [_]wgpu.ColorTargetState{.{
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
                    .target_count = color_targets.len,
                    .targets = &color_targets,
                },
            };
            break :blit_pl gctx.createRenderPipeline(blit_pipeline_layout, pipeline_descriptor);
        };

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
                            samplers[index] =
                                gctx.createSampler(.{
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
                .usage = .{ .texture_binding = true, .copy_dst = true },
                .size = .{
                    .width = @as(u32, 8) << @intCast(i),
                    .height = @as(u32, 8) << @intCast(i),
                    .depth_or_array_layers = Renderer.MaxTextures[i],
                },
                .format = zgpu.GraphicsContext.swapchain_format,
                .mip_level_count = 1, // std.math.log2_int(u32, @as(u32, 8))) + 1,
            });
            texture_array_views[i] = gctx.createTextureView(texture_arrays[i], .{});
        }

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
        const translucent_pipeline_layout = gctx.createPipelineLayout(&.{
            bind_group_layout,
            sampler_bind_group_layout,
            translucent_bind_group_layout,
        });

        const color_targets = [_]wgpu.ColorTargetState{.{
            .format = zgpu.GraphicsContext.swapchain_format,
            .write_mask = .{}, // We won't write to the color attachment
        }};

        const translucent_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = wgpu.VertexState{
                .module = opaque_vertex_shader_module,
                .entry_point = "main",
                .buffer_count = vertex_buffers.len,
                .buffers = &vertex_buffers,
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
        const translucent_pipeline = gctx.createRenderPipeline(translucent_pipeline_layout, translucent_pipeline_descriptor);

        // Translucent fragment blending pipeline

        const blend_compute_module = zgpu.createWgslShaderModule(gctx.device, wgsl_blend_cs, null);
        defer blend_compute_module.release();

        const blend_bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .compute = true }, .uniform, true, 0),
            zgpu.bufferEntry(1, .{ .compute = true }, .storage, false, 0),
            zgpu.bufferEntry(2, .{ .compute = true }, .storage, false, 0),
            zgpu.textureEntry(3, .{ .compute = true }, .float, .tvdim_2d, false),
            zgpu.storageTextureEntry(4, .{ .compute = true }, .write_only, .bgra8_unorm, .tvdim_2d),
        });
        const blend_pipeline_layout = gctx.createPipelineLayout(&.{blend_bind_group_layout});

        const blend_pipeline_descriptor = wgpu.ComputePipelineDescriptor{
            .compute = .{
                .module = blend_compute_module,
                .entry_point = "main",
            },
        };

        const blend_pipeline = gctx.createComputePipeline(blend_pipeline_layout, blend_pipeline_descriptor);

        // Modifier Volumes
        // Implemented using a stencil buffer and the shadow volume algorithm.
        // This first pipeline takes the previous depth buffer and the modifier volume to generate the stencil buffer.

        const modifier_volume_vertex_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modifier_volume_vs, "vs");
        defer modifier_volume_vertex_shader_module.release();
        const modifier_volume_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_modifier_volume_fs, "fs");
        defer modifier_volume_fragment_shader_module.release();

        const modifier_volume_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .vertex = true }, .uniform, true, 0),
        });
        const modifier_volume_pipeline_layout = gctx.createPipelineLayout(&.{modifier_volume_group_layout});
        const modifier_volume_first_pass_pipeline_descriptor = wgpu.RenderPipelineDescriptor{
            .vertex = wgpu.VertexState{
                .module = modifier_volume_vertex_shader_module,
                .entry_point = "main",
                .buffer_count = modifier_volume_vertex_buffers.len,
                .buffers = &modifier_volume_vertex_buffers,
            },
            .primitive = wgpu.PrimitiveState{
                .front_face = .ccw,
                .cull_mode = .none,
                .topology = .triangle_list,
            },
            .depth_stencil = &wgpu.DepthStencilState{
                .format = .depth32_float_stencil8,
                .depth_write_enabled = false,
                .depth_compare = .less,
                .stencil_front = .{
                    .compare = .always,
                    .fail_op = .increment_wrap, // Stencil test failed (we don't care about the stencil test here)
                    .pass_op = .increment_wrap, // Stencil test passed
                    .depth_fail_op = .keep, // Stencil test passed, but depth test failed (this is what we care about, with the front face/back face distinction)
                },
                .stencil_back = .{
                    .compare = .always,
                    .fail_op = .decrement_wrap,
                    .pass_op = .decrement_wrap,
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
        const modifier_volume_pipeline = gctx.createRenderPipeline(modifier_volume_pipeline_layout, modifier_volume_first_pass_pipeline_descriptor);

        const modifier_volume_bind_group = gctx.createBindGroup(modifier_volume_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = 2 * @sizeOf(f32) },
        });

        // Modifier Volume Apply pipeline - Use the stencil from the previous pass to apply modifier volume effects.
        const mv_apply_pipeline = mvp: {
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
                    .stencil_front = .{
                        .compare = .not_equal,
                        .fail_op = .keep,
                        .depth_fail_op = .keep,
                        .pass_op = .keep,
                    },
                    .stencil_back = .{
                        .compare = .not_equal,
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
            break :mvp gctx.createRenderPipeline(mv_apply_pipeline_layout, mv_apply_pipeline_descriptor);
        };

        var renderer: Renderer = .{
            .blit_pipeline = blit_pipeline,
            .blit_vertex_buffer = blit_vertex_buffer,
            .blit_index_buffer = blit_index_buffer,
            .blit_to_window_vertex_buffer = blit_to_window_vertex_buffer,

            .framebuffer_resize_bind_group = framebuffer_resize_bind_group,

            .framebuffer_texture = framebuffer_texture,
            .framebuffer_texture_view = framebuffer_texture_view,

            .opaque_pipelines = std.AutoHashMap(PipelineKey, zgpu.RenderPipelineHandle).init(allocator),

            .modifier_volume_pipeline = modifier_volume_pipeline,
            .modifier_volume_apply_pipeline = mv_apply_pipeline,

            .translucent_pipeline = translucent_pipeline,
            .translucent_bind_group_layout = translucent_bind_group_layout,

            .blend_pipeline = blend_pipeline,
            .blend_bind_group_layout = blend_bind_group_layout,

            .bind_group_layout = bind_group_layout,
            .sampler_bind_group_layout = sampler_bind_group_layout,
            .pipeline_layout = pipeline_layout,
            .blit_vertex_shader_module = blit_vs_module,
            .opaque_vertex_shader_module = opaque_vertex_shader_module,
            .opaque_fragment_shader_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "fs"),

            .bind_group = bind_group,
            .modifier_volume_bind_group = modifier_volume_bind_group,
            .sampler_bind_groups = sampler_bind_groups,
            .vertex_buffer = vertex_buffer,
            .index_buffer = index_buffer,
            .modifier_volume_vertex_buffer = modifier_volume_vertex_buffer,

            .texture_arrays = texture_arrays,
            .texture_array_views = texture_array_views,

            .samplers = samplers,

            .vertices = try std.ArrayList(Vertex).initCapacity(allocator, 4096),
            .modifier_volume_vertices = try std.ArrayList([4]f32).initCapacity(allocator, 4096),
            .opaque_modifier_volumes = std.ArrayList(HollyModule.ModifierVolume).init(allocator),
            ._scratch_pad = try allocator.alloc(u8, 4 * 1024 * 1024),

            ._gctx = gctx,
            ._allocator = allocator,
        };

        for (0..renderer.passes.len) |i| {
            renderer.passes[i] = PassMetadata.init(allocator, @enumFromInt(i));
        }

        renderer.on_inner_resolution_change();

        return renderer;
    }

    pub fn deinit(self: *Renderer) void {
        for (&self.passes) |*pass| {
            pass.deinit();
        }

        self.vertices.deinit();
        self.modifier_volume_vertices.deinit();
        self.opaque_modifier_volumes.deinit();
        self._allocator.free(self._scratch_pad);
        // FIXME: I have a lot more resources to destroy.
        self.deinit_screen_textures();
        self.opaque_vertex_shader_module.release();
        self.opaque_fragment_shader_module.release();
        self._gctx.releaseResource(self.bind_group_layout);
        self._gctx.releaseResource(self.pipeline_layout);
    }

    fn get_texture_index(self: *Renderer, size_index: u3, control_word: HollyModule.TextureControlWord) ?TextureIndex {
        for (0..Renderer.MaxTextures[size_index]) |i| {
            if (self.texture_metadata[size_index][i].status != .Invalid and self.texture_metadata[size_index][i].status != .Outdated and
                // NOTE: In most cases, the address should be enough, but Soul Calibur mixes multiple different pixel formats in the same texture.
                //       Do handle this, we'll treat then as different textures and upload an additional copy of the texture for each pixel format used.
                //       This is pretty wasteful, but I hope this will be okay.
                @as(u32, @bitCast(self.texture_metadata[size_index][i].control_word)) == @as(u32, @bitCast(control_word)))
            {
                return @intCast(i);
            }
        }
        return null;
    }

    fn palette_hash(gpu: *HollyModule.Holly, texture_control_word: HollyModule.TextureControlWord) u32 {
        switch (texture_control_word.pixel_format) {
            .Palette4BPP, .Palette8BPP => |format| {
                const palette_ram = @as([*]const u8, @ptrCast(gpu._get_register(u8, .PALETTE_RAM_START)))[0 .. 4 * 1024];
                const palette_selector: u16 = @truncate(if (format == .Palette4BPP) (((@as(u32, @bitCast(texture_control_word)) >> 21) & 0b111111) << 4) else (((@as(u32, @bitCast(texture_control_word)) >> 25) & 0b11) << 8));
                const size: u16 = if (format == .Palette4BPP) 16 else 256;
                return std.hash.Murmur3_32.hash(palette_ram[4 * palette_selector .. @min(4 * (palette_selector + size), palette_ram.len)]);
            },
            else => return 0,
        }
    }

    inline fn bgra_from_16bits_color(format: HollyModule.TexturePixelFormat, val: u16, twiddled: bool) [4]u8 {
        // See 3.6.3 Color Data Extension (p149)
        if (twiddled) {
            return bgra_from_16bits_color_twiddled(format, val);
        } else {
            return bgra_from_16bits_color_non_twiddled(format, val);
        }
    }

    inline fn bgra_from_16bits_color_non_twiddled(format: HollyModule.TexturePixelFormat, val: u16) [4]u8 {
        const pixel: HollyModule.Color16 = .{ .value = val };
        switch (format) {
            .ARGB1555 => {
                return .{
                    @as(u8, pixel.arbg1555.b) << 3,
                    @as(u8, pixel.arbg1555.g) << 3,
                    @as(u8, pixel.arbg1555.r) << 3,
                    @as(u8, pixel.arbg1555.a) * 0xFF,
                };
            },
            .RGB565 => {
                return .{
                    @as(u8, pixel.rgb565.b) << 3,
                    @as(u8, pixel.rgb565.g) << 2,
                    @as(u8, pixel.rgb565.r) << 3,
                    255,
                };
            },
            .ARGB4444 => {
                return .{
                    @as(u8, pixel.argb4444.b) << 4,
                    @as(u8, pixel.argb4444.g) << 4,
                    @as(u8, pixel.argb4444.r) << 4,
                    @as(u8, pixel.argb4444.a) << 4,
                };
            },
            else => {
                renderer_log.err("Invalid 16-bits pixel format {any}", .{format});
                @panic("Invalid 16-bits pixel format");
            },
        }
    }

    inline fn bgra_from_16bits_color_twiddled(format: HollyModule.TexturePixelFormat, val: u16) [4]u8 {
        const pixel: HollyModule.Color16 = .{ .value = val };
        switch (format) {
            .ARGB1555 => {
                return .{
                    @as(u8, pixel.arbg1555.b) << 3 | @as(u8, pixel.arbg1555.b) >> 2,
                    @as(u8, pixel.arbg1555.g) << 3 | @as(u8, pixel.arbg1555.g) >> 2,
                    @as(u8, pixel.arbg1555.r) << 3 | @as(u8, pixel.arbg1555.r) >> 2,
                    @as(u8, pixel.arbg1555.a) * 0xFF,
                };
            },
            .RGB565 => {
                return .{
                    @as(u8, pixel.rgb565.b) << 3 | @as(u8, pixel.rgb565.b) >> 2,
                    @as(u8, pixel.rgb565.g) << 2 | @as(u8, pixel.rgb565.g) >> 4,
                    @as(u8, pixel.rgb565.r) << 3 | @as(u8, pixel.rgb565.r) >> 2,
                    255,
                };
            },
            .ARGB4444 => {
                return .{
                    @as(u8, pixel.argb4444.b) << 4 | @as(u8, pixel.argb4444.b),
                    @as(u8, pixel.argb4444.g) << 4 | @as(u8, pixel.argb4444.g),
                    @as(u8, pixel.argb4444.r) << 4 | @as(u8, pixel.argb4444.r),
                    @as(u8, pixel.argb4444.a) << 4 | @as(u8, pixel.argb4444.a),
                };
            },
            else => {
                renderer_log.err("Invalid 16-bits pixel format {any}", .{format});
                @panic("Invalid 16-bits pixel format");
            },
        }
    }

    inline fn bgra_scratch_pad(self: *Renderer) [*][4]u8 {
        return @as([*][4]u8, @ptrCast(self._scratch_pad.ptr));
    }

    fn upload_texture(self: *Renderer, gpu: *HollyModule.Holly, tsp_instruction: HollyModule.TSPInstructionWord, texture_control_word: HollyModule.TextureControlWord) TextureIndex {
        renderer_log.debug("[Upload] tsp_instruction: {any}", .{tsp_instruction});
        renderer_log.debug("[Upload] texture_control_word: {any}", .{texture_control_word});

        const texture_control_register = gpu._get_register(HollyModule.TEXT_CONTROL, .TEXT_CONTROL).*;

        const is_paletted = texture_control_word.pixel_format == .Palette4BPP or texture_control_word.pixel_format == .Palette8BPP;

        const scan_order = if (is_paletted) 0 else texture_control_word.scan_order;
        const stride_select = if (is_paletted) 0 else texture_control_word.stride_select;

        const twiddled = scan_order == 0;
        const size_index = tsp_instruction.texture_u_size;

        // NOTE: This is used by stride textures. Stride textures actual size can be smaller than their allocated size, but UV calculation are still done with it.
        const alloc_u_size = (@as(u16, 8) << tsp_instruction.texture_u_size);
        const alloc_v_size = (@as(u16, 8) << tsp_instruction.texture_v_size);

        const u_size: u16 = if (scan_order == 1 and stride_select == 1) @as(u16, 32) * texture_control_register.stride else alloc_u_size;
        const v_size: u16 = if (scan_order == 0 and texture_control_word.mip_mapped == 1) u_size else alloc_v_size;

        var addr: u32 = 8 * @as(u32, texture_control_word.address); // given in units of 64-bits.
        var vq_index_addr = addr;

        if (texture_control_word.mip_mapped == 1) {
            renderer_log.debug(termcolor.yellow(" TODO: Actually support mip mapping."), .{});

            // We only want the highest mip level and we'll compute the others ourself.
            // See DreamcastDevBoxSystemArchitecture.pdf p.148
            if (texture_control_word.vq_compressed == 1) {
                vq_index_addr += switch (u_size) {
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
                        @panic("Invalid u_size for mip mapped texture");
                    },
                };
            } else if (texture_control_word.pixel_format == .Palette4BPP or texture_control_word.pixel_format == .Palette8BPP) {
                const val: u32 = switch (u_size) {
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
                        @panic("Invalid u_size for mip mapped texture");
                    },
                };
                addr += if (texture_control_word.pixel_format == .Palette4BPP) val / 2 else val;
            } else {
                addr += switch (u_size) {
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
        }

        // FIXME: This needs a big refactor.
        if (texture_control_word.vq_compressed == 1) {
            std.debug.assert(twiddled); // Please.
            const code_book = @as([*]u64, @alignCast(@ptrCast(&gpu.vram[addr])))[0..256];
            const indices = @as([*]u8, @ptrCast(&gpu.vram[vq_index_addr + 8 * 256]))[0..];
            // FIXME: It's not an efficient way to run through the texture, but it's already hard enough to wrap my head around the multiple levels of twiddling.
            for (0..v_size / 2) |v| {
                for (0..u_size / 2) |u| {
                    const index = indices[untwiddle(@intCast(u), @intCast(v), u_size / 2, v_size / 2)];
                    const texels = code_book[index];
                    for (0..4) |tidx| {
                        switch (texture_control_word.pixel_format) {
                            .ARGB1555, .RGB565, .ARGB4444 => {
                                //                  Macro 2*2 Block            Pixel within the block
                                const pixel_index = (2 * v * u_size + 2 * u) + u_size * (tidx & 1) + (tidx >> 1);
                                self.bgra_scratch_pad()[pixel_index] = bgra_from_16bits_color(texture_control_word.pixel_format, @truncate(texels >> @intCast(16 * tidx)), twiddled);
                            },
                            else => {
                                renderer_log.err(termcolor.red("Unsupported pixel format in VQ texture {any}"), .{texture_control_word.pixel_format});
                                @panic("Unsupported pixel format in VQ texture");
                            },
                        }
                    }
                }
            }
        } else {
            switch (texture_control_word.pixel_format) {
                .ARGB1555, .RGB565, .ARGB4444 => |format| {
                    for (0..v_size) |v| {
                        for (0..u_size) |u| {
                            const pixel_idx = v * u_size + u;
                            const texel_idx = if (twiddled) untwiddle(@intCast(u), @intCast(v), u_size, v_size) else pixel_idx;
                            self.bgra_scratch_pad()[pixel_idx] = bgra_from_16bits_color(format, @as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * texel_idx]))).*, twiddled);
                        }
                    }
                },
                .Palette4BPP, .Palette8BPP => |format| {
                    const palette_ram = @as([*]const u32, @ptrCast(gpu._get_register(u32, .PALETTE_RAM_START)))[0..1024];
                    const palette_ctrl_ram: u2 = @truncate(gpu._get_register(u32, .PAL_RAM_CTRL).* & 0b11);
                    const palette_selector: u10 = @truncate(if (format == .Palette4BPP) (((@as(u32, @bitCast(texture_control_word)) >> 21) & 0b111111) << 4) else (((@as(u32, @bitCast(texture_control_word)) >> 25) & 0b11) << 8));

                    for (0..v_size) |v| {
                        for (0..u_size) |u| {
                            const pixel_idx = v * u_size + u;
                            const texel_idx = if (twiddled) untwiddle(@intCast(u), @intCast(v), u_size, v_size) else pixel_idx;
                            const ram_addr = if (format == .Palette4BPP) texel_idx >> 1 else texel_idx;
                            const pixel_palette: u8 = gpu.vram[addr + ram_addr];
                            const offset: u10 = if (format == .Palette4BPP) ((pixel_palette >> @intCast(4 * (texel_idx & 0x1))) & 0xF) else pixel_palette;
                            switch (palette_ctrl_ram) {
                                0x0, 0x1, 0x2 => { // ARGB1555, RGB565, ARGB4444. These happen to match the values of TexturePixelFormat.
                                    self.bgra_scratch_pad()[pixel_idx] = bgra_from_16bits_color(@enumFromInt(palette_ctrl_ram), @truncate(palette_ram[palette_selector + offset]), twiddled);
                                },
                                0x3 => { // ARGB8888
                                    self.bgra_scratch_pad()[pixel_idx] = @bitCast(palette_ram[palette_selector + offset]);
                                },
                            }
                        }
                    }
                },
                .YUV422 => {
                    if (twiddled) {
                        for (0..v_size / 2) |v| {
                            for (0..u_size / 2) |u| {
                                const pixel_idx = 2 * v * u_size + 2 * u;
                                const texel_idx = untwiddle(@intCast(u), @intCast(v), u_size / 2, v_size / 2);
                                const halfwords = @as([*]const u16, @alignCast(@ptrCast(&gpu.vram[addr + 8 * texel_idx])))[0..4];
                                const texels_0_1: HollyModule.YUV422 = @bitCast(@as(u32, halfwords[2]) << 16 | @as(u32, halfwords[0]));
                                const texels_2_3: HollyModule.YUV422 = @bitCast(@as(u32, halfwords[3]) << 16 | @as(u32, halfwords[1]));
                                const colors_0 = HollyModule.yuv_to_rgba(texels_0_1);
                                self.bgra_scratch_pad()[pixel_idx] = .{ colors_0[0].b, colors_0[0].g, colors_0[0].r, colors_0[0].a };
                                self.bgra_scratch_pad()[pixel_idx + 1] = .{ colors_0[1].b, colors_0[1].g, colors_0[1].r, colors_0[1].a };
                                const colors_1 = HollyModule.yuv_to_rgba(texels_2_3);
                                self.bgra_scratch_pad()[pixel_idx + u_size] = .{ colors_1[0].b, colors_1[0].g, colors_1[0].r, colors_1[0].a };
                                self.bgra_scratch_pad()[pixel_idx + u_size + 1] = .{ colors_1[1].b, colors_1[1].g, colors_1[1].r, colors_1[1].a };
                            }
                        }
                    } else {
                        for (0..v_size) |v| {
                            for (0..u_size / 2) |u| {
                                const pixel_idx = v * u_size + 2 * u;
                                const texel_idx = 2 * pixel_idx;
                                const texel: HollyModule.YUV422 = @bitCast(@as(*const u32, @alignCast(@ptrCast(&gpu.vram[addr + texel_idx]))).*);
                                const colors = HollyModule.yuv_to_rgba(texel);
                                self.bgra_scratch_pad()[pixel_idx] = .{ colors[0].b, colors[0].g, colors[0].r, colors[0].a };
                                self.bgra_scratch_pad()[pixel_idx + 1] = .{ colors[1].b, colors[1].g, colors[1].r, colors[1].a };
                            }
                        }
                    }
                },
                else => {
                    renderer_log.err(termcolor.red("Unsupported pixel format {any}"), .{texture_control_word.pixel_format});
                    @panic("Unsupported pixel format");
                },
            }
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
            for (0..Renderer.MaxTextures[size_index]) |i| {
                if (self.texture_metadata[size_index][i].status == .Outdated and (texture_index == InvalidTextureIndex or self.texture_metadata[size_index][texture_index].status == .Unused or (self.texture_metadata[size_index][texture_index].status == .Outdated and self.texture_metadata[size_index][texture_index].age < self.texture_metadata[size_index][i].age))) {
                    texture_index = @as(TextureIndex, @intCast(i));
                } else if (self.texture_metadata[size_index][i].status == .Unused and (texture_index == InvalidTextureIndex or self.texture_metadata[size_index][texture_index].age < self.texture_metadata[size_index][i].age)) {
                    texture_index = @as(TextureIndex, @intCast(i));
                }
            }
        }

        if (texture_index == InvalidTextureIndex) {
            renderer_log.err(termcolor.red("Out of textures slot (size: {d})"), .{size_index});
            @panic("Out of textures slot");
        }

        const end_address = addr + switch (texture_control_word.pixel_format) {
            .Palette4BPP => @as(u32, u_size) * v_size / 2,
            .Palette8BPP => @as(u32, u_size) * v_size,
            else => 2 * @as(u32, u_size) * v_size,
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
            .palette_hash = palette_hash(gpu, texture_control_word),
        };

        // Fill with repeating texture data when v_size < u_size to avoid vertical wrapping artifacts.
        // FIXME: We should do the same the stride textures when u_size < alloc_u_size.
        for (0..u_size / v_size) |part| {
            self._gctx.queue.writeTexture(
                .{
                    .texture = self._gctx.lookupResource(self.texture_arrays[size_index]).?,
                    .origin = .{ .y = @intCast(v_size * part), .z = @intCast(texture_index) },
                },
                .{
                    .bytes_per_row = u_size * 4,
                    .rows_per_image = v_size,
                },
                .{ .width = u_size, .height = v_size, .depth_or_array_layers = 1 },
                u8,
                self._scratch_pad,
            );
        }

        if (texture_control_word.mip_mapped == 1) {
            // TODO: Here we'd want to generate mipmaps.
            //       See zgpu.generateMipmaps, maybe?
        }

        return texture_index;
    }

    fn reset_texture_usage(self: *Renderer, gpu: *HollyModule.Holly) void {
        for (0..Renderer.MaxTextures.len) |j| {
            for (0..Renderer.MaxTextures[j]) |i| {
                if (self.texture_metadata[j][i].status != .Invalid) {
                    self.texture_metadata[j][i].usage = 0;
                    const is_palette_texture = self.texture_metadata[j][i].control_word.pixel_format == .Palette4BPP or self.texture_metadata[j][i].control_word.pixel_format == .Palette8BPP;
                    // Texture appears to have changed in memory, or palette has changed. Mark as outdated.
                    if ((is_palette_texture and palette_hash(gpu, self.texture_metadata[j][i].control_word) != self.texture_metadata[j][i].palette_hash) or texture_hash(gpu, self.texture_metadata[j][i].start_address, self.texture_metadata[j][i].end_address) != self.texture_metadata[j][i].hash) {
                        self.texture_metadata[j][i].status = .Outdated;
                    } else if (self.texture_metadata[j][i].status == .Outdated) { // It became valid again, assume it will be used this frame.
                        self.texture_metadata[j][i].status = .Used;
                    }
                }
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

    pub fn update_framebuffer(self: *Renderer, gpu: *HollyModule.Holly) void {
        const SPG_CONTROL = gpu._get_register(HollyModule.SPG_CONTROL, .SPG_CONTROL).*;
        const FB_R_CTRL = gpu._get_register(HollyModule.FB_R_CTRL, .FB_R_CTRL).*;
        // const FB_C_SOF = gpu._get_register(u32, .FB_C_SOF).*;
        const FB_R_SOF1 = gpu._get_register(u32, .FB_R_SOF1).*;
        const FB_R_SOF2 = gpu._get_register(u32, .FB_R_SOF2).*;
        const FB_R_SIZE = gpu._get_register(HollyModule.FB_R_SIZE, .FB_R_SIZE).*;

        self.read_framebuffer_enabled = FB_R_CTRL.enable;

        // Enabled: We have to copy some data from VRAM.
        if (FB_R_CTRL.enable) {
            // TODO: Find a way to avoid unecessary uploads?
            renderer_log.debug("Reading from framebuffer (from the PoV of the Holly Core) enabled.", .{});

            const line_size: u32 = 4 * (@as(u32, FB_R_SIZE.x_size) + 1); // From 32-bit units to bytes.
            const field_size: u32 = @as(u32, FB_R_SIZE.y_size) + 1; // Number of lines

            const bytes_per_pixels: u32 = switch (FB_R_CTRL.format) {
                0, 1 => 2,
                2 => 3,
                3 => 4,
            };

            const interlaced = SPG_CONTROL.interlace == 1;
            const x_size = line_size / bytes_per_pixels;
            const y_size = if (interlaced) field_size * 2 else field_size;
            const line_padding = 4 * (@as(u32, FB_R_SIZE.modulus) - 1); // In bytes

            @memset(self._scratch_pad, 0); // TODO: Fill using VO_BORDER_COL?

            for (0..y_size) |y| {
                const line_in_field = if (interlaced) y / 2 else y;
                const addr = (if (interlaced and (y % 2) == 1) FB_R_SOF2 else FB_R_SOF1) + line_in_field * (line_size + line_padding);
                for (0..x_size) |x| {
                    const pixel_idx = x_size * y + x;
                    const pixel_addr = addr + bytes_per_pixels * x;
                    switch (FB_R_CTRL.format) {
                        0x0 => { // 0555 RGB 16 bit
                            const pixel: HollyModule.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[pixel_addr]))).* };
                            self._scratch_pad[pixel_idx * 4 + 0] = (@as(u8, pixel.arbg1555.b) << 3) | FB_R_CTRL.concat;
                            self._scratch_pad[pixel_idx * 4 + 1] = (@as(u8, pixel.arbg1555.g) << 3) | FB_R_CTRL.concat;
                            self._scratch_pad[pixel_idx * 4 + 2] = (@as(u8, pixel.arbg1555.r) << 3) | FB_R_CTRL.concat;
                            self._scratch_pad[pixel_idx * 4 + 3] = 255;
                        },
                        0x1 => { // 565 RGB
                            const pixel: HollyModule.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[pixel_addr]))).* };
                            self._scratch_pad[pixel_idx * 4 + 0] = (@as(u8, pixel.rgb565.b) << 3) | FB_R_CTRL.concat;
                            self._scratch_pad[pixel_idx * 4 + 1] = (@as(u8, pixel.rgb565.g) << 2) | (FB_R_CTRL.concat & 0b11);
                            self._scratch_pad[pixel_idx * 4 + 2] = (@as(u8, pixel.rgb565.r) << 3) | FB_R_CTRL.concat;
                            self._scratch_pad[pixel_idx * 4 + 3] = 255;
                        },
                        0x2 => { // 888 RGB 24 bit packed
                            const pixel: [3]u8 = @as([*]const u8, @alignCast(@ptrCast(&gpu.vram[pixel_addr])))[0..3].*;
                            self._scratch_pad[pixel_idx * 4 + 0] = pixel[2];
                            self._scratch_pad[pixel_idx * 4 + 1] = pixel[1];
                            self._scratch_pad[pixel_idx * 4 + 2] = pixel[0];
                            self._scratch_pad[pixel_idx * 4 + 3] = 255;
                        },
                        0x3 => { // 0888 RGB 32 bit
                            const pixel: [4]u8 = @as([*]const u8, @alignCast(@ptrCast(&gpu.vram[pixel_addr])))[0..4].*;
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
    }

    // Pulls 3 vertices from the address pointed by ISP_BACKGND_T and places them at the front of the vertex buffer.
    pub fn update_background(self: *Renderer, gpu: *HollyModule.Holly) void {
        const tags = gpu._get_register(HollyModule.ISP_BACKGND_T, .ISP_BACKGND_T).*;
        const param_base = gpu._get_register(u32, .PARAM_BASE).*;
        const addr = param_base + 4 * tags.tag_address;
        const isp_tsp_instruction = @as(*const HollyModule.ISPTSPInstructionWord, @alignCast(@ptrCast(&gpu.vram[addr]))).*;
        const tsp_instruction = @as(*const HollyModule.TSPInstructionWord, @alignCast(@ptrCast(&gpu.vram[addr + 4]))).*;
        const texture_control = @as(*const HollyModule.TextureControlWord, @alignCast(@ptrCast(&gpu.vram[addr + 8]))).*;
        const texture_size_index = tsp_instruction.texture_u_size;

        // FIXME: I don't understand. In the boot menu for example, this depth value is 0.0,
        //        which doesn't make sense. The vertices z position looks more inline with what
        //        I understand of the render pipeline.
        const depth = gpu._get_register(f32, .ISP_BACKGND_D).*;
        _ = depth;

        // Offset into the strip pointed by ISP_BACKGND_T indicated by tag_offset.
        const parameter_volume_mode = (gpu._get_register(u32, .FPU_SHAD_SCALE).* >> 8) & 1 == 1 and tags.shadow == 1;
        const skipped_vertex_byte_size: u32 = @as(u32, 4) * (if (parameter_volume_mode) 3 + tags.skip else 3 + 2 * tags.skip);
        const start = addr + 12 + tags.tag_offset * skipped_vertex_byte_size;

        var vertices: [4]Vertex = undefined;

        var vertex_byte_size: u32 = 4 * (3 + 1);
        var tex_idx: TextureIndex = 0;

        // The unused fields seems to be absent.
        if (isp_tsp_instruction.texture == 1) {
            tex_idx = self.get_texture_index(texture_size_index, texture_control) orelse self.upload_texture(gpu, tsp_instruction, texture_control);
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
            .shading = ShadingInstructions{
                .textured = isp_tsp_instruction.texture,
                .mode = tsp_instruction.texture_shading_instruction,
                .ignore_alpha = tsp_instruction.ignore_texture_alpha,
                .tex_u_size = texture_size_index,
                .tex_v_size = tsp_instruction.texture_v_size,
                .depth_compare = isp_tsp_instruction.depth_compare_mode,
                .fog_control = tsp_instruction.fog_control,
                .offset_bit = isp_tsp_instruction.offset,
                .shadow_bit = 0,
                .gouraud_bit = isp_tsp_instruction.gouraud,
            },
        };

        for (0..3) |i| {
            const vp = @as([*]const u32, @alignCast(@ptrCast(&gpu.vram[start + i * vertex_byte_size])));
            var u: f32 = 0;
            var v: f32 = 0;
            var base_color: HollyModule.PackedColor = @bitCast(vp[3]);
            var offset_color: HollyModule.PackedColor = .{ .b = 0, .g = 0, .r = 0, .a = 0 };
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
            vertices[i] = Vertex{
                .x = @bitCast(vp[0]),
                .y = @bitCast(vp[1]),
                .z = @bitCast(vp[2]),
                .base_color = .{
                    .r = @as(f32, @floatFromInt(base_color.r)) / 255.0,
                    .g = @as(f32, @floatFromInt(base_color.g)) / 255.0,
                    .b = @as(f32, @floatFromInt(base_color.b)) / 255.0,
                    .a = if (use_alpha) @as(f32, @floatFromInt(base_color.a)) / 255.0 else 1.0,
                },
                .offset_color = .{
                    .r = @as(f32, @floatFromInt(offset_color.r)) / 255.0,
                    .g = @as(f32, @floatFromInt(offset_color.g)) / 255.0,
                    .b = @as(f32, @floatFromInt(offset_color.b)) / 255.0,
                    .a = if (use_alpha) @as(f32, @floatFromInt(offset_color.a)) / 255.0 else 1.0,
                },
                .u = u,
                .v = v,
                .tex = tex,
            };
            // FIXME: We should probably render the background in a separate pass with depth test disabled.
            self.min_depth = @min(self.min_depth, 1 / vertices[i].z);
            self.max_depth = @max(self.max_depth, 1 / vertices[i].z);
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
            .x = vertices[1].x,
            .y = vertices[2].y,
            .z = vertices[2].z,
            // NOTE: I have no idea how the color is computed, looking at the boot menu, this seems right.
            .base_color = vertices[2].base_color,
            .u = vertices[2].u,
            .v = vertices[1].v,
            .tex = tex,
        };

        const indices = [_]u32{ 0, 1, 2, 3, 0xFFFFFFFF };

        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, 0, Vertex, &vertices);
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.index_buffer).?, 0, u32, &indices);

        std.debug.assert(FirstVertex == vertices.len);
        std.debug.assert(FirstIndex == indices.len);
    }

    pub fn update(self: *Renderer, gpu: *HollyModule.Holly) !void {
        self.vertices.clearRetainingCapacity();

        self.reset_texture_usage(gpu);
        defer self.check_texture_usage();

        self.min_depth = std.math.floatMax(f32);
        self.max_depth = 0.0;

        self.pt_alpha_ref = @as(f32, @floatFromInt(gpu._get_register(u8, .PT_ALPHA_REF).*)) / 255.0;

        const fpu_shad_scale = gpu._get_register(u32, .FPU_SHAD_SCALE).*;
        self.fpu_shad_scale = if ((fpu_shad_scale & 0x100) != 0) @as(f32, @floatFromInt(fpu_shad_scale & 0xFF)) / 256.0 else 1.0;

        const col_pal = gpu._get_register(HollyModule.PackedColor, .FOG_COL_RAM).*;
        const col_vert = gpu._get_register(HollyModule.PackedColor, .FOG_COL_VERT).*;

        self.fog_col_pal = fRGBA.fromPacked(col_pal);
        self.fog_col_vert = fRGBA.fromPacked(col_vert);
        const fog_density = gpu._get_register(u16, .FOG_DENSITY).*;
        const fog_density_mantissa = (fog_density >> 8) & 0xFF;
        const fog_density_exponent: i8 = @bitCast(@as(u8, @truncate(fog_density & 0xFF)));
        self.fog_density = @as(f32, @floatFromInt(fog_density_mantissa)) / 128.0 * std.math.pow(f32, 2.0, @floatFromInt(fog_density_exponent));
        for (0..0x80) |i| {
            self.fog_lut[i] = @as([*]u32, @ptrCast(gpu._get_register(u32, .FOG_TABLE_START)))[i] & 0x0000FFFF;
        }

        self.update_background(gpu);

        for (&self.passes) |*pass| {
            pass.pass_type = pass.pass_type;

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

        inline for (.{ HollyModule.ListType.Opaque, HollyModule.ListType.Translucent, HollyModule.ListType.PunchThrough }) |list_type| {
            // Parameters specific to a polygon type
            var face_color: fRGBA = undefined; // In Intensity Mode 2, the face color is the one of the previous Intensity Mode 1 Polygon
            var face_offset_color: fRGBA = undefined;
            const display_list = gpu.ta_display_lists[@intFromEnum(list_type)];

            for (0..display_list.vertex_strips.items.len) |idx| {
                const start: u32 = @intCast(self.vertices.items.len);

                // Generic Parameters
                var parameter_control_word: HollyModule.ParameterControlWord = undefined;
                var isp_tsp_instruction: HollyModule.ISPTSPInstructionWord = undefined;
                var tsp_instruction: HollyModule.TSPInstructionWord = undefined;
                var texture_control: HollyModule.TextureControlWord = undefined;

                var sprite_base_color: HollyModule.PackedColor = undefined;
                var sprite_offset_color: HollyModule.PackedColor = undefined;

                switch (display_list.vertex_strips.items[idx].polygon) {
                    .PolygonType0 => |p| {
                        parameter_control_word = p.parameter_control_word;
                        isp_tsp_instruction = p.isp_tsp_instruction;
                        tsp_instruction = p.tsp_instruction;
                        texture_control = p.texture_control;
                    },
                    .PolygonType1 => |p| {
                        parameter_control_word = p.parameter_control_word;
                        isp_tsp_instruction = p.isp_tsp_instruction;
                        tsp_instruction = p.tsp_instruction;
                        texture_control = p.texture_control;
                        if (parameter_control_word.obj_control.col_type == .IntensityMode1)
                            face_color = .{
                                .r = p.face_color_r,
                                .g = p.face_color_g,
                                .b = p.face_color_b,
                                .a = p.face_color_a,
                            };
                    },
                    .PolygonType2 => |p| {
                        parameter_control_word = p.parameter_control_word;
                        isp_tsp_instruction = p.isp_tsp_instruction;
                        tsp_instruction = p.tsp_instruction;
                        texture_control = p.texture_control;
                        if (parameter_control_word.obj_control.col_type == .IntensityMode1)
                            face_color = .{
                                .r = p.face_color_r,
                                .g = p.face_color_g,
                                .b = p.face_color_b,
                                .a = p.face_color_a,
                            };
                        if (parameter_control_word.obj_control.col_type == .IntensityMode1)
                            face_offset_color = .{
                                .r = p.face_offset_color_r,
                                .g = p.face_offset_color_g,
                                .b = p.face_offset_color_b,
                                .a = p.face_offset_color_a,
                            };
                    },
                    // NOTE: In the case of Polygon Type 4 (Intensity, with Two Volumes), the Face Color is used in both the Base Color and the Offset Color.
                    .Sprite => |p| {
                        parameter_control_word = p.parameter_control_word;
                        isp_tsp_instruction = p.isp_tsp_instruction;
                        tsp_instruction = p.tsp_instruction;
                        texture_control = p.texture_control;
                        sprite_base_color = p.base_color;
                        sprite_offset_color = p.offset_color;
                    },
                    else => {
                        renderer_log.err("Unhandled polygon type: {any}", .{display_list.vertex_strips.items[idx].polygon});
                    },
                }

                var tex_idx: TextureIndex = 0;
                const textured = parameter_control_word.obj_control.texture == 1;
                const texture_size_index = tsp_instruction.texture_u_size;
                if (textured) {
                    const tmp_tex_idx = self.get_texture_index(texture_size_index, texture_control);
                    if (tmp_tex_idx == null) {
                        tex_idx = self.upload_texture(gpu, tsp_instruction, texture_control);
                    } else {
                        tex_idx = tmp_tex_idx.?;
                    }
                    self.texture_metadata[texture_size_index][tex_idx].usage += 1;
                }

                const use_alpha = tsp_instruction.use_alpha == 1;
                const use_offset = isp_tsp_instruction.offset == 1; // FIXME: I did not find a way to validate what I'm doing with the offset color yet.

                const clamp_u = tsp_instruction.clamp_uv & 0b10 != 0;
                const clamp_v = tsp_instruction.clamp_uv & 0b01 != 0;

                const flip_u = tsp_instruction.flip_uv & 0b10 != 0 and !clamp_u;
                const flip_v = tsp_instruction.flip_uv & 0b01 != 0 and !clamp_v;

                const u_addr_mode = if (clamp_u) wgpu.AddressMode.clamp_to_edge else if (flip_u) wgpu.AddressMode.mirror_repeat else wgpu.AddressMode.repeat;
                const v_addr_mode = if (clamp_v) wgpu.AddressMode.clamp_to_edge else if (flip_v) wgpu.AddressMode.mirror_repeat else wgpu.AddressMode.repeat;

                const filter_mode: wgpu.FilterMode = if (tsp_instruction.filter_mode == 0) .nearest else .linear; // TODO: Add support for mipmapping (Tri-linear filtering) (And figure out what Pass A and Pass B means!).

                const sampler = if (textured) sampler_index(filter_mode, filter_mode, .linear, u_addr_mode, v_addr_mode) else sampler_index(.linear, .linear, .linear, .clamp_to_edge, .clamp_to_edge);

                const tex = VertexTextureInfo{
                    .index = tex_idx,
                    .shading = ShadingInstructions{
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
                    },
                };

                const first_vertex = display_list.vertex_strips.items[idx].verter_parameter_index;
                const last_vertex = display_list.vertex_strips.items[idx].verter_parameter_index + display_list.vertex_strips.items[idx].verter_parameter_count;

                for (display_list.vertex_parameters.items[first_vertex..last_vertex]) |vertex| {
                    // std.debug.print("Vertex: {any}\n", .{vertex});
                    switch (vertex) {
                        // Packed Color, Non-Textured
                        .Type0 => |v| {
                            // Sanity checks.
                            std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                            std.debug.assert(!textured);
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = @as(f32, @floatFromInt(v.base_color.r)) / 255.0,
                                    .g = @as(f32, @floatFromInt(v.base_color.g)) / 255.0,
                                    .b = @as(f32, @floatFromInt(v.base_color.b)) / 255.0,
                                    .a = if (use_alpha) @as(f32, @floatFromInt(v.base_color.a)) / 255.0 else 1.0,
                                },
                                .tex = tex,
                            });
                        },
                        // Non-Textured, Floating Color
                        .Type1 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .FloatingColor);
                            std.debug.assert(!textured);
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = v.r,
                                    .g = v.g,
                                    .b = v.b,
                                    .a = if (use_alpha) v.a else 1.0,
                                },
                                .tex = tex,
                            });
                        },
                        // Non-Textured, Intensity
                        .Type2 => |v| {
                            std.debug.assert(!textured);
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = v.base_intensity * face_color.r,
                                    .g = v.base_intensity * face_color.g,
                                    .b = v.base_intensity * face_color.b,
                                    .a = if (use_alpha) face_color.a else 1.0,
                                },
                                .tex = tex,
                            });
                        },
                        // Packed Color, Textured 32bit UV
                        .Type3 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = @as(f32, @floatFromInt(v.base_color.r)) / 255.0,
                                    .g = @as(f32, @floatFromInt(v.base_color.g)) / 255.0,
                                    .b = @as(f32, @floatFromInt(v.base_color.b)) / 255.0,
                                    .a = if (use_alpha) @as(f32, @floatFromInt(v.base_color.a)) / 255.0 else 1.0,
                                },
                                .offset_color = if (use_offset) .{
                                    .r = @as(f32, @floatFromInt(v.offset_color.r)) / 255.0,
                                    .g = @as(f32, @floatFromInt(v.offset_color.g)) / 255.0,
                                    .b = @as(f32, @floatFromInt(v.offset_color.b)) / 255.0,
                                    .a = @as(f32, @floatFromInt(v.offset_color.a)) / 255.0,
                                } else .{},
                                .u = v.u,
                                .v = v.v,
                                .tex = tex,
                            });
                        },
                        // Packed Color, Textured 16bit UV
                        .Type4 => |v| {
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = @as(f32, @floatFromInt(v.base_color.r)) / 255.0,
                                    .g = @as(f32, @floatFromInt(v.base_color.g)) / 255.0,
                                    .b = @as(f32, @floatFromInt(v.base_color.b)) / 255.0,
                                    .a = if (use_alpha) @as(f32, @floatFromInt(v.base_color.a)) / 255.0 else 1.0,
                                },
                                .offset_color = if (use_offset) .{
                                    .r = @as(f32, @floatFromInt(v.offset_color.r)) / 255.0,
                                    .g = @as(f32, @floatFromInt(v.offset_color.g)) / 255.0,
                                    .b = @as(f32, @floatFromInt(v.offset_color.b)) / 255.0,
                                    .a = @as(f32, @floatFromInt(v.offset_color.a)) / 255.0,
                                } else .{},
                                .u = @bitCast(@as(u32, v.uv.u) << 16),
                                .v = @bitCast(@as(u32, v.uv.v) << 16),
                                .tex = tex,
                            });
                        },
                        // Floating Color, Textured
                        .Type5 => |v| {
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = v.base_r,
                                    .g = v.base_g,
                                    .b = v.base_b,
                                    .a = if (use_alpha) v.base_a else 1.0,
                                },
                                .offset_color = if (use_offset) .{
                                    .r = v.offset_r,
                                    .g = v.offset_g,
                                    .b = v.offset_b,
                                    .a = v.offset_a,
                                } else .{},
                                .u = v.u,
                                .v = v.v,
                                .tex = tex,
                            });
                        },
                        // Floating Color, Textured 16bit UV
                        .Type6 => |v| {
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = v.base_r,
                                    .g = v.base_g,
                                    .b = v.base_b,
                                    .a = if (use_alpha) v.base_a else 1.0,
                                },
                                .offset_color = if (use_offset) .{
                                    .r = v.offset_r,
                                    .g = v.offset_g,
                                    .b = v.offset_b,
                                    .a = v.offset_a,
                                } else .{},
                                .u = @bitCast(@as(u32, v.uv.u) << 16),
                                .v = @bitCast(@as(u32, v.uv.v) << 16),
                                .tex = tex,
                            });
                        },
                        // Intensity
                        .Type7 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .IntensityMode1 or parameter_control_word.obj_control.col_type == .IntensityMode2);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = v.base_intensity * face_color.r,
                                    .g = v.base_intensity * face_color.g,
                                    .b = v.base_intensity * face_color.b,
                                    .a = if (use_alpha) face_color.a else 1.0,
                                },
                                .offset_color = if (use_offset) .{
                                    .r = v.offset_intensity * face_offset_color.r,
                                    .g = v.offset_intensity * face_offset_color.g,
                                    .b = v.offset_intensity * face_offset_color.b,
                                    .a = face_offset_color.a,
                                } else .{},
                                .u = v.u,
                                .v = v.v,
                                .tex = tex,
                            });
                        },
                        // Intensity, 16bit UV
                        .Type8 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .IntensityMode1 or parameter_control_word.obj_control.col_type == .IntensityMode2);
                            std.debug.assert(textured);
                            try self.vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .base_color = .{
                                    .r = v.base_intensity * face_color.r,
                                    .g = v.base_intensity * face_color.g,
                                    .b = v.base_intensity * face_color.b,
                                    .a = if (use_alpha) face_color.a else 1.0,
                                },
                                .offset_color = if (use_offset) .{
                                    .r = v.offset_intensity * face_offset_color.r,
                                    .g = v.offset_intensity * face_offset_color.g,
                                    .b = v.offset_intensity * face_offset_color.b,
                                    .a = face_offset_color.a,
                                } else .{},
                                .u = @bitCast(@as(u32, v.uv.u) << 16),
                                .v = @bitCast(@as(u32, v.uv.v) << 16),
                                .tex = tex,
                            });
                        },
                        .SpriteType0, .SpriteType1 => {
                            var vs = gen_sprite_vertices(vertex);
                            for (&vs) |*v| {
                                v.base_color.r = @as(f32, @floatFromInt(sprite_base_color.r)) / 255.0;
                                v.base_color.g = @as(f32, @floatFromInt(sprite_base_color.g)) / 255.0;
                                v.base_color.b = @as(f32, @floatFromInt(sprite_base_color.b)) / 255.0;
                                v.base_color.a = if (use_alpha) @as(f32, @floatFromInt(sprite_base_color.a)) / 255.0 else 1.0;
                                // FIXME: This is wrong.
                                //if (use_offset)
                                //    v.offset_color = .{
                                //        .r = @as(f32, @floatFromInt(sprite_offset_color.r)) / 255.0,
                                //        .g = @as(f32, @floatFromInt(sprite_offset_color.g)) / 255.0,
                                //        .b = @as(f32, @floatFromInt(sprite_offset_color.b)) / 255.0,
                                //        .a = @as(f32, @floatFromInt(sprite_offset_color.a)) / 255.0,
                                //    };
                                v.tex = tex;
                                self.min_depth = @min(self.min_depth, 1.0 / v.z);
                                self.max_depth = @max(self.max_depth, 1.0 / v.z);

                                try self.vertices.append(v.*);
                            }
                        },
                        else => {
                            renderer_log.err(termcolor.red("Unsupported vertex type {any}"), .{vertex});
                            @panic("Unsupported vertex type");
                        },
                    }

                    self.min_depth = @min(self.min_depth, 1.0 / self.vertices.getLast().z);
                    self.max_depth = @max(self.max_depth, 1.0 / self.vertices.getLast().z);
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

                    var pipeline = self.passes[@intFromEnum(list_type)].pipelines.getPtr(pipeline_key) orelse put: {
                        try self.passes[@intFromEnum(list_type)].pipelines.put(pipeline_key, PipelineMetadata.init(self._allocator));
                        break :put self.passes[@intFromEnum(list_type)].pipelines.getPtr(pipeline_key).?;
                    };

                    const draw_call_key = .{ .sampler = sampler, .user_clip = display_list.vertex_strips.items[idx].user_clip };

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
        if (self.vertices.items.len > 0) {
            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, FirstVertex * @sizeOf(Vertex), Vertex, self.vertices.items);

            var index = FirstIndex;
            for (&self.passes) |*pass| {
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

        self.opaque_modifier_volumes.clearRetainingCapacity();
        self.modifier_volume_vertices.clearRetainingCapacity();

        std.mem.swap(std.ArrayList(HollyModule.ModifierVolume), &self.opaque_modifier_volumes, &gpu._ta_opaque_modifier_volumes);
        for (gpu._ta_volume_triangles.items) |triangle| {
            try self.modifier_volume_vertices.append(.{ triangle.ax, triangle.ay, triangle.az, 1.0 });
            try self.modifier_volume_vertices.append(.{ triangle.bx, triangle.by, triangle.bz, 1.0 });
            try self.modifier_volume_vertices.append(.{ triangle.cx, triangle.cy, triangle.cz, 1.0 });
        }

        gpu._ta_volume_triangles.clearRetainingCapacity();
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.modifier_volume_vertex_buffer).?, 0, [4]f32, self.modifier_volume_vertices.items);
    }

    fn convert_clipping(self: *Renderer, user_clip: ?HollyModule.UserTileClipInfo) HollyModule.UserTileClipInfo {
        if (user_clip) |uc| {
            // FIXME: Handle other usages.
            //        Use Stencil for OutsideEnabled
            if (uc.usage == .InsideEnabled) {
                const factor = @divTrunc(self.resolution.width, NativeResolution.width);
                return .{
                    .usage = .InsideEnabled,
                    .x = @max(0, factor * uc.x),
                    .y = @max(0, factor * uc.y),
                    .width = @min(factor * uc.width, self.resolution.width),
                    .height = @min(factor * uc.height, self.resolution.height),
                };
            }
        }
        return .{ .usage = .InsideEnabled, .x = 0, .y = 0, .width = self.resolution.width, .height = self.resolution.height };
    }

    // Convert Framebuffer from native 640*480 to window resolution
    pub fn blit_framebuffer(self: *Renderer) void {
        if (self.read_framebuffer_enabled) {
            const gctx = self._gctx;

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

                    pass.setPipeline(gctx.lookupResource(self.blit_pipeline).?);

                    pass.setBindGroup(0, framebuffer_resize_bind_group, &.{});
                    pass.drawIndexed(4, 1, 0, 0, 0);
                }

                break :commands encoder.finish(null);
            };
            defer commands.release();

            gctx.submit(&.{commands});
        }
    }

    pub fn render(self: *Renderer) !void {
        const gctx = self._gctx;

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

            // TODO: Modify the passes to have two ouputs: one for area 0 and one for area 1 (pixels without shadow bit and 'two volumes' will simply output the same color twice)
            //       Then the 'apply' pass of the modifier volume will simply select between the two outputs, rather than doing any computation.
            {
                const color_attachments = [_]wgpu.RenderPassColorAttachment{ .{
                    .view = gctx.lookupResource(self.resized_framebuffer_texture_view).?,
                    .load_op = if (self.read_framebuffer_enabled) .load else .clear,
                    .store_op = .store,
                }, .{
                    .view = gctx.lookupResource(self.resized_framebuffer_area1_texture_view).?,
                    .load_op = .clear,
                    .store_op = .store,
                } };
                const depth_attachment = wgpu.RenderPassDepthStencilAttachment{
                    .view = depth_view,
                    .depth_load_op = .clear,
                    .depth_store_op = .store,
                    .depth_clear_value = 1.0,
                    .stencil_load_op = .clear,
                    .stencil_store_op = .discard,
                    .stencil_clear_value = 0,
                    .stencil_read_only = false,
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
                });
                pass.setPipeline(gctx.lookupResource(background_pipeline).?);
                pass.setBindGroup(1, gctx.lookupResource(self.sampler_bind_groups[sampler_index(.linear, .linear, .linear, .clamp_to_edge, .clamp_to_edge)]).?, &.{});
                pass.drawIndexed(FirstIndex, 1, 0, 0, 0);

                // Opaque and PunchThrough geometry
                inline for (.{ HollyModule.ListType.Opaque, HollyModule.ListType.PunchThrough }) |list_type| {
                    var it = self.passes[@intFromEnum(list_type)].pipelines.iterator();
                    while (it.next()) |entry| {
                        // FIXME: We should also check if at least one of the draw calls is not empty (we're keeping them around even if they are empty right now).
                        if (entry.value_ptr.*.draw_calls.count() > 0) {
                            const pl = try self.get_or_put_opaque_pipeline(entry.key_ptr.*);
                            pass.setPipeline(gctx.lookupResource(pl).?);

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

            if (self.opaque_modifier_volumes.items.len > 0) {
                //  - Write to stencil buffer
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
                        .depth_clear_value = 1.0,
                        .depth_read_only = false,
                        .stencil_load_op = .clear,
                        .stencil_store_op = .store,
                        .stencil_clear_value = 0,
                        .stencil_read_only = false,
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
                    pass.setPipeline(gctx.lookupResource(self.modifier_volume_pipeline).?);

                    for (self.opaque_modifier_volumes.items) |volume| {
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
                        .depth_clear_value = 1.0,
                        .depth_read_only = true,
                        .stencil_load_op = .undef,
                        .stencil_store_op = .undef,
                        .stencil_clear_value = 0,
                        .stencil_read_only = true,
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
                    pass.setPipeline(gctx.lookupResource(self.modifier_volume_apply_pipeline).?);
                    pass.setBindGroup(0, mva_bind_group, &.{});

                    pass.setStencilReference(0);
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
            const heads_info = gctx.lookupResourceInfo(self.list_heads_buffer).?;
            const init_heads_info = gctx.lookupResourceInfo(self.init_list_heads_buffer).?;
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
            };

            const horizontal_slices: u32 = 4;
            const slice_size = self.resolution.height / horizontal_slices;
            for (0..horizontal_slices) |i| {
                const start_y: u32 = @as(u32, @intCast(i)) * slice_size;

                const oit_uniform_mem = gctx.uniformsAllocate(struct { max_fragments: u32, target_width: u32, start_y: u32 }, 1);
                const LinkedListNodeSize = 4 * 4 + 4 + 4 + 4;
                oit_uniform_mem.slice[0].max_fragments = @intCast(self.get_max_storage_buffer_binding_size() / LinkedListNodeSize);
                oit_uniform_mem.slice[0].target_width = self.resolution.width;
                oit_uniform_mem.slice[0].start_y = start_y;

                {
                    // Clear lists
                    encoder.copyBufferToBuffer(init_heads_info.gpuobj.?, 0, heads_info.gpuobj.?, 0, heads_info.size);

                    const pass = encoder.beginRenderPass(oit_render_pass_info);
                    defer {
                        pass.end();
                        pass.release();
                    }

                    pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                    pass.setIndexBuffer(ib_info.gpuobj.?, .uint32, 0, ib_info.size);

                    const translucent_pass = self.passes[@intFromEnum(HollyModule.ListType.Translucent)];

                    pass.setPipeline(gctx.lookupResource(self.translucent_pipeline).?);

                    pass.setBindGroup(0, bind_group, &.{uniform_mem.offset});
                    pass.setBindGroup(2, translucent_bind_group, &.{oit_uniform_mem.offset});

                    var it = translucent_pass.pipelines.iterator();
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
                {
                    const pass = encoder.beginComputePass(null);
                    defer {
                        pass.end();
                        pass.release();
                    }
                    const num_groups = [2]u32{ @divExact(self.resolution.width, 8), @divExact(self.resolution.height, horizontal_slices * 8) };
                    pass.setPipeline(gctx.lookupResource(self.blend_pipeline).?);

                    pass.setBindGroup(0, blend_bind_group, &.{oit_uniform_mem.offset});

                    pass.dispatchWorkgroups(num_groups[0], num_groups[1], 1);
                }
            }

            // TODO: Translucent Modifier Volume

            break :commands encoder.finish(null);
        };
        defer commands.release();

        gctx.submit(&.{commands});
    }

    pub fn draw(self: *Renderer) void {
        const gctx = self._gctx;

        const back_buffer_view = gctx.swapchain.getCurrentTextureView();
        defer back_buffer_view.release();

        const commands = commands: {
            const encoder = gctx.device.createCommandEncoder(null);
            defer encoder.release();

            // Blit to screen
            {
                const vb_info = gctx.lookupResourceInfo(self.blit_to_window_vertex_buffer).?;
                const ib_info = gctx.lookupResourceInfo(self.blit_index_buffer).?;
                const blit_bind_group = gctx.lookupResource(self.blit_bind_group).?;

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

                pass.setPipeline(gctx.lookupResource(self.blit_pipeline).?);

                pass.setBindGroup(0, blit_bind_group, &.{});
                pass.drawIndexed(4, 1, 0, 0, 0);
            }

            break :commands encoder.finish(null);
        };
        defer commands.release();

        gctx.submit(&.{commands});
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
        self._gctx.releaseResource(self.init_list_heads_buffer);
        self._gctx.releaseResource(self.linked_list_buffer);

        self._gctx.releaseResource(self.translucent_bind_group);

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

        const blit_bind_group_layout = create_blit_bind_group_layout(self._gctx);
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
        const head_size = (1 + self.resolution.width * self.resolution.height) * @sizeOf(u32);
        const list_size = self.get_max_storage_buffer_binding_size();

        self.list_heads_buffer = self._gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .storage = true },
            .size = head_size,
        });
        self.init_list_heads_buffer = self._gctx.createBuffer(.{
            .usage = .{ .map_write = true, .copy_src = true },
            .size = head_size,
            .mapped_at_creation = true,
        });

        const init_buffer = self._gctx.lookupResourceInfo(self.init_list_heads_buffer).?.gpuobj.?;
        const mapped = init_buffer.getMappedRange(u32, 0, head_size / 4);
        @memset(mapped.?, 0xFFFFFFFF);
        init_buffer.unmap();

        self.linked_list_buffer = self._gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .storage = true },
            .size = list_size,
        });
    }

    fn create_translucent_bind_group(self: *@This()) void {
        self.translucent_bind_group = self._gctx.createBindGroup(self.translucent_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = 3 * @sizeOf(u32) },
            .{ .binding = 1, .buffer_handle = self.list_heads_buffer, .offset = 0, .size = (1 + self.resolution.width * self.resolution.height) * @sizeOf(u32) },
            .{ .binding = 2, .buffer_handle = self.linked_list_buffer, .offset = 0, .size = self.get_max_storage_buffer_binding_size() },
            .{ .binding = 3, .texture_view_handle = self.depth_only_texture_view },
        });
    }

    fn create_blend_bind_group(self: *@This()) void {
        self.blend_bind_group = self._gctx.createBindGroup(self.blend_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = self._gctx.uniforms.buffer, .offset = 0, .size = 3 * @sizeOf(u32) },
            .{ .binding = 1, .buffer_handle = self.list_heads_buffer, .offset = 0, .size = (1 + self.resolution.width * self.resolution.height) * @sizeOf(u32) },
            .{ .binding = 2, .buffer_handle = self.linked_list_buffer, .offset = 0, .size = self.get_max_storage_buffer_binding_size() },
            .{ .binding = 3, .texture_view_handle = self.resized_framebuffer_copy_texture_view },
            .{ .binding = 4, .texture_view_handle = self.resized_framebuffer_texture_view },
        });
    }

    fn get_max_storage_buffer_binding_size(_: *const @This()) u64 {
        // FIXME: No idea why this always fails.
        // var r: zgpu.wgpu.SupportedLimits = undefined;
        // if (!self._gctx.device.getLimits(&r))
        //     renderer_log.err("Failed to get device limits.", .{});
        // return r.limits.max_storage_buffer_binding_size;
        return 134217728; // FIXME: Hardcoded 'cause I can't be bothered to make it work correctly right now.
    }
};
