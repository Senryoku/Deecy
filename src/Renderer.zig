const std = @import("std");

const renderer_log = std.log.scoped(.renderer);

const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;

const termcolor = @import("termcolor.zig");

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

const fRGBA = struct {
    r: f32,
    g: f32,
    b: f32,
    a: f32,
};

const ShadingInstructions = packed struct(u32) {
    textured: u1 = 0,
    mode: HollyModule.TextureShadingInstruction = .Decal,
    ignore_alpha: u1 = 0,
    tex_u_size: u3,
    tex_v_size: u3,
    _: u22 = 0,
};

fn sampler_index(mag_filter: wgpu.FilterMode, min_filter: wgpu.FilterMode, mip_filter: wgpu.FilterMode, address_mode_u: wgpu.AddressMode, address_mode_v: wgpu.AddressMode) u8 {
    return @as(u8, @intFromEnum(address_mode_v)) << 5 | @as(u8, @intFromEnum(address_mode_u)) << 3 | @as(u8, @intFromEnum(mip_filter)) << 2 | @as(u8, @intFromEnum(min_filter)) << 1 | @as(u8, @intFromEnum(mag_filter));
}

const TextureIndex = u32;
const InvalidTextureIndex = std.math.maxInt(TextureIndex);

const VertexTextureInfo = struct {
    index: TextureIndex,
    shading: ShadingInstructions,
};

const Vertex = struct {
    x: f32,
    y: f32,
    z: f32,
    r: f32,
    g: f32,
    b: f32,
    a: f32,
    u: f32 = 0.0,
    v: f32 = 0.0,
    tex: VertexTextureInfo,
    uv_offset: [2]f32 = .{ 0, 0 },
};

const Polygon = struct {
    vertices: std.ArrayList(Vertex),

    pub fn init(allocator: std.mem.Allocator) Polygon {
        return .{
            .vertices = std.ArrayList(Vertex).init(allocator),
        };
    }
};

const wgsl_vs = @embedFile("./shaders/vs.wgsl");
const wgsl_fs = @embedFile("./shaders/fs.wgsl");
const blit_vs = @embedFile("./shaders/blit_vs.wgsl");
const blit_fs = @embedFile("./shaders/blit_fs.wgsl");

const TextureMetadata = struct {
    const Unused: u32 = 0xFFFFFFFF;

    control_word: HollyModule.TextureControlWord = .{},
    tsp_instruction: HollyModule.TSPInstructionWord = @bitCast(@as(u32, 0)), // For debugging
    index: TextureIndex = InvalidTextureIndex,
    usage: u32 = Unused,
    size: [2]u16 = .{ 0, 0 },
    uv_offset: [2]f32 = .{ 0, 0 },
    start_address: u32 = 0,
};

const PassMetadata = struct {
    pass_type: HollyModule.ListType,
    start_vertex: u32,
    vertex_count: u32,
    start_index: u32,
    index_count: u32,
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

    const dz = 1.0; // FIXME: There is no 'DZ' in the documentation. It needs to be computed from the plane equation.
    // Same thing, texture coordinates have to be deduced from other vertices.
    const du = r[0].u + r[3].u - r[2].u;
    const dv = r[0].v + r[3].v - r[2].v;
    r[1].z = dz;
    r[1].u = du;
    r[1].v = dv;

    return r;
}

pub const Renderer = struct {
    pub const MaxTextures: [8]u16 = .{ 256, 256, 256, 256, 128, 32, 8, 2 }; // Max texture count for each size. FIXME: Not sure what are good values.

    const FirstVertex: u32 = 4; // The 4 first vertices are reserved for the background.
    const FirstIndex: u32 = 5; // The 5 first indices are reserved for the background.

    // That's too much for the higher texture sizes, but that probably doesn't matter.
    texture_metadata: [8][256]TextureMetadata = [_][256]TextureMetadata{[_]TextureMetadata{.{}} ** 256} ** 8,

    framebuffer_resize_bind_group: zgpu.BindGroupHandle,

    blit_pipeline: zgpu.RenderPipelineHandle,
    blit_bind_group: zgpu.BindGroupHandle,
    blit_vertex_buffer: zgpu.BufferHandle,
    blit_index_buffer: zgpu.BufferHandle,

    opaque_pipeline: zgpu.RenderPipelineHandle,
    translucent_pipeline: zgpu.RenderPipelineHandle,
    bind_group: zgpu.BindGroupHandle,

    polygons: std.ArrayList(HollyModule.Polygon),

    vertex_buffer: zgpu.BufferHandle,
    index_buffer: zgpu.BufferHandle,

    texture_arrays: [8]zgpu.TextureHandle,
    texture_array_views: [8]zgpu.TextureViewHandle,

    // Intermediate texture to upload framebuffer from VRAM (and maybe downsample and read back from at some point?)
    framebuffer_texture: zgpu.TextureHandle,
    framebuffer_texture_view: zgpu.TextureViewHandle,
    // Framebuffer at window resolution to draw on
    resized_framebuffer_texture: zgpu.TextureHandle,
    resized_framebuffer_texture_view: zgpu.TextureViewHandle,

    sampler: zgpu.SamplerHandle, // TODO: Have a sample per texture? Or per texture type (we'd have to pass another index to the fragment shader)?

    depth_texture: zgpu.TextureHandle,
    depth_texture_view: zgpu.TextureViewHandle,

    passes: std.ArrayList(PassMetadata),

    max_depth: f32 = 1.0,

    _scratch_pad: []u8, // Used to avoid temporary allocations before GPU uploads for example. 4 * 1024 * 1024, since this is the maximum texture size supported by the DC.

    _gctx: *zgpu.GraphicsContext,
    _allocator: std.mem.Allocator,

    fn create_blit_bind_group_layout(gctx: *zgpu.GraphicsContext) zgpu.BindGroupLayoutHandle {
        return gctx.createBindGroupLayout(&.{
            zgpu.textureEntry(0, .{ .fragment = true }, .float, .tvdim_2d, false),
            zgpu.samplerEntry(1, .{ .fragment = true }, .filtering),
        });
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
                .width = 640,
                .height = 480,
                .depth_or_array_layers = 1,
            },
            .format = zgpu.GraphicsContext.swapchain_format,
            .mip_level_count = 1, // std.math.log2_int(u32, @max(1024, 1024)) + 1,
        });
        const framebuffer_texture_view = gctx.createTextureView(framebuffer_texture, .{});

        const resized_framebuffer = Renderer.create_resized_frammebuffer_texture(gctx);

        const vertex_attributes = [_]wgpu.VertexAttribute{
            .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
            .{ .format = .float32x4, .offset = @offsetOf(Vertex, "r"), .shader_location = 1 },
            .{ .format = .float32x2, .offset = @offsetOf(Vertex, "u"), .shader_location = 2 },
            .{ .format = .uint32x2, .offset = @offsetOf(Vertex, "tex"), .shader_location = 3 },
            .{ .format = .float32x2, .offset = @offsetOf(Vertex, "uv_offset"), .shader_location = 4 },
        };
        const vertex_buffers = [_]wgpu.VertexBufferLayout{.{
            .array_stride = @sizeOf(Vertex),
            .attribute_count = vertex_attributes.len,
            .attributes = &vertex_attributes,
        }};

        const bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .vertex = true }, .uniform, true, 0),
            zgpu.samplerEntry(1, .{ .fragment = true }, .filtering),
            zgpu.textureEntry(2, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(3, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(4, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(5, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(6, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(7, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(8, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.textureEntry(9, .{ .fragment = true }, .float, .tvdim_2d_array, false),
        });
        defer gctx.releaseResource(bind_group_layout);
        const pipeline_layout = gctx.createPipelineLayout(&.{bind_group_layout});
        defer gctx.releaseResource(pipeline_layout);

        const opaque_pipeline = opaque_pl: {
            const vs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_vs, "vs");
            defer vs_module.release();

            const fs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "fs");
            defer fs_module.release();

            const color_targets = [_]wgpu.ColorTargetState{.{
                .format = zgpu.GraphicsContext.swapchain_format,
                .blend = &wgpu.BlendState{
                    // FIXME: Opaque.
                    .color = .{ .operation = .add, .src_factor = .one, .dst_factor = .zero },
                    .alpha = .{ .operation = .add, .src_factor = .one, .dst_factor = .zero },
                },
            }};

            const pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = wgpu.VertexState{
                    .module = vs_module,
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
                    .format = .depth32_float,
                    .depth_write_enabled = true,
                    .depth_compare = .less_equal,
                },
                .fragment = &wgpu.FragmentState{
                    .module = fs_module,
                    .entry_point = "main",
                    .target_count = color_targets.len,
                    .targets = &color_targets,
                },
            };
            break :opaque_pl gctx.createRenderPipeline(pipeline_layout, pipeline_descriptor);
        };

        const translucent_pipeline = transluscent_pl: {
            const vs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_vs, "vs");
            defer vs_module.release();

            const fs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "fs");
            defer fs_module.release();

            const color_targets = [_]wgpu.ColorTargetState{.{
                .format = zgpu.GraphicsContext.swapchain_format,
                .blend = &wgpu.BlendState{
                    // FIXME: These actually depends on the polygon, not sure how I'll handle that yet.
                    .color = .{ .operation = .add, .src_factor = .src_alpha, .dst_factor = .one_minus_src_alpha },
                    // FIXME: And this is probably just wrong.
                    .alpha = .{ .operation = .add, .src_factor = .one, .dst_factor = .zero },
                },
            }};

            const pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = wgpu.VertexState{
                    .module = vs_module,
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
                    .format = .depth32_float,
                    .depth_write_enabled = false,
                    .depth_compare = .less_equal,
                },
                .fragment = &wgpu.FragmentState{
                    .module = fs_module,
                    .entry_point = "main",
                    .target_count = color_targets.len,
                    .targets = &color_targets,
                },
            };
            break :transluscent_pl gctx.createRenderPipeline(pipeline_layout, pipeline_descriptor);
        };

        const blit_bind_group_layout = create_blit_bind_group_layout(gctx);
        defer gctx.releaseResource(blit_bind_group_layout);
        const blit_pipeline_layout = gctx.createPipelineLayout(&.{blit_bind_group_layout});
        defer gctx.releaseResource(blit_pipeline_layout);

        const blit_pipeline = blit_pl: {
            const vs_module = zgpu.createWgslShaderModule(gctx.device, blit_vs, "vs");
            defer vs_module.release();

            const fs_module = zgpu.createWgslShaderModule(gctx.device, blit_fs, "fs");
            defer fs_module.release();

            const color_targets = [_]wgpu.ColorTargetState{.{
                .format = zgpu.GraphicsContext.swapchain_format,
            }};

            const blit_vertex_attributes = [_]wgpu.VertexAttribute{
                .{ .format = .float32x2, .offset = 0, .shader_location = 0 },
                .{ .format = .float32x2, .offset = 2 * @sizeOf(f32), .shader_location = 1 },
            };
            const blit_vertex_buffers = [_]wgpu.VertexBufferLayout{.{
                .array_stride = 4 * @sizeOf(f32),
                .attribute_count = blit_vertex_attributes.len,
                .attributes = &blit_vertex_attributes,
            }};

            const pipeline_descriptor = wgpu.RenderPipelineDescriptor{
                .vertex = wgpu.VertexState{
                    .module = vs_module,
                    .entry_point = "main",
                    .buffer_count = blit_vertex_buffers.len,
                    .buffers = &blit_vertex_buffers,
                },
                .primitive = wgpu.PrimitiveState{
                    .front_face = .ccw,
                    .cull_mode = .none,
                    .topology = .triangle_strip,
                    .strip_index_format = .uint16,
                },
                .depth_stencil = null,
                .fragment = &wgpu.FragmentState{
                    .module = fs_module,
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

        // Create an index buffer.
        const blit_index_data = [_]u16{ 0, 3, 1, 2 };
        const blit_index_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .index = true },
            .size = blit_index_data.len * @sizeOf(u16),
        });
        gctx.queue.writeBuffer(gctx.lookupResource(blit_index_buffer).?, 0, u16, blit_index_data[0..]);

        // TODO: Create samplers ahead of time, but I'm not sure I can actually pass an array of samplers in WGPU...
        //       An example of how this could work: (sampler_index would also be used when creating vertex data to assign samplers)
        if (false) {
            const samplers: [256]wgpu.SampleHandle = undefined;
            for (.{ wgpu.FilterMode.nearest, wgpu.FilterMode.linear }) |mag_filter| {
                for (.{ wgpu.FilterMode.nearest, wgpu.FilterMode.linear }) |min_filter| {
                    for (.{ wgpu.FilterMode.nearest, wgpu.FilterMode.linear }) |mip_filter| {
                        for (.{ wgpu.AddressMode.repeat, wgpu.AddressMode.mirror_repeat, wgpu.AddressMode.clamp_to_edge }) |u_addr_mode| {
                            for (.{ wgpu.AddressMode.repeat, wgpu.AddressMode.mirror_repeat, wgpu.AddressMode.clamp_to_edge }) |v_addr_mode| {
                                samplers[sampler_index(mag_filter, min_filter, mip_filter, u_addr_mode, v_addr_mode)] =
                                    gctx.createSampler(.{
                                    .mag_filter = mag_filter,
                                    .min_filter = min_filter,
                                    .mip_filter = mip_filter,
                                    .address_mode_u = u_addr_mode,
                                    .address_mode_v = v_addr_mode,
                                });
                            }
                        }
                    }
                }
            }
        }
        const sampler = gctx.createSampler(.{
            .mag_filter = .linear,
            .min_filter = .linear,
            .address_mode_u = .repeat,
            .address_mode_v = .repeat,
        });

        const framebuffer_resize_bind_group = gctx.createBindGroup(blit_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = framebuffer_texture_view },
            .{ .binding = 1, .sampler_handle = sampler },
        });
        const blit_bind_group = gctx.createBindGroup(blit_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = resized_framebuffer.view },
            .{ .binding = 1, .sampler_handle = sampler },
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
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = 4 * @sizeOf(f32) },
            .{ .binding = 1, .sampler_handle = sampler },
            .{ .binding = 2, .texture_view_handle = texture_array_views[0] },
            .{ .binding = 3, .texture_view_handle = texture_array_views[1] },
            .{ .binding = 4, .texture_view_handle = texture_array_views[2] },
            .{ .binding = 5, .texture_view_handle = texture_array_views[3] },
            .{ .binding = 6, .texture_view_handle = texture_array_views[4] },
            .{ .binding = 7, .texture_view_handle = texture_array_views[5] },
            .{ .binding = 8, .texture_view_handle = texture_array_views[6] },
            .{ .binding = 9, .texture_view_handle = texture_array_views[7] },
        });

        const vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = 64 * 4096 * @sizeOf(Vertex), // FIXME: Arbitrary size for testing
        });

        const index_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .index = true },
            .size = 64 * 16384 * @sizeOf(u32), // FIXME: Arbitrary size for testing
        });

        // Create a depth texture and its 'view'.
        const depth = create_depth_texture(gctx);

        return .{
            .polygons = std.ArrayList(HollyModule.Polygon).init(allocator),

            .blit_pipeline = blit_pipeline,
            .blit_bind_group = blit_bind_group,
            .blit_vertex_buffer = blit_vertex_buffer,
            .blit_index_buffer = blit_index_buffer,

            .framebuffer_resize_bind_group = framebuffer_resize_bind_group,

            .framebuffer_texture = framebuffer_texture,
            .framebuffer_texture_view = framebuffer_texture_view,
            .resized_framebuffer_texture = resized_framebuffer.texture,
            .resized_framebuffer_texture_view = resized_framebuffer.view,

            .opaque_pipeline = opaque_pipeline,
            .translucent_pipeline = translucent_pipeline,

            .bind_group = bind_group,
            .vertex_buffer = vertex_buffer,
            .index_buffer = index_buffer,

            .texture_arrays = texture_arrays,
            .texture_array_views = texture_array_views,

            .sampler = sampler,

            .depth_texture = depth.texture,
            .depth_texture_view = depth.view,

            .passes = std.ArrayList(PassMetadata).init(allocator),

            ._scratch_pad = try allocator.alloc(u8, 4 * 1024 * 1024),

            ._gctx = gctx,
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *Renderer) void {
        self.passes.deinit();
        self.polygons.deinit();
        self._allocator.free(self._scratch_pad);
        // FIXME: I have a lot of resources to destroy.
    }

    fn get_texture_index(self: *Renderer, size_index: u3, control_word: HollyModule.TextureControlWord) ?TextureIndex {
        for (0..Renderer.MaxTextures[size_index]) |i| {
            if (self.texture_metadata[size_index][i].usage != TextureMetadata.Unused and self.texture_metadata[size_index][i].control_word.address == control_word.address) {
                return @intCast(i);
            }
        }
        return null;
    }

    inline fn bgra_from_16bits_color(format: HollyModule.TexturePixelFormat, val: u16, twiddled: bool) [4]u8 {
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
                    @as(u8, pixel.argb4444.b) << 4 | @as(u8, pixel.argb4444.b) >> 4,
                    @as(u8, pixel.argb4444.g) << 4 | @as(u8, pixel.argb4444.g) >> 4,
                    @as(u8, pixel.argb4444.r) << 4 | @as(u8, pixel.argb4444.r) >> 4,
                    @as(u8, pixel.argb4444.a) << 4 | @as(u8, pixel.argb4444.a) >> 4,
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

        const twiddled = texture_control_word.scan_order == 0;
        const size_index = tsp_instruction.texture_u_size;

        // NOTE: This is used by stride textures. Stride textures actual size can be smaller than their allocated size, but UV calculation are still done with it.
        const alloc_u_size = (@as(u16, 8) << tsp_instruction.texture_u_size);
        const alloc_v_size = (@as(u16, 8) << tsp_instruction.texture_v_size);

        const u_size: u16 = if (texture_control_word.scan_order == 1 and texture_control_word.stride_select == 1) @as(u16, 32) * texture_control_register.stride else alloc_u_size;
        const v_size: u16 = if (texture_control_word.scan_order == 0 and texture_control_word.mip_mapped == 1) u_size else alloc_v_size;

        @memset(self._scratch_pad, 0);

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

        const start_addr = if (texture_control_word.vq_compressed == 1) vq_index_addr else addr;

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
                                self.bgra_scratch_pad()[pixel_index] = bgra_from_16bits_color(texture_control_word.pixel_format, @truncate(texels >> @intCast(16 * tidx)), true);
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
                .ARGB1555, .RGB565, .ARGB4444 => {
                    for (0..v_size) |v| {
                        for (0..u_size) |u| {
                            const pixel_idx = v * u_size + u;
                            const texel_idx = if (twiddled) untwiddle(@intCast(u), @intCast(v), u_size, v_size) else pixel_idx;
                            self.bgra_scratch_pad()[pixel_idx] = bgra_from_16bits_color(texture_control_word.pixel_format, @as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * texel_idx]))).*, twiddled);
                        }
                    }
                },
                .Palette4BPP, .Palette8BPP => {
                    const palette_ram = @as([*]u32, @ptrCast(gpu._get_register(u32, .PALETTE_RAM_START)))[0..1024];
                    const palette_ctrl_ram = gpu._get_register(u32, .PAL_RAM_CTRL).* & 0b11; // NOTE: I'm not sure if this is garanteed to still be correct, I might have to check it when the palette is set (when the program writes to PALETTE_RAM).
                    const palette_selector: u10 = @truncate(if (texture_control_word.pixel_format == .Palette4BPP) (((@as(u32, @bitCast(texture_control_word)) >> 21) & 0b111111) << 4) else (((@as(u32, @bitCast(texture_control_word)) >> 25) & 0b11) << 8));

                    for (0..v_size) |v| {
                        for (0..u_size) |u| {
                            const pixel_idx = v * u_size + u;
                            const texel_idx = if (twiddled) untwiddle(@intCast(u), @intCast(v), u_size, v_size) else pixel_idx;
                            const ram_addr = if (texture_control_word.pixel_format == .Palette4BPP) texel_idx >> 1 else texel_idx;
                            const pixel_palette: u8 = gpu.vram[addr + ram_addr];
                            const offset = if (texture_control_word.pixel_format == .Palette4BPP) ((pixel_palette >> @intCast(4 * (texel_idx & 0x1))) & 0xF) else pixel_palette;
                            switch (palette_ctrl_ram) {
                                0x0, 0x1, 0x2 => { // ARGB1555, RGB565, ARGB4444. These happen to match the values of TexturePixelFormat.
                                    self.bgra_scratch_pad()[pixel_idx] = bgra_from_16bits_color(@enumFromInt(palette_ctrl_ram), @truncate(palette_ram[palette_selector + offset]), twiddled);
                                },
                                0x3 => { // ARGB8888
                                    @panic("Unsupported palette_ctrl_ram ARGB8888");
                                },
                                else => {
                                    renderer_log.err(termcolor.red("Invalid palette_ctrl_ram value {any}"), .{palette_ctrl_ram});
                                    @panic("Invalid palette_ctrl_ram value");
                                },
                            }
                        }
                    }
                },
                .YUV422 => {
                    if (twiddled) {
                        // FIXME: Given the data arangement suggested by the docs, I suspect I'll have to process 2x2 blocks in the twiddled case.
                        //        Still completely wrong, but at least it shouldn't crash because of alignment issues.
                        //     bit    63-48       47-32       31-16       15-0
                        //          Y1V (1,1)   Y1V (1,0)   Y0U (0,1)   Y0U (0,0)
                        for (0..v_size / 2) |v| {
                            for (0..u_size / 2) |u| {
                                const pixel_idx = 2 * v * u_size + 2 * u;
                                const texel_idx = untwiddle(@intCast(u), @intCast(v), u_size / 2, v_size / 2);
                                const halfwords = [4]u16{
                                    @bitCast(@as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * texel_idx + 0]))).*),
                                    @bitCast(@as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * texel_idx + 2]))).*),
                                    @bitCast(@as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * texel_idx + 4]))).*),
                                    @bitCast(@as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * texel_idx + 6]))).*),
                                };
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
                                const texel_idx = pixel_idx;
                                const texel: HollyModule.YUV422 = @bitCast(@as(*const u32, @alignCast(@ptrCast(&gpu.vram[addr + 2 * texel_idx]))).*);
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

        // Search for an unused texture index.
        for (0..Renderer.MaxTextures[size_index]) |i| {
            if (self.texture_metadata[size_index][i].usage == TextureMetadata.Unused) {
                self.texture_metadata[size_index][i] = .{
                    .control_word = texture_control_word,
                    .tsp_instruction = tsp_instruction,
                    .index = @intCast(i),
                    .usage = 0,
                    // NOTE: This is used for UV calculation in the shaders.
                    //       In the case of stride textures, we still need to use the power of two allocation size for UV calculation, not the actual texture size.
                    .size = .{ alloc_u_size, alloc_v_size },
                    .start_address = start_addr,
                };

                // Fill with repeating texture data when v_size < u_size to avoid vertical wrapping artifacts.
                // FIXME: We should do the same the stride textures when u_size < alloc_u_size.
                for (0..u_size / v_size) |part| {
                    self._gctx.queue.writeTexture(
                        .{
                            .texture = self._gctx.lookupResource(self.texture_arrays[size_index]).?,
                            .origin = .{ .y = @intCast(v_size * part), .z = @intCast(i) },
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

                // Write to VRAM at the texture address a signpost value to detect if it has been overwritten (and our GPU texture is thus outdated).
                // FIXME: Make sure this won't cause synchronization issues (Lock?).
                gpu.vram[start_addr + 0] = 0xc0;
                gpu.vram[start_addr + 1] = 0xff;
                gpu.vram[start_addr + 2] = 0xee;
                gpu.vram[start_addr + 3] = 0x0f;

                return @intCast(i);
            }
        }

        @panic("Out of textures slot");
    }

    fn reset_texture_usage(self: *Renderer, gpu: *HollyModule.Holly) void {
        for (0..Renderer.MaxTextures.len) |j| {
            for (0..Renderer.MaxTextures[j]) |i| {
                if (self.texture_metadata[j][i].usage != TextureMetadata.Unused) {
                    self.texture_metadata[j][i].usage = 0;
                    if (gpu.vram[self.texture_metadata[j][i].start_address + 0] != 0xc0 or
                        gpu.vram[self.texture_metadata[j][i].start_address + 1] != 0xff or
                        gpu.vram[self.texture_metadata[j][i].start_address + 2] != 0xee or
                        gpu.vram[self.texture_metadata[j][i].start_address + 3] != 0x0f)
                    {
                        // Texture appears to have changed in memory. Force re-upload.
                        self.texture_metadata[j][i].usage = TextureMetadata.Unused;
                    }
                }
            }
        }
    }

    fn check_texture_usage(self: *Renderer) void {
        for (0..Renderer.MaxTextures.len) |j| {
            for (0..Renderer.MaxTextures[j]) |i| {
                if (self.texture_metadata[j][i].usage == 0) {
                    self.texture_metadata[j][i].usage = TextureMetadata.Unused;
                }
            }
        }
    }

    pub fn update_framebuffer(self: *Renderer, gpu: *HollyModule.Holly) void {
        const SPG_CONTROL = gpu._get_register(HollyModule.SPG_CONTROL, .SPG_CONTROL).*;
        const FB_R_CTRL = gpu._get_register(HollyModule.FB_R_CTRL, .FB_R_CTRL).*;
        const FB_R_SOF1 = gpu._get_register(u32, .FB_R_SOF1).*;
        const FB_R_SOF2 = gpu._get_register(u32, .FB_R_SOF2).*;
        const FB_R_SIZE = gpu._get_register(HollyModule.FB_R_SIZE, .FB_R_SIZE).*;

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
            const tmp_tex_idx = self.get_texture_index(texture_size_index, texture_control);
            if (tmp_tex_idx == null) {
                tex_idx = self.upload_texture(gpu, tsp_instruction, texture_control);
            } else {
                tex_idx = tmp_tex_idx.?;
            }
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
            },
        };

        for (0..3) |i| {
            const vp = @as([*]const u32, @alignCast(@ptrCast(&gpu.vram[start + i * vertex_byte_size])));
            var u: f32 = 0;
            var v: f32 = 0;
            var base_color: HollyModule.PackedColor = @bitCast(vp[3]);
            var offset_color: HollyModule.PackedColor = @bitCast(vp[3]); // TODO
            if (isp_tsp_instruction.texture == 1) {
                if (isp_tsp_instruction.uv_16bit == 1) {
                    u = @as(f32, @bitCast(vp[3] >> 16));
                    v = @as(f32, @bitCast(vp[3] & 0xFFFF));
                    base_color = @bitCast(vp[4]);
                    offset_color = @bitCast(vp[5]);
                } else {
                    u = @bitCast(vp[3]);
                    v = @bitCast(vp[4]);
                    base_color = @bitCast(vp[5]);
                    offset_color = @bitCast(vp[5]);
                }
            }
            vertices[i] = Vertex{
                .x = @bitCast(vp[0]),
                .y = @bitCast(vp[1]),
                .z = @bitCast(vp[2]),
                .r = @as(f32, @floatFromInt(base_color.r)) / 255.0,
                .g = @as(f32, @floatFromInt(base_color.g)) / 255.0,
                .b = @as(f32, @floatFromInt(base_color.b)) / 255.0,
                .a = if (use_alpha) @as(f32, @floatFromInt(base_color.a)) / 255.0 else 1.0,
                .u = u,
                .v = v,
                .tex = tex,
            };
            // FIXME: We should probably render the background in a separate pass with depth test disabled.
            self.max_depth = @max(self.max_depth, 1 / vertices[i].z);
        }
        vertices[3] = Vertex{
            .x = vertices[1].x,
            .y = vertices[2].y,
            .z = vertices[2].z,
            // NOTE: I have no idea how the color is computed, looking at the boot menu, this seems right.
            .r = vertices[2].r,
            .g = vertices[2].g,
            .b = vertices[2].b,
            .a = vertices[2].a,

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
        var vertices = std.ArrayList(Vertex).init(self._allocator);
        defer vertices.deinit();
        var indices = std.ArrayList(u32).init(self._allocator);
        defer indices.deinit();

        self.reset_texture_usage(gpu);
        defer self.check_texture_usage();

        self.max_depth = 1.0;

        self.update_background(gpu);

        self.passes.clearRetainingCapacity();

        // TODO: Handle Modifier Volumes
        inline for (.{ HollyModule.ListType.Opaque, HollyModule.ListType.Translucent, HollyModule.ListType.PunchThrough }) |list_type| {
            const start_vertex = vertices.items.len;
            const start_index = indices.items.len;

            // Parameters specific to a polygon type
            var face_color: fRGBA = undefined; // In Intensity Mode 2, the face color is the one of the previous Intensity Mode 1 Polygon
            var face_offset_color: fRGBA = undefined;
            const display_list = gpu.ta_display_lists[@intFromEnum(list_type)];

            for (0..display_list.vertex_strips.items.len) |idx| {
                const start: u32 = @intCast(vertices.items.len);

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

                const clamp_uv = tsp_instruction.clamp_uv == 1;
                if (clamp_uv) {
                    renderer_log.debug(termcolor.yellow("[Renderer] TODO: Clamp UV!"), .{});
                    // TODO: Use an appropriate sampler with AddressMode ClampToEdge
                }

                const flip_u = tsp_instruction.flip_uv & 0x1 == 1 and !clamp_uv;
                const flip_v = (tsp_instruction.flip_uv >> 1) & 0x1 == 1 and !clamp_uv;
                if (flip_u or flip_v) {
                    renderer_log.debug(termcolor.yellow("[Renderer] TODO: Flip UV!"), .{});
                    // TODO: Use an appropriate sampler with AddressMode MirrorRepeat
                }

                const tex = VertexTextureInfo{
                    .index = tex_idx,
                    .shading = ShadingInstructions{
                        .textured = parameter_control_word.obj_control.texture,
                        .mode = tsp_instruction.texture_shading_instruction,
                        .ignore_alpha = tsp_instruction.ignore_texture_alpha,
                        .tex_u_size = tsp_instruction.texture_u_size,
                        .tex_v_size = tsp_instruction.texture_v_size,
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
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = @as(f32, @floatFromInt(v.base_color.r)) / 255.0,
                                .g = @as(f32, @floatFromInt(v.base_color.g)) / 255.0,
                                .b = @as(f32, @floatFromInt(v.base_color.b)) / 255.0,
                                .a = if (use_alpha) @as(f32, @floatFromInt(v.base_color.a)) / 255.0 else 1.0,
                                .tex = tex,
                            });
                        },
                        // Non-Textured, Floating Color
                        .Type1 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .FloatingColor);
                            std.debug.assert(!textured);
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = v.r,
                                .g = v.g,
                                .b = v.b,
                                .a = if (use_alpha) v.a else 1.0,
                                .tex = tex,
                            });
                        },
                        // Packed Color, Textured 32bit UV
                        .Type3 => |v| {
                            // TODO: Offset color
                            std.debug.assert(parameter_control_word.obj_control.col_type == .PackedColor);
                            std.debug.assert(textured);
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = @as(f32, @floatFromInt(v.base_color.r)) / 255.0,
                                .g = @as(f32, @floatFromInt(v.base_color.g)) / 255.0,
                                .b = @as(f32, @floatFromInt(v.base_color.b)) / 255.0,
                                .a = if (use_alpha) @as(f32, @floatFromInt(v.base_color.a)) / 255.0 else 1.0,
                                .u = v.u,
                                .v = v.v,
                                .tex = tex,
                            });
                        },
                        // Packed Color, Textured 16bit UV
                        .Type4 => |v| {
                            // TODO: Offset color
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = @as(f32, @floatFromInt(v.base_color.r)) / 255.0,
                                .g = @as(f32, @floatFromInt(v.base_color.g)) / 255.0,
                                .b = @as(f32, @floatFromInt(v.base_color.b)) / 255.0,
                                .a = if (use_alpha) @as(f32, @floatFromInt(v.base_color.a)) / 255.0 else 1.0,
                                .u = @bitCast(@as(u32, v.uv.u) << 16),
                                .v = @bitCast(@as(u32, v.uv.v) << 16),
                                .tex = tex,
                            });
                        },
                        // Floating Color, Textured
                        .Type5 => |v| {
                            // TODO: Offset color
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = v.base_r,
                                .g = v.base_g,
                                .b = v.base_b,
                                .a = if (use_alpha) v.base_a else 1.0,
                                .u = v.u,
                                .v = v.v,
                                .tex = tex,
                            });
                        },
                        // Intensity
                        .Type7 => |v| {
                            // TODO: Offset intensity
                            std.debug.assert(parameter_control_word.obj_control.col_type == .IntensityMode1 or parameter_control_word.obj_control.col_type == .IntensityMode2);
                            std.debug.assert(textured);
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = v.base_intensity * face_color.r,
                                .g = v.base_intensity * face_color.g,
                                .b = v.base_intensity * face_color.b,
                                .a = if (use_alpha) face_color.a else 1.0,
                                .u = v.u,
                                .v = v.v,
                                .tex = tex,
                            });
                        },
                        .Type8 => |v| {
                            std.debug.assert(parameter_control_word.obj_control.col_type == .IntensityMode1 or parameter_control_word.obj_control.col_type == .IntensityMode2);
                            std.debug.assert(textured);
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = v.base_intensity * face_color.r,
                                .g = v.base_intensity * face_color.g,
                                .b = v.base_intensity * face_color.b,
                                .a = if (use_alpha) face_color.a else 1.0,
                                .u = @bitCast(@as(u32, v.uv.u) << 16),
                                .v = @bitCast(@as(u32, v.uv.v) << 16),
                                .tex = tex,
                            });
                        },
                        .SpriteType0, .SpriteType1 => {
                            var vs = gen_sprite_vertices(vertex);
                            for (&vs) |*v| {
                                v.r = @as(f32, @floatFromInt(sprite_base_color.r)) / 255.0;
                                v.g = @as(f32, @floatFromInt(sprite_base_color.g)) / 255.0;
                                v.b = @as(f32, @floatFromInt(sprite_base_color.b)) / 255.0;
                                v.a = if (use_alpha) @as(f32, @floatFromInt(sprite_base_color.a)) / 255.0 else 1.0;
                                v.tex = tex;
                                self.max_depth = @max(self.max_depth, 1.0 / v.z);

                                try vertices.append(v.*);
                            }
                        },
                        else => {
                            renderer_log.err(termcolor.red("Unsupported vertex type {any}"), .{vertex});
                            @panic("Unsupported vertex type");
                        },
                    }

                    self.max_depth = @max(self.max_depth, 1.0 / vertices.getLast().z);
                }

                // Triangle Strips
                if (vertices.items.len - start < 3) {
                    renderer_log.err("Not enough vertices in strip: {d} vertices.", .{vertices.items.len - start});
                } else {
                    for (start..vertices.items.len) |i| {
                        try indices.append(@intCast(FirstVertex + i));
                    }
                    try indices.append(std.math.maxInt(u32)); // Primitive Restart: Ends the current triangle strip.
                }
            }

            if (vertices.items.len > start_vertex) {
                try self.passes.append(.{
                    .pass_type = list_type,
                    .start_vertex = @intCast(FirstVertex + start_vertex),
                    .vertex_count = @intCast(vertices.items.len - start_vertex),
                    .start_index = @intCast(FirstIndex + start_index),
                    .index_count = @intCast(indices.items.len - start_index),
                });
            }
        }

        if (vertices.items.len > 0) {
            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, FirstVertex * @sizeOf(Vertex), Vertex, vertices.items);
            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.index_buffer).?, FirstIndex * @sizeOf(u32), u32, indices.items);
        }
    }

    pub fn render(self: *Renderer) void {
        const gctx = self._gctx;

        const commands = commands: {
            const encoder = gctx.device.createCommandEncoder(null);
            defer encoder.release();

            // Convert Framebuffer from native 640*480 to window resolution
            {
                const vb_info = gctx.lookupResourceInfo(self.blit_vertex_buffer).?;
                const ib_info = gctx.lookupResourceInfo(self.blit_index_buffer).?;
                const framebuffer_resize_bind_group = gctx.lookupResource(self.framebuffer_resize_bind_group).?;

                const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                    .view = gctx.lookupResource(self.resized_framebuffer_texture_view).?,
                    .load_op = .clear,
                    .store_op = .store,
                }};
                const render_pass_info = wgpu.RenderPassDescriptor{
                    .color_attachment_count = color_attachments.len,
                    .color_attachments = &color_attachments,
                };

                const pass = encoder.beginRenderPass(render_pass_info);

                pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                pass.setIndexBuffer(ib_info.gpuobj.?, .uint16, 0, ib_info.size);

                pass.setPipeline(gctx.lookupResource(self.blit_pipeline).?);

                pass.setBindGroup(0, framebuffer_resize_bind_group, &.{});
                pass.drawIndexed(4, 1, 0, 0, 0);
                defer {
                    pass.end();
                    pass.release();
                }
            }

            {
                const vb_info = gctx.lookupResourceInfo(self.vertex_buffer).?;
                const ib_info = gctx.lookupResourceInfo(self.index_buffer).?;
                const bind_group = gctx.lookupResource(self.bind_group).?;
                const depth_view = gctx.lookupResource(self.depth_texture_view).?;

                const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                    .view = gctx.lookupResource(self.resized_framebuffer_texture_view).?,
                    .load_op = .load,
                    .store_op = .store,
                }};
                const depth_attachment = wgpu.RenderPassDepthStencilAttachment{
                    .view = depth_view,
                    .depth_load_op = .clear, // FIXME: We don't want to clear after the first pass.
                    .depth_store_op = .store,
                    .depth_clear_value = 1.0,
                };
                const render_pass_info = wgpu.RenderPassDescriptor{
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

                // Set uniforms
                const mem = gctx.uniformsAllocate([4]f32, 1);
                mem.slice[0][0] = self.max_depth;
                pass.setBindGroup(0, bind_group, &.{mem.offset});

                const opaque_pipeline = gctx.lookupResource(self.opaque_pipeline).?;
                const translucent_pipeline = gctx.lookupResource(self.translucent_pipeline).?;

                // Draw background
                pass.setPipeline(opaque_pipeline);
                pass.drawIndexed(FirstIndex, 1, 0, 0, 0);

                // FIXME: Draw the passes in order.
                //          Opaque
                //          Punch Through (Same as opaque, but discarding pixels with alpha = 0.0, I think)
                //          Opaque (and Punch Through) Modifier Volume
                //          Translucent
                //          Translucent Modifier Volume:
                for (self.passes.items) |pass_metadata| {
                    switch (pass_metadata.pass_type) {
                        .Opaque, .PunchThrough => {
                            pass.setPipeline(opaque_pipeline);
                        },
                        .Translucent => {
                            pass.setPipeline(translucent_pipeline);
                        },
                        else => {
                            renderer_log.err("Unsupported pass type {any}", .{pass_metadata.pass_type});
                            @panic("Unsupported pass type");
                        },
                    }

                    pass.drawIndexed(pass_metadata.index_count, 1, pass_metadata.start_index, 0, 0);
                }
            }

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
                const vb_info = gctx.lookupResourceInfo(self.blit_vertex_buffer).?;
                const ib_info = gctx.lookupResourceInfo(self.blit_index_buffer).?;
                const blit_bind_group = gctx.lookupResource(self.blit_bind_group).?;

                const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                    .view = back_buffer_view,
                    .load_op = .clear,
                    .store_op = .store,
                }};
                const render_pass_info = wgpu.RenderPassDescriptor{
                    .color_attachment_count = color_attachments.len,
                    .color_attachments = &color_attachments,
                };

                const pass = encoder.beginRenderPass(render_pass_info);

                pass.setVertexBuffer(0, vb_info.gpuobj.?, 0, vb_info.size);
                pass.setIndexBuffer(ib_info.gpuobj.?, .uint16, 0, ib_info.size);

                pass.setPipeline(gctx.lookupResource(self.blit_pipeline).?);

                pass.setBindGroup(0, blit_bind_group, &.{});
                pass.drawIndexed(4, 1, 0, 0, 0);
                defer {
                    pass.end();
                    pass.release();
                }
            }

            break :commands encoder.finish(null);
        };
        defer commands.release();

        gctx.submit(&.{commands});
    }

    pub fn on_resize(self: *@This()) void {
        // Release old depth texture.
        self._gctx.releaseResource(self.depth_texture_view);
        self._gctx.destroyResource(self.depth_texture);

        // Create a new depth texture to match the new window size.
        const depth = create_depth_texture(self._gctx);
        self.depth_texture = depth.texture;
        self.depth_texture_view = depth.view;

        // Same thing for our screen size framebuffer.
        self._gctx.releaseResource(self.resized_framebuffer_texture_view);
        self._gctx.destroyResource(self.resized_framebuffer_texture);

        const resized_framebuffer = create_resized_frammebuffer_texture(self._gctx);
        self.resized_framebuffer_texture = resized_framebuffer.texture;
        self.resized_framebuffer_texture_view = resized_framebuffer.view;

        const blit_bind_group_layout = create_blit_bind_group_layout(self._gctx);
        defer self._gctx.releaseResource(blit_bind_group_layout);

        self._gctx.releaseResource(self.blit_bind_group);
        self.blit_bind_group = self._gctx.createBindGroup(blit_bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .texture_view_handle = resized_framebuffer.view },
            .{ .binding = 1, .sampler_handle = self.sampler },
        });
    }

    fn create_depth_texture(gctx: *zgpu.GraphicsContext) struct {
        texture: zgpu.TextureHandle,
        view: zgpu.TextureViewHandle,
    } {
        const texture = gctx.createTexture(.{
            .usage = .{ .render_attachment = true },
            .dimension = .tdim_2d,
            .size = .{
                .width = gctx.swapchain_descriptor.width,
                .height = gctx.swapchain_descriptor.height,
                .depth_or_array_layers = 1,
            },
            .format = .depth32_float,
            .mip_level_count = 1,
            .sample_count = 1,
        });
        const view = gctx.createTextureView(texture, .{});
        return .{ .texture = texture, .view = view };
    }

    fn create_resized_frammebuffer_texture(gctx: *zgpu.GraphicsContext) struct {
        texture: zgpu.TextureHandle,
        view: zgpu.TextureViewHandle,
    } {
        const resized_framebuffer_texture = gctx.createTexture(.{
            .usage = .{ .texture_binding = true, .render_attachment = true },
            .size = .{
                .width = gctx.swapchain_descriptor.width,
                .height = gctx.swapchain_descriptor.height, // FIXME: Not dynamic.
                .depth_or_array_layers = 1,
            },
            .format = zgpu.GraphicsContext.swapchain_format,
            .mip_level_count = 1, // std.math.log2_int(u32, @max(1024, 1024)) + 1,
        });
        const resized_framebuffer_texture_view = gctx.createTextureView(resized_framebuffer_texture, .{});
        return .{ .texture = resized_framebuffer_texture, .view = resized_framebuffer_texture_view };
    }
};
