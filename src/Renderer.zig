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

pub fn to_tiddled_index(i: u32, w: u32) u32 {
    return zorder_curve(i % w, i / w);
}

const fRGBA = struct {
    r: f32,
    g: f32,
    b: f32,
    a: f32,
};

const ShadingInstructions = packed struct(u16) {
    textured: u1 = 0,
    mode: HollyModule.TextureShadingInstruction = .Decal,
    ignore_alpha: u1 = 0,
    _: u12 = 0,
};

const TextureIndex = u16;

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
    tex_size: [2]u16 = .{ 1024, 1024 },
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

// zig fmt: off
const wgsl_vs = @embedFile("./shaders/vs.wgsl");
const wgsl_fs = @embedFile("./shaders/fs.wgsl");
// zig fmt: on

// TODO: Pack smaller textures into an Atlas.

const TextureMetadata = struct {
    const Unused: u32 = 0xFFFFFFFF;

    control_word: HollyModule.TextureControlWord = .{},
    index: TextureIndex = Renderer.MaxTextures,
    usage: u32 = Unused,
    size: [2]u16 = .{ 0, 0 },
    uv_offset: [2]f32 = .{ 0, 0 },
};

const PassMetadata = struct {
    pass_type: HollyModule.ListType,
    start_vertex: u32,
    vertex_count: u32,
    start_index: u32,
    index_count: u32,
};

pub const Renderer = struct {
    const MaxTextures: u32 = 256; // FIXME: Not sure what's a good value.
    const FirstVertex: u32 = 4; // The 4 first vertices are reserved for the background.
    const FirstIndex: u32 = 6; // The 6 first indices are reserved for the background.

    texture_metadata: [MaxTextures]TextureMetadata = .{.{}} ** MaxTextures,

    polygons: std.ArrayList(HollyModule.Polygon),

    opaque_pipeline: zgpu.RenderPipelineHandle,
    translucent_pipeline: zgpu.RenderPipelineHandle,
    bind_group: zgpu.BindGroupHandle,

    vertex_buffer: zgpu.BufferHandle,
    index_buffer: zgpu.BufferHandle,

    texture_array: zgpu.TextureHandle,
    texture_array_view: zgpu.TextureViewHandle,

    framebuffer_texture: zgpu.TextureHandle,
    framebuffer_texture_view: zgpu.TextureViewHandle,

    sampler: zgpu.SamplerHandle, // TODO: Have a sample per texture? Or per texture type (we'd have to pass another index to the fragment shader)?

    depth_texture: zgpu.TextureHandle,
    depth_texture_view: zgpu.TextureViewHandle,

    passes: std.ArrayList(PassMetadata),

    max_depth: f32 = 1.0,

    _gctx: *zgpu.GraphicsContext,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) Renderer {

        // FIXME: Make this comptime?
        moser_de_bruijin_sequence[0] = 0;
        for (1..moser_de_bruijin_sequence.len) |idx| {
            moser_de_bruijin_sequence[idx] = (moser_de_bruijin_sequence[idx - 1] + 0xAAAAAAAB) & 0x55555555;
        }

        const bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .vertex = true }, .uniform, true, 0),
            zgpu.textureEntry(1, .{ .fragment = true }, .float, .tvdim_2d_array, false),
            zgpu.samplerEntry(2, .{ .fragment = true }, .filtering),
        });
        defer gctx.releaseResource(bind_group_layout);

        const vertex_attributes = [_]wgpu.VertexAttribute{
            .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
            .{ .format = .float32x4, .offset = @offsetOf(Vertex, "r"), .shader_location = 1 },
            .{ .format = .float32x2, .offset = @offsetOf(Vertex, "u"), .shader_location = 2 },
            .{ .format = .uint16x2, .offset = @offsetOf(Vertex, "tex"), .shader_location = 3 },
            .{ .format = .uint16x2, .offset = @offsetOf(Vertex, "tex_size"), .shader_location = 4 },
            .{ .format = .float32x2, .offset = @offsetOf(Vertex, "uv_offset"), .shader_location = 5 },
        };
        const vertex_buffers = [_]wgpu.VertexBufferLayout{.{
            .array_stride = @sizeOf(Vertex),
            .attribute_count = vertex_attributes.len,
            .attributes = &vertex_attributes,
        }};

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
                    .topology = .triangle_list,
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
                    .topology = .triangle_list,
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
            break :transluscent_pl gctx.createRenderPipeline(pipeline_layout, pipeline_descriptor);
        };

        const sampler = gctx.createSampler(.{
            //.mag_filter = .linear,
            //.min_filter = .linear,
        });

        const texture_array = gctx.createTexture(.{
            .usage = .{ .texture_binding = true, .copy_dst = true },
            .size = .{
                .width = 1024, // FIXME: Should I create pools for each texture sizes?
                .height = 1024,
                .depth_or_array_layers = Renderer.MaxTextures,
            },
            .format = zgpu.imageInfoToTextureFormat(4, 1, false),
            .mip_level_count = 1, // std.math.log2_int(u32, @max(1024, 1024)) + 1,
        });
        const texture_array_view = gctx.createTextureView(texture_array, .{});

        const framebuffer_texture = gctx.createTexture(.{
            .usage = .{ .texture_binding = true, .copy_dst = true },
            .size = .{
                .width = 640,
                .height = 480,
                .depth_or_array_layers = 1,
            },
            .format = zgpu.imageInfoToTextureFormat(4, 1, false),
            .mip_level_count = 1, // std.math.log2_int(u32, @max(1024, 1024)) + 1,
        });
        const framebuffer_texture_view = gctx.createTextureView(framebuffer_texture, .{});

        const bind_group = gctx.createBindGroup(bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = 4 * @sizeOf(f32) },
            .{ .binding = 1, .texture_view_handle = texture_array_view },
            .{ .binding = 2, .sampler_handle = sampler },
        });

        const vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = 4096 * @sizeOf(Vertex), // FIXME: Arbitrary size for testing
        });

        const index_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .index = true },
            .size = 16384 * @sizeOf(u32), // FIXME: Arbitrary size for testing
        });

        // Create a depth texture and its 'view'.
        const depth = createDepthTexture(gctx);

        return .{
            .polygons = std.ArrayList(HollyModule.Polygon).init(allocator),

            .opaque_pipeline = opaque_pipeline,
            .translucent_pipeline = translucent_pipeline,

            .bind_group = bind_group,
            .vertex_buffer = vertex_buffer,
            .index_buffer = index_buffer,

            .texture_array = texture_array,
            .texture_array_view = texture_array_view,

            .framebuffer_texture = framebuffer_texture,
            .framebuffer_texture_view = framebuffer_texture_view,

            .sampler = sampler,

            .depth_texture = depth.texture,
            .depth_texture_view = depth.view,

            .passes = std.ArrayList(PassMetadata).init(allocator),

            ._gctx = gctx,
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *Renderer) void {
        self.polygons.deinit();
    }

    fn get_texture_index(self: *Renderer, control_word: HollyModule.TextureControlWord) ?TextureIndex {
        for (0..Renderer.MaxTextures) |i| {
            if (self.texture_metadata[i].usage != TextureMetadata.Unused and self.texture_metadata[i].control_word.address == control_word.address) {
                return @intCast(i);
            }
        }
        return null;
    }

    fn upload_texture(self: *Renderer, gpu: *HollyModule.Holly, tsp_instruction: HollyModule.TSPInstructionWord, texture_control_word: HollyModule.TextureControlWord) TextureIndex {
        renderer_log.debug("[Upload] tsp_instruction: {any}", .{tsp_instruction});
        renderer_log.debug("[Upload] texture_control_word: {any}", .{texture_control_word});

        const u_size: u16 = (@as(u16, 1) << @intCast(@as(u5, 3) + tsp_instruction.texture_u_size));
        const v_size: u16 = (@as(u16, 1) << @intCast(@as(u5, 3) + tsp_instruction.texture_v_size));

        const twiddled = texture_control_word.scan_order == 0;
        std.debug.assert(texture_control_word.mip_mapped == 0);
        std.debug.assert(texture_control_word.stride_select == 0); // Not implemented
        std.debug.assert(texture_control_word.vq_compressed == 0); // Not implemented

        // FIXME: Allocate a scratch pad once and stick to it?
        var pixels = self._allocator.alloc(u8, @as(u32, 4) * u_size * v_size) catch unreachable;
        defer self._allocator.free(pixels);

        const addr: u32 = 8 * @as(u32, texture_control_word.address); // given in units of 64-bits.

        switch (texture_control_word.pixel_format) {
            .ARGB1555 => {
                for (0..(@as(u32, u_size) * v_size)) |i| {
                    const pixel_idx = if (twiddled) to_tiddled_index(@intCast(i), u_size) else i;
                    const pixel: HollyModule.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * pixel_idx]))).* };
                    pixels[i * 4 + 0] = @as(u8, pixel.arbg1555.r) << 3;
                    pixels[i * 4 + 1] = @as(u8, pixel.arbg1555.g) << 3;
                    pixels[i * 4 + 2] = @as(u8, pixel.arbg1555.b) << 3;
                    pixels[i * 4 + 3] = @as(u8, pixel.arbg1555.a) * 0xFF; // FIXME: TESTING
                }
            },
            .RGB565 => {
                for (0..(@as(u32, u_size) * v_size)) |i| {
                    const pixel_idx = if (twiddled) to_tiddled_index(@intCast(i), u_size) else i;
                    const pixel: HollyModule.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * pixel_idx]))).* };
                    pixels[i * 4 + 0] = @as(u8, pixel.rgb565.r) << 3;
                    pixels[i * 4 + 1] = @as(u8, pixel.rgb565.g) << 2;
                    pixels[i * 4 + 2] = @as(u8, pixel.rgb565.b) << 3;
                    pixels[i * 4 + 3] = 255;
                }
            },
            .ARGB4444 => {
                for (0..(@as(u32, u_size) * v_size)) |i| {
                    const pixel_idx = if (twiddled) to_tiddled_index(@intCast(i), u_size) else i;
                    const pixel: HollyModule.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * pixel_idx]))).* };
                    pixels[i * 4 + 0] = @as(u8, pixel.argb4444.r) << 4;
                    pixels[i * 4 + 1] = @as(u8, pixel.argb4444.g) << 4;
                    pixels[i * 4 + 2] = @as(u8, pixel.argb4444.b) << 4;
                    pixels[i * 4 + 3] = @as(u8, pixel.argb4444.a) << 4;
                }
            },
            else => {
                renderer_log.err(termcolor.red("[Holly] Unsupported pixel format {any}"), .{texture_control_word.pixel_format});
                @panic("Unsupported pixel format");
            },
        }

        // Search for an unused texture index.
        for (0..Renderer.MaxTextures) |i| {
            if (self.texture_metadata[i].usage == TextureMetadata.Unused) {
                self.texture_metadata[i] = .{
                    .control_word = texture_control_word,
                    .index = @intCast(i),
                    .usage = 0,
                    .size = .{ u_size, v_size },
                };
                self._gctx.queue.writeTexture(
                    .{
                        .texture = self._gctx.lookupResource(self.texture_array).?,
                        .origin = .{ .z = @intCast(i) },
                    },
                    .{
                        .bytes_per_row = u_size * 4,
                        .rows_per_image = v_size,
                    },
                    .{ .width = u_size, .height = v_size, .depth_or_array_layers = 1 },
                    u8,
                    pixels,
                );

                // Write to VRAM at the texture address a signpost value to detect if it has been overwritten (and our GPU texture is thus outdated).
                // FIXME: Make sure this won't cause synchronization issues (Lock?).
                gpu.vram[texture_control_word.address + 0] = 0xc0;
                gpu.vram[texture_control_word.address + 1] = 0xff;
                gpu.vram[texture_control_word.address + 2] = 0xee;
                gpu.vram[texture_control_word.address + 3] = 0x0f;

                return @intCast(i);
            }
        }

        @panic("Out of textures slot");
    }

    fn reset_texture_usage(self: *Renderer) void {
        for (0..Renderer.MaxTextures) |i| {
            if (self.texture_metadata[i].usage != TextureMetadata.Unused)
                self.texture_metadata[i].usage = 0;
        }
    }

    fn check_texture_usage(self: *Renderer, gpu: *HollyModule.Holly) void {
        for (0..Renderer.MaxTextures) |i| {
            if (self.texture_metadata[i].usage == 0) {
                self.texture_metadata[i].usage = TextureMetadata.Unused;
            } else if (self.texture_metadata[i].usage != TextureMetadata.Unused) {
                if (gpu.vram[self.texture_metadata[i].control_word.address + 0] != 0xc0 or
                    gpu.vram[self.texture_metadata[i].control_word.address + 1] != 0xff or
                    gpu.vram[self.texture_metadata[i].control_word.address + 2] != 0xee or
                    gpu.vram[self.texture_metadata[i].control_word.address + 3] != 0x0f)
                {
                    renderer_log.warn(termcolor.yellow("[Renderer] Detected outdated texture #{d}. TODO!"), .{i});
                }
            }
        }
    }

    pub fn update_framebuffer(self: *Renderer, gpu: *HollyModule.Holly) void {
        const FB_R_CTRL = gpu._get_register(HollyModule.FB_R_CTRL, .FB_R_CTRL).*;
        const FB_C_SOF = gpu._get_register(u32, .FB_C_SOF).*;
        _ = FB_C_SOF; // Specify the starting address, in 32-bit units, for the frame that is currently being sent to the DAC.
        const FB_R_SOF1 = gpu._get_register(u32, .FB_R_SOF1).*;
        const FB_R_SOF2 = gpu._get_register(u32, .FB_R_SOF2).*;
        _ = FB_R_SOF2;
        const FB_R_SIZE = gpu._get_register(HollyModule.FB_R_SIZE, .FB_R_SIZE).*;

        // Enabled: We have to copy some data from VRAM.
        if (FB_R_CTRL.enable) {
            const addr: u32 = FB_R_SOF1;
            // TODO: Find a way to avoid unecessary uploads?

            // FIXME: Handle interlaced mode? Or not?
            renderer_log.info("Reading from framebuffer (from the PoV of the Holly Core) enabled.", .{});
            renderer_log.info("  FB_R_CTRL: {any}", .{FB_R_CTRL});
            renderer_log.info("  FB_R_SOF1: {X:0>8}", .{FB_R_SOF1});
            renderer_log.info("  FB_R_SIZE: {any}", .{FB_R_SIZE});

            // FIXME: Don't why it's half the framebuffer width.
            const u_size: u32 = 2 * (FB_R_SIZE.x_size + 1);
            const v_size: u32 = FB_R_SIZE.y_size + 1;

            const bytes_per_pixels: u32 = switch (FB_R_CTRL.format) {
                0, 1 => 2,
                2 => 3,
                3 => 4,
            };

            // FIXME: Use scratch pad or something.
            var pixels = self._allocator.alloc(u8, 4 * u_size * v_size) catch unreachable;
            defer self._allocator.free(pixels);

            for (0..v_size) |y| {
                for (0..u_size) |x| {
                    const pixel_idx = u_size * y + x;
                    const pixel_addr = addr + bytes_per_pixels * (y * (u_size + FB_R_SIZE.modulus - 1) + x);
                    switch (FB_R_CTRL.format) {
                        0x0 => { // 0555 RGB 16 bit
                            const pixel: HollyModule.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[pixel_addr]))).* };
                            pixels[pixel_idx * 4 + 0] = (@as(u8, pixel.arbg1555.r) << 3) | FB_R_CTRL.concat;
                            pixels[pixel_idx * 4 + 1] = (@as(u8, pixel.arbg1555.g) << 3) | FB_R_CTRL.concat;
                            pixels[pixel_idx * 4 + 2] = (@as(u8, pixel.arbg1555.b) << 3) | FB_R_CTRL.concat;
                            pixels[pixel_idx * 4 + 3] = 255;
                        },
                        0x1 => { // 565 RGB
                            const pixel: HollyModule.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[pixel_addr]))).* };
                            // FIXME: I think there's something wrong with FB_R_CTRL.concat here.
                            pixels[pixel_idx * 4 + 0] = (@as(u8, pixel.rgb565.r) << 3) | FB_R_CTRL.concat;
                            pixels[pixel_idx * 4 + 1] = (@as(u8, pixel.rgb565.g) << 2) | FB_R_CTRL.concat & 0b11;
                            pixels[pixel_idx * 4 + 2] = (@as(u8, pixel.rgb565.b) << 3) | FB_R_CTRL.concat;
                            pixels[pixel_idx * 4 + 3] = 255;
                        },
                        0x2 => { // 888 RGB 24 bit packed
                            const pixel: [3]u8 = @as([*]const u8, @alignCast(@ptrCast(&gpu.vram[pixel_addr])))[0..3].*;
                            pixels[pixel_idx * 4 + 0] = pixel[0];
                            pixels[pixel_idx * 4 + 1] = pixel[1];
                            pixels[pixel_idx * 4 + 2] = pixel[2];
                            pixels[pixel_idx * 4 + 3] = 255;
                        },
                        0x3 => { // 0888 RGB 32 bit
                            const pixel: [4]u8 = @as([*]const u8, @alignCast(@ptrCast(&gpu.vram[pixel_addr])))[0..4].*;
                            pixels[pixel_idx * 4 + 0] = pixel[1];
                            pixels[pixel_idx * 4 + 1] = pixel[2];
                            pixels[pixel_idx * 4 + 2] = pixel[3];
                            pixels[pixel_idx * 4 + 3] = 255;
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
                    .bytes_per_row = 4 * u_size,
                    .rows_per_image = v_size,
                },
                .{ .width = u_size, .height = v_size, .depth_or_array_layers = 1 },
                u8,
                pixels,
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

        // FIXME: I don't understand. In the boot menu for example, this depth value is 0.0,
        //        which doesn't make sense. The vertices z position looks more inline with what
        //        I understand of render pipeline.
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
            const tmp_tex_idx = self.get_texture_index(texture_control);
            if (tmp_tex_idx == null) {
                tex_idx = self.upload_texture(gpu, tsp_instruction, texture_control);
            } else {
                tex_idx = tmp_tex_idx.?;
            }
            self.texture_metadata[tex_idx].usage += 1;

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
            },
        };
        const tex_size = self.texture_metadata[tex_idx].size;

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
                .tex_size = tex_size,
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
            .tex_size = tex_size,
        };

        const indices: [6]u32 = .{ 0, 1, 2, 2, 1, 3 };

        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, 0, Vertex, &vertices);
        self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.index_buffer).?, 0, u32, &indices);
    }

    pub fn update(self: *Renderer, gpu: *HollyModule.Holly) !void {
        var vertices = std.ArrayList(Vertex).init(self._allocator);
        defer vertices.deinit();
        var indices = std.ArrayList(u32).init(self._allocator);
        defer indices.deinit();

        self.reset_texture_usage();
        defer self.check_texture_usage(gpu);

        self.max_depth = 1.0;

        self.update_background(gpu);

        self.passes.clearRetainingCapacity();

        // TODO: Handle Modifier Volumes
        inline for (.{ HollyModule.ListType.Opaque, HollyModule.ListType.Translucent, HollyModule.ListType.PunchThrough }) |list_type| {
            const start_vertex = vertices.items.len;
            const start_index = indices.items.len;

            // Parameters specific to a polygon type
            var face_color: fRGBA = undefined; // In Intensity Mode 2, the face color is the one of the previous Intensity Mode 1 Polygon
            const display_list = gpu.ta_display_lists[@intFromEnum(list_type)];

            for (0..display_list.polygons.items.len) |idx| {
                const start: u32 = @intCast(FirstVertex + vertices.items.len);

                // Generic Parameters
                var parameter_control_word: HollyModule.ParameterControlWord = undefined;
                var isp_tsp_instruction: HollyModule.ISPTSPInstructionWord = undefined;
                var tsp_instruction: HollyModule.TSPInstructionWord = undefined;
                var texture_control: HollyModule.TextureControlWord = undefined;

                switch (display_list.polygons.items[idx]) {
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
                    else => {
                        renderer_log.err("Unhandled polygon type: {any}", .{display_list.polygons.items[idx]});
                    },
                }

                var tex_idx: TextureIndex = 0;
                const textured = parameter_control_word.obj_control.texture == 1;
                if (textured) {
                    const tmp_tex_idx = self.get_texture_index(texture_control);
                    if (tmp_tex_idx == null) {
                        tex_idx = self.upload_texture(gpu, tsp_instruction, texture_control);
                    } else {
                        tex_idx = tmp_tex_idx.?;
                    }
                    self.texture_metadata[tex_idx].usage += 1;
                }

                const use_alpha = tsp_instruction.use_alpha == 1;

                const clamp_uv = tsp_instruction.clamp_uv == 1;

                const flip_u = tsp_instruction.flip_uv & 0x1 == 1 and !clamp_uv;
                const flip_v = (tsp_instruction.flip_uv >> 1) & 0x1 == 1 and !clamp_uv;
                if (flip_u or flip_v) {
                    renderer_log.warn(termcolor.yellow("[Renderer] TODO: Flip UV!"), .{});
                }

                const tex = VertexTextureInfo{
                    .index = tex_idx,
                    .shading = ShadingInstructions{
                        .textured = parameter_control_word.obj_control.texture,
                        .mode = tsp_instruction.texture_shading_instruction,
                        .ignore_alpha = tsp_instruction.ignore_texture_alpha,
                    },
                };

                for (display_list.vertex_parameters.items[idx].items) |vertex| {
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
                                .tex_size = self.texture_metadata[tex_idx].size,
                            });
                        },
                        // Intensity
                        .Type7 => |v| {
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
                                .tex_size = self.texture_metadata[tex_idx].size,
                            });
                        },
                        else => {
                            std.debug.print(termcolor.red("[Renderer] Unsupported vertex type {any}"), .{vertex});
                            @panic("Unsupported vertex type");
                        },
                    }

                    self.max_depth = @max(self.max_depth, 1.0 / vertices.getLast().z);
                }

                try indices.append(start + 0);
                try indices.append(start + 1);
                try indices.append(start + 2);
                // Triangle Strips
                for (0..display_list.vertex_parameters.items[idx].items.len - 3) |i| {
                    if (i % 2 == 0) {
                        try indices.append(@intCast(start + i + 1));
                        try indices.append(@intCast(start + i + 3));
                        try indices.append(@intCast(start + i + 2));
                    } else {
                        try indices.append(@intCast(start + i + 1));
                        try indices.append(@intCast(start + i + 2));
                        try indices.append(@intCast(start + i + 3));
                    }
                }
            }

            if (vertices.items.len > start_vertex) {
                try self.passes.append(.{
                    .pass_type = list_type,
                    .start_vertex = @intCast(start_vertex),
                    .vertex_count = @intCast(vertices.items.len - start_vertex),
                    .start_index = @intCast(start_index),
                    .index_count = @intCast(indices.items.len - start_index),
                });
            }
        }

        if (vertices.items.len > 0) {
            // Offsets: The 4 first vertices and 6 first indices are used by the background.
            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, FirstVertex * @sizeOf(Vertex), Vertex, vertices.items);
            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.index_buffer).?, FirstIndex * @sizeOf(u32), u32, indices.items);
        }
    }

    pub fn draw(self: *Renderer) void {
        const gctx = self._gctx;
        const fb_width = gctx.swapchain_descriptor.width;
        _ = fb_width;
        const fb_height = gctx.swapchain_descriptor.height;
        _ = fb_height;
        const t = @as(f32, @floatCast(gctx.stats.time));
        _ = t;

        const back_buffer_view = gctx.swapchain.getCurrentTextureView();
        defer back_buffer_view.release();

        // TODO: Draw framebuffer.

        const commands = commands: {
            const encoder = gctx.device.createCommandEncoder(null);
            defer encoder.release();

            pass: {
                const vb_info = gctx.lookupResourceInfo(self.vertex_buffer) orelse break :pass;
                const ib_info = gctx.lookupResourceInfo(self.index_buffer) orelse break :pass;
                const bind_group = gctx.lookupResource(self.bind_group) orelse break :pass;
                const depth_view = gctx.lookupResource(self.depth_texture_view) orelse break :pass;

                const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                    .view = back_buffer_view,
                    .load_op = .clear,
                    .store_op = .store,
                }};
                const depth_attachment = wgpu.RenderPassDepthStencilAttachment{
                    .view = depth_view,
                    .depth_load_op = .clear,
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

                for (self.passes.items) |pass_metadata| {
                    switch (pass_metadata.pass_type) {
                        .Opaque, .PunchThrough => {
                            const pipeline = gctx.lookupResource(self.opaque_pipeline) orelse break :pass;
                            pass.setPipeline(pipeline);
                        },
                        .Translucent => {
                            const pipeline = gctx.lookupResource(self.translucent_pipeline) orelse break :pass;
                            pass.setPipeline(pipeline);
                        },
                        else => {
                            unreachable;
                        },
                    }

                    {
                        const mem = gctx.uniformsAllocate([4]f32, 1);
                        mem.slice[0][0] = self.max_depth;

                        pass.setBindGroup(0, bind_group, &.{mem.offset});
                        pass.drawIndexed(pass_metadata.index_count, 1, pass_metadata.start_index, 0, 0);
                    }
                }
            }
            {
                const color_attachments = [_]wgpu.RenderPassColorAttachment{.{
                    .view = back_buffer_view,
                    .load_op = .load,
                    .store_op = .store,
                }};
                const render_pass_info = wgpu.RenderPassDescriptor{
                    .color_attachment_count = color_attachments.len,
                    .color_attachments = &color_attachments,
                };
                const pass = encoder.beginRenderPass(render_pass_info);
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
        const depth = createDepthTexture(self._gctx);
        self.depth_texture = depth.texture;
        self.depth_texture_view = depth.view;
    }

    fn createDepthTexture(gctx: *zgpu.GraphicsContext) struct {
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
};
