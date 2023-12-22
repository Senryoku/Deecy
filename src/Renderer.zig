const std = @import("std");

const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;

const termcolor = @import("termcolor.zig");

const Holly = @import("Holly.zig");

// First 1024 values of the Moser de Bruijin sequence, Textures on the dreamcast are limited to 1024*1024 pixels.
var moser_de_bruijin_sequence: [1024]u32 = .{0} ** 1024;

// Returns the indice of the z-order curve for the given coordinates.
pub fn zorder_curve(x: u32, y: u32) u32 {
    return (moser_de_bruijin_sequence[x] << 1) | moser_de_bruijin_sequence[y];
}

pub fn to_tiddled_index(i: u32, w: u32) u32 {
    return zorder_curve(i % w, i / w);
}

const Vertex = struct {
    x: f32,
    y: f32,
    z: f32,
    r: f32,
    g: f32,
    b: f32,
    a: f32,
    u: f32,
    v: f32,
    tex: u32,
    tex_size: [2]u16,
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

    control_word: Holly.TextureControlWord = .{},
    index: u32 = Renderer.MaxTextures,
    usage: u32 = Unused,
    size: [2]u16 = .{ 0, 0 },
    uv_offset: [2]f32 = .{ 0, 0 },
};

pub const Renderer = struct {
    const MaxTextures: u32 = 256; // FIXME: Not sure what's a good value.

    texture_metadata: [MaxTextures]TextureMetadata = .{.{}} ** MaxTextures,

    polygons: std.ArrayList(Holly.Polygon),

    pipeline: zgpu.RenderPipelineHandle,
    bind_group: zgpu.BindGroupHandle,

    vertex_buffer: zgpu.BufferHandle,
    index_buffer: zgpu.BufferHandle,

    texture_array: zgpu.TextureHandle,
    texture_array_view: zgpu.TextureViewHandle,

    sampler: zgpu.SamplerHandle,

    depth_texture: zgpu.TextureHandle,
    depth_texture_view: zgpu.TextureViewHandle,

    index_count: u32,

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

        const pipeline_layout = gctx.createPipelineLayout(&.{bind_group_layout});
        defer gctx.releaseResource(pipeline_layout);

        const pipeline = pipline: {
            const vs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_vs, "vs");
            defer vs_module.release();

            const fs_module = zgpu.createWgslShaderModule(gctx.device, wgsl_fs, "fs");
            defer fs_module.release();

            const color_targets = [_]wgpu.ColorTargetState{.{
                .format = zgpu.GraphicsContext.swapchain_format,
                .blend = &wgpu.BlendState{
                    // FIXME: These actually depends on the polygon, not sure how I'll handle that yet.
                    .color = .{ .operation = .add, .src_factor = .src_alpha, .dst_factor = .one_minus_src_alpha },
                    .alpha = .{ .operation = .add, .src_factor = .one, .dst_factor = .zero },
                },
            }};

            const vertex_attributes = [_]wgpu.VertexAttribute{
                .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
                .{ .format = .float32x4, .offset = @offsetOf(Vertex, "r"), .shader_location = 1 },
                .{ .format = .float32x2, .offset = @offsetOf(Vertex, "u"), .shader_location = 2 },
                .{ .format = .uint32, .offset = @offsetOf(Vertex, "tex"), .shader_location = 3 },
                .{ .format = .uint16x2, .offset = @offsetOf(Vertex, "tex_size"), .shader_location = 4 },
                .{ .format = .float32x2, .offset = @offsetOf(Vertex, "uv_offset"), .shader_location = 5 },
            };
            const vertex_buffers = [_]wgpu.VertexBufferLayout{.{
                .array_stride = @sizeOf(Vertex),
                .attribute_count = vertex_attributes.len,
                .attributes = &vertex_attributes,
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
                    .depth_compare = .less,
                },
                .fragment = &wgpu.FragmentState{
                    .module = fs_module,
                    .entry_point = "main",
                    .target_count = color_targets.len,
                    .targets = &color_targets,
                },
            };
            break :pipline gctx.createRenderPipeline(pipeline_layout, pipeline_descriptor);
        };

        const sampler = gctx.createSampler(.{});

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

        const bind_group = gctx.createBindGroup(bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = 4 * 4 * @sizeOf(f32) },
            .{ .binding = 1, .texture_view_handle = texture_array_view },
            .{ .binding = 2, .sampler_handle = sampler },
        });

        const vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = 4096 * @sizeOf(Vertex), // FIXME: Arbitrary size for testing
        });

        const index_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .index = true },
            .size = 4096 * @sizeOf(u32), // FIXME: Arbitrary size for testing
        });

        // Create a depth texture and its 'view'.
        const depth = createDepthTexture(gctx);

        return .{
            .polygons = std.ArrayList(Holly.Polygon).init(allocator),

            .pipeline = pipeline,
            .bind_group = bind_group,
            .vertex_buffer = vertex_buffer,
            .index_buffer = index_buffer,

            .texture_array = texture_array,
            .texture_array_view = texture_array_view,
            .sampler = sampler,

            .depth_texture = depth.texture,
            .depth_texture_view = depth.view,

            .index_count = 0,

            ._gctx = gctx,
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *Renderer) void {
        self.polygons.deinit();
    }

    pub fn update_background(self: *Renderer) void {
        _ = self;

        // Not sure how I'll handle that yet.

        // This won't be done here, but only the necessary data will be passed down here.

        // 1. Get address in ISP_BACKGND_T
        // 2. Get depth from ISP_BACKGND_D
        // Data in VRAM @ISP_BACKGND_T:
        //   ISP/TSP Instruction Word
        //   TSP Instruction Word
        //   Texture Control Word
        //   [Vertices 1 to 3]
        // Vertice 4 is computed based on other vertices (rectangular polygon).
    }

    fn get_texture_index(self: *Renderer, control_word: Holly.TextureControlWord) u32 {
        for (0..Renderer.MaxTextures) |i| {
            if (self.texture_metadata[i].usage != TextureMetadata.Unused and self.texture_metadata[i].control_word.address == control_word.address) {
                return @intCast(i);
            }
        }
        return Renderer.MaxTextures;
    }

    fn upload_texture(self: *Renderer, gpu: *Holly.Holly, tsp_instruction: Holly.TSPInstructionWord, texture_control_word: Holly.TextureControlWord) u32 {
        std.debug.print("[Upload] tsp_instruction: {any}\n", .{tsp_instruction});
        std.debug.print("[Upload] texture_control_word: {any}\n", .{texture_control_word});

        const u_size: u16 = (@as(u16, 1) << @intCast(@as(u5, 3) + tsp_instruction.texture_u_size));
        const v_size: u16 = (@as(u16, 1) << @intCast(@as(u5, 3) + tsp_instruction.texture_v_size));

        const twiddled = texture_control_word.scan_order == 0;
        // FIXME: Handle twiddled textures.
        std.debug.assert(texture_control_word.mip_mapped == 0);
        std.debug.assert(texture_control_word.stride_select == 0); // Not implemented
        std.debug.assert(texture_control_word.vq_compressed == 0); // Not implemented

        var pixels = self._allocator.alloc(u8, @as(u32, 4) * u_size * v_size) catch unreachable;
        defer self._allocator.free(pixels);

        const addr: u32 = 8 * @as(u32, texture_control_word.address); // given in units of 64-bits.

        switch (texture_control_word.pixel_format) {
            .ARGB1555 => {
                for (0..(@as(u32, u_size) * v_size)) |i| {
                    const pixel_idx = if (twiddled) to_tiddled_index(@intCast(i), u_size) else i;
                    const pixel: Holly.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * pixel_idx]))).* };
                    pixels[i * 4 + 0] = @as(u8, pixel.arbg1555.r) << 3;
                    pixels[i * 4 + 1] = @as(u8, pixel.arbg1555.g) << 3;
                    pixels[i * 4 + 2] = @as(u8, pixel.arbg1555.b) << 3;
                    pixels[i * 4 + 3] = @as(u8, pixel.arbg1555.a) * 0xFF; // FIXME: TESTING
                }
            },
            .RGB565 => {
                for (0..(@as(u32, u_size) * v_size)) |i| {
                    const pixel_idx = if (twiddled) to_tiddled_index(@intCast(i), u_size) else i;
                    const pixel: Holly.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * pixel_idx]))).* };
                    pixels[i * 4 + 0] = @as(u8, pixel.rgb565.r) << 3;
                    pixels[i * 4 + 1] = @as(u8, pixel.rgb565.g) << 2;
                    pixels[i * 4 + 2] = @as(u8, pixel.rgb565.b) << 3;
                    pixels[i * 4 + 3] = 255;
                }
            },
            .ARGB4444 => {
                for (0..(@as(u32, u_size) * v_size)) |i| {
                    const pixel_idx = if (twiddled) to_tiddled_index(@intCast(i), u_size) else i;
                    const pixel: Holly.Color16 = .{ .value = @as(*const u16, @alignCast(@ptrCast(&gpu.vram[addr + 2 * pixel_idx]))).* };
                    pixels[i * 4 + 0] = @as(u8, pixel.argb4444.r) << 4;
                    pixels[i * 4 + 1] = @as(u8, pixel.argb4444.g) << 4;
                    pixels[i * 4 + 2] = @as(u8, pixel.argb4444.b) << 4;
                    pixels[i * 4 + 3] = @as(u8, pixel.argb4444.a) << 4;
                }
            },
            else => {
                std.debug.print(termcolor.red("[Holly] Unsupported pixel format {any}\n"), .{texture_control_word.pixel_format});
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

    fn check_texture_usage(self: *Renderer, gpu: *Holly.Holly) void {
        for (0..Renderer.MaxTextures) |i| {
            if (self.texture_metadata[i].usage == 0) {
                self.texture_metadata[i].usage = TextureMetadata.Unused;
            } else if (self.texture_metadata[i].usage != TextureMetadata.Unused) {
                if (gpu.vram[self.texture_metadata[i].control_word.address + 0] != 0xc0 or
                    gpu.vram[self.texture_metadata[i].control_word.address + 1] != 0xff or
                    gpu.vram[self.texture_metadata[i].control_word.address + 2] != 0xee or
                    gpu.vram[self.texture_metadata[i].control_word.address + 3] != 0x0f)
                {
                    std.debug.print(termcolor.yellow("[Renderer] Detected outdated texture #{d}. TODO!\n"), .{i});
                }
            }
        }
    }

    pub fn update(self: *Renderer, gpu: *Holly.Holly) !void {
        var vertices = std.ArrayList(Vertex).init(self._allocator);
        defer vertices.deinit();
        var indices = std.ArrayList(u32).init(self._allocator);
        defer indices.deinit();

        self.reset_texture_usage();
        defer self.check_texture_usage(gpu);

        for (gpu.ta_display_lists) |display_list| {
            for (0..display_list.polygons.items.len) |idx| {
                const start: u32 = @intCast(vertices.items.len);

                const global_parameter = @as(*const Holly.GenericGlobalParameter, @ptrCast(&display_list.polygons.items[idx])).*;

                var tex_idx: u32 = 0;
                if (global_parameter.parameter_control_word.obj_control.texture == 1) {
                    tex_idx = self.get_texture_index(global_parameter.texture_control);
                    if (tex_idx == Renderer.MaxTextures) {
                        tex_idx = self.upload_texture(gpu, global_parameter.tsp_instruction, global_parameter.texture_control);
                    }
                    self.texture_metadata[tex_idx].usage += 1;
                }

                const clamp_uv = global_parameter.tsp_instruction.clamp_uv == 1;

                const flip_u = global_parameter.tsp_instruction.flip_uv & 0x1 == 1 and !clamp_uv;
                const flip_v = (global_parameter.tsp_instruction.flip_uv >> 1) & 0x1 == 1 and !clamp_uv;
                if (flip_u or flip_v) {
                    std.debug.print(termcolor.yellow("[Renderer] TODO: Flip UV!\n"), .{});
                }

                for (display_list.vertex_parameters.items[idx].items) |vertex| {
                    switch (vertex) {
                        .Type0 => |v| {
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = @as(f32, @floatFromInt(v.base_color.r)) / 255.0,
                                .g = @as(f32, @floatFromInt(v.base_color.g)) / 255.0,
                                .b = @as(f32, @floatFromInt(v.base_color.b)) / 255.0,
                                .a = @as(f32, @floatFromInt(v.base_color.a)) / 255.0,
                                .u = 0.0,
                                .v = 0.0,
                                .tex = TextureMetadata.Unused,
                                .tex_size = .{ 0, 0 },
                            });
                        },
                        .Type3 => |v| {
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = @as(f32, @floatFromInt(v.base_color.r)) / 255.0,
                                .g = @as(f32, @floatFromInt(v.base_color.g)) / 255.0,
                                .b = @as(f32, @floatFromInt(v.base_color.b)) / 255.0,
                                .a = @as(f32, @floatFromInt(v.base_color.a)) / 255.0,
                                .u = v.u,
                                .v = v.v,
                                .tex = tex_idx,
                                .tex_size = self.texture_metadata[tex_idx].size,
                            });
                        },
                        // TODO: Intensity
                        .Type7 => |v| {
                            try vertices.append(.{
                                .x = v.x,
                                .y = v.y,
                                .z = v.z,
                                .r = @as(f32, @floatFromInt(v.base_intensity)) / 255.0,
                                .g = @as(f32, @floatFromInt(v.base_intensity)) / 255.0,
                                .b = @as(f32, @floatFromInt(v.base_intensity)) / 255.0,
                                .a = @as(f32, @floatFromInt(v.base_intensity)) / 255.0,
                                .u = v.u,
                                .v = v.v,
                                .tex = tex_idx,
                                .tex_size = self.texture_metadata[tex_idx].size,
                            });
                        },
                        else => {
                            std.debug.print(termcolor.red("[Renderer] Unsupported vertex type {any}\n"), .{vertex});
                            @panic("Unsupported vertex type");
                        },
                    }
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
        }

        if (vertices.items.len > 0) {
            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.vertex_buffer).?, 0, Vertex, vertices.items);
            self._gctx.queue.writeBuffer(self._gctx.lookupResource(self.index_buffer).?, 0, u32, indices.items);

            self.index_count = @intCast(indices.items.len);
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

        const commands = commands: {
            const encoder = gctx.device.createCommandEncoder(null);
            defer encoder.release();

            pass: {
                const vb_info = gctx.lookupResourceInfo(self.vertex_buffer) orelse break :pass;
                const ib_info = gctx.lookupResourceInfo(self.index_buffer) orelse break :pass;
                const pipeline = gctx.lookupResource(self.pipeline) orelse break :pass;
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

                pass.setPipeline(pipeline);

                // We could use this matrix to convert our coordinates.
                // TODO: Try it out when I already have a first working version.
                const convert: [16]f32 = .{
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                };

                {
                    const mem = gctx.uniformsAllocate([4 * 4]f32, 1);
                    mem.slice[0] = convert;

                    pass.setBindGroup(0, bind_group, &.{mem.offset});
                    pass.drawIndexed(self.index_count, 1, 0, 0, 0);
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
