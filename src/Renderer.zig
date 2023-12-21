const std = @import("std");

const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;

const termcolor = @import("termcolor.zig");

const Holly = @import("Holly.zig");

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
const wgsl_vs =
\\  @group(0) @binding(0) var<uniform> conversion_matrix: mat4x4<f32>;
\\  struct VertexOut {
\\      @builtin(position) position_clip: vec4<f32>,
\\      @location(0) color: vec4<f32>,
\\  }
\\  @vertex fn main(
\\      @location(0) position: vec3<f32>,
\\      @location(1) color: vec4<f32>,
\\      @location(2) uv: vec2<f32>,
\\  ) -> VertexOut {
\\      var output: VertexOut;
\\      
\\      // output.position_clip = vec4(position, 1.0) * conversion_matrix;
\\      
\\      // Positions are supplied in screen space (0..640, 0..480)
\\      // Convert it to wgpu clip space (-1..1, -1..1)
\\
\\      let w = 1.0 / position.z;
\\
\\      let screen_size = vec2<f32>(640.0, 480.0); // TODO: Adjust depending on video mode.
\\      
\\      output.position_clip.x = position.x * 2.0 / screen_size.x - 1.0;
\\      output.position_clip.y = position.y * -2.0 / screen_size.y + 1.0;
\\      output.position_clip.z = 0.0; //position.z; // Core doesn't have a depth buffer
\\      output.position_clip.w = 1.0;
\\
\\      // Should we undo the projection? Not if we set w to 1.0, I think.
\\      // output.position_clip.w = w;
\\      // output.position_clip.x *= w;
\\      // output.position_clip.y *= w;
\\
\\      output.color = color;
\\      return output;
\\  }
;
const wgsl_fs =
\\  @fragment fn main(
\\      @location(0) color: vec4<f32>,
\\  ) -> @location(0) vec4<f32> {
\\      return color;
\\  }
// zig fmt: on
;

pub const Renderer = struct {
    polygons: std.ArrayList(Holly.Polygon),

    pipeline: zgpu.RenderPipelineHandle,
    bind_group: zgpu.BindGroupHandle,

    vertex_buffer: zgpu.BufferHandle,
    index_buffer: zgpu.BufferHandle,

    depth_texture: zgpu.TextureHandle,
    depth_texture_view: zgpu.TextureViewHandle,

    index_count: u32,

    _gctx: *zgpu.GraphicsContext,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) Renderer {
        const bind_group_layout = gctx.createBindGroupLayout(&.{
            zgpu.bufferEntry(0, .{ .vertex = true }, .uniform, true, 0),
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
                //.blend = &wgpu.BlendState{
                //    .color = .{ .operation = .add, .src_factor = .src_alpha, .dst_factor = .one_minus_src_alpha },
                //    .alpha = .{ .operation = .add, .src_factor = .one, .dst_factor = .zero },
                //},
            }};

            const vertex_attributes = [_]wgpu.VertexAttribute{
                .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
                .{ .format = .float32x4, .offset = @offsetOf(Vertex, "r"), .shader_location = 1 },
                .{ .format = .float32x2, .offset = @offsetOf(Vertex, "u"), .shader_location = 2 },
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

        const bind_group = gctx.createBindGroup(bind_group_layout, &[_]zgpu.BindGroupEntryInfo{
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = 4 * 4 * @sizeOf(f32) },
        });

        // Create a vertex buffer.
        const vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = 4096 * @sizeOf(Vertex), // FIXME: Arbitrary size for testing
        });

        //const vertex_data = [_]Vertex{
        //    .{ .x = 0.0, .y = 0.5, .z = 0.0, .u = 0.0, .v = 0.0 },
        //    .{ .x = -0.5, .y = -0.5, .z = 0.0, .u = 0.0, .v = 0.0 },
        //    .{ .x = 0.5, .y = -0.5, .z = 0.0, .u = 0.0, .v = 0.0 },
        //
        //};

        const vertex_data = [_]Vertex{};
        gctx.queue.writeBuffer(gctx.lookupResource(vertex_buffer).?, 0, Vertex, vertex_data[0..]);

        // Create an index buffer.
        const index_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .index = true },
            .size = 4096 * @sizeOf(u32), // FIXME: Arbitrary size for testing
        });
        //const index_data = [_]u32{ 0, 1, 2 };
        const index_data = [_]u32{};
        gctx.queue.writeBuffer(gctx.lookupResource(index_buffer).?, 0, u32, index_data[0..]);

        // Create a depth texture and its 'view'.
        const depth = createDepthTexture(gctx);

        return .{
            .polygons = std.ArrayList(Holly.Polygon).init(allocator),

            .pipeline = pipeline,
            .bind_group = bind_group,
            .vertex_buffer = vertex_buffer,
            .index_buffer = index_buffer,
            .depth_texture = depth.texture,
            .depth_texture_view = depth.view,

            .index_count = index_data.len,

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

    pub fn update(self: *Renderer, display_lists: *const [5]Holly.DisplayList) !void {
        var vertices = std.ArrayList(Vertex).init(self._allocator);
        defer vertices.deinit();
        var indices = std.ArrayList(u32).init(self._allocator);
        defer indices.deinit();

        for (display_lists.*) |display_list| {
            for (0..display_list.polygons.items.len) |idx| {
                const start: u32 = @intCast(vertices.items.len);

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
                            });
                        },
                        else => {
                            std.debug.print(termcolor.red("[Holly] Unsupported vertex type {any}\n"), .{vertex});
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
            //std.debug.print("Vertices: {any}\n", .{vertices.items});
            //std.debug.print("Indices: {any}\n", .{indices.items});

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
