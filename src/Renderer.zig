const std = @import("std");

const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;

const Holly = @import("Holly.zig");

const Vertex = struct {
    x: f32,
    y: f32,
    z: f32,
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
\\  // @group(0) @binding(0) var<uniform> object_to_clip: mat4x4<f32>;
\\  @group(0) @binding(0) var<uniform> color: vec3<f32>;
\\  struct VertexOut {
\\      @builtin(position) position_clip: vec4<f32>,
\\      @location(0) color: vec3<f32>,
\\  }
\\  @vertex fn main(
\\      @location(0) position: vec3<f32>,
\\      @location(1) uv: vec2<f32>,
\\  ) -> VertexOut {
\\      var output: VertexOut;
\\      output.position_clip = vec4(position, 1.0);
\\      output.color = vec3(1.0, 1.0, 1.0);
\\      return output;
\\  }
;
const wgsl_fs =
\\  @fragment fn main(
\\      @location(0) color: vec3<f32>,
\\  ) -> @location(0) vec4<f32> {
\\      return vec4(color, 1.0);
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
            }};

            const vertex_attributes = [_]wgpu.VertexAttribute{
                .{ .format = .float32x3, .offset = 0, .shader_location = 0 },
                .{ .format = .float32x2, .offset = @offsetOf(Vertex, "u"), .shader_location = 1 },
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
            //.{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = @sizeOf(zm.Mat) },
            .{ .binding = 0, .buffer_handle = gctx.uniforms.buffer, .offset = 0, .size = 3 * @sizeOf(f32) },
        });

        // Create a vertex buffer.
        const vertex_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .vertex = true },
            .size = 3 * @sizeOf(Vertex),
        });
        const vertex_data = [_]Vertex{
            .{ .x = 0.0, .y = 0.5, .z = 0.0, .u = 0.0, .v = 0.0 },
            .{ .x = -0.5, .y = -0.5, .z = 0.0, .u = 0.0, .v = 0.0 },
            .{ .x = 0.5, .y = -0.5, .z = 0.0, .u = 0.0, .v = 0.0 },
        };
        gctx.queue.writeBuffer(gctx.lookupResource(vertex_buffer).?, 0, Vertex, vertex_data[0..]);

        // Create an index buffer.
        const index_buffer = gctx.createBuffer(.{
            .usage = .{ .copy_dst = true, .index = true },
            .size = 3 * @sizeOf(u32),
        });
        const index_data = [_]u32{ 0, 1, 2 };
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

            ._gctx = gctx,
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *Renderer) void {
        self.polygons.deinit();
    }

    pub fn push_polygon(self: *Renderer, polygon: Holly.Polygon, vertices: *const std.ArrayList(Holly.VertexParameter)) void {
        _ = polygon;
        var p = Polygon.init(self.allocator);

        for (vertices.items) |vertex| {
            switch (vertex) {
                .Type1 => {
                    p.vertices.append(.{
                        .x = vertex.x,
                        .y = vertex.y,
                        .z = vertex.z,
                        .u = vertex.u,
                        .v = vertex.v,
                    });
                },
                else => {
                    unreachable;
                },
            }
        }
        self.polygons.append(p) catch unreachable;
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

                const color: [3]f32 = .{ 1.0, 0.0, 0.0 };

                // Draw triangle 1.
                {
                    //const object_to_world = zm.mul(zm.rotationY(t), zm.translation(-1.0, 0.0, 0.0));
                    //const object_to_clip = zm.mul(object_to_world, cam_world_to_clip);
                    //
                    const mem = gctx.uniformsAllocate([3]f32, 1);
                    mem.slice[0] = color;

                    pass.setBindGroup(0, bind_group, &.{mem.offset});
                    pass.drawIndexed(3, 1, 0, 0, 0);
                }

                // Draw triangle 2.
                {
                    //const object_to_world = zm.mul(zm.rotationY(0.75 * t), zm.translation(1.0, 0.0, 0.0));
                    //const object_to_clip = zm.mul(object_to_world, cam_world_to_clip);
                    //
                    const mem = gctx.uniformsAllocate([3]f32, 1);
                    mem.slice[0] = color;

                    pass.setBindGroup(0, bind_group, &.{mem.offset});
                    pass.drawIndexed(3, 1, 0, 0, 0);
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
