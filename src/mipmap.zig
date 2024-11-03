const std = @import("std");
const zgpu = @import("zgpu");

const ShaderSource = @embedFile("shaders/generate_mipmaps.wgsl");

bind_group_layout: zgpu.BindGroupLayoutHandle,
pipeline: zgpu.ComputePipelineHandle,

pub fn init(gctx: *zgpu.GraphicsContext) @This() {
    const bind_group_layout = gctx.createBindGroupLayout(&.{
        zgpu.textureEntry(0, .{ .compute = true }, .unfilterable_float, .tvdim_2d, false),
        zgpu.storageTextureEntry(1, .{ .compute = true }, .write_only, .bgra8_unorm, .tvdim_2d),
        zgpu.storageTextureEntry(2, .{ .compute = true }, .write_only, .bgra8_unorm, .tvdim_2d),
        zgpu.storageTextureEntry(3, .{ .compute = true }, .write_only, .bgra8_unorm, .tvdim_2d),
    });
    const pipeline_layout = gctx.createPipelineLayout(&.{
        bind_group_layout,
    });
    defer gctx.releaseResource(pipeline_layout);

    const cs_module = zgpu.createWgslShaderModule(gctx.device, ShaderSource, "generate_mipmaps");
    defer cs_module.release();

    return .{
        .bind_group_layout = bind_group_layout,
        .pipeline = gctx.createComputePipeline(pipeline_layout, .{
            .compute = .{
                .module = cs_module,
                .entry_point = "main",
            },
        }),
    };
}

pub fn deinit(self: *@This(), gctx: *zgpu.GraphicsContext) void {
    gctx.releaseResource(self.bind_group_layout);
    gctx.releaseResource(self.pipeline);
}

pub fn generate_mipmaps(self: @This(), gctx: *zgpu.GraphicsContext, texture: zgpu.TextureHandle, layer: u32) void {
    const texture_info = gctx.lookupResourceInfo(texture) orelse return;

    std.debug.assert(texture_info.usage.copy_dst == true);
    std.debug.assert(texture_info.dimension == .tdim_2d);
    std.debug.assert(texture_info.size.width == texture_info.size.height);
    std.debug.assert(texture_info.size.width >= 8 and texture_info.size.width <= 1024);
    std.debug.assert(std.math.isPowerOfTwo(texture_info.size.width));

    const commands = commands: {
        const encoder = gctx.device.createCommandEncoder(null);
        defer encoder.release();

        const pass = encoder.beginComputePass(null);
        pass.setPipeline(gctx.lookupResource(self.pipeline).?);

        const last_mip_level = texture_info.mip_level_count - 1;
        var base_mip_level: u32 = 0;
        while (base_mip_level < last_mip_level) {
            const src_texture_view = gctx.createTextureView(texture, .{
                .dimension = .tvdim_2d,
                .base_array_layer = layer,
                .array_layer_count = 1,
                .base_mip_level = base_mip_level,
                .mip_level_count = 1,
            });
            defer gctx.releaseResource(src_texture_view);
            const dst_texture_views = [3]zgpu.TextureViewHandle{
                gctx.createTextureView(texture, .{
                    .dimension = .tvdim_2d,
                    .base_array_layer = layer,
                    .array_layer_count = 1,
                    .base_mip_level = base_mip_level + 1,
                    .mip_level_count = 1,
                }),
                gctx.createTextureView(texture, .{
                    .dimension = .tvdim_2d,
                    .base_array_layer = layer,
                    .array_layer_count = 1,
                    .base_mip_level = base_mip_level + 2,
                    .mip_level_count = 1,
                }),
                gctx.createTextureView(texture, .{
                    .dimension = .tvdim_2d,
                    .base_array_layer = layer,
                    .array_layer_count = 1,
                    .base_mip_level = base_mip_level + 3,
                    .mip_level_count = 1,
                }),
            };
            defer {
                for (dst_texture_views) |dst_texture_view|
                    gctx.releaseResource(dst_texture_view);
            }

            const bind_group = gctx.createBindGroup(self.bind_group_layout, &.{
                .{ .binding = 0, .texture_view_handle = src_texture_view },
                .{ .binding = 1, .texture_view_handle = dst_texture_views[0] },
                .{ .binding = 2, .texture_view_handle = dst_texture_views[1] },
                .{ .binding = 3, .texture_view_handle = dst_texture_views[2] },
            });
            defer gctx.releaseResource(bind_group);

            pass.setBindGroup(0, gctx.lookupResource(bind_group).?, &.{});
            const num_groups = [2]u32{ @divExact(texture_info.size.width, std.math.pow(u32, 2, 1 + base_mip_level) * 4), @divExact(texture_info.size.height, std.math.pow(u32, 2, 1 + base_mip_level) * 4) };
            pass.dispatchWorkgroups(num_groups[0], num_groups[1], 1);

            // We want the source to always be at least 4 * 4 (a single workgroup). We'll compute some small (<=8x8) mipmaps twice, but that's ok.
            if (last_mip_level - base_mip_level > 3 and last_mip_level - base_mip_level < 6) {
                base_mip_level = last_mip_level - 3;
            } else base_mip_level += 3;
        }

        pass.end();
        pass.release();

        break :commands encoder.finish(null);
    };
    defer commands.release();

    gctx.submit(&.{commands});
}
