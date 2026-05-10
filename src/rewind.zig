snapshots: std.ArrayList(Snapshot) = .empty,

// UI State
current_frame: TextureAndView = .{},
/// selected_snapshot >= snapshots.len means the current state (no rewind)
selected_snapshot: i32 = 0,

const Snapshot = struct {
    preview: TextureAndView = .{},
    data: []u8 = &.{},
    cycle: u64 = 0,

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
        self.preview.release(gctx);
        allocator.free(self.data);
    }
};

pub fn init(self: *@This()) void {
    _ = self;
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
    for (self.snapshots.items) |*snapshot| snapshot.deinit(allocator, gctx);
    self.snapshots.deinit(allocator);
}

pub fn clear(self: *@This(), allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
    for (self.snapshots.items) |*snapshot| snapshot.deinit(allocator, gctx);
    self.snapshots.clearRetainingCapacity();
}

pub fn discard_after(self: *@This(), allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext, snapshot_idx: i32) !void {
    for (@intCast(snapshot_idx + 1)..self.snapshots.items.len) |idx| {
        self.snapshots.items[idx].deinit(allocator, gctx);
    }
    try self.snapshots.resize(allocator, @intCast(snapshot_idx + 1));
}

/// Should be called every time the UI re-appears
pub fn ui_init(self: *@This(), gctx: *zgpu.GraphicsContext, current_frame: zgpu.TextureHandle, resolution: Resolution) void {
    self.selected_snapshot = @intCast(@max(1, self.snapshots.items.len) - 1);
    if (self.current_frame.texture.id == 0)
        self.current_frame = create_preview_texture(gctx, resolution);
    copy_texture(gctx, current_frame, self.current_frame.texture, resolution);
}

pub fn on_resolution_change(self: *@This(), gctx: *zgpu.GraphicsContext) !void {
    // FIXME: Use this!
    self.current_frame.release(gctx);
    self.current_frame = .{};
    // NOTE: We can't easily copy previews to a different resolution, taking a shortcut here and just release them. Nil previews will be ignored.
    for (self.snapshots.items) |*snapshot| {
        snapshot.preview.release(gctx);
        snapshot.preview = .{};
    }
}

pub fn create_preview_texture(gctx: *zgpu.GraphicsContext, resolution: Resolution) TextureAndView {
    return .init(gctx, gctx.createTexture(
        .{
            .usage = .{
                .texture_binding = true,
                .copy_src = true,
                .copy_dst = true,
            },
            .size = .{
                .width = resolution.width,
                .height = resolution.height,
                .depth_or_array_layers = 1,
            },
            .format = .bgra8_unorm,
            .mip_level_count = 1,
            .label = .init("Rewind Preview"),
        },
    ));
}

pub fn copy_texture(gctx: *zgpu.GraphicsContext, src: zgpu.TextureHandle, dst: zgpu.TextureHandle, resolution: Resolution) void {
    const src_tex = gctx.lookupResource(src);
    const dst_tex = gctx.lookupResource(dst);
    if (src_tex == null or dst_tex == null) return;
    const commands = commands: {
        const encoder = gctx.device.createCommandEncoder(null);
        defer encoder.release();
        encoder.copyTextureToTexture(
            .{ .texture = src_tex.? },
            .{ .texture = dst_tex.? },
            .{ .width = resolution.width, .height = resolution.height },
        );
        break :commands encoder.finish(null);
    };
    defer commands.release();
    gctx.submit(&.{commands});
}

const std = @import("std");
const log = std.log.scoped(.rewind);
const zgui = @import("zgui");
const zgpu = @import("zgpu");
const Deecy = @import("deecy.zig");
const TextureAndView = Deecy.RendererModule.TextureAndView;
const Resolution = Deecy.RendererModule.Renderer.Resolution;
