snapshots: std.ArrayList(*Snapshot) = .empty,

texture_pool_mutex: std.Io.Mutex = .init,
texture_pool: std.ArrayList(TextureAndView) = .empty,

// UI State
current_frame: TextureAndView = .{},
/// selected_snapshot >= snapshots.len means the current state (no rewind)
selected_snapshot: i32 = 0,
/// Quick, dirty and framerate dependent animation.
indicator_visual_position: f32 = 1.0,

pub const Snapshot = struct {
    pub const Compression = enum {
        None,
        lz4,
    };

    preview: TextureAndView = .{},
    data: []const u8 = &.{},
    cycle: u64 = 0,
    /// Waits on this future before accessing data.
    compression: std.Io.Future(Compression) = .{ .any_future = null, .result = .None },

    pub fn deinit(self: *@This(), io: std.Io, allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
        _ = self.compression.cancel(io);
        self.preview.release(gctx);
        allocator.free(self.data);
    }
};

pub fn init(self: *@This()) void {
    _ = self;
}

pub fn deinit(self: *@This(), io: std.Io, allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
    for (self.snapshots.items) |snapshot| {
        snapshot.deinit(io, allocator, gctx);
        allocator.destroy(snapshot);
    }
    self.snapshots.deinit(allocator);
    for (self.texture_pool.items) |texture| texture.release(gctx);
    self.texture_pool.deinit(allocator);
}

pub fn clear(self: *@This(), io: std.Io, allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
    for (self.snapshots.items) |snapshot| {
        snapshot.deinit(io, allocator, gctx);
        allocator.destroy(snapshot);
    }
    self.snapshots.clearRetainingCapacity();
    for (self.texture_pool.items) |texture| texture.release(gctx);
    self.texture_pool.clearRetainingCapacity();
}

/// Thread-safe.
pub fn get_preview_texture(self: *@This(), io: std.Io, allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext, resolution: Resolution) !TextureAndView {
    try self.texture_pool_mutex.lock(io);
    defer self.texture_pool_mutex.unlock(io);
    if (self.texture_pool.items.len == 0)
        try self.texture_pool.append(allocator, create_preview_texture(gctx, resolution));
    return self.texture_pool.pop().?;
}

/// Thread-safe.
pub fn release_preview_texture(self: *@This(), io: std.Io, allocator: std.mem.Allocator, texture: TextureAndView) !void {
    if (texture.texture.id == 0) return;
    try self.texture_pool_mutex.lock(io);
    defer self.texture_pool_mutex.unlock(io);
    try self.texture_pool.append(allocator, texture);
}

/// Destroy supplied snapshot, returning its preview texture to the pool.
pub fn reclaim(self: *@This(), io: std.Io, allocator: std.mem.Allocator, snapshot: *Snapshot) !void {
    _ = snapshot.compression.cancel(io);
    allocator.free(snapshot.data);
    try self.release_preview_texture(io, allocator, snapshot.preview);
    allocator.destroy(snapshot);
}

pub fn discard_after(self: *@This(), io: std.Io, allocator: std.mem.Allocator, snapshot_idx: i32) !void {
    for (self.snapshots.items[@intCast(snapshot_idx + 1)..self.snapshots.items.len]) |snapshot| {
        try reclaim(self, io, allocator, snapshot);
    }
    try self.snapshots.resize(allocator, @intCast(snapshot_idx + 1));
}

/// Should be called every time the UI re-appears.
/// Submit commands to the gctx queue.
pub fn ui_init(self: *@This(), gctx: *zgpu.GraphicsContext, current_frame: zgpu.TextureHandle, resolution: Resolution) void {
    self.selected_snapshot = @intCast(self.snapshots.items.len);
    if (self.current_frame.texture.id == 0)
        self.current_frame = create_preview_texture(gctx, resolution);
    copy_texture(gctx, current_frame, self.current_frame.texture, resolution);
}

pub fn update_preview(self: *@This(), gctx: *zgpu.GraphicsContext, current_frame: zgpu.TextureHandle, resolution: Resolution) void {
    const texture = if (self.selected_snapshot < self.snapshots.items.len)
        self.snapshots.items[@intCast(self.selected_snapshot)].preview.texture
    else
        self.current_frame.texture;
    if (texture.id != 0) {
        // Fullscreen preview. Re-uses the renderer framebuffer for simplicity.
        copy_texture(gctx, texture, current_frame, resolution);
    }
}

/// Previews are assumed to be the same resolution as the renderer output for easy copying.
/// This releases all previous previews.
pub fn on_inner_resolution_change(self: *@This(), gctx: *zgpu.GraphicsContext) void {
    self.current_frame.release(gctx);
    self.current_frame = .{};
    for (self.snapshots.items) |snapshot| {
        snapshot.preview.release(gctx);
        snapshot.preview = .{};
    }
    for (self.texture_pool.items) |texture| texture.release(gctx);
    self.texture_pool.clearRetainingCapacity();
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

pub fn compress(allocator: std.mem.Allocator, snapshot: *Snapshot) Snapshot.Compression {
    compress_impl(allocator, snapshot) catch |err| {
        log.err("Failed to compress snapshot: {}", .{err});
        return .None;
    };
    return .lz4;
}

fn compress_impl(allocator: std.mem.Allocator, snapshot: *Snapshot) !void {
    const compressed = try lz4.Standard.compress(allocator, snapshot.data);
    errdefer allocator.free(compressed);
    allocator.free(snapshot.data);
    snapshot.data = compressed;
}

const std = @import("std");
const log = std.log.scoped(.rewind);
const lz4 = @import("lz4");
const zgui = @import("zgui");
const zgpu = @import("zgpu");
const Deecy = @import("deecy.zig");
const TextureAndView = Deecy.RendererModule.TextureAndView;
const Resolution = Deecy.RendererModule.Renderer.Resolution;
