snapshots_array_mutex: std.Io.Mutex = .init,
snapshots: std.ArrayList(*Snapshot) = .empty,

pools_mutex: std.Io.Mutex = .init,
memory_pool: std.ArrayList([]u8) = .empty,
texture_pool: std.ArrayList(TextureAndView) = .empty,

// UI State
current_frame: TextureAndView = .{},
/// selected_snapshot >= snapshots.len means the current state (no rewind)
selected_snapshot: i32 = 0,
/// Quick, dirty and framerate dependent animation.
indicator_visual_position: f32 = 1.0,

pub const Snapshot = struct {
    preview: TextureAndView = .{},
    /// View into backing memory
    data: []const u8 = &.{},
    cycle: u64 = 0,

    /// Actual allocated memory might be larger than the snapshot size, keep track of it.
    backing_memory: []u8 = &.{},

    compressed: std.Io.Future(bool) = .{ .any_future = null, .result = false },

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
        self.preview.release(gctx);
        allocator.free(self.backing_memory);
    }
};

pub fn init(self: *@This()) void {
    _ = self;
}

pub fn deinit(self: *@This(), io: std.Io, allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
    for (self.snapshots.items) |snapshot| {
        _ = snapshot.compressed.await(io);
        snapshot.deinit(allocator, gctx);
        allocator.destroy(snapshot);
    }
    self.snapshots.deinit(allocator);
    for (self.memory_pool.items) |memory| allocator.free(memory);
    self.memory_pool.deinit(allocator);
    for (self.texture_pool.items) |texture| texture.release(gctx);
    self.texture_pool.deinit(allocator);
}

pub fn clear(self: *@This(), io: std.Io, allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
    for (self.snapshots.items) |snapshot| {
        _ = snapshot.compressed.await(io);
        snapshot.deinit(allocator, gctx);
        allocator.destroy(snapshot);
    }
    self.snapshots.clearRetainingCapacity();
    for (self.memory_pool.items) |memory| allocator.free(memory);
    self.memory_pool.clearRetainingCapacity();
    for (self.texture_pool.items) |texture| texture.release(gctx);
    self.texture_pool.clearRetainingCapacity();
}

pub fn get_memory(self: *@This(), io: std.Io, allocator: std.mem.Allocator) ![]u8 {
    try self.pools_mutex.lock(io);
    defer self.pools_mutex.unlock(io);
    if (self.memory_pool.items.len == 0) {
        try self.memory_pool.append(allocator, try allocator.alloc(u8, 32 * 1024 * 1024));
    }
    return self.memory_pool.pop().?;
}

pub fn release_memory(self: *@This(), io: std.Io, allocator: std.mem.Allocator, memory: []u8) !void {
    try self.pools_mutex.lock(io);
    defer self.pools_mutex.unlock(io);
    if (self.memory_pool.items.len >= 3) {
        allocator.free(memory);
    } else {
        try self.memory_pool.append(allocator, memory);
    }
}

pub fn get_preview_texture(self: *@This(), io: std.Io, allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext, resolution: Resolution) !TextureAndView {
    try self.pools_mutex.lock(io);
    defer self.pools_mutex.unlock(io);
    if (self.texture_pool.items.len == 0)
        try self.texture_pool.append(allocator, create_preview_texture(gctx, resolution));
    return self.texture_pool.pop().?;
}

pub fn release_preview_texture(self: *@This(), io: std.Io, allocator: std.mem.Allocator, texture: TextureAndView) !void {
    try self.pools_mutex.lock(io);
    defer self.pools_mutex.unlock(io);
    try self.texture_pool.append(allocator, texture);
}

pub fn discard_after(self: *@This(), io: std.Io, allocator: std.mem.Allocator, snapshot_idx: i32) !void {
    for (self.snapshots.items[@intCast(snapshot_idx + 1)..self.snapshots.items.len]) |snapshot| {
        if (snapshot.compressed.await(io)) {
            allocator.free(snapshot.backing_memory);
        } else {
            try self.release_memory(io, allocator, snapshot.backing_memory);
        }
        try self.release_preview_texture(io, allocator, snapshot.preview);
        allocator.destroy(snapshot);
    }
    try self.snapshots.resize(allocator, @intCast(snapshot_idx + 1));
}

/// Should be called every time the UI re-appears
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

pub fn compress(self: *@This(), io: std.Io, allocator: std.mem.Allocator, snapshot: *Snapshot) bool {
    self.compress_impl(io, allocator, snapshot) catch |err| {
        log.err("Failed to compress snapshot: {}", .{err});
        return false;
    };
    return true;
}

fn compress_impl(self: *@This(), io: std.Io, allocator: std.mem.Allocator, snapshot: *Snapshot) !void {
    const compressed = try lz4.Standard.compress(allocator, snapshot.data);
    errdefer allocator.free(compressed);
    try self.release_memory(io, allocator, snapshot.backing_memory);
    try self.snapshots_array_mutex.lock(io);
    defer self.snapshots_array_mutex.unlock(io);
    snapshot.data = compressed;
    snapshot.backing_memory = @constCast(compressed);
}

const std = @import("std");
const log = std.log.scoped(.rewind);
const lz4 = @import("lz4");
const zgui = @import("zgui");
const zgpu = @import("zgpu");
const Deecy = @import("deecy.zig");
const TextureAndView = Deecy.RendererModule.TextureAndView;
const Resolution = Deecy.RendererModule.Renderer.Resolution;
