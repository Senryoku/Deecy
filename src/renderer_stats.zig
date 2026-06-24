const std = @import("std");

const zgpu = @import("zgpu");
const wgpu = zgpu.wgpu;

const Renderer = @import("renderer.zig");

const CircularBuffer = @import("circular_buffer.zig");

pub const FrameTimestamp = enum(u32) {
    Total = 0,
    Background = 2,
    @"Framebuffer Blit" = 4,

    _,

    pub fn begin(self: @This()) u32 {
        return @intFromEnum(self);
    }

    pub fn end(self: @This()) u32 {
        return @intFromEnum(self) + 1;
    }
};
pub const RenderPassTimestamp = enum(u32) {
    pub const MaxPasses = 32;
    /// Maximum number of slices for the translucent pass.
    pub const MaxSlices = 32;
    /// Maximum number of queries per render pass.
    pub const MaxQueries = 32 + 4 * MaxSlices;

    // Unique
    Opaque = 0,
    @"Depth Resolve" = 2,
    @"Modifier Volume Stencil" = 4,
    @"Modifier Volume Apply" = 6,
    @"Presorted Translucent pass" = 8,
    // Per translucent slice
    @"Translucent Modifier Volumes" = 32,
    @"Translucent Merge Modifier Volumes" = 32 + MaxSlices,
    @"Translucent Pass" = 32 + 2 * MaxSlices,
    @"Translucent Blend Pass" = 32 + 3 * MaxSlices,
    _,

    pub fn begin(self: @This()) u32 {
        return @intFromEnum(self);
    }

    pub fn end(self: @This()) u32 {
        return @intFromEnum(self) + 1;
    }

    pub fn @"type"(idx: u32) @This() {
        return switch ((idx >> 1) << 1) {
            0 => .Opaque,
            2 => .@"Depth Resolve",
            4 => .@"Modifier Volume Stencil",
            6 => .@"Modifier Volume Apply",
            8 => .@"Presorted Translucent pass",
            32...63 => .@"Translucent Modifier Volumes",
            64...95 => .@"Translucent Merge Modifier Volumes",
            96...127 => .@"Translucent Pass",
            128...159 => .@"Translucent Blend Pass",
            else => @enumFromInt(idx),
        };
    }

    pub inline fn writes(self: @This(), set: ?wgpu.QuerySet) ?wgpu.PassTimestampWrites {
        return if (set) |qs| .{
            .query_set = qs,
            .beginning_of_pass_write_index = self.begin(),
            .end_of_pass_write_index = self.end(),
        } else null;
    }

    pub fn per_slice(slice: u8) struct {
        modifier_volumes: RenderPassTimestamp,
        merge_modifier_volumes: RenderPassTimestamp,
        fragments: RenderPassTimestamp,
        blend: RenderPassTimestamp,
    } {
        return .{
            .modifier_volumes = @enumFromInt(@intFromEnum(@This().@"Translucent Modifier Volumes") + 2 * slice),
            .merge_modifier_volumes = @enumFromInt(@intFromEnum(@This().@"Translucent Merge Modifier Volumes") + 2 * slice),
            .fragments = @enumFromInt(@intFromEnum(@This().@"Translucent Pass") + 2 * slice),
            .blend = @enumFromInt(@intFromEnum(@This().@"Translucent Blend Pass") + 2 * slice),
        };
    }

    pub const FrameTime = struct {
        total: std.Io.Duration = .zero,
        pass_count: u8 = 0,
        passes: [MaxPasses]struct {
            const Slice = struct {
                modifier_volumes: std.Io.Duration = .zero,
                merge_modifier_volumes: std.Io.Duration = .zero,
                fragments: std.Io.Duration = .zero,
                blend: std.Io.Duration = .zero,

                pub fn total(self: @This()) std.Io.Duration {
                    return .{ .nanoseconds = self.modifier_volumes.nanoseconds + self.merge_modifier_volumes.nanoseconds + self.fragments.nanoseconds + self.blend.nanoseconds };
                }
            };

            slice_count: u8 = 0,
            @"opaque": std.Io.Duration = .zero,
            depth_resolve: std.Io.Duration = .zero,
            modifier_volume_stencil: std.Io.Duration = .zero,
            modifier_volume_apply: std.Io.Duration = .zero,
            presorted_translucent: std.Io.Duration = .zero,
            slices: [MaxSlices]Slice = @splat(.{}),

            pub fn total(self: @This()) std.Io.Duration {
                var sum = std.Io.Duration.fromNanoseconds(self.@"opaque".nanoseconds + self.depth_resolve.nanoseconds + self.modifier_volume_stencil.nanoseconds + self.modifier_volume_apply.nanoseconds);
                for (0..self.slice_count) |i| {
                    sum.nanoseconds += self.slices[i].total().nanoseconds;
                }
                return sum;
            }

            pub fn summed_slices(self: @This()) Slice {
                var sum: Slice = .{};
                for (0..self.slice_count) |i| {
                    sum.modifier_volumes.nanoseconds += self.slices[i].modifier_volumes.nanoseconds;
                    sum.merge_modifier_volumes.nanoseconds += self.slices[i].merge_modifier_volumes.nanoseconds;
                    sum.fragments.nanoseconds += self.slices[i].fragments.nanoseconds;
                    sum.blend.nanoseconds += self.slices[i].blend.nanoseconds;
                }
                return sum;
            }

            pub fn translucent_pass(self: @This()) std.Io.Duration {
                var sum = self.presorted_translucent;
                for (0..self.slice_count) |i| {
                    sum.nanoseconds += self.slices[i].total().nanoseconds;
                }
                return sum;
            }
        } = @splat(.{}),

        pub fn parse(pass_count: u8, slice_count: u8, timestamps: []const u64) FrameTime {
            var r: FrameTime = .{ .pass_count = pass_count };
            r.total = .fromNanoseconds(timestamps[FrameTimestamp.Total.end()] - timestamps[FrameTimestamp.Total.begin()]);
            for (0..pass_count) |i| {
                const range = timestamps[(i + 1) * MaxQueries ..];
                r.passes[i].@"opaque" = .fromNanoseconds(range[RenderPassTimestamp.Opaque.end()] - range[RenderPassTimestamp.Opaque.begin()]);
                r.passes[i].depth_resolve = .fromNanoseconds(range[RenderPassTimestamp.@"Depth Resolve".end()] - range[RenderPassTimestamp.@"Depth Resolve".begin()]);
                r.passes[i].modifier_volume_stencil = .fromNanoseconds(range[RenderPassTimestamp.@"Modifier Volume Stencil".end()] - range[RenderPassTimestamp.@"Modifier Volume Stencil".begin()]);
                r.passes[i].modifier_volume_apply = .fromNanoseconds(range[RenderPassTimestamp.@"Modifier Volume Apply".end()] - range[RenderPassTimestamp.@"Modifier Volume Apply".begin()]);
                r.passes[i].presorted_translucent = .fromNanoseconds(range[RenderPassTimestamp.@"Presorted Translucent pass".end()] - range[RenderPassTimestamp.@"Presorted Translucent pass".begin()]);
                r.passes[i].slice_count = slice_count;
                for (0..slice_count) |j| {
                    const ps = RenderPassTimestamp.per_slice(@intCast(j));
                    r.passes[i].slices[j].modifier_volumes = .fromNanoseconds(range[ps.modifier_volumes.end()] - range[ps.modifier_volumes.begin()]);
                    r.passes[i].slices[j].merge_modifier_volumes = .fromNanoseconds(range[ps.merge_modifier_volumes.end()] - range[ps.merge_modifier_volumes.begin()]);
                    r.passes[i].slices[j].fragments = .fromNanoseconds(range[ps.fragments.end()] - range[ps.fragments.begin()]);
                    r.passes[i].slices[j].blend = .fromNanoseconds(range[ps.blend.end()] - range[ps.blend.begin()]);
                }
            }
            return r;
        }

        pub fn format(self: @This(), writer: *std.Io.Writer) !void {
            try writer.print("Frame Time: {f}\n", .{self.total});
            for (0..self.pass_count) |i| {
                try writer.print("[{d: >2}] Pass: {f}\n", .{ i, self.passes[i].total() });
                try writer.print("[{d: >2}]   Opaque: {f}\n", .{ i, self.passes[i].@"opaque" });
                try writer.print("[{d: >2}]   Depth resolve: {f}\n", .{ i, self.passes[i].depth_resolve });
                try writer.print("[{d: >2}]   Modifier Volume Stencil: {f}\n", .{ i, self.passes[i].modifier_volume_stencil });
                try writer.print("[{d: >2}]   Modifier Volume Apply: {f}\n", .{ i, self.passes[i].modifier_volume_apply });

                try writer.print("[{d: >2}]   Total Translucent: {f}\n", .{ i, self.passes[i].translucent_pass() });
                try writer.print("[{d: >2}]     Pre-Sort: {f}\n", .{ i, self.passes[i].presorted_translucent });
                const summed_slices = self.passes[i].summed_slices();
                try writer.print("[{d: >2}]     Summed Modifier Volumes: {f}\n", .{ i, summed_slices.modifier_volumes });
                try writer.print("[{d: >2}]     Summed Merge Modifier Volumes: {f}\n", .{ i, summed_slices.merge_modifier_volumes });
                try writer.print("[{d: >2}]     Summed Fragments: {f}\n", .{ i, summed_slices.fragments });
                try writer.print("[{d: >2}]     Summed Blend: {f}\n", .{ i, summed_slices.blend });
                for (0..self.passes[i].slice_count) |j| {
                    const slice = self.passes[i].slices[j];
                    try writer.print("[{d: >2}]     [{d: >3}] Modifier Volumes: {f}\n", .{ i, j, slice.modifier_volumes });
                    try writer.print("[{d: >2}]     [{d: >3}] Merge Modifier Volumes: {f}\n", .{ i, j, slice.merge_modifier_volumes });
                    try writer.print("[{d: >2}]     [{d: >3}] Fragments: {f}\n", .{ i, j, slice.fragments });
                    try writer.print("[{d: >2}]     [{d: >3}] Blend: {f}\n", .{ i, j, slice.blend });
                }
            }
        }
    };
};

pub const History = struct {
    last_frame: RenderPassTimestamp.FrameTime = .{},

    total: CircularBuffer.CircularBuffer(f32),
    passes: [4]struct {
        @"opaque": CircularBuffer.CircularBuffer(f32),
        depth_resolve: CircularBuffer.CircularBuffer(f32),
        modifier_volume_stencil: CircularBuffer.CircularBuffer(f32),
        modifier_volume_apply: CircularBuffer.CircularBuffer(f32),
        presorted_translucent: CircularBuffer.CircularBuffer(f32),
        // Summed slices
        modifier_volumes: CircularBuffer.CircularBuffer(f32),
        merge_modifier_volumes: CircularBuffer.CircularBuffer(f32),
        fragments: CircularBuffer.CircularBuffer(f32),
        blend: CircularBuffer.CircularBuffer(f32),
    } = undefined,

    pub fn init() !@This() {
        var r: @This() = .{
            .total = try .initAtLeast(512),
        };
        for (&r.passes) |*p| {
            p.* = .{
                .@"opaque" = try .initAtLeast(512),
                .depth_resolve = try .initAtLeast(512),
                .modifier_volume_stencil = try .initAtLeast(512),
                .modifier_volume_apply = try .initAtLeast(512),
                .presorted_translucent = try .initAtLeast(512),
                .modifier_volumes = try .initAtLeast(512),
                .merge_modifier_volumes = try .initAtLeast(512),
                .fragments = try .initAtLeast(512),
                .blend = try .initAtLeast(512),
            };
        }
        return r;
    }

    pub fn deinit(self: *@This()) void {
        self.total.deinit();
        for (&self.passes) |*p| {
            p.@"opaque".deinit();
            p.depth_resolve.deinit();
            p.modifier_volume_stencil.deinit();
            p.modifier_volume_apply.deinit();
            p.presorted_translucent.deinit();
            p.modifier_volumes.deinit();
            p.merge_modifier_volumes.deinit();
            p.fragments.deinit();
            p.blend.deinit();
        }
    }

    fn delta(query: FrameTimestamp, timestamps: []const u64) f32 {
        return @as(f32, @floatFromInt(timestamps[query.end()] - timestamps[query.begin()])) / 1_000_000;
    }
    fn deltaRF(query: RenderPassTimestamp, timestamps: []const u64) f32 {
        return @as(f32, @floatFromInt(timestamps[query.end()] - timestamps[query.begin()])) / 1_000_000;
    }

    pub fn add(self: *@This(), passes: []const Renderer.RenderPass, slice_count: u8, timestamps: []const u64) void {
        self.last_frame = .parse(@intCast(passes.len), slice_count, timestamps);

        self.total.push(delta(.Total, timestamps));

        const pass_count = @min(self.passes.len, passes.len);
        for (self.passes[0..pass_count], passes[0..pass_count], 0..) |*p, render_pass, i| {
            const range = timestamps[(i + 1) * RenderPassTimestamp.MaxQueries ..];

            var @"opaque": f32 = 0;
            var depth_resolve: f32 = 0;
            var modifier_volume_stencil: f32 = 0;
            var modifier_volume_apply: f32 = 0;
            var presorted_translucent: f32 = 0;
            var modifier_volumes: f32 = 0;
            var merge_modifier_volumes: f32 = 0;
            var fragments: f32 = 0;
            var blend: f32 = 0;

            if (render_pass.opaque_list_pointer.empty or !render_pass.punchthrough_list_pointer.empty)
                @"opaque" = deltaRF(.Opaque, range);
            depth_resolve = deltaRF(.@"Depth Resolve", range);
            if (!render_pass.opaque_modifier_volume_pointer.empty) {
                modifier_volume_stencil = deltaRF(.@"Modifier Volume Stencil", range);
                modifier_volume_apply = deltaRF(.@"Modifier Volume Apply", range);
            }
            if (!render_pass.translucent_list_pointer.empty) {
                if (render_pass.pre_sort) {
                    presorted_translucent = deltaRF(.@"Presorted Translucent pass", range);
                } else {
                    for (0..slice_count) |j| {
                        const ps = RenderPassTimestamp.per_slice(@intCast(j));
                        if (!render_pass.translucent_modifier_volume_pointer.empty) {
                            modifier_volumes += deltaRF(ps.modifier_volumes, range);
                            merge_modifier_volumes += deltaRF(ps.merge_modifier_volumes, range);
                        }
                        fragments += deltaRF(ps.fragments, range);
                        blend += deltaRF(ps.blend, range);
                    }
                }
            }

            p.@"opaque".push(@"opaque");
            p.depth_resolve.push(depth_resolve);
            p.modifier_volume_stencil.push(modifier_volume_stencil);
            p.modifier_volume_apply.push(modifier_volume_apply);
            p.presorted_translucent.push(presorted_translucent);
            p.modifier_volumes.push(modifier_volumes);
            p.merge_modifier_volumes.push(merge_modifier_volumes);
            p.fragments.push(fragments);
            p.blend.push(blend);
        }
    }
};
