//! Descrambles 1ST_READ file from MIL-CDs.
//! @See https://dreamcast.wiki/Scrambling
//! @See https://mc.pp.se/dc/files/scramble.c

const MaxChunkSize = 2 * 1024 * 1024;

seed: u32,
indices: [MaxChunkSize / 32]u32 = @splat(0),

pub fn init(seed: u32) @This() {
    return .{ .seed = seed & 0xFFFF };
}

fn next(self: *@This()) u32 {
    self.seed = (self.seed *% 2109 +% 9273) & 0x7FFF;
    return (self.seed +% 0xC000) & 0xFFFF;
}

fn load_chunk(self: *@This(), src: []const u8, dst: []u8) void {
    const slice_count = src.len / 32;

    for (0..slice_count) |i| self.indices[i] = @intCast(i);

    var i: i32 = @intCast(slice_count - 1);
    while (i >= 0) {
        const ui: u32 = @intCast(i);
        const x = (self.next() *% ui) >> 16;
        std.mem.swap(u32, &self.indices[ui], &self.indices[x]);
        @memcpy(dst[32 * self.indices[ui] ..][0..32], src[32 * (slice_count - 1 - ui) ..][0..32]);
        i -= 1;
    }
}

pub fn descramble(self: *@This(), src: []const u8, dst: []u8) void {
    var chunk_size: u32 = MaxChunkSize;
    var copied: u32 = 0;
    var remaining_file_size = src.len;
    while (chunk_size >= 32) {
        while (remaining_file_size >= chunk_size) {
            self.load_chunk(src[copied..][0..chunk_size], dst[copied..]);
            remaining_file_size -= chunk_size;
            copied += chunk_size;
        }
        chunk_size >>= 1;
    }
    if (remaining_file_size > 0)
        @memcpy(dst[copied..][0..remaining_file_size], src[copied..][0..remaining_file_size]);
}

const std = @import("std");
