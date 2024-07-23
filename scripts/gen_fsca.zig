const std = @import("std");

// Generates the fsca LUT

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    var table = try allocator.alloc(f32, 2 * 0x10000);

    for (0..0x10000) |i| {
        const fraction = @as(u32, @intCast(i)) & 0x0000_FFFF;
        const angle = 2 * std.math.pi * @as(f32, @floatFromInt(fraction)) / 0x10000;

        table[2 * i] = @sin(angle);
        table[2 * i + 1] = @cos(angle);
    }

    const file = try std.fs.cwd().createFile("fsca.bin", .{});
    defer file.close();
    try file.writeAll(std.mem.sliceAsBytes(table));
}
