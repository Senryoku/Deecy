const std = @import("std");

width: u32,
height: u32,
bgra: []u8,

pub fn init(allocator: std.mem.Allocator, width: u32, height: u32) !@This() {
    return .{
        .width = width,
        .height = height,
        .bgra = try allocator.alloc(u8, 4 * width * height),
    };
}

pub fn deinit(self: @This(), allocator: std.mem.Allocator) void {
    allocator.free(self.bgra);
}
