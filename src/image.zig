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

pub fn write_bmp(self: @This(), writer: *std.Io.Writer) !void {
    try writer.writeAll("BM");
    try writer.writeInt(u32, 54 + 4 * self.width * self.height, .little); // File size
    try writer.writeInt(u16, 0, .little);
    try writer.writeInt(u16, 0, .little);
    try writer.writeInt(u32, 54, .little); // Pixel data offset

    try writer.writeInt(u32, 40, .little); // Header Size
    try writer.writeInt(u32, self.width, .little);
    try writer.writeInt(i32, -@as(i32, @intCast(self.height)), .little); // Height, negative for top to bottom
    try writer.writeInt(u16, 1, .little); // Planes
    try writer.writeInt(u16, 32, .little); // Bits per pixels
    try writer.writeInt(u32, 0, .little); // Compression: BI_RGB
    try writer.writeInt(u32, 0, .little); // Pixel data size, can be 0 for BI_RGB
    try writer.writeInt(u32, 2835, .little); // Horizontal resolution of the image. (pixel per metre, signed integer)
    try writer.writeInt(u32, 2835, .little); // Vertical resolution of the image. (pixel per metre, signed integer)
    try writer.writeInt(u32, 0, .little); // Number of colors in the color palette
    try writer.writeInt(u32, 0, .little); // Number of important colors

    try writer.writeAll(self.bgra);

    try writer.flush();
}
