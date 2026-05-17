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

const PNGChunkWriter = struct {
    writer: *std.Io.Writer,
    crc: std.hash.Crc32 = .init(),

    pub fn init(writer: *std.Io.Writer, data_length: u32, chunk_type: []const u8) !@This() {
        try writer.writeInt(u32, data_length, .big);
        var r = @This(){ .writer = writer };
        std.debug.assert(chunk_type.len == 4);
        try r.write(chunk_type);
        return r;
    }

    pub fn writeInt(self: *@This(), val: u32) !void {
        return self.write(std.mem.asBytes(&@byteSwap(val)));
    }

    pub fn write(self: *@This(), data: []const u8) !void {
        try self.writer.writeAll(data);
        self.crc.update(data);
    }

    pub fn finish(self: *@This()) !void {
        try self.writer.writeInt(u32, self.crc.final(), .big);
    }
};

pub fn write_png(self: @This(), allocator: std.mem.Allocator, writer: *std.Io.Writer) !void {
    var alloc_writer = try std.Io.Writer.Allocating.initCapacity(allocator, self.bgra.len);
    defer alloc_writer.deinit();
    var compress_buffer: [std.compress.flate.max_window_len]u8 = undefined;
    var compress = try std.compress.flate.Compress.init(&alloc_writer.writer, &compress_buffer, .zlib, .default);
    for (0..self.height) |y| {
        try compress.writer.writeByte(0); // Filter Type
        for (0..self.width) |x| {
            const bgra = self.bgra[y * self.width * 4 + 4 * x ..][0..4];
            try compress.writer.writeAll(&[_]u8{ bgra[2], bgra[1], bgra[0] });
        }
    }
    try compress.finish();

    const compressed = try alloc_writer.toOwnedSlice();
    defer allocator.free(compressed);

    // PNG Signature
    try writer.writeAll(&[_]u8{ 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A });
    {
        var w = try PNGChunkWriter.init(writer, 13, "IHDR");
        try w.writeInt(self.width);
        try w.writeInt(self.height);
        try w.write(&[_]u8{
            8, // Bit-depth
            2, // Color type (2: Truecolour)
            0, // Compression Method
            0, // Filter Method
            0, // Interlace Method
        });
        try w.finish();
    }
    {
        var w = try PNGChunkWriter.init(writer, @intCast(compressed.len), "IDAT");
        try w.write(compressed);
        try w.finish();
    }
    {
        var w = try PNGChunkWriter.init(writer, 0, "IEND");
        try w.finish();
    }

    try writer.flush();
}
