const std = @import("std");
const DreamcastModule = @import("dreamcast");
const Colors = DreamcastModule.HollyModule.Colors;
const Renderer = @import("renderer.zig");

const Version: u32 = 0x03525650;

const MetadataFormat = extern struct {
    four_cc: [4]u8,
    key: u32,
    data_size: u32,
    data: [*]u8,
};

pub const GlobalIndexHeader = extern struct {
    gbix: [4]u8 align(1),
    length: u32 align(1),
    global_index: u64 align(1),
};

const PixelFormat = enum(u8) {
    ARGB1555 = 0x00, //  (bilevel translucent alpha 0,255)
    RGB565 = 0x01, //  (no translucent)
    ARGB4444 = 0x02, //  (translucent alpha 0-255)
    YUV442 = 0x03, //
    BumpMap = 0x04, //
    Palette4BPP = 0x05,
    Palette8BPP = 0x06,
};

const ImageDataType = enum(u8) {
    SQUARE_TWIDDLED = 0x01,
    SQUARE_TWIDDLED_MIPMAP = 0x02,
    VQ = 0x03,
    VQ_MIPMAP = 0x04,
    TWIDDLED_8_BIT_CLUT = 0x05,
    TWIDDLED_4_BIT_CLUT = 0x06,
    TWIDDLED_8_BIT_DIRECT = 0x07,
    TWIDDLED_4_BIT_DIRECT = 0x08,
    RECTANGLE = 0x09,
    RECTANGULAR_STRIDE = 0x0B,
    RECTANGULAR_TWIDDLED = 0x0D,
    SMALL_VQ = 0x10,
    SMALL_VQ_MIPMAP = 0x11,
    SQUARE_TWIDDLED_MIPMAP_2 = 0x12,
    _,

    pub fn twiddled(self: @This()) bool {
        return switch (self) {
            .RECTANGLE, .RECTANGULAR_STRIDE => false,
            else => true,
        };
    }

    pub fn mipmapped(self: @This()) bool {
        return switch (self) {
            .SQUARE_TWIDDLED_MIPMAP, .VQ_MIPMAP, .SMALL_VQ_MIPMAP, .SQUARE_TWIDDLED_MIPMAP_2 => true,
            else => false,
        };
    }

    pub fn vq_compressed(self: @This()) bool {
        return switch (self) {
            .VQ, .VQ_MIPMAP, .SMALL_VQ, .SMALL_VQ_MIPMAP => true,
            else => false,
        };
    }
};

pub const PVRTHeader = extern struct {
    pvrt: [4]u8 align(1),
    length: u32 align(1),
    pixel_format: PixelFormat,
    image_data_type: ImageDataType,
    _: [2]u8,
    width: u16 align(1),
    height: u16 align(1),
};

pub const DecodedPVRImage = struct {
    width: u32,
    height: u32,
    bgra: []u8,

    pub fn init(allocator: std.mem.Allocator, width: u32, height: u32) !DecodedPVRImage {
        return .{
            .width = width,
            .height = height,
            .bgra = try allocator.alloc(u8, 4 * width * height),
        };
    }

    pub fn deinit(self: @This(), allocator: std.mem.Allocator) void {
        allocator.free(self.bgra);
    }
};

// Caller owns the allocated memory.
pub fn decode(allocator: std.mem.Allocator, buffer: []const u8) !DecodedPVRImage {
    if (buffer.len < 8) return error.BufferTooSmall;

    if (std.mem.eql(u8, buffer[0..4], "GBIX")) {
        const header: *const GlobalIndexHeader = @alignCast(@ptrCast(buffer));
        if (8 + header.length >= buffer.len) return error.InvalidGlobalIndexHeader;
        return decode(allocator, buffer[8 + header.length ..]);
    } else if (std.mem.eql(u8, buffer[0..4], "PVRT")) {
        if (buffer.len < @sizeOf(PVRTHeader)) return error.InvalidPVRT;
        const header: *const PVRTHeader = @alignCast(@ptrCast(buffer));

        if (header.width > 1024 or header.height > 1024) return error.InvalidImageSize;

        const image = try DecodedPVRImage.init(allocator, header.width, header.height);
        errdefer image.deinit(allocator);

        var texels_offset: u32 = @sizeOf(PVRTHeader);
        const code_book_offset: u32 = texels_offset;
        if (header.image_data_type.mipmapped()) {
            if (header.image_data_type.vq_compressed()) {
                texels_offset += Renderer.vq_mipmap_offset(image.width);
            } else if (header.pixel_format == .Palette4BPP or header.pixel_format == .Palette8BPP) {
                const val: u32 = Renderer.palette_mipmap_offset(image.width);
                texels_offset += if (header.pixel_format == .Palette4BPP) val / 2 else val;
            } else {
                texels_offset += Renderer.mipmap_offset(image.width);
            }
            texels_offset -= 4; // FIXME: I have no idea why is it off, seems to be fine when textures are uploaded to the PVR.
        }

        if (header.image_data_type.vq_compressed()) {
            Renderer.decode_vq(@alignCast(@ptrCast(image.bgra.ptr)), @enumFromInt(@intFromEnum(header.pixel_format)), buffer[code_book_offset..], buffer[8 * 256 + texels_offset ..], image.width, image.height);
        } else {
            Renderer.decode_tex(@alignCast(@ptrCast(image.bgra.ptr)), @enumFromInt(@intFromEnum(header.pixel_format)), buffer[texels_offset..], image.width, image.height, header.image_data_type.twiddled());
        }

        return image;
    }
    return error.UnknownFormat;
}
