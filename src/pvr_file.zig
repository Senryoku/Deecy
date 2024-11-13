const std = @import("std");
const Colors = @import("colors.zig");
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
};

pub fn decode(allocator: std.mem.Allocator, buffer: []const u8) !DecodedPVRImage {
    if (buffer.len < 8) return error.BufferTooSmall;

    if (std.mem.eql(u8, buffer[0..4], "GBIX")) {
        const header: *const GlobalIndexHeader = @alignCast(@ptrCast(buffer));
        if (8 + header.length >= buffer.len) return error.InvalidGlobalIndexHeader;
        return decode(allocator, buffer[8 + header.length ..]);
    } else if (std.mem.eql(u8, buffer[0..4], "PVRT")) {
        if (buffer.len < @sizeOf(PVRTHeader)) return error.InvalidPVRT;
        const header: *const PVRTHeader = @alignCast(@ptrCast(buffer));

        const twiddled = switch (header.image_data_type) {
            .SQUARE_TWIDDLED,
            .SQUARE_TWIDDLED_MIPMAP,
            .SQUARE_TWIDDLED_MIPMAP_2,
            .TWIDDLED_4_BIT_CLUT,
            .TWIDDLED_4_BIT_DIRECT,
            .TWIDDLED_8_BIT_CLUT,
            .TWIDDLED_8_BIT_DIRECT,
            .RECTANGULAR_TWIDDLED,
            .VQ,
            .VQ_MIPMAP,
            .SMALL_VQ,
            .SMALL_VQ_MIPMAP,
            => true,
            else => false,
        };
        const mipmap = switch (header.image_data_type) {
            .SQUARE_TWIDDLED_MIPMAP,
            .VQ_MIPMAP,
            .SMALL_VQ_MIPMAP,
            .SQUARE_TWIDDLED_MIPMAP_2,
            => true,
            else => false,
        };
        const vq_compressed = switch (header.image_data_type) {
            .VQ,
            .VQ_MIPMAP,
            .SMALL_VQ,
            .SMALL_VQ_MIPMAP,
            => true,
            else => false,
        };

        if (header.width > 1024 or header.height > 1024) return error.InvalidImageSize;

        const image = DecodedPVRImage{
            .width = header.width,
            .height = header.height,
            .bgra = try allocator.alloc(u8, @as(u32, 4) * header.width * header.height),
        };
        errdefer allocator.free(image.bgra);

        var offset: u32 = @sizeOf(PVRTHeader);
        var code_book_offset: u32 = offset;
        if (mipmap) {
            if (vq_compressed) {
                offset += Renderer.vq_mipmap_offset(image.width);
                code_book_offset -= 4; // FIXME: Not sure. VQ compressed textures are buggy with or without it atm.
            } else {
                offset += Renderer.mipmap_offset(image.width);
            }
            offset -= 4; // FIXME: I have no idea why is it off, seems to be fine when textures are uploaded to the PVR.
        }

        if (vq_compressed) {
            if (buffer.len < offset + 8 * 256 + image.width * image.height / 4) return error.TruncatedFile;
            const code_book = std.mem.bytesAsSlice(u64, buffer);
            const indices = buffer[8 * 256 + offset ..];
            Renderer.decode_vq(@alignCast(@ptrCast(image.bgra.ptr)), @enumFromInt(@intFromEnum(header.pixel_format)), @alignCast(code_book), indices, image.width, image.height);
        } else {
            if (buffer.len < offset + 2 * image.width * image.height) return error.TruncatedFile;
            const texels = std.mem.bytesAsSlice(Colors.Color16, buffer[offset..]);
            for (0..image.height) |y| {
                for (0..image.width) |x| {
                    const pixel_index: usize = y * image.width + x;
                    const texel_index: usize = if (twiddled) Renderer.untwiddle(@intCast(x), @intCast(y), image.width, image.height) else pixel_index;
                    @memcpy(
                        image.bgra[pixel_index * 4 + 0 .. pixel_index * 4 + 4],
                        texels[texel_index].bgra(@enumFromInt(@intFromEnum(header.pixel_format)), twiddled)[0..4],
                    );
                }
            }
        }

        return image;
    }
    return error.UnknownFormat;
}
