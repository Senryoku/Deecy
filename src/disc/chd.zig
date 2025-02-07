const std = @import("std");
const termcolor = @import("termcolor");

const MemoryMappedFile = @import("../memory_mapped_file.zig");
const Track = @import("track.zig");
const Session = @import("session.zig");

const log = std.log.scoped(.chd);

tracks: std.ArrayList(Track),
sessions: std.ArrayList(Session),
_file: MemoryMappedFile,

// NOTE: All number are big endian.

const Tag = "MComprHD";

const Compression = enum(u32) {
    none = 0,
    zlib = tag("zlib"),
    zstd = tag("zstd"),
    lzma = tag("lzma"),
    huffman = tag("huff"),
    flag = tag("flac"),

    cd_zlib = tag("cdzl"),
    cd_zstd = tag("cdzs"),
    cd_lzma = tag("cdlz"),
    cd_flac = tag("cdfl"),

    _,

    fn tag(comptime t: *const [4:0]u8) u32 {
        return @as(u32, t[0]) << 24 | @as(u32, t[1]) << 16 | @as(u32, t[2]) << 8 | @as(u32, t[3]) << 0;
    }
};

pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !@This() {
    var self: @This() = .{
        .tracks = std.ArrayList(Track).init(allocator),
        .sessions = std.ArrayList(Session).init(allocator),
        ._file = try MemoryMappedFile.init(filepath, allocator),
    };
    errdefer self.deinit();

    const file = try std.fs.cwd().openFile(filepath, .{});
    defer file.close();

    const reader = file.reader();
    const tag = try reader.readBytesNoEof(8);
    if (!std.mem.eql(u8, &tag, Tag)) return error.InvalidCHD;
    const header_length = try reader.readInt(u32, .big);
    const version = try reader.readInt(u32, .big);

    switch (version) {
        5 => {
            if (header_length != 124) return error.InvalidCHDv5;
            const compressors = [4]Compression{
                @enumFromInt(try reader.readInt(u32, .big)),
                @enumFromInt(try reader.readInt(u32, .big)),
                @enumFromInt(try reader.readInt(u32, .big)),
                @enumFromInt(try reader.readInt(u32, .big)),
            };
            const logical_bytes = try reader.readInt(u64, .big);
            const map_offset = try reader.readInt(u64, .big);
            const meta_offset = try reader.readInt(u64, .big);
            const hunk_bytes = try reader.readInt(u32, .big);
            const unit_bytes = try reader.readInt(u32, .big);
            const raw_sha1 = try reader.readBytesNoEof(20);
            const sha1 = try reader.readBytesNoEof(20);
            const parent_sha1 = try reader.readBytesNoEof(20);

            log.debug("  Compressor: {any}", .{compressors});
            log.debug("  Logical Bytes: {X}", .{logical_bytes});
            log.debug("  Map Offset: {X}", .{map_offset});
            log.debug("  Meta Offset: {X}", .{meta_offset});
            log.debug("  Hunk Bytes: {X}", .{hunk_bytes});
            log.debug("  Unit Bytes: {X}", .{unit_bytes});
            log.debug("  Raw SHA1: {X}", .{raw_sha1});
            log.debug("  SHA1: {X}", .{sha1});
            log.debug("  Parent SHA1: {X}", .{parent_sha1});

            try file.seekTo(map_offset);

            const map_bytes = try reader.readInt(u32, .big);
            const first_offset = try reader.readInt(u48, .big);
            const map_crc = try reader.readInt(u16, .big);
            const length_bits = try reader.readByte();
            const self_bits = try reader.readByte();
            const parent_bits = try reader.readByte();

            log.debug("  Map Bytes: {X}", .{map_bytes});
            log.debug("  First Offset: {X}", .{first_offset});
            log.debug("  Map CRC: {X}", .{map_crc});
            log.debug("  Length Bits: {X}", .{length_bits});
            log.debug("  Self Bits: {X}", .{self_bits});
            log.debug("  Parent Bits: {X}", .{parent_bits});

            // Map is compressed using huffman running length encoding.
            // I don't really want to reimplement all of that...

            return error.TODO;
        },
        else => {
            log.err(termcolor.red("CHD version {d} not supported."), .{version});
            return error.CHDVersionNotSupported;
        },
    }

    return self;
}

fn track_header(reader: anytype) !void {
    const null_or_extra = try reader.readInt(u32, .little);
    if (null_or_extra != 0)
        try reader.skipBytes(8, .{});
    const track_start_mark = try reader.readInt(u160, .big);
    if (track_start_mark != 0x0000_0100_0000_FFFF_FFFF_0000_0100_0000_FFFF_FFFF) {
        log.err("  Invalid Track Start Mark: {X}", .{track_start_mark});
        return error.InvalidCDI;
    }
    try reader.skipBytes(4, .{});

    const filename_length = try reader.readInt(u8, .little);
    var buffer: [256]u8 = undefined;
    const filename = buffer[0..filename_length];
    _ = try reader.read(filename);
    log.debug("    Filename: {s}", .{filename});
    try reader.skipBytes(1 + 10 + 4 + 4, .{});

    const v4_mark = try reader.readInt(u32, .little);
    const max_cd_length = mcl: {
        if (v4_mark != 0x80000000) {
            break :mcl v4_mark;
        } else {
            const max_cd_length = try reader.readInt(u32, .little);
            const v4_mark_2 = try reader.readInt(u32, .little);
            if (v4_mark_2 != 0x980000) {
                log.debug("  V4 Mark 2: {X}", .{v4_mark_2});
                return error.InvalidCDI;
            }
            break :mcl max_cd_length;
        }
    };
    if (max_cd_length != 0x514C8 and max_cd_length != 0x57E40) {
        log.err("  Invalid max_cd_length: {X}", .{max_cd_length});
        return error.InvalidCDI;
    }
    log.debug("    Max CD Length: {X}", .{max_cd_length});
}

pub fn deinit(self: *@This()) void {
    self.sessions.deinit();
    self.tracks.deinit();
    self._file.deinit();
}

pub fn get_session_count(self: *const @This()) u32 {
    return @intCast(self.sessions.items.len);
}

pub fn get_session(self: *const @This(), session_number: u32) Session {
    return self.sessions.items[session_number - 1];
}

pub fn get_area_boundaries(self: *const @This(), area: Session.Area) [2]u32 {
    std.debug.assert(area == .SingleDensity);
    return .{ 0, @intCast(self.tracks.items.len - 1) };
}
