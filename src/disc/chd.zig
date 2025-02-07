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
fn crc16(data: []const u8) u16 {
    var crc: u16 = 0xffff;

    const s_table: [256]u16 = .{
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
    };

    for (data) |d|
        crc = (crc << 8) ^ s_table[(crc >> 8) ^ d];
    return crc;
}

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

            const hunk_count = (logical_bytes + hunk_bytes - 1) / hunk_bytes;

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

            try file.seekTo(map_offset + 16);

            const NumCodes = 16;
            const MaxBits = 8;

            var nodes: [NumCodes]struct { bits: u32, numbits: u4 } = undefined;

            var bit_reader = std.io.bitReader(.big, reader);
            var node_idx: u32 = 0;
            while (node_idx < NumCodes) {
                var nodebits: u4 = try bit_reader.readBitsNoEof(u4, 4);
                if (nodebits != 1) {
                    nodes[node_idx].numbits = nodebits;
                    node_idx += 1;
                } else {
                    // a one value is an escape code
                    nodebits = try bit_reader.readBitsNoEof(u4, 4);
                    // a double 1 is just a single 1
                    if (nodebits == 1) {
                        nodes[node_idx].numbits = nodebits;
                        node_idx += 1;
                    } else {
                        // otherwise, we need one for value for the repeat count
                        const repcount = try bit_reader.readBitsNoEof(u4, 4) + 3;
                        for (0..repcount) |_| {
                            nodes[node_idx].numbits = nodebits;
                            node_idx += 1;
                        }
                    }
                }
            }
            log.debug("  Huffman nodes: {any}", .{nodes});

            // huffman_assign_canonical_codes

            var bithisto: [33]u32 = .{0} ** 33;
            for (&nodes) |*node| {
                if (node.numbits > MaxBits)
                    return error.HUFFERR_INTERNAL_INCONSISTENCY;
                if (node.numbits <= 32)
                    bithisto[node.numbits] += 1;
            }
            log.debug("  bithisto: {any}", .{bithisto});

            // for each code length, determine the starting code number
            var curstart: u32 = 0;
            var codelen: u32 = 32;
            while (codelen > 0) {
                const nextstart: u32 = (curstart + bithisto[codelen]) >> 1;
                if (codelen != 1 and nextstart * 2 != (curstart + bithisto[codelen]))
                    return error.HUFFERR_INTERNAL_INCONSISTENCY;
                bithisto[codelen] = curstart;
                curstart = nextstart;

                codelen -= 1;
            }
            log.debug("  bithisto: {any}", .{bithisto});

            // now assign canonical codes
            for (&nodes) |*node| {
                if (node.numbits > 0) {
                    node.bits = bithisto[node.numbits];
                    bithisto[node.numbits] += 1;
                }
            }
            log.debug("  Huffman nodes: {any}", .{nodes});

            var lookup: [1 << MaxBits]u16 = undefined;

            // huffman_build_lookup_table
            for (nodes, 0..) |node, curcode| {
                if (node.numbits > 0) {
                    const value: u16 = ((@as(u16, @intCast(curcode))) << 5) | node.numbits;
                    const shift = MaxBits - node.numbits;
                    const dest = node.bits << shift;
                    const dest_end = ((node.bits + 1) << shift) - 1;
                    if (dest >= lookup.len or dest_end >= lookup.len or dest_end < dest)
                        return error.HUFFERR_INTERNAL_INCONSISTENCY;
                    for (dest..dest_end + 1) |i| {
                        lookup[i] = value;
                    }
                }
            }
            log.debug("  lookup: {X}", .{lookup});

            var decod = decoder{
                .lookup = lookup,
            };

            // const map_entry_bytes = 12; // FIXME: If uncompressed: 4;
            // const raw_map_size = hunk_count * map_entry_bytes;
            const map = try allocator.alloc(packed struct(u96) { compression: CompressionType, length: u24, offset: u48, crc: u16 }, hunk_count);
            defer allocator.free(map);

            // For CRC computation only
            const raw_map = try allocator.alloc(u8, 12 * hunk_count);
            defer allocator.free(raw_map);

            var last_comp: CompressionType = .Type0;
            var rep_count: usize = 0;
            for (0..hunk_count) |i| {
                if (rep_count > 0) {
                    map[i].compression = last_comp;
                    rep_count -= 1;
                } else {
                    const val: CompressionType = @enumFromInt(try decod.next(&bit_reader));
                    if (val == .RLESmall) {
                        map[i].compression = last_comp;
                        rep_count = 2 + try decod.next(&bit_reader);
                    } else if (val == .RLELarge) {
                        map[i].compression = last_comp;
                        rep_count = 2 + 16 + (@as(usize, @intCast(try decod.next(&bit_reader))) << 4);
                        rep_count += try decod.next(&bit_reader);
                    } else {
                        map[i].compression = val;
                        last_comp = val;
                    }
                }
            }

            var curoffset: u48 = first_offset;
            var last_self: u48 = 0;
            for (0..hunk_count) |i| {
                var length: u24 = 0;
                var offset: u48 = curoffset;
                var crc: u16 = 0;
                switch (map[i].compression) {
                    .Type0, .Type1, .Type2, .Type3 => {
                        length = try bit_reader.readBitsNoEof(u24, length_bits);
                        curoffset += length;
                        crc = try bit_reader.readBitsNoEof(u16, 16);
                    },
                    .Self => {
                        offset = try bit_reader.readBitsNoEof(u48, self_bits);
                        last_self = offset;
                    },
                    .Self0, .Self1 => |t| {
                        if (t == .Self1) last_self += 1;
                        map[i].compression = .Self;
                        offset = last_self;
                    },
                    else => {
                        log.err("  Compression type {any} not supported.", .{map[i].compression});
                        return error.COMPRESSION_TYPE_TODO;
                    },
                }
                map[i].length = length;
                map[i].offset = offset;
                map[i].crc = crc;

                raw_map[12 * i] = @intFromEnum(map[i].compression);
                std.mem.bytesAsValue(u24, raw_map[12 * i + 1 ..]).* = @byteSwap(length);
                std.mem.bytesAsValue(u48, raw_map[12 * i + 4 ..]).* = @byteSwap(offset);
                std.mem.bytesAsValue(u16, raw_map[12 * i + 10 ..]).* = @byteSwap(crc);
            }

            //log.debug("  raw_map: {any}", .{raw_map});
            log.debug("  crc: {X} / {X}", .{ map_crc, crc16(raw_map) });
            log.debug("  map: {X}", .{std.mem.sliceAsBytes(map)[0..24]});
            log.debug("  raw_map: {X}", .{std.mem.sliceAsBytes(raw_map)[0..24]});

            return error.TODO;
        },
        else => {
            log.err(termcolor.red("CHD version {d} not supported."), .{version});
            return error.CHDVersionNotSupported;
        },
    }

    return self;
}

const CompressionType = enum(u8) {
    Type0 = 0,
    Type1 = 1,
    Type2 = 2,
    Type3 = 3,
    None = 4,
    Self = 5,
    Parent = 6,
    // ...
    RLESmall = 7,
    RLELarge = 8,
    Self0 = 9,
    Self1 = 10,
    ParentSelf = 11,
    Parent0 = 12,
    Parent1 = 13,
    _,
};

const decoder = struct {
    const MaxBits = 8;

    lookup: [1 << MaxBits]u16,
    unused_bits: u8 = 0,
    bits: u8 = 0,

    pub fn next(self: *@This(), bit_reader: anytype) !u8 {
        const byte = try bit_reader.readBitsNoEof(u8, MaxBits - self.unused_bits);
        self.bits |= byte;
        const tmp = self.lookup[self.bits];

        const used_bits = tmp & 0x1F;
        const value = tmp >> 5;

        self.unused_bits = @intCast(MaxBits - used_bits);
        if (used_bits == 8) {
            self.bits = 0;
        } else {
            self.bits <<= @intCast(used_bits);
        }
        return @truncate(value);
    }
};

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
