const std = @import("std");
const termcolor = @import("termcolor");

const MemoryMappedFile = @import("../memory_mapped_file.zig");
const Track = @import("track.zig");
const Session = @import("session.zig");

const log = std.log.scoped(.chd);

tracks: std.ArrayList(Track),
sessions: std.ArrayList(Session),

version: u32 = undefined,
compressors: [4]Compression = undefined,
logical_bytes: u64 = undefined,
map_offset: u64 = undefined,
meta_offset: u64 = undefined,
hunk_bytes: u32 = undefined,
unit_bytes: u32 = undefined,
map: []MapEntry = undefined,

_file: MemoryMappedFile,

_allocator: std.mem.Allocator,

// NOTE: All number are big endian.

const Tag = "MComprHD";
const CD_MAX_SECTOR_DATA = 2352;
const CD_MAX_SUBCODE_DATA = 96;
const CD_FRAME_SIZE = (CD_MAX_SECTOR_DATA + CD_MAX_SUBCODE_DATA);

fn make_tag(comptime t: *const [4:0]u8) u32 {
    return @as(u32, t[0]) << 24 | @as(u32, t[1]) << 16 | @as(u32, t[2]) << 8 | @as(u32, t[3]) << 0;
}

const Compression = enum(u32) {
    None = 0,
    Zlib = make_tag("zlib"),
    Zstd = make_tag("zstd"),
    LZMA = make_tag("lzma"),
    Huffman = make_tag("huff"),
    Flac = make_tag("flac"),

    CD_Zlib = make_tag("cdzl"),
    CD_Zstd = make_tag("cdzs"),
    CD_LZMA = make_tag("cdlz"),
    CD_Flac = make_tag("cdfl"),

    _,
};

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

const MapEntry = struct { compression: CompressionType, length: u24, offset: u48, crc: u16 };

const MetadataTag = enum(u32) {
    HardDisk = make_tag("GDDD"),
    HardDiskIdent = make_tag("IDNT"),
    HardDiskKey = make_tag("KEY "),
    PCMCIACIS = make_tag("CIS "),
    CDROMOld = make_tag("CHCD"),
    CDROMTrack = make_tag("CHTR"),
    CDROMTrack2 = make_tag("CHT2"),
    GDROMOld = make_tag("CHGT"),
    GDROMTrack = make_tag("CHGD"),
    _,
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

const MetadataEntry = struct {
    tag: MetadataTag,
    offset: u64,
    length: u32,
    next: u64,
    flags: u8,
};

fn search_metadata(file: anytype, meta_offset: u64, tag: MetadataTag, index: u32) !MetadataEntry {
    var offset: u64 = meta_offset;
    var current_index: u32 = 0;

    var reader = file.reader();

    while (offset != 0) {
        var entry: MetadataEntry = undefined;
        try file.seekTo(offset);
        entry.tag = @enumFromInt(try reader.readInt(u32, .big));
        entry.length = try reader.readInt(u32, .big);
        entry.next = try reader.readInt(u64, .big);

        entry.offset = offset;
        entry.flags = @truncate(entry.length >> 24);
        entry.length &= 0x00FFFFFF;

        if (entry.tag == tag) {
            if (current_index == index) return entry;
            current_index += 1;
        }

        std.debug.print("  (search_metadata) Entry: {any}\n", .{entry});

        offset = entry.next;
    }

    return error.NotFound;
}

pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !@This() {
    var self: @This() = .{
        .tracks = std.ArrayList(Track).init(allocator),
        .sessions = std.ArrayList(Session).init(allocator),
        ._file = try MemoryMappedFile.init(filepath, allocator),
        ._allocator = allocator,
    };
    errdefer self.deinit();

    const file = try std.fs.cwd().openFile(filepath, .{});
    defer file.close();

    const reader = file.reader();
    const tag = try reader.readBytesNoEof(8);
    if (!std.mem.eql(u8, &tag, Tag)) return error.InvalidCHD;
    const header_length = try reader.readInt(u32, .big);
    self.version = try reader.readInt(u32, .big);

    switch (self.version) {
        5 => {
            if (header_length != 124) return error.InvalidCHDv5;
            self.compressors = [4]Compression{
                @enumFromInt(try reader.readInt(u32, .big)),
                @enumFromInt(try reader.readInt(u32, .big)),
                @enumFromInt(try reader.readInt(u32, .big)),
                @enumFromInt(try reader.readInt(u32, .big)),
            };
            self.logical_bytes = try reader.readInt(u64, .big);
            self.map_offset = try reader.readInt(u64, .big);
            self.meta_offset = try reader.readInt(u64, .big);
            self.hunk_bytes = try reader.readInt(u32, .big);
            self.unit_bytes = try reader.readInt(u32, .big);
            const raw_sha1 = try reader.readBytesNoEof(20);
            const sha1 = try reader.readBytesNoEof(20);
            const parent_sha1 = try reader.readBytesNoEof(20);

            const hunk_count = (self.logical_bytes + self.hunk_bytes - 1) / self.hunk_bytes;

            self.map = try allocator.alloc(MapEntry, hunk_count);

            log.debug("  Compressor: {any}", .{self.compressors});
            log.debug("  Logical Bytes: {X}", .{self.logical_bytes});
            log.debug("  Map Offset: {X}", .{self.map_offset});
            log.debug("  Meta Offset: {X}", .{self.meta_offset});
            log.debug("  Hunk Bytes: {X}", .{self.hunk_bytes});
            log.debug("  Unit Bytes: {X}", .{self.unit_bytes});
            log.debug("  Raw SHA1: {X}", .{raw_sha1});
            log.debug("  SHA1: {X}", .{sha1});
            log.debug("  Parent SHA1: {X}", .{parent_sha1});
            log.debug("  Hunk Count: {X}", .{hunk_count});

            try file.seekTo(self.map_offset);

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
            try file.seekTo(self.map_offset + 16);

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

            var decod = MapDecoder{
                .lookup = lookup,
            };

            // const map_entry_bytes = 12; // FIXME: If uncompressed: 4;
            // const raw_map_size = hunk_count * map_entry_bytes;

            // For CRC computation only
            const raw_map = try allocator.alloc(u8, 12 * hunk_count);
            defer allocator.free(raw_map);

            var last_comp: CompressionType = .Type0;
            var rep_count: usize = 0;
            for (0..hunk_count) |i| {
                if (rep_count > 0) {
                    self.map[i].compression = last_comp;
                    rep_count -= 1;
                } else {
                    const val: CompressionType = @enumFromInt(try decod.next(&bit_reader));
                    if (val == .RLESmall) {
                        self.map[i].compression = last_comp;
                        rep_count = 2 + try decod.next(&bit_reader);
                    } else if (val == .RLELarge) {
                        self.map[i].compression = last_comp;
                        rep_count = 2 + 16 + (@as(usize, @intCast(try decod.next(&bit_reader))) << 4);
                        rep_count += try decod.next(&bit_reader);
                    } else {
                        self.map[i].compression = val;
                        last_comp = val;
                    }
                }
            }

            if (decod.unused_bits != 0) {
                const pos = try file.getPos();
                try file.seekTo(pos - 1);
                _ = try bit_reader.readBitsNoEof(u8, 8 - decod.unused_bits);
            }

            var curoffset: u48 = first_offset;
            var last_self: u48 = 0;
            for (0..hunk_count) |i| {
                var length: u24 = 0;
                var offset: u48 = curoffset;
                var crc: u16 = 0;
                switch (self.map[i].compression) {
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
                        self.map[i].compression = .Self;
                        offset = last_self;
                    },
                    else => {
                        log.err("  Compression type {any} not supported.", .{self.map[i].compression});
                        return error.COMPRESSION_TYPE_TODO;
                    },
                }
                self.map[i].length = length;
                self.map[i].offset = offset;
                self.map[i].crc = crc;

                //log.debug("  MapEntry[{d}]: {any}", .{ i, map[i] });

                raw_map[12 * i] = @intFromEnum(self.map[i].compression);
                std.mem.bytesAsValue(u24, raw_map[12 * i + 1 ..]).* = @byteSwap(length);
                std.mem.bytesAsValue(u48, raw_map[12 * i + 4 ..]).* = @byteSwap(offset);
                std.mem.bytesAsValue(u16, raw_map[12 * i + 10 ..]).* = @byteSwap(crc);
            }
            log.debug("  Map CRC: {X} / {X}", .{ map_crc, crc16(raw_map) });
            if (map_crc != crc16(raw_map)) return error.InvalidMapCRC;

            const track_types = [_]MetadataTag{
                .GDROMTrack,
                .GDROMOld,
                .CDROMTrack2,
                .CDROMOld,
            };

            var current_track: MetadataEntry = undefined;
            var track_tag: ?MetadataTag = null;
            for (track_types) |t| {
                current_track = search_metadata(file, self.meta_offset, t, 0) catch |err| switch (err) {
                    error.NotFound => continue,
                    else => |e| return e,
                };
                track_tag = t;
                break;
            }

            if (track_tag == null) return error.NoTracks;

            var tracks = std.ArrayList(MetadataEntry).init(allocator);
            defer tracks.deinit();
            try tracks.append(current_track);

            while (current_track.next != 0) {
                current_track = search_metadata(file, current_track.next, track_tag.?, 0) catch |err| switch (err) {
                    error.NotFound => break,
                    else => |e| return e,
                };
                try tracks.append(current_track);
            }

            var current_fad: u32 = 0;
            for (tracks.items) |track| {
                log.debug("Metadata entry: {any}", .{track});
                try file.seekTo(track.offset + 16);
                switch (track.tag) {
                    .GDROMTrack => {
                        const buffer: []u8 = try allocator.alloc(u8, track.length - 1); // Includes a null terminator
                        defer allocator.free(buffer);

                        // "TRACK:%d TYPE:%s SUBTYPE:%s FRAMES:%d PAD:%d PREGAP:%d PGTYPE:%s PGSUB:%s POSTGAP:%d"
                        _ = try reader.read(buffer);
                        log.debug("  '{s}'", .{buffer});
                        var iterator = std.mem.tokenizeScalar(u8, buffer, ' ');
                        const track_num = try std.fmt.parseInt(u32, iterator.next().?["TRACK:".len..], 10);
                        const track_type_str = iterator.next().?["TYPE:".len..];
                        const sub_type = iterator.next().?["SUBTYPE:".len..];
                        const frames = try std.fmt.parseInt(u32, iterator.next().?["FRAMES:".len..], 10);
                        const pad = try std.fmt.parseInt(u32, iterator.next().?["PAD:".len..], 10);
                        const pregap = try std.fmt.parseInt(u32, iterator.next().?["PREGAP:".len..], 10);
                        const pgsub = iterator.next().?["PGSUB:".len..];
                        const pgtype = iterator.next().?["PGTYPE:".len..];
                        const postgap = try std.fmt.parseInt(u32, iterator.next().?["POSTGAP:".len..], 10);

                        log.debug("    track_num: {d}", .{track_num});
                        log.debug("    track_type: {s}", .{track_type_str});
                        log.debug("    sub_type: {s}", .{sub_type});
                        log.debug("    frames: {d}", .{frames});
                        log.debug("    pad: {d}", .{pad});
                        log.debug("    pregap: {d}", .{pregap});
                        log.debug("    pgtype: {s}", .{pgtype});
                        log.debug("    pgsub: {s}", .{pgsub});
                        log.debug("    postgap: {d}", .{postgap});

                        const track_type: u8 = if (std.mem.eql(u8, track_type_str, "AUDIO")) 0 else 4;

                        const format: u32 = if (std.mem.eql(u8, track_type_str, "AUDIO")) 2336 else if (std.mem.eql(u8, track_type_str, "MODE1_RAW")) 2352 else return error.UnsupportedFormat;

                        const data = try allocator.alloc(u8, CD_MAX_SECTOR_DATA * frames);

                        const sectors_per_hunk = self.hunk_bytes / CD_FRAME_SIZE;

                        var cursor: usize = 0;

                        for (@divTrunc(current_fad, sectors_per_hunk)..@divTrunc(current_fad + frames, sectors_per_hunk)) |hunk| {
                            cursor += try self.read_hunk(file, hunk, data[cursor..]);
                        }

                        try self.tracks.append(.{
                            .num = track_num,
                            .fad = current_fad,
                            .track_type = track_type,
                            .format = format,
                            .pregap = 0,
                            .data = data,
                        });
                        current_fad += frames;
                    },
                    else => return error.UnsupportedTrackType,
                }
            }
        },
        else => {
            log.err(termcolor.red("CHD version {d} not supported."), .{self.version});
            return error.CHDVersionNotSupported;
        },
    }

    return self;
}

fn read_hunk(self: @This(), file: std.fs.File, hunk: usize, dest: []u8) !usize {
    log.debug("Reading hunk {d}", .{hunk});

    const compression = switch (self.map[hunk].compression) {
        .Type0 => self.compressors[0],
        .Type1 => self.compressors[1],
        .Type2 => self.compressors[2],
        .Type3 => self.compressors[3],
        .Self => {
            std.debug.assert(hunk != self.map[hunk].offset);
            return self.read_hunk(file, self.map[hunk].offset, dest);
        },
        else => {
            log.err("Unsupported compression type: {any}", .{self.map[hunk].compression});
            return error.UnsupportedCompressionType;
        },
    };

    const sectors_per_hunk = self.hunk_bytes / CD_FRAME_SIZE;
    const complen_bytes: u32 = if (sectors_per_hunk * CD_FRAME_SIZE < 65536) 2 else 3;
    const ecc_bytes: u32 = (sectors_per_hunk + 7) / 8;
    const header_bytes: u32 = ecc_bytes + complen_bytes;

    const file_reader = file.reader();

    switch (compression) {
        .CD_LZMA => {
            // try file.seekTo(map[hunk].offset);
            // const ecc = try reader.readBytesNoEof(ecc_bytes);
            // const complen = if (complen_bytes == 2) try reader.readInt(u16, .big) else try reader.readInt(u24, .big);

            try file.seekTo(self.map[hunk].offset + header_bytes);
            var decompressor = try std.compress.lzma.Decompress(@TypeOf(file_reader)).init(self._allocator, file_reader, .{
                // There is no LZMA headers and these parameters are implicit... FIXME: They also depend on hunk_size, these are the parameters for hunk_size = 0x4C80.
                .properties = .{
                    .lc = 3,
                    .lp = 0,
                    .pb = 2,
                },
                .dict_size = 0x6000,
                .unpacked_size = sectors_per_hunk * CD_MAX_SECTOR_DATA,
            }, null);
            defer decompressor.deinit();
            return try decompressor.read(dest[0 .. sectors_per_hunk * CD_MAX_SECTOR_DATA]);
        },
        .CD_Zlib => {
            try file.seekTo(self.map[hunk].offset + header_bytes);
            var output_stream = std.io.fixedBufferStream(dest[0..]);
            try std.compress.flate.decompress(file_reader, output_stream.writer());
            return try output_stream.getPos();
        },
        .CD_Flac => {
            log.err("Unsupported compression type: {any} (Skipping for test...)", .{compression});

            @memset(dest[0..@min(dest.len, CD_MAX_SECTOR_DATA * sectors_per_hunk)], 0);
            return @min(dest.len, CD_MAX_SECTOR_DATA * sectors_per_hunk);
        },
        else => {
            log.err("Unsupported compression type: {any}", .{compression});
            return error.UnsupportedCompression;
        },
    }
}

const MapDecoder = struct {
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

pub fn deinit(self: *@This()) void {
    self._allocator.free(self.map);

    self.sessions.deinit();
    for (self.tracks.items) |track|
        self._allocator.free(track.data);
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
