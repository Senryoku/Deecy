const std = @import("std");
const termcolor = @import("termcolor");

const BitReader = @import("bit_reader.zig");
const chd_flac = @import("chd_flac.zig");
const host_memory = @import("../host/host_memory.zig");
const MemoryMappedFile = @import("../host/memory_mapped_file.zig");
const Track = @import("track.zig");
const Session = @import("session.zig");

const log = std.log.scoped(.chd);

tracks: std.ArrayList(Track) = .empty,
sessions: std.ArrayList(Session) = .empty,

version: u32 = undefined,
compressors: [4]Compression = undefined,
logical_bytes: u64 = undefined,
map_offset: u64 = undefined,
meta_offset: u64 = undefined,
hunk_bytes: u32 = undefined,
unit_bytes: u32 = undefined,
map: []MapEntry = undefined,

track_offsets: std.ArrayList(u32) = .empty,
track_data: std.ArrayList([]u8) = .empty,

_file: MemoryMappedFile,
_file_view: []const u8 = undefined,

_allocator: std.mem.Allocator,

// NOTE: All numbers in CHD headers are big endian.

const Tag = "MComprHD";
const CDMaxSectorBytes = 2352;
const CDMaxSubcodeBytes = 96;
const CDFrameSize = CDMaxSectorBytes + CDMaxSubcodeBytes;

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

const MapEntry = struct {
    compression: CompressionType,
    length: u24,
    offset: u48,
    crc: u16,
    loaded: bool = false,
};

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

const MetadataEntry = struct {
    tag: MetadataTag,
    offset: u64,
    length: u32,
    next: u64,
    flags: u8,
};

pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !@This() {
    var self: @This() = .{
        ._file = try MemoryMappedFile.init(filepath, allocator),
        ._allocator = allocator,
    };
    errdefer self.deinit();
    self._file_view = try self._file.create_full_view();

    var stream = std.io.fixedBufferStream(self._file_view);
    const reader = stream.reader();
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

            log.debug("  Compressor: {any}", .{self.compressors});
            log.debug("  Logical Bytes: {X}", .{self.logical_bytes});
            log.debug("  Map Offset: {X}", .{self.map_offset});
            log.debug("  Meta Offset: {X}", .{self.meta_offset});
            log.debug("  Hunk Bytes: {X}", .{self.hunk_bytes});
            log.debug("  Unit Bytes: {X}", .{self.unit_bytes});
            log.debug("  Raw SHA1: {X}", .{raw_sha1});
            log.debug("  SHA1: {X}", .{sha1});
            log.debug("  Parent SHA1: {X}", .{parent_sha1});

            try self.decode_map_v5();

            var current_track: MetadataEntry = undefined;
            var track_tag: ?MetadataTag = null;
            // Check each track metadata type to find the one used in this file.
            for ([_]MetadataTag{ .GDROMTrack, .GDROMOld, .CDROMTrack2, .CDROMOld }) |t| {
                current_track = self.search_metadata(null, t, 0) catch |err| switch (err) {
                    error.NotFound => continue,
                    else => return err,
                };
                track_tag = t;
                break;
            }
            if (track_tag == null) return error.NoTracks;

            var tracks_metadata = std.ArrayList(MetadataEntry).init(allocator);
            defer tracks_metadata.deinit();
            try tracks_metadata.append(current_track);

            while (current_track.next != 0) {
                current_track = self.search_metadata(current_track.next, track_tag.?, 0) catch |err| switch (err) {
                    error.NotFound => break,
                    else => |e| return e,
                };
                try tracks_metadata.append(current_track);
            }

            var fad_offset: u32 = 0;
            var current_fad: u32 = 150;
            for (tracks_metadata.items) |track| {
                log.debug("Track Metadata entry: {any}", .{track});
                try stream.seekTo(track.offset + 16);
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
                        const pgtype = iterator.next().?["PGTYPE:".len..];
                        const pgsub = iterator.next().?["PGSUB:".len..];
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

                        log.debug("    fad: {d}", .{current_fad});

                        const sectors_per_hunk = self.hunk_bytes / CDFrameSize;
                        const track_type: Track.TrackType = if (std.mem.eql(u8, track_type_str, "AUDIO")) .Audio else .Data;
                        const format: u32 = if (std.mem.eql(u8, track_type_str, "AUDIO")) 2336 else if (std.mem.eql(u8, track_type_str, "MODE1_RAW")) CDMaxSectorBytes else return error.UnsupportedFormat;
                        const data = try host_memory.virtual_alloc(u8, CDMaxSectorBytes * std.mem.alignForward(u32, frames, sectors_per_hunk));

                        try self.track_data.append(data);
                        try self.tracks.append(.{
                            .num = track_num,
                            .fad = current_fad,
                            .track_type = track_type,
                            .format = format,
                            .pregap = 0,
                            .data = data,
                        });
                        try self.track_offsets.append(current_fad - fad_offset);

                        current_fad += frames;
                        fad_offset += std.mem.alignForward(u32, frames, 4);
                    },
                    else => return error.UnsupportedTrackType,
                }
            }

            try self.decompress_sectors(self.tracks.items[2].fad, 1);
        },
        else => {
            log.err(termcolor.red("CHD version {d} not supported."), .{self.version});
            return error.CHDVersionNotSupported;
        },
    }

    return self;
}

pub fn deinit(self: *@This()) void {
    self._allocator.free(self.map);

    self.sessions.deinit(self._allocator);
    self.track_offsets.deinit(self._allocator);
    for (self.track_data.items) |track_data|
        host_memory.virtual_dealloc(track_data);
    self.track_data.deinit(self._allocator);
    self.tracks.deinit(self._allocator);
    self._file.deinit();
}

pub fn get_first_data_track(self: *const @This()) ?Track {
    for (self.tracks.items[self.get_area_boundaries(.DoubleDensity)[0]..]) |track| {
        if (track.track_type == .Data)
            return track;
    }
    return null;
}

pub fn read_sector(self: *@This(), fad: u32) ![]const u8 {
    try self.decompress_sectors(fad, 1);
    const track = Track.get_corresponding_track(&self.tracks, fad);
    return track.read_sector(fad);
}

pub fn load_sectors(self: *@This(), fad: u32, count: u32, dest: []u8) u32 {
    self.decompress_sectors(fad, count) catch |err| {
        log.err("Failed to decompress sectors [{d}, {d}]: {s}", .{ fad, fad + count - 1, @errorName(err) });
        return 0;
    };
    const track = Track.get_corresponding_track(&self.tracks, fad);
    return track.load_sectors(fad, count, dest);
}

pub fn load_sectors_raw(self: *@This(), fad: u32, count: u32, dest: []u8) u32 {
    self.decompress_sectors(fad, count) catch |err| {
        log.err("Failed to decompress sectors [{d}, {d}]: {s}", .{ fad, fad + count - 1, @errorName(err) });
        return 0;
    };
    const track = Track.get_corresponding_track(&self.tracks, fad);
    return track.load_sectors_raw(fad, count, dest);
}

/// Helper for map decoding: Bit reader with "rewind" functionality.
/// Allows "peeking" for MaxBits but only consuming part of it, i.e. without discarding the rest.
fn MapDecoder(comptime MaxBits: u8) type {
    return struct {
        lookup: [1 << MaxBits]u16 = undefined,
        unused_bits: u8 = 0, // Unused bits count
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
}
/// Map is compressed using huffman running length encoding.
/// This is a custom implementation by MAME, so this function is a direct port for compatibility.
fn decode_map_v5(self: *@This()) !void {
    const hunk_count = (self.logical_bytes + self.hunk_bytes - 1) / self.hunk_bytes;
    log.debug("  Hunk Count: {X}", .{hunk_count});

    self.map = try self._allocator.alloc(MapEntry, hunk_count);

    var reader = std.Io.Reader.fixed(self._file_view);
    std.debug.assert(reader.discardShort(self.map_offset) == self.map_offset);

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

    std.debug.assert(reader.discardShort(1) == 1);

    const NumCodes = 16;
    const MaxBits = 8;

    var nodes: [NumCodes]struct { bits: u32, num_bits: u4 } = undefined;

    var bit_reader: BitReader = .init(reader);
    var node_idx: u32 = 0;
    while (node_idx < NumCodes) {
        var nodebits: u4 = try bit_reader.readBitsNoEof(u4, 4);
        if (nodebits != 1) {
            nodes[node_idx].num_bits = nodebits;
            node_idx += 1;
        } else {
            // A one value is an escape code
            nodebits = try bit_reader.readBitsNoEof(u4, 4);
            // A double 1 is just a single 1
            if (nodebits == 1) {
                nodes[node_idx].num_bits = nodebits;
                node_idx += 1;
            } else {
                // Otherwise, we need one for value for the repeat count
                const repcount = try bit_reader.readBitsNoEof(u4, 4) + 3;
                for (0..repcount) |_| {
                    nodes[node_idx].num_bits = nodebits;
                    node_idx += 1;
                }
            }
        }
    }

    var bit_histogram: [33]u32 = @splat(0);
    for (&nodes) |*node| {
        if (node.num_bits > MaxBits)
            return error.MapHuffmanError;
        if (node.num_bits <= 32)
            bit_histogram[node.num_bits] += 1;
    }

    var cur_start: u32 = 0;
    var code_length: u32 = 32;
    while (code_length > 0) {
        const nextstart: u32 = (cur_start + bit_histogram[code_length]) >> 1;
        if (code_length != 1 and nextstart * 2 != (cur_start + bit_histogram[code_length]))
            return error.MapHuffmanError;
        bit_histogram[code_length] = cur_start;
        cur_start = nextstart;

        code_length -= 1;
    }
    for (&nodes) |*node| {
        if (node.num_bits > 0) {
            node.bits = bit_histogram[node.num_bits];
            bit_histogram[node.num_bits] += 1;
        }
    }

    var decoder = MapDecoder(MaxBits){};
    // Generate decoder lookup table
    for (nodes, 0..) |node, current_code| {
        if (node.num_bits > 0) {
            const value: u16 = ((@as(u16, @intCast(current_code))) << 5) | node.num_bits;
            const shift = MaxBits - node.num_bits;
            const dest = node.bits << shift;
            const dest_end = ((node.bits + 1) << shift) - 1;
            if (dest >= decoder.lookup.len or dest_end >= decoder.lookup.len or dest_end < dest)
                return error.MapHuffmanError;
            for (dest..dest_end + 1) |i|
                decoder.lookup[i] = value;
        }
    }

    var last_comp: CompressionType = .Type0;
    var rep_count: usize = 0;
    for (self.map) |*entry| {
        if (rep_count > 0) {
            entry.compression = last_comp;
            rep_count -= 1;
        } else {
            const val: CompressionType = @enumFromInt(try decoder.next(&bit_reader));
            switch (val) {
                .RLESmall => {
                    entry.compression = last_comp;
                    rep_count = 2 + try decoder.next(&bit_reader);
                },
                .RLELarge => {
                    entry.compression = last_comp;
                    rep_count = 2 + 16 + (@as(usize, @intCast(try decoder.next(&bit_reader))) << 4);
                    rep_count += try decoder.next(&bit_reader);
                },
                else => {
                    entry.compression = val;
                    last_comp = val;
                },
            }
        }
    }

    // FIXME: This might be a bit annoying
    if (decoder.unused_bits != 0) {
        const pos = try stream.getPos();
        try stream.seekTo(pos - 1);
        _ = try bit_reader.readBitsNoEof(u8, 8 - decoder.unused_bits);
    }

    // For CRC computation only
    const raw_map = try self._allocator.alloc(u8, 12 * hunk_count);
    defer self._allocator.free(raw_map);

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
                log.err("  Compression type {} not supported.", .{self.map[i].compression});
                return error.UnsupportedCompressionType;
            },
        }
        self.map[i].length = length;
        self.map[i].offset = offset;
        self.map[i].crc = crc;
        self.map[i].loaded = false;

        raw_map[12 * i] = @intFromEnum(self.map[i].compression);
        std.mem.bytesAsValue(u24, raw_map[12 * i + 1 ..]).* = @byteSwap(length);
        std.mem.bytesAsValue(u48, raw_map[12 * i + 4 ..]).* = @byteSwap(offset);
        std.mem.bytesAsValue(u16, raw_map[12 * i + 10 ..]).* = @byteSwap(crc);
    }

    if (map_crc != crc16(raw_map)) return error.InvalidMapCRC;
}

fn search_metadata(self: *@This(), offset: ?u64, tag: MetadataTag, index: u32) !MetadataEntry {
    var current_offset: u64 = if (offset) |o| o else self.meta_offset;
    var current_index: u32 = 0;

    var stream = std.io.fixedBufferStream(self._file_view);
    var reader = stream.reader();

    while (current_offset != 0) {
        var entry: MetadataEntry = undefined;
        try stream.seekTo(current_offset);
        entry.tag = @enumFromInt(try reader.readInt(u32, .big));
        entry.length = try reader.readInt(u32, .big);
        entry.next = try reader.readInt(u64, .big);

        entry.offset = current_offset;
        entry.flags = @truncate(entry.length >> 24);
        entry.length &= 0x00FFFFFF;

        if (entry.tag == tag) {
            if (current_index == index) return entry;
            current_index += 1;
        }

        current_offset = entry.next;
    }

    return error.NotFound;
}

pub fn decompress_sectors(self: *@This(), fad: u32, count: u32) !void {
    log.debug("Decompressing sectors [{d}, {d}]", .{ fad, fad + count - 1 });
    const sectors_per_hunk = self.hunk_bytes / CDFrameSize;

    var track_idx: usize = 0;
    while (track_idx + 1 < self.tracks.items.len and self.tracks.items[track_idx + 1].fad <= fad) : (track_idx += 1) {}

    // A track can start in the middle of a hunk. Compensate for that.
    const first_hunk = @divTrunc(self.tracks.items[track_idx].fad - self.track_offsets.items[track_idx], sectors_per_hunk);
    const hunk_offset = self.tracks.items[track_idx].fad - (sectors_per_hunk * first_hunk + self.track_offsets.items[track_idx]); // How much sectors belonging to the previous track we'll have to discard.
    const track_byte_offset: u32 = hunk_offset * CDMaxSectorBytes;

    const shifted_fad: usize = fad - self.track_offsets.items[track_idx];
    var start_hunk = @divTrunc(shifted_fad, sectors_per_hunk);
    const end_hunk = @min(self.map.len, 1 + @divTrunc(shifted_fad + count + hunk_offset, sectors_per_hunk));

    var sector_offset = std.mem.alignBackward(usize, fad - self.tracks.items[track_idx].fad, sectors_per_hunk);

    var dest = self.track_data.items[track_idx];

    if (hunk_offset != 0) {
        const usable_sectors_in_first_hunk = sectors_per_hunk - hunk_offset;
        if (start_hunk == first_hunk) {
            // First hunk of track, but there are additional sectors before the first one we're actually interested in. We can't write directly to the track.
            // FIXME: Since we mark the hunk as loaded, access to the last sectors of the previous track will access uninitialized memory...
            //        This probably won't be a problem in practice (because of padding, and I don't even know of a single example of such access),
            //        but we may want to fix this at some point anyway.
            if (!self.map[first_hunk].loaded) {
                self.map[first_hunk].loaded = true;
                const tmp = try self._allocator.alloc(u8, sectors_per_hunk * CDMaxSectorBytes);
                defer self._allocator.free(tmp);
                const bytes = try self.read_hunk(first_hunk, tmp);
                @memcpy(dest[0 .. bytes - track_byte_offset], tmp[track_byte_offset..]);
            }
            sector_offset = usable_sectors_in_first_hunk;
            start_hunk += 1;
        } else {
            sector_offset = std.mem.alignBackward(usize, fad - self.tracks.items[track_idx].fad - usable_sectors_in_first_hunk, sectors_per_hunk) + usable_sectors_in_first_hunk;
        }
    }

    for (start_hunk..end_hunk) |hunk| {
        if (!self.map[hunk].loaded) {
            self.map[hunk].loaded = true;
            const bytes = try self.read_hunk(hunk, dest[sector_offset * CDMaxSectorBytes ..][0 .. sectors_per_hunk * CDMaxSectorBytes]);
            std.debug.assert(bytes == sectors_per_hunk * CDMaxSectorBytes);
        }
        sector_offset += sectors_per_hunk;
    }
}

fn read_hunk(self: *const @This(), hunk: usize, dest: []u8) !usize {
    log.debug("  Reading hunk {d}", .{hunk});

    const compression = switch (self.map[hunk].compression) {
        .Type0 => self.compressors[0],
        .Type1 => self.compressors[1],
        .Type2 => self.compressors[2],
        .Type3 => self.compressors[3],
        .Self => {
            std.debug.assert(hunk != self.map[hunk].offset);
            return self.read_hunk(self.map[hunk].offset, dest);
        },
        else => {
            log.err("Unsupported compression type: {any}", .{self.map[hunk].compression});
            return error.UnsupportedCompressionType;
        },
    };

    const sectors_per_hunk = self.hunk_bytes / CDFrameSize;
    const complen_bytes: u32 = if (self.hunk_bytes < 65536) 2 else 3;
    const ecc_bytes: u32 = (sectors_per_hunk + 7) / 8;
    const header_bytes: u32 = ecc_bytes + complen_bytes;

    var header_stream = std.io.fixedBufferStream(self._file_view[self.map[hunk].offset + ecc_bytes ..]);
    const header_reader = header_stream.reader();
    const compressed_length = if (complen_bytes == 2) try header_reader.readInt(u16, .big) else try header_reader.readInt(u24, .big);

    var file_reader = std.io.Reader.fixed(self._file_view[self.map[hunk].offset + header_bytes ..][0..compressed_length]);

    switch (compression) {
        .CD_LZMA => {
            var decompressor = try std.compress.lzma.Decompress(@TypeOf(file_reader)).init(self._allocator, file_reader, .{
                // There is no LZMA headers and these parameters are implicit...
                // FIXME: They also depend on hunk_size, these are the parameters for hunk_size = 0x4C80.
                .properties = .{
                    .lc = 3,
                    .lp = 0,
                    .pb = 2,
                },
                .dict_size = 0x6000,
                .unpacked_size = sectors_per_hunk * CDMaxSectorBytes,
            }, null);
            defer decompressor.deinit();
            const bytes = try decompressor.read(dest[0 .. sectors_per_hunk * CDMaxSectorBytes]);
            // This probably won't work without subcodes/raw sector data
            // if (self.map[hunk].crc != crc16(dest[0..bytes]))
            //     return error.InvalidCRC;
            return bytes;
        },
        .CD_Zlib => {
            const writer = std.io.Writer.fixed(dest[0 .. sectors_per_hunk * CDMaxSectorBytes]);
            var decompress: std.compress.flate.Decompress = .init(&file_reader, .zlib, &.{});
            const bytes = try decompress.reader.streamRemaining(writer);
            // This probably won't work without subcodes/raw sector data
            // if (self.map[hunk].crc != crc16(dest[0..bytes]))
            //     return error.InvalidCRC;
            return bytes;
        },
        .CD_Flac => {
            const bytes = sectors_per_hunk * CDMaxSectorBytes;
            const num_samples = bytes / @sizeOf(i16);
            var flac_stream = std.io.fixedBufferStream(self._file_view[self.map[hunk].offset..]);
            const flac_reader = flac_stream.reader();
            try chd_flac.decode_frames(i16, self._allocator, flac_reader, @as([*]i16, @ptrCast(@alignCast(dest.ptr)))[0 .. sectors_per_hunk * CDMaxSectorBytes / 2], num_samples, CDMaxSectorBytes, 2, 16);
            return bytes;
        },
        else => {
            log.err("Unsupported compression method: {any}", .{compression});
            return error.UnsupportedCompression;
        },
    }
}

// FIXME: The following methods shouldn't be hardcoded.

pub fn get_session_count(_: *const @This()) u32 {
    return 2;
}

pub fn get_session(self: *const @This(), session_number: u32) Session {
    return switch (session_number) {
        1 => .{
            .first_track = 0,
            .last_track = 1,
            .start_fad = 0,
            .end_fad = @intCast(self.tracks.items[1].get_end_fad()),
        },
        2 => .{
            .first_track = 2,
            .last_track = @intCast(self.tracks.items.len - 1),
            .start_fad = self.tracks.items[2].fad,
            .end_fad = @intCast(self.tracks.items[self.tracks.items.len - 1].get_end_fad()),
        },
        else => std.debug.panic("CDH: Invalid session number: {d}", .{session_number}),
    };
}

pub fn get_area_boundaries(self: *const @This(), area: Session.Area) [2]u32 {
    return switch (area) {
        .SingleDensity => [2]u32{ 0, 1 },
        .DoubleDensity => [2]u32{ 2, @intCast(self.tracks.items.len - 1) },
    };
}

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
