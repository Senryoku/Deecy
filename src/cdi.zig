const std = @import("std");

const FileBacking = @import("file_backing.zig");
const Track = @import("track.zig");

const log = std.log.scoped(.cdi);

pub const CDI = struct {
    const Version = enum(u32) {
        V2 = 0x80000004,
        V3 = 0x80000005,
        V4 = 0x80000006,
        _,
    };

    tracks: std.ArrayList(Track),
    _allocator: std.mem.Allocator,
    _file: FileBacking,

    pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !CDI {
        var self: @This() = .{
            .tracks = std.ArrayList(Track).init(allocator),
            ._allocator = allocator,
            ._file = try FileBacking.init(filepath, allocator),
        };
        errdefer self.deinit();

        const file = try std.fs.cwd().openFile(filepath, .{});
        defer file.close();

        const size = try file.getEndPos();
        if (size < 8) return error.InvalidCDI;
        try file.seekFromEnd(-8);
        var reader = file.reader();
        const version: Version = try reader.readEnum(Version, .little);
        log.debug("Version: {any}", .{version});

        switch (version) {
            .V2, .V3 => {
                const header_offset = try reader.readInt(u32, .little);
                log.debug("Header Offset: {X}", .{header_offset});

                try file.seekTo(header_offset);
                reader = file.reader();

                return error.UnsupportedCDIVersion;
            },
            .V4 => {
                const header_size = try reader.readInt(u32, .little);
                log.debug("Header Size: {X}", .{header_size});

                try file.seekFromEnd(-@as(i64, header_size));
                reader = file.reader();
            },
            else => return error.UnsupportedCDIVersion,
        }

        const session_count = try reader.readInt(u16, .little);
        log.debug("Session Count: {d}", .{session_count});

        var track_offset: u64 = 0;

        for (0..session_count) |_| {
            const track_count = try reader.readInt(u16, .little);
            log.debug("  Track Count: {d}", .{track_count});

            for (0..track_count) |_| {
                const null_or_extra = try reader.readInt(u32, .little);
                if (null_or_extra != 0)
                    try reader.skipBytes(4, .{});
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

                try reader.skipBytes(2, .{});
                const pregap = try reader.readInt(u32, .little);
                const length = try reader.readInt(u32, .little);
                log.debug("    Pregap: {d}, Length: {d}", .{ pregap, length });

                try reader.skipBytes(6, .{});
                const mode = try reader.readInt(u32, .little);
                log.debug("    Mode: {d}", .{mode});

                try reader.skipBytes(4, .{});
                const session_number = try reader.readInt(u32, .little);
                const track_number = try reader.readInt(u32, .little);
                const start_lba = try reader.readInt(u32, .little);
                const total_length = try reader.readInt(u32, .little);
                log.debug("    Session Number: {d}, Track Number: {d}, Start LBA: {X}, Total Length: {X}", .{ session_number, track_number, start_lba, total_length });

                try reader.skipBytes(16, .{});
                const sector_size: u32 = switch (try reader.readInt(u32, .little)) {
                    0 => 2048,
                    1 => 2336,
                    2 => 2352,
                    else => return error.InvalidSectorSize,
                };
                const sector_type = try reader.readInt(u32, .little);
                log.debug("    Sector Size: {d}, Sector Type: {d}", .{ sector_size, sector_type });
                try reader.skipBytes(1, .{});
                const total_length_2 = try reader.readInt(u32, .little);
                log.debug("    Total Length 2: {d}", .{total_length_2});

                try reader.skipBytes(25, .{});
                switch (version) {
                    .V2 => try reader.skipBytes(4, .{}),
                    else => {
                        if (try reader.readInt(u32, .little) == 0xFFFFFFFF)
                            try reader.skipBytes(78, .{});
                    },
                }

                log.debug("Creating view: {X}, length: {X}", .{ track_offset, total_length * sector_size });

                try self.tracks.append(.{
                    .num = track_number,
                    .offset = start_lba, // Start LBA
                    .track_type = @truncate(sector_type),
                    .format = sector_size, // Sector size
                    .pregap = pregap,
                    .data = try self._file.create_view(track_offset, total_length * sector_size),
                });

                track_offset += total_length * sector_size;

                try reader.skipBytes(4 + 8, .{});
                switch (version) {
                    .V2 => {},
                    else => try reader.skipBytes(1, .{}),
                }
            }
        }

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.tracks.deinit();
        self._file.deinit();
    }

    pub fn load_sectors(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
        _ = self;
        log.debug("load_sectors: {X} {X} {X}", .{ lba, count, dest.len });
        return 0;
    }

    pub fn load_sectors_raw(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
        _ = self;
        log.debug("load_sectors_raw: {X} {X} {X}", .{ lba, count, dest.len });
        return 0;
    }

    pub fn load_bytes(self: *const @This(), lba: u32, length: u32, dest: []u8) u32 {
        _ = self;
        log.debug("load_bytes: {X} {X} {X}", .{ lba, length, dest.len });
        return 0;
    }

    pub fn load_file(self: *const @This(), filename: []const u8, dest: []u8) !u32 {
        _ = self;
        log.debug("load_file: {s} {X}", .{ filename, dest.len });
        return error.NotImplemented;
    }
};
