const std = @import("std");

const MemoryMappedFile = @import("../memory_mapped_file.zig");
const Track = @import("track.zig");
const Session = @import("session.zig");

const log = std.log.scoped(.cdi);

pub const CDI = struct {
    const Version = enum(u32) {
        V2 = 0x80000004,
        V3 = 0x80000005,
        V4 = 0x80000006,
        _,
    };

    tracks: std.ArrayList(Track),
    sessions: std.ArrayList(Session),
    _file: MemoryMappedFile,

    pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !CDI {
        var self: @This() = .{
            .tracks = std.ArrayList(Track).init(allocator),
            .sessions = std.ArrayList(Session).init(allocator),
            ._file = try MemoryMappedFile.init(filepath, allocator),
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

        std.debug.assert(session_count == 2);

        for (0..session_count) |_| {
            var session = Session{
                .first_track = @intCast(self.tracks.items.len),
                .last_track = 0,
                .start_fad = 0,
                .end_fad = 0,
            };

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
                log.debug("    Pregap: {X}, Length: {X}", .{ pregap, length });
                std.debug.assert(pregap == 150); // Assumed for the session start fad.

                try reader.skipBytes(6, .{});
                const mode = try reader.readInt(u32, .little);
                log.debug("    Mode: {d}", .{mode});

                try reader.skipBytes(4, .{});
                const session_number = try reader.readInt(u32, .little);
                const track_number = try reader.readInt(u32, .little);
                const start_lba = try reader.readInt(u32, .little);
                const total_length = try reader.readInt(u32, .little);
                log.debug("    Session Number: {d}, Track Number: {d}, Start LBA: {X}, Track Length: {X}", .{ session_number, track_number, start_lba, total_length });
                std.debug.assert(total_length == length + pregap);

                try reader.skipBytes(16, .{});
                const sector_size: u32 = switch (try reader.readInt(u32, .little)) {
                    0 => 2048,
                    1 => 2336,
                    2 => 2352,
                    else => return error.InvalidSectorSize,
                };
                const sector_type = try reader.readInt(u32, .little);
                log.debug("    Sector Size: {d}, Sector Type: {d}", .{ sector_size, sector_type });
                std.debug.assert(sector_type == 0 or sector_type == 4);
                try reader.skipBytes(1, .{});

                const total_length_2 = try reader.readInt(u32, .little); // Repeated?
                log.debug("    Total Length: {d}", .{total_length_2});
                std.debug.assert(total_length_2 == length + pregap);
                std.debug.assert(total_length_2 == total_length);

                try reader.skipBytes(25, .{});
                switch (version) {
                    .V2 => try reader.skipBytes(4, .{}),
                    else => {
                        if (try reader.readInt(u32, .little) == 0xFFFFFFFF)
                            try reader.skipBytes(78, .{});
                    },
                }

                log.debug("     [+] Creating view: {X}, length: {X}", .{ track_offset + pregap * sector_size, length * sector_size });

                try self.tracks.append(.{
                    .num = @truncate(self.tracks.items.len + 1),
                    .fad = start_lba + pregap,
                    .track_type = @truncate(sector_type),
                    .format = sector_size,
                    .pregap = pregap,
                    .data = try self._file.create_view(track_offset + pregap * sector_size + 8, length * sector_size), // FIXME: +8 Found empirically, no idea if this is correct.
                });

                log.debug("Loaded: {any}", .{self.tracks.items[self.tracks.items.len - 1].data[0..32]});

                track_offset += total_length * sector_size;
            }
            const session_type = try reader.readInt(u32, .little);
            try reader.skipBytes(4, .{});
            session.start_fad = try reader.readInt(u32, .little) + 150;
            log.debug("Session Type: {X}, Last Session Start LBA: {X}", .{ session_type, session.start_fad });
            std.debug.assert(session.start_fad == self.tracks.items[session.first_track].fad);
            if (version != .V2) try reader.skipBytes(1, .{});

            session.last_track = @intCast(self.tracks.items.len - 1);
            session.end_fad = self.tracks.items[session.last_track].get_end_fad();
            try self.sessions.append(session);
        }
        const total_tracks = try reader.readInt(u16, .little);
        std.debug.assert(total_tracks == 0);
        const end_lba = try reader.readInt(u32, .little);
        const volume_name_length = try reader.readInt(u8, .little);
        var volume_name_buffer: [256]u8 = undefined;
        const volume_name = volume_name_buffer[0..volume_name_length];
        _ = try reader.read(volume_name);
        log.debug("Total Tracks: {d}, End LBA: {X}, Volume Name Length: {d}, Volume Name: {s}", .{ total_tracks, end_lba, volume_name_length, volume_name });

        return self;
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
};
