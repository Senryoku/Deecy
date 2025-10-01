const std = @import("std");

const MemoryMappedFile = @import("../host/memory_mapped_file.zig");
const Track = @import("track.zig");
const Session = @import("session.zig");

const log = std.log.scoped(.cdi);

const Version = enum(u32) {
    V2 = 0x80000004,
    V3 = 0x80000005,
    V4 = 0x80000006,
    _,
};

tracks: std.ArrayList(Track) = .empty,
sessions: std.ArrayList(Session) = .empty,
_file: MemoryMappedFile,

pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !@This() {
    var self: @This() = .{
        ._file = try .init(filepath, allocator),
    };
    errdefer self.deinit(allocator);

    const file = try std.fs.cwd().openFile(filepath, .{});
    defer file.close();

    const buffer = try allocator.alloc(u8, 8192);
    defer allocator.free(buffer);
    var file_reader = file.reader(buffer);

    const size = try file_reader.getSize();
    if (size < 8) return error.InvalidCDI;
    try file_reader.seekTo(size - 8);

    var reader = &file_reader.interface;
    const version: Version = try reader.takeEnum(Version, .little);
    log.debug("Version: {}", .{version});

    switch (version) {
        .V2, .V3 => {
            const header_offset = try reader.takeInt(u32, .little);
            log.debug("Header Offset: {X}", .{header_offset});

            try file_reader.seekTo(header_offset);
        },
        .V4 => {
            const header_size = try reader.takeInt(u32, .little);
            log.debug("Header Size: {X}", .{header_size});

            try file_reader.seekTo(try file_reader.getSize() - header_size);
        },
        else => return error.UnsupportedCDIVersion,
    }

    const session_count = try reader.takeInt(u16, .little);
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

        const track_count = try reader.takeInt(u16, .little);
        log.debug("  Track Count: {d}", .{track_count});

        for (0..track_count) |_| {
            try track_header(reader);

            std.debug.assert(try reader.discardShort(2) == 2);
            const pregap = try reader.takeInt(u32, .little);
            const length = try reader.takeInt(u32, .little);
            log.debug("    Pregap: {X}, Length: {X}", .{ pregap, length });
            if (pregap != 150) log.warn("Pregap: {d}", .{pregap});

            std.debug.assert(try reader.discardShort(6) == 6);
            const mode = try reader.takeInt(u32, .little);
            log.debug("    Mode: {d}", .{mode});

            std.debug.assert(try reader.discardShort(4) == 4);
            const session_number = try reader.takeInt(u32, .little);
            const track_number = try reader.takeInt(u32, .little);
            const start_lba = try reader.takeInt(u32, .little);
            const total_length = try reader.takeInt(u32, .little);
            log.debug("    Session Number: {d}, Track Number: {d}, Start LBA: {X}, Track Length: {X}", .{ session_number, track_number, start_lba, total_length });
            std.debug.assert(total_length == length + pregap);

            std.debug.assert(try reader.discardShort(16) == 16);
            const sector_size: u32 = switch (try reader.takeInt(u32, .little)) {
                0 => 2048,
                1 => 2336,
                2 => 2352,
                else => return error.InvalidSectorSize,
            };
            const sector_type = try reader.takeInt(u32, .little);
            log.debug("    Sector Size: {d}, Sector Type: {d}", .{ sector_size, sector_type });
            std.debug.assert(sector_type == 0 or sector_type == 4);
            std.debug.assert(try reader.discardShort(1) == 1);

            const total_length_2 = try reader.takeInt(u32, .little); // Repeated?
            log.debug("    Total Length: {d}", .{total_length_2});
            std.debug.assert(total_length_2 == length + pregap);
            std.debug.assert(total_length_2 == total_length);

            std.debug.assert(try reader.discardShort(25) == 25);
            switch (version) {
                .V2 => std.debug.assert(try reader.discardShort(4) == 4),
                else => {
                    if (try reader.takeInt(u32, .little) == 0xFFFFFFFF)
                        std.debug.assert(try reader.discardShort(78) == 78);
                },
            }

            log.debug("     [+] Creating view: {X}, length: {X}", .{ track_offset + pregap * sector_size, length * sector_size });

            try self.tracks.append(allocator, .{
                .num = @truncate(self.tracks.items.len + 1),
                .fad = start_lba + pregap,
                .track_type = @enumFromInt(sector_type),
                .format = sector_size,
                .pregap = pregap,
                .data = try self._file.create_view(track_offset + pregap * sector_size, length * sector_size),
            });

            log.debug("Loaded: {any}", .{self.tracks.items[self.tracks.items.len - 1].data[0..32]});

            track_offset += total_length * sector_size;
        }
        const session_type = try reader.takeInt(u32, .little);
        std.debug.assert(try reader.discardShort(4) == 4);
        session.start_fad = try reader.takeInt(u32, .little) + self.tracks.items[session.first_track].pregap;
        log.debug("Session Type: {X}, Last Session Start LBA: {X}", .{ session_type, session.start_fad });
        if (session.start_fad != self.tracks.items[session.first_track].fad)
            log.warn("Session start fad doesn't match first track: {X} != {X}", .{ session.start_fad, self.tracks.items[session.first_track].fad });
        if (version != .V2) std.debug.assert(try reader.discardShort(1) == 1);

        session.last_track = @intCast(self.tracks.items.len - 1);
        session.end_fad = self.tracks.items[session.last_track].get_end_fad();
        try self.sessions.append(allocator, session);
    }
    const total_tracks = try reader.takeInt(u16, .little);
    std.debug.assert(total_tracks == 0); // Marks the end of sessions.
    try track_header(reader);

    const total_number_of_sectors = try reader.takeInt(u32, .little);
    const volume_name_length = try reader.takeInt(u8, .little);
    var volume_name_buffer: [256]u8 = undefined;
    const volume_name = volume_name_buffer[0..volume_name_length];
    _ = try reader.readSliceAll(volume_name);
    log.debug("Sector count: {X}, Volume Name Length: {d}, Volume Name: {s}", .{ total_number_of_sectors, volume_name_length, volume_name });

    return self;
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    self.sessions.deinit(allocator);
    self.tracks.deinit(allocator);
    self._file.deinit();
}

pub fn get_first_data_track(self: *const @This()) ?Track {
    for (self.tracks.items) |track| {
        if (track.track_type == .Data)
            return track;
    }
    return null;
}

pub fn read_sector(self: *const @This(), fad: u32) []const u8 {
    const track = Track.get_corresponding_track(&self.tracks, fad);
    return track.read_sector(fad);
}

pub fn load_sectors(self: *const @This(), fad: u32, count: u32, dest: []u8) u32 {
    const track = Track.get_corresponding_track(&self.tracks, fad);
    return track.load_sectors(fad, count, dest);
}

pub fn load_sectors_raw(self: *const @This(), fad: u32, count: u32, dest: []u8) u32 {
    const track = Track.get_corresponding_track(&self.tracks, fad);
    return track.load_sectors_raw(fad, count, dest);
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

fn track_header(reader: *std.io.Reader) !void {
    const null_or_extra = try reader.takeInt(u32, .little);
    if (null_or_extra != 0)
        std.debug.assert(try reader.discardShort(8) == 8);
    const track_start_mark = try reader.takeInt(u160, .big);
    if (track_start_mark != 0x0000_0100_0000_FFFF_FFFF_0000_0100_0000_FFFF_FFFF) {
        log.err("  Invalid Track Start Mark: {X}", .{track_start_mark});
        return error.InvalidCDI;
    }
    std.debug.assert(try reader.discardShort(4) == 4);

    const filename_length = try reader.takeInt(u8, .little);
    var buffer: [256]u8 = undefined;
    const filename = buffer[0..filename_length];
    _ = try reader.readSliceAll(filename);
    log.debug("    Filename: {s}", .{filename});
    std.debug.assert(try reader.discardShort(1 + 10 + 4 + 4) == 1 + 10 + 4 + 4);

    const v4_mark = try reader.takeInt(u32, .little);
    const max_cd_length = mcl: {
        if (v4_mark != 0x80000000) {
            break :mcl v4_mark;
        } else {
            const max_cd_length = try reader.takeInt(u32, .little);
            const v4_mark_2 = try reader.takeInt(u32, .little);
            if (v4_mark_2 != 0x980000) {
                log.debug("  V4 Mark 2: {X}", .{v4_mark_2});
                return error.InvalidCDI;
            }
            break :mcl max_cd_length;
        }
    };
    if (max_cd_length != 0x514C8 and max_cd_length != 0x57E40 and max_cd_length != 0x61A80) {
        log.err("  Invalid max_cd_length: {X}", .{max_cd_length});
        return error.InvalidCDI;
    }
    log.debug("    Max CD Length: {X}", .{max_cd_length});
}
