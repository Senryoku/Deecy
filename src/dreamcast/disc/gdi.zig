const std = @import("std");
const log = std.log.scoped(.gdi);

const MemoryMappedFile = @import("../host/memory_mapped_file.zig");

const CD = @import("iso9660.zig");
const Track = @import("track.zig");
const Session = @import("session.zig");

const GDISectorOffset = 150;

tracks: std.ArrayList(Track) = .empty,

_files: std.ArrayList(MemoryMappedFile) = .empty,

pub fn init(allocator: std.mem.Allocator, filepath: []const u8) !@This() {
    var self: @This() = .{};

    const file = std.fs.cwd().openFile(filepath, .{}) catch |err| {
        log.err("Error opening '{s}': {t}", .{ filepath, err });
        return err;
    };
    defer file.close();
    const folder = std.fs.path.dirname(filepath) orelse ".";
    const data = try file.readToEndAlloc(allocator, 1024 * 1024);
    defer allocator.free(data);
    const end_line = if (std.mem.containsAtLeast(u8, data, 1, "\r\n")) "\r\n" else "\n";
    var lines = std.mem.splitSequence(u8, data, end_line);

    const track_count = try std.fmt.parseUnsigned(u32, lines.first(), 10);
    try self.tracks.resize(allocator, track_count);

    for (0..track_count) |i| {
        var vals = std.mem.splitSequence(u8, lines.next() orelse return error.EoF, " ");

        const num = try std.fmt.parseUnsigned(u32, vals.next() orelse return error.EoF, 10);
        if (num > track_count) return error.InvalidTrackNumber;
        var offset = try std.fmt.parseUnsigned(u32, vals.next() orelse return error.EoF, 10);
        const track_type_int = try std.fmt.parseUnsigned(u8, vals.next() orelse return error.EoF, 10);
        if (track_type_int != 0 and track_type_int != 4) return error.UnsupportedTrackType;
        const format = try std.fmt.parseUnsigned(u32, vals.next() orelse return error.EoF, 10);
        var filename = vals.next() orelse return error.EoF;
        // Handle quoted filenames
        if (std.mem.startsWith(u8, filename, "\"")) {
            var filename_len = filename.len;
            while (vals.next()) |next| {
                filename_len += 1 + next.len; // Space plus the next token.
                if (std.mem.endsWith(u8, next, "\"")) break;
            }
            filename = @as([*]const u8, @ptrCast(filename.ptr))[1 .. filename_len - 1]; // Remove quotes
        }
        const pregap = try std.fmt.parseUnsigned(u32, vals.next() orelse return error.EoF, 10);
        if (pregap != 0) return error.UnsupportedNonZeroPregap; // FIXME: Not handled.

        if (i >= 2) offset += GDISectorOffset;

        const track_file_path = try std.fs.path.join(allocator, &[_][]const u8{ folder, filename });
        defer allocator.free(track_file_path);
        try self._files.append(allocator, try MemoryMappedFile.init(allocator, track_file_path));

        self.tracks.items[num - 1] = .{
            .num = num,
            .fad = offset,
            .track_type = @enumFromInt(track_type_int),
            .format = format,
            .pregap = pregap,
            .data = try self._files.items[self._files.items.len - 1].create_full_view(),
        };
    }

    return self;
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    self.tracks.deinit(allocator);
    for (self._files.items) |*file|
        file.deinit();
    self._files.deinit(allocator);
}

pub fn get_first_data_track(self: *const @This()) ?Track {
    for (self.tracks.items[self.get_area_boundaries(.DoubleDensity)[0]..]) |track| {
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
        else => std.debug.panic("GDI: Invalid session number: {d}", .{session_number}),
    };
}

pub fn get_area_boundaries(self: *const @This(), area: Session.Area) [2]u32 {
    return switch (area) {
        .SingleDensity => .{ 0, 1 },
        .DoubleDensity => .{ 2, @intCast(self.tracks.items.len - 1) },
    };
}
