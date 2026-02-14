const std = @import("std");
const log = std.log.scoped(.cue);

const MemoryMappedFile = @import("../host/memory_mapped_file.zig");

const CD = @import("iso9660.zig");
const Track = @import("track.zig");
const Session = @import("session.zig");

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
    var line_it = std.mem.splitSequence(u8, data, end_line);
    var lines_list: std.ArrayList([]const u8) = .empty;
    defer lines_list.deinit(allocator);
    while (line_it.next()) |line|
        try lines_list.append(allocator, std.mem.trim(u8, line, " \t"));
    const lines = lines_list.items;

    var track_fad: usize = 0;

    var line_idx: u32 = 0;
    while (line_idx < lines.len) {
        if (std.mem.startsWith(u8, lines[line_idx], "REM")) {
            if (std.mem.endsWith(u8, lines[line_idx], "HIGH-DENSITY AREA")) {
                track_fad = 0xB05E;
            }
            line_idx += 1;
        } else if (std.mem.startsWith(u8, lines[line_idx], "FILE")) {
            var filename: []const u8 = undefined;
            var file_type: []const u8 = undefined;
            const rest = std.mem.trim(u8, lines[line_idx][4..], " \t");
            if (std.mem.startsWith(u8, rest, "\"")) {
                const end_quote_index = std.mem.indexOfScalar(u8, rest[1..], '"') orelse return error.InvalidFilename;
                filename = rest[1 .. end_quote_index + 1];
                file_type = std.mem.trim(u8, rest[end_quote_index + 2 ..], " \t");
            } else {
                var args = std.mem.tokenizeAny(u8, rest, " \t");
                filename = args.next() orelse return error.EoF;
                file_type = args.next() orelse return error.EoF;
            }
            log.debug("File: '{s}', {s}", .{ filename, file_type });
            line_idx += 1;
            while (line_idx < lines.len and std.mem.startsWith(u8, lines[line_idx], "TRACK")) {
                var args = std.mem.tokenizeAny(u8, lines[line_idx], " \t");
                _ = args.next();
                const num = try std.fmt.parseUnsigned(u32, args.next() orelse return error.EoF, 10);
                const track_type_str = args.next() orelse return error.EoF;
                line_idx += 1;
                log.debug("  Track: {d}, {s}", .{ num, track_type_str });

                const track_file_path = try std.fs.path.join(allocator, &[_][]const u8{ folder, filename });
                defer allocator.free(track_file_path);
                try self._files.append(allocator, try MemoryMappedFile.init(allocator, track_file_path));

                const track_type: Track.TrackType = if (std.mem.startsWith(u8, track_type_str, "MODE")) .Data else if (std.mem.startsWith(u8, track_type_str, "AUDIO")) .Audio else return error.UnsupportedTrackFormat;
                const format: u32 = if (std.mem.eql(u8, track_type_str, "MODE1/2352")) 2352 else if (std.mem.eql(u8, track_type_str, "AUDIO")) 2048 else return error.UnsupportedTrackFormat;
                const pregap = 0; // TODO

                if (self.tracks.items.len < num) try self.tracks.resize(allocator, num);
                self.tracks.items[num - 1] = .{
                    .num = num,
                    .fad = @intCast(track_fad),
                    .track_type = track_type,
                    .format = format,
                    .pregap = pregap,
                    .data = try self._files.items[self._files.items.len - 1].create_full_view(),
                };
                track_fad += self.tracks.items[num - 1].data.len / 2352;

                while (line_idx < lines.len and std.mem.startsWith(u8, lines[line_idx], "INDEX")) {
                    var index_args = std.mem.tokenizeAny(u8, lines[line_idx], " \t");
                    _ = index_args.next() orelse return error.EoF;
                    const index = try std.fmt.parseUnsigned(u32, index_args.next() orelse return error.EoF, 10);
                    const offset_str = index_args.next() orelse return error.EoF;
                    log.debug("    Index: {d}, {s}", .{ index, offset_str });
                    line_idx += 1;
                }
            }
        } else if (lines[line_idx].len > 0) {
            log.err("Unsupported command '{s}'", .{lines[line_idx]});
            return error.UnsupportedCommand;
        } else {
            line_idx += 1;
        }
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
