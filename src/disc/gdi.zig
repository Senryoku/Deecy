const std = @import("std");
const builtin = @import("builtin");
const common = @import("../common.zig");
const termcolor = @import("termcolor");

const gdi_log = std.log.scoped(.gdi);

const windows = @import("../windows.zig");
const FileBacking = @import("../file_backing.zig");

const CD = @import("iso9660.zig");
const Track = @import("track.zig");
const Session = @import("session.zig");

pub const SectorHeader = extern struct {
    sync_filed: [12]u8,
    address: [3]u8,
    mode: u8,
};

const GDI_SECTOR_OFFSET = 150; // FIXME: Still unsure about this.

pub const GDI = struct {
    tracks: std.ArrayList(Track),

    _allocator: std.mem.Allocator,
    _files: std.ArrayList(FileBacking),

    pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !GDI {
        var self: GDI = .{
            .tracks = std.ArrayList(Track).init(allocator),
            ._allocator = allocator,
            ._files = std.ArrayList(FileBacking).init(allocator),
        };

        const file = std.fs.cwd().openFile(filepath, .{}) catch {
            std.debug.print("File not found: {s}\n", .{filepath});
            return error.GDIFileNotFound;
        };
        defer file.close();
        const folder = std.fs.path.dirname(filepath) orelse ".";
        const data = try file.readToEndAlloc(self._allocator, 1024 * 1024 * 1024);
        defer self._allocator.free(data);
        const end_line = if (std.mem.containsAtLeast(u8, data, 1, "\r\n")) "\r\n" else "\n";
        var lines = std.mem.splitSequence(u8, data, end_line);

        const first_line = lines.next().?;
        const track_count = try std.fmt.parseUnsigned(u32, first_line, 10);
        try self.tracks.resize(track_count);

        for (0..track_count) |i| {
            var vals = std.mem.splitSequence(u8, lines.next().?, " ");

            const num = try std.fmt.parseUnsigned(u32, vals.next().?, 10);
            var offset = (try std.fmt.parseUnsigned(u32, vals.next().?, 10));
            const track_type = try std.fmt.parseUnsigned(u8, vals.next().?, 10);
            const format = try std.fmt.parseUnsigned(u32, vals.next().?, 10);
            const filename = vals.next().?;
            const pregap = try std.fmt.parseUnsigned(u32, vals.next().?, 10);

            if (i >= 2)
                offset += GDI_SECTOR_OFFSET;

            std.debug.assert(pregap == 0); // FIXME: Not handled.

            const track_file_path = try std.fs.path.join(self._allocator, &[_][]const u8{ folder, filename });
            defer self._allocator.free(track_file_path);
            var track_file = try FileBacking.init(track_file_path, allocator);
            errdefer track_file.deinit();
            try self._files.append(track_file);

            self.tracks.items[num - 1] = (.{
                .num = num,
                .fad = offset,
                .track_type = track_type,
                .format = format,
                .pregap = pregap,
                .data = try track_file.create_full_view(),
            });
        }

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.tracks.deinit();
        for (self._files.items) |*file| {
            file.deinit();
        }
        self._files.deinit();
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
            else => @panic("GDI: Invalid session number"),
        };
    }

    pub fn get_area_boundaries(self: *const @This(), area: Session.Area) [2]u32 {
        return switch (area) {
            .SingleDensity => [2]u32{ 0, 1 },
            .DoubleDensity => [2]u32{ 2, @intCast(self.tracks.items.len - 1) },
        };
    }
};
