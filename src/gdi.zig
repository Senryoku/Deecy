const std = @import("std");
const builtin = @import("builtin");
const common = @import("common.zig");
const termcolor = @import("termcolor");

const gdi_log = std.log.scoped(.gdi);

const windows = @import("windows.zig");
const FileBacking = @import("file_backing.zig");

const CD = @import("iso9660.zig");
const Track = @import("track.zig");

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
                .offset = offset,
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

    pub fn get_corresponding_track(self: *const @This(), lda: u32) !*const Track {
        std.debug.assert(self.tracks.items.len > 0);
        var idx: u32 = 0;
        while (idx + 1 < self.tracks.items.len and self.tracks.items[idx + 1].offset <= lda) : (idx += 1) {}
        return &self.tracks.items[idx];
    }

    pub fn get_primary_volume_descriptor(self: *const @This()) *const CD.PVD {
        const offset = 0x10 * self.tracks.items[2].format + self.tracks.items[2].header_size(); // 16th sector + skip sector header
        return @ptrCast(@alignCast(self.tracks.items[2].data.ptr + offset));
    }
    pub fn load_file(self: *const @This(), filename: []const u8, dest: []u8) !u32 {
        const root_directory_entry = self.get_primary_volume_descriptor().*.root_directory_entry;
        const root_directory_length = root_directory_entry.length;
        const root_directory_lba = root_directory_entry.location + GDI_SECTOR_OFFSET;
        const root_track = try self.get_corresponding_track(root_directory_lba);
        const sector_start = (root_directory_lba - root_track.offset) * root_track.format;

        var curr_offset = sector_start + self.tracks.items[2].header_size(); // Skip header if any.
        // TODO: Handle directories, and not just root files.
        for (0..root_directory_length) |_| {
            const dir_record = root_track.get_directory_record(curr_offset);
            if (std.mem.eql(u8, dir_record.get_file_identifier(), filename))
                return self.load_bytes(dir_record.location + GDI_SECTOR_OFFSET, dir_record.data_length, dest);
            curr_offset += dir_record.length; // FIXME: Handle sector boundaries?
        }
        return error.NotFound;
    }

    // Bad wrapper around load_sectors. Don't use that in performance sensisive code :)
    pub fn load_bytes(self: *const @This(), lba: u32, length: u32, dest: []u8) u32 {
        const track = try self.get_corresponding_track(lba);
        var remaining = length;
        var curr_sector = lba;
        while (remaining > 0) {
            remaining -= track.load_sectors(curr_sector, 1, dest[length - remaining .. length]);
            curr_sector += 1;
        }
        return length;
    }

    pub fn load_sectors(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
        const track = try self.get_corresponding_track(lba);
        return track.load_sectors(lba, count, dest);
    }

    pub fn load_sectors_raw(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
        const track = try self.get_corresponding_track(lba);
        @memcpy(dest[0 .. 2352 * count], track.data[(lba - track.offset) * 2352 .. 2352 * ((lba - track.offset) + count)]);
        return 2352 * count;
    }
};
