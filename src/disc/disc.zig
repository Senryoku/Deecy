const std = @import("std");

const GDI = @import("gdi.zig").GDI;
const CDI = @import("cdi.zig").CDI;

pub const CD = @import("iso9660.zig");
pub const Track = @import("track.zig");
pub const Session = @import("session.zig");

const Region = @import("../dreamcast.zig").Region;

pub const DiscFormat = enum(u4) {
    CDDA = 0,
    CDROM = 1,
    CDROM_XA = 2,
    CDI = 3,
    GDROM = 8,
};

pub const Disc = union(enum) {
    GDI: GDI,
    CDI: CDI,

    pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !Disc {
        if (std.mem.endsWith(u8, filepath, ".gdi")) {
            return Disc{ .GDI = try GDI.init(filepath, allocator) };
        } else if (std.mem.endsWith(u8, filepath, ".cdi")) {
            return Disc{ .CDI = try CDI.init(filepath, allocator) };
        } else return error.UnknownDiscFormat;
    }

    pub fn deinit(self: *@This()) void {
        switch (self.*) {
            inline else => |*d| d.deinit(),
        }
    }

    pub fn get_format(self: *const @This()) DiscFormat {
        switch (self.*) {
            .GDI => return .GDROM,
            .CDI => return .CDROM_XA,
        }
    }

    pub fn get_tracks(self: *const @This()) std.ArrayList(Track) {
        switch (self.*) {
            inline else => |d| return d.tracks,
        }
    }

    pub fn get_corresponding_track(self: *const @This(), fad: u32) !*const Track {
        std.debug.assert(self.get_tracks().items.len > 0);
        var idx: u32 = 0;
        while (idx + 1 < self.get_tracks().items.len and self.get_tracks().items[idx + 1].fad <= fad) : (idx += 1) {}
        return &self.get_tracks().items[idx];
    }

    pub fn lba_to_fad(lba: u32) u32 {
        return lba + 0x96;
    }

    pub fn load_file(self: *const @This(), filename: []const u8, dest: []u8) !u32 {
        const pvd = try self.get_primary_volume_descriptor();
        const root_directory_entry = pvd.root_directory_entry;
        const root_directory_length = root_directory_entry.length;
        const root_directory_fad = lba_to_fad(root_directory_entry.location);
        const root_track = try self.get_corresponding_track(root_directory_fad);
        const sector_start = (root_directory_fad - root_track.fad) * root_track.format;

        var curr_offset = sector_start + root_track.header_size(); // Skip header if any.
        // TODO: Handle directories, and not just root files.
        for (0..root_directory_length) |_| {
            const dir_record = root_track.get_directory_record(curr_offset);
            if (std.mem.eql(u8, dir_record.get_file_identifier(), filename))
                return self.load_bytes(lba_to_fad(dir_record.location), dir_record.data_length, dest);
            curr_offset += dir_record.length; // FIXME: Handle sector boundaries?
        }
        return error.NotFound;
    }

    // Bad wrapper around load_sectors. Don't use that in performance sensitive code :)
    pub fn load_bytes(self: *const @This(), fad: u32, length: u32, dest: []u8) u32 {
        const track = try self.get_corresponding_track(fad);
        var remaining = length;
        var curr_sector = fad;
        while (remaining > 0) {
            remaining -= track.load_sectors(curr_sector, 1, dest[length - remaining .. length]);
            curr_sector += 1;
        }
        return length;
    }

    pub fn load_sectors(self: *const @This(), fad: u32, count: u32, dest: []u8) u32 {
        const track = try self.get_corresponding_track(fad);
        return track.load_sectors(fad, count, dest);
    }

    pub fn load_sectors_raw(self: *const @This(), fad: u32, count: u32, dest: []u8) u32 {
        const track = try self.get_corresponding_track(fad);
        @memcpy(dest[0 .. track.format * count], track.data[(fad - track.fad) * track.format .. track.format * ((fad - track.fad) + count)]);
        return track.format * count;
    }

    pub fn get_session_count(self: *const @This()) u32 {
        switch (self.*) {
            inline else => |d| return d.get_session_count(),
        }
    }

    pub fn get_session(self: *const @This(), session_number: u32) Session {
        switch (self.*) {
            inline else => |d| return d.get_session(session_number),
        }
    }

    pub fn get_end_fad(self: *const @This()) u32 {
        return self.get_tracks().items[self.get_tracks().items.len - 1].get_end_fad();
    }

    // Returns the first track from the high density area if present, or just the first track otherwise.
    pub fn get_first_data_track(self: @This()) ?Track {
        const idx: u8 = switch (self) {
            .GDI => 2,
            else => 0,
        };
        return if (self.get_tracks().items.len > idx)
            self.get_tracks().items[idx]
        else
            null;
    }

    pub fn get_primary_volume_descriptor(self: *const @This()) !*const CD.PVD {
        if (self.get_first_data_track()) |t| {
            const offset = 0x10 * t.format + t.sector_data_offset(); // 16th sector + skip sector header
            const pvd: *const CD.PVD = @ptrCast(@alignCast(t.data.ptr + offset));
            if (pvd.type_code != .PrimaryVolumeDescriptor or !std.mem.eql(u8, &pvd.standard_identifier, "CD001") or pvd.version != 0x01)
                return error.InvalidPVD;
            return pvd;
        }
        return error.MissingDataTrack;
    }

    pub fn get_product_id(self: *const @This()) ?[]const u8 {
        if (self.get_first_data_track()) |t| return t.data[t.sector_data_offset()..][0x40..0x50];
        return null;
    }

    pub fn get_region(self: *const @This()) Region {
        if (self.get_first_data_track()) |t| {
            if (t.data[t.sector_data_offset() + 0x30] == 'J')
                return .Japan;
            if (t.data[t.sector_data_offset() + 0x31] == 'U')
                return .USA;
            if (t.data[t.sector_data_offset() + 0x32] == 'E')
                return .Europe;
        }
        return .Unknown;
    }

    pub fn get_product_name(self: *const @This()) ?[]const u8 {
        if (self.get_first_data_track()) |t| {
            const name = t.data[t.sector_data_offset()..][0x80..0x90];
            // Trim spaces
            var end = name.len - 1;
            while (end > 0 and name[end] == ' ') end -= 1;
            return name[0 .. end + 1];
        }
        return null;
    }

    pub fn get_area_boundaries(self: *const @This(), area: Session.Area) [2]u32 {
        switch (self.*) {
            inline else => |d| return d.get_area_boundaries(area),
        }
    }

    pub fn write_toc(self: *const @This(), dest: []u8, area: Session.Area) u32 {
        const tracks = self.get_area_boundaries(area);

        @memset(dest[0..396], 0xFF);

        for (self.get_tracks().items[tracks[0] .. tracks[1] + 1]) |track| {
            dest[4 * (track.num - 1) + 0] = track.adr_ctrl_byte();
            dest[4 * (track.num - 1) + 1] = (@truncate(track.fad >> 16));
            dest[4 * (track.num - 1) + 2] = (@truncate(track.fad >> 8));
            dest[4 * (track.num - 1) + 3] = (@truncate(track.fad >> 0));
        }

        const first_track = self.get_tracks().items[tracks[0]];
        const last_track = self.get_tracks().items[tracks[1]];

        @memcpy(dest[396 .. 396 + 2 * 4], &[_]u8{
            first_track.adr_ctrl_byte(), @intCast(first_track.num), 0x00, 0x00, // Start track info: [Control/ADR] [Start Track Number] [0  ] [0  ]
            last_track.adr_ctrl_byte(), @intCast(last_track.num), 0x00, 0x00, //   End track info:   [Control/ADR] [End Track Number  ] [0  ] [0  ]
        });

        const end_fad = last_track.get_end_fad();
        @memcpy(dest[404..408], &[_]u8{
            last_track.adr_ctrl_byte(), @truncate(end_fad >> 16), @truncate(end_fad >> 8), @truncate(end_fad), // Leadout info:     [Control/ADR] [FAD (MSB)]          [FAD] [FAD (LSB)]
        });

        return 408;
    }
};
