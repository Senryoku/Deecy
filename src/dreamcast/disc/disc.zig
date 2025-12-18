const std = @import("std");

const GDI = @import("gdi.zig");
const CDI = @import("cdi.zig");
const CHD = @import("chd.zig");

pub const CD = @import("iso9660.zig");
pub const Track = @import("track.zig");
pub const Session = @import("session.zig");

const Region = @import("../dreamcast.zig").Region;
const IPBIN = @import("ipbin.zig");

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
    CHD: CHD,

    pub fn init(allocator: std.mem.Allocator, filepath: []const u8) !Disc {
        if (std.mem.endsWith(u8, filepath, ".gdi")) {
            return Disc{ .GDI = try .init(allocator, filepath) };
        } else if (std.mem.endsWith(u8, filepath, ".cdi")) {
            return Disc{ .CDI = try .init(allocator, filepath) };
        } else if (std.mem.endsWith(u8, filepath, ".chd")) {
            return Disc{ .CHD = try .init(allocator, filepath) };
        } else return error.UnknownDiscFormat;
    }

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        switch (self.*) {
            inline else => |*d| d.deinit(allocator),
        }
    }

    pub fn get_format(self: *const @This()) DiscFormat {
        switch (self.*) {
            .GDI => return .GDROM,
            .CHD => return .GDROM,
            .CDI => return .CDROM_XA,
        }
    }

    pub inline fn get_tracks(self: *const @This()) std.ArrayList(Track) {
        switch (self.*) {
            inline else => |d| return d.tracks,
        }
    }

    pub fn lba_to_fad(lba: u32) u32 {
        return lba + 0x96;
    }

    pub fn load_file(self: *@This(), filename: []const u8, dest: []u8) !u32 {
        const pvd = try self.get_primary_volume_descriptor();
        std.debug.assert(pvd.root_directory_entry.length <= 2048);
        const root_directory_fad = lba_to_fad(pvd.root_directory_entry.location);

        const sector = try self.read_sector(root_directory_fad);

        var curr_offset: u32 = 0;
        // TODO: Handle directories, and not just root files.
        for (0..pvd.root_directory_entry.length) |_| {
            const dir_record: *const CD.DirectoryRecord = @ptrCast(sector[curr_offset..].ptr);
            if (std.mem.eql(u8, dir_record.get_file_identifier(), filename))
                return try self.load_bytes(lba_to_fad(dir_record.location), dir_record.data_length, dest);
            curr_offset += dir_record.length; // FIXME: Handle sector boundaries?
        }
        return error.NotFound;
    }

    /// Bad wrapper around load_sectors. Don't use that in performance sensitive code :)
    pub fn load_bytes(self: *@This(), fad: u32, length: u32, dest: []u8) !u32 {
        var remaining = length;
        var curr_sector = fad;
        while (remaining > 0) {
            remaining -= self.load_sectors(curr_sector, 1, dest[length - remaining .. length]);
            curr_sector += 1;
        }
        return length;
    }

    /// Returns a view into a single sector
    pub fn read_sector(self: *@This(), fad: u32) ![]const u8 {
        switch (self.*) {
            inline else => |*d| return d.read_sector(fad),
        }
    }

    pub fn load_sectors(self: *@This(), fad: u32, count: u32, dest: []u8) u32 {
        switch (self.*) {
            inline else => |*d| return d.load_sectors(fad, count, dest),
        }
    }

    pub fn load_sectors_raw(self: *@This(), fad: u32, count: u32, dest: []u8) u32 {
        switch (self.*) {
            inline else => |*d| return d.load_sectors_raw(fad, count, dest),
        }
    }

    pub fn get_session_count(self: *const @This()) u32 {
        switch (self.*) {
            inline else => |*d| return d.get_session_count(),
        }
    }

    pub fn get_session(self: *const @This(), session_number: u32) Session {
        switch (self.*) {
            inline else => |*d| return d.get_session(session_number),
        }
    }

    pub fn get_end_fad(self: *const @This()) u32 {
        return self.get_tracks().items[self.get_tracks().items.len - 1].get_end_fad();
    }

    pub fn get_first_data_track(self: *const @This()) ?Track {
        switch (self.*) {
            inline else => |d| return d.get_first_data_track(),
        }
    }

    pub fn get_primary_volume_descriptor(self: *@This()) !*const CD.PVD {
        if (self.get_first_data_track()) |t| {
            const sector = try self.read_sector(t.fad + 0x10);
            const pvd: *const CD.PVD = @ptrCast(@alignCast(sector.ptr));
            if (!pvd.is_valid())
                return error.InvalidPVD;
            return pvd;
        }
        return error.MissingDataTrack;
    }

    pub fn get_ip_bin_header(self: *@This()) ?IPBIN {
        if (self.get_first_data_track()) |t| {
            const sector = self.read_sector(t.fad) catch return null;
            return .{ .data = sector[0..0x100] };
        }
        return null;
    }

    pub fn get_product_id(self: *@This()) ?[]const u8 {
        if (self.get_ip_bin_header()) |ip_bin|
            return ip_bin.product_id();
        return null;
    }

    pub fn get_region(self: *@This()) Region {
        if (self.get_ip_bin_header()) |ip_bin| {
            if (ip_bin.japan_region())
                return .Japan;
            if (ip_bin.usa_region())
                return .USA;
            if (ip_bin.europe_region())
                return .Europe;
        }
        return .Unknown;
    }

    pub fn get_product_name(self: *@This()) ?[]const u8 {
        if (self.get_ip_bin_header()) |ip_bin|
            return ip_bin.product_name();
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
