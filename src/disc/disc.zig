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

    pub fn load_sectors(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
        switch (self.*) {
            inline else => |d| return d.load_sectors(lba, count, dest),
        }
    }

    pub fn load_sectors_raw(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
        switch (self.*) {
            inline else => |d| return d.load_sectors_raw(lba, count, dest),
        }
    }

    pub fn load_bytes(self: *const @This(), lba: u32, length: u32, dest: []u8) u32 {
        switch (self.*) {
            inline else => |d| return d.load_bytes(lba, length, dest),
        }
    }

    pub fn load_file(self: *const @This(), filename: []const u8, dest: []u8) !u32 {
        switch (self.*) {
            inline else => |d| return d.load_file(filename, dest),
        }
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

    pub fn get_product_id(self: *const @This()) ?[]const u8 {
        if (self.get_first_data_track()) |t| return t.data[t.header_size()..][0x40..0x50];
        return null;
    }

    pub fn get_region(self: *const @This()) Region {
        if (self.get_first_data_track()) |t| {
            if (t.data[t.header_size() + 0x30] == 'J')
                return .Japan;
            if (t.data[t.header_size() + 0x31] == 'U')
                return .USA;
            if (t.data[t.header_size() + 0x32] == 'E')
                return .Europe;
        }
        return .Unknown;
    }

    pub fn get_product_name(self: *const @This()) ?[]const u8 {
        if (self.get_first_data_track()) |t| {
            const name = t.data[t.header_size()..][0x80..0x90];
            // Trim spaces
            var end = name.len - 1;
            while (end > 0 and name[end] == ' ') end -= 1;
            return name[0 .. end + 1];
        }
        return null;
    }

    pub fn write_toc(self: *const @This(), dest: []u8, area: Session.Area) u32 {
        switch (self.*) {
            inline else => |d| return d.write_toc(dest, area),
        }
    }
};
