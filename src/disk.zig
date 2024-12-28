const std = @import("std");

const GDI = @import("gdi.zig").GDI;
const CDI = @import("cdi.zig").CDI;

const CD = @import("iso9660.zig");
const Track = @import("track.zig");

const Region = @import("dreamcast.zig").Region;

pub const Disk = union(enum) {
    GDI: GDI,
    CDI: CDI,

    pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !Disk {
        if (std.mem.endsWith(u8, filepath, ".gdi")) {
            return Disk{ .GDI = try GDI.init(filepath, allocator) };
        } else if (std.mem.endsWith(u8, filepath, ".cdi")) {
            return Disk{ .CDI = try CDI.init(filepath, allocator) };
        } else return error.UnknownDiskFormat;
    }

    pub fn deinit(self: *@This()) void {
        switch (self.*) {
            inline else => |*d| d.deinit(),
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

    pub fn get_product_id(self: *const @This()) ?[]const u8 {
        if (self.get_tracks().items.len < 3) return null;
        return self.get_tracks().items[2].data[0x50..0x60];
    }

    pub fn get_region(self: *const @This()) Region {
        if (self.get_tracks().items.len >= 3) {
            if (self.get_tracks().items[2].data[0x40] == 'J')
                return .Japan;
            if (self.get_tracks().items[2].data[0x41] == 'U')
                return .USA;
            if (self.get_tracks().items[2].data[0x42] == 'E')
                return .Europe;
        }
        return .Unknown;
    }

    pub fn get_product_name(self: *const @This()) ?[]const u8 {
        if (self.get_tracks().items.len < 3) return null;
        const name = self.get_tracks().items[2].data[0x90..0x100];
        // Trim spaces
        var end = name.len - 1;
        while (end > 0 and name[end] == ' ') end -= 1;
        return name[0 .. end + 1];
    }
};
