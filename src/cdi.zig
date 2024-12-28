const std = @import("std");

const Track = @import("track.zig");

pub const CDI = struct {
    tracks: std.ArrayList(Track),
    _allocator: std.mem.Allocator,

    pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !CDI {
        const self: @This() = .{
            .tracks = std.ArrayList(Track).init(allocator),
            ._allocator = allocator,
        };

        const file = std.fs.cwd().openFile(filepath, .{}) catch {
            std.debug.print("File not found: {s}\n", .{filepath});
            return error.GDIFileNotFound;
        };
        defer file.close();

        return self;
    }

    pub fn deinit(self: *@This()) void {
        for (self.tracks.items) |*track| {
            track.deinit(self._allocator);
        }
        self.tracks.deinit();
    }

    pub fn load_sectors(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
        _ = self;
        _ = lba;
        _ = count;
        _ = dest;
        return 0;
    }

    pub fn load_sectors_raw(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
        _ = self;
        _ = lba;
        _ = count;
        _ = dest;
        return 0;
    }

    pub fn load_bytes(self: *const @This(), lba: u32, length: u32, dest: []u8) u32 {
        _ = self;
        _ = lba;
        _ = length;
        _ = dest;
        return 0;
    }

    pub fn load_file(self: *const @This(), filename: []const u8, dest: []u8) !u32 {
        _ = self;
        _ = filename;
        _ = dest;
        return error.NotImplemented;
    }
};
