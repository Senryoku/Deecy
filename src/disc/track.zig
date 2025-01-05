const std = @import("std");
const termcolor = @import("termcolor");

const CD = @import("iso9660.zig");

const log = std.log.scoped(.track);

num: u32,
fad: u32, // Start FAD
track_type: u8,
format: u32, // Sector size
pregap: u32,
data: []const u8,

pub fn get_directory_record(self: *const @This(), offset: usize) *const CD.DirectoryRecord {
    return @ptrCast(@alignCast(self.data.ptr + offset));
}

pub fn get_end_fad(self: *const @This()) u32 {
    return @intCast(self.fad + self.data.len / self.format);
}

pub fn header_size(self: *const @This()) u32 {
    return if (self.format == 2352) 0x10 else 0;
}

pub fn adr_ctrl_byte(self: *const @This()) u8 {
    const adr: u4 = 1;
    const control: u8 = if (self.track_type == 4) 0b0100 else 0b0000;
    return (control << 4) | adr;
}

pub fn load_sectors(self: *const @This(), lba: u32, count: u32, dest: []u8) u32 {
    std.debug.assert(lba >= self.fad);
    var sector_start = (lba - self.fad) * self.format;
    if (sector_start >= self.data.len) {
        log.warn(termcolor.yellow("lba out of range (track offset: {d}, size: {d}, lba: {d})"), .{ self.fad, self.data.len, lba });
        return 0;
    }

    // Each sector only has raw data.
    if (self.track_type == 0 or self.format == 2048) {
        const to_copy: u32 = @min(@min(dest.len, count * 2048), self.data[sector_start..].len);
        @memcpy(dest[0..to_copy], self.data[sector_start .. sector_start + to_copy]);
        return to_copy;
    } else if (self.format == 2336) {
        var copied: u32 = 0;
        for (0..count) |_| {
            if (sector_start >= self.data.len or self.data[sector_start..].len < 0x10)
                return copied;
            if (dest.len <= copied) return copied;
            const chunk_size = @min(2336, dest.len - copied);
            @memcpy(dest[copied .. copied + chunk_size], self.data[sector_start .. sector_start + chunk_size]);
            copied += chunk_size;
            sector_start += self.format;
        }
        return copied;
    } else if (self.format == 2352) {
        var copied: u32 = 0;
        for (0..count) |_| {
            if (sector_start >= self.data.len or self.data[sector_start..].len < 0x10)
                return copied;

            const header = self.data[sector_start .. sector_start + self.header_size()];
            // Mode 1 (2048 bytes plus error correction) or Mode 2 (2336 bytes)
            if (header[0x0F] != 1 and header[0x0F] != 2) {
                log.warn(termcolor.yellow("Invalid sector mode: {X:0>2}"), .{header[0x0F]});
                return copied;
            }
            const data_size: u32 = if (header[0x0F] == 1) 2048 else 2336;

            if (dest.len <= copied) return copied;
            const chunk_size = @min(data_size, dest.len - copied);
            @memcpy(dest[copied .. copied + chunk_size], self.data[sector_start + self.header_size() .. sector_start + self.header_size() + chunk_size]);
            copied += chunk_size;
            sector_start += self.format;
        }
        return copied;
    } else {
        log.err(termcolor.red("Unsupported sector format: {d}"), .{self.format});
        @panic("Unimplemented");
    }
}
