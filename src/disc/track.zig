const std = @import("std");
const termcolor = @import("termcolor");

const log = std.log.scoped(.track);

pub const TrackType = enum(u8) { Audio = 0, Data = 4 };

num: u32,
fad: u32, // Start FAD
track_type: TrackType,
format: u32, // Sector size
pregap: u32,
data: []const u8,

pub fn get_end_fad(self: *const @This()) u32 {
    return @intCast(self.fad + self.data.len / self.format);
}

pub fn header_size(self: *const @This()) u32 {
    return if (self.format == 2352) 0x10 else 0;
}
pub fn subheader_size(self: *const @This()) u32 {
    return if (self.format == 2336) 8 else 0;
}
pub fn sector_data_offset(self: *const @This()) u32 {
    return self.header_size() + self.subheader_size();
}

pub fn adr_ctrl_byte(self: *const @This()) u8 {
    const adr: u4 = 1;
    const control: u8 = if (self.track_type == .Data) 0b0100 else 0b0000;
    return (control << 4) | adr;
}

pub fn read_sector(self: *const @This(), fad: u32) []const u8 {
    std.debug.assert(fad >= self.fad);
    const user_bytes_per_sector = 2048; // FIXME
    return self.data[(fad - self.fad) * self.format ..][self.sector_data_offset()..user_bytes_per_sector];
}

pub fn load_sectors(self: *const @This(), fad: u32, count: u32, dest: []u8) u32 {
    std.debug.assert(fad >= self.fad);
    var sector_start = (fad - self.fad) * self.format;
    if (sector_start >= self.data.len) {
        log.warn(termcolor.yellow("fad out of range (track offset: {d}, size: {d}, fad: {d})"), .{ self.fad, self.data.len, fad });
        return 0;
    }

    // Each sector only has raw data.
    if (self.track_type == .Audio or self.format == 2048) {
        const to_copy: u32 = @min(@min(dest.len, count * 2048), self.data[sector_start..].len);
        @memcpy(dest[0..to_copy], self.data[sector_start .. sector_start + to_copy]);
        return to_copy;
    } else if (self.format == 2336) {
        // Pretty much 2352, but without the header.
        // Mode 2, Form 1 (Data)
        std.debug.assert(self.track_type == .Data);
        const data_size: u32 = 2048;
        var copied: u32 = 0;
        for (0..count) |_| {
            if (sector_start >= self.data.len or dest.len <= copied)
                return copied;
            const chunk_size = @min(data_size, dest.len - copied, self.data[sector_start + self.subheader_size() ..].len);
            @memcpy(dest[copied .. copied + chunk_size], self.data[sector_start + self.subheader_size() .. sector_start + self.subheader_size() + chunk_size]);
            copied += chunk_size;
            sector_start += self.format;
        }
        return copied;
    } else if (self.format == 2352) {
        var copied: u32 = 0;
        const first_sector_header = self.data[sector_start .. sector_start + self.header_size()];
        // Mode 1 (2048 bytes plus error correction) or Mode 2 (2336 bytes)
        const data_size: u32 = if (first_sector_header[0x0F] == 1) 2048 else 2336; // FIXME/TODO: Depending on the request, we might want to skip the subheader and copy only 2324 bytes
        if (first_sector_header[0x0F] != 1 and first_sector_header[0x0F] != 2) {
            log.err(termcolor.red("({d}) Invalid sector mode: {X:0>2}"), .{ fad, first_sector_header[0x0F] });
            @panic("Invalid sector mode");
        }

        for (0..count) |_| {
            if (sector_start >= self.data.len or self.data[sector_start..].len < 0x10)
                return copied;

            const header = self.data[sector_start .. sector_start + self.header_size()]; // A track shouldn't have sectors using different modes.
            std.debug.assert(header[0x0F] == first_sector_header[0x0F]);

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

pub fn load_sectors_raw(self: *const @This(), fad: u32, count: u32, dest: []u8) u32 {
    std.debug.assert(fad >= self.fad);
    @memcpy(dest[0 .. self.format * count], self.data[(fad - self.fad) * self.format .. self.format * ((fad - self.fad) + count)]);
    return self.format * count;
}

pub fn get_corresponding_track(arr: *const std.ArrayList(@This()), fad: u32) *const @This() {
    std.debug.assert(arr.items.len > 0);
    var idx: u32 = 0;
    while (idx + 1 < arr.items.len and arr.items[idx + 1].fad <= fad) : (idx += 1) {}
    return &arr.items[idx];
}
