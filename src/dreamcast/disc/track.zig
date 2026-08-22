pub const TrackType = enum(u8) { Audio = 0, Data = 4 };

pub const SectorHeader = packed struct(u128) {
    sync: u96,
    address: packed struct(u24) {
        minute: u8,
        second: u8,
        frame: u8,
    },
    mode: enum(u8) { Mode1 = 1, Mode2 = 2, _ },
};

pub const Mode2Subheader = packed struct(u32) {
    file_number: u8,
    channel_number: u8,
    submode: packed struct(u8) {
        end_of_record: bool,
        real_time: bool,
        trigger: bool,
        data: bool,
        audio: bool,
        video: bool,
        form: enum(u1) { Form1 = 0, Form2 = 1 },
        end_of_file: bool,
    },
    coding_info: u8,

    pub inline fn data_size(self: @This()) u32 {
        return switch (self.submode.form) {
            .Form1 => 2048,
            .Form2 => 2324,
        };
    }
};

num: u32,
fad: u32, // Start FAD
end_fad: u32,
track_type: TrackType,
format: u32, // Sector size
pregap: u32,
data: []const u8,

pub fn get_end_fad(self: *const @This()) u32 {
    return self.end_fad;
}

pub fn adr_ctrl_byte(self: *const @This()) u8 {
    const adr: u4 = 1;
    const control: u8 = if (self.track_type == .Data) 0b0100 else 0b0000;
    return (control << 4) | adr;
}

/// Return a view into the sector data for the given FAD.
pub fn read_sector(self: *const @This(), fad: u32) []const u8 {
    std.debug.assert(fad >= self.fad);
    const sector_start = (fad - self.fad) * self.format;
    var user_bytes_per_sector: u32 = 2048;
    var data_offset: u32 = switch (self.format) {
        2048, 2336 => 0,
        2352 => @sizeOf(SectorHeader),
        else => blk: {
            log.err("Unexpected sector format: {d}", .{self.format});
            break :blk @sizeOf(SectorHeader);
        },
    };
    const header = std.mem.bytesAsValue(SectorHeader, self.data[sector_start..]); // Only valid if format is 2352.
    if (self.format == 2336 or (self.format == 2352 and header.mode == .Mode2)) {
        const subheader = std.mem.bytesAsValue(Mode2Subheader, self.data[sector_start + data_offset ..]);
        user_bytes_per_sector = subheader.data_size();
        data_offset += 8; // Skip subheader (twice)
    }
    return self.data[sector_start + data_offset ..][0..user_bytes_per_sector];
}

pub fn load_sectors(self: *const @This(), fad: u32, requested_count: u32, dest: []u8) u32 {
    std.debug.assert(fad >= self.fad);
    var sector_start = (fad - self.fad) * self.format;
    if (sector_start >= self.data.len) {
        log.warn("FAD out of range (track offset: {d}, size: {d}, fad: {d})", .{ self.fad, self.data.len, fad });
        return 0;
    }

    const count = @min(requested_count, self.end_fad - fad);

    // Each sector only has raw data.
    if (self.track_type == .Audio or self.format == 2048) {
        const to_copy: u32 = @min(@min(dest.len, count * self.format), self.data[sector_start..].len);
        @memcpy(dest[0..to_copy], self.data[sector_start .. sector_start + to_copy]);
        return to_copy;
    } else if (self.format == 2336) {
        // Pretty much 2352, but without the sector header.
        // Mode 2 only.
        std.debug.assert(self.track_type == .Data);
        const data_offset = 8; // Skip subheader (twice)
        var copied: u32 = 0;
        for (0..count) |_| {
            if (sector_start + data_offset >= self.data.len or dest.len <= copied) return copied;
            const subheader = std.mem.bytesAsValue(Mode2Subheader, self.data[sector_start..]);
            const data_size: u32 = subheader.data_size();
            const chunk_size = @min(data_size, dest.len - copied, self.data[sector_start + data_offset ..].len);
            @memcpy(dest[copied .. copied + chunk_size], self.data[sector_start + data_offset ..][0..chunk_size]);
            copied += chunk_size;
            sector_start += self.format;
        }
        return copied;
    } else if (self.format == 2352) {
        var copied: u32 = 0;
        for (0..count) |_| {
            if (sector_start + 0x18 >= self.data.len) return copied;

            const header = std.mem.bytesAsValue(SectorHeader, self.data[sector_start..]);
            var data_offset: u32 = @sizeOf(SectorHeader);

            const data_size: u32 = switch (header.mode) {
                .Mode1 => 2048,
                .Mode2 => blk: {
                    const subheader = std.mem.bytesAsValue(Mode2Subheader, self.data[sector_start + @sizeOf(SectorHeader) ..]);
                    data_offset += 8; // Skip subheader (twice)
                    break :blk subheader.data_size();
                },
                else => blk: {
                    log.err("({d}) Invalid sector mode: {}", .{ fad + copied / self.format, header.mode });
                    break :blk 2048;
                },
            };

            if (dest.len <= copied) return copied;
            const chunk_size = @min(data_size, dest.len - copied);
            @memcpy(dest[copied .. copied + chunk_size], self.data[sector_start + data_offset .. sector_start + data_offset + chunk_size]);
            copied += chunk_size;
            sector_start += self.format;
        }
        return copied;
    } else std.debug.panic("Unsupported sector format: {d}", .{self.format});
}

pub fn load_sectors_raw(self: *const @This(), fad: u32, count: u32, dest: []u8) u32 {
    std.debug.assert(fad >= self.fad);
    if (fad + count > self.end_fad) log.warn("FAD out of range, reading [{d}-{d}) from track #{d} [{d}-{d})", .{ fad, fad + count, self.num, self.fad, self.end_fad });
    const capped_count = @min(count, self.end_fad - fad);
    @memcpy(dest[0 .. self.format * capped_count], self.data[(fad - self.fad) * self.format .. self.format * ((fad - self.fad) + capped_count)]);
    return self.format * capped_count;
}

pub fn get_corresponding_track(arr: *const std.ArrayList(@This()), fad: u32) *const @This() {
    std.debug.assert(arr.items.len > 0);
    var idx: u32 = 0;
    while (idx + 1 < arr.items.len and arr.items[idx + 1].fad <= fad) : (idx += 1) {}
    return &arr.items[idx];
}

const std = @import("std");
const log = std.log.scoped(.track);
