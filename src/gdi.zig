const std = @import("std");
const builtin = @import("builtin");
const common = @import("common.zig");
const termcolor = @import("termcolor");

const gdi_log = std.log.scoped(.gdi);

const windows = @import("windows.zig");

const Region = @import("dreamcast.zig").Region;

const DirectoryRecord = extern struct {
    length: u8 align(1),
    extended_length: u8 align(1),
    location: u32 align(1), // Location of extent (LBA) in both-endian format.
    location_big_endian: [4]u8 align(1),
    data_length: u32 align(1), // Data length (size of extent) in both-endian format.
    data_length_big_endian: [4]u8 align(1),
    recording_date_and_time: [7]u8 align(1),
    file_flags: u8 align(1),
    file_unit_size: u8 align(1), // File unit size for files recorded in interleaved mode, zero otherwise.
    interleave_gap_size: u8 align(1), // Interleave gap size for files recorded in interleaved mode, zero otherwise.
    volume_sequence_number: u16 align(1), // Volume sequence number - the volume that this extent is recorded on, in 16 bit both-endian format.
    volume_sequence_number_big_endian: [2]u8 align(1),
    file_identifier_length: u8 align(1),
    file_identifier: u8 align(1),

    pub fn get_file_identifier(self: *const @This()) []const u8 {
        return (@as([*]const u8, @ptrCast((&self.file_identifier)))[0..self.file_identifier_length]);
    }
};

test "DirectoryRecord" {
    try std.testing.expect(@offsetOf(DirectoryRecord, "length") == 0);
    try std.testing.expect(@offsetOf(DirectoryRecord, "extended_length") == 1);
    try std.testing.expect(@offsetOf(DirectoryRecord, "location") == 2);
    try std.testing.expect(@offsetOf(DirectoryRecord, "data_length") == 10);
    try std.testing.expect(@offsetOf(DirectoryRecord, "recording_date_and_time") == 18);
    try std.testing.expect(@offsetOf(DirectoryRecord, "file_flags") == 25);
    try std.testing.expect(@offsetOf(DirectoryRecord, "file_unit_size") == 26);
    try std.testing.expect(@offsetOf(DirectoryRecord, "interleave_gap_size") == 27);
    try std.testing.expect(@offsetOf(DirectoryRecord, "volume_sequence_number") == 28);
    try std.testing.expect(@offsetOf(DirectoryRecord, "file_identifier_length") == 32);
    try std.testing.expect(@offsetOf(DirectoryRecord, "file_identifier") == 33);
}

const PVD = extern struct {
    type_code: u8,
    standard_identifier: [5]u8,
    version: u8,
    _unused: u8,
    system_identifer: [32]u8,
    volume_identifer: [32]u8,
    _padding: [8]u8,
    volume_space_size: [8]u8,
    _unused0: [32]u8,
    volume_set_size: u16 align(1),
    volume_set_size_big_endian: [2]u8,
    volume_sequence_number: u16 align(1),
    volume_sequence_number_big_endian: [2]u8,
    logical_block_size_: u16 align(1),
    logical_block_size_big_endian: [2]u8,
    path_table_size: u32 align(1),
    path_table_size_big_endian: [4]u8,
    l_path_table_location: u32 align(1),
    optional_l_path_table_location: u32 align(1),
    m_path_table_location_big_endian: [4]u8,
    optional_m_path_table_location_big_endian: [4]u8,
    root_directory_entry: DirectoryRecord, // This is what we're here for.
    volume_set_identifier: [128]u8,
    publisher_identifier: [128]u8,
    data_preparer_identifier: [128]u8,
    application_identifier: [128]u8,
    copyright_file_identifier: [37]u8,
    abstract_file_identifier: [37]u8,
    bibliographic_file_identifier: [37]u8,
    volume_creation_date_and_time: [17]u8,
    volume_modification_date_and_time: [17]u8,
    volume_expiration_date_and_time: [17]u8,
    volume_effective_date_and_time: [17]u8,
    file_structure_version: u8,
    _unused2: u8,
    application_used: [512]u8,
    _reserved: [653]u8,
};

test "PVD" {
    try std.testing.expect(@offsetOf(PVD, "root_directory_entry") == 156);
}

const Track = struct {
    num: u32,
    offset: u32, // Start LBA
    track_type: u8,
    format: u32, // Sector size
    pregap: u32,
    data: []align(std.mem.page_size) const u8,

    platform_specific: if (builtin.os.tag == .windows) struct {
        mapping_handle: *anyopaque,
        file_handle: *anyopaque,
    } else struct {
        file: std.fs.File,
    },

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        _ = allocator;
        if (builtin.os.tag != .windows) {
            std.posix.munmap(self.data);
            self.platform_specific.file.close();
        } else {
            // TODO:
            _ = windows.UnmapViewOfFile(self.data.ptr);
            std.os.windows.CloseHandle(self.platform_specific.mapping_handle);
            std.os.windows.CloseHandle(self.platform_specific.file_handle);
        }
    }

    pub fn get_directory_record(self: *const @This(), offset: usize) *const DirectoryRecord {
        return @ptrCast(@alignCast(self.data.ptr + offset));
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
        std.debug.assert(lba >= self.offset);
        var sector_start = (lba - self.offset) * self.format;
        if (sector_start >= self.data.len) {
            gdi_log.warn(termcolor.yellow("lba out of range (track offset: {d}, lba: {d})"), .{ self.offset, lba });
            return 0;
        }

        // Each sector only has raw data.
        if (self.track_type == 0 or self.format == 2048) {
            const to_copy: u32 = @min(@min(dest.len, count * 2048), self.data[sector_start..].len);
            @memcpy(dest[0..to_copy], self.data[sector_start .. sector_start + to_copy]);
            return to_copy;
        } else if (self.format == 2352) {
            var copied: u32 = 0;
            for (0..count) |_| {
                if (sector_start >= self.data.len or self.data[sector_start..].len < 0x10)
                    return copied;

                const header = self.data[sector_start .. sector_start + self.header_size()];
                // Mode 1 (2048 bytes plus error correction) or Mode 2 (2336 bytes)
                if (header[0x0F] != 1 and header[0x0F] != 2) {
                    gdi_log.warn(termcolor.yellow("Invalid sector mode: {X:0>2}"), .{header[0x0F]});
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
            @panic("Unimplemented");
        }
    }
};

pub const SectorHeader = extern struct {
    sync_filed: [12]u8,
    address: [3]u8,
    mode: u8,
};

const GDI_SECTOR_OFFSET = 150; // FIXME: Still unsure about this.

pub const GDI = struct {
    tracks: std.ArrayList(Track),

    _allocator: std.mem.Allocator,

    pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !GDI {
        var self: GDI = .{
            .tracks = std.ArrayList(Track).init(allocator),
            ._allocator = allocator,
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

            // TODO: Use MMAP on non-windows platforms
            if (builtin.os.tag != .windows) {
                const track_file_path = try std.fs.path.join(self._allocator, &[_][]const u8{ folder, filename });
                defer self._allocator.free(track_file_path);
                const track_file = std.fs.cwd().openFile(track_file_path, .{}) catch {
                    std.debug.print("Track File not found: {s}\n", .{track_file_path});
                    return error.TrackFileNotFound;
                };

                const track_data = try std.posix.mmap(null, (try track_file.stat()).size, std.posix.PROT.READ, .{ .TYPE = .SHARED }, track_file.handle, 0);

                self.tracks.items[num - 1] = (.{
                    .num = num,
                    .offset = offset,
                    .track_type = track_type,
                    .format = format,
                    .pregap = pregap,
                    .data = track_data,
                    .platform_specific = .{ .file = track_file },
                });
            } else {
                const track_file_path = try std.fs.path.joinZ(self._allocator, &[_][]const u8{ folder, filename });
                defer self._allocator.free(track_file_path);
                var track_file_abs_path_buffer: [std.fs.max_path_bytes + 1]u8 = .{0} ** (std.fs.max_path_bytes + 1);
                const track_file_abs_path = try std.fs.cwd().realpathZ(track_file_path, &track_file_abs_path_buffer);
                const file_path_w = try std.os.windows.cStrToPrefixedFileW(null, @as([*:0]u8, @ptrCast(track_file_abs_path.ptr)));

                const file_handle = try std.os.windows.OpenFile(file_path_w.span(), .{
                    .access_mask = std.os.windows.GENERIC_READ | std.os.windows.SYNCHRONIZE,
                    .creation = std.os.windows.FILE_OPEN,
                });
                errdefer std.os.windows.CloseHandle(file_handle);

                const mapping_handle = windows.CreateFileMappingA(file_handle, null, std.os.windows.PAGE_READONLY, 0, 0, null);
                if (mapping_handle == null) return error.FileMapError;
                errdefer std.os.windows.CloseHandle(mapping_handle.?);

                const ptr = windows.MapViewOfFile(mapping_handle.?, std.os.windows.SECTION_MAP_READ, 0, 0, 0);
                if (ptr == null) return error.MapViewOfFileError;
                errdefer _ = windows.UnmapViewOfFile(ptr.?);

                var info: std.os.windows.MEMORY_BASIC_INFORMATION = undefined;
                _ = try std.os.windows.VirtualQuery(ptr, &info, @sizeOf(std.os.windows.MEMORY_BASIC_INFORMATION));

                self.tracks.items[num - 1] = (.{
                    .num = num,
                    .offset = offset,
                    .track_type = track_type,
                    .format = format,
                    .pregap = pregap,
                    .data = @as([*]align(std.mem.page_size) const u8, @alignCast(@ptrCast(ptr)))[0..info.RegionSize],
                    .platform_specific = .{
                        .file_handle = file_handle,
                        .mapping_handle = mapping_handle.?,
                    },
                });
            }
        }

        return self;
    }

    pub fn deinit(self: *@This()) void {
        for (self.tracks.items) |*track| {
            track.deinit(self._allocator);
        }
        self.tracks.deinit();
    }

    pub fn get_region(self: *const @This()) Region {
        if (self.tracks.items.len >= 3) {
            if (self.tracks.items[2].data[0x40] == 'J')
                return .Japan;
            if (self.tracks.items[2].data[0x41] == 'U')
                return .USA;
            if (self.tracks.items[2].data[0x42] == 'E')
                return .Europe;
        }
        return .Unknown;
    }

    pub fn get_product_id(self: *const @This()) ?[]const u8 {
        if (self.tracks.items.len < 3) return null;
        return self.tracks.items[2].data[0x50..0x60];
    }

    pub fn get_product_name(self: *const @This()) ?[]const u8 {
        if (self.tracks.items.len < 3) return null;
        const name = self.tracks.items[2].data[0x90..0x100];
        // Trim spaces
        var end = name.len - 1;
        while (end > 0 and name[end] == ' ') end -= 1;
        return name[0 .. end + 1];
    }

    pub fn get_primary_volume_descriptor(self: *const @This()) *const PVD {
        const offset = 0x10 * self.tracks.items[2].format + self.tracks.items[2].header_size(); // 16th sector + skip sector header
        return @ptrCast(@alignCast(self.tracks.items[2].data.ptr + offset));
    }

    pub fn get_corresponding_track(self: *const @This(), lda: u32) !*const Track {
        std.debug.assert(self.tracks.items.len > 0);
        var idx: u32 = 0;
        while (idx + 1 < self.tracks.items.len and self.tracks.items[idx + 1].offset <= lda) : (idx += 1) {}
        return &self.tracks.items[idx];
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
