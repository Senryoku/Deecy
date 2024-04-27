const std = @import("std");
const common = @import("common.zig");

const windows = @import("windows.zig");

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
    data: []u8,
    _mapping_handle: ?*anyopaque = undefined,
    _file_handle: ?*anyopaque = undefined,

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        if (@import("builtin").os.tag != .windows) {
            allocator.free(self.data);
        } else {
            // TODO:
            _ = windows.UnmapViewOfFile(self.data.ptr);
            std.os.windows.CloseHandle(self._mapping_handle.?);
            std.os.windows.CloseHandle(self._file_handle.?);
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
        var sector_start = (lba - self.offset) * self.format;
        if (self.format == 2048) {
            var copied: u32 = 0;
            for (0..count) |_| {
                const chunk_size = @min(self.format, dest.len - copied);
                @memcpy(dest[copied .. copied + chunk_size], self.data[sector_start .. sector_start + chunk_size]);
                copied += chunk_size;
                sector_start += chunk_size;
            }
            return copied;
        } else if (self.format == 2352) {
            var copied: u32 = 0;
            for (0..count) |_| {
                const header = self.data[sector_start .. sector_start + self.header_size()];
                std.debug.assert(header[0x0F] == 1 or header[0x0F] == 2); // Mode 1 (2048 bytes plus error correction) or Mode 2 (2336 bytes)
                const data_size: u32 = if (header[0x0F] == 1) 2048 else 2336;

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
    tracks: std.ArrayList(Track) = undefined,

    _allocator: std.mem.Allocator = undefined,

    pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !GDI {
        var self: GDI = .{};

        self._allocator = allocator;

        self.tracks = std.ArrayList(Track).init(self._allocator);

        const file = std.fs.cwd().openFile(filepath, .{}) catch {
            std.debug.print("File not found: {s}\n", .{filepath});
            return error.GDIFileNotFound;
        };
        defer file.close();
        const folder = std.fs.path.dirname(filepath) orelse ".";
        const data = try file.readToEndAlloc(self._allocator, 1024 * 1024 * 1024);
        defer self._allocator.free(data);
        const end_line = if (std.mem.containsAtLeast(u8, data, 1, "\r\n")) "\r\n" else "\n";
        var lines = std.mem.split(u8, data, end_line);

        const first_line = lines.next().?;
        const track_count = try std.fmt.parseUnsigned(u32, first_line, 10);
        try self.tracks.resize(track_count);

        for (0..track_count) |_| {
            var vals = std.mem.split(u8, lines.next().?, " ");

            const num = try std.fmt.parseUnsigned(u32, vals.next().?, 10);
            const offset = try std.fmt.parseUnsigned(u32, vals.next().?, 10) + GDI_SECTOR_OFFSET;
            const track_type = try std.fmt.parseUnsigned(u8, vals.next().?, 10);
            const format = try std.fmt.parseUnsigned(u32, vals.next().?, 10);
            const filename = vals.next().?;
            const pregap = try std.fmt.parseUnsigned(u32, vals.next().?, 10);

            std.debug.assert(pregap == 0); // FIXME: Not handled.

            // NOTE: Use MMAP on non-windows platforms?
            if (@import("builtin").os.tag != .windows) {
                const track_file_path = try std.fs.path.join(self._allocator, &[_][]const u8{ folder, filename });
                defer self._allocator.free(track_file_path);
                const track_file = std.fs.cwd().openFile(track_file_path, .{}) catch {
                    std.debug.print("Track File not found: {s}\n", .{track_file_path});
                    return error.TrackFileNotFound;
                };
                defer track_file.close();

                const track_data = try track_file.readToEndAlloc(self._allocator, 4 * 1024 * 1024 * 1024);

                self.tracks.items[num - 1] = (.{
                    .num = num,
                    .offset = offset,
                    .track_type = track_type,
                    .format = format,
                    .pregap = pregap,
                    .data = track_data,
                });
            } else {
                const track_file_path = try std.fs.path.joinZ(self._allocator, &[_][]const u8{ folder, filename });
                defer self._allocator.free(track_file_path);
                var track_file_abs_path_buffer: [std.fs.MAX_PATH_BYTES + 1]u8 = .{0} ** (std.fs.MAX_PATH_BYTES + 1);
                const track_file_abs_path = try std.fs.cwd().realpathZ(track_file_path, &track_file_abs_path_buffer);
                const file_path_w = try std.os.windows.cStrToPrefixedFileW(null, @as([*:0]u8, @ptrCast(track_file_abs_path.ptr)));

                const file_handle = try std.os.windows.OpenFile(file_path_w.span(), .{
                    .access_mask = std.os.windows.GENERIC_READ | std.os.windows.SYNCHRONIZE,
                    .creation = std.os.windows.FILE_OPEN,
                });
                const mapping_handle = windows.CreateFileMappingA(file_handle, null, std.os.windows.PAGE_READONLY, 0, 0, null);
                const ptr = windows.MapViewOfFile(mapping_handle.?, std.os.windows.SECTION_MAP_READ, 0, 0, 0);
                var info: std.os.windows.MEMORY_BASIC_INFORMATION = undefined;
                _ = try std.os.windows.VirtualQuery(ptr, &info, @sizeOf(std.os.windows.MEMORY_BASIC_INFORMATION));

                self.tracks.items[num - 1] = (.{
                    .num = num,
                    .offset = offset,
                    .track_type = track_type,
                    .format = format,
                    .pregap = pregap,
                    .data = @as([*]u8, @ptrCast(ptr))[0..info.RegionSize],
                    ._file_handle = file_handle,
                    ._mapping_handle = mapping_handle,
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

    pub fn get_primary_volume_descriptor(self: *const @This()) *const PVD {
        const offset = 0x10 * self.tracks.items[2].format + self.tracks.items[2].header_size(); // 16th sector + skip sector header
        return @ptrCast(@alignCast(self.tracks.items[2].data.ptr + offset));
    }

    pub fn get_corresponding_track(self: *const @This(), lda: u32) !*const Track {
        std.debug.assert(self.tracks.items.len > 0);
        var idx: u32 = 0;
        while (idx + 1 < self.tracks.items.len and self.tracks.items[idx + 1].offset <= lda) : (idx += 1) {}
        return &self.tracks.items[@max(0, idx)];
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
            if (std.mem.eql(u8, dir_record.get_file_identifier(), filename)) {
                std.debug.print("Loading '{s}' from LBA {X}\n", .{ filename, dir_record.location + GDI_SECTOR_OFFSET });
                return self.load_bytes(dir_record.location + GDI_SECTOR_OFFSET, dir_record.data_length, dest);
            }
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

test "gdi" {
    var gdi = try GDI.init("./bin/Sonic Adventure (PAL)/Sonic Adventure v1.003 (1999)(Sega)(PAL)(M5)[!].gdi", std.testing.allocator);
    defer gdi.deinit();
    try std.testing.expect(gdi.tracks.items.len == 3);

    const root_directory_entry = gdi.get_primary_volume_descriptor().*.root_directory_entry;
    try std.testing.expect(root_directory_entry.file_identifier == 0x0);

    const root_directory_length = root_directory_entry.length;
    const root_directory_lba = root_directory_entry.location;

    const root_track = try gdi.get_corresponding_track(root_directory_lba);
    const root_directory_offset: u32 = (root_directory_lba - root_track.offset) * root_track.format + 0x10; // TODO internalize offset computation to the GDI class. Why +0x10? No idea.

    var curr_offset = root_directory_offset;
    for (0..root_directory_length) |_| {
        const dir_record = root_track.get_directory_record(curr_offset);
        std.debug.print("({X:0>8}) Name: {s: >32} - LBA: {d: >8}, Size: {d: >8}\n", .{ curr_offset, dir_record.get_file_identifier(), dir_record.location, dir_record.data_length });
        curr_offset += dir_record.length;
    }
}
