const std = @import("std");
const common = @import("common.zig");

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

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        allocator.free(self.data);
    }

    pub fn get_directory_record(self: *const @This(), offset: usize) *const DirectoryRecord {
        return @ptrCast(@alignCast(self.data.ptr + offset));
    }
};

pub const GDI = struct {
    tracks: std.ArrayList(Track) = undefined,

    _allocator: std.mem.Allocator = undefined,

    pub fn init(self: *@This(), filepath: []const u8, allocator: std.mem.Allocator) !void {
        self._allocator = allocator;

        self.tracks = std.ArrayList(Track).init(self._allocator);

        const file = try std.fs.cwd().openFile(filepath, .{});
        defer file.close();
        const folder = std.fs.path.dirname(filepath) orelse ".";
        const data = try file.readToEndAlloc(self._allocator, 1024 * 1024 * 1024);
        defer self._allocator.free(data);
        // FIXME: Try to determine the end line sequence before splitting?
        var lines = std.mem.split(u8, data, "\r\n");

        const first_line = lines.next().?;
        const track_count = try std.fmt.parseUnsigned(u32, first_line, 10);
        try self.tracks.resize(track_count);

        for (0..track_count) |_| {
            var vals = std.mem.split(u8, lines.next().?, " ");

            const num = try std.fmt.parseUnsigned(u32, vals.next().?, 10);
            const offset = try std.fmt.parseUnsigned(u32, vals.next().?, 10);
            const track_type = try std.fmt.parseUnsigned(u8, vals.next().?, 10);
            const format = try std.fmt.parseUnsigned(u32, vals.next().?, 10);
            const filename = vals.next().?;
            const pregap = try std.fmt.parseUnsigned(u32, vals.next().?, 10);

            const track_file_path = try std.fs.path.join(self._allocator, &[_][]const u8{ folder, filename });
            defer self._allocator.free(track_file_path);
            const track_file = try std.fs.cwd().openFile(track_file_path, .{});
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
        }
    }

    pub fn deinit(self: *@This()) void {
        for (self.tracks.items) |*track| {
            track.deinit(self._allocator);
        }
        self.tracks.deinit();
    }

    pub fn get_primary_volume_descriptor(self: *const @This()) *const PVD {
        // FIXME: Figure out the actual offset, don't hardcode one.
        const offset = 0x9310; //(0xB06E - (self.tracks.items[2].offset + 150)) * self.tracks.items[2].format + 16;
        return @ptrCast(@alignCast(self.tracks.items[2].data.ptr + offset));
    }

    pub fn get_corresponding_track(self: *const @This(), lda: u32) !*const Track {
        var idx: u32 = 0;
        while (idx < self.tracks.items.len and self.tracks.items[idx].offset < lda) : (idx += 1) {}
        return &self.tracks.items[idx - 1];
    }

    pub fn load_file(self: *const @This(), filename: []const u8, dest: []u8) !void {
        const root_directory_entry = self.get_primary_volume_descriptor().*.root_directory_entry;
        const root_directory_length = root_directory_entry.length;
        const root_directory_lba = root_directory_entry.location;
        const root_track = try self.get_corresponding_track(root_directory_lba);
        const root_directory_offset: u32 = (root_directory_lba - root_track.offset) * root_track.format + 0x10; // Why +0x10? No idea.
        var curr_offset = root_directory_offset;
        // TODO: Handle directories, and not just root files.
        for (0..root_directory_length) |_| {
            const dir_record = root_track.get_directory_record(curr_offset);
            if (std.mem.eql(u8, dir_record.get_file_identifier(), filename)) {
                const lba = dir_record.location;
                const track = try self.get_corresponding_track(lba);
                var offset = (lba - track.offset) * track.format + 0x10;
                if (track.format == 2352) {
                    var copied: u32 = 0;
                    while (copied < dir_record.data_length) {
                        @memcpy(dest[copied .. copied + 2048], track.data[offset .. offset + 2048]);
                        copied += 2048;
                        offset += 2352;
                    }
                } else {
                    @panic("Unimplemented");
                }
                return;
            }
            curr_offset += dir_record.length;
        }
        return error.NotFound;
    }
};

test "gdi" {
    var gdi: GDI = .{};
    try gdi.init("./bin/[GDI] Sonic Adventure (PAL)/Sonic Adventure v1.003 (1999)(Sega)(PAL)(M5)[!].gdi", std.testing.allocator);
    defer gdi.deinit();
    try std.testing.expect(gdi.tracks.items.len == 3);

    const root_directory_entry = gdi.get_primary_volume_descriptor().*.root_directory_entry;
    try std.testing.expect(root_directory_entry.file_identifier == 0x0);

    const root_directory_length = root_directory_entry.length;
    const root_directory_lba = root_directory_entry.location;
    std.debug.print("root_directory_lda: {d}\n", .{root_directory_lba});
    const root_track = try gdi.get_corresponding_track(root_directory_lba);
    const root_directory_offset: u32 = (root_directory_lba - root_track.offset) * root_track.format + 0x10; // TODO internalize offset computation to the GDI class. Why +0x10? No idea.
    std.debug.print("root_directory_offset: {X}\n", .{root_directory_offset});
    var curr_offset = root_directory_offset;
    for (0..root_directory_length) |_| {
        const dir_record = root_track.get_directory_record(curr_offset);
        std.debug.print("({X:0>8}) Name: {s: >32} - LBA: {d: >8}, Size: {d: >8}\n", .{ curr_offset, dir_record.get_file_identifier(), dir_record.location, dir_record.data_length });
        curr_offset += dir_record.length;
    }
}
