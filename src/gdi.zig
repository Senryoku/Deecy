const std = @import("std");
const common = @import("common.zig");

const DirectoryRecord = extern struct {
    length: u8,
    extended_length: u8,
    location_little_endian: u32, // Location of extent (LBA) in both-endian format.
    location_big_endian: [4]u8,
    data_length_little_endian: u32, // Data length (size of extent) in both-endian format.
    data_length_big_endian: [4]u8,
    recording_date_and_time: [7]u8,
    file_flags: u8,
    file_unit_size: u8, // File unit size for files recorded in interleaved mode, zero otherwise.
    interleave_gap_size: u8, // Interleave gap size for files recorded in interleaved mode, zero otherwise.
    volume_sequence_number: [4]u8, // Volume sequence number - the volume that this extent is recorded on, in 16 bit both-endian format.
    file_identifier_length: u8,
    file_identifier: [1]u8,

    pub fn get_file_identifier(self: @This()) []const u8 {
        return @as([*]const u8, @ptrCast((&self.file_identifier)))[0..self.file_identifier_length];
    }
};

const PVD = extern struct {
    _r0: [8]u8,
    system_identifer: [32]u8,
    volume_identifer: [32]u8,
    _padding: [8]u8,
    volume_space_size: [8]u8,
    _unused: [32]u8,
    volume_set_size: [4]u8,
    volume_sequence_number: [4]u8,
    logical_block_size: [4]u8,
    path_table_size: [8]u8,
    l_path_table_location: [4]u8,
    optional_l_path_table_location: [4]u8,
    m_path_table_location: [4]u8,
    optional_m_path_table_location: [4]u8,
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

const Track = struct {
    num: u32,
    offset: u32,
    track_type: u8,
    format: u32,
    pregap: u32,
    data: []u8,

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        allocator.free(self.data);
    }

    pub fn get_directory_record(self: @This(), offset: usize) *const DirectoryRecord {
        return @ptrCast(&self.data[offset .. offset + @sizeOf(DirectoryRecord)]);
    }
};

const GDI = struct {
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
        try self.tracks.ensureTotalCapacity(track_count);

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
            try self.tracks.append(.{
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

    pub fn get_primary_volume_descriptor(self: @This()) *const PVD {
        const offset = (0xB06E - (self.tracks.items[2].offset + 150)) * self.tracks.items[2].format + 16;
        return @ptrCast(&self.tracks.items[2].data[offset .. offset + @sizeOf(PVD)]);
    }

    pub fn get_corresponding_track(self: @This(), lda: u32) !*const Track {
        var idx: u32 = 0;
        while (idx < self.tracks.items.len and self.tracks.items[idx].offset < lda) : (idx += 1) {}
        return &self.tracks.items[idx - 1];
    }
};

test "gdi" {
    var gdi: GDI = .{};
    try gdi.init("./bin/[GDI] Sonic Adventure (PAL)/Sonic Adventure v1.003 (1999)(Sega)(PAL)(M5)[!].gdi", std.testing.allocator);
    defer gdi.deinit();
    try std.testing.expect(gdi.tracks.items.len == 3);

    const root_directory_entry = gdi.get_primary_volume_descriptor().*.root_directory_entry;
    try std.testing.expect(root_directory_entry.file_identifier[0] == 0x0);
    std.debug.print("root_directory: {any}\n", .{root_directory_entry});
    const root_directory_length = root_directory_entry.length;
    const root_directory_lda = root_directory_entry.location_little_endian;
    std.debug.print("root_directory_lda: {d}\n", .{root_directory_lda});
    const root_track = try gdi.get_corresponding_track(root_directory_lda);
    const root_directory_offset: u32 = (root_directory_lda - root_track.offset) * 2352 + 16; // TODO internalize offset computation to the GDI class
    var curr_offset = root_directory_offset;
    for (0..root_directory_length) |_| {
        const dir_record = root_track.get_directory_record(curr_offset);
        std.debug.print("Name: {s}\n", .{dir_record.get_file_identifier()});
        std.debug.print("Name: {s}\n", .{dir_record.get_file_identifier()});
        curr_offset += dir_record.length;
    }
}
