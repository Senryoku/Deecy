const std = @import("std");

pub const DirectoryRecord = extern struct {
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

pub const PVD = extern struct {
    type_code: enum(u8) {
        BootRecordVolumeDescriptor = 0,
        PrimaryVolumeDescriptor = 1,
        SupplementaryVolumeDescriptor = 2,
        VolumePartitionDescriptor = 3,
        VolumeDescriptorSetTerminator = 255,
        _,
    },
    standard_identifier: [5]u8, // == "CD001"
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

    pub fn is_valid(self: *const @This()) bool {
        return self.type_code == .PrimaryVolumeDescriptor and std.mem.eql(u8, &self.standard_identifier, "CD001") and self.version == 0x01;
    }
};

test "PVD" {
    try std.testing.expect(@offsetOf(PVD, "root_directory_entry") == 156);
}
