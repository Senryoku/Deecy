const std = @import("std");

const DirectoryRecord = packed struct {
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
    file_identifier: []u8,
};

const PVD = packed struct {
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
    root_directory_entry: [34]u8, // This is what we're here for.
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
