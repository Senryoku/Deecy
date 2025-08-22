const std = @import("std");

const log = std.log.scoped(.elf);

pub const ELFType = enum(u8) { Relocatable = 1, Executable = 2, Shared = 3, Core = 4, _ };
pub const InstructionSet = enum(u8) { None = 0, Sparc = 0x02, x86 = 0x03, MIPS = 0x08, PowerPC = 0x14, ARM = 0x28, SuperH = 0x2A, IA_64 = 0x32, x86_64 = 0x3E, AArch64 = 0xB7, RISC_V = 0xF3, _ };

pub const SegmentType = enum(u32) {
    Null = 0, // Program header table entry unused.
    Load = 1, // Loadable segment.
    Dynamic = 2, // Dynamic linking information.
    Interp = 3, // Interpreter information.
    Note = 4, // Auxiliary information.
    ShLib = 5, // Reserved
    Phdr = 6, // Segment containing program header table itself.
    Tls = 7, // Thread-Local Storage template.
    LoSunW = 0x6FFFFFFA,
    HiSunW = 0x6FFFFFFF,
    SunWBSS = 0x6FFFFFFB,
    // SunWStack = 0x6FFFFFFA,
    LoProc = 0x70000000,
    HiProc = 0x7FFFFFFF,
    _,
};

pub const SegmentFlags = packed struct(u32) {
    executable: bool = false,
    writable: bool = false,
    readable: bool = false,
    _: u29 = 0,
};

fn ProgramHeader(comptime word_type: type) type {
    switch (word_type) {
        u32 => return extern struct {
            p_type: SegmentType,
            p_offset: u32,
            p_vaddr: u32,
            p_paddr: u32,
            p_filesz: u32,
            p_memsz: u32,
            p_flags: SegmentFlags,
            p_align: u32,
        },
        u64 => return extern struct {
            p_type: SegmentType,
            p_flags: SegmentFlags,
            p_offset: u64,
            p_vaddr: u64,
            p_paddr: u64,
            p_filesz: u64,
            p_memsz: u64,
            p_align: u64,
        },
        else => @compileError("Unsupported word size"),
    }
}

const SectionType = enum(u32) {
    Null = 0,
    ProgBits = 1,
    SymTab = 2,
    StrTab = 3,
    Rela = 4,
    Hash = 5,
    Dynamic = 6,
    Note = 7,
    NoBits = 8,
    Rel = 9,
    Shlib = 10,
    Dynsym = 11,
    SunW_Move = 0x6FFFFFFA,
    SunW_COMDAT = 0x6FFFFFFB,
    SunW_Syminfo = 0x6FFFFFFC,
    SunW_Verdef = 0x6FFFFFFD,
    SunW_Verneed = 0x6FFFFFFE,
    SunW_Versym = 0x6FFFFFFF,
    LoProc = 0x70000000,
    HiProc = 0x7FFFFFFF,
    LoUser = 0x80000000,
    HiUser = 0xFFFFFFFF,
};

const SectionFlags = packed struct(u32) {
    writable: bool = false,
    alloc: bool = false,
    execinstr: bool = false,
    _0: u1 = 0,

    merge: bool = false,
    strings: bool = false,
    info_link: bool = false,
    link_order: bool = false,

    os_nonconforming: bool = false,
    group: bool = false,
    _1: u10 = 0,

    maskos: u8 = 0,

    _2: u2 = 0,
    ordered: bool = false,
    exclude: bool = false,
};

fn SectionHeader(comptime word_type: type) type {
    switch (word_type) {
        u32 => return extern struct {
            sh_name: u32,
            sh_type: SectionType,
            sh_flags: SectionFlags,
            sh_addr: u32,
            sh_offset: u32,
            sh_size: u32,
            sh_link: u32,
            sh_info: u32,
            sh_addralign: u32,
            sh_entsize: u32,
        },
        u64 => return extern struct {
            sh_name: u32,
            sh_type: SectionType,
            sh_flags: SectionFlags,
            sh_addr: u64,
            sh_offset: u64,
            sh_size: u64,
            sh_link: u32,
            sh_info: u32,
            sh_addralign: u64,
            sh_entsize: u64,
        },
        else => @compileError("Unsupported word size"),
    }
}

program_entry_offset: u64,
section_header_string_table_index: u64,
program_headers: []ProgramHeader(u64),
section_headers: []SectionHeader(u64),
allocator: std.mem.Allocator,

pub fn init(allocator: std.mem.Allocator, stream: *std.io.StreamSource) !@This() {
    const reader = stream.reader();
    const signature = try reader.readInt(u32, .little);
    if (signature != 0x464C457F) return error.InvalidELF;

    const word_size: u8 = switch (try reader.readByte()) {
        1 => 32,
        2 => 64,
        else => return error.InvalidELF,
    };
    const endianness: std.builtin.Endian = switch (try reader.readByte()) {
        1 => .little,
        2 => .big,
        else => return error.InvalidELF,
    };
    const header_version = try reader.readByte();
    const abi = try reader.readByte();
    _ = abi;

    try reader.skipBytes(8, .{});

    if (header_version != 1) return error.Unsupported;
    switch (endianness) {
        inline .little, .big => |e| {
            switch (word_size) {
                32 => return _init(e, u32, allocator, stream),
                64 => return _init(e, u64, allocator, stream),
                else => return error.InvalidELF,
            }
        },
    }
}

pub fn string(self: *@This(), index: u64) []const u8 {
    return self.strings[index..][0 .. std.mem.indexOf(u8, self.strings[index..], &[_]u8{0}) orelse 0];
}

fn _init(comptime endianness: std.builtin.Endian, comptime word_type: type, allocator: std.mem.Allocator, stream: *std.io.StreamSource) !@This() {
    const reader = stream.reader();
    const elf_type: ELFType = @enumFromInt(try reader.readInt(u16, endianness));
    const instruction_set: InstructionSet = @enumFromInt(try reader.readInt(u16, endianness));
    const version = try reader.readInt(u32, endianness);
    const program_entry_offset = try reader.readInt(word_type, endianness);
    const program_header_table_offset = try reader.readInt(word_type, endianness);
    const section_header_table_offset = try reader.readInt(word_type, endianness);
    const flags = try reader.readInt(u32, endianness);
    const header_size = try reader.readInt(u16, endianness);
    const program_header_entry_size = try reader.readInt(u16, endianness);
    const program_header_entry_count = try reader.readInt(u16, endianness);
    const section_header_entry_size = try reader.readInt(u16, endianness);
    const section_header_entry_count = try reader.readInt(u16, endianness);
    const section_header_string_table_index = try reader.readInt(u16, endianness);
    log.info(
        \\  elf_type: {any}, 
        \\  instruction_set: {any}, 
        \\  version: {d}, 
        \\  program_entry_offset: {X}, 
        \\  program_header_offset: {X}, 
        \\  section_header_offset: {X}, 
        \\  flags: {any}, 
        \\  header_size: {d}, 
        \\  program_header_entry_size: {d}, 
        \\  program_header_entry_count: {d}, 
        \\  section_header_entry_size: {d}, 
        \\  section_header_entry_count: {d}, 
        \\  section_header_string_table_index: {d}
    , .{
        elf_type,
        instruction_set,
        version,
        program_entry_offset,
        program_header_table_offset,
        section_header_table_offset,
        flags,
        header_size,
        program_header_entry_size,
        program_header_entry_count,
        section_header_entry_size,
        section_header_entry_count,
        section_header_string_table_index,
    });

    var program_headers = std.ArrayList(ProgramHeader(u64)).init(allocator);
    defer program_headers.deinit();

    if (elf_type == .Executable) {
        try stream.seekableStream().seekTo(program_header_table_offset);
        for (0..program_header_entry_count) |i| {
            const ph = try reader.readStruct(ProgramHeader(word_type));
            log.debug("ProgramHeader {d}: {any}", .{ i, ph });
            try program_headers.append(.{
                .p_type = ph.p_type,
                .p_offset = ph.p_offset,
                .p_vaddr = ph.p_vaddr,
                .p_paddr = ph.p_paddr,
                .p_filesz = ph.p_filesz,
                .p_memsz = ph.p_memsz,
                .p_flags = ph.p_flags,
                .p_align = ph.p_align,
            });
        }
    }

    var section_headers = std.ArrayList(SectionHeader(u64)).init(allocator);
    defer section_headers.deinit();

    try stream.seekableStream().seekTo(section_header_table_offset);
    for (0..section_header_entry_count) |i| {
        const sh = try reader.readStruct(SectionHeader(word_type));
        log.debug("SectionHeader {d}: {any}", .{ i, sh });
        try section_headers.append(.{
            .sh_name = sh.sh_name,
            .sh_type = sh.sh_type,
            .sh_flags = sh.sh_flags,
            .sh_addr = sh.sh_addr,
            .sh_offset = sh.sh_offset,
            .sh_size = sh.sh_size,
            .sh_link = sh.sh_link,
            .sh_info = sh.sh_info,
            .sh_addralign = sh.sh_addralign,
            .sh_entsize = sh.sh_entsize,
        });
    }

    return .{
        .program_entry_offset = program_entry_offset,
        .section_header_string_table_index = section_header_string_table_index,
        .program_headers = try program_headers.toOwnedSlice(),
        .section_headers = try section_headers.toOwnedSlice(),
        .allocator = allocator,
    };
}

pub fn deinit(self: *@This()) void {
    self.allocator.free(self.program_headers);
    self.allocator.free(self.section_headers);
}
