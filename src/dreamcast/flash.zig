const std = @import("std");
const termcolor = @import("termcolor");

const flash_log = std.log.scoped(.flashrom);

// MBM29LV002TC (29LV002TC-90PTN)
// NOTE: Special addresses in the documentation are 0x555 and 0x2AA, however the boot ROM uses 0x5555 and 0x2AAA.
//       Maybe I'm missing a mask somewhere?
const CommandAddr0 = 0x5555;
const CommandAddr1 = 0x2AAA;

fn sector_addresses(addr: u32) !struct { start: u32, end: u32 } {
    const tag = (addr >> 13) & 0b11111;

    // MBM29LV002TC
    if ((tag & 0b11000) == 0b00000) return .{ .start = 0x00000, .end = 0x10000 }; // SA0
    if ((tag & 0b11000) == 0b01000) return .{ .start = 0x10000, .end = 0x20000 }; // SA1
    if ((tag & 0b11000) == 0b10000) return .{ .start = 0x20000, .end = 0x30000 }; // SA2
    if ((tag & 0b11100) == 0b11000) return .{ .start = 0x30000, .end = 0x38000 }; // SA3
    if ((tag & 0b11111) == 0b11100) return .{ .start = 0x38000, .end = 0x3A000 }; // SA4
    if ((tag & 0b11111) == 0b11101) return .{ .start = 0x3A000, .end = 0x3C000 }; // SA5
    if ((tag & 0b11110) == 0b11110) return .{ .start = 0x3C000, .end = 0x40000 }; // SA6

    // MBM29LV002BC
    // if ((tag & 0b11110) == 0b00000) return .{ .start = 0x00000, .end = 0x04000 }; // SA0
    // if ((tag & 0b11111) == 0b00010) return .{ .start = 0x04000, .end = 0x06000 }; // SA1
    // if ((tag & 0b11111) == 0b00011) return .{ .start = 0x06000, .end = 0x08000 }; // SA2
    // if ((tag & 0b11100) == 0b00100) return .{ .start = 0x08000, .end = 0x10000 }; // SA3
    // if ((tag & 0b11000) == 0b01000) return .{ .start = 0x10000, .end = 0x20000 }; // SA4
    // if ((tag & 0b11000) == 0b10000) return .{ .start = 0x20000, .end = 0x30000 }; // SA5
    // if ((tag & 0b11000) == 0b11000) return .{ .start = 0x30000, .end = 0x40000 }; // SA6

    return error.InvalidSectorAddress;
}

mode: enum { Normal, Program, Fast, FastProgram, FastReset } = .Normal,
write_cycle: u8 = 0,
data: []u8 align(4),

_allocator: std.mem.Allocator,

pub fn init(allocator: std.mem.Allocator) !@This() {
    return @This(){
        .data = try allocator.alloc(u8, 0x20000),
        ._allocator = allocator,
    };
}

pub fn deinit(self: *@This()) void {
    self._allocator.free(self.data);
}

pub fn reset(self: *@This()) void {
    self.mode = .Normal;
    self.write_cycle = 0;
}

pub fn write(self: *@This(), addr: u32, data: u8) void {
    flash_log.debug("Write: @{X:0>8}, 0x{X:0>2}", .{ addr, data });
    std.debug.assert(addr <= 0x1FFFF);

    switch (self.write_cycle) {
        0 => {
            if (self.mode == .Fast and data == 0xA0) {
                self.mode = .FastProgram;
                self.write_cycle = 1;
                flash_log.debug("Fast Program: @{X:0>8} = 0x{X:0>2}", .{ addr, data });
            } else if (self.mode == .Fast and data == 0x90) {
                self.write_cycle = 1;
            } else if (addr == CommandAddr0 and data == 0xAA) {
                self.write_cycle = 1;
            } else if (data == 0xF0) {
                // Read/Reset?
                self.write_cycle = 0;
                self.mode = .Normal;
            } else {
                self.unexpected_write(addr, data);
            }
        },
        1 => {
            if (self.mode == .FastProgram) {
                // Fast Program
                self.write_cycle = 0;
                self.mode = .Fast;
                self.data[addr] = data;
                flash_log.debug("Program: @{X:0>8} = 0x{X:0>2}", .{ addr, data });
            } else if (self.mode == .FastReset and data == 0x00 or data == 0xF0) {
                // Reset from Fast Mode
                self.write_cycle = 0;
                self.mode = .Normal;
            } else if (addr == CommandAddr1 and data == 0x55) {
                self.write_cycle = 2;
            } else {
                self.unexpected_write(addr, data);
            }
        },
        2 => {
            if (addr == CommandAddr0) {
                switch (data) {
                    0xF0 => {
                        // Read/Reset
                        self.write_cycle = 0;
                    },
                    0x90 => {
                        // Autoselect
                        // TODO?
                        self.write_cycle = 0;
                    },
                    0xA0 => {
                        // Program
                        self.mode = .Program;
                        self.write_cycle = 3;
                    },
                    0x80 => {
                        // Chip or Sector Erase
                        self.write_cycle = 3;
                    },
                    0x20 => { // Set to Fast Mode
                        self.mode = .Fast;
                        self.write_cycle = 0;
                        flash_log.debug("Fast Mode enabled.", .{});
                    },
                    else => self.unexpected_write(addr, data),
                }
            } else {
                self.unexpected_write(addr, data);
            }
        },
        3 => {
            switch (self.mode) {
                .Normal => {
                    if (addr == CommandAddr0 and data == 0xAA) {
                        self.write_cycle = 4;
                    } else self.unexpected_write(addr, data);
                },
                .Program => {
                    flash_log.info("  Program: @{X:0>8} = 0x{X:0>2} (was 0x{X:0>2})", .{ addr, data, self.data[addr] });
                    self.mode = .Normal;
                    self.write_cycle = 0;
                    self.data[addr] &= data;
                },
                else => self.unexpected_write(addr, data),
            }
        },
        4 => {
            if (addr == CommandAddr1 and data == 0x55) {
                self.write_cycle = 5;
            } else self.unexpected_write(addr, data);
        },
        5 => {
            if (addr == CommandAddr0 and data == 0x10) {
                // Chip Erase
                // TODO
                flash_log.warn(termcolor.yellow("Chip Erase {X:0>8}: Unimplemented!"), .{addr});
                self.write_cycle = 0;
            } else if (data == 0x30) {
                // Sector Erase
                self.write_cycle = 0;
                flash_log.info("Sector Erase {X:0>8}", .{addr});
                const range = sector_addresses(addr) catch |err| {
                    flash_log.warn("Sector Erase: Invalid sector address {X:0>8} ({})", .{ addr, err });
                    return;
                };
                @memset(self.data[range.start..range.end], 0xFF);
            } else self.unexpected_write(addr, data);
        },
        else => self.unexpected_write(addr, data),
    }
}

fn unexpected_write(self: *@This(), addr: u32, data: u16) void {
    flash_log.warn("Unexpected write to FlashROM (write cycle: {}): @{X:0>8} = {X:0>4}", .{ self.write_cycle, addr, data });
    self.write_cycle = 0;
    self.mode = .Normal;
}

pub fn serialize(self: @This(), writer: *std.Io.Writer) !usize {
    var bytes: usize = 0;
    bytes += try writer.write(std.mem.asBytes(&self.mode));
    bytes += try writer.write(std.mem.asBytes(&self.write_cycle));
    bytes += try writer.write(std.mem.sliceAsBytes(self.data[0..]));
    return bytes;
}

pub fn deserialize(self: *@This(), reader: *std.Io.Reader) !void {
    try reader.readSliceAll(std.mem.asBytes(&self.mode));
    try reader.readSliceAll(std.mem.asBytes(&self.write_cycle));
    try reader.readSliceAll(std.mem.sliceAsBytes(self.data[0..]));
}

// Dreamcast specific bits
const Language = @import("dreamcast.zig").Language;

const PartitionHeaderMagic = "KATANA_FLASH____";

pub const PartitionHeader = extern struct {
    magic: [16]u8 = PartitionHeaderMagic,
    partition_number: u8,
    version_number: u8,
    reserved: [46]u8 = @splat(0xFF),
};

pub const Partition = struct {
    const BlockSize = 64; // bytes
    const BitmapSize = 512 / 8; // bytes

    offset: u32,
    size: u32,

    pub fn block(self: @This(), block_num: u32) u32 {
        return self.offset + (block_num + 1) * BlockSize;
    }

    pub fn bitmap_count(self: @This()) u32 {
        return (self.size + (32768 - 1)) / 32768;
    }

    pub fn bitmaps(self: @This()) u32 {
        return self.offset + self.size - (BitmapSize * self.bitmap_count());
    }

    pub fn header(self: @This()) u32 {
        return self.offset;
    }

    pub fn end(self: @This()) u32 {
        return self.offset + self.size;
    }
};

pub const FactorySettings = Partition{ .offset = 0x1A000, .size = 0x2000 };
pub const GameSettings = Partition{ .offset = 0x10000, .size = 0x8000 };
pub const SystemSettings = Partition{ .offset = 0x1C000, .size = 0x4000 };

pub const DCPartition = enum(u8) {
    FactorySettings = 0, // Read-only
    Reserved = 1, // Zeroes
    SystemSettings = 2, // Block allocated
    GameSettings = 3, // Block allocated
    Unused = 4, // Block allocated
};

pub fn get_dc_partition(partition: DCPartition) Partition {
    return switch (partition) {
        .FactorySettings => FactorySettings,
        .Reserved => .{ .offset = 0x18000, .size = 0x2000 },
        .SystemSettings => SystemSettings,
        .GameSettings => GameSettings,
        .Unused => .{ .offset = 0, .size = 0x10000 },
    };
}

pub fn UserBlock(comptime T: type) type {
    if (@sizeOf(T) != 60) @compileError("UserBlock payload size must be 60 bytes");
    return extern struct {
        logical_block_number: u16 align(1),
        user_payload: T align(1),
        crc: u16 align(1),

        pub fn update_crc(self: *@This()) void {
            self.crc = block_crc(std.mem.asBytes(self)[0..62]);
        }

        pub fn check_crc(self: *const @This()) bool {
            return self.crc == block_crc(std.mem.asBytes(self)[0..62]);
        }
    };
}

pub const SystemConfigPayload = extern struct {
    pub const LogicalBlockNumber = 0x5;

    pub const SoundMode = enum(u8) { Stereo = 0x00, Mono = 0x01 };
    pub const AutoStart = enum(u8) { On = 0x00, Off = 0x01 };

    time_low: u16 align(1) = 0,
    time_high: u16 align(1) = 0,
    unknown_2: u8 align(1) = 0xFF,
    language: Language align(1) = .English,
    sound: SoundMode align(1) = .Stereo,
    auto_start: AutoStart align(1) = .On,
    unknown_6: u32 align(1) = 0xFFFFFFFF,
    unknown_7: [3 * 16]u8 align(1) = @splat(0xFF),
};

pub fn allocated_block_count(self: *@This(), partition: Partition) u32 {
    var count: u32 = 0;
    var bitmap_offset = partition.bitmaps();
    while (bitmap_offset < partition.end() and self.data[bitmap_offset] == 0) {
        bitmap_offset += 1;
        count += 8;
    }
    if (bitmap_offset < partition.end())
        count += @clz(self.data[bitmap_offset]);
    return count;
}

/// Search the most recent allocated block with the given logical block number.
pub fn get_logical_block(self: *@This(), comptime T: type, partition: Partition, logical_block_number: u16) ?*UserBlock(T) {
    const block_count = self.allocated_block_count(partition);
    if (block_count == 0) return null;
    var physical_block_number: i32 = @intCast(block_count - 1);
    while (physical_block_number >= 0) {
        const block = self.get_physical_block(T, partition, @intCast(physical_block_number));
        if (block.logical_block_number == logical_block_number and block.check_crc()) return block;
        physical_block_number -= 1;
    }
    return null;
}

/// Allocates the next available block.
/// Assumes the partition is not full.
pub fn allocate_block(self: *@This(), comptime T: type, partition: Partition) *UserBlock(T) {
    const block_count = self.allocated_block_count(partition);
    self.allocate_physical_block(partition, block_count);
    std.debug.assert(block_count + 1 == self.allocated_block_count(partition));
    return self.get_physical_block(T, partition, block_count);
}

/// Assumes the partition is not full.
pub fn get_or_allocate_logical_block(self: *@This(), comptime T: type, partition: Partition, logical_block_number: u16) *UserBlock(T) {
    return self.get_logical_block(T, partition, logical_block_number) orelse {
        var block = self.allocate_block(T, partition);
        block.logical_block_number = logical_block_number;
        return block;
    };
}

pub fn get_physical_block(self: *@This(), comptime T: type, partition: Partition, physical_block_number: u32) *UserBlock(T) {
    return @ptrCast(@alignCast(&self.data[partition.block(physical_block_number)]));
}

pub fn allocate_physical_block(self: *@This(), partition: Partition, physical_block_number: u32) void {
    const bitmap_offset = partition.bitmaps() + (physical_block_number / 8);
    self.data[bitmap_offset] &= ~(@as(u8, 1) << @intCast(7 - (physical_block_number % 8)));
}

pub fn block_crc(block: []const u8) u16 {
    var n: u16 = 0xFFFF;
    for (block) |b| {
        n ^= (@as(u16, b) << 8);
        for (0..8) |_| {
            if (n & 0x8000 != 0) {
                n = (n << 1) ^ 4129;
            } else {
                n <<= 1;
            }
        }
    }
    return ~n;
}
