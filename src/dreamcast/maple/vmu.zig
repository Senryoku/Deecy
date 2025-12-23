const std = @import("std");
const log = std.log.scoped(.maple);
const termcolor = @import("termcolor");

const common = @import("../maple.zig");
const FunctionCodesMask = common.FunctionCodesMask;
const DeviceInfoPayload = common.DeviceInfoPayload;

const StorageFunctionDefinition = packed struct(u32) {
    pt: u8, // Number of partitions - 1, probably 0
    bb: u8, // (Number of bytes per block - 1) / 32, probably 0x0F, or Number of bytes per block = 512
    ra: u4, // Number of accesses needed to read one block
    wa: u4, // Number of accesses needed to write one block
    fd: u6 = 0, // Reserved
    crc: u1, // CRC needed flag
    rm: u1, // Removable Media
};

const ScreenFunctionDefinition = packed struct(u32) {
    pt: u8, // Number of LCD - 1
    bb: u8, // (Number of bytes during Block transmission - 1) / 32
    _: u4 = 0,
    wa: u4, // Number of accesses to Block Write
    _fd: u5 = 0, // Reserved,
    bw: enum(u1) {
        NormallyWhite = 0, // LCD Data = '0' is White and LCD Data = '1' is Black
        NormallyBlack = 1, // LCD Data = '1' is White and LCD Data = '0' is Black
    },
    hv: enum(u2) {
        Horizontal1 = 0,
        Horizontal2 = 1,
        Vertical1 = 2,
        Vertical2 = 3,
    },
};

const LocationWord = packed struct(u32) {
    block_lsb: u8,
    block_msb: u8,
    phase: u8,
    partition: u8,
};

const FATValue = enum(u16) {
    DataEnd = 0xFFFA,
    Unused = 0xFFFC,
    BlockDamaged = 0xFFFF,
    _, // # of the next data block.
};

const BlockSize: u32 = 512;
const BlockCount = 256;
const ReadAccessPerBlock = 1;
const WriteAccessPerBlock = 4;
const FATBlock = 0x00FE;
const SystemBlock = BlockCount - 1;

const Capabilities: FunctionCodesMask = .{
    .storage = 1,
    .screen = 1,
    .timer = 1,
};
const Subcapabilities: [3]u32 = .{
    @bitCast(@as(u32, 0b01000000_00111111_01111110_01111110)), // Timer Function
    @bitCast(@as(u32, 0b00000000_00010000_00000101_00000000)), // ScreenFunctionDefinition
    @bitCast(StorageFunctionDefinition{
        .crc = 0,
        .rm = 0,
        .ra = ReadAccessPerBlock,
        .wa = WriteAccessPerBlock,
        .bb = 0x0F,
        .pt = 0,
    }),
};

const Identity: DeviceInfoPayload = .{
    .FunctionCodesMask = Capabilities,
    .SubFunctionCodesMasks = Subcapabilities,
    .RegionCode = 0xFF,
    .DescriptionString = "Visual Memory                  ".*,
    .StandbyConsumption = 0x007C,
    .MaximumConsumption = 0x0082,
};

const StorageMediaInfo = packed struct(u192) {
    total_size: u16,
    partition_number: u16,
    system_area_block_number: u16,
    fat_area_block_number: u16,

    number_of_fat_area_blocks: u16,
    file_information_block_number: u16,
    number_of_file_information_blocks: u16,
    volume_icon: u8,
    _reserved: u8 = 0,

    save_area_block_number: u16,
    number_of_save_area_blocks: u16,
    _reserved_for_execution_file: u32 = 0x80_0000, // Fixed to 0 *if* execution files cannot be executed.
};

blocks: [][BlockSize]u8,

backing_file_path: []const u8,
last_unsaved_change: ?i64 = null,

on_screen_update: ?struct { function: *const fn (userdata: ?*anyopaque, data: [*]const u8) void, userdata: ?*anyopaque } = null,

pub fn init(allocator: std.mem.Allocator, backing_file_path: []const u8) !@This() {
    var vmu: @This() = .{
        .backing_file_path = try allocator.dupe(u8, backing_file_path),
        .blocks = try allocator.alloc([BlockSize]u8, 0x100),
    };
    try vmu.load_or_init();
    return vmu;
}

fn load_or_init(self: *@This()) !void {
    try std.fs.cwd().makePath(std.fs.path.dirname(self.backing_file_path) orelse ".");
    var new_file = std.fs.cwd().createFile(self.backing_file_path, .{ .exclusive = true }) catch |e| {
        switch (e) {
            error.PathAlreadyExists => {
                log.info("Loading VMU from file '{s}'.", .{self.backing_file_path});
                var file = try std.fs.cwd().openFile(self.backing_file_path, .{});
                defer file.close();
                _ = try file.readAll(@as([*]u8, @ptrCast(self.blocks.ptr))[0 .. self.blocks.len * BlockSize]);
                return;
            },
            else => {
                log.err("Failed to create VMU file '{s}': {t}", .{ self.backing_file_path, e });
                return e;
            },
        }
    };
    defer new_file.close();

    // FIXME: Something's wrong here. I'm not initiliazing it properly.
    //        Switching to a dumb copy of a freshly formatted VMU by the bios, until I understand it better.
    if (comptime true) {
        for (0..self.blocks.len) |i| {
            @memset(&self.blocks[i], 0);
        }
        var fat_entries = @as([*]FATValue, @ptrCast(@alignCast(&self.blocks[FATBlock][0])));
        @memset(fat_entries[0..0x100], FATValue.Unused);
        @memcpy(self.blocks[FATBlock][0x1E0..], &[_]u8{
            0xFC, 0xFF, 0xFA, 0xFF, 0xF1, 0x00, 0xF2, 0x00,
            0xF3, 0x00, 0xF4, 0x00, 0xF5, 0x00, 0xF6, 0x00,
            0xF7, 0x00, 0xF8, 0x00, 0xF9, 0x00, 0xFA, 0x00,
            0xFB, 0x00, 0xFC, 0x00, 0xFA, 0xFF, 0xFA, 0xFF,
        });
        @memcpy(self.blocks[0xFF][0..96], &[_]u8{
            0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
            0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x20, 0x24, 0x07, 0x23, 0x01, 0x45, 0x23, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFE, 0x00, 0x01, 0x00, 0xFD, 0x00, 0x0D, 0x00, 0x00, 0x00,
            0xC8, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        });
    } else {
        @memset(self.blocks[0xFF][0..0x200], 0);
        // Fill system area
        @memset(self.blocks[0xFF][0..0x10], 0x55); // Format Information, all 0x55 means formatted.
        @memcpy(self.blocks[0xFF][0x10..0x30], "Volume Label                    "); // Volume Label
        @memcpy(self.blocks[0xFF][0x30..0x38], &[_]u8{ 19, 99, 12, 31, 23, 59, 0, 0 }); // Date and time created
        @memset(self.blocks[0xFF][0x38..0x40], 0); // Reserved

        @memcpy(self.blocks[0xFF][0x40 .. 0x40 + 24], std.mem.asBytes(&StorageMediaInfo{
            .total_size = BlockCount - 1,
            .partition_number = 0x0000,
            .system_area_block_number = SystemBlock,
            .fat_area_block_number = FATBlock,
            .number_of_fat_area_blocks = 0x0001,
            .file_information_block_number = 0x00FD,
            .number_of_file_information_blocks = 0x000D,
            .volume_icon = 0,
            .save_area_block_number = 0x00C8,
            .number_of_save_area_blocks = 0x00C8,
        })[0..24]);

        // "Format" the device.
        var fat_entries = @as([*]FATValue, @ptrCast(@alignCast(&self.blocks[FATBlock][0])));
        @memset(fat_entries[0..0x100], FATValue.Unused);
        fat_entries[FATBlock] = FATValue.DataEnd;
        fat_entries[SystemBlock] = FATValue.DataEnd; // Marks the system area block.
    }
    try new_file.writeAll(@as([*]u8, @ptrCast(self.blocks.ptr))[0 .. self.blocks.len * BlockSize]);
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    if (self.last_unsaved_change != null)
        self.save();
    allocator.free(self.blocks);
    allocator.free(self.backing_file_path);
}

pub fn save(self: *@This()) void {
    self.save_backup();
    var file = std.fs.cwd().openFile(self.backing_file_path, .{ .mode = .write_only }) catch |err| {
        log.err("Failed to open VMU file '{s}': {t}", .{ self.backing_file_path, err });
        return;
    };
    defer file.close();
    file.writeAll(@as([*]u8, @ptrCast(self.blocks.ptr))[0 .. self.blocks.len * BlockSize]) catch |err| {
        log.err("Failed to save VMU: {t}", .{err});
        return;
    };
    log.info("Saved VMU to file '{s}'.", .{self.backing_file_path});
    self.last_unsaved_change = null;
}

pub fn save_backup(self: *const @This()) void {
    const filename = std.fs.path.basename(self.backing_file_path);
    const dir_path = std.fs.path.dirname(self.backing_file_path) orelse ".";
    var dest_dir = std.fs.cwd().openDir(dir_path, .{}) catch |err| {
        log.err("Failed to open VMU destination directory '{s}': {t}", .{ dir_path, err });
        return;
    };
    var buf: [256]u8 = @splat(0);
    const backup_filename = std.fmt.bufPrint(&buf, "{s}.bak", .{filename}) catch |err| {
        log.err("Failed to format backup filename: {t}", .{err});
        return;
    };
    defer dest_dir.close();
    std.fs.cwd().copyFile(self.backing_file_path, dest_dir, backup_filename, .{}) catch |err| {
        log.err("Failed to backup VMU file '{s}': {t}", .{ backup_filename, err });
    };
}

pub fn get_identity(_: *const @This()) DeviceInfoPayload {
    return Identity;
}

// Write Media Info to dest. Returns the payload size in 32-bit words.
pub fn get_media_info(self: *const @This(), dest: [*]u8, function: u32, partition_number: u8) u8 {
    _ = self;
    _ = partition_number;

    switch (function) {
        FunctionCodesMask.Storage.as_u32() => {
            const value: StorageMediaInfo = .{
                .total_size = BlockCount - 1,
                .partition_number = 0x0000,
                .system_area_block_number = SystemBlock,
                .fat_area_block_number = FATBlock,
                .number_of_fat_area_blocks = 0x0001,
                .file_information_block_number = 0x00FD,
                .number_of_file_information_blocks = 0x000D,
                .volume_icon = 0,
                .save_area_block_number = 0x00C8,
                .number_of_save_area_blocks = 0x00C8,
            };
            // NOTE: @sizeOf(GetMediaInformationResponse) == 32 for some reason.
            @memcpy(dest[0..24], std.mem.asBytes(&value)[0..24]);
            return 24 / 4;
        },
        FunctionCodesMask.Screen.as_u32() => {
            const value: packed struct { x_dots: u8, y_dots: u8, contrast: u4, gradation: u4, reserved: u8 = 2 } = .{
                .x_dots = 48 - 1,
                .y_dots = 32 - 1,
                .contrast = 0,
                .gradation = 1,
            };
            @memcpy(dest[0..4], std.mem.asBytes(&value));
            return 1;
        },
        else => log.err(termcolor.red("Unimplemented VMU::GetMediaInformation for function: {f}"), .{@as(FunctionCodesMask, @bitCast(function))}),
    }
    return 0;
}

/// Returns payload size in 32-bit words
pub fn block_read(self: *const @This(), dest: [*]u8, function: u32, partition: u8, block_num: u16, phase: u8) u8 {
    std.debug.assert(partition == 0);
    switch (function) {
        FunctionCodesMask.Storage.as_u32() => {
            if (block_num >= BlockCount)
                log.err(termcolor.red("Invalid block number: {d} (BlockCount: {d})"), .{ block_num, BlockCount });
            const len = BlockSize / ReadAccessPerBlock;
            const start: u32 = len * phase;
            @memcpy(dest[0..len], self.blocks[block_num % BlockCount][start .. start + len]);
            return len / 4;
        },
        else => log.err("Unimplemented VMU.block_read for function: {f}", .{@as(FunctionCodesMask, @bitCast(function))}),
    }
    return 0;
}

/// Returns payload size in 32-bit words
pub fn block_write(self: *@This(), function: u32, partition: u8, phase: u8, block_num: u16, data: []const u32) u8 {
    switch (function) {
        FunctionCodesMask.Screen.as_u32() => {
            //  - Partition is the screen number, should always be zero.
            //  - Phase is used if a frame doesn't fit in one message.
            //  - Block number specify the plane.
            // Since the standard VMU is only 48*32 black and white, we can probably ignore both Phase and Block number.

            if (self.on_screen_update) |callback| {
                callback.function(callback.userdata, @ptrCast(data.ptr));
            }
            return 48 * 32 / 8 / 4;
        },
        FunctionCodesMask.Storage.as_u32() => {
            log.warn(termcolor.yellow("Storage BlockWrite! Partition: {d} Block: {d}, Phase: {d} (data length: {d} bytes)"), .{ partition, block_num, phase, data.len * 4 });

            const start = phase * (BlockSize / WriteAccessPerBlock);
            const size = @min(BlockSize / WriteAccessPerBlock, data.len * 4);
            @memcpy(self.blocks[block_num][start .. start + size], std.mem.sliceAsBytes(data)[0..size]);

            self.last_unsaved_change = std.time.timestamp();
            return @intCast(size / 4);
        },
        else => log.err("Unimplemented VMU.block_write for function: {f}", .{@as(FunctionCodesMask, @bitCast(function))}),
    }
    return 0;
}

pub fn set_condition(self: *@This(), function: u32, data: []const u32) void {
    _ = self;
    _ = data;
    switch (function) {
        FunctionCodesMask.Timer.as_u32() => {
            // "An alarm is buzzer output produced by a pulse generator that is controlled by the Timer Function's built-in counter.
            //  The Timer Function can support a maximum of two alarm types.
            //  The alarm types that can be used are declared in the function definition block. The volume of the alarms cannot be adjusted.""

            // data holds the duty cycle of two alarms.
        },
        else => log.err("Unimplemented VMU.set_condition for function: {f}", .{@as(FunctionCodesMask, @bitCast(function))}),
    }
}
