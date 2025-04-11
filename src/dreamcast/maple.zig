const std = @import("std");
const termcolor = @import("termcolor");

const maple_log = std.log.scoped(.maple);

const DreamcastModule = @import("dreamcast.zig");
const Dreamcast = DreamcastModule.Dreamcast;

const PatternSelection = enum(u3) {
    Normal = 0b000,
    LightGun = 0b001,
    RESET = 0b010,
    ReturnFromLightGun = 0b011,
    NOP = 0b111,
};

const Instruction = packed struct(u32) {
    transfer_length: u8, // In units of 4 bytes.
    pattern: PatternSelection,
    _z0: u5 = 0,
    port_select: u2,
    _z1: u13 = 0,
    end_flag: u1,

    pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        try writer.print("MapleInstruction{{ transfer_length: {X}, pattern: {s}, port_select: {X}, end_flag: {X} }}", .{ self.transfer_length, @tagName(self.pattern), self.port_select, self.end_flag });
    }
};

// Structure of a transfer:
//   Instruction
//   Response Address (System Memory)
//   Data[...]

const Command = enum(u8) {
    DeviceInfoRequest = 0x01,
    ExtendedDeviceInfo = 0x02,
    Reset = 0x03,
    Shutdown = 0x04,
    DeviceInfo = 0x05,
    ExtendedDevice = 0x06,
    Acknowledge = 0x07, // Named "Device Reply" in some documentation
    DataTransfer = 0x08,
    GetCondition = 0x09,
    GetMediaInformation = 0x0A,
    BlockRead = 0x0B,
    BlockWrite = 0x0C,
    GetLastError = 0x0D,
    SetCondition = 0x0E,
    ARError = 0xF9,
    LCDError = 0xFA,
    FileError = 0xFB,
    RequestResend = 0xFC,
    UnknownCommand = 0xFD,
    FunctionCodeNotSupported = 0xFE,

    _,
};

const CommandWord = packed struct(u32) {
    command: Command,
    recipent_address: u8,
    sender_address: u8,
    payload_length: u8, // In 32-bit words.

    pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        try writer.print("{s}{{ recipent: {X}, sender: {X}, payload_length: {X} }}", .{ @tagName(self.command), self.recipent_address, self.sender_address, self.payload_length });
    }
};

pub const InputCapabilities = packed struct(u32) {
    _0: u8 = 0,

    analogRtrigger: u1 = 0,
    analogLtrigger: u1 = 0,
    analogHorizontal: u1 = 0,
    analogVertical: u1 = 0,
    analogHorizontal2: u1 = 0,
    analogVertical2: u1 = 0,

    _1: u2 = 0,

    z: u1 = 0,
    y: u1 = 0,
    x: u1 = 0,
    d: u1 = 0,
    up2: u1 = 0,
    down2: u1 = 0,
    left2: u1 = 0,
    right2: u1 = 0,

    c: u1 = 0,
    b: u1 = 0,
    a: u1 = 0,
    start: u1 = 0,
    up: u1 = 0,
    down: u1 = 0,
    left: u1 = 0,
    right: u1 = 0,

    pub fn as_u32(self: @This()) u32 {
        return @bitCast(self);
    }
};

const FunctionCodesMask = packed struct(u32) {
    _: u20 = 0,

    vibration: u1 = 0,
    mouse: u1 = 0,

    _2: u2 = 0,

    controller: u1 = 0,
    storage: u1 = 0,
    screen: u1 = 0,
    timer: u1 = 0,
    audio_input: u1 = 0,
    argun: u1 = 0,
    keyboard: u1 = 0,
    gun: u1 = 0,

    pub fn as_u32(self: @This()) u32 {
        return @bitCast(self);
    }

    const Screen = @This(){ .screen = 1 };
    const Storage = @This(){ .storage = 1 };
    const Timer = @This(){ .timer = 1 };
    const Controller = @This(){ .controller = 1 };

    pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        if (@popCount(self.as_u32()) == 1) {
            inline for (@typeInfo(FunctionCodesMask).@"struct".fields) |field| {
                if (@field(self, field.name) == 1) try writer.writeAll(field.name);
            }
        } else {
            try writer.print("{X}", .{self.as_u32()});
        }
    }
};

const LocationWord = packed struct(u32) {
    block_lsb: u8,
    block_msb: u8,
    phase: u8,
    partition: u8,
};

const DeviceInfoPayload = extern struct {
    FunctionCodesMask: FunctionCodesMask align(1),
    SubFunctionCodesMasks: [3]u32 align(1),
    RegionCode: u8 align(1) = 0xFF,
    ConnectionDirectionCode: u8 align(1) = 0,
    DescriptionString: [31]u8 align(1) = @splat(0),
    ProducerString: [60]u8 align(1) = @splat(0),
    StandbyConsumption: u16 align(1) = 0,
    MaximumConsumption: u16 align(1) = 0,
    // Possible extension
};

// NOTE: 0 = Pressed!
pub const ControllerButtons = packed struct(u16) {
    _0: u1 = 1,

    b: u1 = 1,
    a: u1 = 1,
    start: u1 = 1,
    up: u1 = 1,
    down: u1 = 1,
    left: u1 = 1,
    right: u1 = 1,

    _1: u1 = 1,

    y: u1 = 1,
    x: u1 = 1,

    _2: u5 = 0b11111,
};

pub const StandardControllerCapabilities: InputCapabilities = .{
    .b = 1,
    .a = 1,
    .start = 1,
    .up = 1,
    .down = 1,
    .left = 1,
    .right = 1,
    .y = 1,
    .x = 1,
    .analogRtrigger = 1,
    .analogLtrigger = 1,
    .analogHorizontal = 1,
    .analogVertical = 1,
};

pub const DualStickControllerCapabilities: InputCapabilities = .{
    .b = 1,
    .a = 1,
    .start = 1,
    .up = 1,
    .down = 1,
    .left = 1,
    .right = 1,
    .y = 1,
    .x = 1,
    .analogRtrigger = 1,
    .analogLtrigger = 1,
    .analogHorizontal = 1,
    .analogVertical = 1,
    .analogHorizontal2 = 1,
    .analogVertical2 = 1,
};

pub const Controller = struct {
    pub const Capabilities: FunctionCodesMask = .{ .controller = 1 };

    subcapabilities: [3]u32 = .{ @bitCast(DualStickControllerCapabilities), 0, 0 },

    buttons: ControllerButtons = .{},
    axis: [6]u8 = @splat(0x80),
    pub fn press_buttons(self: *@This(), buttons: ControllerButtons) void {
        self.buttons = @bitCast(@as(u16, @bitCast(self.buttons)) & @as(u16, @bitCast(buttons)));
    }
    pub fn release_buttons(self: *@This(), buttons: ControllerButtons) void {
        self.buttons = @bitCast(@as(u16, @bitCast(self.buttons)) | ~@as(u16, @bitCast(buttons)));
    }

    pub fn get_identity(self: *const @This()) DeviceInfoPayload {
        return .{
            .FunctionCodesMask = Capabilities,
            .SubFunctionCodesMasks = self.subcapabilities,
            .DescriptionString = "Dreamcast Controller           ".*, // NOTE: dc-arm7wrestler checks for this, maybe some games do too?
            .ProducerString = "Produced By or Under License From SEGA ENTERPRISES,LTD.     ".*,
            .StandbyConsumption = 0x01AE,
            .MaximumConsumption = 0x01F4,
        };
    }

    pub fn get_condition(self: *const @This()) [3]u32 {
        var r = [3]u32{ 0xFFFFFFFF, 0x8080FFFF, 0x80808080 };
        r[0] = @bitCast(Capabilities);
        r[1] = @as(u16, @bitCast(self.buttons));
        for (0..6) |i| {
            @as([*]u8, @ptrCast(&r))[6 + i] = self.axis[i];
        }
        return r;
    }

    pub fn serialize(_: @This(), _: anytype) !usize {
        return 0;
    }

    pub fn deserialize(_: anytype) !@This() {
        return .{};
    }
};

const GetMediaInformationResponse = packed struct(u192) {
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

const FATValue = enum(u16) {
    DataEnd = 0xFFFA,
    Unused = 0xFFFC,
    BlockDamaged = 0xFFFF,
    _, // # of the next data block.
};

pub const VMU = struct {
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
        .ProducerString = "Produced By or Under License From SEGA ENTERPRISES,LTD.     ".*,
        .StandbyConsumption = 0x007C,
        .MaximumConsumption = 0x0082,
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
                    maple_log.info("Loading VMU from file '{s}'.", .{self.backing_file_path});
                    var file = try std.fs.cwd().openFile(self.backing_file_path, .{});
                    defer file.close();
                    _ = try file.readAll(@as([*]u8, @ptrCast(self.blocks.ptr))[0 .. self.blocks.len * BlockSize]);
                    return;
                },
                else => {
                    maple_log.err("Failed to create VMU file '{s}': {any}", .{ self.backing_file_path, e });
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
            var fat_entries = @as([*]FATValue, @alignCast(@ptrCast(&self.blocks[FATBlock][0])));
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

            @memcpy(self.blocks[0xFF][0x40 .. 0x40 + 24], std.mem.asBytes(&GetMediaInformationResponse{
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
            var fat_entries = @as([*]FATValue, @alignCast(@ptrCast(&self.blocks[FATBlock][0])));
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
            maple_log.err("Failed to open VMU file '{s}': {any}", .{ self.backing_file_path, err });
            return;
        };
        defer file.close();
        file.writeAll(@as([*]u8, @ptrCast(self.blocks.ptr))[0 .. self.blocks.len * BlockSize]) catch |err| {
            maple_log.err("Failed to save VMU: {any}", .{err});
        };
        maple_log.info("Saved VMU to file '{s}'.", .{self.backing_file_path});
        self.last_unsaved_change = null;
    }

    pub fn save_backup(self: *const @This()) void {
        const filename = std.fs.path.basename(self.backing_file_path);
        const dir_path = std.fs.path.dirname(self.backing_file_path) orelse ".";
        var dest_dir = std.fs.cwd().openDir(dir_path, .{}) catch |err| {
            maple_log.err("Failed to open VMU destination directory '{s}': {any}", .{ dir_path, err });
            return;
        };
        var buf: [256]u8 = @splat(0);
        const backup_filename = std.fmt.bufPrint(&buf, "{s}.bak", .{filename}) catch |err| {
            maple_log.err("Failed to format backup filename: {any}", .{err});
            return;
        };
        defer dest_dir.close();
        std.fs.cwd().copyFile(self.backing_file_path, dest_dir, backup_filename, .{}) catch |err| {
            maple_log.err("Failed to backup VMU file '{s}': {any}", .{ backup_filename, err });
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
                const value: GetMediaInformationResponse = .{
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
            else => maple_log.err(termcolor.red("Unimplemented VMU::GetMediaInformation for function: {any}"), .{@as(FunctionCodesMask, @bitCast(function))}),
        }
        return 0;
    }

    /// Returns payload size in 32-bit words
    pub fn block_read(self: *const @This(), dest: [*]u8, function: u32, partition: u8, block_num: u16, phase: u8) u8 {
        std.debug.assert(partition == 0);
        switch (function) {
            FunctionCodesMask.Storage.as_u32() => {
                if (block_num >= BlockCount)
                    maple_log.err(termcolor.red("Invalid block number: {any} (BlockCount: {any})"), .{ block_num, BlockCount });
                const len = BlockSize / ReadAccessPerBlock;
                const start: u32 = len * phase;
                @memcpy(dest[0..len], self.blocks[block_num % BlockCount][start .. start + len]);
                return len / 4;
            },
            else => maple_log.err("Unimplemented VMU.block_read for function: {any}", .{function}),
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
                maple_log.warn(termcolor.yellow("Storage BlockWrite! Partition: {any} Block: {any}, Phase: {any} (data length: {d} bytes)"), .{ partition, block_num, phase, data.len * 4 });

                const start = phase * (BlockSize / WriteAccessPerBlock);
                const size = @min(BlockSize / WriteAccessPerBlock, data.len * 4);
                @memcpy(self.blocks[block_num][start .. start + size], std.mem.sliceAsBytes(data)[0..size]);

                self.last_unsaved_change = std.time.timestamp();
                return @intCast(size / 4);
            },
            else => maple_log.err("Unimplemented VMU.block_write for function: {any}", .{function}),
        }
        return 0;
    }

    pub fn set_condition(self: *@This(), function: u32, data: u32) void {
        _ = self;
        _ = data;
        switch (function) {
            FunctionCodesMask.Timer.as_u32() => {
                // "An alarm is buzzer output produced by a pulse generator that is controlled by the Timer Function's built-in counter.
                // The Timer Function can support a maximum of two alarm types.
                // The alarm types that can be used are declared in the function definition block. The volume of the alarms cannot
                // be adjusted.""

                // data holds the duty cycle of two alarms.
            },
            else => maple_log.err("Unimplemented VMU.set_condition for function: {any}", .{function}),
        }
    }

    pub fn serialize(_: @This(), _: anytype) !usize {
        return 0;
    }
};

const Peripheral = union(enum) {
    Controller: Controller,
    VMU: VMU,

    pub fn tag(self: @This()) std.meta.Tag(@This()) {
        return std.meta.activeTag(self);
    }

    pub fn get_identity(self: @This()) DeviceInfoPayload {
        return switch (self) {
            inline else => |impl| impl.get_identity(),
        };
    }

    pub fn block_read(self: *const @This(), dest: [*]u8, function: u32, partition: u8, block_num: u16, phase: u8) u8 {
        return switch (self.*) {
            .VMU => |*v| v.block_read(dest, function, partition, block_num, phase),
            else => s: {
                maple_log.err(termcolor.red("Unimplemented BlockRead for target: {any}"), .{self.tag()});
                break :s 0;
            },
        };
    }

    pub fn block_write(self: *@This(), function: u32, partition: u8, phase: u8, block_num: u16, data: []const u32) u8 {
        switch (self.*) {
            .VMU => |*v| return v.block_write(function, partition, phase, block_num, data),
            else => maple_log.warn(termcolor.yellow("BlockWrite Unimplemented for target: {any}"), .{self.tag()}),
        }
        return 0;
    }

    pub fn serialize(self: @This(), writer: anytype) !usize {
        var bytes: usize = 0;
        switch (self) {
            inline else => |impl| {
                const t: u32 = @intFromEnum(self.tag());
                bytes += try writer.write(std.mem.asBytes(&t));
                bytes += try impl.serialize(writer);
            },
        }
        return bytes;
    }
};

const MaplePort = struct {
    main: ?Peripheral = null,
    subperipherals: [5]?Peripheral = @splat(null),

    /// Returns the number of 32bytes words transferred to the host.
    pub fn handle_command(self: *@This(), dc: *Dreamcast, data: [*]u32) u32 {
        const return_addr = data[0];
        std.debug.assert(return_addr >= 0x0C000000 and return_addr < 0x10000000);
        const command: CommandWord = @bitCast(data[1]);
        const function_type = data[2];
        maple_log.debug("  Dest: {X:0>8}, Command: {any}, Function: {any}", .{ return_addr, command, @as(FunctionCodesMask, @bitCast(function_type)) });

        // NOTE: The sender address should also include the sub-peripheral bit when appropriate.
        // "When a main peripheral identifies itself in the response to a command, it sets the sub-peripheral bit for each sub-peripheral that is connected in addition to bit 5."
        var sender_address = command.recipent_address;
        if (sender_address & 0b100000 != 0) {
            for (0..5) |i| {
                if (self.subperipherals[i] != null)
                    sender_address |= @as(u8, 1) << @intCast(i);
            }
        }
        const recipent_address = command.sender_address;

        const maybe_target = switch (command.recipent_address & 0b11111) {
            0 => &self.main, // Equivalent to setting bit 5.
            else => &self.subperipherals[@ctz(command.recipent_address)],
        };
        if (maybe_target.*) |*target| {
            switch (command.command) {
                .DeviceInfoRequest => {
                    const identity = target.get_identity();
                    dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .DeviceInfo, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = @intCast(@sizeOf(DeviceInfoPayload) / 4) }));
                    const ptr = dc.cpu._get_memory(return_addr + 4);
                    @memcpy(@as([*]u8, @ptrCast(ptr))[0..@sizeOf(DeviceInfoPayload)], std.mem.asBytes(&identity));
                    return 1 + @sizeOf(DeviceInfoPayload) / 4;
                },
                .GetCondition => {
                    switch (target.*) {
                        .Controller => |*c| {
                            std.debug.assert(command.payload_length == 1);
                            std.debug.assert(function_type == 0x01000000);
                            const condition = c.get_condition();
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = @intCast(condition.len) }));
                            const ptr: [*]u32 = @alignCast(@ptrCast(dc.cpu._get_memory(return_addr + 4)));
                            @memcpy(ptr[0..condition.len], &condition);
                            return 1 + condition.len;
                        },
                        else => {
                            maple_log.err(termcolor.red("Unimplemented GetCondition for target: {any}"), .{target.tag()});
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                            return 1;
                        },
                    }
                },
                .GetMediaInformation => {
                    std.debug.assert(command.payload_length == 2);
                    const partition_number: u8 = @truncate(data[3] >> 24);
                    maple_log.warn(termcolor.yellow("  GetMediaInformation: Function: {any}, Partition number: {any}"), .{ @as(FunctionCodesMask, @bitCast(function_type)), partition_number });

                    switch (target.*) {
                        .VMU => |*v| {
                            const dest = @as([*]u8, @ptrCast(dc.cpu._get_memory(return_addr + 8)))[0..];
                            const payload_size = v.get_media_info(dest, function_type, partition_number);
                            if (payload_size > 0) {
                                dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = payload_size + 1 }));
                                dc.cpu.write_physical(u32, return_addr + 4, function_type);
                                return 1 + payload_size + 1;
                            } else {
                                dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                                return 1;
                            }
                        },
                        else => {
                            maple_log.err(termcolor.red("Unimplemented GetMediaInformation for target: {any}"), .{target.tag()});
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                            return 1;
                        },
                    }
                },
                .BlockRead => {
                    std.debug.assert(function_type == @as(u32, @bitCast(FunctionCodesMask{ .storage = 1 })));
                    const partition: u8 = @truncate((data[3] >> 0) & 0xFF);
                    const phase: u8 = @truncate((data[3] >> 8) & 0xFF);
                    const block_num: u16 = @truncate(((data[3] >> 24) & 0xFF) | ((data[3] >> 8) & 0xFF00));
                    maple_log.warn(termcolor.yellow("BlockRead! Partition: {any} Block: {any}, Phase: {any} (data[3]: {X:0>8})"), .{ partition, block_num, phase, data[3] });

                    const dest = @as([*]u8, @ptrCast(dc.cpu._get_memory(return_addr + 12)))[0..];
                    const payload_size = target.block_read(dest, function_type, partition, block_num, phase);

                    if (payload_size > 0) {
                        dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = payload_size + 2 }));
                        dc.cpu.write_physical(u32, return_addr + 4, function_type);
                        dc.cpu.write_physical(u32, return_addr + 8, data[3]); // Header repeating the location.
                        return 1 + payload_size + 2;
                    } else {
                        dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FileError, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                        return 1;
                    }
                },
                .BlockWrite => {
                    const partition: u8 = @truncate((data[3] >> 0) & 0xFF);
                    const phase: u8 = @truncate((data[3] >> 8) & 0xFF);
                    const block_num: u16 = @truncate(((data[3] >> 24) & 0xFF) | ((data[3] >> 8) & 0xFF00));
                    const write_data = data[4 .. 4 + command.payload_length - 2];
                    const words_written = target.block_write(function_type, partition, phase, block_num, write_data);
                    dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .Acknowledge, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                    return 1 + words_written;
                },
                .GetLastError => {
                    dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .Acknowledge, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                    return 1;
                },
                .SetCondition => {
                    switch (target.*) {
                        .VMU => |*vmu| {
                            vmu.set_condition(function_type, data[2]);
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .Acknowledge, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                            return 1;
                        },
                        else => {
                            maple_log.err(termcolor.red("Unimplemented SetCondition for target: {any}"), .{target.tag()});
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                            return 1;
                        },
                    }
                    return 1;
                },
                else => {
                    maple_log.warn(termcolor.yellow("Unimplemented command: {s} ({X:0>2}, {X:0>8})"), .{ std.enums.tagName(Command, command.command) orelse "Unknown", @intFromEnum(command.command), data[0..4] });
                    dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                    return 1;
                },
            }
        } else {
            dc.cpu.write_physical(u32, return_addr, 0xFFFFFFFF); // "No connection"
            return 1;
        }
    }

    pub fn serialize(self: @This(), writer: anytype) !usize {
        // Nothing for now.
        if (comptime true) {
            return 0;
        } else {
            var bytes: usize = 0;
            if (self.main) |main| {
                bytes += try main.serialize(writer);
            } else {
                const tag: u32 = 0xFFFFFFFF;
                bytes += try writer.write(std.mem.asBytes(&tag));
            }

            for (self.subperipherals) |sub| {
                if (sub) |s| {
                    bytes += try s.serialize(writer);
                } else {
                    const tag: u32 = 0xFFFFFFFF;
                    bytes += try writer.write(std.mem.asBytes(&tag));
                }
            }

            return bytes;
        }
    }

    pub fn deserialize(_: @This(), _: anytype) !usize {
        return 0;
    }
};

pub const MapleHost = struct {
    ports: [4]MaplePort = @splat(.{}),

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !MapleHost {
        return .{ ._allocator = allocator };
    }

    pub fn deinit(self: *MapleHost) void {
        for (&self.ports) |*port| {
            // Can't have a VMU as the main peripheral.
            for (&port.subperipherals) |*maybe_peripheral| {
                if (maybe_peripheral.*) |*peripheral| {
                    switch (peripheral.*) {
                        .VMU => |*vmu| {
                            vmu.deinit(self._allocator);
                        },
                        else => {},
                    }
                }
            }
        }
    }

    pub fn transfer(self: *MapleHost, dc: *Dreamcast, data: [*]u32) void {
        var idx: u32 = 0;

        var transferred_words: usize = 0;
        defer {
            transferred_words += idx; // Add the words constituting the command list to the total transferred.
            const cycles: u32 = @intCast(4 * transferred_words * Dreamcast.SH4Clock / (2 * 1024 * 1024 / 8)); // 2Mb/s?
            dc.schedule_interrupt(.{ .EoD_Maple = 1 }, cycles);
            maple_log.debug("    Transferred {d} words, scheduled interrupt in {d} cycles", .{ transferred_words, cycles });
        }

        // A transfer can have a maximum of 1024 words.
        while (idx < 1024) {
            const instr: Instruction = @bitCast(data[idx]);
            idx += 1;

            maple_log.debug("{any}", .{instr});

            switch (instr.pattern) {
                .Normal => {
                    transferred_words += self.ports[instr.port_select].handle_command(dc, data[idx..]);
                    idx += instr.transfer_length + 2;
                },
                .NOP, .RESET => {},
                else => maple_log.warn(termcolor.yellow("[Maple] Unimplemented pattern: {}. Ignoring it, hopefully the payload is empty :D"), .{instr.pattern}),
            }

            if (instr.end_flag == 1)
                return;
        }
    }

    // Checks if we have some unsaved changes in VMUs, and writes them to disc,
    // only if the last write was more than 5 seconds ago.
    // Not writing to disc after every single VMU write is not only wasteful,
    // it also make backups useless (a backup with half an update is useless).
    // VMU will also flush themselves on exit if needed.
    pub fn flush_vmus(self: *@This()) void {
        const now = std.time.timestamp();
        for (&self.ports) |*port| {
            for (&port.subperipherals) |*maybe_peripheral| {
                if (maybe_peripheral.*) |*peripheral| {
                    switch (peripheral.*) {
                        .VMU => |*vmu| {
                            if (vmu.last_unsaved_change) |last_unsaved_change| {
                                if (now - last_unsaved_change > 5)
                                    vmu.save();
                            }
                        },
                        else => {},
                    }
                }
            }
        }
    }

    pub fn serialize(self: @This(), writer: anytype) !usize {
        var bytes: usize = 0;
        for (&self.ports) |port| {
            bytes += try port.serialize(writer);
        }
        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: anytype) !usize {
        var bytes: usize = 0;
        for (&self.ports) |*port| {
            bytes += try port.deserialize(reader);
        }
        return bytes;
    }
};
