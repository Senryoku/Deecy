const std = @import("std");
const termcolor = @import("termcolor");
const common = @import("common.zig");

const maple_log = std.log.scoped(.maple);

const Dreamcast = @import("dreamcast.zig").Dreamcast;

const zglfw = @import("zglfw");

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
};

const InputCapabilities = packed struct(u32) {
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
};

// Note: I'm not sure of the order of fields here...
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
};

const LocationWord = packed struct(u32) {
    block_lsb: u8,
    block_msb: u8,
    phase: u8,
    partition: u8,
};

// Not sure about the order in this, especially around the strings.
const DeviceInfoPayload = extern struct {
    FunctionCodesMask: FunctionCodesMask align(1),
    SubFunctionCodesMasks: [3]u32 align(1),
    RegionCode: u8 align(1) = 0xFF,
    ConnectionDirectionCode: u8 align(1) = 0,
    DescriptionString: [30]u8 align(1) = .{0} ** 30,
    ProducerString: [60]u8 align(1) = .{0} ** 60,
    StandbyConsumption: u16 align(1) = 0,
    MaximumConsumption: u16 align(1) = 0,
    // Possible extension
};

const StandardControllerCapabilities: InputCapabilities = .{
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

const VMUCapabilities: FunctionCodesMask = .{
    .storage = 1,
    .screen = 1,
    .timer = 1,
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

const Controller = struct {
    capabilities: FunctionCodesMask = .{ .controller = 1 },
    subcapabilities: [3]u32 = .{ @bitCast(StandardControllerCapabilities), 0, 0 },

    buttons: ControllerButtons = .{},
    axis: [6]u8 = .{0x80} ** 6,
    pub fn press_buttons(self: *@This(), buttons: ControllerButtons) void {
        self.buttons = @bitCast(@as(u16, @bitCast(self.buttons)) & @as(u16, @bitCast(buttons)));
    }
    pub fn release_buttons(self: *@This(), buttons: ControllerButtons) void {
        self.buttons = @bitCast(@as(u16, @bitCast(self.buttons)) | ~@as(u16, @bitCast(buttons)));
    }

    pub fn get_identity(self: *const @This()) [@sizeOf(DeviceInfoPayload) / @sizeOf(u32)]u32 {
        var r: [@sizeOf(DeviceInfoPayload) / @sizeOf(u32)]u32 = undefined;
        @as(*DeviceInfoPayload, @ptrCast(&r)).* = .{
            .FunctionCodesMask = self.capabilities,
            .SubFunctionCodesMasks = self.subcapabilities,
            .DescriptionString = "Dreamcast Controller         \u{0}".*, // NOTE: dc-arm7wrestler checks for this, maybe some games do too?
            .ProducerString = "Produced By or Under License From SEGA ENTERPRISES,LTD.    \u{0}".*,
            .StandbyConsumption = 0x01AE,
            .MaximumConsumption = 0x01F4,
        };
        return r;
    }

    pub fn get_condition(self: *const @This()) [3]u32 {
        var r = [3]u32{ 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
        r[0] = @bitCast(self.capabilities);
        r[1] = @as(u16, @bitCast(self.buttons));
        for (0..6) |i| {
            @as([*]u8, @ptrCast(&r))[6 + i] = self.axis[i];
        }
        return r;
    }
};

const GetMediaInformationResponse = packed struct {
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
    _reserved_for_execution_file: u32 = 0,
    _reserved2: u16 = 0,
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
    wa: u8, // Number of accesses to Block Write
    hv: u8, // Specifies whether the LCD data rows are horizontal or vertical. 0/1 = Horizontal, 2/3 = Vertical
};

const FATValue = enum(u16) {
    DataEnd = 0xFFFA,
    Unused = 0xFFFC,
    BlockDamaged = 0xFFFF,
    _, // # of the next data block.
};

const VMU = struct {
    const BlockSize: u32 = 512;
    const BlockCount = 256;
    const ReadAccessPerBlock = 1;
    const WriteAccessPerBlock = 1;
    const FATBlock = 0x00FE;
    const SystemBlock = BlockCount - 1;

    backing_file: []const u8 = "userdata/vmu_0.bin", // TODO: Make this customizable.

    capabilities: FunctionCodesMask = VMUCapabilities,
    subcapabilities: [3]u32 = .{
        @bitCast(@as(u32, 0b01000000_00111111_01111110_01111110)),
        @bitCast(@as(u32, 0b00000000_00010000_00000101_00000000)), // One of these is ScreenFunctionDefinition
        @bitCast(StorageFunctionDefinition{
            .crc = 0,
            .rm = 0,
            .ra = ReadAccessPerBlock,
            .wa = WriteAccessPerBlock,
            .bb = 0x0F,
            .pt = 0,
        }),
    },
    blocks: [][BlockSize]u8 = undefined,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        var vmu: @This() = .{};

        vmu.blocks = try allocator.alloc([BlockSize]u8, 0x100);

        var new_file = std.fs.cwd().createFile(vmu.backing_file, .{ .exclusive = true }) catch |e| {
            switch (e) {
                error.PathAlreadyExists => {
                    maple_log.info("Loading VMU from file '{s}'.", .{vmu.backing_file});
                    var file = try std.fs.cwd().openFile(vmu.backing_file, .{});
                    defer file.close();
                    _ = try file.readAll(@as([*]u8, @ptrCast(vmu.blocks.ptr))[0 .. vmu.blocks.len * BlockSize]);
                    return vmu;
                },
                else => {
                    maple_log.err("Failed to create VMU file '{s}': {any}", .{ vmu.backing_file, e });
                    return e;
                },
            }
        };
        defer new_file.close();

        @memset(vmu.blocks[0xFF][0..0x200], 0);
        // Fill system area
        @memset(vmu.blocks[0xFF][0..0x10], 0x55); // Format Information, all 0x55 means formatted.
        @memcpy(vmu.blocks[0xFF][0x10..0x30], "Volume Label                    "); // Volume Label
        @memcpy(vmu.blocks[0xFF][0x30..0x38], &[_]u8{ 19, 99, 12, 31, 23, 59, 0, 0 }); // Date and time created
        @memset(vmu.blocks[0xFF][0x38..0x40], 0); // Reserved

        @as(*GetMediaInformationResponse, @alignCast(@ptrCast(&vmu.blocks[0xFF][0x40]))).* = .{
            .total_size = BlockCount - 1,
            .partition_number = 0x0000,
            .system_area_block_number = SystemBlock,
            .fat_area_block_number = 0x00FE,
            .number_of_fat_area_blocks = 0x0001,
            .file_information_block_number = 0x00FD,
            .number_of_file_information_blocks = 0x000D,
            .volume_icon = 0,
            .save_area_block_number = 0x00C8,
            .number_of_save_area_blocks = 0x00C8,
        };

        // "Format" the device.
        var fat_entries = @as([*]FATValue, @alignCast(@ptrCast(&vmu.blocks[FATBlock][0])));
        @memset(fat_entries[0..0x100], FATValue.Unused);
        fat_entries[FATBlock] = FATValue.DataEnd;
        fat_entries[SystemBlock] = FATValue.DataEnd; // Marks the system area block.

        try new_file.writeAll(@as([*]u8, @ptrCast(vmu.blocks.ptr))[0 .. vmu.blocks.len * BlockSize]);

        return vmu;
    }

    pub fn deinit(self: *const @This(), allocator: std.mem.Allocator) void {
        self.save();
        allocator.free(self.blocks);
    }

    pub fn save(self: *const @This()) void {
        maple_log.info("Save VMU to file '{s}'.", .{self.backing_file});
        var file = std.fs.cwd().openFile(self.backing_file, .{ .mode = .write_only }) catch |err| {
            maple_log.err("Failed to open VMU file '{s}': {any}", .{ self.backing_file, err });
            return;
        };
        defer file.close();
        file.writeAll(@as([*]u8, @ptrCast(self.blocks.ptr))[0 .. self.blocks.len * BlockSize]) catch |err| {
            maple_log.err("Failed to save VMU: {any}", .{err});
        };
    }

    pub fn get_identity(self: *const @This()) [@sizeOf(DeviceInfoPayload) / @sizeOf(u32)]u32 {
        var r: [@sizeOf(DeviceInfoPayload) / @sizeOf(u32)]u32 = undefined;
        @as(*DeviceInfoPayload, @ptrCast(&r)).* = .{
            .FunctionCodesMask = self.capabilities,
            .SubFunctionCodesMasks = self.subcapabilities,
        };
        return r;
    }

    // Write Media Info to dest. Returns the payload size in 32-bit words.
    pub fn get_media_info(self: *const @This(), dest: [*]u8, function: u32, partition_number: u8) u8 {
        _ = self;
        _ = partition_number;

        switch (function) {
            @as(u32, @bitCast(FunctionCodesMask{ .storage = 1 })) => {
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
                @memcpy(dest[0..@sizeOf(GetMediaInformationResponse)], @as([*]const u8, @ptrCast(&value))[0..@sizeOf(GetMediaInformationResponse)]);
                return @sizeOf(GetMediaInformationResponse) / 4;
            },
            else => {
                maple_log.err("Unimplemented VMU::GetMediaInformation for function: {any}", .{function});
            },
        }
        return 0;
    }

    pub fn block_read(self: *const @This(), dest: [*]u8, function: u32, partition: u8, block_num: u16, phase: u8) u8 {
        std.debug.assert(partition == 0);
        switch (function) {
            @as(u32, @bitCast(FunctionCodesMask{ .storage = 1 })) => {
                const start: u32 = (BlockSize / ReadAccessPerBlock) * phase;
                const len = BlockSize / ReadAccessPerBlock;
                @memcpy(dest[start .. start + len], self.blocks[block_num][start .. start + len]);
                return len / 4;
            },
            else => {
                maple_log.err("Unimplemented VMU.block_read for function: {any}", .{function});
            },
        }
        return 0;
    }

    fn bw_ascii(val: u1) []const u8 {
        return switch (val) {
            0 => "  ",
            1 => "@@",
        };
    }

    pub fn block_write(self: *const @This(), function: u32, partition: u8, phase: u8, block_num: u16, data: []const u32) void {
        switch (function) {
            @as(u32, @bitCast(FunctionCodesMask{ .screen = 1 })) => {
                // TODO: Do something with this frame for the LCD.
                //        - Partition is the screen number, should always be zero.
                //        - Phase is used if a frame doesn't fit in one message.
                //        - Block number specify the plane.
                //       Since the standard VMU is only 48*32 black and white, we can probably ignore both Phase and Block number.

                // Enable for a cute animation in the terminal, and to destroy your framerate :^)
                if (false) {
                    std.debug.print("\u{01B}[2J\u{01B}[H  Phase: {any} Block: {any} Data: \n", .{ phase, block_num });
                    const bytes: [*]const u8 = @ptrCast(data.ptr);
                    for (0..data.len * 4) |i| {
                        std.debug.print("{s}{s}{s}{s}{s}{s}{s}{s}", .{
                            bw_ascii(@truncate(bytes[data.len * 4 - 1 - i] >> 0)),
                            bw_ascii(@truncate(bytes[data.len * 4 - 1 - i] >> 1)),
                            bw_ascii(@truncate(bytes[data.len * 4 - 1 - i] >> 2)),
                            bw_ascii(@truncate(bytes[data.len * 4 - 1 - i] >> 3)),
                            bw_ascii(@truncate(bytes[data.len * 4 - 1 - i] >> 4)),
                            bw_ascii(@truncate(bytes[data.len * 4 - 1 - i] >> 5)),
                            bw_ascii(@truncate(bytes[data.len * 4 - 1 - i] >> 6)),
                            bw_ascii(@truncate(bytes[data.len * 4 - 1 - i] >> 7)),
                        });
                        if ((i + 1) % 6 == 0) {
                            std.debug.print("\n", .{});
                        }
                    }
                }
            },
            @as(u32, @bitCast(FunctionCodesMask{ .storage = 1 })) => {
                maple_log.warn(termcolor.yellow("Storage BlockWrite! Partition: {any} Block: {any}, Phase: {any} (data[3]: {X:0>8})"), .{ partition, block_num, phase, data[3] });

                std.debug.assert(data.len == BlockSize / 4);
                const bytes: [*]const u8 = @ptrCast(data.ptr);
                @memcpy(self.blocks[block_num][0..BlockSize], bytes[0..BlockSize]);

                self.save();
            },
            else => {
                maple_log.err("Unimplemented VMU.block_write for function: {any}", .{function});
            },
        }
    }
};

const PeripheralType = enum {
    Controller,
    VMU,
};

const Peripheral = union(PeripheralType) {
    Controller: Controller,
    VMU: VMU,
};

const MaplePort = struct {
    main: ?Peripheral = null,
    subperipherals: [5]?Peripheral = .{null} ** 5,

    pub fn handle_command(self: *@This(), dc: *Dreamcast, data: [*]u32) u32 {
        const return_addr = data[0];

        const command: CommandWord = @bitCast(data[1]);
        maple_log.debug("    Command: {any}", .{command});

        // Note: The sender address should also include the sub-peripheral bit when appropriate.
        var sender_address = command.recipent_address;
        for (0..5) |i| {
            if (self.subperipherals[i] != null)
                sender_address |= @as(u8, 1) << @intCast(i);
        }

        const maybe_target = switch (command.recipent_address & 0b11111) {
            0 => self.main,
            else => self.subperipherals[@ctz(command.recipent_address)],
        };
        if (maybe_target) |target| {
            switch (command.command) {
                .DeviceInfoRequest => {
                    const identity = switch (target) {
                        .Controller => |c| c.get_identity(),
                        .VMU => |v| v.get_identity(),
                    };
                    dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DeviceInfo, .sender_address = sender_address, .recipent_address = command.sender_address, .payload_length = @intCast(identity.len) }));
                    const ptr: [*]u32 = @alignCast(@ptrCast(dc.cpu._get_memory(return_addr + 4)));
                    @memcpy(ptr[0..identity.len], &identity);
                },
                .GetCondition => {
                    switch (target) {
                        .Controller => |c| {
                            std.debug.assert(command.payload_length == 1);
                            const function = data[2];
                            std.debug.assert(function == 0x01000000);

                            const condition = c.get_condition();
                            dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = sender_address, .recipent_address = command.sender_address, .payload_length = @intCast(condition.len) }));
                            const ptr: [*]u32 = @alignCast(@ptrCast(dc.cpu._get_memory(return_addr + 4)));
                            @memcpy(ptr[0..condition.len], &condition);
                        },
                        .VMU => {
                            maple_log.err("TODO: GetCondition for VMU", .{});
                            @panic("TODO VMU GET CONDITION");
                        },
                        //else => {
                        //    maple_log.err("Unimplemented GetCondition for target: {any}", .{target});
                        //    @panic("[Maple] Unimplemented GetCondition for target");
                        //},
                    }
                },
                .GetMediaInformation => {
                    std.debug.assert(command.payload_length == 2);

                    const function_type = data[2];
                    const partition_number: u8 = @truncate(data[3] >> 24);

                    maple_log.warn(termcolor.yellow("  GetMediaInformation: Function type: {X:0>8} Partition number: {any}"), .{ function_type, partition_number });

                    switch (target) {
                        .VMU => |v| {
                            const dest = @as([*]u8, @ptrCast(dc.cpu._get_memory(return_addr + 8)))[0..];
                            const payload_size = v.get_media_info(dest, function_type, partition_number);
                            if (payload_size > 0) {
                                dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = payload_size }));
                                dc.cpu.write32(return_addr + 4, function_type);
                            } else {
                                dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                            }
                        },
                        else => {
                            maple_log.err("Unimplemented GetMediaInformation for target: {any}", .{target});
                        },
                    }
                },
                .BlockRead => {
                    const function_type = data[2];
                    std.debug.assert(function_type == @as(u32, @bitCast(FunctionCodesMask{ .storage = 1 })));
                    const partition: u8 = @truncate((data[3] >> 0) & 0xFF);
                    const phase: u8 = @truncate((data[3] >> 8) & 0xFF);
                    const block_num: u16 = @truncate(((data[3] >> 24) & 0xFF) | ((data[3] >> 8) & 0xFF00));
                    maple_log.warn(termcolor.yellow("BlockRead! Partition: {any} Block: {any}, Phase: {any} (data[3]: {X:0>8})"), .{ partition, block_num, phase, data[3] });

                    const dest = @as([*]u8, @ptrCast(dc.cpu._get_memory(return_addr + 12)))[0..];
                    const payload_size = switch (target) {
                        .VMU => |v| v.block_read(dest, function_type, partition, block_num, phase),
                        else => s: {
                            maple_log.err(termcolor.red("Unimplemented BlockRead for target: {any}"), .{target});
                            break :s 0;
                        },
                    };

                    if (payload_size > 0) {
                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = payload_size }));
                        dc.cpu.write32(return_addr + 4, function_type);
                        dc.cpu.write32(return_addr + 8, data[3]); // Header repeating the location.
                    } else {
                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .FileError, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                    }
                },
                .BlockWrite => {
                    const function_type = data[2];
                    const partition: u8 = @truncate((data[3] >> 0) & 0xFF);
                    const phase: u8 = @truncate((data[3] >> 8) & 0xFF);
                    const block_num: u16 = @truncate(((data[3] >> 24) & 0xFF) | ((data[3] >> 8) & 0xFF00));
                    const write_data = data[4 .. 4 + command.payload_length - 2];

                    switch (target) {
                        .VMU => |v| v.block_write(function_type, partition, phase, block_num, write_data),
                        else => {
                            maple_log.warn(termcolor.yellow("BlockWrite Unimplemented! Recipient: {X:0>2} (Function: {X:0>8}), Partition: {any} Phase: {any} Block: {any}"), .{ command.recipent_address, function_type, partition, phase, block_num });
                        },
                    }

                    dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .Acknowledge, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                },
                .GetLastError => {
                    dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .Acknowledge, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                },
                else => {
                    maple_log.warn(termcolor.yellow("Unimplemented command: {}"), .{command.command});
                    dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                },
            }
        } else {
            dc.cpu.write32(return_addr, 0xFFFFFFFF); // "No connection"
        }

        return 2 + command.payload_length;
    }
};

pub const MapleHost = struct {
    ports: [4]MaplePort = .{
        .{},
        .{},
        .{},
        .{},
    },

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !MapleHost {
        return .{
            .ports = .{
                .{ .main = .{ .Controller = .{} }, .subperipherals = .{ .{ .VMU = try VMU.init(allocator) }, null, null, null, null } },
                .{ .main = .{ .Controller = .{} }, .subperipherals = .{ null, null, null, null, null } },
                .{},
                .{},
            },
            ._allocator = allocator,
        };
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

        defer dc.schedule_interrupt(.{ .EoD_Maple = 1 }, 200); // FIXME: Random delay

        // A transfer can have a maximum of 1024 words.
        while (idx < 1024) {
            const instr: Instruction = @bitCast(data[idx]);
            idx += 1;

            maple_log.debug("    Instruction: {any}", .{instr});

            switch (instr.pattern) {
                .Normal => {
                    idx += self.ports[instr.port_select].handle_command(dc, data[idx..]);
                },
                .NOP, .RESET => {},
                else => {
                    maple_log.warn(termcolor.yellow("[Maple] Unimplemented pattern: {}. Ignoring it, hopefully the payload is empty :D"), .{instr.pattern});
                },
            }

            if (instr.end_flag == 1) {
                return;
            }
        }
    }
};
