const std = @import("std");
const termcolor = @import("termcolor.zig");

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
    Acknowledge = 0x07,
    DataTransfer = 0x08,
    GetCondition = 0x09,
    GetMemoryInformation = 0x0A,
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
    SubFunctionCodesMasks: [3]FunctionCodesMask align(1),
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
    subcapabilities: [3]FunctionCodesMask = .{ @bitCast(StandardControllerCapabilities), .{}, .{} },

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

const GetMemoryInformationResponse = packed struct {
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

const VMU = struct {
    capabilities: FunctionCodesMask = VMUCapabilities,
    subcapabilities: [3]FunctionCodesMask = .{
        @bitCast(@as(u32, 0b01000000_00111111_01111110_01111110)), // FIXME: This is wrong (at least because of endianess, maybe more)
        @bitCast(@as(u32, 0b00000000_00010000_00000101_00000000)),
        @bitCast(@as(u32, 0b00000000_01000001_00001111_00000000)),
    },

    pub fn get_identity(self: *const @This()) [@sizeOf(DeviceInfoPayload) / @sizeOf(u32)]u32 {
        var r: [@sizeOf(DeviceInfoPayload) / @sizeOf(u32)]u32 = undefined;
        @as(*DeviceInfoPayload, @ptrCast(&r)).* = .{
            .FunctionCodesMask = self.capabilities,
            .SubFunctionCodesMasks = self.subcapabilities,
        };
        return r;
    }

    pub fn get_memory_info(self: *const @This()) GetMemoryInformationResponse {
        _ = self;

        return GetMemoryInformationResponse{
            .total_size = 0x00FF,
            .partition_number = 0x0000,
            .system_area_block_number = 0x00FF,
            .fat_area_block_number = 0x00FE,
            .number_of_fat_area_blocks = 0x0001,
            .file_information_block_number = 0x00FD,
            .number_of_file_information_blocks = 0x000D,
            .volume_icon = 0,
            .save_area_block_number = 0x00C8,
            .number_of_save_area_blocks = 0x001F,
        };
    }

    pub fn get_block_data(self: *const @This(), partition: u8, phase: u8, block_num: u8) [512]u8 {
        _ = self;
        _ = partition;
        _ = phase;
        _ = block_num;

        return std.mem.zeroes([512]u8);
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

        if (self.main == null) {
            dc.cpu.write32(return_addr, 0xFFFFFFFF); // "No connection"
        } else {
            const target = switch (command.recipent_address & 0b11111) {
                0 => self.main.?,
                else => self.subperipherals[@ctz(command.recipent_address)],
            };
            if (target == null) {
                dc.cpu.write32(return_addr, 0xFFFFFFFF); // "No connection"
            } else {
                switch (command.command) {
                    .DeviceInfoRequest => {
                        const identity = switch (target.?) {
                            .Controller => |c| c.get_identity(),
                            .VMU => |v| v.get_identity(),
                        };
                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DeviceInfo, .sender_address = sender_address, .recipent_address = command.sender_address, .payload_length = @intCast(identity.len) }));
                        const ptr: [*]u32 = @alignCast(@ptrCast(dc.cpu._get_memory(return_addr + 4)));
                        @memcpy(ptr[0..identity.len], &identity);
                    },
                    .GetCondition => {
                        switch (target.?) {
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
                            //    maple_log.err("Unimplemented GetCondition for target: {any}", .{target.?});
                            //    @panic("[Maple] Unimplemented GetCondition for target");
                            //},
                        }
                    },
                    .GetMemoryInformation => { // Named GetMediaInfo in the documentation
                        std.debug.assert(command.payload_length == 2);

                        const function_type = data[2];
                        const partition_number = data[3];

                        maple_log.warn(termcolor.yellow("  GetMemoryInformation: Function type: {X:0>8} Partition number: {any}"), .{ function_type, partition_number });

                        switch ((target.?)) {
                            .VMU => |v| {
                                dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0x07 }));
                                dc.cpu.write32(return_addr + 4, @bitCast(FunctionCodesMask{ .storage = 1 }));
                                @memcpy(@as([*]u8, @ptrCast(dc.cpu._get_memory(return_addr + 8)))[0..@sizeOf(GetMemoryInformationResponse)], @as([*]const u8, @ptrCast(&v.get_memory_info()))[0..@sizeOf(GetMemoryInformationResponse)]);
                            },
                            else => {
                                maple_log.err("Unimplemented GetMemoryInformation for target: {any}", .{target.?});
                            },
                        }
                    },
                    .BlockRead => {
                        const function_type = data[2];
                        std.debug.assert(function_type == @as(u32, @bitCast(FunctionCodesMask{ .storage = 1 })));
                        const partition: u8 = @truncate(data[3] >> 24 & 0xFF);
                        const phase: u8 = @truncate(data[3] >> 16 & 0xFF);
                        const block_num: u8 = @truncate(data[3] >> 8 & 0xFF);
                        maple_log.warn(termcolor.yellow("BlockRead Unimplemented! Partition: {any} Phase: {any} Block: {any}"), .{ partition, phase, block_num });

                        const response = switch (target.?) {
                            .VMU => |v| v.get_block_data(partition, phase, block_num),
                            else => {
                                @panic("Unimplemented BlockRead for specified target.");
                            },
                        };

                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 1 + response.len / 4 }));
                        dc.cpu.write32(return_addr + 4, function_type);
                        const ptr: [*]u8 = @alignCast(@ptrCast(dc.cpu._get_memory(return_addr + 8)));
                        @memcpy(ptr[0..response.len], response[0..response.len]);
                    },
                    .BlockWrite => {
                        const function_type = data[2];
                        const partition: u8 = @truncate(data[3] >> 24 & 0xFF);
                        const phase: u8 = @truncate(data[3] >> 16 & 0xFF);
                        const block_num: u8 = @truncate(data[3] >> 8 & 0xFF);
                        const write_data = data[4 .. 4 + command.payload_length - 2];
                        _ = write_data;
                        maple_log.warn(termcolor.yellow("BlockWrite Unimplemented! Recipient: {X:0>2} (Function: {X:0>8}), Partition: {any} Phase: {any} Block: {any}"), .{ command.recipent_address, function_type, partition, phase, block_num });

                        // TODO!

                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .Acknowledge, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                    },
                    else => {
                        maple_log.warn(termcolor.yellow("Unimplemented command: {}"), .{command.command});
                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                    },
                }
            }
        }

        return 2 + command.payload_length;
    }
};

pub const MapleHost = struct {
    ports: [4]MaplePort = .{
        .{ .main = .{ .Controller = .{} }, .subperipherals = .{ .{ .VMU = .{} }, null, null, null, null } },
        .{},
        .{},
        .{},
    },

    pub fn init() MapleHost {
        return .{};
    }

    pub fn deinit(self: *MapleHost) void {
        _ = self;
    }

    pub fn transfer(self: *MapleHost, dc: *Dreamcast, data: [*]u32) void {
        var idx: u32 = 0;

        defer dc.raise_normal_interrupt(.{ .EoD_Maple = 1 });

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
