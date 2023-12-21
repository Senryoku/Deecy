const std = @import("std");
const termcolor = @import("termcolor.zig");

const SH4 = @import("sh4.zig").SH4;

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
    Shutdow = 0x04,
    DeviceInfo = 0x05,
    ExtendedDevice = 0x06,
    Acknowledge = 0x07,
    DataTransfer = 0x08,
    GetCondition = 0x09,
    GetMemory = 0x0A,
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
};

const CommandWord = packed struct(u32) {
    command: Command,
    recipent_address: u8,
    sender_address: u8,
    payload_length: u8, // In 32-bit words.
};

// Note: I'm not sure of the order of fields here, the link being big endian (according to some sources, not all :D)...
const FunctionCodesMask = packed struct(u32) {
    // Input capabilities, should be its own thing but we'll see when we get there.
    c: u1 = 0,
    b: u1 = 0,
    a: u1 = 0,
    start: u1 = 0,
    up: u1 = 0,
    down: u1 = 0,
    left: u1 = 0,
    right: u1 = 0,
    z: u1 = 0,
    y: u1 = 0,
    x: u1 = 0,
    d: u1 = 0,
    up2: u1 = 0,
    down2: u1 = 0,
    left2: u1 = 0,
    right2: u1 = 0,
    analogRtrigger: u1 = 0,
    analogLtrigger: u1 = 0,
    analogHorizontal: u1 = 0,
    analogVertical: u1 = 0,
    analogHorizontal2: u1 = 0,
    analogVertical2: u1 = 0,

    // Generic,

    controller: u1 = 0,
    storage: u1 = 0,
    screen: u1 = 0,
    timer: u1 = 0,
    audio_input: u1 = 0,
    argun: u1 = 0,
    keyboard: u1 = 0,
    gun: u1 = 0,
    vibration: u1 = 0,
    mouse: u1 = 0,
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
    SubFunctionFunctionCodesMasks: [3]FunctionCodesMask align(1), // For sub-peripherals?
    RegionCode: u8 align(1) = 0,
    ConnectionDirectionCode: u8 align(1) = 0,
    DescriptionString: [30]u8 align(1) = .{0} ** 30,
    ProducerString: [60]u8 align(1) = .{0} ** 60,
    StandbyConsumption: u16 align(1) = 0,
    MaximumConsumption: u16 align(1) = 0,
    // Possible extension
};

// 0 for pressed, 1 for released
const ControllerData = packed struct {
    _0: u1 = 1,

    b: u1,
    a: u1,
    start: u1,
    up: u1,
    down: u1,
    left: u1,
    right: u1,

    _1: u1 = 1,

    y: u1,
    x: u1,

    _2: u5 = 0b11111,

    axis_1: u8, // Right analog trigger?
    axis_2: u8, // Left analog trigger?
    axis_3: u8, // Analog stick X?
    axis_4: u8, // Analog stick Y?

    _3: u8 = 0b10000000, // axis_5, not in standard controllers
    _4: u8 = 0b10000000, // axis_6, not in standard controllers
};

const StandardControllerCapabilities: FunctionCodesMask = .{
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

const MaplePort = struct {
    connected: bool = false,
    sub_peripheral: [3]bool = .{false} ** 3,
};

pub const MapleHost = struct {
    ports: [4]MaplePort = .{MaplePort{}} ** 4,

    pub fn init() MapleHost {
        return .{};
    }

    pub fn deinit(self: *MapleHost) void {
        _ = self;
    }

    pub fn transfer(self: *MapleHost, cpu: *SH4, data: [*]u32) void {
        _ = self;
        var idx: u32 = 0;

        defer cpu.raise_normal_interrupt(.{ .EoD_Maple = 1 });

        // A frame can have a maximum of 1024 words.
        while (idx < 1024) {
            const instr: Instruction = @bitCast(data[idx]);
            idx += 1;

            // TODO: Switch on port and send the command to the port rather than handling it here, when we actually support some peripherals.

            const return_addr = data[idx];
            const ram_addr = return_addr - 0x0C000000;
            idx += 1;

            const command: CommandWord = @bitCast(data[idx]);
            idx += 1;

            std.debug.print("    Instruction: {any}\n", .{instr});
            std.debug.print("    Command: {any}\n", .{command});

            // Note: The sender address should also include the sub-peripheral bit when appropriate.

            switch (command.command) {
                .DeviceInfoRequest => {
                    cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DeviceInfo, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = @sizeOf(DeviceInfoPayload) / 4 }));
                    @as(*DeviceInfoPayload, @ptrCast(&cpu.ram[ram_addr + 4])).* = .{
                        .FunctionCodesMask = StandardControllerCapabilities,
                        .SubFunctionFunctionCodesMasks = .{ .{ .controller = 1 }, .{}, .{} },
                    };
                },
                else => {
                    std.debug.print(termcolor.yellow("[Maple] Unimplemented command: {}\n"), .{command.command});
                    cpu.write32(return_addr, 0xFFFFFFFF); // "No connection"
                    //cpu.write32(return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                },
            }
            std.debug.print("    Ret Addr: 0x{X:0>8}\n", .{return_addr});
            std.debug.print("      {X:0>2} {X:0>2} {X:0>2} {X:0>2}\n", .{ cpu.read8(return_addr + 0), cpu.read8(return_addr + 1), cpu.read8(return_addr + 2), cpu.read8(return_addr + 3) });
            std.debug.print("      {X:0>2} {X:0>2} {X:0>2} {X:0>2}\n", .{ cpu.read8(return_addr + 4), cpu.read8(return_addr + 5), cpu.read8(return_addr + 6), cpu.read8(return_addr + 7) });
            std.debug.print("      {X:0>2} {X:0>2} {X:0>2} {X:0>2}\n", .{ cpu.read8(return_addr + 8), cpu.read8(return_addr + 9), cpu.read8(return_addr + 10), cpu.read8(return_addr + 11) });

            idx += command.payload_length;

            if (instr.end_flag == 1) {
                return;
            }
        }
    }
};
