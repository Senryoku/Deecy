const std = @import("std");
const termcolor = @import("termcolor.zig");

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

const InputCapabilities = packed struct(u32) {
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

    _: u10 = 0,
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
    RegionCode: u8 align(1) = 0,
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

const Peripheral = struct {
    capabilities: FunctionCodesMask = .{},
    subcapabilities: [3]FunctionCodesMask = .{.{}} ** 3,
};

const MaplePort = struct {
    main: ?Peripheral = null,
    subperipherals: [5]?Peripheral = .{null} ** 5,

    // FIXME: Not every peripheral is a controller :)
    buttons: ControllerButtons = .{},
    pub fn press_buttons(self: *@This(), buttons: ControllerButtons) void {
        self.buttons = @bitCast(@as(u16, @bitCast(self.buttons)) & @as(u16, @bitCast(buttons)));
    }
    pub fn release_buttons(self: *@This(), buttons: ControllerButtons) void {
        self.buttons = @bitCast(@as(u16, @bitCast(self.buttons)) | ~@as(u16, @bitCast(buttons)));
    }

    pub fn handle_command(self: *@This(), dc: *Dreamcast, data: [*]u32) u32 {
        const return_addr = data[0];
        const ram_addr = return_addr - 0x0C000000;

        const command: CommandWord = @bitCast(data[1]);

        std.log.debug("    Command: {any}", .{command});

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
                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DeviceInfo, .sender_address = sender_address, .recipent_address = command.sender_address, .payload_length = @sizeOf(DeviceInfoPayload) / 4 }));
                        @as(*DeviceInfoPayload, @ptrCast(&dc.ram[ram_addr + 4])).* = .{
                            .FunctionCodesMask = target.?.capabilities,
                            .SubFunctionCodesMasks = target.?.subcapabilities,
                        };
                    },
                    .GetCondition => {
                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = sender_address, .recipent_address = command.sender_address, .payload_length = 3 }));
                        // TODO: Write some actual input data!
                        dc.cpu.write32(return_addr + 4, @bitCast(self.main.?.capabilities));
                        dc.cpu.write16(return_addr + 8, @bitCast(self.buttons));
                        dc.cpu.write16(return_addr + 10, 0xFFFF);
                        dc.cpu.write32(return_addr + 12, 0xFFFFFFFF);
                    },
                    else => {
                        std.log.warn(termcolor.yellow("[Maple] Unimplemented command: {}"), .{command.command});
                        dc.cpu.write32(return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = command.recipent_address, .recipent_address = command.sender_address, .payload_length = 0 }));
                    },
                }
            }
        }

        return 2 + command.payload_length;
    }
};

pub const MapleHost = struct {
    ports: [4]MaplePort = .{ .{ .main = .{ .capabilities = .{ .controller = 1 }, .subcapabilities = .{ @bitCast(StandardControllerCapabilities), .{}, .{} } }, .subperipherals = .{ .{ .capabilities = VMUCapabilities }, null, null, null, null } }, .{}, .{}, .{} },

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

            std.log.debug("    Instruction: {any}", .{instr});

            switch (instr.pattern) {
                .Normal => {
                    idx += self.ports[instr.port_select].handle_command(dc, data[idx..]);
                },
                .NOP, .RESET => {},
                else => {
                    std.log.warn(termcolor.yellow("[Maple] Unimplemented pattern: {}. Ignoring it, hopefully the payload is empty :D"), .{instr.pattern});
                },
            }

            if (instr.end_flag == 1) {
                return;
            }
        }
    }
};
