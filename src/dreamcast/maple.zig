const std = @import("std");
const termcolor = @import("termcolor");

const log = std.log.scoped(.maple);

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

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        try writer.print("MapleInstruction{{ transfer_length: {X}, pattern: {t}, port_select: {X}, end_flag: {X} }}", .{ self.transfer_length, self.pattern, self.port_select, self.end_flag });
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

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        try writer.print("{t}{{ recipent: {X}, sender: {X}, payload_length: {X} }}", .{ self.command, self.recipent_address, self.sender_address, self.payload_length });
    }
};

pub const FunctionCodesMask = packed struct(u32) {
    _: u16 = 0,

    vibration: u1 = 0,
    pointing: u1 = 0,
    exchange_media: u1 = 0,
    camera: u1 = 0,

    _2: u4 = 0,

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
    pub fn from_u32(value: u32) FunctionCodesMask {
        return @bitCast(value);
    }

    pub const Vibration = @This(){ .vibration = 1 };
    pub const Pointing = @This(){ .pointing = 1 };
    pub const Controller = @This(){ .controller = 1 };
    pub const Storage = @This(){ .storage = 1 };
    pub const Screen = @This(){ .screen = 1 };
    pub const Timer = @This(){ .timer = 1 };
    pub const Keyboard = @This(){ .keyboard = 1 };

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        if (@popCount(self.as_u32()) == 1) {
            inline for (@typeInfo(FunctionCodesMask).@"struct".fields) |field| {
                if (@field(self, field.name) == 1) try writer.writeAll(field.name);
            }
        } else {
            try writer.print("{X}", .{self.as_u32()});
        }
    }
};

pub const DeviceInfoPayload = extern struct {
    FunctionCodesMask: FunctionCodesMask align(1),
    SubFunctionCodesMasks: [3]u32 align(1),
    RegionCode: u8 align(1) = 0xFF,
    ConnectionDirectionCode: u8 align(1) = 0,
    DescriptionString: [31]u8 align(1) = @splat(0),
    ProducerString: [60]u8 align(1) = "Produced By or Under License From SEGA ENTERPRISES,LTD.     ".*,
    StandbyConsumption: u16 align(1) = 0,
    MaximumConsumption: u16 align(1) = 0,
    // Possible extension
};

pub const Controller = @import("maple/controller.zig");
pub const Keyboard = @import("maple/keyboard.zig");

pub const VMU = @import("maple/vmu.zig");
pub const VibrationPack = @import("maple/vibration_pack.zig");

const Peripheral = union(enum) {
    Controller: Controller,
    Keyboard: Keyboard,
    VMU: VMU,
    VibrationPack: VibrationPack,

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        switch (self.*) {
            .VMU => |*v| v.deinit(allocator),
            inline else => {},
        }
    }

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
            inline .VMU, .VibrationPack => |*v| v.block_read(dest, function, partition, block_num, phase),
            else => s: {
                log.err(termcolor.red("Unimplemented BlockRead for target: {t}"), .{self.tag()});
                break :s 0;
            },
        };
    }

    pub fn block_write(self: *@This(), function: u32, partition: u8, phase: u8, block_num: u16, data: []const u32) u8 {
        switch (self.*) {
            inline .VMU, .VibrationPack => |*v| return v.block_write(function, partition, phase, block_num, data),
            else => log.warn(termcolor.yellow("BlockWrite Unimplemented for target: {t}"), .{self.tag()}),
        }
        return 0;
    }
};

const MaplePort = struct {
    main: ?Peripheral = null,
    subperipherals: [5]?Peripheral = @splat(null),

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        if (self.main) |*p|
            p.deinit(allocator);
        for (&self.subperipherals) |*sub| {
            if (sub.*) |*p|
                p.deinit(allocator);
        }

        self.* = .{};
    }

    /// Returns the number of 32bytes words transferred to the host.
    pub fn handle_command(self: *@This(), dc: *Dreamcast, data: [*]u32) u32 {
        const return_addr = data[0];
        std.debug.assert(return_addr >= 0x0C000000 and return_addr < 0x10000000);
        const command: CommandWord = @bitCast(data[1]);
        const function_type = data[2];
        log.debug("  Dest: {X:0>8}, Command: {f}, Function: {f}", .{ return_addr, command, @as(FunctionCodesMask, @bitCast(function_type)) });

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
                    const ptr = dc._get_memory(return_addr + 4);
                    @memcpy(@as([*]u8, @ptrCast(ptr))[0..@sizeOf(DeviceInfoPayload)], std.mem.asBytes(&identity));
                    return 1 + @sizeOf(DeviceInfoPayload) / 4;
                },
                .GetCondition => {
                    switch (target.*) {
                        inline .Controller, .Keyboard, .VibrationPack => |*c| {
                            std.debug.assert(command.payload_length == 1);
                            const condition = c.get_condition(function_type);
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .DataTransfer, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = @intCast(condition.len) }));
                            const ptr: [*]u32 = @ptrCast(@alignCast(dc._get_memory(return_addr + 4)));
                            @memcpy(ptr[0..condition.len], &condition);
                            return 1 + condition.len;
                        },
                        else => {
                            log.err(termcolor.red("Unimplemented GetCondition for target: {t}"), .{target.tag()});
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                            return 1;
                        },
                    }
                },
                .GetMediaInformation => {
                    std.debug.assert(command.payload_length == 2);
                    const partition_number: u8 = @truncate(data[3] >> 24);
                    log.warn(termcolor.yellow("  GetMediaInformation: Function: {f}, Partition number: {d}"), .{ @as(FunctionCodesMask, @bitCast(function_type)), partition_number });

                    switch (target.*) {
                        inline .VMU, .VibrationPack => |*v| {
                            const dest = @as([*]u8, @ptrCast(dc._get_memory(return_addr + 8)))[0..];
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
                            log.err(termcolor.red("Unimplemented GetMediaInformation for target: {t}"), .{target.tag()});
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                            return 1;
                        },
                    }
                },
                .BlockRead => {
                    const partition: u8 = @truncate((data[3] >> 0) & 0xFF);
                    const phase: u8 = @truncate((data[3] >> 8) & 0xFF);
                    const block_num: u16 = @truncate(((data[3] >> 24) & 0xFF) | ((data[3] >> 8) & 0xFF00));
                    log.warn(termcolor.yellow("BlockRead({f}) Partition: {d}, Block: {d}, Phase: {d} ({X:0>8})"), .{ @as(FunctionCodesMask, @bitCast(function_type)), partition, block_num, phase, data[3] });

                    const dest = @as([*]u8, @ptrCast(dc._get_memory(return_addr + 12)))[0..];
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
                        inline .VMU, .VibrationPack => |*vmu| {
                            vmu.set_condition(function_type, data[3..][0 .. command.payload_length - 1]);
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .Acknowledge, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                            return 1;
                        },
                        else => {
                            log.err(termcolor.red("Unimplemented SetCondition for target: {t}"), .{target.tag()});
                            dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                            return 1;
                        },
                    }
                    return 1;
                },
                else => {
                    log.warn(termcolor.yellow("Unimplemented command: {s} ({X}, {any})"), .{ std.enums.tagName(Command, command.command) orelse "Unknown", @intFromEnum(command.command), data[0..4] });
                    dc.cpu.write_physical(u32, return_addr, @bitCast(CommandWord{ .command = .FunctionCodeNotSupported, .sender_address = sender_address, .recipent_address = recipent_address, .payload_length = 0 }));
                    return 1;
                },
            }
        } else {
            dc.cpu.write_physical(u32, return_addr, 0xFFFFFFFF); // "No connection"
            return 1;
        }
    }

    pub fn serialize(_: @This(), _: *std.Io.Writer) !usize {
        return 0;
    }
    pub fn deserialize(_: @This(), _: anytype) !void {}
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
                if (maybe_peripheral.*) |*peripheral|
                    peripheral.deinit(self._allocator);
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
            log.debug("    Transferred {d} words, scheduled interrupt in {d} cycles", .{ transferred_words, cycles });
        }

        // A transfer can have a maximum of 1024 words.
        while (idx < 1024) {
            const instr: Instruction = @bitCast(data[idx]);
            idx += 1;

            log.debug("{f}", .{instr});

            switch (instr.pattern) {
                .Normal => {
                    transferred_words += self.ports[instr.port_select].handle_command(dc, data[idx..]);
                    idx += instr.transfer_length + 2;
                },
                .NOP, .RESET => {},
                else => log.warn(termcolor.yellow("[Maple] Unimplemented pattern: {}. Ignoring it, hopefully the payload is empty :D"), .{instr.pattern}),
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

    pub fn serialize(self: @This(), writer: *std.Io.Writer) !usize {
        var bytes: usize = 0;
        for (&self.ports) |port| {
            bytes += try port.serialize(writer);
        }
        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: *std.Io.Reader) !void {
        for (&self.ports) |*port| {
            try port.deserialize(reader);
        }
    }
};
