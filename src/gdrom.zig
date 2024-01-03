const std = @import("std");
const termcolor = @import("termcolor.zig");

const gdrom_log = std.log.scoped(.gdrom);

const GDI = @import("gdi.zig").GDI;
const SH4 = @import("sh4.zig").SH4;
const Dreamcast = @import("dreamcast.zig").Dreamcast;

const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;

// HLE

pub const GDROMStatus = enum(u32) {
    Busy = 0,
    Paused = 1,
    Standby = 2,
    Playing = 3,
    Seeking = 4,
    Scanning = 5,
    Open = 6,
    Empty = 7,
};

pub const GDROMCommand = enum(u32) {
    PIORead = 16,
    DMARead = 17,
    GetTOC = 18,
    GetTOC2 = 19,
    Play = 20,
    Play2 = 21,
    Pause = 22,
    Release = 23,
    Init = 24,
    Seek = 27,
    Read = 28,
    ReqMode = 30,
    SetMode = 31,
    Stop = 33,
    GetSCD = 34,
    GetSession = 35,
    GetVersion = 40,
    _,
};

// According to KallistiOS
const CDAudioStatus = enum(u32) {
    Invalid = 0x00,
    Playing = 0x11,
    Paused = 0x12,
    Ended = 0x13,
    Error = 0x14,
    NoInfo = 0x15,
};

// LLE Commands
const Command = enum(u32) {
    NOP = 0x00,
    SoftReset = 0x08,
    ExecuteDeviceDiagnostic = 0x90,
    PacketCommand = 0xA0,
    IdentifyDevice = 0xA1,
    IdentifyDevice2 = 0xEC, // NOTE: This is not in the documentation, but is used byt a KallistiOS example.
    SetFeatures = 0xEF,
    _,
};

const StatusRegister = packed struct(u8) {
    check: u1 = 0,
    _r: u1 = 0,
    corr: u1 = 0,
    drq: u1 = 0,
    dsc: u1 = 0,
    df: u1 = 0,
    drdy: u1 = 1,
    bsy: u1 = 0,
};

const SenseKey = enum(u4) {
    NoSense = 0,
    RecoveredError = 1,
    NotReady = 2,
    MediumError = 3,
    HardwareError = 4,
    IllegalRequest = 5,
    UnitAttention = 6,
    DataProtect = 7,
    AbortedCommand = 0xB,

    _,
};

const ErrorRegister = packed struct(u8) {
    ili: u1 = 0, // Command length is not correct
    eomf: u1 = 0, // Media end was detected
    abrt: u1 = 0, // Drive is not ready and command was made invalid (ATA level).
    mcr: u1 = 0, // Media change was requested and media have been ejected (ATA level).
    sense_key: SenseKey = .NoSense,
};

const ControlRegister = packed struct(u8) {
    _r0: u1 = 0,

    nien: u1 = 0, // This bit sets interrupts to the host. When "0," the interrupt is enabled; when "1," the interrupt is disabled.
    srst: u1 = 0, // This is the bit that the SH4 (i.e., the "system" or the "host") sets in order to initiate a software reset. This protocol is not used, however. To initiate a software reset, use the software reset defined by ATAPI

    _r1: u1 = 1,

    _: u4 = 0,
};

const InterruptReasonRegister = packed struct(u8) {
    cod: u1 = 0, // When "0," this field indicates data; when "1," this field indicates a command
    io: u1 = 0, // When "0," this field indicates the direction of transfer is from the host to the device; when "1," this field indicates the direction of transfer is from the device to the host.

    _: u6 = 0,
};

const ScheduledEvent = struct {
    cycles: u32,
    status: StatusRegister,
};

pub const GDROM = struct {
    disk: GDI = .{},

    // LLE
    status_register: StatusRegister = .{},
    control_register: ControlRegister = .{},
    error_register: ErrorRegister = .{},
    interrupt_reason_register: InterruptReasonRegister = .{},
    byte_count: u16 = 0,

    data_queue: std.fifo.LinearFifo(u8, .{ .Static = 1024 }),

    packet_command_idx: u8 = 0,
    packet_command: [12]u8 = undefined,

    scheduled_event: ?ScheduledEvent = null,

    // HLE

    hle_status: GDROMStatus = GDROMStatus.Standby,
    hle_command: GDROMCommand = undefined,
    hle_params: [4]u32 = undefined,
    hle_result: [4]u32 = undefined,

    _next_command_id: u32 = 1,
    _current_command_id: u32 = 0,

    pub fn init(_: std.mem.Allocator) GDROM {
        var gdrom = GDROM{
            .data_queue = std.fifo.LinearFifo(u8, .{ .Static = 1024 }).init(),
        };
        gdrom.reinit();
        return gdrom;
    }

    pub fn reinit(self: *@This()) void {
        self.hle_status = GDROMStatus.Standby;
        self.hle_command = undefined;
        @memset(&self.hle_params, 0);
        @memset(&self.hle_result, 0);
    }

    pub fn deinit(self: *@This()) void {
        self.data_queue.deinit();
    }

    pub fn update(self: *@This(), dc: *Dreamcast, cycles: u32) void {
        if (self.scheduled_event) |*event| {
            if (event.cycles < cycles) {
                self.status_register = event.status;
                if (event.status.drq == 1 and self.control_register.nien == 0) {
                    dc.raise_external_interrupt(.{ .GDRom = 1 });
                }
                self.scheduled_event = null;
            } else {
                event.cycles -= cycles;
            }
        }
    }

    pub fn read_register(self: *@This(), comptime T: type, addr: u32) T {
        std.debug.assert(addr >= 0x005F7000 and addr <= 0x005F709C);
        switch (@as(MemoryRegister, @enumFromInt(addr))) {
            .GD_AlternateStatus_DeviceControl => {
                gdrom_log.debug("  Read Alternate Status @{X:0>8} = {any}", .{ addr, self.status_register });
                // NOTE: Alternate status reads do NOT clear the pending interrupt signal.
                return @intCast(@as(u8, @bitCast(self.status_register)));
            },
            .GD_Status_Command => {
                gdrom_log.debug("  Read Status @{X:0>8} = {any}", .{ addr, self.status_register });
                const val: T = @intCast(@as(u8, @bitCast(self.status_register)));
                // Clear the pending interrupt signal.
                self.status_register.drq = 0;
                return val;
            },
            .GD_InterruptReason_SectorCount => {
                gdrom_log.debug("  Read Interrupt Reason @{X:0>8} = 0x{X:0>8}", .{ addr, 0b10 });
                return @intCast(@as(u8, @bitCast(self.interrupt_reason_register)));
            },
            .GD_ByteCountLow => {
                gdrom_log.debug("  Read Byte Count Low @{X:0>8} = 0x{X:0>8}", .{ addr, self.byte_count });
                return @as(u8, @truncate(self.byte_count));
            },
            .GD_ByteCountHigh => {
                gdrom_log.debug("  Read Byte Count High @{X:0>8} = 0x{X:0>8}", .{ addr, self.byte_count });
                return @as(u8, @truncate(self.byte_count >> @intCast(8)));
            },
            .GD_DriveSelect => {
                gdrom_log.debug("  Read Drive Select @{X:0>8} = 0x{X:0>8}", .{ addr, 0b10100000 });
                return 0b10100000;
            },
            else => {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Read to @{X:0>8}"), .{addr});
                return 0;
            },
        }
    }

    pub fn write_register(self: *@This(), comptime T: type, addr: u32, value: T) void {
        std.debug.assert(addr >= 0x005F7000 and addr <= 0x005F709C);
        gdrom_log.debug("  write_register @{X:0>8} = 0x{X:0>8}", .{ addr, value });
        switch (@as(MemoryRegister, @enumFromInt(addr))) {
            .GD_Status_Command => {
                self.status_register.check = 0;
                self.error_register = .{};
                switch (@as(Command, @enumFromInt(value))) {
                    .SoftReset => {
                        gdrom_log.debug("  Command: SoftReset", .{});
                    },
                    .ExecuteDeviceDiagnostic => {
                        gdrom_log.debug("  Command: ExecuteDeviceDiagnostic", .{});
                        self.status_register.bsy = 1;
                        self.status_register.drq = 0;
                        // The device clears the BSY bit and initiates an interrup
                        self.scheduled_event = .{
                            .cycles = 200, // FIXME: Random value
                            .status = .{ .bsy = 0, .drq = 1 },
                        };
                        // Always report no errors.
                        self.error_register = .{};
                    },
                    .PacketCommand => {
                        gdrom_log.warn(termcolor.yellow("  Command: PacketCommand - TODO! See cdif131e.pdf"), .{});
                        self.status_register.bsy = 1;
                        self.interrupt_reason_register.cod = 1;
                        self.interrupt_reason_register.io = 0;

                        self.packet_command_idx = 0;

                        self.scheduled_event = .{
                            .cycles = 200, // FIXME: Random value
                            .status = .{ .bsy = 0, .drq = 1 },
                        };
                    },
                    .IdentifyDevice => {
                        gdrom_log.warn(termcolor.yellow("  Command: IdentifyDevice - TODO!"), .{});
                        self.status_register.bsy = 1;
                        self.status_register.drq = 0;

                        // 0x00 Manufacturer's ID
                        self.data_queue.writeAssumeCapacity(&[_]u8{0x0});
                        // 0x01 Model ID
                        self.data_queue.writeAssumeCapacity(&[_]u8{0x0});
                        // 0x02 Version ID
                        self.data_queue.writeAssumeCapacity(&[_]u8{0x0});
                        // 0x03 - 0x0F Reserved
                        for (0..0x10 - 0x03) |_| {
                            self.data_queue.writeAssumeCapacity(&[_]u8{0x0});
                        }
                        // 0x10 - 0x1F Manufacturer's name (16 ASCII characters)
                        self.data_queue.writeAssumeCapacity("            SEGA");
                        // 0x20 - 0x2F Model name (16 ASCII characters)
                        self.data_queue.writeAssumeCapacity("           DUNNO");
                        // 0x30 - 0x3F Firmware version (16 ASCII characters)
                        self.data_queue.writeAssumeCapacity("           DUNNO");
                        // 0x40 - 0x4F Reserved
                        self.data_queue.writeAssumeCapacity("                ");

                        self.scheduled_event = .{
                            .cycles = 200, // FIXME: Random value
                            .status = .{ .bsy = 0, .drq = 1 },
                        };
                    },
                    .IdentifyDevice2 => {
                        // This command is issued by KallistiOS (g1_ata_scan), apparently to "check for a slave device".
                        // My guess is that this does nothing on a stock DC, but I have no way to test it...
                        gdrom_log.warn(termcolor.yellow("  Unhandled GDROM command 'IdentifyDevice2'."), .{});
                        self.nop();
                    },
                    .SetFeatures => {
                        gdrom_log.warn(termcolor.yellow("  Command: SetFeatures - TODO!"), .{});
                    },
                    .NOP => {
                        self.nop();
                    },
                    _ => {
                        gdrom_log.warn(termcolor.yellow("  Unhandled GDROM command {X:0>8}."), .{value});
                        self.nop();
                    },
                }
            },
            .GD_InterruptReason_SectorCount => {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to SectorCount @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
            },
            .GD_SectorNumber => {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to SectorNumber @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
            },
            .GD_ByteCountLow => {
                self.byte_count = (self.byte_count & 0xFF00) | value;
            },
            .GD_ByteCountHigh => {
                self.byte_count = (self.byte_count & 0x00FF) | (@as(u16, @intCast(value)) << @intCast(8));
            },
            .GD_DriveSelect => {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to DriveSelect @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
            },
            .GD_Data => {
                gdrom_log.debug("  Data Write @{X:0>8} = 0x{X:0>8}", .{ addr, value });
                if (T == u8) {
                    self.packet_command[self.packet_command_idx] = value;
                    self.packet_command_idx += 1;
                } else if (T == u16) {
                    self.packet_command[self.packet_command_idx] = @truncate(value >> 8);
                    self.packet_command[self.packet_command_idx + 1] = @truncate(value);
                    self.packet_command_idx += 2;
                } else {
                    std.debug.assert(false);
                }
                if (self.packet_command_idx >= 12) {
                    self.status_register.bsy = 1;
                    self.status_register.drq = 0;
                    gdrom_log.warn(termcolor.yellow("  TODO: Start DMA!"), .{});
                }
            },
            else => {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
            },
        }
    }

    fn nop(self: *@This()) void {
        //   Setting "abort" in the error register
        self.error_register.abrt = 1;
        //   Setting "error" in the status register
        self.status_register.check = 1;
        //   Clearing BUSY in the status register
        self.status_register.bsy = 0;
        // TODO:
        //   Asserting the INTRQ signal
    }

    // HLE - Used by the BIOS syscalls

    pub fn send_command(self: *@This(), command_code: u32, params: [4]u32) u32 {
        if (self.hle_status != GDROMStatus.Standby) return 0;

        self._current_command_id = self._next_command_id;
        self._next_command_id +%= 1;
        if (self._next_command_id == 0) self._next_command_id = 1;

        self.hle_status = GDROMStatus.Busy;
        self.hle_command = @enumFromInt(command_code);
        self.hle_params = params;
        @memset(&self.hle_result, 0);

        return self._current_command_id;
    }

    pub fn mainloop(self: *@This(), dc: *Dreamcast) void {
        if (self.hle_status != GDROMStatus.Busy) {
            gdrom_log.debug("  GDROM Mainloop - No command queued", .{});
            return;
        }

        gdrom_log.info("  GDROM Mainloop - {s}", .{std.enums.tagName(GDROMCommand, self.hle_command) orelse "Unknown"});

        switch (self.hle_command) {
            GDROMCommand.DMARead, GDROMCommand.PIORead => {
                const lba = self.hle_params[0];
                const size = self.hle_params[1];
                const dest = self.hle_params[2] & 0x1FFFFFFF;

                gdrom_log.info("    GDROM {s} sector={d} size={d} destination=0x{X:0>8}", .{ @tagName(self.hle_command), lba, size, dest });
                const byte_size = 2048 * size;
                const read = self.disk.load_sectors(lba, byte_size, @as([*]u8, @ptrCast(dc.cpu._get_memory(dest)))[0..byte_size]);

                dc.raise_normal_interrupt(.{ .EoD_GDROM = 1 });
                dc.raise_external_interrupt(.{ .GDRom = 1 });

                self.hle_result = .{ 0, 0, read, 0 };

                self.hle_status = GDROMStatus.Standby;
            },
            GDROMCommand.Init => {
                gdrom_log.warn("    GDROM Command Init : TODO (Reset some stuff?)", .{});
                self.hle_status = GDROMStatus.Standby;
            },
            GDROMCommand.GetVersion => {
                const dest = self.hle_params[0];
                const version = "GDC Version 1.10 1999-03-31";
                for (0..version.len) |i| {
                    dc.cpu.write8(@intCast(dest + i), version[i]);
                }
                dc.cpu.write8(@intCast(dest + version.len), 0x2);
                self.hle_status = GDROMStatus.Standby;
            },
            GDROMCommand.ReqMode => {
                const dest = self.hle_params[0];
                gdrom_log.info("    GDROM ReqMode  dest=0x{X:0>8}", .{dest});
                dc.cpu.write32(dest + 0, 0); // Speed
                dc.cpu.write32(dest + 4, 0x00B4); // Standby
                dc.cpu.write32(dest + 8, 0x19); // Read Flags
                dc.cpu.write32(dest + 12, 0x08); // Read retry
                self.hle_result = .{ 0, 0, 0xA, 0 };
                self.hle_status = GDROMStatus.Standby;
            },
            GDROMCommand.SetMode => {
                gdrom_log.warn("    GDROM SetMode: TODO", .{});
                self.hle_status = GDROMStatus.Standby;
            },
            GDROMCommand.GetTOC2 => {
                const area = self.hle_params[0];
                const dest = self.hle_params[1];
                gdrom_log.warn(termcolor.yellow("    GDROM GetTOC2: area={d} dest=0x{X:0>8}, TODO!"), .{ area, dest });
                if (area == 1) {
                    // High Density Area, doesn't have a TOC? (What's that thing at 0x110 in track 3?)
                } else {}
                self.hle_status = GDROMStatus.Standby;
            },
            GDROMCommand.GetSCD => {
                gdrom_log.warn(termcolor.yellow("    Unimplemented GDROM command {X:0>8} {s}"), .{ self.hle_command, @tagName(self.hle_command) });
                const dest = self.hle_params[1];
                dc.cpu.write32(dest, @intFromEnum(CDAudioStatus.Ended));
                self.hle_status = GDROMStatus.Standby;
            },
            else => {
                gdrom_log.warn(termcolor.yellow("    Unhandled GDROM command {X:0>8} {s}"), .{ self.hle_command, @tagName(self.hle_command) });
                self.hle_status = GDROMStatus.Standby;
            },
        }
    }

    pub fn check_command(self: *@This(), cmd_id: u32) u32 {
        if (cmd_id != self._current_command_id) {
            @memset(&self.hle_result, 0);
            self.hle_result[0] = 0x5;
            return 0; // no such request active
        }
        if (self.hle_status != GDROMStatus.Standby) return 1; // request is still being processed

        // request has completed
        self._current_command_id = 0;
        return 2;
    }
};
