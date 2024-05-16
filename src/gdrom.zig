const std = @import("std");
const termcolor = @import("termcolor.zig");

const gdrom_log = std.log.scoped(.gdrom);

const GDI = @import("gdi.zig").GDI;
const SH4 = @import("sh4.zig").SH4;
const Dreamcast = @import("dreamcast.zig").Dreamcast;

const HardwareRegisters = @import("hardware_registers.zig");
const HardwareRegister = HardwareRegisters.HardwareRegister;

const GDROMCommand71Reply = @import("gdrom_secu.zig").GDROMCommand71Reply;

pub const GDROMStatus = enum(u4) {
    Busy = 0, // State transition
    Paused = 1,
    Standby = 2,
    Playing = 3, // CD playback
    Seeking = 4,
    Scanning = 5,
    Open = 6, // Tray is open
    Empty = 7, // No disc
    Retry = 8, // Read retry in progress (option)
    Error = 9, // Reading of disc TOC failed (state does not allow access)
};

pub const DiscFormat = enum(u4) {
    CDDA = 0,
    CDROM = 1,
    CDROM_XA = 2,
    CDI = 3,
    GDROM = 8,
};

// HLE
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

const CDAudioStatus = enum(u8) {
    Invalid = 0x00, // Audio status byte not supported or invalid
    Playing = 0x11, // Audio playback in progress
    Paused = 0x12, // Audio playback paused
    Ended = 0x13, // Audio playback ended normally
    Error = 0x14, // Audio playback ended abnormally (error)
    NoInfo = 0x15, // No audio status information
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

const CommandOrData = enum(u1) {
    Data = 0,
    Command = 1,
};

const TransferDirection = enum(u1) {
    HostToDevice = 0,
    DeviceToHost = 1,
};

const InterruptReasonRegister = packed struct(u8) {
    cod: CommandOrData = .Data, // When "0," this field indicates data; when "1," this field indicates a command
    io: TransferDirection = .HostToDevice, // When "0," this field indicates the direction of transfer is from the host to the device; when "1," this field indicates the direction of transfer is from the device to the host.

    _: u6 = 0,
};

const ScheduledEvent = struct {
    cycles: u32,
    status: StatusRegister,
    interrupt_reason: ?InterruptReasonRegister,
};

const SPIPacketCommandCode = enum(u8) {
    TestUnit = 0x00, // Verify access readiness
    ReqStat = 0x10, // Get CD status
    ReqMode = 0x11, // Get various settings
    SetMode = 0x12, // Make various settings
    ReqError = 0x13, // Get error details
    GetToC = 0x14, // Get all TOC data
    ReqSes = 0x15, // Get specified session data
    CDOpen = 0x16, // Open tray
    CDPlay = 0x20, // Play CD
    CDSeek = 0x21, // Seek for playback position
    CDScan = 0x22, // Perform scan
    CDRead = 0x30, // Read CD
    CDRead2 = 0x31, // CD read (pre-read position)
    GetSCD = 0x40, // Get subcode
    // Undocumented Security Commands, see gdrom_secu.zig
    SYS_CHK_SECU = 0x70,
    SYS_REQ_SECU = 0x71,
    SYS_CHG_COMD = 0x72,
    SYS_REQ_COMD = 0x73,

    _,
};

pub fn msf_to_lba(minutes: u8, seconds: u8, frame: u8) u32 {
    return @as(u32, minutes) * 60 * 75 + @as(u32, seconds) * 75 + @as(u32, frame);
}

pub const GDROM = struct {
    disk: ?GDI = null,

    state: GDROMStatus = GDROMStatus.Standby,
    // LLE
    status_register: StatusRegister = .{},
    control_register: ControlRegister = .{},
    error_register: ErrorRegister = .{},
    interrupt_reason_register: InterruptReasonRegister = .{},
    byte_count: u16 = 0,

    data_queue: std.fifo.LinearFifo(u8, .Dynamic),

    packet_command_idx: u8 = 0,
    packet_command: [12]u8 = undefined,

    scheduled_event: ?ScheduledEvent = null,

    // HLE
    hle_command: GDROMCommand = undefined,
    hle_params: [4]u32 = undefined,
    hle_result: [4]u32 = undefined,

    _next_command_id: u32 = 1,
    _current_command_id: u32 = 0,

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) GDROM {
        var gdrom = GDROM{
            .data_queue = std.fifo.LinearFifo(u8, .Dynamic).init(allocator),
            ._allocator = allocator,
        };
        gdrom.reinit();
        return gdrom;
    }

    pub fn reinit(self: *@This()) void {
        self.state = GDROMStatus.Standby;
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
                if (self.control_register.nien == 0)
                    dc.raise_external_interrupt(.{ .GDRom = 1 });
                if (event.interrupt_reason) |reason| {
                    self.interrupt_reason_register = reason;
                    dc.raise_normal_interrupt(.{ .EoD_GDROM = 1 });
                }
                self.scheduled_event = null;
            } else {
                event.cycles -= cycles;
            }
        }
    }

    fn schedule_event(self: *@This(), event: ScheduledEvent) void {
        if (self.scheduled_event) |*e| {
            gdrom_log.warn(termcolor.yellow("Scheduled event already in progress: {any}"), .{e});
        }
        self.scheduled_event = event;
    }

    pub fn read_register(self: *@This(), comptime T: type, addr: u32) T {
        std.debug.assert(addr >= 0x005F7000 and addr <= 0x005F709C);
        switch (@as(HardwareRegister, @enumFromInt(addr))) {
            .GD_AlternateStatus_DeviceControl => {
                gdrom_log.info("  Read Alternate Status @{X:0>8} = {any}", .{ addr, self.status_register });
                // NOTE: Alternate status reads do NOT clear the pending interrupt signal.
                return @intCast(@as(u8, @bitCast(self.status_register)));
            },
            .GD_Status_Command => {
                const val: T = @intCast(@as(u8, @bitCast(self.status_register)));
                gdrom_log.info("  Read Status @{X:0>8} = {any}", .{ addr, self.status_register });
                // Clear the pending interrupt signal.
                self.status_register.drq = 0;
                return val;
            },
            .GD_Data => {
                if (self.data_queue.count == 0) {
                    gdrom_log.warn(termcolor.yellow("  Error: Read to Data while data_queue is empty (@{X:0>8})."), .{addr});
                    return 0;
                }
                if (T == u16) {
                    // Only accessed with 16-bit reads
                    const low = self.data_queue.readItem().?;
                    const high = self.data_queue.readItem().?;
                    const val: u16 = @as(u16, low) | (@as(u16, high) << 8);
                    // gdrom_log.debug("  Read({any}) from Data FIFO: {X:0>4}", .{ T, val });
                    return val;
                }
                @panic("8-bit read to GD_Data");
            },
            .GD_Error_Features => {
                // 7    6    5    4   3    2    1    0
                //    Sense Key      MCR  ABRT EOMF ILI
                const val = @as(u8, @intFromEnum(if (self.status_register.drdy == 0) SenseKey.NotReady else SenseKey.NoSense)) << 4;
                gdrom_log.info("  Read GD_Error_Features @{X:0>8} = 0x{X:0>8}", .{ addr, val });
                return val;
            },
            .GD_InterruptReason_SectorCount => {
                gdrom_log.info("  Read Interrupt Reason @{X:0>8} = {any}", .{ addr, self.interrupt_reason_register });
                return @intCast(@as(u8, @bitCast(self.interrupt_reason_register)));
            },
            .GD_SectorNumber => {
                // TODO: See REQ_STAT
                //  7  6  5  4  | 3  2  1  0
                //  Disc Format |   Status
                const val = if (self.disk == null)
                    (@as(u8, @intFromEnum(DiscFormat.GDROM)) << 4) | @intFromEnum(GDROMStatus.Empty)
                else
                    (@as(u8, @intFromEnum(DiscFormat.GDROM)) << 4) | @intFromEnum(if (self.status_register.drdy == 0) GDROMStatus.Busy else GDROMStatus.Standby);
                gdrom_log.warn(termcolor.yellow("  GDROM Read to SectorNumber @{X:0>8} = {X:0>2}"), .{ addr, val });
                return val;
            },
            .GD_ByteCountLow => {
                const val = @as(u8, @truncate(self.data_queue.readableLength()));
                gdrom_log.info("  Read Byte Count Low @{X:0>8} = 0x{X:0>8}", .{ addr, val });
                return val;
            },
            .GD_ByteCountHigh => {
                const val = @as(u8, @truncate(self.data_queue.readableLength() >> @intCast(8)));
                gdrom_log.info("  Read Byte Count High @{X:0>8} = 0x{X:0>8}", .{ addr, val });
                return val;
            },
            .GD_DriveSelect => {
                const val = 0b10100000;
                gdrom_log.info("  Read Drive Select @{X:0>8} = 0x{X:0>8}", .{ addr, val });
                return val;
            },
            else => {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Read to @{X:0>8}"), .{addr});
                return 0;
            },
        }
    }

    pub fn write_register(self: *@This(), comptime T: type, addr: u32, value: T) void {
        std.debug.assert(addr >= 0x005F7000 and addr <= 0x005F709C);
        switch (@as(HardwareRegister, @enumFromInt(addr))) {
            .GD_Status_Command => {
                self.status_register.check = 0;
                self.error_register = .{};
                switch (@as(Command, @enumFromInt(value))) {
                    .SoftReset => {
                        gdrom_log.info("  Command: SoftReset", .{});
                        self.data_queue.discard(self.data_queue.count);
                        self.status_register = .{};
                    },
                    .ExecuteDeviceDiagnostic => {
                        gdrom_log.info("  Command: ExecuteDeviceDiagnostic", .{});
                        self.status_register.bsy = 1;
                        self.status_register.drq = 0;
                        // The device clears the BSY bit and initiates an interrup
                        self.schedule_event(.{
                            .cycles = 100, // FIXME: Random value
                            .status = .{ .bsy = 0, .drq = 0 },
                            .interrupt_reason = null,
                        });
                        // Always report no errors.
                        self.error_register = .{};
                    },
                    .PacketCommand => {
                        gdrom_log.warn(termcolor.yellow("  Command: PacketCommand"), .{});
                        self.status_register.bsy = 1;

                        self.packet_command_idx = 0;

                        self.schedule_event(.{
                            .cycles = 0, // FIXME: Random value
                            .status = .{ .bsy = 0, .drq = 1 },
                            .interrupt_reason = .{ .cod = .Command, .io = .HostToDevice },
                        });
                    },
                    .IdentifyDevice => {
                        gdrom_log.warn(termcolor.yellow("  Command: IdentifyDevice - TODO!"), .{});
                        self.status_register.bsy = 1;
                        self.status_register.drq = 0;

                        // 0x00 Manufacturer's ID
                        self.data_queue.write(&[_]u8{0x0}) catch unreachable;
                        // 0x01 Model ID
                        self.data_queue.write(&[_]u8{0x0}) catch unreachable;
                        // 0x02 Version ID
                        self.data_queue.write(&[_]u8{0x0}) catch unreachable;
                        // 0x03 - 0x0F Reserved
                        for (0..0x10 - 0x03) |_| {
                            self.data_queue.write(&[_]u8{0x0}) catch unreachable;
                        }
                        // 0x10 - 0x1F Manufacturer's name (16 ASCII characters)
                        self.data_queue.write("            SEGA") catch unreachable;
                        // 0x20 - 0x2F Model name (16 ASCII characters)
                        self.data_queue.write("                ") catch unreachable;
                        // 0x30 - 0x3F Firmware version (16 ASCII characters)
                        self.data_queue.write("                ") catch unreachable;
                        // 0x40 - 0x4F Reserved
                        self.data_queue.write("                ") catch unreachable;

                        self.schedule_event(.{
                            .cycles = 0, // FIXME: Random value
                            .interrupt_reason = .{ .cod = .Data, .io = .DeviceToHost },
                            .status = .{ .bsy = 0, .drq = 1 },
                        });
                    },
                    .IdentifyDevice2 => {
                        // This command is issued by KallistiOS (g1_ata_scan), apparently to "check for a slave device".
                        // My guess is that this does nothing on a stock DC, but I have no way to test it...
                        gdrom_log.warn(termcolor.yellow("  Unhandled GDROM command 'IdentifyDevice2'."), .{});
                        self.nop();
                    },
                    .SetFeatures => {
                        gdrom_log.warn(termcolor.yellow("  Command: SetFeatures"), .{});

                        self.error_register = .{};

                        self.schedule_event(.{
                            .cycles = 0, // FIXME: Random value
                            .status = .{ .check = 0, .df = 0, .dsc = 0, .bsy = 0, .drq = 0, .drdy = 1 },
                            .interrupt_reason = null,
                        });
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
            .GD_Error_Features => {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to Features @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
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
                    self.packet_command[self.packet_command_idx] = @truncate(value);
                    self.packet_command[self.packet_command_idx + 1] = @truncate(value >> 8);
                    self.packet_command_idx += 2;
                } else {
                    std.debug.assert(false);
                }
                if (self.packet_command_idx >= 12) {
                    self.packet_command_idx = 0;

                    self.status_register.bsy = 1;
                    self.status_register.drq = 0;
                    gdrom_log.debug("  Received full SPI Command Packet!", .{});

                    for (0..6) |i| {
                        gdrom_log.debug("      {X:0>2} {X:0>2}", .{ self.packet_command[2 * i + 0], self.packet_command[2 * i + 1] });
                    }

                    switch (@as(SPIPacketCommandCode, @enumFromInt(self.packet_command[0]))) {
                        .TestUnit => self.test_unit(),
                        .ReqStat => self.req_stat() catch unreachable,
                        .ReqMode => self.req_mode() catch unreachable,
                        .SetMode => self.set_mode() catch unreachable,
                        .ReqError => self.req_error() catch unreachable,
                        .GetToC => self.get_toc() catch unreachable,
                        .ReqSes => self.req_ses() catch unreachable,
                        .CDOpen => gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand CDOpen: {X:0>2}"), .{self.packet_command}),
                        .CDPlay => gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand CDPlay: {X:0>2}"), .{self.packet_command}),
                        .CDSeek => gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand CDSeek: {X:0>2}"), .{self.packet_command}),
                        .CDScan => gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand CDScan: {X:0>2}"), .{self.packet_command}),
                        .CDRead => self.cd_read() catch unreachable,
                        .CDRead2 => gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand CDRead2: {X:0>2}"), .{self.packet_command}),
                        .GetSCD => self.get_subcode() catch unreachable,
                        .SYS_CHK_SECU => self.chk_secu() catch unreachable,
                        .SYS_REQ_SECU => self.req_secu() catch unreachable,
                        else => gdrom_log.warn(termcolor.yellow("  Unhandled GDROM PacketCommand 0x{X:0>2}"), .{self.packet_command[0]}),
                    }
                }
            },
            else => {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
            },
        }
    }

    fn nop(self: *@This()) void {
        gdrom_log.info("  NOP Packet command", .{});
        //   Setting "abort" in the error register
        self.error_register.abrt = 1;
        //   Setting "error" in the status register
        self.status_register.check = 1;
        //   Clearing BUSY in the status register
        self.status_register.bsy = 0;
        //   Asserting the INTRQ signal
        self.schedule_event(.{
            .cycles = 0,
            .status = .{ .check = 1, .bsy = 0 },
            .interrupt_reason = null,
        });
    }

    fn test_unit(self: *@This()) void {
        gdrom_log.info("  GDROM PacketCommand TestUnit: {X:0>2}", .{self.packet_command});
        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{},
            .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
        });
    }

    fn req_stat(self: *@This()) !void {
        const start_addr = self.packet_command[2];
        const alloc_length = self.packet_command[4];
        gdrom_log.warn(termcolor.yellow("  GDROM PacketCommand ReqStat - {X:0>2} {X:0>2}"), .{ start_addr, alloc_length });

        // 0 |  0 0 0 0 STATUS
        // 1 |  Disc Format - Repeat Count
        // 2 |  Address - Control
        // 3 |  TNO
        // 4 |  X
        // 5 |  FAD
        // 6 |  FAD
        // 7 |  FAD
        // 8 |  Max Read Error Retry Times
        // 9 |  0 0 0 0 0 0 0 0

        if (alloc_length > 0) {
            try self.data_queue.writeItem(if (self.status_register.drdy == 0) @intFromEnum(GDROMStatus.Busy) else @intFromEnum(GDROMStatus.Standby));
            try self.data_queue.writeItem(@as(u8, @intFromEnum(DiscFormat.GDROM)) << 4 | 0xE);
            try self.data_queue.writeItem(0x04);
            try self.data_queue.writeItem(0x02);
            try self.data_queue.writeItem(0x00);
            try self.data_queue.writeItem(0x00); // FAD
            try self.data_queue.writeItem(0x00); // FAD
            try self.data_queue.writeItem(0x00); // FAD
            try self.data_queue.writeItem(0x00);
            try self.data_queue.writeItem(0x00);

            self.schedule_event(.{
                .cycles = 0,
                .status = .{ .drq = 0, .bsy = 0, .drdy = 1 },
                .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
            });
        }
    }

    fn req_mode(self: *@This()) !void {
        gdrom_log.info(" GDROM PacketCommand ReqMode", .{});
        const start_addr = self.packet_command[2];
        const alloc_length = self.packet_command[4];

        const response = [_]u8{
            0, 0,
            0, // CDROM Speed, 00 = Max. speed
            0,
            0, // Standby Time
            0xB4, // Standby Time, 0xB4 is default
            0b00011001, // From MSB to LSB : 0 0 [Read Continuous] [ECC (Option)] [Read Retry] 0 0 [Form2 Read Retry]
            0,
            0,
            0x08, // Read Retry Times, default is 0x08
            'S', 'E', 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, // Drive Information (ASCII)
            'R', 'e', 'v', 0x20, '5', 0x20, 0x20, 0x20, // System Version (ASCII) - This is checked by the Boot ROM, anything bellow 5 will put it to sleep
            0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, // System Date (ASCII)
        };

        if (alloc_length > 0) {
            try self.data_queue.write(response[start_addr..][0..alloc_length]);
        }
        self.byte_count = alloc_length;

        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{ .bsy = 0, .drq = 1 },
            .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
        });
    }

    fn set_mode(self: *@This()) !void {
        gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand SetMode: {X:0>2}"), .{self.packet_command});
        // TODO: Set some stuff?

        self.interrupt_reason_register.io = .HostToDevice;
        self.interrupt_reason_register.cod = .Command;

        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{ .bsy = 0, .drq = 0 },
            .interrupt_reason = .{ .cod = .Command, .io = .HostToDevice },
        });
    }

    fn req_error(self: *@This()) !void {
        gdrom_log.info("GDROM PacketCommand ReqError", .{});
        const alloc_length = self.packet_command[4];

        const response = [_]u8{
            0xF0,                                        0x00,
            @intFromEnum(self.error_register.sense_key), 0x00,
            0x00, 0x00, 0x00, // Command Specific Information
            0x00, // Additional Sense Code (ASC)
            0x00, // Additional Sense Code Qualifier (ASCQ)
        };

        if (alloc_length > 0) {
            try self.data_queue.write(response[0..@min(response.len, alloc_length)]);
            if (alloc_length > response.len) {
                for (0..alloc_length - response.len) |_| {
                    try self.data_queue.writeItem(0x00);
                }
            }
        }
        self.byte_count = alloc_length;

        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{ .bsy = 0, .drq = 1 },
            .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
        });
    }

    fn write_toc(self: *@This(), dest: []u8, area: enum { SingleDensity, DoubleDensity }) u32 {
        if (self.disk) |disk| {
            std.debug.assert(disk.tracks.items.len >= 3);

            const start_track: usize = if (area == .DoubleDensity) 2 else 0;
            const end_track: usize = if (area == .DoubleDensity) disk.tracks.items.len - 1 else 1;

            @memset(dest[0..396], 0xFF);

            for (start_track..end_track + 1) |i| {
                const track = disk.tracks.items[i];
                const leading_fad = track.offset;

                dest[4 * (track.num - 1) + 0] = track.adr_ctrl_byte();
                dest[4 * (track.num - 1) + 1] = (@truncate(leading_fad >> 16));
                dest[4 * (track.num - 1) + 2] = (@truncate(leading_fad >> 8));
                dest[4 * (track.num - 1) + 3] = (@truncate(leading_fad >> 0));
            }

            @memcpy(dest[396 .. 396 + 2 * 4], &[_]u8{
                disk.tracks.items[start_track].adr_ctrl_byte(), @intCast(disk.tracks.items[start_track].num), 0x00, 0x00, // Start track info: [Control/ADR] [Start Track Number] [0  ] [0  ]
                disk.tracks.items[end_track].adr_ctrl_byte(), @intCast(disk.tracks.items[end_track].num), 0x00, 0x00, //     End track info:   [Control/ADR] [End Track Number  ] [0  ] [0  ]
            });

            if (area == .DoubleDensity) {
                @memcpy(dest[404..408], &[_]u8{
                    0x41, 0x08, 0x61, 0xB4, // Leadout info:     [Control/ADR] [FAD (MSB)]          [FAD] [FAD (LSB)]
                });
            } else {
                @memcpy(dest[404..408], &[_]u8{
                    0x00, 0x00, 0x33, 0x1D, // Leadout info:     [Control/ADR] [FAD (MSB)]          [FAD] [FAD (LSB)]
                });
            }

            self.schedule_event(.{
                .cycles = 0, // FIXME: Random value
                .status = .{ .bsy = 0, .drq = 1 },
                .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
            });

            return 408;
        }
        return 0;
    }

    fn get_toc(self: *@This()) !void {
        // Selects the type of volume.
        //   0: Get TOC information from single-density area.
        //   1: Get TOC information from double-density area
        const select = self.packet_command[1] & 1;
        const alloc_length = @as(u16, self.packet_command[3]) << 8 | self.packet_command[4];

        gdrom_log.warn(" GDROM PacketCommand GetToC - {s} (alloc_length: 0x{X:0>4})", .{ if (select == 0) "Single Density" else "Double Density", alloc_length });

        if (alloc_length > 0) {
            const bytes_written = self.write_toc(try self.data_queue.writableWithSize(408), if (select == 1) .DoubleDensity else .SingleDensity);
            self.data_queue.update(@min(bytes_written, alloc_length));

            // Debug Dump
            const d = self.data_queue.readableSlice(0);
            for (0..d.len / 4) |i| {
                gdrom_log.debug("[{d: >3}]  {X:0>2} {X:0>2} {X:0>2} {X:0>2}", .{ i * 4, d[4 * i + 0], d[4 * i + 1], d[4 * i + 2], d[4 * i + 3] });
            }

            self.schedule_event(.{
                .cycles = 0, // FIXME: Random value
                .status = .{ .bsy = 0, .drq = 1 },
                .interrupt_reason = .{ .cod = .Data, .io = .DeviceToHost },
            });
        }
    }

    fn req_ses(self: *@This()) !void {
        std.debug.assert(self.packet_command[0] == @intFromEnum(SPIPacketCommandCode.ReqSes));

        const session_number = self.packet_command[2];
        const alloc_length = self.packet_command[4];

        gdrom_log.warn(" GDROM PacketCommand ReqSes - Session Number: {d} (alloc_length: 0x{X:0>4})", .{ session_number, alloc_length });

        try self.data_queue.writeItem(@intFromEnum(self.state));
        try self.data_queue.writeItem(0);
        switch (session_number) {
            0 => {
                try self.data_queue.writeItem(2); // Number of Session
                try self.data_queue.write(&[_]u8{ 0x08, 0x61, 0xB4 }); // End FAD
            },
            1 => {
                try self.data_queue.writeItem(0); // Starting Track Number
                try self.data_queue.write(&[_]u8{ 0x00, 0x00, 0x00 }); // Start FAD
            },
            2 => {
                try self.data_queue.writeItem(2); // Starting Track Number
                try self.data_queue.write(&[_]u8{ 0x00, 0xB0, 0x5E }); // Start FAD
            },
            else => {
                gdrom_log.err(termcolor.red("  Unhandled Session Number: {d}"), .{session_number});
            },
        }

        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{ .bsy = 0, .drq = 1 },
            .interrupt_reason = .{ .cod = .Data, .io = .DeviceToHost },
        });
    }

    fn cd_read(self: *@This()) !void {
        const parameter_type = self.packet_command[1] & 0x1;
        const expected_data_type = (self.packet_command[1] >> 1) & 0x7;
        const data_select = (self.packet_command[1] >> 4) & 0xF;

        var start_addr: u32 = if (parameter_type == 0)
            (@as(u32, self.packet_command[2]) << 16) | (@as(u32, self.packet_command[3]) << 8) | self.packet_command[4] // Start FAD
        else
            msf_to_lba(self.packet_command[2], self.packet_command[1], self.packet_command[0]);

        const transfer_length: u32 = (@as(u32, self.packet_command[8]) << 16) | (@as(u32, self.packet_command[9]) << 8) | self.packet_command[10]; // Number of sectors to read

        gdrom_log.info("CD Read: parameter_type: {b:0>1} expected_data_type:{b:0>3} data_select:{b:0>4} - @{X:0>8} ({X})", .{ parameter_type, expected_data_type, data_select, start_addr, transfer_length });
        gdrom_log.info("Command: {X}", .{self.packet_command});

        if (start_addr < 45000) start_addr += 150; // FIXME: GDI stuff I still do have to figure out correctly... The offset is only applied on track 3? 3+?

        if (data_select == 0b0001) {
            // Raw data (all 2352 bytes of each sector)
            const bytes_written = self.disk.?.load_sectors_raw(start_addr, transfer_length, try self.data_queue.writableWithSize(2352 * transfer_length));
            self.data_queue.update(bytes_written);
        } else {
            // FIXME: Everything else isn't implemented
            if (data_select != 0b0010) // Data (no header or subheader)
                gdrom_log.err(termcolor.red("  Unimplemented data_select: {b:0>4}"), .{data_select});

            switch (expected_data_type) {
                0b000 => {
                    // No check for sector type.
                    // However, if reading of mode 2 track or Form 1/Form 2 is
                    // attempted in the case of a disc which is not a CD-ROM XA,
                    // the Mode 2 disc reading will fail.
                },
                0b001 => {
                    // CD-DA
                    // Error if sector other than CD-DA is read.
                },
                0b010 => {
                    // Mode 1
                    // Error if sector other than 2048 byte (Yellow Book) is read
                },
                0b011 => {
                    // Mode 2, Form 1 or Mode 2, Form 1
                    // Error if sector other than 2352 byte (Yellow Book) is read
                },
                0b100 => {
                    // Mode 2, Form 1.
                    // Error if sector other than 2048 byte (Green Book) is read
                },
                0b101 => {
                    // Mode 2, Form 2
                    // Error if sector other than 2324 byte (Green Book) is read
                },
                0b110 => {
                    // Mode 2 of non-CD-ROM XA disc
                    // 2336 bytes are read. No sector type check is performed
                },
                else => unreachable,
            }

            const bytes_written = self.disk.?.load_sectors(start_addr, transfer_length, try self.data_queue.writableWithSize(2352 * transfer_length));
            self.data_queue.update(bytes_written);

            gdrom_log.debug("First 0x20 bytes read: {X:0>2}", .{self.data_queue.readableSlice(0)[0..0x20]});
        }

        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{ .bsy = 0, .drq = 1 },
            .interrupt_reason = .{ .cod = .Data, .io = .DeviceToHost },
        });
    }

    fn get_subcode(self: *@This()) !void {
        const data_format = self.packet_command[1] & 0xF;
        const alloc_length = @as(u16, self.packet_command[3]) << 8 | self.packet_command[4];
        gdrom_log.warn(termcolor.yellow("  GDROM PacketCommand GetSCD - Format: {X:0>1}, AllocLength: {X:0>4}"), .{ data_format, alloc_length });
        try self.data_queue.writeItem(0); // Reserved
        try self.data_queue.writeItem(@intFromEnum(CDAudioStatus.NoInfo)); // Audio Status
        switch (data_format) {
            0 => {
                // All subcode information is transferred as raw data
            },
            1 => {
                // Subcode Q data only
            },
            2 => {
                // Media catalog number (UPC/bar code)
            },
            3 => {
                // International standard recording code (ISRC)
            },
            else => {
                // Reserved
            },
        }

        // TODO
        for (0..alloc_length - 2) |_| {
            try self.data_queue.writeItem(0);
        }

        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{ .bsy = 0, .drq = 1 },
            .interrupt_reason = .{ .cod = .Data, .io = .DeviceToHost },
        });
    }

    fn chk_secu(self: *@This()) !void {
        self.byte_count = 0;
        self.schedule_event(.{
            .cycles = 0,
            .status = .{ .bsy = 0, .drq = 1 },
            .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
        });
    }

    fn req_secu(self: *@This()) !void {
        gdrom_log.info(" GDROM PacketCommand ReqSecu: {X:0>2}", .{self.packet_command});
        const parameter = self.packet_command[1];
        _ = parameter;
        try self.data_queue.write(&GDROMCommand71Reply);

        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{ .bsy = 0, .drq = 1 },
            .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
        });
    }

    // HLE - Used by the BIOS syscalls

    pub fn send_command(self: *@This(), command_code: u32, params: [4]u32) u32 {
        if (self.state != GDROMStatus.Standby) return 0;

        self._current_command_id = self._next_command_id;
        self._next_command_id +%= 1;
        if (self._next_command_id == 0) self._next_command_id = 1;

        self.state = GDROMStatus.Busy;
        self.hle_command = @enumFromInt(command_code);
        self.hle_params = params;
        @memset(&self.hle_result, 0);

        return self._current_command_id;
    }

    pub fn mainloop(self: *@This(), dc: *Dreamcast) void {
        if (self.state != GDROMStatus.Busy) {
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
                const read = self.disk.?.load_sectors(lba, size, @as([*]u8, @ptrCast(dc.cpu._get_memory(dest)))[0 .. 2352 * size]);

                dc.schedule_interrupt(.{ .EoD_GDROM = 1 }, 100_000 * size);
                dc.schedule_external_interrupt(.{ .GDRom = 1 }, 100_000 * size);

                self.hle_result = .{ 0, 0, read, 0 };

                self.state = GDROMStatus.Standby;
            },
            GDROMCommand.Init => {
                gdrom_log.warn("    GDROM Command Init : TODO (Reset some stuff?)", .{});
                self.state = GDROMStatus.Standby;
            },
            GDROMCommand.GetVersion => {
                const dest = self.hle_params[0];
                const version = "GDC Version 1.10 1999-03-31";
                for (0..version.len) |i| {
                    dc.cpu.write8(@intCast(dest + i), version[i]);
                }
                dc.cpu.write8(@intCast(dest + version.len), 0x2);
                self.state = GDROMStatus.Standby;
            },
            GDROMCommand.ReqMode => {
                const dest = self.hle_params[0];
                gdrom_log.info("    GDROM ReqMode  dest=0x{X:0>8}", .{dest});
                dc.cpu.write32(dest + 0, 0); // Speed
                dc.cpu.write32(dest + 4, 0x00B4); // Standby
                dc.cpu.write32(dest + 8, 0x19); // Read Flags
                dc.cpu.write32(dest + 12, 0x08); // Read retry
                self.hle_result = .{ 0, 0, 0xA, 0 };
                self.state = GDROMStatus.Standby;
            },
            GDROMCommand.SetMode => {
                gdrom_log.warn("    GDROM SetMode: TODO", .{});
                self.state = GDROMStatus.Standby;
            },
            GDROMCommand.GetTOC2 => {
                const area = self.hle_params[0];
                const dest = self.hle_params[1];

                gdrom_log.info("    GDROM GetTOC2: area={d} dest=0x{X:0>8}", .{ area, dest });
                for (0..self.hle_params.len) |i| {
                    gdrom_log.debug("      {d}  {X:0>8}", .{ i, self.hle_params[i] });
                }

                const dest_slice = @as([*]u8, @ptrCast(dc.*.cpu._get_memory(dest & 0x1FFFFFFF)))[0..408];
                const bytes_written = self.write_toc(dest_slice, if (area == 1) .DoubleDensity else .SingleDensity);
                self.hle_result = .{ 0, 0, bytes_written, 0 };

                self.state = GDROMStatus.Standby;
            },
            GDROMCommand.GetSCD => {
                gdrom_log.warn(termcolor.yellow("    Unimplemented GDROM command GetSCD"), .{});
                for (0..self.hle_params.len) |i| {
                    gdrom_log.debug("      {d}  {X:0>8}", .{ i, self.hle_params[i] });
                }

                const dest = self.hle_params[2];
                const len = self.hle_params[1];
                for (0..len) |i| {
                    dc.cpu.write32(@intCast(dest + 4 * i), 0);
                }

                self.state = GDROMStatus.Standby;
            },
            else => {
                gdrom_log.warn(termcolor.yellow("    Unhandled GDROM command {any}"), .{self.hle_command});
                for (0..self.hle_params.len) |i| {
                    gdrom_log.debug("      {d}  {X:0>8}", .{ i, self.hle_params[i] });
                }
                self.state = GDROMStatus.Standby;
            },
        }
    }

    pub fn check_command(self: *@This(), cmd_id: u32) u32 {
        if (cmd_id != self._current_command_id) {
            @memset(&self.hle_result, 0);
            self.hle_result[0] = 0x5;
            return 0; // no such request active
        }
        if (self.state != GDROMStatus.Standby) return 1; // request is still being processed

        // request has completed
        self._current_command_id = 0;
        return 2;
    }
};
