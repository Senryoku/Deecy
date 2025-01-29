const std = @import("std");
const termcolor = @import("termcolor");

const gdrom_log = std.log.scoped(.gdrom);

pub const Disc = @import("./disc/disc.zig").Disc;
const Session = @import("./disc/disc.zig").Session;
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

const CDAudioStatus = enum(u8) {
    Invalid = 0x00, // Audio status byte not supported or invalid
    Playing = 0x11, // Audio playback in progress
    Paused = 0x12, // Audio playback paused
    Ended = 0x13, // Audio playback ended normally
    Error = 0x14, // Audio playback ended abnormally (error)
    NoInfo = 0x15, // No audio status information
};

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
    cycles: i32,
    state: ?GDROMStatus = null,
    status: ?StatusRegister = null,
    interrupt_reason: ?InterruptReasonRegister = null,
    clear_interrupt: bool = false,

    fn compare(_: void, a: @This(), b: @This()) std.math.Order {
        return std.math.order(a.cycles, b.cycles);
    }
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

disc: ?Disc = null,

state: GDROMStatus = GDROMStatus.Standby,
status_register: StatusRegister = .{},
control_register: ControlRegister = .{},
error_register: ErrorRegister = .{},
interrupt_reason_register: InterruptReasonRegister = .{},
features: packed struct(u8) { DMA: u1 = 1, _: u7 = 0 } = .{},
byte_count: u16 = 0,

pio_data_queue: std.fifo.LinearFifo(u8, .Dynamic),
dma_data_queue: std.fifo.LinearFifo(u8, .Dynamic),

packet_command_idx: u8 = 0,
packet_command: [12]u8 = [_]u8{0} ** 12,

audio_state: struct {
    mutex: std.Thread.Mutex = .{}, // NOTE: Should not be needed anymore.

    status: CDAudioStatus = .NoInfo,

    start_addr: u32 = 0,
    end_addr: u32 = 0,
    current_addr: u32 = 0,
    repetitions: u4 = 0, // 0xF means infinite

    current_position: u32 = 0,
    samples_in_buffer: u32 = 0,
    buffer: []i16,

    fn set_start_addr(self: *@This(), fad: u32) void {
        self.start_addr = fad;
        self.current_addr = self.start_addr;
        self.samples_in_buffer = 0;
    }

    pub fn serialize(self: @This(), writer: anytype) !usize {
        var bytes: usize = 0;
        bytes += try writer.write(std.mem.asBytes(&self.status));
        bytes += try writer.write(std.mem.asBytes(&self.start_addr));
        bytes += try writer.write(std.mem.asBytes(&self.end_addr));
        bytes += try writer.write(std.mem.asBytes(&self.current_addr));
        bytes += try writer.write(std.mem.asBytes(&self.repetitions));

        bytes += try writer.write(std.mem.asBytes(&self.current_position));
        bytes += try writer.write(std.mem.asBytes(&self.samples_in_buffer));
        bytes += try writer.write(std.mem.sliceAsBytes(self.buffer[0..self.samples_in_buffer]));

        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: anytype) !usize {
        self.mutex.lock();
        defer self.mutex.unlock();

        var bytes: usize = 0;
        bytes += try reader.read(std.mem.asBytes(&self.status));
        bytes += try reader.read(std.mem.asBytes(&self.start_addr));
        bytes += try reader.read(std.mem.asBytes(&self.end_addr));
        bytes += try reader.read(std.mem.asBytes(&self.current_addr));
        bytes += try reader.read(std.mem.asBytes(&self.repetitions));

        bytes += try reader.read(std.mem.asBytes(&self.current_position));
        bytes += try reader.read(std.mem.asBytes(&self.samples_in_buffer));
        bytes += try reader.read(std.mem.sliceAsBytes(self.buffer[0..self.samples_in_buffer]));

        return bytes;
    }
},

// Used for large PIO reads from CD (Quake 3 for example).
// These transfers are expected to be conducted in multiple steps, I suspect
// at least because of the size of the ByteCount register (16bits).
// Right now I'm filling the buffer sector by sector, no idea if this is accurate.
cd_read_state: struct {
    fad: u32 = 0,
    remaining_sectors: u32 = 0,
    data_select: u4 = 0,
    expected_data_type: u3 = 0,
} = .{},

_dc: *Dreamcast,
_allocator: std.mem.Allocator,

pub fn init(allocator: std.mem.Allocator, dc: *Dreamcast) !@This() {
    return .{
        .pio_data_queue = std.fifo.LinearFifo(u8, .Dynamic).init(allocator),
        .dma_data_queue = std.fifo.LinearFifo(u8, .Dynamic).init(allocator),
        .audio_state = .{ .buffer = try allocator.alloc(i16, 2352 / 2) },
        ._dc = dc,
        ._allocator = allocator,
    };
}

pub fn reset(self: *@This()) void {
    self.state = GDROMStatus.Standby;

    self.status_register = .{};
    self.control_register = .{};
    self.error_register = .{};
    self.interrupt_reason_register = .{};
    self.features = .{};
    self.byte_count = 0;
    self.pio_data_queue.discard(self.pio_data_queue.count);
    self.dma_data_queue.discard(self.dma_data_queue.count);
    self.packet_command_idx = 0;
    @memset(&self.packet_command, 0);
    self.audio_state = .{ .buffer = self.audio_state.buffer };
}

pub fn deinit(self: *@This()) void {
    self._allocator.free(self.audio_state.buffer);
    self.pio_data_queue.deinit();
    self.dma_data_queue.deinit();
}

pub fn get_cdda_samples(self: *@This()) [2]i16 {
    self.audio_state.mutex.lock();
    defer self.audio_state.mutex.unlock();

    if (self.audio_state.status != .Playing) return .{ 0, 0 };

    if (self.audio_state.samples_in_buffer <= self.audio_state.current_position + 1) {
        if (!self.fetch_next_cdda_sector()) return .{ 0, 0 };
    }
    defer self.audio_state.current_position += 2;
    return self.audio_state.buffer[self.audio_state.current_position..][0..2].*;
}

// Return true if we're still playing
fn fetch_next_cdda_sector(self: *@This()) bool {
    if (self.audio_state.current_addr >= self.audio_state.end_addr) {
        self.audio_state.current_addr = self.audio_state.start_addr;
        if (self.audio_state.repetitions == 0) {
            self.audio_state.status = .Ended;
            return false;
        }
        if (self.audio_state.repetitions < 0xF)
            self.audio_state.repetitions -= 1;
    }
    self.audio_state.current_position = 0;
    self.audio_state.samples_in_buffer = 0;

    const count = self.disc.?.load_sectors(self.audio_state.current_addr, 1, @as([*]u8, @ptrCast(self.audio_state.buffer.ptr))[0..2352]);
    self.audio_state.samples_in_buffer = count / 2; // 16-bit samples
    self.audio_state.current_addr += 1;

    return true;
}

fn schedule_event(self: *@This(), event: ScheduledEvent) void {
    // NOTE: I wasn't actually delaying any of these events, so I removed the scheduling mechanism internal to GDROM.
    //       If this ends up being necessary, use the global scheduler in the Dreamcast structure instead.
    std.debug.assert(event.cycles == 0);

    gdrom_log.debug(" Event: {any}", .{event});
    if (event.state) |state|
        self.state = state;
    if (event.status) |status|
        self.status_register = status;
    if (event.interrupt_reason) |reason| {
        self.interrupt_reason_register = reason;
        if (self.control_register.nien == 0)
            self._dc.raise_external_interrupt(.{ .GDRom = 1 });
    }
    if (event.clear_interrupt)
        self._dc.clear_external_interrupt(.{ .GDRom = 1 });
}

pub fn read_register(self: *@This(), comptime T: type, addr: u32) T {
    std.debug.assert(addr >= 0x005F7000 and addr <= 0x005F709C);
    switch (@as(HardwareRegister, @enumFromInt(addr))) {
        .GD_AlternateStatus_DeviceControl => {
            gdrom_log.debug("  Read Alternate Status @{X:0>8} = {any}", .{ addr, self.status_register });
            // NOTE: Alternate status reads do NOT clear the pending interrupt signal.

            // FIXME: CDI Hack - See issue #70
            //        IP.BIN (using boot ROM syscalls) is stuck waiting for the GD drive with data in DMA queue.
            const static = struct {
                var consecutive_busy_reads: u64 = 0;
                var last_dma_data_queue_count: u64 = 0;
            };
            if (self.status_register.bsy == 1 and self.dma_data_queue.count > 0 and self.dma_data_queue.count == static.last_dma_data_queue_count) {
                static.consecutive_busy_reads += 1;
                if (static.consecutive_busy_reads >= 10_000) {
                    gdrom_log.err(termcolor.red("CDI Hack: Stuck with data in dma queue, discarding."), .{});
                    self.dma_data_queue.discard(self.dma_data_queue.count);
                    self.status_register.bsy = 0;
                }
            } else static.consecutive_busy_reads = 0;
            static.last_dma_data_queue_count = self.dma_data_queue.count;

            return @intCast(@as(u8, @bitCast(self.status_register)));
        },
        .GD_Status_Command => {
            const val: T = @intCast(@as(u8, @bitCast(self.status_register)));
            gdrom_log.debug("  Read Status @{X:0>8} = {any}", .{ addr, self.status_register });
            // Clear the pending interrupt signal.
            self.schedule_event(.{
                .cycles = 0,
                .clear_interrupt = true,
            });
            return val;
        },
        .GD_Data => {
            if (self.pio_data_queue.count == 0) {
                gdrom_log.warn(termcolor.yellow("  Error: Read to Data while pio_data_queue is empty (@{X:0>8})."), .{addr});
                return 0;
            }
            if (T == u16) {
                // Only accessed with 16-bit reads
                const low = self.pio_data_queue.readItem().?;
                const high = self.pio_data_queue.readItem().?;
                const val: u16 = @as(u16, low) | (@as(u16, high) << 8);
                // gdrom_log.debug("  Read({any}) from Data FIFO: {X:0>4}", .{ T, val });
                if (self.pio_data_queue.count == 0) {
                    // 9. When the host has sent (sic?) entire data, the device clears the DRQ bit.
                    self.status_register.drq = 0;
                    // Check if there's more data to be sent.
                    self.cd_read_pio_fetch() catch unreachable;
                }
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
            var status = self.state;
            if (status != GDROMStatus.Open and self.disc == null) status = .Empty;
            const val = (if (self.disc) |d| @as(u8, @intFromEnum(d.get_format())) << 4 else 0) | @intFromEnum(status);
            gdrom_log.debug("  GDROM Read to SectorNumber @{X:0>8} = {X:0>2}", .{ addr, val });
            return val;
        },
        .GD_ByteCountLow => {
            const val = @as(u8, @truncate(self.pio_data_queue.readableLength()));
            gdrom_log.debug("  Read Byte Count Low @{X:0>8} = 0x{X:0>8}", .{ addr, val });
            return val;
        },
        .GD_ByteCountHigh => {
            const val = @as(u8, @truncate(self.pio_data_queue.readableLength() >> @intCast(8)));
            gdrom_log.debug("  Read Byte Count High @{X:0>8} = 0x{X:0>8}", .{ addr, val });
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

fn non_data_command(self: *@This()) void {
    // The device sets the BSY bit to "1" and executes the command.
    self.status_register.bsy = 1;
    // If an error has occurred during execution of the command, the device sets an appropriate status
    // and error bit that corresponds to the error condition
    // Always report no errors.
    self.error_register = .{};
    // When the command execution is completed, the device asserts INTRQ after the device has cleared the BSY bit.
    self.schedule_event(.{
        .cycles = 0, // FIXME: Random value
        .status = .{ .bsy = 0 },
        .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
    });
}

fn spi_non_data_command(self: *@This()) void {
    // The device sets the BSY bit and executes the command.
    self.status_register.bsy = 1;
    // When the device is ready to send the status, it writes the final status (IO, CoD, DRDY set, BSY,
    // DRQ cleared) to the "Status" register before making INTRQ valid
    self.schedule_event(.{
        .cycles = 0, // FIXME: Random value
        .status = .{ .bsy = 0, .drq = 0, .drdy = 1 },
        .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
    });
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
                    self.reset();
                    self.non_data_command();
                },
                .ExecuteDeviceDiagnostic => {
                    gdrom_log.info("  Command: ExecuteDeviceDiagnostic", .{});
                    self.status_register.drq = 0;
                    self.non_data_command();
                },
                .PacketCommand => {
                    gdrom_log.debug("  Command: PacketCommand", .{});
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

                    self.pio_data_queue.write(&[_]u8{
                        0x0, // 0x00 Manufacturer's ID
                        0x0, // 0x01 Model ID
                        0x0, // 0x02 Version ID
                    }) catch unreachable;
                    // 0x03 - 0x0F Reserved
                    self.pio_data_queue.write(&([1]u8{0x0} ** (0x10 - 0x03))) catch unreachable;
                    // 0x10 - 0x1F Manufacturer's name (16 ASCII characters)
                    self.pio_data_queue.write("            SEGA") catch unreachable;
                    // 0x20 - 0x2F Model name (16 ASCII characters)
                    self.pio_data_queue.write("                ") catch unreachable;
                    // 0x30 - 0x3F Firmware version (16 ASCII characters)
                    self.pio_data_queue.write("                ") catch unreachable;
                    // 0x40 - 0x4F Reserved
                    self.pio_data_queue.write("                ") catch unreachable;

                    self.pio_prep_complete();
                },
                .IdentifyDevice2 => {
                    // This command is issued by KallistiOS (g1_ata_scan), apparently to "check for a slave device".
                    // My guess is that this does nothing on a stock DC, but I have no way to test it...
                    gdrom_log.warn(termcolor.yellow("  Unhandled GDROM command 'IdentifyDevice2'."), .{});
                    self.nop();
                },
                .SetFeatures => {
                    gdrom_log.warn(termcolor.yellow("  Command: SetFeatures"), .{});
                    self.non_data_command();
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
            gdrom_log.debug("  GDROM Write to Features @{X:0>8} = 0x{X:0>8}", .{ addr, value });
            if (T == u8) {
                self.features = @bitCast(value);
            } else {
                gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write({s}) to Features @{X:0>8} = 0x{X:0>8}"), .{ @typeName(T), addr, value });
            }
        },
        .GD_InterruptReason_SectorCount => {
            gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to SectorCount @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
        },
        .GD_SectorNumber => {
            gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to SectorNumber @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
        },
        .GD_ByteCountLow => {
            gdrom_log.debug("  GDROM Write to ByteCountLow @{X:0>8} = 0x{X:0>8}", .{ addr, value });
            self.byte_count = (self.byte_count & 0xFF00) | value;
        },
        .GD_ByteCountHigh => {
            gdrom_log.debug("  GDROM Write to ByteCountHigh @{X:0>8} = 0x{X:0>8}", .{ addr, value });
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

                (switch (@as(SPIPacketCommandCode, @enumFromInt(self.packet_command[0]))) {
                    .TestUnit => self.test_unit(),
                    .ReqStat => self.req_stat(),
                    .ReqMode => self.req_mode(),
                    .SetMode => self.set_mode(),
                    .ReqError => self.req_error(),
                    .GetToC => self.get_toc(),
                    .ReqSes => self.req_ses(),
                    .CDOpen => {
                        gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand CDOpen: {X:0>2}"), .{self.packet_command});
                        self.spi_non_data_command();
                    },
                    .CDPlay => self.cd_play(),
                    .CDSeek => self.cd_seek(),
                    .CDScan => {
                        gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand CDScan: {X:0>2}"), .{self.packet_command});
                        self.state = .Scanning;
                        self.state = .Paused; // FIXME: Do not resolve immediatly?
                        self.spi_non_data_command();
                    },
                    .CDRead => self.cd_read(),
                    .CDRead2 => gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand CDRead2: {X:0>2}"), .{self.packet_command}),
                    .GetSCD => self.get_subcode(),
                    .SYS_CHK_SECU => self.chk_secu(),
                    .SYS_REQ_SECU => self.req_secu(),
                    else => gdrom_log.warn(termcolor.yellow("  Unhandled GDROM PacketCommand 0x{X:0>2}"), .{self.packet_command[0]}),
                } catch |err| {
                    gdrom_log.err("Error in handling GDROM SPI Packet Command: {}\n{any}\n", .{ err, self.packet_command });
                });
            }
        },
        else => {
            gdrom_log.warn(termcolor.yellow("  Unhandled GDROM Write to @{X:0>8} = 0x{X:0>8}"), .{ addr, value });
        },
    }
}

fn pio_prep_complete(self: *@This()) void {
    // When preparations are complete, the following steps are carried out at the device.
    // (1) Number of bytes to be read is set in "Byte Count" register.
    // (2) IO bit is set and CoD bit is cleared.
    // (3) DRQ bit is set, BSY bit is cleared.
    // (4) INTRQ is set, and a host interrupt is issued.

    if (self.pio_data_queue.count > std.math.maxInt(u16)) {
        gdrom_log.warn(termcolor.yellow("PIO transfer is too large: {} bytes."), .{self.pio_data_queue.count});
    }
    self.byte_count = @truncate(self.pio_data_queue.count); // Not really used (reading the register returns self.pio_data_queue.count directly).
    self.schedule_event(.{
        .cycles = 0, // FIXME
        .status = .{ .drq = 1, .bsy = 0, .drdy = 1 },
        .interrupt_reason = .{ .cod = .Data, .io = .DeviceToHost },
    });
}

pub fn on_dma_end(self: *@This(), dc: *Dreamcast) void {
    // When the device is ready to send the status, it writes the final status (IO, CoD, DRDY set, BSY,
    // DRQ cleared) to the "Status" register before making INTRQ valid.
    if (self.dma_data_queue.count == 0) {
        self.status_register.drq = 0;
        self.status_register.bsy = 0;
        self.status_register.drdy = 1;
        self.interrupt_reason_register = .{ .cod = .Command, .io = .DeviceToHost };

        if (self.control_register.nien == 0)
            dc.raise_external_interrupt(.{ .GDRom = 1 });
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
        .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
    });
}

fn test_unit(self: *@This()) void {
    gdrom_log.debug("  GDROM PacketCommand TestUnit: {X:0>2}", .{self.packet_command});
    self.spi_non_data_command();
}

fn req_stat(self: *@This()) !void {
    const start_addr = self.packet_command[2];
    const alloc_length = self.packet_command[4];

    gdrom_log.warn(termcolor.yellow("  GDROM PacketCommand ReqStat - {X:0>2} {X:0>2}"), .{ start_addr, alloc_length });
    if (start_addr != 0)
        gdrom_log.warn(termcolor.yellow("                      ReqStat - Start Addr isn't 0! ({X:0>2})"), .{start_addr});
    if (alloc_length != 0x0A)
        gdrom_log.warn(termcolor.yellow("                      ReqStat - Alloc Length isn't 0x0A! ({X:0>2})"), .{alloc_length});

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
        const fad: u32 = switch (self.state) {
            .Busy => 0x00, // Undefined
            .Paused => 0x00, // Subcode value obtained when the head has moved to the head position for pausing <PAUSE> state.
            .Standby, .Open, .Empty, .Error => 0x96, // Home position
            .Playing => self.audio_state.current_addr, // Head position before head travel, or the current head position
            .Seeking => self.audio_state.current_addr, // Head position before head travel
            .Scanning => self.audio_state.current_addr, // Head position before head travel, or current head position
            .Retry => self.audio_state.current_addr, // Head position before head travel.

        };

        try self.pio_data_queue.writeItem(if (self.disc == null) @intFromEnum(GDROMStatus.Empty) else @intFromEnum(self.state)); // 0000 | Status
        try self.pio_data_queue.writeItem((if (self.disc) |d| @as(u8, @intFromEnum(d.get_format())) << 4 else 0) | self.audio_state.repetitions); // Disc Format | Repeat Count
        try self.pio_data_queue.writeItem(0x04); // Address | Control
        try self.pio_data_queue.writeItem(0x02); // TNO
        try self.pio_data_queue.writeItem(0x00); // X
        try self.pio_data_queue.writeItem(@truncate(fad >> 0)); // FAD
        try self.pio_data_queue.writeItem(@truncate(fad >> 8)); // FAD
        try self.pio_data_queue.writeItem(@truncate(fad >> 16)); // FAD
        try self.pio_data_queue.writeItem(0x00); // Max Read Error Retry Times - Indicates how many read retries were necessary. This item is cleared (set to 0) when read.
        try self.pio_data_queue.writeItem(0x00);
    }
    self.pio_prep_complete();
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
        try self.pio_data_queue.write(response[start_addr..][0..alloc_length]);
    }
    self.pio_prep_complete();
}

fn set_mode(self: *@This()) !void {
    gdrom_log.warn(termcolor.yellow("  Unimplemented GDROM PacketCommand SetMode: {X:0>2}"), .{self.packet_command});
    // TODO: Set some stuff?
    // See "Transfer Packet Command Flow For PIO Data from Host"

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
        try self.pio_data_queue.write(response[0..@min(response.len, alloc_length)]);
        if (alloc_length > response.len) {
            for (0..alloc_length - response.len) |_| {
                try self.pio_data_queue.writeItem(0x00);
            }
        }
    }
    self.pio_prep_complete();
}

pub fn write_toc(self: *@This(), dest: []u8, area: Session.Area) u32 {
    if (self.disc) |disc| {
        self.schedule_event(.{
            .cycles = 0, // FIXME: Random value
            .status = .{ .bsy = 0, .drq = 1 },
            .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
        });

        return disc.write_toc(dest, area);
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
        const bytes_written = self.write_toc(try self.pio_data_queue.writableWithSize(408), if (select == 1) .DoubleDensity else .SingleDensity);
        self.pio_data_queue.update(@min(bytes_written, alloc_length));

        // Debug Dump
        const d = self.pio_data_queue.readableSlice(0);
        for (0..d.len / 4) |i| {
            gdrom_log.debug("[{d: >3}]  {X:0>2} {X:0>2} {X:0>2} {X:0>2}", .{ i * 4, d[4 * i + 0], d[4 * i + 1], d[4 * i + 2], d[4 * i + 3] });
        }
    }
    self.pio_prep_complete();
}

fn req_ses(self: *@This()) !void {
    std.debug.assert(self.packet_command[0] == @intFromEnum(SPIPacketCommandCode.ReqSes));

    const session_number = self.packet_command[2];
    const alloc_length = self.packet_command[4];

    gdrom_log.warn(" GDROM PacketCommand ReqSes - Session Number: {d} (alloc_length: 0x{X:0>4})", .{ session_number, alloc_length });

    if (self.disc) |disc| {
        try self.pio_data_queue.writeItem(@intFromEnum(self.state));
        try self.pio_data_queue.writeItem(0);

        const track_number_or_session_count: u8 = @intCast(if (session_number > 0) disc.get_session(session_number).first_track + 1 else disc.get_session_count());
        const fad = if (session_number > 0) disc.get_session(session_number).start_fad else disc.get_end_fad();

        try self.pio_data_queue.writeItem(track_number_or_session_count);
        try self.pio_data_queue.write(&[_]u8{ @truncate(fad >> 16), @truncate(fad >> 8), @truncate(fad >> 0) });
    } else {
        try self.pio_data_queue.writeItem(@intFromEnum(GDROMStatus.Empty));
        try self.pio_data_queue.writeItem(0);
        try self.pio_data_queue.writeItem(0); // Number of Session
        try self.pio_data_queue.write(&[_]u8{ 0x00, 0x00, 0x00 }); // End FAD
    }
    self.pio_prep_complete();
}

fn cd_play(self: *@This()) !void {
    gdrom_log.warn(termcolor.yellow("  GDROM PacketCommand CDPlay: {X:0>2}"), .{self.packet_command});

    self.audio_state.mutex.lock();
    defer self.audio_state.mutex.unlock();

    const parameter_type = self.packet_command[1] & 0x7;

    self.state = .Playing;
    self.audio_state.status = .Playing;
    self.audio_state.repetitions = @truncate(self.packet_command[6]);

    switch (parameter_type) {
        0b001 => {
            self.audio_state.set_start_addr((@as(u32, self.packet_command[2]) << 16) | (@as(u32, self.packet_command[3]) << 8) | self.packet_command[4]);
            self.audio_state.end_addr = (@as(u32, self.packet_command[8]) << 16) | (@as(u32, self.packet_command[9]) << 8) | self.packet_command[10];
            self.status_register.dsc = 1;
        },
        0b010 => {
            self.audio_state.set_start_addr(msf_to_lba(self.packet_command[2], self.packet_command[3], self.packet_command[4]));
            self.audio_state.end_addr = msf_to_lba(self.packet_command[8], self.packet_command[9], self.packet_command[10]);
            self.status_register.dsc = 1;
        },
        0b111 => {
            // (Re)Start playing without changing position.
        },
        else => {
            gdrom_log.err(termcolor.red("  GDROM CDPlay: Unrecognized parameter type {X:0>1}"), .{parameter_type});
        },
    }

    self.spi_non_data_command();
}

fn cd_seek(self: *@This()) !void {
    gdrom_log.warn(termcolor.yellow("  GDROM PacketCommand CDSeek: {X:0>2}"), .{self.packet_command});

    self.audio_state.status = .Paused;
    self.state = .Seeking;
    self.state = .Paused; // FIXME: Do not resolve immediatly?

    const parameter_type = self.packet_command[1] & 0x7;
    switch (parameter_type) {
        0b001 => {
            self.audio_state.set_start_addr((@as(u32, self.packet_command[2]) << 16) | (@as(u32, self.packet_command[3]) << 8) | self.packet_command[4]);
        },
        0b010 => {
            self.audio_state.set_start_addr(msf_to_lba(self.packet_command[2], self.packet_command[3], self.packet_command[4]));
        },
        0b011 => {
            // Stop playback (move to home position)
        },
        0b100 => {
            // Pause playback (seek position is unchanged)
        },
        else => {
            gdrom_log.err(termcolor.red("  GDROM CDPlay: Unrecognized parameter type {X:0>1}"), .{parameter_type});
        },
    }

    self.status_register.dsc = 1;
    self.spi_non_data_command();
}

fn cd_read_pio_fetch(self: *@This()) !void {
    if (self.cd_read_state.remaining_sectors > 0) {
        // If more data are to be sent, the device sets the BSY bit and repeats the above sequence from step 7.
        self.status_register.bsy = 1;
        try self.cd_read_fetch(&self.pio_data_queue, self.cd_read_state.data_select, self.cd_read_state.expected_data_type, self.cd_read_state.fad, 1);
        self.cd_read_state.fad += 1;
        self.cd_read_state.remaining_sectors -= 1;
        self.pio_prep_complete();
    } else {
        // 10. When the device is ready to send the status, it writes the final status to the "Status" register.
        //     CoD, IO, DRDY are set (before making INTRQ valid), and BSY and DRQ are cleared.
        self.schedule_event(.{
            .cycles = 0,
            .status = .{ .drq = 0, .bsy = 0, .drdy = 1 },
            .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
        });
    }
}

fn cd_read_fetch(self: *@This(), data_queue: *std.fifo.LinearFifo(u8, .Dynamic), data_select: u4, expected_data_type: u3, start_addr: u32, transfer_length: u32) !void {
    if (self.disc) |disc| {
        if (data_select == 0b0001) {
            // Raw data (all 2352 bytes of each sector)
            const bytes_written = disc.load_sectors_raw(start_addr, transfer_length, try data_queue.writableWithSize(2352 * transfer_length));
            data_queue.update(bytes_written);
        } else {
            // FIXME: Everything else isn't implemented
            if (data_select != 0b0010) // Data (no header or subheader)
                gdrom_log.err(termcolor.red("  Unimplemented data_select: {b:0>4}"), .{data_select});

            var expected_sector_size: u32 = 2352;

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
                    expected_sector_size = 2048;
                },
                0b011 => {
                    // Mode 2, Form 1 or Mode 2, Form 1
                    // Error if sector other than 2352 byte (Yellow Book) is read
                    expected_sector_size = 2352;
                },
                0b100 => {
                    // Mode 2, Form 1.
                    // Error if sector other than 2048 byte (Green Book) is read
                    expected_sector_size = 2048;
                },
                0b101 => {
                    // Mode 2, Form 2
                    // Error if sector other than 2324 byte (Green Book) is read
                    expected_sector_size = 2324;
                },
                0b110 => {
                    // Mode 2 of non-CD-ROM XA disc
                    // 2336 bytes are read. No sector type check is performed
                    expected_sector_size = 2336;
                },
                else => unreachable,
            }

            const bytes_written = disc.load_sectors(start_addr, transfer_length, try data_queue.writableWithSize(2352 * transfer_length));
            data_queue.update(bytes_written);

            if (bytes_written != transfer_length * expected_sector_size) {
                gdrom_log.err(termcolor.red("  Unexpected sector size: {d} written out of {d} * {d} = {d} expected."), .{ bytes_written, transfer_length, expected_sector_size, transfer_length * expected_sector_size });
            }

            gdrom_log.debug("First 0x20 bytes read: {X:0>2}", .{data_queue.readableSlice(0)[0..0x20]});
        }
    }
}

fn cd_read(self: *@This()) !void {
    const parameter_type = self.packet_command[1] & 0x1;
    const expected_data_type: u3 = @truncate((self.packet_command[1] >> 1) & 0x7);
    const data_select: u4 = @truncate((self.packet_command[1] >> 4) & 0xF);

    self.state = .Paused;
    self.audio_state.status = .Paused;

    const start_addr: u32 = if (parameter_type == 0)
        (@as(u32, self.packet_command[2]) << 16) | (@as(u32, self.packet_command[3]) << 8) | self.packet_command[4] // Start FAD
    else
        msf_to_lba(self.packet_command[2], self.packet_command[3], self.packet_command[4]);

    const transfer_length: u32 = (@as(u32, self.packet_command[8]) << 16) | (@as(u32, self.packet_command[9]) << 8) | self.packet_command[10]; // Number of sectors to read

    gdrom_log.debug("CD Read: parameter_type: {b:0>1} expected_data_type:{b:0>3} data_select:{b:0>4} - @{X:0>8} ({X})", .{ parameter_type, expected_data_type, data_select, start_addr, transfer_length });
    gdrom_log.debug("Command: {X}", .{self.packet_command});

    const transfer_type: enum { PIO, DMA } = if (self.features.DMA == 1) .DMA else .PIO;

    if (transfer_type == .PIO) {
        gdrom_log.warn(termcolor.yellow("  GDROM CDRead PIO mode: start_addr: {X:0>8}, transfer_length: {X:0>4}"), .{ start_addr, transfer_length });
        self.cd_read_state = .{
            .fad = start_addr,
            .remaining_sectors = transfer_length,
            .data_select = data_select,
            .expected_data_type = expected_data_type,
        };
        try self.cd_read_pio_fetch();
    } else {
        try self.cd_read_fetch(&self.dma_data_queue, data_select, expected_data_type, start_addr, transfer_length);
    }
}

fn get_subcode(self: *@This()) !void {
    const data_format = self.packet_command[1] & 0xF;
    const alloc_length = @as(u16, self.packet_command[3]) << 8 | self.packet_command[4];
    gdrom_log.warn(termcolor.yellow("  GDROM PacketCommand GetSCD - Format: {X:0>1}, AllocLength: {X:0>4}"), .{ data_format, alloc_length });
    try self.pio_data_queue.writeItem(0); // Reserved
    try self.pio_data_queue.writeItem(@intFromEnum(self.audio_state.status)); // Audio Status
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
        try self.pio_data_queue.writeItem(0);
    }

    // Ended normally/ended abnormally is only reported once
    switch (self.audio_state.status) {
        .Ended, .Error => self.audio_state.status = CDAudioStatus.NoInfo,
        else => {},
    }

    self.pio_prep_complete();
}

fn chk_secu(self: *@This()) !void {
    self.byte_count = 0;
    self.schedule_event(.{
        .cycles = 0, // FIXME: Random value
        .status = .{ .bsy = 0, .drq = 0, .drdy = 1 },
        .interrupt_reason = .{ .cod = .Command, .io = .DeviceToHost },
    });
}

fn req_secu(self: *@This()) !void {
    gdrom_log.info(" GDROM PacketCommand ReqSecu: {X:0>2}", .{self.packet_command});
    const parameter = self.packet_command[1];
    _ = parameter;
    try self.pio_data_queue.write(&GDROMCommand71Reply);

    self.pio_prep_complete();
}

pub fn serialize(self: *@This(), writer: anytype) !usize {
    var bytes: usize = 0;
    bytes += try writer.write(std.mem.asBytes(&self.state));
    bytes += try writer.write(std.mem.asBytes(&self.status_register));
    bytes += try writer.write(std.mem.asBytes(&self.control_register));
    bytes += try writer.write(std.mem.asBytes(&self.error_register));
    bytes += try writer.write(std.mem.asBytes(&self.interrupt_reason_register));
    bytes += try writer.write(std.mem.asBytes(&self.features));
    bytes += try writer.write(std.mem.asBytes(&self.byte_count));

    bytes += try writer.write(std.mem.asBytes(&self.pio_data_queue.count));
    if (self.pio_data_queue.count > 0)
        bytes += try writer.write(self.pio_data_queue.readableSlice(0));

    bytes += try writer.write(std.mem.asBytes(&self.dma_data_queue.count));
    if (self.dma_data_queue.count > 0)
        bytes += try writer.write(self.dma_data_queue.readableSlice(0));

    bytes += try writer.write(std.mem.asBytes(&self.packet_command_idx));
    bytes += try writer.write(std.mem.sliceAsBytes(&self.packet_command));

    bytes += try self.audio_state.serialize(writer);

    bytes += try writer.write(std.mem.asBytes(&self.cd_read_state));

    return bytes;
}

pub fn deserialize(self: *@This(), reader: anytype) !usize {
    var bytes: usize = 0;
    bytes += try reader.read(std.mem.asBytes(&self.state));
    bytes += try reader.read(std.mem.asBytes(&self.status_register));
    bytes += try reader.read(std.mem.asBytes(&self.control_register));
    bytes += try reader.read(std.mem.asBytes(&self.error_register));
    bytes += try reader.read(std.mem.asBytes(&self.interrupt_reason_register));
    bytes += try reader.read(std.mem.asBytes(&self.features));
    bytes += try reader.read(std.mem.asBytes(&self.byte_count));

    self.pio_data_queue.discard(self.pio_data_queue.count);
    var pio_data_queue_count: usize = 0;
    bytes += try reader.read(std.mem.asBytes(&pio_data_queue_count));
    if (pio_data_queue_count > 0) {
        bytes += try reader.read(try self.pio_data_queue.writableWithSize(pio_data_queue_count));
        self.pio_data_queue.update(pio_data_queue_count);
    }

    self.dma_data_queue.discard(self.dma_data_queue.count);
    var dma_data_queue_count: usize = 0;
    bytes += try reader.read(std.mem.asBytes(&dma_data_queue_count));
    if (dma_data_queue_count > 0) {
        bytes += try reader.read(try self.dma_data_queue.writableWithSize(dma_data_queue_count));
        self.dma_data_queue.update(dma_data_queue_count);
    }

    bytes += try reader.read(std.mem.asBytes(&self.packet_command_idx));
    bytes += try reader.read(std.mem.sliceAsBytes(&self.packet_command));

    bytes += try self.audio_state.deserialize(reader);

    bytes += try reader.read(std.mem.asBytes(&self.cd_read_state));

    return bytes;
}
