const std = @import("std");
const termcolor = @import("termcolor.zig");

const aica_log = std.log.scoped(.aica);

const arm7 = @import("arm7");

const Dreamcast = @import("dreamcast.zig").Dreamcast;

// Yamaha AICA Audio Chip

// Address of AICA registers. Add 0x00700000 for access from SH4 and 0x00800000 for access from ARM7
pub const AICARegister = enum(u32) {
    _00702800 = 0x00002800,
    TESTB0 = 0x00002804,
    _00702808 = 0x00002808,
    AFSET = 0x0000280C,
    _00702810 = 0x00002810,
    CA = 0x00002814,

    IC_TEST = 0x00002880,
    DMEA = 0x00002884,
    DGATE = 0x00002888,
    DDIR_DEXE = 0x0000288C,

    TACTL_TIMA = 0x00002890,
    TBCTL_TIMB = 0x00002894,
    TCCTL_TIMC = 0x00002898,

    SCIEB = 0x0000289C,
    SCIPD = 0x000028A0,
    SCIRE = 0x000028A4,

    SCILV0 = 0x000028A8,
    SCILV1 = 0x000028AC,
    SCILV2 = 0x000028B0,

    MCIEB = 0x000028B4, // Main CPU Interrupt Enable
    MCIPD = 0x000028B8, // Main CPU Interrupt Pending
    MCIRE = 0x000028BC, // Main CPU Interrupt Reset

    ARMRST = 0x00002C00,
    INTRequest = 0x00002D00,
    INTClear = 0x00002D004,

    RTC_High = 0x00010000,
    RTC_Low = 0x00010004,
    RTC_Write_Enable = 0x00010008,

    _,
};

pub const InterruptBits = packed struct(u32) {
    Ext: u1 = 0,
    _0: u2 = 0,
    MIDI: u1 = 0,
    DMA: u1 = 0,
    SCPU: u1 = 0,
    TimerA: u1 = 0,
    Misc: u1 = 0,
    _1: u24 = 0,
};

pub const SB_ADSUSP = packed struct(u32) {
    // DMA Suspend Request (Write Only
    // 0: Continues DMA transfer without going to the suspended state. Or, bit 2 of the SB_ADTSEL register is "0"
    // 1: Suspends and terminates DMA transfer
    DMASuspendRequest: u1 = 0,

    _reserved1: u3 = 0,

    // DMA Suspend or DMA Stop (Read Only)
    // 0: DMA transfer is in progress, or bit 2 of the SB_ADTSEL register is "0"
    // 1: DMA transfer has ended, or is stopped due to a suspend
    DMASuspend: u1 = 1,

    // DMA Request Input State (Read Only)
    // 0: The DMA transfer request is high (transfer not possible), or bit 2 of the SB_ADTSEL register is "0"
    // 1: The DMA transfer request is low (transfer possible)
    DMARequestInputState: u1 = 1,

    _: u26 = 0,
};

const AICARegisterStart = 0x00700000;

// Some potential memory-mapped registers, I'm really not sure what to do with this yet.
const AICAMemoryRegister = enum(u32) {
    REG_BUS_REQUEST = 0x00802808,
    REG_ARM_INT_ENABLE = 0x0080289c,
    REG_ARM_INT_SEND = 0x008028a0,
    REG_ARM_INT_RESET = 0x008028a4,
    REG_ARM_FIQ_BIT_0 = 0x008028a8,
    REG_ARM_FIQ_BIT_1 = 0x008028ac,
    REG_ARM_FIQ_BIT_2 = 0x008028b0,
    REG_SH4_INT_ENABLE = 0x008028b4,
    REG_SH4_INT_SEND = 0x008028b8,
    REG_SH4_INT_RESET = 0x008028bc,
    REG_ARM_FIQ_CODE = 0x00802d00,
    REG_ARM_FIQ_ACK = 0x00802d04,
};

// Memory Map
// SH4 Side             Internal
// 00800000 - 00FFFFFF  00000000 - 007FFFFF  DRAM_AREA*
// 00700000 - 007027FF  00800000 - 008027FF  CHANNEL_DATA
// 00702800 - 00702FFF  00802800 - 00802FFF  COMMON_DATA
// 00703000 - 00707FFF  00803000 - 00807FFF  DSP_DATA
// 00710000 - 00710008        N/A            RTC_REGISTERS

const EventType = enum {
    EndOfDMA,
    ExternalInterrupt,
};

const ScheduledEvents = struct {
    event_type: EventType,
    cycles: u32,
};

pub const AICA = struct {
    const ARM7CycleRatio = 8;

    arm7: arm7.ARM7 = undefined,

    regs: []u32 = undefined, // All registers are 32-bit afaik
    wave_memory: []u8 align(4) = undefined,

    rtc_write_enabled: bool = false,

    events: std.ArrayList(ScheduledEvents) = undefined,

    _cycles_counter: u32 = 0,
    _allocator: std.mem.Allocator = undefined,

    // NOTE: Call setup_arm after!
    pub fn init(allocator: std.mem.Allocator) !AICA {
        var r = AICA{
            .regs = try allocator.alloc(u32, 0x8000 / 4),
            .wave_memory = try allocator.alloc(u8, 0x200000),
            .events = std.ArrayList(ScheduledEvents).init(allocator),
            ._allocator = allocator,
        };
        @memset(r.regs, 0);
        @memset(r.wave_memory, 0);
        r.arm7 = arm7.ARM7.init(r.wave_memory);
        r.arm7.reset_pipeline();

        r.regs[@intFromEnum(AICARegister._00702800) / 4] = 0x10;

        r.regs[@intFromEnum(AICARegister.SCILV0) / 4] = 0x18;
        r.regs[@intFromEnum(AICARegister.SCILV1) / 4] = 0x50;
        r.regs[@intFromEnum(AICARegister.SCILV2) / 4] = 0x08;

        return r;
    }

    pub fn setup_arm(self: *@This()) void {
        self.arm7.on_external_read = .{
            .callback = @ptrCast(&@This().read_from_arm),
            .data = self,
        };
        self.arm7.on_external_write = .{
            .callback = @ptrCast(&@This().write_from_arm),
            .data = self,
        };
    }

    pub fn deinit(self: *AICA) void {
        self._allocator.free(self.regs);
        self._allocator.free(self.wave_memory);
        self.events.deinit();
    }

    pub fn read_mem(self: *const AICA, comptime T: type, addr: u32) T {
        // FIXME: Hopefully remove this when we have a working AICA (I mean, one can dream.)
        switch (addr) {
            0x0080005C => {
                return 0x1; // Hack for an infinite loop in Power Stone, no idea what this value is supposed to be.
            },
            0x00800104 => {
                return 0x00900000; // Crazy Taxi will hang indefinitely here during the demo if this is zero.
            },
            0x00800284, 0x00800288 => {
                return 0x00900000; // Same thing when trying to start an arcade game.
            },
            else => {},
        }
        return @as(*T, @alignCast(@ptrCast(&self.wave_memory[addr - 0x00800000]))).*;
    }

    pub fn write_mem(self: *AICA, comptime T: type, addr: u32, value: T) void {
        switch (addr) {
            else => {},
        }
        @as(*T, @alignCast(@ptrCast(&self.wave_memory[addr - 0x00800000]))).* = value;
    }

    fn get_reg(self: *const AICA, comptime T: type, reg: AICARegister) T {
        return @as(*const T, @alignCast(@ptrCast(&self.regs[@intFromEnum(reg)]))).*;
    }

    pub fn read_register(self: *const AICA, addr: u32) u32 {
        const local_addr = addr & 0x0000FFFF;
        //aica_log.debug("Read AICA register at 0x{X:0>8} / 0x{X:0>8}", .{ addr, local_addr });
        //aica_log.debug("Read AICA register at 0x{X:0>8} = 0x{X:0>8}", .{ addr, self.regs[local_addr / 4] });
        return self.regs[local_addr / 4];
    }

    pub fn write_register(self: *AICA, addr: u32, value: u32) void {
        const local_addr = addr & 0x0000FFFF;
        aica_log.info("Write to AICA Register at 0x{X:0>8} = 0x{X:0>8}", .{ addr, value });
        switch (@as(AICARegister, @enumFromInt(local_addr))) {
            .DDIR_DEXE => {
                self.regs[local_addr / 4] = value & 0xFFFFFFFC;
                if (value & 1 == 1) {
                    aica_log.info(termcolor.green("DMA Start"), .{});
                    @panic("TODO AICA DMA");
                }
                return;
            },
            .ARMRST => {
                if (value & 1 == 0) {
                    aica_log.info(termcolor.green("ARM reset"), .{});
                    self.arm7.set_reset(.High);
                } else {
                    self.arm7.set_reset(.Low);
                }
            },
            .MCIPD => {
                if (@as(InterruptBits, @bitCast(value)).SCPU == 1 and self.get_reg(InterruptBits, .MCIEB).SCPU == 1) {
                    aica_log.info(termcolor.green("SCPU interrupt"), .{});
                    self.events.append(.{ .event_type = .ExternalInterrupt, .cycles = 0 }) catch unreachable;
                }
            },
            else => {},
        }
        self.regs[local_addr / 4] = value;
    }

    pub fn read_rtc_register(self: *const AICA, addr: u32) u32 {
        _ = self;
        std.debug.assert(addr >= 0x00710000);
        switch (addr - 0x00710000) {
            0x00 => {
                return (@as(u32, @intCast(std.time.timestamp())) >> 16) & 0x0000FFFFF;
            },
            0x04 => {
                return @as(u32, @intCast(std.time.timestamp())) & 0x0000FFFFF;
            },
            else => {
                @panic("Read to unimplemented RTC register.");
            },
        }
    }

    pub fn write_rtc_register(self: *AICA, addr: u32, value: u32) void {
        // RTC[31:0] is normally write protected, but can be written when a "1" is written to the EN bit.
        // Furthermore, when RTC[15:0] is written, the counter below one second is cleared. When RTC[31:16] is
        // written, write protection is enabled again.
        switch (addr - 0x00710000) {
            0x00 => {},
            0x04 => {
                self.rtc_write_enabled = false;
            },
            0x08 => {
                self.rtc_write_enabled = value == 1;
            },
            else => {
                @panic("Read to unimplemented RTC register.");
            },
        }
    }

    pub fn read_from_arm(self: *const AICA, addr: u32) u32 {
        // I think the AICA might try to access out of bounds here.
        if (addr >= 0x00200000) {}
        return self.read_register(addr);
    }

    pub fn write_from_arm(self: *AICA, addr: u32, value: u32) void {
        self.write_register(addr, value);
    }

    pub fn update(self: *AICA, dc: *Dreamcast, cycles: u32) void {
        if (self.events.items.len > 0) {
            var index: u32 = 0;
            while (index < self.events.items.len) {
                const event = self.events.items[index];
                if (event.cycles < cycles) {
                    _ = self.events.swapRemove(index);
                    switch (event.event_type) {
                        .EndOfDMA => {
                            self.end_dma(dc);
                        },
                        .ExternalInterrupt => {
                            dc.raise_external_interrupt(.{ .AICA = 1 }); // FIXME: I'm not sure this is actually what we're meant to do here.
                        },
                    }
                } else {
                    self._cycles_counter -= event.cycles;
                    index += 1;
                }
            }
        }

        if (self.arm7.reset_line == .High) {
            self._cycles_counter += cycles;
            // FIXME: We're not acutally counter ARM7 cycles here (unless all instructions are 1 cycle :^)).
            while (self._cycles_counter >= ARM7CycleRatio) {
                self._cycles_counter -= ARM7CycleRatio;
                //aica_log.debug("arm7: [{X:0>8}] {X:0>8} - {s}", .{ self.arm7.pc().* - 4, self.arm7.instruction_pipeline[0], arm7.ARM7.disassemble(self.arm7.instruction_pipeline[0]) });
                self.arm7.tick();
            }
        }
    }

    pub fn start_dma(self: *AICA, dc: *Dreamcast) void {
        const enabled = dc.read_hw_register(u32, .SB_ADEN);
        if (enabled == 0) return;

        const aica_addr = dc.read_hw_register(u32, .SB_ADSTAG);
        const root_bus_addr = dc.read_hw_register(u32, .SB_ADSTAR);
        const len_reg = dc.read_hw_register(u32, .SB_ADLEN);
        var len = len_reg & 0x7FFFFFFF;
        const direction = dc.read_hw_register(u32, .SB_ADDIR);
        aica_log.info(" AICA G2-DMA Start!", .{});
        aica_log.debug("   AICA Address: 0x{X:0>8}", .{aica_addr});
        aica_log.debug("   Root Bus Address: 0x{X:0>8}", .{root_bus_addr});
        aica_log.debug("   Length: 0x{X:0>8} (0x{X:0>8})", .{ len_reg, len });
        aica_log.debug("   Direction: 0x{X:0>8}", .{direction});
        aica_log.debug("   Trigger Select: 0x{X:0>8}", .{dc.read_hw_register(u32, .SB_ADTSEL)});
        aica_log.debug("   Enable: 0x{X:0>8}", .{enabled});

        // NOTE: Wrong start adresses should raise exception, but we don't emulate them, yet.
        if (aica_addr < 0x00800000 or aica_addr >= 0x00A00000) {
            aica_log.err(termcolor.red("AICA DMA out of bounds! AICA Address: 0x{X:0>8}. Canceled."), .{aica_addr});
            return;
        }
        if (root_bus_addr < 0x0C000000 or root_bus_addr >= 0x0D000000) {
            aica_log.err(termcolor.red("AICA DMA out of bounds! Root Bus Address: 0x{X:0>8}. Canceled."), .{root_bus_addr});
            return;
        }

        // FIXME: I have no idea how to correctly handle this error case.
        if (aica_addr + 4 * len >= 0x00A00000) {
            aica_log.err(termcolor.red("AICA DMA out of bounds! AICA Address: 0x{X:0>8}, length: 0x{X:0>8} => 0x{X:0>8} (Manually kept in bounds)"), .{ aica_addr, len, aica_addr + 4 * len });
            len = (0x00A00000 - aica_addr) / 4;
        }
        if (root_bus_addr + 4 * len >= 0x0D000000) {
            aica_log.err(termcolor.red("AICA DMA out of bounds! Root Bus Address: 0x{X:0>8}, length: 0x{X:0>8} => 0x{X:0>8} (Manually kept in bounds)"), .{ root_bus_addr, len, root_bus_addr + 4 * len });
            len = (0x0D000000 - root_bus_addr) / 4;
        }

        const physical_root_addr = dc.cpu._get_memory(root_bus_addr);
        const physical_aica_addr = &self.wave_memory[aica_addr - 0x00800000];

        // TODO: This might raise some exceptions, if the addresses are wrong.

        if (direction == 0) {
            // DMA transfer from the Root Bus to a G2 device
            const src = physical_root_addr;
            const dst = physical_aica_addr;
            @memcpy(@as([*]u32, @ptrCast(@alignCast(dst)))[0..len], @as([*]u32, @ptrCast(@alignCast(src)))[0..len]);
        } else {
            // DMA transfer from a G2 device to the Root Bus
            const src = physical_aica_addr;
            const dst = physical_root_addr;
            @memcpy(@as([*]u32, @ptrCast(@alignCast(dst)))[0..len], @as([*]u32, @ptrCast(@alignCast(src)))[0..len]);
        }

        // Signals the DMA is in progress
        dc.hw_register(u32, .SB_ADST).* = 1;
        dc.hw_register(u32, .SB_ADSUSP).* &= 0b101111; // Clear "DMA Suspend or DMA Stop"

        // Schedule the end of the transfer interrupt
        self.events.append(.{ .event_type = .EndOfDMA, .cycles = 10 * len }) catch unreachable; // FIXME: Compute the actual cycle count.
    }

    fn end_dma(_: *AICA, dc: *Dreamcast) void {
        const suspended = dc.read_hw_register(u32, .SB_ADSUSP);
        if ((suspended & 1) == 1) {
            // The DMA is suspended, wait.
            // FIXME: This is probably not how it's supposed to work.
            return;
        }

        const len_reg = dc.read_hw_register(u32, .SB_ADLEN);
        const dma_end = (len_reg & 0x80000000) != 0; // DMA Transfer End//Restart
        const len = len_reg & 0x7FFFFFFF;
        // When a transfer ends, the DMA enable register is set to "0".
        if (dma_end)
            dc.hw_register(u32, .SB_ADEN).* = 0;

        dc.hw_register(u32, .SB_ADSTAR).* += len;
        dc.hw_register(u32, .SB_ADSTAG).* += len;
        dc.hw_register(u32, .SB_ADLEN).* = 0;
        dc.hw_register(u32, .SB_ADST).* = 0;

        // Set "DMA Suspend or DMA Stop" bit in SB_ADSUSP
        dc.hw_register(u32, .SB_ADSUSP).* |= 0b010000;

        dc.raise_normal_interrupt(.{ .EoD_AICA = 1 });
        dc.raise_external_interrupt(.{ .AICA = 1 });
    }
};
