const std = @import("std");
const termcolor = @import("termcolor.zig");

const aica_log = std.log.scoped(.aica);

const Dreamcast = @import("dreamcast.zig").Dreamcast;

// Yamaha AICA Audio Chip

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
// 00000000 - 007FFFFF   DRAM_AREA*
// 00800000 - 008027FF   CHANNEL_DATA
// 00802800 - 00802FFF   COMMON_DATA
// 00803000 - 00807FFF   DSP_DATA

pub const AICA = struct {
    regs: []u32 = undefined, // All registers are 32-bit afaik
    wave_memory: []u8 align(4) = undefined,

    rtc_write_enabled: bool = false,

    dma_countdown: u32 = 0,

    _allocator: std.mem.Allocator = undefined,

    pub fn init(allocator: std.mem.Allocator) !AICA {
        return .{
            .regs = try allocator.alloc(u32, 0x8000 / 4),
            .wave_memory = try allocator.alloc(u8, 0x200000),
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *AICA) void {
        self._allocator.free(self.regs);
        self._allocator.free(self.wave_memory);
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

    pub fn read_register(self: *const AICA, addr: u32) u32 {
        aica_log.debug("Read from AICA at 0x{X:0>8} = 0x{X:0>8}", .{ addr, self.regs[(addr - AICARegisterStart) / 4] });
        return self.regs[(addr - AICARegisterStart) / 4];
    }

    pub fn write_register(self: *AICA, addr: u32, value: u32) void {
        aica_log.info("Write to AICA at 0x{X:0>8} = 0x{X:0>8}", .{ addr, value });
        self.regs[(addr - AICARegisterStart) / 4] = value;
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

    pub fn update(self: *AICA, dc: *Dreamcast, cycles: u32) void {
        if (self.dma_countdown > 0) {
            if (self.dma_countdown <= cycles) {
                self.end_dma(dc);
            } else {
                self.dma_countdown -= cycles;
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
        self.dma_countdown = 10 * len; // FIXME: Compute the actual cycle count.
    }

    fn end_dma(self: *AICA, dc: *Dreamcast) void {
        const suspended = dc.read_hw_register(u32, .SB_ADSUSP);
        if ((suspended & 1) == 1) {
            // The DMA is suspended, wait.
            // FIXME: This is probably not how it's supposed to work.
            return;
        }

        self.dma_countdown = 0;

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
