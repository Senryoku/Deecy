const std = @import("std");

const SH4 = @import("sh4.zig").SH4;

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

pub const AICA = struct {
    wave_memory: [0x200000]u8 = undefined,

    rtc_write_enabled: bool = false,

    dma_countdown: u32 = 0,

    pub fn read_register(self: *const AICA, addr: u32) u32 {
        _ = self;
        std.debug.print("Read from AICA at 0x{X:0>8}\n", .{addr});

        return 0;
    }

    pub fn write_register(self: *AICA, addr: u32, value: u32) void {
        _ = self;
        std.debug.print("Write to AICA at 0x{X:0>8} = 0x{X:0>8}\n", .{ addr, value });
    }

    pub fn read_rtc_register(self: *const AICA, addr: u32) u32 {
        _ = self;
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

    pub fn update(self: *AICA, cpu: *SH4, cycles: u32) void {
        if (self.dma_countdown > 0) {
            if (self.dma_countdown <= cycles) {
                self.end_dma(cpu);
            } else {
                self.dma_countdown -= cycles;
            }
        }
    }

    pub fn start_dma(self: *AICA, cpu: *SH4) void {
        const enabled = cpu.read_hw_register(u32, .SB_ADEN);
        if (enabled == 0) return;

        const g2_addr = cpu.read_hw_register(u32, .SB_ADSTAR);
        const sys_addr = cpu.read_hw_register(u32, .SB_ADSTAG);
        const len_reg = cpu.read_hw_register(u32, .SB_ADLEN);
        const len = len_reg & 0x7FFFFFFF;
        const direction = cpu.read_hw_register(u32, .SB_ADDIR);
        std.debug.print(" AICA G2-DMA Start!\n", .{});
        std.debug.print("   G2 Start Address: 0x{X:0>8}\n", .{g2_addr});
        std.debug.print("   System Start Address: 0x{X:0>8}\n", .{sys_addr});
        std.debug.print("   Length: 0x{X:0>8} (0x{X:0>8})\n", .{ len_reg, len });
        std.debug.print("   Direction: 0x{X:0>8}\n", .{direction});
        std.debug.print("   Trigger Select: 0x{X:0>8}\n", .{cpu.read_hw_register(u32, .SB_ADTSEL)});
        std.debug.print("   Enable: 0x{X:0>8}\n", .{enabled});

        const physical_g2_addr = cpu._get_memory(g2_addr);
        const physical_sys_addr = cpu._get_memory(sys_addr);

        // TODO: This might raise some exceptions, if the addresses are wrong.

        if (direction == 0) {
            // DMA transfer from the Root Bus to a G2 device
            const src = physical_sys_addr;
            const dst = physical_g2_addr;
            @memcpy(@as([*]u8, @ptrCast(dst))[0..len], @as([*]u8, @ptrCast(src))[0..len]);
            // FIXME: Should not be instant.
        } else {
            // DMA transfer from a G2 device to the Root Bus
            @panic("AICA DMA reversed direction not implemented");
        }

        // Signals the DMA is in progress
        cpu.hw_register(u32, .SB_ADST).* = 1;
        cpu.hw_register(u32, .SB_ADSUSP).* &= 0b101111; // Clear "DMA Suspend or DMA Stop"

        // Schedule the end of the transfer interrupt
        self.dma_countdown = len; // FIXME: Compute the actual cycle count.
    }

    fn end_dma(self: *AICA, cpu: *SH4) void {
        const suspended = cpu.read_hw_register(u32, .SB_ADSUSP);
        if ((suspended & 1) == 1) {
            // The DMA is suspended, wait.
            // FIXME: This is probably not how it's supposed to work.
            return;
        }

        self.dma_countdown = 0;

        const len_reg = cpu.read_hw_register(u32, .SB_ADLEN);
        const dma_end = (len_reg & 0x80000000) != 0; // DMA Transfer End//Restart
        const len = len_reg & 0x7FFFFFFF;
        // When a transfer ends, the DMA enable register is set to "0".
        if (dma_end)
            cpu.hw_register(u32, .SB_ADEN).* = 0;

        cpu.hw_register(u32, .SB_ADSTAR).* += len;
        cpu.hw_register(u32, .SB_ADSTAG).* += len;
        cpu.hw_register(u32, .SB_ADLEN).* = 0;
        cpu.hw_register(u32, .SB_ADST).* = 0;

        // Set "DMA Suspend or DMA Stop" bit in SB_ADSUSP
        cpu.hw_register(u32, .SB_ADSUSP).* |= 0b010000;

        cpu.raise_normal_interrupt(.{ .EoD_AICA = 1 });
    }
};
