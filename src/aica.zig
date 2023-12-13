const std = @import("std");

// Yamaha AICA Audio Chip

pub const AICA = struct {
    wave_memory: [0x200000]u8 = undefined,

    rtc_write_enabled: bool = false,

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
                return @as(u32, @intCast(std.time.timestamp())) & 0x0000FFFFF;
            },
            0x04 => {
                return (@as(u32, @intCast(std.time.timestamp())) >> 16) & 0x0000FFFFF;
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
};
