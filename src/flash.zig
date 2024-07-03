const std = @import("std");
const termcolor = @import("termcolor");

const flash_log = std.log.scoped(.flash_log);

// MBM29LV002TC (29LV002TC-90PTN)
// NOTE: Special addresses in the documentation are 0x555 and 0x2AA, however the boot ROM uses 0x5555 and 0x2AAA.
//       Maybe I'm missing a mask somewhere?
const CommandAddr0 = 0x5555;
const CommandAddr1 = 0x2AAA;

pub const Flash = struct {
    mode: enum { Normal, Program, Fast, FastProgram, FastReset } = .Normal,

    write_cycle: u8 = 0,

    data: []u8 align(4),

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        return @This(){
            .data = try allocator.alloc(u8, 0x20000),
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.free(self.data);
    }

    pub fn write(self: *@This(), addr: u32, data: u8) void {
        std.debug.assert(addr <= 0x1FFFF);

        switch (self.write_cycle) {
            0 => {
                if (self.mode == .Fast and data == 0xA0) {
                    self.mode = .FastProgram;
                    self.write_cycle = 1;
                    flash_log.debug("Fast Program: @{X:0>8} = 0x{X:0>2}", .{ addr, data });
                } else if (self.mode == .Fast and data == 0x90) {
                    self.write_cycle = 1;
                } else if (addr == CommandAddr0 and data == 0xAA) {
                    self.write_cycle = 1;
                } else if (data == 0xF0) {
                    // Read/Reset?
                    self.write_cycle = 0;
                    self.mode = .Normal;
                } else {
                    self.unexpected_write(addr, data);
                }
            },
            1 => {
                if (self.mode == .FastProgram) {
                    // Fast Program
                    self.write_cycle = 0;
                    self.mode = .Fast;
                    self.data[addr] = data;
                    flash_log.debug("Program: @{X:0>8} = 0x{X:0>2}", .{ addr, data });
                } else if (self.mode == .FastReset and data == 0x00 or data == 0xF0) {
                    // Reset from Fast Mode
                    self.write_cycle = 0;
                    self.mode = .Normal;
                } else if (addr == CommandAddr1 and data == 0x55) {
                    self.write_cycle = 2;
                } else {
                    self.unexpected_write(addr, data);
                }
            },
            2 => {
                if (addr == CommandAddr0) {
                    switch (data) {
                        0xF0 => {
                            // Read/Reset
                            self.write_cycle = 0;
                        },
                        0x90 => {
                            // Autoselect
                            // TODO?
                            self.write_cycle = 0;
                        },
                        0xA0 => {
                            // Program
                            self.mode = .Program;
                            self.write_cycle = 3;
                        },
                        0x80 => {
                            // Chip or Sector Erase
                            self.write_cycle = 3;
                        },
                        0x20 => { // Set to Fast Mode
                            self.mode = .Fast;
                            self.write_cycle = 0;
                            flash_log.debug("Fast Mode enabled.", .{});
                        },
                        else => self.unexpected_write(addr, data),
                    }
                } else {
                    self.unexpected_write(addr, data);
                }
            },
            3 => {
                switch (self.mode) {
                    .Normal => {
                        if (addr == CommandAddr0 and data == 0xAA) {
                            self.write_cycle = 4;
                        } else self.unexpected_write(addr, data);
                    },
                    .Program => {
                        self.mode = .Normal;
                        self.write_cycle = 0;
                        self.data[addr] = data;
                    },
                    else => self.unexpected_write(addr, data),
                }
            },
            4 => {
                if (addr == CommandAddr1 and data == 0x55) {
                    self.write_cycle = 5;
                } else self.unexpected_write(addr, data);
            },
            5 => {
                if (addr == CommandAddr0 and data == 0x10) {
                    // Chip Erase
                    // TODO
                    flash_log.warn(termcolor.yellow("Chip Erase"), .{});
                    self.write_cycle = 0;
                } else if (data == 0x30) {
                    // Sector Erase
                    // TODO
                    flash_log.warn(termcolor.yellow("Sector Erase"), .{});
                    self.write_cycle = 0;
                } else self.unexpected_write(addr, data);
            },
            else => self.unexpected_write(addr, data),
        }
    }

    fn unexpected_write(self: *@This(), addr: u32, data: u16) void {
        flash_log.warn("Unexpected write to FlashROM (write cycle: {}): @{X:0>8} = {X:0>4}", .{ self.write_cycle, addr, data });
        self.write_cycle = 0;
        self.mode = .Normal;
    }
};
