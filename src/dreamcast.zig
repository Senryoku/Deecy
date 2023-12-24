const std = @import("std");

const common = @import("common.zig");
const addr_t = common.addr_t;
const termcolor = @import("termcolor.zig");

const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;
const P4MemoryRegister = MemoryRegisters.P4MemoryRegister;

const Interrupts = @import("Interrupts.zig");
const Interrupt = Interrupts.Interrupt;

const SH4 = @import("sh4.zig").SH4;
const HollyModule = @import("holly.zig");
const Holly = HollyModule.Holly;
const AICA = @import("aica.zig").AICA;
const MapleHost = @import("maple.zig").MapleHost;
const GDROM = @import("GDRom.zig").GDROM;

const dc_log = std.log.scoped(.dc);

const CableType = enum(u16) {
    VGA = 0,
    RGB = 2,
    Composite = 3,
};

pub const Dreamcast = struct {
    cpu: SH4,
    gpu: Holly,
    aica: AICA,
    maple: MapleHost,
    gdrom: GDROM,

    // Pluged in video cable reported to the CPU:e.
    cable_type: CableType = .VGA,

    boot: []u8 align(4) = undefined,
    flash: []u8 align(4) = undefined,
    ram: []u8 align(4) = undefined,
    hardware_registers: []u8 align(4) = undefined, // FIXME

    _allocator: std.mem.Allocator = undefined,

    _dummy: [4]u8 align(32) = undefined, // FIXME: Dummy space for non-implemented features

    pub fn create(allocator: std.mem.Allocator) !*Dreamcast {
        const dc = try allocator.create(Dreamcast);
        dc.* = Dreamcast{
            .cpu = try SH4.init(allocator, dc),
            .gpu = try Holly.init(allocator),
            .aica = try AICA.init(allocator),
            .maple = MapleHost.init(),
            .gdrom = GDROM.init(),
            .ram = try allocator.alloc(u8, 16 * 1024 * 1024),
            .hardware_registers = try allocator.alloc(u8, 0x200000), // FIXME: Huge waste of memory.
            ._allocator = allocator,
        };

        // Load ROM
        dc.boot = try dc._allocator.alloc(u8, 0x200000);
        var boot_file = try std.fs.cwd().openFile("./bin/dc_boot.bin", .{});
        defer boot_file.close();
        const bytes_read = try boot_file.readAll(dc.boot);
        std.debug.assert(bytes_read == 0x200000);

        // Load Flash
        dc.flash = try dc._allocator.alloc(u8, 0x20000);
        var flash_file = try std.fs.cwd().openFile("./bin/dc_flash.bin", .{});
        defer flash_file.close();
        const flash_bytes_read = try flash_file.readAll(dc.flash);
        std.debug.assert(flash_bytes_read == 0x20000);

        dc.reset();

        return dc;
    }

    pub fn deinit(self: *@This()) void {
        self.aica.deinit();
        self.gpu.deinit();
        self.cpu.deinit();
        self._allocator.free(self.flash);
        self._allocator.free(self.boot);
        self._allocator.free(self.hardware_registers);
        self._allocator.free(self.ram);
    }

    pub fn reset(self: *@This()) void {
        self.cpu.reset();
        self.gpu.reset();

        self.hw_register(u32, .SB_FFST).* = 0; // FIFO Status
        self.hw_register(u32, .SB_ISTNRM).* = 0;
    }

    pub fn load_at(self: *@This(), addr: addr_t, bin: []const u8) void {
        const start_addr = ((addr & 0x1FFFFFFF) - 0x0C000000);
        @memcpy(self.ram[start_addr .. start_addr + bin.len], bin);
    }

    pub fn skip_bios(self: *@This()) void {
        self.cpu.skip_bios();

        @memset(self.ram[0x00200000..0x00300000], 0x00); // FIXME: I think KallistiOS relies on that, or maybe I messed up somewhere else. (the BootROM does clear this section of RAM)

        // Copy subroutine to RAM. Some of it will be overwritten, I'm trying to work out what's important and what's not.
        inline for (0..16) |i| {
            self.write16(0x8C0000E0 + 2 * i, self.read16(0x800000FE - 2 * i));
        }
        // Copy a portion of the boot ROM to RAM.
        self.write32(0xA05F74E4, 0x001FFFFF);
        // @memcpy(self.ram[0x00000100 .. 0x100 + 0x0007FFC0], self.boot[0x00000100 .. 0x100 + 0x0007FFC0]);
        @memcpy(self.ram[0x00000100..0x00004000], self.boot[0x00000100..0x00004000]);
        @memcpy(self.ram[0x00008000..0x00200000], self.boot[0x00008000..0x00200000]);

        const IP_bin_HLE = false;
        if (IP_bin_HLE) {
            // Copy a portion of the flash ROM to RAM.
            inline for (0..8) |i| {
                self.write8(0x8C000068 + i, self.read8(0x0021A056 + i));
            }
            inline for (0..5) |i| {
                self.write8(0x8C000068 + 8 + i, self.read8(0x0021A000 + i));
            }
            // FIXME: Load system settings from flashrom (User partition (2), logical block 5), instead of these hardcoded values.
            //inline for (.{ 0xBC, 0xEA, 0x90, 0x5E, 0xFF, 0x04, 0x00, 0x01 }, 0..) |val, i| {
            inline for (.{ 0x00, 0x00, 0x89, 0xFC, 0x5B, 0xFF, 0x01, 0x00, 0x00, 0x7D, 0x0A, 0x62, 0x61 }, 0..) |val, i| {
                self.write8(0x8C000068 + 13 + i, val);
            }
        }

        // Patch some function adresses ("syscalls")

        const HLE_syscalls = true;
        if (HLE_syscalls) {
            // System
            self.write32(0x8C0000B0, 0x8C001000);
            self.write16(0x8C001000, 0b0000000000010000);
            // Font
            self.write32(0x8C0000B4, 0x8C001002);
            self.write16(0x8C001002, 0b0000000000100000);
            // Flashrom
            self.write32(0x8C0000B8, 0x8C001004);
            self.write16(0x8C001004, 0b0000000000110000);
            // GD
            self.write32(0x8C0000BC, 0x8C001006);
            self.write16(0x8C001006, 0b0000000001000000);
            // GD2
            self.write32(0x8C0000C0, 0x8C0010F0);
            self.write16(0x8C0010F0, 0b0000000001010000);
            // Misc
            self.write32(0x8C0000E0, 0x8C001008);
            self.write16(0x8C001008, 0b0000000001100000);
        } else {
            inline for (.{
                .{ 0x8C0000B0, 0x8C003C00 },
                .{ 0x8C0000B4, 0x8C003D80 },
                .{ 0x8C0000B8, 0x8C003D00 },
                .{ 0x8C0000BC, 0x8C001000 },
                .{ 0x8C0000C0, 0x8C0010F0 },
                .{ 0x8C0000E0, 0x8C000800 },
            }) |p| {
                self.write32(p[0], p[1]);
            }
        }

        // Other set values, IDK

        inline for (.{
            .{ 0x8C0000AC, 0xA05F7000 },
            .{ 0x8C0000A8, 0xA0200000 },
            .{ 0x8C0000A4, 0xA0100000 },
            .{ 0x8C0000A0, 0x00000000 },
            .{ 0x8C00002C, 0x00000000 },
            .{ 0x8CFFFFF8, 0x8C000128 },
        }) |p| {
            self.write32(p[0], p[1]);
        }

        // Patch some functions apparently used by interrupts
        // (And some other random stuff that the boot ROM sets for some reason
        //  and I'm afraid some games might use. I'm not taking any more chances)

        // Sleep on error?
        self.write32(0x8C000000, 0x00090009);
        self.write32(0x8C000004, 0x001B0009);
        self.write32(0x8C000008, 0x0009AFFD);
        // ??
        self.write16(0x8C00000C, 0);
        self.write16(0x8C00000E, 0);
        // RTE - Some interrupts jump there instead of having their own RTE, I have NO idea why.
        self.write32(0x8C000010, 0x00090009); // nop nop
        self.write32(0x8C000014, 0x0009002B); // rte nop
        // RTS
        self.write32(0x8C000018, 0x00090009);
        self.write32(0x8C00001C, 0x0009000B);

        // ??
        self.write8(0x8C00002C, 0x16);
        self.write32(0x8C000064, 0x8c008100);
        self.write16(0x8C000090, 0);
        self.write16(0x8C000092, @bitCast(@as(i16, -128)));

        // Holly Version. TODO: Make it configurable?
        self.hw_register(u32, .SB_SBREV).* = 0x0B;
        self.hw_register(u32, .SB_G2ID).* = 0x12; // Only possible value, apparently.
    }

    pub fn hw_register(self: *@This(), comptime T: type, r: MemoryRegister) *T {
        return @as(*T, @alignCast(@ptrCast(&self.hardware_registers[(@intFromEnum(r) & 0x1FFFFFFF) - 0x005F6800])));
    }
    pub fn read_hw_register(self: @This(), comptime T: type, r: MemoryRegister) T {
        return @as(*T, @alignCast(@ptrCast(&self.hardware_registers[(@intFromEnum(r) & 0x1FFFFFFF) - 0x005F6800]))).*;
    }

    pub fn tick(self: *@This()) void {
        const cycles = self.cpu.execute();
        self.gpu.update(self, cycles);
        self.aica.update(self, cycles);
    }

    fn check_sb_interrupts(self: *@This()) void {
        // FIXME: Not sure if this is the right place to check for those.
        const istnrm = self.read_hw_register(u32, .SB_ISTNRM);
        const istext = self.read_hw_register(u32, .SB_ISTEXT);
        const isterr = self.read_hw_register(u32, .SB_ISTERR);
        if (istnrm & self.read_hw_register(u32, .SB_IML6NRM) != 0 or istext & self.read_hw_register(u32, .SB_IML6EXT) != 0 or isterr & self.read_hw_register(u32, .SB_IML6ERR) != 0) {
            self.cpu.request_interrupt(Interrupts.Interrupt.IRL9);
        }
        if (istnrm & self.read_hw_register(u32, .SB_IML4NRM) != 0 or istext & self.read_hw_register(u32, .SB_IML4EXT) != 0 or isterr & self.read_hw_register(u32, .SB_IML4ERR) != 0) {
            self.cpu.request_interrupt(Interrupts.Interrupt.IRL11);
        }
        if (istnrm & self.read_hw_register(u32, .SB_IML2NRM) != 0 or istext & self.read_hw_register(u32, .SB_IML2EXT) != 0 or isterr & self.read_hw_register(u32, .SB_IML2ERR) != 0) {
            self.cpu.request_interrupt(Interrupts.Interrupt.IRL13);
        }
    }

    // TODO: Add helpers for external interrupts and errors.

    pub fn raise_normal_interrupt(self: *@This(), int: MemoryRegisters.SB_ISTNRM) void {
        self.hw_register(u32, .SB_ISTNRM).* |= @bitCast(int);

        self.check_sb_interrupts();
    }

    pub fn raise_external_interrupt(self: *@This(), int: MemoryRegisters.SB_ISTEXT) void {
        self.hw_register(u32, .SB_ISTEXT).* |= @bitCast(int);
        self.hw_register(u32, .SB_ISTNRM).* |= @bitCast(MemoryRegisters.SB_ISTNRM{ .ExtStatus = if (self.hw_register(u32, .SB_ISTEXT).* != 0) 1 else 0 });

        self.check_sb_interrupts();
    }

    fn panic_debug(self: @This(), comptime fmt: []const u8, args: anytype) noreturn {
        std.debug.print("PC: {X:0>8}\n", .{self.cpu.pc});
        std.debug.panic(fmt, args);
        @panic(fmt);
    }

    // Area 0:
    // 0x00000000 - 0x001FFFFF Boot ROM
    // 0x00200000 - 0x0021FFFF Flash Memory
    // 0x005F6800 - 0x005F69FF System Control Reg.
    // 0x005F6C00 - 0x005F6CFF Maple Control Reg.
    // 0x005F7000 - 0x005F70FF GD-ROM
    // 0x005F7400 - 0x005F74FF G1 Control Reg.
    // 0x005F7800 - 0x005F78FF G2 Control Reg.
    // 0x005F7C00 - 0x005F7CFF PVR Control Reg.
    // 0x005F8000 - 0x005F9FFF TA/PVR Core Reg.
    // 0x00600000 - 0x006007FF MODEM
    // 0x00700000 - 0x00707FFF AICA sound Reg.
    // 0x00710000 - 0x00710007 AICA RTC Reg.
    // 0x00800000 - 0x009FFFFF AICA Memory
    // 0x01000000 - 0x01FFFFFF G2 External Device #1
    // 0x02700000 - 0x02FFFFE0 G2 AICA (Image area)
    // 0x03000000 - 0x03FFFFE0 G2 External Device #2

    pub fn _get_memory(self: *@This(), addr: addr_t) *u8 {
        std.debug.assert(addr == addr & 0x1FFFFFFF);

        if (false) {
            // MMU: Looks like most game don't use it at all. TODO: Expose it as an option.
            const physical_addr = self.mmu_translate_utbl(addr) catch |e| {
                // FIXME: Handle exceptions
                dc_log.err("\u{001B}[31mError in utlb _read: {any} at {X:0>8}\u{001B}[0m", .{ e, addr });
                unreachable;
            };

            if (physical_addr != addr)
                dc_log.info("  Write UTLB Hit: {x:0>8} => {x:0>8}", .{ addr, physical_addr });
        }

        switch (addr) {
            0x00000000...0x03FFFFFF => { // Area 0 - Boot ROM, Flash ROM, Hardware Registers
                switch (addr) {
                    0x00000000...0x001FFFFF => {
                        return &self.boot[addr];
                    },
                    0x00200000...0x0021FFFF => {
                        return &self.flash[addr - 0x200000];
                    },
                    0x005F6800...0x005F7FFF => {
                        return &self.hardware_registers[addr - 0x005F6800];
                    },
                    0x005F8000...0x005F9FFF => {
                        return self.gpu._get_register_from_addr(u8, addr);
                    },
                    0x005FA000...0x005FFFFF => {
                        return &self.hardware_registers[addr - 0x005F6800];
                    },
                    0x00600000...0x006007FF => {
                        const static = struct {
                            var once = false;
                        };
                        if (!static.once) {
                            static.once = true;
                            dc_log.warn(termcolor.yellow("  Unimplemented _get_memory to MODEM: {X:0>8} (This will only be reported once)"), .{addr});
                        }
                        return @ptrCast(&self._dummy);
                    },
                    0x00700000...0x00707FE0 => { // G2 AICA Register
                        @panic("_get_memory to AICA Register. This should be handled in read/write functions.");
                    },
                    0x00710000...0x00710008 => { // G2 AICA RTC Registers
                        @panic("_get_memory to AICA RTC Register. This should be handled in read/write functions.");
                    },
                    0x00800000...0x009FFFFF => { // G2 Wave Memory
                        return &self.aica.wave_memory[addr - 0x00800000];
                    },
                    else => {
                        dc_log.warn(termcolor.yellow("  Unimplemented _get_memory to Area 0: {X:0>8}"), .{addr});
                        return @ptrCast(&self._dummy);
                    },
                }
            },
            0x04000000...0x07FFFFFF => {
                return self.gpu._get_vram(addr);
            },
            0x08000000...0x0BFFFFFF => { // Area 2 - Nothing
                self.panic_debug("Invalid _get_memory to Area 2 @{X:0>8}", .{addr});
            },
            0x0C000000...0x0FFFFFFF => { // Area 3 - System RAM (16MB) - 0x0C000000 to 0x0FFFFFFF, mirrored 4 times, I think.
                return &self.ram[addr & 0x00FFFFFF];
            },
            0x10000000...0x13FFFFFF => { // Area 4 - Tile accelerator command input
                self.panic_debug("Unexpected _get_memory to Area 4 @{X:0>8} - This should only be accessible via write32 or DMA.", .{addr});
            },
            0x14000000...0x17FFFFFF => { // Area 5 - Expansion (modem) port
                const static = struct {
                    var once = false;
                };
                if (!static.once) {
                    static.once = true;
                    dc_log.warn(termcolor.yellow("Unimplemented _get_memory to Area 5: {X:0>8} (This will only be reported once)"), .{addr});
                }
                return @ptrCast(&self._dummy);
            },
            0x18000000...0x1BFFFFFF => { // Area 6 - Nothing
                self.panic_debug("Invalid _get_memory to Area 6 @{X:0>8}", .{addr});
            },
            0x1C000000...0x1FFFFFFF => { // Area 7 - Internal I/O registers (same as P4)
                std.debug.assert(self.cpu.sr.md == 1);
                return self.cpu.p4_register_addr(u8, addr);
            },
            else => {
                unreachable;
            },
        }
    }

    pub fn read8(self: @This(), virtual_addr: addr_t) u8 {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                else => {
                    dc_log.debug("  Read8 to hardware register @{X:0>8} {s} = 0x{X:0>2}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), @as(*const u8, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
                    ))).* });
                },
            }
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            dc_log.debug("  Read8 to hardware register @{X:0>8} {s} ", .{ addr, MemoryRegisters.getRegisterName(addr) });
        }

        return @as(*const u8, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub fn read16(self: @This(), virtual_addr: addr_t) u16 {
        const addr = virtual_addr & 0x1FFFFFFF;

        // SH4 Hardware registers
        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                @intFromEnum(P4MemoryRegister.RTCSR), @intFromEnum(P4MemoryRegister.RTCNT), @intFromEnum(P4MemoryRegister.RTCOR) => {
                    return @as(*const u16, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
                    ))).* & 0xF;
                },
                @intFromEnum(P4MemoryRegister.RFCR) => {
                    // Hack: This is the Refresh Count Register, related to DRAM control.
                    //       If don't think its proper emulation is needed, but it's accessed by the bios,
                    //       probably for synchronization purposes. I assume returning a contant value to pass this check
                    //       is enough for now, as games shouldn't access that themselves.
                    dc_log.debug("[Note] Access to Refresh Count Register.", .{});
                    return 0x0011;
                    // Otherwise, this is 10-bits register, respond with the 6 unused upper bits set to 0.
                },
                @intFromEnum(P4MemoryRegister.PDTRA) => {
                    // Note: I have absolutely no idea what's going on here.
                    //       This is directly taken from Flycast, which already got it from Chankast.
                    //       This is needed for the bios to work properly, without it, it will
                    //       go to sleep mode with all interrupts disabled early on.
                    const tpctra: u32 = self.cpu.read_p4_register(u32, .PCTRA);
                    const tpdtra: u32 = self.cpu.read_p4_register(u32, .PDTRA);

                    var tfinal: u16 = 0;
                    if ((tpctra & 0xf) == 0x8) {
                        tfinal = 3;
                    } else if ((tpctra & 0xf) == 0xB) {
                        tfinal = 3;
                    } else {
                        tfinal = 0;
                    }

                    if ((tpctra & 0xf) == 0xB and (tpdtra & 0xf) == 2) {
                        tfinal = 0;
                    } else if ((tpctra & 0xf) == 0xC and (tpdtra & 0xf) == 2) {
                        tfinal = 3;
                    }

                    tfinal |= @intFromEnum(self.cable_type) << 8;

                    return tfinal;
                },
                else => {
                    dc_log.debug("  Read16 to P4 register @{X:0>8} {s} = {X:0>4}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), @as(*const u16, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
                    ))).* });
                },
            }
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            dc_log.debug("  Read16 to hardware register @{X:0>8} {s} ", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr) });
        }

        if (addr >= 0x00710000 and addr <= 0x00710008) {
            return @truncate(self.aica.read_rtc_register(addr));
        }

        return @as(*const u16, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub fn read32(self: @This(), virtual_addr: addr_t) u32 {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                else => {
                    dc_log.debug("  Read32 to P4 register @{X:0>8} {s} = 0x{X:0>8}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), @as(*const u32, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
                    ))).* });
                },
            }
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            dc_log.debug("  Read32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, MemoryRegisters.getRegisterName(addr), @as(*const u32, @alignCast(@ptrCast(
                @constCast(&self)._get_memory(addr),
            ))).* });
        }

        if (addr >= 0x00700000 and addr <= 0x00707FE0) {
            return self.aica.read_register(addr);
        }

        if (addr >= 0x00710000 and addr <= 0x00710008) {
            return self.aica.read_rtc_register(addr);
        }

        return @as(*const u32, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub fn read64(self: @This(), virtual_addr: addr_t) u64 {
        const addr = virtual_addr & 0x1FFFFFFF;
        return @as(*const u64, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub fn write8(self: *@This(), virtual_addr: addr_t, value: u8) void {
        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                else => {
                    dc_log.debug("  Write8 to P4 register @{X:0>8} {s} = 0x{X:0>2}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), value });
                },
            }
        }

        const addr = virtual_addr & 0x1FFFFFFF;
        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            // Hardware registers
            switch (addr) {
                else => {
                    dc_log.debug("  Write8 to hardware register @{X:0>8} {s} = 0x{X:0>2}", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                },
            }
        }
        if (addr >= 0x005F8000 and addr < 0x005FA000) {
            @panic("write8 to GPU register not implemented");
        }
        if (addr >= 0x10000000 and addr < 0x14000000) {
            @panic("write8 to TA not implemented");
        }

        @as(*u8, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub fn write16(self: *@This(), virtual_addr: addr_t, value: u16) void {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                @intFromEnum(P4MemoryRegister.RTCSR), @intFromEnum(P4MemoryRegister.RTCNT), @intFromEnum(P4MemoryRegister.RTCOR) => {
                    std.debug.assert(value & 0xFF00 == 0b10100101_00000000);
                    @as(*u16, @alignCast(@ptrCast(
                        self._get_memory(addr),
                    ))).* = 0b10100101_00000000 | (value & 0xFF);
                },
                @intFromEnum(P4MemoryRegister.RFCR) => {
                    std.debug.assert(value & 0b11111100_00000000 == 0b10100100_00000000);
                    @as(*u16, @alignCast(@ptrCast(
                        self._get_memory(addr),
                    ))).* = 0b10100100_00000000 | (value & 0b11_11111111);
                },
                else => {
                    dc_log.debug("  Write16 to P4 register @{X:0>8} {s} = 0x{X:0>4}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), value });
                },
            }
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            switch (addr) {
                else => {
                    dc_log.debug("  Write16 to hardware register @{X:0>8} {s} = 0x{X:0>4}", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                },
            }
        }
        if (addr >= 0x005F8000 and addr < 0x005FA000) {
            @panic("write16 to GPU register not implemented");
        }
        if (addr >= 0x10000000 and addr < 0x14000000) {
            @panic("write16 to TA not implemented");
        }

        @as(*u16, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub fn write32(self: *@This(), virtual_addr: addr_t, value: u32) void {
        if (virtual_addr >= 0xE0000000) {
            // P4
            if (virtual_addr < 0xE4000000) {
                self.cpu.store_queue_write(virtual_addr, value);
                return;
            }
            if (virtual_addr >= 0xFF000000) {
                switch (virtual_addr) {
                    else => {
                        dc_log.debug("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), value });
                    },
                }
            }
        }

        const addr = virtual_addr & 0x1FFFFFFF;
        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            // Hardware registers
            switch (addr) {
                @intFromEnum(MemoryRegister.SB_SFRES) => {
                    // SB_SFRES, Software Reset
                    if (value == 0x00007611) {
                        self.cpu.software_reset();
                    }
                    return;
                },
                @intFromEnum(MemoryRegister.SB_ADST) => {
                    if (value == 1) {
                        self.aica.start_dma(self);
                    }
                },
                @intFromEnum(MemoryRegister.SB_MDAPRO) => {
                    // This register specifies the address range for Maple-DMA involving the system (work) memory.
                    // Check "Security code"
                    if (value & 0xFFFF0000 != 0x61550000) return;
                },
                @intFromEnum(MemoryRegister.SB_MDST) => {
                    if (value == 1) {
                        self.start_maple_dma();
                        return;
                    }
                },
                @intFromEnum(MemoryRegister.SB_ISTNRM) => {
                    // Interrupt can be cleared by writing "1" to the corresponding bit.
                    self.hw_register(u32, .SB_ISTNRM).* &= ~(value & 0x3FFFFF);
                    return;
                },
                @intFromEnum(MemoryRegister.SB_ISTERR) => {
                    // Interrupt can be cleared by writing "1" to the corresponding bit.
                    self.hw_register(u32, .SB_ISTERR).* &= ~value;
                    return;
                },
                @intFromEnum(MemoryRegister.SB_C2DSTAT) => {
                    self.hw_register(u32, .SB_C2DSTAT).* = 0x10000000 | (0x03FFFFFF & value);
                    return;
                },
                @intFromEnum(MemoryRegister.SB_C2DST) => {
                    if (value == 1) {
                        self.start_ch2_dma();
                    } else {
                        self.end_ch2_dma();
                    }
                    return;
                },
                else => {
                    dc_log.debug("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                },
            }
        }
        if (addr >= 0x005F8000 and addr < 0x005FA000) {
            return self.gpu.write_register(addr, value);
        }

        if (addr >= 0x00700000 and addr < 0x00710000) {
            return self.aica.write_register(addr, value);
        }

        if (addr >= 0x00710000 and addr < 0x00710008) {
            return self.aica.write_rtc_register(addr, value);
        }

        if (addr >= 0x10000000 and addr < 0x14000000) {
            return self.gpu.write_ta(addr, value);
        }

        @as(*u32, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub fn write64(self: *@This(), virtual_addr: addr_t, value: u64) void {
        // This isn't efficient, but avoids repeating all the logic of write32.
        self.write32(virtual_addr, @truncate(value));
        self.write32(virtual_addr + 4, @truncate(value >> 32));
    }

    pub fn start_maple_dma(self: *@This()) void {
        if (self.hw_register(u32, .SB_MDEN).* == 1) {
            // Note: This is useless since DMA in currently instantaneous, but just in case I need it later...
            self.hw_register(u32, .SB_MDST).* = 1;
            defer self.hw_register(u32, .SB_MDST).* = 0;

            dc_log.info(termcolor.yellow("  Maple-DMA initiation!"), .{});
            const sb_mdstar = self.read_hw_register(u32, .SB_MDSTAR);
            std.debug.assert(sb_mdstar >> 28 == 0 and sb_mdstar & 0x1F == 0);
            self.maple.transfer(self, @as([*]u32, @alignCast(@ptrCast(&self.ram[sb_mdstar - 0x0C000000])))[0..]);
        }
    }

    pub fn start_ch2_dma(self: *@This()) void {
        self.hw_register(u32, .SB_C2DST).* = 1;

        const dst_addr = self.read_hw_register(u32, .SB_C2DSTAT);
        const len = self.read_hw_register(u32, .SB_C2DLEN);

        dc_log.info("  Start ch2-DMA: {X:0>8} -> {X:0>8} ({X:0>8} bytes)", .{ self.cpu.read_p4_register(u32, .SAR2), dst_addr, len });

        std.debug.assert(dst_addr & 0xF8000000 == 0x10000000);
        self.cpu.p4_register(u32, .DAR2).* = dst_addr; // FIXME: Not sure this is correct

        const dmac_len = self.cpu.read_p4_register(u32, .DMATCR2);
        std.debug.assert(32 * dmac_len == len);

        self.cpu.start_dmac(2);

        // TODO: Schedule for later?
        self.hw_register(u32, .SB_C2DSTAT).* += len;
        self.hw_register(u32, .SB_C2DLEN).* = 0;
        self.hw_register(u32, .SB_C2DST).* = 0;

        self.raise_normal_interrupt(.{ .EoD_CH2 = 1 });
    }

    pub fn end_ch2_dma(self: *@This()) void {
        self.hw_register(u32, .SB_C2DST).* = 0;

        // TODO: Actually cancel the DMA, right now they're instantaneous.
    }
};

test "boot" {
    var dc = try Dreamcast.create(common.GeneralAllocator);
    defer {
        dc.deinit();
        common.GeneralAllocator.destroy(dc);
    }

    dc.tick(); // mov 0x0F,R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFFFFFFFF);
    dc.tick(); // shll16 R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFFFF0000);
    dc.tick(); // swap.w R4,R3
    try std.testing.expect(dc.cpu.R(4).* == 0x0000FFFF);
    dc.tick(); // shll8 R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFF000000);
    dc.tick(); // shlr2 R4
    try std.testing.expect(dc.cpu.R(4).* == 0x00003FFF);
    dc.tick(); // shlr2 R4
    try std.testing.expect(dc.cpu.R(4).* == 0x00000FFF);
    // Reads EXPEVT (0xFF000024), 0x00000000 on power up, 0x00000020 on reset.
    dc.tick(); // mov.l @(9, R3),R0
    try std.testing.expect(dc.read32(dc.cpu.R(3).* + (9 << 2)) == dc.cpu.R(0).*);
    dc.tick(); // xor R4,R0
    try std.testing.expect(dc.cpu.R(4).* == 0x00000FFF);
    try std.testing.expect(dc.cpu.R(4).* == 0x00000FFF);
    {
        const prev_mach = dc.cpu.mach;
        dc.tick(); // mulu.w R4,R0
        try std.testing.expect(dc.cpu.mach == prev_mach);
        try std.testing.expect(dc.cpu.macl == 0);
    }
    dc.tick(); // sts R0,MACL
    try std.testing.expect(dc.cpu.R(0).* == 0);
    dc.tick(); // tst R0,R0
    try std.testing.expect(dc.cpu.sr.t);
    dc.tick(); // bf 0x8C010108
    try std.testing.expect(dc.cpu.pc == 0xA0000018);
    dc.tick(); // mov.l R0,@(4,R3) - Write 0x0 to MMUCR @ 0xFF000010
    try std.testing.expect(dc.cpu.R(0).* == dc.read32(0xFF000000 + (4 << 2)));
    dc.tick(); // mov 0x9,R1
    try std.testing.expect(dc.cpu.R(1).* == 0x9);
    dc.tick(); // shll8 R1
    try std.testing.expect(dc.cpu.R(1).* == 0x9 << 8);
    dc.tick(); // add 0x29,R1
    try std.testing.expect(dc.cpu.R(1).* == (0x9 << 8) + 0x29);
    dc.tick(); // mov.l R1, @(7, R3) - Write 0x929 to CCR @ 0xFF00001C
    try std.testing.expect(dc.cpu.R(1).* == dc.read32(0xFF000000 + (0x7 << 2)));
    dc.tick(); // shar R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFF800000);
    try std.testing.expect(!dc.cpu.sr.t);
    dc.tick(); // mov 0x01, R0
    try std.testing.expect(dc.cpu.R(0).* == 0x01);
    dc.tick(); // mov.w R0, @(2, R3) - Write 0x01 to BCR2 @ 0xFF800004
    try std.testing.expect(dc.cpu.R(0).* == dc.read16(0xFF800000 + (0x2 << 1)));
    dc.tick(); // mov 0xFFFFFFC3, R0
    try std.testing.expect(dc.cpu.R(0).* == 0xFFFFFFC3);
    dc.tick(); // shll16 R0
    try std.testing.expect(dc.cpu.R(0).* == 0xFFC30000);
    dc.tick(); // or 0xCD, R0
    try std.testing.expect(dc.cpu.R(0).* == 0xFFC300CD);
    dc.tick(); // shll8 R0
    try std.testing.expect(dc.cpu.R(0).* == 0xC300CD00);
    dc.tick(); // or 0xB0, R0
    try std.testing.expect(dc.cpu.R(0).* == 0xC300CDB0);
    dc.tick(); // shlr R0
    try std.testing.expect(dc.cpu.R(0).* == 0xC300CDB0 >> 1);
    try std.testing.expect(!dc.cpu.sr.t);
    dc.tick(); // mov.l R0, @(3, R3) - Write 0x01 to WCR2 @ 0xFF80000C
    try std.testing.expect(dc.cpu.R(0).* == dc.read32(0xFF800000 + (0x3 << 2)));
    dc.tick(); // mov 0x01, R5
    try std.testing.expect(dc.cpu.R(5).* == 0x01);
    dc.tick(); // rotr R5
    try std.testing.expect(dc.cpu.R(5).* == 0x80000000);
    try std.testing.expect(dc.cpu.sr.t);
    dc.tick(); // add 0x60, R5
    try std.testing.expect(dc.cpu.R(5).* == 0x80000060);
    dc.tick(); // mov R5, R6
    try std.testing.expect(dc.cpu.R(5).* == dc.cpu.R(6).*);
    dc.tick(); // add 0x20, R6
    try std.testing.expect(dc.cpu.R(6).* == 0x80000080);
    dc.tick(); // tst 0x00, R0 - Always tue, right?
    try std.testing.expect(dc.cpu.sr.t);
    dc.tick(); // pref @R5
    // TODO
    dc.tick(); // jmp @R6
    try std.testing.expect(dc.cpu.pc == dc.cpu.R(6).*);

    dc.tick(); // mov.l @(0x2,R5),R0 - Read 0x80000068 (0xA3020008) to R0
    try std.testing.expect(0xA3020008 == dc.cpu.R(0).*);
    try std.testing.expect(dc.read32(dc.cpu.R(5).* + (0x2 << 2)) == dc.cpu.R(0).*);

    dc.tick(); // mov.l R0, @(0, R3) - Write 0xA3020008 to BRC1 @ 0xFF800000
    try std.testing.expect(dc.read32(0xFF800000) == 0xA3020008);
    dc.tick(); // mov.l @(4,R5),R0
    try std.testing.expect(dc.read32(dc.cpu.R(5).* + (0x4 << 2)) == dc.cpu.R(0).*);
    dc.tick(); // mov.l R0, @(2, R3) - Write 0x01110111 to WCR1 @ 0xFF800008
    try std.testing.expect(dc.read32(0xFF800008) == 0x01110111);
    dc.tick(); // add 0x10, R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFF800010);
    dc.tick(); // mov.l @(5, R5), R0 - Read 0x80000078 (0x800A0E24) to R0
    try std.testing.expect(dc.cpu.R(0).* == 0x800A0E24);
    dc.tick(); // mov.l R0, @(1, R3) - Write 0x800A0E24 to MCR
    try std.testing.expect(dc.cpu.p4_register(u32, .MCR).* == 0x800A0E24);

    dc.tick(); // mov.l @(7, R5), R2
    try std.testing.expect(dc.cpu.R(2).* == 0xff940190);
    dc.tick(); // mov.b R2, @R2
    try std.testing.expect(dc.cpu.p4_register(u8, .SDMR).* == 0x90);

    dc.tick(); // mov 0xFFFFFFA4, R0
    dc.tick(); // shll8 R0
    dc.tick(); // mov.w R0, @(12, R3)
    try std.testing.expect(dc.cpu.p4_register(u16, .RFCR).* == 0xA400);

    dc.tick(); // mov.w @(0, R5), R0
    dc.tick(); // mov.w R0, @(10, R3)
    try std.testing.expect(dc.cpu.p4_register(u16, .RTCOR).* == 0xA504);

    dc.tick(); // add H'0c, R0
    dc.tick(); // mov.w R0, @(6, R3)
    try std.testing.expect(dc.cpu.p4_register(u16, .RTCSR).* == 0xA510);

    // while((volatile uint16_t)reg[RFCR] <= 0x0010);

    dc.tick(); // mov 0x10, R6
    dc.tick(); // mov.w @(12, R3), R0 - Load RFCR (Refresh Count Register) to R0
    try std.testing.expect(dc.cpu.R(0).* == 0x11); // Note: Refresh Count Register not implemented, this check passes because we always return 0x11.
    dc.tick(); // cmp/hi R6, R0
    dc.tick(); // bf 0x8C0100A2

    dc.tick(); // mov.w @(1, R5), R0
    dc.tick(); // mov.w R0, @(10, R3)
    try std.testing.expect(dc.cpu.p4_register(u16, .RTCOR).* == 0xa55e);

    dc.tick(); // mov.l @(6, R5), R0
    dc.tick(); // mov.l R0, @(1, R3)
    try std.testing.expect(dc.cpu.p4_register(u32, .MCR).* == 0xc00a0e24);

    dc.tick(); // mov.b R2, @R2
    dc.tick(); // mov.l @(1, R5), R1
    try std.testing.expect(dc.cpu.p4_register(u8, .SDMR).* == 0x90);

    dc.tick(); // mov 0x04, R0
    try std.testing.expect(dc.cpu.R(0).* == 0x04);
    dc.tick(); // swap.b R0, R0
    try std.testing.expect(dc.cpu.R(0).* == 0x0400);
    dc.tick(); // mov.w R0, @R1
    try std.testing.expect(dc.read16(0xA05F7480) == 0x400); // SB_G1RRC

    dc.tick(); // mov.l @(3, R5), R3
    dc.tick(); // mova 0x8C0100E0, R0

    for (0..16) |_| {
        try std.testing.expect(dc.cpu.pc == 0x800000BE);
        dc.tick(); // dt R6
        dc.tick(); // mov.w @R0+, R1
        dc.tick(); // mov.w R1, @-R3
        dc.tick(); // bf 0x8C0100BE
    }

    dc.tick(); // mov.l @R3, R1
    dc.tick(); // jmp @R3
    try std.testing.expect(dc.cpu.pc == 0x8C0000E0);

    dc.tick(); //

    dc.tick(); //
}

test "IP.bin" {
    var dc = try Dreamcast.create(common.GeneralAllocator);
    defer {
        dc.deinit();
        common.GeneralAllocator.destroy(dc);
    }

    // Example IP.bin file
    const IPbin_file = try std.fs.cwd().openFile("./bin/IP.bin", .{});
    defer IPbin_file.close();
    const IPbin = try IPbin_file.readToEndAlloc(std.testing.allocator, 0x10000);
    defer std.testing.allocator.free(IPbin);
    dc.load_at(0x8C008000, IPbin);

    for (0..10000000) |_| {
        dc.tick();
    }
}

test "IP.bin init boot" {
    var dc = try Dreamcast.create(common.GeneralAllocator);
    defer {
        dc.deinit();
        common.GeneralAllocator.destroy(dc);
    }

    dc.skip_bios();

    // Example IP.bin file
    const IPbin_file = try std.fs.cwd().openFile("./bin/IP.bin", .{});
    defer IPbin_file.close();
    const IPbin = try IPbin_file.readToEndAlloc(std.testing.allocator, 0x10000);
    defer std.testing.allocator.free(IPbin);
    dc.load_at(0x8C008000, IPbin);

    for (0..10000000) |_| {
        dc.tick();
    }
}

// Loads a binary at 0x8C080000, set R14 to 1, and executes it until R14 == 0 (success condition)
fn load_and_test_binary(comptime filename: []const u8) !void {
    var dc = try Dreamcast.create(common.GeneralAllocator);
    defer {
        dc.deinit();
        common.GeneralAllocator.destroy(dc);
    }

    const bin_file = try std.fs.cwd().openFile("./test/bin/" ++ filename, .{});
    defer bin_file.close();
    const bin = try bin_file.readToEndAlloc(std.testing.allocator, 0x10000);
    defer std.testing.allocator.free(bin);
    dc.load_at(0x8C080000, bin);

    dc.cpu.pc = 0x8C080000;
    dc.cpu.R(14).* = 1;

    var prev = dc.cpu.pc;
    while (dc.cpu.R(14).* != 0) {
        dc.tick();
        try std.testing.expect(dc.cpu.pc != prev); // Crude check for infinite loops, there might be legitiple reason to do this (loops with process in delay slot?), but we'll just choose not to and be fine :))
        prev = dc.cpu.pc;
    }
    try std.testing.expect(dc.cpu.R(14).* == 0);
}

test "Binaries" {
    try load_and_test_binary("0.bin");
    try load_and_test_binary("stack_0.bin");
}
