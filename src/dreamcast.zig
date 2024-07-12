const std = @import("std");

const common = @import("common.zig");
const addr_t = common.addr_t;
const termcolor = @import("termcolor");

const HardwareRegisters = @import("hardware_registers.zig");
const HardwareRegister = HardwareRegisters.HardwareRegister;

const Interrupts = @import("sh4_interrupts.zig");
const Interrupt = Interrupts.Interrupt;

pub const SH4Module = @import("sh4.zig");
const SH4 = SH4Module.SH4;
const SH4JIT = @import("jit/sh4_jit.zig").SH4JIT;
const Flash = @import("flash.zig").Flash;
pub const HollyModule = @import("holly.zig");
const Holly = HollyModule.Holly;
pub const AICAModule = @import("aica.zig");
const AICA = AICAModule.AICA;
pub const Maple = @import("maple.zig");
const MapleHost = Maple.MapleHost;
const GDROM = @import("gdrom.zig").GDROM;

const dc_log = std.log.scoped(.dc);

const Region = enum(u8) {
    Japan = 0,
    USA = 1,
    Europe = 2,
    Unknown = 3,
};

const Language = enum(u8) {
    Japanese = 0,
    English = 1,
    German = 2,
    French = 3,
    Spanish = 4,
    Italian = 5,
    Unknown = 6,
};

const VideoMode = enum(u8) {
    NTSC = 0,
    PAL = 1,
    PAL_M = 2,
    PAL_N = 3,
    Unknown = 4,
};

const CableType = enum(u16) {
    VGA = 0,
    RGB = 2,
    Composite = 3,
};

const Callback = struct {
    function: *const fn (*anyopaque, *Dreamcast) void,
    context: *anyopaque,
};

const ScheduledInterrupt = struct {
    const InterruptType = enum { Normal, External };

    trigger_cycle: u64,
    interrupt: ?union(InterruptType) {
        Normal: HardwareRegisters.SB_ISTNRM,
        External: HardwareRegisters.SB_ISTEXT,
    },
    callback: ?Callback,

    fn compare(_: void, a: ScheduledInterrupt, b: ScheduledInterrupt) std.math.Order {
        return std.math.order(a.trigger_cycle, b.trigger_cycle);
    }
};

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

const user_data_directory = "./userdata/";
const user_flash_path = user_data_directory ++ "flash.bin";

pub const Dreamcast = struct {
    const threaded_aica: bool = false; // FIXME: Ecco crashes when enabled.

    cpu: SH4,
    gpu: Holly,
    aica: AICA,
    maple: MapleHost,
    gdrom: GDROM,

    sh4_jit: SH4JIT,

    // Pluged in video cable reported to the CPU:e.
    cable_type: CableType = .VGA,

    boot: []u8 align(4) = undefined,
    flash: Flash,
    ram: []u8 align(4) = undefined,
    hardware_registers: []u8 align(4) = undefined, // FIXME

    scheduled_interrupts: std.PriorityQueue(ScheduledInterrupt, void, ScheduledInterrupt.compare),
    _scheduled_interrupts_cycles: u64 = 0, // FIXME: Handle the case where _scheduled_interrupts_cycles it might overflow soon?... Is it even realistic?

    _allocator: std.mem.Allocator = undefined,

    _dummy: [4]u8 align(32) = undefined, // FIXME: Dummy space for non-implemented features

    _stopping: bool = false,
    _aica_thread: std.Thread = undefined,
    _aica_condition: std.Thread.Condition = undefined,
    _aica_mutex: std.Thread.Mutex = undefined,
    _aica_pending_cycles: std.atomic.Value(u32) = .{ .raw = 0 },

    pub fn create(allocator: std.mem.Allocator) !*Dreamcast {
        const dc = try allocator.create(Dreamcast);
        dc.* = Dreamcast{
            .cpu = try SH4.init(allocator, dc),
            .gpu = try Holly.init(allocator, dc),
            .aica = try AICA.init(allocator),
            .maple = try MapleHost.init(allocator),
            .gdrom = GDROM.init(allocator),
            .sh4_jit = try SH4JIT.init(allocator),
            .flash = try Flash.init(allocator),
            .ram = try allocator.alloc(u8, 0x0100_0000),
            .hardware_registers = try allocator.alloc(u8, 0x20_0000), // FIXME: Huge waste of memory.
            .scheduled_interrupts = std.PriorityQueue(ScheduledInterrupt, void, ScheduledInterrupt.compare).init(allocator, {}),
            ._allocator = allocator,
        };

        dc.*.aica.setup_arm();
        if (threaded_aica) {
            dc._aica_condition = .{};
            dc._aica_mutex = .{};
            dc._aica_thread = try std.Thread.spawn(.{}, aica_thread, .{dc});
        }

        // Load ROM
        dc.boot = try dc._allocator.alloc(u8, 0x200000);
        const boot_path = "./bin/dc_boot.bin";
        var boot_file = std.fs.cwd().openFile(boot_path, .{}) catch |err| {
            dc_log.err(termcolor.red("Failed to open boot ROM at '" ++ boot_path ++ "', error: {any}."), .{err});
            return err;
        };
        defer boot_file.close();
        const bytes_read = try boot_file.readAll(dc.boot);
        std.debug.assert(bytes_read == 0x200000);

        // FIXME: Hack to bypass some checks I'm failling to emulate.
        //        Pretty sure this is linked to the GD ROM, bios is waiting on something here.
        //        Insert a nop instead of the branch.
        // dc.boot[0x077B] = 0x00;
        // dc.boot[0x077A] = 0x09;

        // Create 'userdata' folder if it doesn't exist
        try std.fs.cwd().makePath(user_data_directory);

        // Load Flash
        var flash_file = std.fs.cwd().openFile(user_flash_path, .{}) catch |err| f: {
            if (err == error.FileNotFound) {
                dc_log.info("Loading default flash ROM.", .{});
                const default_flash_path = "./bin/dc_flash.bin";
                break :f std.fs.cwd().openFile(default_flash_path, .{}) catch |e| {
                    dc_log.err(termcolor.red("Failed to open default flash file at '" ++ default_flash_path ++ "', error: {any}."), .{e});
                    return e;
                };
            } else {
                dc_log.err(termcolor.red("Failed to open user flash file at '" ++ user_flash_path ++ "', error: {any}."), .{err});
                return err;
            }
        };

        defer flash_file.close();
        const flash_bytes_read = try flash_file.readAll(dc.flash.data);
        std.debug.assert(flash_bytes_read == 0x20000);

        try dc.reset();

        return dc;
    }

    pub fn deinit(self: *@This()) void {
        if (threaded_aica) {
            self._stopping = true;
            self._aica_condition.signal();
            self._aica_thread.join();
        }

        // Write flash to disk
        if (!@import("builtin").is_test) {
            if (std.fs.cwd().createFile(user_flash_path, .{})) |file| {
                defer file.close();
                _ = file.writeAll(self.flash.data) catch |err| {
                    dc_log.err("Failed to save user flash: {any}", .{err});
                };
            } else |err| {
                dc_log.err("Failed to open user flash '{s}' for writing: {any}", .{ user_flash_path, err });
            }
        }

        self.scheduled_interrupts.deinit();
        self.sh4_jit.deinit();
        self.gdrom.deinit();
        self.maple.deinit();
        self.aica.deinit();
        self.gpu.deinit();
        self.cpu.deinit();
        self.flash.deinit();
        self._allocator.free(self.boot);
        self._allocator.free(self.hardware_registers);
        self._allocator.free(self.ram);
    }

    pub fn reset(self: *@This()) !void {
        self.cpu.reset();
        self.gpu.reset();

        try self.sh4_jit.block_cache.reset();

        @memset(self.ram[0..], 0x00); // NOTE: Sonic Adventure 2 reads some unitialized memory around 0x0C000050...

        self.hw_register(u32, .SB_FFST).* = 0; // FIFO Status
        self.hw_register(u32, .SB_MDST).* = 0;
        self.hw_register(u32, .SB_DDST).* = 0;
        self.hw_register(u32, .SB_ISTNRM).* = 0;

        // Holly Version. TODO: Make it configurable?
        self.hw_register(u32, .SB_SBREV).* = 0x0B;
        self.hw_register(u32, .SB_G2ID).* = 0x12; // Only possible value, apparently.

    }

    pub fn load_at(self: *@This(), addr: addr_t, bin: []const u8) void {
        const start_addr = ((addr & 0x1FFFFFFF) - 0x0C000000);
        @memcpy(self.ram[start_addr .. start_addr + bin.len], bin);
    }

    pub fn skip_bios(self: *@This(), hle_syscalls: bool) void {
        self.cpu.state_after_boot_rom();

        @memset(self.ram[0x00200000..0x00300000], 0x00); // FIXME: I think KallistiOS relies on that, or maybe I messed up somewhere else. (the BootROM does clear this section of RAM)

        // Copy subroutine to RAM. Some of it will be overwritten, I'm trying to work out what's important and what's not.
        inline for (0..16) |i| {
            self.cpu.write16(0x8C0000E0 + 2 * i, self.cpu.read16(0x800000FE - 2 * i));
        }
        // Copy a portion of the boot ROM to RAM.
        self.cpu.write32(0xA05F74E4, 0x001FFFFF);

        @memcpy(self.ram[0x00000100..0x00004000], self.boot[0x00000100..0x00004000]);
        @memcpy(self.ram[0x00008000..0x00200000], self.boot[0x00008000..0x00200000]);

        const IP_bin_HLE = false;
        if (IP_bin_HLE) {
            // Copy a portion of the flash ROM to RAM.
            inline for (0..8) |i| {
                self.cpu.write8(0x8C000068 + i, self.cpu.read8(0x0021A056 + i));
            }
            inline for (0..5) |i| {
                self.cpu.write8(0x8C000068 + 8 + i, self.cpu.read8(0x0021A000 + i));
            }
            // FIXME: Load system settings from flashrom (User partition (2), logical block 5), instead of these hardcoded values.
            //inline for (.{ 0xBC, 0xEA, 0x90, 0x5E, 0xFF, 0x04, 0x00, 0x01 }, 0..) |val, i| {
            inline for (.{ 0x00, 0x00, 0x89, 0xFC, 0x5B, 0xFF, 0x01, 0x00, 0x00, 0x7D, 0x0A, 0x62, 0x61 }, 0..) |val, i| {
                self.cpu.write8(0x8C000068 + 13 + i, val);
            }
        }

        // Patch some adresses of functions provided by the boot ROM ("syscalls")
        inline for (.{
            .{ 0x8C0000B0, 0x8C003C00 },
            .{ 0x8C0000B4, 0x8C003D80 },
            .{ 0x8C0000B8, 0x8C003D00 },
            .{ 0x8C0000BC, 0x8C001000 },
            .{ 0x8C0000C0, 0x8C0010F0 },
            .{ 0x8C0000E0, 0x8C000800 },
        }) |p| {
            self.cpu.write32(p[0], p[1]);
        }
        // Replace them by HLE counterparts (see syscall.zig) by inserting fake opcodes.
        if (hle_syscalls) {
            // System
            self.cpu.write16(0x8C003C00, 0b0000000000010000);
            // Font
            self.cpu.write16(0x8C003D80, 0b0000000000100000);
            // Flashrom
            self.cpu.write16(0x8C003D00, 0b0000000000110000);
            // GD
            self.cpu.write16(0x8C001000, 0b0000000001000000);
            // GD2
            self.cpu.write16(0x8C0010F0, 0b0000000001010000);
            // Misc
            self.cpu.write16(0x8C000800, 0b0000000001100000);
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
            self.cpu.write32(p[0], p[1]);
        }

        // Load IP.bin from disk (16 first sectors of the last track)
        // FIXME: Here we assume the last track is the 3rd.
        if (self.gdrom.disk) |disk|
            _ = disk.load_bytes(45150, 16 * 2048, self.ram[0x00008000..]);

        // IP.bin patches
        inline for (.{
            .{ 0xAC0090D8, 0x5113 },
            .{ 0xAC00940A, 0x000B },
            .{ 0xAC00940C, 0x0009 },
        }) |p| {
            self.cpu.write16(p[0], p[1]);
        }

        // Patch some functions apparently used by interrupts
        // (And some other random stuff that the boot ROM sets for some reason
        //  and I'm afraid some games might use. I'm not taking any more chances)

        // Sleep on error?
        self.cpu.write32(0x8C000000, 0x00090009);
        self.cpu.write32(0x8C000004, 0x001B0009);
        self.cpu.write32(0x8C000008, 0x0009AFFD);
        // ??
        self.cpu.write16(0x8C00000C, 0);
        self.cpu.write16(0x8C00000E, 0);
        // RTE - Some interrupts jump there instead of having their own RTE, I have NO idea why.
        self.cpu.write32(0x8C000010, 0x00090009); // nop nop
        self.cpu.write32(0x8C000014, 0x0009002B); // rte nop
        // RTS
        self.cpu.write32(0x8C000018, 0x00090009);
        self.cpu.write32(0x8C00001C, 0x0009000B);

        // ??
        self.cpu.write8(0x8C00002C, 0x16);
        self.cpu.write32(0x8C000064, 0x8c008100);
        self.cpu.write16(0x8C000090, 0);
        self.cpu.write16(0x8C000092, @bitCast(@as(i16, -128)));

        self.hw_register(u32, .SB_MDST).* = 0;
        self.hw_register(u32, .SB_DDST).* = 0;
    }

    pub fn set_flash_settings(self: *@This(), region: Region, lang: Language, video_mode: VideoMode) void {
        self.flash[0x1A002] = @as(u8, '0') + @intFromEnum(region);
        self.flash[0x1A0A2] = @as(u8, '0') + @intFromEnum(region);
        self.flash[0x1A003] = @as(u8, '0') + @intFromEnum(lang);
        self.flash[0x1A0A3] = @as(u8, '0') + @intFromEnum(lang);
        self.flash[0x1A004] = @as(u8, '0') + @intFromEnum(video_mode);
        self.flash[0x1A0A4] = @as(u8, '0') + @intFromEnum(video_mode);
    }

    pub inline fn read_hw_register(self: *const @This(), comptime T: type, r: HardwareRegister) T {
        return @constCast(self).hw_register_addr(T, @intFromEnum(r)).*;
    }
    pub inline fn hw_register(self: *@This(), comptime T: type, r: HardwareRegister) *T {
        return self.hw_register_addr(T, @intFromEnum(r));
    }
    pub inline fn hw_register_addr(self: *@This(), comptime T: type, addr: addr_t) *T {
        std.debug.assert(addr >= 0x005F6800 and addr < 0x005F6800 + self.hardware_registers.len);
        return @as(*T, @alignCast(@ptrCast(&self.hardware_registers[addr - 0x005F6800])));
    }

    pub fn tick(self: *@This(), max_instructions: u8) !u32 {
        const cycles = self.cpu.execute(max_instructions);
        return self.tick_peripherals(cycles);
    }

    pub fn tick_jit(self: *@This()) !u32 {
        var cycles: u32 = 0;
        while (cycles < 64)
            cycles += try self.sh4_jit.execute(&self.cpu);
        return self.tick_peripherals(cycles);
    }

    fn tick_peripherals(self: *@This(), cycles: u32) !u32 {
        try self.tick_aica(cycles);
        self.advance_scheduled_interrupts(cycles);
        self.gdrom.update(self, cycles);
        self.gpu.update(self, cycles);
        return cycles;
    }

    fn tick_aica(self: *@This(), cycles: u32) !void {
        if (threaded_aica) {
            if (self._aica_pending_cycles.fetchAdd(cycles, std.builtin.AtomicOrder.seq_cst) == 0)
                self._aica_condition.signal(); // If there was no pending cycles, make sure to wake up the thread.
        } else {
            try self.aica.update(self, cycles);
        }
    }

    fn aica_thread(self: *@This()) void {
        dc_log.info(termcolor.green("AICA thread started"), .{});
        while (!self._stopping) {
            const cycles = self._aica_pending_cycles.load(std.builtin.AtomicOrder.seq_cst);
            if (cycles > 0) {
                self.aica.update(self, cycles) catch |err| {
                    dc_log.err("Failed to update AICA: {}", .{err});
                    return;
                };
                _ = self._aica_pending_cycles.fetchSub(cycles, std.builtin.AtomicOrder.seq_cst);
            } else {
                self._aica_mutex.lock();
                defer self._aica_mutex.unlock();
                self._aica_condition.wait(&self._aica_mutex);
            }
        }
    }

    // TODO: Add helpers for external interrupts and errors.

    pub fn schedule_event(self: *@This(), callback: Callback, cycles: u32) void {
        self.scheduled_interrupts.add(.{
            .trigger_cycle = self._scheduled_interrupts_cycles +% cycles,
            .callback = callback,
            .interrupt = null,
        }) catch |err| {
            std.debug.panic("Failed to schedule event: {}", .{err});
        };
    }

    pub fn schedule_interrupt(self: *@This(), int: HardwareRegisters.SB_ISTNRM, cycles: u32) void {
        self.scheduled_interrupts.add(.{
            .trigger_cycle = self._scheduled_interrupts_cycles +% cycles,
            .interrupt = .{ .Normal = int },
            .callback = null,
        }) catch |err| {
            std.debug.panic("Failed to schedule interrupt: {}", .{err});
        };
    }

    pub fn schedule_external_interrupt(self: *@This(), int: HardwareRegisters.SB_ISTEXT, cycles: u32) void {
        self.scheduled_interrupts.add(.{
            .trigger_cycle = self._scheduled_interrupts_cycles +% cycles,
            .interrupt = .{ .External = int },
            .callback = null,
        }) catch |err| {
            std.debug.panic("Failed to schedule external interrupt: {}", .{err});
        };
    }

    fn advance_scheduled_interrupts(self: *@This(), cycles: u32) void {
        if (self.scheduled_interrupts.count() > 0) {
            self._scheduled_interrupts_cycles += cycles;
            while (self.scheduled_interrupts.peek()) |event| {
                if (event.trigger_cycle <= self._scheduled_interrupts_cycles) {
                    if (event.interrupt) |int| {
                        switch (int) {
                            .Normal => self.raise_normal_interrupt(int.Normal),
                            .External => self.raise_external_interrupt(int.External),
                        }
                    }
                    if (event.callback) |callback| {
                        callback.function(callback.context, self);
                    }
                    _ = self.scheduled_interrupts.remove();
                } else break;
            }
        }
    }

    pub fn raise_normal_interrupt(self: *@This(), int: HardwareRegisters.SB_ISTNRM) void {
        self.hw_register(u32, .SB_ISTNRM).* |= @bitCast(int);

        self.check_sb_interrupts();
    }

    pub fn raise_external_interrupt(self: *@This(), int: HardwareRegisters.SB_ISTEXT) void {
        self.hw_register(u32, .SB_ISTEXT).* |= @bitCast(int);
        self.hw_register(u32, .SB_ISTNRM).* |= @bitCast(HardwareRegisters.SB_ISTNRM{ .ExtStatus = if (self.hw_register(u32, .SB_ISTEXT).* != 0) 1 else 0 });

        self.check_sb_interrupts();
    }

    fn check_sb_interrupts(self: *@This()) void {
        const istnrm = self.read_hw_register(u32, .SB_ISTNRM);
        const istext = self.read_hw_register(u32, .SB_ISTEXT);
        const isterr = self.read_hw_register(u32, .SB_ISTERR);
        if ((istnrm & self.read_hw_register(u32, .SB_IML6NRM)) != 0 or (istext & self.read_hw_register(u32, .SB_IML6EXT)) != 0 or (isterr & self.read_hw_register(u32, .SB_IML6ERR)) != 0) {
            self.cpu.request_interrupt(Interrupts.Interrupt.IRL9);
        } else if ((istnrm & self.read_hw_register(u32, .SB_IML4NRM)) != 0 or (istext & self.read_hw_register(u32, .SB_IML4EXT)) != 0 or (isterr & self.read_hw_register(u32, .SB_IML4ERR)) != 0) {
            self.cpu.request_interrupt(Interrupts.Interrupt.IRL11);
        } else if ((istnrm & self.read_hw_register(u32, .SB_IML2NRM)) != 0 or (istext & self.read_hw_register(u32, .SB_IML2EXT)) != 0 or (isterr & self.read_hw_register(u32, .SB_IML2ERR)) != 0) {
            self.cpu.request_interrupt(Interrupts.Interrupt.IRL13);
        }
    }

    pub fn start_maple_dma(self: *@This()) void {
        if (self.hw_register(u32, .SB_MDEN).* == 1) {
            // Note: This is useless since DMA in currently instantaneous, but just in case I need it later...
            self.hw_register(u32, .SB_MDST).* = 1;
            defer self.hw_register(u32, .SB_MDST).* = 0;

            dc_log.debug("Maple-DMA initiation!", .{});
            const sb_mdstar = self.read_hw_register(u32, .SB_MDSTAR);
            std.debug.assert(sb_mdstar >> 28 == 0 and sb_mdstar & 0x1F == 0);
            self.maple.transfer(self, @as([*]u32, @alignCast(@ptrCast(&self.ram[sb_mdstar - 0x0C000000])))[0..]);
        }
    }

    pub fn start_gd_dma(self: *@This()) void {
        if (self.hw_register(u32, .SB_GDEN).* == 1) {
            const dst_addr = self.read_hw_register(u32, .SB_GDSTAR) & 0x1FFFFFE0;
            const len = self.read_hw_register(u32, .SB_GDLEN);
            const direction = self.read_hw_register(u32, .SB_GDDIR);

            if (direction == 0) {
                dc_log.err(termcolor.red("DMA to GD-ROM not implemented."), .{});
                return;
            }

            self.hw_register(u32, .SB_GDST).* = 1;
            self.hw_register(u32, .SB_GDLEND).* = 0;
            self.hw_register(u32, .SB_GDSTARD).* = dst_addr;

            dc_log.info("GD-ROM-DMA! {X:0>8} ({X:0>8} bytes / {X:0>8} in queue)", .{ dst_addr, len, self.gdrom.data_queue.count });

            // NOTE: This should use ch0-DMA, but the SH4 DMAC implementation can't handle this case (yet?).
            //       Unless we copy u16 by u16 from the data register, but, mmh, yeah.
            const copied = self.gdrom.data_queue.read(@as([*]u8, @ptrCast(self.cpu._get_memory(dst_addr)))[0..len]);

            // Simulate using ch0
            self.cpu.p4_register(SH4Module.P4.CHCR, .CHCR0).*.sm = 0;
            self.cpu.p4_register(SH4Module.P4.CHCR, .CHCR0).*.dm = 1;
            self.cpu.p4_register(u32, .DAR0).* = dst_addr;
            self.cpu.end_dmac(0);

            dc_log.debug("First 0x20 bytes copied: {X}", .{@as([*]u8, @ptrCast(self.cpu._get_memory(dst_addr)))[0..0x20]});

            std.debug.assert(copied == len);

            // TODO: Delay?
            self.hw_register(u32, .SB_GDST).* = 0;
            self.hw_register(u32, .SB_GDLEND).* += len;
            self.hw_register(u32, .SB_GDSTARD).* += len;

            self.gdrom.status_register.drq = 0;
            self.gdrom.status_register.bsy = 0;
            self.gdrom.interrupt_reason_register.cod = .Command;
            self.gdrom.interrupt_reason_register.io = .DeviceToHost;

            self.schedule_interrupt(.{ .EoD_GDROM = 1 }, 20000);
            // self.schedule_external_interrupt(.{ .GDRom = 1 }, 20000);
        }
    }

    pub fn start_ch2_dma(self: *@This()) void {
        self.hw_register(u32, .SB_C2DST).* = 1;

        const dst_addr = self.read_hw_register(u32, .SB_C2DSTAT);
        const len = self.read_hw_register(u32, .SB_C2DLEN);

        dc_log.debug("  Start ch2-DMA: {X:0>8} -> {X:0>8} ({X:0>8} bytes)", .{ self.cpu.read_p4_register(u32, .SAR2), dst_addr, len });

        std.debug.assert(dst_addr & 0xF8000000 == 0x10000000);
        self.cpu.p4_register(u32, .DAR2).* = dst_addr; // FIXME: Not sure this is correct

        const dmac_len = self.cpu.read_p4_register(u32, .DMATCR2);
        std.debug.assert(32 * dmac_len == len);

        self.cpu.start_dmac(2);

        // TODO: Schedule for later?

        // In the case of Direct Texture Path DMA, the destination address is incremented with the transfer, otherwise it is maintained.
        if (dst_addr >= 0x11000000 and dst_addr <= 0x11FFFFE0 or dst_addr >= 0x13000000 and dst_addr <= 0x13FFFFE0) {
            self.hw_register(u32, .SB_C2DSTAT).* += len;
        }
        self.hw_register(u32, .SB_C2DLEN).* = 0;
        self.hw_register(u32, .SB_C2DST).* = 0;

        self.schedule_interrupt(.{ .EoD_CH2 = 1 }, 200);
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

    _ = try dc.tick(1); // mov 0x0F,R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFFFFFFFF);
    _ = try dc.tick(1); // shll16 R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFFFF0000);
    _ = try dc.tick(1); // swap.w R4,R3
    try std.testing.expect(dc.cpu.R(4).* == 0x0000FFFF);
    _ = try dc.tick(1); // shll8 R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFF000000);
    _ = try dc.tick(1); // shlr2 R4
    try std.testing.expect(dc.cpu.R(4).* == 0x00003FFF);
    _ = try dc.tick(1); // shlr2 R4
    try std.testing.expect(dc.cpu.R(4).* == 0x00000FFF);
    // Reads EXPEVT (0xFF000024), 0x00000000 on power up, 0x00000020 on reset.
    _ = try dc.tick(1); // mov.l @(9, R3),R0
    try std.testing.expect(dc.cpu.read32(dc.cpu.R(3).* + (9 << 2)) == dc.cpu.R(0).*);
    _ = try dc.tick(1); // xor R4,R0
    try std.testing.expect(dc.cpu.R(4).* == 0x00000FFF);
    try std.testing.expect(dc.cpu.R(4).* == 0x00000FFF);
    {
        const prev_mach = dc.cpu.mach;
        _ = try dc.tick(1); // mulu.w R4,R0
        try std.testing.expect(dc.cpu.mach == prev_mach);
        try std.testing.expect(dc.cpu.macl == 0);
    }
    _ = try dc.tick(1); // sts R0,MACL
    try std.testing.expect(dc.cpu.R(0).* == 0);
    _ = try dc.tick(1); // tst R0,R0
    try std.testing.expect(dc.cpu.sr.t);
    _ = try dc.tick(1); // bf 0x8C010108
    try std.testing.expect(dc.cpu.pc == 0xA0000018);
    _ = try dc.tick(1); // mov.l R0,@(4,R3) - Write 0x0 to MMUCR @ 0xFF000010
    try std.testing.expect(dc.cpu.R(0).* == dc.cpu.read32(0xFF000000 + (4 << 2)));
    _ = try dc.tick(1); // mov 0x9,R1
    try std.testing.expect(dc.cpu.R(1).* == 0x9);
    _ = try dc.tick(1); // shll8 R1
    try std.testing.expect(dc.cpu.R(1).* == 0x9 << 8);
    _ = try dc.tick(1); // add 0x29,R1
    try std.testing.expect(dc.cpu.R(1).* == (0x9 << 8) + 0x29);
    _ = try dc.tick(1); // mov.l R1, @(7, R3) - Write 0x929 to CCR @ 0xFF00001C
    try std.testing.expect(dc.cpu.R(1).* == dc.cpu.read32(0xFF000000 + (0x7 << 2)));
    _ = try dc.tick(1); // shar R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFF800000);
    try std.testing.expect(!dc.cpu.sr.t);
    _ = try dc.tick(1); // mov 0x01, R0
    try std.testing.expect(dc.cpu.R(0).* == 0x01);
    _ = try dc.tick(1); // mov.w R0, @(2, R3) - Write 0x01 to BCR2 @ 0xFF800004
    try std.testing.expect(dc.cpu.R(0).* == dc.cpu.read16(0xFF800000 + (0x2 << 1)));
    _ = try dc.tick(1); // mov 0xFFFFFFC3, R0
    try std.testing.expect(dc.cpu.R(0).* == 0xFFFFFFC3);
    _ = try dc.tick(1); // shll16 R0
    try std.testing.expect(dc.cpu.R(0).* == 0xFFC30000);
    _ = try dc.tick(1); // or 0xCD, R0
    try std.testing.expect(dc.cpu.R(0).* == 0xFFC300CD);
    _ = try dc.tick(1); // shll8 R0
    try std.testing.expect(dc.cpu.R(0).* == 0xC300CD00);
    _ = try dc.tick(1); // or 0xB0, R0
    try std.testing.expect(dc.cpu.R(0).* == 0xC300CDB0);
    _ = try dc.tick(1); // shlr R0
    try std.testing.expect(dc.cpu.R(0).* == 0xC300CDB0 >> 1);
    try std.testing.expect(!dc.cpu.sr.t);
    _ = try dc.tick(1); // mov.l R0, @(3, R3) - Write 0x01 to WCR2 @ 0xFF80000C
    try std.testing.expect(dc.cpu.R(0).* == dc.cpu.read32(0xFF800000 + (0x3 << 2)));
    _ = try dc.tick(1); // mov 0x01, R5
    try std.testing.expect(dc.cpu.R(5).* == 0x01);
    _ = try dc.tick(1); // rotr R5
    try std.testing.expect(dc.cpu.R(5).* == 0x80000000);
    try std.testing.expect(dc.cpu.sr.t);
    _ = try dc.tick(1); // add 0x60, R5
    try std.testing.expect(dc.cpu.R(5).* == 0x80000060);
    _ = try dc.tick(1); // mov R5, R6
    try std.testing.expect(dc.cpu.R(5).* == dc.cpu.R(6).*);
    _ = try dc.tick(1); // add 0x20, R6
    try std.testing.expect(dc.cpu.R(6).* == 0x80000080);
    _ = try dc.tick(1); // tst 0x00, R0 - Always tue, right?
    try std.testing.expect(dc.cpu.sr.t);
    _ = try dc.tick(1); // pref @R5
    // TODO
    _ = try dc.tick(1); // jmp @R6
    try std.testing.expect(dc.cpu.pc == dc.cpu.R(6).*);

    _ = try dc.tick(1); // mov.l @(0x2,R5),R0 - Read 0x80000068 (0xA3020008) to R0
    try std.testing.expect(0xA3020008 == dc.cpu.R(0).*);
    try std.testing.expect(dc.cpu.read32(dc.cpu.R(5).* + (0x2 << 2)) == dc.cpu.R(0).*);

    _ = try dc.tick(1); // mov.l R0, @(0, R3) - Write 0xA3020008 to BRC1 @ 0xFF800000
    try std.testing.expect(dc.cpu.read32(0xFF800000) == 0xA3020008);
    _ = try dc.tick(1); // mov.l @(4,R5),R0
    try std.testing.expect(dc.cpu.read32(dc.cpu.R(5).* + (0x4 << 2)) == dc.cpu.R(0).*);
    _ = try dc.tick(1); // mov.l R0, @(2, R3) - Write 0x01110111 to WCR1 @ 0xFF800008
    try std.testing.expect(dc.cpu.read32(0xFF800008) == 0x01110111);
    _ = try dc.tick(1); // add 0x10, R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFF800010);
    _ = try dc.tick(1); // mov.l @(5, R5), R0 - Read 0x80000078 (0x800A0E24) to R0
    try std.testing.expect(dc.cpu.R(0).* == 0x800A0E24);
    _ = try dc.tick(1); // mov.l R0, @(1, R3) - Write 0x800A0E24 to MCR
    try std.testing.expect(dc.cpu.p4_register(u32, .MCR).* == 0x800A0E24);

    _ = try dc.tick(1); // mov.l @(7, R5), R2
    try std.testing.expect(dc.cpu.R(2).* == 0xff940190);
    _ = try dc.tick(1); // mov.b R2, @R2
    // NOTE: SDMR is ignored, it should not matter (see note about P4 access optimisation).
    //try std.testing.expect(dc.cpu.p4_register(u8, .SDMR).* == 0x90);

    _ = try dc.tick(1); // mov 0xFFFFFFA4, R0
    _ = try dc.tick(1); // shll8 R0
    _ = try dc.tick(1); // mov.w R0, @(12, R3)
    // NOTE: This writes 0xA400, but the 6 top bits are a code to avoid accidental overwrites, they're discarded.
    try std.testing.expect(dc.cpu.p4_register(u16, .RFCR).* == 0x0000);

    _ = try dc.tick(1); // mov.w @(0, R5), R0
    _ = try dc.tick(1); // mov.w R0, @(10, R3)
    try std.testing.expect(dc.cpu.p4_register(u16, .RTCOR).* == 0x0004);

    _ = try dc.tick(1); // add H'0c, R0
    _ = try dc.tick(1); // mov.w R0, @(6, R3)
    try std.testing.expect(dc.cpu.p4_register(u16, .RTCSR).* == 0x0010);

    // while((volatile uint16_t)reg[RFCR] <= 0x0010);

    _ = try dc.tick(1); // mov 0x10, R6
    _ = try dc.tick(1); // mov.w @(12, R3), R0 - Load RFCR (Refresh Count Register) to R0
    try std.testing.expect(dc.cpu.R(0).* == 0x11); // Note: Refresh Count Register not implemented, this check passes because we always return 0x11.
    _ = try dc.tick(1); // cmp/hi R6, R0
    _ = try dc.tick(1); // bf 0x8C0100A2

    _ = try dc.tick(1); // mov.w @(1, R5), R0
    _ = try dc.tick(1); // mov.w R0, @(10, R3)
    try std.testing.expect(dc.cpu.p4_register(u16, .RTCOR).* == 0x005e);

    _ = try dc.tick(1); // mov.l @(6, R5), R0
    _ = try dc.tick(1); // mov.l R0, @(1, R3)
    try std.testing.expect(dc.cpu.p4_register(u32, .MCR).* == 0xc00a0e24);

    _ = try dc.tick(1); // mov.b R2, @R2
    _ = try dc.tick(1); // mov.l @(1, R5), R1
    // NOTE: See notes on P4 access optimisation.
    // try std.testing.expect(dc.cpu.p4_register(u8, .SDMR).* == 0x90);

    _ = try dc.tick(1); // mov 0x04, R0
    try std.testing.expect(dc.cpu.R(0).* == 0x04);
    _ = try dc.tick(1); // swap.b R0, R0
    try std.testing.expect(dc.cpu.R(0).* == 0x0400);
    _ = try dc.tick(1); // mov.w R0, @R1
    try std.testing.expect(dc.cpu.read16(0xA05F7480) == 0x400); // SB_G1RRC

    _ = try dc.tick(1); // mov.l @(3, R5), R3
    _ = try dc.tick(1); // mova 0x8C0100E0, R0

    for (0..16) |_| {
        try std.testing.expect(dc.cpu.pc == 0x800000BE);
        _ = try dc.tick(1); // dt R6
        _ = try dc.tick(1); // mov.w @R0+, R1
        _ = try dc.tick(1); // mov.w R1, @-R3
        _ = try dc.tick(1); // bf 0x8C0100BE
    }

    _ = try dc.tick(1); // mov.l @R3, R1
    _ = try dc.tick(1); // jmp @R3
    try std.testing.expect(dc.cpu.pc == 0x8C0000E0);

    _ = try dc.tick(1); //

    _ = try dc.tick(1); //
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
        _ = try dc.tick(1);
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
        _ = try dc.tick(1);
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
        _ = try dc.tick(1);
        try std.testing.expect(dc.cpu.pc != prev); // Crude check for infinite loops, there might be legitiple reason to do this (loops with process in delay slot?), but we'll just choose not to and be fine :))
        prev = dc.cpu.pc;
    }
    try std.testing.expect(dc.cpu.R(14).* == 0);
}

test "Binaries" {
    try load_and_test_binary("0.bin");
    try load_and_test_binary("stack_0.bin");
}
