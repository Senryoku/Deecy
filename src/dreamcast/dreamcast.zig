const std = @import("std");
const builtin = @import("builtin");
const dc_config = @import("dc_config");

const termcolor = @import("termcolor");

const dc_log = std.log.scoped(.dc);
pub const HostPaths = @import("host_paths.zig");

pub const HardwareRegisters = @import("hardware_registers.zig");
pub const SH4Module = @import("sh4.zig");
pub const SH4Interpreter = @import("sh4_interpreter.zig");
pub const SH4JITModule = @import("jit/sh4_jit.zig");
pub const HollyModule = @import("holly.zig");
pub const AICAModule = @import("aica.zig");
pub const Maple = @import("maple.zig");
pub const GDROM = @import("gdrom.zig");
pub const GDROM_HLE = @import("gdrom_hle.zig");

const HardwareRegister = HardwareRegisters.HardwareRegister;
const SH4 = SH4Module.SH4;
const SH4JIT = SH4JITModule.SH4JIT;
const Holly = HollyModule.Holly;
const AICA = AICAModule.AICA;
const MapleHost = Maple.MapleHost;
const Flash = @import("flash.zig");

pub const Region = enum(u8) {
    Japan = 0,
    USA = 1,
    Europe = 2,
    Unknown = 3,
};

pub const Language = enum(u8) {
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
    function: ?*const fn (*anyopaque, *Dreamcast) void,
    context: *anyopaque,

    pub fn call(self: Callback, dc: *Dreamcast) void {
        if (self.function != null)
            self.function.?(self.context, dc);
    }
};

const ScheduledEvent = struct {
    const Event = union(enum) {
        None,
        EndGDDMA,
        EndSortDMA,
        EndAICADMA,
        TimerUnderflow: struct { channel: u2 },
        HBlankIn,
        VBlankIn,
        VBlankOut,

        pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
            switch (self) {
                .TimerUnderflow => |e| return writer.print("Timer {d} Underflow", .{e.channel}),
                else => return writer.print("{s}", .{@tagName(self)}),
            }
        }
    };
    trigger_cycle: u64,
    interrupt: ?union(enum) {
        Normal: HardwareRegisters.SB_ISTNRM,
        External: HardwareRegisters.SB_ISTEXT,

        pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
            switch (self) {
                .Normal => |e| return writer.print("{any}", .{e}),
                .External => |e| return writer.print("{any}", .{e}),
            }
        }
    },
    event: Event = .None,

    fn compare(_: void, a: ScheduledEvent, b: ScheduledEvent) std.math.Order {
        return std.math.order(a.trigger_cycle, b.trigger_cycle);
    }
};

inline fn check_type(comptime Valid: []const type, comptime T: type, comptime fmt: []const u8, param: anytype) void {
    inline for (Valid) |V| if (T == V) return;
    std.debug.print(fmt, param);
    unreachable;
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

pub const Dreamcast = struct {
    pub const BootSize = 0x20_0000;
    pub const RAMSize = 0x100_0000;
    pub const VRAMSize = Holly.VRAMSize;
    pub const ARAMSize = AICA.RAMSize;
    pub const SH4Clock = 200_000_000;

    cpu: SH4,
    gpu: Holly = undefined,
    aica: AICA = undefined,
    maple: MapleHost,
    gdrom: GDROM = undefined,
    gdrom_hle: GDROM_HLE = .{}, // NOTE: Currently not serialized in save states. It is now less compatible than the LLE implementation.

    sh4_jit: SH4JIT = undefined,

    cable_type: CableType = .VGA, // Plugged in video cable reported to the CPU.
    region: Region = .Unknown,

    boot: []align(64) u8 = undefined,
    flash: Flash,
    ram: []align(64) u8 = undefined,
    vram: []align(64) u8 = undefined,
    aram: []align(64) u8 = undefined,
    hardware_registers: []align(4) u8,

    scheduled_events: std.PriorityQueue(ScheduledEvent, void, ScheduledEvent.compare),
    _global_cycles: u64 = 0, // Cycles since the start of emulation.

    on_render_start: Callback = undefined,

    _allocator: std.mem.Allocator,

    _dummy: [4]u8 align(32) = @splat(0), // FIXME: Dummy space for non-implemented features

    pub fn create(allocator: std.mem.Allocator) !*Dreamcast {
        const dc = try allocator.create(Dreamcast);
        dc.* = Dreamcast{
            .cpu = try .init(allocator, dc),
            .maple = try .init(allocator),
            .flash = try .init(allocator),
            .hardware_registers = try allocator.allocWithOptions(u8, 0x20_0000, 4, null), // FIXME: Huge waste of memory.
            .scheduled_events = .init(allocator, {}),
            ._allocator = allocator,
        };

        if (SH4JITModule.FastMem) {
            dc.sh4_jit = try .init(allocator, null);
            dc.boot = @as([*]align(64) u8, @alignCast(@ptrCast(dc.sh4_jit.virtual_address_space.base_addr())))[0..BootSize];
            dc.ram = @as([*]align(64) u8, @ptrFromInt(@intFromPtr(dc.sh4_jit.virtual_address_space.base_addr()) + 0x0C00_0000))[0..RAMSize];
            dc.vram = @as([*]align(64) u8, @ptrFromInt(@intFromPtr(dc.sh4_jit.virtual_address_space.base_addr()) + 0x0400_0000))[0..Holly.VRAMSize];
            dc.aram = @as([*]align(64) u8, @ptrFromInt(@intFromPtr(dc.sh4_jit.virtual_address_space.base_addr()) + 0x0080_0000))[0..AICA.RAMSize];
        } else {
            dc.boot = try allocator.allocWithOptions(u8, BootSize, 4, null);
            dc.ram = try allocator.allocWithOptions(u8, RAMSize, 4, null);
            dc.vram = try allocator.allocWithOptions(u8, Holly.VRAMSize, 32, null);
            dc.aram = try allocator.allocWithOptions(u8, AICA.RAMSize, 4, null);
            dc.sh4_jit = try .init(allocator, dc.ram.ptr);
        }

        dc.gpu = try .init(allocator, dc);
        dc.aica = try .init(allocator, dc.aram);
        dc.gdrom = try .init(allocator, dc);
        dc.aica.setup_arm();

        errdefer dc.deinit();

        // Create 'userdata' folder if it doesn't exist
        try std.fs.cwd().makePath(HostPaths.get_userdata_path());

        const bios_path = try std.fs.path.join(allocator, &[_][]const u8{ HostPaths.get_data_path(), "dc_boot.bin" });
        defer allocator.free(bios_path);
        dc.load_bios(bios_path) catch |err| {
            switch (err) {
                error.FileNotFound => return error.BiosNotFound,
                else => return err,
            }
        };

        try dc.reset();

        return dc;
    }

    pub fn deinit(self: *@This()) void {
        // Write flash to disc. FIXME: Unused for now.
        // if (!@import("builtin").is_test) {
        //     const filename = get_user_flash_path();
        //     std.fs.cwd().makePath(std.fs.path.dirname(filename) orelse ".") catch |err| {
        //         dc_log.err("Failed to create user flash directory: {any}", .{err});
        //     };
        //     if (std.fs.cwd().createFile(filename, .{})) |file| {
        //         defer file.close();
        //         _ = file.writeAll(self.flash.data) catch |err| {
        //             dc_log.err("Failed to save user flash: {any}", .{err});
        //         };
        //     } else |err| {
        //         dc_log.err("Failed to open user flash '{s}' for writing: {any}", .{ filename, err });
        //     }
        // }

        self.scheduled_events.deinit();
        self.sh4_jit.deinit();
        self.gdrom.deinit();
        self.maple.deinit();
        self.aica.deinit();
        self.gpu.deinit();
        self.cpu.deinit();
        self.flash.deinit();

        if (!SH4JITModule.FastMem) {
            self._allocator.free(self.aram);
            self._allocator.free(self.vram);
            self._allocator.free(self.ram);
            self._allocator.free(self.boot);
        }
        self._allocator.free(self.hardware_registers);
    }

    pub fn reset(self: *@This()) !void {
        self.cpu.reset();
        self.gpu.reset();
        self.gdrom.reset();
        self.gdrom_hle.reset();
        try self.aica.reset();
        self.flash.reset();
        while (self.scheduled_events.removeOrNull() != null) {}

        try self.sh4_jit.reset();

        @memset(self.ram[0..], 0x00); // NOTE: Sonic Adventure 2 reads some unitialized memory around 0x0C000050...

        @memset(self.hardware_registers, 0x00);

        self.hw_register(u32, .SB_FFST).* = 0; // FIFO Status
        self.hw_register(u32, .SB_MDST).* = 0;
        self.hw_register(u32, .SB_DDST).* = 0;
        self.hw_register(u32, .SB_SDST).* = 0;
        self.hw_register(u32, .SB_GDST).* = 0;
        self.hw_register(u32, .SB_ISTNRM).* = 0;

        // Holly Version. TODO: Make it configurable?
        self.hw_register(u32, .SB_SBREV).* = 0x0B;
        self.hw_register(u32, .SB_G2ID).* = 0x12; // Only possible value, apparently.

        self.hw_register(u32, .SB_MSYS).* = 0x3A980000;
        self.hw_register(u32, .SB_MST).* = 0x000000FF;
        self.hw_register(u32, .SB_MMSEL).* = 0x00000001;

        // NOTE: SB_G1SYSM:
        // G1MRA [18:15]                   G1MRA[14:11]
        //   0 0 0 0  Mass production unit   0 0 0 1  Japan, South Korea, Asia NTSC
        //   1 1 0 0  SET4-8M                0 1 0 0  North America, Brazil, Argentina
        //   1 0 0 0  SET4-32M               1 1 0 0  Europe
        //   1 0 0 1  Dev.Box-16M
        //   1 0 1 0  Dev.Box-32M
        //   1 1 0 1  Graphics box
        //   Other codes Reserved
        self.hw_register(u32, .SB_G1SYSM).* = 0b0000_1100;

        self.hw_register(u32, .SB_G2DSTO).* = 0x000003FF;
        self.hw_register(u32, .SB_G2TRTO).* = 0x000003FF;
    }

    pub fn region_subdir(self: *@This()) []const u8 {
        return switch (self.region) {
            .Japan => "jp",
            .USA => "us",
            .Europe => "eu",
            else => "us",
        };
    }

    pub fn set_region(self: *@This(), region: Region) !void {
        self.region = region;
        try self.load_flash();
    }

    pub fn load_bios(self: *@This(), boot_path: []const u8) !void {
        var boot_file = std.fs.cwd().openFile(boot_path, .{}) catch |err| {
            dc_log.err(termcolor.red("Failed to open boot ROM at '{s}', error: {any}."), .{ boot_path, err });
            return err;
        };
        defer boot_file.close();
        const bytes_read = try boot_file.readAll(self.boot);
        std.debug.assert(bytes_read == 0x200000);
    }

    pub fn load_flash(self: *@This()) !void {
        // FIXME: User flash is sometimes corrupted. Always load default until I understand what's going on.
        const default_flash_path = try std.fs.path.join(self._allocator, &[_][]const u8{ HostPaths.get_data_path(), "dc_flash.bin" });
        defer self._allocator.free(default_flash_path);
        var flash_file = std.fs.cwd().openFile(default_flash_path, .{}) catch |e| {
            dc_log.err(termcolor.red("Failed to open default flash file at '{s}', error: {any}."), .{ default_flash_path, e });
            return e;
        };

        // var flash_file = std.fs.cwd().openFile(get_user_flash_path(), .{}) catch |err| f: {
        //     if (err == error.FileNotFound) {
        //         dc_log.info("Loading default flash ROM.", .{});
        //         break :f std.fs.cwd().openFile(default_flash_path, .{}) catch |e| {
        //             dc_log.err(termcolor.red("Failed to open default flash file at '{s}', error: {any}."), .{ default_flash_path, e });
        //             return e;
        //         };
        //     } else {
        //         dc_log.err(termcolor.red("Failed to open user flash file at '{s}', error: {any}."), .{ get_user_flash_path(), err });
        //         return err;
        //     }
        // };

        defer flash_file.close();
        const flash_bytes_read = try flash_file.readAll(self.flash.data);
        std.debug.assert(flash_bytes_read == 0x20000);

        // Adjust region.
        self.flash.data[0x1A002] = @as(u8, '0') + @intFromEnum(self.region);
        self.flash.data[0x1A0A2] = @as(u8, '0') + @intFromEnum(self.region);

        // Get current system config, update it with user preference and fix block crc.
        const system_bitmap = @as(*u512, @alignCast(@ptrCast(self.flash.data[0x1FFC0..0x20000].ptr)));
        const last_entry = @ctz(system_bitmap.*);
        if (last_entry == 0) return error.InvalidFlash;
        var system_block = self.flash.get_system_block(last_entry - 1);

        // Update saved time to avoid manual time adjustement screen on startup.
        const dc_timestamp = AICA.timestamp();
        system_block.user_payload.time_low = @truncate(dc_timestamp);
        system_block.user_payload.time_high = @truncate(dc_timestamp >> 16);

        system_block.update_crc();
    }

    pub fn load_at(self: *@This(), addr: u32, bin: []const u8) void {
        const start_addr = ((addr & 0x1FFFFFFF) - 0x0C000000);
        @memcpy(self.ram[start_addr .. start_addr + bin.len], bin);
    }

    pub fn skip_bios(self: *@This(), hle_syscalls: bool) !void {
        self.cpu.state_after_boot_rom();

        @memset(self.ram[0x00200000..0x00300000], 0x00); // FIXME: I think KallistiOS relies on that, or maybe I messed up somewhere else. (the BootROM does clear this section of RAM)

        // Copy subroutine to RAM. Some of it will be overwritten, I'm trying to work out what's important and what's not.
        inline for (0..16) |i| {
            try self.cpu.write(u16, 0x8C0000E0 + 2 * i, try self.cpu.read(u16, 0x800000FE - 2 * i));
        }
        // Copy a portion of the boot ROM to RAM.
        try self.cpu.write(u32, 0xA05F74E4, 0x001FFFFF);

        @memcpy(self.ram[0x00000100..0x00004000], self.boot[0x00000100..0x00004000]);
        @memcpy(self.ram[0x00008000..0x00200000], self.boot[0x00008000..0x00200000]);

        const IP_bin_HLE = false;
        if (IP_bin_HLE) {
            // Copy a portion of the flash ROM to RAM.
            inline for (0..8) |i| {
                try self.cpu.write(u8, 0x8C000068 + i, try self.cpu.read(u8, 0x0021A056 + i));
            }
            inline for (0..5) |i| {
                try self.cpu.write(u8, 0x8C000068 + 8 + i, try self.cpu.read(u8, 0x0021A000 + i));
            }
            // FIXME: Load system settings from flashrom (User partition (2), logical block 5), instead of these hardcoded values.
            //inline for (.{ 0xBC, 0xEA, 0x90, 0x5E, 0xFF, 0x04, 0x00, 0x01 }, 0..) |val, i| {
            inline for (.{ 0x00, 0x00, 0x89, 0xFC, 0x5B, 0xFF, 0x01, 0x00, 0x00, 0x7D, 0x0A, 0x62, 0x61 }, 0..) |val, i| {
                try self.cpu.write(u8, 0x8C000068 + 13 + i, val);
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
            try self.cpu.write(u32, p[0], p[1]);
        }
        // Replace them by HLE counterparts (see syscall.zig) by inserting fake opcodes.
        if (hle_syscalls) {
            // System
            try self.cpu.write(u16, 0x8C003C00, 0b0000000000010000);
            // Font
            try self.cpu.write(u16, 0x8C003D80, 0b0000000000100000);
            // Flashrom
            try self.cpu.write(u16, 0x8C003D00, 0b0000000000110000);
            // GD
            try self.cpu.write(u16, 0x8C001000, 0b0000000001000000);
            // GD2
            try self.cpu.write(u16, 0x8C0010F0, 0b0000000001010000);
            // Misc
            try self.cpu.write(u16, 0x8C000800, 0b0000000001100000);
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
            try self.cpu.write(u32, p[0], p[1]);
        }

        // Load IP.bin from disc (16 first sectors of the last track)
        // FIXME: Here we assume the last track is the 3rd.
        if (self.gdrom.disc) |*disc|
            _ = try disc.load_bytes(45150, 16 * 2048, self.ram[0x00008000..]);

        // IP.bin patches
        inline for (.{
            .{ 0xAC0090D8, 0x5113 },
            .{ 0xAC00940A, 0x000B },
            .{ 0xAC00940C, 0x0009 },
        }) |p| {
            try self.cpu.write(u16, p[0], p[1]);
        }

        // Patch some functions apparently used by interrupts
        // (And some other random stuff that the boot ROM sets for some reason
        //  and I'm afraid some games might use. I'm not taking any more chances)

        // Sleep on error?
        try self.cpu.write(u32, 0x8C000000, 0x00090009);
        try self.cpu.write(u32, 0x8C000004, 0x001B0009);
        try self.cpu.write(u32, 0x8C000008, 0x0009AFFD);
        // ??
        try self.cpu.write(u16, 0x8C00000C, 0);
        try self.cpu.write(u16, 0x8C00000E, 0);
        // RTE - Some interrupts jump there instead of having their own RTE, I have NO idea why.
        try self.cpu.write(u32, 0x8C000010, 0x00090009); // nop nop
        try self.cpu.write(u32, 0x8C000014, 0x0009002B); // rte nop
        // RTS
        try self.cpu.write(u32, 0x8C000018, 0x00090009);
        try self.cpu.write(u32, 0x8C00001C, 0x0009000B);

        // ??
        try self.cpu.write(u8, 0x8C00002C, 0x16);
        try self.cpu.write(u32, 0x8C000064, 0x8c008100);
        try self.cpu.write(u16, 0x8C000090, 0);
        try self.cpu.write(u16, 0x8C000092, @bitCast(@as(i16, -128)));

        self.hw_register(u32, .SB_MDST).* = 0;
        self.hw_register(u32, .SB_DDST).* = 0;

        self.gpu._get_register(u32, .SPG_HBLANK_INT).* = 0x03450000;
        self.gpu._get_register(u32, .SPG_VBLANK_INT).* = 0x00150208;
        self.gpu._get_register(u32, .SPG_CONTROL).* = 0x00000100;
        self.gpu._get_register(u32, .SPG_HBLANK).* = 0x007E0345;
        self.gpu._get_register(u32, .SPG_VBLANK).* = 0x00280208;
        self.gpu._get_register(u32, .SPG_WIDTH).* = 0x03F1933F;
        self.gpu._get_register(u32, .SPG_LOAD).* = 0x020C0359;
        self.gpu.finalize_deserialization();
    }

    pub fn set_flash_settings(self: *@This(), region: Region, lang: Language, video_mode: VideoMode) void {
        self.flash[0x1A002] = @as(u8, '0') + @intFromEnum(region);
        self.flash[0x1A0A2] = @as(u8, '0') + @intFromEnum(region);
        self.flash[0x1A003] = @as(u8, '0') + @intFromEnum(lang);
        self.flash[0x1A0A3] = @as(u8, '0') + @intFromEnum(lang);
        self.flash[0x1A004] = @as(u8, '0') + @intFromEnum(video_mode);
        self.flash[0x1A0A4] = @as(u8, '0') + @intFromEnum(video_mode);
    }

    pub inline fn hw_register(self: *@This(), comptime T: type, r: HardwareRegister) *T {
        return self.hw_register_addr(T, @intFromEnum(r));
    }
    pub inline fn hw_register_addr(self: *@This(), comptime T: type, addr: u32) *T {
        std.debug.assert(addr >= 0x005F6800 and addr < 0x005F6800 + self.hardware_registers.len);
        return @as(*T, @alignCast(@ptrCast(&self.hardware_registers[addr - 0x005F6800])));
    }
    pub inline fn read_hw_register_addr(self: *const @This(), comptime T: type, addr: u32) T {
        std.debug.assert(addr >= 0x005F6800 and addr < 0x005F6800 + self.hardware_registers.len);
        return @as(*const T, @alignCast(@ptrCast(&self.hardware_registers[addr - 0x005F6800]))).*;
    }
    pub inline fn read_hw_register(self: *const @This(), comptime T: type, r: HardwareRegister) T {
        return self.read_hw_register_addr(T, @intFromEnum(r));
    }
    pub fn write_hw_register(self: *@This(), comptime T: type, addr: u32, value: T) void {
        const reg: HardwareRegister = @enumFromInt(addr);
        if (addr >= 0x005F7000 and addr <= 0x005F709C) {
            if (T != u8 and T != u16) return dc_log.err("Invalid Write({any}) to 0x{X:0>8} (GDROM)\n", .{ T, addr });
            return self.gdrom.write_register(T, addr, value);
        }
        // Hardware registers
        switch (reg) {
            .SB_SFRES => { // Software Reset
                if (T != u32) return dc_log.err("Invalid Write({any}) to 0x{X:0>8} (SB_SFRES)\n", .{ T, addr });
                if (value == 0x00007611)
                    self.cpu.software_reset();
            },
            .SB_PDST => if (value == 1) self.start_pvr_dma(),
            .SB_SDST => if (value == 1) self.start_sort_dma(),
            .SB_E1ST, .SB_E2ST, .SB_DDST => {
                if (value == 1)
                    dc_log.err(termcolor.red("Unimplemented {any} DMA initiation!"), .{reg});
            },
            .SB_ADSUSP, .SB_E1SUSP, .SB_E2SUSP, .SB_DDSUSP => {
                if ((value & 1) == 1)
                    dc_log.debug(termcolor.yellow("Unimplemented DMA Suspend Request to {any}"), .{reg});
            },
            .SB_ADST => {
                if (value == 1) self.aica.start_dma(self);
            },
            .SB_GDEN => {
                dc_log.debug("Write to SB_GDEN: {X}", .{value});
                if (value == 0) self.abort_gd_dma();
                self.hw_register(T, .SB_GDEN).* = value;
            },
            .SB_GDST => if (value == 1) self.start_gd_dma(),
            .SB_GDSTARD, .SB_GDLEND, .SB_ADSTAGD, .SB_E1STAGD, .SB_E2STAGD, .SB_DDSTAGD, .SB_ADSTARD, .SB_E1STARD, .SB_E2STARD, .SB_DDSTARD, .SB_ADLEND, .SB_E1LEND, .SB_E2LEND, .SB_DDLEND => {
                dc_log.warn(termcolor.yellow("Ignoring write({any}) to Read Only register {s} = {X:0>8}."), .{ T, @tagName(reg), value });
            },
            .SB_MDAPRO => {
                if (T != u32) return dc_log.err("Invalid Write({any}) to 0x{X:0>8} (SB_MDAPRO)\n", .{ T, addr });
                // This register specifies the address range for Maple-DMA involving the system (work) memory.
                // Check "Security code"
                if (value & 0xFFFF0000 != 0x61550000) return;
                self.hw_register(T, .SB_MDAPRO).* = value;
            },
            .SB_MDST => if (value == 1) self.start_maple_dma(),
            .SB_ISTNRM => {
                if (T != u32) return dc_log.err("Invalid Write({any}) to 0x{X:0>8} (SB_ISTNRM)\n", .{ T, addr });
                // Interrupt can be cleared by writing "1" to the corresponding bit.
                self.hw_register(u32, .SB_ISTNRM).* &= ~(value & 0x3FFFFF);
            },
            .SB_ISTERR => {
                if (T != u32) return dc_log.err("Invalid Write({any}) to 0x{X:0>8} (SB_ISTERR)\n", .{ T, addr });
                // Interrupt can be cleared by writing "1" to the corresponding bit.
                self.hw_register(u32, .SB_ISTERR).* &= ~value;
            },
            .SB_C2DSTAT => {
                if (T != u32) return dc_log.err("Invalid Write({any}) to 0x{X:0>8} (SB_C2DSTAT)\n", .{ T, addr });
                self.hw_register(u32, .SB_C2DSTAT).* = 0x10000000 | (0x03FFFFFF & value);
            },
            .SB_C2DST => if (value == 1) self.start_ch2_dma() else self.end_ch2_dma(),
            else => {
                dc_log.debug("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, HardwareRegisters.getRegisterName(addr), value });
                self.hw_register_addr(T, addr).* = value;
            },
        }
    }

    pub inline fn _get_memory(self: *@This(), addr: u32) *u8 {
        std.debug.assert(addr <= 0x1FFFFFFF);

        switch (addr) {
            // Area 3 - System RAM (16MB) - 0x0C000000 to 0x0FFFFFFF, mirrored 4 times.
            0x0C000000...0x0FFFFFFF => return &self.ram[addr & 0x00FFFFFF],
            // Area 1 - 64bit path
            0x04000000...0x04FFFFFF, 0x06000000...0x06FFFFFF => return &self.gpu.vram[addr & (Dreamcast.VRAMSize - 1)],
            0x00000000...0x03FFFFFF => { // Area 0 - Boot ROM, Flash ROM, Hardware Registers
                const area_0_addr = addr & 0x01FFFFFF;
                switch (area_0_addr) {
                    0x00000000...0x001FFFFF => return &self.boot[area_0_addr],
                    0x00200000...0x0021FFFF => return &self.flash.data[area_0_addr & 0x1FFFF],
                    0x005F6800...0x005F6FFF => return self.hw_register_addr(u8, area_0_addr),
                    0x005F7000...0x005F709C => @panic("_get_memory to GDROM Register. This should be handled in read/write functions."),
                    0x005F709D...0x005F7FFF => return self.hw_register_addr(u8, area_0_addr),
                    0x005F8000...0x005F9FFF => return self.gpu._get_register_from_addr(u8, area_0_addr),
                    0x005FA000...0x005FFFFF => return self.hw_register_addr(u8, area_0_addr),
                    0x00600000...0x006007FF => {
                        const static = struct {
                            var once = false;
                        };
                        if (!static.once) {
                            static.once = true;
                            dc_log.warn(termcolor.yellow("  Unimplemented _get_memory to MODEM: {X:0>8} (This will only be reported once)"), .{addr});
                        }
                        self._dummy = .{ 0, 0, 0, 0 };
                        return @ptrCast(&self._dummy);
                    },
                    // G2 AICA Register
                    0x00700000...0x00707FFF => @panic("_get_memory to AICA Register. This should be handled in read/write functions."),
                    // G2 AICA RTC Registers
                    0x00710000...0x00710008 => @panic("_get_memory to AICA RTC Register. This should be handled in read/write functions."),
                    0x00800000...0x009FFFFF, 0x02800000...0x029FFFFF => { // G2 Wave Memory and Mirror
                        dc_log.debug("NOTE: _get_memory to AICA Wave Memory @{X:0>8} ({X:0>8}). This should be handled in read/write functions, except for DMA. Get rid of this warning when the ARM core is stable enough! (Direct access to wave memory specifically should be fine.)", .{ addr, area_0_addr });
                        return @ptrCast(&self.aica.wave_memory[area_0_addr & (self.aica.wave_memory.len - 1)]);
                    },
                    0x01000000...0x01FFFFFF => { // Expansion Devices
                        dc_log.warn(termcolor.yellow("  Unimplemented _get_memory to Expansion Devices: {X:0>8} ({X:0>8})"), .{ addr, area_0_addr });

                        // FIXME: TEMP DEBUG: Crazy Taxi accesses 030100C0 (010100C0) and 030100A0 (010100A0)
                        //        And 0101003C to 0101007C, and 01010014, and 01010008

                        // FIXME: I have no idea why Crazy Taxi seem to expect to find 0x80 at 01010008, but this lets it go further.
                        self._dummy = .{ 0x80, 0, 0, 0 };

                        return @ptrCast(&self._dummy);
                    },
                    else => {
                        dc_log.warn(termcolor.yellow("  Unimplemented _get_memory to Area 0: {X:0>8} ({X:0>8})"), .{ addr, area_0_addr });
                        self._dummy = .{ 0, 0, 0, 0 };
                        return @ptrCast(&self._dummy);
                    },
                }
            },
            // Area 2 - Nothing
            0x08000000...0x0BFFFFFF => std.debug.panic("Invalid _get_memory to Area 2 @{X:0>8}", .{addr}),
            0x10000000...0x13FFFFFF => { // Area 4 - Tile accelerator command input
                // self.panic_debug("Unexpected _get_memory to Area 4 @{X:0>8} - This should only be accessible via write32 or DMA.", .{addr});
                // NOTE: Marvel vs. Capcom 2 reads from here (Addr:103464A0 PC:8C031D3C). Ignoring it doesn't seem to hurt, so... Doing that instead of panicking for now.
                dc_log.err(termcolor.red("[PC: 0x{X:0>8}] Unexpected _get_memory to Area 4 @{X:0>8} - This should only be accessible via write32 or DMA."), .{ self.cpu.pc, addr });
                self._dummy = .{ 0, 0, 0, 0 };
                return @ptrCast(&self._dummy);
            },
            0x14000000...0x17FFFFFF => { // Area 5 - G2 Expansion Devices
                dc_log.warn(termcolor.yellow("Unimplemented _get_memory to Area 5 (G2 Expansion Devices): {X:0>8}"), .{addr});
                self._dummy = .{ 0, 0, 0, 0 };
                return @ptrCast(&self._dummy);
            },
            // Area 6 - Nothing
            0x18000000...0x1BFFFFFF => std.debug.panic("Invalid _get_memory to Area 6 @{X:0>8}", .{addr}),
            // Area 7 - Internal I/O registers (same as P4)
            0x1F000000...0x1FFFFFFF => {
                std.debug.assert(self.cpu.sr.md == 1);
                return self.cpu.p4_register_addr(u8, addr);
            },
            else => {
                // FIXME: This space should be Unassigned/Reserved.
                //        Returns a dummy value instead of panicking.
                //        Metropolis Street Racer and Legacy of the Kain - Soul Reaver write to 0xBCXXXXXX,
                //        and I have no idea if this is an issue with the emulator... See #51.
                //        Ignoring the writes allow these games to progress a bit, but this might become an issue.
                dc_log.err(termcolor.red("Invalid _get_memory @{X:0>8}"), .{addr});
                return @ptrCast(&self._dummy);
            },
        }
    }

    pub inline fn read(self: *const @This(), comptime T: type, addr: u32) T {
        switch (addr) {
            // Area 0
            0x00000000...0x01FFFFFF, 0x02000000...0x02FFFFFF => {
                switch (addr) {
                    0x005F6800...0x005F7FFF => {
                        switch (addr) {
                            0x005F7000...0x005F709C => {
                                check_type(&[_]type{ u8, u16 }, T, "Invalid Read({any}) to GDRom Register 0x{X:0>8}\n", .{ T, addr });
                                return @constCast(self).gdrom.read_register(T, addr);
                            },
                            else => {
                                // Too spammy even for debugging.
                                if (addr != @intFromEnum(HardwareRegister.SB_ISTNRM) and addr != @intFromEnum(HardwareRegister.SB_FFST))
                                    dc_log.debug("  Read({any}) to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{
                                        T, addr, HardwareRegisters.getRegisterName(addr), @as(*const u32, @alignCast(@ptrCast(@constCast(self)._get_memory(addr)))).*,
                                    });
                                return self.read_hw_register_addr(T, addr);
                            },
                        }
                    },
                    0x005F8000...0x005F9FFF => {
                        return self.gpu.read_register(T, @enumFromInt(addr));
                    },
                    // NOTE: 0x00700000...0x00FFFFFF mirrors to 0x02700000...0x02FFFFFF
                    0x00700000...0x00707FE0, 0x02700000...0x02707FE0 => {
                        check_type(&[_]type{ u8, u32 }, T, "Invalid Read({any}) to 0x{X:0>8}\n", .{ T, addr });
                        return self.aica.read_register(T, addr & 0x00FFFFFF);
                    },
                    0x00710000...0x00710008, 0x02710000...0x02710008 => {
                        check_type(&[_]type{u32}, T, "Invalid Read({any}) to 0x{X:0>8}\n", .{ T, addr });
                        return @truncate(self.aica.read_rtc_register(addr & 0x00FFFFFF));
                    },
                    0x00800000...0x00FFFFFF, 0x02800000...0x02FFFFFF => {
                        return self.aica.read_mem(T, addr & 0x00FFFFFF);
                    },
                    else => {},
                }
            },
            // Area 1 - 64bit access
            0x04000000...0x04FFFFFF, 0x06000000...0x06FFFFFF => {
                return @as(*T, @alignCast(@ptrCast(&self.gpu.vram[addr & (Dreamcast.VRAMSize - 1)]))).*;
            },
            // Area 1 - 32bit access
            0x05000000...0x05FFFFFF, 0x07000000...0x07FFFFFF => {
                if (T == u64) {
                    dc_log.debug("Read(64) from 0x{X:0>8}", .{addr});
                    return @as(u64, self.read(u32, addr + 4)) << 32 | self.read(u32, addr);
                }
                return self.gpu.read_vram(T, addr);
            },
            // Area 4 - Tile accelerator command input
            0x10000000...0x13FFFFFF => {
                // DCA3 Hack
                if (addr & (@as(u32, 1) << 25) != 0)
                    return self.cpu.operand_cache_read(T, addr);
            },
            // Area 7
            0x1C000000...0x1FFFFFFF => {
                // Only when area 7 in external memory space is accessed using virtual memory space, addresses H'1F00 0000
                // to H'1FFF FFFF of area 7 are not designated as a reserved area, but are equivalent to the P4 area
                // control register area in the physical memory space
                if (addr >= 0x1F000000) {
                    if (self.cpu._mmu_state != .Disabled) {
                        return self.cpu.read_p4(T, addr | 0xE000_0000);
                    } else {
                        dc_log.err(termcolor.red("Read({any}) to Area 7 without using virtual memory space: {X:0>8}"), .{ T, addr });
                        return 0;
                    }
                }
            },
            else => {},
        }

        return @as(*const T, @alignCast(@ptrCast(
            @constCast(self)._get_memory(addr),
        ))).*;
    }

    pub inline fn write(self: *@This(), comptime T: type, addr: u32, value: T) void {
        switch (addr) {
            // Area 0, and mirrors
            0x00000000...0x01FFFFFF, 0x02000000...0x03FFFFFF => {
                switch (addr) {
                    0x00200000...0x0021FFFF => {
                        check_type(&[_]type{u8}, T, "Invalid Write({any}) to 0x{X:0>8} (Flash) = 0x{X}\n", .{ T, addr, value });
                        return self.flash.write(addr & 0x1FFFF, value);
                    },
                    0x005F6800...0x005F7FFF => return self.write_hw_register(T, addr, value),
                    0x005F8000...0x005F9FFF => {
                        if (T == u64) {
                            // FIXME: Allow 64bit writes to Palette RAM? Metropolis Street Racer does it, not sure how normal it is :)
                            if (addr >= 0x005F9000 and addr <= 0x005F9FFC) {
                                dc_log.warn(termcolor.yellow("Write({any}) to Palette RAM @{X:0>8} = 0x{X:0>16}"), .{ T, addr, value });
                                self.gpu._get_register_from_addr(u64, addr).* = value;
                                return;
                            }
                        }
                        check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (Holly Registers) = 0x{X}\n", .{ T, addr, value });
                        return self.gpu.write_register(addr, value);
                    },
                    // NOTE: 0x00700000...0x00FFFFFF mirrors to 0x02700000...0x02FFFFFF
                    0x00700000...0x0070FFFF, 0x02700000...0x0270FFFF => {
                        check_type(&[_]type{ u8, u32 }, T, "Invalid Write({any}) to 0x{X:0>8} (AICA Registers) = 0x{X}\n", .{ T, addr, value });
                        return self.aica.write_register(T, addr & 0x00FFFFFF, value);
                    },
                    0x00710000...0x00710008, 0x02710000...0x02710008 => {
                        check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (AICA RTC Registers) = 0x{X}\n", .{ T, addr, value });
                        return self.aica.write_rtc_register(addr & 0x00FFFFFF, value);
                    },
                    0x00800000...0x00FFFFFF, 0x02800000...0x02FFFFFF => return self.aica.write_mem(T, addr & 0x00FFFFFF, value),
                    else => {},
                }
            },
            // Area 1 - 64bit access
            0x04000000...0x04FFFFFF, 0x06000000...0x06FFFFFF => {
                @as(*T, @alignCast(@ptrCast(&self.gpu.vram[addr & (Dreamcast.VRAMSize - 1)]))).* = value;
                return;
            },
            // Area 1 - 32bit access
            0x05000000...0x05FFFFFF, 0x07000000...0x07FFFFFF => {
                if (T == u64) {
                    dc_log.debug("Write(64) to 0x{X:0>8} = 0x{X:0>16}", .{ addr, value });
                    self.write(u32, addr, @truncate(value));
                    self.write(u32, addr + 4, @truncate(value >> 32));
                    return;
                }
                return self.gpu.write_vram(T, addr, value);
            },
            // Area 4
            0x10000000...0x13FFFFFF => {
                // DCA3 Hack
                if (addr & (@as(u32, 1) << 25) != 0)
                    return self.cpu.operand_cache_write(T, addr, value);
                check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (TA Registers) = 0x{X}\n", .{ T, addr, value });
                const LMMode = self.read_hw_register(u32, if (addr >= 0x11000000 and addr < 0x12000000) .SB_LMMODE0 else .SB_LMMODE1);
                const access_32bit = LMMode != 0;
                return self.gpu.write_ta(addr, &[1]u32{value}, if (access_32bit) .b32 else .b64);
            },
            // Area 7
            0x1C000000...0x1FFFFFFF => {
                // Only when area 7 in external memory space is accessed using virtual memory space, addresses H'1F00 0000
                // to H'1FFF FFFF of area 7 are not designated as a reserved area, but are equivalent to the P4 area
                // control register area in the physical memory space
                if (addr >= 0x1F000000) {
                    if (self.cpu._mmu_state != .Disabled) {
                        return self.cpu.write_p4(T, addr | 0xE000_0000, value);
                    } else {
                        dc_log.err(termcolor.red("Write({any}) to Area 7 without using virtual memory space: {X:0>8} = {X}"), .{ T, addr, value });
                        return;
                    }
                }
            },
            else => {},
        }

        @as(*T, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub fn tick(self: *@This(), max_instructions: u8) !u32 {
        const cycles = SH4Interpreter.execute(&self.cpu, max_instructions);
        try self.tick_peripherals(cycles);
        return cycles;
    }

    pub fn tick_jit(self: *@This()) !u32 {
        var cycles: u32 = 0;
        while (cycles < 64)
            cycles += try self.sh4_jit.execute(&self.cpu);
        try self.tick_peripherals(cycles);
        return cycles;
    }

    fn tick_peripherals(self: *@This(), cycles: u32) !void {
        self.advance_scheduled_interrupts(cycles);
        try self.aica.update(self, cycles);
    }

    // TODO: Add helpers for external interrupts and errors.

    pub fn schedule_event(self: *@This(), event: ScheduledEvent.Event, cycles: usize) void {
        //std.debug.print("Schedule event in {d}: {any}\n", .{ cycles, event });
        self.scheduled_events.add(.{
            .trigger_cycle = self._global_cycles +% cycles,
            .event = event,
            .interrupt = null,
        }) catch |err| {
            std.debug.panic("Failed to schedule event: {}", .{err});
        };
    }

    pub fn schedule_int_event(self: *@This(), int: HardwareRegisters.SB_ISTNRM, event: ScheduledEvent.Event, cycles: u32) void {
        self.scheduled_events.add(.{
            .trigger_cycle = self._global_cycles +% cycles,
            .interrupt = .{ .Normal = int },
            .event = event,
        }) catch |err| {
            std.debug.panic("Failed to schedule event: {}", .{err});
        };
    }

    pub fn schedule_interrupt(self: *@This(), int: HardwareRegisters.SB_ISTNRM, cycles: u32) void {
        self.scheduled_events.add(.{
            .trigger_cycle = self._global_cycles +% cycles,
            .interrupt = .{ .Normal = int },
        }) catch |err| {
            std.debug.panic("Failed to schedule interrupt: {}", .{err});
        };
    }

    pub fn schedule_external_interrupt(self: *@This(), int: HardwareRegisters.SB_ISTEXT, cycles: u32) void {
        self.scheduled_events.add(.{
            .trigger_cycle = self._global_cycles +% cycles,
            .interrupt = .{ .External = int },
            .event = .None,
        }) catch |err| {
            std.debug.panic("Failed to schedule external interrupt: {}", .{err});
        };
    }

    pub fn clear_event(self: *@This(), event: ScheduledEvent.Event) void {
        var not_done: bool = true;
        while (not_done) {
            not_done = false;
            var it = self.scheduled_events.iterator();
            var idx: u32 = 0;
            while (it.next()) |entry| {
                if (entry.event == std.meta.activeTag(event) and std.meta.eql(entry.event, event)) {
                    _ = self.scheduled_events.removeIndex(idx);
                    not_done = true;
                    break;
                }
                idx += 1;
            }
        }
    }

    fn advance_scheduled_interrupts(self: *@This(), cycles: u32) void {
        self._global_cycles += cycles;
        while (self.scheduled_events.peek()) |event| {
            if (event.trigger_cycle <= self._global_cycles) {
                if (event.interrupt) |int| {
                    switch (int) {
                        .Normal => self.raise_normal_interrupt(int.Normal),
                        .External => self.raise_external_interrupt(int.External),
                    }
                }
                switch (event.event) {
                    .None => {},
                    .EndGDDMA => self.end_gd_dma(),
                    .EndSortDMA => self.end_sort_dma(),
                    .EndAICADMA => self.aica.end_dma(self),
                    .TimerUnderflow => |e| self.cpu.on_timer_underflow(e.channel),
                    .HBlankIn => self.gpu.on_hblank_in(),
                    .VBlankIn => self.gpu.on_vblank_in(),
                    .VBlankOut => self.gpu.on_vblank_out(),
                }
                _ = self.scheduled_events.remove();
            } else break;
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

    pub fn clear_external_interrupt(self: *@This(), int: HardwareRegisters.SB_ISTEXT) void {
        self.hw_register(u32, .SB_ISTEXT).* &= ~@as(u32, @bitCast(int));
        self.hw_register(u32, .SB_ISTNRM).* |= @bitCast(HardwareRegisters.SB_ISTNRM{ .ExtStatus = if (self.hw_register(u32, .SB_ISTEXT).* != 0) 1 else 0 });

        self.check_sb_interrupts();
    }

    fn check_sb_interrupts(self: *@This()) void {
        const istnrm = self.read_hw_register(u32, .SB_ISTNRM);
        const istext = self.read_hw_register(u32, .SB_ISTEXT);
        const isterr = self.read_hw_register(u32, .SB_ISTERR);
        if ((istnrm & self.read_hw_register(u32, .SB_IML6NRM)) != 0 or (istext & self.read_hw_register(u32, .SB_IML6EXT)) != 0 or (isterr & self.read_hw_register(u32, .SB_IML6ERR)) != 0) {
            self.cpu.request_interrupt(.IRL9);
        } else if ((istnrm & self.read_hw_register(u32, .SB_IML4NRM)) != 0 or (istext & self.read_hw_register(u32, .SB_IML4EXT)) != 0 or (isterr & self.read_hw_register(u32, .SB_IML4ERR)) != 0) {
            self.cpu.request_interrupt(.IRL11);
        } else if ((istnrm & self.read_hw_register(u32, .SB_IML2NRM)) != 0 or (istext & self.read_hw_register(u32, .SB_IML2EXT)) != 0 or (isterr & self.read_hw_register(u32, .SB_IML2ERR)) != 0) {
            self.cpu.request_interrupt(.IRL13);
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
        if ((self.hw_register(u32, .SB_GDEN).* & 1) == 1) {
            const dst_addr = self.read_hw_register(u32, .SB_GDSTAR) & 0x1FFFFFE0;
            const len = self.read_hw_register(u32, .SB_GDLEN) & 0x01FFFFE0;
            const direction = self.read_hw_register(u32, .SB_GDDIR);

            if (direction == 0) {
                dc_log.err(termcolor.red("DMA to GD-ROM not implemented."), .{});
                return;
            }

            self.hw_register(u32, .SB_GDST).* = 1;
            // FIXME: Contrary to what I've said in 30cc565, it is clearly stated in the docs that SB_GDLEND does counts up...
            self.hw_register(u32, .SB_GDLEND).* = 0;
            self.hw_register(u32, .SB_GDLEND).* = len; // FIXME: This should start at 0 and count up, but for some reason Jet Set Radio has issues when I try to do this properly (no music in menu; infinite loading screen...). This is clearly not a proper fix, I just don't know what the actual cause is, and how to properly fix it.
            self.hw_register(u32, .SB_GDSTARD).* = dst_addr;

            dc_log.debug("GD-ROM-DMA! {X:0>8} ({X:0>8} bytes / {X:0>8} in queue)", .{ dst_addr, len, self.gdrom.dma_data_queue.count });

            // NOTE: This should use ch0-DMA, but the SH4 DMAC implementation can't handle this case (yet?).
            //       Unless we copy u16 by u16 from the data register, but, mmh, yeah.
            const copied = self.gdrom.dma_data_queue.read(@as([*]u8, @ptrCast(self._get_memory(dst_addr)))[0..len]);

            if (copied < len) {
                dc_log.warn(termcolor.yellow("  GD DMA: {X:0>8} bytes copied out of {X:0>8} expected."), .{ copied, len });
                // Pad with zeroes in this case.
                @memset(@as([*]u8, @ptrCast(self._get_memory(dst_addr)))[copied..len], 0);
            }

            // Simulate using ch0
            const chcr = self.cpu.p4_register(SH4Module.P4.CHCR, .CHCR0);
            chcr.*.sm = 0;
            chcr.*.dm = 1;
            chcr.*.ts = 4;
            self.cpu.p4_register(u32, .DAR0).* = dst_addr;
            self.cpu.p4_register(u32, .DMATCR0).* = @divExact(len, 32);

            self.schedule_int_event(
                .{ .EoD_GDROM = 1 },
                .EndGDDMA,
                if (len <= 128 * 1024) // Transfer fit in GD-ROM 128k buffer. Advertised speed from the buffer: 13.3 MB/s
                    14 * len
                else // GDROM speed: 1.8 MB/s
                    // FIXME: Added a maximum amount of cycles here, as very large DMA were causing visible freezes in Soul Calibur.
                    //        The proper fix is probably to have the GDROM send data in multiple DMA instead of a single, huge one.
                    @as(u32, 14) * 128 * 1024 + @as(u32, 111) * (@min(len, 0x40000) - 128 * 1024),
            );

            self.sh4_jit.invalidate(dst_addr, dst_addr + len);
        }
    }

    fn end_gd_dma(self: *@This()) void {
        const len = self.read_hw_register(u32, .SB_GDLEN) & 0x01FFFFE0;
        self.cpu.end_dmac(0);
        self.hw_register(u32, .SB_GDST).* = 0;
        self.hw_register(u32, .SB_GDLEND).* = len;
        self.hw_register(u32, .SB_GDSTARD).* += len;
        self.gdrom.on_dma_end(self);
    }

    pub fn abort_gd_dma(self: *@This()) void {
        if (self.read_hw_register(u32, .SB_GDST) != 0) {
            self.hw_register(u32, .SB_GDST).* = 0;
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

        self.sh4_jit.invalidate(dst_addr, dst_addr + len);

        self.schedule_interrupt(.{ .EoD_CH2 = 1 }, 200); // FIXME: Arbitrary timing.
    }

    pub fn end_ch2_dma(self: *@This()) void {
        self.hw_register(u32, .SB_C2DST).* = 0;
    }

    pub fn start_pvr_dma(self: *@This()) void {
        if (self.read_hw_register(u32, .SB_PDEN) != 1) return;

        self.hw_register(u32, .SB_PDST).* = 1;

        const pvr_start = self.read_hw_register(u32, .SB_PDSTAP) & 0x1FFF_FFE0;
        const mem_start = self.read_hw_register(u32, .SB_PDSTAR) & 0x1FFF_FFE0;
        const len = self.read_hw_register(u32, .SB_PDLEN) & 0x00FF_FFE0;
        const dir = self.read_hw_register(u32, .SB_PDDIR);

        const src = if (dir == 0) mem_start else pvr_start;
        const dst = if (dir == 0) pvr_start else mem_start;

        dc_log.debug("  Start PVR DMA: {X:0>8} -> {X:0>8} ({X:0>8} bytes)", .{ src, dst, len });

        var chcr = self.cpu.p4_register(SH4Module.P4.CHCR, .CHCR0);
        chcr.ts = 4;
        chcr.rs = 2;
        self.cpu.p4_register(u32, .SAR0).* = src;
        self.cpu.p4_register(u32, .DAR0).* = dst;
        self.cpu.p4_register(u32, .DMATCR0).* = len / 32;

        self.cpu.start_dmac(0);

        if (dir == 1)
            self.sh4_jit.invalidate(dst, dst + len);

        self.hw_register(u32, .SB_PDST).* = 0;

        self.schedule_interrupt(.{ .EoD_PVR = 1 }, 200); // FIXME: Arbitrary timing.
    }

    pub fn start_sort_dma(self: *@This()) void {
        dc_log.debug("Start Sort-DMA", .{});
        // NOTE: Uses ch0:DDT
        self.hw_register(u32, .SB_SDST).* = 1;

        const start_link_address_table = self.read_hw_register(u32, .SB_SDSTAW);
        const start_link_base_address = self.read_hw_register(u32, .SB_SDBAAW);
        const bit_width = self.read_hw_register(u32, .SB_SDWLT);
        const link_address = self.read_hw_register(u32, .SB_SDLAS);
        dc_log.debug("  Start Link Address Table: {X}", .{start_link_address_table});
        dc_log.debug("  Start Link Base Address: {X}", .{start_link_base_address});
        dc_log.debug("  Bit Width: {X}", .{bit_width});
        dc_log.debug("  Link Address: {X}", .{link_address});

        const bytes_transfered = if (bit_width == 0) self.sort_dma_link(u16) else self.sort_dma_link(u32);

        self.schedule_int_event(
            .{ .EoD_PVRSort = 1 },
            .EndSortDMA,
            // TODO: TA Bus? 1 GB/s?
            bytes_transfered / (1024 * 1024 * 1024 / SH4Clock),
        );
    }

    fn sort_dma_link(self: *@This(), comptime T: type) u32 {
        const start_link_address_table = self.hw_register(u32, .SB_SDSTAW).*;
        const start_link_base_address = self.hw_register(u32, .SB_SDBAAW).*;
        const offset_factor: u32 = if (self.hw_register(u32, .SB_SDLAS).* == 1) 32 else 1;

        var offset: u32 = 0;
        var bytes_transfered: u32 = 0;

        // At the end of transfer:
        //   The SB_SDSTAW register value is incremented.
        defer self.hw_register(u32, .SB_SDSTAW).* += offset;
        //   The SB_SDDIV register the number of times that the Sort-DMA operation read the Start Link Address.
        var sb_sddiv: u32 = 0;
        defer self.hw_register(u32, .SB_SDDIV).* = sb_sddiv;

        while (true) {
            const start_link_address: u32 = self.cpu.read_physical(T, @intCast(start_link_address_table + offset));
            sb_sddiv += 1;

            dc_log.debug("  [{d}] {X}", .{ offset, start_link_address });

            var link_address = start_link_address;
            while (true) {
                if (link_address == 1) break; // End of List
                if (link_address == 2) return bytes_transfered; // End of DMA

                link_address *= offset_factor;
                link_address += start_link_base_address;

                const parameter_control_word: HollyModule.ParameterControlWord = @bitCast(self.cpu.read_physical(u32, link_address));
                // TODO: We should search for the first GlobalParameter to get the data_size and next_address, not assume this is the first one.
                std.debug.assert(parameter_control_word.parameter_type == .PolygonOrModifierVolume or parameter_control_word.parameter_type == .SpriteList);

                const current_data_size = self.cpu.read_physical(u32, link_address + 0x18);
                const next_link_address = self.cpu.read_physical(u32, link_address + 0x1C);

                dc_log.debug("   - Size: {d}, Next: {X}", .{ current_data_size, next_link_address });

                const block_count = if (current_data_size == 0) 0x100 else current_data_size;
                const bytes = block_count * 8 * 4;
                var src: [*]u32 = @alignCast(@ptrCast(self._get_memory(link_address)));
                self.gpu.write_ta_fifo_polygon_path(src[0 .. 8 * block_count]);
                bytes_transfered += bytes;

                link_address = next_link_address;
            }

            offset += @sizeOf(T);
        }
    }

    pub fn end_sort_dma(self: *@This()) void {
        self.hw_register(u32, .SB_SDST).* = 0;
    }

    pub fn serialize(self: *@This(), writer: anytype) !usize {
        var bytes: usize = 0;

        bytes += try self.cpu.serialize(writer);
        bytes += try self.gpu.serialize(writer);
        bytes += try self.aica.serialize(writer);
        bytes += try self.maple.serialize(writer);
        bytes += try self.gdrom.serialize(writer);
        bytes += try self.flash.serialize(writer);
        bytes += try writer.write(std.mem.sliceAsBytes(self.ram));
        bytes += try writer.write(std.mem.sliceAsBytes(self.vram));
        bytes += try writer.write(std.mem.sliceAsBytes(self.aram));
        bytes += try writer.write(std.mem.sliceAsBytes(self.hardware_registers));

        bytes += try writer.write(std.mem.asBytes(&self.scheduled_events.count()));
        if (self.scheduled_events.count() > 0)
            bytes += try writer.write(std.mem.sliceAsBytes(self.scheduled_events.items[0..self.scheduled_events.count()]));

        bytes += try writer.write(std.mem.asBytes(&self._global_cycles));
        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: anytype) !usize {
        var bytes: usize = 0;

        bytes += try self.cpu.deserialize(reader);
        bytes += try self.gpu.deserialize(reader);
        bytes += try self.aica.deserialize(reader);
        bytes += try self.maple.deserialize(reader);
        bytes += try self.gdrom.deserialize(reader);
        bytes += try self.flash.deserialize(reader);
        bytes += try reader.read(std.mem.sliceAsBytes(self.ram));
        bytes += try reader.read(std.mem.sliceAsBytes(self.vram));
        bytes += try reader.read(std.mem.sliceAsBytes(self.aram));
        bytes += try reader.read(std.mem.sliceAsBytes(self.hardware_registers));

        while (self.scheduled_events.count() > 0)
            _ = self.scheduled_events.remove();
        var event_count: usize = 0;
        bytes += try reader.read(std.mem.asBytes(&event_count));
        for (0..event_count) |_| {
            var event: ScheduledEvent = undefined;
            bytes += try reader.read(std.mem.asBytes(&event));
            try self.scheduled_events.add(event);
        }

        bytes += try reader.read(std.mem.asBytes(&self._global_cycles));

        self.gpu.finalize_deserialization();

        return bytes;
    }
};

test "boot" {
    var dc = try Dreamcast.create(std.testing.allocator);
    defer {
        dc.deinit();
        std.testing.allocator.destroy(dc);
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
    try std.testing.expect(try dc.cpu.read(u32, dc.cpu.R(3).* + (9 << 2)) == dc.cpu.R(0).*);
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
    try std.testing.expect(dc.cpu.R(0).* == try dc.cpu.read(u32, 0xFF000000 + (4 << 2)));
    _ = try dc.tick(1); // mov 0x9,R1
    try std.testing.expect(dc.cpu.R(1).* == 0x9);
    _ = try dc.tick(1); // shll8 R1
    try std.testing.expect(dc.cpu.R(1).* == 0x9 << 8);
    _ = try dc.tick(1); // add 0x29,R1
    try std.testing.expect(dc.cpu.R(1).* == (0x9 << 8) + 0x29);
    _ = try dc.tick(1); // mov.l R1, @(7, R3) - Write 0x929 to CCR @ 0xFF00001C
    try std.testing.expect(dc.cpu.R(1).* == try dc.cpu.read(u32, 0xFF000000 + (0x7 << 2)));
    _ = try dc.tick(1); // shar R3
    try std.testing.expect(dc.cpu.R(3).* == 0xFF800000);
    try std.testing.expect(!dc.cpu.sr.t);
    _ = try dc.tick(1); // mov 0x01, R0
    try std.testing.expect(dc.cpu.R(0).* == 0x01);
    _ = try dc.tick(1); // mov.w R0, @(2, R3) - Write 0x01 to BCR2 @ 0xFF800004
    try std.testing.expect(dc.cpu.R(0).* == try dc.cpu.read(u16, 0xFF800000 + (0x2 << 1)));
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
    try std.testing.expect(dc.cpu.R(0).* == try dc.cpu.read(u32, 0xFF800000 + (0x3 << 2)));
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
    try std.testing.expect(try dc.cpu.read(u32, dc.cpu.R(5).* + (0x2 << 2)) == dc.cpu.R(0).*);

    _ = try dc.tick(1); // mov.l R0, @(0, R3) - Write 0xA3020008 to BRC1 @ 0xFF800000
    try std.testing.expect(try dc.cpu.read(u32, 0xFF800000) == 0xA3020008);
    _ = try dc.tick(1); // mov.l @(4,R5),R0
    try std.testing.expect(try dc.cpu.read(u32, dc.cpu.R(5).* + (0x4 << 2)) == dc.cpu.R(0).*);
    _ = try dc.tick(1); // mov.l R0, @(2, R3) - Write 0x01110111 to WCR1 @ 0xFF800008
    try std.testing.expect(try dc.cpu.read(u32, 0xFF800008) == 0x01110111);
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
    try std.testing.expect(try dc.cpu.read(u16, 0xA05F7480) == 0x400); // SB_G1RRC

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
    var dc = try Dreamcast.create(std.testing.allocator);
    defer {
        dc.deinit();
        std.testing.allocator.destroy(dc);
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
    var dc = try Dreamcast.create(std.testing.allocator);
    defer {
        dc.deinit();
        std.testing.allocator.destroy(dc);
    }

    try dc.skip_bios(true);

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
    var dc = try Dreamcast.create(std.testing.allocator);
    defer {
        dc.deinit();
        std.testing.allocator.destroy(dc);
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
