const std = @import("std");

const SH4 = @import("sh4.zig").SH4;

const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;

const termcolor = @import("termcolor.zig");

// Holly Video Chip

// Tile Accelerator and PowerVR2 core

const TileAccelerator = struct {};

pub const SPG_STATUS = packed struct(u32) {
    scanline: u10 = 0,
    fieldnum: u1 = 0,
    blank: u1 = 0,
    hblank: u1 = 0,
    vsync: u1 = 0,

    _: u18 = 0,
};

const HollyRegister = enum(u32) {
    ID = 0x005F8000,
    REVISION = 0x005F8004,
    SOFTRESET = 0x005F8008,

    STARTRENDER = 0x005F8014,
    TEST_SELECT = 0x005F8018,

    PARAM_BASE = 0x005F8020,

    REGION_BASE = 0x005F802C,
    SPAN_SOFT_CFG = 0x005F8030,

    VO_BORDER_COL = 0x005F8040,
    FB_R_CTRL = 0x005F8044,
    FB_W_CTRL = 0x005F8048,
    FB_LINESTRIDE = 0x005F804C,
    FB_R_SOF1 = 0x005F8050,
    FB_R_SOF2 = 0x005F8054,

    FB_R_SIZE = 0x005F805C,
    FB_W_SOF1 = 0x005F8060,
    FB_W_SOF2 = 0x005F8064,
    FB_X_CLIP = 0x005F8068,
    FB_Y_CLIP = 0x005F806C,

    FPU_SHAD_SCALE = 0x005F8074,
    FPU_CULL_VAL = 0x005F8078,
    FPU_PARAM_CFG = 0x005F807C,
    HALF_OFFSET = 0x005F8080,
    FPU_PREP_VAL = 0x005F8084,
    ISP_BACKGND_D = 0x005F8088,
    ISP_BACKGND_T = 0x005F808C,

    ISP_FEED_CFG = 0x005F8098,

    SDRAM_REFRESH = 0x005F80A0,
    SDRAM_ARB_CFG = 0x005F80A4,
    SDRAM_CFG = 0x005F80A8,

    FOG_COL_RAM = 0x005F80B0,
    FOG_COL_VERT = 0x005F80B4,
    FOG_DENSITY = 0x005F80B8,
    FOG_CLAMP_MAX = 0x005F80BC,
    FOG_CLAMP_MIN = 0x005F80C0,

    SPG_TRIGGER_POS = 0x005F80C4,
    SPG_HBLANK_INT = 0x005F80C8,
    SPG_VBLANK_INT = 0x005F80CC,
    SPG_CONTROL = 0x005F80D0,
    SPG_HBLANK = 0x005F80D4,
    SPG_LOAD = 0x005F80D8,
    SPG_VBLANK = 0x005F80DC,
    SPG_WIDTH = 0x005F80E0,
    TEXT_CONTROL = 0x005F80E4,
    VO_CONTROL = 0x005F80E8,
    VO_STARTX = 0x005F80EC,
    VO_STARTY = 0x005F80F0,
    SCALER_CTL = 0x005F80F4,

    PAL_RAM_CTRL = 0x005F8108,
    SPG_STATUS = 0x005F810C,
    FB_BURSTCTRL = 0x005F8110,
    FB_C_SOF = 0x005F8114,
    Y_COEFF = 0x005F8118,
    PT_ALPHA_REF = 0x005F811C,

    TA_OL_BASE = 0x005F8124,
    TA_ISP_BASE = 0x005F8128,
    TA_OL_LIMIT = 0x005F812C,
    TA_ISP_LIMIT = 0x005F8130,
    TA_NEXT_OPB = 0x005F8134,
    TA_ITP_CURRENT = 0x005F8138,
    TA_GLOB_TILE_CLIP = 0x005F813C,
    TA_ALLOC_CTRL = 0x005F8140,
    TA_LIST_INIT = 0x005F8144,
    TA_YUV_TEX_BASE = 0x005F8148,
    TA_YUV_TEX_CTRL = 0x005F814C,
    TA_YUV_TEX_CNT = 0x005F8150,

    TA_LIST_CONT = 0x005F8160,
    TA_NEXT_OPB_INIT = 0x005F8164,

    FOG_TABLE_START = 0x005F8200,
    TA_OL_POINTERS_START = 0x005F8600,
    PALETTE_RAM_START = 0x005F9000,
};

const HollyRegisterStart: u32 = 0x005F8000;

pub const Holly = struct {
    vram: []u8 = undefined,
    registers: []u8 = undefined,

    _allocator: std.mem.Allocator = undefined,

    pub fn init(self: *@This(), allocator: std.mem.Allocator) !void {
        self._allocator = allocator;

        self.vram = try self._allocator.alloc(u8, 8 * 1024 * 1024);
        self.registers = try self._allocator.alloc(u8, 0x2000);
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.free(self.registers);
        self._allocator.free(self.vram);
    }

    pub fn reset(self: *@This()) void {
        self._get_register(u32, .ID).* = 0x17FD11DB;
        self._get_register(u32, .SPG_STATUS).* = 0x00000000;
        self._get_register(u32, .SPAN_SOFT_CFG).* = 0x00000101;
        self._get_register(u32, .FPU_PARAM_CFG).* = 0x0007DF77;
        self._get_register(u32, .SDRAM_REFRESH).* = 0x00000020;
        self._get_register(u32, .SDRAM_CFG).* = 0x15D1C951;
        self._get_register(u32, .FB_BURSTCTRL).* = 0x00093F39;

        self._get_register(u32, .SPG_HBLANK).* = 0x007E0345;
    }

    pub fn update(self: *@This(), cpu: *SH4, cycles: u32) void {
        // TODO!
        const static = struct {
            var _tmp_cycles: u64 = 0;
        };
        static._tmp_cycles += cycles;

        // FIXME: Made up numbers for testing
        const cycles_per_line = 10 * (self._get_register(u32, .SPG_HBLANK).* & 0xFFF);
        if (static._tmp_cycles >= cycles_per_line) {
            static._tmp_cycles -= cycles_per_line;

            self._get_register(SPG_STATUS, .SPG_STATUS).*.scanline +%= 1;

            if (self._get_register(SPG_STATUS, .SPG_STATUS).*.scanline == 480) {
                self._get_register(SPG_STATUS, .SPG_STATUS).*.vsync = 1;
                cpu.raise_normal_interrupt(.{ .VBlankIn = 1 });
            }

            if (self._get_register(SPG_STATUS, .SPG_STATUS).*.scanline == 500) {
                self._get_register(SPG_STATUS, .SPG_STATUS).*.scanline = 0;
                self._get_register(SPG_STATUS, .SPG_STATUS).*.vsync = 0;
                cpu.raise_normal_interrupt(.{ .VBlankOut = 1 });
            }
        }
    }

    pub fn write_register(self: *@This(), addr: u32, v: u32) void {
        switch (addr) {
            @intFromEnum(HollyRegister.SOFTRESET) => {
                std.debug.print("[Holly] TODO SOFTRESET: {X:0>8}\n", .{v});
            },
            @intFromEnum(HollyRegister.STARTRENDER) => {
                std.debug.print("[Holly] TODO STARTRENDER: {X:0>8}\n", .{v});
            },
            @intFromEnum(HollyRegister.TA_LIST_INIT) => {
                std.debug.print("[Holly] TODO TA_LIST_INIT: {X:0>8}\n", .{v});
            },
            @intFromEnum(HollyRegister.TA_LIST_CONT) => {
                std.debug.print("[Holly] TODO TA_LIST_CONT: {X:0>8}\n", .{v});
            },
            @intFromEnum(HollyRegister.SPG_CONTROL), @intFromEnum(HollyRegister.SPG_LOAD) => {
                std.debug.print("[Holly] TODO SPG_CONTROL/SPG_LOAD: {X:0>8}\n", .{v});
            },
            else => {},
        }
        self._get_register_from_addr(u32, addr).* = v;
    }

    pub fn _get_register(self: *@This(), comptime T: type, r: HollyRegister) *T {
        return @as(*T, @alignCast(@ptrCast(&self.registers[(@intFromEnum(r) & 0x1FFFFFFF) - HollyRegisterStart])));
    }

    pub fn _get_register_from_addr(self: *@This(), comptime T: type, addr: u32) *T {
        return @as(*T, @alignCast(@ptrCast(&self.registers[(addr & 0x1FFFFFFF) - HollyRegisterStart])));
    }

    pub fn _get_vram(self: *@This(), addr: u32) *u8 {
        // VRAM - 8MB, Mirrored at 0x06000000
        const local_addr = addr - (if (addr >= 0x06000000) @as(u32, 0x06000000) else 0x04000000);
        if (local_addr < 0x0080_0000) { // 64-bit access area
            return &self.vram[local_addr];
        } else if (local_addr < 0x0100_0000) { // Unused
            std.debug.print(termcolor.red("  Out of bounds write to Area 1 (VRAM): {X:0>8}\n"), .{addr});
            return &self.vram[0];
        } else if (local_addr < 0x0180_0000) { // 32-bit access area
            return &self.vram[local_addr - 0x0100_0000];
        } else { // Unused
            std.debug.print(termcolor.red("  Out of bounds write to Area 1 (VRAM): {X:0>8}\n"), .{addr});
            return &self.vram[0];
        }
    }
};
