const std = @import("std");

const SH4 = @import("sh4.zig").SH4;

const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;

const termcolor = @import("termcolor.zig");

// Holly Video Chip

// Tile Accelerator and PowerVR2 core

const TileAccelerator = struct {};

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

    _,
};

const HollyRegisterStart: u32 = 0x005F8000;

pub const SOFT_RESET = packed struct(u32) {
    TASoftReset: u1 = 0,
    PipelineSoftReset: u1 = 0,
    SDRAMInterfaceSoftReset: u1 = 0,
    _: u29 = 0,
};

pub const SPG_STATUS = packed struct(u32) {
    scanline: u10 = 0,
    fieldnum: u1 = 0,
    blank: u1 = 0,
    hblank: u1 = 0,
    vsync: u1 = 0,

    _: u18 = 0,
};

pub const SPG_HBLANK_INT = packed struct(u32) {
    line_comp_val: u10 = 0,
    _r0: u2 = 0,
    hblank_int_mode: u2 = 0,
    _r1: u2 = 0,
    hblank_in_interrupt: u10 = 0x31D,
    _r2: u6 = 0,
};

pub const SPG_VBLANK_INT = packed struct(u32) {
    vblank_in_interrupt_line_number: u10 = 0x104,
    _r0: u6 = 0,
    vblank_out_interrupt_line_number: u10 = 0x150,
    _r1: u6 = 0,
};

pub const SPG_CONTROL = packed struct(u32) {
    mhsync_pol: u1 = 0,
    mvsync_pol: u1 = 0,
    mcsync_pol: u1 = 0,
    spg_lock: u1 = 0,
    interlace: u1 = 0,
    force_field2: u1 = 0,
    NTSC: u1 = 0,
    PAL: u1 = 0,
    sync_direction: u1 = 0,
    csync_on_h: u1 = 0,
    _: u21 = 0,
};

pub const SPG_LOAD = packed struct(u32) {
    hcount: u10 = 0x359,
    _r0: u6 = 0,
    vcount: u10 = 0x106,
    _r1: u6 = 0,
};

pub const SPG_HBLANK = packed struct(u32) {
    hbstart: u10 = 0x345,
    _r0: u6 = 0,
    hbend: u10 = 0x07E,
    _r1: u6 = 0,
};

pub const SPG_VBLANK = packed struct(u32) {
    vbstart: u10 = 0x104,
    _r0: u6 = 0,
    vbend: u10 = 0x150,
    _r1: u6 = 0,
};

pub const ParameterType = enum(u3) {
    // Control Parameter
    EndOfList = 0,
    UserTileClip = 1,
    ObjectListSet = 2,
    // Global Parameter
    PolygonOrModifierVolume = 4,
    SpriteList = 5,
    // Vertex Parameter
    VertexParameter = 7,
};

pub const ListType = enum(u3) {
    Opaque = 0,
    OpaqueModifierVolume = 1,
    Translucent = 2,
    TranslucentModifierVolume = 3,
    PunchThrough = 4,
};

pub const ParameterControl = packed struct(u8) {
    list_type: ListType = .Opaque,
    _: u1 = 0,
    end_of_strip: u1 = 0,
    parameter_type: ParameterType = .EndOfList,
};

pub const GroupControl = packed struct(u8) {
    user_clip: u2 = 0,
    strip_len: u2 = 0,
    _: u3 = 0,
    en: u1 = 0,
};

pub const ColorType = enum(u2) {
    PackedColor = 0,
    FloatingColor = 1,
    IntensityMode1 = 2,
    IntensityMode2 = 3,
};

pub const ObjControl = packed struct(u16) {
    uv_16bit: u1 = 0,
    gouraud: u1 = 0,
    offset: u1 = 0,
    texture: u1 = 0,
    col_type: ColorType = .PackedColor,
    volume: u1 = 0,
    shadow: u1 = 0,
    _: u8 = 0,
};

pub const ParameterControlWord = packed struct(u32) {
    obj_control: ObjControl = .{},
    group_control: GroupControl = .{},
    parameter_control: ParameterControl = .{},
};

// Control Parameter Formats

const UserTileClip = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    _ignored: u96,
    user_clip_x_min: u32,
    user_clip_y_min: u32,
    user_clip_x_max: u32,
    user_clip_y_max: u32,
};

// Global Parameter Formats

// Packed/Floating Color
const PolygonType0 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: u32,
    tsp_instruction: u32,
    texture_control: u32,
    _ignored: u64,
    data_size: u32,
    next_address: u32,
};

// Packed Color, with Two Volumes
const PolygonType3 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: u32,
    tsp_instruction_0: u32,
    texture_control_0: u32,
    tsp_instruction_1: u32,
    texture_control_1: u32,
    data_size: u32,
    next_address: u32,
};

pub const Holly = struct {
    vram: []u8 = undefined,
    registers: []u8 = undefined,

    _allocator: std.mem.Allocator = undefined,

    _ta_command_buffer: [16]u32 align(8) = .{0} ** 16,
    _ta_command_buffer_index: u32 = 0,
    _ta_list_type: ?ListType = null,

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
        self._get_register(u32, .REVISION).* = 0x0011;
        self._get_register(SPG_STATUS, .SPG_STATUS).* = .{};
        self._get_register(u32, .SPAN_SOFT_CFG).* = 0x00000101;
        self._get_register(u32, .FPU_PARAM_CFG).* = 0x0007DF77;
        self._get_register(u32, .SDRAM_REFRESH).* = 0x00000020;
        self._get_register(u32, .SDRAM_CFG).* = 0x15D1C951;
        self._get_register(u32, .FB_BURSTCTRL).* = 0x00093F39;

        self._get_register(SPG_LOAD, .SPG_LOAD).* = .{};
        self._get_register(SPG_HBLANK, .SPG_HBLANK).* = .{};
        self._get_register(SPG_VBLANK, .SPG_VBLANK).* = .{};
        self._get_register(SPG_VBLANK_INT, .SPG_VBLANK_INT).* = .{};
    }

    pub fn update(self: *@This(), cpu: *SH4, cycles: u32) void {
        // TODO!
        const static = struct {
            var _tmp_cycles: u64 = 0;
            var _pixel: u32 = 0;
        };
        static._tmp_cycles += cycles;

        const spg_hblank = self._get_register(SPG_HBLANK, .SPG_HBLANK).*;
        const spg_hblank_int = self._get_register(SPG_HBLANK_INT, .SPG_HBLANK_INT).*;
        const spg_load = self._get_register(SPG_LOAD, .SPG_LOAD).*;
        const cycles_per_pixel = 7; // FIXME: Approximation. ~200/27.
        if (static._tmp_cycles >= cycles_per_pixel) {
            static._tmp_cycles -= cycles_per_pixel;
            static._pixel += 1;

            const spg_status = self._get_register(SPG_STATUS, .SPG_STATUS);

            if (static._pixel == spg_hblank.hbstart) {
                spg_status.*.hblank = 1;
            }
            if (static._pixel == spg_hblank.hbend) {
                spg_status.*.hblank = 0;
            }
            if (static._pixel == spg_hblank_int.hblank_in_interrupt) {
                switch (spg_hblank_int.hblank_int_mode) {
                    0 => {
                        // Output when the display line is the value indicated by line_comp_val.
                        if (spg_status.*.scanline == spg_hblank_int.line_comp_val)
                            cpu.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    1 => {
                        // Output every line_comp_val lines.
                        if (spg_status.*.scanline % spg_hblank_int.line_comp_val == 0) // FIXME: Really?
                            cpu.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    2 => {
                        // Output every line.
                        cpu.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    else => {},
                }
            }

            if (static._pixel >= spg_load.hcount) {
                static._pixel = 0;

                const spg_vblank = self._get_register(SPG_VBLANK, .SPG_VBLANK).*;
                const spg_vblank_int = self._get_register(SPG_VBLANK_INT, .SPG_VBLANK_INT).*;

                self._get_register(SPG_STATUS, .SPG_STATUS).*.scanline +%= 1;

                if (spg_status.*.scanline == spg_vblank_int.vblank_in_interrupt_line_number) {
                    cpu.raise_normal_interrupt(.{ .VBlankIn = 1 });
                }
                if (spg_status.*.scanline == spg_vblank_int.vblank_out_interrupt_line_number) {
                    cpu.raise_normal_interrupt(.{ .VBlankOut = 1 });
                }
                if (spg_status.*.scanline == spg_vblank.vbstart) {
                    spg_status.*.vsync = 1;
                }
                if (spg_status.*.scanline == spg_vblank.vbend) {
                    spg_status.*.vsync = 0;
                }
                if (spg_status.*.scanline == spg_load.vcount) {
                    spg_status.*.scanline = 0;
                }
            }
        }
    }

    pub fn write_register(self: *@This(), addr: u32, v: u32) void {
        switch (addr) {
            @intFromEnum(HollyRegister.SOFTRESET) => {
                std.debug.print("[Holly] TODO SOFTRESET: {X:0>8}\n", .{v});
                const sr: SOFT_RESET = @bitCast(v);
                if (sr.TASoftReset == 1) {
                    std.debug.print("[Holly]   TODO: Tile Accelerator Soft Reset\n", .{});
                }
                if (sr.PipelineSoftReset == 1) {
                    std.debug.print("[Holly]   TODO: Pipeine Soft Reset\n", .{});
                }
                if (sr.SDRAMInterfaceSoftReset == 1) {
                    std.debug.print("[Holly]   TODO: SDRAM Interface Soft Reset\n", .{});
                }
                return;
            },
            @intFromEnum(HollyRegister.STARTRENDER) => {
                std.debug.print(termcolor.green("[Holly] TODO STARTRENDER: {X:0>8}\n"), .{v});
            },
            @intFromEnum(HollyRegister.TA_LIST_INIT) => {
                if (v == 0x80000000) {
                    std.debug.print("[Holly] TODO TA_LIST_INIT: {X:0>8}\n", .{v});
                    if (self._get_register(u32, .TA_LIST_CONT).* == 0x80000000)
                        self._get_register(u32, .TA_NEXT_OPB).* = self._get_register(u32, .TA_NEXT_OPB_INIT).*;
                    self._ta_list_type = null;
                    self._ta_command_buffer_index = 0;
                }
            },
            @intFromEnum(HollyRegister.TA_LIST_CONT) => {
                std.debug.print("[Holly] TODO TA_LIST_CONT: {X:0>8}\n", .{v});
            },
            @intFromEnum(HollyRegister.SPG_CONTROL), @intFromEnum(HollyRegister.SPG_LOAD) => {
                std.debug.print("[Holly] TODO SPG_CONTROL/SPG_LOAD: {X:0>8}\n", .{v});
            },
            else => {
                std.debug.print("[Holly] Write to Register: @{X:0>8} {s} = {X:0>8}\n", .{ addr, std.enums.tagName(HollyRegister, @as(HollyRegister, @enumFromInt(addr))) orelse "Unknown", v });
            },
        }
        self._get_register_from_addr(u32, addr).* = v;
    }

    // Write to the Tile Accelerator
    pub fn write_ta(self: *@This(), cpu: *SH4, addr: u32, v: u32) void {
        std.debug.print("  TA Write: {X:0>8} = {X:0>8}\n", .{ addr, v });

        self._ta_command_buffer[self._ta_command_buffer_index] = v;
        self._ta_command_buffer_index += 1;

        if (self._ta_command_buffer_index < 8) return; // All commands are at least 8 bytes long

        const parameter_control_word: ParameterControlWord = @bitCast(self._ta_command_buffer[0]);
        std.debug.print("    Parameter Control Word: {any}\n", .{parameter_control_word});
        switch (parameter_control_word.parameter_control.parameter_type) {
            .EndOfList => {
                if (self._ta_list_type != null) { // Apprently this happpens?... Why would a game do this?
                    switch (self._ta_list_type.?) {
                        .Opaque => {
                            cpu.raise_normal_interrupt(.{ .EoT_OpaqueList = 1 });
                        },
                        .OpaqueModifierVolume => {
                            cpu.raise_normal_interrupt(.{ .EoT_OpaqueModifierVolumeList = 1 });
                        },
                        .Translucent => {
                            cpu.raise_normal_interrupt(.{ .EoT_TranslucentList = 1 });
                        },
                        .TranslucentModifierVolume => {
                            cpu.raise_normal_interrupt(.{ .EoT_TranslucentModifierVolumeList = 1 });
                        },
                        .PunchThrough => {
                            cpu.raise_normal_interrupt(.{ .EoD_PunchThroughList = 1 });
                        },
                    }
                }
                self._ta_list_type = null;
            },
            .UserTileClip => {},
            .ObjectListSet => {
                if (self._ta_list_type == null) {
                    self._ta_list_type = parameter_control_word.parameter_control.list_type;
                }
            },
            .PolygonOrModifierVolume => {
                if (self._ta_list_type == null) {
                    self._ta_list_type = parameter_control_word.parameter_control.list_type;
                }
                std.debug.assert(self._ta_list_type == parameter_control_word.parameter_control.list_type);
                if (self._ta_list_type == .OpaqueModifierVolume or self._ta_list_type == .TranslucentModifierVolume) {
                    @panic("TODO");
                } else {
                    // "With Two Volumes"
                    if (parameter_control_word.obj_control.volume == 0) {
                        switch (parameter_control_word.obj_control.col_type) {
                            .PackedColor, .FloatingColor => {
                                const polygon_type_3 = @as(*PolygonType3, @ptrCast(&self._ta_command_buffer)).*;
                                std.debug.print("    Polygon Type 3: {any}\n", .{polygon_type_3});
                            },
                            else => {
                                @panic("TODO");
                            },
                        }
                    } else {
                        switch (parameter_control_word.obj_control.col_type) {
                            .PackedColor, .FloatingColor => {
                                const polygon_type_0 = @as(*PolygonType0, @ptrCast(&self._ta_command_buffer)).*;
                                std.debug.print("    Polygon Type 0: {any}\n", .{polygon_type_0});
                            },
                            else => {
                                @panic("TODO");
                            },
                        }
                    }
                }
            },
            .SpriteList => {
                if (self._ta_list_type == null) {
                    self._ta_list_type = parameter_control_word.parameter_control.list_type;
                }
            },
            .VertexParameter => {},
        }
        // Command has been handled, reset buffer.
        self._ta_command_buffer_index = 0;
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
            std.debug.print(termcolor.red("  Out of bounds access to Area 1 (VRAM): {X:0>8}\n"), .{addr});
            @panic("Out of bounds access to Area 1 (VRAM)");
            //return &self.vram[0];
        } else if (local_addr < 0x0180_0000) { // 32-bit access area
            return &self.vram[local_addr - 0x0100_0000];
        } else { // Unused
            std.debug.print(termcolor.red("  Out of bounds access to Area 1 (VRAM): {X:0>8}\n"), .{addr});
            @panic("Out of bounds access to Area 1 (VRAM)");
            //return &self.vram[0];
        }
    }
};
