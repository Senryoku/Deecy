const std = @import("std");
const termcolor = @import("termcolor");

const holly_log = std.log.scoped(.holly);

const DreamcastModule = @import("dreamcast.zig");
const Dreamcast = DreamcastModule.Dreamcast;

pub const Colors = @import("colors.zig");
const PackedColor = Colors.PackedColor;
const YUV422 = Colors.YUV422;
const fARGB = Colors.fARGB;
const fRGBA = Colors.fRGBA;

pub const HollyRegister = enum(u32) {
    ID = 0x005F8000,
    REVISION = 0x005F8004,
    SOFTRESET = 0x005F8008,

    STARTRENDER = 0x005F8014,
    TEST_SELECT = 0x005F8018,

    /// Base address for ISP parameters
    PARAM_BASE = 0x005F8020,

    /// Base address for Region Array
    REGION_BASE = 0x005F802C,
    SPAN_SOFT_CFG = 0x005F8030,

    VO_BORDER_COL = 0x005F8040,
    FB_R_CTRL = 0x005F8044,
    FB_W_CTRL = 0x005F8048,
    FB_W_LINESTRIDE = 0x005F804C,
    /// Starting address, in 32-bit units, for reads from the field-1 frame buffer. (default = 0x000000)
    FB_R_SOF1 = 0x005F8050,
    /// Starting address, in 32-bit units, for reads from the field-2 frame buffer. (default = 0x000000)
    FB_R_SOF2 = 0x005F8054,

    FB_R_SIZE = 0x005F805C,
    /// Specifies, in 32-bit units, the starting address for writes to the field-1 or strip-1 frame buffer. (default = 0x000000)
    /// In the texture map, 0x00000000 to 0x0FFFFFFC is a 32-bit access area and 0x10000000 to 0x1FFFFFFC is a 64-bit access area.
    FB_W_SOF1 = 0x005F8060,
    /// Specifies, in 32-bit units, the starting address for writes to the field-1 or strip-2 frame buffer. (default = 0x000000)
    /// In the texture map, 0x00000000 to 0x0FFFFFFC is a 32-bit access area (frame/strip buffer) and 0x10000000 to 0x1FFFFFFC is a 64-bit access area.
    FB_W_SOF2 = 0x005F8064,
    FB_X_CLIP = 0x005F8068,
    FB_Y_CLIP = 0x005F806C,

    FPU_SHAD_SCALE = 0x005F8074,
    FPU_CULL_VAL = 0x005F8078,
    FPU_PARAM_CFG = 0x005F807C,
    HALF_OFFSET = 0x005F8080,
    FPU_PERP_VAL = 0x005F8084,
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
    /// Specify the starting address, in 32-bit units, for the frame that is currently being sent to the DAC.
    FB_C_SOF = 0x005F8114,
    Y_COEFF = 0x005F8118,
    PT_ALPHA_REF = 0x005F811C,

    /// Specifies (in 8 x 32-bit units) the starting address for storing Object Lists as a relative address, assuming the start of texture memory (32-bit area) as "0." (default = 0x0 0000)
    TA_OL_BASE = 0x005F8124,
    /// Specifies (in 32-bit units) the starting address for storing the ISP/TSP Parameters as a relative address, assuming the start of texture memory (32-bit area) as "0." (default = 0x00 0000)
    TA_ISP_BASE = 0x005F8128,
    /// Specifies (in 8 x 32-bit units) the limit address for storing Object Lists as a relative address, assuming the start of texture memory (32-bit area) as "0." (default = 0x0 0000)
    TA_OL_LIMIT = 0x005F812C,
    /// Specifies (in 32-bit units) the limit address for storing ISP/TSP Parameters as a relative address, assuming the start of texture memory (32-bit area) as "0." (default = 0x0 0000)
    TA_ISP_LIMIT = 0x005F8130,
    /// Indicates (in 8 x 32-bit units) the starting address for the Object Pointer Block that the TA will use next as a relative address, assuming the start of texture memory (32-bit area) as "0."
    TA_NEXT_OPB = 0x005F8134,
    /// Indicates (in 8 x 32-bit units) the starting address for the Object Pointer Block that the TA will use next as a relative address, assuming the start of texture memory (32-bit area) as "0."
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

pub const HollyRegisterStart: u32 = 0x005F8000;

const VideoModeDefaultRegisters = struct { spg_load: SPG_LOAD, spg_hblank: SPG_HBLANK, spg_vblank: SPG_VBLANK, spg_width: SPG_WIDTH, spg_control: SPG_CONTROL, vo_startx: VO_STARTX, vo_starty: VO_STARTY, vo_control: VO_CONTROL };
pub const VideoModes = struct {
    pub const NTSC = VideoModeDefaultRegisters{
        .spg_load = @bitCast(@as(u32, 0x01060359)),
        .spg_hblank = @bitCast(@as(u32, 0x007E0345)),
        .spg_vblank = @bitCast(@as(u32, 0x00120102)),
        .spg_width = @bitCast(@as(u32, 0x03F1933F)),
        .spg_control = @bitCast(@as(u32, 0x00000140)),
        .vo_startx = @bitCast(@as(u32, 0x000000A4)),
        .vo_starty = @bitCast(@as(u32, 0x00120011)),
        .vo_control = @bitCast(@as(u32, 0x00160000)), // 240p: 0x00160100
    };
    pub const NTSCInterlace = VideoModeDefaultRegisters{
        .spg_load = @bitCast(@as(u32, 0x020C0359)),
        .spg_hblank = @bitCast(@as(u32, 0x007E0345)),
        .spg_vblank = @bitCast(@as(u32, 0x00240204)),
        .spg_width = @bitCast(@as(u32, 0x07D6C63F)),
        .spg_control = @bitCast(@as(u32, 0x00000150)),
        .vo_startx = @bitCast(@as(u32, 0x000000A4)),
        .vo_starty = @bitCast(@as(u32, 0x00120012)),
        .vo_control = @bitCast(@as(u32, 0x00160000)), // 240p: 0x00160100
    };
    pub const PAL = VideoModeDefaultRegisters{
        .spg_load = @bitCast(@as(u32, 0x0138035F)),
        .spg_hblank = @bitCast(@as(u32, 0x008D034B)),
        .spg_vblank = @bitCast(@as(u32, 0x002C026C)),
        .spg_width = @bitCast(@as(u32, 0x07F1F53F)),
        .spg_control = @bitCast(@as(u32, 0x00000180)),
        .vo_startx = @bitCast(@as(u32, 0x000000AE)),
        .vo_starty = @bitCast(@as(u32, 0x002E002E)),
        .vo_control = @bitCast(@as(u32, 0x00160000)), // 240p: 0x00160100
    };
    pub const PALInterlace = VideoModeDefaultRegisters{
        .spg_load = @bitCast(@as(u32, 0x0270035F)),
        .spg_hblank = @bitCast(@as(u32, 0x008D034B)),
        .spg_vblank = @bitCast(@as(u32, 0x002C026C)),
        .spg_width = @bitCast(@as(u32, 0x07D6A53F)),
        .spg_control = @bitCast(@as(u32, 0x00000190)),
        .vo_startx = @bitCast(@as(u32, 0x000000AE)),
        .vo_starty = @bitCast(@as(u32, 0x002E002D)),
        .vo_control = @bitCast(@as(u32, 0x00160000)), // 240p: 0x00160100
    };
    pub const VGA = VideoModeDefaultRegisters{
        .spg_load = @bitCast(@as(u32, 0x020C0359)),
        .spg_hblank = @bitCast(@as(u32, 0x007E0345)),
        .spg_vblank = @bitCast(@as(u32, 0x00280208)),
        .spg_width = @bitCast(@as(u32, 0x03F1933F)),
        .spg_control = @bitCast(@as(u32, 0x00000100)),
        .vo_startx = @bitCast(@as(u32, 0x000000A8)),
        .vo_starty = @bitCast(@as(u32, 0x00280028)),
        .vo_control = @bitCast(@as(u32, 0x00160000)),
    };
};

pub const SOFT_RESET = packed struct(u32) {
    TASoftReset: bool = false,
    PipelineSoftReset: bool = false,
    SDRAMInterfaceSoftReset: bool = false,
    _: u29 = 0,
};

pub const FOG_CLAMP = PackedColor;

pub const SPG_STATUS = packed struct(u32) {
    scanline: u10 = 0,
    fieldnum: u1 = 0,
    blank: bool = false,
    hblank: bool = false,
    vsync: bool = false,

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
    interlace: bool = false,
    force_field2: u1 = 0,
    NTSC: u1 = 1,
    PAL: u1 = 0,
    sync_direction: u1 = 0,
    csync_on_h: u1 = 0,
    _: u22 = 0,
};

pub const SPG_LOAD = packed struct(u32) {
    /// Specify "number of video clock cycles per line - 1" for the CRT. (default = 0x359)
    hcount: u10 = 0x359,
    _r0: u6 = 0,
    /// Specify "number of lines per field - 1" for the CRT; in interlace mode, specify "number of lines per field/2 - 1." (default = 0x106)
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

pub const SPG_WIDTH = packed struct(u32) {
    hswidth: u7 = 0x3F,
    _: u1 = 0,
    vswidth: u4 = 0x3,
    bpwidth: u10 = 0x319,
    eqwidth: u10 = 0x01F,
};

pub const ISP_BACKGND_T = packed struct(u32) {
    tag_offset: u3,
    tag_address: u21,
    skip: u3,
    shadow: u1,
    cache_bypass: u1,
    _: u3,
};

pub const ISP_FEED_CFG = packed struct(u32) {
    presort_mode: bool, // This field specifies the Translucent polygon sort mode. Only valid when the region header type bit (bit 21) in the FPU_PARAM_CFG register is "0".
    _r: u2,
    discard_mode: bool, // This field specifies whether to perform discard processing or not when processing Punch Through polygons and Translucent polygons.
    punchthrough_chunk_size: u10,
    cache_size_for_translucency: u10,
    _: u8,
};

pub const SCALER_CTL = packed struct(u32) {
    /// This field specifies the scale factor in the vertical direction. (default = 0x0400)
    /// This value consists of a 6-bit integer portion and a 10-bit decimal portion, and
    /// expands or reduces the screen in the vertical direction by "1/scale factor." When
    /// using flicker-free interlace mode type B, specify 0x0800.
    vertical_scale_factor: u16,
    /// This field specifies whether or not to use the horizontal direction 1/2 scaler.
    horizontal_scaling_enable: bool,
    /// This register specifies whether or not to use flicker-free interlace mode type B.
    interlace: bool,
    /// This register specifies the field that is to be stored in the frame buffer in flicker-free interlace mode type B.
    field_select: u1,
    _reserved: u13,

    pub fn get_x_scale_factor(self: @This()) f32 {
        return if (self.horizontal_scaling_enable) 0.5 else 1.0;
    }

    pub fn get_y_scale_factor(self: @This()) f32 {
        return @as(f32, @floatFromInt(self.vertical_scale_factor)) / 1024.0;
    }
};

pub const FB_W_CTRL = packed struct(u32) {
    fb_packmode: enum(u3) {
        KRGB0555 = 0, // Bit 15 is the value of fb_kval[7].
        RGB565 = 1,
        ARGB4444 = 2,
        ARGB1555 = 3, // The alpha value is determined by comparison with the value of fb_alpha_threshold.
        RGB888 = 4,
        KRGB0888 = 5, // K is the value of fk_kval
        ARGB8888 = 6,
        Reserved = 7,
    },
    /// Dithering enable
    fb_dither: bool,
    _0: u4,
    /// This field sets the K value for writing to the frame buffer. (default = 0x00)
    fb_kval: u8,
    /// This field sets the comparison value that is used to determine the alpha value when the data that is written to the frame buffer is ARGB1555 format data. (default = 0x00) When pixel alpha ≥ fb_alpha_threshold, a "1" is written in the alpha value.
    fb_alpha_threshold: u8,
    _1: u8,
};

pub const FB_CLIP = packed struct(u32) {
    min: u11 = 0,
    _0: u5 = 0,
    max: u11 = 0,
    _1: u5 = 0,
};

pub const FPU_SHAD_SCALE = packed struct(u32) {
    factor: u8,
    enable: bool,
    _: u23,

    pub fn get_factor(self: @This()) f32 {
        return if (self.enable) @as(f32, @floatFromInt(self.factor)) / 256.0 else 1.0;
    }
};

pub const FPU_PARAM_CFG = packed struct(u32) {
    const RegionHeaderType = enum(u1) {
        Type1 = 0,
        Type2 = 1,
        pub fn word_size(self: @This()) u32 {
            return switch (self) {
                .Type1 => 5,
                .Type2 => 6,
            };
        }
    };

    pointer_first_burst_size: u4,
    pointer_burst_size: u4,
    isp_parameter_burst_trigger_threshold: u6,
    tsp_parameter_burst_trigger_threshold: u6,
    _r: u1,
    /// 0: 5 x 32bit/Tile Type 1 (default)
    ///   The Translucent polygon sort mode is specified by the
    ///   ISP_FEED_CFG register.
    /// 1: 6 x 32bit/Tile Type 2
    ///   The Translucent polygon sort mode is specified by the
    ///   pre-sort bit within the Region Array data.
    region_header_type: RegionHeaderType,
    _: u10,
};

pub const HALF_OFFSET = packed struct(u32) {
    const SamplingPosition = enum(u1) {
        UpperLeft = 0,
        Center = 1,
    };

    fpu_pixel_sampling_position: SamplingPosition,
    tsp_pixel_sampling_position: SamplingPosition,
    tsp_texel_sampling_position: SamplingPosition,
    _: u29,
};

pub const FB_R_CTRL = packed struct(u32) {
    enable: bool,
    line_double: bool,
    /// Specifies the bit configuration of the pixel data that is read from the frame buffer. Named "fb_depth" in the documentation
    format: enum(u2) { RGB555 = 0, RGB565 = 1, RGB888 = 2, RGB0888_32bit = 3 },
    /// Specifies the value that is added to the lower end of 5-bit or 6-bit frame buffer color data in order to output 8 bits. (default = 0x0)
    concat: u3,
    _0: u1,
    chroma_threshold: u8,
    /// Specifies the size of the strip buffer in units of 32 lines (default = 0x00).
    stripsize: u6,
    /// Set this bit to "1" when using a strip buffer. (default = 0)
    strip_buf_en: bool,
    /// Specifies the clock that is output on the PCLK pin. 0: PCLK = VLCK/2 (default) For NTSC/PAL / 1: PCLK = VLCK For VGA
    vclk_div: u1,
    _1: u8,
};

pub const FB_R_SIZE = packed struct(u32) {
    /// Number of display pixels on each line - 1, in 32-bit units
    x_size: u10,
    /// Number of display lines - 1
    y_size: u10,
    /// Amount of data between each line, in 32-bit units
    modulus: u10,
    _: u2,
};

pub const TEXT_CONTROL = packed struct(u32) {
    /// This field specifies the U size of the stride texture. The U size is the stride value × 32.
    stride: u5,
    _r0: u3,
    /// This field specifies the position of the bank bit when accessing texture memory (default = 0x00). Normally, set 0x00
    bank_bit: u5 = 0,
    _r1: u3,
    index_endian_reg: enum(u1) { Little = 0, Big = 1 } = .Little,
    code_book_endian_reg: enum(u1) { Little = 0, Big = 1 } = .Little,
    _r2: u14,
};

pub const VO_CONTROL = packed struct(u32) {
    /// Polarity of HSYNC (0 = active low, 1 = active high)
    hsync_pool: u1,
    /// Polarity of VSYNC (0 = active low, 1 = active high)
    vsync_pool: u1,
    /// Polarity of BLANK (0 = active low, 1 = active high)
    blank_pool: u1,
    /// This field specifies whether to display the screen or not. 0: Display the screen, 1: Do not display the screen. (Display the border color.) (default)
    blank_video: u1,
    field_mode: enum(u4) {
        /// Use field flag from SPG. (default)
        SPG = 0,
        /// Use inverse of field flag from SPG.
        InverseSPG = 1,
        /// Field 1 fixed.
        Field1 = 2,
        /// Field 2 fixed.
        Field2 = 3,
        /// Field 1 when the active edges of HSYNC and VSYNC match.
        Field1Sync = 4,
        /// Field 2 when the active edges of HSYNC and VSYNC match.
        Field2Sync = 5,
        /// Field 1 when HSYNC becomes active in the middle of the VSYNC active edge.
        Field1Async = 6,
        /// Field 2 when HSYNC becomes active in the middle of the VSYNC active edge.
        Field2Async = 7,
        /// Inverted at the active edge of VSYNC
        VsyncInverted = 8,
        _, // Reserved
    },
    /// This field specifies whether to output the same pixel or not for two pixels in the horizontal direction. 0: not pixel double, 1: pixel double (default)
    pixel_double: bool,
    _r0: u7,
    /// This field specifies the delay for the PCLK signal to the DAC.
    pclk_delay: u6,
    _r1: u10,
};

pub const VO_STARTX = packed struct(u32) {
    horizontal_start_position: u10,
    _: u22,
};

pub const VO_STARTY = packed struct(u32) {
    vertical_start_position_on_field1: u10,
    _r0: u6,
    vertical_start_position_on_field2: u10,
    _r1: u6,
};

pub const TA_GLOB_TILE_CLIP = packed struct(u32) {
    /// This field specifies the Tile number in the X direction (0 to 39) for the lower right corner of the Global Tile Clip. (default = 0x00) Set [the number of Tiles in the X direction in the valid area] - 1. "40" (0x28) through "63" (0x3F) must not be specified.
    tile_x_num: u6,
    _0: u10,
    /// This field specifies the Tile number in the Y direction (0 to 14) for the lower right corner of the Global Tile Clip. (default = 0x0) Set [the number of Tiles in the Y direction in the valid area] - 1. "15" (0xF) must not be specified.
    tile_y_num: u6,
    _1: u10,

    pub fn pixel_size(self: @This()) struct { x: u32, y: u32 } {
        return .{ .x = 32 * @as(u32, self.tile_x_num + 1), .y = 32 * @as(u32, self.tile_y_num + 1) };
    }
};

/// This register must be set before the lists are initialized through the TA_LIST_INIT register.
pub const TA_ALLOC_CTRL = packed struct(u32) {
    /// These fields specify the Object Pointer Block unit size for each type of list (Opaque, etc.).
    /// Specify "No List" for a list that is not used in the screen. For the Pointer Burst Size value
    /// in the FPU_PARAM_CFG register, set a value that is less than or equal to the Object Pointer Block size specified here.
    const UnitSize = enum(u2) {
        NoList = 0,
        _8x32 = 1,
        _16x32 = 2,
        _32x32 = 3,

        pub fn byteSize(self: @This()) u32 {
            return switch (self) {
                .NoList => 0,
                ._8x32 => 8,
                ._16x32 => 16,
                ._32x32 => 32,
            };
        }
    };

    /// This field specifies the unit size of an Object Pointer Block for an Opaque list.
    O_OPB: UnitSize,
    _r0: u2,
    /// This field specifies the unit size of an Object Pointer Block for an Opaque Modifier Volume list.
    OM_OPB: UnitSize,
    _r1: u2,
    /// This field specifies the unit size of an Object Pointer Block for a Translucent list.
    T_OPB: UnitSize,
    _r2: u2,
    /// This field specifies the unit size of an Object Pointer Block for a Translucent Modifier Volume list.
    TM_OPB: UnitSize,
    _r3: u2,
    /// This field specifies the unit size for the Object Pointer Block of the Punch Through list.
    PT_OPB: UnitSize,
    _r4: u2,
    /// This field specifies the address direction when storing the next Object Pointer Block (OPB) in texture memory,
    /// in the event that the specified Object Pointer Block size has been exceeded.
    OPB_Mode: enum(u1) {
        Increasing = 0,
        Decreasing = 1,
    },
    _r5: u11,
};

pub const TA_YUV_TEX_CTRL = packed struct(u32) {
    /// Actual size in pixels is 16 * (u_size + 1)
    u_size: u6,
    _r0: u2,
    v_size: u6,
    _r1: u2,
    /// 0: One texture of [(YUV_U_Size + 1) * 16] pixels (H) × [(YUV_V_Size + 1) * 16] pixels (V)
    /// 1: [(YUV_U_Size + 1) * (YUV_V_Size + 1)] textures of 16 texels (H) × 16 texels (V)
    tex: u1,
    _r2: u7,
    format: enum(u1) { YUV420 = 0, YUV422 = 1 },
    _r3: u7,
};

pub const RegionArrayDataConfiguration = packed struct(u192) {
    pub const ListPointer = packed struct(u32) {
        _0: u2 = 0,
        pointer_to_object_list: u22,
        _1: u7 = 0,
        empty: bool,

        pub const Empty = ListPointer{ .pointer_to_object_list = std.math.maxInt(u22), .empty = true };

        pub fn format(self: @This(), writer: *std.Io.Writer) !void {
            try writer.print("{s}{X:<6}\u{001b}[0m", .{ termcolor.colored_bool(!self.empty), @as(u24, self.pointer_to_object_list) << 2 });
        }
    };

    settings: packed struct(u32) {
        _r: u2,
        tile_x_position: u6,
        tile_y_position: u6,
        _r1: u14,
        flush_accumulate: bool,
        pre_sort: bool, // Forced 0 for Type 1
        z_clear: enum(u1) { Clear = 0, Load = 1 },
        last_region: bool,

        pub fn format(self: @This(), writer: *std.Io.Writer) !void {
            try writer.print("X:{d:<2} Y:{d:<2} {s}Flush-Accumulate\u{001b}[0m {s}Pre-Sort\u{001b}[0m {s}Z-Clear\u{001b}[0m {s}Last Region\u{001b}[0m", .{
                self.tile_x_position,
                self.tile_y_position,
                termcolor.colored_bool(self.flush_accumulate),
                termcolor.colored_bool(self.pre_sort),
                termcolor.colored_bool(self.z_clear == .Clear),
                termcolor.colored_bool(self.last_region),
            });
        }
    },
    opaque_list_pointer: ListPointer,
    opaque_modifier_volume_pointer: ListPointer,
    translucent_list_pointer: ListPointer,
    translucent_modifier_volume_pointer: ListPointer,
    punch_through_list_pointer: ListPointer, // Absent for Type 1

    pub fn empty(self: @This()) bool {
        return self.opaque_list_pointer.empty and self.opaque_modifier_volume_pointer.empty and self.translucent_list_pointer.empty and self.translucent_modifier_volume_pointer.empty and self.punch_through_list_pointer.empty;
    }

    pub fn format(self: @This(), writer: *std.io.Writer) !void {
        try writer.print("{f}", .{self.settings});
        try writer.print(" | Opaque: {f}", .{self.opaque_list_pointer});
        try writer.print(" | Opaque MV: {f}", .{self.opaque_modifier_volume_pointer});
        try writer.print(" | Translucent: {f}", .{self.translucent_list_pointer});
        try writer.print(" | Translucent MV: {f}", .{self.translucent_modifier_volume_pointer});
        try writer.print(" | PunchThrough: {f}", .{self.punch_through_list_pointer});
    }

    pub fn formatType1(self: @This(), writer: *std.io.Writer) !void {
        try writer.print("{f}", .{self.settings});
        try writer.print(" | Opaque: {f}", .{self.opaque_list_pointer});
        try writer.print(" | Opaque MV: {f}", .{self.opaque_modifier_volume_pointer});
        try writer.print(" | Translucent: {f}", .{self.translucent_list_pointer});
        try writer.print(" | Translucent MV: {f}", .{self.translucent_modifier_volume_pointer});
    }
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

    _,
};

pub const ListType = enum(u3) {
    Opaque = 0,
    OpaqueModifierVolume = 1,
    Translucent = 2,
    TranslucentModifierVolume = 3,
    PunchThrough = 4,
    _,
};

const UserClipUsage = enum(u2) {
    Disable = 0,
    Reserved = 1,
    InsideEnabled = 2,
    OutsideEnabled = 3,
};

pub const GroupControl = packed struct(u8) {
    user_clip: UserClipUsage,
    strip_len: u2,
    _: u3,
    en: u1,
};

pub const ColorType = enum(u2) {
    PackedColor = 0,
    FloatingColor = 1,
    IntensityMode1 = 2,
    IntensityMode2 = 3,
};

pub const ObjControl = packed struct(u16) {
    uv_16bit: u1,
    gouraud: u1,
    offset: u1,
    texture: u1,
    col_type: ColorType,
    volume: u1,
    shadow: u1,
    _: u8 = 0,
};

pub const ParameterControlWord = packed struct(u32) {
    obj_control: ObjControl,
    group_control: GroupControl,
    list_type: ListType,
    _: u1,
    end_of_strip: bool,
    parameter_type: ParameterType,
};

// Control Parameter Formats

pub const UserTileClip = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    _ignored: u96,
    user_clip_x_min: u32,
    user_clip_y_min: u32,
    user_clip_x_max: u32,
    user_clip_y_max: u32,
};

const ObjectListSet = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    object_pointer: u32,
    _ignored: u64,
    bounding_box_x_min: u6,
    _invalid0: u26,
    bounding_box_y_min: u4,
    _invalid1: u28,
    bounding_box_x_max: u36,
    _invalid2: u26,
    bounding_box_y_max: u34,
    _invalid3: u28,
};

// Global Parameter Formats

pub const CullingMode = enum(u2) {
    /// No culling
    None = 0,
    /// Cull if ( |det| < fpu_cull_val )
    Small = 1,
    /// Cull if ( |det| < 0 ) or ( |det| < fpu_cull_val )
    Negative = 2,
    /// Cull if ( |det| > 0 ) or ( |det| < fpu_cull_val )
    Positive = 3,
};

pub const DepthCompareMode = enum(u3) {
    Never = 0,
    Less = 1,
    Equal = 2,
    LessEqual = 3,
    Greater = 4,
    NotEqual = 5,
    GreaterEqual = 6,
    Always = 7,
};

pub const ISPTSPInstructionWord = packed struct(u32) {
    _: u20,
    dcalc_ctrl: u1,
    cache_bypass: u1,
    uv_16bit: u1,
    gouraud: u1,
    offset: u1,
    texture: u1,
    z_write_disable: u1,
    culling_mode: CullingMode,
    depth_compare_mode: DepthCompareMode,
};

pub const TextureShadingInstruction = enum(u2) {
    Decal = 0,
    Modulate = 1,
    DecalAlpha = 2,
    ModulateAlpha = 3,
};

pub const AlphaInstruction = enum(u3) {
    Zero = 0,
    One = 1,
    OtherColor = 2,
    InverseOtherColor = 3,
    SourceAlpha = 4,
    InverseSourceAlpha = 5,
    DestAlpha = 6,
    InverseDestAlpha = 7,
};

pub const FogControl = enum(u2) {
    LookUpTable = 0,
    Vertex = 1,
    None = 2,
    LookUpTableMode2 = 3,
};

pub const TSPInstructionWord = packed struct(u32) {
    texture_v_size: u3,
    texture_u_size: u3,
    texture_shading_instruction: TextureShadingInstruction,
    mipmap_d_adjust: u4,
    supersample_texture: u1,
    filter_mode: enum(u2) { Point = 0, Bilinear = 1, TrilinearPassA = 2, TrilinearPassB = 3 },
    clamp_uv: packed struct(u2) { v: bool, u: bool },
    flip_uv: packed struct(u2) { v: bool, u: bool },
    ignore_texture_alpha: u1,
    use_alpha: u1,
    color_clamp: u1,
    fog_control: FogControl,
    dst_select: u1,
    src_select: u1,
    dst_alpha_instr: AlphaInstruction,
    src_alpha_instr: AlphaInstruction,

    pub fn get_u_size(self: @This()) u32 {
        return (@as(u32, 8) << self.texture_u_size);
    }

    pub fn get_v_size(self: @This()) u32 {
        return (@as(u32, 8) << self.texture_v_size);
    }

    /// "If the clamp function is enabled, the flip function is disabled."
    pub inline fn final_flip_uv(self: @This()) struct { v: bool, u: bool } {
        return .{
            .u = self.flip_uv.u and !self.clamp_uv.u,
            .v = self.flip_uv.v and !self.clamp_uv.v,
        };
    }
};

pub const TexturePixelFormat = enum(u3) {
    ARGB1555 = 0,
    RGB565 = 1,
    ARGB4444 = 2,
    YUV422 = 3,
    BumpMap = 4,
    Palette4BPP = 5,
    Palette8BPP = 6,
    Reserved = 7,
};

// RGB/YUV Texture or Bump Map
pub const TextureControlWord = packed struct(u32) {
    address: u21 = 0,
    _: u4 = 0,
    stride_select: u1 = 0,
    scan_order: u1 = 0,
    pixel_format: TexturePixelFormat = .Reserved,
    vq_compressed: u1 = 0,
    mip_mapped: u1 = 0,

    pub const Mask: u32 = 0xFE1F_FFFF;

    // Mask out reserved bits.
    pub fn masked(self: @This()) u32 {
        return @as(u32, @bitCast(self)) & Mask;
    }

    pub fn palette_selector(self: @This()) u10 {
        return @truncate(switch (self.pixel_format) {
            .Palette4BPP => (((@as(u32, @bitCast(self)) >> 21) & 0b111111) << 4),
            .Palette8BPP => (((@as(u32, @bitCast(self)) >> 25) & 0b11) << 8),
            else => 0,
        });
    }
};

pub const PaletteTextureControlWord = packed struct(u32) {
    address: u21,
    palette_selector: u6,
    pixel_format: TexturePixelFormat,
    vq_compressed: u1,
    mip_mapped: u1,
};

pub const GenericGlobalParameter = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    tsp_instruction: TSPInstructionWord,
    texture_control: TextureControlWord,
    // The rest varies depending on the polygon type
    _ignored: u128,
};

// Packed/Floating Color
const PolygonType0 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    tsp_instruction: TSPInstructionWord,
    texture_control: TextureControlWord,
    _ignored: u64,
    data_size: u32,
    next_address: u32,
};

// Intensity, no Offset Color
const PolygonType1 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    tsp_instruction: TSPInstructionWord,
    texture_control: TextureControlWord,
    face_color: fARGB,
};

// Intensity, use Offset Color
const PolygonType2 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    tsp_instruction: TSPInstructionWord,
    texture_control: TextureControlWord,
    _ignored: u64,
    data_size: u32,
    next_address: u32,
    face_color: fARGB,
    face_offset_color: fARGB,
};

// Packed Color, with Two Volumes
const PolygonType3 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    tsp_instruction_0: TSPInstructionWord,
    texture_control_0: TextureControlWord,
    tsp_instruction_1: TSPInstructionWord,
    texture_control_1: TextureControlWord,
    data_size: u32,
    next_address: u32,
};

// Intensity, with Two Volumes
const PolygonType4 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    tsp_instruction_0: TSPInstructionWord,
    texture_control_0: TextureControlWord,
    tsp_instruction_1: TSPInstructionWord,
    texture_control_1: TextureControlWord,
    data_size: u32,
    next_address: u32,
    face_color_0: fARGB,
    face_color_1: fARGB,
};

const Sprite = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    tsp_instruction: TSPInstructionWord,
    texture_control: TextureControlWord,
    base_color: PackedColor,
    offset_color: PackedColor,
    data_size: u32,
    next_address: u32,
};

pub const Polygon = union(enum) {
    PolygonType0: PolygonType0,
    PolygonType1: PolygonType1,
    PolygonType2: PolygonType2,
    PolygonType3: PolygonType3,
    PolygonType4: PolygonType4,
    Sprite: Sprite,

    pub fn tag(self: @This()) std.meta.Tag(@This()) {
        return std.meta.activeTag(self);
    }

    pub fn size(format: std.meta.Tag(@This())) u32 {
        return switch (format) {
            inline else => |f| @sizeOf(std.meta.TagPayload(@This(), f)) / 4,
        };
    }

    pub fn control_word(self: @This()) ParameterControlWord {
        return switch (self) {
            inline else => |p| p.parameter_control_word,
        };
    }

    pub fn isp_tsp_instruction(self: @This()) ISPTSPInstructionWord {
        return switch (self) {
            inline else => |p| p.isp_tsp_instruction,
        };
    }

    pub fn tsp_instruction(self: @This()) TSPInstructionWord {
        return switch (self) {
            inline .PolygonType3, .PolygonType4 => |p| p.tsp_instruction_0,
            inline else => |p| p.tsp_instruction,
        };
    }

    pub fn texture_control(self: @This()) TextureControlWord {
        return switch (self) {
            inline .PolygonType3, .PolygonType4 => |p| p.texture_control_0,
            inline else => |p| p.texture_control,
        };
    }

    pub fn area1_tsp_instruction(self: @This()) ?TSPInstructionWord {
        return switch (self) {
            inline .PolygonType3, .PolygonType4 => |p| p.tsp_instruction_1,
            else => null,
        };
    }

    pub fn area1_texture_control(self: @This()) ?TextureControlWord {
        return switch (self) {
            inline .PolygonType3, .PolygonType4 => |p| p.texture_control_1,
            else => null,
        };
    }

    pub fn base_color(self: @This()) ?[4]f32 {
        return switch (self) {
            inline .PolygonType1, .PolygonType2 => |p| .{ p.face_color.r, p.face_color.g, p.face_color.b, p.face_color.a },
            .PolygonType4 => |p| .{ p.face_color_0.r, p.face_color_0.g, p.face_color_0.b, p.face_color_0.a },
            .Sprite => |p| @bitCast(fRGBA.from_packed(p.base_color, true)),
            else => null,
        };
    }

    pub fn offset_color(self: @This()) ?[4]f32 {
        return switch (self) {
            .PolygonType2 => |p| .{ p.face_offset_color.r, p.face_offset_color.g, p.face_offset_color.b, p.face_offset_color.a },
            .PolygonType4 => |p| .{ p.face_color_0.r, p.face_color_0.g, p.face_color_0.b, p.face_color_0.a }, // NOTE: In the case of Polygon Type 4 (Intensity, with Two Volumes), the Face Color is used in both the Base Color and the Offset Color.
            .Sprite => |p| @bitCast(fRGBA.from_packed(p.offset_color, true)),
            else => null,
        };
    }
};

fn obj_control_to_polygon_type(obj_control: ObjControl) std.meta.Tag(Polygon) {
    // NOTE: See 3.7.6.2 Parameter Combinations. Some entries are duplicated to account for the fact that the value of offset doesn't matter in these cases.
    // Shadow (Ignored) - Volume - ColType (u2) - Texture - Offset - Gouraud (Ignored) - 16bit UV
    // NOTE: Offset is ignored and fixed at 0 when non-textured
    const mask: u16 = if (obj_control.texture == 0) 0b00000000_0_1_11_1_0_0_1 else 0b00000000_0_1_11_1_1_0_1;
    const masked = @as(u16, @bitCast(obj_control)) & mask;
    switch (masked) {
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0, // FIXME: uv_16bit is supposed to be invalid in this case (Non-textured 16-bit uv doesn't make sense), but Ecco the Dolphin use such polygons? Or am I just receiving bad data?
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType1, // FIXME: Similarly, offset should be forced to 0 here, and UV16 ignored. "Virtua Fighter 3tb (Japan) (Rev 1)" sends such polygons.
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .FloatingColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType1,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType4,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 1, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .FloatingColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 0, .col_type = .FloatingColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .FloatingColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 1, .col_type = .FloatingColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType1,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType2,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType1,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType2,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 1, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType4,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType4,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType4,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode1, .shadow = 0 })) => return .PolygonType4,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 0, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType3,
        @as(u16, @bitCast(ObjControl{ .volume = 1, .texture = 1, .offset = 1, .gouraud = 0, .uv_16bit = 1, .col_type = .IntensityMode2, .shadow = 0 })) => return .PolygonType3,
        else => std.debug.panic(termcolor.red("Unimplemented obj_control_to_polygon_format: {b:0>16}, {}, original: {}\n"), .{ masked, @as(ObjControl, @bitCast(masked)), obj_control }),
    }
}

const VolumeInstruction = enum(u3) {
    Normal = 0,
    InsideLastPolygon = 1,
    OutsideLastPolygon = 2,
    _,

    pub fn masked(self: VolumeInstruction) VolumeInstruction {
        return @enumFromInt(@as(u3, @intFromEnum(self) & 0b11));
    }
};

const ModifierVolumeInstruction = packed struct(u32) {
    _: u27,
    culling_mode: CullingMode,
    volume_instruction: VolumeInstruction,
};

const ModifierVolumeGlobalParameter = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    instructions: ModifierVolumeInstruction,
    _ignored: u192,
};

const ModifierVolumeParameter = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    ax: f32,
    ay: f32,
    az: f32,
    bx: f32,
    by: f32,
    bz: f32,
    cx: f32,
    cy: f32,
    cz: f32,
    _ignored: u192,
};

pub const ModifierVolume = struct {
    parameter_control_word: ParameterControlWord,
    instructions: ModifierVolumeInstruction,
    first_triangle_index: u32,
    triangle_count: u32 = 0,
    closed: bool,
};

const BumpMapParameter = packed struct(u32) {
    q: u8,
    k3: u8,
    k2: u8,
    k1: u8,
};

pub const UV16 = packed struct(u32) {
    v: u16, // Upper bits of a 32-bit float
    u: u16,

    fn to_f32(val: u16) f32 {
        return @bitCast(@as(u32, val) << 16);
    }

    pub fn u_as_f32(self: UV16) f32 {
        return to_f32(self.u);
    }

    pub fn v_as_f32(self: UV16) f32 {
        return to_f32(self.v);
    }
};

// Packed Color, Non-Textured
const VertexParameter_0 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    _ignored0: u64,
    base_color: PackedColor,
    _ignored1: u32,
};
// Non-Textured, Floating Color
const VertexParameter_1 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    base_color: fARGB,
};
// Non-Textured, Intensity
const VertexParameter_2 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    _ignored0: u64,
    base_intensity: f32, // FIXME: Is it really a float?
    _ignored1: u32,
};
// Packed Color, Textured 32bit UV
const VertexParameter_3 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    u: f32,
    v: f32,
    base_color: PackedColor,
    offset_color: PackedColor,
};
// Packed Color, Textured 16bit UV
const VertexParameter_4 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    uv: UV16,
    _ignored: u32,
    base_color: PackedColor,
    offset_color: PackedColor,
};
// Floating Color, Textured
const VertexParameter_5 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    u: f32,
    v: f32,
    _ignored: u64,
    base_color: fARGB,
    offset_color: fARGB,
};
// Floating Color, Textured 16bit UV
const VertexParameter_6 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    uv: UV16,
    _ignored: u96,
    base_color: fARGB,
    offset_color: fARGB,
};
// Intensity, Textured 32bit UV
const VertexParameter_7 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    u: f32,
    v: f32,
    base_intensity: f32,
    offset_intensity: f32,
};
// Intensity, Textured 16bit UV
const VertexParameter_8 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    uv: UV16,
    _ignored: u32,
    base_intensity: f32,
    offset_intensity: f32,
};
// Non-Textured, Packed Color, with Two Volumes
const VertexParameter_9 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    base_color_0: PackedColor,
    base_color_1: PackedColor,
    _ignored: u64,
};
// Non-Textured, Intensity, with Two Volumes
const VertexParameter_10 = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    base_intensity_0: f32,
    base_intensity_1: f32,
    _ignored: u64,
};
// Textured, Packed Color, with Two Volumes
const VertexParameter_11 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    u0: f32,
    v0: f32,
    base_color_0: PackedColor,
    offset_color_0: PackedColor,
    u1: f32,
    v1: f32,
    base_color_1: PackedColor,
    offset_color_1: PackedColor,
    _ignored: u128,
};
// Textured, Packed Color, 16bit UV, with Two Volumes
const VertexParameter_12 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    uv_0: UV16,
    _ignored_0: u32,
    base_color_0: PackedColor,
    offset_color_0: PackedColor,
    uv_1: UV16,
    _ignored_1: u32,
    base_color_1: PackedColor,
    offset_color_1: PackedColor,
    _ignored_2: u128,
};
// Textured, Intensity, with Two Volumes
const VertexParameter_13 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    u0: f32,
    v0: f32,
    base_intensity_0: f32,
    offset_intensity_0: f32,
    u1: f32,
    v1: f32,
    base_intensity_1: f32,
    offset_intensity_1: f32,
    _ignored_2: u128,
};
// Textured, Intensity, with Two Volumes
const VertexParameter_14 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    uv_0: UV16,
    _ignored_0: u32,
    base_intensity_0: f32,
    offset_intensity_0: f32,
    uv_1: UV16,
    _ignored_1: u32,
    base_intensity_1: f32,
    offset_intensity_1: f32,
    _ignored_2: u128,
};

const VertexParameter_Sprite_0 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    ax: f32,
    ay: f32,
    az: f32,
    bx: f32,
    by: f32,
    bz: f32,
    cx: f32,
    cy: f32,
    cz: f32,
    dx: f32,
    dy: f32,
    _: u128,
};

const VertexParameter_Sprite_1 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    ax: f32,
    ay: f32,
    az: f32,
    bx: f32,
    by: f32,
    bz: f32,
    cx: f32,
    cy: f32,
    cz: f32,
    dx: f32,
    dy: f32,
    _: u32,
    auv: UV16,
    buv: UV16,
    cuv: UV16,

    pub fn uvs(self: @This()) [3]UV16 {
        return .{ self.auv, self.buv, self.cuv };
    }
};

pub const VertexParameter = union(enum) {
    Type0: VertexParameter_0,
    Type1: VertexParameter_1,
    Type2: VertexParameter_2,
    Type3: VertexParameter_3,
    Type4: VertexParameter_4,
    Type5: VertexParameter_5,
    Type6: VertexParameter_6,
    Type7: VertexParameter_7,
    Type8: VertexParameter_8,
    Type9: VertexParameter_9,
    Type10: VertexParameter_10,
    Type11: VertexParameter_11,
    Type12: VertexParameter_12,
    Type13: VertexParameter_13,
    Type14: VertexParameter_14,
    SpriteType0: VertexParameter_Sprite_0,
    SpriteType1: VertexParameter_Sprite_1,

    pub fn tag(self: @This()) std.meta.Tag(@This()) {
        return std.meta.activeTag(self);
    }

    // Returns the size in words (4 bytes) of the vertex parameter
    pub fn size(format: std.meta.Tag(@This())) u32 {
        return switch (format) {
            inline else => |f| @sizeOf(std.meta.TagPayload(@This(), f)) / 4,
        };
    }
    pub fn position(self: *const @This()) [3]f32 {
        std.debug.assert(self.tag() != .SpriteType0 and self.tag() != .SpriteType1);
        return @as([*]const f32, @ptrCast(@alignCast(self)))[1..4].*;
    }

    // NOTE: Z position of the last vertex will be garbage.
    pub fn sprite_positions(self: *const @This()) [4][3]f32 {
        std.debug.assert(self.tag() == .SpriteType0 or self.tag() == .SpriteType1);
        return @bitCast(@as([*]const f32, @ptrCast(@alignCast(self)))[1 .. 1 + 4 * 3].*);
    }

    pub fn scale_x(self: *@This(), factor: f32) void {
        return switch (self.*) {
            .SpriteType0 => |*s| {
                s.ax *= factor;
                s.bx *= factor;
                s.cx *= factor;
                s.dx *= factor;
            },
            .SpriteType1 => |*s| {
                s.ax *= factor;
                s.bx *= factor;
                s.cx *= factor;
                s.dx *= factor;
            },
            inline else => |*v| v.x *= factor,
        };
    }
};

fn obj_control_to_vertex_parameter_format(obj_control: ObjControl) std.meta.Tag(VertexParameter) {
    // Shadow (Ignored) - Volume - ColType (u2) - Texture - Offset (Ignored) - Gouraud (Ignored) - 16bit UV
    const masked = @as(u16, @bitCast(obj_control)) & 0b00000000_0_1_11_1_0_0_1;
    switch (masked) {
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .PackedColor, .volume = 0, .shadow = 0 })) => return .Type0,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .PackedColor, .volume = 0, .shadow = 0 })) => return .Type0, // FIXME: uv_16bit is supposed to be invalid here (Non-textured 16-bit uv doesn't make sense). Ecco the Dolphin does this.
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .IntensityMode1, .volume = 0, .shadow = 0 })) => return .Type2, // FIXME: Similarly, offset should be forced to 0 here, and UV16 ignored. "Virtua Fighter 3tb (Japan) (Rev 1)" sends such polygons.
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .FloatingColor, .volume = 0, .shadow = 0 })) => return .Type1,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .IntensityMode1, .volume = 0, .shadow = 0 })) => return .Type2,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .IntensityMode2, .volume = 0, .shadow = 0 })) => return .Type2,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .PackedColor, .volume = 0, .shadow = 0 })) => return .Type3,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .PackedColor, .volume = 0, .shadow = 0 })) => return .Type4,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .FloatingColor, .volume = 0, .shadow = 0 })) => return .Type5,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .FloatingColor, .volume = 0, .shadow = 0 })) => return .Type6,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .IntensityMode1, .volume = 0, .shadow = 0 })) => return .Type7,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .IntensityMode2, .volume = 0, .shadow = 0 })) => return .Type7,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .IntensityMode1, .volume = 0, .shadow = 0 })) => return .Type8,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .IntensityMode2, .volume = 0, .shadow = 0 })) => return .Type8,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .PackedColor, .volume = 1, .shadow = 0 })) => return .Type9,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .IntensityMode1, .volume = 1, .shadow = 0 })) => return .Type10,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .IntensityMode2, .volume = 1, .shadow = 0 })) => return .Type10,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .PackedColor, .volume = 1, .shadow = 0 })) => return .Type11,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .PackedColor, .volume = 1, .shadow = 0 })) => return .Type12,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .IntensityMode1, .volume = 1, .shadow = 0 })) => return .Type13,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .IntensityMode2, .volume = 1, .shadow = 0 })) => return .Type13,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .IntensityMode1, .volume = 1, .shadow = 0 })) => return .Type14,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 1, .col_type = .IntensityMode2, .volume = 1, .shadow = 0 })) => return .Type14,
        else => std.debug.panic(termcolor.red("Unimplemented obj_control_to_vertex_parameter_format: {b}, {}, original: {}"), .{ masked, @as(ObjControl, @bitCast(masked)), obj_control }),
    }
}

pub const UserTileClipInfo = struct {
    usage: UserClipUsage,
    x: u32, // In pixels
    y: u32,
    width: u32,
    height: u32,
};

pub const VertexStrip = struct {
    global_parameters: struct {
        polygon: Polygon,
        face_base_color: fARGB = .one,
        face_offset_color: fARGB = .zero,
        area1_face_base_color: fARGB = .one,
        sprite_face_base_color: PackedColor = .one,
        sprite_face_offset_color: PackedColor = .zero,
    },
    user_clip: ?UserTileClipInfo,
    vertex_parameter_index: usize = 0,
    vertex_parameter_count: usize = 0,
};

pub const DisplayList = struct {
    vertex_strips: std.ArrayList(VertexStrip) = .empty,
    vertex_parameters: std.ArrayList(VertexParameter) = .empty,
    next_first_vertex_parameters_index: usize = 0,

    zclear: bool = true,
    presort: bool = false,

    pub fn init() DisplayList {
        return .{};
    }

    pub fn deinit(self: *DisplayList, allocator: std.mem.Allocator) void {
        self.vertex_parameters.deinit(allocator);
        self.vertex_strips.deinit(allocator);
    }

    pub fn clearRetainingCapacity(self: *DisplayList) void {
        self.vertex_parameters.clearRetainingCapacity();
        self.vertex_strips.clearRetainingCapacity();
        self.next_first_vertex_parameters_index = 0;
    }
};

pub const TALists = struct {
    opaque_list: DisplayList = .{},
    punchthrough_list: DisplayList = .{},
    translucent_list: DisplayList = .{},
    opaque_modifier_volumes: std.ArrayList(ModifierVolume) = .empty,
    translucent_modifier_volumes: std.ArrayList(ModifierVolume) = .empty,
    volume_triangles: std.ArrayList(ModifierVolumeParameter) = .empty,

    _should_reset: bool = false,

    pub fn init() TALists {
        return .{};
    }

    pub fn deinit(self: *TALists, allocator: std.mem.Allocator) void {
        self.opaque_list.deinit(allocator);
        self.punchthrough_list.deinit(allocator);
        self.translucent_list.deinit(allocator);
        self.opaque_modifier_volumes.deinit(allocator);
        self.translucent_modifier_volumes.deinit(allocator);
        self.volume_triangles.deinit(allocator);
    }

    pub fn clearRetainingCapacity(self: *TALists) void {
        self.opaque_list.clearRetainingCapacity();
        self.punchthrough_list.clearRetainingCapacity();
        self.translucent_list.clearRetainingCapacity();
        self.opaque_modifier_volumes.clearRetainingCapacity();
        self.translucent_modifier_volumes.clearRetainingCapacity();
        self.volume_triangles.clearRetainingCapacity();
    }

    pub fn get_list(self: *TALists, list_type: ListType) *DisplayList {
        return switch (list_type) {
            .Opaque => &self.opaque_list,
            .PunchThrough => &self.punchthrough_list,
            .Translucent => &self.translucent_list,
            else => std.debug.panic("Invalid List Type: {t}", .{list_type}),
        };
    }

    pub fn mark_reset(self: *TALists) void {
        // NOTE: Apparently lists are not reinitialized immediately, but on the next write.
        // MetalliC says it matters for some games, although I haven't encountered it myself.
        self._should_reset = true;
    }

    pub fn check_reset(self: *TALists) void {
        if (self._should_reset) {
            self._should_reset = false;
            self.clearRetainingCapacity();
        }
    }

    // TODO? Or safe to ignore?
    pub fn serialize(_: @This(), writer: *std.Io.Writer) !usize {
        _ = writer;
        return 0;
    }
    pub fn deserialize(_: @This(), reader: anytype) !usize {
        _ = reader;
        return 0;
    }
};

pub const Holly = struct {
    pub const VRAMSize = 8 * 1024 * 1024;
    pub const VRAMMask: u32 = VRAMSize - 1;
    pub const RegistersSize = 0x2000;

    vram: []align(64) u8, // Not owned.
    registers: []u8,

    dirty_framebuffer: bool = false,

    _allocator: std.mem.Allocator,
    _dc: *Dreamcast,

    _ta_current_pass: u8 = 0,
    _ta_command_buffer: [16]u32 align(32) = @splat(0),
    _ta_command_buffer_index: u32 = 0,
    _ta_list_type: ?ListType = null,
    _ta_current_polygon: ?Polygon = null,
    _ta_user_tile_clip: ?UserTileClipInfo = null,
    _ta_current_volume: ?ModifierVolume = null,
    _ta_volume_next_polygon_is_last: bool = false,

    // NOTE: The following isn't serialized. I don't think this is worth breaking compatibility for.
    _ta_face_base_color: fARGB = .one,
    _ta_face_offset_color: fARGB = .zero,
    _ta_area1_face_base_color: fARGB = .one,
    _ta_sprite_face_base_color: PackedColor = .one,
    _ta_sprite_face_offset_color: PackedColor = .zero,

    // When starting a render, the user can select where to get the parameters from using
    // the PARAM_BASE register. It is specified in 1MB blocks, meaning it can take at most
    // 16 different values (actually 8 in the case of the base DC and its 8MB of VRAM).
    // We don't emulate the object writes to VRAM, but some games submit multiple lists concurently
    // ("double buffering" the object lists), so we have to keep track of that.
    // Most games will only need a single list per frame, but when using List Continuation for
    // multipass rendering, we'll allocate more as needed.
    _ta_lists: [16]std.ArrayList(TALists) = @splat(.empty),

    _pixel: u64 = 0,
    _tmp_subcycles: u64 = 0,
    _last_spg_update: u64 = 0,

    pub fn init(allocator: std.mem.Allocator, dc: *Dreamcast) !Holly {
        var r = Holly{
            .vram = dc.vram,
            .registers = try allocator.alloc(u8, RegistersSize), // FIXME: Huge waste of memory
            ._allocator = allocator,
            ._dc = dc,
        };
        for (&r._ta_lists) |*ta_list|
            try ta_list.append(allocator, .init());
        return r;
    }

    pub fn deinit(self: *@This()) void {
        for (&self._ta_lists) |*ta_lists| {
            for (ta_lists.items) |*list|
                list.deinit(self._allocator);
            ta_lists.deinit(self._allocator);
        }
        self._allocator.free(self.registers);
    }

    pub fn reset(self: *@This()) void {
        @memset(self.vram, 0);
        @memset(self.registers, 0);

        for (&self._ta_lists) |*ta_lists| {
            for (ta_lists.items) |*list|
                list.clearRetainingCapacity();
        }

        self._get_register(u32, .ID).* = 0x17FD11DB;
        self._get_register(u32, .REVISION).* = 0x0011;
        self._get_register(SPG_STATUS, .SPG_STATUS).* = .{};
        self._get_register(u32, .PARAM_BASE).* = 0;
        self._get_register(u32, .REGION_BASE).* = 0;
        self._get_register(u32, .SPAN_SOFT_CFG).* = 0x00000101;

        self._get_register(u32, .FPU_SHAD_SCALE).* = 0;
        self._get_register(u32, .FPU_CULL_VAL).* = 0;
        self._get_register(u32, .FPU_PARAM_CFG).* = 0x0007DF77;
        self._get_register(u32, .FPU_PERP_VAL).* = 0;

        self._get_register(u32, .SDRAM_REFRESH).* = 0x00000020;
        self._get_register(u32, .SDRAM_ARB_CFG).* = 0;
        self._get_register(u32, .SDRAM_CFG).* = 0x15D1C951;

        self._get_register(u32, .Y_COEFF).* = 0;
        self._get_register(u32, .PT_ALPHA_REF).* = 0xFF;

        self._get_register(u32, .ISP_BACKGND_D).* = 0;
        self._get_register(u32, .ISP_BACKGND_T).* = 0;
        self._get_register(u32, .ISP_FEED_CFG).* = 0x01002000;

        self._get_register(SPG_LOAD, .SPG_LOAD).* = .{};
        self._get_register(SPG_HBLANK, .SPG_HBLANK).* = .{};
        self._get_register(SPG_VBLANK, .SPG_VBLANK).* = .{};
        self._get_register(SPG_HBLANK_INT, .SPG_HBLANK_INT).* = .{};
        self._get_register(SPG_VBLANK_INT, .SPG_VBLANK_INT).* = .{};
        self._get_register(SPG_CONTROL, .SPG_CONTROL).* = .{};
        self._get_register(u32, .SPG_STATUS).* = 0;

        self._get_register(u32, .TEXT_CONTROL).* = 0;
        self._get_register(u32, .VO_CONTROL).* = 0x000108;
        self._get_register(u32, .VO_STARTX).* = 0x09D;
        self._get_register(u32, .VO_STARTY).* = 0x00150015;
        self._get_register(u32, .VO_BORDER_COL).* = 0;
        self._get_register(u32, .SCALER_CTL).* = 0x0400;

        self._get_register(u32, .PAL_RAM_CTRL).* = 0;

        self._get_register(u32, .FOG_COL_RAM).* = 0;
        self._get_register(u32, .FOG_COL_VERT).* = 0;
        self._get_register(u32, .FOG_DENSITY).* = 0;
        self._get_register(u32, .FOG_CLAMP_MAX).* = 0;
        self._get_register(u32, .FOG_CLAMP_MIN).* = 0;

        self._get_register(u32, .FB_R_CTRL).* = 0;
        self._get_register(u32, .FB_W_CTRL).* = 0;
        self._get_register(u32, .FB_W_LINESTRIDE).* = 0;
        self._get_register(u32, .FB_R_SOF1).* = 0;
        self._get_register(u32, .FB_R_SOF2).* = 0;
        self._get_register(u32, .FB_R_SIZE).* = 0;
        self._get_register(u32, .FB_W_SOF1).* = 0;
        self._get_register(u32, .FB_W_SOF2).* = 0;
        self._get_register(u32, .FB_X_CLIP).* = 0;
        self._get_register(u32, .FB_Y_CLIP).* = 0;
        self._get_register(u32, .FB_BURSTCTRL).* = 0x00093F39;
        self._get_register(u32, .FB_C_SOF).* = 0;

        self._get_register(u32, .TA_LIST_CONT).* = 0;

        self.schedule_interrupts();
    }

    inline fn pixels_to_sh4_cycles(self: *const @This(), pixels: u64) u64 {
        const fb_r_ctrl = self.read_register(FB_R_CTRL, .FB_R_CTRL);
        const factor: u64 = if (fb_r_ctrl.vclk_div == 0) 2 else 1;
        return @divTrunc(factor * 200 * pixels, 27);
    }

    pub fn schedule_hblank_in(self: *@This()) void {
        const spg_load = self.read_register(SPG_LOAD, .SPG_LOAD);
        const spg_status = self.read_register(SPG_STATUS, .SPG_STATUS);
        const spg_hblank_int = self.read_register(SPG_HBLANK_INT, .SPG_HBLANK_INT);
        const max_scanline: u32 = spg_load.vcount + 1;
        const target_scanline: u32 = (switch (spg_hblank_int.hblank_int_mode) {
            // Output when the display line is the value indicated by line_comp_val.
            0 => @as(u32, spg_hblank_int.line_comp_val),
            // Output every line_comp_val lines.
            1 => @as(u32, spg_status.scanline) + spg_hblank_int.line_comp_val, // FIXME: Really?
            // Output every line.
            2 => @as(u32, spg_status.scanline) + 1,
            else => std.debug.panic("Invalid hblank_int_mode: {d}", .{spg_hblank_int.hblank_int_mode}),
        }) % max_scanline;
        // NOTE: Interlace does not matter here.
        var line_diff: i64 = if (spg_status.scanline < target_scanline)
            target_scanline - spg_status.scanline
        else
            (if (max_scanline >= spg_status.scanline) max_scanline - spg_status.scanline else 0) + target_scanline;
        const pixel_diff: i64 = @as(i64, @intCast(spg_hblank_int.hblank_in_interrupt)) - @as(i64, @intCast(self._pixel));
        const hcount: i64 = spg_load.hcount;
        if (line_diff == 0 and pixel_diff <= 0) line_diff = max_scanline;
        const pixels: u64 = @intCast((hcount + 1) * line_diff + pixel_diff);

        self._dc.schedule_event(.HBlankIn, self.pixels_to_sh4_cycles(pixels));
    }
    inline fn schedule_event_at_scanline(self: *@This(), event: DreamcastModule.ScheduledEvent.Event, target_scanline: u64) void {
        const spg_control = self.read_register(SPG_CONTROL, .SPG_CONTROL);
        const spg_load = self.read_register(SPG_LOAD, .SPG_LOAD);
        const spg_status = self.read_register(SPG_STATUS, .SPG_STATUS);
        const max_scanline = @as(u32, spg_load.vcount) + 1;
        var line_diff: u64 = if (spg_status.scanline < target_scanline)
            target_scanline - spg_status.scanline
        else
            (if (max_scanline >= spg_status.scanline) max_scanline - spg_status.scanline else 0) + target_scanline;
        if (line_diff == 0) line_diff = max_scanline;
        if (spg_control.interlace) line_diff = @max((line_diff + 1) / 2, 1);
        const hcount: u64 = spg_load.hcount;
        const sh4_cycles = self.pixels_to_sh4_cycles((hcount + 1) * line_diff);
        std.debug.assert(sh4_cycles > 0);
        self._dc.schedule_event(event, sh4_cycles);
    }
    pub fn schedule_vblank_in(self: *@This()) void {
        self.schedule_event_at_scanline(.VBlankIn, self.read_register(SPG_VBLANK_INT, .SPG_VBLANK_INT).vblank_in_interrupt_line_number);
    }
    pub fn schedule_vblank_out(self: *@This()) void {
        self.schedule_event_at_scanline(.VBlankOut, self.read_register(SPG_VBLANK_INT, .SPG_VBLANK_INT).vblank_out_interrupt_line_number);
    }

    pub fn on_hblank_in(self: *@This()) void {
        self._dc.raise_normal_interrupt(.{ .HBlankIn = 1 });
        self.schedule_hblank_in();
    }

    pub fn on_vblank_in(self: *@This()) void {
        self._dc.raise_normal_interrupt(.{ .VBlankIn = 1 });
        self.schedule_vblank_in();
    }
    pub fn on_vblank_out(self: *@This()) void {
        if (self.read_register(SPG_CONTROL, .SPG_CONTROL).interlace)
            self._get_register(SPG_STATUS, .SPG_STATUS).fieldnum +%= 1;
        // If SB_MDTSEL is set, initiate Maple DMA one line before VBlankOut
        // FIXME: This probably shouldn't be here.
        if (self._dc.read_hw_register(u32, .SB_MDEN) & 1 == 1 and self._dc.read_hw_register(u32, .SB_MDTSEL) & 1 == 1) {
            // NOTE: Registers SB_MSYS and SB_MSHTCL control some behaviours here and aren't emulated.
            holly_log.info("Maple DMA triggered over VBlankOut.", .{});
            self._dc.start_maple_dma();
        }
        self._dc.raise_normal_interrupt(.{ .VBlankOut = 1 });
        // self._get_register(SPG_STATUS, .SPG_STATUS).*.vsync = false;
        self.schedule_vblank_out();
    }

    fn schedule_interrupts(self: *@This()) void {
        self.schedule_hblank_in();
        self.schedule_vblank_in();
        self.schedule_vblank_out();
    }

    fn clear_scheduled_interrupts(self: *@This()) void {
        self._dc.clear_event(.HBlankIn);
        self._dc.clear_event(.VBlankIn);
        self._dc.clear_event(.VBlankOut);
    }

    const ClocksLCM = 5400; // Least Common Multiple of the SH4 clock (200MHz) and the Pixel clock (27MHz)

    inline fn subcycles_per_pixel(self: *const @This()) u64 {
        const fb_r_ctrl = self.read_register(FB_R_CTRL, .FB_R_CTRL);
        return (if (fb_r_ctrl.vclk_div == 0) @as(u32, 2) else 1) * (200 * ClocksLCM / 27);
    }

    pub fn update_spg_status(self: *@This()) void {
        if (self._dc._global_cycles == self._last_spg_update) return;
        self._tmp_subcycles += ClocksLCM * (self._dc._global_cycles - self._last_spg_update);
        self._last_spg_update = self._dc._global_cycles;
        const cpp = self.subcycles_per_pixel();

        if (self._tmp_subcycles >= cpp) {
            const spg_hblank = self.read_register(SPG_HBLANK, .SPG_HBLANK);
            const spg_load = self.read_register(SPG_LOAD, .SPG_LOAD);
            self._pixel += self._tmp_subcycles / cpp;
            self._tmp_subcycles %= cpp;

            const spg_status = self._get_register(SPG_STATUS, .SPG_STATUS);

            spg_status.hblank = self._pixel >= spg_hblank.hbstart or self._pixel <= spg_hblank.hbend;

            const hcount = spg_load.hcount + 1;
            if (self._pixel >= hcount) {
                const line_count = self._pixel / hcount;
                self._pixel %= hcount;

                const spg_control = self.read_register(SPG_CONTROL, .SPG_CONTROL);
                const max_scanline = spg_load.vcount + 1;
                var target_scanline: u64 = spg_status.scanline;
                if (spg_control.interlace) {
                    target_scanline += 2 * @as(u64, line_count);
                    if (target_scanline >= max_scanline) {
                        const field = target_scanline & 1;
                        const full_scans = target_scanline / max_scanline;
                        target_scanline %= max_scanline;
                        target_scanline &= ~@as(u64, 1);
                        target_scanline |= (full_scans + field) % 2; // Next field
                    }
                } else {
                    target_scanline += line_count;
                    target_scanline %= max_scanline;
                }
                spg_status.scanline = @intCast(target_scanline);

                const spg_vblank = self.read_register(SPG_VBLANK, .SPG_VBLANK);
                spg_status.vsync = spg_status.scanline >= spg_vblank.vbstart or spg_status.scanline < spg_vblank.vbend;
                if (spg_vblank.vbstart < spg_vblank.vbend) spg_status.vsync = !spg_status.vsync;
            }
        }
    }

    pub fn write_register(self: *@This(), addr: u32, v: u32) void {
        switch (@as(HollyRegister, @enumFromInt(addr))) {
            .ID, .REVISION => return, // Read-only
            .SOFTRESET => {
                const sr: SOFT_RESET = @bitCast(v);
                if (sr.TASoftReset) {
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
                    self._ta_current_volume = null;
                    self._ta_user_tile_clip = null;
                }
                if (sr.PipelineSoftReset) holly_log.debug(termcolor.yellow("TODO: Pipeline Soft Reset"), .{});
                if (sr.SDRAMInterfaceSoftReset) holly_log.debug(termcolor.yellow("TODO: SDRAM Interface Soft Reset"), .{});
                return;
            },
            .STARTRENDER => {
                holly_log.debug(termcolor.green("STARTRENDER!"), .{});

                self._dc.on_render_start.call(self._dc);

                // FIXME: Almost arbitrary values: Lowest delay I found after a quick search that fixes Marvel vs. Capcom 2
                //        (see skmp notes: https://web.archive.org/web/20110809053548/http://drk.emudev.org/blog/?page_id=4)
                self._dc.schedule_interrupt(.{ .RenderDoneTSP = 1 }, 200_000);
                self._dc.schedule_interrupt(.{ .RenderDoneISP = 1 }, 250_000);
                self._dc.schedule_interrupt(.{ .RenderDoneVideo = 1 }, 300_000); // FIXME: Raise an interrupt when the render is actually done?
            },
            .TA_LIST_INIT => {
                if (v == 0x80000000) {
                    self._ta_current_pass = 0;
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
                    self._ta_current_volume = null;
                    self._ta_user_tile_clip = null;
                    self._get_register(u32, .TA_NEXT_OPB).* = self.read_register(u32, .TA_NEXT_OPB_INIT);
                    self._get_register(u32, .TA_ITP_CURRENT).* = self.read_register(u32, .TA_ISP_BASE);

                    holly_log.info("TA_LIST_INIT: PARAM_BASE: {X:0>8} | TA_OL_BASE: {X:0>8} | TA_ISP_BASE: {X:0>8} | TA_NEXT_OPB_INIT: {X:0>8}", .{
                        self.read_register(u32, .PARAM_BASE),
                        self.read_register(u32, .TA_OL_BASE),
                        self.read_register(u32, .TA_ISP_BASE),
                        self.read_register(u32, .TA_NEXT_OPB_INIT),
                    });

                    self.ta_current_lists().mark_reset();
                }
                return;
            },
            .TA_LIST_CONT => {
                holly_log.debug("Write to TA_LIST_CONT: {X:0>8}", .{v});
                // Same thing as TA_LIST_INIT, but without reseting the list, nor the TA registers? (Not really tested yet)
                if (v == 0x80000000) {
                    self._ta_current_pass += 1;
                    while (self._ta_lists[self.ta_list_index()].items.len <= self._ta_current_pass)
                        self._ta_lists[self.ta_list_index()].append(self._allocator, .init()) catch @panic("Out of memory");
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
                    self._ta_current_volume = null;
                    self._ta_user_tile_clip = null;
                }
                return;
            },
            .TA_YUV_TEX_BASE => self._get_register(u32, .TA_YUV_TEX_CNT).* = 0,
            .SPG_HBLANK, .SPG_HBLANK_INT => {
                self._dc.clear_event(.HBlankIn);
                self._get_register_from_addr(u32, addr).* = v;
                self.schedule_hblank_in();
                return;
            },
            .SPG_VBLANK, .SPG_VBLANK_INT => {
                self._dc.clear_event(.VBlankIn);
                self._dc.clear_event(.VBlankOut);
                self._get_register_from_addr(u32, addr).* = v;
                self.schedule_vblank_in();
                self.schedule_vblank_out();
                return;
            },
            .SPG_STATUS, .SPG_CONTROL, .SPG_LOAD, .FB_R_CTRL => {
                self.clear_scheduled_interrupts();
                self._get_register_from_addr(u32, addr).* = v;
                self.schedule_interrupts();
                return;
            },
            else => |reg| {
                holly_log.debug("Write to Register: @{X:0>8} {s} = {X:0>8}", .{ addr, std.enums.tagName(HollyRegister, reg) orelse "Unknown", v });
            },
        }
        self._get_register_from_addr(u32, addr).* = v;
    }

    /// Write to the Tile Accelerator
    pub inline fn write_ta(self: *@This(), addr: u32, v: []const u32, access_type: enum(u32) { b64 = 0, b32 = 1 }) void {
        holly_log.debug("  TA Bulk Write: {X:0>8} = {any}\n", .{ addr, v });
        std.debug.assert(addr >= 0x10000000 and addr < 0x14000000);
        switch (addr) {
            0x10000000...0x107FFFFF, 0x12000000...0x127FFFFF => if (v.len % 8 == 0) self.write_ta_fifo_polygon_path(v) else {
                for (v) |w| {
                    self._ta_command_buffer[self._ta_command_buffer_index] = w;
                    self._ta_command_buffer_index += 1;
                    self.handle_command();
                }
            },
            0x10800000...0x10FFFFFF, 0x12800000...0x12FFFFFF => holly_log.warn(termcolor.yellow("  TODO: YUV Conv. {X:0>8} = {any}"), .{ addr, v }),
            0x11000000...0x11FFFFFF, 0x13000000...0x13FFFFFF => {
                // Direct Texture Path
                switch (access_type) {
                    .b64 => {
                        for (v, 0..) |w, idx|
                            @as(*u32, @ptrCast(@alignCast(&self.vram[addr & VRAMMask + 4 * idx]))).* = w;
                    },
                    .b32 => {
                        for (v, 0..) |w, idx|
                            self.write_vram(u32, @intCast(addr + 4 * idx), w);
                    },
                }
            },
            else => holly_log.err(termcolor.red("  Unhandled TA Bulk Write to @{X:0>8} = {any}"), .{ addr, v }),
        }
    }

    pub fn write_ta_fifo_polygon_path(self: *@This(), v: []const u32) void {
        std.debug.assert(v.len >= 8 and v.len % 8 == 0);
        std.debug.assert(self._ta_command_buffer_index % 8 == 0);
        for (0..v.len / 8) |i|
            self.write_ta_fifo_polygon_path_command(v[8 * i ..][0..8].*);
    }

    pub fn write_ta_fifo_polygon_path_command(self: *@This(), v: @Vector(8, u32)) void {
        std.debug.assert(self._ta_command_buffer_index % 8 == 0);
        @setRuntimeSafety(false);
        const dst: *@Vector(8, u32) = @ptrCast(@alignCast(&self._ta_command_buffer[self._ta_command_buffer_index]));
        dst.* = v;
        self._ta_command_buffer_index += 8;
        self.handle_command();
    }

    pub fn write_ta_direct_texture_path(self: *@This(), addr: u32, access_type: enum(u32) { b64 = 0, b32 = 1 }, v: @Vector(8, u32)) void {
        switch (access_type) {
            .b64 => @as(*@Vector(8, u32), @ptrCast(@alignCast(&self.vram[addr & VRAMMask]))).* = v,
            .b32 => {
                for (0..8) |idx|
                    self.write_vram(u32, @intCast(addr + 4 * idx), v[idx]);
            },
        }
    }

    pub fn write_ta_fifo_direct_texture_path(self: *@This(), addr: u32, value: []u8) void {
        holly_log.debug("  NOTE: DMA to Direct Texture Path to {X:0>8} (len: {X:0>8})", .{ addr, value.len });
        @memcpy(self.vram[addr & VRAMMask .. (addr & VRAMMask) + value.len], value);
    }

    fn yuv_converter_process_macro_block(u_size: u32, v_size: u32, x: u32, y: u32, dest: [*]YUV422, data: []const u8) void {
        _ = v_size;

        const line_size = u_size * 16 / 2; // in YUV422
        const block_start = (16 * y * line_size) + (16 / 2 * x);
        const pixels = dest[block_start..];

        const u = data[0..64];
        const v = data[64..128];
        const y0 = data[128..192]; // (0,0) to (7,7)
        const y1 = data[192..256]; // (8,0) to (15,7)
        const y2 = data[256..320]; // (0,8) to (7,15)
        const y3 = data[320..384]; // (8,8) to (15,15)

        for (0..8) |j| {
            for (0..8) |i| {
                pixels[(2 * j + 0) * line_size + i].u = u[8 * j + i];
                pixels[(2 * j + 0) * line_size + i].v = v[8 * j + i];
                pixels[(2 * j + 1) * line_size + i].u = u[8 * j + i];
                pixels[(2 * j + 1) * line_size + i].v = v[8 * j + i];
            }
        }

        for (0..8) |j| {
            for (0..4) |i| {
                pixels[(j + 0) * line_size + i + 0].y0 = y0[8 * j + 2 * i];
                pixels[(j + 0) * line_size + i + 0].y1 = y0[8 * j + 2 * i + 1];

                pixels[(j + 0) * line_size + i + 4].y0 = y1[8 * j + 2 * i];
                pixels[(j + 0) * line_size + i + 4].y1 = y1[8 * j + 2 * i + 1];

                pixels[(j + 8) * line_size + i + 0].y0 = y2[8 * j + 2 * i];
                pixels[(j + 8) * line_size + i + 0].y1 = y2[8 * j + 2 * i + 1];

                pixels[(j + 8) * line_size + i + 4].y0 = y3[8 * j + 2 * i];
                pixels[(j + 8) * line_size + i + 4].y1 = y3[8 * j + 2 * i + 1];
            }
        }
    }

    /// Expects length of data to be a divisor of 384 (size of a macro block).
    pub fn write_ta_fifo_yuv_converter_path_partial(self: *@This(), data: []u8) void {
        const static = struct {
            var buffer: [6 * 64]u8 = undefined;
            var index: usize = 0;
        };
        @memcpy(static.buffer[static.index..][0..data.len], data);
        static.index += data.len;

        if (static.index == static.buffer.len) {
            const ctrl = self._get_register(TA_YUV_TEX_CTRL, .TA_YUV_TEX_CTRL).*;
            std.debug.assert(ctrl.format == .YUV420 and ctrl.tex == 0); // Other variations aren't implemented.
            const u_size = @as(u32, ctrl.u_size) + 1; // In 16x16 blocks
            const v_size = @as(u32, ctrl.v_size) + 1; // In 16x16 blocks

            const block_counter = self._get_register(u32, .TA_YUV_TEX_CNT);
            const x = block_counter.* % u_size;
            const y = block_counter.* / u_size;

            const tex_base = self._get_register(u32, .TA_YUV_TEX_BASE).*;
            const tex: [*]YUV422 = @ptrCast(@alignCast(&self.vram[tex_base]));

            yuv_converter_process_macro_block(u_size, v_size, x, y, tex, &static.buffer);

            block_counter.* += 1;
            if (block_counter.* == u_size * v_size) {
                self._dc.schedule_interrupt(.{ .EoT_YUV = 1 }, 0);
                block_counter.* = 0; // FIXME: Resident Evil 2 doesn't seem to reset the counter by writing to TA_YUV_TEX_BASE... I don't know if this a proper fix.
            }
            static.index = 0;
        }
    }

    /// Expects data to be large enough to decode the whole texture (e.g. from DMA).
    pub fn write_ta_fifo_yuv_converter_path(self: *@This(), data: []u8) void {
        const tex_base = self._get_register(u32, .TA_YUV_TEX_BASE).*;
        const ctrl = self._get_register(TA_YUV_TEX_CTRL, .TA_YUV_TEX_CTRL).*;
        const u_size = @as(u32, ctrl.u_size) + 1; // In 16x16 blocks
        const v_size = @as(u32, ctrl.v_size) + 1; // In 16x16 blocks
        const tex: [*]YUV422 = @ptrCast(@alignCast(&self.vram[tex_base]));
        switch (ctrl.format) {
            .YUV420 => {
                if (ctrl.tex == 0) {
                    //   YUV_Tex = 0
                    // The YUV data that is input is stored in texture memory as one texture with a size of
                    // [(YUV_U_Size + 1) * 16] (H) * [(YUV_V_Size + 1) * 16] (V). This format has a weakness
                    // in that storage time is longer because the storage addresses in texture memory will
                    // not be continuous every 16 pixels (32 bytes) in the horizontal direction.
                    var offset: u32 = 0;
                    for (0..v_size) |y| {
                        for (0..u_size) |x| {
                            yuv_converter_process_macro_block(u_size, v_size, @intCast(x), @intCast(y), tex, data[offset..]);
                            offset += 6 * 64;
                        }
                    }
                    self._get_register(u32, .TA_YUV_TEX_CNT).* += u_size * v_size; // NOTE: If this was async, we could update it as we go.
                    // FIXME: Delay is arbitrary.
                    if (self._get_register(u32, .TA_YUV_TEX_CNT).* == u_size * v_size)
                        self._dc.schedule_interrupt(.{ .EoT_YUV = 1 }, u_size * v_size * 8000);
                } else {
                    //   YUV_Tex = 1
                    // [(YUV_U_Size + 1) * (YUV_V_Size + 1)] textures of the macro size (16 * 16 pixels) are
                    // stored in texture memory. Storage time is shorter, because the storage addresses in
                    // texture memory are continuous. However, each texture must be used with a different
                    // polygon and arranged on screen.
                    holly_log.err(termcolor.red("ta_fifo_yuv_converter_path: Unimplemented tex=1"), .{});
                }
            },
            else => holly_log.err(termcolor.red("ta_fifo_yuv_converter_path: Unimplemented format: {d}"), .{ctrl.format}),
        }
    }

    inline fn ta_list_index(self: *const @This()) u4 {
        // Should I record the value of TA_ISP_BASE on LIST_INIT?
        // NOTE: We also assume that TA_OL_BASE is in the same 1MB range here.
        return @truncate((self.read_register(u32, .TA_ISP_BASE) >> 20) & 0xF);
    }

    inline fn ta_current_lists(self: *@This()) *TALists {
        while (self._ta_lists[self.ta_list_index()].items.len <= self._ta_current_pass) {
            holly_log.err("Accessing pass {d} of list {d} with only {d} passes", .{ self._ta_current_pass, self.ta_list_index(), self._ta_lists[self.ta_list_index()].items.len });
            self._ta_lists[self.ta_list_index()].append(self._allocator, .init()) catch @panic("Out of memory");
        }
        return &self._ta_lists[self.ta_list_index()].items[self._ta_current_pass];
    }

    inline fn start_list(self: *@This(), list_type: ListType) void {
        holly_log.debug("Starting List {t} - TA_NEXT_OPB: {X:0>6}", .{ list_type, self._get_register(u32, .TA_NEXT_OPB).* });
        self._ta_list_type = list_type;
        self.ta_current_lists().check_reset();
    }

    inline fn update_clip_usage(self: *@This(), group_control: GroupControl) void {
        if (group_control.en == 1) {
            if (self._ta_user_tile_clip) |*uc| {
                uc.usage = group_control.user_clip;
            }
        }
    }

    pub fn handle_command(self: *@This()) void {
        if (self._ta_command_buffer_index % 8 != 0) return; // All commands are 8 or 16 u32 long
        std.debug.assert(self._ta_command_buffer_index == 8 or self._ta_command_buffer_index == 16);

        const parameter_control_word: ParameterControlWord = @bitCast(self._ta_command_buffer[0]);

        holly_log.debug(" TA Parameter Type: {}", .{parameter_control_word.parameter_type});
        for (0..self._ta_command_buffer_index) |i|
            holly_log.debug("      {X:0>8}", .{self._ta_command_buffer[i]});

        switch (parameter_control_word.parameter_type) {
            // Control Parameters
            .EndOfList => {
                self.check_end_of_modifier_volume();
                if (self._ta_list_type) |list| { // Apparently this happens?... Why would a game do this?
                    // Fire corresponding interrupt. FIXME: Delay is completely arbitrary, I just need to delay them for testing, for now.
                    self._dc.schedule_interrupt(switch (list) {
                        .Opaque => .{ .EoT_OpaqueList = 1 },
                        .OpaqueModifierVolume => .{ .EoT_OpaqueModifierVolumeList = 1 },
                        .Translucent => .{ .EoT_TranslucentList = 1 },
                        .TranslucentModifierVolume => .{ .EoT_TranslucentModifierVolumeList = 1 },
                        .PunchThrough => .{ .EoD_PunchThroughList = 1 },
                        else => std.debug.panic(termcolor.red("  Unimplemented List Type {}"), .{list}),
                    }, 800);

                    // NOTE: Probably not actually usefull, more of a debugging tool for now.
                    const global_tile_clip = self.read_register(TA_GLOB_TILE_CLIP, .TA_GLOB_TILE_CLIP);
                    const alloc_ctrl = self.read_register(TA_ALLOC_CTRL, .TA_ALLOC_CTRL);
                    self._get_register(u32, .TA_NEXT_OPB).* += @intCast(@as(u32, 4) * @as(u32, global_tile_clip.tile_x_num + 1) * @as(u32, global_tile_clip.tile_y_num + 1) * (switch (list) {
                        .Opaque => alloc_ctrl.O_OPB.byteSize(),
                        .OpaqueModifierVolume => alloc_ctrl.OM_OPB.byteSize(),
                        .Translucent => alloc_ctrl.T_OPB.byteSize(),
                        .TranslucentModifierVolume => alloc_ctrl.TM_OPB.byteSize(),
                        .PunchThrough => alloc_ctrl.PT_OPB.byteSize(),
                        else => std.debug.panic("Invalid display list type: {}", .{list}),
                    }));
                }

                self._ta_current_polygon = null;
                self._ta_list_type = null;
            },
            .UserTileClip => {
                const user_tile_clip: *const UserTileClip = @ptrCast(&self._ta_command_buffer);
                self._ta_user_tile_clip = .{
                    .usage = .Disable,
                    .x = 32 *| user_tile_clip.user_clip_x_min,
                    .y = 32 *| user_tile_clip.user_clip_y_min,
                    .width = 32 *| (1 +| user_tile_clip.user_clip_x_max -| user_tile_clip.user_clip_x_min),
                    .height = 32 *| (1 +| user_tile_clip.user_clip_y_max -| user_tile_clip.user_clip_y_min),
                };
            },
            .ObjectListSet => holly_log.err(termcolor.red("Unhandled ObjectListSet"), .{}),
            // Global Parameters
            .PolygonOrModifierVolume => {
                if (self._ta_list_type == null)
                    self.start_list(parameter_control_word.list_type);
                self.update_clip_usage(parameter_control_word.group_control);

                // NOTE: I have no idea if this is actually an issue, or if it is just ignored when we've already started a list (and thus set the list type).
                //       But I'm leaning towards "This value is valid in the following four cases" means it's ignored in the others.
                // if (self._ta_list_type != parameter_control_word.list_type)
                //     holly_log.err(termcolor.red("  PolygonOrModifierVolume list type mismatch: Expected {?}, got {}"), .{ self._ta_list_type, parameter_control_word.list_type });

                switch (self._ta_list_type.?) {
                    .OpaqueModifierVolume, .TranslucentModifierVolume => {
                        const modifier_volume: *const ModifierVolumeGlobalParameter = @ptrCast(&self._ta_command_buffer);
                        // NOTE: Dead or Alive 2 emits a modifier volume with an volume instruction of '4'. The following ignores the extra bit and treats it as the start of a new modifier volume.
                        switch (modifier_volume.instructions.volume_instruction.masked()) {
                            .Normal => {
                                if (self._ta_current_volume == null) { // NOTE: I don't remember why this check is here, and I find it a bit suspicious.
                                    self._ta_current_volume = .{
                                        .parameter_control_word = modifier_volume.parameter_control_word,
                                        .instructions = modifier_volume.instructions,
                                        .first_triangle_index = @intCast(self.ta_current_lists().volume_triangles.items.len),
                                        .closed = modifier_volume.parameter_control_word.obj_control.volume == 1,
                                    };
                                }
                            },
                            .InsideLastPolygon, .OutsideLastPolygon => {
                                if (self._ta_current_volume) |*cv| {
                                    self._ta_volume_next_polygon_is_last = true;
                                    cv.instructions = modifier_volume.instructions;
                                    cv.closed = modifier_volume.parameter_control_word.obj_control.volume == 1;
                                }
                            },
                            else => holly_log.debug(termcolor.red("Invalid Volume Instruction: {d}") ++ "\n{}", .{ modifier_volume.instructions.volume_instruction, modifier_volume }),
                        }
                    },
                    else => {
                        const polygon_type = obj_control_to_polygon_type(parameter_control_word.obj_control);
                        if (self._ta_command_buffer_index < Polygon.size(polygon_type)) return;
                        std.debug.assert(self._ta_command_buffer_index == Polygon.size(polygon_type));

                        self.copy_isp_tsp_parameters();

                        self._ta_current_polygon = switch (polygon_type) {
                            .Sprite => std.debug.panic("Invalid polygon format: {t}", .{polygon_type}),
                            inline else => |pt| @unionInit(Polygon, @tagName(pt), @as(*std.meta.TagPayload(Polygon, pt), @ptrCast(&self._ta_command_buffer)).*),
                        };

                        switch (self._ta_current_polygon.?) {
                            .PolygonType1 => |p| {
                                self._ta_face_base_color = p.face_color;
                            },
                            .PolygonType2 => |p| {
                                self._ta_face_base_color = p.face_color;
                                self._ta_face_offset_color = p.face_offset_color;
                            },
                            .PolygonType4 => |p| {
                                // NOTE: In the case of Polygon Type 4 (Intensity, with Two Volumes), the Face Color is used in both the Base Color and the Offset Color.
                                self._ta_face_base_color = p.face_color_0;
                                self._ta_area1_face_base_color = p.face_color_1;
                            },
                            else => |p| if (p.control_word().obj_control.col_type == .IntensityMode1)
                                holly_log.warn(termcolor.yellow("Intensity Mode 1 polygon with unexpected Polygon Type {t}: {}"), .{ p.tag(), p }),
                        }
                    },
                }
            },
            .SpriteList => {
                if (self._ta_list_type == null)
                    self.start_list(parameter_control_word.list_type);
                self.update_clip_usage(parameter_control_word.group_control);

                self.copy_isp_tsp_parameters();

                self._ta_current_polygon = .{ .Sprite = @as(*Sprite, @ptrCast(&self._ta_command_buffer)).* };

                self._ta_sprite_face_base_color = self._ta_current_polygon.?.Sprite.base_color;
                self._ta_sprite_face_offset_color = self._ta_current_polygon.?.Sprite.offset_color;
            },
            // VertexParameter - Yes it's a category of its own.
            .VertexParameter => {
                if (self._ta_list_type) |list_type| {
                    switch (list_type) {
                        .OpaqueModifierVolume, .TranslucentModifierVolume => {
                            if (self._ta_command_buffer_index < @sizeOf(ModifierVolumeParameter) / 4) return;
                            self.ta_current_lists().volume_triangles.append(self._allocator, @as(*ModifierVolumeParameter, @ptrCast(&self._ta_command_buffer)).*) catch |err|
                                holly_log.err(termcolor.red("Failed to append ModifierVolumeParameter: {t}"), .{err});

                            if (self._ta_volume_next_polygon_is_last)
                                self.check_end_of_modifier_volume();
                        },
                        else => {
                            if (self._ta_current_polygon) |polygon| {
                                const display_list = self.ta_current_lists().get_list(list_type);
                                const polygon_obj_control = polygon.control_word().obj_control;
                                switch (polygon) {
                                    .Sprite => {
                                        if (!parameter_control_word.end_of_strip) // Sanity check: For Sprites/Quads, each vertex parameter describes an entire polygon.
                                            holly_log.warn(termcolor.yellow("Unexpected Sprite without end of strip bit:") ++ "\n  {}", .{parameter_control_word});

                                        if (polygon_obj_control.texture == 0) {
                                            if (self._ta_command_buffer_index < VertexParameter.size(.SpriteType0)) return;
                                            display_list.vertex_parameters.append(self._allocator, .{ .SpriteType0 = @as(*VertexParameter_Sprite_0, @ptrCast(&self._ta_command_buffer)).* }) catch |err|
                                                holly_log.err(termcolor.red("Failed to append VertexParameter: {t}"), .{err});
                                        } else {
                                            if (self._ta_command_buffer_index < VertexParameter.size(.SpriteType1)) return;
                                            display_list.vertex_parameters.append(self._allocator, .{ .SpriteType1 = @as(*VertexParameter_Sprite_1, @ptrCast(&self._ta_command_buffer)).* }) catch |err|
                                                holly_log.err(termcolor.red("Failed to append VertexParameter: {t}"), .{err});
                                        }
                                    },
                                    else => {
                                        const format = obj_control_to_vertex_parameter_format(polygon_obj_control);
                                        if (self._ta_command_buffer_index < VertexParameter.size(format)) return;

                                        display_list.vertex_parameters.append(self._allocator, switch (format) {
                                            .SpriteType0, .SpriteType1 => unreachable,
                                            inline else => |t| @unionInit(VertexParameter, @tagName(t), @as(*std.meta.TagPayload(VertexParameter, t), @ptrCast(&self._ta_command_buffer)).*),
                                        }) catch |err|
                                            holly_log.err(termcolor.red("Failed to append VertexParameter: {t}"), .{err});
                                    },
                                }

                                if (parameter_control_word.end_of_strip) {
                                    display_list.vertex_strips.append(self._allocator, .{
                                        .global_parameters = .{
                                            .polygon = polygon,
                                            .face_base_color = self._ta_face_base_color,
                                            .face_offset_color = self._ta_face_offset_color,
                                            .area1_face_base_color = self._ta_area1_face_base_color,
                                            .sprite_face_base_color = self._ta_sprite_face_base_color,
                                            .sprite_face_offset_color = self._ta_sprite_face_offset_color,
                                        },
                                        .user_clip = if (self._ta_user_tile_clip) |uc| if (uc.usage != .Disable) uc else null else null,
                                        .vertex_parameter_index = display_list.next_first_vertex_parameters_index,
                                        .vertex_parameter_count = display_list.vertex_parameters.items.len - display_list.next_first_vertex_parameters_index,
                                    }) catch |err|
                                        holly_log.err(termcolor.red("Failed to append VertexStrip: {t}"), .{err});

                                    display_list.next_first_vertex_parameters_index = display_list.vertex_parameters.items.len;
                                }
                            } else holly_log.err(termcolor.red("No current polygon! Current list type: {t}"), .{list_type});
                        },
                        _ => unreachable,
                    }
                } else holly_log.err(termcolor.red("Received VertexParameter without an active list!"), .{});
            },
            _ => std.debug.panic(termcolor.red("    Invalid parameter type: {d}."), .{parameter_control_word.parameter_type}),
        }
        // Command has been handled, reset buffer.
        self._ta_command_buffer_index = 0;
    }

    fn check_end_of_modifier_volume(self: *@This()) void {
        if (self._ta_list_type) |list_type| {
            if (list_type == .OpaqueModifierVolume or list_type == .TranslucentModifierVolume) {
                if (self._ta_current_volume) |*volume| {
                    // FIXME: I should probably honor _ta_user_tile_clip here too... Given the examples in the doc, modifier volumes can also be clipped.

                    std.debug.assert(volume.first_triangle_index <= self.ta_current_lists().volume_triangles.items.len); // This could happen if TA_LIST_INIT isn't correctly called...

                    volume.triangle_count = @intCast(self.ta_current_lists().volume_triangles.items.len - volume.first_triangle_index);

                    // NOTE: Soul Calibur will push volumes with a single triangle by starting each frame with this sequence:
                    //   Global Parameter: Modifier Volume with holly.VolumeInstruction.Normal
                    //   Global Parameter: Modifier Volume with holly.VolumeInstruction.InsideLastPolygon
                    //     Vertex Parameter: Modifier Volume Triangle
                    // I don't know if I'm the one misinterpreting the doc, but how the triangles are grouped doesn't really matter anyway.

                    if (volume.triangle_count > 0) {
                        const config = self.get_region_array_data_config(0);
                        if (@as(u32, @bitCast(config.opaque_modifier_volume_pointer)) == @as(u32, @bitCast(config.translucent_modifier_volume_pointer))) {
                            // Both lists are actually the same, we'll add it twice for convenience.
                            self.ta_current_lists().opaque_modifier_volumes.append(self._allocator, volume.*) catch |err| std.debug.panic("Failed to append ModifierVolume : {}", .{err});
                            self.ta_current_lists().translucent_modifier_volumes.append(self._allocator, volume.*) catch |err| std.debug.panic("Failed to append ModifierVolume : {}", .{err});
                        } else {
                            (if (list_type == .OpaqueModifierVolume)
                                self.ta_current_lists().opaque_modifier_volumes
                            else
                                self.ta_current_lists().translucent_modifier_volumes).append(self._allocator, volume.*) catch |err| std.debug.panic("Failed to append ModifierVolume : {}", .{err});
                        }
                    }

                    // Soul Calibur will continue sending triangles without submitting a new Global Parameter, prepare for that.
                    volume.first_triangle_index = @intCast(self.ta_current_lists().volume_triangles.items.len);
                    volume.triangle_count = 0;

                    self._ta_volume_next_polygon_is_last = false;
                }
            }
        }
    }

    fn copy_isp_tsp_parameters(self: *@This()) void {
        // "Four bits in the ISP/TSP Instruction Word are overwritten with the corresponding bit values from the Parameter Control Word."
        const global_parameter: *GenericGlobalParameter = @ptrCast(&self._ta_command_buffer);
        global_parameter.isp_tsp_instruction.texture = global_parameter.parameter_control_word.obj_control.texture;
        global_parameter.isp_tsp_instruction.offset = global_parameter.parameter_control_word.obj_control.offset;
        global_parameter.isp_tsp_instruction.gouraud = global_parameter.parameter_control_word.obj_control.gouraud;
        global_parameter.isp_tsp_instruction.uv_16bit = global_parameter.parameter_control_word.obj_control.uv_16bit;
    }

    pub inline fn read_register(self: *const @This(), comptime T: type, r: HollyRegister) T {
        switch (r) {
            .SPG_STATUS => self._dc.gpu.update_spg_status(),
            else => {},
        }
        return @constCast(self)._get_register_from_addr(T, @intFromEnum(r)).*;
    }

    pub inline fn _get_register(self: *@This(), comptime T: type, r: HollyRegister) *T {
        return self._get_register_from_addr(T, @intFromEnum(r));
    }

    pub inline fn _get_register_from_addr(self: *@This(), comptime T: type, addr: u32) *T {
        std.debug.assert(addr >= HollyRegisterStart and addr < HollyRegisterStart + self.registers.len);
        return @as(*T, @ptrCast(@alignCast(&self.registers[addr - HollyRegisterStart])));
    }

    pub inline fn get_palette(self: *const @This()) []const u32 {
        return @as([*]const u32, @ptrCast(@alignCast(&self.registers[@intFromEnum(HollyRegister.PALETTE_RAM_START) - HollyRegisterStart])))[0..1024];
    }

    pub inline fn get_fog_table(self: *const @This()) []const u32 {
        return @as([*]const u32, @ptrCast(@alignCast(&self.registers[@intFromEnum(HollyRegister.FOG_TABLE_START) - HollyRegisterStart])))[0..0x80];
    }

    pub inline fn get_region_header_type(self: *const @This()) FPU_PARAM_CFG.RegionHeaderType {
        return self.read_register(FPU_PARAM_CFG, .FPU_PARAM_CFG).region_header_type;
    }

    pub inline fn get_region_array_data_config(self: *const @This(), idx: usize) RegionArrayDataConfiguration {
        const region_base = self.read_register(u32, .REGION_BASE);
        // Should we skip the first one when it's empty?
        var first_valid = false;
        for (1..self.get_region_header_type().word_size()) |pointer_idx| {
            if (!self.read_vram(RegionArrayDataConfiguration.ListPointer, @intCast(region_base + pointer_idx * 4)).empty) {
                first_valid = true;
                break;
            }
        }
        const stride: u32 = 4 * self.get_region_header_type().word_size();
        const offset = if (first_valid) 0 else stride;

        const region_addr: u32 = @intCast(region_base + offset + idx * stride);
        return .{
            .settings = @bitCast(self.read_vram(u32, region_addr + 0 * 4)),
            .opaque_list_pointer = self.read_vram(RegionArrayDataConfiguration.ListPointer, region_addr + 1 * 4),
            .opaque_modifier_volume_pointer = self.read_vram(RegionArrayDataConfiguration.ListPointer, region_addr + 2 * 4),
            .translucent_list_pointer = self.read_vram(RegionArrayDataConfiguration.ListPointer, region_addr + 3 * 4),
            .translucent_modifier_volume_pointer = self.read_vram(RegionArrayDataConfiguration.ListPointer, region_addr + 4 * 4),
            .punch_through_list_pointer = self.read_vram(RegionArrayDataConfiguration.ListPointer, region_addr + 5 * 4),
        };
    }

    pub inline fn pre_sort(self: *const @This(), pass: usize) bool {
        if (self.get_region_header_type() == 0) {
            return self.read_register(ISP_FEED_CFG, .ISP_FEED_CFG).presort_mode;
        } else {
            return self.get_region_array_data_config(pass).settings.pre_sort == 1;
        }
    }

    fn check_framebuffer_write(self: *@This(), addr: u32) void {
        if (self.dirty_framebuffer) return;

        const local_addr = addr & VRAMMask;

        const spg_control = self.read_register(SPG_CONTROL, .SPG_CONTROL);
        const fb1_start_addr = self.read_register(u32, .FB_R_SOF1) & VRAMMask;
        const fb2_start_addr = self.read_register(u32, .FB_R_SOF2) & VRAMMask;
        const fb_r_size = self.read_register(FB_R_SIZE, .FB_R_SIZE);
        const line_size: u32 = 4 * (@as(u32, fb_r_size.x_size) + @as(u32, fb_r_size.modulus)); // From 32-bit units to bytes.
        const line_count: u32 = @as(u32, fb_r_size.y_size) + 1; // Number of lines
        const fb1_end_addr = fb1_start_addr + line_count * line_size;
        const fb2_end_addr = fb2_start_addr + line_count * line_size;

        if ((local_addr >= fb1_start_addr and local_addr < fb1_end_addr) or
            (spg_control.interlace and (local_addr >= fb2_start_addr and local_addr < fb2_end_addr)))
        {
            self.dirty_framebuffer = true;
        }
    }

    /// 32-bit path read (0x50000000 - 0x5FFFFFFF; or DMA with SB_LMMODE0/1 == 1)
    pub inline fn read_vram(self: *const @This(), comptime T: type, addr: u32) T {
        std.debug.assert(@sizeOf(T) <= 4);
        return @as(*T, @ptrCast(@alignCast(&self.vram[translate_32bit_path_addr(addr & VRAMMask)]))).*;
    }
    /// 32-bit path write (0x50000000 - 0x5FFFFFFF; or DMA with SB_LMMODE0/1 == 1)
    pub inline fn write_vram(self: *@This(), comptime T: type, addr: u32, value: T) void {
        std.debug.assert(@sizeOf(T) <= 4);
        self.check_framebuffer_write(addr);
        @as(*T, @ptrCast(@alignCast(&self.vram[translate_32bit_path_addr(addr & VRAMMask)]))).* = value;
    }
    inline fn translate_32bit_path_addr(offset: u32) u32 {
        //   64bit access             32bit access
        //  Bus A      Bus B         Bus A      Bus B
        // 0x00000000 0x00000004    0x00000000 0x00400000
        // 0x00000008 0x0000000c    0x00000004 0x00400004
        // 0x00000010 0x00000014    0x00000008 0x00400008
        // 0x00000018 0x0000001c    0x0000000c 0x0040000c
        // 0x00000020 0x00000024    0x00000010 0x00400010
        // 0x00000028 0x0000002c    0x00000014 0x00400014
        // ...                      ...
        // 0x007ffff8 0x007ffffc    0x003ffffc 0x007ffffc
        // ----------------------- ----------------------
        // 0x00800000 0x00800004    0x00800000 0x00c00000 (Extented?)
        // 0x00800008 0x0080000c    0x00800004 0x00c00004
        // 0x00800010 0x00800014    0x00800008 0x00c00008
        // 0x00800018 0x0080001c    0x0080000c 0x00c0000c
        // 0x00800020 0x00800024    0x00800010 0x00c00010
        // 0x00800028 0x0080002c    0x00800014 0x00c00014
        // ...                      ...
        // 0x00fffff8 0x00fffffc    0x00bffffc 0x00fffffc

        var addr = (((offset & 0xFFFFFFFC) << 1) & VRAMMask) | (offset & 0x3);
        if (offset & 0x400000 != 0) addr += 4;
        if (offset & 0x800000 != 0) addr |= 0x800000;
        return addr;
    }

    pub fn render_to_texture(self: *const @This()) bool {
        const scaler_ctl = self.read_register(SCALER_CTL, .SCALER_CTL);
        const field = if (scaler_ctl.interlace) scaler_ctl.field_select else 0;
        const FB_W_SOF = self.read_register(u32, if (field == 0) .FB_W_SOF1 else .FB_W_SOF2);
        return FB_W_SOF & 0x1000000 != 0;
    }

    pub const WritebackParameters = struct {
        scaler_ctl: SCALER_CTL,
        w_ctrl: FB_W_CTRL,
        x_clip: FB_CLIP,
        y_clip: FB_CLIP,
        video_out_ctrl: VO_CONTROL,
        fb_w_sof1: u32,
        fb_w_sof2: u32,
        fb_w_linestride: u32,
    };
    pub fn get_write_back_parameters(self: *const @This()) WritebackParameters {
        return .{
            .scaler_ctl = self.read_register(SCALER_CTL, .SCALER_CTL),
            .w_ctrl = self.read_register(FB_W_CTRL, .FB_W_CTRL),
            .x_clip = self.read_register(FB_CLIP, .FB_X_CLIP),
            .y_clip = self.read_register(FB_CLIP, .FB_Y_CLIP),
            .video_out_ctrl = self.read_register(VO_CONTROL, .VO_CONTROL),
            .fb_w_sof1 = self.read_register(u32, .FB_W_SOF1),
            .fb_w_sof2 = self.read_register(u32, .FB_W_SOF2),
            .fb_w_linestride = self.read_register(u32, .FB_W_LINESTRIDE),
        };
    }

    pub fn write_framebuffer(self: *@This(), parameters: WritebackParameters, size: struct { width: u32, height: u32 }, pixels: []const u8) void {
        const interlaced = parameters.scaler_ctl.interlace;
        const field = if (interlaced) parameters.scaler_ctl.field_select else 0;
        const fb_addr = if (field == 0) parameters.fb_w_sof1 else parameters.fb_w_sof2;
        const access_32bit = fb_addr & 0x1000000 == 0;

        const stride = 8 * (parameters.fb_w_linestride & 0x1FF);
        const line_offset = field;
        const line_stride: u32 = if (interlaced) 2 else 1;
        const line_count = size.height / line_stride;
        for (parameters.y_clip.min..@min(line_count, parameters.y_clip.max + 1)) |y| {
            for (parameters.x_clip.min..@min(size.width, parameters.x_clip.max + 1)) |x| {
                const idx = ((line_stride * y + line_offset) * size.width + x) * 4;
                switch (parameters.w_ctrl.fb_packmode) {
                    .RGB565 => {
                        var addr: u32 = (fb_addr & VRAMMask) + @as(u32, @intCast(y)) * stride + 2 * @as(u32, @intCast(x));
                        if (access_32bit) addr = translate_32bit_path_addr(addr);
                        var pixel: *Colors.Color16 = @ptrCast(@alignCast(&self.vram[addr]));
                        pixel.rgb565.r = @truncate(pixels[idx + 2] >> 3);
                        pixel.rgb565.g = @truncate(pixels[idx + 1] >> 2);
                        pixel.rgb565.b = @truncate(pixels[idx + 0] >> 3);
                    },
                    .ARGB1555 => {
                        var addr: u32 = (fb_addr & VRAMMask) + @as(u32, @intCast(y)) * stride + 2 * @as(u32, @intCast(x));
                        if (access_32bit) addr = translate_32bit_path_addr(addr);
                        var pixel: *Colors.Color16 = @ptrCast(@alignCast(&self.vram[addr]));
                        pixel.argb1555.r = @truncate(pixels[idx + 2] >> 3);
                        pixel.argb1555.g = @truncate(pixels[idx + 1] >> 3);
                        pixel.argb1555.b = @truncate(pixels[idx + 0] >> 3);
                        pixel.argb1555.a = 1;
                    },
                    .RGB888 => {
                        var addr = (fb_addr & VRAMMask) + @as(u32, @intCast(y)) * stride + 3 * @as(u32, @intCast(x));
                        if (access_32bit) addr = translate_32bit_path_addr(addr);
                        self.vram[addr + 0] = pixels[idx + 2];
                        self.vram[addr + 1] = pixels[idx + 1];
                        self.vram[addr + 2] = pixels[idx + 0];
                    },
                    else => {
                        std.log.warn("TODO: {t}", .{parameters.w_ctrl.fb_packmode});
                    },
                }
            }
        }
    }

    /// Should be called after full deserialization: Requires the the global cycle counter to be updated as well.
    pub fn finalize_deserialization(self: *@This()) void {
        self.clear_scheduled_interrupts();
        self.schedule_interrupts();
    }

    pub fn serialize(self: *const @This(), writer: *std.Io.Writer) !usize {
        var bytes: usize = 0;
        bytes += try writer.write(std.mem.sliceAsBytes(self.registers[0..]));
        bytes += try writer.write(std.mem.asBytes(&self.dirty_framebuffer));
        bytes += try writer.write(std.mem.sliceAsBytes(self._ta_command_buffer[0..]));
        bytes += try writer.write(std.mem.asBytes(&self._ta_command_buffer_index));
        bytes += try writer.write(std.mem.asBytes(&self._ta_list_type));
        bytes += try writer.write(std.mem.asBytes(&self._ta_current_polygon));
        bytes += try writer.write(std.mem.asBytes(&self._ta_user_tile_clip));
        bytes += try writer.write(std.mem.asBytes(&self._ta_current_volume));
        bytes += try writer.write(std.mem.asBytes(&self._ta_volume_next_polygon_is_last));
        // NOTE: We're not currently serializing TA Lists.
        // for (&self._ta_lists) |*lists| {
        //     var list_count = lists.items.len;
        //     bytes += try writer.write(std.mem.asBytes(&list_count));
        //     for (lists.items) |*list| {
        //         bytes += try list.serialize(writer);
        //     }
        // }
        bytes += try writer.write(std.mem.asBytes(&self._pixel));
        bytes += try writer.write(std.mem.asBytes(&self._tmp_subcycles));
        bytes += try writer.write(std.mem.asBytes(&self._last_spg_update));
        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: *std.Io.Reader) !void {
        try reader.readSliceAll(std.mem.sliceAsBytes(self.registers[0..]));
        try reader.readSliceAll(std.mem.asBytes(&self.dirty_framebuffer));
        try reader.readSliceAll(std.mem.sliceAsBytes(self._ta_command_buffer[0..]));
        try reader.readSliceAll(std.mem.asBytes(&self._ta_command_buffer_index));
        try reader.readSliceAll(std.mem.asBytes(&self._ta_list_type));
        try reader.readSliceAll(std.mem.asBytes(&self._ta_current_polygon));
        try reader.readSliceAll(std.mem.asBytes(&self._ta_user_tile_clip));
        try reader.readSliceAll(std.mem.asBytes(&self._ta_current_volume));
        try reader.readSliceAll(std.mem.asBytes(&self._ta_volume_next_polygon_is_last));
        // for (&self._ta_lists) |*lists| {
        //     for (lists.items) |*list| {
        //         list.deinit();
        //     }
        //     lists.clearRetainingCapacity();
        //     var list_count: u32 = 0;
        //     bytes += try reader.read(std.mem.asBytes(&list_count));
        //     for (0..list_count) |i| {
        //         try lists.append(.init(self._allocator));
        //         bytes += try lists.items[i].deserialize(reader);
        //     }
        // }
        try reader.readSliceAll(std.mem.asBytes(&self._pixel));
        try reader.readSliceAll(std.mem.asBytes(&self._tmp_subcycles));
        try reader.readSliceAll(std.mem.asBytes(&self._last_spg_update));
    }
};

comptime {
    std.debug.assert(Holly.translate_32bit_path_addr(0x00000000) == 0x00000000);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00000004) == 0x00000008);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00000008) == 0x00000010);
    std.debug.assert(Holly.translate_32bit_path_addr(0x0000000c) == 0x00000018);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00000010) == 0x00000020);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00000014) == 0x00000028);
    std.debug.assert(Holly.translate_32bit_path_addr(0x003ffffc) == 0x007ffff8);

    std.debug.assert(Holly.translate_32bit_path_addr(0x00400000) == 0x00000004);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00400004) == 0x0000000c);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00400008) == 0x00000014);
    std.debug.assert(Holly.translate_32bit_path_addr(0x0040000c) == 0x0000001c);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00400010) == 0x00000024);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00400014) == 0x0000002c);
    std.debug.assert(Holly.translate_32bit_path_addr(0x007ffffc) == 0x007ffffc);

    std.debug.assert(Holly.translate_32bit_path_addr(0x00800000) == 0x00800000);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00800004) == 0x00800008);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00800008) == 0x00800010);
    std.debug.assert(Holly.translate_32bit_path_addr(0x0080000c) == 0x00800018);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00800010) == 0x00800020);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00800014) == 0x00800028);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00bffffc) == 0x00fffff8);

    std.debug.assert(Holly.translate_32bit_path_addr(0x00c00000) == 0x00800004);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00c00004) == 0x0080000c);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00c00008) == 0x00800014);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00c0000c) == 0x0080001c);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00c00010) == 0x00800024);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00c00014) == 0x0080002c);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00fffffc) == 0x00fffffc);

    std.debug.assert(Holly.translate_32bit_path_addr(0x00000002) == 0x00000002);
    std.debug.assert(Holly.translate_32bit_path_addr(0x00400002) == 0x00000006);
}
