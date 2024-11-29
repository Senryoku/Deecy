const std = @import("std");

const Dreamcast = @import("dreamcast.zig").Dreamcast;

const holly_log = std.log.scoped(.holly);

const HardwareRegisters = @import("hardware_registers.zig");
const HardwareRegister = HardwareRegisters.HardwareRegister;

const termcolor = @import("termcolor");

const Colors = @import("colors.zig");
const PackedColor = Colors.PackedColor;
const YUV422 = Colors.YUV422;
const fARGB = Colors.fARGB;

pub const HollyRegister = enum(u32) {
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
    FB_W_LINESTRIDE = 0x005F804C,
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
    FB_C_SOF = 0x005F8114, // Specify the starting address, in 32-bit units, for the frame that is currently being sent to the DAC.
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

pub const HollyRegisterStart: u32 = 0x005F8000;

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
    NTSC: u1 = 1,
    PAL: u1 = 0,
    sync_direction: u1 = 0,
    csync_on_h: u1 = 0,
    _: u22 = 0,
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

pub const SCALER_CTL = packed struct(u32) {
    vertical_scale_factor: u16, // This field specifies the scale factor in the vertical direction. (default = 0x0400)
    // This value consists of a 6-bit integer portion and a 10-bit decimal portion, and
    // expands or reduces the screen in the vertical direction by "1/scale factor." When
    // using flicker-free interlace mode type B, specify 0x0800.
    horizontal_scaling_enable: bool, // This field specifies whether or not to use the horizontal direction 1/2 scaler.
    interlace: bool, // This register specifies whether or not to use flicker-free interlace mode type B.
    field_select: u1, // This register specifies the field that is to be stored in the frame buffer in flicker-free interlace mode type B.
    _reserved: u13,
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
    fb_dither: bool, // Dithering enable
    _0: u4,
    fb_kval: u8, // This field sets the K value for writing to the frame buffer. (default = 0x00)
    fb_alpha_threshold: u8, // This field sets the comparison value that is used to determine the alpha value when the data that is written to the frame buffer is ARGB1555 format data. (default = 0x00) When pixel alpha ≥ fb_alpha_threshold, a "1" is written in the alpha value.
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

pub const FB_R_CTRL = packed struct(u32) {
    enable: bool,
    line_double: bool,
    format: u2, // Named "depth" in the documentation
    concat: u3,
    _0: u1,
    chroma_threshold: u8,
    stripsize: u6,
    strip_buf_en: bool,
    vclk_div: u1,
    _1: u8,
};

pub const FB_R_SIZE = packed struct(u32) {
    x_size: u10, // Number of display pixels on each line - 1, in 32-bit units
    y_size: u10, // Number of display lines - 1
    modulus: u10, // Amount of data between each line, in 32-bit units
    _: u2,
};

pub const TEXT_CONTROL = packed struct(u32) {
    stride: u5, // This field specifies the U size of the stride texture. The U size is the stride value × 32.
    _r0: u3,
    bank_bit: u5 = 0, // This field specifies the position of the bank bit when accessing texture memory (default = 0x00). Normally, set 0x00
    _r1: u3,
    index_endian_reg: u1 = 0, // 0 = Little Endian, 1 = Big Endian
    code_book_endian_reg: u1 = 0,
    _r2: u14,
};

pub const VO_CONTROL = packed struct(u32) {
    hsync_pool: u1, // Polarity of HSYNC (0 = active low, 1 = active high)
    vsync_pool: u1, // Polarity of VSYNC (0 = active low, 1 = active high)
    blank_pool: u1, // Polarity of BLANK (0 = active low, 1 = active high)
    blank_video: u1, // This field specifies whether to display the screen or not. 0: Display the screen, 1: Do not display the screen. (Display the border color.) (default)
    field_mode: enum(u4) {
        SPG = 0, // Use field flag from SPG. (default)
        InverseSPG = 1, // Use inverse of field flag from SPG.
        Field1 = 2, // Field 1 fixed.
        Field2 = 3, // Field 2 fixed.
        Field1Sync = 4, // Field 1 when the active edges of HSYNC and VSYNC match.
        Field2Sync = 5, // Field 2 when the active edges of HSYNC and VSYNC match.
        Field1Async = 6, // Field 1 when HSYNC becomes active in the middle of the VSYNC active edge.
        Field2Async = 7, // Field 2 when HSYNC becomes active in the middle of the VSYNC active edge.
        VsyncInverted = 8, // Inverted at the active edge of VSYNC
        _, // Reserved
    },
    pixel_double: bool, // This field specifies whether to output the same pixel or not for two pixels in the horizontal direction. 0: not pixel double, 1: pixel double (default)
    _r0: u7,
    pclk_delay: u6, // This field specifies the delay for the PCLK signal to the DAC.
    _r1: u10,
};

pub const TA_ALLOC_CTRL = packed struct(u32) {
    O_OPB: u2,
    _r0: u2,
    OM_OPB: u2,
    _r1: u2,
    T_OPB: u2,
    _r2: u2,
    TM_OPB: u2,
    _r3: u2,
    PT_OPB: u2,
    _r4: u2,
    OPB_Mode: u1,
    _r5: u15,
};

pub const TA_YUV_TEX_CTRL = packed struct(u32) {
    u_size: u6, // Actual size in pixels is 16 * (u_size + 1)
    _r0: u2,
    v_size: u6,
    _r1: u2,
    tex: u1, // 0: One texture of [(YUV_U_Size + 1) * 16] pixels (H) × [(YUV_V_Size + 1) * 16] pixels (V) ; 1 : [(YUV_U_Size + 1) * (YUV_V_Size + 1)] textures of 16 texels (H) × 16 texels (V)
    _r2: u7,
    format: u1, // 0: YUV420, 1: YUV422
    _r3: u7,
};

pub const RegionArrayDataConfiguration = packed struct(u192) {
    pub const ListPointer = packed struct(u32) {
        _0: u2 = 0,
        pointer_to_object_list: u22,
        _1: u7 = 0,
        empty: bool,
    };

    settings: packed struct(u32) {
        _r: u2,
        tile_x_position: u6,
        tile_y_position: u6,
        _r1: u14,
        flush_accumulate: u1,
        pre_sort: u1, // Forced 0 for Type 1
        z_clear: u1,
        last_region: u1,
    },
    opaque_list_pointer: ListPointer,
    opaque_modifier_volume_pointer: ListPointer,
    translucent_list_pointer: ListPointer,
    translucent_modifier_volume_pointer: ListPointer,
    punch_through_list_pointer: ListPointer, // Absent for Type 1
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
    end_of_strip: u1,
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
    culling_mode: u2,
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
    clamp_uv: u2,
    flip_uv: u2,
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

const PolygonType = enum {
    PolygonType0,
    PolygonType1,
    PolygonType2,
    PolygonType3,
    PolygonType4,
    Sprite,
};

pub const Polygon = union(PolygonType) {
    PolygonType0: PolygonType0,
    PolygonType1: PolygonType1,
    PolygonType2: PolygonType2,
    PolygonType3: PolygonType3,
    PolygonType4: PolygonType4,
    Sprite: Sprite,

    pub fn tag(self: @This()) PolygonType {
        return switch (self) {
            .PolygonType0 => .PolygonType0,
            .PolygonType1 => .PolygonType1,
            .PolygonType2 => .PolygonType2,
            .PolygonType3 => .PolygonType3,
            .PolygonType4 => .PolygonType4,
            .Sprite => .Sprite,
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
            .Sprite => |p| @bitCast(Colors.fRGBA.from_packed(p.base_color, true)),
            else => null,
        };
    }

    pub fn offset_color(self: @This()) ?[4]f32 {
        return switch (self) {
            .PolygonType2 => |p| .{ p.face_offset_color.r, p.face_offset_color.g, p.face_offset_color.b, p.face_offset_color.a },
            .PolygonType4 => |p| .{ p.face_color_0.r, p.face_color_0.g, p.face_color_0.b, p.face_color_0.a }, // NOTE: In the case of Polygon Type 4 (Intensity, with Two Volumes), the Face Color is used in both the Base Color and the Offset Color.
            .Sprite => |p| @bitCast(Colors.fRGBA.from_packed(p.offset_color, true)),
            else => null,
        };
    }
};

fn obj_control_to_polygon_format(obj_control: ObjControl) PolygonType {
    // NOTE: See 3.7.6.2 Parameter Combinations. Some entries are duplicated to account for the fact that the value of offset doesn't matter in these cases.
    // Shadow (Ignored) - Volume - ColType (u2) - Texture - Offset - Gouraud (Ignored) - 16bit UV
    // NOTE: Offset is ignored and fixed at 0 when non-textured
    const mask: u16 = if (obj_control.texture == 0) 0b00000000_0_1_11_1_0_0_1 else 0b00000000_0_1_11_1_1_0_1;
    const masked = @as(u16, @bitCast(obj_control)) & mask;
    switch (masked) {
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0, // FIXME: uv_16bit is supposed to be invalid in this case (Non-textured 16-bit uv doesn't make sense), but Ecco the Dolphin use such polygons? Or am I just receiving bad data?
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
        else => {
            std.debug.print(termcolor.red("Unimplemented obj_control_to_polygon_format: {b:0>16}, {any}\n"), .{ masked, obj_control });
            @panic("Unimplemented");
        },
    }
}

fn polygon_format_size(polygon_format: PolygonType) u32 {
    return switch (polygon_format) {
        .PolygonType0 => @sizeOf(PolygonType0) / 4,
        .PolygonType1 => @sizeOf(PolygonType1) / 4,
        .PolygonType2 => @sizeOf(PolygonType2) / 4,
        .PolygonType3 => @sizeOf(PolygonType3) / 4,
        .PolygonType4 => @sizeOf(PolygonType4) / 4,
        .Sprite => @sizeOf(Sprite) / 4,
    };
}

const VolumeInstruction = enum(u3) {
    Normal = 0,
    InsideLastPolygon = 1,
    OutsideLastPolygon = 2,
    _,
};

const ModifierVolumeInstruction = packed struct(u32) {
    _: u27,
    culling_mode: u2,
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
};

pub const VertexParameterType = enum {
    Type0,
    Type1,
    Type2,
    Type3,
    Type4,
    Type5,
    Type6,
    Type7,
    Type8,
    Type9,
    Type10,
    Type11,
    Type12,
    Type13,
    Type14,
    SpriteType0,
    SpriteType1,
};

pub const VertexParameter = union(VertexParameterType) {
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

    pub fn tag(self: @This()) VertexParameterType {
        return switch (self) {
            .Type0 => .Type0,
            .Type1 => .Type1,
            .Type2 => .Type2,
            .Type3 => .Type3,
            .Type4 => .Type4,
            .Type5 => .Type5,
            .Type6 => .Type6,
            .Type7 => .Type7,
            .Type8 => .Type8,
            .Type9 => .Type9,
            .Type10 => .Type10,
            .Type11 => .Type11,
            .Type12 => .Type12,
            .Type13 => .Type13,
            .Type14 => .Type14,
            .SpriteType0 => .SpriteType0,
            .SpriteType1 => .SpriteType1,
        };
    }

    pub fn position(self: *const @This()) [3]f32 {
        std.debug.assert(self.tag() != .SpriteType0 and self.tag() != .SpriteType1);
        return @as([*]const f32, @alignCast(@ptrCast(self)))[1..4].*;
    }

    // NOTE: Z position of the last vertex will be garbage.
    pub fn sprite_positions(self: *const @This()) [4][3]f32 {
        std.debug.assert(self.tag() == .SpriteType0 or self.tag() == .SpriteType1);
        return @bitCast(@as([*]const f32, @alignCast(@ptrCast(self)))[1 .. 1 + 4 * 3].*);
    }
};

// Returns the size in words (4 bytes) of the vertex parameter
pub fn vertex_parameter_size(format: VertexParameterType) u32 {
    return switch (format) {
        .Type0 => @sizeOf(VertexParameter_0) / 4,
        .Type1 => @sizeOf(VertexParameter_1) / 4,
        .Type2 => @sizeOf(VertexParameter_2) / 4,
        .Type3 => @sizeOf(VertexParameter_3) / 4,
        .Type4 => @sizeOf(VertexParameter_4) / 4,
        .Type5 => @sizeOf(VertexParameter_5) / 4,
        .Type6 => @sizeOf(VertexParameter_6) / 4,
        .Type7 => @sizeOf(VertexParameter_7) / 4,
        .Type8 => @sizeOf(VertexParameter_8) / 4,
        .Type9 => @sizeOf(VertexParameter_9) / 4,
        .Type10 => @sizeOf(VertexParameter_10) / 4,
        .Type11 => @sizeOf(VertexParameter_11) / 4,
        .Type12 => @sizeOf(VertexParameter_12) / 4,
        .Type13 => @sizeOf(VertexParameter_13) / 4,
        .Type14 => @sizeOf(VertexParameter_14) / 4,
        .SpriteType0 => @sizeOf(VertexParameter_Sprite_0) / 4,
        .SpriteType1 => @sizeOf(VertexParameter_Sprite_1) / 4,
    };
}

fn obj_control_to_vertex_parameter_format(obj_control: ObjControl) VertexParameterType {
    // Shadow (Ignored) - Volume - ColType (u2) - Texture - Offset (Ignored) - Gouraud (Ignored) - 16bit UV
    const masked = @as(u16, @bitCast(obj_control)) & 0b00000000_0_1_11_1_0_0_1;
    switch (masked) {
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .PackedColor, .volume = 0, .shadow = 0 })) => return .Type0,
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 1, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .PackedColor, .volume = 0, .shadow = 0 })) => return .Type0, // FIXME: uv_16bit is supposed to be invalid here (Non-textured 16-bit uv doesn't make sense). Ecco the Dolphin does this.
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
        else => {
            std.debug.print(termcolor.red("Unimplemented obj_control_to_vertex_parameter_format: {b} {any}"), .{ masked, obj_control });
            @panic("Unimplemented");
        },
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
    polygon: Polygon,
    user_clip: ?UserTileClipInfo,
    vertex_parameter_index: usize = 0,
    vertex_parameter_count: usize = 0,
};

pub const DisplayList = struct {
    vertex_strips: std.ArrayList(VertexStrip),
    vertex_parameters: std.ArrayList(VertexParameter),
    next_first_vertex_parameters_index: usize = 0,

    pub fn init(allocator: std.mem.Allocator) DisplayList {
        return .{
            .vertex_strips = std.ArrayList(VertexStrip).init(allocator),
            .vertex_parameters = std.ArrayList(VertexParameter).init(allocator),
        };
    }

    pub fn deinit(self: *DisplayList) void {
        self.vertex_parameters.deinit();
        self.vertex_strips.deinit();
    }

    pub fn clearRetainingCapacity(self: *DisplayList) void {
        self.vertex_parameters.clearRetainingCapacity();
        self.vertex_strips.clearRetainingCapacity();
        self.next_first_vertex_parameters_index = 0;
    }
};

const ScheduledInterrupt = struct {
    cycles: u32,
    int: HardwareRegisters.SB_ISTNRM,
};

pub const TALists = struct {
    opaque_list: DisplayList,
    punchthrough_list: DisplayList,
    translucent_list: DisplayList,
    opaque_modifier_volumes: std.ArrayList(ModifierVolume),
    translucent_modifier_volumes: std.ArrayList(ModifierVolume),
    volume_triangles: std.ArrayList(ModifierVolumeParameter),

    _should_reset: bool = false,

    pub fn init(allocator: std.mem.Allocator) TALists {
        return .{
            .opaque_list = DisplayList.init(allocator),
            .punchthrough_list = DisplayList.init(allocator),
            .translucent_list = DisplayList.init(allocator),
            .opaque_modifier_volumes = std.ArrayList(ModifierVolume).init(allocator),
            .translucent_modifier_volumes = std.ArrayList(ModifierVolume).init(allocator),
            .volume_triangles = std.ArrayList(ModifierVolumeParameter).init(allocator),
        };
    }

    pub fn deinit(self: *TALists) void {
        self.opaque_list.deinit();
        self.punchthrough_list.deinit();
        self.translucent_list.deinit();
        self.opaque_modifier_volumes.deinit();
        self.translucent_modifier_volumes.deinit();
        self.volume_triangles.deinit();
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
            else => @panic("Invalid List Type"),
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
    pub fn serialize(_: @This(), writer: anytype) !usize {
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
    pub const RegistersSize = 0x2000;

    vram: []align(32) u8,
    registers: []u8,

    dirty_framebuffer: bool = false,

    _allocator: std.mem.Allocator,
    _dc: *Dreamcast,

    _ta_command_buffer: [16]u32 align(16) = .{0} ** 16,
    _ta_command_buffer_index: u32 = 0,
    _ta_list_type: ?ListType = null,
    _ta_current_polygon: ?Polygon = null,
    _ta_user_tile_clip: ?UserTileClipInfo = null,
    _ta_current_volume: ?ModifierVolume = null,
    _ta_volume_next_polygon_is_last: bool = false,

    // When starting a render, the user can select where to get the parameters from using
    // the PARAM_BASE register. It is specified in 1MB blocks, meaning it can take at most
    // 16 different values (actually 8 in the case of the base DC and its 8MB of VRAM).
    // We don't emulate the object writes to VRAM, but some games submit multiple lists concurently
    // ("double buffering" the object lists), so we have to keep track of that.
    _ta_lists: [16]TALists = undefined,

    _pixel: u32 = 0,
    _tmp_cycles: u32 = 0,

    _vblank_signal: bool = false,

    pub fn init(allocator: std.mem.Allocator, dc: *Dreamcast) !Holly {
        var r = Holly{
            .vram = dc.vram,
            .registers = try allocator.alloc(u8, RegistersSize), // FIXME: Huge waste of memory
            ._allocator = allocator,
            ._dc = dc,
        };
        for (&r._ta_lists) |*ta_list| {
            ta_list.* = TALists.init(allocator);
        }
        return r;
    }

    pub fn deinit(self: *@This()) void {
        for (&self._ta_lists) |*ta_list| {
            ta_list.deinit();
        }
        self._allocator.free(self.registers);
    }

    pub fn reset(self: *@This()) void {
        @memset(self.vram, 0);
        @memset(self.registers, 0);

        for (&self._ta_lists) |*ta_list| {
            ta_list.clearRetainingCapacity();
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
        self._get_register(u32, .VO_STARTY).* = 0x015;
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
    }

    // Cleared on read
    pub fn vblank_signal(self: *@This()) bool {
        const r = self._vblank_signal;
        self._vblank_signal = false;
        return r;
    }

    // NOTE: This is pretty heavy in benchmarks, might be worth optimizing a bit (although I'm not sure how)
    pub fn update(self: *@This(), dc: *Dreamcast, cycles: u32) void {
        self._tmp_cycles += 1000 * cycles;

        // FIXME: Move all of this to its own SPG Module ?
        const spg_hblank = self._get_register(SPG_HBLANK, .SPG_HBLANK).*;
        const spg_hblank_int = self._get_register(SPG_HBLANK_INT, .SPG_HBLANK_INT).*;
        const spg_load = self._get_register(SPG_LOAD, .SPG_LOAD).*;
        const fb_r_ctrl = self._get_register(FB_R_CTRL, .FB_R_CTRL).*;
        const cycles_per_pixel = (if (fb_r_ctrl.vclk_div == 0) @as(u32, 2) else 1) * 7407; // FIXME: Approximation. ~200/27.

        if (self._tmp_cycles >= cycles_per_pixel) {
            const start_pixel = self._pixel;
            self._pixel += self._tmp_cycles / cycles_per_pixel;
            self._tmp_cycles %= cycles_per_pixel;

            const spg_status = self._get_register(SPG_STATUS, .SPG_STATUS);

            if (start_pixel < spg_hblank.hbstart and spg_hblank.hbstart <= self._pixel) {
                spg_status.hblank = 1;
            }
            if (start_pixel < spg_hblank.hbend and spg_hblank.hbend <= self._pixel) {
                spg_status.hblank = 0;
            }
            if (start_pixel < spg_hblank_int.hblank_in_interrupt and spg_hblank_int.hblank_in_interrupt <= self._pixel) {
                switch (spg_hblank_int.hblank_int_mode) {
                    0 => {
                        // Output when the display line is the value indicated by line_comp_val.
                        if (spg_status.scanline == spg_hblank_int.line_comp_val)
                            dc.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    1 => {
                        // Output every line_comp_val lines.
                        if (spg_status.scanline % spg_hblank_int.line_comp_val == 0) // FIXME: Really?
                            dc.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    2 => {
                        // Output every line.
                        dc.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    else => {},
                }
            }

            while (self._pixel >= spg_load.hcount) {
                self._pixel -= spg_load.hcount;

                const spg_vblank = self._get_register(SPG_VBLANK, .SPG_VBLANK).*;
                const spg_vblank_int = self._get_register(SPG_VBLANK_INT, .SPG_VBLANK_INT).*;

                spg_status.scanline +%= 1;

                // If SB_MDTSEL is set, initiate Maple DMA one line before VBlankOut
                // FIXME: This has nothing to do here.
                if (dc.read_hw_register(u32, .SB_MDEN) & 1 == 1 and dc.read_hw_register(u32, .SB_MDTSEL) & 1 == 1 and @as(u11, spg_status.scanline) + 1 == spg_vblank_int.vblank_out_interrupt_line_number) {
                    // NOTE: Registers SB_MSYS and SB_MSHTCL control some behaviours here and aren't emulated.
                    dc.start_maple_dma();
                }

                if (spg_status.scanline == spg_vblank_int.vblank_in_interrupt_line_number) {
                    dc.raise_normal_interrupt(.{ .VBlankIn = 1 });
                }
                if (spg_status.scanline == spg_vblank_int.vblank_out_interrupt_line_number) {
                    dc.raise_normal_interrupt(.{ .VBlankOut = 1 });
                }
                if (spg_status.scanline == spg_vblank.vbstart) {
                    spg_status.vsync = 1;
                }
                if (spg_status.scanline == spg_vblank.vbend) {
                    spg_status.vsync = 0;
                }
                // FIXME: spg_load.vcount is sometimes < spg_vblank.vbend, locking the system in constant VSync.
                //        I don't know why yet, there's probably something I did not understand correcly,
                //        but the important for now is that all the interrupts are fired and states are reached,
                //        even if the timing is wrong.
                // NOTE: vcount: Specify "number of lines per field - 1" for the CRT; in interlace mode, specify"number of lines per field/2 - 1." (default = 0x106)
                if (spg_status.scanline >= @max(spg_load.vcount + 1, spg_vblank.vbend)) {
                    spg_status.scanline = 0;
                }

                if (spg_status.scanline == 0)
                    self._vblank_signal = true;
            }
        }
    }

    pub fn write_register(self: *@This(), addr: u32, v: u32) void {
        switch (@as(HollyRegister, @enumFromInt(addr))) {
            .ID, .REVISION => return, // Read-only
            .SOFTRESET => {
                const sr: SOFT_RESET = @bitCast(v);
                if (sr.TASoftReset == 1) {
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
                    self._ta_current_volume = null;
                    self._ta_user_tile_clip = null;
                }
                if (sr.PipelineSoftReset == 1) {
                    holly_log.debug(termcolor.yellow("  TODO: Pipeine Soft Reset"), .{});
                }
                if (sr.SDRAMInterfaceSoftReset == 1) {
                    holly_log.debug(termcolor.yellow("  TODO: SDRAM Interface Soft Reset"), .{});
                }
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
                    holly_log.debug("TA_LIST_INIT: {X:0>8}", .{v});
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
                    self._ta_current_volume = null;
                    self._ta_user_tile_clip = null;
                    self._get_register(u32, .TA_NEXT_OPB).* = self.read_register(u32, .TA_NEXT_OPB_INIT);
                    self._get_register(u32, .TA_ITP_CURRENT).* = self.read_register(u32, .TA_ISP_BASE);

                    self.ta_current_lists().mark_reset();
                }
                return;
            },
            .TA_LIST_CONT => {
                holly_log.warn("TODO TA_LIST_CONT: {X:0>8}", .{v});
                // TODO: Same thing as TA_LIST_INIT, but without reseting the list?
                if (v == 0x80000000) {
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
                    self._ta_current_volume = null;
                    self._ta_user_tile_clip = null;
                }
                return;
            },
            .TA_YUV_TEX_BASE => self._get_register(u32, .TA_YUV_TEX_CNT).* = 0,
            .SPG_CONTROL => holly_log.warn("Write to SPG_CONTROL: {X:0>8}", .{v}),
            .SPG_LOAD => holly_log.warn("Write to SPG_LOAD: {X:0>8}", .{v}),
            .FB_R_CTRL, .FB_R_SIZE, .FB_R_SOF1, .FB_R_SOF2 => self.dirty_framebuffer = true,
            else => |reg| {
                holly_log.debug("Write to Register: @{X:0>8} {s} = {X:0>8}", .{ addr, std.enums.tagName(HollyRegister, reg) orelse "Unknown", v });
            },
        }
        self._get_register_from_addr(u32, addr).* = v;
    }

    // Write to the Tile Accelerator
    pub fn write_ta(self: *@This(), addr: u32, v: u32) void {
        // std.debug.print("  TA Write: {X:0>8} = {X:0>8}\n", .{ addr, v });
        std.debug.assert(addr >= 0x10000000 and addr < 0x14000000);
        if (addr >= 0x10000000 and addr < 0x10800000 or addr >= 0x12000000 and addr < 0x12800000) {
            // Commands
            self._ta_command_buffer[self._ta_command_buffer_index] = v;
            self._ta_command_buffer_index += 1;
            self.handle_command();
        } else if (addr >= 0x10800000 and addr < 0x11000000 or addr >= 0x12800000 and addr < 0x13000000) {
            // YUV Conv.
            holly_log.warn(termcolor.yellow("  TODO: YUV Conv. {X:0>8} = {X:0>8}"), .{ addr, v });
        } else if (addr >= 0x11000000 and addr < 0x12000000 or addr >= 0x13000000 and addr < 0x14000000) {
            // Direct Texture Path
            if (addr & 0x00FFFFFF > 0x00800000) {
                holly_log.warn(termcolor.yellow("  Direct Texture Path write out of bounds? {X:0>8} = {X:0>8}"), .{ addr, v });
                return;
            }
            @as(*u32, @alignCast(@ptrCast(&self.vram[addr & 0x00FFFFFF]))).* = v;
        }
    }

    pub fn write_ta_fifo_polygon_path(self: *@This(), v: []u32) void {
        std.debug.assert(v.len == 8);
        std.debug.assert(self._ta_command_buffer_index % 8 == 0);

        @memcpy(self._ta_command_buffer[self._ta_command_buffer_index .. self._ta_command_buffer_index + 8], v);

        self._ta_command_buffer_index += 8;

        self.handle_command();
    }

    fn ta_list_index(self: *const @This()) u4 {
        // Should I record the value of TA_ISP_BASE on LIST_INIT?
        // NOTE: We also assume that TA_OL_BASE is in the same 1MB range here.
        return @truncate((self.read_register(u32, .TA_ISP_BASE) >> 20) & 0xF);
    }

    fn ta_current_lists(self: *@This()) *TALists {
        return &self._ta_lists[self.ta_list_index()];
    }

    pub fn handle_command(self: *@This()) void {
        if (self._ta_command_buffer_index % 8 != 0) return; // All commands are 8 or 16 bytes long

        const parameter_control_word: ParameterControlWord = @bitCast(self._ta_command_buffer[0]);

        holly_log.debug(" TA Parameter Type: {any}", .{parameter_control_word.parameter_type});
        for (0..self._ta_command_buffer_index) |i| {
            holly_log.debug("      {X:0>8}", .{self._ta_command_buffer[i]});
        }

        switch (parameter_control_word.parameter_type) {
            // Control Parameters
            .EndOfList => {
                self.check_end_of_modifier_volume();
                if (self._ta_list_type) |list| { // Apprently this happens?... Why would a game do this?
                    // Fire corresponding interrupt. FIXME: Delay is completely arbitrary, I just need to delay them for testing, for now.
                    self._dc.schedule_interrupt(switch (list) {
                        .Opaque => .{ .EoT_OpaqueList = 1 },
                        .OpaqueModifierVolume => .{ .EoT_OpaqueModifierVolumeList = 1 },
                        .Translucent => .{ .EoT_TranslucentList = 1 },
                        .TranslucentModifierVolume => .{ .EoT_TranslucentModifierVolumeList = 1 },
                        .PunchThrough => .{ .EoD_PunchThroughList = 1 },
                        else => {
                            holly_log.err(termcolor.red("  Unimplemented List Type {any}"), .{list});
                            @panic("Unimplemented List Type");
                        },
                    }, 800);
                }
                self._ta_current_polygon = null;
                self._ta_list_type = null;
            },
            .UserTileClip => {
                const user_tile_clip = @as(*const UserTileClip, @ptrCast(&self._ta_command_buffer)).*;
                self._ta_user_tile_clip = .{
                    .usage = .Disable,
                    .x = 32 * user_tile_clip.user_clip_x_min,
                    .y = 32 * user_tile_clip.user_clip_y_min,
                    .width = 32 * (1 + user_tile_clip.user_clip_x_max - user_tile_clip.user_clip_x_min),
                    .height = 32 * (1 + user_tile_clip.user_clip_y_max - user_tile_clip.user_clip_y_min),
                };
            },
            .ObjectListSet => {
                self.handle_object_list_set();
            },
            // Global Parameters
            .PolygonOrModifierVolume => {
                if (self._ta_list_type == null) {
                    self._ta_list_type = parameter_control_word.list_type;
                    self.ta_current_lists().check_reset();
                }
                // NOTE: I have no idea if this is actually an issue, or if it is just ignored when we've already started a list (and thus set the list type).
                //       But I'm leaning towards "This value is valid in the following four cases" means it's ignored in the others.
                // if (self._ta_list_type != parameter_control_word.list_type) {
                //     holly_log.err(termcolor.red("  PolygonOrModifierVolume list type mismatch: Expected {any}, got {any}"), .{ self._ta_list_type, parameter_control_word.list_type });
                // }

                if (parameter_control_word.group_control.en == 1) {
                    if (self._ta_user_tile_clip) |*uc| {
                        uc.usage = parameter_control_word.group_control.user_clip;
                    }
                }

                if (self._ta_list_type.? == .OpaqueModifierVolume or self._ta_list_type.? == .TranslucentModifierVolume) {
                    const modifier_volume = @as(*ModifierVolumeGlobalParameter, @ptrCast(&self._ta_command_buffer));
                    if (self._ta_current_volume == null and modifier_volume.instructions.volume_instruction == .Normal) {
                        self._ta_current_volume = .{
                            .parameter_control_word = modifier_volume.*.parameter_control_word,
                            .instructions = modifier_volume.*.instructions,
                            .first_triangle_index = @intCast(self.ta_current_lists().volume_triangles.items.len),
                            .closed = modifier_volume.parameter_control_word.obj_control.volume == 1,
                        };
                    } else if (modifier_volume.instructions.volume_instruction == .InsideLastPolygon or modifier_volume.instructions.volume_instruction == .OutsideLastPolygon) {
                        if (modifier_volume.*.instructions.volume_instruction == .OutsideLastPolygon) {
                            holly_log.warn("Unsupported Exclusion Modifier Volume.", .{});
                        }
                        if (self._ta_current_volume) |*cv| {
                            self._ta_volume_next_polygon_is_last = true;
                            cv.instructions = modifier_volume.*.instructions;
                            cv.closed = modifier_volume.parameter_control_word.obj_control.volume == 1;
                        }
                    }
                } else {
                    // NOTE: "Four bits in the ISP/TSP Instruction Word are overwritten with the corresponding bit values from the Parameter Control Word."
                    const global_parameter = @as(*GenericGlobalParameter, @ptrCast(&self._ta_command_buffer));
                    global_parameter.*.isp_tsp_instruction.texture = global_parameter.*.parameter_control_word.obj_control.texture;
                    global_parameter.*.isp_tsp_instruction.offset = global_parameter.*.parameter_control_word.obj_control.offset;
                    global_parameter.*.isp_tsp_instruction.gouraud = global_parameter.*.parameter_control_word.obj_control.gouraud;
                    global_parameter.*.isp_tsp_instruction.uv_16bit = global_parameter.*.parameter_control_word.obj_control.uv_16bit;

                    const format = obj_control_to_polygon_format(parameter_control_word.obj_control);
                    if (self._ta_command_buffer_index < polygon_format_size(format)) return;

                    self._ta_current_polygon = switch (format) {
                        .PolygonType0 => .{ .PolygonType0 = @as(*PolygonType0, @ptrCast(&self._ta_command_buffer)).* },
                        .PolygonType1 => .{ .PolygonType1 = @as(*PolygonType1, @ptrCast(&self._ta_command_buffer)).* },
                        .PolygonType2 => .{ .PolygonType2 = @as(*PolygonType2, @ptrCast(&self._ta_command_buffer)).* },
                        .PolygonType3 => .{ .PolygonType3 = @as(*PolygonType3, @ptrCast(&self._ta_command_buffer)).* },
                        .PolygonType4 => .{ .PolygonType4 = @as(*PolygonType4, @ptrCast(&self._ta_command_buffer)).* },
                        else => @panic("Invalid polygon format"),
                    };
                }
            },
            .SpriteList => {
                if (self._ta_list_type == null) {
                    self._ta_list_type = parameter_control_word.list_type;
                    self.ta_current_lists().check_reset();
                }

                if (parameter_control_word.group_control.en == 1) {
                    if (self._ta_user_tile_clip) |*uc| {
                        uc.usage = parameter_control_word.group_control.user_clip;
                    }
                }

                self._ta_current_polygon = .{ .Sprite = @as(*Sprite, @ptrCast(&self._ta_command_buffer)).* };
            },
            // VertexParameter - Yes it's a category of its own.
            .VertexParameter => {
                if (self._ta_list_type) |list_type| {
                    if (list_type == .OpaqueModifierVolume or list_type == .TranslucentModifierVolume) {
                        if (self._ta_command_buffer_index < @sizeOf(ModifierVolumeParameter) / 4) return;
                        self.ta_current_lists().volume_triangles.append(@as(*ModifierVolumeParameter, @ptrCast(&self._ta_command_buffer)).*) catch unreachable;

                        if (self._ta_volume_next_polygon_is_last) {
                            self.check_end_of_modifier_volume();
                        }
                    } else {
                        var display_list = self.ta_current_lists().get_list(list_type);
                        if (self._ta_current_polygon) |*polygon| {
                            const polygon_obj_control = @as(*const GenericGlobalParameter, @ptrCast(polygon)).*.parameter_control_word.obj_control;
                            switch (polygon.*) {
                                .Sprite => {
                                    if (parameter_control_word.end_of_strip != 1) { // Sanity check: For Sprites/Quads, each vertex parameter describes an entire polygon.
                                        holly_log.warn(termcolor.yellow("Unexpected Sprite without end of strip bit:") ++ "\n  {any}", .{parameter_control_word});
                                    }
                                    if (polygon_obj_control.texture == 0) {
                                        if (self._ta_command_buffer_index < vertex_parameter_size(.SpriteType0)) return;
                                        display_list.vertex_parameters.append(.{ .SpriteType0 = @as(*VertexParameter_Sprite_0, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                                    } else {
                                        if (self._ta_command_buffer_index < vertex_parameter_size(.SpriteType1)) return;
                                        display_list.vertex_parameters.append(.{ .SpriteType1 = @as(*VertexParameter_Sprite_1, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                                    }
                                },
                                else => {
                                    const format = obj_control_to_vertex_parameter_format(polygon_obj_control);
                                    if (self._ta_command_buffer_index < vertex_parameter_size(format)) return;

                                    display_list.vertex_parameters.append(switch (format) {
                                        .Type0 => .{ .Type0 = @as(*VertexParameter_0, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type1 => .{ .Type1 = @as(*VertexParameter_1, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type2 => .{ .Type2 = @as(*VertexParameter_2, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type3 => .{ .Type3 = @as(*VertexParameter_3, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type4 => .{ .Type4 = @as(*VertexParameter_4, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type5 => .{ .Type5 = @as(*VertexParameter_5, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type6 => .{ .Type6 = @as(*VertexParameter_6, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type7 => .{ .Type7 = @as(*VertexParameter_7, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type8 => .{ .Type8 = @as(*VertexParameter_8, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type9 => .{ .Type9 = @as(*VertexParameter_9, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type10 => .{ .Type10 = @as(*VertexParameter_10, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type11 => .{ .Type11 = @as(*VertexParameter_11, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type12 => .{ .Type12 = @as(*VertexParameter_12, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type13 => .{ .Type13 = @as(*VertexParameter_13, @ptrCast(&self._ta_command_buffer)).* },
                                        .Type14 => .{ .Type14 = @as(*VertexParameter_14, @ptrCast(&self._ta_command_buffer)).* },
                                        else => {
                                            holly_log.err(termcolor.red("  Unexpected vertex parameter type: {any}."), .{format});
                                            @panic("Unexpected vertex parameter type");
                                        },
                                    }) catch unreachable;
                                },
                            }

                            if (parameter_control_word.end_of_strip == 1) {
                                display_list.vertex_strips.append(.{
                                    .polygon = polygon.*,
                                    .user_clip = if (self._ta_user_tile_clip) |uc| if (uc.usage != .Disable) uc else null else null,
                                    .vertex_parameter_index = display_list.next_first_vertex_parameters_index,
                                    .vertex_parameter_count = display_list.vertex_parameters.items.len - display_list.next_first_vertex_parameters_index,
                                }) catch unreachable;

                                display_list.next_first_vertex_parameters_index = display_list.vertex_parameters.items.len;
                            }
                        } else {
                            holly_log.err(termcolor.red("    No current polygon! Current list type: {s}"), .{@tagName(list_type)});
                            @panic("No current polygon");
                        }
                    }
                } else {
                    holly_log.err(termcolor.red("Received VertexParameter without an active list!"), .{});
                }
            },
            _ => {
                holly_log.err(termcolor.red("    Invalid parameter type: {d}."), .{parameter_control_word.parameter_type});
                @panic("Invalid parameter type");
            },
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
                        const config = self.get_region_array_data_config();
                        if (@as(u32, @bitCast(config.opaque_modifier_volume_pointer)) == @as(u32, @bitCast(config.translucent_modifier_volume_pointer))) {
                            // Both lists are actually the same, we'll add it twice for convience.
                            self.ta_current_lists().opaque_modifier_volumes.append(volume.*) catch unreachable;
                            self.ta_current_lists().translucent_modifier_volumes.append(volume.*) catch unreachable;
                        } else {
                            (if (list_type == .OpaqueModifierVolume)
                                self.ta_current_lists().opaque_modifier_volumes
                            else
                                self.ta_current_lists().translucent_modifier_volumes).append(volume.*) catch unreachable;
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

    fn handle_object_list_set(self: *@This()) void {
        const parameter_control_word: ParameterControlWord = @bitCast(self._ta_command_buffer[0]);
        std.debug.assert(parameter_control_word.parameter_type == .ObjectListSet);

        if (self._ta_list_type == null) {
            self._ta_list_type = parameter_control_word.list_type;
            self.ta_current_lists().check_reset();
        }
        holly_log.err(termcolor.red("  Unimplemented ObjectListSet"), .{});
        // FIXME: Really not sure if I need to do any thing here...
        //        Is it meant to separate objects by tiles? Are they already submitted elsewhere anyway?
        if (false) {
            const object_list_set = @as(*ObjectListSet, @ptrCast(&self._ta_command_buffer)).*;
            const param_base = self._get_register(u32, .PARAM_BASE).*;
            const ta_alloc_ctrl = self._get_register(TA_ALLOC_CTRL, .TA_ALLOC_CTRL).*;
            std.debug.assert(ta_alloc_ctrl.OPB_Mode == 0);
            const addr = 4 * object_list_set.object_pointer; // 32bit word address
            while (true) {
                const object = @as(*u32, @ptrCast(&self.vram[addr])).*;
                if (object & 0x80000000 == 0) {
                    // Triangle Strip
                    const strip_addr = param_base + 4 * (object & 0x1FFFFF);
                    _ = strip_addr;
                } else {
                    switch ((object >> 29) & 0b11) {
                        0b00 => {
                            // Triangle Array
                            @panic("Unimplemented Triangle Array");
                        },
                        0b01 => {
                            // Quad Array
                            @panic("Unimplemented Quad Array");
                        },
                        0b11 => {
                            std.debug.assert(object & 0b11 == 0);
                            // Object Pointer Block Link
                            if (object & 0x10000000 == 0x10000000) {
                                // End of list
                                break;
                            } else {
                                @panic("Unimplemented Object Pointer Block Link");
                            }
                        },
                        else => {
                            @panic("Invalid Object type");
                        },
                    }
                }
                addr += 4;
            }
        }
    }

    pub fn ta_fifo_yuv_converter_path(self: *@This(), data: []u8) void {
        const tex_base = self._get_register(u32, .TA_YUV_TEX_BASE).*;
        const ctrl = self._get_register(TA_YUV_TEX_CTRL, .TA_YUV_TEX_CTRL).*;
        const u_size = @as(u32, ctrl.u_size) + 1; // In 16x16 blocks
        const v_size = @as(u32, ctrl.v_size) + 1; // In 16x16 blocks
        var tex: [*]YUV422 = @alignCast(@ptrCast(&self.vram[tex_base]));
        const line_size = u_size * 16 / 2; // in YUV422
        if (ctrl.format == 0) {
            // YUV 420
            if (ctrl.tex == 0) {
                //   YUV_Tex = 0
                // The YUV data that is input is stored in texture memory as one texture with a size of
                // [(YUV_U_Size + 1) * 16] (H) * [(YUV_V_Size + 1) * 16] (V). This format has a weakness
                // in that storage time is longer because the storage addresses in texture memory will
                // not be continuous every 16 pixels (32 bytes) in the horizontal direction.
                var offset: u32 = 0;
                for (0..v_size) |y| {
                    for (0..u_size) |x| {
                        const block_start = (16 * y * line_size) + (16 / 2 * x);
                        const pixels = tex[block_start..];

                        const u = data[offset .. offset + 64];
                        offset += 64;
                        const v = data[offset .. offset + 64];
                        offset += 64;
                        const y0 = data[offset .. offset + 64]; // (0,0) to (7,7)
                        offset += 64;
                        const y1 = data[offset .. offset + 64]; // (8,0) to (15,7)
                        offset += 64;
                        const y2 = data[offset .. offset + 64]; // (0,8) to (7,15)
                        offset += 64;
                        const y3 = data[offset .. offset + 64]; // (8,8) to (15,15)
                        offset += 64;

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
                @panic("ta_fifo_yuv_converter_path: Unimplemented tex=1");
            }
        } else {
            @panic("ta_fifo_yuv_converter_path: Unimplemented format");
        }
    }

    pub fn write_ta_fifo_direct_texture_path(self: *@This(), addr: u32, value: []u8) void {
        holly_log.debug("  NOTE: DMA to Direct Texture Path to {X:0>8} (len: {X:0>8})", .{ addr, value.len });
        @memcpy(self.vram[addr & 0x00FFFFFF .. (addr & 0x00FFFFFF) + value.len], value);
    }

    pub inline fn read_register(self: *const @This(), comptime T: type, r: HollyRegister) T {
        return @constCast(self)._get_register_from_addr(T, @intFromEnum(r)).*;
    }

    pub inline fn _get_register(self: *@This(), comptime T: type, r: HollyRegister) *T {
        return self._get_register_from_addr(T, @intFromEnum(r));
    }

    pub inline fn _get_register_from_addr(self: *@This(), comptime T: type, addr: u32) *T {
        std.debug.assert(addr >= HollyRegisterStart and addr < HollyRegisterStart + self.registers.len);
        return @as(*T, @alignCast(@ptrCast(&self.registers[addr - HollyRegisterStart])));
    }

    pub inline fn get_palette(self: *const @This()) []const u32 {
        return @as([*]const u32, @alignCast(@ptrCast(&self.registers[@intFromEnum(HollyRegister.PALETTE_RAM_START) - HollyRegisterStart])))[0..1024];
    }

    pub inline fn get_fog_table(self: *const @This()) []const u32 {
        return @as([*]const u32, @alignCast(@ptrCast(&self.registers[@intFromEnum(HollyRegister.FOG_TABLE_START) - HollyRegisterStart])))[0..0x80];
    }

    pub inline fn get_region_array_data_config(self: *const @This()) RegionArrayDataConfiguration {
        // Sadly we can't just return a pointer to the RegionArrayDataConfiguration directly in VRAM because of alignment.
        const r: [*]u32 = @alignCast(@ptrCast(&self.vram[@constCast(self)._get_register(u32, .REGION_BASE).*]));
        return .{
            .settings = @bitCast(r[0]),
            .opaque_list_pointer = @bitCast(r[1]),
            .opaque_modifier_volume_pointer = @bitCast(r[2]),
            .translucent_list_pointer = @bitCast(r[3]),
            .translucent_modifier_volume_pointer = @bitCast(r[4]),
            .punch_through_list_pointer = @bitCast(r[5]),
        };
    }

    fn check_framebuffer_write(self: *@This(), addr: u32) void {
        if (self.dirty_framebuffer) return;

        const mask: u32 = 0x007FFFFF;
        const local_addr = addr & mask;

        const spg_control = self.read_register(SPG_CONTROL, .SPG_CONTROL);
        const fb1_start_addr = self.read_register(u32, .FB_R_SOF1) & mask;
        const fb2_start_addr = self.read_register(u32, .FB_R_SOF2) & mask;
        const fb_r_size = self.read_register(FB_R_SIZE, .FB_R_SIZE);
        const line_size: u32 = 4 * (@as(u32, fb_r_size.x_size) + @as(u32, fb_r_size.modulus)); // From 32-bit units to bytes.
        const line_count: u32 = @as(u32, fb_r_size.y_size) + 1; // Number of lines
        const interlaced = spg_control.interlace == 1;
        const fb1_end_addr = fb1_start_addr + line_count * line_size;
        const fb2_end_addr = fb2_start_addr + line_count * line_size;

        if ((local_addr >= fb1_start_addr and local_addr < fb1_end_addr) or
            (interlaced and (local_addr >= fb2_start_addr and local_addr < fb2_end_addr)))
        {
            self.dirty_framebuffer = true;
        }
    }

    pub inline fn write_vram(self: *@This(), comptime T: type, addr: u32, value: T) void {
        self.check_framebuffer_write(addr);

        @as(*T, @alignCast(@ptrCast(
            self._get_vram(addr),
        ))).* = value;
    }

    pub fn write_framebuffer(self: *@This(), pixels: []const u8) void {
        // TODO: Absolutely not done.
        const scaler_ctl = self._get_register(SCALER_CTL, .SCALER_CTL).*;
        const w_ctrl = self._get_register(FB_W_CTRL, .FB_W_CTRL).*;
        const x_clip = self._get_register(FB_CLIP, .FB_X_CLIP).*;
        const y_clip = self._get_register(FB_CLIP, .FB_Y_CLIP).*;
        const video_out_ctrl = self._get_register(VO_CONTROL, .VO_CONTROL).*;

        _ = scaler_ctl;
        _ = video_out_ctrl;

        const interlaced = false; // TODO: Support interlacing?
        const field = if (interlaced and false) 1 else 0; // TODO
        const FB_W_SOF = self._get_register(u32, if (field == 0) .FB_W_SOF1 else .FB_W_SOF2).*; // TODO: Support interlacing?
        if (FB_W_SOF & 0x1000000 != 0) {
            std.log.warn(termcolor.yellow("TODO: Write to texture! FB_W_SOF:{X:0>8}"), .{FB_W_SOF});
            return;
        }

        const resolution = struct {
            const width = 640;
            const height = 480;
        };

        const FB_W_LINESTRIDE = 8 * (self._get_register(u32, .FB_W_LINESTRIDE).* & 0x1FF);
        const line_offset = field; // TODO: Support interlacing?
        const height = resolution.height / 2; // When?
        for (y_clip.min..@min(height, y_clip.max)) |y| {
            for (x_clip.min..@min(resolution.width, x_clip.max)) |x| {
                const idx = ((2 * y + line_offset) * resolution.width + x) * 4;
                switch (w_ctrl.fb_packmode) {
                    .RGB565 => {
                        const addr = FB_W_SOF + y * FB_W_LINESTRIDE + 2 * x;
                        var pixel: *Colors.Color16 = @alignCast(@ptrCast(&self.vram[addr]));
                        pixel.rgb565.r = @truncate(pixels[idx + 2] >> 3);
                        pixel.rgb565.g = @truncate(pixels[idx + 1] >> 2);
                        pixel.rgb565.b = @truncate(pixels[idx + 0] >> 3);
                    },
                    .RGB888 => {
                        const addr = FB_W_SOF + y * FB_W_LINESTRIDE + 3 * x;
                        self.vram[addr + 0] = pixels[idx + 2];
                        self.vram[addr + 1] = pixels[idx + 1];
                        self.vram[addr + 2] = pixels[idx + 0];
                    },
                    else => {
                        std.log.warn("TODO: {}", .{w_ctrl.fb_packmode});
                    },
                }
            }
        }
    }

    pub inline fn _get_vram(self: *@This(), addr: u32) *u8 {
        // VRAM - 8MB, Mirrored at 0x06000000
        const local_addr = addr - (if (addr >= 0x06000000) @as(u32, 0x06000000) else 0x04000000);

        if (local_addr < 0x0080_0000) { // 64-bit access area
            return &self.vram[local_addr];
        } else if (local_addr < 0x0100_0000) { // Unused
            holly_log.err(termcolor.red(" Out of bounds access to Area 1 (VRAM): {X:0>8}"), .{addr});
            @panic("Out of bounds access to Area 1 (VRAM)");
        } else if (local_addr < 0x0180_0000) { // 32-bit access area
            return &self.vram[local_addr - 0x0100_0000];
        } else { // Unused
            holly_log.err(termcolor.red(" Out of bounds access to Area 1 (VRAM): {X:0>8}"), .{addr});
            @panic("Out of bounds access to Area 1 (VRAM)");
        }
    }

    pub fn serialize(self: *const @This(), writer: anytype) !usize {
        var bytes: usize = 0;
        bytes += try writer.write(std.mem.sliceAsBytes(self.vram[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self.registers[0..]));
        bytes += try writer.write(std.mem.asBytes(&self.dirty_framebuffer));
        bytes += try writer.write(std.mem.sliceAsBytes(self._ta_command_buffer[0..]));
        bytes += try writer.write(std.mem.asBytes(&self._ta_command_buffer_index));
        bytes += try writer.write(std.mem.asBytes(&self._ta_list_type));
        bytes += try writer.write(std.mem.asBytes(&self._ta_current_polygon));
        bytes += try writer.write(std.mem.asBytes(&self._ta_user_tile_clip));
        bytes += try writer.write(std.mem.asBytes(&self._ta_current_volume));
        bytes += try writer.write(std.mem.asBytes(&self._ta_volume_next_polygon_is_last));
        for (self._ta_lists) |list| {
            bytes += try list.serialize(writer);
        }
        bytes += try writer.write(std.mem.asBytes(&self._pixel));
        bytes += try writer.write(std.mem.asBytes(&self._tmp_cycles));
        bytes += try writer.write(std.mem.asBytes(&self._vblank_signal));
        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: anytype) !usize {
        var bytes: usize = 0;
        bytes += try reader.read(std.mem.sliceAsBytes(self.vram[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self.registers[0..]));
        bytes += try reader.read(std.mem.asBytes(&self.dirty_framebuffer));
        bytes += try reader.read(std.mem.sliceAsBytes(self._ta_command_buffer[0..]));
        bytes += try reader.read(std.mem.asBytes(&self._ta_command_buffer_index));
        bytes += try reader.read(std.mem.asBytes(&self._ta_list_type));
        bytes += try reader.read(std.mem.asBytes(&self._ta_current_polygon));
        bytes += try reader.read(std.mem.asBytes(&self._ta_user_tile_clip));
        bytes += try reader.read(std.mem.asBytes(&self._ta_current_volume));
        bytes += try reader.read(std.mem.asBytes(&self._ta_volume_next_polygon_is_last));
        for (&self._ta_lists) |*list| {
            bytes += try list.deserialize(reader);
        }
        bytes += try reader.read(std.mem.asBytes(&self._pixel));
        bytes += try reader.read(std.mem.asBytes(&self._tmp_cycles));
        bytes += try reader.read(std.mem.asBytes(&self._vblank_signal));
        return bytes;
    }
};
