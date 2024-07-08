const std = @import("std");

const Dreamcast = @import("dreamcast.zig").Dreamcast;

const holly_log = std.log.scoped(.holly);

const HardwareRegisters = @import("hardware_registers.zig");
const HardwareRegister = HardwareRegisters.HardwareRegister;

const termcolor = @import("termcolor");

pub const Color16 = packed union {
    value: u16,
    arbg1555: packed struct(u16) {
        b: u5,
        g: u5,
        r: u5,
        a: u1,
    },
    rgb565: packed struct(u16) {
        b: u5,
        g: u6,
        r: u5,
    },
    argb4444: packed struct(u16) {
        b: u4,
        g: u4,
        r: u4,
        a: u4,
    },
};

pub const YUV422 = packed struct(u32) {
    u: u8,
    y0: u8,
    v: u8,
    y1: u8,
};

pub const YUV420 = packed struct(u32) {
    u: u8,
    y0: u8,
    v: u8,
    y1: u8,
};

pub const RGBA = packed struct(u32) {
    a: u8,
    b: u8,
    g: u8,
    r: u8,
};

// Expects u and v to already be shifted, then, per the documentation:
//   R = Y + (11/8) × (V-128)
//   G = Y - 0.25 × (11/8) × (U-128) - 0.5 × (11/8) × (V-128)
//   B = Y + 1.25 × (11/8) × (U-128)
//   α= 255
inline fn _yuv(y: f32, u: f32, v: f32) RGBA {
    return .{
        .r = @intFromFloat(std.math.clamp(y + (11.0 / 8.0) * v, 0.0, 255.0)),
        .g = @intFromFloat(std.math.clamp(y - 0.25 * (11.0 / 8.0) * u - 0.5 * (11.0 / 8.0) * v, 0.0, 255.0)),
        .b = @intFromFloat(std.math.clamp(y + 1.25 * (11.0 / 8.0) * u, 0.0, 255.0)),
        .a = 255,
    };
}

pub fn yuv_to_rgba(yuv: YUV422) [2]RGBA {
    const v = @as(f32, @floatFromInt(yuv.v)) - 128.0;
    const u = @as(f32, @floatFromInt(yuv.u)) - 128.0;
    const y0: f32 = @floatFromInt(yuv.y0);
    const y1: f32 = @floatFromInt(yuv.y1);
    return .{ _yuv(y0, u, v), _yuv(y1, u, v) };
}

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

pub const FB_W_CTRL = packed struct(u32) {
    fb_packmode: u3,
    fb_dither: u1,
    _0: u4,
    fb_kval: u8,
    fb_alpha_threshold: u8,
    _1: u8,
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
    filter_mode: u2,
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
    face_color_a: f32,
    face_color_r: f32,
    face_color_g: f32,
    face_color_b: f32,
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
    face_color_a: f32,
    face_color_r: f32,
    face_color_g: f32,
    face_color_b: f32,
    face_offset_color_a: f32,
    face_offset_color_r: f32,
    face_offset_color_g: f32,
    face_offset_color_b: f32,
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
    face_color_a: f32,
    face_color_r: f32,
    face_color_g: f32,
    face_color_b: f32,
    face_offset_color_a: f32,
    face_offset_color_r: f32,
    face_offset_color_g: f32,
    face_offset_color_b: f32,
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
};

fn obj_control_to_polygon_format(obj_control: ObjControl) PolygonType {
    // NOTE: See 3.7.6.2 Parameter Combinations. Some entries are duplicated to account for the fact that the value of offset doesn't matter in these cases.
    // Shadow (Ignored) - Volume - ColType (u2) - Texture - Offset - Gouraud (Ignored) - 16bit UV
    const masked = @as(u16, @bitCast(obj_control)) & 0b00000000_0_1_11_1_1_0_1;
    switch (masked) {
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0,
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 0, .gouraud = 0, .uv_16bit = 1, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0, // FIXME: uv_16bit is supposed to be invalid in this case (Non-textured 16-bit uv doesn't make sense), but Ecco the Dolphin use such polygons? Or am I just receiving bad data?
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 1, .gouraud = 0, .uv_16bit = 0, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0, // FIXME: Same thing, offset should be fixed to 0 for non-textured polygons. (Hydro Thunder does this)
        @as(u16, @bitCast(ObjControl{ .volume = 0, .texture = 0, .offset = 1, .gouraud = 0, .uv_16bit = 1, .col_type = .PackedColor, .shadow = 0 })) => return .PolygonType0, // FIXME: Same thing, offset should be fixed to 0 for non-textured polygons.
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
};

pub const PackedColor = packed struct(u32) {
    b: u8,
    g: u8,
    r: u8,
    a: u8,
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
    a: f32,
    r: f32,
    g: f32,
    b: f32,
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
    base_a: f32,
    base_r: f32,
    base_g: f32,
    base_b: f32,
    offset_a: f32,
    offset_r: f32,
    offset_g: f32,
    offset_b: f32,
};
// Floating Color, Textured 16bit UV
const VertexParameter_6 = packed struct(u512) {
    parameter_control_word: ParameterControlWord,
    x: f32,
    y: f32,
    z: f32,
    uv: UV16,
    _ignored: u96,
    base_a: f32,
    base_r: f32,
    base_g: f32,
    base_b: f32,
    offset_a: f32,
    offset_r: f32,
    offset_g: f32,
    offset_b: f32,
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
    base_0: PackedColor,
    base_1: PackedColor,
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

const VertexStrip = struct {
    polygon: Polygon,
    user_clip: ?UserTileClipInfo,
    verter_parameter_index: usize = 0,
    verter_parameter_count: usize = 0,
};

pub const DisplayList = struct {
    vertex_strips: std.ArrayList(VertexStrip) = undefined,
    vertex_parameters: std.ArrayList(VertexParameter) = undefined,
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

    pub fn reset(self: *DisplayList) void {
        self.vertex_parameters.clearRetainingCapacity();
        self.vertex_strips.clearRetainingCapacity();
        self.next_first_vertex_parameters_index = 0;
    }
};

const ScheduledInterrupt = struct {
    cycles: u32,
    int: HardwareRegisters.SB_ISTNRM,
};

pub const Holly = struct {
    vram: []u8 = undefined,
    registers: []u8 = undefined,

    render_start: bool = false, // Signals to start rendering. TODO: Find a better way to start rendering (and run the CPU on another thread I guess).

    _allocator: std.mem.Allocator = undefined,
    _dc: *Dreamcast = undefined,

    _ta_command_buffer: [16]u32 align(16) = .{0} ** 16,
    _ta_command_buffer_index: u32 = 0,
    _ta_list_type: ?ListType = null,

    _ta_current_polygon: ?Polygon = null,
    _ta_user_tile_clip: ?UserTileClipInfo = null,
    _ta_current_volume: ?ModifierVolume = null,
    _ta_opaque_modifier_volumes: std.ArrayList(ModifierVolume),
    _ta_translucent_modifier_volumes: std.ArrayList(ModifierVolume),
    _ta_volume_triangles: std.ArrayList(ModifierVolumeParameter),

    ta_display_lists: [5]DisplayList = undefined,

    _pixel: u32 = 0,
    _tmp_cycles: u32 = 0,

    pub fn init(allocator: std.mem.Allocator, dc: *Dreamcast) !Holly {
        var holly = @This(){
            ._allocator = allocator,
            ._dc = dc,
            .vram = try allocator.alloc(u8, 8 * 1024 * 1024),
            .registers = try allocator.alloc(u8, 0x2000), // FIXME: Huge waste of memory
            ._ta_opaque_modifier_volumes = std.ArrayList(ModifierVolume).init(allocator),
            ._ta_translucent_modifier_volumes = std.ArrayList(ModifierVolume).init(allocator),
            ._ta_volume_triangles = std.ArrayList(ModifierVolumeParameter).init(allocator),
        };
        for (0..holly.ta_display_lists.len) |i| {
            holly.ta_display_lists[i] = DisplayList.init(allocator);
        }

        return holly;
    }

    pub fn deinit(self: *@This()) void {
        for (&self.ta_display_lists) |*display_list| {
            display_list.deinit();
        }

        self._ta_opaque_modifier_volumes.deinit();
        self._ta_translucent_modifier_volumes.deinit();
        self._ta_volume_triangles.deinit();
        self._allocator.free(self.registers);
        self._allocator.free(self.vram);
    }

    pub fn reset(self: *@This()) void {
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

    // NOTE: This is pretty heavy in benchmarks, might be worth optimizing a bit (although I'm not sure how)
    pub fn update(self: *@This(), dc: *Dreamcast, cycles: u32) void {
        self._tmp_cycles += cycles;

        // FIXME: Move all of this to its own SPG Module ?
        const spg_hblank = self._get_register(SPG_HBLANK, .SPG_HBLANK).*;
        const spg_hblank_int = self._get_register(SPG_HBLANK_INT, .SPG_HBLANK_INT).*;
        const spg_load = self._get_register(SPG_LOAD, .SPG_LOAD).*;
        const cycles_per_pixel = 7; // FIXME: Approximation. ~200/27.

        if (self._tmp_cycles >= cycles_per_pixel) {
            const start_pixel = self._pixel;
            self._pixel += self._tmp_cycles / cycles_per_pixel;
            self._tmp_cycles %= cycles_per_pixel;

            const spg_status = self._get_register(SPG_STATUS, .SPG_STATUS);

            if (start_pixel < spg_hblank.hbstart and spg_hblank.hbstart <= self._pixel) {
                spg_status.*.hblank = 1;
            }
            if (start_pixel < spg_hblank.hbend and spg_hblank.hbend <= self._pixel) {
                spg_status.*.hblank = 0;
            }
            if (start_pixel < spg_hblank_int.hblank_in_interrupt and spg_hblank_int.hblank_in_interrupt <= self._pixel) {
                switch (spg_hblank_int.hblank_int_mode) {
                    0 => {
                        // Output when the display line is the value indicated by line_comp_val.
                        if (spg_status.*.scanline == spg_hblank_int.line_comp_val)
                            dc.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    1 => {
                        // Output every line_comp_val lines.
                        if (spg_status.*.scanline % spg_hblank_int.line_comp_val == 0) // FIXME: Really?
                            dc.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    2 => {
                        // Output every line.
                        dc.raise_normal_interrupt(.{ .HBlankIn = 1 });
                    },
                    else => {},
                }
            }

            if (self._pixel >= spg_load.hcount) {
                self._pixel = 0;

                const spg_vblank = self._get_register(SPG_VBLANK, .SPG_VBLANK).*;
                const spg_vblank_int = self._get_register(SPG_VBLANK_INT, .SPG_VBLANK_INT).*;

                self._get_register(SPG_STATUS, .SPG_STATUS).*.scanline +%= 1;

                // If SB_MDTSEL is set, initiate Maple DMA one line before VBlankOut
                // FIXME: This has nothing to do here.
                if (dc.read_hw_register(u32, .SB_MDEN) & 1 == 1 and dc.read_hw_register(u32, .SB_MDTSEL) & 1 == 1 and @as(u11, spg_status.*.scanline) + 1 == spg_vblank_int.vblank_out_interrupt_line_number) {
                    dc.start_maple_dma();
                }

                if (spg_status.*.scanline == spg_vblank_int.vblank_in_interrupt_line_number) {
                    dc.raise_normal_interrupt(.{ .VBlankIn = 1 });
                }
                if (spg_status.*.scanline == spg_vblank_int.vblank_out_interrupt_line_number) {
                    dc.raise_normal_interrupt(.{ .VBlankOut = 1 });
                }
                if (spg_status.*.scanline == spg_vblank.vbstart) {
                    spg_status.*.vsync = 1;
                }
                if (spg_status.*.scanline == spg_vblank.vbend) {
                    spg_status.*.vsync = 0;
                }
                // FIXME: spg_load.vcount is sometimes < spg_vblank.vbend, locking the system in constant VSync.
                //        I don't know why yet, there's probably something I did not understand correcly,
                //        but the important for now is that all the interrupts are fired and states are reached,
                //        even if the timing is wrong.
                if (spg_status.*.scanline == @max(spg_load.vcount, spg_vblank.vbend)) {
                    spg_status.*.scanline = 0;
                }
            }
        }
    }

    pub fn write_register(self: *@This(), addr: u32, v: u32) void {
        switch (@as(HollyRegister, @enumFromInt(addr))) {
            .ID, .REVISION => return, // Read-only
            .SOFTRESET => {
                holly_log.debug("TODO SOFTRESET: {X:0>8}", .{v});
                const sr: SOFT_RESET = @bitCast(v);
                if (sr.TASoftReset == 1) {
                    holly_log.debug(termcolor.yellow("  TODO: Tile Accelerator Soft Reset"), .{});
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
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

                self.render_start = true;

                self._dc.schedule_interrupt(.{ .RenderDoneTSP = 1 }, 200);
                self._dc.schedule_interrupt(.{ .RenderDoneISP = 1 }, 400);
                self._dc.schedule_interrupt(.{ .RenderDoneVideo = 1 }, 600);
            },
            .TA_LIST_INIT => {
                if (v == 0x80000000) {
                    holly_log.debug("TA_LIST_INIT: {X:0>8}", .{v});
                    if (self._get_register(u32, .TA_LIST_CONT).* & 0x80000000 == 0) {
                        self._get_register(u32, .TA_NEXT_OPB).* = self._get_register(u32, .TA_NEXT_OPB_INIT).*;
                        self._get_register(u32, .TA_ITP_CURRENT).* = self._get_register(u32, .TA_ISP_BASE).*;
                    }
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
                    self._ta_current_volume = null;
                    self._ta_user_tile_clip = null;

                    self._ta_opaque_modifier_volumes.clearRetainingCapacity();
                    self._ta_translucent_modifier_volumes.clearRetainingCapacity();
                    self._ta_volume_triangles.clearRetainingCapacity();
                }
            },
            .TA_LIST_CONT => {
                holly_log.warn("TODO TA_LIST_CONT: {X:0>8}", .{v});
            },
            .SPG_CONTROL, .SPG_LOAD => {
                holly_log.warn("TODO SPG_CONTROL/SPG_LOAD: {X:0>8}", .{v});
            },
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
                    self.ta_display_lists[@intFromEnum(self._ta_list_type.?)].reset();
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
                    // New modifier volume starts
                    self.check_end_of_modifier_volume();
                    if (self._ta_current_volume == null) {
                        self._ta_current_volume = .{
                            .parameter_control_word = modifier_volume.*.parameter_control_word,
                            .instructions = modifier_volume.*.instructions,
                            .first_triangle_index = @intCast(self._ta_volume_triangles.items.len),
                        };
                        if (modifier_volume.*.instructions.volume_instruction == .OutsideLastPolygon) {
                            holly_log.warn("Unsupported Exclusion Modifier Volume.", .{});
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
                    self.ta_display_lists[@intFromEnum(self._ta_list_type.?)].reset();
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
                var display_list = &self.ta_display_lists[@intFromEnum(self._ta_list_type.?)];

                if (self._ta_list_type.? == .OpaqueModifierVolume or self._ta_list_type.? == .TranslucentModifierVolume) {
                    if (self._ta_command_buffer_index < @sizeOf(ModifierVolumeParameter) / 4) return;
                    self._ta_volume_triangles.append(@as(*ModifierVolumeParameter, @ptrCast(&self._ta_command_buffer)).*) catch unreachable;
                    // NOTE: I feel like the documentation contradicts itself here, volume == 1 is said to indicate the last triangle of a modifier volume,
                    //       but the example use an extra global parameter to end the modifier volume. The later seem correct in practice.
                    //           if (parameter_control_word.obj_control.volume == 1) end_volume()?
                } else {
                    if (self._ta_current_polygon == null) {
                        holly_log.debug(termcolor.red("    No current polygon! Current list type: {s}"), .{@tagName(self._ta_list_type.?)});
                        @panic("No current polygon");
                    }

                    const polygon_obj_control = @as(*const GenericGlobalParameter, @ptrCast(&self._ta_current_polygon.?)).*.parameter_control_word.obj_control;
                    switch (self._ta_current_polygon.?) {
                        .Sprite => {
                            std.debug.assert(parameter_control_word.end_of_strip == 1); // Sanity check: For Sprites/Quads, each vertex parameter describes an entire polygon.
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
                            .polygon = self._ta_current_polygon.?,
                            .user_clip = if (self._ta_user_tile_clip) |uc| if (uc.usage != .Disable) uc else null else null,
                            .verter_parameter_index = display_list.next_first_vertex_parameters_index,
                            .verter_parameter_count = display_list.vertex_parameters.items.len - display_list.next_first_vertex_parameters_index,
                        }) catch unreachable;

                        display_list.next_first_vertex_parameters_index = display_list.vertex_parameters.items.len;
                    }
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
        if (self._ta_list_type.? == .OpaqueModifierVolume or self._ta_list_type.? == .TranslucentModifierVolume) {
            if (self._ta_current_volume) |*volume| {
                // FIXME: I should probably honor _ta_user_tile_clip here too... Given the examples in the doc, modifier volume can also be clipped.

                volume.triangle_count = @intCast(self._ta_volume_triangles.items.len - volume.first_triangle_index);
                if (volume.triangle_count > 0) {
                    if (self._ta_list_type.? == .OpaqueModifierVolume) {
                        self._ta_opaque_modifier_volumes.append(volume.*) catch unreachable;
                    } else {
                        self._ta_translucent_modifier_volumes.append(volume.*) catch unreachable;
                    }
                }
                self._ta_current_volume = null;
            }
        }
    }

    fn handle_object_list_set(self: *@This()) void {
        const parameter_control_word: ParameterControlWord = @bitCast(self._ta_command_buffer[0]);
        std.debug.assert(parameter_control_word.parameter_type == .ObjectListSet);

        if (self._ta_list_type == null) {
            self._ta_list_type = parameter_control_word.list_type;
            self.ta_display_lists[@intFromEnum(self._ta_list_type.?)].reset();
        }
        holly_log.debug(termcolor.red("  Unimplemented ObjectListSet"), .{});
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
        holly_log.info("ta_fifo_yuv_converter_path: tex_base={X:0>8}, data.len={X:0>8}\n    ctrl={any}", .{ data.len, tex_base, ctrl });
        if (ctrl.format == 0) {
            if (ctrl.tex == 0) {
                var addr = tex_base;
                var offset: u32 = 0;
                for (0..ctrl.v_size + 1) |v| {
                    _ = v;

                    for (0..ctrl.u_size + 1) |u| {
                        _ = u;

                        //var pixels: [8 * 16]YUV420 = undefined;
                        // FIXME: Should not copy directly, but convert it to YUV422
                        var pixels: [*]YUV420 = @alignCast(@ptrCast(&self.vram[addr]));
                        for (0..8 * 8) |i| {
                            pixels[2 * i].u = data[offset + i];
                            pixels[2 * i + 1].u = data[offset + i];
                        }
                        offset += 8 * 8;
                        for (0..8 * 8) |i| {
                            pixels[2 * i].v = data[offset + i];
                            pixels[2 * i + 1].v = data[offset + i];
                        }
                        offset += 8 * 8;
                        for (0..8 * 16) |idx| {
                            // FIXME: This is wrong, Y value are arranged in 8*8 blocks
                            pixels[idx].y0 = data[offset];
                            pixels[idx].y1 = data[offset + 1];
                            offset += 2;
                        }

                        addr += 8 * 16 * @sizeOf(YUV420);
                    }
                }
                // FIXME: Delay is arbitrary.
                self._dc.schedule_interrupt(.{ .EoT_YUV = 1 }, 200);
            } else {
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

    pub inline fn get_palette_data(self: *const @This()) []u8 {
        return @as([*]u8, @ptrCast(&self.registers[@intFromEnum(HollyRegister.PALETTE_RAM_START) - HollyRegisterStart]))[0 .. 4 * 1024];
    }

    pub inline fn _get_register(self: *@This(), comptime T: type, r: HollyRegister) *T {
        return self._get_register_from_addr(T, @intFromEnum(r));
    }

    pub inline fn _get_register_from_addr(self: *@This(), comptime T: type, addr: u32) *T {
        std.debug.assert(addr >= HollyRegisterStart and addr < HollyRegisterStart + self.registers.len);
        return @as(*T, @alignCast(@ptrCast(&self.registers[addr - HollyRegisterStart])));
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
};
