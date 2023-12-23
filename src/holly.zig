const std = @import("std");

const SH4 = @import("sh4.zig").SH4;

const holly_log = std.log.scoped(.holly);

const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;

const termcolor = @import("termcolor.zig");

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
    v: u8,
    y1: u8,
    u: u8,
    y0: u8,
};

pub const RGBA = packed struct(u32) {
    a: u8,
    b: u8,
    g: u8,
    r: u8,
};

fn yuv_to_rgba(yuv: YUV422) [2]RGBA {
    const v = @as(f32, @floatFromInt(yuv.v)) - 128.0;
    const u = @as(f32, @floatFromInt(yuv.u)) - 128.0;
    const y0 = @as(f32, @floatFromInt(yuv.y0));
    const y1 = @as(f32, @floatFromInt(yuv.y1));
    return .{ .{
        .r = @intFromFloat(std.math.clamp(y0 + (11 / 8) * v, 0, 255)),
        .g = @intFromFloat(std.math.clamp(y0 - 0.25 * (11 / 8) * u - 0.5 * (11 / 8) * v, 0, 255)),
        .b = @intFromFloat(std.math.clamp(y0 + 1.25 * (11 / 8) * u, 0, 255)),
        .a = 255,
    }, .{
        .r = @intFromFloat(std.math.clamp(y1 + (11 / 8) * v, 0, 255)),
        .g = @intFromFloat(std.math.clamp(y1 - 0.25 * (11 / 8) * u - 0.5 * (11 / 8) * v, 0, 255)),
        .b = @intFromFloat(std.math.clamp(y1 + 1.25 * (11 / 8) * u, 0, 255)),
        .a = 255,
    } };
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

pub const ISP_BACKGND_T = packed struct(u32) {
    tag_offset: u3,
    tag_address: u21,
    skip: u3,
    shadow: u1,
    cache_bypass: u1,
    _: u3,
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

pub const GroupControl = packed struct(u8) {
    user_clip: u2,
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

// Global Parameter Formats

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
    depth_compare_mode: u3,
};

pub const TextureShadingInstruction = enum(u2) {
    Decal = 0,
    Modulate = 1,
    DecalAlpha = 2,
    ModulateAlpha = 3,
};

pub const TSPInstructionWord = packed struct(u32) {
    texture_v_size: u3,
    texture_u_size: u3,
    texture_shading_instruction: TextureShadingInstruction,
    mapmap_d_adjust: u4,
    supersample_texture: u1,
    filter_mode: u2,
    clamp_uv: u2,
    flip_uv: u2,
    ignore_texture_alpha: u1,
    use_alpha: u1,
    color_clamp: u1,
    fog_control: u2,
    dst_select: u1,
    src_select: u1,
    dst_alpha_instr: u3,
    src_alpha_instr: u3,
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

const PolygonType = enum {
    PolygonType0,
    PolygonType1,
    PolygonType2,
    PolygonType3,
    PolygonType4,
};

pub const Polygon = union(PolygonType) {
    PolygonType0: PolygonType0,
    PolygonType1: PolygonType1,
    PolygonType2: PolygonType2,
    PolygonType3: PolygonType3,
    PolygonType4: PolygonType4,
};

const Sprite = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    tsp_instruction: TSPInstructionWord,
    texture_control: TextureControlWord,
    base_color: u32,
    offset_color: u32,
    data_size: u32,
    next_address: u32,
};

const ModifierVolume = packed struct(u256) {
    parameter_control_word: ParameterControlWord,
    isp_tsp_instruction: ISPTSPInstructionWord,
    _ignored: u192,
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

const UV16 = packed struct(u32) {
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
// Intensity, Textured 32bit UV
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
    };
}

fn obj_control_to_vertex_parameter_format(obj_control: ObjControl) VertexParameterType {
    // Shadow (Ignored) - Volume - ColType (u2) - Texture - Offset (Ignored) - Gouraud (Ignored) - 16bit UV
    const masked = @as(u16, @bitCast(obj_control)) & 0b00000000_0_1_11_1_0_0_1;
    switch (masked) {
        @as(u16, @bitCast(ObjControl{ .uv_16bit = 0, .gouraud = 0, .offset = 0, .texture = 0, .col_type = .PackedColor, .volume = 0, .shadow = 0 })) => return .Type0,
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
            std.debug.print(termcolor.red("Unimplemented obj_control_to_vertex_parameter_format: {}"), .{masked});
            @panic("Unimplemented");
        },
    }
}

pub const DisplayList = struct {
    polygons: std.ArrayList(Polygon) = undefined,
    vertex_parameters: std.ArrayList(std.ArrayList(VertexParameter)) = undefined,

    pub fn init(allocator: std.mem.Allocator) DisplayList {
        return .{
            .polygons = std.ArrayList(Polygon).init(allocator),
            .vertex_parameters = std.ArrayList(std.ArrayList(VertexParameter)).init(allocator),
        };
    }

    pub fn deinit(self: *DisplayList) void {
        self.vertex_parameters.deinit();
        self.polygons.deinit();
    }

    pub fn reset(self: *DisplayList) void {
        for (self.vertex_parameters.items) |*vertex_parameters| {
            vertex_parameters.deinit();
        }
        self.vertex_parameters.clearRetainingCapacity();
        self.polygons.clearRetainingCapacity();
    }
};

pub const Holly = struct {
    vram: []u8 = undefined,
    registers: []u8 = undefined,

    render_start: bool = false, // Signals to start rendering. TODO: Find a better way to start rendering (and run the CPU on another thread I guess).

    _allocator: std.mem.Allocator = undefined,

    _ta_command_buffer: [16]u32 align(8) = .{0} ** 16,
    _ta_command_buffer_index: u32 = 0,
    _ta_list_type: ?ListType = null,

    _ta_current_polygon: ?Polygon = null,
    _ta_current_polygon_vertex_parameters: std.ArrayList(VertexParameter) = undefined, // FIXME: Move out.

    ta_display_lists: [5]DisplayList = undefined,

    _scheduled_interrupts: std.ArrayList(struct { cycles: u32, int: MemoryRegisters.SB_ISTNRM }) = undefined,

    pub fn init(self: *@This(), allocator: std.mem.Allocator) !void {
        self._allocator = allocator;

        self.vram = try self._allocator.alloc(u8, 8 * 1024 * 1024);
        self.registers = try self._allocator.alloc(u8, 0x2000);

        self._ta_current_polygon_vertex_parameters = std.ArrayList(VertexParameter).init(self._allocator);

        for (0..self.ta_display_lists.len) |i| {
            self.ta_display_lists[i] = DisplayList.init(self._allocator);
        }

        self._scheduled_interrupts = try @TypeOf(self._scheduled_interrupts).initCapacity(self._allocator, 32);
    }

    pub fn deinit(self: *@This()) void {
        self._scheduled_interrupts.deinit();

        for (&self.ta_display_lists) |*display_list| {
            display_list.deinit();
        }

        self._ta_current_polygon_vertex_parameters.deinit();

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

        self._get_register(u32, .TA_LIST_CONT).* = 0;
    }

    pub fn update(self: *@This(), cpu: *SH4, cycles: u32) void {
        // TODO!
        const static = struct {
            var _tmp_cycles: u64 = 0;
            var _pixel: u32 = 0;
        };
        static._tmp_cycles += cycles;

        // Update scheduled interrupts
        var idx: u32 = 0;
        while (idx < self._scheduled_interrupts.items.len) {
            if (self._scheduled_interrupts.items[idx].cycles < cycles) {
                cpu.raise_normal_interrupt(self._scheduled_interrupts.items[idx].int);
                _ = self._scheduled_interrupts.swapRemove(idx);
            } else {
                self._scheduled_interrupts.items[idx].cycles -= cycles;
                idx += 1;
            }
        }

        // FIXME: Move all of this to its own SPG Module ?

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

                // If SB_MDTSEL is set, initiate Maple DMA one line before VBlankOut
                // FIXME: This has nothing to do here.
                if (cpu.read_hw_register(u32, .SB_MDEN) & 1 == 1 and cpu.read_hw_register(u32, .SB_MDTSEL) & 1 == 1 and @as(u11, spg_status.*.scanline) + 1 == spg_vblank_int.vblank_out_interrupt_line_number) {
                    cpu.start_maple_dma();
                }

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

    pub fn schedule_interrupt(self: *@This(), cycles: u32, int: MemoryRegisters.SB_ISTNRM) void {
        self._scheduled_interrupts.appendAssumeCapacity(.{ .cycles = cycles, .int = int });
    }

    pub fn write_register(self: *@This(), addr: u32, v: u32) void {
        switch (addr) {
            @intFromEnum(HollyRegister.SOFTRESET) => {
                holly_log.warn("TODO SOFTRESET: {X:0>8}", .{v});
                const sr: SOFT_RESET = @bitCast(v);
                if (sr.TASoftReset == 1) {
                    holly_log.warn(termcolor.yellow("  TODO: Tile Accelerator Soft Reset"), .{});
                }
                if (sr.PipelineSoftReset == 1) {
                    holly_log.warn(termcolor.yellow("  TODO: Pipeine Soft Reset"), .{});
                }
                if (sr.SDRAMInterfaceSoftReset == 1) {
                    holly_log.warn(termcolor.yellow("  TODO: SDRAM Interface Soft Reset"), .{});
                }
                return;
            },
            @intFromEnum(HollyRegister.STARTRENDER) => {
                holly_log.info(termcolor.green("STARTRENDER!"), .{});

                self.render_start = true;

                self.schedule_interrupt(200, .{ .RenderDoneTSP = 1 });
                self.schedule_interrupt(400, .{ .RenderDoneISP = 1 });
                self.schedule_interrupt(600, .{ .RenderDoneVideo = 1 });
            },
            @intFromEnum(HollyRegister.TA_LIST_INIT) => {
                if (v == 0x80000000) {
                    holly_log.debug("TA_LIST_INIT: {X:0>8}", .{v});
                    if (self._get_register(u32, .TA_LIST_CONT).* & 0x80000000 == 0) {
                        self._get_register(u32, .TA_NEXT_OPB).* = self._get_register(u32, .TA_NEXT_OPB_INIT).*;
                        self._get_register(u32, .TA_ITP_CURRENT).* = self._get_register(u32, .TA_ISP_BASE).*;
                    }
                    self._ta_command_buffer_index = 0;
                    self._ta_list_type = null;
                    self._ta_current_polygon = null;
                }
            },
            @intFromEnum(HollyRegister.TA_LIST_CONT) => {
                holly_log.warn("TODO TA_LIST_CONT: {X:0>8}", .{v});
            },
            @intFromEnum(HollyRegister.SPG_CONTROL), @intFromEnum(HollyRegister.SPG_LOAD) => {
                holly_log.warn("TODO SPG_CONTROL/SPG_LOAD: {X:0>8}", .{v});
            },
            else => {
                holly_log.debug("Write to Register: @{X:0>8} {s} = {X:0>8}", .{ addr, std.enums.tagName(HollyRegister, @as(HollyRegister, @enumFromInt(addr))) orelse "Unknown", v });
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
        @memcpy(self._ta_command_buffer[self._ta_command_buffer_index .. self._ta_command_buffer_index + 8], v);

        self._ta_command_buffer_index += 8;

        self.handle_command();
    }

    pub fn handle_command(self: *@This()) void {
        if (self._ta_command_buffer_index % 8 != 0) return; // All commands are 8 bytes or 16 bytes long

        const parameter_control_word: ParameterControlWord = @bitCast(self._ta_command_buffer[0]);

        holly_log.debug(" TA Parameter Type: {any}\n", .{parameter_control_word.parameter_type});
        for (0..8) |i| {
            holly_log.debug("      {X:0>8}\n", .{self._ta_command_buffer[i]});
        }

        switch (parameter_control_word.parameter_type) {
            .EndOfList => {
                if (self._ta_list_type != null) { // Apprently this happens?... Why would a game do this?
                    // Fire corresponding interrupt.
                    // FIXME: Delay is completely arbitrary, I just need to delay them for testing, for now.
                    switch (self._ta_list_type.?) {
                        .Opaque => {
                            self.schedule_interrupt(800, .{ .EoT_OpaqueList = 1 });
                        },
                        .OpaqueModifierVolume => {
                            self.schedule_interrupt(800, .{ .EoT_OpaqueModifierVolumeList = 1 });
                        },
                        .Translucent => {
                            self.schedule_interrupt(800, .{ .EoT_TranslucentList = 1 });
                        },
                        .TranslucentModifierVolume => {
                            self.schedule_interrupt(800, .{ .EoT_TranslucentModifierVolumeList = 1 });
                        },
                        .PunchThrough => {
                            self.schedule_interrupt(800, .{ .EoD_PunchThroughList = 1 });
                        },
                    }
                }
                self._ta_list_type = null;
                self._ta_current_polygon = null;
            },
            .UserTileClip => {
                @panic("Unimplemented UserTileClip");
            },
            .ObjectListSet => {
                if (self._ta_list_type == null) {
                    self._ta_list_type = parameter_control_word.list_type;
                    self.ta_display_lists[@intFromEnum(self._ta_list_type.?)].reset();
                }
                @panic("Unimplemented ObjectListSet");
            },
            .PolygonOrModifierVolume => {
                if (self._ta_list_type == null) {
                    self._ta_list_type = parameter_control_word.list_type;
                    self.ta_display_lists[@intFromEnum(self._ta_list_type.?)].reset();
                }
                std.debug.assert(self._ta_list_type == parameter_control_word.list_type);

                // Note: "Four bits in the ISP/TSP Instruction Word are overwritten with the corresponding bit values from the Parameter Control Word."
                const global_parameter = @as(*GenericGlobalParameter, @ptrCast(&self._ta_command_buffer));
                global_parameter.*.isp_tsp_instruction.texture = global_parameter.*.parameter_control_word.obj_control.texture;
                global_parameter.*.isp_tsp_instruction.offset = global_parameter.*.parameter_control_word.obj_control.offset;
                global_parameter.*.isp_tsp_instruction.gouraud = global_parameter.*.parameter_control_word.obj_control.gouraud;
                global_parameter.*.isp_tsp_instruction.uv_16bit = global_parameter.*.parameter_control_word.obj_control.uv_16bit;

                if (self._ta_list_type == .OpaqueModifierVolume or self._ta_list_type == .TranslucentModifierVolume) {
                    holly_log.err(termcolor.red("  Unimplemented OpaqueModifierVolume/TranslucentModifierVolume"), .{});
                    self._ta_current_polygon = null;
                } else {
                    if (parameter_control_word.obj_control.volume == 0) {
                        switch (parameter_control_word.obj_control.col_type) {
                            .PackedColor, .FloatingColor => {
                                const polygon_type_0 = @as(*PolygonType0, @ptrCast(&self._ta_command_buffer)).*;
                                self._ta_current_polygon = .{ .PolygonType0 = polygon_type_0 };
                            },
                            .IntensityMode1, .IntensityMode2 => {
                                if (parameter_control_word.obj_control.offset == 0) {
                                    const polygon_type_1 = @as(*PolygonType1, @ptrCast(&self._ta_command_buffer)).*;
                                    self._ta_current_polygon = .{ .PolygonType1 = polygon_type_1 };
                                } else {
                                    if (self._ta_command_buffer_index < 16) return; // Command not fully received yet
                                    const polygon_type_2 = @as(*PolygonType2, @ptrCast(&self._ta_command_buffer)).*;
                                    self._ta_current_polygon = .{ .PolygonType2 = polygon_type_2 };
                                }
                            },
                        }
                    } else { // "With Two Volumes"
                        switch (parameter_control_word.obj_control.col_type) {
                            .PackedColor, .FloatingColor => {
                                const polygon_type_3 = @as(*PolygonType3, @ptrCast(&self._ta_command_buffer)).*;
                                self._ta_current_polygon = .{ .PolygonType3 = polygon_type_3 };
                            },
                            .IntensityMode1, .IntensityMode2 => {
                                if (self._ta_command_buffer_index < 16) return; // Command not fully received yet
                                const polygon_type_4 = @as(*PolygonType4, @ptrCast(&self._ta_command_buffer)).*;
                                self._ta_current_polygon = .{ .PolygonType4 = polygon_type_4 };
                            },
                        }
                    }
                }
            },
            .SpriteList => {
                if (self._ta_list_type == null) {
                    self._ta_list_type = parameter_control_word.list_type;
                    self.ta_display_lists[@intFromEnum(self._ta_list_type.?)].reset();
                }
                @panic("Unimplemented SpriteList");
            },
            .VertexParameter => {
                if (self._ta_current_polygon == null) {
                    holly_log.err(termcolor.red("    No current polygon! Current list type: {s}"), .{@tagName(self._ta_list_type.?)});
                    @panic("No current polygon");
                } else {
                    const polygon_obj_control = @as(*const GenericGlobalParameter, @ptrCast(&self._ta_current_polygon.?)).*.parameter_control_word.obj_control;
                    const format = obj_control_to_vertex_parameter_format(polygon_obj_control);
                    if (self._ta_command_buffer_index < vertex_parameter_size(format)) return;

                    switch (format) {
                        .Type0 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type0 = @as(*VertexParameter_0, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type1 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type1 = @as(*VertexParameter_1, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type2 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type2 = @as(*VertexParameter_2, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type3 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type3 = @as(*VertexParameter_3, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type4 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type4 = @as(*VertexParameter_4, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type5 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type5 = @as(*VertexParameter_5, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type6 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type6 = @as(*VertexParameter_6, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type7 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type7 = @as(*VertexParameter_7, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type8 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type8 = @as(*VertexParameter_8, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type9 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type9 = @as(*VertexParameter_9, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type10 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type10 = @as(*VertexParameter_10, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type11 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type11 = @as(*VertexParameter_11, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type12 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type12 = @as(*VertexParameter_12, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type13 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type13 = @as(*VertexParameter_13, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                        .Type14 => {
                            self._ta_current_polygon_vertex_parameters.append(.{ .Type14 = @as(*VertexParameter_14, @ptrCast(&self._ta_command_buffer)).* }) catch unreachable;
                        },
                    }

                    if (parameter_control_word.end_of_strip == 1) {
                        // std.debug.print("  End of Strip - Length: {X:0>8}\n", .{self._ta_current_polygon_vertex_parameters.items.len});
                        self.ta_display_lists[@intFromEnum(self._ta_list_type.?)].polygons.append(self._ta_current_polygon.?) catch unreachable;
                        self.ta_display_lists[@intFromEnum(self._ta_list_type.?)].vertex_parameters.append(self._ta_current_polygon_vertex_parameters) catch unreachable;

                        self._ta_current_polygon_vertex_parameters = @TypeOf(self._ta_current_polygon_vertex_parameters).init(self._allocator);
                    }
                }
            },
        }
        // Command has been handled, reset buffer.
        self._ta_command_buffer_index = 0;
    }

    pub fn ta_fifo_yuv_converter_path(self: *@This()) void {
        _ = self;

        @panic("Unimplemented ta_fifo_yuv_converter_path");
    }

    pub fn write_ta_fifo_direct_texture_path(self: *@This(), addr: u32, value: []u8) void {
        holly_log.info("  NOTE: DMA to Direct Texture Path to {X:0>8} (len: {X:0>8})", .{ addr, value.len });
        @memcpy(self.vram[addr & 0x00FFFFFF .. (addr & 0x00FFFFFF) + value.len], value);
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
            holly_log.err(termcolor.red(" Out of bounds access to Area 1 (VRAM): {X:0>8}"), .{addr});
            @panic("Out of bounds access to Area 1 (VRAM)");
            //return &self.vram[0];
        } else if (local_addr < 0x0180_0000) { // 32-bit access area
            return &self.vram[local_addr - 0x0100_0000];
        } else { // Unused
            holly_log.err(termcolor.red(" Out of bounds access to Area 1 (VRAM): {X:0>8}"), .{addr});
            @panic("Out of bounds access to Area 1 (VRAM)");
            //return &self.vram[0];
        }
    }
};
