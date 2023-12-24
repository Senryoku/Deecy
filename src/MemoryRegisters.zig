const std = @import("std");

pub const P4MemoryRegister = enum(u32) {
    // CCN
    PTEH = 0xFF000000,
    PTEL = 0xFF000004,
    TTB = 0xFF000008,
    TEA = 0xFF00000C,
    MMUCR = 0xFF000010,
    BASRA = 0xFF000014,
    BASRB = 0xFF000018,
    CCR = 0xFF00001C,
    TRA = 0xFF000020,
    EXPEVT = 0xFF000024,
    INTEVT = 0xFF000028,
    PTEA = 0xFF000034,
    QACR0 = 0xFF000038,
    QACR1 = 0xFF00003C,

    _FF000030 = 0xFF000030,

    // UBC
    BARA = 0xFF200000,
    BAMRA = 0xFF200004,
    BBRA = 0xFF200008,
    BARB = 0xFF20000C,
    BAMRB = 0xFF200010,
    BBRB = 0xFF200014,
    BDRB = 0xFF200018,
    BDMRB = 0xFF20001C,
    BRCR = 0xFF200020,

    // BSC
    BCR1 = 0xFF800000,
    BCR2 = 0xFF800004,
    WCR1 = 0xFF800008,
    WCR2 = 0xFF80000C,
    WCR3 = 0xFF800010,
    MCR = 0xFF800014,
    PCR = 0xFF800018,
    RTCSR = 0xFF80001C,
    RTCNT = 0xFF800020,
    RTCOR = 0xFF800024,
    RFCR = 0xFF800028,
    PCTRA = 0xFF80002C,
    PDTRA = 0xFF800030,
    PCTRB = 0xFF800040,
    PDTRB = 0xFF800044,
    GPIOIC = 0xFF800048,
    // SDMR2 = 0xFF90xxxx, // "Virtual" registers, idk.
    // SDMR3 = 0xFF94xxxx, // "Virtual" registers

    // CPG
    FRQCR = 0xFFC00000,
    STBCR = 0xFFC00004,
    WTCNT = 0xFFC00008,
    WTCSR = 0xFFC0000C,
    STBCR2 = 0xFFC00010,

    // RTC
    R64CNT = 0xFFC80000,
    RSECCNT = 0xFFC80004,
    RMINCNT = 0xFFC80008,
    RHRCNT = 0xFFC8000C,
    RWKCNT = 0xFFC80010,
    RDAYCNT = 0xFFC80014,
    RMONCNT = 0xFFC80018,
    RYRCNT = 0xFFC8001C,
    RSECAR = 0xFFC80020,
    RMINAR = 0xFFC80024,
    RHRAR = 0xFFC80028,
    RWKAR = 0xFFC8002C,
    RDAYAR = 0xFFC80030,
    RMONAR = 0xFFC80034,
    RCR1 = 0xFFC80038,
    RCR2 = 0xFFC8003C,

    // INTC
    ICR = 0xFFD00000,
    IPRA = 0xFFD00004,
    IPRB = 0xFFD00008,
    IPRC = 0xFFD0000C,

    // TMU
    TOCR = 0xFFD80000,
    TSTR = 0xFFD80004,
    TCOR0 = 0xFFD80008,
    TCNT0 = 0xFFD8000C,
    TCR0 = 0xFFD80010,
    TCOR1 = 0xFFD80014,
    TCNT1 = 0xFFD80018,
    TCR1 = 0xFFD8001C,
    TCOR2 = 0xFFD80020,
    TCNT2 = 0xFFD80024,
    TCR2 = 0xFFD80028,
    TCPR2 = 0xFFD8002C,

    // SCI
    SCSMR1 = 0xFFE00000,
    SCBRR1 = 0xFFE00004,
    SCSCR1 = 0xFFE00008,
    SCTDR1 = 0xFFE0000C,
    SCSSR1 = 0xFFE00010,
    SCRDR1 = 0xFFE00014,
    SCSCMR1 = 0xFFE00018,
    SCSPTR1 = 0xFFE0001C,

    // SCIF
    SCSMR2 = 0xFFE80000,
    SCBRR2 = 0xFFE80004,
    SCSCR2 = 0xFFE80008,
    SCFTDR2 = 0xFFE8000C,
    SCFSR2 = 0xFFE80010,
    SCFRDR2 = 0xFFE80014,
    SCFCR2 = 0xFFE80018,
    SCFDR2 = 0xFFE8001C,
    SCSPTR2 = 0xFFE80020,
    SCLSR2 = 0xFFE80024,

    SDMR = 0xFF940190,

    SAR0 = 0xFFA00000,
    DAR0 = 0xFFA00004,
    DMATCR0 = 0xFFA00008,
    CHCR0 = 0xFFA0000C,
    SAR1 = 0xFFA00010,
    DAR1 = 0xFFA00014,
    DMATCR1 = 0xFFA00018,
    CHCR1 = 0xFFA0001C,
    SAR2 = 0xFFA00020,
    DAR2 = 0xFFA00024,
    DMATCR2 = 0xFFA00028,
    CHCR2 = 0xFFA0002C,
    SAR3 = 0xFFA00030,
    DAR3 = 0xFFA00034,
    DMATCR3 = 0xFFA00038,
    CHCR3 = 0xFFA0003C,
    DMAOR = 0xFFA00040,

    _,
};

// Dreamcast specific
pub const MemoryRegister = enum(u32) {
    ROMChecksum = 0x005F74E4,

    SB_C2DSTAT = 0x005F6800,
    SB_C2DLEN = 0x005F6804,
    SB_C2DST = 0x005F6808,
    SB_SDSTAW = 0x005F6810,
    SB_SDBAAW = 0x005F6814,
    SB_SDWLT = 0x005F6818,
    SB_SDLAS = 0x005F681C,
    SB_SDST = 0x005F6820,
    SB_DBREQM = 0x005F6840,
    SB_BAVLWC = 0x005F6844,
    SB_C2DPRYC = 0x005F6848,
    SB_C2DMAXL = 0x005F684C,
    SB_TFREM = 0x005F6880,
    SB_LMMODE0 = 0x005F6884,
    SB_LMMODE1 = 0x005F6888,
    SB_FFST = 0x005F688C,
    SB_SFRES = 0x005F6890,
    SB_SBREV = 0x005F689C,
    SB_RBSPLT = 0x005F68A0,

    // Interrupt Control Registers
    SB_ISTNRM = 0x005F6900,
    SB_ISTEXT = 0x005F6904,
    SB_ISTERR = 0x005F6908,
    SB_IML2NRM = 0x005F6910,
    SB_IML4NRM = 0x005F6920,
    SB_IML6NRM = 0x005F6930,
    SB_IML2EXT = 0x005F6914,
    SB_IML4EXT = 0x005F6924,
    SB_IML6EXT = 0x005F6934,
    SB_IML2ERR = 0x005F6918,
    SB_IML4ERR = 0x005F6928,
    SB_IML6ERR = 0x005F6938,

    // DMA Hard Trigger Control Registers
    SB_PDTNRM = 0x005F6940,
    SB_PDTEXT = 0x005F6944,
    SB_G2DTNRM = 0x005F6950,
    SB_G2DTEXT = 0x005F6954,

    // Maple-DMA Control Registers
    SB_MDSTAR = 0x005F6C04, // DMA Command Table Address
    SB_MDTSEL = 0x005F6C10, // DMA Trigger Selection
    SB_MDEN = 0x005F6C14, // DMA Enable
    SB_MDST = 0x005F6C18, // DMA Start / Status

    // Maple Interface Block Control Registers
    SB_MSYS = 0x005F6C80, // Maple System Control
    SB_MST = 0x005F6C84, // Maple Status
    SB_MSHTCL = 0x005F6C88, // Maple Status Hard Trigger Clear

    // Maple-DMA Secret Register
    SB_MDAPRO = 0x005F6C8C,
    // Maple Interface Block Hardware Control Register
    SB_MMSEL = 0x005F6CE8,
    // Maple-DMA Debug Registers
    SB_MTXDAD = 0x005F6CF4,
    SB_MRXDAD = 0x005F6CF8,
    SB_MRXDBD = 0x005F6CFC,

    // GD-ROM Registers
    GD_AlternateStatus_DeviceControl = 0x005F7018,
    GD_Data = 0x005F7080,
    GD_Error_Features = 0x005F7084,
    GD_InterruptReason_SectorCount = 0x005F7088,
    GD_SectorNumber = 0x005F708C,
    GD_ByteCountLow = 0x005F7090,
    GD_ByteCountLowHigh = 0x005F7094,
    GD_DriveSelect = 0x005F7098,
    GD_Status_Command = 0x005F709C, // Read: Status, Write: Command

    // GD-DMA Control Registers
    SB_GDSTAR = 0x005F7404,
    SB_GDLEN = 0x005F7408,
    SB_GDDIR = 0x005F740C,
    SB_GDEN = 0x005F7414,
    SB_GDST = 0x005F7418,
    SB_G1RRC = 0x005F7480,
    SB_G1RWC = 0x005F7484,
    SB_G1FRC = 0x005F7488,
    SB_G1FWC = 0x005F748C,
    SB_G1CRC = 0x005F7490,
    SB_G1CWC = 0x005F7494,
    SB_G1GDRC = 0x005F74A0,
    SB_G1GDWC = 0x005F74A4,
    SB_G1SYSM = 0x005F74B0,
    SB_G1CRDYC = 0x005F74B4,
    SB_GDAPRO = 0x005F74B8,

    // GD-DMA Debug Registers
    SB_GDSTARD = 0x005F74F4,
    SB_GDLEND = 0x005F74F8,

    // G2 DMA Control Registers
    SB_ADSTAG = 0x005F7800,
    SB_E1STAG = 0x005F7820,
    SB_E2STAG = 0x005F7840,
    SB_DDSTAG = 0x005F7860,

    SB_ADSTAR = 0x005F7804,
    SB_E1STAR = 0x005F7824,
    SB_E2STAR = 0x005F7844,
    SB_DDSTAR = 0x005F7864,

    SB_ADLEN = 0x005F7808,
    SB_E1LEN = 0x005F7828,
    SB_E2LEN = 0x005F7848,
    SB_DDLEN = 0x005F7868,

    SB_ADDIR = 0x005F780C,
    SB_E1DIR = 0x005F782C,
    SB_E2DIR = 0x005F784C,
    SB_DDDIR = 0x005F786C,

    SB_ADTSEL = 0x005F7810,
    SB_E1TSEL = 0x005F7830,
    SB_E2TSEL = 0x005F7850,
    SB_DDTSEL = 0x005F7870,

    SB_ADEN = 0x005F7814,
    SB_E1EN = 0x005F7834,
    SB_E2EN = 0x005F7854,
    SB_DDEN = 0x005F7874,

    SB_ADST = 0x005F7818,
    SB_E1ST = 0x005F7838,
    SB_E2ST = 0x005F7858,
    SB_DDST = 0x005F7878,

    SB_ADSUSP = 0x005F781C,
    SB_E1SUSP = 0x005F783C,
    SB_E2SUSP = 0x005F785C,
    SB_DDSUSP = 0x005F787C,

    // G2 I/F Block Hardware Control Registers
    SB_G2ID = 0x005F7880,

    SB_G2DSTO = 0x005F7890,
    SB_G2TRTO = 0x005F7894,
    SB_G2MDMTO = 0x005F7898,
    SB_G2MDMW = 0x005F789C,

    // G2-DMA Secret Registers
    SB_G2APRO = 0x005F78BC,

    // G2-DMA Debug Registers
    SB_ADSTAGD = 0x005F78C0,
    SB_E1STAGD = 0x005F78D0,
    SB_E2STAGD = 0x005F78E0,
    SB_DDSTAGD = 0x005F78F0,
    SB_ADSTARD = 0x005F78C4,
    SB_E1STARD = 0x005F78D4,
    SB_E2STARD = 0x005F78E4,
    SB_DDSTARD = 0x005F78F4,
    SB_ADLEND = 0x005F78C8,
    SB_E1LEND = 0x005F78D8,
    SB_E2LEND = 0x005F78E8,
    SB_DDLEND = 0x005F78F8,

    // PVR-DMA Control Registers
    SB_PDSTAP = 0x005F7C00,
    SB_PDSTAR = 0x005F7C04,
    SB_PDLEN = 0x005F7C08,
    SB_PDDIR = 0x005F7C0C,
    SB_PDTSEL = 0x005F7C10,
    SB_PDEN = 0x005F7C14,
    SB_PDST = 0x005F7C18,
    // PVR-DMA Secret Registers
    SB_PDAPRO = 0x005F7C80,

    // Other - Not sure what these are - Probably test registers - but they are accessed by the boot ROM.
    RBERRC = 0x005F68A4, // "RootBus control register (secret register, for chip debugging)" ?
    _5F68AC = 0x005F68AC,
    _5F78A0 = 0x005F78A0,
    _5F78A4 = 0x005F78A4,
    _5F78A8 = 0x005F78A8,
    _5F78AC = 0x005F78AC,

    _5F78B0 = 0x005F78B0,
    _5F78B4 = 0x005F78B4,
    _5F78B8 = 0x005F78B8,

    _,
};

pub fn getRegisterName(addr: u32) []const u8 {
    return std.enums.tagName(MemoryRegister, @as(MemoryRegister, @enumFromInt(addr))) orelse "Unknown";
}

pub fn getP4RegisterName(addr: u32) []const u8 {
    return std.enums.tagName(P4MemoryRegister, @as(P4MemoryRegister, @enumFromInt(addr))) orelse "Unknown";
}

pub const CCR = packed struct(u32) {
    oce: u1,
    wt: u1,
    cb: u1,
    oci: u1,

    _0: u1,

    ora: u1,

    _1: u1,

    oix: u1,
    ice: u1,

    _2: u2,

    ici: u1,

    _3: u3,

    iix: u1,

    _4: u16,
};

pub const BRCR = packed struct(u16) {
    ubde: u1 = 0,

    _r0: u2 = undefined,

    seq: u1 = undefined,

    _r1: u2 = undefined,

    pcbb: u1 = undefined,
    dbeb: u1 = undefined,

    _r2: u2 = undefined,

    pcba: u1 = undefined,

    _r3: u3 = undefined,

    cmfb: u1 = 0,
    cmfa: u1 = 0,
};

pub const TSTR = packed struct(u8) {
    str0: u1 = 0,
    str1: u1 = 0,
    str2: u1 = 0,
    _: u5 = undefined,
};

pub const TCR = packed struct(u16) {
    tpsc: u3 = 0,
    ckeg: u2 = 0,
    unie: u1 = 0,
    icpe: u2 = 0, // Only available on channel 2
    unf: u1 = 0,
    icpf: u1 = 0,
    _: u6 = 0,
};

pub const CHCR = packed struct(u32) {
    de: u1 = 0, // DMAC Enable
    te: u1 = 0, // Transfer End
    ie: u1 = 0, // Interrupt Enable
    _r0: u1 = 0,
    ts: u3 = 0, // Transfer Size
    tm: u1 = 0, // Transfer Mode
    rs: u4 = 0, // Resource Select
    sm: u2 = 0, // Source Address Mode
    dm: u2 = 0, // Destination Address Mode
    al: u1 = 0, // Acknowledge Level
    am: u1 = 0, // Acknowledge Mode
    rl: u1 = 0, // Request Check Level
    ds: u1 = 0, // SREQ Select
    _r1: u4 = 0,
    dtc: u1 = 0, // Destination Address Wait Control Select
    dsa: u3 = 0, // Destination Address Space Attribute Specification
    stc: u1 = 0, // Source Address Wait Control Select
    ssa: u3 = 0, // Source Address Space Attribute Specification
};

pub const DMAOR = packed struct(u32) {
    dme: u1 = 0, // DMAC Master Enable
    nmif: u1 = 0, // NMI Flag
    ae: u1 = 0, // Address Error Flag
    _r0: u5 = 0,
    pr: u2 = 0, // Priority Mode
    _r1: u5 = 0,
    ddt: u1 = 0, // On-Demand Data Transfer
    _r2: u16 = 0,
};

pub const SB_ISTNRM = packed struct(u32) {
    RenderDoneVideo: u1 = 0,
    RenderDoneISP: u1 = 0,
    RenderDoneTSP: u1 = 0,
    VBlankIn: u1 = 0,
    VBlankOut: u1 = 0,
    HBlankIn: u1 = 0,

    EoT_YUV: u1 = 0, // End of Transfering
    EoT_OpaqueList: u1 = 0,
    EoT_OpaqueModifierVolumeList: u1 = 0,

    EoT_TranslucentList: u1 = 0,
    EoT_TranslucentModifierVolumeList: u1 = 0,
    EoD_PVR: u1 = 0, // End of DMA
    EoD_Maple: u1 = 0,

    MapleVBlankOver: u1 = 0,
    EoD_GDROM: u1 = 0,
    EoD_AICA: u1 = 0,

    EoD_EXT1: u1 = 0,
    EoD_EXT2: u1 = 0,
    EoD_DEV: u1 = 0,

    EoD_CH2: u1 = 0,
    EoD_PVRSort: u1 = 0,
    EoD_PunchThroughList: u1 = 0,

    _: u8 = 0,

    ExtStatus: u1 = 0,
    ErrorStatus: u1 = 0,
};

pub const SB_ISTEXT = packed struct(u32) {
    GDRom: u1 = 0,
    AICA: u1 = 0,
    Modem: u1 = 0,
    ExternalDevice: u1 = 0,
    _: u28 = 0,
};
