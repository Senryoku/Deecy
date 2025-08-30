const std = @import("std");

// Dreamcast specific
pub const HardwareRegister = enum(u32) {
    ROMChecksum = 0x005F74E4,

    SB_C2DSTAT = 0x005F6800,
    SB_C2DLEN = 0x005F6804,
    SB_C2DST = 0x005F6808,
    /// Sort-DMA start link table address
    SB_SDSTAW = 0x005F6810,
    /// Sort-DMA link base address
    SB_SDBAAW = 0x005F6814,
    /// Sort-DMA link address bit width
    SB_SDWLT = 0x005F6818,
    /// Sort-DMA link address shift control. 1: Link address offset should be multiplied by 32.
    SB_SDLAS = 0x005F681C,
    /// Sort-DMA start
    SB_SDST = 0x005F6820,
    /// Indicates the number of times a Start Link Address was retrieved.
    SB_SDDIV = 0x005F6860,
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
    GD_ByteCountHigh = 0x005F7094,
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
    return std.enums.tagName(HardwareRegister, @as(HardwareRegister, @enumFromInt(addr))) orelse "Unknown";
}

pub const SB_ISTNRM = packed struct(u32) {
    RenderDoneVideo: u1 = 0,
    RenderDoneISP: u1 = 0,
    RenderDoneTSP: u1 = 0,
    VBlankIn: u1 = 0,
    VBlankOut: u1 = 0,
    HBlankIn: u1 = 0,

    EoT_YUV: u1 = 0, // End of Transfer
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
    EoD_PVRSort: u1 = 0, // Sort-DMA (Transferring for alpha sorting)
    EoD_PunchThroughList: u1 = 0,

    _: u8 = 0,

    ExtStatus: u1 = 0,
    ErrorStatus: u1 = 0,

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        const as_u32: u32 = @bitCast(self);
        if (@popCount(as_u32) == 1) {
            inline for (@typeInfo(@This()).@"struct".fields) |field| {
                if (@field(self, field.name) == 1) try writer.writeAll(field.name);
            }
        } else {
            try writer.print("{X}", .{as_u32});
        }
    }
};

pub const SB_ISTEXT = packed struct(u32) {
    GDRom: u1 = 0,
    AICA: u1 = 0,
    Modem: u1 = 0,
    ExternalDevice: u1 = 0,
    _: u28 = 0,

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        const as_u32: u32 = @bitCast(self);
        if (@popCount(as_u32) == 1) {
            inline for (@typeInfo(@This()).@"struct".fields) |field| {
                if (@field(self, field.name) == 1) try writer.writeAll(field.name);
            }
        } else {
            try writer.print("{X}", .{as_u32});
        }
    }
};

pub const SCFSR2 = packed struct(u16) {
    dr: u1 = 0,
    rdf: u1 = 0,
    per: u1 = 0,
    fer: u1 = 0,
    brk: u1 = 0,
    tdfe: u1 = 1,
    tend: u1 = 1,
    er: u1 = 0,
    fer_number: u4 = 0,
    per_number: u4 = 0,
};

// SB_ADSUSP - SB_E1SUSP - SB_E2SUSP - SB_DDSUSP
pub const SB_SUSP = packed struct(u32) {
    dma_suspend_request: u1 = 0, // Write Only
    // 0: Continues DMA transfer without going to the suspended state. Or, bit 2 of the SB_ADTSEL register is "0"
    // 1: Suspends and terminates DMA transfer
    _r: u3 = 0,
    dma_suspend_or_dma_stop: u1 = 1, // Read Only
    // 0: DMA transfer is in progress, or bit 2 of the SB_ADTSEL register is "0"
    // 1: DMA transfer has ended, or is stopped due to a suspend
    // * When bit 2 of the SB_ADTSEL register is "1" and bit 0 of the SB_ADSUSP
    // register is "1", and data is not being transferred due to being in the suspended
    // state, this bit becomes "1" when G2-DMA ends.
    dma_request_input_state: u1 = 1, // Read Only
    // 0: The DMA transfer request is high (transfer not possible), or bit 2 of the SB_ADTSEL register is "0"
    // 1: The DMA transfer request is low (transfer possible)
    _: u26 = 0,
};
