const std = @import("std");

// SH4 internal interrupts

pub const IPRA = packed struct(u8) {
    rtc: u4 = 0,
    tmu2: u4 = 0,
    tmu1: u4 = 0,
    tmu0: u4 = 0,
};

pub const IPRB = packed struct(u8) {
    _: u4 = 0,
    sci1: u4 = 0,
    ref: u4 = 0,
    wdt: u4 = 0,
};

pub const IPRC = packed struct(u8) {
    hitachiudi: u4 = 0,
    scif: u4 = 0,
    dmac: u4 = 0,
    gpio: u4 = 0,
};

pub const ICR = packed struct(u8) {
    _r0: u7 = 0,
    irlm: u1 = 0, // IRL Pin Mode (IRLM): Specifies whether pins IRL3-IRL0 are to be used as level-encoded interrupt requests or as four independent interrupt requests.
    nmie: u1 = 0, // NMI Edge Select (NMIE): Specifies whether the falling or rising edge of the interrupt request signal to the NMI pin is detected.
    nmib: u1 = 0, // NMI Block Mode (NMIB): Specifies whether an NMI request is to be held pending or detected immediately while the SR.BL bit is set to 1.
    _r1: u4 = 0,
    mai: u1 = 0, // NMI Interrupt Mask (MAI): Specifies whether or not all interrupts are to be masked while the NMI pin input level is low, irrespective of the CPU’s SR.BL bit.
    nmil: u1 = 0, // NMI Input Level (NMIL): Sets the level of the signal input at the NMI pin. This bit can be read to determine the NMI pin level. It cannot be modified.
};

pub const Interrupt = enum {
    NMI,
    IRL0,
    IRL1,
    IRL2,
    IRL3,
    IRL4,
    IRL5,
    IRL6,
    IRL7,
    IRL8,
    IRL9,
    IRL10,
    IRL11,
    IRL12,
    IRL13,
    IRL14,
    HitachiUDI,
    GPIO,
    DMTE0,
    DMTE1,
    DMTE2,
    DMTE3,
    DMAE,
    TUNI0,
    TUNI1,
    TUNI2,
    TICPI2,
    ATI,
    PRI,
    CUI,
    SCI1_ERI,
    SCI1_RXI,
    SCI1_TXI,
    SCI1_TEI,
    SCIF_ERI,
    SCIF_RXI,
    SCIF_TXI,
    SCIF_TEI,
    ITI,
    RCMI,
    ROVI,
};

pub const InterruptINTEVTCodes: [41]u32 = .{
    0x1C0, // NMI
    0x200, // IRL0
    0x220, // IRL1
    0x240, // IRL2
    0x260, // IRL3
    0x280, // IRL4
    0x2A0, // IRL5
    0x2C0, // IRL6
    0x2E0, // IRL7
    0x300, // IRL8
    0x320, // IRL9
    0x340, // IRLA
    0x360, // IRLB
    0x380, // IRLC
    0x3A0, // IRLD
    0x3C0, // IRLE
    0x600, // Hitachi
    0x620, // GPIO
    0x640, // DMTE0
    0x660, // DMTE1
    0x680, // DMTE2
    0x6A0, // DMTE3
    0x6C0, // DMAE
    0x400, // TUNI0
    0x420, // TUNI1
    0x440, // TUNI2
    0x460, // TICPI2
    0x480, // ATI
    0x4A0, // PRI
    0x4C0, // CUI
    0x4E0, // SCI1_ERI
    0x500, // SCI1_RXI
    0x520, // SCI1_TXI
    0x540, // SCI1_TEI
    0x700, // SCIF_ERI
    0x720, // SCIF_RXI
    0x740, // SCIF_TXI
    0x760, // SCIF_TEI
    0x560, // ITI
    0x580, // RCMI
    0x5A0, // ROVI
};
