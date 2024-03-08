const std = @import("std");

// SH4 internal interrupts

// Ordered by fixed priorities. For configurable priorities, this can also be used as a tie breaker.
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
    // Configurable priorities
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
    0x340, // IRL10
    0x360, // IRL11
    0x380, // IRL12
    0x3A0, // IRL13
    0x3C0, // IRL14
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

pub const InterruptLevel: [41]u32 = .{
    16, // NMI
    15, // IRL0
    14, // IRL1
    13, // IRL2
    12, // IRL3
    11, // IRL4
    10, // IRL5
    9, // IRL6
    8, // IRL7
    7, // IRL8
    6, // IRL9
    5, // IRL10
    4, // IRL11
    3, // IRL12
    2, // IRL13
    1, // IRL14
    // Configurable priorities
    0, // Hitachi
    0, // GPIO
    0, // DMTE0
    0, // DMTE1
    0, // DMTE2
    0, // DMTE3
    0, // DMAE
    0, // TUNI0
    0, // TUNI1
    0, // TUNI2
    0, // TICPI2
    0, // ATI
    0, // PRI
    0, // CUI
    0, // SCI1_ERI
    0, // SCI1_RXI
    0, // SCI1_TXI
    0, // SCI1_TEI
    0, // SCIF_ERI
    0, // SCIF_RXI
    0, // SCIF_TXI
    0, // SCIF_TEI
    0, // ITI
    0, // RCMI
    0, // ROVI
};
