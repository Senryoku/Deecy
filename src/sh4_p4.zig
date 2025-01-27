const std = @import("std");

pub const P4Register = enum(u32) {
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

    PMCR1 = 0xFF000084, // Performance Counter Control Register  1
    PMCR2 = 0xFF000088, // Performance Counter Control Register 2
    PC1H = 0xFF100004, // Performance counter 1H
    PC1L = 0xFF100008, // Performance counter 1L
    PC2H = 0xFF10000C, // Performance counter 2H
    PC2L = 0xFF100010, // Performance counter 2L

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
    SCSMR2 = 0xFFE80000, // Serial mode register
    SCBRR2 = 0xFFE80004, // Bit rate register
    SCSCR2 = 0xFFE80008, // Serial control register
    SCFTDR2 = 0xFFE8000C, // Transmit FIFO data register
    SCFSR2 = 0xFFE80010, // Serial status register
    SCFRDR2 = 0xFFE80014, // Receive FIFO data register
    SCFCR2 = 0xFFE80018, // FIFO control register
    SCFDR2 = 0xFFE8001C, // FIFO data count register
    SCSPTR2 = 0xFFE80020, // Serial port register
    SCLSR2 = 0xFFE80024, // Line status register

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

pub fn getP4RegisterName(addr: u32) []const u8 {
    return std.enums.tagName(P4Register, @as(P4Register, @enumFromInt(addr))) orelse "Unknown";
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
    icpf: u1 = 0, // Only available on channel 2
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

    pub fn transfer_size(self: @This()) u32 {
        return switch (self.ts) {
            0 => 8, // Quadword size
            1 => 1, // Byte
            2 => 2, // Word
            3 => 4, // Longword
            4 => 32, // 32-bytes block
            else => @panic("Invalid transfer size"),
        };
    }
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

pub const IPRA = packed struct(u16) {
    rtc: u4 = 0,
    tmu2: u4 = 0,
    tmu1: u4 = 0,
    tmu0: u4 = 0,
};

pub const IPRB = packed struct(u16) {
    _: u4 = 0,
    sci1: u4 = 0,
    ref: u4 = 0,
    wdt: u4 = 0,
};

pub const IPRC = packed struct(u16) {
    hitachiudi: u4 = 0,
    scif: u4 = 0,
    dmac: u4 = 0,
    gpio: u4 = 0,
};

pub const ICR = packed struct(u16) {
    _r0: u7 = 0,
    irlm: u1 = 0, // IRL Pin Mode (IRLM): Specifies whether pins IRL3-IRL0 are to be used as level-encoded interrupt requests or as four independent interrupt requests.
    nmie: u1 = 0, // NMI Edge Select (NMIE): Specifies whether the falling or rising edge of the interrupt request signal to the NMI pin is detected.
    nmib: u1 = 0, // NMI Block Mode (NMIB): Specifies whether an NMI request is to be held pending or detected immediately while the SR.BL bit is set to 1.
    _r1: u4 = 0,
    mai: u1 = 0, // NMI Interrupt Mask (MAI): Specifies whether or not all interrupts are to be masked while the NMI pin input level is low, irrespective of the CPU’s SR.BL bit.
    nmil: u1 = 0, // NMI Input Level (NMIL): Sets the level of the signal input at the NMI pin. This bit can be read to determine the NMI pin level. It cannot be modified.
};

pub const FRQCR = packed struct(u16) {
    pfc: u3 = 0b010, // Peripheral clock
    bfc: u3 = 0b001, // Bus clock
    ifc: u3 = 0b000, // CPU clock
    pll2en: u1 = 0, // Use PLL2
    pll1en: u1 = 0, // Use PLL1
    ckoen: u1 = 0, // CKIO clock input
    _r0: u4 = 0,

    pub fn peripheral_clock_ratio(self: @This()) u32 {
        const PeripheralClockRatio: u32 = switch (self.pfc) {
            0b000 => 2,
            0b001 => 3,
            0b010 => 4,
            0b011 => 6,
            0b100 => 8,
            else => @panic("Prohibited value in FRQCR.pfc"),
        };
        std.debug.assert(PeripheralClockRatio == 4); // For optimisation purposes
        return PeripheralClockRatio;
    }
};
