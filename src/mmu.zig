const std = @import("std");
const common = @import("common.zig");

pub const PTEH = packed struct {
    asid: u8 = undefined,
    _: u2 = undefined,
    vpn: u22 = undefined,
};

pub const PTEL = packed struct {
    wt: u1 = undefined, // Write-through bit
    sh: u1 = undefined, // Share status bit
    d: u1 = undefined, // Sirty bit
    c: u1 = undefined, // Cacheability bit
    sz0: u1 = undefined, // Page size bit
    pr: u2 = undefined, // Protection key data
    sz1: u1 = undefined, // Page size bit
    v: u1 = undefined, // Validity bit.

    _r0: u1 = undefined,

    ppn: u19 = undefined, // Physical page number

    _r1: u3 = undefined,
};

pub const MMUCR = packed struct {
    at: u1 = 0, // Address translation bit

    _r0: u1 = undefined,

    ti: u1 = 0, // TLB invalidate

    _r1: u5 = undefined,

    sv: u1 = 0, // Single virtual mode bit.
    sqmd: u1 = 0, // Store queue mode bit.
    urc: u6 = 0,

    _r2: u2 = undefined,

    urb: u6 = 0,

    _r3: u2 = undefined,

    lrui: u6 = 0,
};

pub const MMU = packed struct {
    pteh: PTEH = .{}, // Page table entry high register - 0xFF00 0000 ; 0x1F00 0000
    ptel: PTEL = .{}, // Page table entry low register
    ttb: u32 = undefined, // Translation table base register
    tea: u32 = undefined, // Translation table address register
    mmucr: u32 = 0x00000000, // MMU control register
};
