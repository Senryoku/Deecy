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
    d: u1 = undefined, // Dirty bit
    c: u1 = undefined, // Cacheability bit
    sz0: u1 = undefined, // Page size bit
    pr: u2 = undefined, // Protection key data
    sz1: u1 = undefined, // Page size bit
    v: u1 = undefined, // Validity bit.

    _r0: u1 = undefined,

    ppn: u19 = undefined, // Physical page number

    _r1: u3 = undefined, // TC or Timing control bit in UTBL Entry, not used.
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

pub const UTLBEntry = packed struct {
    vpn: u22 = undefined,
    v: u1 = 0, // Validity bit
    sh: u1 = undefined, // Share status bit, 1 => Shared
    asid: u8 = undefined, //
    ppn: u22 = undefined, // Physical page number
    sz: u2 = undefined, // Page size
    c: u1 = undefined, // Cacheability bit
    pr: u2 = undefined, // Protection key data
    d: u1 = undefined, // Dirty bit
    wt: u1 = undefined, // Write-through bit
    sa: u2 = undefined, // Space attribute bits
    tc: u1 = undefined, // Timing control bit
};

pub const UTLBAddressData = packed struct(u32) {
    asid: u8,
    v: u1,
    d: u1,
    vpn: u22,
};

pub const UTLBArrayData1 = packed struct(u32) {
    wt: u1,
    sh: u1,
    d: u1,
    c: u1,
    sz0: u1,
    pr: u2,
    sz1: u1,
    v: u1,
    _0: u1,
    ppn: u19,
    _1: u3,
};

pub fn vpn_match(lhs: u22, rhs: u22, sz: u2) bool {
    switch (sz) {
        0b00 => return lhs == rhs,
        0b01 => return (lhs & 0b1111_1111_1111_1111_1111_00) == (rhs & 0b1111_1111_1111_1111_1111_00),
        0b10 => return (lhs & 0b1111_1111_1111_1111_0000_00) == (rhs & 0b1111_1111_1111_1111_0000_00),
        0b11 => return (lhs & 0b1111_1111_1111_0000_0000_00) == (rhs & 0b1111_1111_1111_0000_0000_00),
    }
}
