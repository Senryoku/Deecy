const std = @import("std");
const common = @import("common.zig");

pub const PTEH = packed struct {
    asid: u8 = undefined,
    _: u2 = undefined,
    vpn: u22 = undefined,
};

pub const PTEL = packed struct {
    /// Write-through bit
    wt: u1 = undefined,
    /// Share status bit
    sh: u1 = undefined,
    /// Dirty bit
    d: u1 = undefined,
    /// Cacheability bit
    c: u1 = undefined,
    /// Page size bit
    sz0: u1 = undefined,
    /// Protection key data
    pr: u2 = undefined,
    /// Page size bit
    sz1: u1 = undefined,
    /// Validity bit
    v: u1 = undefined,

    _r0: u1 = undefined,
    /// Physical page number
    ppn: u19 = undefined,
    /// TC or Timing control bit in UTBL Entry, not used.
    _r1: u3 = undefined,

    pub fn sz(self: @This()) u2 {
        return @as(u2, self.sz1) << 1 | self.sz0;
    }
};

pub const PTEA = packed struct {
    sa: u3 = undefined,
    tc: u1 = undefined,
    _: u28 = undefined,
};

pub const MMUCR = packed struct {
    at: bool = false, // Address translation bit

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
    asid: u8 = undefined, // Address space identifier
    vpn: u22 = undefined, // Virtual page number
    v: u1 = 0, // Validity bit

    tc: u1 = undefined, // Timing control bit
    sa: u3 = undefined, // Space attribute bits
    wt: u1 = undefined, // Write-through bit
    d: u1 = undefined, // Dirty bit
    pr: u2 = undefined, // Protection key data
    c: u1 = undefined, // Cacheability bit
    sh: u1 = undefined, // Share status bit, 1 => Shared
    sz: u2 = undefined, // Page size
    ppn: u19 = undefined, // Physical page number
    _: u2 = 0,

    pub inline fn valid(self: @This()) bool {
        return self.v == 1;
    }

    pub inline fn match(self: @This(), vpn: u22) bool {
        return self.valid() and vpn_match(self.vpn, vpn, self.sz);
    }
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
    _0: u1 = 0,
    ppn: u19,
    _1: u3 = 0,
};

pub fn vpn_match(lhs: u22, rhs: u22, sz: u2) bool {
    switch (sz) {
        0b00 => return lhs == rhs,
        0b01 => return (lhs & 0b1111_1111_1111_1111_1111_00) == (rhs & 0b1111_1111_1111_1111_1111_00),
        0b10 => return (lhs & 0b1111_1111_1111_1111_0000_00) == (rhs & 0b1111_1111_1111_1111_0000_00),
        0b11 => return (lhs & 0b1111_1111_1111_0000_0000_00) == (rhs & 0b1111_1111_1111_0000_0000_00),
    }
}
