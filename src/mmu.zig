const std = @import("std");
const common = @import("common.zig");

pub const ProtectionKey = enum(u2) {
    PrivilegedReadOnly = 0,
    PrivilegedReadWrite = 1,
    ReadOnly = 2,
    ReadWrite = 3,
};

pub const PTEH = packed struct {
    asid: u8 = undefined,
    _: u2 = undefined,
    vpn: u22 = undefined,
};

pub const PTEL = packed struct {
    /// Write-through bit
    wt: bool = undefined,
    /// Share status bit
    sh: bool = undefined,
    /// Dirty bit
    d: bool = undefined,
    /// Cacheability bit
    c: bool = undefined,
    /// Page size bit
    sz0: u1 = undefined,
    /// Protection key data
    pr: ProtectionKey = undefined,
    /// Page size bit
    sz1: u1 = undefined,
    /// Validity bit
    v: bool = undefined,

    _r0: u1 = 0,
    /// Physical page number
    ppn: u19 = undefined,
    /// TC or Timing control bit in UTBL Entry, not used.
    _r1: u3 = 0,

    pub fn sz(self: @This()) u2 {
        return @as(u2, self.sz1) << 1 | self.sz0;
    }
};

pub const PTEA = packed struct {
    sa: u3 = undefined,
    tc: u1 = undefined,
    _: u28 = 0,
};

pub const MMUCR = packed struct {
    /// Address translation bit
    at: bool = false,

    _r0: u1 = undefined,
    /// TLB invalidate
    ti: bool = false,

    _r1: u5 = undefined,
    /// Single virtual mode bit.
    sv: bool = false,
    /// Store queue mode bit.
    sqmd: u1 = 0,
    /// UTLB replace counter
    urc: u6 = 0,

    _r2: u2 = undefined,

    /// UTLB replace boundary
    urb: u6 = 0,

    _r3: u2 = undefined,

    /// Least recently used ITLB
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
    v: bool = false, // Validity bit

    tc: u1 = undefined, // Timing control bit
    sa: u3 = undefined, // Space attribute bits
    wt: bool = undefined, // Write-through bit
    d: bool = undefined, // Dirty bit
    pr: ProtectionKey = undefined, // Protection key data
    c: bool = undefined, // Cacheability bit
    sh: bool = undefined, // Share status bit, 1 => Shared
    sz: u2 = undefined, // Page size
    ppn: u19 = undefined, // Physical page number
    _: u2 = 0,

    pub inline fn valid(self: @This()) bool {
        return self.v;
    }

    pub inline fn match(self: @This(), check_asid: bool, asid: u8, vpn: u22) bool {
        return self.valid() and (!check_asid or self.sh or self.asid == asid) and vpn_match(self.vpn, vpn, self.sz);
    }

    pub inline fn translate(self: @This(), virtual_address: u32) u32 {
        const physical_page = @as(u32, @intCast(self.ppn)) << 10;
        const mask = (@as(u32, 1) << switch (self.sz) {
            0b00 => 10, // 1-Kbyte page
            0b01 => 12, // 4-Kbyte page
            0b10 => 16, // 64-Kbyte page
            0b11 => 20, // 1-Mbyte page
        }) - 1;
        return (physical_page & ~mask) | (virtual_address & mask);
    }

    pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        try writer.print("UTLB(.vpn = {X:0>8}, .ppn = {X:0>8}, .asid = {X}, .v = {}, .tc = {}, .sa = {}, .wt = {}, .d = {}, .pr = {}, .c = {}, .sh = {}, .sz = {})", .{
            @as(u32, self.vpn) << 10,
            @as(u32, self.ppn) << 10,
            self.asid,
            self.v,
            self.tc,
            self.sa,
            self.wt,
            self.d,
            self.pr,
            self.c,
            self.sh,
            self.sz,
        });
    }
};

pub const UTLBAddressData = packed struct(u32) {
    asid: u8,
    v: bool,
    d: bool,
    vpn: u22,
};

pub const UTLBArrayData1 = PTEL;

pub const UTLBArrayData2 = PTEA;

pub fn vpn_match(lhs: u22, rhs: u22, sz: u2) bool {
    switch (sz) {
        0b00 => return lhs == rhs, // For 1-kbyte page: upper 22 bits of virtual address
        0b01 => return (lhs & 0b1111_1111_1111_1111_1111_00) == (rhs & 0b1111_1111_1111_1111_1111_00), // For 4-kbyte page: upper 20 bits of virtual address
        0b10 => return (lhs & 0b1111_1111_1111_1111_0000_00) == (rhs & 0b1111_1111_1111_1111_0000_00), // For 64-kbyte page: upper 16 bits of virtual address
        0b11 => return (lhs & 0b1111_1111_1111_0000_0000_00) == (rhs & 0b1111_1111_1111_0000_0000_00), // For 1-Mbyte page: upper 12 bits of virtual address
    }
}
