const std = @import("std");

pub const ProtectionKey = enum(u2) {
    PrivilegedReadOnly = 0,
    PrivilegedReadWrite = 1,
    ReadOnly = 2,
    ReadWrite = 3,

    pub fn privileged(self: @This()) bool {
        return self == .PrivilegedReadOnly or self == .PrivilegedReadWrite;
    }

    pub fn read_only(self: @This()) bool {
        return self == .ReadOnly or self == .PrivilegedReadOnly;
    }
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
    /// TLB invalidate. Writing 1 to this bit invalidates (clears to 0) all valid UTLB/ITLB bits. This bit always returns 0 when read.
    ti: bool = false,

    _r1: u5 = undefined,
    /// Single virtual mode bit. 0: Multiple virtual memory mode. 1: Single virtual memory mode.
    sv: bool = false,
    /// Store queue mode bit. 0: User/privileged access possible. 1: Privileged access possible (address error exception in case of user acces).
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

// Memory layout isn't hardware accurate
pub const TLBEntry = struct {
    asid: u8 = 0xAA, // Address space identifier
    vpn: u22 = 0xBAD, // Virtual page number
    v: bool = false, // Validity bit

    tc: u1 = 0, // Timing control bit
    sa: u3 = 0, // Space attribute bits
    wt: bool = false, // Write-through bit
    d: bool = false, // Dirty bit
    pr: ProtectionKey = .ReadWrite, // Protection key data
    c: bool = true, // Cacheability bit
    sh: bool = false, // Share status bit, 1 => Shared
    sz: u2 = 0, // Page size

    _ppn: u32 = 0xDEADCAFE, // Physical page number, stored as u19 << 10

    pub fn get_ppn(self: *const @This()) u19 {
        return @truncate(self._ppn >> 10);
    }

    pub fn set_ppn(self: *@This(), ppn: u19) void {
        self._ppn = @as(u32, ppn) << 10;
    }

    pub inline fn valid(self: *const @This()) bool {
        return self.v;
    }

    pub inline fn match(self: *const @This(), multiple_virtual_memory_mode: bool, asid: u8, vpn: u22) bool {
        const check_asid = !self.sh and multiple_virtual_memory_mode;
        if (check_asid) {
            return self.valid() and (self.asid == asid) and vpn_match(self.vpn, vpn, self.sz);
        } else {
            return self.valid() and vpn_match(self.vpn, vpn, self.sz);
        }
    }

    pub inline fn translate(self: *const @This(), virtual_address: u32) u32 {
        const mask: u32 = switch (self.sz) {
            0b00 => (1 << 10) - 1, // 1-Kbyte page
            0b01 => (1 << 12) - 1, // 4-Kbyte page
            0b10 => (1 << 16) - 1, // 64-Kbyte page
            0b11 => (1 << 20) - 1, // 1-Mbyte page
        };
        return (self._ppn & ~mask) | (virtual_address & mask);
    }

    pub inline fn first_physical_address(self: *const @This()) u32 {
        return self.translate(@as(u32, self.vpn) << 10);
    }

    pub inline fn size(self: *const @This()) u32 {
        return @as(u32, 1) << switch (self.sz) {
            0b00 => 10, // 1-Kbyte page
            0b01 => 12, // 4-Kbyte page
            0b10 => 16, // 64-Kbyte page
            0b11 => 20, // 1-Mbyte page
        };
    }

    pub fn format(self: *const @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        try writer.print("TLB{{.vpn = {X:0>8}, .ppn = {X:0>8}, .asid = {X}, .v = {}, .tc = {}, .sa = {}, .wt = {}, .d = {}, .pr = {s}, .c = {}, .sh = {}, .sz = {}}}", .{
            @as(u32, self.vpn) << 10,
            self._ppn,
            self.asid,
            self.v,
            self.tc,
            self.sa,
            self.wt,
            self.d,
            @tagName(self.pr),
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

pub inline fn vpn_match(lhs: u22, rhs: u22, sz: u2) bool {
    switch (sz) {
        0b00 => return lhs == rhs, // For 1-kbyte page: upper 22 bits of virtual address
        0b01 => return (lhs & 0b1111_1111_1111_1111_1111_00) == (rhs & 0b1111_1111_1111_1111_1111_00), // For 4-kbyte page: upper 20 bits of virtual address
        0b10 => return (lhs & 0b1111_1111_1111_1111_0000_00) == (rhs & 0b1111_1111_1111_1111_0000_00), // For 64-kbyte page: upper 16 bits of virtual address
        0b11 => return (lhs & 0b1111_1111_1111_0000_0000_00) == (rhs & 0b1111_1111_1111_0000_0000_00), // For 1-Mbyte page: upper 12 bits of virtual address
    }
}
