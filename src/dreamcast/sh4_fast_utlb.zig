const sh4 = @import("sh4.zig");
const mmu = sh4.mmu;

const host_memory = @import("host/host_memory.zig");

pub const None = struct {
    pub fn init() !@This() {
        return .{};
    }
    pub fn deinit(_: *@This()) void {}
    pub fn reset(_: *@This()) !void {}
    pub fn load(_: *@This(), _: mmu.TLBEntry, _: u8) void {}
    pub fn evict(_: *@This(), _: mmu.TLBEntry) void {}
};

/// Uses a virtual memory lookup table of indices into the regular UTLB to speed up UTLB lookups.
pub const Indexed = struct {
    entries: []u8,

    pub fn init() !@This() {
        return .{
            .entries = try host_memory.virtual_alloc(u8, @as(u32, 1) << (22 + 8)),
        };
    }

    pub fn deinit(self: *@This()) void {
        host_memory.virtual_dealloc(self.entries);
    }

    pub fn reset(self: *@This()) !void {
        host_memory.virtual_dealloc(self.entries);
        self.entries = try host_memory.virtual_alloc(u8, @as(u32, 1) << (22 + 8));
        for (self.cpu().utlb, 0..) |entry, idx| {
            if (entry.valid()) self.load(entry, @intCast(idx));
        }
    }

    pub inline fn lookup(self: *const @This(), asid: u8, vpn: u22) !mmu.TLBEntry {
        const key: u32 = @as(u32, asid) << 22 | vpn;
        const idx = self.entries[key];
        if (idx != 0) {
            const entry = self.cpu().utlb[idx - 1];
            if (sh4.EmulateAccessViolation and self.cpu().sr.md == 0 and entry.pr.privileged())
                return error.TLBProtectionViolation;
            return entry;
        }
        return error.TLBMiss;
    }

    pub inline fn evict(self: *@This(), entry: mmu.TLBEntry) void {
        self.update(entry, 0);
    }

    pub inline fn load(self: *@This(), entry: mmu.TLBEntry, idx: u8) void {
        if (entry.valid()) {
            self.update(entry, idx + 1);
        } else {
            self.update(entry, 0);
        }
    }

    inline fn update(self: *@This(), entry: mmu.TLBEntry, idx: u8) void {
        const mmucr = self.cpu().read_p4_register(mmu.MMUCR, .MMUCR);
        // Single Virtual Memory Mode: ASID does not matter and all checks will use 0.
        if (mmucr.sv) {
            self.set(0, entry, idx);
        } else {
            // Shared: Update it for all ASID
            if (entry.sh) {
                for (0..256) |asid|
                    self.set(@intCast(asid), entry, idx);
            } else {
                self.set(entry.asid, entry, idx);
            }
        }
    }

    inline fn set(self: *@This(), asid: u8, entry: mmu.TLBEntry, idx: u8) void {
        const key: u32 = @as(u32, asid) << 22 | (entry.vpn & (~((entry.size() - 1) >> 10)));
        @memset(self.entries[key..][0 .. entry.size() >> 10], idx);
    }

    inline fn cpu(self: *const @This()) *const sh4.SH4 {
        return @alignCast(@fieldParentPtr("fast_utlb", self));
    }
};

/// Keeps mapping alive in the UTLB fast lookup table as long as they're not explicitely unvalidated by software, effectively acting as a infinitely sized UTLB,
/// reducing the overhead of the software having to maintain its own mappings into the real UTLB.
/// This relies on the default value of virtual allocation begin 0, thus the `v` bit being false at initialization.
pub const Cached = struct {
    entries: []mmu.TLBEntry,

    pub fn init() !@This() {
        return .{
            .entries = try host_memory.virtual_alloc(mmu.TLBEntry, @as(u32, 1) << (22 + 8)),
        };
    }

    pub fn deinit(_: *@This()) void {}

    pub fn reset(self: *@This()) !void {
        host_memory.virtual_dealloc(self.entries);
        self.entries = try host_memory.virtual_alloc(mmu.TLBEntry, @as(u32, 1) << (22 + 8));
        for (self.cpu().utlb) |entry| {
            if (entry.valid()) self.update(entry);
        }
    }

    pub inline fn lookup(self: *const @This(), asid: u8, vpn: u22) !mmu.TLBEntry {
        const key: u32 = @as(u32, asid) << 22 | vpn;
        const entry = self.entries[key];
        if (entry.valid()) {
            if (sh4.EmulateAccessViolation and self.cpu().sr.md == 0 and entry.pr.privileged())
                return error.TLBProtectionViolation;
            return entry;
        }
        return error.TLBMiss;
    }

    pub inline fn evict(_: *@This(), _: mmu.TLBEntry) void {}

    pub inline fn load(self: *@This(), entry: mmu.TLBEntry, _: u8) void {
        self.update(entry);
    }

    inline fn update(self: *@This(), entry: mmu.TLBEntry) void {
        const mmucr = self.cpu().read_p4_register(mmu.MMUCR, .MMUCR);
        // Single Virtual Memory Mode: ASID does not matter and all checks will use 0.
        if (mmucr.sv) {
            self.set(0, entry);
        } else {
            // Shared: Update it for all ASID
            if (entry.sh) {
                for (0..256) |asid|
                    self.set(@intCast(asid), entry);
            } else {
                self.set(entry.asid, entry);
            }
        }
    }

    inline fn set(self: *@This(), asid: u8, entry: mmu.TLBEntry) void {
        const key: u32 = @as(u32, asid) << 22 | (entry.vpn & (~((entry.size() - 1) >> 10)));
        @memset(self.entries[key..][0 .. entry.size() >> 10], entry);
    }

    inline fn cpu(self: *const @This()) *const sh4.SH4 {
        return @alignCast(@fieldParentPtr("fast_utlb", self));
    }
};
