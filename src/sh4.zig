// Hitachi SH-4
// FIXME: Exact model is actually SH7091, I think.

const std = @import("std");
const builtin = @import("builtin");
const common = @import("./common.zig");
const termcolor = @import("termcolor");

pub const sh4_log = std.log.scoped(.sh4);

const Dreamcast = @import("dreamcast.zig").Dreamcast;

pub const mmu = @import("./mmu.zig");
pub const P4 = @import("./sh4_p4.zig");
pub const P4Register = P4.P4Register;
const Interrupts = @import("sh4_interrupts.zig");
const Interrupt = Interrupts.Interrupt;

pub const Exception = @import("sh4_exceptions.zig").Exception;

const addr_t = common.addr_t;

pub const sh4_instructions = @import("sh4_instructions.zig");
pub const sh4_disassembly = @import("sh4_disassembly.zig");
pub const interpreter_handlers = @import("sh4_interpreter_handlers.zig");

const HardwareRegisters = @import("hardware_registers.zig");
const HardwareRegister = HardwareRegisters.HardwareRegister;

pub const ExperimentalFullMMUSupport = false;

pub const SR = packed struct(u32) {
    t: bool = false, // True/False condition or carry/borrow bit. NOTE: Undefined at startup, set for repeatability.
    s: bool = false, // Specifies a saturation operation for a MAC instruction. NOTE: Undefined at startup, set for repeatability.

    _r0: u2 = 0,

    imask: u4 = 0xF, // Interrupt mask level
    q: bool = false, // State for divide step. NOTE: Undefined at startup, set for repeatability.
    m: bool = false, // NOTE: Undefined at startup, set for repeatability.

    _r1: u5 = 0,

    fd: bool = false, // FPU disable bit.

    _r2: u12 = 0,

    bl: bool = true, // Exception/interrupt block bit (set to 1 by a reset, exception, or interrupt).
    rb: u1 = 1, // General register bank specifier in privileged mode (set to 1 by a reset, exception or interrupt).
    md: u1 = 1, // Processor mode. MD = 0: User mode (Some instructions cannot be executed, and some resources cannot be accessed). MD = 1: Privileged mode.

    _r3: u1 = 0,
};

pub const FPSCR = packed struct(u32) {
    rm: enum(u2) { RoundToNearest = 0, RoundToZero = 1, _ } = .RoundToZero, // Rounding mode
    inexact: bool = false,
    underflow: bool = false,
    overflow: bool = false,
    dividion_by_zero: bool = false,
    invalid_operation: bool = false,
    enable_inexact: bool = false,
    enable_underflow: bool = false,
    enable_overflow: bool = false,
    enable_dividion_by_zero: bool = false,
    enable_invalid_operation: bool = false,
    cause_inexact: bool = false,
    cause_underflow: bool = false,
    cause_overflow: bool = false,
    cause_dividion_by_zero: bool = false,
    cause_invalid_operation: bool = false,
    cause_fpu_error: bool = false,
    dn: bool = false, // Denormalization mode. True means "Denormals are zeroes"
    pr: u1 = 1, // Precision mode
    sz: u1 = 0, // Transfer size mode
    fr: u1 = 0, // Floating-point register bank

    _: u10 = 0, // Reserved
};

const TimerRegisters = [3]struct { counter: P4Register, control: P4Register, constant: P4Register, interrupt: Interrupt }{
    .{ .counter = P4Register.TCNT0, .control = P4Register.TCR0, .constant = P4Register.TCOR0, .interrupt = Interrupt.TUNI0 },
    .{ .counter = P4Register.TCNT1, .control = P4Register.TCR1, .constant = P4Register.TCOR1, .interrupt = Interrupt.TUNI1 },
    .{ .counter = P4Register.TCNT2, .control = P4Register.TCR2, .constant = P4Register.TCOR2, .interrupt = Interrupt.TUNI2 },
};

pub const VirtualAddr = packed struct {
    region: u3,
    addr: u29,
};

pub const StoreQueueAddr = packed struct(u32) {
    _: u2 = 0,
    lw_spec: u3 = 0,
    sq: u1 = 0,
    address: u20 = 0,
    spec: u6 = 0b111000,
};

pub const Instr = packed union {
    value: u16,
    // Reminder: We're in little endian.
    nmd: packed struct { d: u4 = undefined, m: u4 = undefined, n: u4 = undefined, _: u4 = undefined },
    nd8: packed struct { d: u8, n: u4 = undefined, _: u4 = undefined },
    d12: packed struct { d: u12, _: u4 = undefined },
};

comptime {
    std.debug.assert(@bitSizeOf(Instr) == @bitSizeOf(u16));
}

const ExecutionState = enum {
    Running,
    Sleep, // CPG: Operating, CPU: Halted, On-chip Peripheral Modules: Operating, Exiting Method: Interrupt, Reset
    DeepSleep, // CPG: Operating, CPU: Halted, On-chip Peripheral Modules: Operating, Exiting Method: Interrupt, Reset
    Standby, // CPG: Halted, CPU: Halted, On-chip Peripheral Modules: Halted, Exiting Method: Interrupt, Reset
    ModuleStandby, // Not implemented at all
};

const DMACChannels: [3]struct { chcr: P4Register, sar: P4Register, dar: P4Register, dmatcr: P4Register, dmte: Interrupts.Interrupt } = .{
    .{ .chcr = .CHCR0, .sar = .SAR0, .dar = .DAR0, .dmatcr = .DMATCR0, .dmte = .DMTE0 },
    .{ .chcr = .CHCR1, .sar = .SAR1, .dar = .DAR1, .dmatcr = .DMATCR1, .dmte = .DMTE1 },
    .{ .chcr = .CHCR2, .sar = .SAR2, .dar = .DAR2, .dmatcr = .DMATCR2, .dmte = .DMTE2 },
};

pub var DebugHooks: struct {
    read8: ?*const fn (addr: addr_t) u8 = null,
    read16: ?*const fn (addr: addr_t) u16 = null,
    read32: ?*const fn (addr: addr_t) u32 = null,
    read64: ?*const fn (addr: addr_t) u64 = null,
    write8: ?*const fn (addr: addr_t, value: u8) void = null,
    write16: ?*const fn (addr: addr_t, value: u16) void = null,
    write32: ?*const fn (addr: addr_t, value: u32) void = null,
    write64: ?*const fn (addr: addr_t, value: u64) void = null,
} = .{};

comptime {
    // Make sure the compiler can treat those as a single u64.
    std.debug.assert(@offsetOf(SH4, "mach") == @offsetOf(SH4, "macl") + 4);
}

// DCA3 Hack
const OperandCacheState = struct {
    addr: [256]u32 = undefined, // Tag (19bits)
    dirty: [256]bool = .{false} ** 256, // U bit
    // V bit not implemented

    pub fn serialize(self: *const @This(), writer: anytype) !usize {
        var bytes: usize = 0;
        bytes += try writer.write(std.mem.sliceAsBytes(self.addr[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self.dirty[0..]));
        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: anytype) !usize {
        var bytes: usize = 0;
        bytes += try reader.read(std.mem.sliceAsBytes(self.addr[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self.dirty[0..]));
        return bytes;
    }
};

pub const SH4 = struct {
    pub const EnableTRAPACallback = false;

    // General Registers
    r: [16]u32 = undefined, // R0-R15
    r_bank: [8]u32 = undefined, // R0-R7_BANK

    // Control Registers
    sr: SR = .{}, // NOTE: Don't set directly, see set_sr.
    gbr: u32 = undefined,
    ssr: u32 = undefined,
    spc: u32 = undefined,
    sgr: u32 = undefined,
    dbr: u32 = 0, // undefined,
    vbr: u32 = 0x00000000,

    // System Registers
    pc: u32 = 0xA0000000, // Program counter
    macl: u32 = undefined, // Multiply-and-accumulate register low
    mach: u32 = undefined, // Multiply-and-accumulate register high
    pr: u32 = undefined, // Procedure register
    fpscr: FPSCR = .{}, // Floating-point status/control register
    fpul: u32 = undefined, // Floating-point communication register

    fp_banks: [2]extern union {
        fr: [16]f32,
        dr: [8]f64,
        qf: [4]f128,
    } = undefined,

    store_queues: [2][8]u32 align(32) = undefined,
    _operand_cache: []u8 align(4),
    p4_registers: []u8 align(4),
    utlb: []mmu.UTLBEntry,

    interrupt_requests: u64 = 0,
    // NOTE: These are not serialized as they can easily be computed from P4 registers IPRA/B/C. See compute_interrupt_priorities.
    _sorted_interrupts: [41]Interrupt = Interrupts.DefaultInterruptPriorities, // Interrupts ordered by their current priority.
    _interrupts_indices: [41]u8 = .{0} ** 41, // Inverse mapping of _sorted_interrupts
    _interrupt_levels: [41]u32 = Interrupts.DefaultInterruptLevels,

    _mmu_enabled: bool = false, // Cached value of MMUCR.at
    _last_timer_update: [3]u64 = .{0} ** 3,

    execution_state: ExecutionState = .Running,

    on_trapa: if (EnableTRAPACallback) ?struct {
        callback: *const fn (userdata: *anyopaque) void, // Debugging callback
        userdata: *anyopaque,
    } else void = if (EnableTRAPACallback) null else {},
    debug_trace: bool = true, // FIXME: Turn this off :)

    _allocator: std.mem.Allocator,
    _dc: ?*Dreamcast = null,
    _pending_cycles: u32 = 0,

    _operand_cache_state: *OperandCacheState, // DCA3 Hacks

    // Allows passing a null DC for testing purposes (Mostly for instructions that do not need access to RAM).
    pub fn init(allocator: std.mem.Allocator, dc: ?*Dreamcast) !SH4 {
        sh4_instructions.init_table();

        var sh4: SH4 = .{
            ._dc = dc,
            ._operand_cache = try allocator.alloc(u8, 0x2000), // NOTE: Actual Operand cache is 16k, but we're only emulating the RAM accessible part, which is 8k.
            .p4_registers = try allocator.alloc(u8, 0x1000),
            .utlb = try allocator.alloc(mmu.UTLBEntry, 64),
            ._allocator = allocator,
            ._operand_cache_state = try allocator.create(OperandCacheState),
        };

        @memset(sh4._operand_cache, 0);
        sh4._operand_cache_state.* = .{};

        // P4 registers are remapped on this smaller range. See p4_register_addr.
        //   Addresses starts with FF/1F, this can be ignored.
        //   Then, there are 5 bits that separate registers into different functions.
        //   Within each function, only the lower 7 bits of the address are relevant,
        //   the rest is always 0. By shifting the 5 'function' bits, we can easily
        //   and cheaply remap it to a way smaller memory range.
        //   The one exception is the virtual register SDMR (SDMR2/SDMR3), which is
        //   handled by read8 (but not emulated).
        // + Operand cache RAM mode also clashes with this, it's also dealt with in read/write functions.
        // + Two performance registers (PMCR1/2, exclusive to SH7091 afaik) also screw this pattern,
        //   I ignore them in read16/write16.
        @memset(sh4.p4_registers, 0);

        sh4.reset();

        return sh4;
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.free(self.utlb);
        self._allocator.free(self.p4_registers);
        self._allocator.free(self._operand_cache);
        self._allocator.destroy(self._operand_cache_state);
    }

    pub fn reset(self: *@This()) void {
        // NOTE: These should be undefined, but it prevents the bios to run in debug mode, at least since the update to zig 0.13.
        self.r = .{0} ** 16;
        self.r_bank = .{0} ** 8;

        self.R(0xF).* = 0x8C00F400;
        self.set_sr(.{});
        self.gbr = undefined;
        self.ssr = undefined;
        self.spc = undefined;
        self.sgr = undefined;
        self.dbr = undefined;
        self.vbr = 0x00000000;
        self.mach = undefined;
        self.macl = undefined;
        self.pr = undefined;
        self.pc = 0xA0000000;
        self.fpscr = .{};
        self.fpul = undefined;
        self.fp_banks = undefined;

        self.p4_register(mmu.PTEH, .PTEH).* = .{};
        self.p4_register(mmu.PTEL, .PTEL).* = .{};
        self.p4_register(u32, .TTB).* = 0;
        self.p4_register(u32, .TEA).* = 0;
        self.p4_register(u32, .MMUCR).* = 0;

        self.p4_register(u32, ._FF000030).* = 0x040205C1;

        self.p4_register(u8, .BASRA).* = undefined;
        self.p4_register(u8, .BASRB).* = undefined;
        self.p4_register(u32, .CCR).* = 0;

        self.p4_register(u32, .TRA).* = 0;
        self.p4_register(u32, .EXPEVT).* = 0;
        self.p4_register(u32, .INTEVT).* = 0;

        self.p4_register(u32, .QACR0).* = undefined;
        self.p4_register(u32, .QACR1).* = undefined;
        self.p4_register(u32, .BARA).* = undefined;
        self.p4_register(u32, .BAMRA).* = undefined;

        self.p4_register(u32, .MCR).* = 0xC0091224;
        // Not emulated, not readable, and mess with our P4 remapping.
        //self.p4_register(u16, .SDMR).* = 0x00FF;
        self.p4_register(u16, .RTCSR).* = 0xA510;
        self.p4_register(u16, .RTCNT).* = 0xA500;
        self.p4_register(u16, .RTCOR).* = 0xA55E;
        self.p4_register(u16, .RFCR).* = undefined;

        self.p4_register(u32, .BCR1).* = 0;
        self.p4_register(u32, .BCR2).* = 0x3FFC;
        self.p4_register(u32, .PCTRA).* = 0;

        self.p4_register(P4.FRQCR, .FRQCR).* = .{};

        self.p4_register(u16, .ICR).* = 0x0000;
        self.p4_register(u16, .IPRA).* = 0x0000;
        self.p4_register(u16, .IPRB).* = 0x0000;
        self.p4_register(u16, .IPRC).* = 0x0000;
        self.compute_interrupt_priorities();

        self.p4_register(u8, .TOCR).* = 0x00;
        self.p4_register(u8, .TSTR).* = 0x00;
        for (TimerRegisters) |timer| {
            self.p4_register(u32, timer.constant).* = 0xFFFFFFFF;
            self.p4_register(u32, timer.counter).* = 0xFFFFFFFF;
            self.p4_register(u16, timer.control).* = 0x0000;
        }
        self._last_timer_update = .{0} ** 3;

        self.p4_register(u8, .SCBRR2).* = 0xFF;
        self.p4_register(u16, .SCSCR2).* = 0x0000;
        self.p4_register(u16, .SCFSR2).* = 0x0060;
        self.p4_register(u16, .SCFCR2).* = 0x0000;
        self.p4_register(u16, .SCFDR2).* = 0x0000;
        self.p4_register(u16, .SCFTDR2).* = 0x0000;
        self.p4_register(u16, .SCSPTR2).* = 0x0000;
        self.p4_register(u16, .SCLSR2).* = 0x0000;

        for (0..self.utlb.len) |i| {
            self.utlb[i].v = false;
        }
        self._mmu_enabled = false;
    }

    pub fn software_reset(self: *@This()) void {
        sh4_log.warn("  SOFTWARE RESET!", .{});

        self.r = undefined;
        self.r_bank = undefined;
        self.set_sr(.{});
        self.gbr = undefined;
        self.ssr = undefined;
        self.spc = undefined;
        self.sgr = undefined;
        self.dbr = undefined;
        self.vbr = 0x00000000;
        self.mach = undefined;
        self.macl = undefined;
        self.pr = undefined;
        self.pc = 0xA0000000 - 2; // FIXME: The instruction loop will increase it.
        self.set_fpscr(@as(u32, @bitCast(FPSCR{})));
        self.fpul = undefined;
        self.fp_banks = undefined;

        self.p4_register(mmu.PTEH, .PTEH).* = .{};
        self.p4_register(mmu.PTEL, .PTEL).* = .{};
        self.p4_register(u32, .TTB).* = 0;
        self.p4_register(u32, .TEA).* = 0;
        self.p4_register(u32, .MMUCR).* = 0;
        // BASRA Held
        // BASRB Held
        self.p4_register(u32, .CCR).* = 0;
        self.p4_register(u32, .TRA).* = 0;
        self.p4_register(u32, .EXPEVT).* = 0x00000020;
        // INTEVT Held
        self.p4_register(u32, .QACR0).* = undefined;
        self.p4_register(u32, .QACR1).* = undefined;
        // BARA Held
        // BAMRA Held

        // BCR1 Held
        // BCR2 Held

        self._mmu_enabled = false;
    }

    // Reset state to after bios.
    pub fn state_after_boot_rom(self: *@This()) void {
        self.set_sr(@bitCast(@as(u32, 0x400000F1)));
        self.set_fpscr(@as(u32, 0x00040001));

        self.R(0x0).* = 0xAC0005D8;
        self.R(0x1).* = 0x00000009;
        self.R(0x2).* = 0xAC00940C;
        self.R(0x3).* = 0x00000000;
        self.R(0x4).* = 0xAC008300;
        self.R(0x5).* = 0xF4000000;
        self.R(0x6).* = 0xF4002000;
        self.R(0x7).* = 0x00000044; // 0x00000070;
        self.R(0x8).* = 0x00000000;
        self.R(0x9).* = 0x00000000;
        self.R(0xA).* = 0x00000000;
        self.R(0xB).* = 0x00000000;
        self.R(0xC).* = 0x00000000;
        self.R(0xD).* = 0x00000000;
        self.R(0xE).* = 0x00000000;
        self.R(0xF).* = 0x8D000000;

        self.r_bank[0] = 0xDFFFFFFF;
        self.r_bank[1] = 0x500000F1;
        self.r_bank[2] = 0x00000000;
        self.r_bank[3] = 0x00000000;
        self.r_bank[4] = 0x00000000;
        self.r_bank[5] = 0x00000000;
        self.r_bank[6] = 0x00000000;
        self.r_bank[7] = 0x00000000;

        self.gbr = 0x8C000000;
        self.ssr = 0x40000001;
        self.spc = 0x8C000776;
        self.sgr = 0x8D000000;
        self.dbr = 0x8C000010;
        self.vbr = 0x8C000000;
        self.pr = 0x0C00043C; // 0xAC00043C;
        self.fpul = 0x00000000;

        self.pc = 0xAC008300; // Start address of IP.bin Licence screen
    }

    pub fn set_sr(self: *@This(), value: SR) void {
        const prev_rb = if (self.sr.md == 1) self.sr.rb else 0;
        const new_rb = if (value.md == 1) value.rb else 0;
        if (new_rb != prev_rb) {
            std.mem.swap([8]u32, self.r[0..8], &self.r_bank);
        }
        self.sr = @bitCast(@as(u32, @bitCast(value)) & 0x700083F3);
        self.sr.rb = new_rb; // In case it was forced to 0 by md.
    }

    fn update_sse_settings(self: *@This()) void {
        // Adjust SSE settings to match the guest system configuration
        var mxcsr: u32 = 0x1F80; // Default MXCSR value
        if (self.fpscr.dn) {
            mxcsr |= 0x0040; // DAZ - Denormals are zeros
        }
        switch (self.fpscr.rm) {
            .RoundToNearest => mxcsr |= 0x0000, // Yes, there's no bit associated with this mode, it's the default.
            .RoundToZero => mxcsr |= 0x6000,
            else => {},
        }
        asm volatile ("ldmxcsr (%%rax)"
            :
            : [_] "{rax}" (&mxcsr),
            : "rax"
        );
    }

    pub fn set_fpscr(self: *@This(), value: u32) void {
        const new_value: FPSCR = @bitCast(value & 0x003FFFFF);
        if (new_value.fr != self.fpscr.fr) {
            std.mem.swap(@TypeOf(self.fp_banks[0]), &self.fp_banks[0], &self.fp_banks[1]);
        }
        self.fpscr = new_value;
        self.update_sse_settings();
    }

    inline fn read_operand_cache(self: *const @This(), comptime T: type, virtual_addr: addr_t) T {
        return @constCast(self).operand_cache(T, virtual_addr).*;
    }

    inline fn write_operand_cache(self: *@This(), comptime T: type, virtual_addr: addr_t, value: T) void {
        if (self.read_p4_register(P4.CCR, .CCR).ora == 0) {
            sh4_log.err(termcolor.red("Write to operand cache with RAM mode disabled: @{X:0>8} = {X:0>8}"), .{ virtual_addr, value });
            return;
        }
        self.operand_cache(T, virtual_addr).* = value;
    }

    pub inline fn operand_cache_lines(self: *const @This()) [][8]u32 {
        return @as([*][32 / 4]u32, @alignCast(@ptrCast(self._operand_cache.ptr)))[0..256];
    }

    inline fn operand_cache(self: *@This(), comptime T: type, virtual_addr: addr_t) *T {
        if ((comptime builtin.mode == .Debug) and self.read_p4_register(P4.CCR, .CCR).ora == 0)
            sh4_log.err(termcolor.red("Read to operand cache with RAM mode disabled: @{X:0>8}"), .{virtual_addr});

        // Half of the operand cache can be used as RAM when CCR.ORA == 1, and some games do.
        // std.debug.assert(self.read_p4_register(P4.CCR, .CCR).ora == 1);
        std.debug.assert(virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF);

        // NOTE: The operand cache as RAM (8K) is split into two 4K areas and the correct addressing changes depending on the CCR.OIX bit.
        //       I suspect most games will either stick to a single 4K area, or use one of the contiguous ranges.
        //       (For example 0x7C001000-0x7C002FFF for CCR.OIX == 0, or the unique 0x7DFFF000-0x7E000FFF for CCR.OIX == 1)
        //       Assuming this gives us a really nice performance boost for games that use the operand cache in this way.
        if (comptime true) {
            // These are seemingly not automatically optimized away, not sure why.
            if (comptime builtin.mode == .Debug) {
                // We can't easily assert for the CCR.OIX == 0 case since there are many contiguous ranges, and in practice most adresses can be
                // used in a contiguous manner. Only the first and last 4K area cannot.
                // Ranges like 0x7C003000-0x7C004FFF (Area 2 then Area 1) are not contiguous, but won't repeat, so they **might** be fine?...
                std.debug.assert(self.read_p4_register(P4.CCR, .CCR).oix == 1 or (virtual_addr >= 0x7C001000 and virtual_addr <= 0x7FFFF000));
                // This checks the only contiguous range when CCR.OIX = 1.
                std.debug.assert(self.read_p4_register(P4.CCR, .CCR).oix == 0 or (virtual_addr >= 0x7DFFF000 and virtual_addr <= 0x7E000FFF));
            }

            return @alignCast(@ptrCast(&self._operand_cache[virtual_addr & 0x1FFF]));
        } else {
            // Correct addressing, in case we end up needing it.
            if (self.read_p4_register(P4.CCR, .CCR).oix == 0) {
                // 0x7C00_0000 - 0x7C00_0FFF : RAM Area 1
                // 0x7C00_1000 - 0x7C00_1FFF : RAM Area 1
                // 0x7C00_2000 - 0x7C00_2FFF : RAM Area 2
                // 0x7C00_2000 - 0x7C00_2FFF : RAM Area 2
                // 0x7C00_3000 - 0x7C00_3FFF : RAM Area 1
                // 0x7C00_3000 - 0x7C00_3FFF : RAM Area 1
                // [...]
                const index = ((virtual_addr & 0x0000_2000) >> 1) | (virtual_addr & 0x0FFF);
                return @alignCast(@ptrCast(&self._operand_cache[index]));
            } else {
                // RAM Area 1 Mirroring from 0x7C00_0000 to 0x7DFF_FFFF
                // RAM Area 2 Mirroring from 0x7E00_0000 to 0x7FFF_FFFF
                const index = ((virtual_addr & 0x02000000) >> 13) | (virtual_addr & 0x0FFF);
                return @alignCast(@ptrCast(&self._operand_cache[index]));
            }
        }
    }

    pub inline fn read_p4_register(self: *const @This(), comptime T: type, r: P4Register) T {
        return @constCast(self).p4_register_addr(T, @intFromEnum(r)).*;
    }

    pub inline fn p4_register(self: *@This(), comptime T: type, r: P4Register) *T {
        return self.p4_register_addr(T, @intFromEnum(r));
    }

    pub inline fn p4_register_addr(self: *@This(), comptime T: type, addr: addr_t) *T {
        std.debug.assert(addr & 0xFF000000 == 0xFF000000 or addr & 0xFF000000 == 0x1F000000);
        std.debug.assert(addr & 0b0000_0000_0000_0111_1111_1111_1000_0000 == 0);

        const real_addr = ((0b0000_0000_1111_1000_0000_0000_0000_0000 & addr) >> 12) | (addr & 0b0111_1111);
        return @as(*T, @alignCast(@ptrCast(&self.p4_registers[real_addr])));
    }

    pub inline fn R(self: *@This(), r: u4) *u32 {
        return &self.r[r];
    }

    pub inline fn FR(self: *@This(), r: u4) *f32 {
        return &self.fp_banks[0].fr[r];
    }

    pub inline fn getDRPtr(self: *@This(), r: u4) *f64 {
        return &self.fp_banks[0].dr[r >> 1];
    }

    pub inline fn getDR(self: *@This(), r: u4) f64 {
        std.debug.assert(r & 1 == 0);
        return @bitCast((@as(u64, @as(u32, @bitCast(self.fp_banks[0].fr[(r & 0xE) + 0]))) << 32) | @as(u64, @as(u32, @bitCast(self.fp_banks[0].fr[(r & 0xE) + 1]))));
    }
    pub inline fn setDR(self: *@This(), r: u4, v: f64) void {
        std.debug.assert(r & 1 == 0);
        const fp: [2]f32 = @bitCast(v);
        self.fp_banks[0].fr[(r & 0xE) + 1] = fp[0];
        self.fp_banks[0].fr[(r & 0xE) + 0] = fp[1];
    }

    pub inline fn XF(self: *@This(), r: u4) *f32 {
        return &self.fp_banks[1].fr[r];
    }

    pub inline fn getXDPtr(self: *@This(), r: u4) *f64 {
        return &self.fp_banks[1].dr[r >> 1];
    }

    fn jump_to_interrupt(self: *@This()) void {
        sh4_log.debug(" => Jump to Interrupt: VBR: {X:0>8}, Code: {X:0>4}", .{ self.vbr, self.read_p4_register(u32, .INTEVT) });

        self.execution_state = .Running;
        self.spc = self.pc;
        self.ssr = @bitCast(self.sr);
        self.sgr = self.R(15).*;

        var new_sr = self.sr;
        new_sr.bl = true;
        new_sr.md = 1;
        new_sr.rb = 1;
        self.set_sr(new_sr);

        self.pc = self.vbr + 0x600;
    }

    pub fn report_address_exception(self: *@This(), virtual_address: addr_t) void {
        self.p4_register(u32, .TEA).* = virtual_address;
        self.p4_register(mmu.PTEH, .PTEH).vpn = @truncate(virtual_address >> 10);
    }

    pub fn jump_to_exception(self: *@This(), exception: Exception) void {
        sh4_log.warn("Jump to Exception: {}\n  VBR: {X:0>8}, Offset: {X:0>4}", .{ exception, self.vbr, exception.offset() });

        self.spc = self.pc;
        self.ssr = @bitCast(self.sr);
        self.sgr = self.R(15).*;

        var new_sr = self.sr;
        new_sr.bl = true;
        new_sr.md = 1;
        new_sr.rb = 1;
        self.set_sr(new_sr);

        self.p4_register(u16, .EXPEVT).* = exception.code();

        const UserBreak = false; // TODO
        if (self.read_p4_register(P4.BRCR, .BRCR).ubde == 1 and UserBreak) {
            self.pc = self.dbr;
        } else {
            self.pc = self.vbr + exception.offset();
        }
    }

    pub fn handle_interrupts(self: *@This()) void {
        // When the BL bit in SR is 0, exceptions and interrupts are accepted.

        // See h14th002d2.pdf page 665 (or 651)
        if (!self.sr.bl or self.execution_state != .Running) {
            if (self.interrupt_requests != 0) {
                const int_index = @ctz(self.interrupt_requests);
                const interrupt = self._sorted_interrupts[int_index];
                // Check it against the cpu interrupt mask
                if (self._interrupt_levels[@intFromEnum(interrupt)] > self.sr.imask) {
                    self.interrupt_requests &= ~(@as(u64, 1) << @truncate(int_index)); // Clear the request
                    self.p4_register(u32, .INTEVT).* = Interrupts.InterruptINTEVTCodes[@intFromEnum(interrupt)];
                    self.jump_to_interrupt();
                }
            } else if (false) {
                // TODO: Check for exceptions

                // self.io_register(u32, MemoryRegister.EXPEVT).* = code;
            }
        } else {
            // When the BL bit in SR is 1 and an exception other than a user break is generated, the CPU’s
            // internal registers are set to their post-reset state, the registers of the other modules retain their
            // contents prior to the exception, and the CPU branches to the same address as in a reset (H'A000
            // 0000). For the operation in the event of a user break, see section 20, User Break Controller. If an
            // ordinary interrupt occurs, the interrupt request is held pending and is accepted after the BL bit
            // has been cleared to 0 by software. If a nonmaskable interrupt (NMI) occurs, it can be held
            // pending or accepted according to the setting made by software.

            // A setting can also be made to have the NMI interrupt accepted even if the BL bit is set to 1.
            // NMI interrupt exception handling does not affect the interrupt mask level bits (I3–I0) in the
            // status register (SR).
        }
    }

    pub fn execute(self: *@This(), max_instructions: u8) u32 {
        self.handle_interrupts();

        if (self.execution_state == .Running or self.execution_state == .ModuleStandby) {
            for (0..max_instructions) |_| {
                self._execute(self.pc);
                self.pc += 2;
            }

            const cycles = self._pending_cycles;
            self._pending_cycles = 0;
            return cycles;
        } else {
            const cycles = self._pending_cycles;
            self._pending_cycles = 0;
            return cycles;
        }
    }

    pub fn request_interrupt(self: *@This(), int: Interrupt) void {
        sh4_log.debug(" (Interrupt request! {s})", .{std.enums.tagName(Interrupt, int) orelse "Unknown"});
        self.interrupt_requests |= @as(u64, 1) << @intCast(self._interrupts_indices[@intFromEnum(int)]);
    }

    pub fn order_interrupt(ctx: *const @This(), lhs: Interrupt, rhs: Interrupt) bool {
        if (ctx._interrupt_levels[@intFromEnum(lhs)] == ctx._interrupt_levels[@intFromEnum(rhs)])
            return @intFromEnum(lhs) < @intFromEnum(rhs);
        return ctx._interrupt_levels[@intFromEnum(lhs)] > ctx._interrupt_levels[@intFromEnum(rhs)];
    }

    pub fn compute_interrupt_priorities(self: *@This()) void {
        // If any requests are pending, we have to convert them to the new indices.
        var saved_requests: u64 = 0;
        if (self.interrupt_requests != 0) {
            // Convert priority indices to the base enum
            for (0..self._sorted_interrupts.len) |i| {
                if ((self.interrupt_requests >> @intCast(i)) & 1 == 1) {
                    saved_requests |= (@as(u64, 1) << @intFromEnum(self._sorted_interrupts[i]));
                }
            }
        }

        const IPRA = self.read_p4_register(P4.IPRA, .IPRA);
        const IPRB = self.read_p4_register(P4.IPRB, .IPRB);
        const IPRC = self.read_p4_register(P4.IPRC, .IPRC);

        self._interrupt_levels[@intFromEnum(Interrupt.HitachiUDI)] = IPRC.hitachiudi;
        self._interrupt_levels[@intFromEnum(Interrupt.GPIO)] = IPRC.gpio;
        self._interrupt_levels[@intFromEnum(Interrupt.DMTE0)] = IPRC.dmac;
        self._interrupt_levels[@intFromEnum(Interrupt.DMTE1)] = IPRC.dmac;
        self._interrupt_levels[@intFromEnum(Interrupt.DMTE2)] = IPRC.dmac;
        self._interrupt_levels[@intFromEnum(Interrupt.DMTE3)] = IPRC.dmac;
        self._interrupt_levels[@intFromEnum(Interrupt.DMAE)] = IPRC.dmac;
        self._interrupt_levels[@intFromEnum(Interrupt.TUNI0)] = IPRA.tmu0;
        self._interrupt_levels[@intFromEnum(Interrupt.TUNI1)] = IPRA.tmu1;
        self._interrupt_levels[@intFromEnum(Interrupt.TUNI2)] = IPRA.tmu2;
        self._interrupt_levels[@intFromEnum(Interrupt.TICPI2)] = IPRA.tmu2;
        self._interrupt_levels[@intFromEnum(Interrupt.ATI)] = IPRA.rtc;
        self._interrupt_levels[@intFromEnum(Interrupt.PRI)] = IPRA.rtc;
        self._interrupt_levels[@intFromEnum(Interrupt.CUI)] = IPRA.rtc;
        self._interrupt_levels[@intFromEnum(Interrupt.SCI1_ERI)] = IPRB.sci1;
        self._interrupt_levels[@intFromEnum(Interrupt.SCI1_RXI)] = IPRB.sci1;
        self._interrupt_levels[@intFromEnum(Interrupt.SCI1_TXI)] = IPRB.sci1;
        self._interrupt_levels[@intFromEnum(Interrupt.SCI1_TEI)] = IPRB.sci1;
        self._interrupt_levels[@intFromEnum(Interrupt.SCIF_ERI)] = IPRC.scif;
        self._interrupt_levels[@intFromEnum(Interrupt.SCIF_RXI)] = IPRC.scif;
        self._interrupt_levels[@intFromEnum(Interrupt.SCIF_TXI)] = IPRC.scif;
        self._interrupt_levels[@intFromEnum(Interrupt.SCIF_TEI)] = IPRC.scif;
        self._interrupt_levels[@intFromEnum(Interrupt.ITI)] = IPRB.wdt;
        self._interrupt_levels[@intFromEnum(Interrupt.RCMI)] = IPRB.ref;
        self._interrupt_levels[@intFromEnum(Interrupt.ROVI)] = IPRB.ref;

        std.sort.insertion(Interrupt, &self._sorted_interrupts, self, order_interrupt);
        // Update reverse mapping (Interrupt enum to its index in the interrupt_requests bitfield)
        for (0..self._sorted_interrupts.len) |i| {
            self._interrupts_indices[@intFromEnum(self._sorted_interrupts[i])] = @intCast(i);
        }

        if (saved_requests != 0) {
            self.interrupt_requests = 0;
            // Interrupt enum to new priority indices.
            for (0..self._interrupts_indices.len) |i| {
                if ((saved_requests >> @intCast(i)) & 1 == 1) {
                    self.interrupt_requests |= (@as(u64, 1) << @intCast(self._interrupts_indices[i]));
                }
            }
        }
    }

    inline fn timer_prescaler(value: u3) u32 {
        switch (value) {
            0 => return 4,
            1 => return 16,
            2 => return 64,
            3 => return 256,
            4 => return 1024,
            // 5 - Reserved (Do not set)
            // 6 - Counts on on-chip RTC output clock
            // 7 - Counts on external clock
            else => @panic("Invalid prescaler"),
        }
    }

    fn schedule_timer(self: *@This(), channel: u2) void {
        const TSTR = self.read_p4_register(u32, .TSTR);
        if ((TSTR >> @intCast(channel)) & 0x1 == 1) {
            const pcr = self.read_p4_register(P4.FRQCR, .FRQCR).peripheral_clock_ratio();

            self.update_timer_registers(channel);
            const tcnt: usize = self.read_p4_register(u32, TimerRegisters[channel].counter);
            const tcr = self.read_p4_register(P4.TCR, TimerRegisters[channel].control);
            const scale: usize = pcr * SH4.timer_prescaler(tcr.tpsc);
            const cycles = scale * (tcnt + 1);
            if (self._dc) |dc|
                dc.schedule_event(.{ .TimerUnderflow = .{ .channel = channel } }, cycles);
            sh4_log.debug("Scheduled timer {d} underflow in {d} cycles\n", .{ channel, cycles });
        }
    }

    pub fn on_timer_underflow(self: *@This(), channel: u2) void {
        self.update_timer_registers(channel);

        const tcr = self.p4_register(P4.TCR, TimerRegisters[channel].control);
        tcr.*.unf = 1; // Signals underflow
        if (tcr.*.unie == 1)
            self.request_interrupt(TimerRegisters[channel].interrupt);

        self.schedule_timer(channel);
    }

    fn update_timer_registers(self: *@This(), channel: u2) void {
        if (self._dc) |dc| {
            const TSTR = self.read_p4_register(u32, .TSTR);
            if ((TSTR >> @intCast(channel)) & 0x1 == 1) {
                const tcr = self.read_p4_register(P4.TCR, TimerRegisters[channel].control);
                const cycles = dc._global_cycles - self._last_timer_update[channel];

                const pcr: u8 = @intCast(self.read_p4_register(P4.FRQCR, .FRQCR).peripheral_clock_ratio());
                const shift = ([_]u5{
                    @ctz(@as(u16, 4) * pcr),
                    @ctz(@as(u16, 16) * pcr),
                    @ctz(@as(u16, 64) * pcr),
                    @ctz(@as(u16, 256) * pcr),
                    @ctz(@as(u16, 1024) * pcr),
                })[tcr.tpsc];
                const diff = cycles >> shift;
                self._last_timer_update[channel] += diff << shift;

                const tcnt = self.p4_register(u32, TimerRegisters[channel].counter);
                if (tcnt.* >= diff) {
                    tcnt.* -= @intCast(diff);
                } else {
                    const reset_constant = self.p4_register(u32, TimerRegisters[channel].constant).*;
                    const mod = (@as(u32, @truncate(diff)) % reset_constant);
                    if (tcnt.* < mod) {
                        tcnt.* = reset_constant - mod;
                    } else {
                        tcnt.* -%= mod;
                    }
                }
            } else self._last_timer_update[channel] = dc._global_cycles;
        }
    }

    pub inline fn add_cycles(self: *@This(), cycles: u32) void {
        self._pending_cycles += cycles;
    }

    pub fn handle_tlb_miss(self: *@This(), exception: Exception, virtual_addr: addr_t) addr_t {
        sh4_log.warn("handle_tlb_miss({any}, {X:0>8})", .{ exception, virtual_addr });
        const initial_pc = self.pc;

        self.report_address_exception(virtual_addr);

        self.jump_to_exception(exception);
        while (self.pc != initial_pc) {
            self._execute(self.pc);
            self.pc +%= 2;
        }

        sh4_log.warn("  Returned from exception handler PC={X:0>8}", .{self.pc});

        return self.translate_address(virtual_addr) catch |err| {
            // Still didn't work: Give up.
            std.log.err("[PC:{X:0>8}] MMU missed twice: {any} @{X:0>8}.", .{ initial_pc, err, virtual_addr });
            for (self.utlb, 0..) |utlb, idx| {
                if (utlb.valid()) {
                    sh4_log.err("  [{d}] {any}", .{ idx, utlb });
                }
            }
            std.process.abort();
        };
    }

    pub fn _execute(self: *@This(), virtual_addr: addr_t) void {
        // Guiding the compiler a bit. Yes, that helps a lot :)
        // Instruction should be in Boot ROM, or RAM.
        const opcode = if (comptime !builtin.is_test) oc: {
            // NOTE: This should first search the ITLB, and only the UTLB in case of a miss, then copy the result to ITLB.
            //       However, as I understand it, this is only an additional layer of cache to speed up instruction fetching,
            //       this won't matter... Unless the software accesses ITLB entries directly.
            var physical_addr = self.translate_address(virtual_addr) catch a: {
                break :a self.handle_tlb_miss(.InstructionTLBMiss, virtual_addr);
            };
            physical_addr &= 0x1FFFFFFF;
            if (!((physical_addr >= 0x00000000 and physical_addr < 0x00020000) or (physical_addr >= 0x0C000000 and physical_addr < 0x10000000))) {
                std.debug.print(" ! PC virtual_addr {X:0>8} => physical_addr: {X:0>8}\n", .{ virtual_addr, physical_addr });
            }
            std.debug.assert((physical_addr >= 0x00000000 and physical_addr < 0x00020000) or (physical_addr >= 0x0C000000 and physical_addr < 0x10000000));
            break :oc @call(.always_inline, @This().read_physical, .{ self, u16, physical_addr });
        } else self.read(u16, virtual_addr);

        if (interpreter_handlers.Enable) {
            interpreter_handlers.InstructionHandlers[opcode](self);
        } else {
            const instr = Instr{ .value = opcode };
            const desc = sh4_instructions.Opcodes[sh4_instructions.JumpTable[opcode]];

            if ((comptime builtin.mode == .Debug or builtin.mode == .ReleaseSafe) and self.debug_trace)
                std.debug.print("[{X:0>8}] {b:0>16} {s: <20} R{d: <2}={X:0>8}, R{d: <2}={X:0>8}, T={b:0>1}, Q={b:0>1}, M={b:0>1}\n", .{ virtual_addr, opcode, sh4_disassembly.disassemble(instr, self._allocator) catch {
                    std.debug.print("Failed to disassemble instruction {b:0>16}\n", .{opcode});
                    unreachable;
                }, instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).*, if (self.sr.t) @as(u1, 1) else 0, if (self.sr.q) @as(u1, 1) else 0, if (self.sr.m) @as(u1, 1) else 0 });

            desc.fn_(self, instr);

            if ((comptime builtin.mode == .Debug or builtin.mode == .ReleaseSafe) and self.debug_trace)
                std.debug.print("[{X:0>8}] {X: >16} {s: <20} R{d: <2}={X:0>8}, R{d: <2}={X:0>8}, T={b:0>1}, Q={b:0>1}, M={b:0>1}\n", .{ virtual_addr, opcode, "", instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).*, if (self.sr.t) @as(u1, 1) else 0, if (self.sr.q) @as(u1, 1) else 0, if (self.sr.m) @as(u1, 1) else 0 });

            self.add_cycles(desc.issue_cycles);
        }
    }

    pub fn store_queue_write(self: *@This(), comptime T: type, virtual_addr: addr_t, value: T) void {
        const sq_addr: StoreQueueAddr = @bitCast(virtual_addr);
        // sh4_log.debug("  StoreQueue write @{X:0>8} = 0x{X:0>8} ({any})", .{ virtual_addr, value, sq_addr });
        std.debug.assert(sq_addr.spec == 0b111000);
        switch (T) {
            u32 => self.store_queues[sq_addr.sq][sq_addr.lw_spec] = value,
            u64 => {
                self.store_queues[sq_addr.sq][sq_addr.lw_spec] = @truncate(value);
                self.store_queues[sq_addr.sq][sq_addr.lw_spec + 1] = @truncate(value >> 32);
            },
            else => unreachable,
        }
    }

    pub fn start_dmac(self: *@This(), comptime channel: u8) void {
        const c = DMACChannels[channel];
        const chcr = self.read_p4_register(P4.CHCR, c.chcr);

        // NOTE: I think the DC only uses 32 bytes transfers, but I'm not 100% sure.
        std.debug.assert(chcr.ts == 0b100);
        std.debug.assert(chcr.rs == 2); // "External request, single address mode"

        const src_addr = self.read_p4_register(u32, c.sar) & 0x1FFFFFFF;
        const dst_addr = self.read_p4_register(u32, c.dar) & 0x1FFFFFFF;
        const transfer_size = chcr.transfer_size();
        const len = self.read_p4_register(u32, c.dmatcr);
        const byte_len = transfer_size * len;

        sh4_log.info("DMAC ({d}) CHCR: {any}\n  src_addr: 0x{X:0>8} => dst_addr: 0x{X:0>8} (transfer_size: 0x{X:0>8}, len: 0x{X:0>8}, byte_len: 0x{X:0>8})", .{ channel, chcr, src_addr, dst_addr, transfer_size, len, byte_len });

        const dst_stride: i32 = switch (chcr.dm) {
            0 => 0,
            1 => 1,
            2 => -1,
            else => @panic("Invalid destination stride"),
        };
        const src_stride: i32 = switch (chcr.sm) {
            0 => 0,
            1 => 1,
            2 => -1,
            else => @panic("Invalid source stride"),
        };

        const is_memcpy_safe = dst_stride == 1 and src_stride == 1;

        // Write to Tile Accelerator, we can bypass write32
        // 0x10000000 ~ 0x107FFFE0 : TA FIFO - Polygon Path (8MB)         (0x12000000 ~ 0x127FFFE0 Mirror)
        // 0x10800000 ~ 0x10FFFFE0 : TA FIFO - YUV Converter Path (8MB)   (0x12800000 ~ 0x12FFFFE0 Mirror)
        // 0x11000000 ~ 0x11FFFFE0 : TA FIFO - Direct Texture Path (16MB) (0x13000000 ~ 0x13FFFFE0 Mirror)
        if (dst_addr >= 0x10000000 and dst_addr < 0x14000000) {
            std.debug.assert(src_stride > 0);
            std.debug.assert(transfer_size == 32); // We'll copy 32-bytes blocks
            std.debug.assert(chcr.ts == 0b100); // We'll copy 32-bytes blocks
            switch (dst_addr) {
                // Polygon Path
                0x10000000...0x10800000 - 1, 0x12000000...0x12800000 - 1 => {
                    var src: [*]u32 = @alignCast(@ptrCast(self._get_memory(src_addr)));
                    self._dc.?.gpu.write_ta_fifo_polygon_path(src[0 .. 8 * len]);
                },
                // YUV Converter Path
                0x10800000...0x11000000 - 1, 0x12800000...0x13000000 - 1 => {
                    var src: [*]u8 = @alignCast(@ptrCast(self._get_memory(src_addr)));
                    self._dc.?.gpu.ta_fifo_yuv_converter_path(src[0..byte_len]);
                },
                // Direct Texture Path
                0x11000000...0x12000000 - 1, 0x13000000...0x14000000 - 1 => {
                    const LMMode = self._dc.?.read_hw_register(u32, if (dst_addr >= 0x11000000 and dst_addr < 0x12000000) .SB_LMMODE0 else .SB_LMMODE1);
                    const access_32bit = LMMode != 0;
                    if (access_32bit) {
                        var curr_dst: i32 = @intCast(dst_addr);
                        var curr_src: i32 = @intCast(src_addr);
                        for (0..byte_len / 4) |_| {
                            self._dc.?.gpu.write_ta(@intCast(curr_dst), &[1]u32{self.read(u32, @intCast(curr_src))}, if (access_32bit) .b32 else .b64);
                            curr_dst += 4 * dst_stride;
                            curr_src += 4 * src_stride;
                        }
                    } else {
                        self._dc.?.gpu.write_ta_fifo_direct_texture_path(dst_addr, @as([*]u8, @ptrCast(self._get_memory(src_addr)))[0..byte_len]);
                    }
                },
                else => unreachable,
            }
        } else if (is_memcpy_safe) {
            std.debug.assert(dst_stride == src_stride);
            // FIXME: When can we safely memcpy?
            const src = @as([*]u8, @ptrCast(self._get_memory(src_addr)));
            const dst = @as([*]u8, @ptrCast(self._get_memory(dst_addr)));
            @memcpy(dst[0..byte_len], src[0..byte_len]);
        } else {
            var curr_dst: i32 = @intCast(dst_addr);
            var curr_src: i32 = @intCast(src_addr);
            for (0..byte_len / 4) |_| {
                self.write(u32, @intCast(curr_dst), self.read(u32, @intCast(curr_src)));
                curr_dst += 4 * dst_stride;
                curr_src += 4 * src_stride;
            }
        }

        // TODO: Schedule for later?
        end_dmac(self, channel);
    }

    pub fn end_dmac(self: *@This(), channel: u32) void {
        const c = DMACChannels[channel];
        const chcr = self.read_p4_register(P4.CHCR, c.chcr);

        const len = chcr.transfer_size() * self.read_p4_register(u32, c.dmatcr);
        if (chcr.ie == 1)
            self.request_interrupt(c.dmte);
        if (chcr.sm != 0)
            self.p4_register(u32, c.sar).* += len;
        if (chcr.dm != 0)
            self.p4_register(u32, c.dar).* += len;
        self.p4_register(u32, c.dmatcr).* = 0;
        self.p4_register(P4.CHCR, c.chcr).*.te = 1;
    }

    fn panic_debug(self: *const @This(), comptime fmt: []const u8, args: anytype) noreturn {
        std.debug.print("panic_debug: PC: {X:0>8}\n", .{self.pc});
        std.debug.print(fmt ++ "\n", args);
        @panic(fmt);
    }

    // Memory access/mapping functions
    // NOTE: These would make a lot more sense in the Dreamcast struct, however since this is
    //       by far the hostest part, it looks like avoiding the extra indirection helps a lot
    //       with performance. I don't really understand it since it will end up doing it anyway
    //       most of the time.
    //       I tried having some small functions just for the P4 registers that will delegate to
    //       the proper functions, but it was also worse performance wise (although a little less
    //       that calling directly to DC).

    pub inline fn _get_memory(self: *@This(), addr: addr_t) *u8 {
        std.debug.assert(addr <= 0x1FFFFFFF);
        const dc = self._dc.?;

        // NOTE: These cases are out of order as an optimization.
        //       Empirically tested on interpreter_perf (i.e. 200_000_000 first 'ticks' of Sonic Adventure and the Boot ROM).
        //       The compiler seems to like really having equal length ranges (and also easily maskable, I guess)!
        switch (addr) {
            // Area 3 - System RAM (16MB) - 0x0C000000 to 0x0FFFFFFF, mirrored 4 times.
            0x0C000000...0x0FFFFFFF => return &dc.ram[addr & 0x00FFFFFF],
            // Area 1 - 64bit path
            0x04000000...0x04FFFFFF, 0x06000000...0x06FFFFFF => return &dc.gpu.vram[addr & (Dreamcast.VRAMSize - 1)],
            0x00000000...0x03FFFFFF => { // Area 0 - Boot ROM, Flash ROM, Hardware Registers
                const area_0_addr = addr & 0x01FFFFFF;
                switch (area_0_addr) {
                    0x00000000...0x001FFFFF => return &dc.boot[area_0_addr],
                    0x00200000...0x0021FFFF => return &dc.flash.data[area_0_addr & 0x1FFFF],
                    0x005F6800...0x005F6FFF => return dc.hw_register_addr(u8, area_0_addr),
                    0x005F7000...0x005F709C => @panic("_get_memory to GDROM Register. This should be handled in read/write functions."),
                    0x005F709D...0x005F7FFF => return dc.hw_register_addr(u8, area_0_addr),
                    0x005F8000...0x005F9FFF => return dc.gpu._get_register_from_addr(u8, area_0_addr),
                    0x005FA000...0x005FFFFF => return dc.hw_register_addr(u8, area_0_addr),
                    0x00600000...0x006007FF => {
                        const static = struct {
                            var once = false;
                        };
                        if (!static.once) {
                            static.once = true;
                            sh4_log.warn(termcolor.yellow("  Unimplemented _get_memory to MODEM: {X:0>8} (This will only be reported once)"), .{addr});
                        }
                        dc._dummy = .{ 0, 0, 0, 0 };
                        return @ptrCast(&dc._dummy);
                    },
                    // G2 AICA Register
                    0x00700000...0x00707FFF => @panic("_get_memory to AICA Register. This should be handled in read/write functions."),
                    // G2 AICA RTC Registers
                    0x00710000...0x00710008 => @panic("_get_memory to AICA RTC Register. This should be handled in read/write functions."),
                    0x00800000...0x009FFFFF, 0x02800000...0x029FFFFF => { // G2 Wave Memory and Mirror
                        sh4_log.debug("NOTE: _get_memory to AICA Wave Memory @{X:0>8} ({X:0>8}). This should be handled in read/write functions, except for DMA. Get rid of this warning when the ARM core is stable enough! (Direct access to wave memory specifically should be fine.)", .{ addr, area_0_addr });
                        return @ptrCast(&dc.aica.wave_memory[area_0_addr & (dc.aica.wave_memory.len - 1)]);
                    },
                    0x01000000...0x01FFFFFF => { // Expansion Devices
                        sh4_log.warn(termcolor.yellow("  Unimplemented _get_memory to Expansion Devices: {X:0>8} ({X:0>8})"), .{ addr, area_0_addr });

                        // FIXME: TEMP DEBUG: Crazy Taxi accesses 030100C0 (010100C0) and 030100A0 (010100A0)
                        //        And 0101003C to 0101007C, and 01010014, and 01010008
                        // self.on_trapa.?();

                        // FIXME: I have no idea why Crazy Taxi seem to expect to find 0x80 at 01010008, but this lets it go further.
                        dc._dummy = .{ 0x80, 0, 0, 0 };

                        return @ptrCast(&dc._dummy);
                    },
                    else => {
                        sh4_log.warn(termcolor.yellow("  Unimplemented _get_memory to Area 0: {X:0>8} ({X:0>8})"), .{ addr, area_0_addr });

                        dc._dummy = .{ 0, 0, 0, 0 };
                        return @ptrCast(&dc._dummy);
                    },
                }
            },
            // Area 2 - Nothing
            0x08000000...0x0BFFFFFF => self.panic_debug("Invalid _get_memory to Area 2 @{X:0>8}", .{addr}),
            0x10000000...0x13FFFFFF => { // Area 4 - Tile accelerator command input
                // self.panic_debug("Unexpected _get_memory to Area 4 @{X:0>8} - This should only be accessible via write32 or DMA.", .{addr});
                // NOTE: Marvel vs. Capcom 2 reads from here (Addr:103464A0 PC:8C031D3C). Ignoring it doesn't seem to hurt, so... Doing that instead of panicking for now.
                sh4_log.err(termcolor.red("[PC: 0x{X:0>8}] Unexpected _get_memory to Area 4 @{X:0>8} - This should only be accessible via write32 or DMA."), .{ self.pc, addr });
                dc._dummy = .{ 0, 0, 0, 0 };
                return @ptrCast(&dc._dummy);
            },
            0x14000000...0x17FFFFFF => { // Area 5 - G2 Expansion Devices
                sh4_log.warn(termcolor.yellow("Unimplemented _get_memory to Area 5 (G2 Expansion Devices): {X:0>8}"), .{addr});
                dc._dummy = .{ 0, 0, 0, 0 };
                return @ptrCast(&dc._dummy);
            },
            // Area 6 - Nothing
            0x18000000...0x1BFFFFFF => self.panic_debug("Invalid _get_memory to Area 6 @{X:0>8}", .{addr}),
            // Area 7 - Internal I/O registers (same as P4)
            0x1F000000...0x1FFFFFFF => {
                std.debug.assert(self.sr.md == 1);
                return self.p4_register_addr(u8, addr);
            },
            else => {
                // FIXME: This space should be Unassigned/Reserved.
                //        Returns a dummy value instead of panicking.
                //        Metropolis Street Racer and Legacy of the Kain - Soul Reaver write to 0xBCXXXXXX,
                //        and I have no idea if this is an issue with the emulator... See #51.
                //        Ignoring the writes allow these games to progress a bit, but this might become an issue.
                sh4_log.err(termcolor.red("Invalid _get_memory @{X:0>8}"), .{addr});
                return @ptrCast(&dc._dummy);
            },
        }
    }

    inline fn check_type(comptime Valid: []const type, comptime T: type, comptime fmt: []const u8, param: anytype) void {
        inline for (Valid) |V| {
            if (T == V) return;
        }
        std.debug.print(fmt, param);
        unreachable;
    }

    pub fn read_p4(self: *const @This(), comptime T: type, virtual_addr: addr_t) T {
        std.debug.assert(virtual_addr & 0xE0000000 == 0xE0000000);

        switch (virtual_addr) {
            0xE0000000...0xE3FFFFFF => {
                // Store Queue
                @panic("Store Queue Read?");
            },
            0xE4000000...0xEFFFFFFF => {
                // Reserved
                sh4_log.err(termcolor.red("Read({any}) from Reserved space in P4 (PC: {X:0>8}): {X:0>8}"), .{ T, self.pc, virtual_addr });
            },
            0xF0000000...0xF0FFFFFF => {
                // Instruction cache address array
            },
            0xF1000000...0xF1FFFFFF => {
                // Instruction cache data array
            },
            0xF2000000...0xF2FFFFFF => {
                // Instruction TLB address array (ITLB)
                sh4_log.warn(termcolor.yellow("Read({any}) from Instruction TLB address array (ITLB): {X:0>8}"), .{ T, virtual_addr });
            },
            0xF3000000...0xF3FFFFFF => {
                // Instruction TLB data arrays 1 and 2 (ITLB)
                sh4_log.warn(termcolor.yellow("Read({any}) from Instruction TLB data array (ITLB): {X:0>8}"), .{ T, virtual_addr });
            },
            0xF4000000...0xF4FFFFFF => {
                // Operand cache address array
            },
            0xF5000000...0xF5FFFFFF => {
                // Operand cache data array
            },
            0xF6000000...0xF6FFFFFF => {
                // Unified TLB address array
                if (T == u32) {
                    const entry_index: u6 = @truncate(virtual_addr >> 8);
                    const entry = self.utlb[entry_index];
                    const val: mmu.UTLBAddressData = .{
                        .asid = entry.asid,
                        .d = entry.d,
                        .v = entry.v,
                        .vpn = entry.vpn,
                    };
                    return @bitCast(val);
                }
            },
            0xF7000000...0xF7FFFFFF => {
                // Unified TLB data arrays 1 and 2
                if (T == u32) {
                    const entry_index: u6 = @truncate(virtual_addr >> 8);
                    const entry = self.utlb[entry_index];
                    const val: mmu.UTLBArrayData1 = .{
                        .wt = entry.wt,
                        .sh = entry.sh,
                        .d = entry.d,
                        .c = entry.c,
                        .sz0 = @truncate(entry.sz),
                        .pr = entry.pr,
                        .sz1 = @truncate(entry.sz >> 1),
                        .v = entry.v,
                        .ppn = entry.ppn,
                    };
                    return @bitCast(val);
                }
            },
            0xF8000000...0xFBFFFFFF => {
                // Reserved
                @panic("Reserved");
            },
            0xFC000000...0xFFFFFFFF => {
                // Control register area
                if (virtual_addr >= 0xFF000000) {
                    switch (@as(P4Register, @enumFromInt(virtual_addr))) {
                        P4Register.RFCR => {
                            check_type(&[_]type{u16}, T, "Invalid P4 Write({any}) to RFCR\n", .{T});
                            // Hack: This is the Refresh Count Register, related to DRAM control.
                            //       If don't think its proper emulation is needed, but it's accessed by the bios,
                            //       probably for synchronization purposes. I assume returning a contant value to pass this check
                            //       is enough for now, as games shouldn't access that themselves.
                            sh4_log.debug("[Note] Access to Refresh Count Register.", .{});
                            return 0x0011;
                            // Otherwise, this is 10-bits register, respond with the 6 unused upper bits set to 0.
                        },
                        P4Register.PDTRA => {
                            check_type(&[_]type{u16}, T, "Invalid P4 Read({any}) to PDTRA\n", .{T});
                            // Port data register A (PDTRA) is a 16-bit readable/writable register used as a data latch for each
                            // bit in the 16-bit port. When a bit is set as an output, the value written to the PDTRA register is
                            // output from the external pin. When a value is read from the PDTRA register while a bit is set as
                            // an input, the external pin value sampled on the external bus clock is read. When a bit is set as an
                            // output, the value written to the PDTRA register is read.

                            var out: u16 = @as(u16, @intFromEnum(self._dc.?.cable_type)) << 8;

                            const ctrl: u32 = self.read_p4_register(u32, .PCTRA) & 0xF;
                            const data: u16 = self.read_p4_register(u16, .PDTRA) & 0xF;

                            // NOTE: The following hack accounts for the fact that pins A0 and A1 (GPIO Port A 0 and 1) are shorted together on the DC.
                            //       It only directly responds to the specific BIOS checks. A proper fix would be to fully emulate the GPIO pins.
                            //
                            //       MetalliC: "SH7091 reuses A0 and A1 for GPIO port A bits 0 and 1"
                            //                 "DC have 2 of SH4's GPIO pins connected together, and boot ROM code does quite nasty checks if they really are shorted.
                            //                  One of checks include: configure both pins for input, enable pullup for one of them
                            //                  (which will effectively pull up both of them), then check if other pin will become 1"
                            //
                            //       This is adapted from Flycast, which already got it from Chankast.
                            //       This is needed for the bios to work properly, without it, it will
                            //       go to sleep mode with all interrupts disabled early on.

                            if (ctrl == 0x8 or (ctrl == 0xB and data != 0x2) or (ctrl == 0xC and data == 0x2)) {
                                out |= 3;
                            }

                            return out;
                        },
                        // FIXME: Not emulated at all, these clash with my P4 access pattern :(
                        P4Register.PMCR1 => return 0,
                        P4Register.PMCR2 => return 0,
                        P4Register.TCNT0, P4Register.TCNT1, P4Register.TCNT2 => {
                            @constCast(self).update_timer_registers(switch (@as(P4Register, @enumFromInt(virtual_addr))) {
                                P4Register.TCNT0 => 0,
                                P4Register.TCNT1 => 1,
                                P4Register.TCNT2 => 2,
                                else => unreachable,
                            });
                            return @constCast(self).p4_register_addr(T, virtual_addr).*;
                        },
                        else => {
                            if (!(virtual_addr & 0xFF000000 == 0xFF000000 or virtual_addr & 0xFF000000 == 0x1F000000) or !(virtual_addr & 0b0000_0000_0000_0111_1111_1111_1000_0000 == 0)) {
                                sh4_log.warn(termcolor.yellow(" [{X:0>8}] Invalid Read({any}) to P4 register @{X:0>8}"), .{ self.pc, T, virtual_addr });
                                return 0;
                            }
                            sh4_log.debug("  Read({any}) to P4 register @{X:0>8} {s} = 0x{X}", .{ T, virtual_addr, P4.getP4RegisterName(virtual_addr), @constCast(self).p4_register_addr(T, virtual_addr).* });
                            return @constCast(self).p4_register_addr(T, virtual_addr).*;
                        },
                    }
                } else @panic("Unhandled Control register area read.");
            },
            else => @panic("Unhandled P4 read."),
        }

        return 0;
    }

    pub fn write_p4(self: *@This(), comptime T: type, virtual_addr: addr_t, value: T) void {
        std.debug.assert(virtual_addr & 0xE0000000 == 0xE0000000);

        switch (virtual_addr) {
            0xE0000000...0xE3FFFFFF => {
                // Store Queue
                self.store_queue_write(T, virtual_addr, value);
                return;
            },
            0xE4000000...0xEFFFFFFF => {
                // Reserved
                @panic("Write to Reserved space in P4 (E4000000...0xEFFFFFFF)");
            },
            0xF0000000...0xF0FFFFFF => {
                // Instruction cache address array
            },
            0xF1000000...0xF1FFFFFF => {
                // Instruction cache data array
            },
            0xF2000000...0xF2FFFFFF => {
                // Instruction TLB address array (ITLB)
                sh4_log.warn(termcolor.yellow("Write({any}) to Instruction TLB address array (ITLB): {X:0>8} = {X:0>8}"), .{ T, virtual_addr, value });
            },
            0xF3000000...0xF3FFFFFF => {
                // Instruction TLB data arrays 1 and 2 (ITLB)
                sh4_log.warn(termcolor.yellow("Write({any}) to Instruction TLB data array (ITLB): {X:0>8} = {X:0>8}"), .{ T, virtual_addr, value });
            },
            0xF4000000...0xF4FFFFFF => {
                // Operand cache address array
            },
            0xF5000000...0xF5FFFFFF => {
                // Operand cache data array
            },
            0xF6000000...0xF6FFFFFF => {
                // Unified TLB address array (UTLB)
                sh4_log.warn(termcolor.yellow("Write({any}) to Unified TLB address array: {X:0>8} ({X:0>8})"), .{ T, virtual_addr, value });
                if (T == u32) {
                    const entry: u6 = @truncate(virtual_addr >> 8);
                    const association_bit: u1 = @truncate(virtual_addr >> 7);
                    const val: mmu.UTLBAddressData = @bitCast(value);
                    sh4_log.warn(termcolor.yellow("  Entry {X:0>3} (A:{X:0>1}): {any} (VPN: {X:0>6})"), .{ entry, association_bit, val, val.vpn });
                    self.utlb[entry].asid = val.asid;
                    self.utlb[entry].v = val.v;
                    self.utlb[entry].d = val.d;
                    self.utlb[entry].vpn = val.vpn;
                }
            },
            0xF7000000...0xF7FFFFFF => {
                // Unified TLB data arrays 1 and 2 (UTLB)
                sh4_log.warn(termcolor.yellow("Write({any}) to Unified TLB data arrays:   {X:0>8} ({X:0>8})"), .{ T, virtual_addr, value });
                if (T == u32) {
                    const entry: u6 = @truncate(virtual_addr >> 8);
                    const val: mmu.UTLBArrayData1 = @bitCast(value);
                    sh4_log.warn(termcolor.yellow("  Entry {X:0>3}: {any} (PPN: {X:0>5})"), .{ entry, val, val.ppn });
                    self.utlb[entry].wt = val.wt;
                    self.utlb[entry].sh = val.sh;
                    self.utlb[entry].d = val.d;
                    self.utlb[entry].c = val.c;
                    self.utlb[entry].sz = val.sz();
                    self.utlb[entry].pr = val.pr;
                    self.utlb[entry].v = val.v;
                    self.utlb[entry].ppn = val.ppn;
                }
            },
            0xF8000000...0xFBFFFFFF => {
                // Reserved
                @panic("Reserved");
            },
            0xFC000000...0xFFFFFFFF => {
                // Control register area
                if (virtual_addr >= 0xFF000000) {
                    switch (virtual_addr) {
                        @intFromEnum(P4Register.MMUCR) => {
                            if (T == u32) {
                                var val: mmu.MMUCR = @bitCast(value);
                                sh4_log.warn("Write({any}) to MMUCR: {X:0>8}: {any}", .{ T, value, val });
                                if (val.ti) {
                                    // Invalidate all TLB entries
                                    for (self.utlb) |*entry|
                                        entry.v = false;
                                    val.ti = false; // Always return 0 when read.
                                    if (self._dc) |dc| dc.sh4_jit.request_reset();
                                }
                                self._mmu_enabled = val.at;
                                self.p4_register_addr(mmu.MMUCR, virtual_addr).* = val;
                                self.debug_trace = true;
                                return;
                            } else {
                                sh4_log.warn("Write({any}) to MMUCR: {X}", .{ T, value });
                            }
                        },
                        // SDMR2/SDMR3
                        0xFF900000...0xFF90FFFF, 0xFF940000...0xFF94FFFF => {
                            // Ignore it, it's not implemented but it also doesn't fit in our P4 register remapping.
                            return;
                        },
                        @intFromEnum(P4Register.SCFTDR2) => {
                            check_type(&[_]type{u8}, T, "Invalid P4 Write({any}) to SCFTDR2\n", .{T});

                            std.fmt.format(std.io.getStdOut().writer(), "\u{001b}[44m\u{001b}[97m{c}\u{001b}[0m", .{value}) catch unreachable;

                            // Immediately mark transfer as complete.
                            //   Or rather, attempts to, this is not enough.
                            // const SCFSR2 = self.p4_register(HardwareRegisters.SCFSR2, .SCFSR2);
                            // SCFSR2.*.tend = 1;
                            // FIXME: The serial interface is not implemented at all.
                            return;
                        },
                        @intFromEnum(P4Register.RTCSR), @intFromEnum(P4Register.RTCNT), @intFromEnum(P4Register.RTCOR) => {
                            check_type(&[_]type{u16}, T, "Invalid P4 Write({any}) to RTCSR\n", .{T});
                            std.debug.assert(value & 0xFF00 == 0b10100101_00000000);
                            self.p4_register_addr(u16, virtual_addr).* = (value & 0xFF);
                            return;
                        },
                        @intFromEnum(P4Register.RFCR) => {
                            check_type(&[_]type{u16}, T, "Invalid P4 Write({any}) to RFCR\n", .{T});
                            std.debug.assert(value & 0b11111100_00000000 == 0b10100100_00000000);
                            self.p4_register_addr(u16, virtual_addr).* = (value & 0b11_11111111);
                            return;
                        },
                        // FIXME: Not emulated at all, these clash with my P4 access pattern :(
                        @intFromEnum(P4Register.PMCR1) => {
                            sh4_log.warn("Write to non implemented P4 register PMCR1: {X:0>4}.", .{value});
                            return;
                        },
                        @intFromEnum(P4Register.PMCR2) => {
                            sh4_log.warn("Write to non implemented P4 register PMCR2: {X:0>4}.", .{value});
                            return;
                        },
                        // Serial Interface
                        @intFromEnum(P4Register.SCFSR2) => {
                            check_type(&[_]type{u16}, T, "Invalid P4 Write({any}) to SCFSR2\n", .{T});
                            // Writable bits can only be cleared.
                            // self.p4_register(u16, .SCFSR2).* &= (value | 0b11111111_00001100);
                            return;
                        },
                        @intFromEnum(P4Register.CCR) => {
                            check_type(&[_]type{u32}, T, "Invalid P4 Write({any}) to CCR\n", .{T});
                            var ccr: P4.CCR = @bitCast(value);
                            sh4_log.debug("Write to CCR: {any}", .{ccr});
                            // ICI: IC invalidation bit - When 1 is written to this bit, the V bits of all IC entries are cleared to 0. This bit always returns 0 when read.
                            // OCI: OC invalidation bit - When 1 is written to this bit, the V and U bits of all OC entries are cleared to 0. This bit always returns 0 when read.
                            if (ccr.ici == 1 or ccr.oci == 1) {
                                // Instruction cache invalidation - We'll use it as a clue to flush our JIT cache.
                                sh4_log.info("Instruction cache invalidation - Purging JIT cache.", .{});
                                if (self._dc) |dc| dc.sh4_jit.request_reset();
                            }
                            if (ccr.oci == 1)
                                @memset(&self._operand_cache_state.dirty, false);
                            ccr.ici = 0;
                            ccr.oci = 0;
                            self.p4_register_addr(T, virtual_addr).* = @bitCast(ccr);
                            return;
                        },
                        @intFromEnum(P4Register.CHCR0), @intFromEnum(P4Register.CHCR1), @intFromEnum(P4Register.CHCR2) => {
                            check_type(&[_]type{u32}, T, "Invalid P4 Write({any}) to 0x{X:0>8}\n", .{ T, virtual_addr });
                            const chcr: P4.CHCR = @bitCast(value);
                            if (chcr.de == 1 and chcr.rs & 0b1100 == 0b0100) {
                                sh4_log.warn(" CHCR {X:0>8} write with DMAC enable and auto request on! Value {X:0>8}", .{ virtual_addr, value });
                                @panic("Unimplemented");
                            }
                        },
                        @intFromEnum(P4Register.WTCNT), @intFromEnum(P4Register.WTCSR) => {
                            sh4_log.warn(termcolor.yellow("Write to non implemented-P4 register {s}: {X:0>4}."), .{ @tagName(@as(P4Register, @enumFromInt(virtual_addr))), value });
                        },
                        @intFromEnum(P4Register.IPRA), @intFromEnum(P4Register.IPRB), @intFromEnum(P4Register.IPRC) => {
                            self.p4_register_addr(T, virtual_addr).* = value;
                            self.compute_interrupt_priorities();
                            return;
                        },
                        // Any timer register
                        0xFFD80000...0xFFD80030 => {
                            // TODO: We can be a lot more precise here.
                            inline for (0..3) |i| {
                                self.update_timer_registers(i);
                            }
                            self.p4_register_addr(T, virtual_addr).* = value;
                            inline for (0..3) |i| {
                                self._dc.?.clear_event(.{ .TimerUnderflow = .{ .channel = i } });
                                self.update_timer_registers(i);
                                self.schedule_timer(i);
                            }
                            return;
                        },
                        else => {
                            sh4_log.debug("  Write({any}) to P4 register @{X:0>8} {s} = 0x{X}", .{ T, virtual_addr, P4.getP4RegisterName(virtual_addr), value });
                        },
                    }

                    self.p4_register_addr(T, virtual_addr).* = value;
                } else @panic("Unhandled Control register area write.");
            },
            else => @panic("Unhandled P4 write."),
        }
    }

    pub fn mmu_translate_address(self: *@This(), virtual_addr: addr_t) !addr_t {
        const mmucr = self.p4_register(mmu.MMUCR, .MMUCR);
        std.debug.assert(mmucr.at);

        // P4 access
        if (virtual_addr >= 0xE0000000) return virtual_addr;
        // Operand Cache RAM Mode
        if (virtual_addr >= 0x7C00_0000 and virtual_addr <= 0x7FFF_FFFF) return virtual_addr;
        // Return if not in P0 (excluding 0x7C00_0000-0x7FFF_FFFF) or P3 (FIXME: Cleanup this)
        if (!(virtual_addr < 0x7C00_0000 or virtual_addr & 0xE000_0000 == 0xC000_0000)) return virtual_addr & 0x1FFF_FFFF;

        // URC: Random counter for indicating the UTLB entry for which replacement is to be
        // performed with an LDTLB instruction. URC is incremented each time the UTLB is accessed.
        // When URB > 0, URC is reset to 0 when the condition URC = URB occurs
        mmucr.urc +%= 1;
        if (mmucr.urb > 0 and mmucr.urc == mmucr.urb) mmucr.urc = 0;

        const check_asid = mmucr.sv or self.sr.md == 0;

        const asid = self.read_p4_register(mmu.PTEH, .PTEH).asid;
        const vpn: u22 = @truncate(virtual_addr >> 10);
        for (self.utlb) |entry| {
            // NOTE: Here we assume only one entry will match, TLB multiple hit exception isn't emulated.
            if (entry.match(check_asid, asid, vpn)) {
                const physical_address = entry.translate(virtual_addr);
                sh4_log.warn("UTLB Hit: {x:0>8} -> {x:0>8}", .{ virtual_addr, physical_address });
                sh4_log.warn("  Entry: {any}", .{entry});
                return physical_address;
            }
        }
        sh4_log.err("UTLB Miss: {x:0>8}", .{virtual_addr});
        return error.TLBMiss;
    }

    pub inline fn translate_address(self: *@This(), virtual_addr: addr_t) !addr_t {
        if (ExperimentalFullMMUSupport and self._mmu_enabled) return self.mmu_translate_address(virtual_addr);
        return virtual_addr;
    }

    pub fn read(self: *@This(), comptime T: type, virtual_addr: addr_t) T {
        const physical_address = self.translate_address(virtual_addr) catch a: {
            break :a self.handle_tlb_miss(.DataTLBMissRead, virtual_addr);
        };
        return self.read_physical(T, physical_address);
    }

    pub fn read_physical(self: *const @This(), comptime T: type, physical_addr: addr_t) T {
        if ((comptime builtin.is_test) and self._dc == null) {
            switch (T) {
                u8 => return DebugHooks.read8.?(physical_addr),
                u16 => return DebugHooks.read16.?(physical_addr),
                u32 => return DebugHooks.read32.?(physical_addr),
                u64 => return DebugHooks.read64.?(physical_addr),
                else => @compileError("Invalid read type"),
            }
        }

        if (physical_addr >= 0x7C000000 and physical_addr <= 0x7FFFFFFF)
            return self.read_operand_cache(T, physical_addr);

        if (physical_addr >= 0xE0000000) return self.read_p4(T, physical_addr);

        const addr = physical_addr & 0x1FFFFFFF;

        switch (addr) {
            // Area 0
            0x00000000...0x01FFFFFF, 0x02000000...0x02FFFFFF => {
                switch (addr) {
                    0x005F6800...0x005F7FFF => {
                        switch (addr) {
                            0x005F7000...0x005F709C => {
                                check_type(&[_]type{ u8, u16 }, T, "Invalid Read({any}) to GDRom Register 0x{X:0>8}\n", .{ T, addr });
                                return self._dc.?.gdrom.read_register(T, addr);
                            },
                            else => {
                                // Too spammy even for debugging.
                                if (addr != @intFromEnum(HardwareRegister.SB_ISTNRM) and addr != @intFromEnum(HardwareRegister.SB_FFST))
                                    sh4_log.debug("  Read({any}) to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{
                                        T, addr, HardwareRegisters.getRegisterName(addr), @as(*const u32, @alignCast(@ptrCast(@constCast(self)._get_memory(addr)))).*,
                                    });
                                return self._dc.?.hw_register_addr(T, addr).*;
                            },
                        }
                    },
                    0x005F8000...0x005F9FFF => {
                        check_type(&[_]type{u32}, T, "Invalid Read({any}) to 0x{X:0>8} (Holly Registers)\n", .{ T, addr });
                        return self._dc.?.gpu.read_register(T, @enumFromInt(addr));
                    },
                    // NOTE: 0x00700000...0x00FFFFFF mirrors to 0x02700000...0x02FFFFFF
                    0x00700000...0x00707FE0, 0x02700000...0x02707FE0 => {
                        check_type(&[_]type{ u8, u32 }, T, "Invalid Read({any}) to 0x{X:0>8}\n", .{ T, addr });
                        return self._dc.?.aica.read_register(T, addr & 0x00FFFFFF);
                    },
                    0x00710000...0x00710008, 0x02710000...0x02710008 => {
                        check_type(&[_]type{u32}, T, "Invalid Read({any}) to 0x{X:0>8}\n", .{ T, addr });
                        return @truncate(self._dc.?.aica.read_rtc_register(addr & 0x00FFFFFF));
                    },
                    0x00800000...0x00FFFFFF, 0x02800000...0x02FFFFFF => {
                        return self._dc.?.aica.read_mem(T, addr & 0x00FFFFFF);
                    },
                    else => {},
                }
            },
            // Area 1 - 64bit access
            0x04000000...0x04FFFFFF, 0x06000000...0x06FFFFFF => {
                return @as(*T, @alignCast(@ptrCast(&self._dc.?.gpu.vram[addr & (Dreamcast.VRAMSize - 1)]))).*;
            },
            // Area 1 - 32bit access
            0x05000000...0x05FFFFFF, 0x07000000...0x07FFFFFF => {
                if (T == u64) {
                    sh4_log.debug("Read(64) from 0x{X:0>8}", .{physical_addr});
                    return @as(u64, self.read_physical(u32, physical_addr + 4)) << 32 | self.read_physical(u32, physical_addr);
                }
                return self._dc.?.gpu.read_vram(T, addr);
            },
            // Area 4 - Tile accelerator command input
            0x10000000...0x13FFFFFF => {
                // DCA3 Hack
                if (physical_addr & (@as(u32, 1) << 25) != 0) {
                    const index: u32 = (addr / 32) & 255;
                    const offset: u32 = addr & 31;

                    sh4_log.debug("Operand Cache addr = {X:0>8}, index = {d}, offset = {d} (OC.addr[index] = {X:0>8})", .{ addr, index, offset, self._operand_cache_state.addr[index] });

                    if (self._operand_cache_state.addr[index] != addr & ~@as(u32, 31))
                        sh4_log.warn("  (read)  Expected OC.addr[index] = {X:0>8}, got {X:0>8}\n", .{ addr & ~@as(u32, 31), self._operand_cache_state.addr[index] });

                    return @as([*]T, @alignCast(@ptrCast(&self.operand_cache_lines()[index])))[offset / @sizeOf(T)];
                }
            },
            // Area 7
            0x1C000000...0x1FFFFFFF => {
                // Only when area 7 in external memory space is accessed using virtual memory space, addresses H'1F00 0000
                // to H'1FFF FFFF of area 7 are not designated as a reserved area, but are equivalent to the P4 area
                // control register area in the physical memory space
                if (self._mmu_enabled) {
                    return self.read_p4(T, addr | 0xE000_0000);
                } else {
                    sh4_log.err(termcolor.red("Read({any}) to Area 7 without using virtual memory space: {X:0>8}"), .{ T, addr });
                    return 0;
                }
            },
            else => {},
        }

        return @as(*const T, @alignCast(@ptrCast(
            @constCast(self)._get_memory(addr),
        ))).*;
    }

    pub fn write(self: *@This(), comptime T: type, virtual_addr: addr_t, value: T) void {
        const physical_address = self.translate_address(virtual_addr) catch a: {
            break :a self.handle_tlb_miss(.DataTLBMissWrite, virtual_addr);
        };
        return self.write_physical(T, physical_address, value);
    }

    pub fn write_physical(self: *@This(), comptime T: type, physical_addr: addr_t, value: T) void {
        if ((comptime builtin.is_test) and self._dc == null) {
            switch (T) {
                u8 => return DebugHooks.write8.?(physical_addr, value),
                u16 => return DebugHooks.write16.?(physical_addr, value),
                u32 => return DebugHooks.write32.?(physical_addr, value),
                u64 => return DebugHooks.write64.?(physical_addr, value),
                else => @compileError("Invalid write type"),
            }
        }

        if (physical_addr >= 0x7C000000 and physical_addr <= 0x7FFFFFFF)
            return self.write_operand_cache(T, physical_addr, value);

        if (physical_addr >= 0xE0000000)
            return write_p4(self, T, physical_addr, value);

        // const addr = self.translate_address(virtual_addr); // We don't want that in JIT mode
        const addr = physical_addr & 0x1FFFFFFF;

        switch (addr) {
            // Area 0, and mirrors
            0x00000000...0x01FFFFFF, 0x02000000...0x03FFFFFF => {
                switch (addr) {
                    0x00200000...0x0021FFFF => {
                        check_type(&[_]type{u8}, T, "Invalid Write({any}) to 0x{X:0>8} (Flash) = 0x{X}\n", .{ T, addr, value });
                        self._dc.?.flash.write(addr & 0x1FFFF, value);
                        return;
                    },
                    0x005F6800...0x005F7FFF => {
                        const reg: HardwareRegister = @enumFromInt(addr);
                        if (addr >= 0x005F7000 and addr <= 0x005F709C) {
                            check_type(&[_]type{ u8, u16 }, T, "Invalid Write({any}) to 0x{X:0>8} (GDROM)\n", .{ T, addr });
                            return self._dc.?.gdrom.write_register(T, addr, value);
                        }
                        // Hardware registers
                        switch (reg) {
                            .SB_SFRES => { // Software Reset
                                check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (SB_SFRES)\n", .{ T, addr });
                                if (value == 0x00007611)
                                    self.software_reset();
                            },
                            .SB_E1ST, .SB_E2ST, .SB_DDST, .SB_SDST, .SB_PDST => {
                                if (value == 1)
                                    sh4_log.err(termcolor.red("Unimplemented {any} DMA initiation!"), .{reg});
                            },
                            .SB_ADSUSP, .SB_E1SUSP, .SB_E2SUSP, .SB_DDSUSP => {
                                if ((value & 1) == 1) {
                                    sh4_log.debug(termcolor.yellow("Unimplemented DMA Suspend Request to {any}"), .{reg});
                                }
                                return;
                            },
                            .SB_ADST => {
                                if (value == 1) {
                                    self._dc.?.aica.start_dma(self._dc.?);
                                }
                            },
                            .SB_GDEN => {
                                sh4_log.info("Write to SB_GDEN: {X}", .{value});
                                if (value == 0) self._dc.?.abort_gd_dma();
                                self._dc.?.hw_register_addr(T, addr).* = value;
                            },
                            .SB_GDST => {
                                if (value == 1) {
                                    sh4_log.info("SB_GDST DMA (ch0-DMA) initiation!", .{});
                                    self._dc.?.start_gd_dma();
                                } else {
                                    sh4_log.warn("Unexpected write to SB_GDST: {X}", .{value});
                                }
                            },
                            .SB_GDSTARD, .SB_GDLEND, .SB_ADSTAGD, .SB_E1STAGD, .SB_E2STAGD, .SB_DDSTAGD, .SB_ADSTARD, .SB_E1STARD, .SB_E2STARD, .SB_DDSTARD, .SB_ADLEND, .SB_E1LEND, .SB_E2LEND, .SB_DDLEND => {
                                sh4_log.warn(termcolor.yellow("Ignoring write({any}) to Read Only register {s} = {X:0>8}."), .{ T, @tagName(reg), value });
                                return;
                            },
                            .SB_MDAPRO => {
                                check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (SB_MDAPRO)\n", .{ T, addr });
                                // This register specifies the address range for Maple-DMA involving the system (work) memory.
                                // Check "Security code"
                                if (value & 0xFFFF0000 != 0x61550000) return;
                                self._dc.?.hw_register_addr(T, addr).* = value;
                            },
                            .SB_MDST => {
                                if (value == 1)
                                    self._dc.?.start_maple_dma();
                            },
                            .SB_ISTNRM => {
                                check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (SB_ISTNRM)\n", .{ T, addr });
                                // Interrupt can be cleared by writing "1" to the corresponding bit.
                                self._dc.?.hw_register(u32, .SB_ISTNRM).* &= ~(value & 0x3FFFFF);
                            },
                            .SB_ISTERR => {
                                check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (SB_ISTERR)\n", .{ T, addr });
                                // Interrupt can be cleared by writing "1" to the corresponding bit.
                                self._dc.?.hw_register(u32, .SB_ISTERR).* &= ~value;
                            },
                            .SB_C2DSTAT => {
                                check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (SB_C2DSTAT)\n", .{ T, addr });
                                self._dc.?.hw_register(u32, .SB_C2DSTAT).* = 0x10000000 | (0x03FFFFFF & value);
                            },
                            .SB_C2DST => {
                                if (value == 1) {
                                    self._dc.?.start_ch2_dma();
                                } else {
                                    self._dc.?.end_ch2_dma();
                                }
                            },
                            else => {
                                sh4_log.debug("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, HardwareRegisters.getRegisterName(addr), value });
                                self._dc.?.hw_register_addr(T, addr).* = value;
                            },
                        }
                        return;
                    },
                    0x005F8000...0x005F9FFF => {
                        if (T == u64) {
                            // FIXME: Allow 64bit writes to Palette RAM? Metropolis Street Racer does it, not sure how normal it is :)
                            if (addr >= 0x005F9000 and addr <= 0x005F9FFC) {
                                sh4_log.warn(termcolor.yellow("Write({any}) to Palette RAM @{X:0>8} = 0x{X:0>16}"), .{ T, addr, value });
                                self._dc.?.gpu._get_register_from_addr(u64, addr).* = value;
                                return;
                            }
                        }
                        check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (Holly Registers) = 0x{X}\n", .{ T, addr, value });
                        return self._dc.?.gpu.write_register(addr, value);
                    },
                    // NOTE: 0x00700000...0x00FFFFFF mirrors to 0x02700000...0x02FFFFFF
                    0x00700000...0x0070FFFF, 0x02700000...0x0270FFFF => {
                        check_type(&[_]type{ u8, u32 }, T, "Invalid Write({any}) to 0x{X:0>8} (AICA Registers) = 0x{X}\n", .{ T, addr, value });
                        return self._dc.?.aica.write_register(T, addr & 0x00FFFFFF, value);
                    },
                    0x00710000...0x00710008, 0x02710000...0x02710008 => {
                        check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (AICA RTC Registers) = 0x{X}\n", .{ T, addr, value });
                        return self._dc.?.aica.write_rtc_register(addr & 0x00FFFFFF, value);
                    },
                    0x00800000...0x00FFFFFF, 0x02800000...0x02FFFFFF => return self._dc.?.aica.write_mem(T, addr & 0x00FFFFFF, value),
                    else => {},
                }
            },
            // Area 1 - 64bit access
            0x04000000...0x04FFFFFF, 0x06000000...0x06FFFFFF => {
                @as(*T, @alignCast(@ptrCast(&self._dc.?.gpu.vram[addr & (Dreamcast.VRAMSize - 1)]))).* = value;
                return;
            },
            // Area 1 - 32bit access
            0x05000000...0x05FFFFFF, 0x07000000...0x07FFFFFF => {
                if (T == u64) {
                    sh4_log.debug("Write(64) to 0x{X:0>8} = 0x{X:0>16}", .{ addr, value });
                    self.write_physical(u32, addr, @truncate(value));
                    self.write_physical(u32, addr + 4, @truncate(value >> 32));
                    return;
                }
                return self._dc.?.gpu.write_vram(T, addr, value);
            },
            // Area 4
            0x10000000...0x13FFFFFF => {
                // DCA3 Hack
                if (physical_addr & (@as(u32, 1) << 25) != 0) {
                    const index: u32 = (addr / 32) & 255;
                    const offset: u32 = addr & 31;

                    sh4_log.debug("Operand Cache addr = {X:0>8}, index = {d}, offset = {d} (OC.addr[index] = {X:0>8})\n", .{ addr, index, offset, self._operand_cache_state.addr[index] });

                    if (self._operand_cache_state.addr[index] != addr & ~@as(u32, 31))
                        sh4_log.warn("  (write) Expected OC.addr[index] = {X:0>8}, got {X:0>8}\n", .{ addr & ~@as(u32, 31), self._operand_cache_state.addr[index] });
                    self._operand_cache_state.dirty[index] = true;

                    @as([*]T, @alignCast(@ptrCast(&self.operand_cache_lines()[index])))[offset / @sizeOf(T)] = value;
                    return;
                }
                check_type(&[_]type{u32}, T, "Invalid Write({any}) to 0x{X:0>8} (TA Registers) = 0x{X}\n", .{ T, addr, value });
                const LMMode = self._dc.?.read_hw_register(u32, if (addr >= 0x11000000 and addr < 0x12000000) .SB_LMMODE0 else .SB_LMMODE1);
                const access_32bit = LMMode != 0;
                return self._dc.?.gpu.write_ta(addr, &[1]u32{value}, if (access_32bit) .b32 else .b64);
            },
            // Area 7
            0x1C000000...0x1FFFFFFF => {
                // Only when area 7 in external memory space is accessed using virtual memory space, addresses H'1F00 0000
                // to H'1FFF FFFF of area 7 are not designated as a reserved area, but are equivalent to the P4 area
                // control register area in the physical memory space
                if (self._mmu_enabled) {
                    return self.write_p4(T, addr | 0xE000_0000, value);
                } else {
                    sh4_log.err(termcolor.red("Write({any}) to Area 7 without using virtual memory space: {X:0>8} = {X}"), .{ T, addr, value });
                    return;
                }
            },
            else => {},
        }

        @as(*T, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub fn set_trapa_callback(self: *@This(), callback: *const fn (userdata: *anyopaque) void, userdata: *anyopaque) void {
        if (EnableTRAPACallback) {
            self.on_trapa = .{ .callback = callback, .userdata = userdata };
        }
    }

    pub fn serialize(self: *const @This(), writer: anytype) !usize {
        var bytes: usize = 0;
        bytes += try writer.write(std.mem.sliceAsBytes(self.r[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self.r_bank[0..]));
        bytes += try writer.write(std.mem.asBytes(&self.sr));
        bytes += try writer.write(std.mem.asBytes(&self.gbr));
        bytes += try writer.write(std.mem.asBytes(&self.ssr));
        bytes += try writer.write(std.mem.asBytes(&self.spc));
        bytes += try writer.write(std.mem.asBytes(&self.sgr));
        bytes += try writer.write(std.mem.asBytes(&self.dbr));
        bytes += try writer.write(std.mem.asBytes(&self.vbr));
        bytes += try writer.write(std.mem.asBytes(&self.pc));
        bytes += try writer.write(std.mem.asBytes(&self.macl));
        bytes += try writer.write(std.mem.asBytes(&self.mach));
        bytes += try writer.write(std.mem.asBytes(&self.pr));
        bytes += try writer.write(std.mem.asBytes(&self.fpscr));
        bytes += try writer.write(std.mem.asBytes(&self.fpul));
        bytes += try writer.write(std.mem.sliceAsBytes(self.fp_banks[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self.store_queues[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self._operand_cache[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self.p4_registers[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self.utlb[0..]));
        bytes += try writer.write(std.mem.asBytes(&self.interrupt_requests));
        bytes += try writer.write(std.mem.sliceAsBytes(self._last_timer_update[0..]));
        bytes += try writer.write(std.mem.asBytes(&self.execution_state));
        bytes += try writer.write(std.mem.asBytes(&self._pending_cycles));
        bytes += try self._operand_cache_state.serialize(writer);
        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: anytype) !usize {
        var bytes: usize = 0;
        bytes += try reader.read(std.mem.sliceAsBytes(self.r[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self.r_bank[0..]));
        bytes += try reader.read(std.mem.asBytes(&self.sr));
        bytes += try reader.read(std.mem.asBytes(&self.gbr));
        bytes += try reader.read(std.mem.asBytes(&self.ssr));
        bytes += try reader.read(std.mem.asBytes(&self.spc));
        bytes += try reader.read(std.mem.asBytes(&self.sgr));
        bytes += try reader.read(std.mem.asBytes(&self.dbr));
        bytes += try reader.read(std.mem.asBytes(&self.vbr));
        bytes += try reader.read(std.mem.asBytes(&self.pc));
        bytes += try reader.read(std.mem.asBytes(&self.macl));
        bytes += try reader.read(std.mem.asBytes(&self.mach));
        bytes += try reader.read(std.mem.asBytes(&self.pr));
        bytes += try reader.read(std.mem.asBytes(&self.fpscr));
        bytes += try reader.read(std.mem.asBytes(&self.fpul));
        bytes += try reader.read(std.mem.sliceAsBytes(self.fp_banks[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self.store_queues[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self._operand_cache[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self.p4_registers[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self.utlb[0..]));
        bytes += try reader.read(std.mem.asBytes(&self.interrupt_requests));
        bytes += try reader.read(std.mem.sliceAsBytes(self._last_timer_update[0..]));
        bytes += try reader.read(std.mem.asBytes(&self.execution_state));
        bytes += try reader.read(std.mem.asBytes(&self._pending_cycles));
        bytes += try self._operand_cache_state.deserialize(reader);

        self.compute_interrupt_priorities();
        self.update_sse_settings();
        self._mmu_enabled = self.read_p4_register(mmu.MMUCR, .MMUCR).at;

        return bytes;
    }
};
