// Hitachi SH-4
// FIXME: Exact model is actually SH7091, I think.

const std = @import("std");
const builtin = @import("builtin");
const common = @import("./common.zig");
const termcolor = @import("termcolor.zig");

const sh4_log = std.log.scoped(.sh4);

const Dreamcast = @import("dreamcast.zig").Dreamcast;
const mmu = @import("./mmu.zig");
const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;
const P4MemoryRegister = MemoryRegisters.P4MemoryRegister;
const Interrupts = @import("Interrupts.zig");
const Interrupt = Interrupts.Interrupt;
const syscall = @import("syscall.zig");

const addr_t = common.addr_t;

const SR = packed struct(u32) {
    t: bool = undefined, // True/False condition or carry/borrow bit.
    s: bool = undefined, // Specifies a saturation operation for a MAC instruction

    _r0: u2 = 0,

    imask: u4 = 0xF, // Interrupt mask level
    q: bool = undefined, // State for divide step.
    m: bool = undefined,

    _r1: u5 = 0,

    fd: bool = false, // FPU disable bit.

    _r2: u12 = 0,

    bl: bool = true, // Exception/interrupt block bit (set to 1 by a reset, exception, or interrupt).
    rb: u1 = 1, // General register bank specifier in privileged mode (set to 1 by a reset, exception or interrupt).
    md: u1 = 1, // Processor mode. MD = 0: User mode (Some instructions cannot be executed, and some resources cannot be accessed). MD = 1: Privileged mode.

    _r3: u1 = 0,
};

const FPSCR = packed struct {
    rm: u2 = 1, // Rounding mode
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
    dn: bool = false, // Denormalization mode
    pr: u1 = 1, // Precision mode
    sz: u1 = 0, // Transfer size mode
    fr: u1 = 0, // Floating-point register bank

    _: u10 = undefined, // Reserved
};

const VirtualAddr = packed union {
    region: u3,
    addr: u29,
};

const StoreQueueAddr = packed struct(u32) {
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
    std.debug.assert(@bitSizeOf(SR) == @bitSizeOf(u32));
    std.debug.assert(@bitSizeOf(FPSCR) == @bitSizeOf(u32));
    std.debug.assert(@bitSizeOf(Instr) == @bitSizeOf(u16));
}

pub var JumpTable: [0x10000]u8 = .{1} ** 0x10000;

const ExecutionState = enum {
    Running,
    Sleep, // CPG: Operating, CPU: Halted, On-chip Peripheral Modules: Operating, Exiting Method: Interrupt, Reset
    DeepSleep, // CPG: Operating, CPU: Halted, On-chip Peripheral Modules: Operating, Exiting Method: Interrupt, Reset
    Standby, // CPG: Halted, CPU: Halted, On-chip Peripheral Modules: Halted, Exiting Method: Interrupt, Reset
    ModuleStandby, // Not implemented at all
};

pub const SH4 = struct {
    on_trapa: ?*const fn () void = null, // Debugging callback

    debug_trace: bool = false,

    execution_state: ExecutionState = .Running,

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
    mach: u32 = undefined, // Multiply-and-accumulate register high
    macl: u32 = undefined, // Multiply-and-accumulate register low
    pr: u32 = undefined, // Procedure register
    pc: u32 = 0xA0000000, // Program counter
    fpscr: FPSCR = .{}, // Floating-point status/control register
    fpul: u32 = undefined, // Floating-point communication register

    fp_banks: [2]extern union {
        fr: [16]f32,
        dr: [8]f64,
        qf: [4]f128,
    } = undefined,

    store_queues: [2][8]u32 align(4) = undefined,
    _operand_cache: []u8 align(4) = undefined,
    p4_registers: []u8 align(4) = undefined,

    interrupt_requests: u64 = 0,

    timer_cycle_counter: [3]u32 = .{0} ** 3, // Cycle counts before scaling.

    _allocator: std.mem.Allocator = undefined,
    _dc: ?*Dreamcast = null,
    _pending_cycles: u32 = 0,

    // Allows passing a null DC for testing purposes (Mostly for instructions that do not need access to RAM).
    pub fn init(allocator: std.mem.Allocator, dc: ?*Dreamcast) !SH4 {
        var sh4: SH4 = .{ ._dc = dc };

        sh4._allocator = allocator;

        // NOTE: Actual Operand cache is 16k, but we're only emulating the RAM accessible part, which is 8k.
        sh4._operand_cache = try sh4._allocator.alloc(u8, 0x2000);

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
        sh4.p4_registers = try sh4._allocator.alloc(u8, 0x1000);

        init_jump_table();

        sh4.reset();

        return sh4;
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.free(self._operand_cache);
        self._allocator.free(self.p4_registers);
    }

    pub fn reset(self: *@This()) void {
        self.r = undefined;
        self.r_bank = undefined;
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
        self.fpscr = .{};
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
    }

    // Reset state to after bios.
    pub fn state_after_boot_rom(self: *@This()) void {
        self.set_sr(@bitCast(@as(u32, 0x400000F1)));
        self.fpscr = @bitCast(@as(u32, 0x00040001));

        self.R(0x0).* = 0xAC0005D8;
        self.R(0x1).* = 0x00000009;
        self.R(0x2).* = 0xAC00940C;
        self.R(0x3).* = 0x00000000;
        self.R(0x4).* = 0xAC008300;
        self.R(0x5).* = 0xF4000000;
        self.R(0x6).* = 0xF4002000;
        self.R(0x7).* = 0x00000070;
        self.R(0x8).* = 0x00000000;
        self.R(0x9).* = 0x00000000;
        self.R(0xA).* = 0x00000000;
        self.R(0xB).* = 0x00000000;
        self.R(0xC).* = 0x00000000;
        self.R(0xD).* = 0x00000000;
        self.R(0xE).* = 0x00000000;
        self.R(0xF).* = 0x8D000000;

        self.gbr = 0x8C000000;
        self.ssr = 0x40000001;
        self.spc = 0x8C000776;
        self.sgr = 0x8d000000;
        self.dbr = 0x8C000010;
        self.vbr = 0x8C000000;
        self.pr = 0xAC00043C;
        self.fpul = 0x00000000;

        self.pc = 0xAC008300; // Start address of IP.bin Licence screen
    }

    inline fn set_sr(self: *@This(), value: SR) void {
        if (value.rb != self.sr.rb) {
            std.mem.swap([8]u32, self.r[0..8], &self.r_bank);
        }
        self.sr = @bitCast(@as(u32, @bitCast(value)) & 0x700083F3);
    }

    inline fn read_operand_cache(self: *const @This(), comptime T: type, virtual_addr: addr_t) T {
        return @constCast(self).operand_cache(T, virtual_addr).*;
    }
    inline fn operand_cache(self: *@This(), comptime T: type, virtual_addr: addr_t) *T {
        // Half of the operand cache can be used as RAM when CCR.ORA == 1, and some games do.
        std.debug.assert(self.read_p4_register(MemoryRegisters.CCR, .CCR).ora == 1);
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
                std.debug.assert(self.read_p4_register(MemoryRegisters.CCR, .CCR).oix == 1 or (virtual_addr >= 0x7C001000 and virtual_addr <= 0x7FFFF000));
                // This checks the only contiguous range when CCR.OIX = 1.
                std.debug.assert(self.read_p4_register(MemoryRegisters.CCR, .CCR).oix == 0 or (virtual_addr >= 0x7DFFF000 and virtual_addr <= 0x7E000FFF));
            }

            return @alignCast(@ptrCast(&self._operand_cache[virtual_addr & 0x3FFF]));
        } else {
            // Correct addressing, in case we end up needing it.
            if (self.read_p4_register(MemoryRegisters.CCR, .CCR).oix == 0) {
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
                const index = ((virtual_addr & 0x03000000) >> 16) | (virtual_addr & 0x3FFF);
                return @alignCast(@ptrCast(&self._operand_cache[index]));
            }
        }
    }

    pub inline fn read_p4_register(self: *const @This(), comptime T: type, r: P4MemoryRegister) T {
        return @constCast(self).p4_register_addr(T, @intFromEnum(r)).*;
    }

    pub inline fn p4_register(self: *@This(), comptime T: type, r: P4MemoryRegister) *T {
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
        return &self.fp_banks[self.fpscr.fr].fr[r];
    }

    pub inline fn DR(self: *@This(), r: u4) *f64 {
        std.debug.assert(r < 8);
        return &self.fp_banks[self.fpscr.fr].dr[r];
    }
    pub inline fn XF(self: *@This(), r: u4) *f32 {
        return &self.fp_banks[self.fpscr.fr +% 1].fr[r];
    }

    pub inline fn XD(self: *@This(), r: u4) *f64 {
        std.debug.assert(r < 8);
        return &self.fp_banks[self.fpscr.fr +% 1].dr[r];
    }

    pub inline fn QR(self: *@This(), r: u4) *f32 {
        std.debug.assert(r < 4);
        return &self.fp_banks[self.fpscr.fr].qr[r];
    }

    fn jump_to_interrupt(self: *@This()) void {
        sh4_log.debug(" => Jump to Interrupt: VBR: {X:0>8}, Code: {X:0>4}", .{ self.vbr, self.read_p4_register(u32, .INTEVT) });

        self.execution_state = .Running;
        self.spc = self.pc;
        self.ssr = @bitCast(self.sr);

        var new_sr = self.sr;
        new_sr.bl = true;
        new_sr.md = 1;
        new_sr.rb = 1;
        self.set_sr(new_sr);

        self.pc = self.vbr + 0x600;
    }

    fn jump_to_exception(self: *@This()) void {
        self.spc = self.pc;
        self.ssr = @bitCast(self.sr);

        var new_sr = self.sr;
        new_sr.bl = true;
        new_sr.md = 1;
        new_sr.rb = 1;
        self.set_sr(new_sr);

        const offset = 0x600; // TODO
        const UserBreak = false; // TODO
        if (self.dc.read_p4_register(MemoryRegisters.BRCR, .BRCR).ubde == 1 and UserBreak) {
            self.pc = self.dbr;
        } else {
            self.pc = self.vbr + offset;
        }
    }

    pub fn execute(self: *@This()) u32 {
        // When the BL bit in SR is 0, exceptions and interrupts are accepted.

        // See h14th002d2.pdf page 665 (or 651)
        if (!self.sr.bl or self.execution_state != .Running) {
            if (self.interrupt_requests != 0) {
                // TODO: Search the highest priority interrupt.
                const first_set = @ctz(self.interrupt_requests);
                // Check it against the cpu interrupt mask
                if (Interrupts.InterruptLevel[first_set] >= self.sr.imask) {
                    self.interrupt_requests &= ~(@as(u64, 1) << @truncate(first_set)); // Clear the request
                    self.p4_register(u32, .INTEVT).* = Interrupts.InterruptINTEVTCodes[first_set];
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

        if (self.execution_state == .Running or self.execution_state == .ModuleStandby) {
            self._execute(self.pc);
            self.pc += 2;

            const cycles = self._pending_cycles;
            self._pending_cycles = 0;
            return cycles;
        } else {
            // FIXME: Not sure if this is a thing.
            self.add_cycles(8);
            return 8;
        }
    }

    pub fn request_interrupt(self: *@This(), int: Interrupt) void {
        sh4_log.debug(" (Interrupt request! {s})", .{std.enums.tagName(Interrupt, int) orelse "Unknown"});
        self.interrupt_requests |= @as(u33, 1) << @intFromEnum(int);
    }

    inline fn timer_prescaler(value: u3) u32 {
        switch (value) {
            0 => return 4,
            1 => return 16,
            2 => return 64,
            3 => return 256,
            5 => return 1024,
            else => unreachable,
        }
    }

    pub inline fn advance_timers(self: *@This(), cycles: u32) void {
        const TSTR = self.read_p4_register(u32, .TSTR);

        // When one of bits STR0–STR2 is set to 1 in the timer start register (TSTR), the timer counter
        // (TCNT) for the corresponding channel starts counting. When TCNT underflows, the UNF flag is
        // set in the corresponding timer control register (TCR). If the UNIE bit in TCR is set to 1 at this
        // time, an interrupt request is sent to the CPU. At the same time, the value is copied from TCOR
        // into TCNT, and the count-down continues (auto-reload function).
        const timers = .{
            .{ .counter = P4MemoryRegister.TCNT0, .control = P4MemoryRegister.TCR0, .constant = P4MemoryRegister.TCOR0, .interrupt = Interrupt.TUNI0 },
            .{ .counter = P4MemoryRegister.TCNT1, .control = P4MemoryRegister.TCR1, .constant = P4MemoryRegister.TCOR1, .interrupt = Interrupt.TUNI1 },
            .{ .counter = P4MemoryRegister.TCNT2, .control = P4MemoryRegister.TCR2, .constant = P4MemoryRegister.TCOR2, .interrupt = Interrupt.TUNI2 },
        };

        inline for (0..3) |i| {
            if ((TSTR >> @intCast(i)) & 0x1 == 1) {
                const tcnt = self.p4_register(u32, timers[i].counter);
                const tcr = self.p4_register(MemoryRegisters.TCR, timers[i].control);

                self.timer_cycle_counter[i] += cycles;

                const scale = SH4.timer_prescaler(tcr.*.tpsc);
                if (self.timer_cycle_counter[i] >= scale) {
                    self.timer_cycle_counter[i] -= scale;

                    if (tcnt.* == 0) {
                        tcr.*.unf = 1; // Signals underflow
                        tcnt.* = self.p4_register(u32, timers[i].constant).*; // Reset counter
                        if (tcr.*.unie == 1)
                            self.request_interrupt(timers[i].interrupt);
                    } else {
                        tcnt.* -= 1;
                    }
                }
            }
        }
    }

    pub inline fn add_cycles(self: *@This(), cycles: u32) void {
        self.advance_timers(cycles);
        self._pending_cycles += cycles;
    }

    pub inline fn _execute(self: *@This(), addr: addr_t) void {
        // Guiding the compiler a bit. Yes, that helps a lot :)
        // Instruction should be in Boot ROM, or RAM.
        const physical_addr = addr & 0x1FFFFFFF;
        std.debug.assert(physical_addr >= 0x00000000 and physical_addr <= 0x00020000 or physical_addr >= 0x0C000000 and physical_addr <= 0x0D000000);

        const opcode = self.read16(physical_addr);
        const instr = Instr{ .value = opcode };
        const desc = Opcodes[JumpTable[opcode]];

        if ((comptime builtin.mode == .Debug or builtin.mode == .ReleaseSafe) and self.debug_trace)
            std.debug.print("[{X:0>4}] {b:0>16} {s: <20}\tR{d: <2}={X:0>8}, R{d: <2}={X:0>8}, d={X:0>1}, d8={X:0>2}, d12={X:0>3}\n", .{ addr, opcode, disassemble(instr, self._allocator) catch unreachable, instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).*, instr.nmd.d, instr.nd8.d, instr.d12.d });

        desc.fn_(self, instr);

        if ((comptime builtin.mode == .Debug or builtin.mode == .ReleaseSafe) and self.debug_trace)
            std.debug.print("[{X:0>4}] {X: >16} {s: <20}\tR{d: <2}={X:0>8}, R{d: <2}={X:0>8}\n", .{ addr, opcode, "", instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).* });

        self.add_cycles(desc.issue_cycles);
    }

    pub fn store_queue_write(self: *@This(), virtual_addr: addr_t, value: u32) void {
        const sq_addr: StoreQueueAddr = @bitCast(virtual_addr);
        sh4_log.debug("  StoreQueue write @{X:0>8} = 0x{X:0>8} ({any})", .{ virtual_addr, value, sq_addr });
        std.debug.assert(sq_addr.spec == 0b111000);
        self.store_queues[sq_addr.sq][sq_addr.lw_spec] = value;
    }

    pub fn start_dmac(self: *@This(), channel: u32) void {
        std.debug.assert(channel == 2); // TODO: Implement others? It is needed?

        const chcr = self.read_p4_register(MemoryRegisters.CHCR, .CHCR2);

        sh4_log.debug(" CHCR: {any}", .{chcr});

        // NOTE: I think the DC only uses 32 bytes transfers, but I'm not 100% sure.
        std.debug.assert(chcr.ts == 0b100);
        std.debug.assert(chcr.rs == 2); // "External request, single address mode"

        const src_addr = self.read_p4_register(u32, .SAR2) & 0x1FFFFFFF;
        const dst_addr = self.read_p4_register(u32, .DAR2) & 0x1FFFFFFF;
        const transfer_size: u32 = switch (chcr.ts) {
            0 => 8, // Quadword size
            1 => 1, // Byte
            2 => 2, // Word
            3 => 4, // Longword
            4 => 32, // 32-bytes block
            else => unreachable,
        };
        const len = self.read_p4_register(u32, .DMATCR2);
        const byte_len = transfer_size * len;

        const dst_stride: i32 = switch (chcr.dm) {
            0 => 0,
            1 => 1,
            2 => -1,
            else => unreachable,
        };
        const src_stride: i32 = switch (chcr.sm) {
            0 => 0,
            1 => 1,
            2 => -1,
            else => unreachable,
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
            // Polygon Path
            if (dst_addr >= 0x10000000 and dst_addr < 0x10800000 or dst_addr >= 0x12000000 and dst_addr < 0x12800000) {
                var src: [*]u32 = @alignCast(@ptrCast(self._get_memory(src_addr)));
                for (0..len) |_| {
                    self._dc.?.gpu.write_ta_fifo_polygon_path(src[0..8]);
                    src += 8;
                }
            }
            // YUV Converter Path
            if (dst_addr >= 0x10800000 and dst_addr < 0x11000000 or dst_addr >= 0x12800000 and dst_addr < 0x13000000) {
                self._dc.?.gpu.ta_fifo_yuv_converter_path();
            }
            // Direct Texture Path
            if (dst_addr >= 0x11000000 and dst_addr < 0x12000000 or dst_addr >= 0x13000000 and dst_addr < 0x14000000) {
                self._dc.?.gpu.write_ta_fifo_direct_texture_path(dst_addr, @as([*]u8, @ptrCast(self._get_memory(src_addr)))[0..byte_len]);
            }
        } else if (is_memcpy_safe) {
            std.debug.assert(dst_stride == src_stride);
            // FIXME: When can we safely memset?
            const src = @as([*]u8, @ptrCast(self._get_memory(src_addr)));
            const dst = @as([*]u8, @ptrCast(self._get_memory(dst_addr)));
            @memcpy(dst[0..byte_len], src[0..byte_len]);
        } else {
            var curr_dst: i32 = @intCast(dst_addr);
            var curr_src: i32 = @intCast(src_addr);
            for (0..byte_len / 4) |_| {
                self.write32(@intCast(curr_dst), self.read32(@intCast(curr_src)));
                curr_dst += 4 * dst_stride;
                curr_src += 4 * src_stride;
            }
        }

        // TODO: Schedule for later?
        if (chcr.ie == 1)
            self.request_interrupt(Interrupts.Interrupt.DMTE2);
        self.p4_register(u32, .SAR2).* += len;
        self.p4_register(u32, .DAR2).* += len;
        self.p4_register(u32, .DMATCR2).* = 0;
        self.p4_register(MemoryRegisters.CHCR, .CHCR2).*.te = 1;
    }

    fn panic_debug(self: @This(), comptime fmt: []const u8, args: anytype) noreturn {
        std.debug.print("PC: {X:0>8}\n", .{self.pc});
        std.debug.panic(fmt, args);
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

        if (false) {
            // MMU: Looks like most game don't use it at all. TODO: Expose it as an option.
            const physical_addr = self.mmu_translate_utbl(addr) catch |e| {
                // FIXME: Handle exceptions
                sh4_log.err("\u{001B}[31mError in utlb _read: {any} at {X:0>8}\u{001B}[0m", .{ e, addr });
                unreachable;
            };

            if (physical_addr != addr)
                sh4_log.info("  Write UTLB Hit: {x:0>8} => {x:0>8}", .{ addr, physical_addr });
        }

        // NOTE: These cases are out of order as an optimization.
        //       Empirically tested on interpreter_perf (i.e. 200_000_000 first 'ticks' of Sonic Adventure and the Boot ROM).
        //       The compile seems to really having equal length ranges (and also easily maskable, I guess)!
        switch (addr) {
            0x0C000000...0x0FFFFFFF => { // Area 3 - System RAM (16MB) - 0x0C000000 to 0x0FFFFFFF, mirrored 4 times, I think.
                return &self._dc.?.ram[addr & 0x00FFFFFF];
            },
            0x04000000...0x07FFFFFF => {
                return self._dc.?.gpu._get_vram(addr);
            },

            0x00000000...0x03FFFFFF => { // Area 0 - Boot ROM, Flash ROM, Hardware Registers
                switch (addr) {
                    0x00000000...0x001FFFFF => {
                        return &self._dc.?.boot[addr];
                    },
                    0x00200000...0x0021FFFF => {
                        return &self._dc.?.flash[addr - 0x200000];
                    },
                    0x005F6800...0x005F7FFF => {
                        return self._dc.?.hw_register_addr(u8, addr);
                    },
                    0x005F8000...0x005F9FFF => {
                        return self._dc.?.gpu._get_register_from_addr(u8, addr);
                    },
                    0x005FA000...0x005FFFFF => {
                        return self._dc.?.hw_register_addr(u8, addr);
                    },
                    0x00600000...0x006007FF => {
                        const static = struct {
                            var once = false;
                        };
                        if (!static.once) {
                            static.once = true;
                            sh4_log.warn(termcolor.yellow("  Unimplemented _get_memory to MODEM: {X:0>8} (This will only be reported once)"), .{addr});
                        }
                        return @ptrCast(&self._dc.?._dummy);
                    },
                    0x00700000...0x00707FE0 => { // G2 AICA Register
                        @panic("_get_memory to AICA Register. This should be handled in read/write functions.");
                    },
                    0x00710000...0x00710008 => { // G2 AICA RTC Registers
                        @panic("_get_memory to AICA RTC Register. This should be handled in read/write functions.");
                    },
                    0x00800000...0x009FFFFF => { // G2 Wave Memory
                        return &self._dc.?.aica.wave_memory[addr - 0x00800000];
                    },
                    else => {
                        sh4_log.warn(termcolor.yellow("  Unimplemented _get_memory to Area 0: {X:0>8}"), .{addr});
                        return @ptrCast(&self._dc.?._dummy);
                    },
                }
            },
            0x08000000...0x0BFFFFFF => { // Area 2 - Nothing
                self.panic_debug("Invalid _get_memory to Area 2 @{X:0>8}", .{addr});
            },
            0x10000000...0x13FFFFFF => { // Area 4 - Tile accelerator command input
                self.panic_debug("Unexpected _get_memory to Area 4 @{X:0>8} - This should only be accessible via write32 or DMA.", .{addr});
            },
            0x14000000...0x17FFFFFF => { // Area 5 - Expansion (modem) port
                const static = struct {
                    var once = false;
                };
                if (!static.once) {
                    static.once = true;
                    sh4_log.warn(termcolor.yellow("Unimplemented _get_memory to Area 5: {X:0>8} (This will only be reported once)"), .{addr});
                }
                return @ptrCast(&self._dc.?._dummy);
            },
            0x18000000...0x1BFFFFFF => { // Area 6 - Nothing
                self.panic_debug("Invalid _get_memory to Area 6 @{X:0>8}", .{addr});
            },
            // Area 7 - Internal I/O registers (same as P4)
            0x1F000000...0x1FFFFFFF => {
                std.debug.assert(self.sr.md == 1);
                return self.p4_register_addr(u8, addr);
            },
            else => {
                unreachable;
            },
        }
    }

    pub inline fn read8(self: @This(), virtual_addr: addr_t) u8 {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                else => {
                    sh4_log.debug("  Read8 to hardware register @{X:0>8} {s} = 0x{X:0>2}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), @as(*const u8, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
                    ))).* });
                },
            }
        } else if (virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF) {
            return self.read_operand_cache(u8, virtual_addr);
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            sh4_log.debug("  Read8 to hardware register @{X:0>8} {s} ", .{ addr, MemoryRegisters.getRegisterName(addr) });
        }

        return @as(*const u8, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub inline fn read16(self: @This(), virtual_addr: addr_t) u16 {
        const addr = virtual_addr & 0x1FFFFFFF;

        // SH4 Hardware registers
        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                @intFromEnum(P4MemoryRegister.RFCR) => {
                    // Hack: This is the Refresh Count Register, related to DRAM control.
                    //       If don't think its proper emulation is needed, but it's accessed by the bios,
                    //       probably for synchronization purposes. I assume returning a contant value to pass this check
                    //       is enough for now, as games shouldn't access that themselves.
                    sh4_log.debug("[Note] Access to Refresh Count Register.", .{});
                    return 0x0011;
                    // Otherwise, this is 10-bits register, respond with the 6 unused upper bits set to 0.
                },
                @intFromEnum(P4MemoryRegister.PDTRA) => {
                    // Note: I have absolutely no idea what's going on here.
                    //       This is directly taken from Flycast, which already got it from Chankast.
                    //       This is needed for the bios to work properly, without it, it will
                    //       go to sleep mode with all interrupts disabled early on.
                    const tpctra: u32 = self.read_p4_register(u32, .PCTRA);
                    const tpdtra: u32 = self.read_p4_register(u32, .PDTRA);

                    var tfinal: u16 = 0;
                    if ((tpctra & 0xf) == 0x8) {
                        tfinal = 3;
                    } else if ((tpctra & 0xf) == 0xB) {
                        tfinal = 3;
                    } else {
                        tfinal = 0;
                    }

                    if ((tpctra & 0xf) == 0xB and (tpdtra & 0xf) == 2) {
                        tfinal = 0;
                    } else if ((tpctra & 0xf) == 0xC and (tpdtra & 0xf) == 2) {
                        tfinal = 3;
                    }

                    tfinal |= @intFromEnum(self._dc.?.cable_type) << 8;

                    return tfinal;
                },
                // FIXME: Not emulated at all, these clash with my P4 access pattern :(
                @intFromEnum(P4MemoryRegister.PMCR1) => {
                    return 0;
                },
                @intFromEnum(P4MemoryRegister.PMCR2) => {
                    return 0;
                },
                else => {
                    sh4_log.debug("  Read16 to P4 register @{X:0>8} {s} = {X:0>4}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), @as(*const u16, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
                    ))).* });
                },
            }
        } else if (virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF) {
            return self.read_operand_cache(u16, virtual_addr);
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            sh4_log.debug("  Read16 to hardware register @{X:0>8} {s} ", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr) });
        }
        if (addr >= 0x00710000 and addr <= 0x00710008) {
            return @truncate(self._dc.?.aica.read_rtc_register(addr));
        }

        return @as(*const u16, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub inline fn read32(self: @This(), virtual_addr: addr_t) u32 {
        const addr = virtual_addr & 0x1FFFFFFF;

        switch (virtual_addr) {
            0xFF000000...0xFFFFFFFF => {
                sh4_log.debug("  Read32 to P4 register @{X:0>8} {s} = 0x{X:0>8}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), @as(*const u32, @alignCast(@ptrCast(
                    @constCast(&self)._get_memory(addr),
                ))).* });
            },
            0x7C000000...0x7FFFFFFF => {
                return self.read_operand_cache(u32, virtual_addr);
            },
            else => {},
        }

        switch (addr) {
            0x005F6800...0x005F7FFF => {
                sh4_log.debug("  Read32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, MemoryRegisters.getRegisterName(addr), @as(*const u32, @alignCast(@ptrCast(
                    @constCast(&self)._get_memory(addr),
                ))).* });
            },
            0x00700000...0x00707FE0 => {
                return self._dc.?.aica.read_register(addr);
            },
            0x00710000...0x00710008 => {
                return self._dc.?.aica.read_rtc_register(addr);
            },
            else => {},
        }

        return @as(*const u32, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub inline fn read64(self: @This(), virtual_addr: addr_t) u64 {
        const addr = virtual_addr & 0x1FFFFFFF;

        const r = @as(*const u64, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;

        // Small sanity check.
        if (comptime builtin.mode == .Debug or builtin.mode == .ReleaseSafe) {
            const low: u64 = self.read32(addr);
            const high: u64 = self.read32(addr + 4);
            const check = high << 32 | low;
            std.debug.assert(check == r);
        }

        return r;
    }

    pub inline fn write8(self: *@This(), virtual_addr: addr_t, value: u8) void {
        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                // SDMR2/SDMR3
                0xFF900000...0xFF90FFFF, 0xFF940000...0xFF94FFFF => {
                    // Ignore it, it's not implemented but it also doesn't fit in our P4 register remapping.
                    return;
                },
                else => {
                    sh4_log.debug("  Write8 to P4 register @{X:0>8} {s} = 0x{X:0>2}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), value });
                },
            }
        } else if (virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF) {
            self.operand_cache(u8, virtual_addr).* = value;
            return;
        }

        const addr = virtual_addr & 0x1FFFFFFF;
        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            // Hardware registers
            switch (addr) {
                else => {
                    sh4_log.debug("  Write8 to hardware register @{X:0>8} {s} = 0x{X:0>2}", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                },
            }
        }

        // write8 to GPU registers not implemented
        std.debug.assert(!(addr >= 0x005F8000 and addr <= 0x005FA000));
        // write8 to TA not implemented
        std.debug.assert(!(addr >= 0x10000000 and addr <= 0x14000000));

        @as(*u8, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub inline fn write16(self: *@This(), virtual_addr: addr_t, value: u16) void {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                @intFromEnum(P4MemoryRegister.RTCSR), @intFromEnum(P4MemoryRegister.RTCNT), @intFromEnum(P4MemoryRegister.RTCOR) => {
                    std.debug.assert(value & 0xFF00 == 0b10100101_00000000);
                    self.p4_register_addr(u16, addr).* = (value & 0xFF);
                    return;
                },
                @intFromEnum(P4MemoryRegister.RFCR) => {
                    std.debug.assert(value & 0b11111100_00000000 == 0b10100100_00000000);
                    self.p4_register_addr(u16, addr).* = (value & 0b11_11111111);
                    return;
                },
                // FIXME: Not emulated at all, these clash with my P4 access pattern :(
                @intFromEnum(P4MemoryRegister.PMCR1) => {
                    sh4_log.warn("Write to non implemented P4 register PMCR1: {X:0>4}.", .{value});
                    return;
                },
                @intFromEnum(P4MemoryRegister.PMCR2) => {
                    sh4_log.warn("Write to non implemented P4 register PMCR2: {X:0>4}.", .{value});
                    return;
                },
                else => {
                    sh4_log.debug("  Write16 to P4 register @{X:0>8} {s} = 0x{X:0>4}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), value });
                },
            }
        } else if (virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF) {
            self.operand_cache(u16, virtual_addr).* = value;
            return;
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            switch (addr) {
                else => {
                    sh4_log.debug("  Write16 to hardware register @{X:0>8} {s} = 0x{X:0>4}", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                },
            }
        }

        // write16 to GPU registers not implemented
        std.debug.assert(!(addr >= 0x005F8000 and addr <= 0x005FA000));
        // write16 to TA not implemented
        std.debug.assert(!(addr >= 0x10000000 and addr <= 0x14000000));

        @as(*u16, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub inline fn write32(self: *@This(), virtual_addr: addr_t, value: u32) void {
        if (virtual_addr >= 0xE0000000) {
            // P4
            if (virtual_addr < 0xE4000000) {
                self.store_queue_write(virtual_addr, value);
                return;
            }
            if (virtual_addr >= 0xFF000000) {
                switch (virtual_addr) {
                    else => {
                        sh4_log.debug("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ virtual_addr, MemoryRegisters.getP4RegisterName(virtual_addr), value });
                    },
                }
            }
        } else if (virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF) {
            self.operand_cache(u32, virtual_addr).* = value;
            return;
        }

        const addr = virtual_addr & 0x1FFFFFFF;
        switch (addr) {
            0x005F6800...0x005F7FFF => {
                // Hardware registers
                switch (addr) {
                    @intFromEnum(MemoryRegister.SB_SFRES) => {
                        // SB_SFRES, Software Reset
                        if (value == 0x00007611) {
                            self.software_reset();
                        }
                        return;
                    },
                    @intFromEnum(MemoryRegister.SB_ADST) => {
                        if (value == 1) {
                            self._dc.?.aica.start_dma(self._dc.?);
                        }
                    },
                    @intFromEnum(MemoryRegister.SB_MDAPRO) => {
                        // This register specifies the address range for Maple-DMA involving the system (work) memory.
                        // Check "Security code"
                        if (value & 0xFFFF0000 != 0x61550000) return;
                    },
                    @intFromEnum(MemoryRegister.SB_MDST) => {
                        if (value == 1) {
                            self._dc.?.start_maple_dma();
                            return;
                        }
                    },
                    @intFromEnum(MemoryRegister.SB_ISTNRM) => {
                        // Interrupt can be cleared by writing "1" to the corresponding bit.
                        self._dc.?.hw_register(u32, .SB_ISTNRM).* &= ~(value & 0x3FFFFF);
                        return;
                    },
                    @intFromEnum(MemoryRegister.SB_ISTERR) => {
                        // Interrupt can be cleared by writing "1" to the corresponding bit.
                        self._dc.?.hw_register(u32, .SB_ISTERR).* &= ~value;
                        return;
                    },
                    @intFromEnum(MemoryRegister.SB_C2DSTAT) => {
                        self._dc.?.hw_register(u32, .SB_C2DSTAT).* = 0x10000000 | (0x03FFFFFF & value);
                        return;
                    },
                    @intFromEnum(MemoryRegister.SB_C2DST) => {
                        if (value == 1) {
                            self._dc.?.start_ch2_dma();
                        } else {
                            self._dc.?.end_ch2_dma();
                        }
                        return;
                    },
                    else => {
                        sh4_log.debug("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                    },
                }
            },
            0x005F8000...0x005F9FFF => {
                return self._dc.?.gpu.write_register(addr, value);
            },
            0x00700000...0x0070FFFF => {
                return self._dc.?.aica.write_register(addr, value);
            },
            0x00710000...0x00710008 => {
                return self._dc.?.aica.write_rtc_register(addr, value);
            },
            0x10000000...0x13FFFFFF => {
                return self._dc.?.gpu.write_ta(addr, value);
            },
            else => {},
        }

        @as(*u32, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub inline fn write64(self: *@This(), virtual_addr: addr_t, value: u64) void {
        // This isn't efficient, but avoids repeating all the logic of write32.
        self.write32(virtual_addr, @truncate(value));
        self.write32(virtual_addr + 4, @truncate(value >> 32));
    }

    // MMU Stub functions
    // NOTE: This is dead code, the MMU is not emulated and utlb_entries are not in this struct anymore (reducing the size of the struct helps a lot with performance).

    pub fn mmu_utlb_match(self: @This(), virtual_addr: addr_t) !mmu.UTLBEntry {
        const asid = self.dc.read_p4_register(mmu.PTEH, MemoryRegister.PTEH).asid;
        const vpn: u22 = @truncate(virtual_addr >> 10);

        const shared_access = self.dc.read_p4_register(mmu.MMUCR, MemoryRegister.MMUCR).sv == 0 or self.sr.md == 0;
        var found: ?mmu.UTLBEntry = null;
        for (self.utlb_entries) |entry| {
            if (entry.v == 1 and mmu.vpn_match(vpn, entry.vpn, entry.sz) and ((entry.sh == 0 and shared_access) or
                (entry.asid == asid)))
            {
                if (found != null)
                    return error.DataTLBMutipleHitExpection;
                found = entry;
            }
        }
        if (found == null)
            return error.DataTLBMissExpection;
        return found.?;
    }

    pub fn mmu_translate_utbl(self: @This(), virtual_addr: addr_t) !addr_t {
        std.debug.assert(virtual_addr & 0xE0000000 == 0 or virtual_addr & 0xE0000000 == 0x60000000);
        if (self.dc.read_p4_register(mmu.MMUCR, MemoryRegister.MMUCR).at == 0) return virtual_addr;

        const entry = try mmu_utlb_match(self, virtual_addr);

        //try self.check_memory_protection(entry, write);

        const ppn = @as(u32, @intCast(entry.ppn));
        switch (entry.sz) {
            0b00 => { // 1-Kbyte page
                return ppn << 10 | (virtual_addr & 0b11_1111_1111);
            },
            0b01 => { // 4-Kbyte page
                return ppn << 12 | (virtual_addr & 0b1111_1111_1111);
            },
            0b10 => { //64-Kbyte page
                return ppn << 16 | (virtual_addr & 0b1111_1111_1111_1111);
            },
            0b11 => { // 1-Mbyte page
                return ppn << 20 | (virtual_addr & 0b1111_1111_1111_1111_1111);
            },
        }
    }

    pub fn mmu_translate_itbl(self: @This(), virtual_addr: addr_t) !u32 {
        _ = virtual_addr;
        _ = self;
        unreachable;
    }

    pub fn check_memory_protection(self: @This(), entry: mmu.UTLBEntry, write: bool) !void {
        if (self.sr.md == 0) {
            switch (entry.pr) {
                0b00, 0b01 => return error.DataTLBProtectionViolationExpection,
                0b10 => if (write and entry.w) return error.DataTLBProtectionViolationExpection,
                0b11 => if (write and entry.w and entry.d == 0) return error.InitalPageWriteException,
                else => {},
            }
        } else {
            // switch (entry.pr) {
            //    0b01, 0b11 => if(write and entry.d) return error.InitalPageWriteException,
            //    0b00, 0b01 => if(write) return error.DataTLBProtectionViolationExpection,
            // }
        }
    }
};

inline fn zero_extend(d: anytype) u32 {
    return @intCast(d);
}

inline fn sign_extension_u8(d: u8) i32 {
    if ((d & 0x80) == 0) {
        return @bitCast(0x000000FF & zero_extend(d));
    } else {
        return @bitCast(0xFFFFFF00 | zero_extend(d));
    }
}

inline fn sign_extension_u12(d: u12) i32 {
    if ((d & 0x800) == 0) {
        return @bitCast(0x00000FFF & zero_extend(d));
    } else {
        return @bitCast(0xFFFFF000 | zero_extend(d));
    }
}

inline fn sign_extension_u16(d: u16) i32 {
    if ((d & 0x8000) == 0) {
        return @bitCast(0x0000FFFF & zero_extend(d));
    } else {
        return @bitCast(0xFFFF0000 | zero_extend(d));
    }
}

inline fn as_i32(val: u32) i32 {
    return @bitCast(val);
}

fn unknown(cpu: *SH4, opcode: Instr) void {
    std.debug.print("Unknown opcode: 0x{X:0>4} 0b{b:0>16}\n", .{ opcode.value, opcode.value });
    std.debug.print("  CPU State: PC={X:0>8}\n", .{cpu.pc});
    @panic("Unknown opcode");
}

fn nop(_: *SH4, _: Instr) void {}

fn unimplemented(_: *SH4, opcode: Instr) void {
    std.debug.print("Unimplemented opcode: {s}\n", .{Opcodes[JumpTable[@as(u16, @bitCast(opcode))]].name});
    @panic("Unimplemented");
}

fn mov_rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.m).*;
}

fn mov_imm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nd8.n).* = @bitCast(sign_extension_u8(opcode.nd8.d));
}

// Stores the effective address of the source operand into general register R0.
// The 8-bit displacement is zero-extended and quadrupled. Consequently, the relative interval from the operand is PC + 1020 bytes.
// The PC is the address four bytes after this instruction, but the lowest two bits of the PC are fixed at 00.
// TODO: If this instruction is executed in a delay slot, a slot illegal instruction exception will be generated.
fn mova_atdispPC_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = (cpu.pc & 0xFFFFFFFC) + 4 + (zero_extend(opcode.nd8.d) << 2);
}

// Stores immediate data, sign-extended to longword, in general register Rn.
// The data is stored from memory address (PC + 4 + displacement * 2).
// The 8-bit displacement is multiplied by two after zero-extension, and so the relative distance from the table is in the range up to PC + 4 + 510 bytes.
// The PC value is the address of this instruction.
fn movw_atdispPC_Rn(cpu: *SH4, opcode: Instr) void {
    const n = opcode.nd8.n;
    const d = zero_extend(opcode.nd8.d) << 1;
    cpu.R(n).* = @bitCast(sign_extension_u16(cpu.read16(cpu.pc + 4 + d)));
}

// The 8-bit displacement is multiplied by four after zero-extension, and so the relative distance from the operand is in the range up to PC + 4 + 1020 bytes.
// The PC value is the address of this instruction. A value with the lower 2 bits adjusted to 00 is used in address calculation.
fn movl_atdispPC_Rn(cpu: *SH4, opcode: Instr) void {
    const n = opcode.nd8.n;
    const d = zero_extend(opcode.nd8.d) << 2;
    cpu.R(n).* = cpu.read32((cpu.pc & 0xFFFFFFFC) + 4 + d);
}

fn movb_at_rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(cpu.read8(cpu.R(opcode.nmd.m).*)));
}

fn movw_at_rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(cpu.read16(cpu.R(opcode.nmd.m).*)));
}

fn movl_at_rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.read32(cpu.R(opcode.nmd.m).*);
}

fn movb_rm_at_rn(cpu: *SH4, opcode: Instr) void {
    cpu.write8(cpu.R(opcode.nmd.n).*, @truncate(cpu.R(opcode.nmd.m).*));
}

fn movw_rm_at_rn(cpu: *SH4, opcode: Instr) void {
    cpu.write16(cpu.R(opcode.nmd.n).*, @truncate(cpu.R(opcode.nmd.m).*));
}

fn movl_rm_at_rn(cpu: *SH4, opcode: Instr) void {
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.R(opcode.nmd.m).*);
}

fn movb_at_rm_inc_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(cpu.read8(cpu.R(opcode.nmd.m).*)));
    if (opcode.nmd.n != opcode.nmd.m) {
        cpu.R(opcode.nmd.m).* += 1;
    }
}

// The loaded data is sign-extended to 32 bit before being stored in the destination register.
fn movw_at_rm_inc_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(cpu.read16(cpu.R(opcode.nmd.m).*)));
    if (opcode.nmd.n != opcode.nmd.m) {
        cpu.R(opcode.nmd.m).* += 2;
    }
}

fn movl_at_rm_inc_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.read32(cpu.R(opcode.nmd.m).*);
    if (opcode.nmd.n != opcode.nmd.m) {
        cpu.R(opcode.nmd.m).* += 4;
    }
}

fn movb_rm_at_rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 1;
    cpu.write8(cpu.R(opcode.nmd.n).*, @truncate(cpu.R(opcode.nmd.m).*));
}

fn movw_rm_at_rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 2;
    cpu.write16(cpu.R(opcode.nmd.n).*, @truncate(cpu.R(opcode.nmd.m).*));
    // TODO: Possible Exceptions
    // Data TLB multiple-hit exception
    // Data TLB miss exception
    // Data TLB protection violation exception
    // Data address error
    // Initial page write exception
}

fn movl_rm_at_rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.R(opcode.nmd.m).*);
}

fn movb_at_disp_Rm_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = @bitCast(sign_extension_u8(cpu.read8(cpu.R(opcode.nmd.m).* + opcode.nmd.d)));
}

// The 4-bit displacement is multiplied by two after zero-extension, enabling a range up to +30 bytes to be specified.
// The loaded data is sign-extended to 32 bit before being stored in the destination register.
fn movw_at_disp_Rm_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = @bitCast(sign_extension_u16(cpu.read16(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 1))));
}

fn movl_at_dispRm_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = cpu.read32(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 2));
}

// Transfers the source operand to the destination. The 4-bit displacement is multiplied by four after zero-extension, enabling a range up to +60 bytes to be specified.
fn movl_at_disp_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.read32(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 2));
}

// The 4-bit displacement is only zero-extended, so a range up to +15 bytes can be specified.
fn movb_R0_at_dispRm(cpu: *SH4, opcode: Instr) void {
    cpu.write8(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d)), @truncate(cpu.R(0).*));
}
// The 4-bit displacement is multiplied by two after zero-extension, enabling a range up to +30 bytes to be specified.
fn movw_R0_at_dispRm(cpu: *SH4, opcode: Instr) void {
    cpu.write16(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 1), @truncate(cpu.R(0).*));
}

// Transfers the source operand to the destination.
// The 4-bit displacement is multiplied by four after zero-extension, enabling a range up to +60 bytes to be specified.
fn movl_Rm_atdispRn(cpu: *SH4, opcode: Instr) void {
    const d = zero_extend(opcode.nmd.d) << 2;
    cpu.write32(cpu.R(opcode.nmd.n).* +% d, cpu.R(opcode.nmd.m).*);
}

fn movb_atR0Rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(cpu.read8(cpu.R(opcode.nmd.m).* +% cpu.R(0).*)));
}

fn movw_atR0Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(cpu.read16(cpu.R(opcode.nmd.m).* +% cpu.R(0).*)));
}

fn movl_atR0Rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.read32(cpu.R(opcode.nmd.m).* +% cpu.R(0).*);
}

fn movb_Rm_atR0Rn(cpu: *SH4, opcode: Instr) void {
    cpu.write8(cpu.R(opcode.nmd.n).* +% cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
}
fn movw_Rm_atR0Rn(cpu: *SH4, opcode: Instr) void {
    cpu.write16(cpu.R(opcode.nmd.n).* +% cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
}
fn movl_Rm_atR0Rn(cpu: *SH4, opcode: Instr) void {
    cpu.write32(cpu.R(opcode.nmd.n).* +% cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
}

fn movb_atdisp_GBR_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = @bitCast(sign_extension_u8(cpu.read8(cpu.gbr + opcode.nd8.d)));
}
fn movw_atdisp_GBR_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = @bitCast(sign_extension_u16(cpu.read16(cpu.gbr + (opcode.nd8.d << 1))));
}
fn movl_atdisp_GBR_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = cpu.read32(cpu.gbr + (opcode.nd8.d << 2));
}

fn movb_R0_atdisp_GBR(cpu: *SH4, opcode: Instr) void {
    cpu.write8(cpu.gbr + opcode.nd8.d, @truncate(cpu.R(0).*));
}
fn movw_R0_atdisp_GBR(cpu: *SH4, opcode: Instr) void {
    cpu.write16(cpu.gbr + (opcode.nd8.d << 1), @truncate(cpu.R(0).*));
}
fn movl_R0_atdisp_GBR(cpu: *SH4, opcode: Instr) void {
    cpu.write32(cpu.gbr + (opcode.nd8.d << 2), cpu.R(0).*);
}

fn movt_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = if (cpu.sr.t) 1 else 0;
}

// Swaps the upper and lower parts of the contents of general register Rm and stores the result in Rn.
// The 8 bits from bit 15 to bit 8 of Rm are swapped with the 8 bits from bit 7 to bit 0.
// The upper 16 bits of Rm are transferred directly to the upper 16 bits of Rn.
fn swapb(cpu: *SH4, opcode: Instr) void {
    const val = cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = (val & 0xFFFF0000) | (val & 0x0000FF00) >> 8 | (val & 0x000000FF) << 8;
}
fn swapw(cpu: *SH4, opcode: Instr) void {
    const val = cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = val << 16 | val >> 16;
}

test "swapb" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0xAABBCCDD;
    swapb(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0xAABBDDCC);
}

test "swapw" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0xAABBCCDD;
    swapw(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0xCCDDAABB);
}

fn xtrct_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = (cpu.R(opcode.nmd.m).* << 16) & 0xFFFF0000 | (cpu.R(opcode.nmd.m).* >> 16) & 0x0000FFFF;
}

fn add_rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* +%= cpu.R(opcode.nmd.m).*;
}

fn add_imm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nd8.n).* = @bitCast(@as(i32, @bitCast(cpu.R(opcode.nd8.n).*)) +% sign_extension_u8(opcode.nd8.d));
}

test "add imm rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0;
    add_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 1 } });
    try std.testing.expect(cpu.R(0).* == 1);

    cpu.R(0).* = 0;
    add_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0xFF } });
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFF);

    cpu.R(0).* = 0xFFFFFFFF;
    add_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0xFF } });
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFE);

    cpu.R(0).* = 0xFFFFFFFF;
    add_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 1 } });
    try std.testing.expect(cpu.R(0).* == 0);

    cpu.R(0).* = 0x12345678;
    add_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0 } });
    try std.testing.expect(cpu.R(0).* == 0x12345678);

    cpu.R(0).* = 0x12345678;
    add_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 1 } });
    try std.testing.expect(cpu.R(0).* == 0x12345679);

    cpu.R(0).* = 0x12345678;
    add_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0xFF } });
    try std.testing.expect(cpu.R(0).* == 0x12345677);
}

fn addc_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const prev_Rn = cpu.R(opcode.nmd.n).*;
    const sum = prev_Rn +% cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = sum +% (if (cpu.sr.t) @as(u32, @intCast(1)) else 0);
    cpu.sr.t = (prev_Rn > sum);
    if (sum > cpu.R(opcode.nmd.n).*)
        cpu.sr.t = true;
}

fn addv_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

// Compares general register R0 and the sign-extended 8-bit immediate data and sets the T bit if the values are equal.
// If they are not equal the T bit is cleared. The contents of R0 are not changed.
fn cmpeq_imm_r0(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (cpu.R(0).* == @as(u32, @bitCast(sign_extension_u8(opcode.nd8.d))));
}
fn cmpeq_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (cpu.R(opcode.nmd.n).* == cpu.R(opcode.nmd.m).*);
}
// The values for the comparison are interpreted as unsigned integer values
fn cmphs_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (cpu.R(opcode.nmd.n).* >= cpu.R(opcode.nmd.m).*);
}
fn cmphi_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (cpu.R(opcode.nmd.n).* > cpu.R(opcode.nmd.m).*);
}
// The values for the comparison are interpreted as signed integer values.
fn cmpge_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (as_i32(cpu.R(opcode.nmd.n).*) >= as_i32(cpu.R(opcode.nmd.m).*));
}
fn cmpgt_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (as_i32(cpu.R(opcode.nmd.n).*) > as_i32(cpu.R(opcode.nmd.m).*));
}

// The value in Rn for the comparison is interpreted as signed integer.
fn cmppl_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (as_i32(cpu.R(opcode.nmd.n).*) > 0);
}
fn cmppz_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (as_i32(cpu.R(opcode.nmd.n).*) >= 0);
}
// Compares general registers Rn and Rm, and sets the T bit if any of the 4 bytes in Rn are equal to the corresponding byte in Rm.
fn cmpstr_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const l = cpu.R(opcode.nmd.n).*;
    const r = cpu.R(opcode.nmd.m).*;
    cpu.sr.t = (l & 0xFF000000 == r & 0xFF000000) or (l & 0x00FF0000 == r & 0x00FF0000) or (l & 0x0000FF00 == r & 0x0000FF00) or (l & 0x000000FF == r & 0x000000FF);
}
// Performs initial settings for signed division.
// This instruction is followed by a DIV1 instruction that executes 1-digit division, for example, and repeated division steps are executed to find the quotient.
fn div0s_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.q = (0x80000000 & cpu.R(opcode.nmd.n).*) != 0;
    cpu.sr.m = (0x80000000 & cpu.R(opcode.nmd.m).*) != 0;
    cpu.sr.t = cpu.sr.q != cpu.sr.m;
}

fn div0u_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
// Performs initial settings for unsigned division.
// This instruction is followed by a DIV1 instruction that executes 1-digit division, for example,
// and repeated division steps are executed to find the quotient.
fn div0u(cpu: *SH4, _: Instr) void {
    cpu.sr.m = false;
    cpu.sr.q = false;
    cpu.sr.t = false;
}
// Performs 1-digit division (1-step division) of the 32-bit contents of general register Rn (dividend) by the contents of Rm (divisor)
fn div1(cpu: *SH4, opcode: Instr) void {
    const pRn = cpu.R(opcode.nmd.n);

    const prev_q = cpu.sr.q;
    var Q = (0x80000000 & cpu.R(opcode.nmd.n).*) != 0;

    const Rm = cpu.R(opcode.nmd.m).*;

    pRn.* <<= 1;
    pRn.* |= if (cpu.sr.t) 1 else 0;

    const prev_Rn = pRn.*;

    if (!prev_q) {
        if (!cpu.sr.m) {
            pRn.* -%= Rm;
            Q = Q != (pRn.* > prev_Rn); // XOR
        } else {
            pRn.* +%= Rm;
            Q = !Q != (pRn.* < prev_Rn);
        }
    } else {
        if (!cpu.sr.m) {
            pRn.* +%= Rm;
            Q = Q != (pRn.* < prev_Rn);
        } else {
            pRn.* -%= Rm;
            Q = !Q != (pRn.* > prev_Rn);
        }
    }
    cpu.sr.q = Q;
    cpu.sr.t = (Q == cpu.sr.m);
}

test "div1" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0b00111110111110001001111010110000;
    cpu.R(1).* = 0b00000011100100011000110101000110;

    div0u(&cpu, .{ .value = 0b0000000000011001 });
    try std.testing.expect(!cpu.sr.m);
    try std.testing.expect(!cpu.sr.q);
    try std.testing.expect(!cpu.sr.t);
    div1(&cpu, .{ .value = 0b0011_0000_0001_0100 });
    try std.testing.expect(cpu.R(0).* == 0b01111101111100010011110101100000 - 0b00000011100100011000110101000110);
    try std.testing.expect(cpu.sr.t);
}

test "div1 r1 (32 bits) / r0 (16 bits) = r1 (16 bits)" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    const dividend = 0b00111110111110001001111010110000;
    const divisor = 0b00000000000000000100110001100010;

    cpu.R(0).* = divisor;
    cpu.R(1).* = dividend;

    shll16(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    div0u(&cpu, .{ .value = 0b0000000000011001 });

    for (0..16) |_| {
        div1(&cpu, .{ .nmd = .{ ._ = 0b0011, .n = 1, .m = 0, .d = 0b0100 } });
    }
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = 0b0100, .n = 1, .m = 0b0010, .d = 0b0100 } });
    extuw_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 1, .d = undefined } });

    try std.testing.expect(cpu.R(1).* == dividend / divisor);
}

test "div1 r3:r1 (64 bits) / r4 (32 bits) = r1 (32 bits)  (unsigned)" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    // Example from KallistiOS

    const dividend: u64 = 0x1743D174;

    const dividend_low: u32 = @truncate(dividend);
    const dividend_high: u32 = @truncate(dividend >> 32);
    const divisor: u32 = 0x0000000A;

    cpu.R(1).* = dividend_low;
    cpu.R(3).* = dividend_high;
    cpu.R(4).* = divisor;

    div0u(&cpu, .{ .value = 0b0000000000011001 });

    for (0..32) |_| {
        rotcl_Rn(&cpu, .{ .nmd = .{ ._ = 0b0100, .n = 1, .m = 0b0010, .d = 0b0100 } }); // rotcl R1
        div1(&cpu, .{ .nmd = .{ ._ = 0b0011, .n = 3, .m = 4, .d = 0b0100 } }); // div1 R4,R3
    }
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = 0b0100, .n = 1, .m = 0b0010, .d = 0b0100 } }); // rotcl R1

    try std.testing.expect(cpu.R(1).* == @as(u32, @truncate(dividend / divisor)));
}

// FIXME: These are not tested at all.
fn dmulsl_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const r: u64 = @bitCast(@as(i64, cpu.R(opcode.nmd.n).*) * @as(i64, cpu.R(opcode.nmd.m).*));
    cpu.mach = @truncate(r >> 32);
    cpu.macl = @truncate(r);
}
fn dmulul_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const r = @as(u64, cpu.R(opcode.nmd.n).*) * @as(u64, cpu.R(opcode.nmd.m).*);
    cpu.mach = @truncate(r >> 32);
    cpu.macl = @truncate(r);
}

// Decrements the contents of general register Rn by 1 and compares the result with zero.
// If the result is zero, the T bit is set to 1. If the result is nonzero, the T bit is cleared to 0.
fn dt_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -%= 1;
    cpu.sr.t = (cpu.R(opcode.nmd.n).* == 0);
}

fn extsb_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(@truncate(cpu.R(opcode.nmd.m).*)));
}
fn extsw_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(@truncate(cpu.R(opcode.nmd.m).*)));
}
// Zero-extends the contents of general register Rm and stores the result in Rn. 0 is transferred to Rn bits 8 to 31.
fn extub_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.m).* & 0xFF;
}
fn extuw_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.m).* & 0xFFFF;
}

fn mull_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.macl = cpu.R(opcode.nmd.n).* *% cpu.R(opcode.nmd.m).*;
}
// Performs 16-bit multiplication of the contents of general registers Rn and Rm, and stores the 32-bit result in the MACL register.
// The multiplication is performed as a signed arithmetic operation.
fn mulsw_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const n: i16 = @bitCast(@as(u16, @truncate(cpu.R(opcode.nmd.n).*)));
    const m: i16 = @bitCast(@as(u16, @truncate(cpu.R(opcode.nmd.m).*)));
    cpu.macl = @bitCast(@as(i32, @intCast(n)) * @as(i32, @intCast(m)));
}

// Performs 16-bit multiplication of the contents of general registers Rn and Rm, and stores the 32-bit result in the MACL register.
// The multiplication is performed as an unsigned arithmetic operation. The contents of MACH are not changed
fn muluw_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.macl = @as(u32, @intCast(@as(u16, @truncate(cpu.R(opcode.nmd.n).*)))) * @as(u32, @intCast(@as(u16, @truncate(cpu.R(opcode.nmd.m).*))));
}

fn neg_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = 0 -% cpu.R(opcode.nmd.m).*;
}

test "neg Rm,Rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 1;
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == -1);

    cpu.R(0).* = @bitCast(@as(i32, -1));
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == 1);

    cpu.R(0).* = 1337;
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == -1337);

    cpu.R(0).* = @bitCast(@as(i32, -1337));
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == 1337);

    cpu.R(0).* = 0x180;
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == -0x180);
}

fn negc_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const tmp = 0 -% cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = tmp -% (if (cpu.sr.t) @as(u32, 1) else 0);
    cpu.sr.t = (0 < tmp);
    if (tmp < cpu.R(opcode.nmd.n).*)
        cpu.sr.t = true;
}

test "negc Rm,Rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    // Examples from the manual

    cpu.R(1).* = 1;
    cpu.sr.t = false;
    negc_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 1, .d = undefined } });
    try std.testing.expect(cpu.R(1).* == 0xFFFFFFFF);
    try std.testing.expect(cpu.sr.t);

    cpu.R(0).* = 0;
    cpu.sr.t = true;
    negc_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = 0, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFF);
    try std.testing.expect(cpu.sr.t);

    cpu.R(1).* = @bitCast(@as(i32, -1));
    cpu.sr.t = true;
    negc_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = 1, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0);
    try std.testing.expect(cpu.sr.t);

    cpu.R(1).* = @bitCast(@as(i32, -1));
    cpu.sr.t = false;
    negc_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = 1, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 1);
    try std.testing.expect(cpu.sr.t);
}

fn sub_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -%= cpu.R(opcode.nmd.m).*;
}
fn subc_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const prev_Rn = cpu.R(opcode.nmd.n).*;
    const diff = prev_Rn -% cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = diff -% (if (cpu.sr.t) @as(u32, @intCast(1)) else 0);
    cpu.sr.t = (prev_Rn < diff);
    if (diff < cpu.R(opcode.nmd.n).*)
        cpu.sr.t = true;
}

fn and_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* &= cpu.R(opcode.nmd.m).*;
}
fn and_imm_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* &= zero_extend(opcode.nd8.d);
}

fn not_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = ~cpu.R(opcode.nmd.m).*;
}
fn or_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* |= cpu.R(opcode.nmd.m).*;
}
fn or_imm_r0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* |= zero_extend(opcode.nd8.d);
}

fn tasb_at_Rn(cpu: *SH4, opcode: Instr) void {
    // Reads byte data from the address specified by general register Rn, and sets the T bit to 1 if the data is 0, or clears the T bit to 0 if the data is not 0.
    // Then, data bit 7 is set to 1, and the data is written to the address specified by Rn.
    const tmp = cpu.read8(cpu.R(opcode.nmd.n).*);
    cpu.sr.t = (tmp == 0);
    cpu.write8(cpu.R(opcode.nmd.n).*, tmp | 0x80);
}
fn tst_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (cpu.R(opcode.nmd.n).* & cpu.R(opcode.nmd.m).*) == 0;
}
fn tst_imm_r0(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (cpu.R(0).* & zero_extend(opcode.nd8.d)) == 0;
}

fn xorRmRn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* ^= cpu.R(opcode.nmd.m).*;
}
fn xorImmR0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* ^= zero_extend(opcode.nd8.d);
}

fn rotcl_Rn(cpu: *SH4, opcode: Instr) void {
    const tmp = !((cpu.R(opcode.nmd.n).* & 0x80000000) == 0);
    cpu.R(opcode.nmd.n).* <<= 1;
    if (cpu.sr.t) {
        cpu.R(opcode.nmd.n).* |= 0x00000001;
    } else {
        cpu.R(opcode.nmd.n).* &= 0xFFFFFFFE;
    }
    cpu.sr.t = tmp;
}
test "rotcl" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0;
    cpu.sr.t = false;
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0;
    cpu.sr.t = true;
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 1);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b01;
    cpu.sr.t = false;
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b10);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b01;
    cpu.sr.t = true;
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b11);
    try std.testing.expect(!cpu.sr.t);
}

fn rotcr_Rn(cpu: *SH4, opcode: Instr) void {
    const tmp = (cpu.R(opcode.nmd.n).* & 0x00000001) == 1;
    cpu.R(opcode.nmd.n).* >>= 1;
    if (cpu.sr.t) {
        cpu.R(opcode.nmd.n).* |= 0x80000000;
    } else {
        cpu.R(opcode.nmd.n).* &= 0x7FFFFFFF;
    }
    cpu.sr.t = tmp;
}

test "rotcr" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0;
    cpu.sr.t = false;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0;
    cpu.sr.t = true;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b1000_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b01;
    cpu.sr.t = false;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0);
    try std.testing.expect(cpu.sr.t);

    cpu.R(0).* = 0b01;
    cpu.sr.t = true;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b1000_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(cpu.sr.t);

    cpu.R(0).* = 0b1000_0000_0000_0000_0000_0000_0000_0000;
    cpu.sr.t = false;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b0100_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b1000_0000_0000_0000_0000_0000_0000_0000;
    cpu.sr.t = true;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b1100_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b1000_0000_0000_0000_0000_0000_0000_0001;
    cpu.sr.t = false;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b0100_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(cpu.sr.t);

    cpu.R(0).* = 0b1000_0000_0000_0000_0000_0000_0000_0001;
    cpu.sr.t = true;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b1100_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(cpu.sr.t);
}

fn rotl_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 0x80000000) == 1);
    cpu.R(opcode.nmd.n).* = std.math.rotl(u32, cpu.R(opcode.nmd.n).*, 1);
}
fn rotr_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 1) == 1);
    cpu.R(opcode.nmd.n).* = std.math.rotr(u32, cpu.R(opcode.nmd.n).*, 1);
}
// Arithmetically shifts the contents of general register Rn. General register Rm specifies the shift direction and the number of bits to be shifted.
fn shad_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const sign = cpu.R(opcode.nmd.m).* & 0x80000000;

    if (sign == 0) {
        cpu.R(opcode.nmd.n).* <<= @intCast(cpu.R(opcode.nmd.m).* & 0x1F);
    } else if (cpu.R(opcode.nmd.m).* & 0x1F == 0) {
        if (cpu.R(opcode.nmd.n).* & 0x80000000 == 0) {
            cpu.R(opcode.nmd.n).* = 0;
        } else {
            cpu.R(opcode.nmd.n).* = 0xFFFFFFFF;
        }
    } else {
        cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* >> @intCast((~cpu.R(opcode.nmd.m).* & 0x1F) + 1);
    }
}
fn shal_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
// Arithmetically shifts the contents of general register Rn one bit to the right and stores the result in Rn. The bit shifted out of the operand is transferred to the T bit.
fn shar_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 1) == 1);

    const tmp = !((cpu.R(opcode.nmd.n).* & 0x80000000) == 0);

    cpu.R(opcode.nmd.n).* >>= 1;

    if (tmp) {
        cpu.R(opcode.nmd.n).* |= 0x80000000;
    } else {
        cpu.R(opcode.nmd.n).* &= 0x7FFFFFFF;
    }
}

fn shld_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const sign = cpu.R(opcode.nmd.m).* & 0x80000000;
    if (sign == 0) {
        cpu.R(opcode.nmd.n).* <<= @intCast(cpu.R(opcode.nmd.m).* & 0x1F);
    } else if ((cpu.R(opcode.nmd.m).* & 0x1F) == 0) {
        cpu.R(opcode.nmd.n).* = 0;
    } else {
        cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* >> @intCast((~cpu.R(opcode.nmd.m).* & 0x1F) + 1);
    }
}
fn shll(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 0x80000000) == 1);
    cpu.R(opcode.nmd.n).* <<= 1;
}
fn shll2(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* <<= 2;
}
fn shll8(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* <<= 8;
}
fn shll16(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* <<= 16;
}
fn shlr(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 1) == 1);
    cpu.R(opcode.nmd.n).* >>= 1;
}
fn shlr2(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* >>= 2;
}
fn shlr8(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* >>= 8;
}
fn shlr16(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* >>= 16;
}

inline fn d8_label(cpu: *SH4, opcode: Instr) void {
    const displacement = sign_extension_u8(opcode.nd8.d);
    var pc: i32 = @intCast(cpu.pc & 0x1FFFFFFF);
    pc += 4 + (displacement * 2);
    cpu.pc = (cpu.pc & 0xE0000000) | @as(u32, @bitCast(pc & 0x1FFFFFFF));
    // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    cpu.pc -= 2;
}

inline fn d12_label(cpu: *SH4, opcode: Instr) void {
    const displacement = sign_extension_u12(opcode.d12.d);
    var pc: i32 = @intCast(cpu.pc & 0x1FFFFFFF);
    pc += 4 + (displacement * 2);
    cpu.pc = (cpu.pc & 0xE0000000) | @as(u32, @bitCast(pc & 0x1FFFFFFF));
    // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    cpu.pc -= 2;
}

inline fn execute_delay_slot(cpu: *SH4, addr: addr_t) void {
    // TODO: If the instruction at addr is a branch instruction, raise a Slot illegal instruction exception

    // FIXME: If the delayed instruction references PC in any way (e.g. @(disp,PC)), it will be wrong because
    //        we always substract 2 to compensate the automatic increment. Hence this weird workaround.
    //        The instructions referencing PC are probably all sources of Slot Illegal instruction exceptions,
    //        but just to be sure...
    cpu.pc += 2;

    cpu._execute(addr);

    cpu.pc -= 2;
}

fn bf_label(cpu: *SH4, opcode: Instr) void {
    if (!cpu.sr.t) {
        d8_label(cpu, opcode);
    }
}
fn bfs_label(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    if (!cpu.sr.t) {
        d8_label(cpu, opcode);
    } else { // Don't execute the delay slot twice.
        cpu.pc += 4;
        cpu.pc -= 2; // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    }
    execute_delay_slot(cpu, delay_slot);
}
fn bt_label(cpu: *SH4, opcode: Instr) void {
    if (cpu.sr.t) {
        d8_label(cpu, opcode);
    }
}
fn bts_label(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    if (cpu.sr.t) {
        d8_label(cpu, opcode);
    } else { // Don't execute the delay slot twice.
        cpu.pc += 4;
        cpu.pc -= 2; // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    }
    execute_delay_slot(cpu, delay_slot);
}
fn bra_label(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    d12_label(cpu, opcode);
    execute_delay_slot(cpu, delay_slot);
}
fn braf_Rn(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.pc +%= 4 + cpu.R(opcode.nmd.n).*;
    cpu.pc -= 2; // execute will allready add +2
    execute_delay_slot(cpu, delay_slot);
}
fn bsr_label(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.pr = cpu.pc + 4;
    d12_label(cpu, opcode);
    execute_delay_slot(cpu, delay_slot);
}
fn bsrf_Rn(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.pr = cpu.pc + 4;
    // Note: The Boot ROM seem to intentionally wrap around the address.
    cpu.pc +%= 4 + cpu.R(opcode.nmd.n).*;
    cpu.pc -= 2; // execute will allready add +2
    execute_delay_slot(cpu, delay_slot);
}
fn jmp_atRn(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.pc = cpu.R(opcode.nmd.n).*;
    cpu.pc -= 2; // -2 to account for the standard +2
    execute_delay_slot(cpu, delay_slot);
}
// Makes a delayed branch to the subroutine procedure at the specified address after execution of the following instruction.
// Return address (PC + 4) is saved in PR, and a branch is made to the address indicated by general register Rm. JSR is used in combination with RTS for subroutine procedure calls.
// Note: As this is a delayed branch instruction, the instruction following this instruction is executed before the branch destination instruction.
// Interrupts are not accepted between this instruction and the following instruction.
// If the following instruction is a branch instruction, it is identified as a slot illegal instruction.
fn jsr_Rn(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.pr = cpu.pc + 4;
    cpu.pc = cpu.R(opcode.nmd.n).*;
    cpu.pc -= 2; // -2 to account for the standard +2
    execute_delay_slot(cpu, delay_slot);
}
fn rts(cpu: *SH4, _: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.pc = cpu.pr;
    cpu.pc -= 2; // execute will add +2
    execute_delay_slot(cpu, delay_slot);
}

fn clrmac(cpu: *SH4, _: Instr) void {
    cpu.mach = 0;
    cpu.macl = 0;
}
fn clrs(cpu: *SH4, _: Instr) void {
    cpu.sr.s = false;
}
fn clrt(cpu: *SH4, _: Instr) void {
    cpu.sr.t = false;
}

fn ldc_Rn_SR(cpu: *SH4, opcode: Instr) void {
    cpu.set_sr(@bitCast(cpu.R(opcode.nmd.n).*));
}
fn ldcl_at_Rn_inc_SR(cpu: *SH4, opcode: Instr) void {
    cpu.set_sr(@bitCast(cpu.read32(cpu.R(opcode.nmd.n).*)));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldc_Rn_GBR(cpu: *SH4, opcode: Instr) void {
    cpu.gbr = cpu.R(opcode.nmd.n).*;
}
fn ldcl_at_Rn_inc_GBR(cpu: *SH4, opcode: Instr) void {
    cpu.gbr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldc_Rn_VBR(cpu: *SH4, opcode: Instr) void {
    cpu.vbr = cpu.R(opcode.nmd.n).*;
}
fn ldcl_at_Rn_inc_VBR(cpu: *SH4, opcode: Instr) void {
    cpu.vbr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldc_Rn_SSR(cpu: *SH4, opcode: Instr) void {
    cpu.ssr = cpu.R(opcode.nmd.n).*;
}
fn ldcl_at_Rn_inc_SSR(cpu: *SH4, opcode: Instr) void {
    cpu.ssr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldc_Rn_SPC(cpu: *SH4, opcode: Instr) void {
    cpu.spc = cpu.R(opcode.nmd.n).*;
}
fn ldcl_at_Rn_inc_SPC(cpu: *SH4, opcode: Instr) void {
    cpu.spc = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldc_Rn_DBR(cpu: *SH4, opcode: Instr) void {
    cpu.dbr = cpu.R(opcode.nmd.n).*;
}
fn ldcl_at_Rn_inc_DBR(cpu: *SH4, opcode: Instr) void {
    cpu.dbr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldc_Rn_Rm_BANK(cpu: *SH4, opcode: Instr) void {
    cpu.r_bank[opcode.nmd.m & 0b0111] = cpu.R(opcode.nmd.n).*;
}
fn ldcl_at_Rn_inc_Rm_BANK(cpu: *SH4, opcode: Instr) void {
    cpu.r_bank[opcode.nmd.m & 0b0111] = cpu.read32(cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
fn lds_Rn_MACH(cpu: *SH4, opcode: Instr) void {
    cpu.mach = cpu.R(opcode.nmd.n).*;
}
fn ldsl_at_Rn_inc_MACH(cpu: *SH4, opcode: Instr) void {
    cpu.mach = cpu.read32(cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
fn lds_Rn_MACL(cpu: *SH4, opcode: Instr) void {
    cpu.macl = cpu.R(opcode.nmd.n).*;
}
fn ldsl_at_Rn_inc_MACL(cpu: *SH4, opcode: Instr) void {
    cpu.macl = cpu.read32(cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
fn lds_Rn_PR(cpu: *SH4, opcode: Instr) void {
    cpu.pr = cpu.R(opcode.nmd.n).*;
}
fn ldsl_atRn_inc_PR(cpu: *SH4, opcode: Instr) void {
    cpu.pr = cpu.read32(cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
fn movcal_R0_atRn(cpu: *SH4, opcode: Instr) void {
    // TODO: This instruction deals with cache, no idea if this is important to get right.

    // Stores the contents of general register R0 in the memory location indicated by effective address Rn.
    // If write-back is selected for the accessed memory, and a cache miss occurs, the cache block will be allocated but an
    // R0 data write will be performed to that cache block without performing a block read. Other cache block contents are undefined.

    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.R(0).*);
}
fn lds_Rn_FPSCR(cpu: *SH4, opcode: Instr) void {
    cpu.fpscr = @bitCast(cpu.R(opcode.nmd.n).* & 0x003FFFFF);
}
fn sts_FPSCR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @as(u32, @bitCast(cpu.fpscr)) & 0x003FFFFF;
}
fn ldsl_at_Rn_inc_FPSCR(cpu: *SH4, opcode: Instr) void {
    cpu.fpscr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*) & 0x003FFFFF);
    cpu.R(opcode.nmd.n).* += 4;
}
fn stsl_FPSCR_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, @as(u32, @bitCast(cpu.fpscr)) & 0x003FFFFF);
}
fn lds_Rn_FPUL(cpu: *SH4, opcode: Instr) void {
    cpu.fpul = cpu.R(opcode.nmd.n).*;
}
fn sts_FPUL_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.fpul;
}
fn ldsl_at_Rn_inc_FPUL(cpu: *SH4, opcode: Instr) void {
    cpu.fpul = cpu.read32(cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
fn stsl_FPUL_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.fpul);
}

// Inverts the FR bit in floating-point register FPSCR.
fn frchg(cpu: *SH4, _: Instr) void {
    std.debug.assert(cpu.fpscr.pr == 0);
    cpu.fpscr.fr +%= 1;
}
fn fschg(cpu: *SH4, _: Instr) void {
    std.debug.assert(cpu.fpscr.pr == 0);
    cpu.fpscr.sz +%= 1;
}

fn ocbi_atRn(_: *SH4, _: Instr) void {
    const static = struct {
        var once = true;
    };
    if (static.once) {
        static.once = false;
        sh4_log.warn("Note: ocbi @Rn not implemented", .{});
    }
}

fn ocbp_atRn(_: *SH4, _: Instr) void {
    // Accesses data using the contents indicated by effective address Rn.
    // If the cache is hit and there is unwritten information (U bit = 1),
    // the corresponding cache block is written back to external memory and
    // that block is invalidated (the V bit is cleared to 0).
    // If there is no unwritten information (U bit = 0), the block is simply
    // invalidated. No operation is performed in the case of a cache miss
    // or an access to a non-cache area.

    const static = struct {
        var once = true;
    };
    if (static.once) {
        static.once = false;
        sh4_log.warn("Note: obcp @Rn not implemented", .{});
    }
}

fn ocbwb_atRn(_: *SH4, _: Instr) void {
    const static = struct {
        var once = true;
    };
    if (static.once) {
        static.once = false;
        sh4_log.warn("Note: ocbwb @Rn not implemented", .{});
    }
}

// Reads a 32-byte data block starting at a 32-byte boundary into the operand cache.
// The lower 5 bits of the address specified by Rn are masked to zero.
// This instruction is also used to trigger a Store Queue write-back operation if the specified address points to the Store Queue area.
fn pref_atRn(cpu: *SH4, opcode: Instr) void {
    const addr = cpu.R(opcode.nmd.n).*;
    if (addr & 0xEC000000 == 0xE0000000) {
        if (cpu.read_p4_register(mmu.MMUCR, .MMUCR).at == 1) {
            sh4_log.err(termcolor.yellow("  MMU ON: Not implemented"), .{});
            @panic("pref @Rn with MMU ON: Not implemented");
        } else {
            const sq_addr: StoreQueueAddr = @bitCast(addr);
            std.debug.assert(sq_addr.spec == 0b111000);
            //               The full address also includes the sq bit.
            const ext_addr = (addr & 0x03FFFFE0) | (((cpu.read_p4_register(u32, if (sq_addr.sq == 0) .QACR0 else .QACR1) & 0b11100) << 24));
            std.log.debug("pref @R{d}={X:0>8} : Store queue write back to {X:0>8}", .{ opcode.nmd.n, addr, ext_addr });
            inline for (0..8) |i| {
                cpu.write32(@intCast(ext_addr + 4 * i), cpu.store_queues[sq_addr.sq][i]);
            }
        }
    } else {
        const static = struct {
            var once = true;
        };
        if (static.once) {
            static.once = false;
            sh4_log.warn("Note: pref @Rn not implemented outside of store queue operation.", .{});
        }
    }
}
// Returns from an exception or interrupt handling routine by restoring the PC and SR values. Delayed jump.
fn rte(cpu: *SH4, _: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.set_sr(@bitCast(cpu.ssr));
    cpu.pc = cpu.spc;
    cpu.pc -= 2; // Execute will add 2
    execute_delay_slot(cpu, delay_slot);

    //cpu.debug_trace = false;
}
fn sets(cpu: *SH4, _: Instr) void {
    cpu.sr.s = true;
}
fn sett(cpu: *SH4, _: Instr) void {
    cpu.sr.t = true;
}
fn sleep(cpu: *SH4, _: Instr) void {
    const standby_control_register = cpu.read_p4_register(u8, .STBCR);
    if ((standby_control_register & 0b1000_0000) == 0b1000_0000) {
        const standby_control_register_2 = cpu.read_p4_register(u8, .STBCR2);
        if ((standby_control_register_2 & 0b1000_0000) == 0b1000_0000) {
            cpu.execution_state = .DeepSleep;
        } else {
            cpu.execution_state = .Standby;
        }
    } else {
        cpu.execution_state = .Sleep;
    }
    std.debug.print("\u{001B}[33mSleep State: .{s}\u{001B}[0m\n", .{@tagName(cpu.execution_state)});
}

fn stc_SR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(cpu.sr);
}
fn stcl_SR_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, @bitCast(cpu.sr));
}
fn stc_TBR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.tbr;
}
fn stc_GBR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.gbr;
}
fn stcl_GBR_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.gbr);
}
fn stc_VBR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.vbr;
}
fn stcl_VBR_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.vbr);
}
fn stc_SGR_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.sgr;
}
fn stcl_SGR_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.sgr);
}
fn stc_SSR_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.ssr;
}
fn stcl_SSR_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.ssr);
}
fn stc_SPC_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.spc;
}
fn stcl_SPC_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.spc);
}
fn stc_DBR_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.dbr;
}
fn stcl_DBR_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.dbr);
}
fn stc_Rm_BANK_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.r_bank[opcode.nmd.m & 0b0111];
}
fn stcl_Rm_BANK_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.r_bank[opcode.nmd.m & 0b0111]);
}
fn sts_MACH_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.mach;
}
fn stsl_MACH_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.mach);
}
fn sts_MACL_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.macl;
}
fn stsl_MACL_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.macl);
}
fn sts_PR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.pr;
}
fn stsl_PR_atRn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.pr);
}

fn trapa_imm(cpu: *SH4, opcode: Instr) void {
    // Hijack this instruction for debugging purposes.
    std.debug.print("TRAPA #0x{X}\n", .{opcode.nd8.d});

    if (cpu.on_trapa != null) {
        cpu.on_trapa.?();
    }
}

fn fmov_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = cpu.FR(opcode.nmd.m).*;
    } else {
        if (opcode.nmd.n & 0x1 == 0 and opcode.nmd.m & 0x1 == 0) {
            // fmov DRm,DRn
            cpu.DR(opcode.nmd.n >> 1).* = cpu.DR(opcode.nmd.m >> 1).*;
        } else if (opcode.nmd.n & 0x1 == 1 and opcode.nmd.m & 0x1 == 0) {
            // fmov DRm,XDn
            cpu.XD(opcode.nmd.n >> 1).* = cpu.DR(opcode.nmd.m >> 1).*;
        } else if (opcode.nmd.n & 0x1 == 0 and opcode.nmd.m & 0x1 == 1) {
            // fmov XDm,DRn
            cpu.DR(opcode.nmd.n >> 1).* = cpu.XD(opcode.nmd.m >> 1).*;
        } else if (opcode.nmd.n & 0x1 == 1 and opcode.nmd.m & 0x1 == 1) {
            // fmov XDm,XDn
            cpu.XD(opcode.nmd.n >> 1).* = cpu.XD(opcode.nmd.m >> 1).*;
        }
    }
}
fn fmovs_atRm_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @bitCast(cpu.read32(cpu.R(opcode.nmd.m).*));
    } else {
        if (opcode.nmd.n & 0x1 == 0) {
            cpu.DR(opcode.nmd.n >> 1).* = @bitCast(cpu.read64(cpu.R(opcode.nmd.m).*));
        } else {
            cpu.XD(opcode.nmd.n >> 1).* = @bitCast(cpu.read64(cpu.R(opcode.nmd.m).*));
        }
    }
}
fn fmovs_FRm_atRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.write32(cpu.R(opcode.nmd.n).*, @bitCast(cpu.FR(opcode.nmd.m).*));
    } else {
        if (opcode.nmd.m & 0x1 == 0) {
            cpu.write64(cpu.R(opcode.nmd.n).*, @bitCast(cpu.DR(opcode.nmd.m >> 1).*));
        } else {
            cpu.write64(cpu.R(opcode.nmd.n).*, @bitCast(cpu.XD(opcode.nmd.m >> 1).*));
        }
    }
}
fn fmovs_at_Rm_inc_FRn(cpu: *SH4, opcode: Instr) void {
    // Single-precision
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @bitCast(cpu.read32(cpu.R(opcode.nmd.m).*));
        cpu.R(opcode.nmd.m).* += 4;
    } else { // Double-precision
        if (opcode.nmd.n & 0x1 == 0) {
            cpu.DR(opcode.nmd.n >> 1).* = @bitCast(cpu.read64(cpu.R(opcode.nmd.m).*));
        } else {
            cpu.XD(opcode.nmd.n >> 1).* = @bitCast(cpu.read64(cpu.R(opcode.nmd.m).*));
        }
        cpu.R(opcode.nmd.m).* += 8;
    }
}
fn fmovs_FRm_at_dec_Rn(cpu: *SH4, opcode: Instr) void {
    // Single-precision
    if (cpu.fpscr.sz == 0) {
        cpu.R(opcode.nmd.n).* -= 4;
        cpu.write32(cpu.R(opcode.nmd.n).*, @bitCast(cpu.FR(opcode.nmd.m).*));
    } else { // Double-precision
        cpu.R(opcode.nmd.n).* -= 8;
        if (opcode.nmd.m & 0x1 == 0) {
            // fmov.d	DRm,@-Rn
            cpu.write64(cpu.R(opcode.nmd.n).*, @bitCast(cpu.DR(opcode.nmd.m >> 1).*));
        } else {
            // fmov.d	XDm,@-Rn
            cpu.write64(cpu.R(opcode.nmd.n).*, @bitCast(cpu.XD(opcode.nmd.m >> 1).*));
        }
    }
}
fn fmovs_at_R0_Rm_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @bitCast(cpu.read32(cpu.R(0).* +% cpu.R(opcode.nmd.m).*));
    } else {
        @panic("Unimplemented");
    }
}
fn fmovs_FRm_at_R0_Rn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.write32(cpu.R(0).* +% cpu.R(opcode.nmd.n).*, @bitCast(cpu.FR(opcode.nmd.m).*));
    } else {
        @panic("Unimplemented");
    }
}

fn fldi0_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.pr == 1) @panic("Illegal instruction");
    cpu.FR(opcode.nmd.n).* = 0.0;
}
fn fldi1_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.pr == 1) @panic("Illegal instruction");
    cpu.FR(opcode.nmd.n).* = 1.0;
}

test "fldi1_FRn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.fpscr.pr = 0;
    fldi1_FRn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    std.debug.assert(@as(u32, @bitCast(cpu.FR(0).*)) == 0x3F800000);
}

fn flds_FRn_FPUL(cpu: *SH4, opcode: Instr) void {
    cpu.fpul = @bitCast(cpu.FR(opcode.nmd.n).*);
}

fn fsts_FPUL_FRn(cpu: *SH4, opcode: Instr) void {
    cpu.FR(opcode.nmd.n).* = @bitCast(cpu.fpul);
}
fn fabs_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @abs(cpu.FR(opcode.nmd.n).*);
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* = @abs(cpu.DR(opcode.nmd.n >> 1).*);
    }
}
fn fneg_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = -cpu.FR(opcode.nmd.n).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* = -cpu.DR(opcode.nmd.n >> 1).*;
    }
}

fn fadd_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        // TODO: Handle exceptions
        // if(!cpu.fpscr.dn and (n is denorm or m  is denorm)) ...
        cpu.FR(opcode.nmd.n).* += cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* += cpu.DR(opcode.nmd.m >> 1).*;
    }
}
fn fsub_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* -= cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* -= cpu.DR(opcode.nmd.m >> 1).*;
    }
}
fn fmul_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    // FIXME: There's a lot more to do here.
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* *= cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* *= cpu.DR(opcode.nmd.m >> 1).*;
    }
}
fn fmac_FR0_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    std.debug.assert(cpu.fpscr.pr == 0);
    std.debug.assert(cpu.fpscr.sz == 0);
    cpu.FR(opcode.nmd.n).* += cpu.FR(0).* * cpu.FR(opcode.nmd.m).*;
}
fn fdiv_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    // FIXME: There's a lot more to do here.
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* /= cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* /= cpu.DR(opcode.nmd.m >> 1).*;
    }
}
fn fsqrt_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @sqrt(cpu.FR(opcode.nmd.n).*);
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* = @sqrt(cpu.DR(opcode.nmd.n >> 1).*);
    }
}
fn fcmp_gt_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    // TODO: Special float values checks?
    if (cpu.fpscr.sz == 0) {
        cpu.sr.t = (cpu.FR(opcode.nmd.n).* > cpu.FR(opcode.nmd.m).*);
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.sr.t = (cpu.DR(opcode.nmd.n >> 1).* > cpu.DR(opcode.nmd.m >> 1).*);
    }
}
fn fcmp_eq_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    // TODO: Special float values checks?
    if (cpu.fpscr.sz == 0) {
        cpu.sr.t = (cpu.FR(opcode.nmd.n).* == cpu.FR(opcode.nmd.m).*);
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.sr.t = (cpu.DR(opcode.nmd.n >> 1).* == cpu.DR(opcode.nmd.m >> 1).*);
    }
}
fn float_FPUL_FRn(cpu: *SH4, opcode: Instr) void {
    // FIXME: We're skipping a lot of error checking here.
    // NOTE: Experimentation shows the FPUL is treated as signed, at least here. I don't know if this is ALWAYS the case, or not.
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @floatFromInt(as_i32(cpu.fpul));
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* = @floatFromInt(as_i32(cpu.fpul));
    }
}
fn ftrc_FRn_FPUL(cpu: *SH4, opcode: Instr) void {
    // Converts the single-precision floating-point number in FRm to a 32-bit integer, and stores the result in FPUL.
    // FIXME: We're skipping a lot of error checking here.
    // NOTE: I have no evidence that the conversion should be to a signed integer or not here, however,
    //       it makes sense to be symetrical with float FPUL,FRn, which is signed, I'm pretty sure.
    if (cpu.fpscr.sz == 0) {
        cpu.fpul = @bitCast(std.math.lossyCast(i32, cpu.FR(opcode.nmd.n).*));
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        cpu.fpul = @bitCast(std.math.lossyCast(i32, cpu.DR(opcode.nmd.n >> 1).*));
    }
}

fn fipr_FVm_FVn(cpu: *SH4, opcode: Instr) void {
    // Computes the dot product of FVn and FVm and stores it into Fn+3.
    const n = opcode.nmd.n & 0b1100;
    const m = (opcode.nmd.n << 2) & 0b1100;
    // FIXME: Not accurate to the actual hardware implementation.

    // TODO: Use direct address, unless the compiler can figure it out itself?
    const FVn = @Vector(4, f32){ cpu.FR(n + 0).*, cpu.FR(n + 1).*, cpu.FR(n + 2).*, cpu.FR(n + 3).* };
    const FVm = @Vector(4, f32){ cpu.FR(m + 0).*, cpu.FR(m + 1).*, cpu.FR(m + 2).*, cpu.FR(m + 3).* };
    cpu.FR(n + 3).* = @reduce(.Add, FVn * FVm);
}

fn test_fipr(n: [4]f32, m: [4]f32) !void {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.fpscr.pr = 0;

    for (0..4) |i| {
        cpu.FR(@intCast(i)).* = n[i];
    }

    for (0..4) |i| {
        cpu.FR(@intCast(4 + i)).* = m[i];
    }

    fipr_FVm_FVn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0b0001, .m = undefined, .d = undefined } });

    try std.testing.expect(cpu.FR(3).* == n[0] * m[0] + n[1] * m[1] + n[2] * m[2] + n[3] * m[3]);
}

test "fipr" {
    try test_fipr(.{ 1, 2, 3, 4 }, .{ 4, 3, 2, 1 });
    try test_fipr(.{ 0, 0, 0, 0 }, .{ 0, 0, 0, 0 });
    try test_fipr(.{ 1.5, 2.5, 3.5, 4.5 }, .{ 4.5, 3.5, 2.5, 1.5 });
    try test_fipr(.{ 1, 2, 3, 4 }, .{ 0, 0, 0, 0 });
}

fn ftrv_XMTRX_FVn(cpu: *SH4, opcode: Instr) void {
    std.debug.assert(cpu.fpscr.pr == 0);

    // Note: Doesn't handle exceptions.
    // TODO: Improve Vectorization?

    const n = opcode.nmd.n & 0b1100;
    const FVn: @Vector(4, f32) = @as([*]f32, @ptrCast(cpu.FR(n + 0)))[0..4].*;
    inline for (0..4) |u| {
        const i: u4 = @intCast(u);
        cpu.FR(n + i).* = @reduce(.Add, FVn * @Vector(4, f32){ cpu.XF(i + 0).*, cpu.XF(i + 4).*, cpu.XF(i + 8).*, cpu.XF(i + 12).* });
    }
}

// Column major matrix
fn test_ftrv(v: [4]f32, m: [4 * 4]f32, r: [4]f32) !void {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.fpscr.pr = 0;

    for (0..4) |i| {
        cpu.FR(@intCast(i)).* = v[i];
    }
    for (0..16) |i| {
        cpu.XF(@intCast(i)).* = m[i];
    }

    ftrv_XMTRX_FVn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });

    for (0..4) |i| {
        try std.testing.expect(cpu.FR(@intCast(i)).* == r[i]);
    }
}

test "ftrv XMTRX_FVn" {
    try test_ftrv(.{ 1.0, 2.0, 3.0, 4.0 }, .{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    }, .{ 1.0, 2.0, 3.0, 4.0 });

    try test_ftrv(.{ 1.0, 2.0, 3.0, 4.0 }, .{
        2, 0, 0, 0,
        0, 2, 0, 0,
        0, 0, 2, 0,
        0, 0, 0, 2,
    }, .{ 2.0, 4.0, 6.0, 8.0 });

    try test_ftrv(.{ 1.0, 2.0, 3.0, 4.0 }, .{
        2, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
    }, .{ 2.0, 0.0, 0.0, 0.0 });
}

fn fsrra_FRn(cpu: *SH4, opcode: Instr) void {
    std.debug.assert(cpu.fpscr.pr == 0);
    cpu.FR(opcode.nmd.n).* = 1.0 / @sqrt(cpu.FR(opcode.nmd.n).*);
}
fn fsca_FPUL_DRn(cpu: *SH4, opcode: Instr) void {
    std.debug.assert(cpu.fpscr.pr == 0);
    std.debug.assert(opcode.nmd.n & 1 == 0);

    // TODO: Check implementation.

    const fraction = cpu.fpul & 0x0000_FFFF;
    const angle = 2 * std.math.pi * @as(f32, @floatFromInt(fraction)) / 0x10000;

    cpu.FR(opcode.nmd.n).* = @sin(angle);
    cpu.FR(opcode.nmd.n + 1).* = @cos(angle);
}

const OpcodeDescription = struct {
    code: u16,
    mask: u16,
    fn_: *const fn (*SH4, Instr) void,
    name: []const u8,
    privileged: bool = true,
    issue_cycles: u5 = 1,
    latency_cycles: u5 = 1,
};

fn syscall_sysinfo(cpu: *SH4, _: Instr) void {
    syscall.syscall_sysinfo(cpu._dc.?);
}

fn syscall_romfont(cpu: *SH4, _: Instr) void {
    syscall.syscall_romfont(cpu._dc.?);
}

fn syscall_flashrom(cpu: *SH4, _: Instr) void {
    syscall.syscall_flashrom(cpu._dc.?);
}

fn syscall_gdrom(cpu: *SH4, _: Instr) void {
    syscall.syscall_gdrom(cpu._dc.?);
}

fn syscall_misc(cpu: *SH4, _: Instr) void {
    syscall.syscall_misc(cpu._dc.?);
}

fn syscall_unknown(cpu: *SH4, _: Instr) void {
    syscall.syscall(cpu._dc.?);
}

pub const Opcodes: [217]OpcodeDescription = .{
    .{ .code = 0b0000000000000000, .mask = 0b0000000000000000, .fn_ = nop, .name = "NOP", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000000000, .mask = 0b1111111111111111, .fn_ = unknown, .name = "Unknown opcode", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    // Fake opcodes to catch emulated syscalls
    .{ .code = 0b0000000000010000, .mask = 0b0000000000000000, .fn_ = syscall_sysinfo, .name = "Syscall Sysinfo", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000000100000, .mask = 0b0000000000000000, .fn_ = syscall_romfont, .name = "Syscall ROMFont", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000000110000, .mask = 0b0000000000000000, .fn_ = syscall_flashrom, .name = "Syscall FlashROM", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000001000000, .mask = 0b0000000000000000, .fn_ = syscall_gdrom, .name = "Syscall GDROM", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000001010000, .mask = 0b0000000000000000, .fn_ = syscall_unknown, .name = "Syscall", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000001100000, .mask = 0b0000000000000000, .fn_ = syscall_misc, .name = "Syscall Misc.", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },

    .{ .code = 0b0110000000000011, .mask = 0b0000111111110000, .fn_ = mov_rm_rn, .name = "mov Rm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1110000000000000, .mask = 0b0000111111111111, .fn_ = mov_imm_rn, .name = "mov #imm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1100011100000000, .mask = 0b0000000011111111, .fn_ = mova_atdispPC_R0, .name = "mova @(d:8,PC),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1001000000000000, .mask = 0b0000111111111111, .fn_ = movw_atdispPC_Rn, .name = "mov.w @(d:8,PC),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1101000000000000, .mask = 0b0000111111111111, .fn_ = movl_atdispPC_Rn, .name = "mov.l @(d:8,PC),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0110000000000000, .mask = 0b0000111111110000, .fn_ = movb_at_rm_rn, .name = "mov.b @Rm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0110000000000001, .mask = 0b0000111111110000, .fn_ = movw_at_rm_rn, .name = "mov.w @Rm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0110000000000010, .mask = 0b0000111111110000, .fn_ = movl_at_rm_rn, .name = "mov.l @Rm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0010000000000000, .mask = 0b0000111111110000, .fn_ = movb_rm_at_rn, .name = "mov.b Rm,@Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0010000000000001, .mask = 0b0000111111110000, .fn_ = movw_rm_at_rn, .name = "mov.w Rm,@Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0010000000000010, .mask = 0b0000111111110000, .fn_ = movl_rm_at_rn, .name = "mov.l Rm,@Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0110000000000100, .mask = 0b0000111111110000, .fn_ = movb_at_rm_inc_rn, .name = "mov.b @Rm+,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 }, // TODO: or 2
    .{ .code = 0b0110000000000101, .mask = 0b0000111111110000, .fn_ = movw_at_rm_inc_rn, .name = "mov.w @Rm+,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 }, // TODO: or 2
    .{ .code = 0b0110000000000110, .mask = 0b0000111111110000, .fn_ = movl_at_rm_inc_rn, .name = "mov.l @Rm+,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 }, // TODO: or 2
    .{ .code = 0b0010000000000100, .mask = 0b0000111111110000, .fn_ = movb_rm_at_rn_dec, .name = "mov.b Rm,@-Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0010000000000101, .mask = 0b0000111111110000, .fn_ = movw_rm_at_rn_dec, .name = "mov.w Rm,@-Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0010000000000110, .mask = 0b0000111111110000, .fn_ = movl_rm_at_rn_dec, .name = "mov.l Rm,@-Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1000010000000000, .mask = 0b0000000011111111, .fn_ = movb_at_disp_Rm_R0, .name = "mov.b @(disp,Rm),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1000010100000000, .mask = 0b0000000011111111, .fn_ = movw_at_disp_Rm_R0, .name = "mov.w @(disp,Rm),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0101000000000000, .mask = 0b0000111111111111, .fn_ = movl_at_disp_Rm_Rn, .name = "mov.l @(disp,Rm),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1000000000000000, .mask = 0b0000000011111111, .fn_ = movb_R0_at_dispRm, .name = "mov.b R0,@(disp,Rm)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1000000100000000, .mask = 0b0000000011111111, .fn_ = movw_R0_at_dispRm, .name = "mov.w R0,@(disp,Rm)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0001000000000000, .mask = 0b0000111111111111, .fn_ = movl_Rm_atdispRn, .name = "mov.l Rm,@(disp,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000001100, .mask = 0b0000111111110000, .fn_ = movb_atR0Rm_rn, .name = "mov.b @(R0,Rm),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0000000000001101, .mask = 0b0000111111110000, .fn_ = movw_atR0Rm_Rn, .name = "mov.w @(R0,Rm),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0000000000001110, .mask = 0b0000111111110000, .fn_ = movl_atR0Rm_rn, .name = "mov.l @(R0,Rm),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0000000000000100, .mask = 0b0000111111110000, .fn_ = movb_Rm_atR0Rn, .name = "mov.b Rm,@(R0,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000000101, .mask = 0b0000111111110000, .fn_ = movw_Rm_atR0Rn, .name = "mov.w Rm,@(R0,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000000110, .mask = 0b0000111111110000, .fn_ = movl_Rm_atR0Rn, .name = "mov.l Rm,@(R0,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1100010000000000, .mask = 0b0000000011111111, .fn_ = movb_atdisp_GBR_R0, .name = "mov.b @(d:8,GBR),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1100010100000000, .mask = 0b0000000011111111, .fn_ = movw_atdisp_GBR_R0, .name = "mov.w @(d:8,GBR),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1100011000000000, .mask = 0b0000000011111111, .fn_ = movl_atdisp_GBR_R0, .name = "mov.l @(d:8,GBR),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1100000000000000, .mask = 0b0000000011111111, .fn_ = movb_R0_atdisp_GBR, .name = "mov.b R0,@(d:8,GBR)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1100000100000000, .mask = 0b0000000011111111, .fn_ = movw_R0_atdisp_GBR, .name = "mov.w R0,@(d:8,GBR)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1100001000000000, .mask = 0b0000000011111111, .fn_ = movl_R0_atdisp_GBR, .name = "mov.l R0,@(d:8,GBR)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000101001, .mask = 0b0000111100000000, .fn_ = movt_Rn, .name = "movt Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0110000000001000, .mask = 0b0000111111110000, .fn_ = swapb, .name = "swap.b Rm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0110000000001001, .mask = 0b0000111111110000, .fn_ = swapw, .name = "swap.w Rm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0010000000001101, .mask = 0b0000111111110000, .fn_ = xtrct_Rm_Rn, .name = "xtrct Rm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0011000000001100, .mask = 0b0000111111110000, .fn_ = add_rm_rn, .name = "add Rm,Rn", .privileged = false },
    .{ .code = 0b0111000000000000, .mask = 0b0000111111111111, .fn_ = add_imm_rn, .name = "add #imm,Rn", .privileged = false },
    .{ .code = 0b0011000000001110, .mask = 0b0000111111110000, .fn_ = addc_Rm_Rn, .name = "addc Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000001111, .mask = 0b0000111111110000, .fn_ = addv_Rm_Rn, .name = "addv Rm,Rn", .privileged = false },
    .{ .code = 0b1000100000000000, .mask = 0b0000000011111111, .fn_ = cmpeq_imm_r0, .name = "cmp/eq #imm,R0", .privileged = false },
    .{ .code = 0b0011000000000000, .mask = 0b0000111111110000, .fn_ = cmpeq_Rm_Rn, .name = "cmp/eq Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000000010, .mask = 0b0000111111110000, .fn_ = cmphs_Rm_Rn, .name = "cmp/hs Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000000011, .mask = 0b0000111111110000, .fn_ = cmpge_Rm_Rn, .name = "cmp/ge Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000000110, .mask = 0b0000111111110000, .fn_ = cmphi_Rm_Rn, .name = "cmp/hi Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000000111, .mask = 0b0000111111110000, .fn_ = cmpgt_Rm_Rn, .name = "cmp/gt Rm,Rn", .privileged = false },
    .{ .code = 0b0100000000010101, .mask = 0b0000111100000000, .fn_ = cmppl_Rn, .name = "cmp/pl Rn", .privileged = false },
    .{ .code = 0b0100000000010001, .mask = 0b0000111100000000, .fn_ = cmppz_Rn, .name = "cmp/pz Rn", .privileged = false },
    .{ .code = 0b0010000000001100, .mask = 0b0000111111110000, .fn_ = cmpstr_Rm_Rn, .name = "cmp/str Rm,Rn", .privileged = false },
    .{ .code = 0b0010000000000111, .mask = 0b0000111111110000, .fn_ = div0s_Rm_Rn, .name = "div0s Rm,Rn", .privileged = false },
    .{ .code = 0b0000000000011001, .mask = 0b0000000000000000, .fn_ = div0u, .name = "div0u", .privileged = false },
    .{ .code = 0b0011000000000100, .mask = 0b0000111111110000, .fn_ = div1, .name = "div1 Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000001101, .mask = 0b0000111111110000, .fn_ = dmulsl_Rm_Rn, .name = "dmuls.l Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0011000000000101, .mask = 0b0000111111110000, .fn_ = dmulul_Rm_Rn, .name = "dmulu.l Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0100000000010000, .mask = 0b0000111100000000, .fn_ = dt_Rn, .name = "dt Rn", .privileged = false },
    .{ .code = 0b0110000000001110, .mask = 0b0000111111110000, .fn_ = extsb_Rm_Rn, .name = "exts.b Rm,Rn", .privileged = false },
    .{ .code = 0b0110000000001111, .mask = 0b0000111111110000, .fn_ = extsw_Rm_Rn, .name = "exts.w Rm,Rn", .privileged = false },
    .{ .code = 0b0110000000001100, .mask = 0b0000111111110000, .fn_ = extub_Rm_Rn, .name = "extu.b Rm,Rn", .privileged = false },
    .{ .code = 0b0110000000001101, .mask = 0b0000111111110000, .fn_ = extuw_Rm_Rn, .name = "extu.w Rm,Rn", .privileged = false },
    .{ .code = 0b0000000000001111, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "mac.l @Rm+,@Rn+", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000001111, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "mac.w @Rm+,@Rn+", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0000000000000111, .mask = 0b0000111111110000, .fn_ = mull_Rm_Rn, .name = "mul.l Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0010000000001111, .mask = 0b0000111111110000, .fn_ = mulsw_Rm_Rn, .name = "muls.w Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0010000000001110, .mask = 0b0000111111110000, .fn_ = muluw_Rm_Rn, .name = "mulu.w Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0110000000001011, .mask = 0b0000111111110000, .fn_ = neg_Rm_Rn, .name = "neg Rm,Rn", .privileged = false },
    .{ .code = 0b0110000000001010, .mask = 0b0000111111110000, .fn_ = negc_Rm_Rn, .name = "negc Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000001000, .mask = 0b0000111111110000, .fn_ = sub_Rm_Rn, .name = "sub Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000001010, .mask = 0b0000111111110000, .fn_ = subc_Rm_Rn, .name = "subc Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000001011, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "subv Rm,Rn", .privileged = false },
    .{ .code = 0b0010000000001001, .mask = 0b0000111111110000, .fn_ = and_Rm_Rn, .name = "and Rm,Rn", .privileged = false },
    .{ .code = 0b1100100100000000, .mask = 0b0000000011111111, .fn_ = and_imm_R0, .name = "and #imm,R0", .privileged = false },
    .{ .code = 0b1100110100000000, .mask = 0b0000000011111111, .fn_ = unimplemented, .name = "and.b #imm,@(R0,GBR)", .privileged = false, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0110000000000111, .mask = 0b0000111111110000, .fn_ = not_Rm_Rn, .name = "not Rm,Rn", .privileged = false },
    .{ .code = 0b0010000000001011, .mask = 0b0000111111110000, .fn_ = or_Rm_Rn, .name = "or Rm,Rn", .privileged = false },
    .{ .code = 0b1100101100000000, .mask = 0b0000000011111111, .fn_ = or_imm_r0, .name = "or #imm,R0", .privileged = false },
    .{ .code = 0b1100111100000000, .mask = 0b0000000011111111, .fn_ = unimplemented, .name = "or.b #imm,@(R0,GBR)", .privileged = false, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0100000000011011, .mask = 0b0000111100000000, .fn_ = tasb_at_Rn, .name = "tas.b @Rn", .privileged = false, .issue_cycles = 5, .latency_cycles = 5 },
    .{ .code = 0b0010000000001000, .mask = 0b0000111111110000, .fn_ = tst_Rm_Rn, .name = "tst Rm,Rn", .privileged = false },
    .{ .code = 0b1100100000000000, .mask = 0b0000000011111111, .fn_ = tst_imm_r0, .name = "tst #imm,R0", .privileged = false },
    .{ .code = 0b1100110000000000, .mask = 0b0000000011111111, .fn_ = unimplemented, .name = "tst.b #imm,@(R0,GBR)", .privileged = false, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0010000000001010, .mask = 0b0000111111110000, .fn_ = xorRmRn, .name = "xor Rm,Rn", .privileged = false },
    .{ .code = 0b1100101000000000, .mask = 0b0000000011111111, .fn_ = xorImmR0, .name = "xor #imm,R0", .privileged = false },
    .{ .code = 0b1100111000000000, .mask = 0b0000000011111111, .fn_ = unimplemented, .name = "xor.b #imm,@(R0,GBR)", .privileged = false, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0100000000100100, .mask = 0b0000111100000000, .fn_ = rotcl_Rn, .name = "rotcl Rn", .privileged = false },
    .{ .code = 0b0100000000100101, .mask = 0b0000111100000000, .fn_ = rotcr_Rn, .name = "rotcr Rn", .privileged = false },
    .{ .code = 0b0100000000000100, .mask = 0b0000111100000000, .fn_ = rotl_Rn, .name = "rotl Rn", .privileged = false },
    .{ .code = 0b0100000000000101, .mask = 0b0000111100000000, .fn_ = rotr_Rn, .name = "rotr Rn", .privileged = false },
    .{ .code = 0b0100000000001100, .mask = 0b0000111111110000, .fn_ = shad_Rm_Rn, .name = "shad Rm,Rn", .privileged = false },
    .{ .code = 0b0100000000100000, .mask = 0b0000111100000000, .fn_ = shal_Rn, .name = "shal Rn", .privileged = false },
    .{ .code = 0b0100000000100001, .mask = 0b0000111100000000, .fn_ = shar_Rn, .name = "shar Rn", .privileged = false },
    .{ .code = 0b0100000000001101, .mask = 0b0000111111110000, .fn_ = shld_Rm_Rn, .name = "shld Rm,Rn", .privileged = false },
    .{ .code = 0b0100000000000000, .mask = 0b0000111100000000, .fn_ = shll, .name = "shll Rn", .privileged = false },
    .{ .code = 0b0100000000001000, .mask = 0b0000111100000000, .fn_ = shll2, .name = "shll2 Rn", .privileged = false },
    .{ .code = 0b0100000000011000, .mask = 0b0000111100000000, .fn_ = shll8, .name = "shll8 Rn", .privileged = false },
    .{ .code = 0b0100000000101000, .mask = 0b0000111100000000, .fn_ = shll16, .name = "shll16 Rn", .privileged = false },
    .{ .code = 0b0100000000000001, .mask = 0b0000111100000000, .fn_ = shlr, .name = "shlr Rn", .privileged = false },
    .{ .code = 0b0100000000001001, .mask = 0b0000111100000000, .fn_ = shlr2, .name = "shlr2 Rn", .privileged = false },
    .{ .code = 0b0100000000011001, .mask = 0b0000111100000000, .fn_ = shlr8, .name = "shlr8 Rn", .privileged = false },
    .{ .code = 0b0100000000101001, .mask = 0b0000111100000000, .fn_ = shlr16, .name = "shlr16 Rn", .privileged = false },
    .{ .code = 0b1000101100000000, .mask = 0b0000000011111111, .fn_ = bf_label, .name = "bf label", .privileged = false },
    .{ .code = 0b1000111100000000, .mask = 0b0000000011111111, .fn_ = bfs_label, .name = "bf/s label", .privileged = false },
    .{ .code = 0b1000100100000000, .mask = 0b0000000011111111, .fn_ = bt_label, .name = "bt label", .privileged = false },
    .{ .code = 0b1000110100000000, .mask = 0b0000000011111111, .fn_ = bts_label, .name = "bt/s label", .privileged = false },
    .{ .code = 0b1010000000000000, .mask = 0b0000111111111111, .fn_ = bra_label, .name = "bra label", .privileged = false },
    .{ .code = 0b0000000000100011, .mask = 0b0000111100000000, .fn_ = braf_Rn, .name = "braf Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b1011000000000000, .mask = 0b0000111111111111, .fn_ = bsr_label, .name = "bsr label", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0000000000000011, .mask = 0b0000111100000000, .fn_ = bsrf_Rn, .name = "bsrf Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0100000000101011, .mask = 0b0000111100000000, .fn_ = jmp_atRn, .name = "jmp @Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0100000000001011, .mask = 0b0000111100000000, .fn_ = jsr_Rn, .name = "jsr @Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0000000000001011, .mask = 0b0000000000000000, .fn_ = rts, .name = "rts", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0000000000101000, .mask = 0b0000000000000000, .fn_ = clrmac, .name = "clrmac", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0000000001001000, .mask = 0b0000000000000000, .fn_ = clrs, .name = "clrs", .privileged = false },
    .{ .code = 0b0000000000001000, .mask = 0b0000000000000000, .fn_ = clrt, .name = "clrt", .privileged = false },
    .{ .code = 0b0100000000001110, .mask = 0b0000111100000000, .fn_ = ldc_Rn_SR, .name = "ldc Rn,SR", .privileged = true, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0100000000000111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_SR, .name = "ldc.l @Rn+,SR", .privileged = true, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0100000000011110, .mask = 0b0000111100000000, .fn_ = ldc_Rn_GBR, .name = "ldc Rn,GBR", .privileged = false, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0100000000010111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_GBR, .name = "ldc.l @Rn+,GBR", .privileged = false, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0100000000101110, .mask = 0b0000111100000000, .fn_ = ldc_Rn_VBR, .name = "ldc Rn,VBR", .privileged = true, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000100111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_VBR, .name = "ldc.l @Rn+,VBR", .privileged = true },
    .{ .code = 0b0100000000111110, .mask = 0b0000111100000000, .fn_ = ldc_Rn_SSR, .name = "ldc Rn,SSR", .privileged = true, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000110111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_SSR, .name = "ldc.l @Rn+,SSR", .privileged = true },
    .{ .code = 0b0100000001001110, .mask = 0b0000111100000000, .fn_ = ldc_Rn_SPC, .name = "ldc Rn,SPC", .privileged = true, .issue_cycles = 3, .latency_cycles = 1 },
    .{ .code = 0b0100000001000111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_SPC, .name = "ldc.l @Rn+,SPC", .privileged = true },
    .{ .code = 0b0100000011111010, .mask = 0b0000111100000000, .fn_ = ldc_Rn_DBR, .name = "ldc Rn,DBR", .privileged = true, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000011110110, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_DBR, .name = "ldc.l @Rn+,DBR", .privileged = true },
    .{ .code = 0b0100000010001110, .mask = 0b0000111101110000, .fn_ = ldc_Rn_Rm_BANK, .name = "ldc Rn,Rm_BANK", .privileged = true, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000010000111, .mask = 0b0000111101110000, .fn_ = ldcl_at_Rn_inc_Rm_BANK, .name = "ldc.l @Rn+,Rm_BANK", .privileged = true },
    .{ .code = 0b0100000000001010, .mask = 0b0000111100000000, .fn_ = lds_Rn_MACH, .name = "lds Rn,MACH", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000000110, .mask = 0b0000111100000000, .fn_ = ldsl_at_Rn_inc_MACH, .name = "lds.l @Rn+,MACH", .privileged = false },
    .{ .code = 0b0100000000011010, .mask = 0b0000111100000000, .fn_ = lds_Rn_MACL, .name = "lds Rn,MACL", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000010110, .mask = 0b0000111100000000, .fn_ = ldsl_at_Rn_inc_MACL, .name = "lds.l @Rn+,MACL", .privileged = false },
    .{ .code = 0b0100000000101010, .mask = 0b0000111100000000, .fn_ = lds_Rn_PR, .name = "lds Rn,PR", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0100000000100110, .mask = 0b0000111100000000, .fn_ = ldsl_atRn_inc_PR, .name = "lds.l @Rn+,PR", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000111000, .mask = 0b0000000000000000, .fn_ = unimplemented, .name = "ldtbl", .privileged = true },
    .{ .code = 0b0000000011000011, .mask = 0b0000111100000000, .fn_ = movcal_R0_atRn, .name = "movca.l R0,@Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0000000000001001, .mask = 0b0000000000000000, .fn_ = nop, .name = "nop", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b0000000010010011, .mask = 0b0000111100000000, .fn_ = ocbi_atRn, .name = "ocbi @Rn", .privileged = false },
    .{ .code = 0b0000000010100011, .mask = 0b0000111100000000, .fn_ = ocbp_atRn, .name = "ocbp @Rn", .privileged = false },
    .{ .code = 0b0000000010110011, .mask = 0b0000111100000000, .fn_ = ocbwb_atRn, .name = "ocbwb @Rn", .privileged = false },
    .{ .code = 0b0000000010000011, .mask = 0b0000111100000000, .fn_ = pref_atRn, .name = "pref @Rn", .privileged = false },
    .{ .code = 0b0000000000101011, .mask = 0b0000000000000000, .fn_ = rte, .name = "rte", .privileged = true, .issue_cycles = 5, .latency_cycles = 5 },
    .{ .code = 0b0000000001011000, .mask = 0b0000000000000000, .fn_ = sets, .name = "sets", .privileged = false },
    .{ .code = 0b0000000000011000, .mask = 0b0000000000000000, .fn_ = sett, .name = "sett", .privileged = false },
    .{ .code = 0b0000000000011011, .mask = 0b0000000000000000, .fn_ = sleep, .name = "sleep", .privileged = true, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0000000000000010, .mask = 0b0000111100000000, .fn_ = stc_SR_Rn, .name = "stc SR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000000011, .mask = 0b0000111100000000, .fn_ = stcl_SR_at_Rn_dec, .name = "stc.l SR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000010010, .mask = 0b0000111100000000, .fn_ = stc_GBR_Rn, .name = "stc GBR,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000010011, .mask = 0b0000111100000000, .fn_ = stcl_GBR_at_Rn_dec, .name = "stc.l GBR,@-Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000100010, .mask = 0b0000111100000000, .fn_ = stc_VBR_Rn, .name = "stc VBR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000100011, .mask = 0b0000111100000000, .fn_ = stcl_VBR_at_Rn_dec, .name = "stc.l VBR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000111010, .mask = 0b0000111100000000, .fn_ = stc_SGR_rn, .name = "stc SGR,Rn", .privileged = true, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0100000000110010, .mask = 0b0000111100000000, .fn_ = stcl_SGR_at_Rn_dec, .name = "stc.l SGR,@-Rn", .privileged = true, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0000000000110010, .mask = 0b0000111100000000, .fn_ = stc_SSR_rn, .name = "stc SSR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000110011, .mask = 0b0000111100000000, .fn_ = stcl_SSR_at_Rn_dec, .name = "stc.l SSR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000001000010, .mask = 0b0000111100000000, .fn_ = stc_SPC_rn, .name = "stc SPC,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000001000011, .mask = 0b0000111100000000, .fn_ = stcl_SPC_at_Rn_dec, .name = "stc.l SPC,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000011111010, .mask = 0b0000111100000000, .fn_ = stc_DBR_rn, .name = "stc DBR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000011110010, .mask = 0b0000111100000000, .fn_ = stcl_DBR_at_Rn_dec, .name = "stc.l DBR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000010000010, .mask = 0b0000111101110000, .fn_ = stc_Rm_BANK_Rn, .name = "stc Rm_BANK,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000010000011, .mask = 0b0000111101110000, .fn_ = stcl_Rm_BANK_at_Rn_dec, .name = "stc.l Rm_BANK,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000001010, .mask = 0b0000111100000000, .fn_ = sts_MACH_Rn, .name = "sts MACH,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000000010, .mask = 0b0000111100000000, .fn_ = stsl_MACH_at_Rn_dec, .name = "sts.l MACH,@-Rn", .privileged = false },
    .{ .code = 0b0000000000011010, .mask = 0b0000111100000000, .fn_ = sts_MACL_Rn, .name = "sts MACL,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000010010, .mask = 0b0000111100000000, .fn_ = stsl_MACL_at_Rn_dec, .name = "sts.l MACL,@-Rn", .privileged = false },
    .{ .code = 0b0000000000101010, .mask = 0b0000111100000000, .fn_ = sts_PR_Rn, .name = "sts PR,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000100010, .mask = 0b0000111100000000, .fn_ = stsl_PR_atRn_dec, .name = "sts.l PR,@-Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b1100001100000000, .mask = 0b0000000011111111, .fn_ = trapa_imm, .name = "trapa #imm", .privileged = false, .issue_cycles = 7, .latency_cycles = 7 },

    .{ .code = 0b1111000000001100, .mask = 0b0000111111110000, .fn_ = fmov_FRm_FRn, .name = "fmov FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000000001000, .mask = 0b0000111111110000, .fn_ = fmovs_atRm_FRn, .name = "fmov.s @Rm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1111000000001010, .mask = 0b0000111111110000, .fn_ = fmovs_FRm_atRn, .name = "fmov.s FRm,@Rn", .privileged = false },
    .{ .code = 0b1111000000001001, .mask = 0b0000111111110000, .fn_ = fmovs_at_Rm_inc_FRn, .name = "fmov.s @Rm+,FRn", .privileged = false },
    .{ .code = 0b1111000000001011, .mask = 0b0000111111110000, .fn_ = fmovs_FRm_at_dec_Rn, .name = "fmov.s FRm,@-Rn", .privileged = false },
    .{ .code = 0b1111000000000110, .mask = 0b0000111111110000, .fn_ = fmovs_at_R0_Rm_FRn, .name = "fmov.s @(R0,Rm),FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1111000000000111, .mask = 0b0000111111110000, .fn_ = fmovs_FRm_at_R0_Rn, .name = "fmov.s FRm,@(R0,Rn)", .privileged = false },

    // Actually handled by single precision version - Switched by SR register
    //.{ .code = 0b1111000000001100, .mask = 0b0000111011100000, .fn_ = fmov_DRm_DRn, .name = "fmov DRm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    //.{ .code = 0b1111000100001100, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fmov DRm,XDn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    //.{ .code = 0b1111000000011100, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fmov XDm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    //.{ .code = 0b1111000100011100, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fmov XDm,XDn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    //.{ .code = 0b1111000000001000, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @Rm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    //.{ .code = 0b1111000100001000, .mask = 0b0000111011110000, .fn_ = fmovd_atRm_XDn, .name = "fmov.d @Rm,XDn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    //.{ .code = 0b1111000000001010, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d DRm,@Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    //.{ .code = 0b1111000000011010, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d XDm,@Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    //.{ .code = 0b1111000000001001, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @Rm+,DRn", .privileged = false },
    //.{ .code = 0b1111000100001001, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @Rm+,XDn", .privileged = false },
    //.{ .code = 0b1111000000001011, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d DRm,@-Rn", .privileged = false },
    //.{ .code = 0b1111000000011011, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d XDm,@-Rn", .privileged = false },
    //.{ .code = 0b1111000000000110, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @(R0,Rm),DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    //.{ .code = 0b1111000100000110, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @(R0,Rm),XDn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    //.{ .code = 0b1111000000000111, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d DRm,@(R0,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    //.{ .code = 0b1111000000010111, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d XDm,@(R0,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },

    .{ .code = 0b1111000010001101, .mask = 0b0000111100000000, .fn_ = fldi0_FRn, .name = "fldi0 FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000010011101, .mask = 0b0000111100000000, .fn_ = fldi1_FRn, .name = "fldi1 FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000000011101, .mask = 0b0000111100000000, .fn_ = flds_FRn_FPUL, .name = "flds FRm,FPUL", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000000001101, .mask = 0b0000111100000000, .fn_ = fsts_FPUL_FRn, .name = "fsts FPUL,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000001011101, .mask = 0b0000111100000000, .fn_ = fabs_FRn, .name = "fabs FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000001001101, .mask = 0b0000111100000000, .fn_ = fneg_FRn, .name = "fneg FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000000000000, .mask = 0b0000111111110000, .fn_ = fadd_FRm_FRn, .name = "fadd FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000000001, .mask = 0b0000111111110000, .fn_ = fsub_FRm_FRn, .name = "fsub FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000000010, .mask = 0b0000111111110000, .fn_ = fmul_FRm_FRn, .name = "fmul FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000001110, .mask = 0b0000111111110000, .fn_ = fmac_FR0_FRm_FRn, .name = "fmac FR0,FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000000011, .mask = 0b0000111111110000, .fn_ = fdiv_FRm_FRn, .name = "fdiv FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 11 },
    .{ .code = 0b1111000001101101, .mask = 0b0000111100000000, .fn_ = fsqrt_FRn, .name = "fsqrt FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 11 },
    .{ .code = 0b1111000000000100, .mask = 0b0000111111110000, .fn_ = fcmp_eq_FRm_FRn, .name = "fcmp/eq FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1111000000000101, .mask = 0b0000111111110000, .fn_ = fcmp_gt_FRm_FRn, .name = "fcmp/gt FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1111000000101101, .mask = 0b0000111100000000, .fn_ = float_FPUL_FRn, .name = "float FPUL,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000111101, .mask = 0b0000111100000000, .fn_ = ftrc_FRn_FPUL, .name = "ftrc FRn,FPUL", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000011101101, .mask = 0b0000111100000000, .fn_ = fipr_FVm_FVn, .name = "fipr FVm,FVn", .privileged = false, .issue_cycles = 1, .latency_cycles = 4 },
    .{ .code = 0b1111000111111101, .mask = 0b0000110000000000, .fn_ = ftrv_XMTRX_FVn, .name = "ftrv XMTRX,FVn", .privileged = false, .issue_cycles = 1, .latency_cycles = 5 },
    // Undocumented opcodes - Supposed to be exclusive to the SH4A, but some games seem to use them (I hope this is not sue to a mistake I made somewhere else :D).
    .{ .code = 0b1111000001111101, .mask = 0b0000111100000000, .fn_ = fsrra_FRn, .name = "fsrra FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1111000011111101, .mask = 0b0000111000000000, .fn_ = fsca_FPUL_DRn, .name = "fsca FPUL,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },

    //.{ .code = 0b1111000001011101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fabs DRn", .privileged = false },
    //.{ .code = 0b1111000001001101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fneg DRn", .privileged = false },
    //.{ .code = 0b1111000000000000, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fadd DRm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 7 },
    //.{ .code = 0b1111000000000001, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fsub DRm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 7 },
    //.{ .code = 0b1111000000000010, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fmul DRm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 7 },
    //.{ .code = 0b1111000000000011, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fdiv DRm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 24 },
    //.{ .code = 0b1111000001101101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fsqrt DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 23 },
    //.{ .code = 0b1111000000000100, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fcmp/eq DRm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    //.{ .code = 0b1111000000000101, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fcmp/gt DRm,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    //.{ .code = 0b1111000000101101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "float FPUL,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    //.{ .code = 0b1111000000111101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "ftrc DRm,FPUL", .privileged = false, .issue_cycles = 1, .latency_cycles = 4 },
    .{ .code = 0b1111000010111101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fcnvds DRm,FPUL", .privileged = false, .issue_cycles = 1, .latency_cycles = 4 },
    .{ .code = 0b1111000010101101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fcnvsd FPUL,DRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },

    .{ .code = 0b0100000001101010, .mask = 0b0000111100000000, .fn_ = lds_Rn_FPSCR, .name = "lds Rn,FPSCR", .privileged = false, .issue_cycles = 1, .latency_cycles = 4 },
    .{ .code = 0b0000000001101010, .mask = 0b0000111100000000, .fn_ = sts_FPSCR_Rn, .name = "sts FPSCR,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000001100110, .mask = 0b0000111100000000, .fn_ = ldsl_at_Rn_inc_FPSCR, .name = "lds.l @Rn+,FPSCR", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000001100010, .mask = 0b0000111100000000, .fn_ = stsl_FPSCR_at_Rn_dec, .name = "sts.l FPSCR,@-Rn", .privileged = false },
    .{ .code = 0b0100000001011010, .mask = 0b0000111100000000, .fn_ = lds_Rn_FPUL, .name = "lds Rn,FPUL", .privileged = false },
    .{ .code = 0b0000000001011010, .mask = 0b0000111100000000, .fn_ = sts_FPUL_Rn, .name = "sts FPUL,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000001010110, .mask = 0b0000111100000000, .fn_ = ldsl_at_Rn_inc_FPUL, .name = "lds.l @Rn+,FPUL", .privileged = false },
    .{ .code = 0b0100000001010010, .mask = 0b0000111100000000, .fn_ = stsl_FPUL_at_Rn_dec, .name = "sts.l FPUL,@-Rn", .privileged = false },
    .{ .code = 0b1111101111111101, .mask = 0b0000000000000000, .fn_ = frchg, .name = "frchg", .privileged = false },
    .{ .code = 0b1111001111111101, .mask = 0b0000000000000000, .fn_ = fschg, .name = "fschg", .privileged = false },
};

fn init_jump_table() void {
    if (JumpTable[0] == 1) {
        JumpTable[0] = 0; // NOP
        for (1..0x10000) |i| {
            for (2..Opcodes.len) |idx| {
                if ((i & ~Opcodes[idx].mask) == Opcodes[idx].code) {
                    if (JumpTable[i] != 1) {
                        std.debug.print("{b:0>16}: Matches {s} but already set to {s}\n", .{ i, Opcodes[idx].name, Opcodes[JumpTable[i]].name });
                        @panic("Duplicate matching instruction.");
                    }
                    JumpTable[i] = @intCast(idx);
                    //break;
                }
            }
        }
    }
}

var DisassemblyCache: [0x10000]?[]const u8 = .{null} ** 0x10000;

pub fn disassemble(opcode: Instr, allocator: std.mem.Allocator) ![]const u8 {
    if (DisassemblyCache[opcode.value] != null) {
        return DisassemblyCache[opcode.value].?;
    }

    const desc = Opcodes[JumpTable[opcode.value]];

    const Rn = try std.fmt.allocPrint(allocator, "R{d}", .{opcode.nmd.n});
    const Rm = try std.fmt.allocPrint(allocator, "R{d}", .{opcode.nmd.m});
    const disp = try std.fmt.allocPrint(allocator, "{d}", .{opcode.nmd.d});
    const d8 = try std.fmt.allocPrint(allocator, "{d}", .{@as(i8, @bitCast(opcode.nd8.d))});
    const d12 = try std.fmt.allocPrint(allocator, "{d}", .{@as(i12, @bitCast(opcode.d12.d))});
    const imm = try std.fmt.allocPrint(allocator, "#{d}", .{@as(i8, @bitCast(opcode.nd8.d))});
    defer allocator.free(Rn);
    defer allocator.free(Rm);
    defer allocator.free(disp);
    defer allocator.free(d8);
    defer allocator.free(d12);
    defer allocator.free(imm);

    const n0 = try std.mem.replaceOwned(u8, allocator, desc.name, "Rn", Rn);
    defer allocator.free(n0);
    const n1 = try std.mem.replaceOwned(u8, allocator, n0, "Rm", Rm);
    defer allocator.free(n1);
    const n2 = try std.mem.replaceOwned(u8, allocator, n1, "disp", disp);
    defer allocator.free(n2);
    const n3 = try std.mem.replaceOwned(u8, allocator, n2, "d:8", d8);
    defer allocator.free(n3);
    const n4 = try std.mem.replaceOwned(u8, allocator, n3, "d:12", d12);
    defer allocator.free(n4);
    const final_buff = try std.mem.replaceOwned(u8, allocator, n4, "#imm", imm);

    DisassemblyCache[opcode.value] = final_buff;

    return final_buff;
}

pub fn free_disassembly_cache(allocator: std.mem.Allocator) void {
    for (DisassemblyCache) |d| {
        if (d != null) {
            allocator.free(d.?);
        }
    }
}

fn test_decoding(instruction: Instr, comptime expected: []const u8) !void {
    const dis = try disassemble(instruction, std.testing.allocator);
    std.debug.print("{b:0>16}: {s} - {s}\n", .{ instruction.value, dis, expected });
    try std.testing.expect(std.mem.eql(u8, dis, expected));
}

test "Instruction Decoding" {
    defer free_disassembly_cache(std.testing.allocator);
    init_jump_table();

    try test_decoding(.{ .value = 0b1110_0000_0000_0001 }, "mov #1,R0");

    try test_decoding(.{ .value = 0b1001_0000_0000_0001 }, "mov.w @(1,PC),R0");
    try test_decoding(.{ .value = 0b1101_0000_0000_0001 }, "mov.l @(1,PC),R0");

    try test_decoding(.{ .value = 0b0110_0000_0000_0011 }, "mov R0,R0");

    try test_decoding(.{ .value = 0b0010_0000_0001_0000 }, "mov.b R1,@R0");
    try test_decoding(.{ .value = 0b0010_0000_0001_0001 }, "mov.w R1,@R0");
    try test_decoding(.{ .value = 0b0010_0000_0001_0010 }, "mov.l R1,@R0");

    try test_decoding(.{ .value = 0b0110_0000_0001_0000 }, "mov.b @R1,R0");
    try test_decoding(.{ .value = 0b0110_0000_0001_0001 }, "mov.w @R1,R0");
    try test_decoding(.{ .value = 0b0110_0000_0001_0010 }, "mov.l @R1,R0");

    try test_decoding(.{ .value = 0b0010_0000_0001_0100 }, "mov.b R1,@-R0");
    try test_decoding(.{ .value = 0b0010_0000_0001_0101 }, "mov.w R1,@-R0");
    try test_decoding(.{ .value = 0b0010_0000_0001_0110 }, "mov.l R1,@-R0");

    try test_decoding(.{ .value = 0b0110_0000_0001_0100 }, "mov.b @R1+,R0");
    try test_decoding(.{ .value = 0b0110_0000_0001_0101 }, "mov.w @R1+,R0");
    try test_decoding(.{ .value = 0b0110_0000_0001_0110 }, "mov.l @R1+,R0");

    try test_decoding(.{ .value = 0b1000_0000_0001_0010 }, "mov.b R0,@(2,R1)");
    try test_decoding(.{ .value = 0b1000_0001_0001_0010 }, "mov.w R0,@(2,R1)");

    try test_decoding(.{ .value = 0b0001_0001_0011_0010 }, "mov.l R3,@(2,R1)");

    try test_decoding(.{ .value = 0b1000_0100_0001_0010 }, "mov.b @(2,R1),R0");
    try test_decoding(.{ .value = 0b1000_0101_0001_0010 }, "mov.w @(2,R1),R0");

    try test_decoding(.{ .value = 0b0101_0011_0001_0010 }, "mov.l @(2,R1),R3");

    try test_decoding(.{ .value = 0b0000_0001_0011_0100 }, "mov.b R3,@(R0,R1)");
    try test_decoding(.{ .value = 0b0000_0001_0011_0101 }, "mov.w R3,@(R0,R1)");
    try test_decoding(.{ .value = 0b0000_0001_0011_0110 }, "mov.l R3,@(R0,R1)");

    try test_decoding(.{ .value = 0b0000_0001_0011_1100 }, "mov.b @(R0,R3),R1");
    try test_decoding(.{ .value = 0b0000_0001_0011_1101 }, "mov.w @(R0,R3),R1");
    try test_decoding(.{ .value = 0b0000_0001_0011_1110 }, "mov.l @(R0,R3),R1");

    try test_decoding(.{ .value = 0b1100_0000_0000_0001 }, "mov.b R0,@(1,GBR)");
    try test_decoding(.{ .value = 0b1100_0001_0000_0001 }, "mov.w R0,@(1,GBR)");
    try test_decoding(.{ .value = 0b1100_0010_0000_0001 }, "mov.l R0,@(1,GBR)");

    try test_decoding(.{ .value = 0b1100_0100_0000_0001 }, "mov.b @(1,GBR),R0");
    try test_decoding(.{ .value = 0b1100_0101_0000_0001 }, "mov.w @(1,GBR),R0");
    try test_decoding(.{ .value = 0b1100_0110_0000_0001 }, "mov.l @(1,GBR),R0");

    try test_decoding(.{ .value = 0b1100_0111_0000_0000 }, "mova @(0,PC),R0");
    try test_decoding(.{ .value = 0b0000_0011_0010_1001 }, "movt R3");

    try test_decoding(.{ .value = 0b1000_1011_0000_0100 }, "bf label");
    try test_decoding(.{ .value = 0b1000_1111_0000_0100 }, "bf/s label");
    try test_decoding(.{ .value = 0b1000_1001_0000_0100 }, "bt label");
    try test_decoding(.{ .value = 0b1000_1101_0000_0100 }, "bt/s label");
    try test_decoding(.{ .value = 0b1010_0000_0000_0100 }, "bra label");
    try test_decoding(.{ .value = 0b0000_0001_0010_0011 }, "braf R1");
    try test_decoding(.{ .value = 0b1011_0000_0000_0100 }, "bsr label");
    try test_decoding(.{ .value = 0b0000_0001_0000_0011 }, "bsrf R1");
    try test_decoding(.{ .value = 0b0100_0001_0010_1011 }, "jmp @R1");
    try test_decoding(.{ .value = 0b0100_0001_0000_1011 }, "jsr @R1");
    try test_decoding(.{ .value = 0b0000_0000_0000_1011 }, "rts");
}

test "mov #imm,Rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0 } });
    try std.testing.expect(cpu.R(0).* == 0);
    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0xFF } });
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFF);
    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 1, .d = 1 } });
    try std.testing.expect(cpu.R(1).* == 1);
    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 2, .d = 2 } });
    try std.testing.expect(cpu.R(2).* == 2);
    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 3, .d = 3 } });
    try std.testing.expect(cpu.R(3).* == 3);

    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 15, .d = 0 } }); // mov #0,R15
    try std.testing.expect(cpu.R(15).* == 0);
    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 15, .d = 1 } }); // mov #1,R15
    try std.testing.expect(cpu.R(15).* == 1);
}

test "mov Rm,Rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0 } }); // mov #0,R0
    try std.testing.expect(cpu.R(0).* == 0);
    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 1, .d = 1 } }); // mov #1,R1
    try std.testing.expect(cpu.R(1).* == 1);
    mov_rm_rn(&cpu, .{ .nmd = .{ .n = 0, .m = 1 } }); // mov R1,R0
    try std.testing.expect(cpu.R(0).* == 1);
}

test "ldc Rn,SR" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 3 } }); // mov #3,R0
    try std.testing.expect(cpu.R(0).* == 0b000000011);
    ldc_Rn_SR(&cpu, .{ .nmd = .{ .n = 0 } }); // ldc R0,SR
    try std.testing.expect(@as(u32, @bitCast(cpu.sr)) == 0b00000011 & 0x700083F3);

    cpu.set_sr(.{});

    mov_imm_rn(&cpu, .{ .nd8 = .{ .n = 15, .d = 3 } }); // mov #3,R15
    try std.testing.expect(cpu.R(15).* == 0b00000011);
    ldc_Rn_SR(&cpu, .{ .nmd = .{ .n = 15 } }); // ldc R15,SR
    try std.testing.expect(@as(u32, @bitCast(cpu.sr)) == 0b00000011 & 0x700083F3);
}
