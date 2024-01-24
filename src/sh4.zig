// Hitachi SH-4
// FIXME: Exact model is actually SH7091, I think.

const std = @import("std");
const builtin = @import("builtin");
const common = @import("./common.zig");
const termcolor = @import("termcolor.zig");

pub const sh4_log = std.log.scoped(.sh4);

const Dreamcast = @import("dreamcast.zig").Dreamcast;

const mmu = @import("./mmu.zig");
pub const P4 = @import("./sh4_p4.zig");
pub const P4Register = P4.P4Register;
const Interrupts = @import("Interrupts.zig");
const Interrupt = Interrupts.Interrupt;

const addr_t = common.addr_t;

const sh4_instructions = @import("sh4_instructions.zig");
const init_jump_table = sh4_instructions.init_jump_table;
const sh4_disassembly = @import("sh4_disassembly.zig");

const HardwareRegisters = @import("hardware_registers.zig");
const HardwareRegister = HardwareRegisters.HardwareRegister;

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

pub const FPSCR = packed struct(u32) {
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

        self.p4_register(u16, .SCSMR2).* = 0x0000;
        self.p4_register(u8, .SCBRR2).* = 0xFF;
        self.p4_register(u16, .SCSCR2).* = 0x0000;
        self.p4_register(u16, .SCFSR2).* = 0x0060;
        self.p4_register(u16, .SCFCR2).* = 0x0000;
        self.p4_register(u16, .SCFDR2).* = 0x0000;
        self.p4_register(u16, .SCSPTR2).* = 0x0000;
        self.p4_register(u16, .SCLSR2).* = 0x0000;
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
        self.pr = 0xAC00043C;
        self.fpul = 0x00000000;

        self.pc = 0xAC008300; // Start address of IP.bin Licence screen
    }

    pub inline fn set_sr(self: *@This(), value: SR) void {
        const prev_rb = if (self.sr.md == 1) self.sr.rb else 0;
        const new_rb = if (value.md == 1) value.rb else 0;
        if (new_rb != prev_rb) {
            std.mem.swap([8]u32, self.r[0..8], &self.r_bank);
        }
        self.sr = @bitCast(@as(u32, @bitCast(value)) & 0x700083F3);
    }

    pub inline fn set_fpscr(self: *@This(), value: u32) void {
        const new_value: FPSCR = @bitCast(value & 0x003FFFFF);
        if (new_value.fr != self.fpscr.fr) {
            std.mem.swap(@TypeOf(self.fp_banks[0]), &self.fp_banks[0], &self.fp_banks[1]);
        }
        self.fpscr = new_value;
    }

    inline fn read_operand_cache(self: *const @This(), comptime T: type, virtual_addr: addr_t) T {
        return @constCast(self).operand_cache(T, virtual_addr).*;
    }
    inline fn operand_cache(self: *@This(), comptime T: type, virtual_addr: addr_t) *T {
        // Half of the operand cache can be used as RAM when CCR.ORA == 1, and some games do.
        std.debug.assert(self.read_p4_register(P4.CCR, .CCR).ora == 1);
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

            return @alignCast(@ptrCast(&self._operand_cache[virtual_addr & 0x3FFF]));
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
                const index = ((virtual_addr & 0x03000000) >> 16) | (virtual_addr & 0x3FFF);
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

    pub inline fn DR(self: *@This(), r: u4) *f64 {
        std.debug.assert(r < 8);
        return &self.fp_banks[0].dr[r];
    }
    pub inline fn XF(self: *@This(), r: u4) *f32 {
        return &self.fp_banks[1].fr[r];
    }

    pub inline fn XD(self: *@This(), r: u4) *f64 {
        std.debug.assert(r < 8);
        return &self.fp_banks[1].dr[r];
    }

    pub inline fn QR(self: *@This(), r: u4) *f32 {
        std.debug.assert(r < 4);
        return &self.fp_banks[0].qr[r];
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

    fn jump_to_exception(self: *@This()) void {
        self.spc = self.pc;
        self.ssr = @bitCast(self.sr);
        self.sgr = self.R(15).*;

        var new_sr = self.sr;
        new_sr.bl = true;
        new_sr.md = 1;
        new_sr.rb = 1;
        self.set_sr(new_sr);

        const offset = 0x600; // TODO
        const UserBreak = false; // TODO
        if (self.dc.read_p4_register(HardwareRegisters.BRCR, .BRCR).ubde == 1 and UserBreak) {
            self.pc = self.dbr;
        } else {
            self.pc = self.vbr + offset;
        }
    }

    pub fn handle_interrupts(self: *@This()) void {
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
            // FIXME: Not sure if this is a thing.
            self.add_cycles(8);
            const cycles = self._pending_cycles;
            self._pending_cycles = 0;
            return cycles;
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
            else => @panic("Invalid prescaler"),
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
            .{ .counter = P4Register.TCNT0, .control = P4Register.TCR0, .constant = P4Register.TCOR0, .interrupt = Interrupt.TUNI0 },
            .{ .counter = P4Register.TCNT1, .control = P4Register.TCR1, .constant = P4Register.TCOR1, .interrupt = Interrupt.TUNI1 },
            .{ .counter = P4Register.TCNT2, .control = P4Register.TCR2, .constant = P4Register.TCOR2, .interrupt = Interrupt.TUNI2 },
        };

        inline for (0..3) |i| {
            if ((TSTR >> @intCast(i)) & 0x1 == 1) {
                const tcnt = self.p4_register(u32, timers[i].counter);
                const tcr = self.p4_register(P4.TCR, timers[i].control);

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
        const desc = sh4_instructions.Opcodes[sh4_instructions.JumpTable[opcode]];

        if ((comptime builtin.mode == .Debug or builtin.mode == .ReleaseSafe) and self.debug_trace)
            std.debug.print("[{X:0>8}] {b:0>16} {s: <20} R{d: <2}={X:0>8}, R{d: <2}={X:0>8}, d={X:0>1}, d8={X:0>2}, d12={X:0>3}\n", .{ addr, opcode, sh4_disassembly.disassemble(instr, self._allocator) catch {
                std.debug.print("Failed to disassemble instruction {b:0>16}\n", .{opcode});
                unreachable;
            }, instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).*, instr.nmd.d, instr.nd8.d, instr.d12.d });

        desc.fn_(self, instr);

        if ((comptime builtin.mode == .Debug or builtin.mode == .ReleaseSafe) and self.debug_trace)
            std.debug.print("[{X:0>8}] {X: >16} {s: <20} R{d: <2}={X:0>8}, R{d: <2}={X:0>8}\n", .{ addr, opcode, "", instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).* });

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

        const chcr = self.read_p4_register(P4.CHCR, .CHCR2);

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
            else => @panic("Invalid transfer size"),
        };
        const len = self.read_p4_register(u32, .DMATCR2);
        const byte_len = transfer_size * len;

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
        self.p4_register(P4.CHCR, .CHCR2).*.te = 1;
    }

    fn panic_debug(self: @This(), comptime fmt: []const u8, args: anytype) noreturn {
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
                // NOTE: I think 0x02000000-0x03FFFFFF is a mirror of 0x00000000-0x01FFFFFF
                //       0x03000000-0x03FFFFFF is definitely a mirror of 0x01000000-0x01FFFFFF
                //       See page 232 of DreamcastDevBoxSystemArchitecture.pdf for the peripheral registers.
                const area_0_addr = addr & 0x01FFFFFF;
                switch (area_0_addr) {
                    0x00000000...0x001FFFFF => {
                        return &self._dc.?.boot[area_0_addr];
                    },
                    0x00200000...0x0021FFFF => {
                        return &self._dc.?.flash[area_0_addr - 0x200000];
                    },
                    0x005F6800...0x005F6FFF => {
                        return self._dc.?.hw_register_addr(u8, area_0_addr);
                    },
                    0x005F7000...0x005F709C => {
                        @panic("_get_memory to GDROM Register. This should be handled in read/write functions.");
                    },
                    0x005F709D...0x005F7FFF => {
                        return self._dc.?.hw_register_addr(u8, area_0_addr);
                    },
                    0x005F8000...0x005F9FFF => {
                        return self._dc.?.gpu._get_register_from_addr(u8, area_0_addr);
                    },
                    0x005FA000...0x005FFFFF => {
                        return self._dc.?.hw_register_addr(u8, area_0_addr);
                    },
                    0x00600000...0x006007FF => {
                        const static = struct {
                            var once = false;
                        };
                        if (!static.once) {
                            static.once = true;
                            sh4_log.warn(termcolor.yellow("  Unimplemented _get_memory to MODEM: {X:0>8} (This will only be reported once)"), .{addr});
                        }
                        self._dc.?._dummy = .{ 0, 0, 0, 0 };
                        return @ptrCast(&self._dc.?._dummy);
                    },
                    0x00700000...0x00707FFF => { // G2 AICA Register
                        @panic("_get_memory to AICA Register. This should be handled in read/write functions.");
                    },
                    0x00710000...0x00710008 => { // G2 AICA RTC Registers
                        @panic("_get_memory to AICA RTC Register. This should be handled in read/write functions.");
                    },
                    0x00800000...0x009FFFFF => { // G2 Wave Memory
                        @panic("_get_memory to AICA Wave Memory. This should be handled in read/write functions.");
                    },
                    0x01000000...0x01FFFFFF => { // Expansion Devices
                        sh4_log.warn(termcolor.yellow("  Unimplemented _get_memory to Expansion Devices: {X:0>8} ({X:0>8})"), .{ addr, area_0_addr });

                        // FIXME: TEMP DEBUG: Crazy Taxi accesses 030100C0 (010100C0) and 030100A0 (010100A0)
                        //        And 0101003C to 0101007C, and 01010014, and 01010008
                        // self.on_trapa.?();

                        // FIXME: I have no idea why Crazy Taxi seem to expect to find 0x80 at 01010008, but this lets it go further.
                        self._dc.?._dummy = .{ 0x80, 0, 0, 0 };

                        return @ptrCast(&self._dc.?._dummy);
                    },
                    else => {
                        sh4_log.warn(termcolor.yellow("  Unimplemented _get_memory to Area 0: {X:0>8} ({X:0>8})"), .{ addr, area_0_addr });

                        self._dc.?._dummy = .{ 0, 0, 0, 0 };
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
                    sh4_log.warn(termcolor.yellow("Unimplemented _get_memory to Area 5 (MODEM): {X:0>8} (This will only be reported once)"), .{addr});
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
                sh4_log.err(termcolor.red("Invalid _get_memory @{X:0>8}"), .{addr});
                @panic("Invalid _get_memory");
            },
        }
    }

    pub inline fn read8(self: @This(), virtual_addr: addr_t) u8 {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                else => {
                    sh4_log.debug("  Read8 to hardware register @{X:0>8} {s} = 0x{X:0>2}", .{ virtual_addr, P4.getP4RegisterName(virtual_addr), @as(*const u8, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
                    ))).* });
                },
            }
        } else if (virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF) {
            return self.read_operand_cache(u8, virtual_addr);
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            if (addr >= 0x005F7000 and addr <= 0x005F709C) {
                return self._dc.?.gdrom.read_register(u8, addr);
            } else {
                sh4_log.debug("  Read8 to hardware register @{X:0>8} {s} ", .{ addr, P4.getP4RegisterName(addr) });
            }
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
                @intFromEnum(P4Register.RFCR) => {
                    // Hack: This is the Refresh Count Register, related to DRAM control.
                    //       If don't think its proper emulation is needed, but it's accessed by the bios,
                    //       probably for synchronization purposes. I assume returning a contant value to pass this check
                    //       is enough for now, as games shouldn't access that themselves.
                    sh4_log.debug("[Note] Access to Refresh Count Register.", .{});
                    return 0x0011;
                    // Otherwise, this is 10-bits register, respond with the 6 unused upper bits set to 0.
                },
                @intFromEnum(P4Register.PDTRA) => {
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
                @intFromEnum(P4Register.PMCR1) => {
                    return 0;
                },
                @intFromEnum(P4Register.PMCR2) => {
                    return 0;
                },
                else => {
                    sh4_log.debug("  Read16 to P4 register @{X:0>8} {s} = {X:0>4}", .{ virtual_addr, P4.getP4RegisterName(virtual_addr), @as(*const u16, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
                    ))).* });
                },
            }
        } else if (virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF) {
            return self.read_operand_cache(u16, virtual_addr);
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            if (addr >= 0x005F7000 and addr <= 0x005F709C) {
                return self._dc.?.gdrom.read_register(u16, addr);
            } else {
                sh4_log.debug("  Read16 to hardware register @{X:0>8} {s} ", .{ virtual_addr, P4.getP4RegisterName(virtual_addr) });
            }
        }
        if (addr >= 0x00710000 and addr <= 0x00710008) {
            return @truncate(self._dc.?.aica.read_rtc_register(addr));
        }

        return @as(*const u16, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub noinline fn _out_of_line_read32(self: @This(), virtual_addr: addr_t) u32 {
        return read32(self, virtual_addr);
    }

    pub inline fn read32(self: @This(), virtual_addr: addr_t) u32 {
        const addr = virtual_addr & 0x1FFFFFFF;

        switch (virtual_addr) {
            0xFF000000...0xFFFFFFFF => {
                sh4_log.debug("  Read32 to P4 register @{X:0>8} {s} = 0x{X:0>8}", .{ virtual_addr, P4.getP4RegisterName(virtual_addr), @as(*const u32, @alignCast(@ptrCast(
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
                switch (addr) {
                    @intFromEnum(HardwareRegister.SB_ADSUSP), @intFromEnum(HardwareRegister.SB_E1SUSP), @intFromEnum(HardwareRegister.SB_E2SUSP), @intFromEnum(HardwareRegister.SB_DDSUSP) => {
                        // DMA status, always report transfer possible and not in progress.
                        //    Bit 5: DMA Request Input State
                        //      0: The DMA transfer request is high (transfer not possible), or bit 2 of the SB_ADTSEL register is "0"
                        //      1: The DMA transfer request is low (transfer possible)
                        //    Bit 4: DMA Suspend or DMA Stop
                        //      0: DMA transfer is in progress, or bit 2 of the SB_ADTSEL register is "0"
                        //      1: DMA transfer has ended, or is stopped due to a suspen
                        sh4_log.warn("  Read32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, P4.getP4RegisterName(addr), @as(*const u32, @alignCast(@ptrCast(
                            @constCast(&self)._get_memory(addr),
                        ))).* });
                        return 0x30;
                    },
                    else => {
                        sh4_log.debug("  Read32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, P4.getP4RegisterName(addr), @as(*const u32, @alignCast(@ptrCast(
                            @constCast(&self)._get_memory(addr),
                        ))).* });
                    },
                }
            },
            0x00700000...0x00707FE0 => {
                return self._dc.?.aica.read_register(u32, addr);
            },
            0x00710000...0x00710008 => {
                return self._dc.?.aica.read_rtc_register(addr);
            },
            0x00800000...0x00FFFFFF => {
                return self._dc.?.aica.read_mem(u32, addr);
            },
            else => {},
        }

        return @as(*const u32, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub noinline fn _out_of_line_read64(self: @This(), virtual_addr: addr_t) u64 {
        return read64(self, virtual_addr);
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
                @intFromEnum(P4Register.SCFTDR2) => {
                    sh4_log.warn(termcolor.yellow("Write8 to non-implemented P4 register SCFTDR2: 0x{X:0>2}={c}."), .{ value, value });
                    // Immediately mark transfer as complete.
                    //   Or rather, attempts to, this is not enough.
                    const SCFSR2 = self.p4_register(HardwareRegisters.SCFSR2, .SCFSR2);
                    SCFSR2.*.tend = 1;
                    // FIXME: The serial interface is not implemented at all.
                    return;
                },
                else => {
                    sh4_log.debug("  Write8 to P4 register @{X:0>8} {s} = 0x{X:0>2}", .{ virtual_addr, P4.getP4RegisterName(virtual_addr), value });
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
                0x005F7000...0x005F709C => {
                    return self._dc.?.gdrom.write_register(u8, addr, value);
                },
                else => {
                    sh4_log.debug("  Write8 to hardware register @{X:0>8} {s} = 0x{X:0>2}", .{ addr, P4.getP4RegisterName(addr), value });
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
                @intFromEnum(P4Register.RTCSR), @intFromEnum(P4Register.RTCNT), @intFromEnum(P4Register.RTCOR) => {
                    std.debug.assert(value & 0xFF00 == 0b10100101_00000000);
                    self.p4_register_addr(u16, addr).* = (value & 0xFF);
                    return;
                },
                @intFromEnum(P4Register.RFCR) => {
                    std.debug.assert(value & 0b11111100_00000000 == 0b10100100_00000000);
                    self.p4_register_addr(u16, addr).* = (value & 0b11_11111111);
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
                    // Writable bits can only be cleared.
                    self.p4_register(u16, .SCFSR2).* &= (value | 0b11111111_00001100);
                    return;
                },
                else => {
                    sh4_log.debug("  Write16 to P4 register @{X:0>8} {s} = 0x{X:0>4}", .{ virtual_addr, P4.getP4RegisterName(virtual_addr), value });
                },
            }
        } else if (virtual_addr >= 0x7C000000 and virtual_addr <= 0x7FFFFFFF) {
            self.operand_cache(u16, virtual_addr).* = value;
            return;
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            switch (addr) {
                0x005F7000...0x005F70A0 => {
                    return self._dc.?.gdrom.write_register(u16, addr, value);
                },
                else => {
                    sh4_log.debug("  Write16 to hardware register @{X:0>8} {s} = 0x{X:0>4}", .{ addr, P4.getP4RegisterName(addr), value });
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

    pub noinline fn _out_of_line_write32(self: *@This(), virtual_addr: addr_t, value: u32) void {
        write32(self, virtual_addr, value);
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
                        sh4_log.debug("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ virtual_addr, P4.getP4RegisterName(virtual_addr), value });
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
                const reg: HardwareRegister = @enumFromInt(addr);
                // Hardware registers
                switch (reg) {
                    .SB_SFRES => {
                        // SB_SFRES, Software Reset
                        if (value == 0x00007611) {
                            self.software_reset();
                        }
                        return;
                    },
                    .SB_ADST => {
                        if (value == 1) {
                            self._dc.?.aica.start_dma(self._dc.?);
                            return;
                        }
                    },
                    .SB_E1ST, .SB_E2ST, .SB_DDST, .SB_SDST, .SB_PDST => {
                        if (value == 1) {
                            sh4_log.warn(termcolor.yellow("Unimplemented {any} DMA initiation!"), .{reg});
                            return;
                        }
                    },
                    .SB_GDST => {
                        if (value == 1) {
                            sh4_log.err(termcolor.red("Unimplemented {any} DMA (ch0-DMA) initiation!"), .{reg});
                            return;
                        }
                    },
                    .SB_MDAPRO => {
                        // This register specifies the address range for Maple-DMA involving the system (work) memory.
                        // Check "Security code"
                        if (value & 0xFFFF0000 != 0x61550000) return;
                    },
                    .SB_MDST => {
                        if (value == 1) {
                            self._dc.?.start_maple_dma();
                            return;
                        }
                    },
                    .SB_ISTNRM => {
                        // Interrupt can be cleared by writing "1" to the corresponding bit.
                        self._dc.?.hw_register(u32, .SB_ISTNRM).* &= ~(value & 0x3FFFFF);
                        return;
                    },
                    .SB_ISTERR => {
                        // Interrupt can be cleared by writing "1" to the corresponding bit.
                        self._dc.?.hw_register(u32, .SB_ISTERR).* &= ~value;
                        return;
                    },
                    .SB_C2DSTAT => {
                        self._dc.?.hw_register(u32, .SB_C2DSTAT).* = 0x10000000 | (0x03FFFFFF & value);
                        return;
                    },
                    .SB_C2DST => {
                        if (value == 1) {
                            self._dc.?.start_ch2_dma();
                        } else {
                            self._dc.?.end_ch2_dma();
                        }
                        return;
                    },
                    else => {
                        sh4_log.debug("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}", .{ addr, P4.getP4RegisterName(addr), value });
                    },
                }
            },
            0x005F8000...0x005F9FFF => {
                return self._dc.?.gpu.write_register(addr, value);
            },
            0x00700000...0x0070FFFF => {
                return self._dc.?.aica.write_register(u32, addr, value);
            },
            0x00710000...0x00710008 => {
                return self._dc.?.aica.write_rtc_register(addr, value);
            },
            0x00800000...0x00FFFFFF => {
                return self._dc.?.aica.write_mem(u32, addr, value);
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

    pub noinline fn _out_of_line_write64(self: *@This(), virtual_addr: addr_t, value: u64) void {
        write64(self, virtual_addr, value);
    }

    pub inline fn write64(self: *@This(), virtual_addr: addr_t, value: u64) void {
        // This isn't efficient, but avoids repeating all the logic of write32.
        self.write32(virtual_addr, @truncate(value));
        self.write32(virtual_addr + 4, @truncate(value >> 32));
    }

    // MMU Stub functions
    // NOTE: This is dead code, the MMU is not emulated and utlb_entries are not in this struct anymore (reducing the size of the struct helps a lot with performance).

    pub fn mmu_utlb_match(self: @This(), virtual_addr: addr_t) !mmu.UTLBEntry {
        const asid = self.dc.read_p4_register(mmu.PTEH, HardwareRegister.PTEH).asid;
        const vpn: u22 = @truncate(virtual_addr >> 10);

        const shared_access = self.dc.read_p4_register(mmu.MMUCR, HardwareRegister.MMUCR).sv == 0 or self.sr.md == 0;
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
        if (self.dc.read_p4_register(mmu.MMUCR, HardwareRegister.MMUCR).at == 0) return virtual_addr;

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
