// Hitachi SH-4

const std = @import("std");
const common = @import("./common.zig");
const mmu = @import("./mmu.zig");
const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;
const Interrupts = @import("Interrupts.zig");
const Interrupt = Interrupts.Interrupt;

const addr_t = u32;
const byte_t = u8;
const word_t = u16;
const longword_t = u32;

const SR = packed struct {
    t: bool = undefined, // True/False condition or carry/borrow bit.
    s: bool = undefined, // Specifies a saturation operation for a MAC instruction

    _r0: u2 = undefined,

    imask: u4 = 1, // Interrupt mask level
    q: bool = undefined, // State for divide step.
    m: bool = undefined,

    _r1: u5 = undefined,

    fd: bool = false, // FPU disable bit.

    _r2: u12 = undefined,

    bl: bool = true, // Exception/interrupt block bit (set to 1 by a reset, exception, or interrupt).
    rb: u1 = 1, // General register bank specifier in privileged mode (set to 1 by a reset, exception or interrupt).
    md: u1 = 1, // Processor mode. MD = 0: User mode (Some instructions cannot be executed, and some resources cannot be accessed). MD = 1: Privileged mode.

    _r3: u1 = undefined,
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

pub fn is_p0(addr: addr_t) bool {
    return (addr & 0b11100000000000000000000000000000) == 0b00000000000000000000000000000000;
}
pub fn is_p1(addr: addr_t) bool {
    return (addr & 0b11100000000000000000000000000000) == 0b10000000000000000000000000000000;
}
pub fn is_p2(addr: addr_t) bool {
    return (addr & 0b11100000000000000000000000000000) == 0b10100000000000000000000000000000;
}
pub fn is_p3(addr: addr_t) bool {
    return (addr & 0b11100000000000000000000000000000) == 0b11000000000000000000000000000000;
}
pub fn is_p4(addr: addr_t) bool {
    return (addr & 0b11100000000000000000000000000000) == 0b11100000000000000000000000000000;
}

const Instr = packed union {
    value: u16,
    // Reminder: We're in little endian.
    nmd: packed struct { d: u4, m: u4, n: u4, _: u4 },
    nd8: packed struct { d: u8, n: u4, _: u4 },
    d12: packed struct { d: u12, _: u4 },
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
    execution_state: ExecutionState = .Running,

    // General Registers
    r_bank0: [8]u32 = undefined, // R0-R7_BANK0
    r_bank1: [8]u32 = undefined, // R0-R7_BANK1
    r_8_15: [8]u32 = undefined,

    // Control Registers
    sr: SR = .{},
    gbr: u32 = undefined,
    ssr: u32 = undefined,
    spc: u32 = undefined,
    sgr: u32 = undefined,
    dbr: u32 = undefined,
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

    utlb_entries: [64]mmu.UTLBEntry = undefined,

    boot: []u8 align(4) = undefined,
    flash: []u8 align(4) = undefined,
    ram: []u8 align(4) = undefined,
    area7: []u8 align(4) = undefined,
    hardware_registers: [0x10000]u8 align(4) = undefined, // FIXME

    interrupt_requests: u64 = 0,

    debug_trace: bool = false,

    pub fn init(self: *@This()) !void {
        self.area7 = try common.GeneralAllocator.alloc(u8, 64 * 1024 * 1024);
        self.ram = try common.GeneralAllocator.alloc(u8, 16 * 1024 * 1024);

        // Load ROM
        self.boot = try common.GeneralAllocator.alloc(u8, 0x200000);
        var boot_file = try std.fs.cwd().openFile("./bin/dc_boot.bin", .{});
        defer boot_file.close();
        const bytes_read = try boot_file.readAll(self.boot);
        std.debug.assert(bytes_read == 0x200000);

        // Load Flash
        self.flash = try common.GeneralAllocator.alloc(u8, 0x20000);
        var flash_file = try std.fs.cwd().openFile("./bin/dc_flash.bin", .{});
        defer flash_file.close();
        const flash_bytes_read = try flash_file.readAll(self.flash);
        std.debug.assert(flash_bytes_read == 0x20000);

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

        self.reset();
    }

    pub fn deinit(self: *@This()) void {
        common.GeneralAllocator.free(self.flash);
        common.GeneralAllocator.free(self.boot);
        common.GeneralAllocator.free(self.ram);
    }

    pub fn reset(self: *@This()) void {
        self.r_bank0 = undefined;
        self.r_bank1 = undefined;
        self.r_8_15 = undefined;
        self.sr = .{};
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

        self.io_register(mmu.PTEH, MemoryRegister.PTEH).* = .{};
        self.io_register(mmu.PTEL, MemoryRegister.PTEL).* = .{};
        self.io_register(u32, MemoryRegister.TTB).* = 0;
        self.io_register(u32, MemoryRegister.TEA).* = 0;
        self.io_register(u32, MemoryRegister.MMUCR).* = 0;

        self.io_register(u8, MemoryRegister.BASRA).* = undefined;
        self.io_register(u8, MemoryRegister.BASRB).* = undefined;
        self.io_register(u32, MemoryRegister.CCR).* = 0;

        self.io_register(u32, MemoryRegister.TRA).* = 0;
        self.io_register(u32, MemoryRegister.EXPEVT).* = 0;
        self.io_register(u32, MemoryRegister.INTEVT).* = 0;

        self.io_register(u32, MemoryRegister.QACR0).* = undefined;
        self.io_register(u32, MemoryRegister.QACR1).* = undefined;
        self.io_register(u32, MemoryRegister.BARA).* = undefined;
        self.io_register(u32, MemoryRegister.BAMRA).* = undefined;

        self.io_register(u32, MemoryRegister.MCR).* = 0xC0091224;
        self.io_register(u16, MemoryRegister.SDMR).* = 0x00FF;
        self.io_register(u16, MemoryRegister.RTCSR).* = 0xA510;
        self.io_register(u16, MemoryRegister.RTCNT).* = 0xA500;
        self.io_register(u16, MemoryRegister.RTCOR).* = 0xA55E;
        self.io_register(u16, MemoryRegister.RFCR).* = undefined;

        self.io_register(u32, MemoryRegister.BCR1).* = 0;
        self.io_register(u32, MemoryRegister.BCR2).* = 0x3FFC;
    }

    pub fn read_io_register(self: @This(), comptime T: type, r: MemoryRegister) T {
        return @as(*T, @alignCast(@ptrCast(&self.area7[(@intFromEnum(r) & 0x1FFFFFFF) - 0x1C000000]))).*;
    }

    pub fn io_register(self: *@This(), comptime T: type, r: MemoryRegister) *T {
        return @as(*T, @alignCast(@ptrCast(&self.area7[(@intFromEnum(r) & 0x1FFFFFFF) - 0x1C000000])));
    }

    pub fn io_register_addr(self: *@This(), comptime T: type, addr: addr_t) *T {
        return @as(*T, @alignCast(@ptrCast(&self.area7[(addr & 0x1FFFFFFF) - 0x1C000000])));
    }

    pub fn software_reset(self: *@This()) void {
        std.debug.print("  SOFTWARE RESET!\n", .{});

        self.r_bank0 = undefined;
        self.r_bank1 = undefined;
        self.r_8_15 = undefined;
        self.sr = .{};
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

        self.io_register(mmu.PTEH, MemoryRegister.PTEH).* = .{};
        self.io_register(mmu.PTEL, MemoryRegister.PTEL).* = .{};
        self.io_register(u32, MemoryRegister.TTB).* = 0;
        self.io_register(u32, MemoryRegister.TEA).* = 0;
        self.io_register(u32, MemoryRegister.MMUCR).* = 0;
        // BASRA Held
        // BASRB Held
        self.io_register(u32, MemoryRegister.CCR).* = 0;
        self.io_register(u32, MemoryRegister.TRA).* = 0;
        self.io_register(u32, MemoryRegister.EXPEVT).* = 0x00000020;
        // INTEVT Held
        self.io_register(u32, MemoryRegister.QACR0).* = undefined;
        self.io_register(u32, MemoryRegister.QACR1).* = undefined;
        // BARA Held
        // BAMRA Held

        // BCR1 Held
        // BCR2 Held
    }

    pub fn R(self: *@This(), r: u4) *u32 {
        if (r >= 8) return &self.r_8_15[r - 8];
        // Note: Rather than checking all the time, we could swap only once
        //       when SR is updated.
        if (self.sr.md == 1 and self.sr.rb == 1) return &self.r_bank1[r];
        return &self.r_bank0[r];
    }

    pub fn FR(self: *@This(), r: u4) *f32 {
        return &self.fp_banks[self.fpscr.fr].fr[r];
    }

    pub fn DR(self: *@This(), r: u4) *f64 {
        std.debug.assert(r < 8);
        return &self.fp_banks[self.fpscr.fr].dr[r];
    }

    pub fn QR(self: *@This(), r: u4) *f32 {
        std.debug.assert(r < 4);
        return &self.fp_banks[self.fpscr.fr].qr[r];
    }

    pub fn execute(self: *@This()) void {
        // TODO: Check interrupts.

        // When the BL bit in SR is 0, exceptions and interrupts are accepted.

        // See h14th002d2.pdf page 665 (or 651)
        if (!self.sr.bl or self.execution_state != .Running) {
            var interrupt_or_exception = false;

            var offset: u32 = 0; // TODO: Get the offset corresponding to the code

            if (self.interrupt_requests != 0) {
                // TODO: Search the highest priority interrupt.
                const first_set = @ctz(self.interrupt_requests);
                self.interrupt_requests &= ~@as(u64, 1) << @truncate(first_set); // Clear the request
                self.io_register(u32, MemoryRegister.INTEVT).* = Interrupts.InterruptINTEVTCodes[first_set];
                interrupt_or_exception = true;
                offset = 0x600;

                self.debug_trace = true;
            } else if (false) {
                // TODO: Check for exceptions

                // self.io_register(u32, MemoryRegister.EXPEVT).* = code;
            }

            if (interrupt_or_exception) {
                self.execution_state = .Running;
                // 1. The PC and SR contents are saved in SPC and SSR.
                // 2. The block bit (BL) in SR is set to 1.
                // 3. The mode bit (MD) in SR is set to 1.
                // 4. The register bank bit (RB) in SR is set to 1.
                // 5. In a reset, the FPU disable bit (FD) in SR is cleared to 0.
                // 6. The exception code is written to bits 11–0 of the exception event register (EXPEVT) or
                // interrupt event register (INTEVT).
                // 7. The CPU branches to the determined exception handling vector address, and the exception
                // handling routine begins.
                self.spc = self.pc;
                self.ssr = @bitCast(self.sr);
                self.sr.bl = true;
                self.sr.md = 1;
                self.sr.rb = 1;

                // self.io_register(u32, MemoryRegister.EXPEVT).* = code;
                // or
                // self.io_register(u32, MemoryRegister.INTEVT).* = code;

                const UserBreak = false; // TODO
                if (self.read_io_register(MemoryRegisters.BRCR, MemoryRegister.BRCR).ubde == 1 and UserBreak) {
                    self.pc = self.dbr;
                } else {
                    self.pc = self.vbr + offset;
                }
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
        } else {
            // FIXME: Not sure if this is a thing.
            self.advance_timers(1);

            // FIXME: Just testing things.
            self.request_interrupt(Interrupt.TUNI0);
        }
    }

    fn request_interrupt(self: *@This(), int: Interrupt) void {
        std.debug.print("Interrupt request! {any}\n", .{int});
        self.interrupt_requests |= @as(u33, 1) << @intFromEnum(int);
    }

    pub fn advance_timers(self: *@This(), cycles: u32) void {
        // TODO: Proper timer.
        const TSTR = self.read_io_register(MemoryRegisters.TSTR, MemoryRegister.TSTR);
        // TODO: Scale depending on Timer Control Registers' TPSC values

        // When one of bits STR0–STR2 is set to 1 in the timer start register (TSTR), the timer counter
        // (TCNT) for the corresponding channel starts counting. When TCNT underflows, the UNF flag is
        // set in the corresponding timer control register (TCR). If the UNIE bit in TCR is set to 1 at this
        // time, an interrupt request is sent to the CPU. At the same time, the value is copied from TCOR
        // into TCNT, and the count-down continues (auto-reload function).
        if (TSTR.str0 == 1) {
            const tcnt = self.io_register(u32, MemoryRegister.TCNT0);
            const tcr = self.io_register(MemoryRegisters.TCR, MemoryRegister.TCR0);
            if (tcnt.* < cycles) {
                tcr.*.unf = 1;
                tcnt.* = self.io_register(u32, MemoryRegister.TCOR0).*;
                if (tcr.*.unie == 1)
                    self.request_interrupt(Interrupt.TUNI0);
            } else tcnt.* -= cycles;
        }
        if (TSTR.str1 == 1) {
            const tcnt = self.io_register(u32, MemoryRegister.TCNT1);
            const tcr = self.io_register(MemoryRegisters.TCR, MemoryRegister.TCR1);
            if (tcnt.* < cycles) {
                tcr.*.unf = 1;
                tcnt.* = self.io_register(u32, MemoryRegister.TCOR1).*;
                if (tcr.*.unie == 1)
                    self.request_interrupt(Interrupt.TUNI1);
            } else tcnt.* -= cycles;
        }
        if (TSTR.str2 == 1) {
            const tcnt = self.io_register(u32, MemoryRegister.TCNT2);
            const tcr = self.io_register(MemoryRegisters.TCR, MemoryRegister.TCR2);
            if (tcnt.* < cycles) {
                tcr.*.unf = 1;
                tcnt.* = self.io_register(u32, MemoryRegister.TCOR2).*;
                if (tcr.*.unie == 1)
                    self.request_interrupt(Interrupt.TUNI2);
            } else tcnt.* -= cycles;
        }
    }

    pub fn _execute(self: *@This(), addr: addr_t) void {
        const opcode = self.read16(addr);
        const instr = Instr{ .value = opcode };
        if (self.debug_trace)
            std.debug.print("[{X:0>4}] {b:0>16} {s: <20}\tR{d: <2}={X:0>8}, R{d: <2}={X:0>8}, d={X:0>8}\n", .{ addr, opcode, Opcodes[JumpTable[opcode]].name, instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).*, instr.nmd.d });
        Opcodes[JumpTable[opcode]].fn_(self, instr);
        if (self.debug_trace)
            std.debug.print("[{X:0>4}] {X: >16} {s: <20}\tR{d: <2}={X:0>8}, R{d: <2}={X:0>8}\n", .{ addr, opcode, "", instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).* });

        self.advance_timers(Opcodes[JumpTable[opcode]].issue_cycles);
    }

    pub fn mmu_utlb_match(self: @This(), virtual_addr: addr_t) !mmu.UTLBEntry {
        const asid = self.read_io_register(mmu.PTEH, MemoryRegister.PTEH).asid;
        const vpn: u22 = @truncate(virtual_addr >> 10);

        const shared_access = self.read_io_register(mmu.MMUCR, MemoryRegister.MMUCR).sv == 0 or self.sr.md == 0;
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
        std.debug.assert(is_p0(virtual_addr) or is_p3(virtual_addr));
        if (self.read_io_register(mmu.MMUCR, MemoryRegister.MMUCR).at == 0) return virtual_addr;

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

    pub fn _read(self: @This(), virtual_addr: addr_t) *const u8 {
        if (is_p0(virtual_addr)) {
            const physical_addr = self.mmu_translate_utbl(virtual_addr) catch |e| {
                // FIXME: Handle exceptions
                std.debug.print("Error in utlb _read: {any} at {X:0>8}\n", .{ e, virtual_addr });
                unreachable;
            };

            if (physical_addr != virtual_addr)
                std.debug.print("  UTLB Hit: {x:0>8} => {x:0>8}\n", .{ virtual_addr, physical_addr });

            if (physical_addr < 0x04000000) {
                // Area 0 - Boot ROM, Flash ROM, Hardware Registers
                if (physical_addr < 0x200000) {
                    return &self.boot[physical_addr];
                } else if (physical_addr < 0x200000 + 0x20000) {
                    return &self.flash[physical_addr - 0x200000];
                }
                return &self.hardware_registers[physical_addr - 0x005F6800];
            } else if (physical_addr < 0x08000000) {
                // Area 1 - Video RAM
                std.debug.print("Invalid _read, Area 1: {X:0>8}\n", .{physical_addr});
                unreachable;
            } else if (physical_addr < 0x0C000000) {
                // Area 2 - Nothing
                std.debug.print("Invalid _read, Area 2: {X:0>8}\n", .{physical_addr});
                unreachable;
            } else if (physical_addr < 0x10000000) {
                // Area 3 - System RAM (16MB)
                return &self.ram[(physical_addr - 0x0C000000) % self.ram.len];
            } else if (physical_addr < 0x14000000) {
                // Area 4 - Tile accelerator command input
                std.debug.print("Unimplemented _read, Area 4: {X:0>8}\n", .{physical_addr});
                unreachable;
            } else if (physical_addr < 0x18000000) {
                // Area 5 - Expansion (modem) port
                std.debug.print("Invalid _read, Area 5: {X:0>8}\n", .{virtual_addr});
                unreachable;
            } else if (physical_addr < 0x1C000000) {
                // Area 6 - Nothing
                std.debug.print("Invalid _read, Area 6: {X:0>8}\n", .{physical_addr});
                unreachable;
            } else {
                // Area 7 - Internal I/O registers (same as P4)
                std.debug.assert(self.sr.md == 1);
                return &self.area7[physical_addr - 0x1C000000];
            }
        } else if (is_p1(virtual_addr)) {
            // TODO: Cache control, although it's probably not needed.
            // !CCR.OCE? => P2
            // !CCR.CB? => Cache access in write-through mode
            //     else => Cache access in copy-back mode
            return self._read(virtual_addr & 0x01FFFFFFF);
        } else if (is_p2(virtual_addr)) {
            return self._read(virtual_addr & 0x01FFFFFFF);
        } else if (is_p3(virtual_addr)) {
            std.debug.assert(self.sr.md == 1);
            return self._read(virtual_addr & 0x01FFFFFFF);
        } else if (is_p4(virtual_addr)) {
            std.debug.assert(self.sr.md == 1);
            return self._read(virtual_addr & 0x01FFFFFFF);
        } else {
            unreachable;
        }
    }

    pub fn _write(self: *@This(), virtual_addr: addr_t) *u8 {
        if (is_p0(virtual_addr)) {
            if (virtual_addr < 0x04000000) {
                if (virtual_addr < 0x005F6800)
                    unreachable;
                // FIXME: Unimplemented
                return &self.hardware_registers[virtual_addr - 0x005F6800];
            } else if (virtual_addr < 0x08000000) {
                // Area 1 - Video RAM
                unreachable;
            } else if (virtual_addr < 0x0C000000) {
                // Area 2 - Nothing
                unreachable;
            } else if (virtual_addr < 0x10000000) {
                // Area 3 - System RAM (16MB)
                return &self.ram[(virtual_addr - 0x0C000000) % self.ram.len];
            } else if (virtual_addr < 0x14000000) {
                // Area 4 - Tile accelerator command input
                unreachable;
            } else if (virtual_addr < 0x18000000) {
                // Area 5 - Expansion (modem) port
                unreachable;
            } else if (virtual_addr < 0x1C000000) {
                // Area 6 - Nothing
                unreachable;
            } else {
                // Area 7 - Internal I/O registers (same as P4)
                std.debug.assert(self.sr.md == 1);
                return &self.area7[virtual_addr - 0x1C000000];
            }
        } else if (is_p1(virtual_addr)) {
            std.debug.assert(self.sr.md == 1);
            return self._write(virtual_addr & 0x01FFFFFFF);
        } else if (is_p2(virtual_addr)) {
            std.debug.assert(self.sr.md == 1);
            return self._write(virtual_addr & 0x01FFFFFFF);
        } else if (is_p3(virtual_addr)) {
            std.debug.assert(self.sr.md == 1);
            return self._write(virtual_addr & 0x01FFFFFFF);
        } else if (is_p4(virtual_addr)) {
            std.debug.assert(self.sr.md == 1);
            return self._write(virtual_addr & 0x01FFFFFFF);
        } else {
            unreachable;
        }
    }

    pub fn read8(self: @This(), virtual_addr: addr_t) u8 {
        return @as(*const u8, @alignCast(@ptrCast(
            self._read(virtual_addr),
        ))).*;
    }

    pub fn read16(self: @This(), virtual_addr: addr_t) u16 {
        switch (virtual_addr) {
            @intFromEnum(MemoryRegister.RTCSR), @intFromEnum(MemoryRegister.RTCNT), @intFromEnum(MemoryRegister.RTCOR) => {
                return @as(*const u16, @alignCast(@ptrCast(
                    self._read(virtual_addr),
                ))).* & 0xF;
            },
            @intFromEnum(MemoryRegister.RFCR) => {
                // Hack: This is the Refresh Count Register, related to DRAM control.
                //       If don't think its proper emulation is needed, but it's accessed by the bios,
                //       probably for synchronization purposes. I assume returning a contant value to pass this check
                //       is enough for now, as games shouldn't access that themselves.
                std.debug.print("[Note] Access to Refresh Count Register.\n", .{});
                return 0x0011;
                // Otherwise, this is 10-bits register, respond with the 6 unused upper bits set to 0.
            },
            else => {},
        }

        return @as(*const u16, @alignCast(@ptrCast(
            self._read(virtual_addr),
        ))).*;
    }

    pub fn read32(self: @This(), virtual_addr: addr_t) u32 {
        return @as(*const u32, @alignCast(@ptrCast(
            self._read(virtual_addr),
        ))).*;
    }

    pub fn read64(self: @This(), virtual_addr: addr_t) u64 {
        return @as(*const u64, @alignCast(@ptrCast(
            self._read(virtual_addr),
        ))).*;
    }

    pub fn write8(self: *@This(), virtual_addr: addr_t, value: u8) void {
        const addr = virtual_addr & 0x01FFFFFFF;
        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            // Hardware registers
            switch (addr) {
                else => {
                    std.debug.print("  Write8 to hardware register @{X:0>8} = 0x{X:0>2}\n", .{ addr, value });
                    std.debug.print("                             {s} = 0x{X:0>2}\n", .{ @tagName(@as(MemoryRegister, @enumFromInt(addr))), value });
                },
            }
        }

        @as(*u8, @alignCast(@ptrCast(
            self._write(virtual_addr),
        ))).* = value;
    }

    pub fn write16(self: *@This(), virtual_addr: addr_t, value: u16) void {
        // TODO: Check region/permissions?
        switch (virtual_addr) {
            @intFromEnum(MemoryRegister.RTCSR), @intFromEnum(MemoryRegister.RTCNT), @intFromEnum(MemoryRegister.RTCOR) => {
                std.debug.assert(value & 0xFF00 == 0b10100101_00000000);
                @as(*u16, @alignCast(@ptrCast(
                    self._write(virtual_addr),
                ))).* = 0b10100101_00000000 | (value & 0xFF);
            },
            @intFromEnum(MemoryRegister.RFCR) => {
                std.debug.assert(value & 0b11111100_00000000 == 0b10100100_00000000);
                @as(*u16, @alignCast(@ptrCast(
                    self._write(virtual_addr),
                ))).* = 0b10100100_00000000 | (value & 0b11_11111111);
            },
            else => {},
        }
        const addr = virtual_addr & 0x01FFFFFFF;
        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            // Hardware registers
            switch (addr) {
                else => {
                    std.debug.print("  Write16 to hardware register @{X:0>8} = 0x{X:0>4}\n", .{ addr, value });
                    std.debug.print("                               {s} = 0x{X:0>4}\n", .{ @tagName(@as(MemoryRegister, @enumFromInt(addr))), value });
                },
            }
        }

        @as(*u16, @alignCast(@ptrCast(
            self._write(virtual_addr),
        ))).* = value;
    }

    pub fn write32(self: *@This(), virtual_addr: addr_t, value: u32) void {
        // Addresses with side effects
        // TODO: Check region/permissions?
        const addr = virtual_addr & 0x01FFFFFFF;
        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            // Hardware registers
            switch (addr) {
                0x005F6890 => {
                    // SB_SFRES, Software Reset
                    if (value == 0x00007611) {
                        self.software_reset();
                    }
                    return;
                },
                else => {
                    std.debug.print("  Write32 to hardware register @{X:0>8} = 0x{X:0>8}\n", .{ addr, value });
                    std.debug.print("                               {s} = 0x{X:0>8}\n", .{ @tagName(@as(MemoryRegister, @enumFromInt(addr))), value });
                },
            }
        }

        @as(*u32, @alignCast(@ptrCast(
            self._write(virtual_addr),
        ))).* = value;
    }

    pub fn write64(self: *@This(), virtual_addr: addr_t, value: u64) void {
        @as(*u64, @alignCast(@ptrCast(
            self._write(virtual_addr),
        ))).* = value;
    }

    pub fn prefetch_operand_cache_block(self: *@This(), virtual_addr: addr_t) void {
        _ = self;
        // TODO: (Rn) → operand cache
        std.debug.print("prefetch_operand_cache_block: {X:0>8}\n", .{virtual_addr});
    }
};

fn sign_extension_u8(d: u8) i32 {
    if ((d & 0x80) == 0) {
        return @bitCast(0x000000FF & @as(u32, @intCast(d)));
    } else {
        return @bitCast(0xFFFFFF00 | @as(u32, @intCast(d)));
    }
}

fn sign_extension_u12(d: u12) i32 {
    if ((d & 0x800) == 0) {
        return @bitCast(0x00000FFF & @as(u32, @intCast(d)));
    } else {
        return @bitCast(0xFFFFF000 | @as(u32, @intCast(d)));
    }
}

fn sign_extension_u16(d: u16) i32 {
    if ((d & 0x8000) == 0) {
        return @bitCast(0x0000FFFF & @as(u32, @intCast(d)));
    } else {
        return @bitCast(0xFFFF0000 | @as(u32, @intCast(d)));
    }
}

fn zero_extend(d: anytype) u32 {
    return @intCast(d);
}

fn as_i32(val: u32) i32 {
    return @bitCast(val);
}

fn unknown(_: *SH4, _: Instr) void {
    @panic("Unknown opcode");
}

fn nop(_: *SH4, _: Instr) void {}

fn unimplemented(_: *SH4, _: Instr) void {
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
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
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

fn movb_at_dispRm_R0(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

// The 4-bit displacement is multiplied by two after zero-extension, enabling a range up to +30 bytes to be specified.
// The loaded data is sign-extended to 32 bit before being stored in the destination register.
fn movw_at_dispRm_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = cpu.read16(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 1));
}

fn movl_at_dispRm_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* = cpu.read32(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 2));
}

// Transfers the source operand to the destination. The 4-bit displacement is multiplied by four after zero-extension, enabling a range up to +60 bytes to be specified.
fn movl_atdispRm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.read32(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 2));
}

fn movw_atdispRm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

// The 4-bit displacement is only zero-extended, so a range up to +15 bytes can be specified.
fn movb_R0_at_dispRn(cpu: *SH4, opcode: Instr) void {
    //                Yes, it's m.
    cpu.write8(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d)), @truncate(cpu.R(0).*));
}
// The 4-bit displacement is multiplied by two after zero-extension, enabling a range up to +30 bytes to be specified.
fn movw_R0_at_dispRn(cpu: *SH4, opcode: Instr) void {
    //                Yes, it's m.
    cpu.write16(cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 1), @truncate(cpu.R(0).*));
}

// Transfers the source operand to the destination.
// The 4-bit displacement is multiplied by four after zero-extension, enabling a range up to +60 bytes to be specified.
fn movl_Rm_atdispRn(cpu: *SH4, opcode: Instr) void {
    const d = zero_extend(opcode.nmd.d) << 2;
    cpu.write32(cpu.R(opcode.nmd.n).* + d, cpu.R(opcode.nmd.m).*);
}

fn movb_atR0Rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(cpu.read8(cpu.R(opcode.nmd.m).* + cpu.R(0).*)));
}

fn movw_atR0Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(cpu.read16(cpu.R(opcode.nmd.m).* + cpu.R(0).*)));
}

fn movl_atR0Rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.read32(cpu.R(opcode.nmd.m).* + cpu.R(0).*);
}

fn movb_Rm_atR0Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn movw_Rm_atR0Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn movl_Rm_atR0Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
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
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
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

fn xtrct_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

fn add_rm_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* +%= cpu.R(opcode.nmd.m).*;
}

fn add_imm_rn(cpu: *SH4, opcode: Instr) void {
    // FIXME: Not such about the signed arithmetic here.
    cpu.R(opcode.nd8.n).* = @bitCast(@as(i32, @bitCast(cpu.R(opcode.nd8.n).*)) + sign_extension_u8(opcode.nd8.d));
}

fn addc_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

fn addv_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

// Compares general register R0 and the sign-extended 8-bit immediate data and sets the T bit if the values are equal.
// If they are not equal the T bit is cleared. The contents of R0 are not changed.
fn cmpeq_imm_r0(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = (cpu.R(0).* == sign_extension_u8(opcode.nd8.d));
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

fn div0s_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

fn div0u_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

fn div0u(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

fn div1(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

// Decrements the contents of general register Rn by 1 and compares the result with zero.
// If the result is zero, the T bit is set to 1. If the result is nonzero, the T bit is cleared to 0.
fn dt_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* -% 1;
    cpu.sr.t = (cpu.R(opcode.nmd.n).* == 0);
}

fn extsb_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn extsw_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn extub_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn extuw_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

fn mull_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn mulsw_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

// Performs 16-bit multiplication of the contents of general registers Rn and Rm, and stores the 32-bit result in the MACL register.
// The multiplication is performed as an unsigned arithmetic operation. The contents of MACH are not changed
fn muluwRmRn(cpu: *SH4, opcode: Instr) void {
    cpu.macl = @as(u32, @intCast(@as(u16, @truncate(cpu.R(opcode.nmd.n).*)))) * @as(u32, @intCast(@as(u16, @truncate(cpu.R(opcode.nmd.m).*))));
}

fn neg_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

fn sub_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn subc_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

fn and_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* &= cpu.R(opcode.nmd.m).*;
}
fn and_imm_R0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* &= zero_extend(opcode.nd8.d);
}

fn or_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* |= cpu.R(opcode.nmd.m).*;
}
fn or_imm_r0(cpu: *SH4, opcode: Instr) void {
    cpu.R(0).* |= zero_extend(opcode.nd8.d);
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
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn rotcr_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn rotl_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 0x80000000) == 1);
    cpu.R(opcode.nmd.n).* = std.math.rotl(u32, cpu.R(opcode.nmd.n).*, 1);
}
fn rotr_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 1) == 1);
    cpu.R(opcode.nmd.n).* = std.math.rotr(u32, cpu.R(opcode.nmd.n).*, 1);
}

fn shad_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
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
        cpu.R(opcode.nmd.n).* &= 0x7fffffff;
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
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* << 2;
}
fn shll8(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* << 8;
}
fn shll16(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* << 16;
}
fn shlr(cpu: *SH4, opcode: Instr) void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 1) == 1);
    cpu.R(opcode.nmd.n).* >>= 1;
}
fn shlr2(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* >> 2;
}
fn shlr8(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* >> 8;
}
fn shlr16(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.n).* >> 16;
}

inline fn d8_label(cpu: *SH4, opcode: Instr) void {
    const displacement = sign_extension_u8(opcode.nd8.d);
    var pc: i32 = @intCast(cpu.pc & 0x1FFFFFFF);
    // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    pc += 4 + (displacement << 1) - 2;
    cpu.pc = (cpu.pc & 0xE0000000) | @as(u32, @bitCast(pc & 0x1FFFFFFF));
}

inline fn d12_label(cpu: *SH4, opcode: Instr) void {
    const displacement = sign_extension_u12(opcode.d12.d);
    var pc: i32 = @intCast(cpu.pc & 0x1FFFFFFF);
    pc += 4 + (displacement << 1) - 2; // -2 Because of the unconditional +2 in execute
    cpu.pc = (cpu.pc & 0xE0000000) | @as(u32, @bitCast(pc & 0x1FFFFFFF));
}

inline fn execute_delay_slot(cpu: *SH4, addr: addr_t) void {
    // TODO: If the instruction at addr is a branch instruction,
    // raise a Slot illegal instruction exception

    cpu._execute(addr);
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
    } else {
        cpu.pc += 2;
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
    } else {
        cpu.pc += 2;
    }
    execute_delay_slot(cpu, delay_slot);
}
fn bra_label(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    d12_label(cpu, opcode);
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
    cpu.pc += 4 + cpu.R(opcode.nmd.n).* - 2; // execute will allready add +2
    execute_delay_slot(cpu, delay_slot);
}
fn jmp_atRn(cpu: *SH4, opcode: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.pc = cpu.R(opcode.nmd.n).* - 2; // -2 to account for the standard +2
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
    cpu.pc = cpu.R(opcode.nmd.n).* - 2; // -2 to account for the standard +2
    execute_delay_slot(cpu, delay_slot);
}
fn rts(cpu: *SH4, _: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.pc = cpu.pr - 2; // execute will add +2
    execute_delay_slot(cpu, delay_slot);

    // Note: (I don't know if this is actually relevant)
    // In an RTE delay slot, status register (SR) bits are referenced as follows. In instruction access, the
    // MD bit is used before modification, and in data access, the MD bit is accessed after
    // modification. The other bits—S, T, M, Q, FD, BL, and RB—after modification are used for
    // delay slot instruction execution. The STC and STC.L SR instructions access all SR bits after
    // modification.
}

fn ldc_Rn_SR(cpu: *SH4, opcode: Instr) void {
    // TODO: Issuing this instruction in user mode will cause an illegal instruction exception.
    std.debug.assert(cpu.sr.md == 1); // This instruction is only usable in privileged mode.
    cpu.sr = @bitCast(cpu.R(opcode.nmd.n).* & 0x700083F3);
}
fn ldcl_at_Rn_inc_sr(cpu: *SH4, opcode: Instr) void {
    cpu.sr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*) & 0x700083F3);
    std.debug.print("  SR Overwrite: ({x:0>8}) {any}\n", .{ @as(u32, @bitCast(cpu.sr)), cpu.sr });
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldcl_at_Rn_inc_gbr(cpu: *SH4, opcode: Instr) void {
    cpu.gbr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldc_Rn_VBR(cpu: *SH4, opcode: Instr) void {
    cpu.vbr = cpu.R(opcode.nmd.n).*;
}
fn ldcl_at_Rn_inc_vbr(cpu: *SH4, opcode: Instr) void {
    cpu.vbr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldcl_at_Rn_inc_ssr(cpu: *SH4, opcode: Instr) void {
    cpu.ssr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldcl_at_Rn_inc_spc(cpu: *SH4, opcode: Instr) void {
    cpu.spc = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldc_Rn_DBR(cpu: *SH4, opcode: Instr) void {
    cpu.dbr = cpu.R(opcode.nmd.n).*;
}
fn ldcl_at_Rn_inc_dbr(cpu: *SH4, opcode: Instr) void {
    cpu.dbr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldsl_at_Rn_inc_mach(cpu: *SH4, opcode: Instr) void {
    cpu.mach = cpu.read32(cpu.R(opcode.nmd.n).*) | 0xFFFFFC00;
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldsl_at_Rn_inc_macl(cpu: *SH4, opcode: Instr) void {
    cpu.mach = cpu.read32(cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
fn lds_Rn_PR(cpu: *SH4, opcode: Instr) void {
    cpu.pr = cpu.R(opcode.nmd.n).*;
}
fn ldsl_atRn_inc_PR(cpu: *SH4, opcode: Instr) void {
    cpu.pr = cpu.read32(cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
fn lds_Rn_fpscr(cpu: *SH4, opcode: Instr) void {
    cpu.fpscr = @bitCast(cpu.R(opcode.nmd.n).* & 0x003FFFFF);
}
fn ldsl_at_Rn_inc_fpscr(cpu: *SH4, opcode: Instr) void {
    cpu.fpscr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*) & 0x003FFFFF);
    cpu.R(opcode.nmd.n).* += 4;
}
fn ldsl_at_Rn_inc_fpul(cpu: *SH4, opcode: Instr) void {
    cpu.fpul = cpu.read32(cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}

// Inverts the FR bit in floating-point register FPSCR.
fn frchg(cpu: *SH4, _: Instr) void {
    cpu.fpscr.fr +%= 1;
}

fn ocbp_atRn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}

// Reads a 32-byte data block starting at a 32-byte boundary into the operand cache.
// The lower 5 bits of the address specified by Rn are masked to zero.
// This instruction is also used to trigger a Store Queue write-back operation if the specified address points to the Store Queue area.
fn pref_atRn(cpu: *SH4, opcode: Instr) void {
    cpu.prefetch_operand_cache_block(cpu.R(opcode.nmd.n).*);
}
// Returns from an exception or interrupt handling routine by restoring the PC and SR values. Delayed jump.
fn rte(cpu: *SH4, _: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.sr = @bitCast(cpu.ssr);
    cpu.pc = @bitCast(cpu.spc);
    execute_delay_slot(cpu, delay_slot);
}
fn sets(cpu: *SH4, _: Instr) void {
    cpu.sr.s = true;
}
fn sett(cpu: *SH4, _: Instr) void {
    cpu.sr.t = true;
}
fn sleep(cpu: *SH4, _: Instr) void {
    const standby_control_register = cpu.read_io_register(u8, MemoryRegister.STBCR);
    if ((standby_control_register & 0b1000_0000) == 0b1000_0000) {
        const standby_control_register_2 = cpu.read_io_register(u8, MemoryRegister.STBCR2);
        if ((standby_control_register_2 & 0b1000_0000) == 0b1000_0000) {
            cpu.execution_state = .DeepSleep;
        } else {
            cpu.execution_state = .Standby;
        }
    } else {
        cpu.execution_state = .Sleep;
    }
}

fn stc_SR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = @bitCast(cpu.sr);
}
fn stc_TBR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.tbr;
}
fn stc_GBR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.gbr;
}
fn stc_VBR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.vbr;
}
fn stc_SGR_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.sgr;
}
fn stc_SSR_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.ssr;
}
fn stc_SPC_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.spc;
}
fn stc_DBR_rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.dbr;
}

fn sts_MACH_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.mach;
}
fn sts_l_MACH_atRn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn sts_MACL_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.macl;
}
fn sts_l_MACL_atRn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn sts_PR_Rn(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* = cpu.pr;
}
fn sts_l_PR_atRn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    cpu.write32(cpu.R(opcode.nmd.n).*, cpu.pr);
}

fn fmov_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* = cpu.DR(opcode.nmd.m >> 1).*;
    }
}
fn fmovs_at_Rm_inc_FRn(cpu: *SH4, opcode: Instr) void {
    // Single-precision
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @bitCast(cpu.read32(cpu.R(opcode.nmd.m).*));
        cpu.R(opcode.nmd.m).* += 4;
    } else { // Double-precision
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* = @bitCast(cpu.read64(cpu.R(opcode.nmd.m).*));
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
        cpu.write64(cpu.R(opcode.nmd.n).*, @bitCast(cpu.DR(opcode.nmd.m).*));
    }
}
fn fmov_DRm_DRn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn fmovd_atRm_XDn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn fmul_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
}
fn fdiv_FRm_FRn(cpu: *SH4, opcode: Instr) void {
    _ = opcode;
    _ = cpu;
    @panic("Unimplemented");
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

pub const Opcodes: [209]OpcodeDescription = .{
    .{ .code = 0b0000000000000000, .mask = 0b0000000000000000, .fn_ = nop, .name = "NOP", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000000000, .mask = 0b1111111111111111, .fn_ = unknown, .name = "Unknown opcode", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },

    .{ .code = 0b0110000000000011, .mask = 0b0000111111110000, .fn_ = mov_rm_rn, .name = "mov Rm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1110000000000000, .mask = 0b0000111111111111, .fn_ = mov_imm_rn, .name = "mov #imm,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1100011100000000, .mask = 0b0000000011111111, .fn_ = mova_atdispPC_R0, .name = "mova @(disp,PC),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1001000000000000, .mask = 0b0000111111111111, .fn_ = movw_atdispPC_Rn, .name = "mov.w @(disp,PC),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1101000000000000, .mask = 0b0000111111111111, .fn_ = movl_atdispPC_Rn, .name = "mov.l @(disp,PC),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
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
    .{ .code = 0b1000010000000000, .mask = 0b0000000011111111, .fn_ = movb_at_dispRm_R0, .name = "mov.b @(disp,Rm),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1000010100000000, .mask = 0b0000000011111111, .fn_ = movw_at_dispRm_R0, .name = "mov.w @(disp,Rm),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0101000000000000, .mask = 0b0000111111111111, .fn_ = movl_atdispRm_Rn, .name = "mov.l @(disp,Rm),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1000000000000000, .mask = 0b0000000011111111, .fn_ = movb_R0_at_dispRn, .name = "mov.b R0,@(disp,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1000000100000000, .mask = 0b0000000011111111, .fn_ = movw_R0_at_dispRn, .name = "mov.w R0,@(disp,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0001000000000000, .mask = 0b0000111111111111, .fn_ = movl_Rm_atdispRn, .name = "mov.l Rm,@(disp,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000001100, .mask = 0b0000111111110000, .fn_ = movb_atR0Rm_rn, .name = "mov.b @(R0,Rm),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0000000000001101, .mask = 0b0000111111110000, .fn_ = movw_atR0Rm_Rn, .name = "mov.w @(R0,Rm),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0000000000001110, .mask = 0b0000111111110000, .fn_ = movl_atR0Rm_rn, .name = "mov.l @(R0,Rm),Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0000000000000100, .mask = 0b0000111111110000, .fn_ = movb_Rm_atR0Rn, .name = "mov.b Rm,@(R0,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000000101, .mask = 0b0000111111110000, .fn_ = movw_Rm_atR0Rn, .name = "mov.w Rm,@(R0,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000000110, .mask = 0b0000111111110000, .fn_ = movl_Rm_atR0Rn, .name = "mov.l Rm,@(R0,Rn)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1100010000000000, .mask = 0b0000000011111111, .fn_ = movb_atdisp_GBR_R0, .name = "mov.b @(disp,GBR),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1100010100000000, .mask = 0b0000000011111111, .fn_ = movw_atdisp_GBR_R0, .name = "mov.w @(disp,GBR),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1100011000000000, .mask = 0b0000000011111111, .fn_ = movl_atdisp_GBR_R0, .name = "mov.l @(disp,GBR),R0", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1100000000000000, .mask = 0b0000000011111111, .fn_ = movb_R0_atdisp_GBR, .name = "mov.b R0,@(disp,GBR)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1100000100000000, .mask = 0b0000000011111111, .fn_ = movw_R0_atdisp_GBR, .name = "mov.w R0,@(disp,GBR)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b1100001000000000, .mask = 0b0000000011111111, .fn_ = movl_R0_atdisp_GBR, .name = "mov.l R0,@(disp,GBR)", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
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
    .{ .code = 0b0011000000001101, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "dmuls.l Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0011000000000101, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "dmulu.l Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0100000000010000, .mask = 0b0000111100000000, .fn_ = dt_Rn, .name = "dt Rn", .privileged = false },
    .{ .code = 0b0110000000001110, .mask = 0b0000111111110000, .fn_ = extsb_Rm_Rn, .name = "exts.b Rm,Rn", .privileged = false },
    .{ .code = 0b0110000000001111, .mask = 0b0000111111110000, .fn_ = extsw_Rm_Rn, .name = "exts.w Rm,Rn", .privileged = false },
    .{ .code = 0b0110000000001100, .mask = 0b0000111111110000, .fn_ = extub_Rm_Rn, .name = "extu.b Rm,Rn", .privileged = false },
    .{ .code = 0b0110000000001101, .mask = 0b0000111111110000, .fn_ = extuw_Rm_Rn, .name = "extu.w Rm,Rn", .privileged = false },
    .{ .code = 0b0000000000001111, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "mac.l @Rm+,@Rn+", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000001111, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "mac.w @Rm+,@Rn+", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0000000000000111, .mask = 0b0000111111110000, .fn_ = mull_Rm_Rn, .name = "mul.l Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0010000000001111, .mask = 0b0000111111110000, .fn_ = mulsw_Rm_Rn, .name = "muls.w Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0010000000001110, .mask = 0b0000111111110000, .fn_ = muluwRmRn, .name = "mulu.w Rm,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 4 },
    .{ .code = 0b0110000000001011, .mask = 0b0000111111110000, .fn_ = neg_Rm_Rn, .name = "neg Rm,Rn", .privileged = false },
    .{ .code = 0b0110000000001010, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "negc Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000001000, .mask = 0b0000111111110000, .fn_ = sub_Rm_Rn, .name = "sub Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000001010, .mask = 0b0000111111110000, .fn_ = subc_Rm_Rn, .name = "subc Rm,Rn", .privileged = false },
    .{ .code = 0b0011000000001011, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "subv Rm,Rn", .privileged = false },
    .{ .code = 0b0010000000001001, .mask = 0b0000111111110000, .fn_ = and_Rm_Rn, .name = "and Rm,Rn", .privileged = false },
    .{ .code = 0b1100100100000000, .mask = 0b0000000011111111, .fn_ = and_imm_R0, .name = "and #imm,R0", .privileged = false },
    .{ .code = 0b1100110100000000, .mask = 0b0000000011111111, .fn_ = unimplemented, .name = "and.b #imm,@(R0,GBR)", .privileged = false, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0110000000000111, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "not Rm,Rn", .privileged = false },
    .{ .code = 0b0010000000001011, .mask = 0b0000111111110000, .fn_ = or_Rm_Rn, .name = "or Rm,Rn", .privileged = false },
    .{ .code = 0b1100101100000000, .mask = 0b0000000011111111, .fn_ = or_imm_r0, .name = "or #imm,R0", .privileged = false },
    .{ .code = 0b1100111100000000, .mask = 0b0000000011111111, .fn_ = unimplemented, .name = "or.b #imm,@(R0,GBR)", .privileged = false, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0100000000011011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "tas.b @Rn", .privileged = false, .issue_cycles = 5, .latency_cycles = 5 },
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
    .{ .code = 0b0000000000100011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "braf Rm", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b1011000000000000, .mask = 0b0000111111111111, .fn_ = bsr_label, .name = "bsr label", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b0000000000000011, .mask = 0b0000111100000000, .fn_ = bsrf_Rn, .name = "bsrf Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0100000000101011, .mask = 0b0000111100000000, .fn_ = jmp_atRn, .name = "jmp @Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0100000000001011, .mask = 0b0000111100000000, .fn_ = jsr_Rn, .name = "jsr @Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0000000000001011, .mask = 0b0000000000000000, .fn_ = rts, .name = "rts", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0000000000101000, .mask = 0b0000000000000000, .fn_ = unimplemented, .name = "clrmac", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0000000001001000, .mask = 0b0000000000000000, .fn_ = unimplemented, .name = "clrs", .privileged = false },
    .{ .code = 0b0000000000001000, .mask = 0b0000000000000000, .fn_ = unimplemented, .name = "clrt", .privileged = false },
    .{ .code = 0b0100000000001110, .mask = 0b0000111100000000, .fn_ = ldc_Rn_SR, .name = "ldc Rn,SR", .privileged = true, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0100000000000111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_sr, .name = "ldc.l @Rn+,SR", .privileged = true, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0100000000011110, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "ldc Rm,GBR", .privileged = false, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0100000000010111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_gbr, .name = "ldc.l @Rn+,GBR", .privileged = false, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0100000000101110, .mask = 0b0000111100000000, .fn_ = ldc_Rn_VBR, .name = "ldc Rn,VBR", .privileged = true, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000100111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_vbr, .name = "ldc.l @Rn+,VBR", .privileged = true },
    .{ .code = 0b0100000000111110, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "ldc Rm,SSR", .privileged = true, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000110111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_ssr, .name = "ldc.l @Rn+,SSR", .privileged = true },
    .{ .code = 0b0100000001001110, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "ldc Rm,SPC", .privileged = true, .issue_cycles = 3, .latency_cycles = 1 },
    .{ .code = 0b0100000001000111, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_spc, .name = "ldc.l @Rn+,SPC", .privileged = true },
    .{ .code = 0b0100000011111010, .mask = 0b0000111100000000, .fn_ = ldc_Rn_DBR, .name = "ldc Rn,DBR", .privileged = true, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000011110110, .mask = 0b0000111100000000, .fn_ = ldcl_at_Rn_inc_dbr, .name = "ldc.l @Rn+,DBR", .privileged = true },
    .{ .code = 0b0100000010001110, .mask = 0b0000111101110000, .fn_ = unimplemented, .name = "ldc Rm,Rn_BANK", .privileged = true, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000010000111, .mask = 0b0000111101110000, .fn_ = unimplemented, .name = "ldc.l @Rm+,Rn_BANK", .privileged = true },
    .{ .code = 0b0100000000001010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "lds Rm,MACH", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000000110, .mask = 0b0000111100000000, .fn_ = ldsl_at_Rn_inc_mach, .name = "lds.l @Rn+,MACH", .privileged = false },
    .{ .code = 0b0100000000011010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "lds Rm,MACL", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000010110, .mask = 0b0000111100000000, .fn_ = ldsl_at_Rn_inc_macl, .name = "lds.l @Rn+,MACL", .privileged = false },
    .{ .code = 0b0100000000101010, .mask = 0b0000111100000000, .fn_ = lds_Rn_PR, .name = "lds Rn,PR", .privileged = false, .issue_cycles = 2, .latency_cycles = 3 },
    .{ .code = 0b0100000000100110, .mask = 0b0000111100000000, .fn_ = ldsl_atRn_inc_PR, .name = "lds.l @Rn+,PR", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000111000, .mask = 0b0000000000000000, .fn_ = unimplemented, .name = "ldtbl", .privileged = true },
    .{ .code = 0b0000000011000011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "movca.l R0,@Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0000000000001001, .mask = 0b0000000000000000, .fn_ = nop, .name = "nop", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b0000000010010011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "ocbi @Rn", .privileged = false },
    .{ .code = 0b0000000010100011, .mask = 0b0000111100000000, .fn_ = ocbp_atRn, .name = "ocbp @Rn", .privileged = false },
    .{ .code = 0b0000000010110011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "ocbwb @Rn", .privileged = false },
    .{ .code = 0b0000000010000011, .mask = 0b0000111100000000, .fn_ = pref_atRn, .name = "pref @Rn", .privileged = false },
    .{ .code = 0b0000000000101011, .mask = 0b0000000000000000, .fn_ = rte, .name = "rte", .privileged = true, .issue_cycles = 5, .latency_cycles = 5 },
    .{ .code = 0b0000000001011000, .mask = 0b0000000000000000, .fn_ = sets, .name = "sets", .privileged = false },
    .{ .code = 0b0000000000011000, .mask = 0b0000000000000000, .fn_ = sett, .name = "sett", .privileged = false },
    .{ .code = 0b0000000000011011, .mask = 0b0000000000000000, .fn_ = sleep, .name = "sleep", .privileged = true, .issue_cycles = 4, .latency_cycles = 4 },
    .{ .code = 0b0000000000000010, .mask = 0b0000111100000000, .fn_ = stc_SR_Rn, .name = "stc SR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000000011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "stc.l SR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000010010, .mask = 0b0000111100000000, .fn_ = stc_GBR_Rn, .name = "stc GBR,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000010011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "stc.l GBR,@-Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000100010, .mask = 0b0000111100000000, .fn_ = stc_VBR_Rn, .name = "stc VBR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000100011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "stc.l VBR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000111010, .mask = 0b0000111100000000, .fn_ = stc_SGR_rn, .name = "stc SGR,Rn", .privileged = true, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0100000000110010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "stc.l SGR,@-Rn", .privileged = true, .issue_cycles = 3, .latency_cycles = 3 },
    .{ .code = 0b0000000000110010, .mask = 0b0000111100000000, .fn_ = stc_SSR_rn, .name = "stc SSR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000110011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "stc.l SSR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000001000010, .mask = 0b0000111100000000, .fn_ = stc_SPC_rn, .name = "stc SPC,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000001000011, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "stc.l SPC,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000011111010, .mask = 0b0000111100000000, .fn_ = stc_DBR_rn, .name = "stc DBR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000011110010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "stc.l DBR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000010000010, .mask = 0b0000111101110000, .fn_ = unimplemented, .name = "stc Rm_BANK,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000010000011, .mask = 0b0000111101110000, .fn_ = unimplemented, .name = "stc.l Rm_BANK,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0000000000001010, .mask = 0b0000111100000000, .fn_ = sts_MACH_Rn, .name = "sts MACH,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000000010, .mask = 0b0000111100000000, .fn_ = sts_l_MACH_atRn, .name = "sts.l MACH,@-Rn", .privileged = false },
    .{ .code = 0b0000000000011010, .mask = 0b0000111100000000, .fn_ = sts_MACL_Rn, .name = "sts MACL,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000000010010, .mask = 0b0000111100000000, .fn_ = sts_l_MACL_atRn, .name = "sts.l MACL,@-Rn", .privileged = false },
    .{ .code = 0b0000000000101010, .mask = 0b0000111100000000, .fn_ = sts_PR_Rn, .name = "sts PR,Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b0100000000100010, .mask = 0b0000111100000000, .fn_ = sts_l_PR_atRn_dec, .name = "sts.l PR,@-Rn", .privileged = false, .issue_cycles = 2, .latency_cycles = 2 },
    .{ .code = 0b1100001100000000, .mask = 0b0000000011111111, .fn_ = unimplemented, .name = "trapa #imm", .privileged = false, .issue_cycles = 7, .latency_cycles = 7 },

    .{ .code = 0b1111000000001100, .mask = 0b0000111111110000, .fn_ = fmov_FRm_FRn, .name = "fmov FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000000001000, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fmov.s @Rm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1111000000001010, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fmov.s FRm,@Rn", .privileged = false },
    .{ .code = 0b1111000000001001, .mask = 0b0000111111110000, .fn_ = fmovs_at_Rm_inc_FRn, .name = "fmov.s @Rm+,FRn", .privileged = false },
    .{ .code = 0b1111000000001011, .mask = 0b0000111111110000, .fn_ = fmovs_FRm_at_dec_Rn, .name = "fmov.s FRm,@-Rn", .privileged = false },
    .{ .code = 0b1111000000000110, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fmov.s @(R0,Rm),FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1111000000000111, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fmov.s FRm,@(R0,Rn)", .privileged = false },

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

    .{ .code = 0b1111000010001101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "fldi0 FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000010011101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "fldi1 FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000000011101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "flds FRm,FPUL", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000000001101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "fsts FPUL,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000001011101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "fabs FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000001001101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "fneg FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
    .{ .code = 0b1111000000000000, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fadd FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000000001, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fsub FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000000010, .mask = 0b0000111111110000, .fn_ = fmul_FRm_FRn, .name = "fmul FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000001110, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fmac FR0,FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000000011, .mask = 0b0000111111110000, .fn_ = fdiv_FRm_FRn, .name = "fdiv FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 11 },
    .{ .code = 0b1111000001101101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "fsqrt FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 11 },
    .{ .code = 0b1111000000000100, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fcmp/eq FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1111000000000101, .mask = 0b0000111111110000, .fn_ = unimplemented, .name = "fcmp/gt FRm,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 2 },
    .{ .code = 0b1111000000101101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "float FPUL,FRn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000000111101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "ftrc FRm,FPUL", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b1111000011101101, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "fipr FVm,FVn", .privileged = false, .issue_cycles = 1, .latency_cycles = 4 },
    .{ .code = 0b1111000111111101, .mask = 0b0000110000000000, .fn_ = unimplemented, .name = "ftrv XMTRX,FVn", .privileged = false, .issue_cycles = 1, .latency_cycles = 5 },

    //.{ .code = 0b1111000001011101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fabs DRn", .privileged = false },
    //.{ .code = 0b1111000001001101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fneg DRn", .privileged = false },
    // Actually handled by fadd FRm,FRn - Switched by SR register
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

    .{ .code = 0b0100000001101010, .mask = 0b0000111100000000, .fn_ = lds_Rn_fpscr, .name = "lds Rn,FPSCR", .privileged = false, .issue_cycles = 1, .latency_cycles = 4 },
    .{ .code = 0b0000000001101010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "sts FPSCR,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000001100110, .mask = 0b0000111100000000, .fn_ = ldsl_at_Rn_inc_fpscr, .name = "lds.l @Rn+,FPSCR", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000001100010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "sts.l FPSCR,@-Rn", .privileged = false },
    .{ .code = 0b0100000001011010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "lds Rm,FPUL", .privileged = false },
    .{ .code = 0b0000000001011010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "sts FPUL,Rn", .privileged = false, .issue_cycles = 1, .latency_cycles = 3 },
    .{ .code = 0b0100000001010110, .mask = 0b0000111100000000, .fn_ = ldsl_at_Rn_inc_fpul, .name = "lds.l @Rn+,FPUL", .privileged = false },
    .{ .code = 0b0100000001010010, .mask = 0b0000111100000000, .fn_ = unimplemented, .name = "sts.l FPUL,@-Rn", .privileged = false },
    .{ .code = 0b1111101111111101, .mask = 0b0000000000000000, .fn_ = frchg, .name = "frchg", .privileged = false },
    .{ .code = 0b1111001111111101, .mask = 0b0000000000000000, .fn_ = unimplemented, .name = "fschg", .privileged = false },
};

fn write_and_execute(cpu: *SH4, code: u16) void {
    cpu.write16(cpu.pc, code);
    cpu.execute();
}

test "mov #imm,Rn" {
    var cpu: SH4 = .{};
    try cpu.init();
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    write_and_execute(&cpu, 0b1110_0000_00000000);
    try std.testing.expect(cpu.R(0).* == 0);
    write_and_execute(&cpu, 0b1110_0000_11111111);
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFF);
    write_and_execute(&cpu, 0b1110_0001_00000001);
    try std.testing.expect(cpu.R(1).* == 1);
    write_and_execute(&cpu, 0b1110_0010_00000010);
    try std.testing.expect(cpu.R(2).* == 2);
    write_and_execute(&cpu, 0b1110_0011_00000011);
    try std.testing.expect(cpu.R(3).* == 3);

    write_and_execute(&cpu, 0b1110_1111_00000000); // mov #0,R15
    try std.testing.expect(cpu.R(15).* == 0);
    write_and_execute(&cpu, 0b1110_1111_00000001); // mov #1,R15
    try std.testing.expect(cpu.R(15).* == 1);
}

test "mov Rm,Rn" {
    var cpu: SH4 = .{};
    try cpu.init();
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    write_and_execute(&cpu, 0b1110_0000_00000000); // mov #0,R0
    try std.testing.expect(cpu.R(0).* == 0);
    write_and_execute(&cpu, 0b1110_0001_00000001); // mov #1,R1
    try std.testing.expect(cpu.R(1).* == 1);
    write_and_execute(&cpu, 0b0110_0000_0001_0011); // mov R1,R0
    try std.testing.expect(cpu.R(0).* == 1);
}

test "ldc Rn,SR" {
    var cpu: SH4 = .{};
    try cpu.init();
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    write_and_execute(&cpu, 0b1110_0000_00000011); // mov #3,R0
    try std.testing.expect(cpu.R(0).* == 0b000000011);
    write_and_execute(&cpu, 0b0100_0000_00001110); // ldc R0,SR
    try std.testing.expect(@as(u32, @bitCast(cpu.sr)) == 0b00000011 & 0x700083F3);

    cpu.sr = .{}; // Reset SR

    write_and_execute(&cpu, 0b1110_1111_00000011); // mov #3,R15
    try std.testing.expect(cpu.R(15).* == 0b00000011);
    write_and_execute(&cpu, 0b0100_1111_00001110); // ldc R15,SR
    try std.testing.expect(@as(u32, @bitCast(cpu.sr)) == 0b00000011 & 0x700083F3);
}

test "boot" {
    var cpu: SH4 = .{};
    try cpu.init();
    defer cpu.deinit();

    cpu.execute(); // mov 0x0F,R3
    try std.testing.expect(cpu.R(3).* == 0xFFFFFFFF);
    cpu.execute(); // shll16 R3
    try std.testing.expect(cpu.R(3).* == 0xFFFF0000);
    cpu.execute(); // swap.w R4,R3
    try std.testing.expect(cpu.R(4).* == 0x0000FFFF);
    cpu.execute(); // shll8 R3
    try std.testing.expect(cpu.R(3).* == 0xFF000000);
    cpu.execute(); // shlr2 R4
    try std.testing.expect(cpu.R(4).* == 0x00003FFF);
    cpu.execute(); // shlr2 R4
    try std.testing.expect(cpu.R(4).* == 0x00000FFF);
    // Reads EXPEVT (0xFF000024), 0x00000000 on power up, 0x00000020 on reset.
    cpu.execute(); // mov.l @(9, R3),R0
    try std.testing.expect(cpu.read32(cpu.R(3).* + (9 << 2)) == cpu.R(0).*);
    cpu.execute(); // xor R4,R0
    try std.testing.expect(cpu.R(4).* == 0x00000FFF);
    try std.testing.expect(cpu.R(4).* == 0x00000FFF);
    {
        const prev_mach = cpu.mach;
        cpu.execute(); // mulu.w R4,R0
        try std.testing.expect(cpu.mach == prev_mach);
        try std.testing.expect(cpu.macl == 0);
    }
    cpu.execute(); // sts R0,MACL
    try std.testing.expect(cpu.R(0).* == 0);
    cpu.execute(); // tst R0,R0
    try std.testing.expect(cpu.sr.t);
    cpu.execute(); // bf 0x8C010108
    try std.testing.expect(cpu.pc == 0xA0000018);
    cpu.execute(); // mov.l R0,@(4,R3) - Write 0x0 to MMUCR @ 0xFF000010
    try std.testing.expect(cpu.R(0).* == cpu.read32(0xFF000000 + (4 << 2)));
    cpu.execute(); // mov 0x9,R1
    try std.testing.expect(cpu.R(1).* == 0x9);
    cpu.execute(); // shll8 R1
    try std.testing.expect(cpu.R(1).* == 0x9 << 8);
    cpu.execute(); // add 0x29,R1
    try std.testing.expect(cpu.R(1).* == (0x9 << 8) + 0x29);
    cpu.execute(); // mov.l R1, @(7, R3) - Write 0x929 to CCR @ 0xFF00001C
    try std.testing.expect(cpu.R(1).* == cpu.read32(0xFF000000 + (0x7 << 2)));
    cpu.execute(); // shar R3
    try std.testing.expect(cpu.R(3).* == 0xFF800000);
    try std.testing.expect(!cpu.sr.t);
    cpu.execute(); // mov 0x01, R0
    try std.testing.expect(cpu.R(0).* == 0x01);
    cpu.execute(); // mov.w R0, @(2, R3) - Write 0x01 to BCR2 @ 0xFF800004
    try std.testing.expect(cpu.R(0).* == cpu.read16(0xFF800000 + (0x2 << 1)));
    cpu.execute(); // mov 0xFFFFFFC3, R0
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFC3);
    cpu.execute(); // shll16 R0
    try std.testing.expect(cpu.R(0).* == 0xFFC30000);
    cpu.execute(); // or 0xCD, R0
    try std.testing.expect(cpu.R(0).* == 0xFFC300CD);
    cpu.execute(); // shll8 R0
    try std.testing.expect(cpu.R(0).* == 0xC300CD00);
    cpu.execute(); // or 0xB0, R0
    try std.testing.expect(cpu.R(0).* == 0xC300CDB0);
    cpu.execute(); // shlr R0
    try std.testing.expect(cpu.R(0).* == 0xC300CDB0 >> 1);
    try std.testing.expect(!cpu.sr.t);
    cpu.execute(); // mov.l R0, @(3, R3) - Write 0x01 to WCR2 @ 0xFF80000C
    try std.testing.expect(cpu.R(0).* == cpu.read32(0xFF800000 + (0x3 << 2)));
    cpu.execute(); // mov 0x01, R5
    try std.testing.expect(cpu.R(5).* == 0x01);
    cpu.execute(); // rotr R5
    try std.testing.expect(cpu.R(5).* == 0x80000000);
    try std.testing.expect(cpu.sr.t);
    cpu.execute(); // add 0x60, R5
    try std.testing.expect(cpu.R(5).* == 0x80000060);
    cpu.execute(); // mov R5, R6
    try std.testing.expect(cpu.R(5).* == cpu.R(6).*);
    cpu.execute(); // add 0x20, R6
    try std.testing.expect(cpu.R(6).* == 0x80000080);
    cpu.execute(); // tst 0x00, R0 - Always tue, right?
    try std.testing.expect(cpu.sr.t);
    cpu.execute(); // pref @R5
    // TODO
    cpu.execute(); // jmp @R6
    std.debug.print("{x:0>8}, {x:0>8}\n", .{ cpu.pc, cpu.R(6).* });
    try std.testing.expect(cpu.pc == cpu.R(6).*);

    cpu.execute(); // mov.l @(0x2,R5),R0 - Read 0x80000068 (0xA3020008) to R0
    try std.testing.expect(0xA3020008 == cpu.R(0).*);
    try std.testing.expect(cpu.read32(cpu.R(5).* + (0x2 << 2)) == cpu.R(0).*);

    cpu.execute(); // mov.l R0, @(0, R3) - Write 0xA3020008 to BRC1 @ 0xFF800000
    try std.testing.expect(cpu.read32(0xFF800000) == 0xA3020008);
    cpu.execute(); // mov.l @(4,R5),R0
    try std.testing.expect(cpu.read32(cpu.R(5).* + (0x4 << 2)) == cpu.R(0).*);
    cpu.execute(); // mov.l R0, @(2, R3) - Write 0x01110111 to WCR1 @ 0xFF800008
    try std.testing.expect(cpu.read32(0xFF800008) == 0x01110111);
    cpu.execute(); // add 0x10, R3
    try std.testing.expect(cpu.R(3).* == 0xFF800010);
    cpu.execute(); // mov.l @(5, R5), R0 - Read 0x80000078 (0x800A0E24) to R0
    try std.testing.expect(cpu.R(0).* == 0x800A0E24);
    cpu.execute(); // mov.l R0, @(1, R3) - Write 0x800A0E24 to MCR
    try std.testing.expect(cpu.io_register(u32, MemoryRegister.MCR).* == 0x800A0E24);

    cpu.execute(); // mov.l @(7, R5), R2
    try std.testing.expect(cpu.R(2).* == 0xff940190);
    cpu.execute(); // mov.b R2, @R2
    try std.testing.expect(cpu.io_register(u8, MemoryRegister.SDMR).* == 0x90);

    cpu.execute(); // mov 0xFFFFFFA4, R0
    cpu.execute(); // shll8 R0
    cpu.execute(); // mov.w R0, @(12, R3)
    try std.testing.expect(cpu.io_register(u16, MemoryRegister.RFCR).* == 0xA400);

    cpu.execute(); // mov.w @(0, R5), R0
    cpu.execute(); // mov.w R0, @(10, R3)
    try std.testing.expect(cpu.io_register(u16, MemoryRegister.RTCOR).* == 0xA504);

    cpu.execute(); // add H'0c, R0
    cpu.execute(); // mov.w R0, @(6, R3)
    try std.testing.expect(cpu.io_register(u16, MemoryRegister.RTCSR).* == 0xA510);

    // while((volatile uint16_t)reg[RFCR] <= 0x0010);

    cpu.execute(); // mov 0x10, R6
    cpu.execute(); // mov.w @(12, R3), R0 - Load RFCR (Refresh Count Register) to R0
    try std.testing.expect(cpu.R(0).* == 0x11); // Note: Refresh Count Register not implemented, this check passes because we always return 0x11.
    cpu.execute(); // cmp/hi R6, R0
    cpu.execute(); // bf 0x8C0100A2

    cpu.execute(); // mov.w @(1, R5), R0
    cpu.execute(); // mov.w R0, @(10, R3)
    try std.testing.expect(cpu.io_register(u16, MemoryRegister.RTCOR).* == 0xa55e);

    cpu.execute(); // mov.l @(6, R5), R0
    cpu.execute(); // mov.l R0, @(1, R3)
    try std.testing.expect(cpu.io_register(u32, MemoryRegister.MCR).* == 0xc00a0e24);

    cpu.execute(); // mov.b R2, @R2
    cpu.execute(); // mov.l @(1, R5), R1
    try std.testing.expect(cpu.io_register(u8, MemoryRegister.SDMR).* == 0x90);

    cpu.execute(); // mov 0x04, R0
    try std.testing.expect(cpu.R(0).* == 0x04);
    cpu.execute(); // swap.b R0, R0
    try std.testing.expect(cpu.R(0).* == 0x0400);
    cpu.execute(); // mov.w R0, @R1
    try std.testing.expect(cpu.read32(0xA05F7480) == 0x400); // SB_G1RRC

    cpu.execute(); // mov.l @(3, R5), R3
    cpu.execute(); // mova 0x8C0100E0, R0

    for (0..16) |_| {
        try std.testing.expect(cpu.pc == 0x800000BE);
        cpu.execute(); // dt R6
        cpu.execute(); // mov.w @R0+, R1
        cpu.execute(); // mov.w R1, @-R3
        cpu.execute(); // bf 0x8C0100BE
    }

    cpu.execute(); // mov.l @R3, R1
    cpu.execute(); // jmp @R3
    try std.testing.expect(cpu.pc == 0x8C0000E0);

    cpu.execute(); //

    cpu.execute(); //
}
