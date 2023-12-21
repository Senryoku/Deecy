// Hitachi SH-4

const std = @import("std");
const common = @import("./common.zig");
const termcolor = @import("termcolor.zig");

const mmu = @import("./mmu.zig");
const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;
const Interrupts = @import("Interrupts.zig");
const Interrupt = Interrupts.Interrupt;
const Holly = @import("Holly.zig").Holly;
const AICA = @import("aica.zig").AICA;
const syscall = @import("syscall.zig");

const addr_t = u32;
const byte_t = u8;
const word_t = u16;
const longword_t = u32;

// FIXME: Move
// Pluged in video cable reported to the CPU:
// 0 = VGA, 1 = VGA, 2 = RGB, 3 = TV Composite.
const CableType = 0;

// FIXME: Move.
var aica: AICA = .{};

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

pub fn is_p0(addr: addr_t) bool {
    return (addr & 0xE0000000) == 0b00000000000000000000000000000000;
}
pub fn is_p1(addr: addr_t) bool {
    return (addr & 0xE0000000) == 0b10000000000000000000000000000000;
}
pub fn is_p2(addr: addr_t) bool {
    return (addr & 0xE0000000) == 0b10100000000000000000000000000000;
}
pub fn is_p3(addr: addr_t) bool {
    return (addr & 0xE0000000) == 0b11000000000000000000000000000000;
}
pub fn is_p4(addr: addr_t) bool {
    return (addr & 0xE0000000) == 0xE0000000;
}

pub const Instr = packed union {
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

    utlb_entries: [64]mmu.UTLBEntry = undefined,

    boot: []u8 align(4) = undefined,
    flash: []u8 align(4) = undefined,
    ram: []u8 align(4) = undefined,
    dummy_area5: u8 align(32) = 0,
    store_queues: [2][8]u32 align(4) = undefined,
    area7: []u8 align(4) = undefined, // FIXME: This is a huge waste of memory.
    hardware_registers: []u8 align(4) = undefined, // FIXME

    gpu: Holly = .{},

    _dummy: u32 align(32) = undefined, // FIXME: Dummy space for non-implemented features

    interrupt_requests: u64 = 0,

    timer_cycle_counter: [3]u32 = .{0} ** 3, // Cycle counts before scaling.

    debug_trace: bool = false,

    _allocator: std.mem.Allocator = undefined,

    pub fn init(allocator: std.mem.Allocator) !SH4 {
        var self: SH4 = .{};

        self._allocator = allocator;

        self.area7 = try self._allocator.alloc(u8, 64 * 1024 * 1024);
        self.ram = try self._allocator.alloc(u8, 16 * 1024 * 1024);
        self.hardware_registers = try self._allocator.alloc(u8, 0x200000);

        try self.gpu.init(self._allocator);

        // Load ROM
        self.boot = try self._allocator.alloc(u8, 0x200000);
        var boot_file = try std.fs.cwd().openFile("./bin/dc_boot.bin", .{});
        defer boot_file.close();
        const bytes_read = try boot_file.readAll(self.boot);
        std.debug.assert(bytes_read == 0x200000);

        // Load Flash
        self.flash = try self._allocator.alloc(u8, 0x20000);
        var flash_file = try std.fs.cwd().openFile("./bin/dc_flash.bin", .{});
        defer flash_file.close();
        const flash_bytes_read = try flash_file.readAll(self.flash);
        std.debug.assert(flash_bytes_read == 0x20000);

        init_jump_table();

        self.reset();

        return self;
    }

    pub fn deinit(self: *@This()) void {
        self.gpu.deinit();
        self._allocator.free(self.flash);
        self._allocator.free(self.boot);
        self._allocator.free(self.hardware_registers);
        self._allocator.free(self.ram);
        self._allocator.free(self.area7);
    }

    pub fn reset(self: *@This()) void {
        self.r_bank0 = undefined;
        self.r_bank1 = undefined;
        self.r_8_15 = undefined;
        self.R(0xF).* = 0x8C00F400;
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

        self.io_register(mmu.PTEH, .PTEH).* = .{};
        self.io_register(mmu.PTEL, .PTEL).* = .{};
        self.io_register(u32, .TTB).* = 0;
        self.io_register(u32, .TEA).* = 0;
        self.io_register(u32, .MMUCR).* = 0;

        self.io_register(u32, ._FF000030).* = 0x040205C1;

        self.io_register(u8, .BASRA).* = undefined;
        self.io_register(u8, .BASRB).* = undefined;
        self.io_register(u32, .CCR).* = 0;

        self.io_register(u32, .TRA).* = 0;
        self.io_register(u32, .EXPEVT).* = 0;
        self.io_register(u32, .INTEVT).* = 0;

        self.io_register(u32, .QACR0).* = undefined;
        self.io_register(u32, .QACR1).* = undefined;
        self.io_register(u32, .BARA).* = undefined;
        self.io_register(u32, .BAMRA).* = undefined;

        self.io_register(u32, .MCR).* = 0xC0091224;
        self.io_register(u16, .SDMR).* = 0x00FF;
        self.io_register(u16, .RTCSR).* = 0xA510;
        self.io_register(u16, .RTCNT).* = 0xA500;
        self.io_register(u16, .RTCOR).* = 0xA55E;
        self.io_register(u16, .RFCR).* = undefined;

        self.io_register(u32, .BCR1).* = 0;
        self.io_register(u32, .BCR2).* = 0x3FFC;
        self.io_register(u32, .PCTRA).* = 0;

        self.gpu.reset();

        self.hw_register(u32, .SB_FFST).* = 0; // FIFO Status
        self.hw_register(u32, .SB_ISTNRM).* = 0;
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

        self.io_register(mmu.PTEH, .PTEH).* = .{};
        self.io_register(mmu.PTEL, .PTEL).* = .{};
        self.io_register(u32, .TTB).* = 0;
        self.io_register(u32, .TEA).* = 0;
        self.io_register(u32, .MMUCR).* = 0;
        // BASRA Held
        // BASRB Held
        self.io_register(u32, .CCR).* = 0;
        self.io_register(u32, .TRA).* = 0;
        self.io_register(u32, .EXPEVT).* = 0x00000020;
        // INTEVT Held
        self.io_register(u32, .QACR0).* = undefined;
        self.io_register(u32, .QACR1).* = undefined;
        // BARA Held
        // BAMRA Held

        // BCR1 Held
        // BCR2 Held
    }

    // Reset state to after bios.
    pub fn init_boot(self: *@This()) void {
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

        self.sr = @bitCast(@as(u32, 0x400000F1));
        self.fpscr = @bitCast(@as(u32, 0x00040001));

        @memset(self.ram[0x00200000..0x00300000], 0x00); // FIXME: I think KallistiOS relies on that, or maybe I messed up somewhere else. (the BootROM does clear this section of RAM)

        // Copy subroutine to RAM. Some of it will be overwritten, I'm trying to work out what's important and what's not.
        inline for (0..16) |i| {
            self.write16(0x8C0000E0 + 2 * i, self.read16(0x800000FE - 2 * i));
        }
        // Copy a portion of the boot ROM to RAM.
        self.write32(0xA05F74E4, 0x001FFFFF);
        // @memcpy(self.ram[0x00000100 .. 0x100 + 0x0007FFC0], self.boot[0x00000100 .. 0x100 + 0x0007FFC0]);
        @memcpy(self.ram[0x00000100..0x00004000], self.boot[0x00000100..0x00004000]);
        @memcpy(self.ram[0x00008000..0x00200000], self.boot[0x00008000..0x00200000]);

        const IP_bin_HLE = false;
        if (IP_bin_HLE) {
            // Copy a portion of the flash ROM to RAM.
            inline for (0..8) |i| {
                self.write8(0x8C000068 + i, self.read8(0x0021A056 + i));
            }
            inline for (0..5) |i| {
                self.write8(0x8C000068 + 8 + i, self.read8(0x0021A000 + i));
            }
            // FIXME: Load system settings from flashrom (User partition (2), logical block 5), instead of these hardcoded values.
            //inline for (.{ 0xBC, 0xEA, 0x90, 0x5E, 0xFF, 0x04, 0x00, 0x01 }, 0..) |val, i| {
            inline for (.{ 0x00, 0x00, 0x89, 0xFC, 0x5B, 0xFF, 0x01, 0x00, 0x00, 0x7D, 0x0A, 0x62, 0x61 }, 0..) |val, i| {
                self.write8(0x8C000068 + 13 + i, val);
            }
        }

        // Patch some function adresses ("syscalls")

        const HLE_syscalls = true;
        if (HLE_syscalls) {
            // System
            self.write32(0x8C0000B0, 0x8C001000);
            self.write16(0x8C001000, 0b0000000000010000);
            // Font
            self.write32(0x8C0000B4, 0x8C001002);
            self.write16(0x8C001002, 0b0000000000100000);
            // Flashrom
            self.write32(0x8C0000B8, 0x8C001004);
            self.write16(0x8C001004, 0b0000000000110000);
            // GD
            self.write32(0x8C0000BC, 0x8C001006);
            self.write16(0x8C001006, 0b0000000001000000);
            // GD2
            self.write32(0x8C0000C0, 0x8C0010F0);
            self.write16(0x8C0010F0, 0b0000000001010000);
            // Misc
            self.write32(0x8C0000E0, 0x8C001008);
            self.write16(0x8C001008, 0b0000000001100000);
        } else {
            inline for (.{
                .{ 0x8C0000B0, 0x8C003C00 },
                .{ 0x8C0000B4, 0x8C003D80 },
                .{ 0x8C0000B8, 0x8C003D00 },
                .{ 0x8C0000BC, 0x8C001000 },
                .{ 0x8C0000C0, 0x8C0010F0 },
                .{ 0x8C0000E0, 0x8C000800 },
            }) |p| {
                self.write32(p[0], p[1]);
            }
        }

        // Other set values, IDK

        inline for (.{
            .{ 0x8C0000AC, 0xA05F7000 },
            .{ 0x8C0000A8, 0xA0200000 },
            .{ 0x8C0000A4, 0xA0100000 },
            .{ 0x8C0000A0, 0x00000000 },
            .{ 0x8C00002C, 0x00000000 },
            .{ 0x8CFFFFF8, 0x8C000128 },
        }) |p| {
            self.write32(p[0], p[1]);
        }

        // Patch some functions apparently used by interrupts
        // (And some other random stuff that the boot ROM sets for some reason
        //  and I'm afraid some games might use. I'm not taking any more chances)

        // Sleep on error?
        self.write32(0x8C000000, 0x00090009);
        self.write32(0x8C000004, 0x001B0009);
        self.write32(0x8C000008, 0x0009AFFD);
        // ??
        self.write16(0x8C00000C, 0);
        self.write16(0x8C00000E, 0);
        // RTE - Some interrupts jump there instead of having their own RTE, I have NO idea why.
        self.write32(0x8C000010, 0x00090009); // nop nop
        self.write32(0x8C000014, 0x0009002B); // rte nop
        // RTS
        self.write32(0x8C000018, 0x00090009);
        self.write32(0x8C00001C, 0x0009000B);

        // ??
        self.write8(0x8C00002C, 0x16);
        self.write32(0x8C000064, 0x8c008100);
        self.write16(0x8C000090, 0);
        self.write16(0x8C000092, @bitCast(@as(i16, -128)));

        // Holly Version. TODO: Make it configurable?
        self.hw_register(u32, .SB_SBREV).* = 0x0B;
        self.hw_register(u32, .SB_G2ID).* = 0x12; // Only possible value, apparently.
    }

    pub fn load_IP_bin(self: *@This(), bin: []const u8) void {
        @memcpy(self.ram[0x8000 .. 0x8000 + bin.len], bin);
    }
    pub fn load_at(self: *@This(), addr: addr_t, bin: []const u8) void {
        const start_addr = ((addr & 0x1FFFFFFF) - 0x0C000000);
        @memcpy(self.ram[start_addr .. start_addr + bin.len], bin);
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

    pub fn hw_register(self: *@This(), comptime T: type, r: MemoryRegister) *T {
        return @as(*T, @alignCast(@ptrCast(&self.hardware_registers[(@intFromEnum(r) & 0x1FFFFFFF) - 0x005F6800])));
    }
    pub fn read_hw_register(self: @This(), comptime T: type, r: MemoryRegister) T {
        return @as(*T, @alignCast(@ptrCast(&self.hardware_registers[(@intFromEnum(r) & 0x1FFFFFFF) - 0x005F6800]))).*;
    }

    pub inline fn R(self: *@This(), r: u4) *u32 {
        if (r >= 8) return &self.r_8_15[r - 8];
        // Note: Rather than checking all the time, we could swap only once
        //       when SR is updated.
        if (self.sr.md == 1 and self.sr.rb == 1) return &self.r_bank1[r];
        return &self.r_bank0[r];
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
        std.debug.print(" => Jump to Interrupt: VBR: {X:0>8}, Code: {X:0>4}\n", .{ self.vbr, self.read_io_register(u32, MemoryRegister.INTEVT) });
        //if (self.read_io_register(u32, MemoryRegister.INTEVT) == 0x03A0)
        //self.debug_trace = true;

        self.execution_state = .Running;
        self.spc = self.pc;
        self.ssr = @bitCast(self.sr);
        self.sr.bl = true;
        self.sr.md = 1;
        self.sr.rb = 1;
        self.pc = self.vbr + 0x600;
    }

    fn jump_to_exception(self: *@This()) void {
        self.spc = self.pc;
        self.ssr = @bitCast(self.sr);
        self.sr.bl = true;
        self.sr.md = 1;
        self.sr.rb = 1;

        const offset = 0x600; // TODO
        const UserBreak = false; // TODO
        if (self.read_io_register(MemoryRegisters.BRCR, MemoryRegister.BRCR).ubde == 1 and UserBreak) {
            self.pc = self.dbr;
        } else {
            self.pc = self.vbr + offset;
        }
    }

    pub fn execute(self: *@This()) void {
        // When the BL bit in SR is 0, exceptions and interrupts are accepted.

        // See h14th002d2.pdf page 665 (or 651)
        if (!self.sr.bl or self.execution_state != .Running) {
            if (self.interrupt_requests != 0) {
                // TODO: Search the highest priority interrupt.
                const first_set = @ctz(self.interrupt_requests);
                // Check it against the cpu interrupt mask
                if (Interrupts.InterruptLevel[first_set] >= self.sr.imask) {
                    self.interrupt_requests &= ~(@as(u64, 1) << @truncate(first_set)); // Clear the request
                    self.io_register(u32, MemoryRegister.INTEVT).* = Interrupts.InterruptINTEVTCodes[first_set];
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
        } else {
            // FIXME: Not sure if this is a thing.
            self.add_cycles(1000);
        }
    }

    fn request_interrupt(self: *@This(), int: Interrupt) void {
        std.debug.print(" (Interrupt request! {s})\n", .{std.enums.tagName(Interrupt, int) orelse "Unknown"});
        self.interrupt_requests |= @as(u33, 1) << @intFromEnum(int);
    }

    fn check_sb_interrupts(self: *@This()) void {

        // FIXME: Not sure if this is the right place to check for those.
        // FIXME: Also check external interrupts (SB_ISTEXT) and errors (SB_ISTERR)
        const istnrm = self.read_hw_register(u32, .SB_ISTNRM);
        const istext = self.read_hw_register(u32, .SB_ISTEXT);
        const isterr = self.read_hw_register(u32, .SB_ISTERR);
        if (istnrm & self.read_hw_register(u32, .SB_IML6NRM) != 0 or istext & self.read_hw_register(u32, .SB_IML6EXT) != 0 or isterr & self.read_hw_register(u32, .SB_IML6ERR) != 0) {
            self.request_interrupt(Interrupts.Interrupt.IRL9);
        }
        if (istnrm & self.read_hw_register(u32, .SB_IML4NRM) != 0 or istext & self.read_hw_register(u32, .SB_IML4EXT) != 0 or isterr & self.read_hw_register(u32, .SB_IML4ERR) != 0) {
            self.request_interrupt(Interrupts.Interrupt.IRL11);
        }
        if (istnrm & self.read_hw_register(u32, .SB_IML2NRM) != 0 or istext & self.read_hw_register(u32, .SB_IML2EXT) != 0 or isterr & self.read_hw_register(u32, .SB_IML2ERR) != 0) {
            self.request_interrupt(Interrupts.Interrupt.IRL13);
        }
    }

    // TODO: Add helpers for external interrupts and errors.

    pub fn raise_normal_interrupt(self: *@This(), int: MemoryRegisters.SB_ISTNRM) void {
        self.hw_register(u32, .SB_ISTNRM).* |= @bitCast(int);

        self.check_sb_interrupts();
    }

    pub fn raise_external_interrupt(self: *@This(), int: MemoryRegisters.SB_ISTEXT) void {
        self.hw_register(u32, .SB_ISTEXT).* |= @bitCast(int);
        self.hw_register(u32, .SB_ISTNRM).* |= @bitCast(MemoryRegisters.SB_ISTNRM{ .ExtStatus = if (self.hw_register(u32, .SB_ISTEXT).* != 0) 1 else 0 });

        self.check_sb_interrupts();
    }

    fn timer_prescaler(value: u3) u32 {
        switch (value) {
            0 => return 4,
            1 => return 16,
            2 => return 64,
            3 => return 256,
            5 => return 1024,
            else => unreachable,
        }
    }

    pub fn advance_timers(self: *@This(), cycles: u32) void {
        const TSTR = self.read_io_register(u32, MemoryRegister.TSTR);

        // When one of bits STR0–STR2 is set to 1 in the timer start register (TSTR), the timer counter
        // (TCNT) for the corresponding channel starts counting. When TCNT underflows, the UNF flag is
        // set in the corresponding timer control register (TCR). If the UNIE bit in TCR is set to 1 at this
        // time, an interrupt request is sent to the CPU. At the same time, the value is copied from TCOR
        // into TCNT, and the count-down continues (auto-reload function).
        const timers = .{
            .{ .counter = MemoryRegister.TCNT0, .control = MemoryRegister.TCR0, .constant = MemoryRegister.TCOR0, .interrupt = Interrupt.TUNI0 },
            .{ .counter = MemoryRegister.TCNT1, .control = MemoryRegister.TCR1, .constant = MemoryRegister.TCOR1, .interrupt = Interrupt.TUNI1 },
            .{ .counter = MemoryRegister.TCNT2, .control = MemoryRegister.TCR2, .constant = MemoryRegister.TCOR2, .interrupt = Interrupt.TUNI2 },
        };

        inline for (0..3) |i| {
            if ((TSTR >> @intCast(i)) & 0x1 == 1) {
                const tcnt = self.io_register(u32, timers[i].counter);
                const tcr = self.io_register(MemoryRegisters.TCR, timers[i].control);

                self.timer_cycle_counter[i] += cycles;

                const scale = SH4.timer_prescaler(tcr.*.tpsc);
                if (self.timer_cycle_counter[i] >= scale) {
                    self.timer_cycle_counter[i] -= scale;

                    if (tcnt.* == 0) {
                        tcr.*.unf = 1; // Signals underflow
                        tcnt.* = self.io_register(u32, timers[i].constant).*; // Reset counter
                        if (tcr.*.unie == 1)
                            self.request_interrupt(timers[i].interrupt);
                    } else {
                        tcnt.* -= 1;
                    }
                }
            }
        }
    }

    pub fn add_cycles(self: *@This(), cycles: u32) void {
        self.advance_timers(cycles);
        self.gpu.update(self, cycles);
        aica.update(self, cycles);
    }

    pub fn _execute(self: *@This(), addr: addr_t) void {
        const opcode = self.read16(addr);
        const instr = Instr{ .value = opcode };
        if (self.debug_trace)
            std.debug.print("[{X:0>4}] {b:0>16} {s: <20}\tR{d: <2}={X:0>8}, R{d: <2}={X:0>8}, d={X:0>1}, d8={X:0>2}, d12={X:0>3}\n", .{ addr, opcode, disassemble(instr, self._allocator) catch unreachable, instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).*, instr.nmd.d, instr.nd8.d, instr.d12.d });
        Opcodes[JumpTable[opcode]].fn_(self, instr);
        if (self.debug_trace)
            std.debug.print("[{X:0>4}] {X: >16} {s: <20}\tR{d: <2}={X:0>8}, R{d: <2}={X:0>8}\n", .{ addr, opcode, "", instr.nmd.n, self.R(instr.nmd.n).*, instr.nmd.m, self.R(instr.nmd.m).* });

        self.add_cycles(Opcodes[JumpTable[opcode]].issue_cycles);
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

    // Area 0:
    // 0x00000000 - 0x001FFFFF Boot ROM
    // 0x00200000 - 0x0021FFFF Flash Memory
    // 0x005F6800 - 0x005F69FF System Control Reg.
    // 0x005F6C00 - 0x005F6CFF Maple Control Reg.
    // 0x005F7000 - 0x005F70FF GD-ROM
    // 0x005F7400 - 0x005F74FF G1 Control Reg.
    // 0x005F7800 - 0x005F78FF G2 Control Reg.
    // 0x005F7C00 - 0x005F7CFF PVR Control Reg.
    // 0x005F8000 - 0x005F9FFF TA/PVR Core Reg.
    // 0x00600000 - 0x006007FF MODEM
    // 0x00700000 - 0x00707FFF AICA sound Reg.
    // 0x00710000 - 0x00710007 AICA RTC Reg.
    // 0x00800000 - 0x009FFFFF AICA Memory
    // 0x01000000 - 0x01FFFFFF G2 External Device #1
    // 0x02700000 - 0x02FFFFE0 G2 AICA (Image area)
    // 0x03000000 - 0x03FFFFE0 G2 External Device #2

    pub fn _get_memory(self: *@This(), addr: addr_t) *u8 {
        std.debug.assert(self.sr.md == 1 or is_p0(addr));

        std.debug.assert(addr == addr & 0x1FFFFFFF);

        if (false) {
            // MMU: Looks like most game don't use it at all. TODO: Expose it as an option.
            const physical_addr = self.mmu_translate_utbl(addr) catch |e| {
                // FIXME: Handle exceptions
                std.debug.print("\u{001B}[31mError in utlb _read: {any} at {X:0>8}\u{001B}[0m\n", .{ e, addr });
                unreachable;
            };

            if (physical_addr != addr)
                std.debug.print("  Write UTLB Hit: {x:0>8} => {x:0>8}\n", .{ addr, physical_addr });
        }

        if (addr < 0x04000000) {
            // Area 0 - Boot ROM, Flash ROM, Hardware Registers
            if (addr < 0x00200000) {
                return &self.boot[addr];
            }
            if (addr < 0x00200000 + 0x20000) {
                return &self.flash[addr - 0x200000];
            }

            if (addr < 0x005F6800) {
                std.debug.print(termcolor.red("  Unimplemented _get_memory to Area 0: {X:0>8}\n"), .{addr});
                unreachable;
            }

            if (addr >= 0x005F8000 and addr < 0x005FA000) {
                return self.gpu._get_register_from_addr(u8, addr);
            }

            if (addr < 0x00600000) {
                return &self.hardware_registers[addr - 0x005F6800];
            }

            if (addr >= 0x00700000 and addr <= 0x00707FE0) {
                // G2 AICA Register
                @panic("_get_memory to AICA Register. This should be handled in read/write functions.");
            }

            if (addr >= 0x00710000 and addr <= 0x00710008) {
                // G2 AICA RTC Registers
                @panic("_get_memory to AICA RTC Register. This should be handled in read/write functions.");
            }

            //                          is it 0x009FFFE0?
            if (addr >= 0x00800000 and addr < 0x00A00000) {
                // G2 Wave Memory
                return &aica.wave_memory[addr - 0x00800000];
            }

            std.debug.print("  \u{001B}[33mUnimplemented _get_memory to Area 0: {X:0>8}\u{001B}[0m\n", .{addr});
            return @ptrCast(&self._dummy);
        } else if (addr < 0x0800_0000) {
            return self.gpu._get_vram(addr);
        } else if (addr < 0x0C000000) {
            // Area 2 - Nothing
            std.debug.print("\u{001B}[31mInvalid _get_memory to Area 2: {X:0>8}\u{001B}[0m\n", .{addr});
            unreachable;
        } else if (addr < 0x10000000) {
            // Area 3 - System RAM (16MB) - 0x0C000000 to 0x0FFFFFFF, mirrored 4 times, I think.
            return &self.ram[addr & 0x00FFFFFF];
        } else if (addr < 0x14000000) {
            // Area 4 - Tile accelerator command input
            @panic("Unexpected _get_memory to Area 4 - This should only be accessible via write32 or DMA.");
        } else if (addr < 0x18000000) {
            // Area 5 - Expansion (modem) port
            std.debug.print("\u{001B}[33mUnimplemented _get_memory to Area 5: {X:0>8}\u{001B}[0m\n", .{addr});
            return &self.dummy_area5;
        } else if (addr < 0x1C000000) {
            // Area 6 - Nothing
            std.debug.print("\u{001B}[31mInvalid _get_memory to Area 6: {X:0>8}\u{001B}[0m\n", .{addr});
            unreachable;
        } else {
            // Area 7 - Internal I/O registers (same as P4)
            std.debug.assert(self.sr.md == 1);
            return &self.area7[addr - 0x1C000000];
        }
    }

    pub fn read8(self: @This(), virtual_addr: addr_t) u8 {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                else => {
                    if (self.debug_trace)
                        std.debug.print("  Read8 to hardware register @{X:0>8} {s} = 0x{X:0>2}\n", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr), @as(*const u8, @alignCast(@ptrCast(
                            @constCast(&self)._get_memory(addr),
                        ))).* });
                },
            }
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            if (self.debug_trace)
                std.debug.print("  Read8 to hardware register @{X:0>8} {s} \n", .{ addr, MemoryRegisters.getRegisterName(addr) });
        }

        return @as(*const u8, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub fn read16(self: @This(), virtual_addr: addr_t) u16 {
        const addr = virtual_addr & 0x1FFFFFFF;

        // SH4 Hardware registers
        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                @intFromEnum(MemoryRegister.RTCSR), @intFromEnum(MemoryRegister.RTCNT), @intFromEnum(MemoryRegister.RTCOR) => {
                    return @as(*const u16, @alignCast(@ptrCast(
                        @constCast(&self)._get_memory(addr),
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
                @intFromEnum(MemoryRegister.PDTRA) => {
                    // Note: I have absolutely no idea what's going on here.
                    //       This is directly taken from Flycast, which already got it from Chankast.
                    //       This is needed for the bios to work properly, without it, it will
                    //       go to sleep mode with all interrupts disabled early on.
                    const tpctra: u32 = self.read_io_register(u32, MemoryRegister.PCTRA);
                    const tpdtra: u32 = self.read_io_register(u32, MemoryRegister.PDTRA);

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

                    tfinal |= CableType << 8;

                    return tfinal;
                },
                else => {
                    if (self.debug_trace)
                        std.debug.print("  Read16 to hardware register @{X:0>8} {s} = {X:0>4}\n", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr), @as(*const u16, @alignCast(@ptrCast(
                            @constCast(&self)._get_memory(addr),
                        ))).* });
                },
            }
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            if (self.debug_trace)
                std.debug.print("  Read16 to hardware register @{X:0>8} {s} \n", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr) });
        }

        if (addr >= 0x00710000 and addr <= 0x00710008) {
            return @truncate(aica.read_rtc_register(addr));
        }

        return @as(*const u16, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub fn read32(self: @This(), virtual_addr: addr_t) u32 {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                else => {
                    if (self.debug_trace)
                        std.debug.print("  Read32 to hardware register @{X:0>8} {s} = 0x{X:0>8}\n", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr), @as(*const u32, @alignCast(@ptrCast(
                            @constCast(&self)._get_memory(addr),
                        ))).* });
                },
            }
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            if (self.debug_trace)
                std.debug.print("  Read32 to hardware register @{X:0>8} {s} = 0x{X:0>8}\n", .{ addr, MemoryRegisters.getRegisterName(addr), @as(*const u32, @alignCast(@ptrCast(
                    @constCast(&self)._get_memory(addr),
                ))).* });
        }

        if (addr >= 0x00700000 and addr <= 0x00707FE0) {
            return aica.read_register(addr);
        }

        if (addr >= 0x00710000 and addr <= 0x00710008) {
            return aica.read_rtc_register(addr);
        }

        return @as(*const u32, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub fn read64(self: @This(), virtual_addr: addr_t) u64 {
        const addr = virtual_addr & 0x1FFFFFFF;
        return @as(*const u64, @alignCast(@ptrCast(
            @constCast(&self)._get_memory(addr),
        ))).*;
    }

    pub fn write8(self: *@This(), virtual_addr: addr_t, value: u8) void {
        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                else => {
                    std.debug.print("  Write8 to hardware register @{X:0>8} {s} = 0x{X:0>2}\n", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr), value });
                },
            }
        }

        const addr = virtual_addr & 0x1FFFFFFF;
        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            // Hardware registers
            switch (addr) {
                else => {
                    std.debug.print("  Write8 to hardware register @{X:0>8} {s} = 0x{X:0>2}\n", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                },
            }
        }
        if (addr >= 0x005F8000 and addr < 0x005FA000) {
            @panic("write8 to GPU register not implemented");
        }
        if (addr >= 0x10000000 and addr < 0x14000000) {
            @panic("write8 to TA not implemented");
        }

        @as(*u8, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub fn write16(self: *@This(), virtual_addr: addr_t, value: u16) void {
        const addr = virtual_addr & 0x1FFFFFFF;

        if (virtual_addr >= 0xFF000000) {
            switch (virtual_addr) {
                @intFromEnum(MemoryRegister.RTCSR), @intFromEnum(MemoryRegister.RTCNT), @intFromEnum(MemoryRegister.RTCOR) => {
                    std.debug.assert(value & 0xFF00 == 0b10100101_00000000);
                    @as(*u16, @alignCast(@ptrCast(
                        self._get_memory(addr),
                    ))).* = 0b10100101_00000000 | (value & 0xFF);
                },
                @intFromEnum(MemoryRegister.RFCR) => {
                    std.debug.assert(value & 0b11111100_00000000 == 0b10100100_00000000);
                    @as(*u16, @alignCast(@ptrCast(
                        self._get_memory(addr),
                    ))).* = 0b10100100_00000000 | (value & 0b11_11111111);
                },
                else => {
                    std.debug.print("  Write16 to hardware register @{X:0>8} {s} = 0x{X:0>4}\n", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr), value });
                },
            }
        }

        if (addr >= 0x005F6800 and addr < 0x005F8000) {
            switch (addr) {
                else => {
                    std.debug.print("  Write16 to hardware register @{X:0>8} {s} = 0x{X:0>4}\n", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                },
            }
        }
        if (addr >= 0x005F8000 and addr < 0x005FA000) {
            @panic("write16 to GPU register not implemented");
        }
        if (addr >= 0x10000000 and addr < 0x14000000) {
            @panic("write16 to TA not implemented");
        }

        @as(*u16, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    fn store_queue_write(self: *@This(), virtual_addr: addr_t, value: u32) void {
        const sq_addr: StoreQueueAddr = @bitCast(virtual_addr);
        // std.debug.print("  StoreQueue write @{X:0>8} = 0x{X:0>8} ({any})\n", .{ virtual_addr, value, sq_addr });
        std.debug.assert(sq_addr.spec == 0b111000);
        self.store_queues[sq_addr.sq][sq_addr.lw_spec] = value;
    }

    pub fn write32(self: *@This(), virtual_addr: addr_t, value: u32) void {
        if (virtual_addr >= 0xE0000000) {
            // P4
            if (virtual_addr < 0xE4000000) {
                self.store_queue_write(virtual_addr, value);
                return;
            }
            if (virtual_addr >= 0xFF000000) {
                switch (virtual_addr) {
                    else => {
                        std.debug.print("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}\n", .{ virtual_addr, MemoryRegisters.getRegisterName(virtual_addr), value });
                    },
                }
            }
        }

        const addr = virtual_addr & 0x01FFFFFFF;
        if (addr >= 0x005F6800 and addr < 0x005F8000) {
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
                        aica.start_dma(self);
                    }
                },
                @intFromEnum(MemoryRegister.SB_MDAPRO) => {
                    // This register specifies the address range for Maple-DMA involving the system (work) memory.
                    // Check "Security code"
                    if (value & 0xFFFF0000 != 0x61550000) return;
                },
                @intFromEnum(MemoryRegister.SB_MDST) => {
                    if (value == 1) {
                        std.debug.print(termcolor.yellow("  Unimplemented Maple-DMA initiation !!\n"), .{});
                    }
                },
                @intFromEnum(MemoryRegister.SB_ISTNRM) => {
                    // Interrupt can be cleared by writing "1" to the corresponding bit.
                    self.hw_register(u32, MemoryRegister.SB_ISTNRM).* &= ~(value & 0x3FFFFF);
                    return;
                },
                @intFromEnum(MemoryRegister.SB_ISTERR) => {
                    // Interrupt can be cleared by writing "1" to the corresponding bit.
                    self.hw_register(u32, MemoryRegister.SB_ISTERR).* &= ~value;
                    return;
                },
                @intFromEnum(MemoryRegister.SB_C2DSTAT) => {
                    self.hw_register(u32, .SB_C2DSTAT).* = 0x10000000 | (0x03FFFFFF & value);
                    return;
                },
                @intFromEnum(MemoryRegister.SB_C2DST) => {
                    if (value == 1) {
                        self.start_ch2_dma();
                    } else {
                        self.end_ch2_dma();
                    }
                    return;
                },
                else => {
                    std.debug.print("  Write32 to hardware register @{X:0>8} {s} = 0x{X:0>8}\n", .{ addr, MemoryRegisters.getRegisterName(addr), value });
                },
            }
        }
        if (addr >= 0x005F8000 and addr < 0x005FA000) {
            return self.gpu.write_register(addr, value);
        }

        if (addr >= 0x00700000 and addr < 0x00710000) {
            return aica.write_register(addr, value);
        }

        if (addr >= 0x00710000 and addr < 0x00710008) {
            return aica.write_rtc_register(addr, value);
        }

        if (addr >= 0x10000000 and addr < 0x14000000) {
            return self.gpu.write_ta(addr, value);
        }

        @as(*u32, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    pub fn write64(self: *@This(), virtual_addr: addr_t, value: u64) void {
        const addr = virtual_addr & 0x01FFFFFFF;
        @as(*u64, @alignCast(@ptrCast(
            self._get_memory(addr),
        ))).* = value;
    }

    // FIXME: This should probably not be here.
    pub fn start_ch2_dma(self: *@This()) void {
        self.hw_register(u32, MemoryRegister.SB_C2DST).* = 1;

        const dst_addr = self.read_hw_register(u32, MemoryRegister.SB_C2DSTAT);
        const len = self.read_hw_register(u32, MemoryRegister.SB_C2DLEN);

        std.debug.print("  Start ch2-DMA: {X:0>8} -> {X:0>8} ({X:0>8} bytes)\n", .{ self.read_io_register(u32, MemoryRegister.SAR2), dst_addr, len });

        std.debug.assert(dst_addr & 0xF8000000 == 0x10000000);
        self.io_register(u32, MemoryRegister.DAR2).* = dst_addr; // FIXME: Not sure this is correct

        const dmac_len = self.read_io_register(u32, MemoryRegister.DMATCR2);
        std.debug.assert(32 * dmac_len == len);

        self.start_dmac(2);

        // TODO: Schedule for later?
        self.hw_register(u32, MemoryRegister.SB_C2DSTAT).* += len;
        self.hw_register(u32, MemoryRegister.SB_C2DLEN).* = 0;
        self.hw_register(u32, MemoryRegister.SB_C2DST).* = 0;

        self.raise_normal_interrupt(.{ .EoD_CH2 = 1 });
    }

    pub fn end_ch2_dma(self: *@This()) void {
        self.hw_register(u32, MemoryRegister.SB_C2DST).* = 0;

        // TODO: Actually cancel the DMA, right now they're instantaneous.
    }

    pub fn start_dmac(self: *@This(), channel: u32) void {
        std.debug.assert(channel == 2); // TODO: Implement others? It is needed?

        const chcr = self.read_io_register(MemoryRegisters.CHCR, .CHCR2);

        std.debug.print(" CHCR: {any}\n", .{chcr});
        // NOTE: I think the DC only uses 32 bytes transfers, but I'm not 100% sure.
        std.debug.assert(chcr.ts == 0b100);
        std.debug.assert(chcr.rs == 2); // "External request, single address mode"

        const src_addr = self.read_io_register(u32, MemoryRegister.SAR2) & 0x1FFFFFFF;
        const dst_addr = self.read_io_register(u32, MemoryRegister.DAR2) & 0x1FFFFFFF;
        const transfer_size: u32 = switch (chcr.ts) {
            0 => 8, // Quadword size
            1 => 1, // Byte
            2 => 2, // Word
            3 => 4, // Longword
            4 => 32, // 32-bytes block
            else => unreachable,
        };
        const len = self.read_io_register(u32, MemoryRegister.DMATCR2);
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
                    self.gpu.write_ta_fifo_polygon_path(src[0..8]);
                    src += 8;
                }
            }
            // YUV Converter Path
            if (dst_addr >= 0x10800000 and dst_addr < 0x11000000 or dst_addr >= 0x12800000 and dst_addr < 0x13000000) {
                self.gpu.ta_fifo_yuv_converter_path();
            }
            // Direct Texture Path
            if (dst_addr >= 0x11000000 and dst_addr < 0x12000000 or dst_addr >= 0x13000000 and dst_addr < 0x14000000) {
                self.gpu.write_ta_fifo_direct_texture_path(dst_addr, @as([*]u8, @ptrCast(self._get_memory(src_addr)))[0..byte_len]);
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
        self.io_register(u32, .SAR2).* += len;
        self.io_register(u32, .DAR2).* += len;
        self.io_register(u32, .DMATCR2).* = 0;
        self.io_register(MemoryRegisters.CHCR, .CHCR2).*.te = 1;
    }
};

fn zero_extend(d: anytype) u32 {
    return @intCast(d);
}

fn sign_extension_u8(d: u8) i32 {
    if ((d & 0x80) == 0) {
        return @bitCast(0x000000FF & zero_extend(d));
    } else {
        return @bitCast(0xFFFFFF00 | zero_extend(d));
    }
}

fn sign_extension_u12(d: u12) i32 {
    if ((d & 0x800) == 0) {
        return @bitCast(0x00000FFF & zero_extend(d));
    } else {
        return @bitCast(0xFFFFF000 | zero_extend(d));
    }
}

fn sign_extension_u16(d: u16) i32 {
    if ((d & 0x8000) == 0) {
        return @bitCast(0x0000FFFF & zero_extend(d));
    } else {
        return @bitCast(0xFFFF0000 | zero_extend(d));
    }
}

fn as_i32(val: u32) i32 {
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
    cpu.write32(cpu.R(opcode.nmd.n).* + d, cpu.R(opcode.nmd.m).*);
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
    cpu.write8(cpu.R(opcode.nmd.n).* + cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
}
fn movw_Rm_atR0Rn(cpu: *SH4, opcode: Instr) void {
    cpu.write16(cpu.R(opcode.nmd.n).* + cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
}
fn movl_Rm_atR0Rn(cpu: *SH4, opcode: Instr) void {
    cpu.write32(cpu.R(opcode.nmd.n).* + cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
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
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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
}

fn negc_Rm_Rn(cpu: *SH4, opcode: Instr) void {
    const tmp = 0 -% cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = tmp -% (if (cpu.sr.t) @as(u32, 1) else 0);
    cpu.sr.t = (0 < tmp);
    if (tmp < cpu.R(opcode.nmd.n).*)
        cpu.sr.t = true;
}

test "negc Rm,Rn" {
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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
    cpu.sr = @bitCast(cpu.R(opcode.nmd.n).* & 0x700083F3);
}
fn ldcl_at_Rn_inc_SR(cpu: *SH4, opcode: Instr) void {
    cpu.sr = @bitCast(cpu.read32(cpu.R(opcode.nmd.n).*) & 0x700083F3);
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
    if (cpu.sr.rb == 0) {
        cpu.r_bank1[opcode.nmd.m & 0b0111] = cpu.R(opcode.nmd.n).*;
    } else {
        cpu.r_bank0[opcode.nmd.m & 0b0111] = cpu.R(opcode.nmd.n).*;
    }
}
fn ldcl_at_Rn_inc_Rm_BANK(cpu: *SH4, opcode: Instr) void {
    if (cpu.sr.rb == 0) {
        cpu.r_bank1[opcode.nmd.m & 0b0111] = cpu.read32(cpu.R(opcode.nmd.n).*);
    } else {
        cpu.r_bank0[opcode.nmd.m & 0b0111] = cpu.read32(cpu.R(opcode.nmd.n).*);
    }
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
        std.debug.print("Note: ocbi @Rn not implemented\n", .{});
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
        std.debug.print("Note: obcp @Rn not implemented\n", .{});
    }
}

fn ocbwb_atRn(_: *SH4, _: Instr) void {
    const static = struct {
        var once = true;
    };
    if (static.once) {
        static.once = false;
        std.debug.print("Note: ocbwb @Rn not implemented\n", .{});
    }
}

// Reads a 32-byte data block starting at a 32-byte boundary into the operand cache.
// The lower 5 bits of the address specified by Rn are masked to zero.
// This instruction is also used to trigger a Store Queue write-back operation if the specified address points to the Store Queue area.
fn pref_atRn(cpu: *SH4, opcode: Instr) void {
    const addr = cpu.R(opcode.nmd.n).*;
    if (addr & 0xEC000000 == 0xE0000000) {
        // std.debug.print("pref @Rn: Transfer to External Memory from Store Queue\n", .{});
        if (cpu.read_io_register(mmu.MMUCR, .MMUCR).at == 1) {
            std.debug.print(termcolor.yellow("  MMU ON: Not implemented\n"), .{});
            @panic("pref @Rn with MMU ON: Not implemented");
        } else {
            const sq_addr: StoreQueueAddr = @bitCast(addr);
            std.debug.assert(sq_addr.spec == 0b111000);
            //               The full address also includes the sq bit.
            const ext_addr = (addr & 0x03FFFFE0) | (((cpu.read_io_register(u32, if (sq_addr.sq == 0) .QACR0 else .QACR1) & 0b11100) << 24));
            // std.debug.print("  Start Ext Addr: {X:0>8}; Addr: {any}\n", .{ ext_addr, sq_addr });
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
            std.debug.print("Note: pref @Rn not implemented\n", .{});
        }
    }
}
// Returns from an exception or interrupt handling routine by restoring the PC and SR values. Delayed jump.
fn rte(cpu: *SH4, _: Instr) void {
    const delay_slot = cpu.pc + 2;
    cpu.sr = @bitCast(cpu.ssr);
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
    if (cpu.sr.rb == 0) {
        cpu.R(opcode.nmd.n).* = cpu.r_bank1[opcode.nmd.m & 0b0111];
    } else {
        cpu.R(opcode.nmd.n).* = cpu.r_bank0[opcode.nmd.m & 0b0111];
    }
}
fn stcl_Rm_BANK_at_Rn_dec(cpu: *SH4, opcode: Instr) void {
    cpu.R(opcode.nmd.n).* -= 4;
    if (cpu.sr.rb == 0) {
        cpu.write32(cpu.R(opcode.nmd.n).*, cpu.r_bank1[opcode.nmd.m & 0b0111]);
    } else {
        cpu.write32(cpu.R(opcode.nmd.n).*, cpu.r_bank0[opcode.nmd.m & 0b0111]);
    }
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
        cpu.FR(opcode.nmd.n).* = @bitCast(cpu.read32(cpu.R(0).* + cpu.R(opcode.nmd.m).*));
    } else {
        @panic("Unimplemented");
    }
}
fn fmovs_FRm_at_R0_Rn(cpu: *SH4, opcode: Instr) void {
    if (cpu.fpscr.sz == 0) {
        cpu.write32(cpu.R(0).* + cpu.R(opcode.nmd.n).*, @bitCast(cpu.FR(opcode.nmd.m).*));
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
    var cpu = try SH4.init(std.testing.allocator);
    defer cpu.deinit();
    cpu.fpscr.pr = 0;
    fldi1_FRn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    std.debug.assert(@as(u32, @bitCast(cpu.FR(0).*)) == 0x3F800000);
}

fn fdls_FRn_FPUL(cpu: *SH4, opcode: Instr) void {
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
        cpu.DR(opcode.nmd.n).* = @sqrt(cpu.DR(opcode.nmd.n).*);
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
    // FIXME: There's a lot more to do here.
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @floatFromInt(cpu.fpul);
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        cpu.DR(opcode.nmd.n >> 1).* = @floatFromInt(cpu.fpul);
    }
}
fn ftrc_FRn_FPUL(cpu: *SH4, opcode: Instr) void {
    // Converts the single-precision floating-point number in FRm to a 32-bit integer, and stores the result in FPUL.
    // FIXME: There's a lot more to do here.
    if (cpu.fpscr.sz == 0) {
        cpu.fpul = std.math.lossyCast(u32, cpu.FR(opcode.nmd.n).*);
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        cpu.fpul = std.math.lossyCast(u32, cpu.DR(opcode.nmd.n >> 1).*);
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
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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

pub const Opcodes: [217]OpcodeDescription = .{
    .{ .code = 0b0000000000000000, .mask = 0b0000000000000000, .fn_ = nop, .name = "NOP", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    .{ .code = 0b0000000000000000, .mask = 0b1111111111111111, .fn_ = unknown, .name = "Unknown opcode", .privileged = false, .issue_cycles = 1, .latency_cycles = 1 },
    // Fake opcodes to catch emulated syscalls
    .{ .code = 0b0000000000010000, .mask = 0b0000000000000000, .fn_ = syscall.syscall_sysinfo, .name = "Syscall Sysinfo", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000000100000, .mask = 0b0000000000000000, .fn_ = syscall.syscall_romfont, .name = "Syscall ROMFont", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000000110000, .mask = 0b0000000000000000, .fn_ = syscall.syscall_flashrom, .name = "Syscall FlashROM", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000001000000, .mask = 0b0000000000000000, .fn_ = syscall.syscall_gdrom, .name = "Syscall GDROM", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000001010000, .mask = 0b0000000000000000, .fn_ = syscall.syscall, .name = "Syscall", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },
    .{ .code = 0b0000000001100000, .mask = 0b0000000000000000, .fn_ = syscall.syscall_misc, .name = "Syscall Misc.", .privileged = false, .issue_cycles = 0, .latency_cycles = 0 },

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
    .{ .code = 0b1100001100000000, .mask = 0b0000000011111111, .fn_ = unimplemented, .name = "trapa #imm", .privileged = false, .issue_cycles = 7, .latency_cycles = 7 },

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
    .{ .code = 0b1111000000011101, .mask = 0b0000111100000000, .fn_ = fdls_FRn_FPUL, .name = "flds FRm,FPUL", .privileged = false, .issue_cycles = 1, .latency_cycles = 0 },
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

fn write_and_execute(cpu: *SH4, code: u16) void {
    cpu.write16(cpu.pc, code);
    cpu.execute();
}

test "mov #imm,Rn" {
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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
    var cpu = try SH4.init(std.testing.allocator);
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
    try std.testing.expect(cpu.read16(0xA05F7480) == 0x400); // SB_G1RRC

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

test "IP.bin" {
    var cpu = try SH4.init(std.testing.allocator);
    defer cpu.deinit();

    // Example IP.bin file
    const IPbin_file = try std.fs.cwd().openFile("./bin/IP.bin", .{});
    defer IPbin_file.close();
    const IPbin = try IPbin_file.readToEndAlloc(std.testing.allocator, 0x10000);
    defer std.testing.allocator.free(IPbin);
    cpu.load_at(0x8C008000, IPbin);

    for (0..10000000) |_| {
        cpu.execute();
    }
}

test "IP.bin init boot" {
    var cpu = try SH4.init(std.testing.allocator);
    defer cpu.deinit();

    cpu.init_boot();

    // Example IP.bin file
    const IPbin_file = try std.fs.cwd().openFile("./bin/IP.bin", .{});
    defer IPbin_file.close();
    const IPbin = try IPbin_file.readToEndAlloc(std.testing.allocator, 0x10000);
    defer std.testing.allocator.free(IPbin);
    cpu.load_at(0x8C008000, IPbin);

    for (0..10000000) |_| {
        cpu.execute();
    }
}

// Loads a binary at 0x8C080000, set R14 to 1, and executes it until R14 == 0 (success condition)
fn load_and_test_binary(comptime filename: []const u8) !void {
    var cpu = try SH4.init(std.testing.allocator);
    defer cpu.deinit();

    const bin_file = try std.fs.cwd().openFile("./test/bin/" ++ filename, .{});
    defer bin_file.close();
    const bin = try bin_file.readToEndAlloc(std.testing.allocator, 0x10000);
    defer std.testing.allocator.free(bin);
    cpu.load_at(0x8C080000, bin);

    cpu.pc = 0x8C080000;
    cpu.R(14).* = 1;

    var prev = cpu.pc;
    while (cpu.R(14).* != 0) {
        cpu.execute();
        try std.testing.expect(cpu.pc != prev); // Crude check for infinite loops, there might be legitiple reason to do this (loops with process in delay slot?), but we'll just choose not to and be fine :))
        prev = cpu.pc;
    }
    try std.testing.expect(cpu.R(14).* == 0);
}

test "Binaries" {
    try load_and_test_binary("0.bin");
    try load_and_test_binary("stack_0.bin");
}
