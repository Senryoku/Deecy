const std = @import("std");
const builtin = @import("builtin");
const log = std.log.scoped(.dsp);

const host_memory = @import("host/host_memory.zig");

const IRBlock = @import("jit/ir_block.zig").IRBlock;
const Architecture = @import("jit/x86_64.zig");

const AICAModule = @import("aica.zig");

// HEAVILY based on Neill Corlett's Yamaha AICA notes, most of the comments come directly from there.

/// If disabled, registers that are not typically written to by the CPU are not reflected in their memory mapped location,
/// and instead use a simpler implementation.
const FullRegisterEmulation = false;

// NOTE: A bunch of right shifts here as expected to be arithmetic (i.e. signed), which contradicts current Zig documentation:
//         "Moves all bits to the right, inserting zeroes at the most-significant bit."
//       In practice however, right shifts on signed integers does appear to be arithmetic (as of Zig 0.14.0-dev.2577+271452d22).
//       Small checks in case this changes in the future :) (https://github.com/ziglang/zig/issues/20367)
comptime {
    std.debug.assert((@as(i16, -1) >> 2) < 0);
    std.debug.assert((@as(i16, -16) >> 2) == -4);
}

const Instruction = packed struct(u64) {
    _0_6: u7,
    NXADR: bool,
    ADREB: bool,
    MASA: u6,
    NOFL: bool,
    // -----
    BSEL: u1,
    ZERO: bool,
    NEGB: bool,
    YRL: bool,
    SHFT: u2,
    FRCL: bool,
    ADRL: bool,
    EWA: u4,
    EWT: bool,
    MRD: bool,
    MWT: bool,
    TABLE: u1,
    // -----
    _32: u1,
    IWA: u5,
    IWT: bool,
    IRA: u6,
    YSEL: u2,
    XSEL: u1,
    // -----
    _48: u1,
    TWA: u7,
    TWT: bool,
    TRA: u7,

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        try writer.writeAll(if (self.NXADR) "NXADR " else "      ");
        try writer.writeAll(if (self.ADREB) "ADREB " else "      ");
        try writer.print("MASA:{d: >2} ", .{self.MASA});
        try writer.writeAll(if (self.NOFL) "NOFL " else "     ");
        try writer.print("BSEL:{d} ", .{self.BSEL});
        try writer.writeAll(if (self.ZERO) "ZERO " else "     ");
        try writer.writeAll(if (self.NEGB) "NEGB " else "     ");
        try writer.writeAll(if (self.YRL) "YRL " else "    ");
        try writer.print("SHFT:{d} ", .{self.SHFT});
        try writer.writeAll(if (self.FRCL) "FRCL " else "     ");
        try writer.writeAll(if (self.ADRL) "ADRL " else "     ");
        try writer.print("EWA:{d} ", .{self.EWA});
        try writer.writeAll(if (self.EWT) "EWT " else "    ");
        try writer.writeAll(if (self.MRD) "MRD " else "    ");
        try writer.writeAll(if (self.MWT) "MWT " else "    ");
        try writer.print("TABLE:{d} ", .{self.TABLE});
        try writer.print("IWA:{d: >2} ", .{self.IWA});
        try writer.writeAll(if (self.IWT) "IWT " else "    ");
        try writer.print("IRA:{d: >2} ", .{self.IRA});
        try writer.print("YSEL:{d} ", .{self.YSEL});
        try writer.print("XSEL:{d} ", .{self.XSEL});
        try writer.print("TWA:{d: >3} ", .{self.TWA});
        try writer.writeAll(if (self.TWT) "TWT " else "    ");
        try writer.print("TRA:{d: >3} ", .{self.TRA});
    }
};

_regs: []u32, // Memory backing for internal registers
_memory: []u8,
_ring_buffer: *const AICAModule.RingBufferAddress,

_dirty_mpro: bool = true,
_jit_buffer: ?[]align(std.heap.page_size_min) u8 = null,
_allocator: std.mem.Allocator,

pub fn init(ring_buffer: *const AICAModule.RingBufferAddress, registers: []u32, memory: []u8, allocator: std.mem.Allocator) @This() {
    return .{
        ._ring_buffer = ring_buffer,
        ._regs = registers,
        ._memory = memory,
        ._allocator = allocator,
    };
}

pub fn deinit(self: *@This()) void {
    if (self._jit_buffer) |buffer| self._allocator.free(buffer);
}

pub inline fn read_register(self: *const @This(), comptime T: type, local_addr: u32) T {
    if (local_addr >= 4 * MDEC_CT_base)
        log.warn("Read to DSP register {X:0>4}: We use it for internal operations!", .{local_addr});

    return switch (T) {
        u8 => @as([*]const u8, @ptrCast(&self._regs[0]))[local_addr],
        u32 => self._regs[(local_addr) / 4],
        else => @compileError("Invalid value type"),
    };
}

pub inline fn write_register(self: *@This(), comptime T: type, local_addr: u32, value: T) void {
    if (local_addr >= 0x0400 and local_addr < 0x0C00)
        self._dirty_mpro = true;

    if (local_addr >= 4 * MDEC_CT_base)
        log.warn("Write to DSP register {X:0>4}: We use it for internal operations!", .{local_addr});

    switch (T) {
        u8 => @as([*]u8, @ptrCast(self._regs.ptr))[local_addr] = value,
        u32 => self._regs[local_addr / 4] = value & 0xFFFF,
        else => @compileError("Invalid value type"),
    }
}

fn read(self: *@This(), comptime T: type, addr: u32) T {
    log.debug("Read({any}): {X}", .{ T, addr });
    return std.mem.bytesAsValue(T, self._memory[addr & 0x1FFFFF ..]).*;
}

fn write(self: *@This(), comptime T: type, addr: u32, value: T) void {
    log.debug("Write({any}): {X} = {X}", .{ T, addr, value });
    std.mem.bytesAsValue(T, self._memory[addr & 0x1FFFFF ..]).* = value;
}

/// 16-bit unsigned register which is decremented on every sample
fn MDEC_CT(self: *@This()) *u16 {
    return @ptrCast(&self._regs[MDEC_CT_base]);
}

// There aren't actually memory mapped (that I know of), but we have the
// memory, might as well use it. It also simplifies the JIT a bit: we
// can use the register array as base for all memory operations.
//   Added a warning in read/write functions just in case.
const MDEC_CT_base = 0x1600 / 4;
const TEMP_MEM_base = 0x1604 / 4;

/// 0x3000-0x31FF: Coefficients (COEF), 128 registers, 13 bits each
///               0x3000: bits 15-3 = COEF(0)
///               0x3004: bits 15-3 = COEF(1)
///               ...
///               0x31FC: bits 15-3 = COEF(127)
///               You could interpret these as 16-bit signed (2's complement)
///               coefficients with the lowest 3 bits are always 0.
///               Each of the 128 COEFs is used by the corresponding instruction (MPRO).
pub fn read_coef(self: *@This(), idx: usize) i13 {
    const u: u16 = @truncate(self._regs[idx]);
    const s: i16 = @bitCast(u);
    return @intCast(s >> 3);
}

/// 0x3200-0x32FF: External memory addresses (MADRS), 64 registers, 16 bits each
///                0x3200: bits 15-0 = MADRS(0)
///                ...
///                0x3204: bits 15-0 = MADRS(1)
///                0x32FC: bits 15-0 = MADRS(63)
///                These are memory offsets that refer to locations in the
///                external ringbuffer.  Every increment of a MADRS register
///                represents 2 bytes.
const MADRS_base: u32 = 0x200 / 4;
fn _madrs(self: *@This(), idx: usize) *u16 {
    return @ptrCast(&self._regs[MADRS_base + idx]);
}

/// 0x3400-0x3BFF: DSP program (MPRO), 128 registers, 64 bits each
///                0x3400: bits 15-0 = bits 63-48 of first instruction
///                0x3404: bits 15-0 = bits 47-32 of first instruction
///                0x3408: bits 15-0 = bits 31-16 of first instruction
///                0x340C: bits 15-0 = bits 15-0  of first instruction
///                0x3410: bits 15-0 = bits 63-48 of second instruction
///                ...
///                0x3BFC: bits 15-0 = bits 15-0  of last instruction
const MPRO_base: u32 = 0x400 / 4;
pub fn read_mpro(self: *@This(), idx: usize) Instruction {
    const parts = [4]u16{
        @truncate(self._regs[MPRO_base + 4 * idx + 0]),
        @truncate(self._regs[MPRO_base + 4 * idx + 1]),
        @truncate(self._regs[MPRO_base + 4 * idx + 2]),
        @truncate(self._regs[MPRO_base + 4 * idx + 3]),
    };
    return @bitCast((@as(u64, parts[0]) << 48) | (@as(u64, parts[1]) << 32) | (@as(u64, parts[2]) << 16) | (@as(u64, parts[3]) << 0));
}

/// 0x4000-0x43FF: Temp buffer (TEMP), 128 registers, 24 bits each
///                0x4000: bits 7-0  = bits 7-0 of TEMP(0)
///                0x4004: bits 15-0 = bits 23-8 of TEMP(0)
///                0x4008: bits 7-0  = bits 7-0 of TEMP(1)
///                ...
///                0x43FC: bits 15-0 = bits 23-8 of TEMP(127)
///                The temp buffer is configured as a ring buffer, so pointers
///                referring to it decrement by 1 each sample.
const TEMP_base: u32 = 0x1000 / 4;
fn get_temp_base_ptr(self: *@This()) [*]align(4) i24 {
    return @as([*]align(4) i24, @ptrCast(&self._regs[TEMP_base]));
}
fn read_temp(self: *@This(), idx: usize) i24 {
    if (FullRegisterEmulation) {
        const lo = self._regs[TEMP_base + 2 * idx + 0];
        const hi = self._regs[TEMP_base + 2 * idx + 1];
        const u: u24 = @truncate(((hi & 0xFFFF) << 8) | (lo & 0xFF));
        return @bitCast(u);
    } else {
        return self.get_temp_base_ptr()[idx];
    }
}
fn write_temp(self: *@This(), idx: usize, value: i24) void {
    if (FullRegisterEmulation) {
        const u: u24 = @bitCast(value);
        self._regs[TEMP_base + 2 * idx + 0] = u & 0xFF;
        self._regs[TEMP_base + 2 * idx + 1] = (u >> 8) & 0xFFFF;
    } else {
        self.get_temp_base_ptr()[idx] = value;
    }
}

// 0x4400-0x44FF: Memory data (MEMS), 32 registers, 24 bits each
//                0x4400: bits 7-0  = bits 7-0 of MEMS(0)
//                0x4404: bits 15-0 = bits 23-8 of MEMS(0)
//                0x4408: bits 7-0  = bits 7-0 of MEMS(1)
//                ...
//                0x44FC: bits 15-0 = bits 23-8 of MEMS(31)
//                Used for holding data that was read out of the ringbuffer.
const MEMS_base: u32 = 0x1400 / 4;
fn get_mems_base_ptr(self: *@This()) [*]align(4) u24 {
    return @as([*]align(4) u24, @ptrCast(&self._regs[MEMS_base]));
}
fn read_mems(self: *@This(), idx: usize) u24 {
    if (FullRegisterEmulation) {
        const lo = self._regs[MEMS_base + 2 * idx + 0];
        const hi = self._regs[MEMS_base + 2 * idx + 1];
        return @truncate(((hi & 0xFFFF) << 8) | (lo & 0xFF));
    } else {
        return self.get_mems_base_ptr()[idx];
    }
}
fn write_mems(self: *@This(), idx: usize, value: u24) void {
    if (FullRegisterEmulation) {
        self._regs[MEMS_base + 2 * idx + 0] = value & 0xFF;
        self._regs[MEMS_base + 2 * idx + 1] = (value >> 8) & 0xFFFF;
    } else {
        self.get_mems_base_ptr()[idx] = value;
    }
}

// 0x4500-0x457F: Mixer input data (MIXS), 16 registers, 20 bits each
//                0x4500: bits 3-0  = bits 3-0 of MIXS(0)
//                0x4504: bits 15-0 = bits 19-4 of MIXS(0)
//                0x4508: bits 3-0  = bits 3-0 of MIXS(1)
//                ...
//                0x457C: bits 15-0 = bits 19-4 of MIXS(15)
//                These are the 16 send buses coming from the 64 main channels.
const MIXS_base: u32 = 0x1500 / 4;
fn get_mixs_base_ptr(self: *@This()) [*]align(4) i20 {
    return @as([*]align(4) i20, @ptrCast(&self._regs[MIXS_base]));
}
pub fn read_mixs(self: *@This(), idx: usize) i20 {
    if (FullRegisterEmulation) {
        const lo = self._regs[MIXS_base + 2 * idx + 0];
        const hi = self._regs[MIXS_base + 2 * idx + 1];
        const u: u20 = @truncate(((hi & 0xFFFF) << 4) | (lo & 0xF));
        return @bitCast(u);
    } else {
        return self.get_mixs_base_ptr()[idx];
    }
}
fn write_mixs(self: *@This(), idx: usize, value: i20) void {
    if (FullRegisterEmulation) {
        const u: u20 = @bitCast(value);
        self._regs[MIXS_base + 2 * idx + 0] = u & 0xF;
        self._regs[MIXS_base + 2 * idx + 1] = (u >> 4) & 0xFFFF;
    } else {
        self.get_mixs_base_ptr()[idx] = value;
    }
}
pub fn add_mixs(self: *@This(), idx: usize, value: i20) void {
    self.write_mixs(idx, self.read_mixs(idx) + value);
}

// 0x4580-0x45BF: Effect output data (EFREG), 16 registers, 16 bits each
//                0x4580: bits 15-0 = EFREG(0)
//                0x4584: bits 15-0 = EFREG(1)
//                ...
//                0x45BC: bits 15-0 = EFREG(15)
//                These are the 16 sound outputs.
const EFREG_base: u32 = 0x1580 / 4;
fn _efreg(self: *@This(), idx: usize) *i16 {
    return @ptrCast(&self._regs[EFREG_base + idx]);
}
pub fn read_efreg(self: *@This(), idx: usize) i16 {
    return self._efreg(idx).*;
}

// 0x45C0-0x45C7: External input data stack (EXTS), 2 registers, 16 bits each
//                0x45C0: bits 15-0 = EXTS(0)
//                0x45C4: bits 15-0 = EXTS(1)
//                These come from CDDA left and right, respectively.
const EXTS_base: u32 = 0x15C0 / 4;
fn _exts(self: *@This(), idx: usize) *u16 {
    return @ptrCast(&self._regs[EXTS_base + idx]);
}

pub fn set_exts(self: *@This(), idx: usize, value: u16) void {
    self._exts(idx).* = value;
}

fn clear_mixs(self: *@This()) void {
    @memset(self._regs[MIXS_base .. MIXS_base + 2 * 16], 0);
}
fn clear_efreg(self: *@This()) void {
    @memset(self._regs[EFREG_base .. EFREG_base + 16], 0);
}

fn saturate(comptime T: type, value: anytype) T {
    std.debug.assert(@bitSizeOf(T) < @bitSizeOf(@TypeOf(value)));
    return @intCast(@max(@min(value, @as(@TypeOf(value), std.math.maxInt(T))), @as(@TypeOf(value), std.math.minInt(T))));
}

test {
    try std.testing.expectEqual(saturate(i24, @as(i26, 0)), 0);
    try std.testing.expectEqual(saturate(i24, @as(i26, 1)), 1);
    try std.testing.expectEqual(saturate(i24, @as(i26, -1234)), -1234);
    try std.testing.expectEqual(saturate(i24, @as(i26, 1234)), 1234);
    try std.testing.expectEqual(saturate(i24, @as(i26, -0x800000)), -0x800000);
    try std.testing.expectEqual(saturate(i24, @as(i26, -0x800001)), -0x800000);
    try std.testing.expectEqual(saturate(i24, @as(i26, -0x8FFFFF)), -0x800000);
    try std.testing.expectEqual(saturate(i24, @as(i26, 0x7FFFFF)), 0x7FFFFF);
    try std.testing.expectEqual(saturate(i24, @as(i26, 0x800000)), 0x7FFFFF);
    try std.testing.expectEqual(saturate(i24, @as(i26, 0xFFFFFF)), 0x7FFFFF);
}

pub fn compile(self: *@This()) !void {
    std.debug.assert(!FullRegisterEmulation); // TODO: Not supported yet.

    if (self._jit_buffer == null)
        self._jit_buffer = try host_memory.allocate_executable(self._allocator, 0x8000);

    var b = try IRBlock.init(self._allocator);
    defer b.deinit();

    const EAX: Architecture.Operand = .{ .reg = .rax };

    const RegistersBase = Architecture.Register.rbp;
    const ACC: Architecture.Operand = .{ .reg = Architecture.SavedRegisters[0] };
    const Y_REG: Architecture.Operand = .{ .reg = Architecture.SavedRegisters[1] };
    const FRC_REG: Architecture.Operand = .{ .reg = Architecture.SavedRegisters[2] };
    const ADRS_REG: Architecture.Operand = .{ .reg = Architecture.SavedRegisters[3] };

    const SHIFTED: Architecture.Operand = .{ .reg = Architecture.SavedRegisters[4] };

    // Should be 16-bits, be easier this way.
    const MDEC_CT_op: Architecture.Operand = .{ .mem = .{ .base = RegistersBase, .displacement = 4 * MDEC_CT_base, .size = 32 } };

    for (0..5) |i| {
        try b.push(.{ .reg64 = Architecture.SavedRegisters[i] });
    }
    try b.push(.{ .reg64 = Architecture.SavedRegisters[4] }); // Ensure stack alignment

    try b.mov(.{ .reg = RegistersBase }, .{ .reg = Architecture.ArgRegisters[0] });

    try b.mov(ACC, .{ .imm32 = 0 });
    try b.mov(Y_REG, .{ .imm32 = 0 });
    try b.mov(FRC_REG, .{ .imm32 = 0 });
    try b.mov(ADRS_REG, .{ .imm32 = 0 });

    // Look for instructions at the end of the program that don't write anything and thus
    // have no visible effect outside of internal states, and discard them.
    var max_step: u32 = 128;
    while (max_step > 0) {
        const instruction = self.read_mpro(max_step - 1);
        if (!instruction.IWT and !instruction.TWT and !instruction.MWT and !instruction.EWT) {
            max_step -= 1;
        } else {
            break;
        }
    }

    for (0..max_step) |step| {
        const instruction = self.read_mpro(@intCast(step));

        const INPUTS: Architecture.Operand = .{ .reg = Architecture.ArgRegisters[0] };

        switch (instruction.IRA) {
            0x00...0x1F => |reg| {
                try b.mov(INPUTS, .{ .mem = .{ .base = RegistersBase, .displacement = 4 * (MEMS_base + reg), .size = 32 } });
                // Sign extend from i24 to i32
                try b.shl(INPUTS, .{ .imm8 = 8 });
                try b.sar(INPUTS, 8);
            },
            0x20...0x2F => |reg| {
                try b.mov(INPUTS, .{ .mem = .{ .base = RegistersBase, .displacement = 4 * (MIXS_base + (reg - 0x20)), .size = 32 } });
                // Shift 4 left then sign extend from i24 to i32
                try b.shl(INPUTS, .{ .imm8 = 4 + 8 });
                try b.sar(INPUTS, 8);
            },
            0x30...0x31 => |reg| {
                try b.movsx(INPUTS, .{ .mem = .{ .base = RegistersBase, .displacement = 4 * (EXTS_base + (reg - 0x30)), .size = 16 } });
                try b.shl(INPUTS, .{ .imm8 = 8 });
            },
            else => {
                try b.mov(INPUTS, .{ .imm32 = 0 });
            },
        }

        // NOTE: IWT is handled at the end of the step for simplicity (allow calling functions without worrying about registers)

        {
            const X: Architecture.Operand = .{ .reg = Architecture.ArgRegisters[1] };
            const Y: Architecture.Operand = .{ .reg = Architecture.ArgRegisters[2] };
            const B: Architecture.Operand = .{ .reg = Architecture.ArgRegisters[3] };

            // Load TEMP[TRA + MDEC_CT] for X, B, or both. Optimizes the 'both' case, seems unlikely, but idk.
            const b_temp = (!instruction.ZERO and instruction.BSEL == 0);
            const x_temp = instruction.XSEL == 0;
            if (b_temp or x_temp) {
                try b.mov(EAX, .{ .imm32 = instruction.TRA });
                try b.add(EAX, MDEC_CT_op);
                try b.append(.{ .And = .{ .dst = EAX, .src = .{ .imm32 = 0x7F } } });
                const TRA_op: Architecture.Operand = .{ .mem = .{ .base = RegistersBase, .index = EAX.reg, .scale = ._4, .displacement = 4 * TEMP_base, .size = 32 } };
                if (x_temp and b_temp) {
                    try b.mov(X, TRA_op);
                    try b.mov(B, X);
                } else if (x_temp) {
                    try b.mov(X, TRA_op);
                } else {
                    try b.mov(B, TRA_op);
                }
            }

            if (instruction.ZERO) {
                // We'll simply skip the addition later
            } else {
                if (instruction.BSEL == 0) {
                    // Already loaded
                } else {
                    try b.mov(B, ACC);
                }

                if (instruction.NEGB)
                    try b.append(.{ .Neg = .{ .dst = B } });
            }

            if (instruction.XSEL == 0) {
                // Already loaded
            } else {
                try b.mov(X, INPUTS);
            }

            switch (instruction.YSEL) {
                0 => {
                    try b.mov(Y, FRC_REG);
                },
                1 => { // Y = COEF[step]
                    try b.movsx(Y, .{ .mem = .{ .base = RegistersBase, .displacement = @intCast(4 * step), .size = 16 } });
                    try b.sar(Y, 3);
                },
                2 => {
                    try b.mov(Y, Y_REG);
                    try b.sar(Y, 11);
                },
                3 => {
                    try b.mov(Y, Y_REG);
                    try b.sar(Y, 4);
                    try b.append(.{ .And = .{ .dst = Y, .src = .{ .imm32 = 0xFFF } } });
                },
            }

            if (instruction.YRL)
                try b.mov(Y_REG, INPUTS);

            try b.mov(SHIFTED, ACC);
            if (instruction.SHFT == 1 or instruction.SHFT == 2) {
                try b.shl(SHIFTED, .{ .imm8 = 1 });
            }
            if (instruction.SHFT == 0 or instruction.SHFT == 1) {
                // Saturate
                try b.mov(EAX, .{ .imm32 = @bitCast(@as(i32, -0x800000)) });
                try b.append(.{ .Cmp = .{ .lhs = SHIFTED, .rhs = EAX } });
                try b.cmov(.Less, SHIFTED, EAX);

                try b.mov(EAX, .{ .imm32 = 0x7FFFFF });
                try b.append(.{ .Cmp = .{ .lhs = SHIFTED, .rhs = EAX } });
                try b.cmov(.Greater, SHIFTED, EAX);
            } else {
                // Truncate to 24 bits.
                try b.shl(SHIFTED, .{ .imm8 = 8 });
                try b.sar(SHIFTED, 8);
            }

            // Compute ACC = ((X * Y) >> 12) + B
            try b.movsx(.{ .reg64 = ACC.reg }, X);
            try b.movsx(.{ .reg64 = Y.reg }, Y);
            try b.append(.{ .Mul = .{ .dst = .{ .reg64 = ACC.reg }, .src = .{ .reg64 = Y.reg } } });
            try b.sar(.{ .reg64 = ACC.reg }, 12);
            if (!instruction.ZERO)
                try b.add(ACC, B);
        }

        if (instruction.TWT) {
            try b.mov(EAX, .{ .imm32 = instruction.TWA });
            try b.add(EAX, MDEC_CT_op);
            try b.append(.{ .And = .{ .dst = EAX, .src = .{ .imm32 = 0x7F } } });
            try b.mov(.{ .mem = .{ .base = RegistersBase, .index = EAX.reg, .scale = ._4, .displacement = 4 * TEMP_base, .size = 32 } }, SHIFTED);
        }

        if (instruction.FRCL) {
            try b.mov(FRC_REG, SHIFTED);
            switch (instruction.SHFT) {
                3 => {
                    try b.append(.{ .And = .{ .dst = FRC_REG, .src = .{ .imm32 = 0xFFF } } });
                },
                else => {
                    try b.shr(FRC_REG, 11);
                    try b.append(.{ .And = .{ .dst = FRC_REG, .src = .{ .imm32 = 0x1FFF } } });
                },
            }
        }

        if (instruction.EWT) {
            try b.mov(EAX, SHIFTED);
            try b.sar(EAX, 8);
            try b.mov(.{ .mem = .{ .base = RegistersBase, .displacement = 4 * (EFREG_base + instruction.EWA), .size = 16 } }, EAX);
        }

        // Out-of-order sections

        if (instruction.MRD or instruction.MWT) {
            const addr: Architecture.Operand = .{ .reg = Architecture.ScratchRegisters[0] };

            try b.mov(addr, .{ .mem = .{ .base = RegistersBase, .displacement = 4 * (MADRS_base + instruction.MASA), .size = 32 } });
            if (instruction.TABLE == 0)
                try b.add(addr, MDEC_CT_op);
            if (instruction.ADREB)
                try b.add(addr, ADRS_REG);
            if (instruction.NXADR)
                try b.add(addr, .{ .imm8 = 1 });
            // NOTE: We assume ring_buffer is constant here, we don't track it.
            const mask: u32 = if (instruction.TABLE == 0)
                switch (self._ring_buffer.size) {
                    .@"8k" => 0x1FFF,
                    .@"16k" => 0x3FFF,
                    .@"32k" => 0x7FFF,
                    .@"64k" => 0xFFFF,
                }
            else
                0xFFFF;
            try b.append(.{ .And = .{ .dst = addr, .src = .{ .imm32 = mask } } });
            try b.shl(addr, .{ .imm8 = 1 });
            try b.add(addr, .{ .imm32 = @as(u32, self._ring_buffer.pointer) << 11 });
            try b.append(.{ .And = .{ .dst = addr, .src = .{ .imm32 = 0x1FFFFF } } });

            if (instruction.MRD) {
                try b.mov(.{ .reg64 = .rax }, .{ .imm64 = @intFromPtr(self._memory.ptr) });
                try b.mov(EAX, .{ .mem = .{ .base = .rax, .index = addr.reg, .size = 16 } });
                try b.mov(.{ .mem = .{ .base = RegistersBase, .displacement = @intCast(4 * (TEMP_MEM_base + (step % 4))), .size = 16 } }, EAX);
            }

            if (instruction.MWT) {
                if (instruction.NOFL) {
                    try b.mov(EAX, SHIFTED);
                    try b.sar(EAX, 8);
                } else {
                    try b.push(.{ .reg64 = addr.reg }); // Preserve addr and INPUTS through the call
                    try b.push(.{ .reg64 = INPUTS.reg });

                    try b.mov(.{ .reg = Architecture.ArgRegisters[0] }, SHIFTED);
                    try b.append(.{ .And = .{ .dst = .{ .reg = Architecture.ArgRegisters[0] }, .src = .{ .imm32 = 0xFFF } } });

                    try b.call(f16_from_i32);

                    try b.pop(.{ .reg64 = INPUTS.reg });
                    try b.pop(.{ .reg64 = addr.reg });
                }
                try b.mov(.{ .reg64 = Architecture.ArgRegisters[0] }, .{ .imm64 = @intFromPtr(self._memory.ptr) });
                try b.mov(.{ .mem = .{ .base = Architecture.ArgRegisters[0], .index = addr.reg, .size = 16 } }, EAX);
            }
        }

        // Should be after the read/write section (only one that uses ADRS_REG)
        if (instruction.ADRL) {
            if (instruction.SHFT == 3) {
                try b.mov(ADRS_REG, INPUTS);
                try b.sar(FRC_REG, 16);
            } else {
                try b.mov(ADRS_REG, SHIFTED);
                try b.sar(FRC_REG, 12);
            }
            try b.append(.{ .And = .{ .dst = ADRS_REG, .src = .{ .imm32 = 0xFFF } } });
        }

        if (instruction.IWT) {
            const TEMP_MEM_op: Architecture.Operand = .{ .mem = .{ .base = RegistersBase, .displacement = @intCast(4 * (TEMP_MEM_base + ((step - 2) % 4))), .size = 16 } };
            if (self.read_mpro(step - 2).NOFL) {
                try b.mov(EAX, TEMP_MEM_op);
                try b.shl(EAX, .{ .imm8 = 8 });
            } else {
                try b.mov(.{ .reg = Architecture.ArgRegisters[0] }, TEMP_MEM_op);
                try b.call(i32_from_f16);
            }
            try b.mov(.{ .mem = .{ .base = RegistersBase, .displacement = 4 * (MEMS_base + instruction.IWA), .size = 32 } }, EAX);
        }
    }

    try b.pop(.{ .reg64 = Architecture.SavedRegisters[4] });
    for (0..5) |i| {
        try b.pop(.{ .reg64 = Architecture.SavedRegisters[4 - i] });
    }

    const block_size = try b.emit(self._jit_buffer.?);

    for (b.instructions.items, 0..) |instr, idx|
        log.debug("[{d: >4}] {f}", .{ idx, instr });
    log.debug("Compiled: {X}", .{self._jit_buffer.?[0..block_size]});

    self._dirty_mpro = false;
}

pub fn generate_sample_jit(self: *@This()) !void {
    if (self._dirty_mpro) try self.compile();

    self.clear_efreg();

    if (self._regs[MDEC_CT_base] == 0)
        self._regs[MDEC_CT_base] = @intCast(self._ring_buffer.size_in_samples() - 1);

    if (self._jit_buffer) |buffer|
        @as(*const fn ([*]u32) callconv(Architecture.CallingConvention) void, @ptrCast(&buffer[0]))(self._regs.ptr);

    self._regs[MDEC_CT_base] -= 1;

    self.clear_mixs();
}

pub fn generate_sample(self: *@This()) void {
    // 26-bit signed accumulator
    var ACC: i26 = 0;
    // 24-bit signed latch register
    var Y_REG: i24 = 0;
    // 13-bit fractional address
    var FRC_REG: u13 = 0;
    // 12-bit whole address
    var ADRS_REG: u12 = 0;

    var temp_word: [4]u16 = @splat(0);

    self.clear_efreg();

    if (self.MDEC_CT().* == 0)
        self.MDEC_CT().* = @intCast(self._ring_buffer.size_in_samples() - 1);

    for (0..128) |step| {
        const instruction = self.read_mpro(@intCast(step));

        // If the source register is less than 24 bits, it's promoted to 24-bit by shifting left.
        const INPUTS: i24 = switch (instruction.IRA) {
            0x00...0x1F => |reg| @bitCast(self.read_mems(reg)),
            0x20...0x2F => |reg| @as(i24, @intCast(self.read_mixs(reg - 0x20))) << 4,
            0x30...0x31 => |reg| @bitCast(@as(u24, self._exts(reg - 0x30).*) << 8),
            else => 0,
        };

        if (instruction.IWT) {
            // If IWT is set, then the memory data from a MRD (either 2 or 3 instructions ago) is copied into the MEMS register indicated by IWA (0x00-0x1F).
            // If NOFL was set 2 instructions ago (only), the value is simply promoted from 16 to 24-bit by shifting left.
            // Otherwise, it's converted from 16-bit float format to 24-bit integer.  (See float notes)
            std.debug.assert(step >= 2);
            const temp = temp_word[(step - 2) % temp_word.len];
            const value: u24 = if (self.read_mpro(step - 2).NOFL)
                @truncate(@as(u24, temp) << 8)
            else
                @bitCast(i24_from_f16(temp));
            self.write_mems(instruction.IWA, value);
        }

        // - B selection
        //   A 26-bit value (B) is read from one of two sources:
        //     If BSEL=0: The TEMP register indicated by ((TRA + MDEC_CT) & 0x7F).
        //                This value is sign-extended from 24 to 26 bits, not shifted.
        //     If BSEL=1: The accumulator (ACC), already 26 bits.
        // 26-bit signed addend (Always overwritten)
        var B: i26 = if (instruction.BSEL == 0)
            @intCast(self.read_temp((instruction.TRA +% self.MDEC_CT().*) & 0x7F))
        else
            ACC;
        // If NEGB=1, this value is then made negative. (B becomes 0-B)
        if (instruction.NEGB)
            B = -B;
        // If ZERO=1, this value becomes zero, regardless of all other bits.
        if (instruction.ZERO)
            B = 0;

        //- X selection
        //   A 24-bit value (X) is read from one of two sources:
        //     If XSEL=0: One of the TEMP registers indicated by ((TRA + MDEC_CT) & 0x7F).
        //     If XSEL=1: The INPUTS register.
        // 24-bit signed multiplicand (Always overwritten)
        const X: i24 = if (instruction.XSEL == 0)
            self.read_temp((instruction.TRA +% self.MDEC_CT().*) & 0x7F)
        else
            INPUTS;

        // - Y selection
        //   A 13-bit value (Y) is read from one of four sources:
        //     If YSEL=0: Y becomes FRC_REG
        //     If YSEL=1: Y becomes this instruction's COEF
        //     If YSEL=2: Y becomes bits 23-11 of Y_REG
        //     If YSEL=3: Y becomes bits 15-4 of Y_REG, ZERO-EXTENDED to 13 bits.
        // 13-bit signed multiplier (Always overwritten)
        const Y: i13 = switch (instruction.YSEL) {
            0 => @bitCast(FRC_REG),
            1 => self.read_coef(step),
            2 => @truncate(Y_REG >> 11),
            3 => @truncate((Y_REG >> 4) & 0xFFF),
        };

        // - Y latch
        //   If YRL is set, Y_REG becomes INPUTS.
        if (instruction.YRL)
            Y_REG = INPUTS;

        // - Shift of previous accumulator
        //   A 24-bit value (SHIFTED) is set to one of the following:
        //     If SHFT=0: SHIFTED becomes ACC, with saturation
        //     If SHFT=1: SHIFTED becomes ACC*2, with saturation
        //     If SHFT=2: SHIFTED becomes ACC*2, with the highest bits simply cut off
        //     If SHFT=3: SHIFTED becomes ACC, with the highest bits simply cut off
        //   If saturation is enabled, SHIFTED is clipped to the range -0x800000...0x7FFFFF.
        // 24-bit signed shifted output value (Always overwritten)
        const SHIFTED: i24 = switch (instruction.SHFT) {
            0 => saturate(i24, ACC),
            1 => saturate(i24, @as(i32, @intCast(ACC)) << 1),
            2 => @truncate(ACC << 1),
            3 => @truncate(ACC),
        };

        //  - Multiply and accumulate
        //   A 26-bit value (ACC) becomes ((X * Y) >> 12) + B.
        //   The multiplication is signed.  I don't think the addition includes saturation.
        ACC = @intCast((@as(i64, @intCast(X)) * @as(i64, @intCast(Y))) >> 12);
        ACC += B;

        // - Temp write
        //  If TWT is set, the value SHIFTED is written to the temp buffer address indicated by ((TWA + MDEC_CT) & 0x7F).
        if (instruction.TWT)
            self.write_temp((instruction.TWA +% self.MDEC_CT().*) & 0x7F, SHIFTED);

        //  - Fractional address latch
        //   If FRCL is set, FRC_REG (13-bit) becomes one of the following:
        //     If SHFT=3: FRC_REG = lowest 12 bits of SHIFTED, ZERO-EXTENDED to 13 bits.
        //     Other values of SHFT: FRC_REG = bits 23-11 of SHIFTED.
        if (instruction.FRCL) {
            const s = @as(u24, @bitCast(SHIFTED));
            FRC_REG = switch (instruction.SHFT) {
                3 => @as(u12, @truncate(s)),
                else => @truncate(s >> 11),
            };
        }

        // - Memory operations
        //   If either MRD or MWT are set, we are performing a memory operation (on the
        //   external ringbuffer) and we'll need to compute an address.
        if (instruction.MRD or instruction.MWT) {
            //   Start with the 16-bit value of the MADRS register indicated by MASA (0x00-0x3F).
            var addr: u32 = self._madrs(instruction.MASA).*;
            //   If TABLE=0, add the 16-bit value MDEC_CT.
            if (instruction.TABLE == 0)
                addr +%= self.MDEC_CT().*;
            //   If ADREB=1, add the 12-bit value ADRS_REG, zero-extended.
            if (instruction.ADREB)
                addr +%= ADRS_REG;
            //   If NXADR=1, add 1.
            if (instruction.NXADR)
                addr +%= 1;
            //   If TABLE=0, mod the address so it's within the proper ringbuffer size.
            //     For a 8Kword  ringbuffer: AND by 0x1FFF
            //     For a 16Kword ringbuffer: AND by 0x3FFF
            //     For a 32Kword ringbuffer: AND by 0x7FFF
            //     For a 64Kword ringbuffer: AND by 0xFFFF
            //   If TABLE=1, simply AND the address by 0xFFFF.
            if (instruction.TABLE == 0)
                switch (self._ring_buffer.size) {
                    .@"8k" => addr &= 0x1FFF,
                    .@"16k" => addr &= 0x3FFF,
                    .@"32k" => addr &= 0x7FFF,
                    .@"64k" => addr &= 0xFFFF,
                }
            else
                addr &= 0xFFFF;
            //   Shift the address left by 1 (so it's referring to a word offset).
            addr <<= 1;
            //   Add (RBP<<11).
            addr +%= @as(u32, self._ring_buffer.pointer) << 11;
            //   This is the address, in main memory, you'll be reading or writing.

            //   If MRD is set, read the 16-bit word at the calculated address and put it
            //   in a temporary place.  It will then be accessible by the instruction either
            //   2 or 3 steps ahead.  That instruction will have to use a IWT to get the
            //   result.  Don't perform any conversions yet; let the IWT handle it later on.
            if (instruction.MRD)
                temp_word[step % temp_word.len] = self.read(u16, addr);

            //   If MWT is set, write the value of SHIFTED into memory at the calculated
            //   address.  If NOFL=1, simply shift it right by 8 to make it 16-bit.
            //   Otherwise, convert from 24-bit integer to 16-bit float (see float notes).
            if (instruction.MWT) {
                const value: u16 = if (instruction.NOFL) @truncate(@as(u24, @bitCast(SHIFTED)) >> 8) else f16_from_i24(SHIFTED);
                self.write(u16, addr, value);
            }
        }

        // - Address latch
        //   If ADRL is set, ADRS_REG (12-bit) becomes one of the following:
        //     If SHFT=3: ADRS_REG = INPUTS >> 16 (signed shift with sign extension).
        //     Other values of SHFT: ADRS_REG = bits 23-12 of SHIFTED.
        if (instruction.ADRL)
            ADRS_REG = @truncate(@as(u24, @bitCast(if (instruction.SHFT == 3)
                INPUTS >> 16
            else
                SHIFTED >> 12)));

        // - Effect output write
        //   If EWT is set, write (SHIFTED >> 8) into the EFREG register specified by EWA (0x0-0xF).
        if (instruction.EWT) self._efreg(instruction.EWA).* = @intCast(SHIFTED >> 8);
    }

    self.MDEC_CT().* -= 1;

    self.clear_mixs();
}

// To convert from 16-bit float to 24-bit integer (on read):
//  - Take the mantissa and shift it left by 11
//  - Make bit 23 be the sign bit
//  - Make bit 22 be the reverse of the sign bit
//  - Shift right (signed) by the exponent
fn i24_from_f16(value: u16) i24 {
    const sign_bit: u1 = @truncate(value >> 15);
    const exponent: u4 = @truncate(value >> 11);
    var val: u32 = (@as(u32, value) & 0x7FF) << 11;
    if (exponent <= 11)
        val |= @as(u32, 1 ^ sign_bit) << 22;
    val |= @as(u32, sign_bit) << 23;
    return @as(i24, @bitCast(@as(u24, @truncate(val)))) >> @min(11, exponent);
}

fn i32_from_f16(value: u16) callconv(Architecture.CallingConvention) i32 {
    return i24_from_f16(value);
}

// To convert from 24-bit integer to 16-bit float (on write):
// - While the top 2 bits are the same, shift left
//   The number of times you have to shift becomes the exponent
// - Shift right (signed) by 11
// - Set bits 14-11 to the exponent value
fn f16_from_i24(value: i24) u16 {
    const u: u24 = @bitCast(value);
    const sign_bit: u1 = @truncate(u >> 23);
    const exponent: u8 = @min(12, @clz((u ^ (u << 1))));
    var val: i24 = if (exponent < 12)
        (value << @intCast(exponent)) & 0x3FFFFF
    else
        value << 11;
    val >>= 11;
    var r: u16 = @truncate(@as(u24, @bitCast(val)));
    r |= @as(u16, exponent) << 11;
    r |= @as(u16, sign_bit) << 15;
    return r;
}

fn f16_from_i32(value: i32) callconv(Architecture.CallingConvention) u16 {
    return f16_from_i24(@intCast(value));
}

test {
    try std.testing.expectEqual(@as(i24, 0x400000), i24_from_f16(0));
    try std.testing.expectEqual(f16_from_i24(i24_from_f16(0)), 0);

    try std.testing.expectEqual(@as(i24, 0x400800), i24_from_f16(1));
    try std.testing.expectEqual(f16_from_i24(i24_from_f16(1)), 1);

    try std.testing.expectEqual(0x146800, i24_from_f16(0x1234));
    try std.testing.expectEqual(f16_from_i24(i24_from_f16(0x1234)), 0x1234);

    try std.testing.expectEqual(@as(i24, @bitCast(@as(u24, 0x91A000))), i24_from_f16(0x8234));
    try std.testing.expectEqual(f16_from_i24(i24_from_f16(0x8234)), 0x8234);

    try std.testing.expectEqual(@as(i24, @bitCast(@as(u24, 0xE46800))), i24_from_f16(0x9234));
    try std.testing.expectEqual(f16_from_i24(i24_from_f16(0x9234)), 0x9234);

    try std.testing.expectEqual(@as(i24, @bitCast(@as(u24, 0x7FF))), i24_from_f16(0x7FFF));
    try std.testing.expectEqual(f16_from_i24(i24_from_f16(0x7FFF)), 0x67FF);

    try std.testing.expectEqual(@as(i24, @bitCast(@as(u24, 0xFFF7FF))), i24_from_f16(0xFFFF));
    try std.testing.expectEqual(f16_from_i24(i24_from_f16(0xFFFF)), 0xDFFF);
}

pub fn serialize(_: @This(), _: *std.Io.Writer) !usize {
    // All our internal state is within _regs
    return 0;
}

pub fn deserialize(self: *@This(), _: *std.Io.Reader) !void {
    self._dirty_mpro = true;
}
