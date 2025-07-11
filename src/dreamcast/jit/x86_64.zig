const std = @import("std");
const builtin = @import("builtin");
const termcolor = @import("termcolor");

const x86_64_emitter_log = std.log.scoped(.x86_64_emitter);

pub const ReturnRegister = Register.rax;
pub const ScratchRegisters = [_]Register{ .r10, .r11 };

pub const ABI = enum {
    SystemV,
    Win64,
};

// Tried using builtin.abi, but it returns .gnu on Windows.
pub const JITABI: ABI = switch (builtin.os.tag) {
    .windows => .Win64,
    .linux => .SystemV,
    else => @compileError("Unsupported OS"),
};

// ArgRegisters are also used as scratch registers, but have a special meaning for function calls.
pub const ArgRegisters = switch (JITABI) {
    .Win64 => [_]Register{ .rcx, .rdx, .r8, .r9 },
    .SystemV => [_]Register{ .rdi, .rsi, .rdx, .rcx, .r8, .r9 },
};
pub const SavedRegisters = switch (JITABI) {
    .Win64 => [_]Register{
        .rbx,
        .rsi,
        .r12,
        .r13,
        .r14,
        .r15,
        .rdi,
        // NOTE: Both are saved registers, but I don't think I should expose them.
        // .rbp,
        // .rsp,
    },
    .SystemV => [_]Register{
        .rbx,
        .r12,
        .r13,
        .r14,
        .r15,
        // .rbp,
        // .rsp,
    },
};

pub const FPArgRegisters = switch (JITABI) {
    .Win64 => [_]FPRegister{
        .xmm0,
        .xmm1,
        .xmm2,
        .xmm3,
    },
    .SystemV => [_]FPRegister{
        .xmm0,
        .xmm1,
        .xmm2,
        .xmm3,
        .xmm4,
        .xmm5,
        .xmm6,
        .xmm7,
    },
};

pub const FPScratchRegisters = switch (JITABI) {
    .Win64 => [_]FPRegister{
        .xmm4,
        .xmm5,
    },
    .SystemV => [_]FPRegister{
        // NOTE: These are scratch registers for the ABI, but we're using them as saved registers!
        // .xmm8,
        // .xmm9,
        // .xmm10,
        // .xmm11,
        // .xmm12,
        // .xmm13,
        .xmm14,
        .xmm15,
    },
};

pub const FPSavedRegisters = switch (JITABI) {
    .Win64 => [_]FPRegister{
        .xmm6,
        .xmm7,
        .xmm8,
        .xmm9,
        .xmm10,
        .xmm11,
        .xmm12,
        .xmm13,
        .xmm14,
        .xmm15,
    },
    .SystemV => [_]FPRegister{}, // NOTE: SH4 JIT uses xmm8-xmm13 (manually saving them)
};

// RegOpcodes (ModRM) for 0x81: OP r/m32, imm32 - 0x83: OP r/m32, imm8 (sign extended)
const RegOpcode = enum(u3) { Add = 0, Adc = 2, Sub = 5, Sbb = 3, And = 4, Or = 1, Xor = 6, Cmp = 7 };
// I mean, I don't know what to call these... (0xF7 opcode)
const OtherRegOpcode = enum(u3) { Test = 0, Not = 2, Neg = 3, Mul = 4, IMul = 5, Div = 6, IDiv = 7 };
// Opcode: C1 / D3
const ShiftRegOpcode = enum(u3) { Rol = 0, Ror = 1, Rcl = 2, Rcr = 3, Shl = 4, Shr = 5, Sar = 7 };

// Above/Below: Unsigned
// Greater/Less: Signed
pub const Condition = enum {
    Always,
    Overflow,
    NotOverflow,
    Carry,
    Below,
    NotCarry,
    AboveEqual,
    NotBelow,
    Equal,
    Zero,
    NotEqual,
    NotZero,
    NotAbove,
    BelowEqual,
    Above,
    NotBelowEqual,
    Sign,
    NotSign,
    ParityEven,
    ParityOdd,
    Less,
    NotGreaterEqual,
    GreaterEqual,
    NotLess,
    LessEqual,
    NotGreater,
    Greater,
    NotLessEqual,

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return writer.print("{s}", .{@tagName(value)});
    }

    /// Returns the low nibble of opcode using this condition
    pub fn nibble(self: @This()) u8 {
        return switch (self) {
            .Overflow => 0x0,
            .NotOverflow => 0x1,
            .Carry, .Below => 0x2,
            .NotCarry, .AboveEqual, .NotBelow => 0x3,
            .Equal, .Zero => 0x4,
            .NotEqual, .NotZero => 0x5,
            .NotAbove, .BelowEqual => 0x6,
            .Above, .NotBelowEqual => 0x7,
            .Sign => 0x8,
            .NotSign => 0x9,
            .ParityEven => 0xA,
            .ParityOdd => 0xB,
            .Less, .NotGreaterEqual => 0xC,
            .GreaterEqual, .NotLess => 0xD,
            .LessEqual, .NotGreater => 0xE,
            .Greater, .NotLessEqual => 0xF,
            else => std.debug.panic("Invalid condition: {any}", .{self}),
        };
    }
};

pub const EFLAGSCondition = enum(u4) {
    Overflow = 0b0000,
    NoOverflow = 0b0001,
    Carry = 0b0010,
    NotCarry = 0b0011,
    Equal = 0b0100,
    NotEqual = 0b0101,
    BelowOrEqual = 0b0110,
    Above = 0b0111,
    Sign = 0b1000,
    NotSign = 0b1001,
    ParityEven = 0b1010,
    ParityOdd = 0b1011,
    Less = 0b1100,
    GreaterOrEqual = 0b1101,
    LessOrEqual = 0b1110,
    Greater = 0b1111,
};

pub const OperandSize = enum(u8) { _8 = 8, _16 = 16, _32 = 32, _64 = 64 };

pub const Scale = enum(u2) {
    _1 = 0b00,
    _2 = 0b01,
    _4 = 0b10,
    _8 = 0b11,

    pub fn format(value: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        switch (value) {
            ._1 => try writer.writeAll("1"),
            ._2 => try writer.writeAll("2"),
            ._4 => try writer.writeAll("4"),
            ._8 => try writer.writeAll("8"),
        }
    }
};

pub const MemOperand = struct {
    base: Register, // NOTE: This could be made optional as well, to allow for absolute addressing. However this is only possible on (r)ax on x86_64.
    index: ?Register = null,
    scale: Scale = ._1, // Only valid if index is supplied.
    displacement: u32 = 0,
    size: u8,

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        if (value.index) |index| {
            if (value.displacement > 0) {
                return writer.print("[{any}+{any}*{any}+0x{X}]", .{
                    value.base,
                    value.scale,
                    index,
                    value.displacement,
                });
            } else {
                return writer.print("[{any}+{any}*{any}]", .{
                    value.base,
                    value.scale,
                    index,
                });
            }
        } else if (value.displacement > 0) {
            return writer.print("[{any}+0x{X}]", .{
                value.base,
                value.displacement,
            });
        } else {
            return writer.print("[{any}]", .{value.base});
        }
    }
};

pub const Operand = union(enum) {
    reg8: Register,
    reg16: Register,
    reg: Register, // FIXME: This will sometimes be treated as a 32-bit register, and sometimes as a 64-bit register, depending on the instruction, or the size of the other operand. Make it explicit.
    reg64: Register,
    freg32: FPRegister,
    freg64: FPRegister,
    freg128: FPRegister,
    imm8: u8,
    imm16: u16,
    imm32: u32,
    imm64: u64,
    mem: MemOperand,

    pub fn tag(self: @This()) std.meta.Tag(@This()) {
        return std.meta.activeTag(self);
    }

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return switch (value) {
            .reg8 => |reg| writer.print("{any}<8>", .{reg}),
            .reg16 => |reg| writer.print("{any}<16>", .{reg}),
            .reg => |reg| writer.print("{any}", .{reg}),
            .reg64 => |reg| writer.print("{any}<64>", .{reg}),
            .freg32 => |reg| writer.print("{any}<32>", .{reg}),
            .freg64 => |reg| writer.print("{any}<64>", .{reg}),
            .freg128 => |reg| writer.print("{any}<128>", .{reg}),
            .imm8 => |imm| writer.print("0x{X:0>2}", .{imm}),
            .imm16 => |imm| writer.print("0x{X:0>4}", .{imm}),
            .imm32 => |imm| writer.print("0x{X:0>8}", .{imm}),
            .imm64 => |imm| writer.print("0x{X:0>16}", .{imm}),
            .mem => |mem| writer.print("{any}", .{mem}),
        };
    }

    pub fn size(self: @This()) u8 {
        return switch (self) {
            .imm8, .reg8 => 8,
            .imm16, .reg16 => 16,
            .imm32, .reg, .freg32 => 32,
            .imm64, .reg64, .freg64, .freg128 => 64,
            .mem => |mem| mem.size,
        };
    }

    pub fn Reg(reg: Register, comptime bitsize: u8) Operand {
        return switch (bitsize) {
            8 => return .{ .reg8 = reg },
            16 => return .{ .reg16 = reg },
            32 => return .{ .reg = reg },
            64 => return .{ .reg64 = reg },
            else => @compileError("Invalid register size"),
        };
    }
};

pub const Instruction = union(enum) {
    Nop, // Usefull to patch out instructions without having to rewrite the entire block.
    Break, // For Debugging
    FunctionCall: ?*const anyopaque, // FIXME: Is there a better type for generic function pointers? NOTE: If null, expects the function pointer to be in rax
    Mov: struct { dst: Operand, src: Operand, preserve_flags: bool = false }, // Mov with zero extention (NOTE: This is NOT the same as the x86 mov instruction, which doesn't zero extend from 8 and 16-bit memory accesses)
    Cmov: struct { condition: Condition, dst: Operand, src: Operand }, // Conditional Move
    Movsx: struct { dst: Operand, src: Operand }, // Mov with sign extension
    Push: Operand,
    Pop: Operand,
    Not: struct { dst: Operand },
    Neg: struct { dst: Operand },
    Add: struct { dst: Operand, src: Operand },
    Adc: struct { dst: Operand, src: Operand },
    Sub: struct { dst: Operand, src: Operand },
    Sbb: struct { dst: Operand, src: Operand },
    Mul: struct { dst: Operand, src: Operand },
    Div: struct { dst: Operand, src: Operand },
    Fma: struct { dst: FPRegister, src1: FPRegister, src2: Operand }, // Fused Multiply Add
    Sqrt: struct { dst: Operand, src: Operand },
    Min: struct { dst: Operand, src: Operand },
    Max: struct { dst: Operand, src: Operand },
    And: struct { dst: Operand, src: Operand },
    Or: struct { dst: Operand, src: Operand },
    Xor: struct { dst: Operand, src: Operand },
    Cmp: struct { lhs: Operand, rhs: Operand },
    SetByteCondition: struct { condition: Condition, dst: Operand },
    BitTest: struct { reg: Register, offset: Operand },
    Rol: struct { dst: Operand, amount: Operand },
    Ror: struct { dst: Operand, amount: Operand },
    Rcl: struct { dst: Operand, amount: Operand },
    Rcr: struct { dst: Operand, amount: Operand },
    Shl: struct { dst: Operand, amount: Operand },
    Shr: struct { dst: Operand, amount: Operand },
    Sar: struct { dst: Operand, amount: Operand },
    Jmp: struct { condition: Condition, dst: union(enum) { rel: i32, abs_indirect: Operand } },
    BlockEpilogue: void,
    Convert: struct { dst: Operand, src: Operand },
    // FIXME: This only exists because I haven't added a way to specify the size the GPRs.
    Div64_32: struct { dividend_high: Register, dividend_low: Register, divisor: Register, result: Register },

    SaveFPRegisters: struct { count: u8 },
    RestoreFPRegisters: struct { count: u8 },

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return switch (value) {
            .Nop => writer.print("nop", .{}),
            .Break => writer.print("break", .{}),
            .FunctionCall => |function| writer.print("call {any}", .{function}),
            .Mov => |mov| writer.print("mov {any}, {any}", .{ mov.dst, mov.src }),
            .Cmov => |cmov| writer.print("cmov {} {}, {}", .{ cmov.condition, cmov.dst, cmov.src }),
            .Movsx => |movsx| writer.print("movsx {any}, {any}", .{ movsx.dst, movsx.src }),
            .Push => |reg| writer.print("push {any}", .{reg}),
            .Pop => |reg| writer.print("pop {any}", .{reg}),
            .Not => |not| writer.print("not {any}", .{not.dst}),
            .Neg => |neg| writer.print("neg {any}", .{neg.dst}),
            .Add => |add| writer.print("add {any}, {any}", .{ add.dst, add.src }),
            .Adc => |add| writer.print("adc {any}, {any}", .{ add.dst, add.src }),
            .Sub => |sub| writer.print("sub {any}, {any}", .{ sub.dst, sub.src }),
            .Sbb => |sub| writer.print("sub {any}, {any}", .{ sub.dst, sub.src }),
            .Mul => |mul| writer.print("mul {any}, {any}", .{ mul.dst, mul.src }),
            .Div => |div| writer.print("div {any}, {any}", .{ div.dst, div.src }),
            .Fma => |fma| writer.print("fma {any} += {any} * {any}", .{ fma.dst, fma.src1, fma.src2 }),
            .Sqrt => |sqrt| writer.print("sqrt {any}, {any}", .{ sqrt.dst, sqrt.src }),
            .Min => |min| writer.print("min {any}, {any}", .{ min.dst, min.src }),
            .Max => |max| writer.print("max {any}, {any}", .{ max.dst, max.src }),
            .And => |and_| writer.print("and {any}, {any}", .{ and_.dst, and_.src }),
            .Or => |or_| writer.print("or {any}, {any}", .{ or_.dst, or_.src }),
            .Xor => |xor_| writer.print("xor {any}, {any}", .{ xor_.dst, xor_.src }),
            .Cmp => |cmp| writer.print("cmp {any}, {any}", .{ cmp.lhs, cmp.rhs }),
            .SetByteCondition => |set| writer.print("set{any} {any}", .{ set.condition, set.dst }),
            .BitTest => |bit_test| writer.print("bt {any}, {any}", .{ bit_test.reg, bit_test.offset }),
            .Jmp => |jmp| switch (jmp.dst) {
                .rel => writer.print("jmp {any} 0x{x}", .{ jmp.condition, jmp.dst.rel }),
                .abs_indirect => writer.print("jmp {any} {any}", .{ jmp.condition, jmp.dst.abs_indirect }),
            },
            .BlockEpilogue => writer.print("block_epilogue", .{}),
            .Rol => |rol| writer.print("rol {any}, {any}", .{ rol.dst, rol.amount }),
            .Ror => |ror| writer.print("ror {any}, {any}", .{ ror.dst, ror.amount }),
            .Rcl => |rcl| writer.print("rcl {any}, {any}", .{ rcl.dst, rcl.amount }),
            .Rcr => |rcr| writer.print("rcr {any}, {any}", .{ rcr.dst, rcr.amount }),
            .Shl => |shl| writer.print("shl {any}, {any}", .{ shl.dst, shl.amount }),
            .Shr => |shr| writer.print("shr {any}, {any}", .{ shr.dst, shr.amount }),
            .Sar => |sar| writer.print("sar {any}, {any}", .{ sar.dst, sar.amount }),
            .Convert => |cvt| writer.print("convert {any}, {any}", .{ cvt.dst, cvt.src }),
            .Div64_32 => |div| writer.print("div64_32 {any},{any}:{any},{any},", .{ div.result, div.dividend_high, div.dividend_low, div.divisor }),
            .SaveFPRegisters => |instr| writer.print("SaveFPRegisters {any}", .{instr.count}),
            .RestoreFPRegisters => |instr| writer.print("RestoreFPRegisters {any}", .{instr.count}),
        };
    }
};

pub const Register = enum(u4) {
    rax = 0,
    rcx = 1,
    rdx = 2,
    rbx = 3,
    rsp = 4,
    rbp = 5,
    rsi = 6,
    rdi = 7,
    r8 = 8,
    r9 = 9,
    r10 = 10,
    r11 = 11,
    r12 = 12,
    r13 = 13,
    r14 = 14,
    r15 = 15,

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return writer.print("{s}", .{@tagName(value)});
    }

    /// Returns true for registers requiring a REX prefix for 8bit operations, when they normally would not.
    ///   Without REX prefix, only AL, CL, DL, BL, AH,  CH,  DH,  BH   are accessible (NOTE: xH registers are supported by this emitter anyway).
    ///   With REX prefix:         AL, CL, DL, BL, SPL, BPL, SIL, DIL, R8B, R9B, R10B, R11B, R12B, R13B, R14B, R15B
    pub fn require_rex_8bit(self: @This()) bool {
        return switch (self) {
            .rsp, .rbp, .rsi, .rdi => true,
            else => false,
        };
    }
};

pub const FPRegister = enum(u4) {
    xmm0 = 0,
    xmm1 = 1,
    xmm2 = 2,
    xmm3 = 3,
    xmm4 = 4,
    xmm5 = 5,
    xmm6 = 6,
    xmm7 = 7,
    xmm8 = 8,
    xmm9 = 9,
    xmm10 = 10,
    xmm11 = 11,
    xmm12 = 12,
    xmm13 = 13,
    xmm14 = 14,
    xmm15 = 15,

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return writer.print("{s}", .{@tagName(value)});
    }
};

pub const REX = packed struct(u8) {
    b: bool = false, // Extension of the ModR/M r/m field, SIB base field, or Opcode reg field
    x: bool = false, // Extension of the SIB index field
    r: bool = false, // Extension of the ModR/M reg field
    w: bool = false, // 0 = Operand size determined by CS.D; 1 = 64 Bit Operand Siz
    _: u4 = 0b0100,
};

const VEX3 = packed struct(u24) {
    prefix: u8 = 0xC4,
    // Byte 1
    m: enum(u5) {
        x0F = 0x1,
        x0F38 = 0x2,
        x0F3A = 0x3,
        _,
    }, // Opcode map.
    not_b: bool,
    not_x: bool,
    not_r: bool, // Inversion of REX r, x and b bits.
    // Byte 2
    p: enum(u2) {
        no = 0x0,
        x66 = 0x1,
        xF3 = 0x2,
        xF2 = 0x3,
    }, // Additional prefix bytes. The values 0, 1, 2, and 3 correspond to implied no, 0x66, 0xF3, and 0xF2 prefixes.
    l: u1, // Vector length, 0: 128bit SSE (XMM), 1: 256bit AVX (YMM)
    not_v: u4, // Inversion of additional source register index
    w: bool, // 64-bit operand?
};

const Mod = enum(u2) {
    indirect = 0b00, // Register indirect, or SIB without displacement (r/m = 0b100), or Displacement only (r/m = 0b101)
    disp8 = 0b01, // 8-bit displacement
    disp32 = 0b10, // 32-bit displacement
    reg = 0b11,
};

pub const MODRM = packed struct(u8) {
    r_m: u3, // The r/m field can specify a register as an operand or it can be combined with the mod field to encode an addressing mode. Sometimes, certain combinations of the mod field and the r/m field are used to express opcode information for some instructions.
    reg_opcode: u3, // The reg/opcode field specifies either a register number or three more bits of opcode information. The purpose of the reg/opcode field is specified in the primary opcode.
    mod: Mod, // The mod field combines with the r/m field to form 32 possible values: eight registers and 24 addressing modes
};

pub const SIB = packed struct(u8) {
    base: u3,
    index: u3,
    scale: Scale,
};

const ScalarFPOpcodes = enum(u8) {
    Mov = 0x10,
    Sqrt = 0x51,
    Rsqrt = 0x52, // Reciprocal of Square Root
    Rcp = 0x53, // Reciprocal
    Add = 0x58,
    Mul = 0x59,
    Convert_SS_SD = 0x5A,
    Convert_PS_DQ = 0x5B,
    Sub = 0x5C,
    Min = 0x5D,
    Div = 0x5E,
    Max = 0x5F,
};

// Recommended Multi-Byte Sequence of NOP Instruction
const NOPs = [_][]const u8{
    "\x90",
    "\x66\x90",
    "\x0f\x1f\x00",
    "\x0f\x1f\x40\x00",
    "\x0f\x1f\x44\x00\x00",
    "\x66\x0f\x1f\x44\x00\x00",
    "\x0f\x1f\x80\x00\x00\x00\x00",
    "\x0f\x1f\x84\x00\x00\x00\x00\x00",
    "\x66\x0f\x1f\x84\x00\x00\x00\x00\x00",
};

pub fn convert_to_nops(instructions: []u8) void {
    var idx: usize = 0;
    while (instructions.len - idx >= NOPs.len) {
        @memcpy(instructions[idx..NOPs.len], NOPs[NOPs.len - 1]);
        idx += NOPs.len;
    }
    if (idx < instructions.len)
        @memcpy(instructions[idx..], NOPs[instructions.len - idx - 1]);
}

const PatchableJump = struct {
    size: enum { r8, r32 },
    source: u32 = Invalid,
    address_to_patch: u32 = Invalid,

    const Invalid: u32 = 0xFFFFFFFF;

    pub fn invalid(self: @This()) bool {
        return self.source == Invalid or self.address_to_patch == Invalid;
    }
};

const PatchableJumpList = struct {
    items: [128]PatchableJump = @splat(.{ .size = .r32 }),

    pub fn add(self: *@This(), patchable_jump: PatchableJump) !void {
        std.debug.assert(!patchable_jump.invalid());

        for (&self.items) |*item| {
            if (item.invalid()) {
                item.* = patchable_jump;
                return;
            }
        }
        return error.PatchableJumpFull;
    }
};

pub const Emitter = struct {
    block_buffer: []u8 = undefined,
    block_size: u32 = 0,

    forward_jumps_to_patch: std.AutoHashMap(u32, PatchableJumpList),

    _instruction_offsets: []u32,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        return .{
            .forward_jumps_to_patch = .init(allocator),
            ._instruction_offsets = try allocator.alloc(u32, 64),
            ._allocator = allocator,
        };
    }

    // Call this before emitting
    pub fn set_buffer(self: *@This(), block_buffer: []u8) void {
        self.block_buffer = block_buffer;
        self.block_size = 0;
        std.debug.assert(self.forward_jumps_to_patch.count() == 0);
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.free(self._instruction_offsets);
        self.forward_jumps_to_patch.deinit();
    }

    pub fn emit_instructions(self: *@This(), instructions: []const Instruction) !void {
        if (self._instruction_offsets.len < instructions.len)
            self._instruction_offsets = try self._allocator.realloc(self._instruction_offsets, instructions.len);

        for (instructions, 0..) |instr, idx| {
            self._instruction_offsets[idx] = self.block_size;

            if (self.forward_jumps_to_patch.get(@intCast(idx))) |jumps| {
                for (jumps.items) |jump| {
                    if (!jump.invalid()) {
                        switch (jump.size) {
                            .r8 => {
                                std.debug.assert(self.block_size - jump.source < 128);
                                const rel: u8 = @intCast(self.block_size - jump.source);
                                self.block_buffer[jump.address_to_patch] = rel;
                            },
                            .r32 => {
                                const rel: u32 = @intCast(self.block_size - jump.source);
                                @memcpy(self.block_buffer[jump.address_to_patch..][0..4], std.mem.asBytes(&rel));
                            },
                        }
                    }
                }
                _ = self.forward_jumps_to_patch.remove(@intCast(idx));
            }

            switch (instr) {
                .Nop => {},
                .Break => {
                    if (builtin.mode != .Debug) {
                        x86_64_emitter_log.warn("[x86_64 Emitter] Warning: Emitting a break instruction outside of Debug Build.", .{});
                    }
                    try self.emit_byte(0xCC);
                },
                .FunctionCall => |function| try self.native_call(function),
                .Mov => |m| try self.mov(m.dst, m.src, m.preserve_flags),
                .Cmov => |m| try self.cmov(m.condition, m.dst, m.src),
                .Movsx => |m| try self.movsx(m.dst, m.src),
                .Push => |reg_or_imm| try self.push(reg_or_imm),
                .Pop => |reg| try self.pop(reg),
                .Add => |a| try self.add(a.dst, a.src),
                .Adc => |a| try self.adc(a.dst, a.src),
                .Sub => |a| try self.sub(a.dst, a.src),
                .Sbb => |a| try self.sbb(a.dst, a.src),
                .Mul => |a| try self.mul(a.dst, a.src),
                .Div => |a| try self.div(a.dst, a.src),
                .Fma => |a| try self.fma(a.dst, a.src1, a.src2),
                .Sqrt => |a| try self.sqrt(a.dst, a.src),
                .Min => |a| try self.min(a.dst, a.src),
                .Max => |a| try self.max(a.dst, a.src),
                .And => |a| try self.and_(a.dst, a.src),
                .Or => |a| try self.or_(a.dst, a.src),
                .Xor => |a| try self.xor_(a.dst, a.src),
                .Cmp => |a| try self.cmp(a.lhs, a.rhs),
                .SetByteCondition => |a| try self.set_byte_condition(a.condition, a.dst),
                .Jmp => |j| try self.jmp(j.condition, @intCast(idx), j.dst),
                .BlockEpilogue => try self.emit_block_epilogue(),
                .BitTest => |b| try self.bit_test(b.reg, b.offset),
                .Rol => |r| try self.shift_instruction(.Rol, r.dst, r.amount),
                .Ror => |r| try self.shift_instruction(.Ror, r.dst, r.amount),
                .Rcl => |r| try self.shift_instruction(.Rcl, r.dst, r.amount),
                .Rcr => |r| try self.shift_instruction(.Rcr, r.dst, r.amount),
                .Sar => |r| try self.shift_instruction(.Sar, r.dst, r.amount),
                .Shr => |r| try self.shift_instruction(.Shr, r.dst, r.amount),
                .Shl => |r| try self.shift_instruction(.Shl, r.dst, r.amount),
                .Not => |r| try self.f7_op(r.dst, .Not),
                .Neg => |r| try self.f7_op(r.dst, .Neg),
                .Convert => |r| try self.convert(r.dst, r.src),
                .Div64_32 => |d| try self.div64_32(d.dividend_high, d.dividend_low, d.divisor, d.result),

                .SaveFPRegisters => |s| try self.save_fp_registers(s.count),
                .RestoreFPRegisters => |s| try self.restore_fp_registers(s.count),
                // else => return error.UnsupportedInstruction,
            }
        }
        if (self.forward_jumps_to_patch.count() > 0) {
            std.debug.print("Jumps left to patch: {}\n", .{self.forward_jumps_to_patch.count()});
            @panic("Error: Unpatched jumps!");
        }
    }

    pub fn push(self: *@This(), reg_or_imm: Operand) !void {
        switch (reg_or_imm) {
            .reg8, .reg16, .reg, .reg64 => |reg| {
                try self.emit_rex_if_needed(.{ .b = need_rex(reg) });
                try self.emit(u8, encode_opcode(0x50, reg));
            },
            else => return error.UnimplementedPushImmediate,
        }
    }

    pub fn pop(self: *@This(), reg: Operand) !void {
        switch (reg) {
            .reg8, .reg16, .reg, .reg64 => |r| {
                try self.emit_rex_if_needed(.{ .b = need_rex(r) });
                try self.emit(u8, encode_opcode(0x58, r));
            },
            else => return error.InvalidPopOperand,
        }
    }

    pub fn emit_byte(self: *@This(), value: u8) !void {
        self.block_buffer[self.block_size] = value;
        self.block_size += 1;
    }

    pub fn emit_slice(self: *@This(), comptime T: type, value: []const T) !void {
        for (value) |v|
            try self.emit(T, v);
    }

    pub fn emit(self: *@This(), comptime T: type, value: T) !void {
        if (T == MODRM) {
            // See Intel Manual Vol. 2A 2-11.
            if (value.mod == .indirect and value.r_m == 0b101)
                return error.UnhandledSpecialCase;
        }
        if (T == VEX3) { // Could be extended to other packed structs.
            const slice = @as([@bitSizeOf(VEX3) / 8]u8, @bitCast(value));
            for (slice) |b| {
                try self.emit_byte(b);
            }
            return;
        }

        if (@sizeOf(T) == 1) {
            if (@typeInfo(T) == .@"enum") {
                try self.emit_byte(@intFromEnum(value));
            } else {
                try self.emit_byte(@bitCast(value));
            }
        } else {
            for (0..@sizeOf(T)) |i| {
                try self.emit_byte(@truncate((value >> @intCast(8 * i)) & 0xFF));
            }
        }
    }

    pub fn emit_block_prologue(self: *@This()) !void {
        try self.emit_slice(u8, &[_]u8{
            // push rbp
            0x55,
            // mov rbp,rsp
            0x48,
            0x89,
            0xE5,
        });
    }

    pub fn emit_block_epilogue(self: *@This()) !void {
        // pop rbp
        try self.emit(u8, 0x5D);
        try self.ret();
    }

    fn encode(reg: anytype) u3 {
        return switch (@TypeOf(reg)) {
            Register => @truncate(@intFromEnum(reg)),
            FPRegister => @truncate(@intFromEnum(reg)),
            Operand => switch (reg) {
                .reg8, .reg16, .reg, .reg64 => |r| encode(r),
                .freg32, .freg64 => |r| encode(r),
                else => @panic("Operand must be a register"),
            },
            else => @compileError("Unsupported register type"),
        };
    }

    fn need_rex(reg: anytype) bool {
        return switch (comptime @TypeOf(reg)) {
            Register => @intFromEnum(reg) >= 8,
            FPRegister => @intFromEnum(reg) >= 8,
            Operand => switch (reg) {
                .reg8, .reg16, .reg, .reg64 => |r| need_rex(r),
                .freg32, .freg64 => |r| need_rex(r),
                else => @panic("Operand must be a register"),
            },
            else => @compileError("Unsupported register type"),
        };
    }

    fn encode_opcode(opcode: u8, reg: anytype) u8 {
        return opcode + encode(reg);
    }

    fn emit_rex_if_needed(self: *@This(), rex: REX) !void {
        if (@as(u8, @bitCast(rex)) != @as(u8, @bitCast(REX{})))
            try self.emit(u8, @bitCast(rex));
    }

    pub fn mov_reg_reg(self: *@This(), dst: Register, src: Register) !void {
        // Always 64bits
        try self.emit_rex_if_needed(.{ .w = true, .r = need_rex(src), .b = need_rex(dst) });
        try self.emit(u8, 0x89);
        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(src), .r_m = encode(dst) });
    }

    // movd xmm, r/m32 / movq xmm, r/m64
    pub fn mov_freg_reg(self: *@This(), comptime size: OperandSize, dst: FPRegister, src: Register) !void {
        try self.emit(u8, 0x66);
        try self.emit_rex_if_needed(.{ .w = size == ._64, .r = need_rex(dst), .b = need_rex(src) });
        try self.emit(u8, 0x0F);
        try self.emit(u8, 0x6E);
        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst), .r_m = encode(src) });
    }

    // movd r/m32, xmm / movq r/m64, xmm
    pub fn mov_reg_freg(self: *@This(), comptime size: OperandSize, dst: Register, src: FPRegister) !void {
        try self.emit(u8, 0x66);
        try self.emit_rex_if_needed(.{ .w = size == ._64, .r = need_rex(src), .b = need_rex(dst) });
        try self.emit(u8, 0x0F);
        try self.emit(u8, 0x7E);
        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(src), .r_m = encode(dst) });
    }

    // <op>ss xmm1, xmm2/m32
    pub fn scalar_floating_point_operation(self: *@This(), comptime size: OperandSize, opcode: ScalarFPOpcodes, dst: FPRegister, src: Operand) !void {
        try self.emit(u8, switch (size) {
            ._32 => 0xF3,
            ._64 => 0xF2,
            else => @compileError("Unsupported operand size"),
        });
        switch (src) {
            .freg32, .freg64 => |src_reg| {
                try self.emit_rex_if_needed(.{ .w = false, .r = need_rex(dst), .b = need_rex(src_reg) });
                try self.emit(u8, 0x0F);
                try self.emit(ScalarFPOpcodes, opcode);
                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst), .r_m = encode(src_reg) });
            },
            .mem => |src_mem| {
                x86_64_emitter_log.warn(termcolor.yellow("Untested <{s}>ss xmm1, xmm2/m32 with a memory operand. Be careful :)"), .{@tagName(opcode)});

                try self.emit_rex_if_needed(.{ .w = false, .r = need_rex(dst), .b = need_rex(src_mem.base) });
                try self.emit(u8, 0x0F);
                try self.emit(ScalarFPOpcodes, opcode);
                try self.emit_mem_addressing(encode(dst), src_mem);
            },
            else => return error.UnsupportedSourceForScalarFPOperation,
        }
    }

    pub fn cmp_scalar_fp(self: *@This(), comptime size: OperandSize, lhs: FPRegister, rhs: FPRegister) !void {
        // COMISS xmm1, xmm2/m64 / COMISD xmm1, xmm2/m64
        switch (size) {
            ._32 => {},
            ._64 => try self.emit(u8, 0x66),
            else => @compileError("Unsupported operand size"),
        }
        try self.emit_rex_if_needed(.{ .w = false, .r = need_rex(lhs), .b = need_rex(rhs) });
        try self.emit(u8, 0x0F);
        try self.emit(u8, 0x2F);
        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(lhs), .r_m = encode(rhs) });
    }

    // Emits ModRM, SIB and displacement bytes, as needed.
    fn emit_mem_addressing(self: *@This(), reg_opcode: u3, mem: MemOperand) !void {
        const r_m: u3 = if (mem.index != null) 0b100 // A SIB is following
            else encode(mem.base); // If base is ESP/R12, a SIB will also be emitted.

        const mod: Mod = if (mem.base == .rbp and mem.displacement == 0) .disp8 // Special case: If the base is rbp, the displacement is mandatory. See Intel Manual Vol. 2A 2-11.
            else (if (mem.displacement == 0) .indirect else if (mem.displacement < 0x80) .disp8 else .disp32);

        try self.emit(MODRM, .{
            .mod = mod,
            .reg_opcode = reg_opcode,
            .r_m = r_m,
        });

        if (r_m == 0b100) {
            try self.emit(SIB, .{
                .scale = mem.scale,
                // NOTE: ESP/R12-based addressing needs a SIB byte, this is the 0b100 case.
                .index = if (mem.index) |i| encode(i) else 0b100,
                .base = encode(mem.base),
            });
        }

        switch (mod) {
            .disp8 => try self.emit(u8, @truncate(mem.displacement)),
            .disp32 => try self.emit(u32, mem.displacement),
            else => {},
        }
    }

    pub fn mov_reg_mem(self: *@This(), comptime direction: enum { MemToReg, RegToMem }, reg: Operand, mem: MemOperand) !void {
        var reg_64 = mem.size == 64;

        if (direction == .MemToReg and mem.size < 32 and reg == .reg)
            reg_64 = true; // Force 64-bit register to be 100% sure all bits are cleared.

        const opcode: []const u8 = switch (direction) {
            .MemToReg => switch (reg) {
                .reg8 => &[_]u8{0x8A},
                .reg => switch (mem.size) {
                    8 => &[_]u8{ 0x0F, 0xB6 }, // Emit a movzx (zero extend) in this case.
                    16 => &[_]u8{ 0x0F, 0xB7 }, // Emit a movzx (zero extend) in this case.
                    32, 64 => &[_]u8{0x8B},
                    else => return error.InvalidMemSize,
                },
                .freg32, .freg64 => &[_]u8{ 0x0F, 0x6E }, // movd/movq
                else => return error.InvalidRegisterType,
            },
            .RegToMem => switch (reg) {
                .reg8 => &[_]u8{0x88},
                .reg16 => &[_]u8{0x89}, // With 0x66 prefix
                .reg => switch (mem.size) {
                    8 => &[_]u8{0x88},
                    16 => &[_]u8{0x89}, // With 0x66 prefix
                    32, 64 => &[_]u8{0x89},
                    else => return error.InvalidMemSize,
                },
                .freg32, .freg64 => &[_]u8{ 0x0F, 0x7E }, // movd/movq
                else => return error.InvalidRegisterType,
            },
        };

        if (mem.size == 16 or reg == .freg32 or reg == .freg64)
            try self.emit(u8, 0x66);

        if (reg == .reg8 and reg.reg8.require_rex_8bit()) {
            // NOTE: Byte access to the lower 8 bits of these registers is only possible with a rex prefix,
            // emit it unconditionally.
            // FIXME: This should probably be done elsewhere too...
            try self.emit(REX, .{
                .w = reg_64,
                .r = need_rex(reg),
                .x = if (mem.index) |i| need_rex(i) else false,
                .b = need_rex(mem.base),
            });
        } else {
            try self.emit_rex_if_needed(.{
                .w = reg_64,
                .r = need_rex(reg),
                .x = if (mem.index) |i| need_rex(i) else false,
                .b = need_rex(mem.base),
            });
        }

        try self.emit_slice(u8, opcode);

        try self.emit_mem_addressing(encode(reg), mem);
    }

    // If preserve_flags is false, mov reg, 0 will be replaced by xor reg, reg.
    pub fn mov(self: *@This(), dst: Operand, src: Operand, preserve_flags: bool) !void {
        switch (dst) {
            .mem => |dst_m| {
                switch (src) {
                    .reg8, .reg16 => {
                        if (src.size() != dst.mem.size) return error.OperandSizeMismatch;
                        try mov_reg_mem(self, .RegToMem, src, dst_m);
                    },
                    .reg => try mov_reg_mem(self, .RegToMem, src, dst_m),
                    .imm32 => |imm| {
                        if (dst.mem.size != 32) return error.OperandSizeMismatch;
                        try self.emit_rex_if_needed(.{ .b = need_rex(dst_m.base) });
                        try self.emit(u8, 0xC7);
                        try self.emit_mem_addressing(0, dst_m);
                        try self.emit(u32, imm);
                    },
                    .freg32 => try mov_reg_mem(self, .RegToMem, src, dst_m),
                    .freg64 => try mov_reg_mem(self, .RegToMem, src, dst_m),
                    else => return error.InvalidMovSourceFromMem,
                }
            },
            .reg8 => |_| {
                switch (src) {
                    .mem => |src_m| try mov_reg_mem(self, .MemToReg, dst, src_m),
                    else => return error.InvalidMovSourceFromReg8,
                }
            },
            .reg => |dst_reg| {
                switch (src) {
                    // NOTE: Both reg8 to reg and reg16 to reg zero extend the source to 64-bits.
                    .reg8 => |src_reg| {
                        // NOTE: If for some reason this is changed so that .w is not necessarily set,
                        //       remember to always emit a REX prefix in the rsp/rbp/rsi/rdi case.
                        try self.emit_rex_if_needed(.{ .w = true, .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0xB6);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                    },
                    .reg16 => |src_reg| {
                        try self.emit_rex_if_needed(.{ .w = true, .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0xB7);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                    },
                    .reg => |src_reg| try self.mov_reg_reg(dst_reg, src_reg),
                    .imm64 => |imm| {
                        if (imm == 0 and !preserve_flags) {
                            try self.xor_(dst, dst);
                        } else return error.Invalid64bMovSource;
                    },
                    .imm32 => |imm| {
                        if (imm == 0 and !preserve_flags) {
                            try self.xor_(dst, dst);
                        } else {
                            // mov    <reg>,<imm32>
                            try self.emit_rex_if_needed(.{ .b = need_rex(dst_reg) });
                            try self.emit(u8, 0xB8 + @as(u8, encode(dst_reg)));
                            try self.emit(u32, imm);
                        }
                    },
                    .mem => |src_m| try mov_reg_mem(self, .MemToReg, dst, src_m),
                    .freg32 => |src_reg| try mov_reg_freg(self, ._32, dst_reg, src_reg),
                    .freg64 => |src_reg| try mov_reg_freg(self, ._64, dst_reg, src_reg),
                    else => return error.InvalidMovSourceFromReg,
                }
            },
            .reg64 => |dst_reg| {
                switch (src) {
                    .reg64 => |src_reg| try self.mov_reg_reg(dst_reg, src_reg),
                    .imm64 => |imm| {
                        if (imm == 0 and !preserve_flags) {
                            try self.xor_(dst, dst);
                        } else {
                            // movabs <reg>,<imm64>
                            try self.emit_rex_if_needed(.{ .w = true, .b = need_rex(dst_reg) });
                            try self.emit(u8, 0xB8 + @as(u8, encode(dst_reg)));
                            try self.emit(u64, imm);
                        }
                    },
                    .mem => |src_m| try mov_reg_mem(self, .MemToReg, dst, src_m),
                    else => {
                        std.debug.print("Mov Unimplemented: {any}, {any}\n", .{ dst, src });
                        return error.Unimplemented;
                    },
                }
            },
            .freg32 => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| try mov_freg_reg(self, ._32, dst_reg, src_reg),
                    .freg32 => try scalar_floating_point_operation(self, ._32, .Mov, dst_reg, src),
                    .mem => |src_mem| try mov_reg_mem(self, .MemToReg, dst, src_mem),
                    else => return error.InvalidMovSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| try mov_freg_reg(self, ._64, dst_reg, src_reg),
                    .freg64 => try scalar_floating_point_operation(self, ._64, .Mov, dst_reg, src),
                    .mem => |src_mem| {
                        if (src_mem.size != 64) return error.InvalidMemSize;
                        try mov_reg_mem(self, .MemToReg, dst, src_mem);
                    },
                    else => return error.InvalidMovSource,
                }
            },
            else => return error.InvalidMovDestination,
        }
    }

    pub fn cmov(self: *@This(), condition: Condition, dst: Operand, src: Operand) !void {
        if (condition == .Always) return error.InvalidCondition;

        switch (dst) {
            .reg16, .reg, .reg64 => |reg| {
                const b64 = (dst == .reg64);
                if (dst == .reg16)
                    try self.emit(u8, 0x66);
                switch (src) {
                    .reg16, .reg, .reg64 => {
                        if (std.meta.activeTag(dst) != std.meta.activeTag(src)) return error.IncompatibleSourceAndDestination;
                        try self.emit_rex_if_needed(.{ .w = b64, .r = need_rex(dst), .b = need_rex(src) });
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0x40 | condition.nibble());
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst), .r_m = encode(src) });
                    },
                    .mem => |mem| {
                        if (dst.size() != mem.size) return error.OperandSizeMismatch;
                        try self.emit(REX, .{
                            .w = b64,
                            .r = need_rex(reg),
                            .x = if (mem.index) |i| need_rex(i) else false,
                            .b = need_rex(mem.base),
                        });
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0x40 | condition.nibble());
                        try self.emit_mem_addressing(encode(reg), mem);
                        return error.UntestedCmovEmit; // Implemented but unused, thus untested! Be careful!
                    },
                    else => return error.InvalidCmovSource,
                }
            },
            else => return error.InvalidCmovDestination,
        }
    }

    pub fn movsx(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            // FIXME: We don't keep track of registers sizes and default to 32bit. We might want to support explicit 64bit at some point.
            .reg, .reg64 => |dst_reg| {
                const is_64 = dst == .reg64;
                switch (src) {
                    .mem => |src_m| {
                        try self.emit_rex_if_needed(.{ .w = is_64, .r = need_rex(dst_reg), .b = need_rex(src_m.base) });
                        try self.emit(u8, 0x0F);
                        switch (src_m.size) {
                            8 => try self.emit(u8, 0xBE),
                            16 => try self.emit(u8, 0xBF),
                            else => return error.UnsupportedMovsxSourceSize,
                        }
                        try self.emit_mem_addressing(encode(dst_reg), src_m);
                    },
                    .reg8 => |src_reg| {
                        if (src_reg.require_rex_8bit()) {
                            // NOTE: Byte access to the lower 8 bits of these registers is only possible with a rex prefix,
                            // emit it unconditionally.
                            try self.emit(REX, .{ .w = is_64, .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        } else {
                            try self.emit_rex_if_needed(.{ .w = is_64, .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        }
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0xBE);
                        try self.emit(MODRM, .{
                            .mod = .reg,
                            .reg_opcode = encode(dst_reg),
                            .r_m = encode(src_reg),
                        });
                    },
                    .reg16 => |src_reg| {
                        try self.emit_rex_if_needed(.{ .w = is_64, .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0xBF);
                        try self.emit(MODRM, .{
                            .mod = .reg,
                            .reg_opcode = encode(dst_reg),
                            .r_m = encode(src_reg),
                        });
                    },
                    .reg => |src_reg| {
                        if (!is_64) return error.Movsx32to32; // Mov a normal mov instead!
                        try self.emit_rex_if_needed(.{ .w = is_64, .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit(u8, 0x63);
                        try self.emit(MODRM, .{
                            .mod = .reg,
                            .reg_opcode = encode(dst_reg),
                            .r_m = encode(src_reg),
                        });
                    },
                    else => return error.InvalidMovsxSource,
                }
            },
            else => return error.InvalidMovsxDestination,
        }
    }

    // Helper for 0x81 / 0x83 opcodes (Add, And, Sub...)
    fn mem_dest_imm_src(self: *@This(), reg_opcode: RegOpcode, dst_m: MemOperand, comptime ImmType: type, imm: ImmType) !void {
        std.debug.assert(dst_m.size == 32);
        try self.emit_rex_if_needed(.{ .b = need_rex(dst_m.base) });

        // 0x83: OP r/m32, imm8 - Sign-extended imm8 - Shorter encoding
        try self.emit(u8, if (imm < 0x80) 0x83 else 0x81);

        try self.emit_mem_addressing(@intFromEnum(reg_opcode), dst_m);

        if (imm < 0x80) {
            try self.emit(u8, @truncate(imm));
        } else {
            try self.emit(ImmType, imm);
        }
    }

    // FIXME: I don't have a better name.
    pub fn opcode_81_83(self: *@This(), comptime rax_dst_opcode_8: u8, comptime rax_dst_opcode: u8, comptime mr_opcode_8: u8, comptime mr_opcode: u8, comptime rm_opcode_8: u8, comptime rm_opcode: u8, comptime rm_imm_opcode: RegOpcode, dst: Operand, src: Operand) !void {
        switch (dst) {
            .reg8 => |dst_reg| {
                switch (src) {
                    .reg8 => |src_reg| {
                        if (dst_reg.require_rex_8bit() or src_reg.require_rex_8bit()) {
                            try self.emit(REX, .{ .r = need_rex(src), .b = need_rex(dst) });
                        } else {
                            try self.emit_rex_if_needed(.{ .r = need_rex(src), .b = need_rex(dst) });
                        }
                        try self.emit(u8, rm_opcode_8);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                    },
                    .imm8 => |imm8| {
                        if (dst_reg == .rax) { // OP AL, imm8
                            try self.emit(u8, rax_dst_opcode_8);
                            try self.emit(u8, imm8);
                        } else {
                            return error.Unimplemented80;
                        }
                    },
                    else => return error.Unimplemented80,
                }
            },
            .reg, .reg64 => |dst_reg| {
                const b64 = (dst == .reg64);
                switch (src) {
                    .reg, .reg64 => |src_reg| {
                        if (dst.tag() != src.tag()) return error.OperandSizeMismatch;
                        try self.emit_rex_if_needed(.{ .w = b64, .r = need_rex(src_reg), .b = need_rex(dst_reg) });
                        try self.emit(u8, mr_opcode);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(src_reg), .r_m = encode(dst_reg) });
                    },
                    .imm8, .imm32 => |imm| {
                        if (dst_reg == .rax and imm >= 0x80) { // OP EAX, imm32
                            try self.emit_rex_if_needed(.{ .w = b64 });
                            try self.emit(u8, rax_dst_opcode);
                            try self.emit(u32, imm);
                        } else {
                            try self.emit_rex_if_needed(.{ .w = b64, .b = need_rex(dst_reg) });
                            if (imm < 0x80) { // We can use the imm8 sign extended version for a shorter encoding.
                                try self.emit(u8, 0x83); // OP r/m32, imm8
                                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(rm_imm_opcode), .r_m = encode(dst_reg) });
                                try self.emit(u8, @truncate(imm));
                            } else {
                                try self.emit(u8, 0x81); // OP r/m32, imm32
                                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(rm_imm_opcode), .r_m = encode(dst_reg) });
                                try self.emit(u32, imm);
                            }
                        }
                    },
                    .mem => |src_m| {
                        switch (src_m.size) {
                            32, 64 => {
                                if (src_m.size == 64 and !b64) return error.OperandSizeMismatch;
                                try self.emit_rex_if_needed(.{ .w = b64, .r = need_rex(dst_reg), .b = need_rex(src_m.base) });
                                try self.emit(u8, rm_opcode);
                                try self.emit_mem_addressing(encode(dst_reg), src_m);
                            },
                            else => return error.OperandSizeMismatch,
                        }
                    },
                    else => return error.InvalidSource,
                }
            },
            .mem => |dst_m| {
                switch (src) {
                    .reg8 => |src_reg| {
                        if (dst_m.size != 8) return error.OperandSizeMismatch;
                        if (src_reg.require_rex_8bit()) {
                            try self.emit(REX, .{ .r = need_rex(src_reg), .b = need_rex(dst_m.base) });
                        } else {
                            try self.emit_rex_if_needed(.{ .r = need_rex(src_reg), .b = need_rex(dst_m.base) });
                        }
                        try self.emit(u8, mr_opcode_8);
                        try self.emit_mem_addressing(encode(src_reg), dst_m);
                    },
                    .reg => |src_reg| {
                        switch (dst_m.size) {
                            32, 64 => {
                                try self.emit_rex_if_needed(.{ .w = dst_m.size == 64, .r = need_rex(src_reg), .b = need_rex(dst_m.base) });
                                try self.emit(u8, mr_opcode);
                                try self.emit_mem_addressing(encode(src_reg), dst_m);
                            },
                            else => return error.OperandSizeMismatch,
                        }
                    },
                    .imm8 => |imm| {
                        switch (dst_m.size) {
                            8 => {
                                try self.emit_rex_if_needed(.{ .b = need_rex(dst_m.base) });
                                try self.emit(u8, 0x80);
                                try self.emit_mem_addressing(@intFromEnum(rm_imm_opcode), dst_m);
                                try self.emit(u8, imm);
                            },
                            else => {
                                // NOTE: The immediate value might be sign-extended (arithmetic instructions for example).
                                //       There are obviously legitimate use cases for this, but since this is not reflected in the JIT API,
                                //       and because I don't currently have a use for it, this is here to alert of potential misuse.
                                std.debug.assert(imm < 0x80);

                                if (dst_m.size == 16) try self.emit(u8, 0x66);
                                try self.emit_rex_if_needed(.{ .w = dst_m.size == 64, .b = need_rex(dst_m.base) });
                                try self.emit(u8, 0x83);
                                try self.emit_mem_addressing(@intFromEnum(rm_imm_opcode), dst_m);
                                try self.emit(u8, imm);
                            },
                        }
                    },
                    .imm32 => |imm| try mem_dest_imm_src(self, rm_imm_opcode, dst_m, u32, imm),
                    else => return error.InvalidSource,
                }
            },
            else => return error.InvalidDestination,
        }
    }

    pub fn add(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| try scalar_floating_point_operation(self, ._32, .Add, dst_reg, src),
            .freg64 => |dst_reg| try scalar_floating_point_operation(self, ._64, .Add, dst_reg, src),
            else => return opcode_81_83(self, 0x04, 0x05, 0x00, 0x01, 0x02, 0x03, .Add, dst, src),
        }
    }
    pub fn or_(self: *@This(), dst: Operand, src: Operand) !void {
        return opcode_81_83(self, 0x0C, 0x0D, 0x08, 0x09, 0x0A, 0x0B, .Or, dst, src);
    }
    pub fn adc(self: *@This(), dst: Operand, src: Operand) !void {
        return opcode_81_83(self, 0x14, 0x15, 0x10, 0x11, 0x12, 0x13, .Adc, dst, src);
    }
    pub fn sbb(self: *@This(), dst: Operand, src: Operand) !void {
        return opcode_81_83(self, 0x1C, 0x1D, 0x18, 0x19, 0x1A, 0x1B, .Sbb, dst, src);
    }
    pub fn and_(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            // Yes, it's the same thing for both sizes, we're operating on the full size of the xmm registers here.
            .freg32 => |dst_reg| {
                switch (src) {
                    .freg32 => |src_reg| {
                        try self.emit_rex_if_needed(.{ .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit_slice(u8, &[_]u8{ 0x0F, 0x54 });
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                    },
                    else => return error.InvalidSubSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .freg64 => |src_reg| {
                        try self.emit_rex_if_needed(.{ .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit_slice(u8, &[_]u8{ 0x0F, 0x54 });
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                    },
                    else => return error.InvalidSubSource,
                }
            },
            else => return opcode_81_83(self, 0x24, 0x25, 0x20, 0x21, 0x22, 0x23, .And, dst, src),
        }
    }
    pub fn sub(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| try scalar_floating_point_operation(self, ._32, .Sub, dst_reg, src),
            .freg64 => |dst_reg| try scalar_floating_point_operation(self, ._64, .Sub, dst_reg, src),
            else => return opcode_81_83(self, 0x2C, 0x2D, 0x28, 0x29, 0x2A, 0x2B, .Sub, dst, src),
        }
    }
    pub fn xor_(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            // Yes, it's the same thing for both sizes, we're operating on the full size of the xmm registers here.
            .freg32 => |dst_reg| {
                switch (src) {
                    .freg32 => |src_reg| {
                        try self.emit_rex_if_needed(.{ .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit_slice(u8, &[_]u8{ 0x0F, 0x57 });
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                    },
                    else => return error.InvalidXorSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .freg64 => |src_reg| {
                        try self.emit_rex_if_needed(.{ .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit_slice(u8, &[_]u8{ 0x0F, 0x57 });
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                    },
                    else => return error.InvalidXorSource,
                }
            },
            else => return opcode_81_83(self, 0x34, 0x35, 0x30, 0x31, 0x32, 0x33, .Xor, dst, src),
        }
    }
    pub fn cmp(self: *@This(), lhs: Operand, rhs: Operand) !void {
        switch (lhs) {
            .freg32 => |lhs_reg| {
                switch (rhs) {
                    .freg32 => |rhs_reg| try cmp_scalar_fp(self, ._32, lhs_reg, rhs_reg),
                    else => return error.InvalidCmpSource,
                }
            },
            .freg64 => |lhs_reg| {
                switch (rhs) {
                    .freg64 => |rhs_reg| try cmp_scalar_fp(self, ._64, lhs_reg, rhs_reg),
                    else => return error.InvalidCmpSource,
                }
            },
            else => return opcode_81_83(self, 0x3C, 0x3D, 0x38, 0x39, 0x3A, 0x3B, .Cmp, lhs, rhs),
        }
    }

    pub fn set_byte_condition(self: *@This(), condition: Condition, dst: Operand) !void {
        // NOTE: In 64-bit mode, r/m8 can not be encoded to access the following byte registers if a REX prefix is used: AH, BH, CH, DH.
        switch (dst) {
            .reg8 => |dst_reg| {
                if (dst_reg.require_rex_8bit()) {
                    try self.emit(REX, .{ .w = false, .b = need_rex(dst_reg) });
                } else {
                    try self.emit_rex_if_needed(.{ .w = false, .b = need_rex(dst_reg) });
                }
                try self.emit(u8, 0x0F);
                try self.emit(u8, switch (condition) {
                    .Always => return error.InvalidSetByteCondition,
                    else => |c| 0x90 | c.nibble(),
                });
                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = 0, .r_m = encode(dst_reg) });
            },
            else => return error.InvalidSetByteConditionDestination,
        }
    }

    pub fn mul(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .reg, .reg64 => |dst_reg| {
                switch (src) {
                    .reg, .reg64 => |src_reg| {
                        if (dst.tag() != src.tag()) return error.MulOperandMismatch;
                        const is_64 = dst == .reg64;

                        // FIXME: This is supposed to be a condensed version of the instruction for rax,
                        //        but it's measurably slower on my machine. What?
                        //        Disabling it for now.
                        if (comptime false and dst_reg == .rax) {
                            try self.emit_rex_if_needed(.{ .w = is_64, .b = need_rex(src_reg) });
                            try self.emit(u8, 0xF7);
                            try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = 5, .r_m = encode(src_reg) });
                        } else {
                            try self.emit_rex_if_needed(.{ .w = is_64, .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                            try self.emit(u8, 0x0F);
                            try self.emit(u8, 0xAF);
                            try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                        }
                    },
                    else => return error.InvalidMulSource,
                }
            },
            .freg32 => |dst_reg| try scalar_floating_point_operation(self, ._32, .Mul, dst_reg, src),
            .freg64 => |dst_reg| try scalar_floating_point_operation(self, ._64, .Mul, dst_reg, src),
            else => return error.InvalidMulDestination,
        }
    }

    pub fn div(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| try scalar_floating_point_operation(self, ._32, .Div, dst_reg, src),
            .freg64 => |dst_reg| try scalar_floating_point_operation(self, ._64, .Div, dst_reg, src),
            else => return error.InvalidDivDestination,
        }
    }

    pub fn fma(self: *@This(), dst: FPRegister, src1: FPRegister, src2: Operand) !void {
        // Assumes dst and src1 are f32
        // vfmadd231ss xmm1, xmm2, xmm3/m32 - Multiply scalar single precision floating-point value from xmm2 and xmm3/m32, add to xmm1 and put result in xmm1.
        switch (src2) {
            .freg32 => |src2_reg| {
                try self.emit(VEX3, .{
                    .not_r = !need_rex(dst),
                    .not_x = true,
                    .not_b = !need_rex(src2_reg),
                    .m = .x0F38,
                    .w = false,
                    .not_v = ~@intFromEnum(src1),
                    .l = 0,
                    .p = .x66,
                });
                try self.emit(u8, 0xB9); // Opcode
                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst), .r_m = encode(src2_reg) });
            },
            else => return error.UnsupportedFmaSource,
        }
    }

    pub fn sqrt(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| try scalar_floating_point_operation(self, ._32, .Sqrt, dst_reg, src),
            else => return error.InvalidSqrtDestination,
        }
    }

    pub fn min(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| try scalar_floating_point_operation(self, ._32, .Min, dst_reg, src),
            else => return error.InvalidMinDestination,
        }
    }

    pub fn max(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| try scalar_floating_point_operation(self, ._32, .Max, dst_reg, src),
            else => return error.InvalidMaxDestination,
        }
    }

    fn shift_instruction(self: *@This(), reg_opcode: ShiftRegOpcode, dst: Operand, amount: Operand) !void {
        switch (dst) {
            .reg8, .reg16, .reg, .reg64 => |dst_reg| {
                var is_64 = false;
                var opcode_offset: u8 = 1;

                switch (dst) {
                    .reg8 => opcode_offset = 0,
                    .reg16 => try self.emit(u8, 0x66),
                    .reg64 => is_64 = true,
                    else => {},
                }

                try self.emit_rex_if_needed(.{ .w = is_64, .b = need_rex(dst_reg) });

                switch (amount) {
                    .imm8 => |imm8| {
                        if (imm8 == 1) {
                            try self.emit(u8, 0xD0 + opcode_offset);
                            try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
                        } else {
                            try self.emit(u8, 0xC0 + opcode_offset);
                            try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
                            try self.emit(u8, imm8);
                        }
                    },
                    .reg => |src_reg| {
                        if (src_reg != .rcx)
                            return error.InvalidShiftRegister; // Only rcx is supported as a source for the shift amount in x86!
                        try self.emit(u8, 0xD2 + opcode_offset);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
                    },
                    else => return error.InvalidShiftAmount,
                }
            },
            .mem => return error.TODOMemShift,
            else => return error.InvalidShiftDestination,
        }
    }

    fn f7_op(self: *@This(), dst: Operand, opcode: OtherRegOpcode) !void {
        switch (dst) {
            .reg => |dst_reg| {
                try self.emit_rex_if_needed(.{ .b = need_rex(dst_reg) });
                try self.emit(u8, 0xF7);
                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(opcode), .r_m = encode(dst) });
            },
            else => return error.InvalidF7Destination,
        }
    }

    fn convert(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .reg => |dst_reg| {
                switch (src) {
                    .freg32, .freg64 => |src_reg| {
                        try self.emit_byte(switch (src) {
                            .freg32 => 0xF3, // CVTTSS2SI - Convert With Truncation Scalar Single Precision Floating-Point Value to Integer
                            .freg64 => 0xF2, // CVTTSD2SI - Convert With Truncation Scalar Double Precision Floating-Point Value to Integer
                            else => unreachable,
                        });
                        try self.emit_rex_if_needed(.{
                            .w = false,
                            .r = need_rex(dst_reg),
                            .b = need_rex(src_reg),
                        });
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0x2C);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst_reg), .r_m = encode(src_reg) });
                    },
                    else => return error.InvalidConvertSource,
                }
            },
            .freg32 => |dst_reg| {
                switch (src) {
                    .mem => |src_mem| {
                        std.debug.assert(src_mem.size == 32);
                        // CVTSI2SS
                        try self.emit_byte(0xF3);
                        try self.emit_rex_if_needed(.{
                            .w = false,
                            .r = need_rex(dst_reg),
                            .x = if (src_mem.index) |i| need_rex(i) else false,
                            .b = need_rex(src_mem.base),
                        });
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0x2A);
                        try self.emit_mem_addressing(encode(dst_reg), src_mem);
                    },
                    else => return error.InvalidConvertSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .mem => |src_mem| {
                        std.debug.assert(src_mem.size == 32);
                        // CVTSI2SD
                        try self.emit_byte(0xF2);
                        try self.emit_rex_if_needed(.{
                            .w = false,
                            .r = need_rex(dst_reg),
                            .x = if (src_mem.index) |i| need_rex(i) else false,
                            .b = need_rex(src_mem.base),
                        });
                        try self.emit(u8, 0x0F);
                        try self.emit(u8, 0x2A);
                        try self.emit_mem_addressing(encode(dst_reg), src_mem);
                    },
                    else => return error.InvalidConvertSource,
                }
            },
            else => return error.InvalidConvertDestination,
        }
    }

    fn div64_32(self: *@This(), dividend_high: Register, dividend_low: Register, divisor: Register, result: Register) !void {
        if (divisor == .rax or divisor == .rdx) return error.InvalidDivisorRegister;
        // Some register shuffling
        if (dividend_high == .rax) {
            try self.mov(.{ .reg = .rdx }, .{ .reg = dividend_high }, false); // Avoid overwriting it before copying it.
            try self.mov(.{ .reg = .rax }, .{ .reg = dividend_low }, false);
        } else {
            if (dividend_low != .rax)
                try self.mov(.{ .reg = .rax }, .{ .reg = dividend_low }, false);
            if (dividend_high != .rdx)
                try self.mov(.{ .reg = .rdx }, .{ .reg = dividend_high }, false);
        }
        // Actual div instruction
        try self.emit_rex_if_needed(.{ .b = need_rex(divisor) });
        try self.emit(u8, 0xF7);
        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = 6, .r_m = encode(divisor) });
        if (result != .rax)
            try self.mov(.{ .reg = result }, .{ .reg = .rax }, false);
    }

    pub fn bit_test(self: *@This(), reg: Register, offset: Operand) !void {
        // NOTE: We only support 32-bit registers here.
        switch (offset) {
            .imm8 => |imm| {
                try self.emit(u8, 0x0F);
                try self.emit(u8, 0xBA);
                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = 4, .r_m = encode(reg) });
                try self.emit(u8, imm);
            },
            else => return error.UnsupportedBitTestOffset,
        }
    }

    fn emit_jmp_rel8(self: *@This(), condition: Condition, rel: i8) !void {
        try self.emit(u8, switch (condition) {
            .Always => 0xEB,
            else => |c| 0x70 | c.nibble(),
        });
        try self.emit(i8, rel);
    }

    pub fn jmp(self: *@This(), condition: Condition, current_idx: u32, dst: anytype) !void {
        switch (dst) {
            .rel => |rel| {
                // TODO: Support more destination than just immediate relative.
                //       Support different sizes of rel (rel8 in particular).
                //         We don't know the size of the jump yet, and we have to reserve enough space
                //         for the operand. Not sure what's the best way to handle this, or this is even worth it.

                const target_idx: u32 = @intCast(@as(i32, @intCast(current_idx)) + rel);
                if (rel < 0 and @as(i32, @intCast(self._instruction_offsets[target_idx])) - @as(i32, @intCast(self.block_size + 2)) > -128) {
                    try self.emit_jmp_rel8(condition, @intCast(@as(i32, @intCast(self._instruction_offsets[target_idx])) - @as(i32, @intCast(self.block_size + 2))));
                    return;
                }

                // Try to emit a rel8 jump. Our instruction can map to multiple x86 instructions of up to 15 bytes each, so I'm being really conservative here.
                // The important thing is to include forward jumps due to skipped fallback memory access with FastMem, which are extremely common and should be 3 "instructions" at most.
                if (rel > 0 and rel <= 4) {
                    const address_to_patch = self.block_size + 1;
                    try self.emit_jmp_rel8(condition, 0);
                    const jumps = try self.forward_jumps_to_patch.getOrPut(target_idx);
                    if (!jumps.found_existing)
                        jumps.value_ptr.* = .{};

                    try jumps.value_ptr.*.add(.{
                        .size = .r8,
                        .source = self.block_size,
                        .address_to_patch = address_to_patch,
                    });
                    return;
                }

                if (condition != .Always)
                    try self.emit(u8, 0x0F);

                try self.emit(u8, switch (condition) {
                    .Always => 0xE9,
                    else => |c| 0x80 | c.nibble(),
                });

                const address_to_patch = self.block_size;
                std.debug.assert(rel != 0);
                if (rel < 0) {
                    const next_instr_address = address_to_patch + 4;
                    std.debug.assert(target_idx >= 0);
                    try self.emit(u32, @bitCast(@as(i32, @intCast(self._instruction_offsets[target_idx])) - @as(i32, @intCast(next_instr_address))));
                } else {
                    try self.emit(u32, 0xA0C0FFEE);

                    const jumps = try self.forward_jumps_to_patch.getOrPut(target_idx);
                    if (!jumps.found_existing)
                        jumps.value_ptr.* = .{};

                    try jumps.value_ptr.*.add(.{
                        .size = .r32,
                        .source = self.block_size,
                        .address_to_patch = address_to_patch,
                    });
                }
            },
            .abs_indirect => |op| {
                if (condition != .Always) std.debug.panic("Unsupported indirect jump condition: {s}", .{@tagName(condition)});
                switch (op) {
                    .reg64 => |reg| {
                        try self.emit(u8, 0xFF);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = 4, .r_m = encode(reg) });
                    },
                    else => std.debug.panic("Unsupported indirect jump destination: {any}", .{op}),
                }
            },
        }
    }

    pub fn native_call(self: *@This(), function: ?*const anyopaque) !void {
        if (function) |fp| {
            // mov rax, function
            try self.emit(u8, 0x48);
            try self.emit(u8, 0xB8);
            try self.emit(u64, @intFromPtr(fp));
        }

        if (builtin.os.tag == .windows) {
            // Allocate shadow space - We still don't support specifying register sizes, so, hardcoding it.
            // sub rsp, 0x20
            try self.emit(u8, 0x48);
            try self.emit(u8, 0x83);
            try self.emit(u8, 0xEC);
            try self.emit(u8, 0x20);
        }

        // call rax
        try self.emit(u8, 0xFF);
        try self.emit(u8, 0xD0);

        if (builtin.os.tag == .windows) {
            // add rsp, 0x20
            try self.emit(u8, 0x48);
            try self.emit(u8, 0x83);
            try self.emit(u8, 0xC4);
            try self.emit(u8, 0x20);
        }
    }

    pub fn ret(self: *@This()) !void {
        try self.emit(u8, 0xC3);
    }

    pub fn save_fp_registers(self: *@This(), count: u8) !void {
        if (count > 0) {
            // sub rsp, count * 16
            const byte_count: u32 = count * 16;
            if (byte_count < 0x80) {
                try self.emit(u8, 0x48);
                try self.emit(u8, 0x83);
                try self.emit(u8, 0xEC);
                try self.emit(u8, @truncate(byte_count));
            } else {
                try self.emit(u8, 0x48);
                try self.emit(u8, 0x81);
                try self.emit(u8, 0xEC);
                try self.emit(u32, byte_count);
            }

            // movdqa XMMWORD PTR [rsp],xmm6      16bytes aligned version of movd, basically.
            // movdqa XMMWORD PTR [rsp+0x10],xmm7
            // ...
            // Yes, I know. I'm too lazy to provide actual codegen for movsqa
            const instrs = [_][]const u8{
                &[_]u8{ 0x66, 0x0f, 0x7f, 0x34, 0x24 },
                &[_]u8{ 0x66, 0x0f, 0x7f, 0x7c, 0x24, 0x10 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x7f, 0x44, 0x24, 0x20 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x7f, 0x4c, 0x24, 0x30 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x7f, 0x54, 0x24, 0x40 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x7f, 0x5c, 0x24, 0x50 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x7f, 0x64, 0x24, 0x60 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x7f, 0x6c, 0x24, 0x70 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x7f, 0xb4, 0x24, 0x80, 0x00, 0x00, 0x00 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x7f, 0xbc, 0x24, 0x90, 0x00, 0x00, 0x00 },
            };
            for (0..count) |i| {
                try self.emit_slice(u8, instrs[i]);
            }
        }
    }

    pub fn restore_fp_registers(self: *@This(), count: u8) !void {
        if (count > 0) {
            // movdqa xmm6,XMMWORD PTR [rsp]
            // movdqa xmm7,XMMWORD PTR [rsp+0x10]
            // ...
            const instrs = [_][]const u8{
                &[_]u8{ 0x66, 0x0f, 0x6f, 0x34, 0x24 },
                &[_]u8{ 0x66, 0x0f, 0x6f, 0x7c, 0x24, 0x10 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x6f, 0x44, 0x24, 0x20 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x6f, 0x4c, 0x24, 0x30 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x6f, 0x54, 0x24, 0x40 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x6f, 0x5c, 0x24, 0x50 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x6f, 0x64, 0x24, 0x60 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x6f, 0x6c, 0x24, 0x70 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x6f, 0xb4, 0x24, 0x80, 0x00, 0x00, 0x00 },
                &[_]u8{ 0x66, 0x44, 0x0f, 0x6f, 0xbc, 0x24, 0x90, 0x00, 0x00, 0x00 },
            };
            for (0..count) |i| {
                try self.emit_slice(u8, instrs[i]);
            }

            // add rsp, count * 16
            const byte_count: u32 = count * 16;
            if (byte_count < 0x80) {
                try self.emit(u8, 0x48);
                try self.emit(u8, 0x83);
                try self.emit(u8, 0xC4);
                try self.emit(u8, @truncate(byte_count));
            } else {
                try self.emit(u8, 0x48);
                try self.emit(u8, 0x81);
                try self.emit(u8, 0xC4);
                try self.emit(u32, byte_count);
            }
        }
    }
};
