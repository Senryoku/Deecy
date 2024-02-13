const std = @import("std");
const builtin = @import("builtin");

const x86_64_emitter_log = std.log.scoped(.x86_64_emitter);

const BasicBlock = @import("basic_block.zig");
const JIT = @import("jit_block.zig");
const JITBlock = @import("jit_block.zig").JITBlock;

// Tried using builtin.abi, but it returns .gnu on Windows.

pub const ReturnRegister = Register.rax;
pub const ScratchRegisters = [_]Register{ .r10, .r11 };

pub const ABI = enum {
    SystemV,
    Win64,
};

pub const JITABI = switch (builtin.os.tag) {
    .windows => .Win64,
    .linux => .SystemV,
    else => @compileError("Unsupported OS"),
};

// ArgRegisters are also used as scratch registers, but have a special meaning for function calls.
pub const ArgRegisters = switch (JITABI) {
    .Win64 => [_]Register{ .rcx, .rdx, .r8, .r9 },
    .SystemV => [_]Register{ .rdi, .rsi, .rdx, .rcx, .r8, .r9 },
    else => @compileError("Unsupported ABI"),
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
    else => @compileError("Unsupported ABI"),
};

pub const FPScratchRegisters = [_]FPRegister{
    .xmm0,
    .xmm1,
    .xmm2,
    .xmm3,
    .xmm4,
    .xmm5,
    .xmm6,
    .xmm7,
};

// This is only true for Win64
pub const FPSavedRegisters = [_]FPRegister{
    .xmm8,
    .xmm9,
    .xmm10,
    .xmm11,
    .xmm12,
    .xmm13,
    .xmm14,
    .xmm15,
};

const PatchableJump = struct {
    source: u32,
    address_to_patch: u32,
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

pub const MemOperand = struct {
    base: Register, // NOTE: This could be made optional as well, to allow for absolute addressing. However this is only possible on (r)ax on x86_64.
    index: ?Register = null,
    displacement: u32 = 0,
    size: u8,

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        if (value.index) |index| {
            return writer.print("[{any}+1*{any}+0x{X}]", .{
                value.base,
                index,
                value.displacement,
            });
        } else {
            return writer.print("[{any}+0x{X}]", .{
                value.base,
                value.displacement,
            });
        }
    }
};

const OperandType = enum {
    reg,
    freg32,
    freg64,
    freg128,
    imm8,
    imm16,
    imm32,
    imm64,
    mem,
};

pub const Operand = union(OperandType) {
    reg: Register,
    freg32: FPRegister,
    freg64: FPRegister,
    freg128: FPRegister,
    imm8: u8,
    imm16: u16,
    imm32: u32,
    imm64: u64,
    mem: MemOperand,

    pub fn tag(self: @This()) OperandType {
        return switch (self) {
            .reg => .reg,
            .freg32 => .freg32,
            .freg64 => .freg64,
            .freg128 => .freg128,
            .imm8 => .imm8,
            .imm16 => .imm16,
            .imm32 => .imm32,
            .imm64 => .imm64,
            .mem => .mem,
        };
    }

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return switch (value) {
            .reg => |reg| writer.print("{any}", .{reg}),
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
};

pub const InstructionType = enum {
    Nop,
    Break, // For Debugging
    FunctionCall,
    Mov, // Mov with zero extention (NOTE: This is NOT the same as the x86 mov instruction, which doesn't zero extend from 8 and 16-bit memory accesses)
    Movsx, // Mov with sign extension
    Push,
    Pop,
    Not,
    Add,
    Sub,
    Mul,
    Div,
    And,
    Or,
    Xor,
    Cmp,
    BitTest,
    Rol,
    Ror,
    Rcl,
    Rcr,
    Shl,
    Shr,
    Sar,
    Jmp,
    Div64_32, // FIXME: This only exists because I haven't added a way to specify the size the GPRs.
};

pub const Instruction = union(InstructionType) {
    Nop, // Usefull to patch out instructions without having to rewrite the entire block.
    Break,
    FunctionCall: *const anyopaque, // FIXME: Is there a better type for generic function pointers?
    Mov: struct { dst: Operand, src: Operand },
    Movsx: struct { dst: Operand, src: Operand },
    Push: Operand,
    Pop: Operand,
    Not: struct { dst: Register },
    Add: struct { dst: Operand, src: Operand },
    Sub: struct { dst: Operand, src: Operand },
    Mul: struct { dst: Operand, src: Operand },
    Div: struct { dst: Operand, src: Operand },
    And: struct { dst: Operand, src: Operand },
    Or: struct { dst: Operand, src: Operand },
    Xor: struct { dst: Operand, src: Operand },
    Cmp: struct { lhs: Operand, rhs: Operand },
    BitTest: struct { reg: Register, offset: Operand },
    Rol: struct { dst: Operand, amount: Operand },
    Ror: struct { dst: Operand, amount: Operand },
    Rcl: struct { dst: Operand, amount: Operand },
    Rcr: struct { dst: Operand, amount: Operand },
    Shl: struct { dst: Operand, amount: Operand },
    Shr: struct { dst: Operand, amount: Operand },
    Sar: struct { dst: Operand, amount: Operand },
    Jmp: struct { condition: Condition, dst: struct { rel: u32 } },
    Div64_32: struct { dividend_high: Register, dividend_low: Register, divisor: Register, result: Register },

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return switch (value) {
            .Nop => writer.print("nop", .{}),
            .Break => writer.print("break", .{}),
            .FunctionCall => |function| writer.print("call {any}", .{function}),
            .Mov => |mov| writer.print("mov {any}, {any}", .{ mov.dst, mov.src }),
            .Movsx => |movsx| writer.print("movsx {any}, {any}", .{ movsx.dst, movsx.src }),
            .Push => |push| writer.print("push {any}", .{push}),
            .Pop => |pop| writer.print("pop {any}", .{pop}),
            .Not => |not| writer.print("not {any}", .{not.dst}),
            .Add => |add| writer.print("add {any}, {any}", .{ add.dst, add.src }),
            .Sub => |sub| writer.print("sub {any}, {any}", .{ sub.dst, sub.src }),
            .Mul => |sub| writer.print("mul {any}, {any}", .{ sub.dst, sub.src }),
            .Div => |sub| writer.print("div {any}, {any}", .{ sub.dst, sub.src }),
            .And => |and_| writer.print("and {any}, {any}", .{ and_.dst, and_.src }),
            .Or => |or_| writer.print("or {any}, {any}", .{ or_.dst, or_.src }),
            .Xor => |or_| writer.print("or {any}, {any}", .{ or_.dst, or_.src }),
            .Cmp => |cmp| writer.print("cmp {any}, {any}", .{ cmp.lhs, cmp.rhs }),
            .BitTest => |bit_test| writer.print("bt {any}, {any}", .{ bit_test.reg, bit_test.offset }),
            .Jmp => |jmp| writer.print("jmp {any} 0x{x}", .{ jmp.condition, jmp.dst.rel }),
            .Rol => |rol| writer.print("rol {any}, {any}", .{ rol.dst, rol.amount }),
            .Ror => |ror| writer.print("ror {any}, {any}", .{ ror.dst, ror.amount }),
            .Rcl => |rcl| writer.print("rcl {any}, {any}", .{ rcl.dst, rcl.amount }),
            .Rcr => |rcr| writer.print("rcr {any}, {any}", .{ rcr.dst, rcr.amount }),
            .Shl => |shl| writer.print("shl {any}, {any}", .{ shl.dst, shl.amount }),
            .Shr => |shr| writer.print("shr {any}, {any}", .{ shr.dst, shr.amount }),
            .Sar => |sar| writer.print("sar {any}, {any}", .{ sar.dst, sar.amount }),
            .Div64_32 => |div| writer.print("div64_32 {any},{any}:{any},{any},", .{ div.result, div.dividend_high, div.dividend_low, div.divisor }),
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

const REX = packed struct(u8) {
    b: bool = false, // Extension of the ModR/M r/m field, SIB base field, or Opcode reg field
    x: bool = false, // Extension of the SIB index field
    r: bool = false, // Extension of the ModR/M reg field
    w: bool = false, // 0 = Operand size determined by CS.D; 1 = 64 Bit Operand Siz
    _: u4 = 0b0100,
};

const Mod = enum(u2) {
    indirect = 0b00, // Register indirect, or SIB without displacement (r/m = 0b100), or Displacement only (r/m = 0b101)
    disp8 = 0b01, // 8-bit displacement
    disp32 = 0b10, // 32-bit displacement
    reg = 0b11,
};

const MODRM = packed struct(u8) {
    r_m: u3, // The r/m field can specify a register as an operand or it can be combined with the mod field to encode an addressing mode. Sometimes, certain combinations of the mod field and the r/m field are used to express opcode information for some instructions.
    reg_opcode: u3, // The reg/opcode field specifies either a register number or three more bits of opcode information. The purpose of the reg/opcode field is specified in the primary opcode.
    mod: Mod, // The mod field combines with the r/m field to form 32 possible values: eight registers and 24 addressing modes
};

const SIB = packed struct(u8) {
    base: u3,
    index: u3,
    scale: u2,
};

const ScalarFPOpcodes = enum(u8) {
    Mov = 0x10,
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

pub const Emitter = struct {
    block: BasicBlock,
    block_size: u32 = 0,

    jumps_to_patch: std.AutoHashMap(u32, std.ArrayList(PatchableJump)) = undefined,

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, block_buffer: []u8) @This() {
        return .{
            .block = BasicBlock.init(block_buffer),
            .jumps_to_patch = std.AutoHashMap(u32, std.ArrayList(PatchableJump)).init(allocator),
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        self.jumps_to_patch.deinit();
    }

    pub fn emit_instructions(self: *@This(), instructions: []const Instruction) !void {
        for (instructions, 0..) |instr, idx| {
            if (self.jumps_to_patch.get(@intCast(idx))) |jumps| {
                for (jumps.items) |jump| {
                    const rel: u32 = @intCast(self.block_size - jump.source);
                    @memcpy(@as([*]u8, @ptrCast(&self.block.buffer[jump.address_to_patch]))[0..4], @as([*]const u8, @ptrCast(&rel)));
                }
                jumps.deinit();
                _ = self.jumps_to_patch.remove(@intCast(idx));
            }

            switch (instr) {
                .Nop => {},
                .Break => {
                    if (builtin.mode == .Debug) {
                        try self.emit_byte(0xCC);
                    } else std.debug.print("[x86_64 Emitter] Warning: Emitting a break instruction outside of Debug Build.\n", .{});
                },
                .FunctionCall => |function| try self.native_call(function),
                .Mov => |m| try self.mov(m.dst, m.src),
                .Movsx => |m| try self.movsx(m.dst, m.src),
                .Push => |reg_or_imm| {
                    switch (reg_or_imm) {
                        .reg => |reg| {
                            try self.emit_rex_if_needed(.{ .b = need_rex(reg) });
                            try self.emit(u8, encode_opcode(0x50, reg));
                        },
                        else => return error.UnimplementedPushImmediate,
                    }
                },
                .Pop => |reg_or_imm| {
                    switch (reg_or_imm) {
                        .reg => |reg| {
                            try self.emit_rex_if_needed(.{ .b = need_rex(reg) });
                            try self.emit(u8, encode_opcode(0x58, reg));
                        },
                        else => return error.UnimplementedPushImmediate,
                    }
                },
                .Add => |a| try self.add(a.dst, a.src),
                .Sub => |a| try self.sub(a.dst, a.src),
                .Mul => |a| try self.mul(a.dst, a.src),
                .Div => |a| try self.div(a.dst, a.src),
                .And => |a| try self.and_(a.dst, a.src),
                .Or => |a| try self.or_(a.dst, a.src),
                .Xor => |a| try self.xor_(a.dst, a.src),
                .Cmp => |a| try self.cmp(a.lhs, a.rhs),
                .Jmp => |j| {
                    std.debug.assert(j.dst.rel > 0); // We don't support backward jumps, yet.
                    try self.jmp(j.condition, @intCast(idx + j.dst.rel));
                },
                .BitTest => |b| {
                    try self.bit_test(b.reg, b.offset);
                },
                .Rol => |r| try self.rol(r.dst, r.amount),
                .Ror => |r| try self.ror(r.dst, r.amount),
                .Sar => |r| try self.sar(r.dst, r.amount),
                .Shr => |r| try self.shr(r.dst, r.amount),
                .Shl => |r| try self.shl(r.dst, r.amount),
                .Not => |r| try self.not(r.dst),
                .Div64_32 => |d| try self.div64_32(d.dividend_high, d.dividend_low, d.divisor, d.result),
                else => return error.UnsupportedInstruction,
            }
        }
        if (self.jumps_to_patch.count() > 0) {
            std.debug.print("Jumps left to patch: {}\n", .{self.jumps_to_patch.count()});
            @panic("Error: Unpatched jumps!");
        }
    }

    pub fn emit_byte(self: *@This(), value: u8) !void {
        self.block.buffer[self.block_size] = value;
        self.block_size += 1;
    }

    pub fn emit(self: *@This(), comptime T: type, value: T) !void {
        if (T == MODRM) {
            // See Intel Manual Vol. 2A 2-11.
            if (value.mod == .indirect and value.r_m == 0b101)
                return error.UnhandledSpecialCase;
        }

        if (@sizeOf(T) == 1) {
            if (@typeInfo(T) == .Enum) {
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
        // FIXME: Don't hardcode this? Please?

        // push rbp
        try self.emit(u8, 0x55);
        // mov    rbp,rsp
        try self.emit(u8, 0x48);
        try self.emit(u8, 0x89);
        try self.emit(u8, 0xE5);
    }

    pub fn emit_block_epilogue(self: *@This()) !void {
        // pop    rbp
        try self.emit(u8, 0x5D);
        // ret
        try self.ret();
    }

    fn encode(reg: anytype) u3 {
        return switch (@TypeOf(reg)) {
            Register => @truncate(@intFromEnum(reg)),
            FPRegister => @truncate(@intFromEnum(reg)),
            Operand => switch (reg) {
                .reg => |r| @truncate(@intFromEnum(r)),
                .freg32 => |r| @truncate(@intFromEnum(r)),
                .freg64 => |r| @truncate(@intFromEnum(r)),
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
                .reg => |r| @intFromEnum(r) >= 8,
                .freg32 => |r| @intFromEnum(r) >= 8,
                .freg64 => |r| @intFromEnum(r) >= 8,
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

    // <op>ss xmm1, xmm2/m32 (mem operand not yet supported here)
    pub fn scalar_floating_point_operation(self: *@This(), comptime size: OperandSize, opcode: ScalarFPOpcodes, dst: FPRegister, src: FPRegister) !void {
        try self.emit(u8, switch (size) {
            ._32 => 0xF3,
            ._64 => 0xF2,
            else => @compileError("Unsupported operand size"),
        });
        try self.emit_rex_if_needed(.{ .w = false, .r = need_rex(dst), .b = need_rex(src) });
        try self.emit(u8, 0x0F);
        try self.emit(ScalarFPOpcodes, opcode);
        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(dst), .r_m = encode(src) });
    }

    pub fn mov_reg_mem(self: *@This(), comptime direction: enum { MemToReg, RegToMem }, reg: Operand, mem: MemOperand) !void {
        var reg_64 = mem.size == 64;

        if (mem.base == .rbp and mem.displacement == 0) {
            // See Intel Manual Vol. 2A 2-11.
            return error.UnimplementedMandatoryExplicitDisplacement;
        }

        if (direction == .MemToReg and mem.size < 32)
            reg_64 = true; // Force 64-bit register to be 100% sure all bits are cleared.

        const opcode: []const u8 = switch (direction) {
            .MemToReg => switch (reg) {
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
                .reg => switch (mem.size) {
                    8 => &[_]u8{0x88},
                    16 => &[_]u8{ 0x66, 0x88 },
                    32, 64 => &[_]u8{0x89},
                    else => return error.InvalidMemSize,
                },
                .freg32, .freg64 => &[_]u8{ 0x0F, 0x7E }, // movd/movq
                else => return error.InvalidRegisterType,
            },
        };

        if (reg.tag() == .freg32 or reg.tag() == .freg64)
            try self.emit(u8, 0x66);

        try self.emit_rex_if_needed(.{
            .w = reg_64,
            .r = need_rex(reg),
            .x = if (mem.index) |i| need_rex(i) else false,
            .b = need_rex(mem.base),
        });

        for (opcode) |o| try self.emit(u8, o);

        const r_m: u3 = if (mem.index != null) 0b100 // A SIB is following
        else encode(mem.base); // If base is ESP/R12, a SIB will also be emitted.

        try self.emit(MODRM, .{
            .mod = if (mem.displacement == 0) .indirect else if (mem.displacement < 0x80) .disp8 else .disp32,
            .reg_opcode = encode(reg),
            .r_m = r_m,
        });

        if (r_m == 0b100) {
            try self.emit(SIB, .{
                .scale = 0,
                // NOTE: ESP/R12-based addressing needs a SIB byte, this is the 0b100 case.
                .index = if (mem.index) |i| encode(i) else 0b100,
                .base = encode(mem.base),
            });
        }

        if (mem.displacement >= 0x80) {
            try self.emit(u32, mem.displacement);
        } else if (mem.displacement != 0) {
            try self.emit(u8, @truncate(mem.displacement));
        }
    }

    pub fn mov(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .mem => |dst_m| {
                switch (src) {
                    .reg => try mov_reg_mem(self, .RegToMem, src, dst_m),
                    .imm32 => |imm| {
                        try self.emit_rex_if_needed(.{ .b = need_rex(dst_m.base) });
                        try self.emit(u8, 0xC7);
                        try self.emit(MODRM, .{
                            .mod = if (dst_m.displacement == 0) .indirect else .disp32,
                            .reg_opcode = 0,
                            .r_m = encode(dst_m.base),
                        });
                        // NOTE: ESP/R12-based addressing need a SIB byte.
                        if (encode(dst_m.base) == 0b100)
                            try self.emit(SIB, .{ .scale = 0, .index = 0b100, .base = 0b100 });
                        if (dst_m.displacement != 0)
                            try self.emit(u32, dst_m.displacement);
                        try self.emit(u32, imm);
                    },
                    .freg32 => try mov_reg_mem(self, .RegToMem, src, dst_m),
                    .freg64 => try mov_reg_mem(self, .RegToMem, src, dst_m),
                    else => return error.InvalidMovSource,
                }
            },
            .reg => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| try self.mov_reg_reg(dst_reg, src_reg),
                    .imm64 => |imm| {
                        if (imm == 0) {
                            try self.xor_(dst, dst);
                        } else {
                            // movabs <reg>,<imm64>
                            try self.emit_rex_if_needed(.{ .w = true, .b = need_rex(dst_reg) });
                            try self.emit(u8, 0xB8 + @as(u8, encode(dst_reg)));
                            try self.emit(u64, imm);
                        }
                    },
                    .imm32 => |imm| {
                        if (imm == 0) {
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
                    else => return error.InvalidMovSource,
                }
            },
            .freg32 => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| try mov_freg_reg(self, ._32, dst_reg, src_reg),
                    .freg32 => |src_reg| try scalar_floating_point_operation(self, ._32, .Mov, dst_reg, src_reg),
                    .mem => |src_mem| try mov_reg_mem(self, .MemToReg, dst, src_mem),
                    else => return error.InvalidMovSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| try mov_freg_reg(self, ._64, dst_reg, src_reg),
                    .freg64 => |src_reg| try scalar_floating_point_operation(self, ._64, .Mov, dst_reg, src_reg),
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

    pub fn movsx(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .reg => |dst_reg| {
                switch (src) {
                    .mem => |src_m| {
                        // FIXME: We don't keep track of registers sizes and default to 32bit. We might want to support explicit 64bit at some point.
                        try self.emit_rex_if_needed(.{ .w = false, .r = need_rex(dst_reg), .b = need_rex(src_m.base) });
                        try self.emit(u8, 0x0F);
                        switch (src_m.size) {
                            8 => try self.emit(u8, 0xBE),
                            16 => try self.emit(u8, 0xBF),
                            else => return error.UnsupportedMovsxSourceSize,
                        }
                        try self.emit(MODRM, .{
                            .mod = if (src_m.displacement == 0) .indirect else .disp32,
                            .reg_opcode = encode(dst.reg),
                            .r_m = encode(src_m.base),
                        });
                        // NOTE: ESP/R12-based addressing need a SIB byte.
                        if (encode(src_m.base) == 0b100)
                            try self.emit(SIB, .{ .scale = 0, .index = 0b100, .base = 0b100 });
                        if (src_m.displacement != 0)
                            try self.emit(u32, src_m.displacement);
                    },
                    .reg => |src_reg| {

                        // FIXME: We only support sign extending from 16-bits to 32-bits right now.
                        //        Registers don't currently have a size and we can't express other instructions!
                        const src_size = 16;

                        try self.emit_rex_if_needed(.{ .w = false, .r = need_rex(dst_reg), .b = need_rex(src_reg) });
                        try self.emit(u8, 0x0F);
                        switch (src_size) {
                            8 => try self.emit(u8, 0xBE),
                            16 => try self.emit(u8, 0xBF),
                            else => return error.UnsupportedMovsxSourceSize,
                        }
                        try self.emit(MODRM, .{
                            .mod = .reg,
                            .reg_opcode = encode(dst.reg),
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

        // NOTE: I'm not entirely sure how emitting a 32-bit displacement works here.
        try self.emit_rex_if_needed(.{ .b = need_rex(dst_m.base) });

        // 0x83: OP r/m32, imm8 - Sign-extended imm8 - Shorter encoding
        try self.emit(u8, if (imm < 0x80) 0x83 else 0x81);

        try self.emit(MODRM, .{
            .mod = if (dst_m.displacement == 0) .indirect else (if (dst_m.displacement < 0x80) .disp8 else .disp32),
            .reg_opcode = @intFromEnum(reg_opcode),
            .r_m = encode(dst_m.base),
        });

        if (encode(dst_m.base) == 0b100) // Special case for r12
            try self.emit(SIB, .{ .scale = 0, .index = 0b100, .base = 0b100 });

        if (dst_m.displacement != 0) {
            if (dst_m.displacement < 0x80) {
                try self.emit(u8, @truncate(dst_m.displacement));
            } else {
                try self.emit(u32, dst_m.displacement);
            }
        }

        if (imm < 0x80) {
            try self.emit(u8, @truncate(imm));
        } else {
            try self.emit(ImmType, imm);
        }
    }

    fn mem_dest_reg_src(self: *@This(), opcode: u8, dst_m: MemOperand, reg: Register) !void {
        std.debug.assert(dst_m.size == 32);

        // NOTE: I'm not entirely sure how emitting a 32-bit displacement works here.
        try self.emit_rex_if_needed(.{ .b = need_rex(dst_m.base) });

        try self.emit(u8, opcode);

        try self.emit(MODRM, .{
            .mod = if (dst_m.displacement == 0) .indirect else (if (dst_m.displacement < 0x80) .disp8 else .disp32),
            .reg_opcode = encode(reg),
            .r_m = encode(dst_m.base),
        });

        if (encode(dst_m.base) == 0b100) // Special case for r12
            try self.emit(SIB, .{ .scale = 0, .index = 0b100, .base = 0b100 });

        if (dst_m.displacement != 0) {
            if (dst_m.displacement < 0x80) {
                try self.emit(u8, @truncate(dst_m.displacement));
            } else {
                try self.emit(u32, dst_m.displacement);
            }
        }
    }

    fn reg_dest_imm_src(self: *@This(), reg_opcode: RegOpcode, dst_reg: Register, imm32: u32) !void {
        // Register is always 32bits
        try self.emit_rex_if_needed(.{ .b = need_rex(dst_reg) });
        if (imm32 < 0x80) { // We can use the imm8 sign extended version for a shorter encoding.
            try self.emit(u8, 0x83); // OP r/m32, imm8
            try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
            try self.emit(u8, @truncate(imm32));
        } else {
            try self.emit(u8, 0x81); // OP r/m32, imm32
            try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
            try self.emit(u32, imm32);
        }
    }

    // FIXME: I don't have a better name.
    pub fn opcode_81_83(self: *@This(), comptime rax_dst_opcode_8: u8, comptime rax_dst_opcode: u8, comptime mr_opcode_8: u8, comptime mr_opcode: u8, comptime rm_opcode_8: u8, comptime rm_opcode: u8, comptime rm_imm_opcode: RegOpcode, dst: Operand, src: Operand) !void {
        _ = rax_dst_opcode_8;
        _ = mr_opcode_8;
        _ = rm_opcode_8;
        _ = rm_opcode;

        switch (dst) {
            .reg => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| {
                        try self.emit_rex_if_needed(.{ .r = need_rex(src_reg), .b = need_rex(dst_reg) });
                        try self.emit(u8, mr_opcode);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(src_reg), .r_m = encode(dst_reg) });
                    },
                    .imm32 => |imm32| {
                        if (dst_reg == .rax and imm32 >= 0x80) {
                            // OP EAX, imm32
                            try self.emit(u8, rax_dst_opcode);
                            try self.emit(u32, imm32);
                        } else try reg_dest_imm_src(self, rm_imm_opcode, dst_reg, imm32);
                    },
                    .imm8 => |imm8| {
                        try reg_dest_imm_src(self, rm_imm_opcode, dst_reg, imm8);
                    },
                    else => return error.InvalidSource,
                }
            },
            .mem => |dst_m| {
                switch (src) {
                    .reg => |src_reg| {
                        try mem_dest_reg_src(self, mr_opcode, dst_m, src_reg);
                    },
                    .imm32 => |imm| {
                        try mem_dest_imm_src(self, rm_imm_opcode, dst_m, u32, imm);
                    },
                    else => return error.InvalidSource,
                }
            },
            else => return error.InvalidDestination,
        }
    }

    pub fn add(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| {
                switch (src) {
                    .freg32 => |src_reg| try scalar_floating_point_operation(self, ._32, .Add, dst_reg, src_reg),
                    else => return error.InvalidAddSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .freg64 => |src_reg| try scalar_floating_point_operation(self, ._64, .Add, dst_reg, src_reg),
                    else => return error.InvalidAddSource,
                }
            },
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
        return opcode_81_83(self, 0x24, 0x25, 0x20, 0x21, 0x22, 0x23, .And, dst, src);
    }
    pub fn sub(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| {
                switch (src) {
                    .freg32 => |src_reg| try scalar_floating_point_operation(self, ._32, .Sub, dst_reg, src_reg),
                    else => return error.InvalidSubSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .freg64 => |src_reg| try scalar_floating_point_operation(self, ._64, .Sub, dst_reg, src_reg),
                    else => return error.InvalidSubSource,
                }
            },
            else => return opcode_81_83(self, 0x2C, 0x2D, 0x28, 0x29, 0x2A, 0x2B, .Sub, dst, src),
        }
    }
    pub fn xor_(self: *@This(), dst: Operand, src: Operand) !void {
        return opcode_81_83(self, 0x34, 0x35, 0x30, 0x31, 0x32, 0x33, .Xor, dst, src);
    }
    pub fn cmp(self: *@This(), lhs: Operand, rhs: Operand) !void {
        return opcode_81_83(self, 0x3C, 0x3D, 0x38, 0x39, 0x3A, 0x3B, .Cmp, lhs, rhs);
    }

    pub fn mul(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| {
                switch (src) {
                    .freg32 => |src_reg| try scalar_floating_point_operation(self, ._32, .Mul, dst_reg, src_reg),
                    else => return error.InvalidMulSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .freg64 => |src_reg| try scalar_floating_point_operation(self, ._64, .Mul, dst_reg, src_reg),
                    else => return error.InvalidMulSource,
                }
            },
            else => return error.InvalidMulDestination,
        }
    }

    pub fn div(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .freg32 => |dst_reg| {
                switch (src) {
                    .freg32 => |src_reg| try scalar_floating_point_operation(self, ._32, .Div, dst_reg, src_reg),
                    else => return error.InvalidDivSource,
                }
            },
            .freg64 => |dst_reg| {
                switch (src) {
                    .freg64 => |src_reg| try scalar_floating_point_operation(self, ._64, .Div, dst_reg, src_reg),
                    else => return error.InvalidDivSource,
                }
            },
            else => return error.InvalidDivDestination,
        }
    }

    fn shift_instruction(self: *@This(), reg_opcode: ShiftRegOpcode, dst: Operand, amount: Operand) !void {
        switch (dst) {
            .reg => |dst_reg| {
                switch (amount) {
                    .imm8 => |imm8| {
                        try self.emit_rex_if_needed(.{ .b = need_rex(dst_reg) });
                        if (imm8 == 1) {
                            try self.emit(u8, 0xD1);
                            try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
                        } else {
                            try self.emit(u8, 0xC1);
                            try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
                            try self.emit(u8, imm8);
                        }
                    },
                    .reg => |src_reg| {
                        if (src_reg != .rcx) {
                            return error.InvalidShiftRegister; // Only rcx is supported as a source for the shift amount in x86!
                        }
                        try self.emit_rex_if_needed(.{ .b = need_rex(dst_reg) });
                        try self.emit(u8, 0xD3);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
                    },
                    else => return error.InvalidShiftAmount,
                }
            },
            .mem => return error.TODOMemShift,
            else => return error.InvalidShiftDestination,
        }
    }

    fn sar(self: *@This(), dst: Operand, amount: Operand) !void {
        try self.shift_instruction(.Sar, dst, amount);
    }
    fn shr(self: *@This(), dst: Operand, amount: Operand) !void {
        try self.shift_instruction(.Shr, dst, amount);
    }
    fn shl(self: *@This(), dst: Operand, amount: Operand) !void {
        try self.shift_instruction(.Shl, dst, amount);
    }
    fn rol(self: *@This(), dst: Operand, amount: Operand) !void {
        try self.shift_instruction(.Rol, dst, amount);
    }
    fn ror(self: *@This(), dst: Operand, amount: Operand) !void {
        try self.shift_instruction(.Ror, dst, amount);
    }

    fn not(self: *@This(), dst: Register) !void {
        try self.emit(u8, 0xF7);
        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(OtherRegOpcode.Not), .r_m = encode(dst) });
    }

    fn div64_32(self: *@This(), dividend_high: Register, dividend_low: Register, divisor: Register, result: Register) !void {
        // Some register shuffling
        if (dividend_high == .rax) {
            try self.mov(.{ .reg = .rdx }, .{ .reg = dividend_high }); // Avoid overwriting it before copying it.
            try self.mov(.{ .reg = .rax }, .{ .reg = dividend_low });
        } else {
            if (dividend_low != .rax)
                try self.mov(.{ .reg = .rax }, .{ .reg = dividend_low });
            if (dividend_high != .rdx)
                try self.mov(.{ .reg = .rdx }, .{ .reg = dividend_high });
        }
        // Actual div instruction
        try self.emit_rex_if_needed(.{ .b = need_rex(divisor) });
        try self.emit(u8, 0xF7);
        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = 6, .r_m = encode(divisor) });
        try self.mov(.{ .reg = result }, .{ .reg = .rax });
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

    pub fn jmp(self: *@This(), condition: JIT.Condition, dst_instruction_index: u32) !void {
        // TODO: Support more destination than just immediate relative.
        //       Support different sizes of rel (rel8 in particular).
        //         We don't know the size of the jump yet, and we have to reserve enough space
        //         for the operand. Not sure what's the best way to handle this, or this is even worth it.

        var address = self.block_size;

        if (condition != .Always)
            try self.emit(u8, 0x0F);

        try self.emit(u8, switch (condition) {
            .Always => 0xE9,
            .Overflow => 0x80,
            .NotOverflow => 0x81,
            .Carry, .Below => 0x82,
            .NotCarry, .AboveEqual, .NotBelow => 0x83,
            .Equal, .Zero => 0x84,
            .NotEqual, .NotZero => 0x85,
            .NotAbove, .BelowEqual => 0x86,
            .Above, .NotBelowEqual => 0x87,
            .Sign => 0x88,
            .NotSign => 0x89,
            .ParityEven => 0x8A,
            .ParityOdd => 0x8B,
            .Less, .NotGreaterEqual => 0x8C,
            .GreaterEqual, .NotLess => 0x8D,
            .LessEqual, .NotGreater => 0x8E,
            .Greater, .NotLessEqual => 0x8F,
        });

        address = self.block_size;
        try self.emit(u32, 0x00C0FFEE);

        const jumps = try self.jumps_to_patch.getOrPut(dst_instruction_index);
        if (!jumps.found_existing)
            jumps.value_ptr.* = std.ArrayList(PatchableJump).init(self._allocator);

        try jumps.value_ptr.*.append(.{
            .source = self.block_size,
            .address_to_patch = address,
        });
    }

    pub fn native_call(self: *@This(), function: *const anyopaque) !void {
        // mov rax, function
        try self.emit(u8, 0x48);
        try self.emit(u8, 0xB8);
        try self.emit(u64, @intFromPtr(function));

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
};
