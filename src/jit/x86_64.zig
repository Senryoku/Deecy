const std = @import("std");
const builtin = @import("builtin");

const x86_64_emitter_log = std.log.scoped(.x86_64_emitter);

const BasicBlock = @import("basic_block.zig");
const JIT = @import("jit_block.zig");
const JITBlock = @import("jit_block.zig").JITBlock;

// Tried using builtin.abi, but it returns .gnu on Windows.

pub const ReturnRegister = Register.rax;
pub const ScratchRegisters = [_]Register{ .R10, .R11 };
// ArgRegisters are also used as scratch registers, but have a special meaning for function calls.
pub const ArgRegisters = if (builtin.os.tag == .windows) [_]Register{
    .rcx,
    .rdx,
    .r8,
    .r9,
} else if (builtin.os.tag == .linux) [_]Register{
    .rdi,
    .rsi,
    .rdx,
    .rcx,
    .r8,
    .r9,
} else @compileError("Unsupported ABI");
pub const SavedRegisters = if (builtin.os.tag == .windows) [_]Register{
    .r12,
    .r13,
    .r14,
    .r15,
    .rbx,
    .rsi,
    .rdi,
    // NOTE: Both are saved registers, but I don't think I should expose them.
    // .rbp,
    // .rsp,
} else if (builtin.os.tag == .linux) [_]Register{
    .r12,
    .r13,
    .r14,
    .r15,
    .rbx,
    // .rbp,
    // .rsp,
} else @compileError("Unsupported ABI");

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

pub const Condition = enum {
    Always,
    Equal,
    NotEqual,
    Carry,
    NotCarry,
    Greater, // Signed Values
    GreaterEqual,
    Above, // Unsigned Values

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return writer.print("{s}", .{@tagName(value)});
    }
};

pub const OperandSize = enum { _8, _16, _32, _64 };

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
    imm8,
    imm16,
    imm32,
    imm64,
    mem,
};

pub const Operand = union(OperandType) {
    reg: Register,
    imm8: u8,
    imm16: u16,
    imm32: u32,
    imm64: u64,
    mem: MemOperand,

    pub fn format(value: @This(), comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;
        return switch (value) {
            .reg => |reg| writer.print("{any}", .{reg}),
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
    Mov,
    Movsx, // Mov with sign extension
    Push,
    Pop,
    Not,
    Add,
    Sub,
    And,
    Or,
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
    And: struct { dst: Operand, src: Operand },
    Or: struct { dst: Operand, src: Operand },
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
            .And => |and_| writer.print("and {any}, {any}", .{ and_.dst, and_.src }),
            .Or => |or_| writer.print("or {any}, {any}", .{ or_.dst, or_.src }),
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

const REX = packed struct(u8) {
    b: bool = false,
    x: bool = false,
    r: bool = false,
    w: bool = false,
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

            x86_64_emitter_log.debug("[{d: >4}] {any}", .{ idx, instr });

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
                .And => |a| try self.and_(a.dst, a.src),
                .Or => |a| try self.or_(a.dst, a.src),
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
            try self.emit_byte(@bitCast(value));
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

    fn encode(reg: Register) u3 {
        return @truncate(@intFromEnum(reg));
    }

    fn need_rex(reg: Register) bool {
        return @intFromEnum(reg) >= 8;
    }

    fn encode_opcode(opcode: u8, reg: Register) u8 {
        return opcode + encode(reg);
    }

    fn emit_rex_if_needed(self: *@This(), rex: REX) !void {
        if (@as(u8, @bitCast(rex)) != @as(u8, @bitCast(REX{})))
            try self.emit(u8, @bitCast(rex));
    }

    pub fn mov_reg_reg(self: *@This(), dst: Register, src: Register) !void {
        try self.emit_rex_if_needed(.{ .w = true, .r = need_rex(src), .b = need_rex(dst) });
        try self.emit(u8, 0x89);
        const modrm: MODRM = .{ .mod = .reg, .reg_opcode = encode(src), .r_m = encode(dst) };
        try self.emit(u8, @bitCast(modrm));
    }

    pub fn mov(self: *@This(), dst: Operand, src: Operand) !void {
        switch (dst) {
            .mem => |dst_m| {
                switch (src) {
                    .reg => |src_reg| {
                        if (dst_m.index != null) {
                            if (dst_m.displacement != 0) // TODO
                                return error.MovIndexWithDisplacementNotSupported;
                            try self.emit_rex_if_needed(.{
                                .w = dst_m.size == 64,
                                .r = need_rex(src_reg),
                                .x = need_rex(dst_m.index.?),
                                .b = need_rex(dst_m.base),
                            });
                            const opcode = 0x89;
                            try self.emit(u8, opcode);
                            const modrm: MODRM = .{
                                .mod = .disp8,
                                .reg_opcode = encode(src_reg),
                                .r_m = 0b100, // FIXME: I don't know what I'm doing :D
                            };
                            try self.emit(u8, @bitCast(modrm));
                            const sib: SIB = .{
                                .scale = 0,
                                .index = encode(dst_m.index.?),
                                .base = encode(dst_m.base),
                            };
                            try self.emit(u8, @bitCast(sib));
                            try self.emit(u8, 0x00); // Zero displacement
                        } else {
                            try self.emit_rex_if_needed(.{ .w = dst_m.size == 64, .r = need_rex(src_reg), .b = need_rex(dst_m.base) });
                            const opcode = 0x89;
                            if (dst_m.size == 16) // Operand size prefix
                                try self.emit(u8, 0x66);
                            try self.emit(u8, opcode);
                            const modrm: MODRM = .{
                                .mod = if (dst_m.displacement == 0) .indirect else .disp32,
                                .reg_opcode = encode(src_reg),
                                .r_m = encode(dst_m.base),
                            };
                            try self.emit(u8, @bitCast(modrm));
                            // NOTE: ESP/R12-based addressing need a SIB byte.
                            if (encode(dst_m.base) == 0b100) {
                                try self.emit(u8, @bitCast(SIB{ .scale = 0, .index = 0b100, .base = 0b100 }));
                            }
                            if (dst_m.displacement != 0)
                                try self.emit(u32, dst_m.displacement);
                        }
                    },
                    .imm32 => |imm| {
                        try self.emit_rex_if_needed(.{ .b = need_rex(dst_m.base) });
                        try self.emit(u8, 0xC7);
                        const modrm: MODRM = .{
                            .mod = if (dst_m.displacement == 0) .indirect else .disp32,
                            .reg_opcode = 0,
                            .r_m = encode(dst_m.base),
                        };
                        try self.emit(u8, @bitCast(modrm));
                        // NOTE: ESP/R12-based addressing need a SIB byte.
                        if (encode(dst_m.base) == 0b100)
                            try self.emit(u8, @bitCast(SIB{ .scale = 0, .index = 0b100, .base = 0b100 }));
                        if (dst_m.displacement != 0)
                            try self.emit(u32, dst_m.displacement);
                        try self.emit(u32, imm);
                    },
                    else => return error.InvalidMovSource,
                }
            },
            .reg => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| {
                        try self.mov_reg_reg(dst_reg, src_reg);
                    },
                    .imm64 => |imm| {
                        // movabs <reg>,<imm64>
                        try self.emit_rex_if_needed(.{ .w = true, .b = need_rex(dst_reg) });
                        try self.emit(u8, 0xB8 + @as(u8, encode(dst_reg)));
                        try self.emit(u64, imm);
                    },
                    .imm32 => |imm| {
                        // mov    <reg>,<imm32>
                        try self.emit_rex_if_needed(.{ .b = need_rex(dst_reg) });
                        try self.emit(u8, 0xB8 + @as(u8, encode(dst_reg)));
                        try self.emit(u32, imm);
                    },
                    .mem => |src_m| {
                        if (src_m.index) |index| {
                            if (src_m.displacement != 0) {
                                return error.MovIndexWithDisplacementNotSupported;
                            }
                            try self.emit_rex_if_needed(.{
                                .w = src_m.size == 64,
                                .r = need_rex(dst_reg),
                                .x = need_rex(index),
                                .b = need_rex(src_m.base),
                            });
                            const opcode = 0x8B;
                            try self.emit(u8, opcode);
                            try self.emit(MODRM, .{
                                .mod = .disp8,
                                .reg_opcode = encode(dst.reg),
                                .r_m = 0b100, // FIXME: I don't know what I'm doing :D
                            });
                            const sib: SIB = .{
                                .scale = 0,
                                .index = encode(index),
                                .base = encode(src_m.base),
                            };
                            try self.emit(u8, @bitCast(sib));
                            try self.emit(u8, 0x00); // Zero displacement
                        } else {
                            try self.emit_rex_if_needed(.{
                                .w = src_m.size == 64,
                                .r = need_rex(dst_reg),
                                .b = need_rex(src_m.base),
                            });
                            const opcode = 0x8B;
                            if (src_m.size == 16) // Operand size prefix
                                try self.emit(u8, 0x66);
                            try self.emit(u8, opcode);
                            try self.emit(MODRM, .{
                                .mod = if (src_m.displacement == 0) .indirect else .disp32,
                                .reg_opcode = encode(dst.reg),
                                .r_m = encode(src_m.base),
                            });
                            // NOTE: ESP/R12-based addressing need a SIB byte.
                            if (encode(src_m.base) == 0b100)
                                try self.emit(u8, @bitCast(SIB{ .scale = 0, .index = 0b100, .base = 0b100 }));
                            if (src_m.displacement != 0)
                                try self.emit(u32, src_m.displacement);
                        }
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
                            try self.emit(u8, @bitCast(SIB{ .scale = 0, .index = 0b100, .base = 0b100 }));
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
            try self.emit(u8, @bitCast(SIB{ .scale = 0, .index = 0b100, .base = 0b100 }));

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
            try self.emit(u8, @bitCast(SIB{ .scale = 0, .index = 0b100, .base = 0b100 }));

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
    pub fn opcode_81_83(self: *@This(), comptime rax_dst_opcode_8: u8, comptime rax_dst_opcode: u8, comptime mr_opcode_8: u8, comptime mr_opcode: u8, comptime rm_opcode_8: u8, comptime rm_opcode: u8, comptime rm_imm__opcode: RegOpcode, dst: Operand, src: Operand) !void {
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
                        } else try reg_dest_imm_src(self, rm_imm__opcode, dst_reg, imm32);
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
                        try mem_dest_imm_src(self, rm_imm__opcode, dst_m, u32, imm);
                    },
                    else => return error.InvalidSource,
                }
            },
            else => return error.InvalidDestination,
        }
    }

    pub fn add(self: *@This(), dst: Operand, src: Operand) !void {
        return opcode_81_83(self, 0x04, 0x05, 0x00, 0x01, 0x02, 0x03, .Add, dst, src);
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
        return opcode_81_83(self, 0x2C, 0x2D, 0x28, 0x29, 0x2A, 0x2B, .Sub, dst, src);
    }
    pub fn xor_(self: *@This(), dst: Operand, src: Operand) !void {
        return opcode_81_83(self, 0x34, 0x35, 0x30, 0x31, 0x32, 0x33, .Xor, dst, src);
    }
    pub fn cmp(self: *@This(), lhs: Operand, rhs: Operand) !void {
        return opcode_81_83(self, 0x3C, 0x3D, 0x38, 0x39, 0x3A, 0x3B, .Cmp, lhs, rhs);
    }

    fn shift_instruction(self: *@This(), reg_opcode: ShiftRegOpcode, dst: Operand, amount: Operand) !void {
        switch (dst) {
            .reg => |dst_reg| {
                switch (amount) {
                    .imm8 => |imm8| {
                        try self.emit(u8, 0xC1);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = @intFromEnum(reg_opcode), .r_m = encode(dst_reg) });
                        try self.emit(u8, imm8);
                    },
                    .reg => |src_reg| {
                        if (src_reg != .rcx) {
                            return error.InvalidShiftAmount; // Only rcx is supported as a source for the shift amount in x86!
                        }
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
            .Equal => 0x84,
            .NotEqual => 0x85,
            .Carry => 0x82,
            .NotCarry => 0x83,
            .Greater => 0x8F,
            .GreaterEqual => 0x8D,
            .Above => 0x87,
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
        // call rax
        try self.emit(u8, 0xFF);
        try self.emit(u8, 0xD0);
    }

    pub fn ret(self: *@This()) !void {
        try self.emit(u8, 0xC3);
    }
};
