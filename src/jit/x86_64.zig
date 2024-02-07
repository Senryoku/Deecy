const std = @import("std");
const builtin = @import("builtin");

const x86_64_emitter_log = std.log.scoped(.x86_64_emitter);

const BasicBlock = @import("basic_block.zig").BasicBlock;
const JIT = @import("jit_block.zig");
const JITBlock = @import("jit_block.zig").JITBlock;

const Register = enum(u4) {
    RAX = 0,
    RCX = 1,
    RDX = 2,
    RBX = 3,
    RSP = 4,
    RBP = 5,
    RSI = 6,
    RDI = 7,
    R8 = 8,
    R9 = 9,
    R10 = 10,
    R11 = 11,
    R12 = 12,
    R13 = 13,
    R14 = 14,
    R15 = 15,
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

// Tried using builtin.abi, but it returns .gnu on Windows.

const ReturnRegister = Register.RAX;
const ScratchRegisters = [_]Register{ .R10, .R11 };
// ArgRegisters are also used as scratch registers, but have a special meaning for function calls.
const ArgRegisters = if (builtin.os.tag == .windows) [_]Register{
    .RCX,
    .RDX,
    .R8,
    .R9,
} else if (builtin.os.tag == .linux) [_]Register{
    .RDI,
    .RSI,
    .RDX,
    .RCX,
    .R8,
    .R9,
} else @compileError("Unsupported ABI");
const SavedRegisters = if (builtin.os.tag == .windows) [_]Register{
    .R12,
    .R13,
    .R14,
    .R15,
    .RBX,
    .RSI,
    .RDI,
    .RBP,
    .RSP,
} else if (builtin.os.tag == .linux) [_]Register{
    .R12,
    .R13,
    .R14,
    .R15,
    .RBX,
    .RBP,
    .RSP,
} else @compileError("Unsupported ABI");

const PatchableJump = struct {
    source: u32,
    address_to_patch: u32,
};

// RegOpcodes (ModRM) for 0x81: OP r/m32, imm32 - 0x83: OP r/m32, imm8 (sign extended)
const RegOpcode = enum(u3) { Add = 0, Adc = 2, Sub = 5, Sbb = 3, And = 4, Or = 1, Xor = 6, Cmp = 7 };

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

    pub fn get_reg(reg: JIT.Register) Register {
        return switch (reg) {
            .ReturnRegister => ReturnRegister,
            .ArgRegister0 => ArgRegisters[0],
            .ArgRegister1 => ArgRegisters[1],
            .ArgRegister2 => ArgRegisters[2],
            .ArgRegister3 => ArgRegisters[3],
            .SavedRegister0 => SavedRegisters[0],
            .SavedRegister1 => SavedRegisters[1],
            .SavedRegister2 => SavedRegisters[2],
            .SavedRegister3 => SavedRegisters[3],
            .SavedRegister4 => SavedRegisters[4],
            .SavedRegister5 => SavedRegisters[5],
            .SavedRegister6 => SavedRegisters[6],
        };
    }

    pub fn emit_block(self: *@This(), jb: *const JITBlock) !void {
        self.emit_block_prologue();
        for (0..jb.instructions.items.len) |i| {
            if (self.jumps_to_patch.get(@intCast(i))) |jumps| {
                for (jumps.items) |jump| {
                    const rel: u32 = @intCast(self.block_size - jump.source);
                    @memcpy(@as([*]u8, @ptrCast(&self.block.buffer[jump.address_to_patch]))[0..4], @as([*]const u8, @ptrCast(&rel)));
                }
                jumps.deinit();
                _ = self.jumps_to_patch.remove(@intCast(i));
            }

            x86_64_emitter_log.debug("[{d: >4}] {any}", .{ i, jb.instructions.items[i] });

            switch (jb.instructions.items[i]) {
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
                    try self.jmp(j.condition, @intCast(i + j.dst.rel));
                },
                .BitTest => |b| {
                    try self.bit_test(b.reg, b.offset);
                },
                //else => @panic("Unhandled JIT instruction"),
            }
        }
        if (self.jumps_to_patch.count() > 0) {
            std.debug.print("Jumps left to patch: {}\n", .{self.jumps_to_patch.count()});
            @panic("Error: Unpatched jumps!");
        }
        self.emit_block_epilogue();
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

    pub fn emit_block_prologue(self: *@This()) void {
        // FIXME: Don't hardcode this? Please?

        // push rbp
        try self.emit(u8, 0x55);
        // mov    rbp,rsp
        try self.emit(u8, 0x48);
        try self.emit(u8, 0x89);
        try self.emit(u8, 0xE5);
    }

    pub fn emit_block_epilogue(self: *@This()) void {
        // pop    rbp
        try self.emit(u8, 0x5D);
        // ret
        try self.ret();
    }

    fn encode_reg(reg: Register) u3 {
        return @truncate(@intFromEnum(reg));
    }

    fn encode(reg: JIT.Register) u3 {
        return encode_reg(get_reg(reg));
    }

    fn need_rex(reg: JIT.Register) bool {
        return @intFromEnum(get_reg(reg)) >= 8;
    }

    fn encode_opcode(opcode: u8, reg: JIT.Register) u8 {
        return opcode + encode(reg);
    }

    fn emit_rex_if_needed(self: *@This(), rex: REX) !void {
        if (@as(u8, @bitCast(rex)) != @as(u8, @bitCast(REX{})))
            try self.emit(u8, @bitCast(rex));
    }

    pub fn mov_reg_reg(self: *@This(), dst: JIT.Register, src: JIT.Register) !void {
        try self.emit_rex_if_needed(.{ .w = true, .r = need_rex(src), .b = need_rex(dst) });
        try self.emit(u8, 0x89);
        const modrm: MODRM = .{ .mod = .reg, .reg_opcode = encode(src), .r_m = encode(dst) };
        try self.emit(u8, @bitCast(modrm));
    }

    pub fn mov(self: *@This(), dst: JIT.Operand, src: JIT.Operand) !void {
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
                    .imm => |imm| {
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

    pub fn movsx(self: *@This(), dst: JIT.Operand, src: JIT.Operand) !void {
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
    fn mem_dest_imm_src(self: *@This(), reg_opcode: RegOpcode, dst_m: JIT.MemOperand, comptime ImmType: type, imm: ImmType) !void {
        std.debug.assert(dst_m.size == 32);

        // NOTE: I'm not entirely sure how emitting a 32-bit displacement works here.
        try self.emit_rex_if_needed(.{ .b = need_rex(dst_m.base) });

        // 0x83: ADD r/m32, imm8 - Sign-extended imm8 - Shorter encoding
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

    fn mem_dest_reg_src(self: *@This(), opcode: u8, dst_m: JIT.MemOperand, reg: JIT.Register) !void {
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

    fn reg_dest_imm_src(self: *@This(), reg_opcode: RegOpcode, dst_reg: JIT.Register, imm32: u32) !void {
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

    pub fn add(self: *@This(), dst: JIT.Operand, src: JIT.Operand) !void {
        // FIXME: Handle different sizes. We expect a u32 immediate.
        switch (dst) {
            .reg => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| {
                        try self.emit_rex_if_needed(.{ .r = need_rex(src_reg), .b = need_rex(dst_reg) });
                        try self.emit(u8, 0x01);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(src_reg), .r_m = encode(dst_reg) });
                    },
                    .imm32 => |imm32| {
                        if (get_reg(dst_reg) == .RAX and imm32 >= 0x80) {
                            // ADD EAX, imm32
                            try self.emit(u8, 0x05);
                            try self.emit(u32, imm32);
                        } else try reg_dest_imm_src(self, .Add, dst_reg, imm32);
                    },
                    else => return error.InvalidAddSource,
                }
            },
            .mem => |dst_m| {
                switch (src) {
                    .reg => |src_reg| {
                        try mem_dest_reg_src(self, 0x01, dst_m, src_reg);
                    },
                    .imm32 => |imm| {
                        try mem_dest_imm_src(self, .Add, dst_m, u32, imm);
                    },
                    else => return error.InvalidAddSource,
                }
            },
            else => return error.InvalidAddDestination,
        }
    }

    pub fn sub(self: *@This(), dst: JIT.Register, src: JIT.Operand) !void {
        // FIXME: Handle different sizes. We expect a u32 immediate.
        switch (src) {
            .reg => |src_reg| {
                try self.emit_rex_if_needed(.{ .r = need_rex(dst), .b = need_rex(src_reg) });
                try self.emit(u8, 0x29);
                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(src_reg), .r_m = encode(dst) });
            },
            .imm32 => |imm32| {
                if (get_reg(dst) == .RAX and imm32 >= 0x80) {
                    try self.emit(u8, 0x2D);
                    try self.emit(u32, imm32);
                } else try reg_dest_imm_src(self, .Sub, dst, imm32);
            },
            else => return error.InvalidSubSource,
        }
    }

    pub fn and_(self: *@This(), dst: JIT.Operand, src: JIT.Operand) !void {
        switch (dst) {
            .reg => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| {
                        try self.emit_rex_if_needed(.{ .r = need_rex(src_reg), .b = need_rex(dst_reg) });
                        try self.emit(u8, 0x21);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(src_reg), .r_m = encode(dst_reg) });
                    },
                    .imm32 => |imm| {
                        if (get_reg(dst_reg) == .RAX and imm >= 0x80) {
                            try self.emit(u8, 0x25);
                            try self.emit(u32, imm);
                        } else try reg_dest_imm_src(self, .And, dst_reg, imm);
                    },
                    else => return error.InvalidAndSource,
                }
            },
            .mem => |dst_m| {
                switch (src) {
                    .imm32 => |imm| {
                        try mem_dest_imm_src(self, .And, dst_m, u32, imm);
                    },
                    else => return error.InvalidAndSource,
                }
            },
            else => return error.InvalidAndDestination,
        }
    }

    pub fn or_(self: *@This(), dst: JIT.Operand, src: JIT.Operand) !void {
        switch (dst) {
            .reg => |dst_reg| {
                switch (src) {
                    .reg => |src_reg| {
                        try self.emit_rex_if_needed(.{ .w = false, .r = need_rex(src_reg), .b = need_rex(dst_reg) });
                        try self.emit(u8, 0x09);
                        try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(src_reg), .r_m = encode(dst_reg) });
                    },
                    .imm32 => |imm| {
                        if (get_reg(dst_reg) == .RAX and imm >= 0x80) {
                            try self.emit(u8, 0x0D);
                            try self.emit(u32, imm);
                        } else try reg_dest_imm_src(self, .Or, dst_reg, imm);
                    },
                    else => return error.InvalidAndSource,
                }
            },
            .mem => |dst_m| {
                switch (src) {
                    .imm32 => |imm| {
                        try mem_dest_imm_src(self, .Or, dst_m, u32, imm);
                    },
                    else => return error.InvalidAndSource,
                }
            },
            else => return error.InvalidAndDestination,
        }
    }

    pub fn cmp(self: *@This(), lhs: JIT.Register, rhs: JIT.Operand) !void {
        switch (rhs) {
            .reg => |rhs_reg| {
                try self.emit_rex_if_needed(.{ .w = false, .r = need_rex(rhs_reg), .b = need_rex(lhs) });
                try self.emit(u8, 0x39);
                try self.emit(MODRM, .{ .mod = .reg, .reg_opcode = encode(rhs_reg), .r_m = encode(lhs) });
            },
            .imm32 => |imm| {
                if (get_reg(lhs) == .RAX and imm >= 0x80) {
                    try self.emit(u8, 0x3D);
                    try self.emit(u32, imm);
                } else try reg_dest_imm_src(self, .Cmp, lhs, imm);
            },
            else => return error.UnsupportedCmpRHS,
        }
    }

    pub fn bit_test(self: *@This(), reg: JIT.Register, offset: JIT.Operand) !void {
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

        switch (condition) {
            .Always => {
                try self.emit(u8, 0xE9);
                address = self.block_size;
                try self.emit(u32, 0x00C0FFEE);
            },
            .Equal => {
                try self.emit(u8, 0x0F);
                try self.emit(u8, 0x84);
                address = self.block_size;
                try self.emit(u32, 0x00C0FFEE);
            },
            .NotEqual => {
                try self.emit(u8, 0x0F);
                try self.emit(u8, 0x85);
                address = self.block_size;
                try self.emit(u32, 0x00C0FFEE);
            },
            .Carry => {
                try self.emit(u8, 0x0F);
                try self.emit(u8, 0x82);
                address = self.block_size;
                try self.emit(u32, 0x00C0FFEE);
            },
            .NotCarry => {
                try self.emit(u8, 0x0F);
                try self.emit(u8, 0x83);
                address = self.block_size;
                try self.emit(u32, 0x00C0FFEE);
            },
            .Greater => {
                try self.emit(u8, 0x0F);
                try self.emit(u8, 0x8F);
                address = self.block_size;
                try self.emit(u32, 0x00C0FFEE);
            },
            .GreaterEqual => {
                try self.emit(u8, 0x0F);
                try self.emit(u8, 0x8D);
                address = self.block_size;
                try self.emit(u32, 0x00C0FFEE);
            },
            .Above => {
                try self.emit(u8, 0x0F);
                try self.emit(u8, 0x87);
                address = self.block_size;
                try self.emit(u32, 0x00C0FFEE);
            },
        }

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
