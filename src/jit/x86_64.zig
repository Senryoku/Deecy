const std = @import("std");
const builtin = @import("builtin");

const BasicBlock = @import("basic_block.zig").BasicBlock;
const JIT = @import("jit_block.zig");
const JITBlock = @import("jit_block.zig").JITBlock;

const Registers = enum(u4) {
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

const MODRM = packed struct(u8) {
    r_m: u3, // The r/m field can specify a register as an operand or it can be combined with the mod field to encode an addressing mode. Sometimes, certain combinations of the mod field and the r/m field are used to express opcode information for some instructions.
    reg_opcode: u3, // The reg/opcode field specifies either a register number or three more bits of opcode information. The purpose of the reg/opcode field is specified in the primary opcode.
    mod: u2, // The mod field combines with the r/m field to form 32 possible values: eight registers and 24 addressing modes
};

const SIB = packed struct(u8) {
    base: u3,
    index: u3,
    scale: u2,
};

// Tried using builtin.abi, but it returns .gnu on Windows.

const ReturnRegister = if (builtin.os.tag == .windows) Registers.RAX else @compileError("Unsupported ABI");
const ArgRegisters = if (builtin.os.tag == .windows) [_]Registers{
    Registers.RCX,
    Registers.RDX,
    Registers.R8,
    Registers.R9,
} else if (builtin.os.tag == .linux) [_]Registers{
    Registers.RDI,
    Registers.RSI,
    Registers.RDX,
    Registers.RCX,
    Registers.R8,
    Registers.R9,
} else @compileError("Unsupported ABI");
const SavedRegisters = if (builtin.os.tag == .windows) [_]Registers{
    Registers.R12,
    Registers.R13,
    Registers.R14,
    Registers.R15,
    Registers.RDI,
    Registers.RSI,
    Registers.RBX,
    Registers.RBP,
    Registers.RSP,
} else if (builtin.os.tag == .linux) [_]Registers{
    Registers.R12,
    Registers.R13,
    Registers.R14,
    Registers.R15,
    Registers.RBX,
    Registers.RBP,
    Registers.RSP,
} else @compileError("Unsupported ABI");

pub const Emitter = struct {
    block: BasicBlock,

    pub fn init(block_buffer: []u8) @This() {
        return .{
            .block = BasicBlock.init(block_buffer),
        };
    }

    pub fn deinit(self: *@This()) void {
        self.block.deinit();
    }

    pub fn get_reg(reg: JIT.Register) Registers {
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
        };
    }

    pub fn emit_block(self: *@This(), jb: *const JITBlock) !void {
        self.emit_block_prologue();
        for (0..jb.instructions.items.len) |i| {
            switch (jb.instructions.items[i]) {
                .FunctionCall => |function| {
                    try self.native_call(function);
                },
                .Mov => |m| {
                    try self.mov(m.dst, m.src);
                },
                .Add => |a| {
                    try self.add(a.dst, a.src);
                },
                //else => @panic("Unhandled JIT instruction"),
            }
        }
        self.emit_block_epilogue();
    }

    pub fn emit(self: *@This(), comptime T: type, value: T) !void {
        for (0..@sizeOf(T)) |i| {
            try self.block.emit(@truncate((value >> @intCast(8 * i)) & 0xFF));
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

        // Save user_data to SavedRegisters[0]. FIXME: Should probably no be there.
        try self.mov_reg_reg(.SavedRegister0, .ArgRegister0);
    }

    pub fn emit_block_epilogue(self: *@This()) void {
        // pop    rbp
        try self.emit(u8, 0x5D);
        // ret
        try self.ret();
    }

    fn encode_reg(reg: Registers) u3 {
        return @truncate(@intFromEnum(reg));
    }

    fn encode(reg: JIT.Register) u3 {
        return encode_reg(get_reg(reg));
    }

    fn need_rex(reg: JIT.Register) bool {
        return @intFromEnum(get_reg(reg)) >= 8;
    }

    fn emit_rex_if_needed(self: *@This(), rex: REX) !void {
        if (@as(u8, @bitCast(rex)) != @as(u8, @bitCast(REX{})))
            try self.emit(u8, @bitCast(rex));
    }

    pub fn mov_reg_imm(self: *@This(), dst: JIT.Register, value: u64) !void {
        // movabs <reg>,<imm64>
        try self.emit_rex_if_needed(.{ .w = true, .b = need_rex(dst) });
        try self.emit(u8, 0xB8 + @as(u8, encode(dst)));
        try self.emit(u64, value);
    }

    pub fn mov_reg_reg(self: *@This(), dst: JIT.Register, src: JIT.Register) !void {
        try self.emit_rex_if_needed(.{ .w = true, .r = need_rex(src), .b = need_rex(dst) });
        try self.emit(u8, 0x89);
        const modrm: MODRM = .{ .mod = 0b11, .reg_opcode = encode(src), .r_m = encode(dst) };
        try self.block.emit(@bitCast(modrm));
    }

    pub fn mov(self: *@This(), dst: JIT.Operand, src: JIT.Operand) !void {
        switch (dst) {
            .mem => |dst_m| {
                try self.emit_rex_if_needed(.{ .r = need_rex(src.reg), .b = need_rex(dst_m.reg) });
                const opcode = 0x89;
                const modrm: MODRM = .{ .mod = 0b10, .reg_opcode = encode(src.reg), .r_m = encode(dst_m.reg) };
                try self.block.emit(opcode);
                try self.block.emit(@bitCast(modrm));
                // NOTE: ESP/R12-based addressing need a SIB byte.
                if (encode(dst_m.reg) == 0b100) {
                    try self.block.emit(@bitCast(SIB{ .scale = 0, .index = 0b100, .base = 0b100 }));
                }
                try self.emit(u32, dst_m.offset);
            },
            .reg => |dst_reg| {
                switch (src) {
                    .reg => |reg| {
                        try self.mov_reg_reg(dst_reg, reg);
                    },
                    .imm => |imm| {
                        try self.mov_reg_imm(dst_reg, imm);
                    },
                    .mem => |src_m| {
                        try self.emit_rex_if_needed(.{ .r = need_rex(dst_reg), .b = need_rex(src_m.reg) });
                        const opcode = 0x8B;
                        const modrm: MODRM = .{ .mod = 0b10, .reg_opcode = encode(dst.reg), .r_m = encode(src_m.reg) };
                        try self.block.emit(opcode);
                        try self.block.emit(@bitCast(modrm));
                        // NOTE: ESP/R12-based addressing need a SIB byte.
                        if (encode(src_m.reg) == 0b100) {
                            try self.block.emit(@bitCast(SIB{ .scale = 0, .index = 0b100, .base = 0b100 }));
                        }
                        try self.emit(u32, src_m.offset);
                    },
                }
            },
            .imm => {
                return error.InvalidDestination;
            },
        }
    }

    pub fn add(self: *@This(), dst: JIT.Register, src: JIT.Operand) !void {
        // FIXME: Handle different sizes. We expect a u32 immediate.
        switch (src) {
            .reg => |src_reg| {
                try self.emit_rex_if_needed(.{ .r = need_rex(dst), .b = need_rex(src_reg) });
                try self.emit(u8, 0x81);
                const modrm: MODRM = .{ .mod = 0b11, .reg_opcode = encode(dst), .r_m = encode(src_reg) };
                try self.emit(u8, @bitCast(modrm));
            },
            .imm => |imm| {
                try self.emit_rex_if_needed(.{ .b = need_rex(dst) });
                try self.emit(u8, 0x81); // ADD r/m32, imm32
                const modrm: MODRM = .{ .mod = 0b11, .reg_opcode = 0, .r_m = encode(dst) };
                try self.emit(u8, @bitCast(modrm));
                //try self.emit(@TypeOf(imm), imm); // FIXME
                try self.emit(u32, @truncate(imm));
            },
            else => return error.InvalidSource,
        }
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
