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
                .PushArg => |arg| {
                    try self.mov_reg_imm(ArgRegisters[arg.number], arg.value);
                },
                .FunctionCall => |function| {
                    try self.native_call(function);
                },
                .Mov => |m| {
                    try self.mov(m.dst, m.src);
                },
                .IncPC => {
                    // FIXME: Big Hack. This should be split into proper instructions at the JITBlock level (see sh4_jit). I'm just too lazy to properly abstract it right now.
                    // Increment PC of sh4.

                    const offset: u32 = @offsetOf(@import("../sh4.zig").SH4, "pc");

                    // mov    r9d,QWORD PTR [r12+@offsetOf(sh4.SH4, "pc")]
                    for ([_]u8{ 0x45, 0x8b, 0x8c, 0x24 }) |val| {
                        try self.block.emit(val);
                    }
                    try self.block.emit(@truncate(offset >> 0));
                    try self.block.emit(@truncate(offset >> 8));
                    try self.block.emit(@truncate(offset >> 16));
                    try self.block.emit(@truncate(offset >> 24));
                    // add    r9d,0x2
                    for ([_]u8{ 0x41, 0x83, 0xC1, 0x02 }) |val| {
                        try self.block.emit(val);
                    }
                    //mov    QWORD PTR [r12+@offsetOf(sh4.SH4, "pc")],r9d
                    for ([_]u8{ 0x45, 0x89, 0x8C, 0x24 }) |val| {
                        try self.block.emit(val);
                    }
                    try self.block.emit(@truncate(offset >> 0));
                    try self.block.emit(@truncate(offset >> 8));
                    try self.block.emit(@truncate(offset >> 16));
                    try self.block.emit(@truncate(offset >> 24));
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

        // Save user_date to SavedRegisters[0]. FIXME: Should probably no be there.
        try self.mov_reg_reg(SavedRegisters[0], ArgRegisters[0]);
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

    pub fn mov_reg_imm(self: *@This(), reg: Registers, value: u64) !void {
        try self.emit(u8, 0x48);
        try self.emit(u8, 0xB8 + @as(u8, @intCast(@intFromEnum(reg))));
        try self.emit(u64, value);
    }

    pub fn mov_reg_reg(self: *@This(), dst: Registers, src: Registers) !void {
        const rex: REX = .{ .w = true, .r = @intFromEnum(src) >= 8, .b = @intFromEnum(dst) >= 8 };
        try self.emit(u8, @bitCast(rex));
        try self.emit(u8, 0x89);
        try self.emit(u8, 0xC0 + (@as(u8, @intCast(@intFromEnum(src) & 0x7)) << 3) + @as(u8, @intCast(@intFromEnum(dst) & 0x7)));
    }

    pub fn mov(self: *@This(), dst: JIT.Operand, src: JIT.Operand) !void {
        switch (dst) {
            .mem => |dst_m| {
                const rex: REX = .{ .r = need_rex(src.reg), .b = need_rex(dst_m.reg) };
                const opcode = 0x89;
                const modrm: MODRM = .{ .mod = 0b10, .reg_opcode = encode(src.reg), .r_m = encode(dst_m.reg) };
                try self.block.emit(@bitCast(rex));
                try self.block.emit(opcode);
                try self.block.emit(@bitCast(modrm));
                // FIXME: ESP / R12 need a SIB byte. I don't know why, or how SIB works yet.
                if (encode(dst_m.reg) == 0b100) {
                    try self.block.emit(0x24);
                }
                try self.emit(u32, dst_m.offset);
            },
            .reg => |dst_reg| {
                switch (src) {
                    .reg => |reg| {
                        try self.mov_reg_reg(get_reg(dst_reg), get_reg(reg));
                    },
                    .imm => |imm| {
                        try self.mov_reg_imm(get_reg(dst_reg), imm);
                    },
                    .mem => |src_m| {
                        const rex: REX = .{ .r = need_rex(dst_reg), .b = need_rex(src_m.reg) };
                        const opcode = 0x8B;
                        const modrm: MODRM = .{ .mod = 0b10, .reg_opcode = encode(dst.reg), .r_m = encode(src_m.reg) };
                        try self.block.emit(@bitCast(rex));
                        try self.block.emit(opcode);
                        try self.block.emit(@bitCast(modrm));
                        // FIXME: ESP / R12 need a SIB byte. I don't know why, or how SIB works yet.
                        if (encode(src_m.reg) == 0b100) {
                            try self.block.emit(0x24);
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
