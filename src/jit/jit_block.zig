const std = @import("std");

pub const InstructionType = enum {
    Break, // For Debugging
    FunctionCall,
    Mov,
    Movsx, // Mov with sign extension
    Push,
    Pop,
    Add,
    And,
    Cmp,
    Jmp,
};

pub const Register = enum {
    ReturnRegister,
    ArgRegister0,
    ArgRegister1,
    ArgRegister2,
    ArgRegister3,
    SavedRegister0,
    SavedRegister1,
    SavedRegister2,
    SavedRegister3,
};

pub const Condition = enum {
    Always,
    Equal,
    NotEqual,
};

const OperandType = enum {
    reg,
    imm32,
    imm,
    mem,
};

pub const Operand = union(OperandType) {
    reg: Register,
    imm32: u32,
    imm: u64,
    mem: struct {
        base: Register, // NOTE: This could be made optional as well, to allow for absolute addressing. However this is only possible on (r)ax on x86_64.
        index: ?Register = null,
        displacement: u32 = 0,
        size: u8,
    },
};

pub const Instruction = union(InstructionType) {
    Break: u8, // FIXME: Could be void, but I don't know how to initialize a void value :')
    FunctionCall: *const anyopaque, // FIXME: Is there a better type for generic function pointers?
    Mov: struct { dst: Operand, src: Operand },
    Movsx: struct { dst: Operand, src: Operand },
    Push: Operand,
    Pop: Operand,
    Add: struct { dst: Register, src: Operand },
    And: struct { dst: Register, src: Operand },
    Cmp: struct { lhs: Register, rhs: Operand },
    Jmp: struct { condition: Condition, dst: struct { rel: u32 } },
};

const PatchableJump = struct {
    source_index: usize,
    block: *JITBlock,

    pub fn patch(self: *@This()) void {
        switch (self.block.instructions.items[self.source_index]) {
            .Jmp => |*jmp| jmp.dst.rel = @intCast(self.block.instructions.items.len - self.source_index),
            else => unreachable,
        }
    }
};

pub const JITBlock = struct {
    instructions: std.ArrayList(Instruction),

    pub fn init(allocator: std.mem.Allocator) JITBlock {
        return .{
            .instructions = std.ArrayList(Instruction).init(allocator),
        };
    }

    pub fn deinit(self: *@This()) void {
        self.instructions.deinit();
    }

    pub fn append(self: *@This(), instr: Instruction) !void {
        try self.instructions.append(instr);
    }

    pub fn call(self: *@This(), func: *const anyopaque) !void {
        try self.instructions.append(.{ .FunctionCall = func });
    }

    pub fn mov(self: *@This(), dst: Operand, src: Operand) !void {
        try self.instructions.append(.{ .Mov = .{ .dst = dst, .src = src } });
    }

    pub fn movsx(self: *@This(), dst: Operand, src: Operand) !void {
        try self.instructions.append(.{ .Movsx = .{ .dst = dst, .src = src } });
    }

    pub fn push(self: *@This(), op: Operand) !void {
        try self.instructions.append(.{ .Push = op });
    }

    pub fn pop(self: *@This(), op: Operand) !void {
        try self.instructions.append(.{ .Pop = op });
    }

    pub fn add(self: *@This(), dst: Register, src: Operand) !void {
        try self.instructions.append(.{ .Add = .{ .dst = dst, .src = src } });
    }

    // Forward Jump
    pub fn jmp(self: *@This(), condition: Condition) !PatchableJump {
        try self.instructions.append(.{ .Jmp = .{ .condition = condition, .dst = .{ .rel = 0x00C0FFEE } } });

        return .{
            .source_index = self.instructions.items.len - 1,
            .block = self,
        };
    }
};
