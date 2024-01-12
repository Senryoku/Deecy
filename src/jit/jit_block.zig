const std = @import("std");

pub const InstructionType = enum {
    Break, // For Debugging
    FunctionCall,
    Mov,
    Push,
    Pop,
    Add,
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
        reg: Register,
        offset: u32,
    },
};

pub const Instruction = union(InstructionType) {
    Break: u8, // FIXME: Could be void, but I don't know how to initialize a void value :')
    FunctionCall: *const anyopaque, // FIXME: Is there a better type for generic function pointers?
    Mov: struct { dst: Operand, src: Operand },
    Push: Operand,
    Pop: Operand,
    Add: struct { dst: Register, src: Operand },
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

    pub fn push(self: *@This(), op: Operand) !void {
        try self.instructions.append(.{ .Push = op });
    }

    pub fn pop(self: *@This(), op: Operand) !void {
        try self.instructions.append(.{ .Pop = op });
    }

    pub fn add(self: *@This(), dst: Register, src: Operand) !void {
        try self.instructions.append(.{ .Add = .{ .dst = dst, .src = src } });
    }
};
