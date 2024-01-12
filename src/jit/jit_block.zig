const std = @import("std");

pub const InstructionType = enum {
    FunctionCall,
    Mov,
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
    imm,
    mem,
};

pub const Operand = union(OperandType) {
    reg: Register,
    imm: u64,
    mem: struct {
        reg: Register,
        offset: u32,
    },
};

pub const Instruction = union(InstructionType) {
    FunctionCall: *const anyopaque,
    Mov: struct { dst: Operand, src: Operand },
    Add: struct { dst: Register, src: Operand },
};

pub const JITBlock = struct {
    instructions: std.ArrayList(Instruction),

    pub fn init(allocator: std.mem.Allocator) JITBlock {
        return .{
            .instructions = std.ArrayList(Instruction).init(allocator),
        };
    }

    pub fn deinit(self: *JITBlock) void {
        self.instructions.deinit();
    }

    pub fn call(self: *@This(), func: *const fn () void) void {
        _ = self;
        _ = func;
    }

    pub fn call_1(self: *@This(), func: *const fn (*anyopaque) void) void {
        _ = self;
        _ = func;
    }

    pub fn call_2(self: *@This(), comptime arg_0: type, comptime arg_1: type, func: *const fn (arg_0, arg_1) void) !void {
        try self.instructions.append(.{ .FunctionCall = func });
    }

    pub fn mov(self: *@This(), dst: Operand, src: Operand) !void {
        try self.instructions.append(.{ .Mov = .{ .dst = dst, .src = src } });
    }

    pub fn add(self: *@This(), dst: Register, src: Operand) !void {
        try self.instructions.append(.{ .Add = .{ .dst = dst, .src = src } });
    }

    pub fn inc_pc(self: *@This()) !void {
        try self.instructions.append(.{ .IncPC = {} });
    }
};
