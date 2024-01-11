const std = @import("std");

pub const InstructionType = enum {
    FunctionCall,
    PushArg,
    Mov,
    IncPC, // FIXME: Hack. Remove once we have the proper abstractions.
};

pub const Register = enum {
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
};

const RegisterOrImmediate = union(OperandType) {
    reg: Register,
    imm: u64,
};

pub const Instruction = union(InstructionType) {
    FunctionCall: *const anyopaque,
    PushArg: struct { number: u8, value: u64 },
    Mov: struct { dst: Register, src: RegisterOrImmediate },
    IncPC: void, // FIXME: Remove.
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

    pub fn push_arg(self: *@This(), number: u8, value: u64) !void {
        try self.instructions.append(.{ .PushArg = .{ .number = number, .value = value } });
    }

    pub fn mov(self: *@This(), dst: Register, src: RegisterOrImmediate) !void {
        try self.instructions.append(.{ .Mov = .{ .dst = dst, .src = src } });
    }

    pub fn inc_pc(self: *@This()) !void {
        try self.instructions.append(.{ .IncPC = {} });
    }
};
