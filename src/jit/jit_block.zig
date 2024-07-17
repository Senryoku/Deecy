const std = @import("std");

pub const Architecture = @import("x86_64.zig");
pub const Register = Architecture.Register;
pub const FPRegister = Architecture.FPRegister;
pub const Instruction = Architecture.Instruction;
pub const Operand = Architecture.Operand;
pub const Condition = Architecture.Condition;

const BasicBlock = @import("basic_block.zig");

pub const PatchableJump = struct {
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

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) JITBlock {
        return .{
            .instructions = std.ArrayList(Instruction).init(allocator),
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        self.instructions.deinit();
    }

    pub fn append(self: *@This(), instr: Instruction) !void {
        try self.instructions.append(instr);
    }

    // Insert a breakpoint for debugging.
    pub fn bp(self: *@This()) !void {
        try self.instructions.append(.Break);
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

    pub fn add(self: *@This(), dst: Operand, src: Operand) !void {
        try self.instructions.append(.{ .Add = .{ .dst = dst, .src = src } });
    }

    pub fn sub(self: *@This(), dst: Operand, src: Operand) !void {
        try self.instructions.append(.{ .Sub = .{ .dst = dst, .src = src } });
    }

    pub fn shl(self: *@This(), dst: Operand, amount: Operand) !void {
        // Combine shift instructions if possible
        if (self.instructions.items.len > 0 and amount == .imm8) {
            const prev = &self.instructions.items[self.instructions.items.len - 1];
            if (prev.* == .Shl and prev.Shl.dst.equal(dst) and prev.Shl.amount == .imm8) {
                prev.Shl.amount.imm8 += amount.imm8;
                return;
            }
        }
        try self.instructions.append(.{ .Shl = .{ .dst = dst, .amount = amount } });
    }

    pub fn shr(self: *@This(), dst: Operand, amount: Operand) !void {
        // Combine shift instructions if possible
        if (self.instructions.items.len > 0 and amount == .imm8) {
            const prev = &self.instructions.items[self.instructions.items.len - 1];
            if (prev.* == .Shr and prev.Shr.dst.equal(dst) and prev.Shr.amount == .imm8) {
                prev.Shr.amount.imm8 += amount.imm8;
                return;
            }
        }
        try self.instructions.append(.{ .Shr = .{ .dst = dst, .amount = amount } });
    }

    // Forward Jump
    pub fn jmp(self: *@This(), condition: Condition) !PatchableJump {
        try self.instructions.append(.{ .Jmp = .{ .condition = condition, .dst = .{ .rel = 0x00C0FFEE } } });

        return .{
            .source_index = self.instructions.items.len - 1,
            .block = self,
        };
    }

    // NOTE: offset could also be a register
    pub fn bit_test(self: *@This(), reg: Register, offset: u8) !void {
        try self.instructions.append(.{ .BitTest = .{ .reg = reg, .offset = .{ .imm8 = offset } } });
    }

    pub fn emit(self: *@This(), buffer: []u8) !BasicBlock {
        var emitter = try Architecture.Emitter.init(self._allocator, buffer);
        defer emitter.deinit();

        try emitter.emit_block_prologue();
        try emitter.emit_instructions(self.instructions.items);
        try emitter.emit_block_epilogue();

        emitter.block.buffer = emitter.block.buffer[0..emitter.block_size]; // Update slice size.
        return emitter.block;
    }
};
