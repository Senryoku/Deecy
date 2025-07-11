const std = @import("std");
const builtin = @import("builtin");

pub const Architecture = @import("x86_64.zig");
pub const Register = Architecture.Register;
pub const FPRegister = Architecture.FPRegister;
pub const Instruction = Architecture.Instruction;
pub const Operand = Architecture.Operand;
pub const Condition = Architecture.Condition;

pub const PatchableJump = struct {
    source_index: usize,
    block: *IRBlock,

    pub fn patch(self: *const @This()) void {
        switch (self.block.instructions.items[self.source_index]) {
            .Jmp => |*jmp| jmp.dst.rel = @intCast(self.block.instructions.items.len - self.source_index),
            else => @panic("Jump source is not a jump instruction."),
        }
    }
};

/// Intermediate Representation, actually way too close to x86 to be very useful.
pub const IRBlock = struct {
    instructions: std.ArrayList(Instruction),

    _emitter: Architecture.Emitter,

    pub fn init(allocator: std.mem.Allocator) !IRBlock {
        return .{
            .instructions = .init(allocator),
            ._emitter = try .init(allocator),
        };
    }

    pub inline fn clearRetainingCapacity(self: *@This()) void {
        self.instructions.clearRetainingCapacity();
    }

    pub fn deinit(self: *@This()) void {
        self._emitter.deinit();
        self.instructions.deinit();
    }

    pub fn append(self: *@This(), instr: Instruction) !void {
        try self.instructions.append(instr);
    }

    // Insert a breakpoint for debugging.
    pub fn bp(self: *@This()) !void {
        try self.instructions.append(.Break);
    }

    pub fn call(self: *@This(), func: ?*const anyopaque) !void {
        try self.instructions.append(.{ .FunctionCall = func });
    }

    pub fn mov(self: *@This(), dst: Operand, src: Operand) !void {
        try self.instructions.append(.{ .Mov = .{ .dst = dst, .src = src } });
    }

    pub fn cmov(self: *@This(), condition: Condition, dst: Operand, src: Operand) !void {
        try self.instructions.append(.{ .Cmov = .{ .condition = condition, .dst = dst, .src = src } });
    }

    pub fn cmp(self: *@This(), lhs: Operand, rhs: Operand) !void {
        try self.instructions.append(.{ .Cmp = .{ .lhs = lhs, .rhs = rhs } });
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
            if (prev.* == .Shl and std.meta.eql(prev.Shl.dst, dst) and prev.Shl.amount == .imm8) {
                prev.Shl.amount.imm8 += amount.imm8;
                return;
            }
        }
        try self.instructions.append(.{ .Shl = .{ .dst = dst, .amount = amount } });
    }

    pub fn shr(self: *@This(), dst: Operand, amount: anytype) !void {
        switch (@TypeOf(amount)) {
            comptime_int, u8 => try self._shr(dst, .{ .imm8 = amount }),
            Register => try self._shr(dst, .{ .reg = amount }),
            Operand => try self._shr(dst, amount),
            else => @compileError("Invalid Type used as SHR amount"),
        }
    }

    fn _shr(self: *@This(), dst: Operand, amount: Operand) !void {
        // Combine shift instructions if possible
        if (self.instructions.items.len > 0 and amount == .imm8) {
            const prev = &self.instructions.items[self.instructions.items.len - 1];
            if (prev.* == .Shr and std.meta.eql(prev.Shr.dst, dst) and prev.Shr.amount == .imm8) {
                prev.Shr.amount.imm8 += amount.imm8;
                return;
            }
        }
        try self.instructions.append(.{ .Shr = .{ .dst = dst, .amount = amount } });
    }

    pub fn sar(self: *@This(), dst: Operand, amount: anytype) !void {
        switch (@TypeOf(amount)) {
            comptime_int, u8 => try self.instructions.append(.{ .Sar = .{ .dst = dst, .amount = .{ .imm8 = amount } } }),
            Register => try self.instructions.append(.{ .Sar = .{ .dst = dst, .amount = .{ .reg = amount } } }),
            Operand => try self.instructions.append(.{ .Sar = .{ .dst = dst, .amount = amount } }),
            else => @compileError("Invalid Type used as SAR amount"),
        }
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

    pub fn emit(self: *@This(), buffer: []u8) !u32 {
        self._emitter.set_buffer(buffer);

        try self._emitter.emit_block_prologue();
        try self._emitter.emit_instructions(self.instructions.items);
        try self._emitter.emit_block_epilogue();

        return self._emitter.block_size;
    }

    pub fn emit_naked(self: *@This(), buffer: []u8) !u32 {
        self._emitter.set_buffer(buffer);
        try self._emitter.emit_instructions(self.instructions.items);
        return self._emitter.block_size;
    }
};
