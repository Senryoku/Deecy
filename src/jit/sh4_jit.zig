const std = @import("std");

const sh4 = @import("../sh4.zig");
const JITBlock = @import("jit_block.zig").JITBlock;
const BasicBlock = @import("basic_block.zig").BasicBlock;

const sh4_instructions = @import("../sh4_instructions.zig");

const BlockCache = struct {
    buffer: []u8,
    cursor: u32 = 0,
    blocks: std.AutoHashMap(u32, BasicBlock),

    pub fn init(allocator: std.mem.Allocator) @This() {
        return .{
            .blocks = std.AutoHashMap(u32, BasicBlock).init(allocator),
        };
    }

    pub fn deinit(self: *@This()) void {
        self.blocks.deinit();
    }

    pub fn get(self: *@This(), address: u32) ?BasicBlock {
        return self.blocks.get(address);
    }

    pub fn compile(self: *@This(), address: u32, instructions: [*]u16) !BasicBlock {
        var bb = BasicBlock.init(self.buffer[self.cursor..], address);

        var curr_pc = address;
        for (instructions) |instr| {
            const next_pc = sh4.JumpTable[instr].emit_fn(&bb, instr);
            _ = next_pc;
            bb.cycles += sh4.JumpTable[instr].issue_cycles;
            curr_pc += 2;
        }

        self.blocks.put(address, bb);
    }
};

pub fn interpreter_fallback(block: *JITBlock, instr: sh4.Instr) bool {
    block.call(sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr.value]].fn_);
    return false;
}
