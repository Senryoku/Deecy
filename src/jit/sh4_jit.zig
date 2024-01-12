const std = @import("std");

const sh4 = @import("../sh4.zig");
const JITBlock = @import("jit_block.zig").JITBlock;
const Emitter = @import("x86_64.zig").Emitter;
const BasicBlock = @import("basic_block.zig").BasicBlock;

const sh4_instructions = @import("../sh4_instructions.zig");

const sh4_jit_log = std.log.scoped(.sh4_jit);

const BlockBufferSize = 1024 * 4096;

const BlockCache = struct {
    buffer: []align(std.mem.page_size) u8,
    cursor: u32 = 0,
    blocks: std.AutoHashMap(u32, BasicBlock),

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        const buffer = try allocator.alignedAlloc(u8, std.mem.page_size, BlockBufferSize);
        try std.os.mprotect(buffer, 0b111); // 0b111 => std.os.windows.PAGE_EXECUTE_READWRITE
        return .{
            .buffer = buffer,
            .blocks = std.AutoHashMap(u32, BasicBlock).init(allocator),
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        std.os.munmap(self.buffer);
        self.blocks.deinit();
    }

    pub fn get(self: *@This(), address: u32) ?BasicBlock {
        return self.blocks.get(address);
    }

    pub fn compile(self: *@This(), address: u32, instructions: [*]u16) !BasicBlock {
        var emitter = Emitter.init(self.buffer[self.cursor..]);

        var jb = JITBlock.init(self._allocator);

        var index: u32 = 0;
        while (true) {
            const instr = instructions[index];
            sh4_jit_log.debug("  [{X:0>8}] {s}", .{ address + 2 * index, sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].name });
            const branch = try sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].jit_emit_fn(&jb, @bitCast(instr));

            // Increment PC
            try jb.mov(.{ .reg = .ArgRegister3 }, .{ .mem = .{ .reg = .SavedRegister0, .offset = @offsetOf(sh4.SH4, "pc") } });
            try jb.add(.ArgRegister3, .{ .imm = 2 });
            try jb.mov(.{ .mem = .{ .reg = .SavedRegister0, .offset = @offsetOf(sh4.SH4, "pc") } }, .{ .reg = .ArgRegister3 });

            emitter.block.cycles += sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].issue_cycles;
            emitter.block.instr_count += 1;
            index += 1;
            if (branch)
                break;
        }

        try emitter.emit_block(&jb);

        self.cursor += emitter.block.size;

        if (self.cursor > BlockBufferSize) {
            @panic("JIT block buffer overflow. Please increase BlockBufferSize :)");
        }

        // Debug Dump
        if (false) {
            for (0..emitter.block.size) |i| {
                std.debug.print("{X:0>2} ", .{emitter.block.buffer[i]});
            }
            std.debug.print("\n", .{});
        }

        try self.blocks.put(address, emitter.block);
        return self.get(address).?;
    }
};

pub const SH4JIT = struct {
    block_cache: BlockCache,

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        return .{
            .block_cache = try BlockCache.init(allocator),
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        self.block_cache.deinit();
    }

    pub fn execute(self: *@This(), cpu: *sh4.SH4) !u32 {
        cpu.handle_interrupts();

        if (cpu.execution_state == .Running or cpu.execution_state == .ModuleStandby) {
            const pc = cpu.pc & 0x1FFFFFFF;
            var block = self.block_cache.get(pc);
            if (block == null) {
                sh4_jit_log.info("(Cache Miss) Compiling {X:0>8}...", .{pc});
                const instructions: [*]u16 = @alignCast(@ptrCast(cpu._get_memory(pc)));
                block = try self.block_cache.compile(pc, instructions);
            }
            sh4_jit_log.debug("Running {X:0>8} ({} cycles)", .{ pc, block.?.cycles });
            block.?.execute(cpu);

            cpu.add_cycles(block.?.cycles);
            cpu._pending_cycles = 0;
            return block.?.cycles;
        } else {
            // FIXME: Not sure if this is a thing.
            cpu.add_cycles(8);
            const cycles = cpu._pending_cycles;
            cpu._pending_cycles = 0;
            return cycles;
        }
    }
};

pub fn interpreter_fallback(block: *JITBlock, instr: sh4.Instr) !bool {
    try block.mov(.{ .reg = .ArgRegister0 }, .{ .reg = .SavedRegister0 });
    try block.mov(.{ .reg = .ArgRegister1 }, .{ .imm = @as(u16, @bitCast(instr)) });
    try block.call_2(*sh4.SH4, sh4.Instr, sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr.value]].fn_);
    return false;
}

pub fn interpreter_fallback_branch(block: *JITBlock, instr: sh4.Instr) !bool {
    _ = try interpreter_fallback(block, instr);
    return true;
}

pub fn mov_rm_rn(block: *JITBlock, instr: sh4.Instr) !bool {
    // cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.m).*;
    try block.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .reg = .SavedRegister0, .offset = @offsetOf(sh4.SH4, "r") + @as(u32, instr.nmd.m) * 4 } });
    try block.mov(.{ .mem = .{ .reg = .SavedRegister0, .offset = @offsetOf(sh4.SH4, "r") + @as(u32, instr.nmd.n) * 4 } }, .{ .reg = .ReturnRegister });
    return false;
}
