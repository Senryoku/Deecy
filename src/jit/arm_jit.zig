const std = @import("std");

const arm7 = @import("arm7");
const bit_manip = @import("../bit_manip.zig");
const JIT = @import("jit_block.zig");
const JITBlock = JIT.JITBlock;
const Emitter = @import("x86_64.zig").Emitter;
const BasicBlock = @import("basic_block.zig").BasicBlock;
const Dreamcast = @import("../dreamcast.zig").Dreamcast;

const arm_jit_log = std.log.scoped(.arm_jit);

const BlockBufferSize = 2 * 1024 * 1024;

const HashMapContext = struct {
    pub fn hash(_: @This(), addr: u32) u64 {
        return @as(u64, addr) * 2654435761;
    }
    pub fn eql(_: @This(), a: u32, b: u32) bool {
        return a == b;
    }
};

const BlockCache = struct {
    buffer: []align(std.mem.page_size) u8,
    cursor: u32 = 0,
    blocks: std.HashMap(u32, BasicBlock, HashMapContext, std.hash_map.default_max_load_percentage),

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        const buffer = try allocator.alignedAlloc(u8, std.mem.page_size, BlockBufferSize);
        try std.os.mprotect(buffer, 0b111); // 0b111 => std.os.windows.PAGE_EXECUTE_READWRITE
        return .{
            .buffer = buffer,
            .blocks = std.HashMap(u32, BasicBlock, HashMapContext, std.hash_map.default_max_load_percentage).init(allocator),
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        std.os.munmap(self.buffer);
        self.blocks.deinit();
    }

    pub fn reset(self: *@This()) void {
        self.cursor = 0;
        self.blocks.clearRetainingCapacity();
    }

    pub fn get(self: *@This(), address: u32) ?BasicBlock {
        return self.blocks.get(address);
    }

    pub fn put(self: *@This(), address: u32, block: BasicBlock) !void {
        try self.blocks.put(address, block);
    }
};

pub const JITContext = struct {
    address: u32,
};

pub const ARM7JIT = struct {
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

    pub fn run_for(self: *@This(), cpu: *arm7.ARM7, cycles: i32) !i32 {
        if (!cpu.running) return 0;

        var spent_cycles: i32 = 0;
        while (spent_cycles < cycles) {
            const pc = cpu.pc().* - 4; // Pipelining...
            var block = self.block_cache.get(pc);
            if (block == null) {
                arm_jit_log.debug("(Cache Miss) Compiling {X:0>8}...", .{pc});
                const instructions: [*]u32 = @alignCast(@ptrCast(&cpu.memory[pc]));
                block = try (self.compile(.{ .address = pc }, instructions) catch |err| retry: {
                    if (err == error.JITCacheFull) {
                        self.block_cache.reset();
                        arm_jit_log.info("JIT cache purged.", .{});
                        break :retry self.compile(.{ .address = pc }, instructions);
                    } else break :retry err;
                });
            }
            // arm_jit_log.debug("Running {X:0>8} ({} cycles)", .{ pc, block.?.cycles });
            block.?.execute(cpu);
            spent_cycles += @intCast(block.?.cycles);

            // Not necessary, just here to allow compatibility with the interpreter if we need it.
            // (Right now we're always call to the interpreted so this should stay in sync, but once
            //  we start actually JIT some instructions, we won't keep instruction_pipeline updated)
            cpu.instruction_pipeline[0] = cpu.pc().* - 4;
        }

        return spent_cycles;
    }

    pub fn compile(self: *@This(), start_ctx: JITContext, instructions: [*]u32) !BasicBlock {
        var ctx = start_ctx;

        var emitter = Emitter.init(self._allocator, self.block_cache.buffer[self.block_cache.cursor..]);
        defer emitter.deinit();

        var jb = JITBlock.init(self._allocator);
        defer jb.deinit();

        // We'll be using these callee saved registers, push 'em to the stack.
        try jb.push(.{ .reg = .SavedRegister0 });
        try jb.push(.{ .reg = .SavedRegister1 }); // NOTE: We need to align the stack to 16 bytes anyway.

        try jb.mov(.{ .reg = .SavedRegister0 }, .{ .reg = .ArgRegister0 }); // Save the pointer to the cpu struct

        var index: u32 = 0;
        while (true) {
            const instr = instructions[index];
            arm_jit_log.debug("  [{X:0>8}] {s}", .{ ctx.address, arm7.ARM7.disassemble(instr) });

            // Update PC. Done before the instruction is executed, emulating fetching.
            // NOTE: x86 has a "in memory" add instruction, I think.
            try jb.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(arm7.ARM7, "_r") + 15 * @sizeOf(u32), .size = 32 } }); // Load PC value
            try jb.add(.ReturnRegister, .{ .imm32 = 4 }); // PC += 4
            try jb.mov(.{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(arm7.ARM7, "_r") + 15 * @sizeOf(u32), .size = 32 } }, .{ .reg = .ReturnRegister }); // Store new PC value

            var jump = try handle_condition(&jb, &ctx, instr);

            const tag = arm7.ARM7.get_instr_tag(instr);
            const branch = try InstructionHandlers[arm7.JumpTable[tag]](&jb, &ctx, instr);

            if (jump) |*j| {
                j.patch();
            }

            emitter.block.cycles += 1; // FIXME
            index += 1;
            ctx.address += 4;

            if (branch)
                break;
        }

        // Crude appromixation, better purging slightly too often than crashing.
        // Also feels better than checking the length at each insertion.
        if (self.block_cache.cursor + 4 * 32 + 8 * 4 * emitter.block_size >= BlockBufferSize) {
            return error.JITCacheFull;
        }

        // Restore callee saved registers.
        try jb.pop(.{ .reg = .SavedRegister1 });
        try jb.pop(.{ .reg = .SavedRegister0 });

        try emitter.emit_block(&jb);
        emitter.block.buffer = emitter.block.buffer[0..emitter.block_size]; // Update slice size.

        self.block_cache.cursor += emitter.block_size;

        try self.block_cache.put(start_ctx.address, emitter.block);
        return self.block_cache.get(start_ctx.address).?;
    }
};

fn handle_condition(b: *JITBlock, ctx: *JITContext, instruction: u32) !?JIT.PatchableJump {
    _ = ctx;
    const condition = arm7.ARM7.get_instr_condition(instruction);
    switch (condition) {
        .EQ => {
            // return cpu.cpsr.z
            try b.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(arm7.ARM7, "cpsr"), .size = 32 } });
            try b.append(.{ .And = .{ .dst = .ReturnRegister, .src = .{ .imm32 = @as(u32, 1 << @bitOffsetOf(arm7.CPSR, "z")) } } });
            try b.append(.{ .Cmp = .{ .lhs = .ReturnRegister, .rhs = .{ .imm32 = 0x00000000 } } });
            return try b.jmp(.NotEqual);
        },
        .NE => {
            // return !cpu.cpsr.z
            try b.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(arm7.ARM7, "cpsr"), .size = 32 } });
            try b.append(.{ .And = .{ .dst = .ReturnRegister, .src = .{ .imm32 = @as(u32, 1 << @bitOffsetOf(arm7.CPSR, "z")) } } });
            try b.append(.{ .Cmp = .{ .lhs = .ReturnRegister, .rhs = .{ .imm32 = 0x00000000 } } });
            return try b.jmp(.Equal);
        },
        .CS => {
            // return cpu.cpsr.c
            @panic("Unimplemented");
        },
        .CC => {
            // return !cpu.cpsr.c
            @panic("Unimplemented");
        },
        .MI => {
            // return cpu.cpsr.n
            @panic("Unimplemented");
        },
        .PL => {
            // return !cpu.cpsr.n
            @panic("Unimplemented");
        },
        .VS => {
            // return cpu.cpsr.v
            @panic("Unimplemented");
        },
        .VC => {
            // return !cpu.cpsr.v
            @panic("Unimplemented");
        },
        .HI => {
            // return cpu.cpsr.c and !cpu.cpsr.z
            @panic("Unimplemented");
        },
        .LS => {
            // return !cpu.cpsr.c or cpu.cpsr.z
            @panic("Unimplemented");
        },
        .GE => {
            // return cpu.cpsr.n == cpu.cpsr.v
            @panic("Unimplemented");
        },
        .LT => {
            // return cpu.cpsr.n != cpu.cpsr.v
            @panic("Unimplemented");
        },
        .GT => {
            // return !cpu.cpsr.z and (cpu.cpsr.n == cpu.cpsr.v)
            @panic("Unimplemented");
        },
        .LE => {
            // return cpu.cpsr.z or (cpu.cpsr.n != cpu.cpsr.v)
            @panic("Unimplemented");
        },
        .AL => return null,
        .Invalid => unreachable,
    }
}

pub const InstructionHandlers = [_]*const fn (b: *JITBlock, ctx: *JITContext, instruction: u32) anyerror!bool{
    handle_branch_and_exchange,
    handle_block_data_transfer,
    handle_branch,
    handle_software_interrupt,
    handle_undefined,
    handle_single_data_transfer,
    handle_single_data_swap,
    handle_multiply,
    handle_multiply_long,
    handle_halfword_data_transfer_register_offset,
    handle_halfword_data_transfer_immediate_offset,
    handle_coprocessor_data_transfer,
    handle_coprocessor_data_operation,
    handle_coprocessor_register_transfer,
    handle_mrs,
    handle_msr,
    handle_data_processing,

    handle_invalid,
};

fn handle_branch_and_exchange(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented branch and exchange", .{});
    @panic("Unimplemented branch and exchange");
}

fn handle_block_data_transfer(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    try interpreter_fallback(b, ctx, instruction);
    return true; // FIXME: Return true only if we're writing to PC
}

fn handle_branch(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    try interpreter_fallback(b, ctx, instruction);
    return true;
}

fn handle_software_interrupt(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    try interpreter_fallback(b, ctx, instruction);
    return true;
}

fn handle_undefined(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented undefined", .{});
    @panic("Unimplemented undefined");
}

fn handle_single_data_transfer(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.SingleDataTransferInstruction = @bitCast(instruction);
    try interpreter_fallback(b, ctx, instruction);
    return inst.l == 1 and inst.rd == 15;
}

fn handle_single_data_swap(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented single data swap", .{});
    @panic("Unimplemented single data swap");
}

fn handle_multiply(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented multiply", .{});
    @panic("Unimplemented multiply");
}

fn handle_multiply_long(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented multiply long", .{});
    @panic("Unimplemented multiply long");
}

fn handle_halfword_data_transfer_register_offset(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented halfword data transfer register offset", .{});
    @panic("Unimplemented halfword data transfer register offset");
}

fn handle_halfword_data_transfer_immediate_offset(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented halfword data transfer immediate offset", .{});
    @panic("Unimplemented halfword data transfer immediate offset");
}

fn handle_coprocessor_data_transfer(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented coprocessor data transfer", .{});
    @panic("Unimplemented coprocessor data transfer");
}

fn handle_coprocessor_data_operation(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented coprocessor data operation", .{});
    @panic("Unimplemented coprocessor data operation");
}

fn handle_coprocessor_register_transfer(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented coprocessor register transfer", .{});
    @panic("Unimplemented coprocessor register transfer");
}

fn handle_mrs(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.MRSInstruction = @bitCast(instruction);
    try interpreter_fallback(b, ctx, instruction);
    std.debug.assert(inst.rd != 15);
    return false;
}

fn handle_msr(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.MSRInstruction = @bitCast(instruction);
    _ = inst;
    try interpreter_fallback(b, ctx, instruction);
    return false;
}

fn handle_data_processing(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.DataProcessingInstruction = @bitCast(instruction);
    try interpreter_fallback(b, ctx, instruction);
    switch (inst.opcode) {
        .TST, .TEQ, .CMP, .CMN => return false,
        else => return inst.rd == 15,
    }
}

fn handle_invalid(_: *JITBlock, _: *JITContext, _: u32) !bool {
    arm_jit_log.err("Invalid instruction", .{});
    @panic("Invalid instruction");
}

fn interpreter_fallback(b: *JITBlock, ctx: *JITContext, instruction: u32) !void {
    _ = ctx;
    try b.mov(.{ .reg = .ArgRegister0 }, .{ .reg = .SavedRegister0 });
    try b.mov(.{ .reg = .ArgRegister1 }, .{ .imm32 = instruction });
    try b.call(arm7.interpreter.InstructionHandlers[arm7.JumpTable[arm7.ARM7.get_instr_tag(instruction)]]);
}
