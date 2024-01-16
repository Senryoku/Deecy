const std = @import("std");

const sh4 = @import("../sh4.zig");
const bit_manip = @import("../bit_manip.zig");
const JIT = @import("jit_block.zig");
const JITBlock = JIT.JITBlock;
const Emitter = @import("x86_64.zig").Emitter;
const BasicBlock = @import("basic_block.zig").BasicBlock;

const sh4_instructions = @import("../sh4_instructions.zig");

const sh4_jit_log = std.log.scoped(.sh4_jit);

const Dreamcast = @import("../dreamcast.zig").Dreamcast;

const BlockBufferSize = 1024 * 4096;

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

    pub fn compile(self: *@This(), start_ctx: JITContext, instructions: [*]u16) !BasicBlock {
        var ctx = start_ctx;

        var emitter = Emitter.init(self.buffer[self.cursor..]);

        var jb = JITBlock.init(self._allocator);
        defer jb.deinit();

        // We'll be using these callee saved registers, push 'em to the stack.
        try jb.push(.{ .reg = .SavedRegister0 });
        try jb.push(.{ .reg = .SavedRegister1 }); // NOTE: Not needed anymore, but we need to align the stack to 16 bytes, so might as well.

        try jb.mov(.{ .reg = .SavedRegister0 }, .{ .reg = .ArgRegister0 }); // Save the pointer to the SH4

        var index: u32 = 0;
        while (true) {
            const instr = instructions[index];

            sh4_jit_log.debug("  [{X:0>8}] {s}", .{ ctx.address, sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].name });

            const branch = try sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].jit_emit_fn(&jb, ctx, @bitCast(instr));

            emitter.block.cycles += sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].issue_cycles;
            if (branch)
                break;

            index += 1;
            ctx.address += 2;

            if (self.cursor + emitter.block_size >= BlockBufferSize) {
                return error.JITCacheFull;
            }
        }

        // We still rely on the interpreter implementation of the branch instructions which expects the PC to be updated automatically.
        // cpu.pc += 2;
        try jb.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .reg = .SavedRegister0, .offset = @offsetOf(sh4.SH4, "pc"), .size = 32 } });
        try jb.add(.ReturnRegister, .{ .imm = 2 });
        try jb.mov(.{ .mem = .{ .reg = .SavedRegister0, .offset = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = .ReturnRegister });

        // Restore callee saved registers.
        try jb.pop(.{ .reg = .SavedRegister1 });
        try jb.pop(.{ .reg = .SavedRegister0 });

        try emitter.emit_block(&jb);
        emitter.block.buffer = emitter.block.buffer[0..emitter.block_size]; // Update slice size.

        self.cursor += emitter.block_size;

        try self.blocks.put(start_ctx.address, emitter.block);
        return self.get(start_ctx.address).?;
    }
};

pub const JITContext = struct {
    address: u32,
    dc: *Dreamcast,
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
                sh4_jit_log.debug("(Cache Miss) Compiling {X:0>8}...", .{pc});
                const instructions: [*]u16 = @alignCast(@ptrCast(cpu._get_memory(pc)));
                block = try (self.block_cache.compile(.{ .address = pc, .dc = cpu._dc.? }, instructions) catch |err| retry: {
                    if (err == error.JITCacheFull) {
                        self.block_cache.reset();
                        sh4_jit_log.info("JIT cache purged.", .{});
                        break :retry self.block_cache.compile(.{ .address = pc, .dc = cpu._dc.? }, instructions);
                    } else break :retry err;
                });
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

pub fn interpreter_fallback(block: *JITBlock, _: JITContext, instr: sh4.Instr) !bool {
    try block.mov(.{ .reg = .ArgRegister0 }, .{ .reg = .SavedRegister0 });
    try block.mov(.{ .reg = .ArgRegister1 }, .{ .imm = @as(u16, @bitCast(instr)) });
    try block.call(sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr.value]].fn_);
    return false;
}

pub fn interpreter_fallback_branch(block: *JITBlock, ctx: JITContext, instr: sh4.Instr) !bool {
    // Restore PC in memory.
    try block.mov(.{ .mem = .{ .reg = .SavedRegister0, .offset = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .imm32 = ctx.address });
    _ = try interpreter_fallback(block, ctx, instr);
    return true;
}

fn get_reg_mem(r: u4) JIT.Operand {
    return .{ .mem = .{ .reg = .SavedRegister0, .offset = @offsetOf(sh4.SH4, "r") + @as(u32, r) * 4, .size = 32 } };
}

pub fn mov_rm_rn(block: *JITBlock, _: JITContext, instr: sh4.Instr) !bool {
    try block.mov(.{ .reg = .ReturnRegister }, get_reg_mem(instr.nmd.m));
    try block.mov(get_reg_mem(instr.nmd.n), .{ .reg = .ReturnRegister });
    return false;
}

pub fn mov_imm_rn(block: *JITBlock, _: JITContext, instr: sh4.Instr) !bool {
    // FIXME: Should keep the "signess" in the type system?  ---v
    try block.mov(get_reg_mem(instr.nmd.n), .{ .imm32 = @bitCast(bit_manip.sign_extension_u8(instr.nd8.d)) });
    return false;
}

pub fn mova_atdispPC_R0(block: *JITBlock, ctx: JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nd8.d) << 2;
    const addr = (ctx.address & 0xFFFFFFFC) + 4 + d;
    try block.mov(get_reg_mem(0), .{ .imm32 = addr });
    return false;
}

pub fn movw_atdispPC_Rn(block: *JITBlock, ctx: JITContext, instr: sh4.Instr) !bool {
    // Adress should be either in Boot ROM, or in RAM.
    std.debug.assert(ctx.address < 0x00200000 or (ctx.address >= 0x0C000000 and ctx.address < 0x10000000));
    // @(d8,PC) is fixed, compute its real absolute address
    const d = bit_manip.zero_extend(instr.nd8.d) << 1;
    const addr = ctx.address + 4 + d;
    const abs_addr = @intFromPtr(if (ctx.address < 0x00200000) &ctx.dc.boot[addr] else &ctx.dc.ram[addr & 0x00FFFFFF]);
    // Set it to a scratch register
    try block.mov(.{ .reg = .ReturnRegister }, .{ .imm = abs_addr });
    // Load the pointed value
    try block.movsx(.{ .reg = .ReturnRegister }, .{ .mem = .{ .reg = .ReturnRegister, .offset = 0, .size = 16 } });
    // Store it into Rn
    try block.mov(get_reg_mem(instr.nd8.n), .{ .reg = .ReturnRegister });
    return false;
}

pub fn movl_atdispPC_Rn(block: *JITBlock, ctx: JITContext, instr: sh4.Instr) !bool {
    // Adress should be either in Boot ROM, or in RAM.
    std.debug.assert(ctx.address < 0x00200000 or (ctx.address >= 0x0C000000 and ctx.address < 0x10000000));
    // @(d8,PC) is fixed, compute its real absolute address
    const d = bit_manip.zero_extend(instr.nd8.d) << 2;
    const addr = (ctx.address & 0xFFFFFFFC) + 4 + d;
    const abs_addr = @intFromPtr(if (ctx.address < 0x00200000) &ctx.dc.boot[addr] else &ctx.dc.ram[addr & 0x00FFFFFF]);
    // Set it to a scratch register
    try block.mov(.{ .reg = .ReturnRegister }, .{ .imm = abs_addr });
    // Load the pointed value
    try block.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .reg = .ReturnRegister, .offset = 0, .size = 32 } });
    // Store it into Rn
    try block.mov(get_reg_mem(instr.nd8.n), .{ .reg = .ReturnRegister });
    return false;
}
