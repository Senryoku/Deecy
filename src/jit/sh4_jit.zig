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

const BlockBufferSize = 16 * 1024 * 1024;

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
    blocks: []?BasicBlock,

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        const buffer = try allocator.alignedAlloc(u8, std.mem.page_size, BlockBufferSize);
        try std.os.mprotect(buffer, 0b111); // 0b111 => std.os.windows.PAGE_EXECUTE_READWRITE
        return .{
            .buffer = buffer,
            .blocks = allocator.alloc(?BasicBlock, 0x02200000 >> 1) catch unreachable,
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        std.os.munmap(self.buffer);
        self.blocks.deinit();
    }

    pub fn reset(self: *@This()) void {
        self.cursor = 0;
        @memset(self.blocks, null);
    }

    inline fn compute_key(address: u32, sz: u1, pr: u1) u32 {
        _ = sz;
        _ = pr;
        // FIXME: I'm not actually using sz or pr right now because I doubt this will be a problem,
        //        and switching to an array for the lookup wastes a lot of memory.
        return (if (address > 0x0C000000) (address & 0x01FFFFFF) + 0x00200000 else address) >> 1;
    }

    pub fn get(self: *@This(), address: u32, sz: u1, pr: u1) ?BasicBlock {
        return self.blocks[compute_key(address, sz, pr)];
    }

    pub fn put(self: *@This(), address: u32, sz: u1, pr: u1, block: BasicBlock) void {
        self.blocks[compute_key(address, sz, pr)] = block;
    }
};

const JITBitState = enum(u2) {
    Zero = 0,
    One = 1,
    Unknown,
};

pub const JITContext = struct {
    address: u32,
    dc: *Dreamcast,

    // Hacked on state to support delayed branches.
    delay_slot: ?u32 = null,
    // Jitted branches do not need to increment the PC manually.
    outdated_pc: bool = true,

    fpscr_sz: JITBitState,
    fpscr_pr: JITBitState,
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

    pub noinline fn execute(self: *@This(), cpu: *sh4.SH4) !u32 {
        cpu.handle_interrupts();

        if (cpu.execution_state == .Running or cpu.execution_state == .ModuleStandby) {
            const pc = cpu.pc & 0x1FFFFFFF;
            var block = self.block_cache.get(pc, cpu.fpscr.sz, cpu.fpscr.pr);
            if (block == null) {
                sh4_jit_log.debug("(Cache Miss) Compiling {X:0>8}...", .{pc});
                const instructions: [*]u16 = @alignCast(@ptrCast(cpu._get_memory(pc)));
                block = try (self.compile(.{
                    .address = pc,
                    .dc = cpu._dc.?,
                    .fpscr_sz = if (cpu.fpscr.sz == 1) .One else .Zero,
                    .fpscr_pr = if (cpu.fpscr.pr == 1) .One else .Zero,
                }, instructions) catch |err| retry: {
                    if (err == error.JITCacheFull) {
                        self.block_cache.reset();
                        sh4_jit_log.info("JIT cache purged.", .{});
                        break :retry self.compile(.{
                            .address = pc,
                            .dc = cpu._dc.?,
                            .fpscr_sz = if (cpu.fpscr.sz == 1) .One else .Zero,
                            .fpscr_pr = if (cpu.fpscr.pr == 1) .One else .Zero,
                        }, instructions);
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

    pub fn compile(self: *@This(), start_ctx: JITContext, instructions: [*]u16) !BasicBlock {
        var ctx = start_ctx;

        var emitter = Emitter.init(self._allocator, self.block_cache.buffer[self.block_cache.cursor..]);
        defer emitter.deinit();

        var jb = JITBlock.init(self._allocator);
        defer jb.deinit();

        // We'll be using these callee saved registers, push 'em to the stack.
        try jb.push(.{ .reg = .SavedRegister0 });
        try jb.push(.{ .reg = .SavedRegister1 }); // NOTE: We need to align the stack to 16 bytes. Used in load_mem().

        try jb.mov(.{ .reg = .SavedRegister0 }, .{ .reg = .ArgRegister0 }); // Save the pointer to the SH4

        var index: u32 = 0;
        while (true) {
            const instr = instructions[index];
            sh4_jit_log.debug("  [{X:0>8}] {s}", .{ ctx.address, sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].name });
            const branch = try sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].jit_emit_fn(&jb, &ctx, @bitCast(instr));
            emitter.block.cycles += sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].issue_cycles;
            index += 1;
            ctx.address += 2;

            if (branch) {
                if (ctx.delay_slot != null) {
                    const delay_slot = instructions[index];
                    sh4_jit_log.debug("  [{X:0>8}]   {s}", .{ ctx.address, sh4_instructions.Opcodes[sh4_instructions.JumpTable[delay_slot]].name });
                    const brach_delay_slot = try sh4_instructions.Opcodes[sh4_instructions.JumpTable[delay_slot]].jit_emit_fn(&jb, &ctx, @bitCast(delay_slot));
                    emitter.block.cycles += sh4_instructions.Opcodes[sh4_instructions.JumpTable[delay_slot]].issue_cycles;
                    std.debug.assert(!brach_delay_slot);
                }
                break;
            }
        }

        // Crude appromixation, better purging slightly too often than crashing.
        // Also feels better than checking the length at each insertion.
        if (self.block_cache.cursor + 4 * 32 + 8 * 4 * emitter.block_size >= BlockBufferSize) {
            return error.JITCacheFull;
        }

        // We still rely on the interpreter implementation of the branch instructions which expects the PC to be updated automatically.
        // cpu.pc += 2;
        if (ctx.outdated_pc) {
            try jb.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } });
            try jb.add(.{ .reg = .ReturnRegister }, .{ .imm32 = 2 });
            try jb.mov(.{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = .ReturnRegister });
        }

        // Restore callee saved registers.
        try jb.pop(.{ .reg = .SavedRegister1 });
        try jb.pop(.{ .reg = .SavedRegister0 });

        try emitter.emit_block(&jb);
        emitter.block.buffer = emitter.block.buffer[0..emitter.block_size]; // Update slice size.

        self.block_cache.cursor += emitter.block_size;

        self.block_cache.put(start_ctx.address, @truncate(@intFromEnum(start_ctx.fpscr_sz)), @truncate(@intFromEnum(start_ctx.fpscr_pr)), emitter.block);
        return self.block_cache.get(start_ctx.address, @truncate(@intFromEnum(start_ctx.fpscr_sz)), @truncate(@intFromEnum(start_ctx.fpscr_pr))).?;
    }
};

pub fn interpreter_fallback(block: *JITBlock, _: *JITContext, instr: sh4.Instr) !bool {
    try block.mov(.{ .reg = .ArgRegister0 }, .{ .reg = .SavedRegister0 });
    try block.mov(.{ .reg = .ArgRegister1 }, .{ .imm = @as(u16, @bitCast(instr)) });
    try block.call(sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr.value]].fn_);
    return false;
}

pub fn interpreter_fallback_branch(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Restore PC in memory.
    try block.mov(.{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .imm32 = ctx.address });
    _ = try interpreter_fallback(block, ctx, instr);
    return true;
}

fn guest_reg_mem(r: u4) JIT.Operand {
    return .{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "r") + @as(u32, r) * 4, .size = 32 } };
}

fn get_fp_reg_mem(r: u4) JIT.Operand {
    return .{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "fp_banks") + @as(u32, r) * 4, .size = 32 } };
}

fn get_dfp_reg_mem(r: u4) JIT.Operand {
    const bank: u32 = if ((r & 1) == 1) @sizeOf([16]f32) else 0;
    return .{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "fp_banks") + bank + @as(u32, r >> 1) * 8, .size = 64 } };
}

fn load_register(block: *JITBlock, _: *JITContext, host_reg: JIT.Register, guest_reg: u4) !void {
    // We could use this to cache register into host saved registers.
    try block.mov(.{ .reg = host_reg }, guest_reg_mem(guest_reg));
}

fn store_register(block: *JITBlock, _: *JITContext, guest_reg: u4, host_reg: JIT.Register) !void {
    try block.mov(guest_reg_mem(guest_reg), .{ .reg = host_reg });
}

// Load a u<size> from memory into a host register, with a fast path if the address lies in RAM.
fn load_mem(block: *JITBlock, ctx: *JITContext, dest: JIT.Register, guest_reg: u4, displacement: u32, comptime size: u32) !void {
    try load_register(block, ctx, .ArgRegister1, guest_reg);
    if (displacement != 0)
        try block.add(.{ .reg = .ArgRegister1 }, .{ .imm32 = displacement });

    // RAM Fast path
    try block.mov(.{ .reg = .ReturnRegister }, .{ .reg = .ArgRegister1 });
    try block.append(.{ .And = .{ .dst = .{ .reg = .ReturnRegister }, .src = .{ .imm32 = 0x1C000000 } } });
    try block.append(.{ .Cmp = .{ .lhs = .ReturnRegister, .rhs = .{ .imm32 = 0x0C000000 } } });
    var not_branch = try block.jmp(.NotEqual);
    // We're in RAM!
    try block.mov(.{ .reg = .ReturnRegister }, .{ .reg = .ArgRegister1 });
    try block.append(.{ .And = .{ .dst = .{ .reg = .ReturnRegister }, .src = .{ .imm32 = 0x00FFFFFF } } });
    const ram_addr: u64 = @intFromPtr(ctx.dc.ram.ptr);
    try block.mov(.{ .reg = .SavedRegister1 }, .{ .imm = ram_addr }); // FIXME: I'm using a saved register here because right now I know it's not used, this might be worth it to keep it at all times!

    try block.mov(.{ .reg = dest }, .{ .mem = .{ .base = .SavedRegister1, .index = .ReturnRegister, .size = size } });
    var to_end = try block.jmp(.Always);

    not_branch.patch();
    try block.mov(.{ .reg = .ArgRegister0 }, .{ .reg = .SavedRegister0 });
    // Address is already loaded into .ArgRegister1
    if (size == 32) {
        try block.call(&sh4.SH4._out_of_line_read32);
    } else if (size == 64) {
        try block.call(&sh4.SH4._out_of_line_read64);
    } else @compileError("load_mem: Unsupported size.");

    if (dest != .ReturnRegister)
        try block.mov(.{ .reg = dest }, .{ .reg = .ReturnRegister });

    to_end.patch();
}

fn store_mem(block: *JITBlock, ctx: *JITContext, dest_guest_reg: u4, displacement: u32, value: JIT.Register, comptime size: u32) !void {
    try load_register(block, ctx, .ArgRegister1, dest_guest_reg);
    if (displacement != 0)
        try block.add(.{ .reg = .ArgRegister1 }, .{ .imm32 = displacement });
    if (value != .ArgRegister2)
        try block.mov(.{ .reg = .ArgRegister2 }, .{ .reg = value });

    // RAM Fast path
    try block.mov(.{ .reg = .ReturnRegister }, .{ .reg = .ArgRegister1 });
    try block.append(.{ .And = .{ .dst = .{ .reg = .ReturnRegister }, .src = .{ .imm32 = 0x1C000000 } } });
    try block.append(.{ .Cmp = .{ .lhs = .ReturnRegister, .rhs = .{ .imm32 = 0x0C000000 } } });
    var not_branch = try block.jmp(.NotEqual);
    // We're in RAM!
    try block.mov(.{ .reg = .ReturnRegister }, .{ .reg = .ArgRegister1 });
    try block.append(.{ .And = .{ .dst = .{ .reg = .ReturnRegister }, .src = .{ .imm32 = 0x00FFFFFF } } });
    const ram_addr: u64 = @intFromPtr(ctx.dc.ram.ptr);
    try block.mov(.{ .reg = .SavedRegister1 }, .{ .imm = ram_addr }); // FIXME: I'm using a saved register here because right now I know it's not used, this might be worth it to keep it at all times!

    try block.mov(.{ .mem = .{ .base = .SavedRegister1, .index = .ReturnRegister, .size = size } }, .{ .reg = .ArgRegister2 });
    var to_end = try block.jmp(.Always);

    not_branch.patch();
    try block.mov(.{ .reg = .ArgRegister0 }, .{ .reg = .SavedRegister0 });
    // Address is already loaded into .ArgRegister1
    // Value is already loaded into .ArgRegister2
    if (size == 32) {
        try block.call(&sh4.SH4._out_of_line_write32);
    } else if (size == 64) {
        try block.call(&sh4.SH4._out_of_line_write64);
    } else @compileError("store_mem: Unsupported size.");
    if (value != .ReturnRegister)
        try block.mov(.{ .reg = value }, .{ .reg = .ReturnRegister });

    to_end.patch();
}

pub fn mov_rm_rn(block: *JITBlock, _: *JITContext, instr: sh4.Instr) !bool {
    try block.mov(.{ .reg = .ReturnRegister }, guest_reg_mem(instr.nmd.m));
    try block.mov(guest_reg_mem(instr.nmd.n), .{ .reg = .ReturnRegister });
    return false;
}

pub fn mov_imm_rn(block: *JITBlock, _: *JITContext, instr: sh4.Instr) !bool {
    // FIXME: Should keep the "signess" in the type system?  ---v
    try block.mov(guest_reg_mem(instr.nmd.n), .{ .imm32 = @bitCast(bit_manip.sign_extension_u8(instr.nd8.d)) });
    return false;
}

pub fn movl_at_rm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try load_mem(block, ctx, .ReturnRegister, instr.nmd.m, 0, 32);
    try store_register(block, ctx, instr.nmd.n, .ReturnRegister);
    return false;
}

pub fn movl_rm_at_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try load_register(block, ctx, .ReturnRegister, instr.nmd.m);
    try store_mem(block, ctx, instr.nmd.n, 0, .ReturnRegister, 32);
    return false;
}

pub fn movl_at_disp_rm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d) << 2;
    try load_mem(block, ctx, .ReturnRegister, instr.nmd.m, d, 32);
    try store_register(block, ctx, instr.nmd.n, .ReturnRegister);
    return false;
}

pub fn movl_rm_at_disp_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d) << 2;
    try load_register(block, ctx, .ReturnRegister, instr.nmd.m);
    try store_mem(block, ctx, instr.nmd.n, d, .ReturnRegister, 32);
    return false;
}

pub fn add_imm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    _ = ctx;
    try block.add(guest_reg_mem(instr.nmd.n), .{ .imm32 = @bitCast(bit_manip.sign_extension_u8(instr.nd8.d)) });
    return false;
}

pub fn fmovs_at_rm_frn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero => {
            // FRn = [Rm]
            try load_mem(block, ctx, .ReturnRegister, instr.nmd.m, 0, 32);
            try block.mov(get_fp_reg_mem(instr.nmd.n), .{ .reg = .ReturnRegister });
        },
        .One => {
            try load_mem(block, ctx, .ReturnRegister, instr.nmd.m, 0, 64);
            try block.mov(get_dfp_reg_mem(instr.nmd.n), .{ .reg = .ReturnRegister });
        },
        .Unknown => {
            _ = try interpreter_fallback(block, ctx, instr);
        },
    }
    return false;
}

pub fn fmovs_frm_at_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero => {
            // [Rn] = FRm
            try block.mov(.{ .reg = .ReturnRegister }, get_fp_reg_mem(instr.nmd.m));
            try store_mem(block, ctx, instr.nmd.n, 0, .ReturnRegister, 32);
        },
        .One => {
            try block.mov(.{ .reg = .ReturnRegister }, get_dfp_reg_mem(instr.nmd.m));
            try store_mem(block, ctx, instr.nmd.n, 0, .ReturnRegister, 64);
        },
        .Unknown => {
            _ = try interpreter_fallback(block, ctx, instr);
        },
    }
    return false;
}

pub fn fmovs_at_rm_inc_frn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero, .One => {
            _ = try fmovs_at_rm_frn(block, ctx, instr);
            // Inc Rm
            try load_register(block, ctx, .ReturnRegister, instr.nmd.m);
            try block.add(.{ .reg = .ReturnRegister }, .{ .imm32 = if (ctx.fpscr_sz == .One) 8 else 4 });
            try store_register(block, ctx, instr.nmd.m, .ReturnRegister);
        },
        .Unknown => {
            _ = try interpreter_fallback(block, ctx, instr);
        },
    }
    return false;
}

pub fn fmovs_frm_at_dec_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero, .One => {
            // Dec Rn
            try load_register(block, ctx, .ReturnRegister, instr.nmd.n);
            try block.sub(.ReturnRegister, .{ .imm32 = if (ctx.fpscr_sz == .One) 8 else 4 });
            try store_register(block, ctx, instr.nmd.n, .ReturnRegister);
            _ = try fmovs_frm_at_rn(block, ctx, instr);
        },
        .Unknown => {
            _ = try interpreter_fallback(block, ctx, instr);
        },
    }
    return false;
}

pub fn lds_rn_FPSCR(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    _ = try interpreter_fallback(block, ctx, instr);
    ctx.fpscr_sz = .Unknown;
    ctx.fpscr_pr = .Unknown;
    return false;
}

pub fn ldsl_at_rn_inc_FPSCR(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    _ = try interpreter_fallback(block, ctx, instr);
    ctx.fpscr_sz = .Unknown;
    ctx.fpscr_pr = .Unknown;
    return false;
}

pub fn fschg(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    _ = try interpreter_fallback(block, ctx, instr);
    switch (ctx.fpscr_sz) {
        .Zero => ctx.fpscr_sz = .One,
        .One => ctx.fpscr_sz = .Zero,
        else => {},
    }
    return false;
}

pub fn mova_atdispPC_R0(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nd8.d) << 2;
    const addr = (ctx.address & 0xFFFFFFFC) + 4 + d;
    try block.mov(guest_reg_mem(0), .{ .imm32 = addr });
    return false;
}

pub fn movw_atdispPC_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Adress should be either in Boot ROM, or in RAM.
    std.debug.assert(ctx.address < 0x00200000 or (ctx.address >= 0x0C000000 and ctx.address < 0x10000000));
    // @(d8,PC) is fixed, compute its real absolute address
    const d = bit_manip.zero_extend(instr.nd8.d) << 1;
    const addr = ctx.address + 4 + d;
    const abs_addr = @intFromPtr(if (ctx.address < 0x00200000) &ctx.dc.boot[addr] else &ctx.dc.ram[addr & 0x00FFFFFF]);
    // Set it to a scratch register
    try block.mov(.{ .reg = .ReturnRegister }, .{ .imm = abs_addr });
    // Load the pointed value
    try block.movsx(.{ .reg = .ReturnRegister }, .{ .mem = .{ .base = .ReturnRegister, .size = 16 } });
    // Store it into Rn
    try block.mov(guest_reg_mem(instr.nd8.n), .{ .reg = .ReturnRegister });
    return false;
}

pub fn movl_atdispPC_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Adress should be either in Boot ROM, or in RAM.
    std.debug.assert(ctx.address < 0x00200000 or (ctx.address >= 0x0C000000 and ctx.address < 0x10000000));
    // @(d8,PC) is fixed, compute its real absolute address
    const d = bit_manip.zero_extend(instr.nd8.d) << 2;
    const addr = (ctx.address & 0xFFFFFFFC) + 4 + d;
    const abs_addr = @intFromPtr(if (ctx.address < 0x00200000) &ctx.dc.boot[addr] else &ctx.dc.ram[addr & 0x00FFFFFF]);
    // Set it to a scratch register
    try block.mov(.{ .reg = .ReturnRegister }, .{ .imm = abs_addr });
    // Load the pointed value
    try block.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .base = .ReturnRegister, .size = 32 } });
    // Store it into Rn
    try block.mov(guest_reg_mem(instr.nd8.n), .{ .reg = .ReturnRegister });
    return false;
}

pub fn jsr_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // cpu.pr = cpu.pc + 4;
    try block.mov(.{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "pr"), .size = 32 } }, .{ .imm32 = ctx.address + 4 });
    // cpu.pc = Rn
    try block.mov(.{ .reg = .ReturnRegister }, guest_reg_mem(instr.nmd.n));
    try block.mov(.{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = .ReturnRegister });

    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}

pub fn rts(block: *JITBlock, ctx: *JITContext, _: sh4.Instr) !bool {
    // cpu.pc = cpu.pr
    try block.mov(.{ .reg = .ReturnRegister }, .{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "pr"), .size = 32 } });
    try block.mov(.{ .mem = .{ .base = .SavedRegister0, .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = .ReturnRegister });

    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}
