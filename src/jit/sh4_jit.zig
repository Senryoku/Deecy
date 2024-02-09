const std = @import("std");

const sh4 = @import("../sh4.zig");
const sh4_interpreter = @import("../sh4_interpreter.zig");
const sh4_disassembly = @import("../sh4_disassembly.zig");
const bit_manip = @import("../bit_manip.zig");
const JIT = @import("jit_block.zig");
const JITBlock = JIT.JITBlock;

const Architecture = @import("x86_64.zig");
const ReturnRegister = Architecture.ReturnRegister;
const ArgRegisters = Architecture.ArgRegisters;
const SavedRegisters = Architecture.SavedRegisters;

const BasicBlock = @import("basic_block.zig");

const sh4_instructions = @import("../sh4_instructions.zig");

const sh4_jit_log = std.log.scoped(.sh4_jit);

const Dreamcast = @import("../dreamcast.zig").Dreamcast;

const BlockBufferSize = 16 * 1024 * 1024;

const BlockCache = struct {
    // 0x02200000 possible addresses, but 16bit aligned, multiplied by permutations of (sz, pr)
    const BlockEntryCount = (0x02200000 >> 1) << 2;

    buffer: []align(std.mem.page_size) u8,
    cursor: usize = 0,
    blocks: []?BasicBlock = undefined,

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        const buffer = try allocator.alignedAlloc(u8, std.mem.page_size, BlockBufferSize);
        try std.os.mprotect(buffer, 0b111); // 0b111 => std.os.windows.PAGE_EXECUTE_READWRITE

        var r: @This() = .{
            .buffer = buffer,
            ._allocator = allocator,
        };
        try r.allocate_blocks();
        return r;
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.free(self.buffer);

        std.os.windows.VirtualFree(self.blocks.ptr, 0, std.os.windows.MEM_RELEASE);
    }

    fn allocate_blocks(self: *@This()) !void {
        if (@import("builtin").os.tag != .windows) {
            @compileError("Unsupported OS - Use mmap on Linux here.");
        }

        const blocks = try std.os.windows.VirtualAlloc(
            null,
            @sizeOf(?BasicBlock) * BlockEntryCount,
            std.os.windows.MEM_RESERVE | std.os.windows.MEM_COMMIT,
            std.os.windows.PAGE_READWRITE,
        );
        self.blocks = @as([*]?BasicBlock, @alignCast(@ptrCast(blocks)))[0..BlockEntryCount];
    }

    pub fn reset(self: *@This()) !void {
        self.cursor = 0;

        // FIXME: Can I merely decommit and re-commit it?
        std.os.windows.VirtualFree(self.blocks.ptr, 0, std.os.windows.MEM_RELEASE);

        try self.allocate_blocks();
    }

    inline fn compute_key(address: u32, sz: u1, pr: u1) u32 {
        return ((if (address > 0x0C000000) (address & 0x01FFFFFF) + 0x00200000 else address) + (@as(u32, sz) << 25) + (@as(u32, pr) << 26)) >> 1;
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

    highest_saved_register_used: u8 = 0,
    host_registers: [5]struct { host: JIT.Register, last_access: u32, modified: bool, guest: ?u4 } = .{
        .{ .host = SavedRegisters[1], .last_access = 0, .modified = false, .guest = null },
        .{ .host = SavedRegisters[2], .last_access = 0, .modified = false, .guest = null },
        .{ .host = SavedRegisters[3], .last_access = 0, .modified = false, .guest = null },
        .{ .host = SavedRegisters[4], .last_access = 0, .modified = false, .guest = null },
        .{ .host = SavedRegisters[5], .last_access = 0, .modified = false, .guest = null },
    },

    pub fn guest_reg_memory(guest_reg: u4) JIT.Operand {
        return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "r") + @as(u32, guest_reg) * 4, .size = 32 } };
    }

    fn get_cached_register(self: *@This(), guest_reg: u4) ?*@TypeOf(self.host_registers[0]) {
        for (&self.host_registers) |*reg| {
            if (reg.guest) |r| {
                if (r == guest_reg) {
                    return reg;
                }
            }
        }
        return null;
    }

    // load: If not already in cache, will load the value from memory.
    // modified: Mark the cached value as modified, ensuring it will be written back to memory.
    pub fn guest_reg_cache(self: *@This(), block: *JITBlock, guest_reg: u4, comptime load: bool, comptime modified: bool) JIT.Register {
        for (&self.host_registers) |*hreg| {
            hreg.last_access += 1;
        }

        // Is it already cached?
        if (self.get_cached_register(guest_reg)) |hreg| {
            hreg.last_access = 0;
            if (modified) hreg.modified = true;
            return hreg.host;
        }

        var slot = s: {
            var lru = &self.host_registers[0];
            // Do we have a free slot?
            for (&self.host_registers, 0..) |*hreg, idx| {
                if (hreg.guest == null) {
                    self.highest_saved_register_used = @max(self.highest_saved_register_used, @as(u8, @intCast(idx)));
                    break :s hreg;
                }
                if (hreg.last_access > lru.last_access) {
                    lru = hreg;
                }
            }
            // No. Commit to memory and evict the oldest.
            if (lru.modified)
                block.mov(
                    guest_reg_memory(lru.guest.?),
                    .{ .reg = lru.host },
                ) catch |err| {
                    sh4_jit_log.err("Error emitting JIT instruction: {}", .{err});
                    @panic("Error emitting JIT instruction");
                };
            break :s lru;
        };

        slot.guest = guest_reg;
        slot.last_access = 0;
        slot.modified = modified;
        if (load)
            block.mov(.{ .reg = slot.host }, guest_reg_memory(guest_reg)) catch |err| {
                sh4_jit_log.err("Error emitting JIT instruction: {}", .{err});
                @panic("Error emitting JIT instruction");
            };
        return slot.host;
    }

    pub fn commit_and_invalidate_all_cached_registers(self: *@This(), block: *JITBlock) !void {
        for (&self.host_registers) |*reg| {
            if (reg.guest) |r| {
                if (reg.modified)
                    try block.mov(
                        guest_reg_memory(r),
                        .{ .reg = reg.host },
                    );
                reg.guest = null;
            }
        }
    }

    pub fn commit_all_cached_registers(self: *@This(), block: *JITBlock) !void {
        for (&self.host_registers) |*reg| {
            if (reg.guest) |r| {
                if (reg.modified)
                    try block.mov(
                        guest_reg_memory(r),
                        .{ .reg = reg.host },
                    );
            }
        }
    }

    // Means this will be overwritten outside of the JIT
    pub fn invalidate_cached_register(self: *@This(), guest_reg: u4) !void {
        if (self.get_cached_register(guest_reg)) |reg| {
            reg.guest = null;
        }
    }
    pub fn commit_cached_register(self: *@This(), block: *JITBlock, guest_reg: u4) !void {
        if (self.get_cached_register(guest_reg)) |reg| {
            if (reg.modified)
                return block.mov(
                    guest_reg_memory(reg.guest.?),
                    .{ .reg = reg.host },
                );
        }
    }
    pub fn commit_and_invalidate_cached_register(self: *@This(), block: *JITBlock, guest_reg: u4) !void {
        if (self.get_cached_register(guest_reg)) |reg| {
            if (reg.modified)
                try block.mov(
                    guest_reg_memory(reg.guest.?),
                    .{ .reg = reg.host },
                );
            reg.guest = null;
        }
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
                        try self.block_cache.reset();
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

        var b = JITBlock.init(self._allocator);
        defer b.deinit();

        // We'll be using these callee saved registers, push 'em to the stack.
        try b.push(.{ .reg = SavedRegisters[0] });
        try b.push(.{ .reg = SavedRegisters[1] }); // NOTE: We need to align the stack to 16 bytes. Used in load_mem().

        const optional_saved_register_offset = b.instructions.items.len;
        // We'll turn those into NOP if they're not used.
        try b.push(.{ .reg = SavedRegisters[2] });
        try b.push(.{ .reg = SavedRegisters[3] });
        try b.push(.{ .reg = SavedRegisters[4] });
        try b.push(.{ .reg = SavedRegisters[5] });

        try b.mov(.{ .reg = SavedRegisters[0] }, .{ .reg = ArgRegisters[0] }); // Save the pointer to the SH4

        var cycles: u32 = 0;

        var index: u32 = 0;
        while (true) {
            const instr = instructions[index];
            sh4_jit_log.debug(" [{X:0>8}] {s} {s}", .{
                ctx.address,
                if (sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].jit_emit_fn == interpreter_fallback or sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].jit_emit_fn == interpreter_fallback_branch or sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].jit_emit_fn == interpreter_fallback_cached) "!" else " ",
                try sh4_disassembly.disassemble(@bitCast(instr), self._allocator),
            });
            const branch = try sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].jit_emit_fn(&b, &ctx, @bitCast(instr));
            cycles += sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]].issue_cycles;
            index += 1;
            ctx.address += 2;

            if (branch) {
                if (ctx.delay_slot != null) {
                    const delay_slot = instructions[index];
                    sh4_jit_log.debug(" [{X:0>8}] {s}  {s}", .{
                        ctx.address,
                        if (sh4_instructions.Opcodes[sh4_instructions.JumpTable[delay_slot]].jit_emit_fn == interpreter_fallback or sh4_instructions.Opcodes[sh4_instructions.JumpTable[delay_slot]].jit_emit_fn == interpreter_fallback_branch or sh4_instructions.Opcodes[sh4_instructions.JumpTable[delay_slot]].jit_emit_fn == interpreter_fallback_cached) "!" else " ",
                        try sh4_disassembly.disassemble(@bitCast(delay_slot), self._allocator),
                    });
                    const branch_delay_slot = try sh4_instructions.Opcodes[sh4_instructions.JumpTable[delay_slot]].jit_emit_fn(&b, &ctx, @bitCast(delay_slot));
                    cycles += sh4_instructions.Opcodes[sh4_instructions.JumpTable[delay_slot]].issue_cycles;
                    std.debug.assert(!branch_delay_slot);
                }
                break;
            }
        }

        // Crude appromixation, better purging slightly too often than crashing.
        // Also feels better than checking the length at each insertion.
        if (self.block_cache.cursor + 4 * 32 + 8 * 4 * b.instructions.items.len >= BlockBufferSize) {
            return error.JITCacheFull;
        }

        // We still rely on the interpreter implementation of the branch instructions which expects the PC to be updated automatically.
        // cpu.pc += 2;
        if (ctx.outdated_pc) {
            try b.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } });
            try b.add(.{ .reg = ReturnRegister }, .{ .imm32 = 2 });
            try b.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = ReturnRegister });
        }

        try ctx.commit_and_invalidate_all_cached_registers(&b);

        // Restore callee saved registers.
        if (ctx.highest_saved_register_used >= 3) {
            try b.pop(.{ .reg = SavedRegisters[5] });
            try b.pop(.{ .reg = SavedRegisters[4] });
        } else {
            b.instructions.items[optional_saved_register_offset + 2] = .Nop;
            b.instructions.items[optional_saved_register_offset + 3] = .Nop;
        }
        if (ctx.highest_saved_register_used >= 1) {
            try b.pop(.{ .reg = SavedRegisters[3] });
            try b.pop(.{ .reg = SavedRegisters[2] });
        } else {
            b.instructions.items[optional_saved_register_offset + 0] = .Nop;
            b.instructions.items[optional_saved_register_offset + 1] = .Nop;
        }

        try b.pop(.{ .reg = SavedRegisters[1] });
        try b.pop(.{ .reg = SavedRegisters[0] });

        for (b.instructions.items, 0..) |instr, idx|
            sh4_jit_log.debug("[{d: >4}] {any}", .{ idx, instr });

        var block = try b.emit(self.block_cache.buffer[self.block_cache.cursor..]);
        self.block_cache.cursor += block.buffer.len;
        block.cycles = cycles;

        // sh4_jit_log.debug("Compiled: {X:0>2}", .{block.buffer});

        self.block_cache.put(start_ctx.address, @truncate(@intFromEnum(start_ctx.fpscr_sz)), @truncate(@intFromEnum(start_ctx.fpscr_pr)), block);
        return block;
    }
};

inline fn call_interpreter_fallback(block: *JITBlock, instr: sh4.Instr) !void {
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    try block.mov(.{ .reg = ArgRegisters[1] }, .{ .imm64 = @as(u16, @bitCast(instr)) });
    try block.call(sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr.value]].fn_);
}

// We need pointers to all of these functions, can't really refactor that mess sadly.

pub fn interpreter_fallback_cached(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    var cache_access = sh4_instructions.Opcodes[sh4_instructions.JumpTable[@as(u16, @bitCast(instr))]].access;

    // We don't want to invalidate or commit the same register twice.

    if (instr.nmd.n != 0 and instr.nmd.m != 0) {
        if (cache_access.r.r0 and cache_access.w.r0) {
            try ctx.commit_and_invalidate_cached_register(block, 0);
        } else if (cache_access.w.r0) {
            try ctx.invalidate_cached_register(0);
        } else if (cache_access.r.r0) {
            try ctx.commit_cached_register(block, 0);
        }
    }

    // We also don't want to miss a constraint.
    if (instr.nmd.n == 0) {
        cache_access.r.rn = cache_access.r.rn or cache_access.r.r0;
        cache_access.w.rn = cache_access.w.rn or cache_access.w.r0;
    }
    if (instr.nmd.m == 0) {
        cache_access.r.rm = cache_access.r.rm or cache_access.r.r0;
        cache_access.w.rm = cache_access.w.rm or cache_access.w.r0;
    }
    if (instr.nmd.n == instr.nmd.m) {
        cache_access.r.rn = cache_access.r.rn or cache_access.r.rm;
        cache_access.w.rn = cache_access.w.rn or cache_access.w.rm;
    }

    if (cache_access.r.rn and cache_access.w.rn) {
        try ctx.commit_and_invalidate_cached_register(block, instr.nmd.n);
    } else if (cache_access.w.rn) {
        try ctx.invalidate_cached_register(instr.nmd.n);
    } else if (cache_access.r.rn) {
        try ctx.commit_cached_register(block, instr.nmd.n);
    }

    if (instr.nmd.m != instr.nmd.n) {
        if (cache_access.r.rm and cache_access.w.rm) {
            try ctx.commit_and_invalidate_cached_register(block, instr.nmd.m);
        } else if (cache_access.w.rm) {
            try ctx.invalidate_cached_register(instr.nmd.m);
        } else if (cache_access.r.rm) {
            try ctx.commit_cached_register(block, instr.nmd.m);
        }
    }

    try call_interpreter_fallback(block, instr);
    return false;
}

pub fn interpreter_fallback(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try ctx.commit_and_invalidate_all_cached_registers(block);
    try call_interpreter_fallback(block, instr);
    return false;
}

pub fn interpreter_fallback_branch(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Restore PC in memory.
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .imm32 = ctx.address });
    _ = try interpreter_fallback(block, ctx, instr);
    return true;
}

pub fn nop(_: *JITBlock, _: *JITContext, _: sh4.Instr) !bool {
    return false;
}

fn get_fp_reg_mem(r: u4) JIT.Operand {
    return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "fp_banks") + @as(u32, r) * 4, .size = 32 } };
}
// NOTE: Also returns banked register (XDn) if r is odd.
fn get_dfp_reg_mem(r: u4) JIT.Operand {
    const bank: u32 = if ((r & 1) == 1) @sizeOf([16]f32) else 0;
    return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "fp_banks") + bank + @as(u32, r >> 1) * 8, .size = 64 } };
}

// NOTE: Ideally we'd use the type system to ensure the return values of the two following functions are
//       used correctly (i.e. never write to a register returned by load_register), but I think this would
//       require updating JITBlock and might hinder its genericity? (It's already pretty specific...)

// Returns a host register contained the requested guest register.
fn load_register(block: *JITBlock, ctx: *JITContext, guest_reg: u4) JIT.Register {
    return ctx.guest_reg_cache(block, guest_reg, true, false);
}
// Returns a host register contained the requested guest register and mark it as modified (we plan on writing to it).
fn load_register_for_writing(block: *JITBlock, ctx: *JITContext, guest_reg: u4) JIT.Register {
    return ctx.guest_reg_cache(block, guest_reg, true, true);
}

fn store_register(block: *JITBlock, ctx: *JITContext, guest_reg: u4, value: JIT.Operand) !void {
    try block.mov(.{ .reg = ctx.guest_reg_cache(block, guest_reg, false, true) }, value);
}

// Sets T bit in SR if Condition is fullfilled (In the Host!), otherwise clears it.
// TODO: We'll want to cache the T bit at some point too!
fn set_t(block: *JITBlock, _: *JITContext, condition: JIT.Condition) !void {
    var set = try block.jmp(condition);
    // Clear T
    // NOTE: We could use the sign extended version with an immediate of 0xFE here for a shorter encoding, but the emitter doesn't support it yet.
    try block.append(.{ .And = .{ .dst = .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "sr"), .size = 32 } }, .src = .{ .imm32 = ~(@as(u32, 1) << @bitOffsetOf(sh4.SR, "t")) } } });
    var end = try block.jmp(.Always);
    // Set T
    set.patch();
    try block.append(.{ .Or = .{ .dst = .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "sr"), .size = 32 } }, .src = .{ .imm32 = @as(u32, 1) << @bitOffsetOf(sh4.SR, "t") } } });
    end.patch();
}

// Load a u<size> from memory into a host register, with a fast path if the address lies in RAM.
fn load_mem(block: *JITBlock, ctx: *JITContext, dest: JIT.Register, guest_reg: u4, comptime addressing: enum { Reg, Reg_R0 }, displacement: u32, comptime size: u32) !void {
    const src_guest_reg_location = load_register(block, ctx, guest_reg);
    try block.mov(.{ .reg = ArgRegisters[1] }, .{ .reg = src_guest_reg_location });
    if (displacement != 0)
        try block.add(.{ .reg = ArgRegisters[1] }, .{ .imm32 = displacement });
    if (addressing == .Reg_R0) {
        const r0 = load_register(block, ctx, 0);
        try block.add(.{ .reg = ArgRegisters[1] }, .{ .reg = r0 });
    }

    // RAM Fast path
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = ArgRegisters[1] });
    try block.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = 0x1C000000 } } });
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = 0x0C000000 } } });
    var not_branch = try block.jmp(.NotEqual);
    // We're in RAM!
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = ArgRegisters[1] });
    try block.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = 0x00FFFFFF } } });
    const ram_addr: u64 = @intFromPtr(ctx.dc.ram.ptr);
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .imm64 = ram_addr });
    try block.mov(.{ .reg = dest }, .{ .mem = .{ .base = ArgRegisters[0], .index = ReturnRegister, .size = size } });
    var to_end = try block.jmp(.Always);

    not_branch.patch();

    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    // Address is already loaded into ArgRegisters[1]
    if (size == 16) {
        try block.call(&sh4.SH4._out_of_line_read16);
    } else if (size == 32) {
        try block.call(&sh4.SH4._out_of_line_read32);
    } else if (size == 64) {
        try block.call(&sh4.SH4._out_of_line_read64);
    } else @compileError("load_mem: Unsupported size.");

    if (dest != ReturnRegister)
        try block.mov(.{ .reg = dest }, .{ .reg = ReturnRegister });

    to_end.patch();
}

fn store_mem(block: *JITBlock, ctx: *JITContext, dest_guest_reg: u4, comptime addressing: enum { Reg, Reg_R0 }, displacement: u32, value: JIT.Register, comptime size: u32) !void {
    const dest_guest_reg_location = load_register(block, ctx, dest_guest_reg);
    try block.mov(.{ .reg = ArgRegisters[1] }, .{ .reg = dest_guest_reg_location });

    if (displacement != 0)
        try block.add(.{ .reg = ArgRegisters[1] }, .{ .imm32 = displacement });
    if (addressing == .Reg_R0) {
        const r0 = load_register(block, ctx, 0);
        try block.add(.{ .reg = ArgRegisters[1] }, .{ .reg = r0 });
    }

    if (value != ArgRegisters[2])
        try block.mov(.{ .reg = ArgRegisters[2] }, .{ .reg = value });

    // RAM Fast path
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = ArgRegisters[1] });
    try block.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = 0x1C000000 } } });
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = 0x0C000000 } } });
    var not_branch = try block.jmp(.NotEqual);
    // We're in RAM!
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = ArgRegisters[1] });
    try block.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = 0x00FFFFFF } } });
    const ram_addr: u64 = @intFromPtr(ctx.dc.ram.ptr);
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .imm64 = ram_addr });
    try block.mov(.{ .mem = .{ .base = ArgRegisters[0], .index = ReturnRegister, .size = size } }, .{ .reg = ArgRegisters[2] });
    var to_end = try block.jmp(.Always);

    not_branch.patch();

    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    // Address is already loaded into ArgRegisters[1]
    //   Value is already loaded into ArgRegisters[2]
    if (size == 32) {
        try block.call(&sh4.SH4._out_of_line_write32);
    } else if (size == 64) {
        try block.call(&sh4.SH4._out_of_line_write64);
    } else @compileError("store_mem: Unsupported size.");

    to_end.patch();
}

pub fn mov_rm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rm = load_register(block, ctx, instr.nmd.m);
    try store_register(block, ctx, instr.nmd.n, .{ .reg = rm });
    return false;
}

pub fn mov_imm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // FIXME: Should keep the "signess" in the type system?  ---v
    try store_register(block, ctx, instr.nmd.n, .{ .imm32 = @bitCast(bit_manip.sign_extension_u8(instr.nd8.d)) });
    return false;
}

pub fn movl_at_rm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 32);
    try store_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
    return false;
}

pub fn movl_rm_at_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rm = load_register(block, ctx, instr.nmd.m);
    try store_mem(block, ctx, instr.nmd.n, .Reg, 0, rm, 32);
    return false;
}

pub fn movw_at_rm_inc_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Rn = [Rm]
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 16);
    // Sign extend
    try block.movsx(.{ .reg = ReturnRegister }, .{ .reg = ReturnRegister });
    try store_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
    // if(n != m) Rm += 2
    if (instr.nmd.n != instr.nmd.m) {
        const rm = load_register_for_writing(block, ctx, instr.nmd.m);
        try block.add(.{ .reg = rm }, .{ .imm32 = 2 });
    }
    return false;
}

pub fn movl_at_rm_inc_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Rn = [Rm]
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 32);
    try store_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
    // if(n != m) Rm += 4
    if (instr.nmd.n != instr.nmd.m) {
        const rm = load_register_for_writing(block, ctx, instr.nmd.m);
        try block.add(.{ .reg = rm }, .{ .imm32 = 4 });
    }
    return false;
}

pub fn movl_rm_at_rn_dec(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Rm -= 4
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.sub(.{ .reg = rn }, .{ .imm32 = 4 });
    // [Rn] = Rm
    const rm = load_register(block, ctx, instr.nmd.m);
    try store_mem(block, ctx, instr.nmd.n, .Reg, 0, rm, 32);
    return false;
}

pub fn movl_at_disp_rm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d) << 2;
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, d, 32);
    try store_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
    return false;
}

pub fn movl_rm_at_disp_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rm = load_register(block, ctx, instr.nmd.m);
    const d = bit_manip.zero_extend(instr.nmd.d) << 2;
    try store_mem(block, ctx, instr.nmd.n, .Reg, d, rm, 32);
    return false;
}

pub fn add_rm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = load_register(block, ctx, instr.nmd.m);
    try block.add(.{ .reg = rn }, .{ .reg = rm });
    return false;
}

pub fn add_imm_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.add(.{ .reg = rn }, .{ .imm32 = @bitCast(bit_manip.sign_extension_u8(instr.nd8.d)) });
    return false;
}

pub fn cmphi_Rm_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register(block, ctx, instr.nmd.n);
    const rm = load_register(block, ctx, instr.nmd.m);
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = rn }, .rhs = .{ .reg = rm } } });
    try set_t(block, ctx, .Above);
    return false;
}

pub fn fmovs_at_rm_frn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero => {
            // FRn = [Rm]
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 32);
            try block.mov(get_fp_reg_mem(instr.nmd.n), .{ .reg = ReturnRegister });
        },
        .One => {
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 64);
            try block.mov(get_dfp_reg_mem(instr.nmd.n), .{ .reg = ReturnRegister });
        },
        .Unknown => {
            _ = try interpreter_fallback_cached(block, ctx, instr);
        },
    }
    return false;
}

pub fn fmovs_frm_at_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero => {
            // [Rn] = FRm
            try block.mov(.{ .reg = ReturnRegister }, get_fp_reg_mem(instr.nmd.m));
            try store_mem(block, ctx, instr.nmd.n, .Reg, 0, ReturnRegister, 32);
        },
        .One => {
            try block.mov(.{ .reg = ReturnRegister }, get_dfp_reg_mem(instr.nmd.m));
            try store_mem(block, ctx, instr.nmd.n, .Reg, 0, ReturnRegister, 64);
        },
        .Unknown => {
            _ = try interpreter_fallback_cached(block, ctx, instr);
        },
    }
    return false;
}

pub fn fmovs_at_rm_inc_frn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero, .One => {
            _ = try fmovs_at_rm_frn(block, ctx, instr);
            // Inc Rm
            const rm = load_register_for_writing(block, ctx, instr.nmd.m);
            try block.add(.{ .reg = rm }, .{ .imm32 = if (ctx.fpscr_sz == .One) 8 else 4 });
        },
        .Unknown => {
            _ = try interpreter_fallback_cached(block, ctx, instr);
        },
    }
    return false;
}

pub fn fmovs_frm_at_dec_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero, .One => {
            // Dec Rn
            const rn = load_register_for_writing(block, ctx, instr.nmd.n);
            try block.sub(.{ .reg = rn }, .{ .imm32 = if (ctx.fpscr_sz == .One) 8 else 4 });
            _ = try fmovs_frm_at_rn(block, ctx, instr);
        },
        .Unknown => {
            _ = try interpreter_fallback_cached(block, ctx, instr);
        },
    }
    return false;
}

pub fn fmovs_at_R0_Rm_FRn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero => {
            // FRn = [Rm+R0]
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg_R0, 0, 32);
            try block.mov(get_fp_reg_mem(instr.nmd.n), .{ .reg = ReturnRegister });
        },
        .One => {
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg_R0, 0, 64);
            try block.mov(get_dfp_reg_mem(instr.nmd.n), .{ .reg = ReturnRegister });
        },
        .Unknown => {
            _ = try interpreter_fallback_cached(block, ctx, instr);
        },
    }
    return false;
}

pub fn fmovs_FRm_at_R0_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Zero => {
            // [Rn+R0] = FRm
            try block.mov(.{ .reg = ReturnRegister }, get_fp_reg_mem(instr.nmd.m));
            try store_mem(block, ctx, instr.nmd.n, .Reg_R0, 0, ReturnRegister, 32);
        },
        .One => {
            try block.mov(.{ .reg = ReturnRegister }, get_dfp_reg_mem(instr.nmd.m));
            try store_mem(block, ctx, instr.nmd.n, .Reg_R0, 0, ReturnRegister, 64);
        },
        .Unknown => {
            _ = try interpreter_fallback_cached(block, ctx, instr);
        },
    }
    return false;
}

pub fn lds_rn_FPSCR(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register(block, ctx, instr.nmd.n);
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    try block.mov(.{ .reg = ArgRegisters[1] }, .{ .reg = rn });
    try block.call(sh4.SH4.set_fpscr);
    ctx.fpscr_sz = .Unknown;
    ctx.fpscr_pr = .Unknown;
    return false;
}

pub fn ldsl_at_rn_inc_FPSCR(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // FIXME: This is buggy, and I have no clue why. It (sometimes) causes weird behavior is Soulcalibur. The interpreter fallback doesn't seem to have this issue.
    //        Might be another cache invalidation issue...

    //  try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    //  try load_mem(block, ctx, ArgRegisters[1], instr.nmd.n, 0, 32);
    //  try block.call(sh4.SH4.set_fpscr);
    //  const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    //  try block.add(.{ .reg = rn }, .{ .imm32 = 4 });

    _ = try interpreter_fallback_cached(block, ctx, instr);

    ctx.fpscr_sz = .Unknown;
    ctx.fpscr_pr = .Unknown;
    return false;
}

pub fn fschg(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    _ = try interpreter_fallback_cached(block, ctx, instr);
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
    try store_register(block, ctx, 0, .{ .imm32 = addr });
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
    try block.mov(.{ .reg = ReturnRegister }, .{ .imm64 = abs_addr });
    // Load the pointed value and sign extend it to 32-bits.
    try block.movsx(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = ReturnRegister, .size = 16 } });
    // Store it into Rn
    try store_register(block, ctx, instr.nd8.n, .{ .reg = ReturnRegister });
    return false;
}

pub fn movl_atdispPC_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Adress should be either in Boot ROM, or in RAM.
    std.debug.assert(ctx.address < 0x00200000 or (ctx.address >= 0x0C000000 and ctx.address < 0x10000000));
    // @(d8,PC) is fixed, compute its real absolute address
    const d = bit_manip.zero_extend(instr.nd8.d) << 2;
    const addr = (ctx.address & 0xFFFFFFFC) + 4 + d;
    const abs_addr = @intFromPtr(if (ctx.address < 0x00200000) &ctx.dc.boot[addr] else &ctx.dc.ram[addr & 0x00FFFFFF]);
    // TODO: This could be turned into a single movabs, but emitter doesn't support it yet.
    // Set it to a scratch register
    try block.mov(.{ .reg = ReturnRegister }, .{ .imm64 = abs_addr });
    // Load the pointed value and store it into Rn
    try store_register(block, ctx, instr.nd8.n, .{ .mem = .{ .base = ReturnRegister, .size = 32 } });
    return false;
}

pub fn and_Rm_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = load_register(block, ctx, instr.nmd.m);
    try block.append(.{ .And = .{ .dst = .{ .reg = rn }, .src = .{ .reg = rm } } });
    return false;
}

pub fn and_imm_R0(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const r0 = load_register_for_writing(block, ctx, 0);
    try block.append(.{ .And = .{ .dst = .{ .reg = r0 }, .src = .{ .imm32 = bit_manip.zero_extend(instr.nd8.d) } } });
    return false;
}

pub fn or_Rm_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = load_register(block, ctx, instr.nmd.m);
    try block.append(.{ .Or = .{ .dst = .{ .reg = rn }, .src = .{ .reg = rm } } });
    return false;
}

pub fn or_imm_R0(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const r0 = load_register_for_writing(block, ctx, 0);
    try block.append(.{ .Or = .{ .dst = .{ .reg = r0 }, .src = .{ .imm32 = bit_manip.zero_extend(instr.nd8.d) } } });
    return false;
}

pub fn tst_Rm_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // sr.t = (Rm & Rn) == 0
    if (instr.nmd.n == instr.nmd.m) {
        const rn = load_register(block, ctx, instr.nmd.n);
        try block.append(.{ .Cmp = .{ .lhs = .{ .reg = rn }, .rhs = .{ .imm32 = 0 } } });
    } else {
        const rn = load_register(block, ctx, instr.nmd.n);
        const rm = load_register(block, ctx, instr.nmd.m);
        try block.mov(.{ .reg = ReturnRegister }, .{ .reg = rn });
        try block.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .reg = rm } } });
        try block.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = 0 } } });
    }
    try set_t(block, ctx, .Equal);
    return false;
}

pub fn shll(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = rn }, .rhs = .{ .imm32 = 0x80000000 } } });
    try set_t(block, ctx, .NotCarry); // Equivalent to Above or Equal
    try block.append(.{ .Shl = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 1 } } });
    return false;
}

pub fn shll2(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Shl = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 2 } } });
    return false;
}

pub fn shll8(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Shl = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 8 } } });
    return false;
}

pub fn shll16(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Shl = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 16 } } });
    return false;
}

pub fn shlr(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Shr = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 1 } } });
    // The CF flag contains the value of the last bit shifted out of the destination operand.
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn shlr2(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Shr = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 2 } } });
    return false;
}

pub fn shlr8(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Shr = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 8 } } });
    return false;
}

pub fn shlr16(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Shr = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 16 } } });
    return false;
}

fn conditional_branch(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr, comptime jump_if: bool, comptime delay_slot: bool) !bool {
    try block.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "sr"), .size = 32 } });
    try block.bit_test(ReturnRegister, @bitOffsetOf(sh4.SR, "t"));
    var skip_branch = try block.jmp(if (jump_if) .NotCarry else .Carry);

    const dest = sh4_interpreter.d8_disp(ctx.address, instr);
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .imm32 = dest });

    var to_end = try block.jmp(.Always);
    skip_branch.patch();

    if (delay_slot) {
        ctx.delay_slot = ctx.address + 2;
        // Don't execute delay slot twice when not taking the branch.
        try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .imm32 = ctx.address + 4 });
    } else {
        try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .imm32 = ctx.address + 2 });
    }

    to_end.patch();

    ctx.outdated_pc = false;
    return true;
}

pub fn bf_label(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return conditional_branch(block, ctx, instr, false, false);
}
pub fn bfs_label(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return conditional_branch(block, ctx, instr, false, true);
}
pub fn bt_label(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return conditional_branch(block, ctx, instr, true, false);
}
pub fn bts_label(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return conditional_branch(block, ctx, instr, true, true);
}

pub fn bra_label(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const dest = sh4_interpreter.d12_disp(ctx.address, instr);
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .imm32 = dest });
    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}

pub fn braf_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // pc += Rn + 4;
    const rn = load_register(block, ctx, instr.nmd.n);
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = rn });
    try block.add(.{ .reg = ReturnRegister }, .{ .imm32 = 4 + ctx.address });
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = ReturnRegister });

    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}

pub fn bsr_label(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // pr = pc + 4
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pr"), .size = 32 } }, .{ .imm32 = ctx.address + 4 });
    const dest = sh4_interpreter.d12_disp(ctx.address, instr);
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .imm32 = dest });
    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}

pub fn bsrf_Rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // pr = pc + 4
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pr"), .size = 32 } }, .{ .imm32 = ctx.address + 4 });
    // pc += Rn + 4;
    const rn = load_register(block, ctx, instr.nmd.n);
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = rn });
    try block.add(.{ .reg = ReturnRegister }, .{ .imm32 = 4 + ctx.address });
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = ReturnRegister });

    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}

pub fn jmp_atRn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // pc = Rn
    const rn = load_register(block, ctx, instr.nmd.n);
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = rn });

    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}

pub fn jsr_rn(block: *JITBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // cpu.pr = cpu.pc + 4;
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pr"), .size = 32 } }, .{ .imm32 = ctx.address + 4 });
    // cpu.pc = Rn
    const rn = load_register(block, ctx, instr.nmd.n);
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = rn });

    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}

pub fn rts(block: *JITBlock, ctx: *JITContext, _: sh4.Instr) !bool {
    // cpu.pc = cpu.pr
    try block.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pr"), .size = 32 } });
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = ReturnRegister });

    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}

pub fn rte(block: *JITBlock, ctx: *JITContext, _: sh4.Instr) !bool {
    // We might change GPR banks.
    for (0..8) |i| {
        try ctx.commit_and_invalidate_cached_register(block, @intCast(i));
    }

    // call set_sr
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    try block.mov(.{ .reg = ArgRegisters[1] }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "ssr"), .size = 32 } });
    try block.call(sh4.SH4.set_sr);
    // pc = spc
    try block.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "spc"), .size = 32 } });
    try block.mov(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "pc"), .size = 32 } }, .{ .reg = ReturnRegister });

    ctx.delay_slot = ctx.address + 2;
    ctx.outdated_pc = false;
    return true;
}
