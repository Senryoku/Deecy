const std = @import("std");
const builtin = @import("builtin");
const dc_config = @import("dc_config");

const termcolor = @import("termcolor");

const host_memory = @import("../host/host_memory.zig");

const sh4 = @import("../sh4.zig");
const sh4_interpreter = @import("../sh4_interpreter.zig");
const sh4_disassembly = @import("../sh4_disassembly.zig");
const MMU = @import("../mmu.zig");
const bit_manip = @import("../bit_manip.zig");
const JIT = @import("ir_block.zig");
const IRBlock = JIT.IRBlock;

const Architecture = @import("x86_64.zig");
const ReturnRegister = Architecture.ReturnRegister;
const ScratchRegisters = Architecture.ScratchRegisters;
const ArgRegisters = Architecture.ArgRegisters;
const SavedRegisters = Architecture.SavedRegisters;
const FPSavedRegisters = Architecture.FPSavedRegisters;
const FPScratchRegisters = Architecture.FPScratchRegisters;

pub const BasicBlock = @import("sh4_basic_block.zig");

const sh4_instructions = @import("../sh4_instructions.zig");
const sh4_interpreter_handlers = @import("../sh4_interpreter_handlers.zig");

const sh4_jit_log = std.log.scoped(.sh4_jit);

const Dreamcast = @import("../dreamcast.zig").Dreamcast;

const windows = @import("../host/windows.zig");

const BlockBufferSize = 16 * 1024 * 1024;
const MaxCyclesPerBlock = 32;
pub const FastMem = dc_config.fast_mem; // Keep this option around. Turning FastMem off is sometimes useful for debugging.

// Enable or Disable some optimizations
const Optimizations = .{
    // Allow resetting the block while in the JIT.
    // IMPORTANT: Do not allow this when link_small_blocks is enabled: It would allow jumps
    //            to released blocks (this is somehow fine otherwise, but this might be abusing cache or something)
    .allow_immediate_reset = false,

    .div1_simplification = true,
    .inline_small_forward_jumps = true,
    .inline_jumps_to_start_of_block = false, // This seems worse than 'link_small_blocks' (and even counter productive if 'allow_recursion' is enabled, which make sense)
    .link_small_blocks = .{ .enabled = true, .max_cycles = 16, .allow_recursion = true }, // Experimental. Call the next block directly in the current one is small (typically, a single ret)
    .idle_speedup = true, // Stupid hack: Search for known idle block patterns and add fake cpu cycles to them.
    .inline_backwards_bra = true, // Inlining of backward inconditional branches, before current block entry point. This isn't correctly supported and implementation is very hackish.
};

const VirtualAddressSpace = if (FastMem) switch (builtin.os.tag) {
    .windows => @import("sh4_virtual_address_space_windows.zig"),
    .linux => @import("sh4_virtual_address_space_linux.zig"),
    else => @compileError("FastMem: Unsupported OS."),
} else void;

const BlockCache = struct {
    const RAMEntryCount = Dreamcast.RAMSize;
    const BootEntryCount = Dreamcast.BootSize;
    const RAMMask = RAMEntryCount - 1;
    // 0x02000000 possible addresses (0x0100_0000 for RAM and... 0x0100_0000 for boot), but 16bit aligned, multiplied by permutations of (sz, pr)
    const BlockEntryCount = (0x0200_0000 >> 1) << 2;

    buffer: []align(std.mem.page_size) u8,
    cursor: usize = 0,
    blocks: []BasicBlock = undefined,

    min_address: u32 = 0xFFFFFFFF,
    max_address: u32 = 0,

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        var r: @This() = .{
            .buffer = try host_memory.allocate_executable(allocator, BlockBufferSize),
            ._allocator = allocator,
        };
        try r.allocate_blocks();
        return r;
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.free(self.buffer);

        self.deallocate_blocks();
    }

    fn allocate_blocks(self: *@This()) !void {
        self.blocks = try host_memory.virtual_alloc(BasicBlock, BlockEntryCount);
    }

    fn deallocate_blocks(self: *@This()) void {
        host_memory.virtual_dealloc(self.blocks);
        self.blocks = &[0]BasicBlock{};
    }

    pub fn reset(self: *@This()) !void {
        if (self.cursor == 0) return;

        self.cursor = 0;
        self.min_address = 0xFFFFFFFF;
        self.max_address = 0;

        self.deallocate_blocks();
        try self.allocate_blocks();
    }

    pub fn invalidate(self: *@This(), start_addr: u32, end_addr: u32) void {
        if (start_addr > self.max_address or end_addr < self.min_address) return;
        sh4_jit_log.info("Invalidating {X:0>8}..{X:0>8}", .{ start_addr, end_addr });

        inline for (0..2) |sz| {
            inline for (0..2) |pr| {
                // Addresses are used in the low bits of keys, so this should be contiguous.
                comptime {
                    std.debug.assert(compute_key(0, sz, pr) == compute_key(1, sz, pr)); // 16bits aligned
                    std.debug.assert(compute_key(0, sz, pr) + 1 == compute_key(2, sz, pr)); // contiguous
                }
                const start = compute_key(@intCast(start_addr), sz, pr);
                const end = compute_key(@intCast(end_addr), sz, pr);
                @memset(self.blocks[start..end], .{ .offset = 0 });
            }
        }
    }

    // NOTE: I could save a shift in compute_key_from_fpscr by moving the ram bit after
    //       pr and sz, but it was measurably slower in my testing.
    const Key = packed struct(u32) {
        addr: u23,
        ram: u1,
        pr: u1,
        sz: u1,
        _: u6 = 0,
    };

    inline fn compute_key(address: u32, sz: u1, pr: u1) u32 {
        return @bitCast(Key{
            .addr = @truncate(address >> 1),
            .ram = @truncate(address >> 26),
            .pr = pr,
            .sz = sz,
        });
    }

    pub inline fn get(self: *@This(), address: u32, sz: u1, pr: u1) *BasicBlock {
        return &self.blocks[compute_key(address, sz, pr)];
    }

    pub fn put(self: *@This(), address: u32, sz: u1, pr: u1, block: BasicBlock) void {
        self.min_address = @min(self.min_address, address);
        self.max_address = @max(self.max_address, address);
        self.blocks[compute_key(address, sz, pr)] = block;
    }
};

inline fn instr_lookup(instr: u16) sh4_instructions.OpcodeDescription {
    return sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr]];
}

const FPPrecision = enum(u2) {
    Single = 0,
    Double = 1,
    Unknown,
};

fn RegisterCache(comptime reg_type: type, comptime entries: u8) type {
    return struct {
        highest_saved_register_used: ?u8 = null,
        entries: [entries]struct {
            host: reg_type,
            size: u8 = 32,
            last_access: u32 = 0,
            modified: bool = false,
            guest: ?u4 = null,

            pub fn load(self: *@This(), block: *IRBlock) !void {
                if (self.guest) |guest_reg| {
                    try block.mov(
                        switch (reg_type) {
                            JIT.Register => .{ .reg = self.host },
                            JIT.FPRegister => switch (self.size) {
                                32 => .{ .freg32 = self.host },
                                64 => .{ .freg64 = self.host },
                                else => @panic("Invalid floating point register size."),
                            },
                            else => @compileError("Invalid register type."),
                        },
                        JITContext.guest_reg_memory(reg_type, self.size, guest_reg),
                    );
                }
            }

            pub fn commit(self: *@This(), block: *IRBlock) !void {
                if (self.guest) |guest_reg| {
                    if (self.modified) {
                        try block.mov(
                            JITContext.guest_reg_memory(reg_type, self.size, guest_reg),
                            switch (reg_type) {
                                JIT.Register => .{ .reg = self.host },
                                JIT.FPRegister => switch (self.size) {
                                    32 => .{ .freg32 = self.host },
                                    64 => .{ .freg64 = self.host },
                                    else => @panic("Invalid floating point register size."),
                                },
                                else => @compileError("Invalid register type."),
                            },
                        );
                        self.modified = false;
                    }
                }
            }

            pub fn commit_and_invalidate(self: *@This(), block: *IRBlock) !void {
                try self.commit(block);
                self.guest = null;
            }
        },

        fn get_cached_register(self: *@This(), size: u8, guest_reg: u4) ?*@TypeOf(self.entries[0]) {
            for (&self.entries) |*reg| {
                if (reg.guest) |r| {
                    if (r == guest_reg and reg.size == size) {
                        return reg;
                    }
                }
            }
            return null;
        }

        pub fn commit_and_invalidate_all(self: *@This(), block: *IRBlock) !void {
            for (&self.entries) |*reg| {
                try reg.commit(block);
                reg.guest = null;
            }

            // In this case all registers are temporary, we can forget we ever used them.
            if (Architecture.JITABI == .SystemV and reg_type == JIT.FPRegister) {
                self.highest_saved_register_used = null;
            }
        }

        pub fn commit_all(self: *@This(), block: *IRBlock) !void {
            for (&self.entries) |*reg| {
                try reg.commit(block);
            }
        }

        // NOTE: The following functions are only used by the GPR cache, we assume a size of 32bit!

        // Means this will be overwritten outside of the JIT
        pub fn invalidate(self: *@This(), guest_reg: u4) !void {
            std.debug.assert(reg_type == JIT.Register);
            if (self.get_cached_register(32, guest_reg)) |reg| {
                reg.guest = null;
            }
        }
        pub fn commit(self: *@This(), block: *IRBlock, guest_reg: u4) !void {
            std.debug.assert(reg_type == JIT.Register);
            if (self.get_cached_register(32, guest_reg)) |reg| {
                try reg.commit(block);
            }
        }
        pub fn commit_and_invalidate(self: *@This(), block: *IRBlock, guest_reg: u4) !void {
            std.debug.assert(reg_type == JIT.Register);
            if (self.get_cached_register(32, guest_reg)) |reg|
                try reg.commit_and_invalidate(block);
        }
    };
}

pub const JITContext = struct {
    cpu: *sh4.SH4,
    entry_point_address: u32,

    instructions: [*]u16 = undefined,

    // Jitted branches do not need to increment the PC manually.
    outdated_pc: bool = true,
    may_have_pending_cycles: bool = false,
    jumps_to_end: [32]?JIT.PatchableJump = .{null} ** 32,

    mmu_enabled: bool,
    start_pc: u32, // Address of the first instruction in the instructions array.
    start_physical_pc: u32, // Physical address of the first instruction in the instructions array.
    start_index: u32 = undefined, // Index in the block corresponding to the first guest instruction.

    current_pc: u32, // Address of the current instruction (actual PC).
    current_physical_pc: u32, // Physical address of the current instruction.
    index: u32 = 0, // Index in the instructions array of the current instruction.
    cycles: u32 = 0,

    fpscr_sz: FPPrecision,
    fpscr_pr: FPPrecision,

    in_delay_slot: bool = false,

    gpr_cache: RegisterCache(JIT.Register, if (Architecture.JITABI == .Win64) 5 else 4) = .{
        .highest_saved_register_used = 0,
        .entries = if (Architecture.JITABI == .Win64) .{
            .{ .host = SavedRegisters[1] },
            .{ .host = SavedRegisters[2] },
            .{ .host = SavedRegisters[3] },
            .{ .host = SavedRegisters[4] },
            .{ .host = SavedRegisters[5] },
        } else .{
            .{ .host = SavedRegisters[1] },
            .{ .host = SavedRegisters[2] },
            .{ .host = SavedRegisters[3] },
            .{ .host = SavedRegisters[4] },
        },
    },

    fpr_cache: RegisterCache(JIT.FPRegister, 8) = .{
        .highest_saved_register_used = null,
        .entries = if (Architecture.JITABI == .Win64) .{
            .{ .host = FPSavedRegisters[0] },
            .{ .host = FPSavedRegisters[1] },
            .{ .host = FPSavedRegisters[2] },
            .{ .host = FPSavedRegisters[3] },
            .{ .host = FPSavedRegisters[4] },
            .{ .host = FPSavedRegisters[5] },
            .{ .host = FPSavedRegisters[6] },
            .{ .host = FPSavedRegisters[7] },
        } else .{
            .{ .host = .xmm6 },
            .{ .host = .xmm7 },
            .{ .host = .xmm8 },
            .{ .host = .xmm9 },
            .{ .host = .xmm10 },
            .{ .host = .xmm11 },
            .{ .host = .xmm12 },
            .{ .host = .xmm13 },
        },
    },

    pub fn init(cpu: *sh4.SH4) @This() {
        const physical_pc = if (sh4.ExperimentalFullMMUSupport)
            cpu.translate_intruction_address(cpu.pc)
        else
            cpu.pc & 0x1FFF_FFFF;

        if (!((physical_pc >= 0x00000000 and physical_pc < 0x00020000) or (physical_pc >= 0x0C000000 and physical_pc < 0x10000000)))
            std.debug.panic("Invalid physical PC: {X}", .{physical_pc});

        return .{
            .cpu = cpu,
            .entry_point_address = cpu.pc,
            .mmu_enabled = sh4.ExperimentalFullMMUSupport and cpu._mmu_enabled,
            .start_pc = cpu.pc,
            .start_physical_pc = physical_pc,
            .current_pc = cpu.pc,
            .current_physical_pc = physical_pc,
            .instructions = @alignCast(@ptrCast(cpu._get_memory(physical_pc))),
            .fpscr_sz = if (cpu.fpscr.sz == 1) .Double else .Single,
            .fpscr_pr = if (cpu.fpscr.pr == 1) .Double else .Single,
        };
    }

    // Unconditional forward jump
    pub fn skip_instructions(self: *@This(), count: u32) void {
        self.index += count;
        self.current_pc += 2 * count;
        self.current_physical_pc += 2 * count;
    }

    pub fn compile_delay_slot(self: *@This(), b: *IRBlock) !void {
        self.in_delay_slot = true;
        defer self.in_delay_slot = false;

        const instr = self.instructions[self.index + 1];
        const op = instr_lookup(instr);
        if (comptime std.log.logEnabled(.debug, .sh4_jit))
            sh4_jit_log.debug("[{X:0>8}] {s} \\ {s}", .{
                self.current_pc,
                if (op.use_fallback()) "!" else " ",
                try sh4_disassembly.disassemble(@bitCast(instr), self.cpu._allocator),
            });
        self.current_physical_pc += 2; // Same thing as in the interpreter, this is probably useless as instructions referring to PC should be illegal here, but just in case...
        self.current_pc += 2;
        const branch = try op.jit_emit_fn(b, self, @bitCast(instr));
        std.debug.assert(!branch);
        self.current_pc -= 2;
        self.current_physical_pc -= 2;
        self.cycles += op.issue_cycles;
    }

    pub fn guest_reg_memory(comptime reg_type: type, size: u8, guest_reg: u4) JIT.Operand {
        std.debug.assert(size == 32 or size == 64);
        switch (reg_type) {
            JIT.Register => return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "r") + @as(u32, guest_reg) * 4, .size = size } },
            JIT.FPRegister => {
                if (size == 32) {
                    return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "fp_banks") + @as(u32, guest_reg) * 4, .size = size } };
                } else {
                    const bank: u32 = if ((guest_reg & 1) == 1) @sizeOf([16]f32) else 0;
                    return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "fp_banks") + bank + @as(u32, guest_reg >> 1) * 8, .size = 64 } };
                }
            },
            else => @compileError("Invalid register type."),
        }
    }

    fn guest_register_cache(self: *@This(), comptime reg_type: type, size: u8, block: *IRBlock, guest_reg: u4, comptime load: bool, comptime modified: bool) !reg_type {
        const cache = switch (reg_type) {
            JIT.Register => &self.gpr_cache,
            JIT.FPRegister => &self.fpr_cache,
            else => @compileError("Invalid register type."),
        };

        for (&cache.entries) |*hreg| {
            hreg.last_access += 1;
        }

        // Is it already cached?
        if (cache.get_cached_register(size, guest_reg)) |hreg| {
            hreg.last_access = 0;
            if (modified) hreg.modified = true;
            return hreg.host;
        }

        // FP registers aliasing check! FIXME: This can probably be improved a lot.
        // If we're asking a register while we already have part of it cached in a different size, invalidate it.
        if (reg_type == JIT.FPRegister) {
            if (size == 64) {
                for (&cache.entries) |*reg| {
                    if (reg.guest) |r| {
                        // Search of both halves.
                        if (r == (guest_reg & 0xE) or (r == (guest_reg | 1)) and reg.size == 32) {
                            try reg.commit_and_invalidate(block);
                        }
                    }
                }
            } else {
                for (&cache.entries) |*reg| {
                    if (reg.guest) |r| {
                        if ((r == (guest_reg & 0xE)) and reg.size == 64) {
                            try reg.commit_and_invalidate(block);
                        }
                    }
                }
            }
        }

        var slot = s: {
            var lru = &cache.entries[0];
            // Do we have a free slot?
            for (&cache.entries, 0..) |*hreg, idx| {
                if (hreg.guest == null) {
                    cache.highest_saved_register_used = @max(cache.highest_saved_register_used orelse 0, @as(u8, @intCast(idx)));
                    break :s hreg;
                }
                if (hreg.last_access > lru.last_access) {
                    lru = hreg;
                }
            }
            // No. Commit to memory and evict the oldest.
            if (lru.modified)
                try lru.commit(block);
            break :s lru;
        };

        slot.guest = guest_reg;
        slot.last_access = 0;
        slot.modified = modified;
        slot.size = size;
        if (load)
            try slot.load(block);
        return slot.host;
    }

    // load: If not already in cache, will load the value from memory.
    // modified: Mark the cached value as modified, ensuring it will be written back to memory.
    pub fn guest_reg_cache(self: *@This(), block: *IRBlock, guest_reg: u4, comptime load: bool, comptime modified: bool) !JIT.Register {
        return try self.guest_register_cache(JIT.Register, 32, block, guest_reg, load, modified);
    }

    pub fn guest_freg_cache(self: *@This(), block: *IRBlock, comptime size: u8, guest_reg: u4, comptime load: bool, comptime modified: bool) !JIT.FPRegister {
        return try self.guest_register_cache(JIT.FPRegister, size, block, guest_reg, load, modified);
    }

    pub fn add_jump_to_end(self: *@This(), jmp: JIT.PatchableJump) !void {
        var idx: usize = 0;
        while (idx < self.jumps_to_end.len and self.jumps_to_end[idx] != null) : (idx += 1) {}
        if (idx >= self.jumps_to_end.len)
            return error.TooManyJumpsToEndOfBlock;
        self.jumps_to_end[idx] = jmp;
    }
};

pub const SH4JIT = struct {
    block_cache: BlockCache,

    virtual_address_space: VirtualAddressSpace = undefined,

    /// Used to delay reset AFTER the current block is done excecuting.
    _reset_requested: if (Optimizations.allow_immediate_reset) void else bool = if (Optimizations.allow_immediate_reset) {} else false,

    _working_block: IRBlock,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) !@This() {
        var r: @This() = .{
            .block_cache = try .init(allocator),
            ._working_block = try .init(allocator),
            ._allocator = allocator,
        };
        if (FastMem)
            r.virtual_address_space = try .init(allocator);
        try r.init_compile_and_run_handler();
        return r;
    }

    pub fn deinit(self: *@This()) void {
        self._working_block.deinit();
        self.block_cache.deinit();
        if (FastMem)
            self.virtual_address_space.deinit();
    }

    pub fn invalidate(self: *@This(), start_addr: u32, end_addr: u32) void {
        self.block_cache.invalidate(start_addr, end_addr);
    }

    fn init_compile_and_run_handler(self: *@This()) !void {
        std.debug.assert(self.block_cache.cursor == 0);
        var b = &self._working_block;
        b.clearRetainingCapacity();
        try b.mov(.{ .reg64 = ArgRegisters[1] }, .{ .imm64 = @intFromPtr(self) });
        try b.call(compile_and_run);
        const block_size = try b.emit(self.block_cache.buffer[0..]);
        self.block_cache.cursor = std.mem.alignForward(usize, block_size, 0x10);
    }

    pub fn request_reset(self: *@This()) void {
        sh4_jit_log.debug("Reset requested.", .{});
        if (Optimizations.allow_immediate_reset) {
            // Delaying is safer, but it works without it, and it's faster...
            self.reset() catch |err| {
                sh4_jit_log.err("Failed to reset JIT: {}", .{err});
            };
        } else {
            self._reset_requested = true;
        }
    }

    pub fn reset(self: *@This()) !void {
        try self.block_cache.reset();
        try self.init_compile_and_run_handler();
    }

    pub fn execute(self: *@This(), cpu: *sh4.SH4) !u32 {
        if (!Optimizations.allow_immediate_reset and self._reset_requested) {
            @branchHint(.cold);
            self._reset_requested = false;
            sh4_jit_log.warn("Executing requested reset.", .{});
            try self.reset();
        }

        cpu.handle_interrupts();

        if (cpu.execution_state == .Running or cpu.execution_state == .ModuleStandby) {
            @branchHint(.likely);
            std.debug.assert((cpu.pc & 0xFC00_0000) != 0x7C00_0000);
            const physical_pc = cpu.translate_intruction_address(cpu.pc);
            var block = self.block_cache.get(physical_pc, cpu.fpscr.sz, cpu.fpscr.pr);

            const start = if (BasicBlock.EnableInstrumentation) std.time.nanoTimestamp() else {}; // Make sure this isn't called when instrumentation is disabled.

            const cycles = block.execute(self.block_cache.buffer, cpu);

            if (BasicBlock.EnableInstrumentation and block.offset > 0) { // Might have been invalidated.
                block.time_spent += std.time.nanoTimestamp() - start;
                block.call_count += 1;
            }

            return cycles;
        } else {
            @branchHint(.cold);
            return 8;
        }
    }

    // Default handler sitting the offset 0 of our executable buffer
    pub noinline fn compile_and_run(cpu: *sh4.SH4, self: *@This()) u32 {
        sh4_jit_log.info("(Cache Miss) Compiling {X:0>8} (SZ={d}, PR={d})...", .{ cpu.pc, cpu.fpscr.sz, cpu.fpscr.pr });

        const block = self.compile(.init(cpu)) catch |err| retry: {
            if (err == error.JITCacheFull) {
                sh4_jit_log.warn("JIT cache full: Resetting.", .{});
                self.reset() catch |reset_err| {
                    sh4_jit_log.err("Failed to reset JIT: {s}", .{@errorName(reset_err)});
                    std.process.exit(1);
                };
                break :retry self.compile(.init(cpu));
            } else break :retry err;
        } catch |err| {
            sh4_jit_log.err("Failed to compile {X:0>8}: {s}\n", .{ cpu.pc, @errorName(err) });
            std.process.exit(1);
        };
        return block.execute(self.block_cache.buffer, cpu);
    }

    pub noinline fn compile(self: *@This(), start_ctx: JITContext) !*BasicBlock {
        var ctx = start_ctx;

        var b = &self._working_block;

        b.clearRetainingCapacity();

        // We'll be using these callee saved registers, push 'em to the stack.
        try b.push(.{ .reg = SavedRegisters[0] });
        try b.push(.{ .reg = SavedRegisters[1] }); // NOTE: We need to align the stack to 16 bytes. Used in load_mem().

        const optional_saved_register_offset = b.instructions.items.len;
        // We'll turn those into NOP if they're not used.
        try b.push(.{ .reg = SavedRegisters[2] });
        try b.push(.{ .reg = SavedRegisters[3] });
        if (SavedRegisters.len == 5) {
            // Preserve alignment
            try b.push(.{ .reg = SavedRegisters[4] });
            try b.push(.{ .reg = SavedRegisters[4] });
        } else if (SavedRegisters.len >= 6) {
            try b.push(.{ .reg = SavedRegisters[4] });
            try b.push(.{ .reg = SavedRegisters[5] });
        }

        // Save some space for potential callee-saved FP registers
        const optional_saved_fp_register_offset = b.instructions.items.len;
        try b.append(.Nop);

        if (FastMem) {
            const addr_space: u64 = @intFromPtr(self.virtual_address_space.base_addr());
            try b.mov(.{ .reg64 = .rbp }, .{ .imm64 = addr_space }); // Provide a pointer to the base of the virtual address space
        } else {
            const ram_addr: u64 = @intFromPtr(ctx.cpu._dc.?.ram.ptr);
            try b.mov(.{ .reg64 = .rbp }, .{ .imm64 = ram_addr }); // Provide a pointer to the SH4's RAM
        }
        try b.mov(.{ .reg = SavedRegisters[0] }, .{ .reg = ArgRegisters[0] }); // Save the pointer to the SH4

        ctx.start_index = @intCast(b.instructions.items.len);

        var branch = false;
        while (true) {
            try self.simplify_div(b, &ctx);

            const instr = ctx.instructions[ctx.index];
            const op = instr_lookup(instr);
            if (comptime std.log.logEnabled(.debug, .sh4_jit))
                sh4_jit_log.debug("[{X:0>8}] {s} {s}", .{
                    ctx.current_pc,
                    if (op.use_fallback()) "!" else " ",
                    try sh4_disassembly.disassemble(@bitCast(instr), self._allocator),
                });
            branch = try op.jit_emit_fn(b, &ctx, @bitCast(instr));

            ctx.cycles += op.issue_cycles;
            ctx.index += 1;
            ctx.current_pc += 2;
            ctx.current_physical_pc += 2;

            if (branch or ctx.cycles >= MaxCyclesPerBlock)
                break;
        }

        // Crude appromixation, better purging slightly too often than crashing.
        // Also feels better than checking the length at each insertion.
        if (self.block_cache.cursor + 4 * 32 + 8 * 4 * b.instructions.items.len >= BlockBufferSize) {
            return error.JITCacheFull;
        }

        // We still rely on the interpreter implementation of the branch instructions which expects the PC to be updated automatically.
        // cpu.pc += 2;
        if (!branch) {
            try b.mov(sh4_mem("pc"), .{ .imm32 = ctx.current_pc });
        } else if (ctx.outdated_pc) {
            try b.add(sh4_mem("pc"), .{ .imm32 = 2 });
        }

        try ctx.gpr_cache.commit_and_invalidate_all(b);
        try ctx.fpr_cache.commit_and_invalidate_all(b);

        try self.idle_speedup(&ctx);

        for (ctx.jumps_to_end) |jmp|
            if (jmp) |j| j.patch();

        if (Optimizations.link_small_blocks.enabled and ctx.cycles <= Optimizations.link_small_blocks.max_cycles and ctx.fpscr_pr == start_ctx.fpscr_pr and ctx.fpscr_sz == start_ctx.fpscr_sz and !ctx.mmu_enabled) {
            ctx.may_have_pending_cycles = true;
            const const_key = BlockCache.Key{
                .addr = 0,
                .ram = 0,
                .pr = if (ctx.fpscr_pr == .Single) 0 else 1,
                .sz = if (ctx.fpscr_sz == .Single) 0 else 1,
            };
            const Key: JIT.Operand = .{ .reg = ArgRegisters[0] };
            try b.mov(.{ .reg = ReturnRegister }, sh4_mem("_pending_cycles"));
            try b.add(.{ .reg = ReturnRegister }, .{ .imm32 = ctx.cycles });
            try b.mov(sh4_mem("_pending_cycles"), .{ .reg = ReturnRegister });
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = 66 } } });
            var skip = try b.jmp(.AboveEqual); // Avoid cycles of small blocks

            try b.mov(Key, sh4_mem("pc"));
            var skip_recursion = if (!Optimizations.link_small_blocks.allow_recursion) s: {
                try b.append(.{ .Cmp = .{ .lhs = Key, .rhs = .{ .imm32 = ctx.start_pc } } }); // Forbid recursive calls.
                break :s try b.jmp(.Equal);
            } else {};

            // Compute block key
            try b.mov(.{ .reg = ReturnRegister }, Key);
            try b.shr(.{ .reg = ReturnRegister }, 3);
            try b.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = 0x80_0000 } } });
            try b.shr(Key, 1);
            try b.append(.{ .And = .{ .dst = Key, .src = .{ .imm32 = 0x7F_FFFF } } });
            try b.append(.{ .Or = .{ .dst = Key, .src = .{ .reg = ReturnRegister } } });
            try b.append(.{ .Or = .{ .dst = Key, .src = .{ .imm32 = @bitCast(const_key) } } });
            // Retrieve offset
            if (@sizeOf(BasicBlock) != 4)
                @compileError("TODO: Implement with instrumentation enabled (or disable the optimisation)");
            try b.mov(.{ .reg64 = ReturnRegister }, .{ .imm64 = @intFromPtr(self.block_cache.blocks.ptr) });
            try b.mov(.{ .reg = ArgRegisters[0] }, .{ .mem = .{ .base = ReturnRegister, .index = Key.reg, .scale = ._4, .size = 32 } });
            try b.mov(.{ .reg64 = ReturnRegister }, .{ .imm64 = @intFromPtr(self.block_cache.buffer.ptr) });
            try b.add(.{ .reg64 = ReturnRegister }, .{ .reg64 = ArgRegisters[0] });
            try b.mov(.{ .reg64 = ArgRegisters[0] }, .{ .reg64 = SavedRegisters[0] });
            try b.call(null);
            // ReturnRegister holds the cycle count

            if (!Optimizations.link_small_blocks.allow_recursion)
                skip_recursion.patch();

            skip.patch();
        } else {
            try b.mov(.{ .reg = ReturnRegister }, .{ .imm32 = ctx.cycles });
        }

        if (ctx.may_have_pending_cycles) {
            try b.add(.{ .reg = ReturnRegister }, sh4_mem("_pending_cycles"));
            try b.append(.{ .Mov = .{ .dst = sh4_mem("_pending_cycles"), .src = .{ .imm32 = 0 }, .preserve_flags = false } });
        }

        if (Architecture.JITABI == .Win64) {
            // Save and restore XMM registers as needed (they're all 16 bytes, so no need to worry about alignment).
            if (ctx.fpr_cache.highest_saved_register_used) |highest_saved_register_used| {
                b.instructions.items[optional_saved_fp_register_offset] = .{ .SaveFPRegisters = .{
                    .count = @intCast(highest_saved_register_used + 1),
                } };
                try b.append(.{ .RestoreFPRegisters = .{
                    .count = @intCast(highest_saved_register_used + 1),
                } });
            }
        }

        // Restore callee saved registers.
        const highest_saved_gpr_used = ctx.gpr_cache.highest_saved_register_used.?;
        if (SavedRegisters.len == 5) {
            if (highest_saved_gpr_used >= 3) {
                try b.pop(.{ .reg = SavedRegisters[4] });
                try b.pop(.{ .reg = SavedRegisters[4] });
            } else {
                b.instructions.items[optional_saved_register_offset + 2] = .Nop;
                b.instructions.items[optional_saved_register_offset + 3] = .Nop;
            }
        } else if (SavedRegisters.len >= 6) {
            if (highest_saved_gpr_used >= 3) {
                try b.pop(.{ .reg = SavedRegisters[5] });
                try b.pop(.{ .reg = SavedRegisters[4] });
            } else {
                b.instructions.items[optional_saved_register_offset + 2] = .Nop;
                b.instructions.items[optional_saved_register_offset + 3] = .Nop;
            }
        }
        if (highest_saved_gpr_used >= 1) {
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

        const block_size = try b.emit(self.block_cache.buffer[self.block_cache.cursor..]);
        var block = BasicBlock{ .offset = @intCast(self.block_cache.cursor) };
        if (BasicBlock.EnableInstrumentation) {
            block.cycles = ctx.cycles;
            block.start_addr = ctx.entry_point_address;
            block.len = ctx.index;
        }
        self.block_cache.cursor += block_size;
        // Align next block to 16 bytes. Not necessary, but might give a very small performance boost.
        self.block_cache.cursor = std.mem.alignForward(usize, self.block_cache.cursor, 0x10);

        sh4_jit_log.debug("Compiled: {X:0>2}", .{self.block_cache.buffer[block.offset..][0..block_size]});

        self.block_cache.put(start_ctx.start_physical_pc, @truncate(@intFromEnum(start_ctx.fpscr_sz)), @truncate(@intFromEnum(start_ctx.fpscr_pr)), block);
        return self.block_cache.get(start_ctx.start_physical_pc, @truncate(@intFromEnum(start_ctx.fpscr_sz)), @truncate(@intFromEnum(start_ctx.fpscr_pr)));
    }

    // Try to match some common division patterns and replace them with a "single" instruction.
    fn simplify_div(self: *@This(), b: *IRBlock, ctx: *JITContext) !void {
        if (!Optimizations.div1_simplification) return;

        _ = self;
        // FIXME: Very experimental. Very ugly. This is also not 100% accurate as it misses a bunch of side effects of the division.
        //        Tested by looking at the timer in Soulcalibur :^)
        if (instr_lookup(ctx.instructions[ctx.index]).fn_ == sh4_interpreter.div0u) {
            var index: u32 = 1;
            var cycles: u32 = instr_lookup(ctx.instructions[ctx.index]).issue_cycles;
            var rotcl_Rn_div1_count: u32 = 0;
            while (true) {
                if (instr_lookup(ctx.instructions[ctx.index + index]).fn_ != sh4_interpreter.rotcl_Rn) break;
                index += 1;
                cycles += instr_lookup(ctx.instructions[ctx.index + index]).issue_cycles;
                if (instr_lookup(ctx.instructions[ctx.index + index]).fn_ != sh4_interpreter.div1) break;
                index += 1;
                cycles += instr_lookup(ctx.instructions[ctx.index + index]).issue_cycles;
                rotcl_Rn_div1_count += 1;
            }
            if (rotcl_Rn_div1_count == 32 and (index % 2) == 0) {
                const dividend_low = (sh4.Instr{ .value = ctx.instructions[ctx.index + 1] }).nmd.n;
                const dividend_high = (sh4.Instr{ .value = ctx.instructions[ctx.index + 2] }).nmd.n;
                const divisor = (sh4.Instr{ .value = ctx.instructions[ctx.index + 2] }).nmd.m;

                const dl = try load_register_for_writing(b, ctx, dividend_low);
                const dh = try load_register(b, ctx, dividend_high);
                const div = try load_register(b, ctx, divisor);

                try b.append(.{ .Div64_32 = .{ .dividend_high = dh, .dividend_low = dl, .divisor = div, .result = dl } });

                ctx.skip_instructions(index);
                ctx.cycles += cycles;
                return;
            }
        }
        if (instr_lookup(ctx.instructions[ctx.index]).fn_ == sh4_interpreter.div0s_Rm_Rn) {
            // TODO: Or not. This case seems messier?
        }
    }

    fn idle_speedup(self: *@This(), ctx: *JITContext) !void {
        if (!Optimizations.idle_speedup) return;

        const AddedCycles = 512;

        _ = self;

        const Blocks = [_][]const u16{
            // Boot ROM (Waiting on VSync by hammering SPG_STATUS)
            //   mov.l @R5, R2
            //   tst R4, R2
            //   bt -4
            &[_]u16{ 0x6252, 0x2248, 0x89FC },
            // Boot ROM (Same instructions as Soul Calibur, but using other addresses/registers)
            &[_]u16{
                0xD223, 0x6322, 0x430B, 0x5421,
                0xD022, 0x6302, 0xD222, 0x6122,
                0x313C, 0xD21D, 0x7101, 0x6322,
                0x3316, 0x8B01, 0xA004, 0xEE01,
                0xD31E, 0x6232, 0x2228, 0x8BEB,
            },
            // Soul Calibur (it will actually end up split into multiple basic blocks, I included it all to be sure not to have false positives)
            &[_]u16{
                0xD321, 0x6232, 0x420B, 0x5431,
                0xD120, 0x6312, 0xD020, 0x6202,
                0xD11B, 0x323C, 0x7201, 0x6312,
                0x3326, 0x8B01, 0xA004, 0x6CE3,
                0xD318, 0x6232, 0x2228, 0x8BEB,
            },
        };

        for (Blocks) |block| next_block: {
            var i: u32 = 0;
            for (block) |instr| {
                if (instr != ctx.instructions[i]) {
                    break :next_block;
                }
                i += 1;
            }
            sh4_jit_log.debug("Detected Idle Block at 0x{X:0>8}", .{ctx.current_pc});
            ctx.cycles += AddedCycles; // Add an arbritrary number of cycles.
            return;
        }
    }
};

fn call(block: *IRBlock, ctx: *JITContext, func: *const anyopaque) !void {
    if (Architecture.JITABI == .SystemV) {
        if (ctx.fpr_cache.highest_saved_register_used) |highest_saved_register_used| {
            try block.append(.{ .SaveFPRegisters = .{
                .count = highest_saved_register_used + 1,
            } });
        }
    }

    try block.call(func);

    if (Architecture.JITABI == .SystemV) {
        if (ctx.fpr_cache.highest_saved_register_used) |highest_saved_register_used| {
            try block.append(.{ .RestoreFPRegisters = .{
                .count = highest_saved_register_used + 1,
            } });
        }
    }
}

/// Helper function to ignore possible exceptions thrown by the interpreter fallback.
//  (It's complicated to call zig functions with error handling from the JIT, wrapping it in a function that can't return an error makes it way easier).
fn InterpreterFallback(comptime instr_index: u8) type {
    const entry = sh4_instructions.Opcodes[instr_index];
    return struct {
        pub fn handler(cpu: *sh4.SH4, instr: sh4.Instr) void {
            entry.fn_(cpu, instr) catch |err| {
                std.debug.panic("Interpreter fallback to {s} generated an exception: {s}", .{ entry.name, @errorName(err) });
            };
        }
    };
}

inline fn call_interpreter_fallback(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !void {
    if (sh4_interpreter_handlers.Enable) {
        try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
        try call(block, ctx, sh4_interpreter_handlers.InstructionHandlers[instr.value]);
    } else {
        try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
        try block.mov(.{ .reg64 = ArgRegisters[1] }, .{ .imm64 = @as(u16, @bitCast(instr)) });
        const instr_index = sh4_instructions.JumpTable[instr.value];
        switch (instr_index) {
            sh4_instructions.Opcodes.len...std.math.maxInt(u8) => std.debug.panic("Invalid instruction: {X:0>4}", .{instr.value}),
            inline else => |idx| try call(block, ctx, InterpreterFallback(idx).handler),
        }
        // try call(block, ctx, sh4_instructions.Opcodes[sh4_instructions.JumpTable[instr.value]].fn_);
    }
}

// We need pointers to all of these functions, can't really refactor that mess sadly.

pub fn interpreter_fallback_cached(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    var cache_access = sh4_instructions.Opcodes[sh4_instructions.JumpTable[@as(u16, @bitCast(instr))]].access;

    // We don't want to invalidate or commit the same register twice.

    if (instr.nmd.n != 0 and instr.nmd.m != 0) {
        if (cache_access.r.r0 and cache_access.w.r0) {
            try ctx.gpr_cache.commit_and_invalidate(block, 0);
        } else if (cache_access.w.r0) {
            try ctx.gpr_cache.invalidate(0);
        } else if (cache_access.r.r0) {
            try ctx.gpr_cache.commit(block, 0);
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
        try ctx.gpr_cache.commit_and_invalidate(block, instr.nmd.n);
    } else if (cache_access.w.rn) {
        try ctx.gpr_cache.invalidate(instr.nmd.n);
    } else if (cache_access.r.rn) {
        try ctx.gpr_cache.commit(block, instr.nmd.n);
    }

    if (instr.nmd.m != instr.nmd.n) {
        if (cache_access.r.rm and cache_access.w.rm) {
            try ctx.gpr_cache.commit_and_invalidate(block, instr.nmd.m);
        } else if (cache_access.w.rm) {
            try ctx.gpr_cache.invalidate(instr.nmd.m);
        } else if (cache_access.r.rm) {
            try ctx.gpr_cache.commit(block, instr.nmd.m);
        }
    }

    if (cache_access.r.pc) {
        try block.mov(sh4_mem("pc"), .{ .imm32 = ctx.current_pc });
    }

    try ctx.fpr_cache.commit_and_invalidate_all(block); // FIXME: Hopefully we'll be able to remove this soon!

    try call_interpreter_fallback(block, ctx, instr);
    return false;
}

pub fn interpreter_fallback(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try ctx.gpr_cache.commit_and_invalidate_all(block);
    try ctx.fpr_cache.commit_and_invalidate_all(block);
    try call_interpreter_fallback(block, ctx, instr);
    return false;
}

pub fn interpreter_fallback_branch(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Restore PC in memory.
    try block.mov(sh4_mem("pc"), .{ .imm32 = ctx.current_pc });
    _ = try interpreter_fallback(block, ctx, instr);
    return true;
}

pub fn nop(_: *IRBlock, _: *JITContext, _: sh4.Instr) !bool {
    return false;
}

// NOTE: Ideally we'd use the type system to ensure the return values of the two following functions are
//       used correctly (i.e. never write to a register returned by load_register), but I think this would
//       require updating JITBlock and might hinder its genericity? (It's already pretty specific...)

// Returns a host register contained the requested guest register.
fn load_register(block: *IRBlock, ctx: *JITContext, guest_reg: u4) !JIT.Register {
    return try ctx.guest_reg_cache(block, guest_reg, true, false);
}
// Returns a host register contained the requested guest register and mark it as modified (we plan on writing to it).
fn load_register_for_writing(block: *IRBlock, ctx: *JITContext, guest_reg: u4) !JIT.Register {
    return try ctx.guest_reg_cache(block, guest_reg, true, true);
}
// Return a host register representing the requested guest register, but don't load its actual value.
// Use this as a destination to another instruction that doesn't need the previous value of the destination.
fn get_register_for_writing(block: *IRBlock, ctx: *JITContext, guest_reg: u4) !JIT.Operand {
    return .{ .reg = try ctx.guest_reg_cache(block, guest_reg, false, true) };
}
fn store_register(block: *IRBlock, ctx: *JITContext, guest_reg: u4, value: JIT.Operand) !void {
    try block.mov(try get_register_for_writing(block, ctx, guest_reg), value);
}

fn load_fp_register(block: *IRBlock, ctx: *JITContext, guest_reg: u4) !JIT.Operand {
    return .{ .freg32 = try ctx.guest_freg_cache(block, 32, guest_reg, true, false) };
}
fn load_dfp_register(block: *IRBlock, ctx: *JITContext, guest_reg: u4) !JIT.Operand {
    return .{ .freg64 = try ctx.guest_freg_cache(block, 64, guest_reg, true, false) };
}
fn load_fp_register_for_writing(block: *IRBlock, ctx: *JITContext, guest_reg: u4) !JIT.Operand {
    return .{ .freg32 = try ctx.guest_freg_cache(block, 32, guest_reg, true, true) };
}
fn load_dfp_register_for_writing(block: *IRBlock, ctx: *JITContext, guest_reg: u4) !JIT.Operand {
    return .{ .freg64 = try ctx.guest_freg_cache(block, 64, guest_reg, true, true) };
}
fn store_fp_register(block: *IRBlock, ctx: *JITContext, guest_reg: u4, value: JIT.Operand) !void {
    try block.mov(.{ .freg32 = try ctx.guest_freg_cache(block, 32, guest_reg, false, true) }, value);
}
fn store_dfp_register(block: *IRBlock, ctx: *JITContext, guest_reg: u4, value: JIT.Operand) !void {
    try block.mov(.{ .freg64 = try ctx.guest_freg_cache(block, 64, guest_reg, false, true) }, value);
}

//// Returns a JIT Operand to the memory location of the supplied SH4 struct member.
fn sh4_mem(comptime name: []const u8) Architecture.Operand {
    std.debug.assert(@sizeOf(@FieldType(sh4.SH4, name)) == 4);
    return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, name), .size = 32 } };
}

/// Loads the guest t bit into the host carry flag. NOTE: Overwrites ReturnRegister.
fn load_t(block: *IRBlock, _: *JITContext) !void {
    try block.mov(.{ .reg = ReturnRegister }, sh4_mem("sr"));
    try block.bit_test(ReturnRegister, @bitOffsetOf(sh4.SR, "t"));
}

/// Sets T bit in SR if Condition is fullfilled (In the Host!), otherwise clears it.
// TODO: We'll want to cache the T bit at some point too!
fn set_t(block: *IRBlock, _: *JITContext, condition: JIT.Condition) !void {
    std.debug.assert(@bitOffsetOf(sh4.SR, "t") == 0);
    try block.append(.{ .SetByteCondition = .{ .condition = condition, .dst = .{ .reg8 = ArgRegisters[0] } } });
    try block.mov(.{ .reg = ReturnRegister }, sh4_mem("sr"));
    try block.append(.{ .And = .{ .dst = .{ .reg8 = ReturnRegister }, .src = .{ .imm8 = 0xFE } } });
    try block.append(.{ .Or = .{ .dst = .{ .reg8 = ReturnRegister }, .src = .{ .reg8 = ArgRegisters[0] } } });
    try block.mov(sh4_mem("sr"), .{ .reg = ReturnRegister });
}

pub noinline fn _out_of_line_read8(cpu: *const sh4.SH4, virtual_addr: u32) u8 {
    if (!FastMem) std.debug.assert(virtual_addr < 0x0C000000 or virtual_addr >= 0x10000000); // We can't garantee this won't be called with a RAM address in FastMem mode (even if it is highly unlikely)
    return @call(.always_inline, sh4.SH4.read_physical, .{ cpu, u8, virtual_addr });
}
pub noinline fn _out_of_line_read16(cpu: *const sh4.SH4, virtual_addr: u32) u16 {
    if (!FastMem) std.debug.assert(virtual_addr < 0x0C000000 or virtual_addr >= 0x10000000);
    return @call(.always_inline, sh4.SH4.read_physical, .{ cpu, u16, virtual_addr });
}
pub noinline fn _out_of_line_read32(cpu: *const sh4.SH4, virtual_addr: u32) u32 {
    if (!FastMem) std.debug.assert(virtual_addr < 0x0C000000 or virtual_addr >= 0x10000000);
    return @call(.always_inline, sh4.SH4.read_physical, .{ cpu, u32, virtual_addr });
}
pub noinline fn _out_of_line_read64(cpu: *const sh4.SH4, virtual_addr: u32) u64 {
    if (!FastMem) std.debug.assert(virtual_addr < 0x0C000000 or virtual_addr >= 0x10000000);
    return @call(.always_inline, sh4.SH4.read_physical, .{ cpu, u64, virtual_addr });
}
pub noinline fn _out_of_line_write8(cpu: *sh4.SH4, virtual_addr: u32, value: u8) void {
    if (!FastMem) std.debug.assert(virtual_addr < 0x0C000000 or virtual_addr >= 0x10000000);
    return @call(.always_inline, sh4.SH4.write_physical, .{ cpu, u8, virtual_addr, value });
}
pub noinline fn _out_of_line_write16(cpu: *sh4.SH4, virtual_addr: u32, value: u16) void {
    if (!FastMem) std.debug.assert(virtual_addr < 0x0C000000 or virtual_addr >= 0x10000000);
    return @call(.always_inline, sh4.SH4.write_physical, .{ cpu, u16, virtual_addr, value });
}
pub noinline fn _out_of_line_write32(cpu: *sh4.SH4, virtual_addr: u32, value: u32) void {
    if (!FastMem) std.debug.assert(virtual_addr < 0x0C000000 or virtual_addr >= 0x10000000);
    return @call(.always_inline, sh4.SH4.write_physical, .{ cpu, u32, virtual_addr, value });
}
pub noinline fn _out_of_line_write64(cpu: *sh4.SH4, virtual_addr: u32, value: u64) void {
    if (!FastMem) std.debug.assert(virtual_addr < 0x0C000000 or virtual_addr >= 0x10000000);
    return @call(.always_inline, sh4.SH4.write_physical, .{ cpu, u64, virtual_addr, value });
}

/// A call to the handler will return the physical address in the lower 32bits, and a non-zero value in the upper 32bits if an exception occurred.
fn runtime_mmu_translation(comptime access_type: sh4.SH4.AccessType) type {
    return struct {
        fn handler(cpu: *sh4.SH4, virtual_addr: u32) packed struct(u64) { address: u32, exception: u32 } {
            const physical = cpu.translate_address(access_type, virtual_addr) catch |err| {
                sh4_jit_log.warn("MMU miss {s} for {X:0>8}. UTLBs:", .{ @errorName(err), virtual_addr });
                for (cpu.utlb, 0..) |utlb, idx| {
                    if (utlb.valid()) {
                        sh4_jit_log.warn("  [{d}] {any}", .{ idx, utlb });
                    }
                }

                cpu.report_address_exception(virtual_addr);
                switch (err) {
                    error.DataTLBMissRead, error.DataTLBMissWrite => {
                        cpu.jump_to_exception(switch (access_type) {
                            .Read => .DataTLBMissRead,
                            .Write => .DataTLBMissWrite,
                        });
                        return .{ .address = 0, .exception = 1 };
                    },
                    error.InitialPageWrite => {
                        std.debug.assert(access_type == .Write);
                        cpu.jump_to_exception(.InitialPageWrite);
                        return .{ .address = 0, .exception = 2 };
                    },
                    error.DataTLBProtectionViolation => {
                        cpu.jump_to_exception(switch (access_type) {
                            .Read => .DataTLBProtectionViolationRead,
                            .Write => .DataTLBProtectionViolationWrite,
                        });
                        return .{ .address = 0, .exception = 3 };
                    },
                    error.DataTLBMultipleHit => {
                        cpu.jump_to_exception(.DataTLBMultipleHit);
                        return .{ .address = 0, .exception = 4 };
                    },
                }
            };
            return .{ .address = physical, .exception = 0 };
        }
    };
}

/// Attempts an address translation and return the translated address to addr. Exits the current block if an exception is raised.
fn mmu_translation(comptime access_type: sh4.SH4.AccessType, block: *IRBlock, ctx: *JITContext, addr: JIT.Register, register_to_save: ?JIT.Register) !void {
    try ctx.gpr_cache.commit_all(block);
    try ctx.fpr_cache.commit_all(block);

    std.debug.assert(register_to_save != ArgRegisters[0]);

    // Backup current PC (mostly in case we're in a branch delay slot)
    try block.mov(.{ .reg = ArgRegisters[0] }, sh4_mem("pc"));
    try block.push(.{ .reg = ArgRegisters[0] });

    if (ctx.in_delay_slot) {
        try block.mov(sh4_mem("pc"), .{ .imm32 = ctx.current_pc - 2 });
    } else {
        try block.mov(sh4_mem("pc"), .{ .imm32 = ctx.current_pc });
    }

    if (register_to_save) |r| {
        try block.push(.{ .reg = r });
    } else {
        try block.push(.{ .reg = ArgRegisters[0] }); // Stack alignemnt
    }

    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    if (addr != ArgRegisters[1])
        try block.mov(.{ .reg = ArgRegisters[1] }, .{ .reg = addr });
    try call(block, ctx, &runtime_mmu_translation(access_type).handler);
    if (addr != ReturnRegister)
        try block.mov(.{ .reg = addr }, .{ .reg = ReturnRegister });

    // Check if an exception was raised.
    try block.mov(.{ .reg64 = ArgRegisters[0] }, .{ .imm64 = 0xFFFFFFFF });
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg64 = ReturnRegister }, .rhs = .{ .reg64 = ArgRegisters[0] } } });

    if (register_to_save) |r| {
        try block.pop(.{ .reg = r });
    } else {
        try block.pop(.{ .reg = ArgRegisters[0] });
    }
    try block.pop(.{ .reg = ArgRegisters[0] });
    // Terminate the block immediately if an exception was raised.
    try ctx.add_jump_to_end(try block.jmp(.Above));

    try block.mov(sh4_mem("pc"), .{ .reg = ArgRegisters[0] });
}

// Load a u<size> from memory into a host register, with a fast path if the address lies in RAM.
fn load_mem(block: *IRBlock, ctx: *JITContext, dest: JIT.Register, guest_reg: u4, comptime addressing: enum { Reg, Reg_R0 }, displacement: u32, comptime size: u32) !void {
    const src_guest_reg_location = try load_register(block, ctx, guest_reg);

    const addr = ArgRegisters[1];

    try block.mov(.{ .reg = addr }, .{ .reg = src_guest_reg_location });
    if (displacement != 0)
        try block.add(.{ .reg = addr }, .{ .imm32 = displacement });
    if (addressing == .Reg_R0) {
        const r0 = try load_register(block, ctx, 0);
        try block.add(.{ .reg = addr }, .{ .reg = r0 });
    }

    if (ctx.mmu_enabled)
        try mmu_translation(.Read, block, ctx, addr, null);

    if (FastMem) {
        try block.mov(.{ .reg = dest }, .{ .mem = .{ .base = .rbp, .index = addr, .size = size } });

        var skip_fallback = try block.jmp(.Always);
        try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
        // Address is already loaded into ArgRegisters[1]
        switch (size) {
            8 => try call(block, ctx, &_out_of_line_read8),
            16 => try call(block, ctx, &_out_of_line_read16),
            32 => try call(block, ctx, &_out_of_line_read32),
            64 => try call(block, ctx, &_out_of_line_read64),
            else => @compileError("load_mem: Unsupported size."),
        }
        if (dest != ReturnRegister) try block.mov(.{ .reg = dest }, .{ .reg = ReturnRegister });
        skip_fallback.patch();
    } else { // RAM Fast path
        try block.mov(.{ .reg = ReturnRegister }, .{ .reg = ArgRegisters[1] });
        try block.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = 0x1C000000 } } });
        try block.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = 0x0C000000 } } });
        // TODO: Could it be worth to use a conditional move here to have a single jump (skipping the call)?
        var not_branch = try block.jmp(.NotEqual);
        // We're in RAM!
        try block.append(.{ .And = .{ .dst = .{ .reg = ArgRegisters[1] }, .src = .{ .imm32 = 0x00FFFFFF } } });
        try block.mov(.{ .reg = dest }, .{ .mem = .{ .base = .rbp, .index = ArgRegisters[1], .size = size } });
        var to_end = try block.jmp(.Always);

        not_branch.patch();

        try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
        // Address is already loaded into ArgRegisters[1]
        switch (size) {
            8 => try call(block, ctx, &_out_of_line_read8),
            16 => try call(block, ctx, &_out_of_line_read16),
            32 => try call(block, ctx, &_out_of_line_read32),
            64 => try call(block, ctx, &_out_of_line_read64),
            else => @compileError("load_mem: Unsupported size."),
        }

        if (dest != ReturnRegister)
            try block.mov(.{ .reg = dest }, .{ .reg = ReturnRegister });

        to_end.patch();
    }
}

fn store_mem(block: *IRBlock, ctx: *JITContext, dest_guest_reg: u4, comptime addressing: enum { Reg, Reg_R0, GBR }, displacement: u32, value: JIT.Operand, comptime size: u32) !void {
    std.debug.assert(value.size() == size);

    const addr = ArgRegisters[1];

    switch (addressing) {
        .Reg, .Reg_R0 => {
            const dest_guest_reg_location = try load_register(block, ctx, dest_guest_reg);
            try block.mov(.{ .reg = addr }, .{ .reg = dest_guest_reg_location });
            if (addressing == .Reg_R0) {
                const r0 = try load_register(block, ctx, 0);
                try block.add(.{ .reg = addr }, .{ .reg = r0 });
            }
        },
        .GBR => try block.mov(.{ .reg = addr }, sh4_mem("gbr")),
    }
    if (displacement != 0)
        try block.add(.{ .reg = addr }, .{ .imm32 = displacement });

    if (ctx.mmu_enabled) {
        // Make sure value isn't overwritten
        const register_to_save = switch (value) {
            .reg8, .reg16, .reg, .reg64 => |r| r,
            else => null,
        };

        try mmu_translation(.Write, block, ctx, addr, register_to_save);
    }

    if (FastMem) {
        try block.mov(.{ .mem = .{ .base = .rbp, .index = addr, .size = size } }, value);

        var skip_fallback = try block.jmp(.Always);
        try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
        // Address is already loaded into ArgRegisters[1]
        if (value.tag() != .reg or value.reg != ArgRegisters[2])
            try block.mov(.{ .reg = ArgRegisters[2] }, value);
        switch (size) {
            8 => try call(block, ctx, &_out_of_line_write8),
            16 => try call(block, ctx, &_out_of_line_write16),
            32 => try call(block, ctx, &_out_of_line_write32),
            64 => try call(block, ctx, &_out_of_line_write64),
            else => @compileError("store_mem: Unsupported size."),
        }
        skip_fallback.patch();
    } else {
        // RAM Fast path
        try block.mov(.{ .reg = ArgRegisters[3] }, .{ .reg = addr });
        try block.append(.{ .And = .{ .dst = .{ .reg = ArgRegisters[3] }, .src = .{ .imm32 = 0x1C000000 } } });
        try block.append(.{ .Cmp = .{ .lhs = .{ .reg = ArgRegisters[3] }, .rhs = .{ .imm32 = 0x0C000000 } } });
        var not_branch = try block.jmp(.NotEqual);
        // We're in RAM!
        try block.append(.{ .And = .{ .dst = .{ .reg = addr }, .src = .{ .imm32 = 0x00FFFFFF } } });
        try block.mov(.{ .mem = .{ .base = .rbp, .index = addr, .size = size } }, value);
        var to_end = try block.jmp(.Always);

        not_branch.patch();

        try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
        // Address is already loaded into ArgRegisters[1]
        if (value.tag() != .reg or value.reg != ArgRegisters[2])
            try block.mov(.{ .reg = ArgRegisters[2] }, value);
        switch (size) {
            8 => try call(block, ctx, &_out_of_line_write8),
            16 => try call(block, ctx, &_out_of_line_write16),
            32 => try call(block, ctx, &_out_of_line_write32),
            64 => try call(block, ctx, &_out_of_line_write64),
            else => @compileError("store_mem: Unsupported size."),
        }
        to_end.patch();
    }
}

pub fn mov_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rm = try load_register(block, ctx, instr.nmd.m);
    try store_register(block, ctx, instr.nmd.n, .{ .reg = rm });
    return false;
}

pub fn mov_imm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // FIXME: Should keep the "signess" in the type system?  ---v
    try store_register(block, ctx, instr.nmd.n, .{ .imm32 = @bitCast(bit_manip.sign_extension_u8(instr.nd8.d)) });
    return false;
}

pub fn mov_atRm_Rn(comptime size: u8) *const fn (block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) anyerror!bool {
    return (struct {
        pub fn handler(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, size);
            if (size < 32) { // Sign extend
                try block.movsx(try get_register_for_writing(block, ctx, instr.nmd.n), .Reg(ReturnRegister, size));
            } else {
                try store_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
            }
            return false;
        }
    }).handler;
}

pub fn mov_Rm_atRn(comptime size: u8) *const fn (block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) anyerror!bool {
    return (struct {
        pub fn handler(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
            const rm = try load_register(block, ctx, instr.nmd.m);
            try store_mem(block, ctx, instr.nmd.n, .Reg, 0, .Reg(rm, size), size);
            return false;
        }
    }).handler;
}

pub fn mov_atRmInc_Rn(comptime size: u8) *const fn (block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) anyerror!bool {
    return (struct {
        pub fn handler(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
            _ = try mov_atRm_Rn(size)(block, ctx, instr);
            if (instr.nmd.n != instr.nmd.m) {
                const rm = try load_register_for_writing(block, ctx, instr.nmd.m);
                try block.add(.{ .reg = rm }, .{ .imm32 = size / 8 });
            }
            return false;
        }
    }).handler;
}

// mov.x Rm,@-Rn when Rn == Rm
fn mov_Rn_atDecRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr, comptime size: u8) !bool {
    std.debug.assert(instr.nmd.n == instr.nmd.m);
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = rn }); // The value stored is the value of Rn before decrement.
    try block.sub(.{ .reg = rn }, .{ .imm32 = size / 8 });
    try store_mem(block, ctx, instr.nmd.n, .Reg, 0, .Reg(ReturnRegister, size), size);
    return false;
}

pub fn mov_Rm_atDecRn(comptime size: u8) *const fn (block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) anyerror!bool {
    return (struct {
        pub fn handler(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
            if (instr.nmd.n == instr.nmd.m) return mov_Rn_atDecRn(block, ctx, instr, size);
            const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
            try block.sub(.{ .reg = rn }, .{ .imm32 = size / 8 });
            return mov_Rm_atRn(size)(block, ctx, instr);
        }
    }).handler;
}

pub fn movb_atDispRm_R0(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d);
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, d, 8);
    try block.movsx(try get_register_for_writing(block, ctx, 0), .{ .reg8 = ReturnRegister });
    return false;
}

pub fn movw_atDispRm_R0(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d) << 1;
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, d, 16);
    try block.movsx(try get_register_for_writing(block, ctx, 0), .{ .reg16 = ReturnRegister });
    return false;
}

pub fn movl_atDispRm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d) << 2;
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, d, 32);
    try store_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
    return false;
}

pub fn movb_R0_atDispRm(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d);
    const r0 = try load_register(block, ctx, 0);
    try store_mem(block, ctx, instr.nmd.m, .Reg, d, .{ .reg8 = r0 }, 8);
    return false;
}

pub fn movw_R0_atDispRm(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d) << 1;
    const r0 = try load_register(block, ctx, 0);
    try store_mem(block, ctx, instr.nmd.m, .Reg, d, .{ .reg16 = r0 }, 16);
    return false;
}

pub fn movl_Rm_atDispRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nmd.d) << 2;
    const rm = try load_register(block, ctx, instr.nmd.m);
    try store_mem(block, ctx, instr.nmd.n, .Reg, d, .{ .reg = rm }, 32);
    return false;
}

pub fn movb_atR0Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg_R0, 0, 8);
    try block.movsx(try get_register_for_writing(block, ctx, instr.nmd.n), .{ .reg8 = ReturnRegister });
    return false;
}

pub fn movw_atR0Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg_R0, 0, 16);
    try block.movsx(try get_register_for_writing(block, ctx, instr.nmd.n), .{ .reg16 = ReturnRegister });
    return false;
}

pub fn movl_atR0Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg_R0, 0, 32);
    try store_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
    return false;
}

pub fn movb_Rm_atR0Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rm = try load_register(block, ctx, instr.nmd.m);
    try store_mem(block, ctx, instr.nmd.n, .Reg_R0, 0, .{ .reg8 = rm }, 8);
    return false;
}

pub fn movw_Rm_atR0Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rm = try load_register(block, ctx, instr.nmd.m);
    try store_mem(block, ctx, instr.nmd.n, .Reg_R0, 0, .{ .reg16 = rm }, 16);
    return false;
}

pub fn movl_Rm_atR0Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rm = try load_register(block, ctx, instr.nmd.m);
    try store_mem(block, ctx, instr.nmd.n, .Reg_R0, 0, .{ .reg = rm }, 32);
    return false;
}

pub fn mov_R0_atDispGBR(comptime T: type, block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const r0 = try load_register(block, ctx, 0);
    const displacement: u32 = bit_manip.zero_extend(instr.nd8.d) * @sizeOf(T);
    try store_mem(block, ctx, 0, .GBR, displacement, .Reg(r0, @bitSizeOf(T)), @bitSizeOf(T));
    return false;
}

pub fn movb_R0_atDispGBR(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return mov_R0_atDispGBR(u8, block, ctx, instr);
}

pub fn movw_R0_atDispGBR(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return mov_R0_atDispGBR(u16, block, ctx, instr);
}

pub fn movl_R0_atDispGBR(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return mov_R0_atDispGBR(u32, block, ctx, instr);
}

pub fn movt_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    std.debug.assert(@bitOffsetOf(sh4.SR, "t") == 0);
    const rn = try get_register_for_writing(block, ctx, instr.nmd.n);
    try block.mov(rn, sh4_mem("sr"));
    try block.append(.{ .And = .{ .dst = rn, .src = .{ .imm32 = 0x00000001 } } });
    return false;
}

pub fn swapb_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    if (instr.nmd.n == instr.nmd.m) {
        const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
        try block.append(.{ .Ror = .{ .dst = .{ .reg16 = rn }, .amount = .{ .imm8 = 8 } } });
    } else {
        const rn = try get_register_for_writing(block, ctx, instr.nmd.n);
        const rm = try load_register(block, ctx, instr.nmd.m);
        try block.mov(rn, .{ .reg = rm });
        try block.append(.{ .Ror = .{ .dst = .{ .reg16 = rn.reg }, .amount = .{ .imm8 = 8 } } });
    }
    return false;
}

pub fn swapw_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    if (instr.nmd.n == instr.nmd.m) {
        const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
        try block.append(.{ .Ror = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 16 } } });
    } else {
        const rn = try get_register_for_writing(block, ctx, instr.nmd.n);
        const rm = try load_register(block, ctx, instr.nmd.m);
        try block.mov(rn, .{ .reg = rm });
        try block.append(.{ .Ror = .{ .dst = rn, .amount = .{ .imm8 = 16 } } });
    }
    return false;
}

pub fn add_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.add(.{ .reg = rn }, .{ .reg = rm });
    return false;
}

pub fn add_imm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.add(.{ .reg = rn }, .{ .imm32 = @bitCast(bit_manip.sign_extension_u8(instr.nd8.d)) });
    return false;
}

pub fn addc_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try load_t(block, ctx); // t is also the carry flag
    try block.append(.{ .Adc = .{ .dst = .{ .reg = rn }, .src = .{ .reg = rm } } });
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn cmpeq_imm_R0(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const r0 = try load_register(block, ctx, 0);
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = r0 }, .rhs = .{ .imm32 = @bitCast(bit_manip.sign_extension_u8(instr.nd8.d)) } } });
    try set_t(block, ctx, .Equal);
    return false;
}

fn cmp_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr, condition: JIT.Condition) !bool {
    const rn = try load_register(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = rn }, .rhs = .{ .reg = rm } } });
    try set_t(block, ctx, condition);
    return false;
}

pub fn cmpeq_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return cmp_Rm_Rn(block, ctx, instr, .Equal);
}

pub fn cmpge_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return cmp_Rm_Rn(block, ctx, instr, .GreaterEqual);
}

pub fn cmpgt_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return cmp_Rm_Rn(block, ctx, instr, .Greater);
}

pub fn cmphs_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return cmp_Rm_Rn(block, ctx, instr, .AboveEqual);
}

pub fn cmphi_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return cmp_Rm_Rn(block, ctx, instr, .Above);
}

pub fn cmppl_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register(block, ctx, instr.nmd.n);
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = rn }, .rhs = .{ .imm32 = 0 } } });
    try set_t(block, ctx, .Greater);
    return false;
}

pub fn cmppz_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register(block, ctx, instr.nmd.n);
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = rn }, .rhs = .{ .imm32 = 0 } } });
    try set_t(block, ctx, .GreaterEqual);
    return false;
}

pub fn stc_Reg_Rn(comptime reg: []const u8) fn (block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) anyerror!bool {
    const T = struct {
        fn stc(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
            try store_register(block, ctx, instr.nmd.n, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, reg), .size = 32 } });
            return false;
        }
    };
    return T.stc;
}

pub fn stcl_Reg_atRnDec(comptime reg: []const u8) fn (block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) anyerror!bool {
    const T = struct {
        fn stcl(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
            const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
            try block.sub(.{ .reg = rn }, .{ .imm32 = 4 });
            try block.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, reg), .size = 32 } });
            try store_mem(block, ctx, instr.nmd.n, .Reg, 0, .{ .reg = ReturnRegister }, 32);
            return false;
        }
    };
    return T.stcl;
}

pub fn fmov_FRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Single => try store_fp_register(block, ctx, instr.nmd.n, try load_fp_register(block, ctx, instr.nmd.m)),
        .Double => try store_dfp_register(block, ctx, instr.nmd.n, try load_dfp_register(block, ctx, instr.nmd.m)),
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fmovs_atRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Single => {
            // FRn = [Rm]
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 32);
            try store_fp_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
        },
        .Double => {
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 64);
            try store_dfp_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
        },
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fmovs_FRm_atRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Single => {
            // [Rn] = FRm
            const frm = try load_fp_register(block, ctx, instr.nmd.m);
            try store_mem(block, ctx, instr.nmd.n, .Reg, 0, frm, 32);
        },
        .Double => {
            const drm = try load_dfp_register(block, ctx, instr.nmd.m);
            try store_mem(block, ctx, instr.nmd.n, .Reg, 0, drm, 64);
        },
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fmovs_atRmInc_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Single, .Double => |size| {
            _ = try fmovs_atRm_FRn(block, ctx, instr);
            // Inc Rm
            const rm = try load_register_for_writing(block, ctx, instr.nmd.m);
            try block.add(.{ .reg = rm }, .{ .imm32 = if (size == .Double) 8 else 4 });
        },
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fmovs_FRm_atDecRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Single, .Double => |size| {
            // Dec Rn
            const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
            try block.sub(.{ .reg = rn }, .{ .imm32 = if (size == .Double) 8 else 4 });
            return fmovs_FRm_atRn(block, ctx, instr);
        },
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fmovs_atR0Rm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Single => {
            // FRn = [Rm+R0]
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg_R0, 0, 32);
            try store_fp_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
        },
        .Double => {
            try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg_R0, 0, 64);
            try store_dfp_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
        },
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fmovs_FRm_atR0Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_sz) {
        .Single => {
            // [Rn+R0] = FRm
            const frm = try load_fp_register(block, ctx, instr.nmd.m);
            try store_mem(block, ctx, instr.nmd.n, .Reg_R0, 0, frm, 32);
        },
        .Double => {
            const drm = try load_dfp_register(block, ctx, instr.nmd.m);
            try store_mem(block, ctx, instr.nmd.n, .Reg_R0, 0, drm, 64);
        },
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fldi0_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => {
            // Might be a xor, but we don't care about the previous value here.
            const frn: JIT.Operand = .{ .freg32 = try ctx.guest_freg_cache(block, 32, instr.nmd.n, false, true) };
            try block.append(.{ .Xor = .{ .dst = frn, .src = frn } });
        },
        .Double => return error.IllegalInstruction,
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn flds_FRn_FPUL(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const frn = try load_fp_register(block, ctx, instr.nmd.n);
    try block.mov(sh4_mem("fpul"), frn);
    return false;
}

pub fn fsts_FPUL_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try store_fp_register(block, ctx, instr.nmd.n, sh4_mem("fpul"));
    return false;
}

pub fn fldi1_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => {
            try block.mov(.{ .reg = ReturnRegister }, .{ .imm32 = 0x3F800000 }); // 1.0
            try store_fp_register(block, ctx, instr.nmd.n, .{ .reg = ReturnRegister });
        },
        .Double => return error.IllegalInstruction,
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fabs_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const frn = try load_fp_register_for_writing(block, ctx, instr.nmd.n);
    const tmp: JIT.Operand = .{ .reg = ReturnRegister };
    const ftmp: JIT.Operand = .{ .freg32 = FPScratchRegisters[0] };
    try block.mov(tmp, .{ .imm32 = 0x7FFFFFFF });
    try block.mov(ftmp, tmp);
    try block.append(.{ .And = .{ .dst = frn, .src = ftmp } });
    return false;
}

pub fn fneg_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const frn = try load_fp_register_for_writing(block, ctx, instr.nmd.n);
    const tmp: JIT.Operand = .{ .reg = ReturnRegister };
    const ftmp: JIT.Operand = .{ .freg32 = FPScratchRegisters[0] };
    try block.mov(tmp, .{ .imm32 = 0x80000000 });
    try block.mov(ftmp, tmp);
    try block.append(.{ .Xor = .{ .dst = frn, .src = ftmp } });
    return false;
}

pub fn fadd_FRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => {
            const frn = try load_fp_register_for_writing(block, ctx, instr.nmd.n);
            const frm = try load_fp_register(block, ctx, instr.nmd.m);
            try block.add(frn, frm);
        },
        .Double => {
            return interpreter_fallback_cached(block, ctx, instr);
            // const frn = try load_dfp_register_for_writing(block, ctx, instr.nmd.n);
            // const frm = try load_dfp_register(block, ctx, instr.nmd.m);
            // try block.add(frn, frm);
        },
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fsub_FRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => try block.sub(
            try load_fp_register_for_writing(block, ctx, instr.nmd.n),
            try load_fp_register(block, ctx, instr.nmd.m),
        ),
        .Double => return interpreter_fallback_cached(block, ctx, instr),
        // try block.sub(
        //  try load_dfp_register_for_writing(block, ctx, instr.nmd.n),
        //  try load_dfp_register(block, ctx, instr.nmd.m),
        // ),
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fmul_FRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => try block.append(.{ .Mul = .{
            .dst = try load_fp_register_for_writing(block, ctx, instr.nmd.n),
            .src = try load_fp_register(block, ctx, instr.nmd.m),
        } }),
        .Double => return interpreter_fallback_cached(block, ctx, instr),
        // try block.append(.{ .Mul = .{
        //     .dst = try load_dfp_register_for_writing(block, ctx, instr.nmd.n),
        //     .src = try load_dfp_register(block, ctx, instr.nmd.m),
        // } }),
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fmac_FR0_FRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => {
            const frn = try load_fp_register_for_writing(block, ctx, instr.nmd.n);
            const fr0 = try load_fp_register(block, ctx, 0);
            const frm = try load_fp_register(block, ctx, instr.nmd.m);
            try block.append(.{ .Fma = .{ .dst = frn.freg32, .src1 = frm.freg32, .src2 = fr0 } });
        },
        .Double => {
            return interpreter_fallback_cached(block, ctx, instr);
            // const frn = try load_dfp_register_for_writing(block, ctx, instr.nmd.n);
            // const fr0 = try load_dfp_register(block, ctx, 0);
            // const frm = try load_dfp_register(block, ctx, instr.nmd.m);
            // // TODO: Actually use a FMA instruction (VFMADD132SD).
            // const tmp: JIT.Operand = .{ .freg64 = .xmm0 }; // Use a temporary register, we don't want to modify FR0 or FRm.
            // try block.mov(tmp, fr0);
            // try block.append(.{ .Mul = .{ .dst = tmp, .src = frm } });
            // try block.add(frn, tmp);
        },
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fdiv_FRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => try block.append(.{ .Div = .{
            .dst = try load_fp_register_for_writing(block, ctx, instr.nmd.n),
            .src = try load_fp_register(block, ctx, instr.nmd.m),
        } }),
        .Double => return interpreter_fallback_cached(block, ctx, instr),
        // try block.append(.{ .Div = .{
        //     .dst = try load_dfp_register_for_writing(block, ctx, instr.nmd.n),
        //     .src = try load_dfp_register(block, ctx, instr.nmd.m),
        // } }),
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fsqrt_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => {
            const rn = try load_fp_register_for_writing(block, ctx, instr.nmd.n);
            try block.append(.{ .Sqrt = .{ .dst = rn, .src = rn } });
        },
        .Double => return interpreter_fallback_cached(block, ctx, instr),
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn fcmp_eq_FRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => try block.append(.{ .Cmp = .{
            .lhs = try load_fp_register(block, ctx, instr.nmd.n),
            .rhs = try load_fp_register(block, ctx, instr.nmd.m),
        } }),
        .Double => return interpreter_fallback_cached(block, ctx, instr),
        // try block.append(.{ .Cmp = .{
        //     .lhs = try load_dfp_register(block, ctx, instr.nmd.n),
        //     .rhs = try load_dfp_register(block, ctx, instr.nmd.m),
        // } }),
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    try set_t(block, ctx, .Equal);
    return false;
}

pub fn fcmp_gt_FRm_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => try block.append(.{ .Cmp = .{
            .lhs = try load_fp_register(block, ctx, instr.nmd.n),
            .rhs = try load_fp_register(block, ctx, instr.nmd.m),
        } }),
        .Double => return interpreter_fallback_cached(block, ctx, instr),
        // try block.append(.{ .Cmp = .{
        //     .lhs = try load_dfp_register(block, ctx, instr.nmd.n),
        //     .rhs = try load_dfp_register(block, ctx, instr.nmd.m),
        // } }),
        .Unknown => return interpreter_fallback_cached(block, ctx, instr),
    }
    try set_t(block, ctx, .Above);
    return false;
}

pub fn float_FPUL_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    switch (ctx.fpscr_pr) {
        .Single => try block.append(.{ .Convert = .{
            .dst = .{ .freg32 = try ctx.guest_freg_cache(block, 32, instr.nmd.n, false, true) },
            .src = sh4_mem("fpul"),
        } }),
        .Double => return interpreter_fallback_cached(block, ctx, instr),
        // try block.append(.{ .Convert = .{
        //     .dst = .{ .freg64 = try ctx.guest_freg_cache(block, 64, instr.nmd.n, false, true) },
        //     .src = SH4Member("fpul"),
        // } }),
        else => return interpreter_fallback_cached(block, ctx, instr),
    }
    return false;
}

pub fn ftrc_FRn_FPUL(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // NOTE: Overflow behaviour differs from x86_64.
    //        x86_64 will return the "indefinite integer value" (0x80000000 or 0x80000000_00000000 if operand size is 64 bits)
    //        See interpreter implementation (ftrc_FRn_FPUL) for more details.
    const dest_tmp: JIT.Operand = .{ .reg = ReturnRegister };

    switch (ctx.fpscr_pr) {
        .Single => {
            const FRn = try load_fp_register(block, ctx, instr.nmd.n);
            const tmp_float: Architecture.Operand = .{ .freg32 = FPScratchRegisters[0] };
            const tmp_max_float: Architecture.Operand = .{ .freg32 = FPScratchRegisters[1] };
            try block.mov(tmp_float, FRn); // Copy FRn to a temporary we can manipulate.

            // Clamp it. This doesn't match the interpreter implementation in edge cases (inf and NaN basically),
            // but sounds good enough for now (literally, this fixes popping sounds in SA2 for example).
            const max_f32: u32 = 0x4EFFFFFF; // 1.FFFFFE * 2^30
            try block.mov(dest_tmp, .{ .imm32 = max_f32 });
            try block.mov(tmp_max_float, dest_tmp);
            try block.append(.{ .Min = .{ .dst = tmp_float, .src = tmp_max_float } });
            // CVTTSS2SI will already return 0x80000000 in case of negative overflow.

            try block.append(.{ .Convert = .{
                .dst = dest_tmp,
                .src = tmp_float,
            } });
        },
        .Double => return interpreter_fallback_cached(block, ctx, instr),
        else => return interpreter_fallback_cached(block, ctx, instr),
    }
    try block.mov(sh4_mem("fpul"), dest_tmp);
    return false;
}

pub fn fsrra_FRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    std.debug.assert(ctx.fpscr_pr == .Single);
    // NOTE: x86 rsqrt is less precise than its SH4 equivalent.
    const frn = try load_fp_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Sqrt = .{ .dst = .{ .freg32 = .xmm0 }, .src = frn } }); // xmm0 = sqrt(FRn)
    try block.mov(.{ .reg = ReturnRegister }, .{ .imm32 = 0x3F800000 }); // rax = 1.0
    try block.mov(frn, .{ .reg = ReturnRegister }); // Frn = 1.0
    try block.append(.{ .Div = .{ .dst = frn, .src = .{ .freg32 = .xmm0 } } }); // FRn /= xmm0

    return false;
}

const fsca_table = @embedFile("../data/fsca.bin");

pub fn fsca_FPUL_DRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    std.debug.assert(ctx.fpscr_pr == .Single);
    std.debug.assert(instr.nmd.n & 1 == 0);

    try block.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "fpul"), .size = 16 } }); // Load low 16-bits of FPUL
    try block.mov(.{ .reg64 = ArgRegisters[0] }, .{ .imm64 = @intFromPtr(fsca_table.ptr) });
    // Lookup into the pre-computed table ([0x10000][2]f32)
    //    This is 512kB LUT, I wonder how efficient it really is. Looks like ~5% gain in my very stupid benchmark.
    try store_fp_register(block, ctx, instr.nmd.n + 0, .{ .mem = .{ .base = ArgRegisters[0], .index = ReturnRegister, .scale = ._8, .displacement = 0, .size = 32 } });
    try store_fp_register(block, ctx, instr.nmd.n + 1, .{ .mem = .{ .base = ArgRegisters[0], .index = ReturnRegister, .scale = ._8, .displacement = 4, .size = 32 } });

    return false;
}

pub fn lds_rn_FPSCR(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try ctx.fpr_cache.commit_and_invalidate_all(block); // We may switch FP register banks

    const rn = try load_register(block, ctx, instr.nmd.n);
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    try block.mov(.{ .reg = ArgRegisters[1] }, .{ .reg = rn });
    try call(block, ctx, sh4.SH4.set_fpscr);

    ctx.fpscr_sz = .Unknown;
    ctx.fpscr_pr = .Unknown;

    return false;
}

pub fn ldsl_atRnInc_FPSCR(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try ctx.fpr_cache.commit_and_invalidate_all(block); // We may switch FP register banks

    try load_mem(block, ctx, ArgRegisters[1], instr.nmd.n, .Reg, 0, 32);
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    try call(block, ctx, sh4.SH4.set_fpscr);

    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.add(.{ .reg = rn }, .{ .imm32 = 4 });

    ctx.fpscr_sz = .Unknown;
    ctx.fpscr_pr = .Unknown;
    return false;
}

pub fn lds_Rn_FPUL(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register(block, ctx, instr.nmd.n);
    try block.mov(sh4_mem("fpul"), .{ .reg = rn });
    return false;
}

pub fn sts_FPUL_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try store_register(block, ctx, instr.nmd.n, sh4_mem("fpul"));
    return false;
}

pub fn ldsl_atRnInc_FPUL(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try load_mem(block, ctx, ReturnRegister, instr.nmd.n, .Reg, 0, 32);
    try block.mov(sh4_mem("fpul"), .{ .reg = ReturnRegister });
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.add(.{ .reg = rn }, .{ .imm32 = 4 });
    return false;
}

pub fn fschg(block: *IRBlock, ctx: *JITContext, _: sh4.Instr) !bool {
    try block.append(.{
        .Xor = .{
            .dst = sh4_mem("fpscr"),
            .src = .{ .imm32 = @as(u32, 1) << @bitOffsetOf(sh4.FPSCR, "sz") },
        },
    });
    switch (ctx.fpscr_sz) {
        .Single => ctx.fpscr_sz = .Double,
        .Double => ctx.fpscr_sz = .Single,
        else => {},
    }
    return false;
}

pub fn mova_atDispPC_R0(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const d = bit_manip.zero_extend(instr.nd8.d) << 2;
    const addr = (ctx.current_pc & 0xFFFFFFFC) + 4 + d;
    try store_register(block, ctx, 0, .{ .imm32 = addr });
    return false;
}

pub fn movw_atDispPC_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const physical_address = ctx.current_physical_pc;
    // Physical address should be either in Boot ROM, or in RAM.
    std.debug.assert(physical_address < 0x00200000 or (physical_address >= 0x0C000000 and physical_address < 0x10000000));
    // @(d8,PC) is fixed, compute its real absolute address
    const d = bit_manip.zero_extend(instr.nd8.d) << 1;
    const addr = physical_address + 4 + d;
    if (addr < 0x00200000) { // We're in ROM.
        try store_register(block, ctx, instr.nd8.n, .{ .imm32 = @bitCast(bit_manip.sign_extension_u16(ctx.cpu.read_physical(u16, addr))) });
    } else { // Load from RAM and sign extend
        const offset = if (FastMem) 0x0C00_0000 else 0;
        try block.movsx(.{ .reg = try ctx.guest_reg_cache(block, instr.nd8.n, false, true) }, .{ .mem = .{ .base = .rbp, .displacement = offset + (addr & 0x00FFFFFF), .size = 16 } });
    }
    return false;
}

pub fn movl_atDispPC_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const physical_address = ctx.current_physical_pc;
    // Physical address should be either in Boot ROM, or in RAM.
    std.debug.assert(physical_address < 0x00200000 or (physical_address >= 0x0C000000 and physical_address < 0x10000000));
    // @(d8,PC) is fixed, compute its real absolute address
    const d = bit_manip.zero_extend(instr.nd8.d) << 2;
    const addr = (physical_address & 0xFFFFFFFC) + 4 + d;
    if (addr < 0x00200000) { // We're in ROM.
        try store_register(block, ctx, instr.nd8.n, .{ .imm32 = ctx.cpu.read_physical(u32, addr) });
    } else {
        const offset = if (FastMem) 0x0C00_0000 else 0;
        try store_register(block, ctx, instr.nd8.n, .{ .mem = .{ .base = .rbp, .displacement = offset + (addr & 0x00FFFFFF), .size = 32 } });
    }
    return false;
}

pub fn dt_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.sub(.{ .reg = rn }, .{ .imm32 = 1 });
    try set_t(block, ctx, .Equal); // ZF=1
    return false;
}

pub fn extsb_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    if (instr.nmd.n == instr.nmd.m) {
        const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
        try block.movsx(.{ .reg = rn }, .{ .reg8 = rn });
    } else {
        const rn = try get_register_for_writing(block, ctx, instr.nmd.n);
        const rm = try load_register(block, ctx, instr.nmd.m);
        try block.movsx(rn, .{ .reg8 = rm });
    }
    return false;
}

pub fn extsw_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    if (instr.nmd.n == instr.nmd.m) {
        const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
        try block.movsx(.{ .reg = rn }, .{ .reg16 = rn });
    } else {
        const rn = try get_register_for_writing(block, ctx, instr.nmd.n);
        const rm = try load_register(block, ctx, instr.nmd.m);
        try block.movsx(rn, .{ .reg16 = rm });
    }
    return false;
}

pub fn extub_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    if (instr.nmd.n == instr.nmd.m) {
        const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
        try block.append(.{ .And = .{ .dst = .{ .reg = rn }, .src = .{ .imm32 = 0x000000FF } } });
    } else {
        const rn = try get_register_for_writing(block, ctx, instr.nmd.n);
        const rm = try load_register(block, ctx, instr.nmd.m);
        try block.mov(rn, .{ .reg8 = rm });
    }
    return false;
}

pub fn extuw_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    if (instr.nmd.n == instr.nmd.m) {
        const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
        try block.append(.{ .And = .{ .dst = .{ .reg = rn }, .src = .{ .imm32 = 0x0000FFFF } } });
    } else {
        const rn = try get_register_for_writing(block, ctx, instr.nmd.n);
        const rm = try load_register(block, ctx, instr.nmd.m);
        try block.mov(rn, .{ .reg16 = rm });
    }
    return false;
}

pub fn macl_atRmInc_atRnInc(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // NOTE: SR.S == 1 mode not implemented.

    try load_mem(block, ctx, ReturnRegister, instr.nmd.n, .Reg, 0, 32);
    try block.movsx(.{ .reg64 = ReturnRegister }, .{ .reg = ReturnRegister }); // Sign extend to 64bits
    try block.push(.{ .reg = ReturnRegister }); // load_mem will make some function calls, make sure to save the first fetched operand.
    try block.push(.{ .reg = ReturnRegister }); // Twice to stay 16 bytes aligned.

    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 32);
    try block.movsx(.{ .reg64 = ArgRegisters[0] }, .{ .reg = ReturnRegister });

    try block.pop(.{ .reg = ReturnRegister });
    try block.pop(.{ .reg = ReturnRegister });

    try block.append(.{ .Mul = .{ .dst = .{ .reg64 = ReturnRegister }, .src = .{ .reg64 = ArgRegisters[0] } } });

    std.debug.assert(@offsetOf(sh4.SH4, "macl") + 4 == @offsetOf(sh4.SH4, "mach"));
    try block.add(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "macl"), .size = 64 } }, .{ .reg = ReturnRegister });

    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.add(.{ .reg = rn }, .{ .imm32 = 4 });
    const rm = try load_register_for_writing(block, ctx, instr.nmd.m);
    try block.add(.{ .reg = rm }, .{ .imm32 = 4 });
    return false;
}

pub fn macw_atRmInc_atRnInc(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // NOTE: SR.S == 1 mode not implemented.

    sh4_jit_log.warn("Emitting mac.w @Rm+,@Rn+: This is untested!", .{});

    try load_mem(block, ctx, ReturnRegister, instr.nmd.n, .Reg, 0, 16);
    try block.movsx(.{ .reg64 = ReturnRegister }, .{ .reg16 = ReturnRegister }); // Sign extend to 64bits
    try block.push(.{ .reg = ReturnRegister }); // load_mem will make some function calls, make sure to save the first fetched operand.
    try block.push(.{ .reg = ReturnRegister }); // Twice to stay 16 bytes aligned.

    try load_mem(block, ctx, ReturnRegister, instr.nmd.m, .Reg, 0, 16);
    try block.movsx(.{ .reg64 = ArgRegisters[0] }, .{ .reg16 = ReturnRegister });

    try block.pop(.{ .reg = ReturnRegister });
    try block.pop(.{ .reg = ReturnRegister });

    try block.append(.{ .Mul = .{ .dst = .{ .reg64 = ReturnRegister }, .src = .{ .reg64 = ArgRegisters[0] } } });

    std.debug.assert(@offsetOf(sh4.SH4, "macl") + 4 == @offsetOf(sh4.SH4, "mach"));
    try block.add(.{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(sh4.SH4, "macl"), .size = 64 } }, .{ .reg = ReturnRegister });

    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.add(.{ .reg = rn }, .{ .imm32 = 2 });
    const rm = try load_register_for_writing(block, ctx, instr.nmd.m);
    try block.add(.{ .reg = rm }, .{ .imm32 = 2 });
    return false;
}

pub fn mull_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    const tmp: JIT.Operand = .{ .reg = ReturnRegister };
    try block.mov(tmp, .{ .reg = rn });
    try block.append(.{ .Mul = .{ .dst = tmp, .src = .{ .reg = rm } } });
    try block.mov(sh4_mem("macl"), tmp);
    return false;
}

pub fn mulsw_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.movsx(.{ .reg = ReturnRegister }, .{ .reg16 = rn });
    try block.movsx(.{ .reg = ArgRegisters[0] }, .{ .reg16 = rm });
    try block.append(.{ .Mul = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .reg = ArgRegisters[0] } } });
    try block.mov(sh4_mem("macl"), .{ .reg = ReturnRegister });
    return false;
}

pub fn muluw_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg16 = rn });
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg16 = rm });
    try block.append(.{ .Mul = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .reg = ArgRegisters[0] } } });
    try block.mov(sh4_mem("macl"), .{ .reg = ReturnRegister });
    return false;
}

pub fn neg_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    if (instr.nmd.n == instr.nmd.m) {
        const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
        try block.append(.{ .Neg = .{ .dst = .{ .reg = rn } } });
    } else {
        const rn = try get_register_for_writing(block, ctx, instr.nmd.n);
        const rm = try load_register(block, ctx, instr.nmd.m);
        try block.mov(rn, .{ .reg = rm });
        try block.append(.{ .Neg = .{ .dst = rn } });
    }
    return false;
}

pub fn sub_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.sub(.{ .reg = rn }, .{ .reg = rm });
    return false;
}

pub fn and_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.append(.{ .And = .{ .dst = .{ .reg = rn }, .src = .{ .reg = rm } } });
    return false;
}

pub fn and_imm_R0(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const r0 = try load_register_for_writing(block, ctx, 0);
    try block.append(.{ .And = .{ .dst = .{ .reg = r0 }, .src = .{ .imm32 = bit_manip.zero_extend(instr.nd8.d) } } });
    return false;
}

pub fn not_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.mov(.{ .reg = rn }, .{ .reg = rm });
    try block.append(.{ .Not = .{ .dst = .{ .reg = rn } } });
    return false;
}

pub fn or_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.append(.{ .Or = .{ .dst = .{ .reg = rn }, .src = .{ .reg = rm } } });
    return false;
}

pub fn or_imm_R0(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const r0 = try load_register_for_writing(block, ctx, 0);
    try block.append(.{ .Or = .{ .dst = .{ .reg = r0 }, .src = .{ .imm32 = bit_manip.zero_extend(instr.nd8.d) } } });
    return false;
}

pub fn tst_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // sr.t = (Rm & Rn) == 0
    if (instr.nmd.n == instr.nmd.m) {
        const rn = try load_register(block, ctx, instr.nmd.n);
        try block.append(.{ .Cmp = .{ .lhs = .{ .reg = rn }, .rhs = .{ .imm32 = 0 } } });
        try set_t(block, ctx, .Equal);
    } else {
        const rn = try load_register(block, ctx, instr.nmd.n);
        const rm = try load_register(block, ctx, instr.nmd.m);
        try block.mov(.{ .reg = ReturnRegister }, .{ .reg = rn });
        try block.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .reg = rm } } }); // TODO: Use an actual TEST instruction instead?
        try set_t(block, ctx, .Zero);
    }
    return false;
}

pub fn tst_imm_R0(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const r0 = try load_register(block, ctx, 0);
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = r0 });
    try block.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = bit_manip.zero_extend(instr.nd8.d) } } });
    try block.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = 0 } } });
    try set_t(block, ctx, .Zero);
    return false;
}

pub fn xor_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.append(.{ .Xor = .{ .dst = .{ .reg = rn }, .src = .{ .reg = rm } } });
    return false;
}

pub fn xor_imm_R0(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const r0 = try load_register_for_writing(block, ctx, 0);
    try block.append(.{ .Xor = .{ .dst = .{ .reg = r0 }, .src = .{ .imm32 = bit_manip.zero_extend(instr.nd8.d) } } });
    return false;
}

pub fn rotcl_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try load_t(block, ctx);
    try block.append(.{ .Rcl = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 1 } } });
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn rotcr_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try load_t(block, ctx);
    try block.append(.{ .Rcr = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 1 } } });
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn rotl_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Rol = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 1 } } });
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn rotr_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Ror = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 1 } } });
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn shad_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);

    const amount: JIT.Operand = .{ .reg = .rcx };

    try block.mov(amount, .{ .reg = rm });
    try block.append(.{ .Cmp = .{ .lhs = amount, .rhs = .{ .imm32 = 0 } } });
    var neg = try block.jmp(.Less);

    try block.shl(.{ .reg = rn }, amount);
    var end = try block.jmp(.Always);
    defer end.patch();

    neg.patch();
    try block.append(.{ .Cmp = .{ .lhs = amount, .rhs = .{ .imm32 = 0xFFFFE000 } } });
    var rm_neg_zero = try block.jmp(.Equal);
    try block.append(.{ .Neg = .{ .dst = amount } });
    try block.append(.{ .Sar = .{ .dst = .{ .reg = rn }, .amount = amount } });
    var end_2 = try block.jmp(.Always);
    defer end_2.patch();

    // Right shift by zero special case.
    rm_neg_zero.patch();
    {
        try block.append(.{ .Cmp = .{ .lhs = .{ .reg = rn }, .rhs = .{ .imm32 = 0 } } });
        var rn_neg = try block.jmp(.Less);
        try block.mov(.{ .reg = rn }, .{ .imm32 = 0 });
        var end_3 = try block.jmp(.Always);
        defer end_3.patch();

        rn_neg.patch();
        try block.mov(.{ .reg = rn }, .{ .imm32 = 0xFFFFFFFF });
    }

    return false;
}

pub fn shal_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    sh4_jit_log.warn("Emitting untested instruction: shal_Rn", .{});
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Shl = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 1 } } });
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn shar_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.append(.{ .Sar = .{ .dst = .{ .reg = rn }, .amount = .{ .imm8 = 1 } } });
    try set_t(block, ctx, .Carry);
    return false;
}

fn shld(n: u32, m: u32) u32 {
    const sign = m & 0x80000000;
    if (sign == 0) {
        return n << @intCast(m & 0x1F);
    } else if ((m & 0x1F) == 0) {
        return 0;
    } else {
        return n >> @intCast(((~m) & 0x1F) + 1);
    }
}
pub fn shld_Rm_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    const rm = try load_register(block, ctx, instr.nmd.m);
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = rn });
    try block.mov(.{ .reg = ArgRegisters[1] }, .{ .reg = rm });
    try call(block, ctx, shld); // TODO: Getting rid of this call will require implementing the TEST instruction in the x86_64 backend.
    try block.mov(.{ .reg = rn }, .{ .reg = ReturnRegister });
    return false;
}

pub fn shll(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.shl(.{ .reg = rn }, .{ .imm8 = 1 });
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn shll2(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.shl(.{ .reg = rn }, .{ .imm8 = 2 });
    return false;
}

pub fn shll8(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.shl(.{ .reg = rn }, .{ .imm8 = 8 });
    return false;
}

pub fn shll16(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.shl(.{ .reg = rn }, .{ .imm8 = 16 });
    return false;
}

pub fn shlr(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.shr(.{ .reg = rn }, 1);
    // The CF flag contains the value of the last bit shifted out of the destination operand.
    try set_t(block, ctx, .Carry);
    return false;
}

pub fn shlr2(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.shr(.{ .reg = rn }, 2);
    return false;
}

pub fn shlr8(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.shr(.{ .reg = rn }, 8);
    return false;
}

pub fn shlr16(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.shr(.{ .reg = rn }, 16);
    return false;
}

fn default_conditional_branch(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr, comptime jump_if: bool, comptime delay_slot: bool) !bool {
    const dest = sh4_interpreter.d8_disp(ctx.current_pc, instr);
    try load_t(block, ctx);

    try block.mov(.{ .reg = ReturnRegister }, .{ .imm32 = dest });
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .imm32 = ctx.current_pc + if (delay_slot) 4 else 2 });
    try block.cmov(if (jump_if) .NotCarry else .Carry, .{ .reg = ReturnRegister }, .{ .reg = ArgRegisters[0] });
    try block.mov(sh4_mem("pc"), .{ .reg = ReturnRegister });

    ctx.outdated_pc = false;
    if (delay_slot) try ctx.compile_delay_slot(block);

    return true;
}

fn conditional_branch(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr, comptime jump_if: bool, comptime delay_slot: bool) !bool {
    const dest = sh4_interpreter.d8_disp(ctx.current_pc, instr);

    // Jump back at the start of this block
    if (Optimizations.inline_jumps_to_start_of_block and dest == ctx.entry_point_address and ctx.cycles < MaxCyclesPerBlock) {
        if (delay_slot) {
            // Delay slot might change the T bit, push it.
            try block.mov(.{ .reg = ReturnRegister }, sh4_mem("sr"));
            try block.push(.{ .reg = ReturnRegister });
            try block.push(.{ .reg = ReturnRegister }); // Twice to stay 16 bytes aligned.
            try ctx.compile_delay_slot(block);
        }

        const loop_cycles = ctx.cycles + instr_lookup(@bitCast(instr)).issue_cycles;

        // Here we have to flush all registers to memory since jumping back to the start of the block means reloading them from memory.
        try ctx.gpr_cache.commit_and_invalidate_all(block);
        try ctx.fpr_cache.commit_and_invalidate_all(block);

        if (delay_slot) {
            try block.pop(.{ .reg = ReturnRegister });
            try block.pop(.{ .reg = ReturnRegister });
        } else {
            try block.mov(.{ .reg = ReturnRegister }, sh4_mem("sr"));
        }
        try block.bit_test(ReturnRegister, @bitOffsetOf(sh4.SR, "t"));
        var not_taken = try block.jmp(if (jump_if) .NotCarry else .Carry);

        // Break out if we already spent too many cycles here. NOTE: This doesn't currently take the base cycles into account.
        try block.mov(.{ .reg = ReturnRegister }, sh4_mem("_pending_cycles"));
        try block.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = MaxCyclesPerBlock } } });
        var break_loop = try block.jmp(.Greater);

        // Count cycles spent into one traversal of the loop.
        try block.add(.{ .reg = ReturnRegister }, .{ .imm32 = loop_cycles });
        try block.mov(sh4_mem("_pending_cycles"), .{ .reg = ReturnRegister });

        const rel: i32 = @as(i32, @intCast(ctx.start_index)) - @as(i32, @intCast(block.instructions.items.len));
        try block.append(.{ .Jmp = .{ .condition = .Always, .dst = .{ .rel = rel } } }); // Jump back

        break_loop.patch(); // Branch taken, but we already spend enough cycles here. Break out to handle interrupts.
        try block.mov(sh4_mem("pc"), .{ .imm32 = dest });
        var to_end = try block.jmp(.Always);

        not_taken.patch();
        try block.mov(sh4_mem("pc"), .{ .imm32 = ctx.current_pc + if (delay_slot) 4 else 2 });
        to_end.patch();

        ctx.may_have_pending_cycles = true;
        ctx.outdated_pc = false;
        return true;
    }

    if (Optimizations.inline_small_forward_jumps) {
        // Optimize small forward jumps if possible
        const max_instructions = 6;
        const first_instr = if (delay_slot) 2 else 1;
        if (dest > ctx.current_pc and (dest - ctx.current_pc) / 2 < max_instructions + first_instr) {
            const instr_count = (dest - ctx.current_pc) / 2 - first_instr;
            // Make sure there's no jump in there
            for (first_instr..first_instr + instr_count) |i| {
                const op = instr_lookup(ctx.instructions[ctx.index + i]);
                if (op.is_branch or op.use_fallback())
                    return default_conditional_branch(block, ctx, instr, jump_if, delay_slot);
            }
            // All good, we can inline the branch.

            if (delay_slot) {
                // Delay slot might change the T bit, push it.
                try block.mov(.{ .reg = ReturnRegister }, sh4_mem("sr"));
                try block.push(.{ .reg = ReturnRegister });
                try block.push(.{ .reg = ReturnRegister }); // Twice to stay 16 bytes aligned.
                try ctx.compile_delay_slot(block);
                ctx.index += 1;
                ctx.current_pc += 2;
                ctx.current_physical_pc += 2;
            }
            // Since this is conditionally executed, cache isn't reliable anymore. Invalidating it is the easy way out.
            // This might seem like a heavy cost, but the alternative is simply to end the block here.
            // A first step into improving this would be to keep track of register usage in the conditional block and only invalidate the relevant registers.
            try ctx.gpr_cache.commit_and_invalidate_all(block);
            try ctx.fpr_cache.commit_and_invalidate_all(block);

            if (delay_slot) {
                try block.pop(.{ .reg = ReturnRegister });
                try block.pop(.{ .reg = ReturnRegister });
            } else {
                try block.mov(.{ .reg = ReturnRegister }, sh4_mem("sr"));
            }
            try block.bit_test(ReturnRegister, @bitOffsetOf(sh4.SR, "t"));
            var taken = try block.jmp(if (jump_if) .Carry else .NotCarry);

            var optional_cycles: u32 = 0;
            for (0..instr_count) |_| {
                ctx.index += 1;
                ctx.current_pc += 2;
                ctx.current_physical_pc += 2;
                const i = ctx.instructions[ctx.index];
                const op = instr_lookup(i);
                if (comptime std.log.logEnabled(.debug, .sh4_jit))
                    sh4_jit_log.debug("[{X:0>8}] {s} > {s}", .{
                        ctx.current_pc,
                        if (op.use_fallback()) "!" else " ",
                        try sh4_disassembly.disassemble(@bitCast(i), ctx.cpu._allocator),
                    });
                const branch = try op.jit_emit_fn(block, ctx, @bitCast(i));
                std.debug.assert(!branch);
                optional_cycles += op.issue_cycles;
            }
            try block.add(sh4_mem("_pending_cycles"), .{ .imm32 = optional_cycles });

            ctx.may_have_pending_cycles = true;

            try ctx.gpr_cache.commit_and_invalidate_all(block);
            try ctx.fpr_cache.commit_and_invalidate_all(block);

            taken.patch();

            return false;
        }
    }

    return default_conditional_branch(block, ctx, instr, jump_if, delay_slot);
}

pub fn bf_label(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return conditional_branch(block, ctx, instr, false, false);
}
pub fn bfs_label(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return conditional_branch(block, ctx, instr, false, true);
}
pub fn bt_label(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return conditional_branch(block, ctx, instr, true, false);
}
pub fn bts_label(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    return conditional_branch(block, ctx, instr, true, true);
}

pub fn bra_label(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const dest = sh4_interpreter.d12_disp(ctx.current_pc, instr);
    // Unconditional branch, and fixed destination, don't treat it like a branch.
    if (dest > ctx.current_pc) {
        try ctx.compile_delay_slot(block);
        ctx.skip_instructions((dest - ctx.current_pc) / 2 - 1);
        return false;
    } else if (Optimizations.inline_backwards_bra and dest < ctx.start_pc and !ctx.mmu_enabled) {
        try ctx.compile_delay_slot(block);
        // FIXME: Very hackish. We don't normally have access to previous instructions. May lead to infinite loops?
        sh4_jit_log.info("(Unstable) Inlining backwards bra instruction: {X:0>8} -> {X:0>8}", .{ ctx.current_pc, dest });
        // ctx.index and ctx.address will be incremented right after, so point to one instruction before.
        ctx.instructions = @ptrFromInt(@intFromPtr(ctx.instructions) - (ctx.start_pc - dest + 2));
        ctx.index = 0;
        ctx.current_pc = dest - 2;
        ctx.start_pc = dest - 2;
        std.debug.assert(!ctx.mmu_enabled);
        ctx.current_physical_pc = (dest - 2) & 0x1FFFFFFF;
        return false;
    } else {
        try block.mov(sh4_mem("pc"), .{ .imm32 = dest });
        ctx.outdated_pc = false;
        try ctx.compile_delay_slot(block);
        return true;
    }
}

pub fn braf_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // pc += Rn + 4;
    const rn = try load_register(block, ctx, instr.nmd.n);
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = rn });
    try block.add(.{ .reg = ReturnRegister }, .{ .imm32 = 4 + ctx.current_pc });
    try block.mov(sh4_mem("pc"), .{ .reg = ReturnRegister });

    ctx.outdated_pc = false;
    try ctx.compile_delay_slot(block);
    return true;
}

pub fn bsr_label(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const dest = sh4_interpreter.d12_disp(ctx.current_pc, instr);
    try block.mov(sh4_mem("pc"), .{ .imm32 = dest });

    ctx.outdated_pc = false;
    try ctx.compile_delay_slot(block);

    // NOTE: If the accepted exception (the highest-priority exception) is a delay slot instruction re-
    //       execution type exception, the branch instruction PR register write operation (PC → PR
    //       operation performed in BSR, BSRF, JSR) is inhibited.

    // pr = pc + 4
    try block.mov(sh4_mem("pr"), .{ .imm32 = ctx.current_pc + 4 });
    return true;
}

pub fn bsrf_Rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // pc += Rn + 4;
    const rn = try load_register(block, ctx, instr.nmd.n);
    try block.mov(.{ .reg = ReturnRegister }, .{ .reg = rn });
    try block.add(.{ .reg = ReturnRegister }, .{ .imm32 = 4 + ctx.current_pc });
    try block.mov(sh4_mem("pc"), .{ .reg = ReturnRegister });

    ctx.outdated_pc = false;
    try ctx.compile_delay_slot(block);

    // pr = pc + 4
    try block.mov(sh4_mem("pr"), .{ .imm32 = ctx.current_pc + 4 });
    return true;
}

pub fn jmp_atRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // pc = Rn
    const rn = try load_register(block, ctx, instr.nmd.n);
    try block.mov(sh4_mem("pc"), .{ .reg = rn });

    ctx.outdated_pc = false;
    try ctx.compile_delay_slot(block);
    return true;
}

pub fn jsr_rn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // cpu.pc = Rn
    const rn = try load_register(block, ctx, instr.nmd.n);
    try block.mov(sh4_mem("pc"), .{ .reg = rn });

    ctx.outdated_pc = false;
    try ctx.compile_delay_slot(block);

    // cpu.pr = cpu.pc + 4;
    try block.mov(sh4_mem("pr"), .{ .imm32 = ctx.current_pc + 4 });
    return true;
}

pub fn rts(block: *IRBlock, ctx: *JITContext, _: sh4.Instr) !bool {
    // cpu.pc = cpu.pr
    try block.mov(.{ .reg = ReturnRegister }, sh4_mem("pr"));
    try block.mov(sh4_mem("pc"), .{ .reg = ReturnRegister });

    ctx.outdated_pc = false;
    try ctx.compile_delay_slot(block);
    return true;
}

pub fn clrmac(block: *IRBlock, _: *JITContext, _: sh4.Instr) !bool {
    try block.mov(sh4_mem("mach"), .{ .imm32 = 0 });
    try block.mov(sh4_mem("macl"), .{ .imm32 = 0 });
    return false;
}

fn set_sr(block: *IRBlock, ctx: *JITContext, sr_value: JIT.Operand) !void {
    // We might change GPR banks.
    for (0..8) |i| {
        try ctx.gpr_cache.commit_and_invalidate(block, @intCast(i));
    }

    // call set_sr
    try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    try block.mov(.{ .reg = ArgRegisters[1] }, sr_value);
    try call(block, ctx, sh4.SH4.set_sr);
}

pub fn rte(block: *IRBlock, ctx: *JITContext, _: sh4.Instr) !bool {
    // NOTE: This differs from the interpreter behavior, but I'm nut sure which one is actually correct, and
    //       the other one is harder to implement in JIT.
    try set_sr(block, ctx, sh4_mem("ssr"));
    // pc = spc
    try block.mov(.{ .reg = ReturnRegister }, sh4_mem("spc"));
    try block.mov(sh4_mem("pc"), .{ .reg = ReturnRegister });

    ctx.outdated_pc = false;
    try ctx.compile_delay_slot(block);
    return true;
}

pub fn sett(block: *IRBlock, _: *JITContext, _: sh4.Instr) !bool {
    // Warning: Invalid t here if we ever cache it. (set_t/load_t)
    var sr = sh4_mem("sr");
    sr.mem.size = 8;
    try block.append(.{ .Or = .{ .dst = sr, .src = .{ .imm8 = @as(u8, 1) << @bitOffsetOf(sh4.SR, "t") } } });
    return false;
}
pub fn clrt(block: *IRBlock, _: *JITContext, _: sh4.Instr) !bool {
    // Warning: Invalid t here if we ever cache it. (set_t/load_t)
    var sr = sh4_mem("sr");
    sr.mem.size = 8;
    try block.append(.{ .And = .{ .dst = sr, .src = .{ .imm8 = ~(@as(u8, 1) << @bitOffsetOf(sh4.SR, "t")) } } });
    return false;
}

pub fn sets(block: *IRBlock, _: *JITContext, _: sh4.Instr) !bool {
    try block.append(.{ .Or = .{ .dst = sh4_mem("sr"), .src = .{ .imm32 = @as(u32, 1) << @bitOffsetOf(sh4.SR, "s") } } });
    return false;
}
pub fn clrs(block: *IRBlock, _: *JITContext, _: sh4.Instr) !bool {
    try block.append(.{ .And = .{ .dst = sh4_mem("sr"), .src = .{ .imm32 = ~(@as(u32, 1) << @bitOffsetOf(sh4.SR, "s")) } } });
    return false;
}

pub fn ldc_Rn_Reg(comptime reg: []const u8) fn (block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) anyerror!bool {
    std.debug.assert(!std.mem.eql(u8, reg, "sr"));
    const T = struct {
        fn ldc(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
            const rn = try load_register(block, ctx, instr.nmd.n);
            try block.mov(sh4_mem(reg), .{ .reg = rn });
            return false;
        }
    };
    return T.ldc;
}

pub fn ldc_Rn_SR(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    const rn = try load_register(block, ctx, instr.nmd.n);
    try set_sr(block, ctx, .{ .reg = rn });
    return false;
}

pub fn ldcl_atRnInc_Reg(comptime reg: []const u8) fn (block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) anyerror!bool {
    std.debug.assert(!std.mem.eql(u8, reg, "sr"));
    const T = struct {
        fn ldcl(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
            try load_mem(block, ctx, ReturnRegister, instr.nmd.n, .Reg, 0, 32);
            try block.mov(sh4_mem(reg), .{ .reg = ReturnRegister });

            const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
            try block.add(.{ .reg = rn }, .{ .imm32 = 4 });
            return false;
        }
    };
    return T.ldcl;
}

pub fn ldcl_atRnInc_SR(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    try load_mem(block, ctx, ReturnRegister, instr.nmd.n, .Reg, 0, 32);
    try set_sr(block, ctx, .{ .reg = ReturnRegister });
    const rn = try load_register_for_writing(block, ctx, instr.nmd.n);
    try block.add(.{ .reg = rn }, .{ .imm32 = 4 });
    return false;
}

pub fn movcal_R0_atRn(block: *IRBlock, ctx: *JITContext, instr: sh4.Instr) !bool {
    // Stores the contents of general register R0 in the memory location indicated by effective address Rn. This instruction differs from other store instructions as follows.
    // If write-back is selected for the accessed memory, and a cache miss occurs, the cache block will be allocated but an R0 data write will be performed to that cache block without performing a block read. Other cache block contents are undefined.
    const r0 = try load_register(block, ctx, 0);
    try store_mem(block, ctx, instr.nmd.n, .Reg, 0, .{ .reg = r0 }, 32);
    return false;
}
