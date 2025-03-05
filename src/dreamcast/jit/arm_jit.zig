const std = @import("std");
const builtin = @import("builtin");

const termcolor = @import("termcolor");

const host_memory = @import("../host/host_memory.zig");

const arm7 = @import("arm7");
const bit_manip = @import("../bit_manip.zig");
const JIT = @import("ir_block.zig");
const IRBlock = JIT.IRBlock;

const Architecture = @import("x86_64.zig");
const ReturnRegister = Architecture.ReturnRegister;
const ArgRegisters = Architecture.ArgRegisters;
const SavedRegisters = Architecture.SavedRegisters;

const BasicBlock = struct {
    offset: u32,
    cycles: u32,

    pub inline fn execute(self: *const @This(), buffer: []const u8, user_data: *anyopaque) void {
        @setRuntimeSafety(false);
        @as(*const fn (*anyopaque) void, @ptrCast(&buffer[self.offset]))(user_data);
    }

    pub inline fn is_valid(self: *const @This()) bool {
        return self.offset != 0 and self.cycles != 0;
    }
};

const Dreamcast = @import("../dreamcast.zig").Dreamcast;

const arm_jit_log = std.log.scoped(.arm_jit);

const BlockBufferSize = 2 * 1024 * 1024;
const MaxCyclesPerBlock = 32;
const DebugAlwaysFallbackToInterpreter = false;

const BlockCache = struct {
    const BlockEntryCount = 0x200000 >> 2;

    buffer: []align(std.mem.page_size) u8,
    cursor: usize = 0,
    blocks: []BasicBlock = undefined,

    min_address: u32 = std.math.maxInt(u32),
    max_address: u32 = 0,

    addr_mask: u32 = 0xFFFFFFFF,

    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, addr_mask: u32) !@This() {
        var r: @This() = .{
            .buffer = try host_memory.allocate_executable(allocator, BlockBufferSize),
            .addr_mask = addr_mask,
            ._allocator = allocator,
        };
        try r.allocate_blocks();
        return r;
    }

    pub fn deinit(self: *@This()) void {
        self._allocator.free(self.buffer);
        self.deallocate_blocks();
    }

    pub fn signal_write(self: *@This(), addr: u32) void {
        if (addr >= self.min_address and addr <= self.max_address) {
            arm_jit_log.debug(termcolor.blue("Signal write: {X:0>8} in [{X:0>8}, {X:0>8}]"), .{ addr, self.min_address, self.max_address });
            // NOTE: Currently cycles are a perfect stand in for instruction count. This might change in the future.
            //       This should still be a good upper bound though, so it's not critical.
            // Invalidate blocks containing the address.
            const from: u32 = @intCast(@max(0, @as(i64, @intCast(addr & 0xFFFFFFFC)) - 4 * MaxCyclesPerBlock));
            var instr_addr = from;
            while (instr_addr <= addr) : (instr_addr += 4) {
                const block = self.get(instr_addr);
                if (block.is_valid() and instr_addr + 4 * (block.cycles - 1) >= 4 * MaxCyclesPerBlock) {
                    self.invalidate(instr_addr);
                }
            }
        }
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

        arm_jit_log.info(termcolor.blue("Resetting block cache."), .{});

        self.cursor = 0;
        self.min_address = std.math.maxInt(u32);
        self.max_address = 0;

        self.deallocate_blocks();
        try self.allocate_blocks();
    }

    pub fn invalidate(self: *@This(), address: u32) void {
        self.blocks[(address & self.addr_mask) >> 2] = .{ .offset = 0, .cycles = 0 };
    }

    pub fn get(self: *@This(), address: u32) BasicBlock {
        return self.blocks[(address & self.addr_mask) >> 2];
    }

    pub fn put(self: *@This(), address: u32, end_address: u32, block: BasicBlock) void {
        self.min_address = @min(self.min_address, address & self.addr_mask);
        self.max_address = @max(self.max_address, end_address & self.addr_mask);

        self.blocks[(address & self.addr_mask) >> 2] = block;
    }
};

pub const JITContext = struct {
    address: u32,
    cpu: *arm7.ARM7,

    did_fallback: bool = false, // Only there for analysis.
};

pub const ARM7JIT = struct {
    block_cache: BlockCache,

    _working_block: IRBlock,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, addr_mask: u32) !@This() {
        var r = @This(){
            .block_cache = try .init(allocator, addr_mask),
            ._working_block = try .init(allocator),
            ._allocator = allocator,
        };
        try r.init_compile_and_run_handler();
        return r;
    }

    pub fn deinit(self: *@This()) void {
        self._working_block.deinit();
        self.block_cache.deinit();
    }

    pub fn reset(self: *@This()) !void {
        try self.block_cache.reset();
        try self.init_compile_and_run_handler();
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

    pub noinline fn run_for(self: *@This(), cpu: *arm7.ARM7, cycles: i32) !i32 {
        if (!cpu.running) return 0;

        var spent_cycles: u32 = 0;
        while (spent_cycles < cycles) {
            const pc = (cpu.pc() -% 4) & cpu.memory_address_mask; // Pipelining...
            const block = self.block_cache.get(pc);
            // arm_jit_log.debug("Running {X:0>8} ({} cycles)", .{ pc, block.?.cycles });
            block.execute(self.block_cache.buffer, cpu);

            spent_cycles += block.cycles;

            cpu.check_fiq(); // FIXME: Non-tested.

            if (cpu.pc() > cpu.memory_address_mask) {
                @branchHint(.unlikely);
                arm_jit_log.warn("arm7: PC out of bounds: {X:0>8}, stopping.", .{cpu.pc()});
                cpu.running = false;
                break;
            }
        }

        // Not necessary, just here to allow compatibility with the interpreter if we need it.
        // (Right now we're always calling to the interpreter so this should stay in sync, but once
        //  we start actually JITing some instructions, we won't keep instruction_pipeline updated)
        cpu.instruction_pipeline[0] = @as(*const u32, @alignCast(@ptrCast(&cpu.memory[(cpu.pc() -% 4) & cpu.memory_address_mask]))).*;

        return @intCast(spent_cycles);
    }

    // Default handler sitting the offset 0 of our executable buffer
    pub noinline fn compile_and_run(cpu: *arm7.ARM7, self: *@This()) void {
        const pc = (cpu.pc() -% 4) & cpu.memory_address_mask; // Pipelining...
        arm_jit_log.debug("(Cache Miss) Compiling {X:0>8}...", .{pc});
        const instructions: [*]u32 = @alignCast(@ptrCast(&cpu.memory[pc]));
        const block = (self.compile(.{ .address = pc, .cpu = cpu }, instructions) catch |err| retry: {
            if (err == error.JITCacheFull) {
                self.block_cache.reset() catch |reset_err| {
                    arm_jit_log.err("Failed to reset ARM JIT: {s}", .{@errorName(reset_err)});
                    std.process.exit(1);
                };
                arm_jit_log.info("JIT cache purged.", .{});
                break :retry self.compile(.{ .address = pc, .cpu = cpu }, instructions);
            } else break :retry err;
        }) catch |err| {
            arm_jit_log.err("Failed to compile {X:0>8}: {s}\n", .{ pc, @errorName(err) });
            std.process.exit(1);
        };
        block.execute(self.block_cache.buffer, cpu);
    }

    pub fn compile(self: *@This(), start_ctx: JITContext, instructions: [*]u32) !BasicBlock {
        var ctx = start_ctx;

        var b = &self._working_block;
        b.clearRetainingCapacity();

        // We'll be using these callee saved registers, push 'em to the stack.
        try b.push(.{ .reg = SavedRegisters[0] });
        try b.push(.{ .reg = SavedRegisters[1] }); // NOTE: We need to align the stack to 16 bytes anyway.

        try b.mov(.{ .reg = SavedRegisters[0] }, .{ .reg = ArgRegisters[0] }); // Save the pointer to the cpu struct

        var cycles: u32 = 0;
        var index: u32 = 0;
        while (true) {
            const instr = instructions[index];

            ctx.did_fallback = false;

            // Update PC. Done before the instruction is executed, emulating fetching.
            try b.add(guest_register(15), .{ .imm32 = 4 }); // PC += 4

            var jump = try handle_condition(b, &ctx, instr);

            const tag = arm7.ARM7.get_instr_tag(instr);
            const branch = try InstructionHandlers[arm7.JumpTable[tag]](b, &ctx, instr);

            if (jump) |*j|
                j.patch();

            if (comptime std.log.logEnabled(.debug, .arm_jit))
                arm_jit_log.debug("  [{X:0>8}] {s} {s}", .{ ctx.address, if (ctx.did_fallback) "!" else " ", arm7.ARM7.disassemble(instr) });

            cycles += 1; // FIXME
            index += 1;
            ctx.address += 4;

            if (branch or cycles > MaxCyclesPerBlock)
                break;
        }

        // Crude appromixation, better purging slightly too often than crashing.
        // Also feels better than checking the length at each insertion.
        if (self.block_cache.cursor + 4 * 32 + 8 * 4 * b.instructions.items.len >= BlockBufferSize) {
            return error.JITCacheFull;
        }

        // Restore callee saved registers.
        try b.pop(.{ .reg = SavedRegisters[1] });
        try b.pop(.{ .reg = SavedRegisters[0] });

        for (b.instructions.items, 0..) |instr, idx|
            arm_jit_log.debug("[{d: >4}] {any}", .{ idx, instr });

        const block_size = try b.emit(self.block_cache.buffer[self.block_cache.cursor..]);
        const block = BasicBlock{
            .offset = @intCast(self.block_cache.cursor),
            .cycles = cycles,
        };
        self.block_cache.cursor += block_size;

        arm_jit_log.debug("Compiled: {X:0>2}", .{self.block_cache.buffer[block.offset..][0..block_size]});

        self.block_cache.put(start_ctx.address, ctx.address, block);
        return block;
    }
};

fn guest_register(arm_reg: u5) JIT.Operand {
    return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(arm7.ARM7, "r") + @sizeOf(u32) * @as(u32, arm_reg), .size = 32 } };
}

fn load_register(b: *IRBlock, host_register: JIT.Register, arm_reg: u5) !void {
    try b.mov(.{ .reg = host_register }, guest_register(arm_reg));
}

fn store_register(b: *IRBlock, arm_reg: u5, value: JIT.Operand) !void {
    try b.mov(guest_register(arm_reg), value);
}

noinline fn read8(self: *arm7.ARM7, address: u32) u8 {
    return self.read(u8, address);
}
noinline fn read32(self: *arm7.ARM7, address: u32) u32 {
    return self.read(u32, address);
}
noinline fn write8(self: *arm7.ARM7, address: u32, value: u8) void {
    self.write(u8, address, value);
}
noinline fn write32(self: *arm7.ARM7, address: u32, value: u32) void {
    self.write(u32, address, value);
}

fn load_wave_memory(b: *IRBlock, ctx: *const JITContext, comptime T: type, dst: JIT.Register, addr: u32) !void {
    const aligned_addr = if (T == u32) addr & 0xFFFFFFFC else addr;
    // TODO: This could be turned into a single movabs, but emitter doesn't support it yet.
    try b.mov(.{ .reg64 = dst }, .{ .imm64 = @intFromPtr(ctx.cpu.memory.ptr) + aligned_addr });
    try b.mov(.{ .reg = dst }, .{ .mem = .{ .base = dst, .displacement = 0, .size = @bitSizeOf(T) } });
    // Apply rotate on non-aligned read
    if (T == u32 and addr & 3 != 0)
        try b.append(.{ .Ror = .{ .dst = .{ .reg = dst }, .amount = .{ .imm8 = @truncate(8 * (addr & 3)) } } });
}

// NOTE: Uses ReturnRegister as a temporary!
fn store_wave_memory(b: *IRBlock, ctx: *const JITContext, comptime T: type, addr: u32, value: JIT.Register) !void {
    const aligned_addr = if (T == u32) addr & 0xFFFFFFFC else addr;
    // TODO: This could be turned into a single movabs, but emitter doesn't support it yet.
    try b.mov(.{ .reg64 = ReturnRegister }, .{ .imm64 = @intFromPtr(ctx.cpu.memory.ptr) + aligned_addr });
    try b.mov(.{ .mem = .{ .base = ReturnRegister, .displacement = 0, .size = @bitSizeOf(T) } }, .{ .reg = value });
}

/// Loads into ReturnRegister
/// NOTE: Uses ArgRegisters 0 and 1. And calls a function, so don't rely on caller saved registers anyway.
fn load_mem(b: *IRBlock, ctx: *const JITContext, comptime T: type, dst: JIT.Register, addr: JIT.Operand) !void {
    switch (addr) {
        .imm32 => |addr_imm32| {
            if ((addr_imm32 & ctx.cpu.external_memory_address_mask) == 0) {
                try load_wave_memory(b, ctx, T, dst, addr_imm32);
            } else {
                // TODO: External memory, fallback for now!
                try b.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
                try b.mov(.{ .reg = ArgRegisters[1] }, .{ .imm32 = addr_imm32 });
                if (T == u8) {
                    try b.call(read8);
                } else if (T == u32) {
                    try b.call(read32);
                } else @compileError("Unsupported type: " ++ @typeName(T));
                if (dst != ReturnRegister)
                    try b.mov(.{ .reg = dst }, .{ .reg = ReturnRegister });
            }
        },
        .reg => |addr_reg| {
            // TODO: Fallback for now. Can't do everything at once.
            if (addr_reg != ArgRegisters[1])
                try b.mov(.{ .reg = ArgRegisters[1] }, .{ .reg = addr_reg });
            try b.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
            if (T == u8) {
                try b.call(read8);
            } else if (T == u32) {
                try b.call(read32);
            } else @compileError("Unsupported type: " ++ @typeName(T));
            if (dst != ReturnRegister)
                try b.mov(.{ .reg = dst }, .{ .reg = ReturnRegister });
        },
        else => return error.UnsupportedAddrOperand,
    }
}

/// NOTE: Overwrites ArgRegisters 0, 1 and 2 (ReturnRegister too). And calls a function, so don't rely on caller saved registers anyway.
fn store_mem(b: *IRBlock, ctx: *const JITContext, comptime T: type, addr: JIT.Operand, value: JIT.Register) !void {
    switch (addr) {
        .imm32 => |addr_imm32| {
            if ((addr_imm32 & ctx.cpu.external_memory_address_mask) == 0) {
                try store_wave_memory(b, ctx, T, addr_imm32, value);
            } else {
                // TODO: External memory, fallback for now!
                if (value != ArgRegisters[2])
                    try b.mov(.{ .reg = ArgRegisters[2] }, .{ .reg = value });
                try b.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
                try b.mov(.{ .reg = ArgRegisters[1] }, .{ .imm32 = addr_imm32 });
                if (T == u8) {
                    try b.call(write8);
                } else if (T == u32) {
                    try b.call(write32);
                } else @compileError("Unsupported type: " ++ @typeName(T));
            }
        },
        .reg => |addr_reg| {
            std.debug.assert(value != ArgRegisters[1]); // It would be overwritten.
            // TODO: Fallback for now.
            if (addr_reg != ArgRegisters[1])
                try b.mov(.{ .reg = ArgRegisters[1] }, .{ .reg = addr_reg });
            if (value != ArgRegisters[2])
                try b.mov(.{ .reg = ArgRegisters[2] }, .{ .reg = value });
            try b.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
            if (T == u8) {
                try b.call(write8);
            } else if (T == u32) {
                try b.call(write32);
            } else @compileError("Unsupported type: " ++ @typeName(T));
        },
        else => return error.UnsupportedAddrOperand,
    }
}

fn cpsr_mask(comptime flags: []const []const u8) u32 {
    var mask: u32 = 0;
    inline for (flags) |flag| {
        mask |= @as(u32, 1 << @bitOffsetOf(arm7.CPSR, flag));
    }
    return mask;
}

fn extract_cpsr_flags(b: *IRBlock, comptime flags: []const []const u8) !void {
    try b.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(arm7.ARM7, "cpsr"), .size = 32 } });
    try b.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = cpsr_mask(flags) } } });
}

fn test_cpsr_flags(b: *IRBlock, comptime flags: []const []const u8, comptime expected_flags: []const []const u8) !JIT.PatchableJump {
    try extract_cpsr_flags(b, flags);
    try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(expected_flags) } } });
    return b.jmp(.NotEqual);
}

// Emits code testing the flags in the CPSR and returns a patchable jump meant to point at the next instruction, taken if the condition is not met.
fn handle_condition(b: *IRBlock, ctx: *JITContext, instruction: u32) !?JIT.PatchableJump {
    _ = ctx;
    const condition = arm7.ARM7.get_instr_condition(instruction);
    switch (condition) {
        .EQ => {
            // return cpu.cpsr.z
            return try test_cpsr_flags(b, &[_][]const u8{"z"}, &[_][]const u8{"z"});
        },
        .NE => {
            // return !cpu.cpsr.z
            return try test_cpsr_flags(b, &[_][]const u8{"z"}, &[_][]const u8{});
        },
        .CS => {
            // return cpu.cpsr.c
            return try test_cpsr_flags(b, &[_][]const u8{"c"}, &[_][]const u8{"c"});
        },
        .CC => {
            // return !cpu.cpsr.c
            return try test_cpsr_flags(b, &[_][]const u8{"c"}, &[_][]const u8{});
        },
        .MI => {
            // return cpu.cpsr.n
            return try test_cpsr_flags(b, &[_][]const u8{"n"}, &[_][]const u8{"n"});
        },
        .PL => {
            // return !cpu.cpsr.n
            return try test_cpsr_flags(b, &[_][]const u8{"n"}, &[_][]const u8{});
        },
        .VS => {
            // return cpu.cpsr.v
            return try test_cpsr_flags(b, &[_][]const u8{"v"}, &[_][]const u8{"v"});
        },
        .VC => {
            // return !cpu.cpsr.v
            return try test_cpsr_flags(b, &[_][]const u8{"v"}, &[_][]const u8{});
        },
        .HI => {
            // return cpu.cpsr.c and !cpu.cpsr.z
            return try test_cpsr_flags(b, &[_][]const u8{ "c", "z" }, &[_][]const u8{"c"});
        },
        .LS => {
            // return !cpu.cpsr.c or cpu.cpsr.z
            std.debug.assert(@bitOffsetOf(arm7.CPSR, "c") < @bitOffsetOf(arm7.CPSR, "z"));
            // FIXME: This is untested.
            try b.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(arm7.ARM7, "cpsr"), .size = 32 } });
            try b.bit_test(ReturnRegister, @bitOffsetOf(arm7.CPSR, "c")); // Set carry flag to 'c'.
            var do_label = try b.jmp(.NotCarry);
            try b.bit_test(ReturnRegister, @bitOffsetOf(arm7.CPSR, "z")); // Set carry flag to 'z'.
            const skip_label = try b.jmp(.NotCarry);
            do_label.patch();
            return skip_label;
        },
        .GE => {
            // return cpu.cpsr.n == cpu.cpsr.v
            try extract_cpsr_flags(b, &[_][]const u8{ "n", "v" });
            // v == 1 and n == 1
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{ "v", "n" }) } } });
            var do_label_0 = try b.jmp(.Equal);
            // v == 0 and n == 0
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{}) } } });
            const skip_label = try b.jmp(.NotEqual);
            do_label_0.patch();
            return skip_label;
        },
        .LT => {
            // return cpu.cpsr.n != cpu.cpsr.v
            try extract_cpsr_flags(b, &[_][]const u8{ "n", "v" });
            // v == 1 and n == 0
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{"v"}) } } });
            var do_label_0 = try b.jmp(.Equal);
            // v == 0 and n == 1
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{"n"}) } } });
            const skip_label = try b.jmp(.NotEqual);
            do_label_0.patch();
            return skip_label;
        },
        .GT => {
            // return !cpu.cpsr.z and (cpu.cpsr.n == cpu.cpsr.v)
            try extract_cpsr_flags(b, &[_][]const u8{ "z", "n", "v" });
            // z == 0 and v == 1 and n == 1
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{ "v", "n" }) } } });
            var do_label_0 = try b.jmp(.Equal);
            // z == 0 and v == 0 and n == 0
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{}) } } });
            const skip_label = try b.jmp(.NotEqual);
            do_label_0.patch();
            return skip_label;
        },
        .LE => {
            // return cpu.cpsr.z or (cpu.cpsr.n != cpu.cpsr.v)
            try b.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(arm7.ARM7, "cpsr"), .size = 32 } });
            try b.bit_test(ReturnRegister, @bitOffsetOf(arm7.CPSR, "z")); // Set carry flag to 'z'.
            var do_label_0 = try b.jmp(.Carry);

            try extract_cpsr_flags(b, &[_][]const u8{ "n", "v" });
            // v == 1 and n == 0
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{"v"}) } } });
            var do_label_1 = try b.jmp(.Equal);
            // v == 0 and n == 1
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{"n"}) } } });
            const skip_label = try b.jmp(.NotEqual);

            do_label_0.patch();
            do_label_1.patch();

            return skip_label;
        },
        .AL => return null,
        .Invalid => {
            arm_jit_log.err(termcolor.red("Invalid condition, instruction: 0x{x}"), .{instruction});
            const skip_label = try b.jmp(.Always);
            return skip_label;
        },
    }
}

pub const InstructionHandlers = [_]*const fn (b: *IRBlock, ctx: *JITContext, instruction: u32) anyerror!bool{
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

// NOTE: With this stupid setup, SavedRegisters[0] is a pointer to the cpu struct and should not be changed.
//       SavedRegisters[1] is the only register we save and that will survive a function call (e.g. load_mem/store_mem).

fn handle_branch_and_exchange(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    std.debug.panic("Unimplemented branch and exchange", .{});
}

fn handle_block_data_transfer(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.BlockDataTransferInstruction = @bitCast(instruction);
    try interpreter_fallback(b, ctx, instruction);
    return inst.l == 1 and inst.reg(15);
}

fn handle_branch(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.BranchInstruction = @bitCast(instruction);
    if (DebugAlwaysFallbackToInterpreter) {
        try interpreter_fallback(b, ctx, instruction);
        return true;
    }

    const offset: u32 = arm7.sign_extend(@TypeOf(inst.offset), inst.offset) << 2;

    if (inst.l == 1) {
        try load_register(b, ReturnRegister, 15);
        try b.sub(.{ .reg = ReturnRegister }, .{ .imm32 = 4 });
        try b.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = 0xFFFFFFFC } } });
        try store_register(b, 14, .{ .reg = ReturnRegister });
    }

    // +4 to simulate next prefetch
    try b.add(guest_register(15), .{ .imm32 = offset +% 4 });

    return true;
}

fn handle_software_interrupt(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    try interpreter_fallback(b, ctx, instruction);
    return true;
}

fn handle_undefined(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    arm_jit_log.err("Unimplemented undefined", .{});
    @panic("Unimplemented undefined");
}

// Directly taken from the interpreter: Generate specialized version of the different versions of the instruction.
// We miss some optimisation opportunities here: For example if Rn is 15, read is PC relative and could be inlined to
// a wave_memory_read.
fn comptime_handle_single_data_transfer(comptime i: u1, comptime u: u1, comptime w: u1, comptime p: u1, comptime l: u1, comptime b: u1) type {
    return struct {
        fn handler(cpu: *arm7.ARM7, instruction: u32) void {
            const inst: arm7.SingleDataTransferInstruction = @bitCast(instruction);

            var offset: u32 = inst.offset;
            if (comptime i == 1) { // Offset is a register
                var sro: arm7.interpreter.ScaledRegisterOffset = @bitCast(inst.offset);
                if (sro.register_specified == 1) sro.register_specified = 0;
                offset = arm7.interpreter.offset_from_register(cpu, @bitCast(sro)).shifter_operand;
            }

            const signed_offset: u32 = if (comptime u == 1) offset else (~offset +% 1);

            const base = cpu.r[inst.rn];
            const addr: u32 = @bitCast(if (comptime p == 1) base +% signed_offset else base);

            if (comptime l == 0) {
                // Store to memory
                var val = cpu.r[inst.rd];

                if (inst.rd == 15) val += 4;

                if (comptime b == 1)
                    cpu.write(u8, addr, @truncate(val))
                else {
                    cpu.write(u32, addr, val);
                }
                // Post-indexed data transfers always write back the modified base.
                if (comptime w == 1 or p == 0)
                    cpu.r[inst.rn] +%= signed_offset;
            } else {
                if (comptime w == 1 or p == 0)
                    cpu.r[inst.rn] +%= signed_offset;

                // Load from memory
                if (comptime b == 1) {
                    cpu.r[inst.rd] = cpu.read(u8, addr);
                } else {
                    cpu.r[inst.rd] = cpu.read(u32, addr);
                }
                if (inst.rd == 15) cpu.reset_pipeline();
            }
        }
    };
}

fn handle_single_data_transfer(block: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.SingleDataTransferInstruction = @bitCast(instruction);
    if (DebugAlwaysFallbackToInterpreter) {
        try interpreter_fallback(block, ctx, instruction);
        return inst.l == 1 and inst.rd == 15;
    }

    inline for (0..2) |i| {
        inline for (0..2) |u| {
            inline for (0..2) |w| {
                inline for (0..2) |p| {
                    inline for (0..2) |l| {
                        inline for (0..2) |b| {
                            if (inst.i == i and inst.u == u and inst.w == w and inst.p == p and inst.l == l and inst.b == b) {
                                try block.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
                                try block.mov(.{ .reg = ArgRegisters[1] }, .{ .imm32 = instruction });
                                try block.call(comptime_handle_single_data_transfer(i, u, w, p, l, b).handler);
                            }
                        }
                    }
                }
            }
        }
    }
    return inst.l == 1 and inst.rd == 15;
}

fn handle_single_data_swap(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.SingleDataSwapInstruction = @bitCast(instruction);
    if (DebugAlwaysFallbackToInterpreter) {
        try interpreter_fallback(b, ctx, instruction);
        return inst.rd == 15;
    }

    std.debug.assert(inst.rd != 15);
    std.debug.assert(inst.rn != 15);
    std.debug.assert(inst.rm != 15);

    const addr: JIT.Operand = .{ .reg = SavedRegisters[1] };
    const rd = ReturnRegister;
    try load_register(b, addr.reg, inst.rn);
    if (inst.b == 1) {
        // cpu.r[inst.rd] = cpu.read(u8, addr);
        try load_mem(b, ctx, u8, rd, addr);
        try store_register(b, inst.rd, .{ .reg = rd }); // rd is 0 extended
        // cpu.write(u8, addr, @truncate(reg));
        const reg = ArgRegisters[2];
        try load_register(b, reg, inst.rm);
        try store_mem(b, ctx, u8, addr, reg);
    } else {
        // cpu.r[inst.rd] = cpu.read(u32, addr);
        try load_mem(b, ctx, u32, rd, addr);
        try store_register(b, inst.rd, .{ .reg = rd });
        // cpu.write(u32, addr, reg);
        const reg = ArgRegisters[2];
        try load_register(b, reg, inst.rm);
        try store_mem(b, ctx, u32, addr, reg);
    }

    return inst.rd == 15; // Illegal?
}

fn handle_multiply(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.MultiplyInstruction = @bitCast(instruction);
    std.debug.assert(inst.rd != inst.rm);
    std.debug.assert(inst.rd != 15);
    std.debug.assert(inst.rm != 15);
    std.debug.assert(inst.rs != 15);

    if (DebugAlwaysFallbackToInterpreter or inst.s == 1) {
        try interpreter_fallback(b, ctx, instruction);
        return false;
    }

    std.debug.assert(inst.s == 0); // TODO

    const rm = ReturnRegister;
    const rs = ArgRegisters[0];
    try load_register(b, rm, inst.rm);
    try load_register(b, rs, inst.rs);
    try b.append(.{ .Mul = .{ .dst = .{ .reg = rm }, .src = .{ .reg = rs } } });
    if (inst.a == 1) {
        const rn = ArgRegisters[1];
        try load_register(b, rn, inst.rn);
        try b.append(.{ .Add = .{ .dst = .{ .reg = rm }, .src = .{ .reg = rn } } });
    }
    try store_register(b, inst.rd, .{ .reg = rm });

    return false;
}

fn handle_multiply_long(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    std.debug.panic("Unimplemented multiply long", .{});
}

fn handle_halfword_data_transfer_register_offset(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    std.debug.panic("Unimplemented halfword data transfer register offset", .{});
}

fn handle_halfword_data_transfer_immediate_offset(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    std.debug.panic("Unimplemented halfword data transfer immediate offset", .{});
}

fn handle_coprocessor_data_transfer(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    std.debug.panic("Unimplemented coprocessor data transfer", .{});
}

fn handle_coprocessor_data_operation(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    std.debug.panic("Unimplemented coprocessor data operation", .{});
}

fn handle_coprocessor_register_transfer(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    _ = b;
    _ = ctx;
    _ = instruction;
    std.debug.panic("Unimplemented coprocessor register transfer", .{});
}

fn handle_mrs(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.MRSInstruction = @bitCast(instruction);
    try interpreter_fallback(b, ctx, instruction);
    std.debug.assert(inst.rd != 15);
    return false;
}

fn handle_msr(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.MSRInstruction = @bitCast(instruction);
    _ = inst;
    try interpreter_fallback(b, ctx, instruction);
    return false;
}

fn handle_data_processing(b: *IRBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.DataProcessingInstruction = @bitCast(instruction);
    if (DebugAlwaysFallbackToInterpreter) {
        try interpreter_fallback(b, ctx, instruction);
        switch (inst.opcode) {
            .TST, .TEQ, .CMP, .CMN => return false,
            else => return inst.rd == 15,
        }
    }

    const sro: arm7.interpreter.ScaledRegisterOffset = @bitCast(inst.operand2);

    // Some fast paths/static decoding.
    // FIXME: We can handle more cases, and generate better code.
    if (inst.s == 0 and
        (inst.i == 1 or sro.register_specified == 0) and
        (inst.opcode == .AND or
            inst.opcode == .EOR or
            inst.opcode == .SUB or
            inst.opcode == .RSB or
            inst.opcode == .ADD or
            inst.opcode == .ORR or
            inst.opcode == .MOV or
            inst.opcode == .BIC or
            inst.opcode == .MVN))
    {
        // op2
        var op2: JIT.Operand = undefined;
        if (inst.i == 1) {
            op2 = .{ .imm32 = arm7.interpreter.immediate_shifter_operand(inst.operand2) };
        } else {
            const rm = ArgRegisters[0];
            try load_register(b, rm, sro.rm);
            if (sro.rm == 15 and sro.register_specified == 1) {
                try b.add(.{ .reg = rm }, .{ .imm32 = 4 });
            }
            op2 = .{ .reg = rm };

            if (sro.register_specified == 0) {
                const shift_amount = sro.shift_amount.imm;
                switch (sro.shift_type) {
                    .LSL => if (shift_amount > 0) try b.append(.{ .Shl = .{ .dst = .{ .reg = rm }, .amount = .{ .imm8 = shift_amount } } }),
                    .LSR => {
                        if (shift_amount == 0) {
                            // The form of the shift field which might be expected to correspond to LSR #0 is used to
                            // encode LSR #32, which has a zero result with bit 31 of Rm as the carry output. Logical
                            // shift right zero is redundant as it is the same as logical shift left zero, so the assembler
                            // will convert LSR #0 (and ASR #0 and ROR #0) into LSL #0, and allow LSR #32 to be
                            // specified.
                            try b.mov(op2, .{ .imm32 = 0 });
                        } else {
                            try b.append(.{ .Shr = .{ .dst = .{ .reg = rm }, .amount = .{ .imm8 = shift_amount } } });
                        }
                    },
                    .ASR => {
                        if (shift_amount == 0) {
                            // The form of the shift field which might be expected to give ASR #0 is used to encode
                            // ASR #32. Bit 31 of Rm is again used as the carry output, and each bit of operand 2 is
                            // also equal to bit 31 of Rm. The result is therefore all ones or all zeros, according to the
                            // value of bit 31 of Rm.
                            return error.ZeroASRUnimplemented;
                        } else {
                            try b.append(.{ .Sar = .{ .dst = .{ .reg = rm }, .amount = .{ .imm8 = shift_amount } } });
                        }
                    },
                    .ROR => {
                        if (shift_amount == 0) {
                            // RRX
                            return error.RRXUnimplemented;
                        } else {
                            try b.append(.{ .Ror = .{ .dst = .{ .reg = rm }, .amount = .{ .imm8 = shift_amount } } });
                        }
                    },
                }
            } else {
                try load_register(b, ArgRegisters[1], sro.shift_amount.reg.rs);
                try b.append(.{ .And = .{ .dst = .{ .reg = ArgRegisters[1] }, .src = .{ .imm32 = 0xFF } } });
                return error.RegisterSpecifiedSROUnimplemented;
            }
        }

        switch (inst.opcode) {
            .AND => {
                // cpu.r(inst.rd).* = op1 & op2;
                if (inst.rd == inst.rn) {
                    try b.append(.{ .And = .{ .dst = guest_register(inst.rd), .src = op2 } });
                } else {
                    try load_register(b, ReturnRegister, inst.rn); // op1
                    try b.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = op2 } });
                    try store_register(b, inst.rd, .{ .reg = ReturnRegister });
                }
            },
            .EOR => {
                // cpu.r(inst.rd).* = op1 ^ op2;
                try load_register(b, ReturnRegister, inst.rn);
                try b.append(.{ .Xor = .{ .dst = .{ .reg = ReturnRegister }, .src = op2 } });
                try store_register(b, inst.rd, .{ .reg = ReturnRegister });
            },
            .SUB => {
                // cpu.r(inst.rd).* = op1 -% op2;
                try load_register(b, ReturnRegister, inst.rn);
                try b.append(.{ .Sub = .{ .dst = .{ .reg = ReturnRegister }, .src = op2 } });
                try store_register(b, inst.rd, .{ .reg = ReturnRegister });
            },
            .RSB => {
                // cpu.r(inst.rd).* = op2 -% op1;
                try load_register(b, ReturnRegister, inst.rn);
                if (op2 == .imm32) {
                    try b.mov(.{ .reg = ArgRegisters[0] }, op2);
                } else std.debug.assert(op2.reg == ArgRegisters[0]);
                try b.append(.{ .Sub = .{ .dst = .{ .reg = ArgRegisters[0] }, .src = .{ .reg = ReturnRegister } } });
                try store_register(b, inst.rd, .{ .reg = ArgRegisters[0] });
            },
            .ADD => {
                // cpu.r(inst.rd).* = op1 +% op2;
                if (inst.rd == inst.rn) {
                    try b.append(.{ .Add = .{ .dst = guest_register(inst.rd), .src = op2 } });
                } else {
                    try load_register(b, ReturnRegister, inst.rn);
                    try b.append(.{ .Add = .{ .dst = .{ .reg = ReturnRegister }, .src = op2 } });
                    try store_register(b, inst.rd, .{ .reg = ReturnRegister });
                }
            },
            .ORR => {
                try load_register(b, ReturnRegister, inst.rn);
                try b.append(.{ .Or = .{ .dst = .{ .reg = ReturnRegister }, .src = op2 } });
                try store_register(b, inst.rd, .{ .reg = ReturnRegister });
            },
            .MOV => {
                try store_register(b, inst.rd, op2);
            },
            .BIC => {
                // cpu.r(inst.rd).* = op1 & ~op2;
                try load_register(b, ReturnRegister, inst.rn);

                if (inst.i == 1) {
                    op2 = .{ .imm32 = ~arm7.interpreter.immediate_shifter_operand(inst.operand2) };
                } else {
                    try b.append(.{ .Not = .{ .dst = .{ .reg = op2.reg } } });
                }

                try b.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = op2 } });
                try store_register(b, inst.rd, .{ .reg = ReturnRegister });
            },
            .MVN => {
                // cpu.r(inst.rd).* = ~op2;
                if (inst.i == 1) {
                    op2 = .{ .imm32 = ~arm7.interpreter.immediate_shifter_operand(inst.operand2) };
                } else {
                    try b.append(.{ .Not = .{ .dst = .{ .reg = op2.reg } } });
                }
                try store_register(b, inst.rd, op2);
            },
            else => @panic("Not implemented"),
        }

        // Simulate an additional fetch
        if (inst.rd == 15) {
            try b.add(guest_register(15), .{ .imm32 = 4 });
        }
    } else {
        try interpreter_fallback(b, ctx, instruction);
    }
    switch (inst.opcode) {
        .TST, .TEQ, .CMP, .CMN => return false,
        else => return inst.rd == 15,
    }
}

fn handle_invalid(_: *IRBlock, _: *JITContext, _: u32) !bool {
    arm_jit_log.err("Invalid instruction", .{});
    @panic("Invalid instruction");
}

fn interpreter_fallback(b: *IRBlock, ctx: *JITContext, instruction: u32) !void {
    try b.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    try b.mov(.{ .reg = ArgRegisters[1] }, .{ .imm32 = instruction });
    try b.call(arm7.interpreter.InstructionHandlers[arm7.JumpTable[arm7.ARM7.get_instr_tag(instruction)]]);

    ctx.did_fallback = true;
}
