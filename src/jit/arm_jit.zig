const std = @import("std");

const termcolor = @import("../termcolor.zig");

const arm7 = @import("arm7");
const bit_manip = @import("../bit_manip.zig");
const JIT = @import("jit_block.zig");
const JITBlock = JIT.JITBlock;

const Architecture = @import("x86_64.zig");
const ReturnRegister = Architecture.ReturnRegister;
const ArgRegisters = Architecture.ArgRegisters;
const SavedRegisters = Architecture.SavedRegisters;

const BasicBlock = @import("basic_block.zig");
const Dreamcast = @import("../dreamcast.zig").Dreamcast;

const arm_jit_log = std.log.scoped(.arm_jit);

const BlockBufferSize = 2 * 1024 * 1024;

const HashMapContext = struct {
    pub fn hash(_: @This(), addr: u32) u64 {
        return @as(u64, addr >> 2) * 2654435761;
    }
    pub fn eql(_: @This(), a: u32, b: u32) bool {
        return a == b;
    }
};

const BlockCache = struct {
    const BlockEntryCount = 0x200000 >> 2;

    buffer: []align(std.mem.page_size) u8,
    cursor: usize = 0,
    blocks: []?BasicBlock = undefined,

    min_address: u32 = std.math.maxInt(u32),
    max_address: u32 = 0,

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

    pub fn signal_write(self: *@This(), addr: u32) void {
        if (addr >= self.min_address and addr <= self.max_address) {
            self.reset() catch {
                arm_jit_log.err("Failed to reset block cache.", .{});
                @panic("Failed to reset block cache.");
            };
        }
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
        arm_jit_log.info(termcolor.blue("Resetting block cache."), .{});

        self.cursor = 0;

        std.os.windows.VirtualFree(self.blocks.ptr, 0, std.os.windows.MEM_RELEASE);
        try self.allocate_blocks();

        self.min_address = std.math.maxInt(u32);
        self.max_address = 0;
    }

    pub fn get(self: *@This(), address: u32) ?BasicBlock {
        return self.blocks[address >> 2];
    }

    pub fn put(self: *@This(), address: u32, end_address: u32, block: BasicBlock) void {
        self.min_address = @min(self.min_address, address);
        self.max_address = @max(self.max_address, end_address);

        self.blocks[address >> 2] = block;
    }
};

pub const JITContext = struct {
    address: u32,
    cpu: *arm7.ARM7,

    did_fallback: bool = false, // Only there for analysis.
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

    pub noinline fn run_for(self: *@This(), cpu: *arm7.ARM7, cycles: i32) !i32 {
        if (!cpu.running) return 0;

        var spent_cycles: i32 = 0;
        while (spent_cycles < cycles) {
            const pc = cpu.pc() - 4; // Pipelining...
            var block = self.block_cache.get(pc);
            if (block == null) {
                arm_jit_log.debug("(Cache Miss) Compiling {X:0>8}...", .{pc});
                const instructions: [*]u32 = @alignCast(@ptrCast(&cpu.memory[pc]));
                block = try (self.compile(.{ .address = pc, .cpu = cpu }, instructions) catch |err| retry: {
                    if (err == error.JITCacheFull) {
                        try self.block_cache.reset();
                        arm_jit_log.info("JIT cache purged.", .{});
                        break :retry self.compile(.{ .address = pc, .cpu = cpu }, instructions);
                    } else break :retry err;
                });
            }
            // arm_jit_log.debug("Running {X:0>8} ({} cycles)", .{ pc, block.?.cycles });
            block.?.execute(cpu);
            spent_cycles += @intCast(block.?.cycles);

            // Not necessary, just here to allow compatibility with the interpreter if we need it.
            // (Right now we're always calling to the interpreter so this should stay in sync, but once
            //  we start actually JITing some instructions, we won't keep instruction_pipeline updated)
            cpu.instruction_pipeline[0] = @as(*const u32, @alignCast(@ptrCast(&cpu.memory[(cpu.pc() - 4) & cpu.memory_address_mask]))).*;
        }

        return spent_cycles;
    }

    pub fn compile(self: *@This(), start_ctx: JITContext, instructions: [*]u32) !BasicBlock {
        var ctx = start_ctx;

        var b = JITBlock.init(self._allocator);
        defer b.deinit();

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

            var jump = try handle_condition(&b, &ctx, instr);

            const tag = arm7.ARM7.get_instr_tag(instr);
            const branch = try InstructionHandlers[arm7.JumpTable[tag]](&b, &ctx, instr);

            if (jump) |*j| {
                j.patch();
            }

            cycles += 1; // FIXME
            index += 1;
            ctx.address += 4;

            arm_jit_log.debug("  [{X:0>8}] {s} {s}", .{ ctx.address, if (ctx.did_fallback) "!" else " ", arm7.ARM7.disassemble(instr) });

            if (branch)
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

        var block = try b.emit(self.block_cache.buffer[self.block_cache.cursor..]);
        self.block_cache.cursor += block.buffer.len;
        block.cycles = cycles;

        arm_jit_log.debug("Compiled: {X:0>2}", .{block.buffer});

        self.block_cache.put(start_ctx.address, ctx.address, block);
        return block;
    }
};

fn guest_register(arm_reg: u5) JIT.Operand {
    return .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(arm7.ARM7, "r") + @sizeOf(u32) * @as(u32, arm_reg), .size = 32 } };
}

fn load_register(b: *JITBlock, host_register: JIT.Register, arm_reg: u5) !void {
    try b.mov(.{ .reg = host_register }, guest_register(arm_reg));
}

fn store_register(b: *JITBlock, arm_reg: u5, value: JIT.Operand) !void {
    try b.mov(guest_register(arm_reg), value);
}

fn read8(self: *arm7.ARM7, address: u32) u8 {
    return self.read(u8, address);
}
fn read32(self: *arm7.ARM7, address: u32) u32 {
    return self.read(u32, address);
}
fn write8(self: *arm7.ARM7, address: u32, value: u8) void {
    self.write(u8, address, value);
}
fn write32(self: *arm7.ARM7, address: u32, value: u32) void {
    self.write(u32, address, value);
}

fn load_wave_memory(b: *JITBlock, ctx: *const JITContext, comptime T: type, dst: JIT.Register, addr: u32) !void {
    // TODO: This could be turned into a single movabs, but emitter doesn't support it yet.
    try b.mov(.{ .reg = dst }, .{ .imm64 = @intFromPtr(ctx.cpu.memory.ptr) + addr });
    try b.mov(.{ .reg = dst }, .{ .mem = .{ .base = dst, .displacement = 0, .size = @bitSizeOf(T) } });
}

// NOTE: Uses ReturnRegister as a temporary!
fn store_wave_memory(b: *JITBlock, ctx: *const JITContext, comptime T: type, addr: u32, value: JIT.Register) !void {
    // TODO: This could be turned into a single movabs, but emitter doesn't support it yet.
    try b.mov(.{ .reg = ReturnRegister }, .{ .imm64 = @intFromPtr(ctx.cpu.memory.ptr) + addr });
    try b.mov(.{ .mem = .{ .base = ReturnRegister, .displacement = 0, .size = @bitSizeOf(T) } }, .{ .reg = value });
}

// Loads into ReturnRegister
fn load_mem(b: *JITBlock, ctx: *const JITContext, comptime T: type, dst: JIT.Register, addr: JIT.Operand) !void {
    switch (addr) {
        .imm32 => |addr_imm32| {
            if (addr_imm32 < ctx.cpu.external_memory_address_threshold) {
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

fn store_mem(b: *JITBlock, ctx: *const JITContext, comptime T: type, addr: JIT.Operand, value: JIT.Register) !void {
    switch (addr) {
        .imm32 => |addr_imm32| {
            if (addr_imm32 < ctx.cpu.external_memory_address_threshold) {
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

fn extract_cpsr_flags(b: *JITBlock, comptime flags: []const []const u8) !void {
    try b.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(arm7.ARM7, "cpsr"), .size = 32 } });
    try b.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = .{ .imm32 = cpsr_mask(flags) } } });
}

fn test_cpsr_flags(b: *JITBlock, comptime flags: []const []const u8, comptime expected_flags: []const []const u8) !JIT.PatchableJump {
    try extract_cpsr_flags(b, flags);
    try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(expected_flags) } } });
    return b.jmp(.NotEqual);
}

// Emits code testing the flags in the CPSR and returns a patchable jump meant to point at the next instruction, taken if the condition is not met.
fn handle_condition(b: *JITBlock, ctx: *JITContext, instruction: u32) !?JIT.PatchableJump {
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
            var do_label_2 = try b.jmp(.Carry);
            const skip_label = try b.jmp(.Always);
            do_label.patch();
            do_label_2.patch();
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
            var do_label_1 = try b.jmp(.Equal);

            const skip_label = try b.jmp(.Always);

            do_label_0.patch();
            do_label_1.patch();

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
            var do_label_1 = try b.jmp(.Equal);

            const skip_label = try b.jmp(.Always);

            do_label_0.patch();
            do_label_1.patch();

            return skip_label;
        },
        .GT => {
            // return !cpu.cpsr.z and (cpu.cpsr.n == cpu.cpsr.v)
            try b.mov(.{ .reg = ReturnRegister }, .{ .mem = .{ .base = SavedRegisters[0], .displacement = @offsetOf(arm7.ARM7, "cpsr"), .size = 32 } });
            try b.bit_test(ReturnRegister, @bitOffsetOf(arm7.CPSR, "z")); // Set carry flag to 'z'.
            var do_label_0 = try b.jmp(.NotCarry);

            try extract_cpsr_flags(b, &[_][]const u8{ "n", "v" });
            // v == 1 and n == 1
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{ "v", "n" }) } } });
            var do_label_1 = try b.jmp(.Equal);
            // v == 0 and n == 0
            try b.append(.{ .Cmp = .{ .lhs = .{ .reg = ReturnRegister }, .rhs = .{ .imm32 = cpsr_mask(&[_][]const u8{}) } } });
            var do_label_2 = try b.jmp(.Equal);

            const skip_label = try b.jmp(.Always);

            do_label_0.patch();
            do_label_1.patch();
            do_label_2.patch();

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
            var do_label_2 = try b.jmp(.Equal);

            const skip_label = try b.jmp(.Always);

            do_label_0.patch();
            do_label_1.patch();
            do_label_2.patch();

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

fn handle_branch(b: *JITBlock, _: *JITContext, instruction: u32) !bool {
    const inst: arm7.BranchInstruction = @bitCast(instruction);

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

    if (inst.i == 0) {
        const offset: u32 = inst.offset;

        var addr: JIT.Operand = .{ .reg = ArgRegisters[1] };
        const offset_addr = ReturnRegister;

        // PC-relative addressing: A constant in this case.
        if (inst.rn == 15) {
            std.debug.assert(inst.w == 0);
            std.debug.assert(inst.p == 1);
            addr = .{ .imm32 = if (inst.u == 1) ctx.address + 8 + offset else ctx.address + 8 - offset };
        } else {
            try load_register(b, addr.reg, inst.rn);
            if (inst.offset != 0) {
                try b.mov(.{ .reg = offset_addr }, addr);

                if (inst.u == 1) {
                    try b.add(.{ .reg = offset_addr }, .{ .imm32 = offset });
                } else {
                    try b.sub(.{ .reg = offset_addr }, .{ .imm32 = offset });
                }

                if (inst.p == 1) try b.mov(addr, .{ .reg = offset_addr });

                if (inst.w == 1 or inst.p == 0) try store_register(b, inst.rn, .{ .reg = offset_addr });
            }
        }

        if (inst.l == 0) {
            // Store
            const val = ArgRegisters[2];
            try load_register(b, val, inst.rd);

            if (inst.rd == 15)
                try b.add(.{ .reg = val }, .{ .imm32 = 4 });

            if (inst.b == 1) {
                try store_mem(b, ctx, u8, addr, val);
            } else {
                if (addr.tag() == .imm32) {
                    addr.imm32 &= 0xFFFFFFFC;
                } else {
                    try b.append(.{ .And = .{ .dst = addr, .src = .{ .imm32 = 0xFFFFFFFC } } });
                }
                try store_mem(b, ctx, u32, addr, val);
            }
        } else {
            // Load
            const val = ReturnRegister;
            if (inst.b == 1) {
                try load_mem(b, ctx, u8, val, addr);
                try store_register(b, inst.rd, .{ .reg = val });
            } else {
                if (addr.tag() == .imm32) {
                    const rotate_amount = (addr.imm32 & 3) * 8;
                    addr.imm32 &= 0xFFFFFFFC;
                    try load_mem(b, ctx, u32, val, addr);
                    if (rotate_amount != 0)
                        try b.append(.{ .Ror = .{ .dst = .{ .reg = val }, .amount = .{ .imm32 = rotate_amount } } });
                    try store_register(b, inst.rd, .{ .reg = val });
                } else {
                    // A word load (LDR) will normally use a word aligned address. However, an address
                    // offset from a word boundary will cause the data to be rotated into the register so that
                    // the addressed byte occupies bits 0 to 7. This means that half-words accessed at offsets
                    // 0 and 2 from the word boundary will be correctly loaded into bits 0 through 15 of the
                    // register. Two shift operations are then required to clear or to sign extend the upper 16
                    // bits.
                    try b.mov(.{ .reg = SavedRegisters[1] }, addr); // We need to hold to it and the call might invalidate it.
                    try b.append(.{ .And = .{ .dst = addr, .src = .{ .imm32 = 0xFFFFFFFC } } });
                    try load_mem(b, ctx, u32, val, addr);
                    // addr = (addr & 3) * 8
                    const rotate_amount = JIT.Register.rcx; // Only valid register for shifts.
                    try b.mov(.{ .reg = rotate_amount }, .{ .reg = SavedRegisters[1] }); // Restore addr.
                    try b.append(.{ .And = .{ .dst = .{ .reg = rotate_amount }, .src = .{ .imm32 = 0x00000003 } } });
                    try b.append(.{ .Shl = .{ .dst = .{ .reg = rotate_amount }, .amount = .{ .imm8 = 3 } } });
                    // val ROR addr
                    try b.append(.{ .Ror = .{ .dst = .{ .reg = val }, .amount = .{ .reg = rotate_amount } } });
                    try store_register(b, inst.rd, .{ .reg = val });
                }
            }

            // Simulate an additional fetch
            if (inst.rd == 15) {
                try b.add(guest_register(15), .{ .imm32 = 4 });
            }
        }
    } else {
        try interpreter_fallback(b, ctx, instruction);
    }
    return inst.l == 1 and inst.rd == 15;
}

fn handle_single_data_swap(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.MRSInstruction = @bitCast(instruction);
    try interpreter_fallback(b, ctx, instruction);
    return inst.rd == 15;
}

fn handle_multiply(b: *JITBlock, ctx: *JITContext, instruction: u32) !bool {
    const inst: arm7.MultiplyInstruction = @bitCast(instruction);
    std.debug.assert(inst.rd != inst.rm);
    std.debug.assert(inst.rd != 15);
    std.debug.assert(inst.rm != 15);
    std.debug.assert(inst.rs != 15);
    try interpreter_fallback(b, ctx, instruction);
    return false;
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

    const sro: arm7.interpreter.ScaledRegisterOffset = @bitCast(inst.operand2);

    // Some fast paths/static decoding.
    // FIXME: We can handle more cases, and generate better code.
    if (inst.s == 0 and
        (inst.i == 1 or sro.register_specified == 0) and
        inst.rd != 15 and
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
                    .LSL => try b.append(.{ .Shl = .{ .dst = .{ .reg = rm }, .amount = .{ .imm8 = shift_amount } } }),
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
                try b.append(.{ .Sub = .{ .dst = .{ .reg = ArgRegisters[0] }, .src = op2 } });
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
                    try b.append(.{ .Not = .{ .dst = op2.reg } });
                }

                try b.append(.{ .And = .{ .dst = .{ .reg = ReturnRegister }, .src = op2 } });
                try store_register(b, inst.rd, .{ .reg = ReturnRegister });
            },
            .MVN => {
                // cpu.r(inst.rd).* = ~op2;
                if (inst.i == 1) {
                    op2 = .{ .imm32 = ~arm7.interpreter.immediate_shifter_operand(inst.operand2) };
                } else {
                    try b.append(.{ .Not = .{ .dst = op2.reg } });
                }
                try store_register(b, inst.rd, op2);
            },
            else => @panic("Not implemented"),
        }
    } else {
        try interpreter_fallback(b, ctx, instruction);
    }
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
    try b.mov(.{ .reg = ArgRegisters[0] }, .{ .reg = SavedRegisters[0] });
    try b.mov(.{ .reg = ArgRegisters[1] }, .{ .imm32 = instruction });
    try b.call(arm7.interpreter.InstructionHandlers[arm7.JumpTable[arm7.ARM7.get_instr_tag(instruction)]]);

    ctx.did_fallback = true;
}
