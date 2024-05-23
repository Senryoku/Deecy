// https://github.com/SingleStepTests/sh4

const std = @import("std");
const SH4Module = @import("sh4");

const CPUState = struct {
    R: []u32,
    R_: []u32,
    FP0: []f32,
    FP1: []f32,
    PC: u32,
    GBR: u32,
    SR: u32,
    SSR: u32,
    SPC: u32,
    VBR: u32,
    SGR: u32,
    DBR: u32,
    MACL: u32,
    MACH: u32,
    PR: u32,
    FPSCR: u32,
    FPUL: u32,

    fn log(self: *const @This()) void {
        for (0..8) |i| {
            std.debug.print("  R{d}: {X:0>8}   R{d: <2}: {X:0>8} | R'{d}: {X:0>8}\n", .{ i, self.R[i], i + 8, self.R[i + 8], i, self.R_[i] });
        }
        std.debug.print("  PC:    {X:0>8}\n", .{self.PC});
        std.debug.print("  GBR:   {X:0>8}\n", .{self.GBR});
        std.debug.print("  SR:    {X:0>8}\n", .{@as(u32, @bitCast(self.SR))});
        std.debug.print("  SSR:   {X:0>8}\n", .{@as(u32, @bitCast(self.SSR))});
        std.debug.print("  SPC:   {X:0>8}\n", .{self.SPC});
        std.debug.print("  VBR:   {X:0>8}\n", .{self.VBR});
        std.debug.print("  SGR:   {X:0>8}\n", .{self.SGR});
        std.debug.print("  DBR:   {X:0>8}\n", .{self.DBR});
        std.debug.print("  MACH:  {X:0>8}\n", .{self.MACH});
        std.debug.print("  MACL:  {X:0>8}\n", .{self.MACL});
        std.debug.print("  PR:    {X:0>8}\n", .{self.PR});
        std.debug.print("  FPSCR: {X:0>8}\n", .{@as(u32, @bitCast(self.FPSCR))});
        std.debug.print("  FPUL:  {X:0>8}\n", .{self.FPUL});
    }
};

const Test = struct {
    initial: CPUState,
    final: CPUState,
    cycles: []struct {
        actions: u32,
        fetch_addr: u32,
        fetch_val: u16,
        read_addr: ?u32 = null,
        read_val: ?u64 = null,
        write_addr: ?u32 = null,
        write_val: ?u64 = null,
    },
    opcodes: []u16,

    fn log(self: *const @This()) void {
        self.initial.log();

        for (self.opcodes) |opcode| {
            std.debug.print("   > {s}\n", .{SH4Module.sh4_disassembly.disassemble(SH4Module.Instr{ .value = opcode }, std.testing.allocator) catch unreachable});
        }
    }
};

pub const std_options: std.Options = .{
    .log_level = .info,
    .log_scope_levels = &[_]std.log.ScopeLevel{
        .{ .scope = .dc, .level = .err },
        .{ .scope = .sh4, .level = .err },
        .{ .scope = .sh4_jit, .level = .err },
        .{ .scope = .arm_jit, .level = .err },
        .{ .scope = .x86_64_emitter, .level = .err },
        .{ .scope = .syscall_log, .level = .err },
        .{ .scope = .aica, .level = .err },
        .{ .scope = .holly, .level = .err },
        .{ .scope = .gdrom, .level = .err },
        .{ .scope = .maple, .level = .err },
        .{ .scope = .renderer, .level = .err },
    },
};

var TestState: struct {
    cpu: *SH4Module.SH4 = undefined,
    cycle: u32 = 0,
    test_data: Test = undefined,
} = .{};

fn read8(addr: u32) u8 {
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].read_addr, addr) catch {
        std.debug.print("  Read8 failed at cycle {d}. Expected address {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].read_addr, addr });
        TestState.test_data.log();
        @panic("read8 address error");
    };
    return @truncate(TestState.test_data.cycles[TestState.cycle].read_val.?);
}

fn read16(addr: u32) u16 {
    if (TestState.test_data.cycles[TestState.cycle].fetch_addr == addr)
        return @truncate(TestState.test_data.cycles[TestState.cycle].fetch_val);
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].read_addr == addr);
    return @truncate(TestState.test_data.cycles[TestState.cycle].read_val.?);
}

fn read32(addr: u32) u32 {
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].read_addr == addr);
    return @truncate(TestState.test_data.cycles[TestState.cycle].read_val.?);
}

fn read64(addr: u32) u64 {
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].read_addr == addr);
    return @truncate(TestState.test_data.cycles[TestState.cycle].read_val.?);
}

fn write8(addr: u32, val: u8) void {
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].write_addr, addr) catch {
        std.debug.print("  Write8 failed at cycle {d}. Expected address {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].write_addr, addr });
        TestState.test_data.log();
        @panic("write8 address error");
    };
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].write_val, val) catch {
        std.debug.print("  Write8 failed at cycle {d}. Expected value {?X:0>2}, got {?X:0>2}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].write_val, val });
        TestState.test_data.log();
        @panic("write8 value error");
    };
}

fn write16(addr: u32, val: u16) void {
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].write_addr == addr);
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].write_val == val);
}

fn write32(addr: u32, val: u32) void {
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].write_addr == addr);
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].write_val == val);
}

fn write64(addr: u32, val: u64) void {
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].write_addr == addr);
    std.debug.assert(TestState.test_data.cycles[TestState.cycle].write_val == val);
}

fn run_test(t: Test, cpu: *SH4Module.SH4, comptime log: bool) !void {
    TestState = .{ .cpu = cpu, .cycle = 0, .test_data = t };

    for (0..16) |i| {
        cpu.r[i] = t.initial.R[i];
        cpu.fp_banks[0].fr[i] = t.initial.FP0[i];
        cpu.fp_banks[1].fr[i] = t.initial.FP1[i];
    }
    for (0..8) |i| {
        cpu.r_bank[i] = t.initial.R_[i];
    }
    cpu.pc = t.initial.PC;
    cpu.gbr = t.initial.GBR;
    cpu.sr = @bitCast(t.initial.SR);
    cpu.ssr = @bitCast(t.initial.SSR);
    cpu.spc = t.initial.SPC;
    cpu.vbr = t.initial.VBR;
    cpu.sgr = t.initial.SGR;
    cpu.dbr = t.initial.DBR;
    cpu.mach = t.initial.MACH;
    cpu.macl = t.initial.MACL;
    cpu.pr = t.initial.PR;
    cpu.fpscr = @bitCast(t.initial.FPSCR);
    cpu.fpul = t.initial.FPUL;

    if (log) {
        t.initial.log();
    }

    for (0..4) |_| {
        try std.testing.expect(t.cycles[TestState.cycle].fetch_addr == cpu.pc);
        TestState.cycle += 1;

        const addr = cpu.pc;
        const offset: i64 = @divFloor(@as(i64, cpu.pc) - t.initial.PC, 2);
        const opcode = t.opcodes[@intCast(if (offset >= 0 and offset < 4) offset else 4)];

        const instr = SH4Module.Instr{ .value = opcode };
        const desc = SH4Module.sh4_instructions.Opcodes[SH4Module.sh4_instructions.JumpTable[opcode]];

        if (log) std.debug.print("    [{X:0>8}] {b:0>16} {s: <20} R{d: <2}={X:0>8}, R{d: <2}={X:0>8}, T={b:0>1}, Q={b:0>1}, M={b:0>1}\n", .{ addr, opcode, SH4Module.sh4_disassembly.disassemble(instr, cpu._allocator) catch {
            std.debug.print("Failed to disassemble instruction {b:0>16}\n", .{opcode});
            unreachable;
        }, instr.nmd.n, cpu.R(instr.nmd.n).*, instr.nmd.m, cpu.R(instr.nmd.m).*, if (cpu.sr.t) @as(u1, 1) else 0, if (cpu.sr.q) @as(u1, 1) else 0, if (cpu.sr.m) @as(u1, 1) else 0 });

        desc.fn_(cpu, instr);

        if (log) std.debug.print("    [{X:0>8}] {X: >16} {s: <20} R{d: <2}={X:0>8}, R{d: <2}={X:0>8}, T={b:0>1}, Q={b:0>1}, M={b:0>1}\n", .{ addr, opcode, "", instr.nmd.n, cpu.R(instr.nmd.n).*, instr.nmd.m, cpu.R(instr.nmd.m).*, if (cpu.sr.t) @as(u1, 1) else 0, if (cpu.sr.q) @as(u1, 1) else 0, if (cpu.sr.m) @as(u1, 1) else 0 });

        cpu.pc += 2;
    }

    try std.testing.expectEqualSlices(u32, t.final.R, &cpu.r);
    try std.testing.expectEqualSlices(u32, t.final.R_, &cpu.r_bank);
    for (0..16) |i| {
        try std.testing.expectEqual(t.final.FP0[i], cpu.fp_banks[0].fr[i]);
        try std.testing.expectEqual(t.final.FP1[i], cpu.fp_banks[1].fr[i]);
    }
    try std.testing.expectEqual(t.final.PC, cpu.pc);
    try std.testing.expectEqual(t.final.GBR, cpu.gbr);
    try std.testing.expectEqual(t.final.SR, @as(u32, @bitCast(cpu.sr)));
    try std.testing.expectEqual(t.final.SPC, cpu.spc);
    try std.testing.expectEqual(t.final.VBR, cpu.vbr);
    try std.testing.expectEqual(t.final.SGR, cpu.sgr);
    try std.testing.expectEqual(t.final.DBR, cpu.dbr);
    try std.testing.expectEqual(t.final.MACH, cpu.mach);
    try std.testing.expectEqual(t.final.MACL, cpu.macl);
    try std.testing.expectEqual(t.final.PR, cpu.pr);
    try std.testing.expectEqual(t.final.FPSCR, @as(u32, @bitCast(cpu.fpscr)));
    try std.testing.expectEqual(t.final.FPUL, cpu.fpul);
}

test {
    defer SH4Module.sh4_disassembly.free_disassembly_cache(std.testing.allocator);

    SH4Module.DebugHooks.read8 = read8;
    SH4Module.DebugHooks.read16 = read16;
    SH4Module.DebugHooks.read32 = read32;
    SH4Module.DebugHooks.read64 = read64;
    SH4Module.DebugHooks.write8 = write8;
    SH4Module.DebugHooks.write16 = write16;
    SH4Module.DebugHooks.write32 = write32;
    SH4Module.DebugHooks.write64 = write64;

    const TestDir = "../SingleStepTests_sh4/";
    var test_dir = try std.fs.cwd().openDir(TestDir, .{ .iterate = true });
    defer test_dir.close();

    var walker = try test_dir.walk(std.testing.allocator);
    defer walker.deinit();

    var cpu = try SH4Module.SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.debug_trace = true;

    tests_loop: while (try walker.next()) |entry| {
        if (entry.kind == .file and std.mem.endsWith(u8, entry.basename, ".json")) {
            // Filter tests with delay slots for now as the sh4 emulator architecture isn't suited for easy testing.
            for ([_][]const u8{
                "0000000000001011_sz0_pr0.json", // rts
                "0000000000011011_sz0_pr0.json", // sleep
                "0000000000101011_sz0_pr0.json", // rte
                "0000mmmm00000011_sz0_pr0.json", // bsrf Rm
                "0100mmmm00101011_sz0_pr0.json", // jmp @Rm
                "0100mmmm00001011_sz0_pr0.json", // jsr @Rm
                "1011dddddddddddd_sz0_pr0.json", // bsr
                "0000mmmm00100011_sz0_pr0.json", // braf Rm
                "1010dddddddddddd_sz0_pr0.json", // bra
                "10001101dddddddd_sz0_pr0.json", // bt/s
                "10001111dddddddd_sz0_pr0.json", // bf/s
            }) |filename| {
                if (std.mem.eql(u8, entry.basename, filename)) {
                    std.debug.print("! Skipping {s}\n", .{entry.basename});
                    continue :tests_loop;
                }
            }

            const fullpath = try std.fs.path.join(std.testing.allocator, &[_][]const u8{ TestDir, entry.basename });
            std.debug.print("Opening {s}\n", .{fullpath});
            defer std.testing.allocator.free(fullpath);
            const data = try std.fs.cwd().readFileAlloc(std.testing.allocator, fullpath, 512 * 1024 * 1024);
            defer std.testing.allocator.free(data);

            const test_data = try std.json.parseFromSlice([]Test, std.testing.allocator, data, .{});
            defer test_data.deinit();

            var test_num: u32 = 0;
            for (test_data.value) |t| {
                run_test(t, &cpu, false) catch |err| {
                    std.debug.print("Failed to run test {s}: {s}\n", .{ entry.basename, @errorName(err) });
                    return run_test(t, &cpu, true);
                };
                test_num += 1;
            }
        }
    }
}
