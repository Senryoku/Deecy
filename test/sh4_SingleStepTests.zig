// https://github.com/SingleStepTests/sh4

const std = @import("std");
const SH4Module = @import("sh4");

const termcolor = @import("termcolor");

pub fn is_nan(x: anytype) bool {
    return x != x;
}

const CPUState = struct {
    R: []u32,
    R_: []u32,
    FP0: []u32,
    FP1: []u32,
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
            std.debug.print("  R{d}: {X:0>8}   R{d: <2}: {X:0>8} | R'{d}: {X:0>8} | FR{d}: {e: >12.2}  {X:0>8}   FR{d: <2}: {e: >12.2}  {X:0>8} | FR'{d}: {e: >12.2}  {X:0>8}   FR'{d: <2}: {e: >12.2}  {X:0>8}\n", .{
                i,
                self.R[i],
                i + 8,
                self.R[i + 8],
                i,
                self.R_[i],
                i,
                @as(f32, @bitCast(self.FP0[i])),
                @as(u32, @bitCast(self.FP0[i])),
                i + 8,
                @as(f32, @bitCast(self.FP0[i + 8])),
                @as(u32, @bitCast(self.FP0[i + 8])),
                i,
                @as(f32, @bitCast(self.FP1[i])),
                @as(u32, @bitCast(self.FP1[i])),
                i + 8,
                @as(f32, @bitCast(self.FP1[i + 8])),
                @as(u32, @bitCast(self.FP1[i + 8])),
            });
        }
        std.debug.print("  PC:    {X:0>8}   PR:   {X:0>8}   SPC: {X:0>8}\n", .{ self.PC, self.PR, self.SPC });
        std.debug.print("  GBR:   {X:0>8}\n", .{self.GBR});
        std.debug.print("  SR:    {X:0>8}   SSR:  {X:0>8}\n", .{ @as(u32, @bitCast(self.SR)), @as(u32, @bitCast(self.SSR)) });
        std.debug.print("  VBR:   {X:0>8}\n", .{self.VBR});
        std.debug.print("  SGR:   {X:0>8}\n", .{self.SGR});
        std.debug.print("  DBR:   {X:0>8}\n", .{self.DBR});
        std.debug.print("  MACH:  {X:0>8}   MACL: {X:0>8}\n", .{ self.MACH, self.MACL });
        std.debug.print("  FPSCR: {X:0>8}\n", .{@as(u32, @bitCast(self.FPSCR))});
        std.debug.print("  FPUL:  {X:0>8}\n", .{self.FPUL});
    }
};

fn cpu_log(cpu: *const SH4Module.SH4) void {
    for (0..8) |i| {
        std.debug.print("  R{d}: {X:0>8}   R{d: <2}: {X:0>8} | R'{d}: {X:0>8} | FR{d}: {e: >12.2}  {X:0>8}   FR{d: <2}: {e: >12.2}  {X:0>8} | FR'{d}: {e: >12.2}  {X:0>8}   FR'{d: <2}: {e: >12.2}  {X:0>8}\n", .{
            i,
            cpu.r[i],
            i + 8,
            cpu.r[i + 8],
            i,
            cpu.r_bank[i],
            i,
            @as(f32, @bitCast(cpu.fp_banks[0].fr[i])),
            @as(u32, @bitCast(cpu.fp_banks[0].fr[i])),
            i + 8,
            @as(f32, @bitCast(cpu.fp_banks[0].fr[i + 8])),
            @as(u32, @bitCast(cpu.fp_banks[0].fr[i + 8])),
            i,
            @as(f32, @bitCast(cpu.fp_banks[1].fr[i])),
            @as(u32, @bitCast(cpu.fp_banks[1].fr[i])),
            i + 8,
            @as(f32, @bitCast(cpu.fp_banks[1].fr[i + 8])),
            @as(u32, @bitCast(cpu.fp_banks[1].fr[i + 8])),
        });
    }
    std.debug.print("  PC:    {X:0>8}   PR:   {X:0>8}   SPC: {X:0>8}\n", .{ cpu.pc, cpu.pr, cpu.spc });
    std.debug.print("  GBR:   {X:0>8}\n", .{cpu.gbr});
    std.debug.print("  SR:    {X:0>8}   SSR:  {X:0>8}\n", .{ @as(u32, @bitCast(cpu.sr)), @as(u32, @bitCast(cpu.ssr)) });
    std.debug.print("  VBR:   {X:0>8}\n", .{cpu.vbr});
    std.debug.print("  SGR:   {X:0>8}\n", .{cpu.sgr});
    std.debug.print("  DBR:   {X:0>8}\n", .{cpu.dbr});
    std.debug.print("  MACH:  {X:0>8}   MACL: {X:0>8}\n", .{ cpu.mach, cpu.macl });
    std.debug.print("  FPSCR: {X:0>8}\n", .{@as(u32, @bitCast(cpu.fpscr))});
    std.debug.print("  FPUL:  {X:0>8}\n", .{cpu.fpul});
}

fn compare_state(cpu: *const SH4Module.SH4, expected_state: *const CPUState) void {
    for (0..8) |i| {
        if (cpu.r[i] != expected_state.R[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i, cpu.r[i], expected_state.R[i] });
        if (cpu.r[i + 8] != expected_state.R[i + 8]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i + 8, cpu.r[i + 8], expected_state.R[i + 8] });
        if (cpu.r_bank[i] != expected_state.R_[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R'{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m\n", .{ i, cpu.r_bank[i], expected_state.R_[i] });
    }
    for (0..16) |i| {
        if (cpu.fp_banks[0].fr[i] != @as(f32, @bitCast(expected_state.FP0[i]))) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  FR{d: <2}: {e: >12.2} {e: >12.2}\u{001b}[0m  |", .{ i, cpu.fp_banks[0].fr[i], @as(f32, @bitCast(expected_state.FP0[i])) });
        if (cpu.fp_banks[1].fr[i] != @as(f32, @bitCast(expected_state.FP1[i]))) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  FR'{d: <2}: {e: >12.2} {e: >12.2}\u{001b}[0m\n", .{ i, cpu.fp_banks[1].fr[i], @as(f32, @bitCast(expected_state.FP1[i])) });
    }
    if (cpu.pc != expected_state.PC) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  PC:    {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.pc, expected_state.PC });
    if (cpu.pr != expected_state.PR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  PR:    {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.pr, expected_state.PR });
    if (cpu.spc != expected_state.SPC) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPC:   {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.spc, expected_state.SPC });
    if (cpu.gbr != expected_state.GBR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  GBR:   {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.gbr, expected_state.GBR });
    if (@as(u32, @bitCast(cpu.sr)) != expected_state.SR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SR:    {X:0>8} {X:0>8}\u{001b}[0m\n", .{ @as(u32, @bitCast(cpu.sr)), expected_state.SR });
    if (@as(u32, @bitCast(cpu.ssr)) != expected_state.SSR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SSR:   {X:0>8} {X:0>8}\u{001b}[0m\n", .{ @as(u32, @bitCast(cpu.ssr)), expected_state.SSR });
    if (cpu.vbr != expected_state.VBR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  VBR:   {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.vbr, expected_state.VBR });
    if (cpu.sgr != expected_state.SGR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SGR:   {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.sgr, expected_state.SGR });
    if (cpu.dbr != expected_state.DBR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  DBR:   {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.dbr, expected_state.DBR });
    if (cpu.mach != expected_state.MACH) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  MACH:  {X:0>8} {X:0>8}\u{001b}[0m", .{ cpu.mach, expected_state.MACH });
    if (cpu.macl != expected_state.MACL) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  MACL:  {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.macl, expected_state.MACL });
    if (@as(u32, @bitCast(cpu.fpscr)) != expected_state.FPSCR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  FPSCR: {X:0>8} {X:0>8}\u{001b}[0m\n", .{ @as(u32, @bitCast(cpu.fpscr)), expected_state.FPSCR });
    if (cpu.fpul != expected_state.FPUL) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  FPUL:  {X:0>8} {X:0>8}\u{001b}[0m\n", .{ cpu.fpul, expected_state.FPUL });
}

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

        for (self.cycles) |cycle| {
            std.debug.print("   * {any}\n", .{cycle});
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
    //if (TestState.test_data.cycles[TestState.cycle].fetch_addr == addr) {
    //    const value = TestState.test_data.cycles[TestState.cycle].fetch_val;
    //    TestState.cycle += 1;
    //    return @truncate(value);
    //}
    // Hackish support for fetching from delay slots
    if (TestState.cycle < TestState.test_data.cycles.len - 1 and TestState.test_data.cycles[TestState.cycle + 1].fetch_addr == addr) {
        TestState.cycle += 1;
        return @truncate(TestState.test_data.cycles[TestState.cycle].fetch_val);
    }
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].read_addr, addr) catch {
        std.debug.print("  Read16 failed at cycle {d}. Expected address {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].read_addr, addr });
        TestState.test_data.log();
        @panic("read16 address error");
    };
    return @truncate(TestState.test_data.cycles[TestState.cycle].read_val.?);
}

fn read32(addr: u32) u32 {
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].read_addr, addr) catch {
        std.debug.print("  Read32 failed at cycle {d}. Expected address {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].read_addr, addr });
        TestState.test_data.log();
        @panic("read32 address error");
    };
    return @truncate(TestState.test_data.cycles[TestState.cycle].read_val.?);
}

fn read64(addr: u32) u64 {
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].read_addr, addr) catch {
        std.debug.print("  Read64 failed at cycle {d}. Expected address {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].read_addr, addr });
        TestState.test_data.log();
        @panic("read64 address error");
    };
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
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].write_addr, addr) catch {
        std.debug.print("  Write16 failed at cycle {d}. Expected address {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].write_addr, addr });
        TestState.test_data.log();
        @panic("write16 address error");
    };
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].write_val, val) catch {
        std.debug.print("  Write16 failed at cycle {d}. Expected value {?X:0>4}, got {?X:0>4}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].write_val, val });
        TestState.test_data.log();
        @panic("write16 value error");
    };
}

fn write32(addr: u32, val: u32) void {
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].write_addr, addr) catch {
        std.debug.print("  Write32 failed at cycle {d}. Expected address {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].write_addr, addr });
        TestState.test_data.log();
        @panic("write32 address error");
    };
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].write_val, val) catch {
        std.debug.print("  Write32 failed at cycle {d}. Expected value {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].write_val, val });
        TestState.test_data.log();
        @panic("write32 value error");
    };
}

fn write64(addr: u32, val: u64) void {
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].write_addr, addr) catch {
        std.debug.print("  Write64 failed at cycle {d}. Expected address {?X:0>8}, got {?X:0>8}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].write_addr, addr });
        TestState.test_data.log();
        @panic("write64 address error");
    };
    std.testing.expectEqual(TestState.test_data.cycles[TestState.cycle].write_val, val) catch {
        std.debug.print("  Write64 failed at cycle {d}. Expected value {?X:0>16}, got {?X:0>16}\n", .{ TestState.cycle, TestState.test_data.cycles[TestState.cycle].write_val, val });
        TestState.test_data.log();
        @panic("write64 value error");
    };
}

fn run_test(t: Test, cpu: *SH4Module.SH4, comptime log: bool) !void {
    TestState = .{ .cpu = cpu, .cycle = 0, .test_data = t };

    cpu.set_sr(@bitCast(t.initial.SR));
    cpu.set_fpscr(@bitCast(t.initial.FPSCR));
    for (0..16) |i| {
        cpu.r[i] = t.initial.R[i];
        cpu.fp_banks[0].fr[i] = @bitCast(t.initial.FP0[i]);
        cpu.fp_banks[1].fr[i] = @bitCast(t.initial.FP1[i]);
    }
    for (0..8) |i| {
        cpu.r_bank[i] = t.initial.R_[i];
    }
    cpu.pc = t.initial.PC;
    cpu.gbr = t.initial.GBR;
    cpu.ssr = @bitCast(t.initial.SSR);
    cpu.spc = t.initial.SPC;
    cpu.vbr = t.initial.VBR;
    cpu.sgr = t.initial.SGR;
    cpu.dbr = t.initial.DBR;
    cpu.mach = t.initial.MACH;
    cpu.macl = t.initial.MACL;
    cpu.pr = t.initial.PR;
    cpu.fpul = t.initial.FPUL;

    cpu.debug_trace = log;
    if (log) {
        t.log();
    }

    while (TestState.cycle < 4) {
        try std.testing.expectEqual(t.cycles[TestState.cycle].fetch_addr, cpu.pc);

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

        TestState.cycle += 1;
        cpu.pc += 2;
    }

    try std.testing.expectEqualSlices(u32, t.final.R, &cpu.r);
    try std.testing.expectEqualSlices(u32, t.final.R_, &cpu.r_bank);
    for (0..16) |i| {
        if (!is_nan(@as(f32, @bitCast(t.final.FP0[i]))) or !is_nan(cpu.fp_banks[0].fr[i]))
            try std.testing.expectEqual(@as(f32, @bitCast(t.final.FP0[i])), cpu.fp_banks[0].fr[i]);
        if (!is_nan(@as(f32, @bitCast(t.final.FP1[i]))) or !is_nan(cpu.fp_banks[1].fr[i]))
            try std.testing.expectEqual(@as(f32, @bitCast(t.final.FP1[i])), cpu.fp_banks[1].fr[i]);
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

    var skipped_tests: u32 = 0;
    var failed_tests = std.ArrayList(struct {
        instruction: []const u8,
        failed_cases: u32,
    }).init(std.testing.allocator);
    defer failed_tests.deinit();
    var file_num: u32 = 0;
    tests_loop: while (try walker.next()) |entry| {
        if (entry.kind == .file and std.mem.endsWith(u8, entry.basename, ".json")) {
            for ([_][]const u8{
                // Others
                "0000000000011011_sz0_pr0.json", // sleep
                "0011nnnnmmmm0100_sz0_pr0.json", // div1 Rm, Rn - This one has a *potential* bug in the test data when n == m. Skipping for now.
                "0011nnnnmmmm1011_sz0_pr0.json", // subv Rm, Rn - Unimplemented
                "0011nnnnmmmm1111_sz0_pr0.json", // addv Rm, Rn - Unimplemented
                "0100mmmm01100110_sz0_pr0.json", // lds.l @Rn+,FPSCR - I'm zeroing the unused upper bits of FPSCR, which apparently reicast doesn't do? They should always be read as 0s anyway.
                "0100mmmm01101010_sz0_pr0.json", // lds Rn,FPSCR - Same thing
                "1111mmm000111101_sz0_pr1.json", // ftrc DRn,FPUL - These tests cause some FPU exceptions that I'm not emulating.
                "1111nnn010101101_sz0_pr0.json", // fcnvsd FPUL,DRn - Causes an exception when PR == 0.
                "1111mmm010111101_sz0_pr0.json", // fcnvds DRm,FPUL - Causes an exception when PR == 0.
            }) |filename| {
                if (std.mem.eql(u8, entry.basename, filename)) {
                    std.debug.print(termcolor.yellow("! Skipping {s}\n"), .{entry.basename});
                    skipped_tests += 1;
                    continue :tests_loop;
                }
            }
            file_num += 1;

            const fullpath = try std.fs.path.join(std.testing.allocator, &[_][]const u8{ TestDir, entry.basename });
            std.debug.print(termcolor.green("[{d: >3}/{d: >3}]") ++ " Opening {s}\n", .{ file_num, 233, entry.basename });
            defer std.testing.allocator.free(fullpath);
            const data = try std.fs.cwd().readFileAlloc(std.testing.allocator, fullpath, 512 * 1024 * 1024);
            defer std.testing.allocator.free(data);

            const test_data = try std.json.parseFromSlice([]Test, std.testing.allocator, data, .{});
            defer test_data.deinit();

            var failed_test_cases: u32 = 0;
            for (test_data.value) |t| {
                run_test(t, &cpu, false) catch |err| {
                    if (failed_test_cases == 0) {
                        std.debug.print(termcolor.red("Failed to run test {s}: {s}\n"), .{ entry.basename, @errorName(err) });
                        run_test(t, &cpu, true) catch {};
                        compare_state(&cpu, &t.final);
                    }
                    failed_test_cases += 1;
                };
            }
            if (failed_test_cases > 0) {
                std.debug.print(termcolor.red("  [{s}] {d}/{d} test cases failed.\n"), .{ entry.basename, failed_test_cases, test_data.value.len });
                try failed_tests.append(.{
                    .instruction = SH4Module.sh4_instructions.Opcodes[SH4Module.sh4_instructions.JumpTable[test_data.value[0].opcodes[1]]].name,
                    .failed_cases = failed_test_cases,
                });
            }
        }
    }
    if (skipped_tests > 0) {
        std.debug.print(termcolor.yellow("Skipped {d} tests.\n"), .{skipped_tests});
    }
    if (failed_tests.items.len > 0) {
        std.debug.print(termcolor.red("{d}/{d} tests failed.\n"), .{ failed_tests.items.len, file_num });
        for (failed_tests.items) |f| {
            std.debug.print(termcolor.red(" {s: <20} {d: >3}/{d} test cases failed.\n"), .{ f.instruction, f.failed_cases, 500 });
        }
        return error.TestFailed;
    }
}
