// https://github.com/SingleStepTests/sh4

const std = @import("std");
const DreamcastModule = @import("dreamcast");

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
};

const Test = struct {
    initial: CPUState,
    final: CPUState,
    cycles: []struct {
        actions: u32,
        fetch_addr: u32,
        fetch_val: u32,
        read_addr: ?u32 = null,
        read_val: ?u32 = null,
    },
    opcodes: []u16,
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

fn run_test(t: Test, cpu: *DreamcastModule.SH4Module.SH4, comptime log: bool) !void {
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

    if (log) {
        for (0..8) |i| {
            std.debug.print("  R{d}: {X:0>8}   R{d: <2}: {X:0>8} | R'{d}: {X:0>8}\n", .{ i, cpu.r[i], i + 8, cpu.r[i + 8], i, cpu.r_bank[i] });
        }
        std.debug.print("  PC:    {X:0>8}\n", .{cpu.pc});
        std.debug.print("  GBR:   {X:0>8}\n", .{cpu.gbr});
        std.debug.print("  SR:    {X:0>8}\n", .{@as(u32, @bitCast(cpu.sr))});
        std.debug.print("  SSR:   {X:0>8}\n", .{@as(u32, @bitCast(cpu.ssr))});
        std.debug.print("  SPC:   {X:0>8}\n", .{cpu.spc});
        std.debug.print("  VBR:   {X:0>8}\n", .{cpu.vbr});
        std.debug.print("  SGR:   {X:0>8}\n", .{cpu.sgr});
        std.debug.print("  DBR:   {X:0>8}\n", .{cpu.dbr});
        std.debug.print("  MACH:  {X:0>8}\n", .{cpu.mach});
        std.debug.print("  MACL:  {X:0>8}\n", .{cpu.macl});
        std.debug.print("  PR:    {X:0>8}\n", .{cpu.pr});
        std.debug.print("  FPSCR: {X:0>8}\n", .{@as(u32, @bitCast(cpu.fpscr))});
        std.debug.print("  FPUL:  {X:0>8}\n", .{cpu.fpul});
    }

    for (0..4) |cycle| {
        try std.testing.expect(t.cycles[cycle].fetch_addr == cpu.pc);

        const addr = cpu.pc;
        const offset: i64 = @divFloor(@as(i64, cpu.pc) - t.initial.PC, 2);
        const opcode = t.opcodes[@intCast(if (offset >= 0 and offset < 4) offset else 4)];

        const instr = DreamcastModule.SH4Module.Instr{ .value = opcode };
        const desc = DreamcastModule.SH4Module.sh4_instructions.Opcodes[DreamcastModule.SH4Module.sh4_instructions.JumpTable[opcode]];

        if (log) std.debug.print("    [{X:0>8}] {b:0>16} {s: <20} R{d: <2}={X:0>8}, R{d: <2}={X:0>8}, T={b:0>1}, Q={b:0>1}, M={b:0>1}\n", .{ addr, opcode, DreamcastModule.SH4Module.sh4_disassembly.disassemble(instr, cpu._allocator) catch {
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
}

test {
    defer DreamcastModule.SH4Module.sh4_disassembly.free_disassembly_cache(std.testing.allocator);

    const TestDir = "../SingleStepTests_sh4/";
    var test_dir = try std.fs.cwd().openDir(TestDir, .{ .iterate = true });
    defer test_dir.close();

    var walker = try test_dir.walk(std.testing.allocator);
    defer walker.deinit();

    var dc = try DreamcastModule.Dreamcast.create(std.testing.allocator);
    defer {
        dc.deinit();
        std.testing.allocator.destroy(dc);
    }

    var cpu = &dc.cpu;
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
                run_test(t, cpu, false) catch |err| {
                    std.debug.print("Failed to run test {s}: {s}\n", .{ entry.basename, @errorName(err) });
                    return run_test(t, cpu, true);
                };
                test_num += 1;
            }
        }
    }
}
