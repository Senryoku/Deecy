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

test {
    defer DreamcastModule.SH4Module.sh4_disassembly.free_disassembly_cache(std.testing.allocator);

    const TestDir = "../SingleStepTests_sh4/";
    var test_dir = try std.fs.cwd().openDir(TestDir, .{ .iterate = true });
    defer test_dir.close();

    var walker = try test_dir.walk(std.testing.allocator);
    defer walker.deinit();

    while (try walker.next()) |entry| {
        if (entry.kind == .file and std.mem.endsWith(u8, entry.basename, ".json")) {
            const fullpath = try std.fs.path.join(std.testing.allocator, &[_][]const u8{ TestDir, entry.basename });
            std.debug.print("Opening {s}\n", .{fullpath});
            defer std.testing.allocator.free(fullpath);
            const data = try std.fs.cwd().readFileAlloc(std.testing.allocator, fullpath, 512 * 1024 * 1024);
            defer std.testing.allocator.free(data);

            const test_data = try std.json.parseFromSlice([]Test, std.testing.allocator, data, .{});
            defer test_data.deinit();

            for (test_data.value) |t| {
                var dc = try DreamcastModule.Dreamcast.create(std.testing.allocator);
                defer {
                    dc.deinit();
                    std.testing.allocator.destroy(dc);
                }

                var cpu = &dc.cpu;

                for (0..16) |i| {
                    cpu.r[i] = t.initial.R[i];
                    cpu.fp_banks[0].fr[i] = t.initial.FP0[i];
                    cpu.fp_banks[1].fr[i] = t.initial.FP1[i];
                }
                for (0..8) |i| {
                    cpu.r_bank[i] = t.initial.R_[i];
                }
                cpu.pc = 0x0C000000 + (t.initial.PC & 0x00FFFFFF);
                cpu.gbr = t.initial.GBR;
                cpu.sr = @bitCast(t.initial.SR);
                cpu.spc = t.initial.SPC;
                cpu.vbr = t.initial.VBR;
                cpu.sgr = t.initial.SGR;
                cpu.dbr = t.initial.DBR;
                cpu.mach = t.initial.MACH;
                cpu.macl = t.initial.MACL;
                cpu.pr = t.initial.PR;
                cpu.fpscr = @bitCast(t.initial.FPSCR);

                var addr = cpu.pc;
                for (t.opcodes) |opcode| {
                    // @as(*u16, @alignCast(@ptrCast(&dc.ram[addr & 0x00FFFFFF]))).* = opcode;
                    dc.cpu.write16(addr, opcode);
                    addr += 2;
                }

                var cycles: u32 = 0;
                while (cycles < 4) {
                    std.debug.print(" > {s}\n", .{try DreamcastModule.SH4Module.sh4_disassembly.disassemble(@bitCast(dc.cpu.read16(dc.cpu.pc)), std.testing.allocator)});
                    cycles += cpu.execute(1);
                }

                try std.testing.expectEqualSlices(u32, t.final.R, &cpu.r);
                try std.testing.expectEqualSlices(u32, t.final.R_, &cpu.r_bank);
                for (0..16) |i| {
                    try std.testing.expectEqual(t.final.FP0[i], cpu.fp_banks[0].fr[i]);
                    try std.testing.expectEqual(t.final.FP1[i], cpu.fp_banks[1].fr[i]);
                }
                try std.testing.expectEqual(0x0C000000 + (t.final.PC & 0x00FFFFFF), 0x0C000000 + (cpu.pc & 0x00FFFFFF));
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
        }
    }
}
