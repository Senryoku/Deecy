const std = @import("std");

const common = @import("../src/common.zig");
const GDI = @import("../src/gdi.zig").GDI;
const Dreamcast = @import("dreamcast").Dreamcast;

// Very quick and dirty perf test to avoid doing random optimisation completely blindly.

pub const std_options: std.Options = .{
    .log_level = .err,
};

pub fn main() !void {
    const cycles_target = 5 * 200_000_000;
    var total_time: u64 = 0;

    {
        var dc = try Dreamcast.create(common.GeneralAllocator);
        defer {
            dc.deinit();
            common.GeneralAllocator.destroy(dc);
        }

        dc.skip_bios(true);
        // Skip IP.bin.
        dc.cpu.pc = 0xAC010000;

        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick_jit();
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[JIT] Boot:  Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    {
        var dc = try Dreamcast.create(common.GeneralAllocator);
        defer {
            dc.deinit();
            common.GeneralAllocator.destroy(dc);
        }

        dc.gdrom.disk = try GDI.init("./bin/[GDI] Sonic Adventure (US)[51000-A]/Sonic Adventure v1.005 (1999)(Sega)(NTSC)(US)(M5)[!][%51000-A].gdi", common.GeneralAllocator);
        _ = dc.gdrom.disk.?.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);
        _ = try dc.gdrom.disk.?.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]);

        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick_jit();
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[JIT] Boot Logo: Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    {
        var dc = try Dreamcast.create(common.GeneralAllocator);
        defer {
            dc.deinit();
            common.GeneralAllocator.destroy(dc);
        }

        dc.skip_bios(true);
        dc.gdrom.disk = try GDI.init("./bin/[GDI] Sonic Adventure (US)[51000-A]/Sonic Adventure v1.005 (1999)(Sega)(NTSC)(US)(M5)[!][%51000-A].gdi", common.GeneralAllocator);
        _ = dc.gdrom.disk.?.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);
        _ = try dc.gdrom.disk.?.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]);

        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick_jit();
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[JIT] Sonic: Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    std.debug.print("[JIT] Total: {} ms\n", .{total_time / std.time.ns_per_ms});
}
