const std = @import("std");

const common = @import("common.zig");
const GDI = @import("gdi.zig").GDI;
const Dreamcast = @import("dreamcast.zig").Dreamcast;

// FIXME
const syscall = @import("syscall.zig");

// Very quick and dirty perf test to avoid doing random optimisation completely blindly.

pub fn main() !void {
    const steps = 200_000_000;

    var total_time: u64 = 0;

    {
        var dc = try Dreamcast.create(common.GeneralAllocator);
        defer {
            dc.deinit();
            common.GeneralAllocator.destroy(dc);
        }

        dc.skip_bios();
        // Skip IP.bin.
        dc.cpu.pc = 0xAC010000;

        const start = try std.time.Instant.now();
        for (0..steps) |_| {
            _ = dc.tick();
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("Boot:  Ran {d} instructions in {} ms\n", .{ steps, elapsed / std.time.ns_per_ms });
    }

    {
        var dc = try Dreamcast.create(common.GeneralAllocator);
        defer {
            dc.deinit();
            common.GeneralAllocator.destroy(dc);
        }

        dc.skip_bios();
        dc.gdrom.disk = try GDI.init("./bin/[GDI] Sonic Adventure (PAL)/Sonic Adventure v1.003 (1999)(Sega)(PAL)(M5)[!].gdi", common.GeneralAllocator);
        _ = dc.gdrom.disk.?.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);
        syscall.FirstReadBINSectorSize = (try dc.gdrom.disk.?.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]) + 2047) / 2048;

        const start = try std.time.Instant.now();
        for (0..steps) |_| {
            _ = dc.tick();
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("Sonic: Ran {d} instructions in {} ms\n", .{ steps, elapsed / std.time.ns_per_ms });
    }

    std.debug.print("Total: {} ms\n", .{total_time / std.time.ns_per_ms});
}
