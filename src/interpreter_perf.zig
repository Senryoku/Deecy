const std = @import("std");

const common = @import("common.zig");
const Dreamcast = @import("dreamcast.zig").Dreamcast;

// FIXME
const syscall = @import("syscall.zig");

// Very quick and dirty perf test to avoid doing random optimisation completely blindly.

pub fn main() !void {
    var dc = try Dreamcast.create(common.GeneralAllocator);
    defer {
        dc.deinit();
        common.GeneralAllocator.destroy(dc);
    }

    dc.skip_bios();
    try dc.gdrom.disk.init("./bin/[GDI] Sonic Adventure (PAL)/Sonic Adventure v1.003 (1999)(Sega)(PAL)(M5)[!].gdi", common.GeneralAllocator);
    _ = dc.gdrom.disk.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);
    syscall.FirstReadBINSectorSize = (try dc.gdrom.disk.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]) + 2047) / 2048;

    const steps = 100_000_000;
    const start = try std.time.Instant.now();
    for (0..steps) |_| {
        _ = dc.tick();
    }
    const elapsed = (try std.time.Instant.now()).since(start);
    std.debug.print("Ran {d} instructions in {} ms\n", .{ steps, elapsed / std.time.ns_per_ms });
}
