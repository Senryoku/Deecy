const std = @import("std");

const common = @import("common.zig");
const sh4 = @import("sh4.zig");

// FIXME
const syscall = @import("syscall.zig");

// Very quick and dirty perf test to avoid doing random optimisation completely blindly.

pub fn main() !void {
    var cpu = try sh4.SH4.init(common.GeneralAllocator);
    defer cpu.deinit();

    var gdrom = &syscall.gdrom; // FIXME
    defer gdrom.disk.deinit();
    cpu.init_boot();
    try gdrom.disk.init("./bin/[GDI] Sonic Adventure (PAL)/Sonic Adventure v1.003 (1999)(Sega)(PAL)(M5)[!].gdi", common.GeneralAllocator);
    _ = gdrom.disk.load_sectors(45150, 16 * 2048, cpu.ram[0x00008000..]);
    syscall.FirstReadBINSectorSize = (try gdrom.disk.load_file("1ST_READ.BIN;1", cpu.ram[0x00010000..]) + 2047) / 2048;

    const start = try std.time.Instant.now();
    for (0..100_000_000) |_| {
        cpu.execute();
    }
    const elapsed = (try std.time.Instant.now()).since(start);
    std.debug.print("{} ms\n", .{elapsed / std.time.ns_per_ms});
}
