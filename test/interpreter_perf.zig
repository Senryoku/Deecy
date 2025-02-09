const std = @import("std");

const common = @import("../src/common.zig");
const DreamcastModule = @import("dreamcast");
const Dreamcast = DreamcastModule.Dreamcast;

// Very quick and dirty perf test to avoid doing random optimisation completely blindly.

pub const std_options: std.Options = .{
    .log_level = .err,
};

pub fn main() !void {
    const cycles_target = 5 * 200_000_000;
    const max_instructions = 16;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    var allocator = gpa.allocator();

    var total_time: u64 = 0;

    {
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        try dc.skip_bios(true);
        // Skip IP.bin.
        dc.cpu.pc = 0xAC010000;

        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick(max_instructions);
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[Interpreter] Boot:  Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    // Boot Logo
    {
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        dc.gdrom.disc = try DreamcastModule.GDROM.Disc.init("D:/DC Games/[GDI] Sonic Adventure (US)[51000-A]/Sonic Adventure v1.005 (1999)(Sega)(NTSC)(US)(M5)[!][%51000-A].gdi", allocator);
        _ = dc.gdrom.disc.?.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);
        _ = try dc.gdrom.disc.?.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]);

        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick(max_instructions);
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[Interpreter] Boot Logo: Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    {
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        try dc.skip_bios(true);
        dc.gdrom.disc = try DreamcastModule.GDROM.Disc.init("D:/DC Games/[GDI] Sonic Adventure (US)[51000-A]/Sonic Adventure v1.005 (1999)(Sega)(NTSC)(US)(M5)[!][%51000-A].gdi", allocator);
        _ = dc.gdrom.disc.?.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);
        _ = try dc.gdrom.disc.?.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]);

        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick(max_instructions);
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[Interpreter] Sonic: Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    {
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        dc.gdrom.disc = try DreamcastModule.GDROM.Disc.init("D:/DC Games/dca3.cdi", allocator);
        {
            const file = try std.fs.cwd().openFile("logs/dc_states/dca3_game_start.bin", .{ .mode = .read_only });
            defer file.close();
            _ = try dc.deserialize(file.reader());
        }
        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick(max_instructions);
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[Interpreter] DCA3: Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    std.debug.print("[Interpreter] Total: {} ms\n", .{total_time / std.time.ns_per_ms});
}
