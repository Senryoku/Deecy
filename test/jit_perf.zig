const std = @import("std");
const builtin = @import("builtin");

const DreamcastModule = @import("dreamcast");
const Dreamcast = DreamcastModule.Dreamcast;

// Very quick and dirty perf test to avoid doing random optimisation completely blindly.

pub const std_options: std.Options = .{
    .log_level = .err,
};

const Root = if (builtin.os.tag == .windows) "D:/DC Games/" else "/media/senryoku/Data1/DC Games/";

pub fn main() !void {
    const cycles_target = 10 * 200_000_000;
    var total_time: u64 = 0;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    var allocator = gpa.allocator();

    {
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
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
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        dc.gdrom.disc = try DreamcastModule.GDROM.Disc.init(Root ++ "[GDI] Sonic Adventure (US)[51000-A]/Sonic Adventure v1.005 (1999)(Sega)(NTSC)(US)(M5)[!][%51000-A].gdi", allocator);
        _ = dc.gdrom.disc.?.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);
        _ = try dc.gdrom.disc.?.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]);

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
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        dc.skip_bios(true);
        dc.gdrom.disc = try DreamcastModule.GDROM.Disc.init(Root ++ "[GDI] Sonic Adventure (US)[51000-A]/Sonic Adventure v1.005 (1999)(Sega)(NTSC)(US)(M5)[!][%51000-A].gdi", allocator);
        _ = dc.gdrom.disc.?.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);
        _ = try dc.gdrom.disc.?.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]);

        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick_jit();
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[JIT] Sonic: Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    {
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        dc.gdrom.disc = try DreamcastModule.GDROM.Disc.init(Root ++ "dca3.cdi", allocator);
        {
            const file = try std.fs.cwd().openFile("logs/dc_states/dca3_game_start.bin", .{ .mode = .read_only });
            defer file.close();
            _ = try dc.deserialize(file.reader());
        }
        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick_jit();
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[JIT] DCA3: Ran {d} cycles in {} ms\n", .{ cycles, elapsed / std.time.ns_per_ms });
    }

    std.debug.print("[JIT] Total: {} ms\n", .{total_time / std.time.ns_per_ms});
}
