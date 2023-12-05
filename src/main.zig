const std = @import("std");

const common = @import("./common.zig");
const sh4 = @import("./sh4.zig");

pub fn main() !void {
    std.debug.print("\r  == Katana ==                             \n", .{});

    var cpu: sh4.SH4 = .{};
    try cpu.init();
    defer cpu.deinit();

    const IPbin_file = try std.fs.cwd().openFile("./bin/IP.bin", .{});
    defer IPbin_file.close();
    const IPbin = try IPbin_file.readToEndAlloc(common.GeneralAllocator, 0x10000);
    defer common.GeneralAllocator.free(IPbin);
    cpu.load_IP_bin(IPbin);
    cpu.init_boot();

    for (0..80000000) |_| {
        cpu.execute();
    }
}

test "all tests" {
    _ = sh4;
}
