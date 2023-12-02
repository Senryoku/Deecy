const std = @import("std");

const sh4 = @import("./sh4.zig");

pub fn main() !void {
    std.debug.print("\r  == Katana ==                             \n", .{});

    var cpu: sh4.SH4 = .{};
    try cpu.init();
    defer cpu.deinit();

    for (0..8000000) |_| {
        if (cpu.pc == 0x8C0000E8) cpu.debug_trace = false;
        if (cpu.pc > 0x8C0000F0) cpu.debug_trace = true;
        cpu.execute();
    }
}

test "all tests" {
    _ = sh4;
}
