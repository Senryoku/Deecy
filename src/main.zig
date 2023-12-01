const std = @import("std");

const sh4 = @import("./sh4.zig");

pub fn main() !void {
    std.debug.print("\r  == Katana ==                             \n", .{});

    var cpu: sh4.SH4 = .{};
    try cpu.init();
    defer cpu.deinit();

    for (0..120) |_| {
        cpu.execute();
    }
}

test "all tests" {
    _ = sh4;
}
