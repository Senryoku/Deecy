const std = @import("std");

const sh4 = @import("./sh4.zig");

pub fn main() !void {
    std.debug.print("\r  == Katana ==                             \n", .{});

    var cpu: sh4.SH4 = .{};
    try cpu.init();
    defer cpu.deinit();

    for (0..100) |_| {
        cpu.execute();
    }
}

test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}
