const std = @import("std");

// Wrapper calling Deecy.exe to get proper console interaction when launched from a terminal on Windows.
// Windows forces to select the GUI or Console subsystem at compilation time, you can't have a executable
// that will behave correctly in both modes.
// You can reattach to the console from a GUI executable to get logs, but it won't block the prompt,
// this solution is the best I could find, even if it requires a second executable and a second process.

pub fn main() !void {
    const allocator = std.heap.page_allocator;

    const args = try std.process.argsAlloc(allocator);
    defer allocator.free(args);

    var deecy_args = try allocator.alloc([]const u8, args.len + 1);
    defer allocator.free(deecy_args);

    deecy_args[0] = "Deecy.exe";
    for (args[1..], 1..) |arg, i|
        deecy_args[i] = arg;
    deecy_args[args.len] = "--attach-console";

    var child = std.process.Child.init(deecy_args, allocator);

    child.stdin_behavior = .Inherit;
    child.stdout_behavior = .Inherit;
    child.stderr_behavior = .Inherit;

    try child.spawn();
    const term = try child.wait();
    switch (term) {
        .Exited => |code| std.process.exit(code),
        else => std.process.exit(1),
    }
}
