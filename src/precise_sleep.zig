const std = @import("std");
const builtin = @import("builtin");

pub extern "kernel32" fn timeBeginPeriod(
    uPeriod: std.os.windows.UINT,
) callconv(.winapi) std.os.windows.UINT; // MMRESULT
pub extern "kernel32" fn timeEndPeriod(
    uPeriod: std.os.windows.UINT,
) callconv(.winapi) std.os.windows.UINT; // MMRESULT

next_deadline: i128,

pub fn init() @This() {
    // Request 1ms timer resolution on Windows.
    if (builtin.os.tag == .windows) _ = timeBeginPeriod(1);
    return .{ .next_deadline = nano_timestamp() };
}

pub fn deinit(_: *@This()) void {
    if (builtin.os.tag == .windows) _ = timeEndPeriod(1);
}

/// Wait for ns nanoseconds *from the last call to this function*.
pub fn wait_for_interval(self: *@This(), ns: u64) void {
    self.next_deadline += ns;
    const now = nano_timestamp();
    if (now < self.next_deadline) {
        // Sleep until we're close to the deadline.
        if (self.next_deadline - now > 1_000_000)
            std.Io.sleep(std.Options.debug_io, .fromNanoseconds(@intCast(self.next_deadline - now - 1_000_000)), .real) catch {};
        // Spin for the last millisecond.
        while (nano_timestamp() < self.next_deadline) {}
    } else if (self.next_deadline < now - ns) {
        // We are very late, push the deadline forward to give us a chance to catch up.
        self.next_deadline = now;
    }
}

fn nano_timestamp() i128 {
    return std.Io.Clock.real.now(std.Options.debug_io).toNanoseconds();
}
