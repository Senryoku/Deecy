const std = @import("std");
const builtin = @import("builtin");

pub extern "kernel32" fn timeBeginPeriod(
    uPeriod: std.os.windows.UINT,
) callconv(.winapi) std.os.windows.UINT; // MMRESULT
pub extern "kernel32" fn timeEndPeriod(
    uPeriod: std.os.windows.UINT,
) callconv(.winapi) std.os.windows.UINT; // MMRESULT

next_deadline: i96,

pub fn init(io: std.Io) @This() {
    // Request 1ms timer resolution on Windows.
    if (builtin.os.tag == .windows) _ = timeBeginPeriod(1);
    return .{ .next_deadline = nano_timestamp(io) };
}

pub fn deinit(_: *@This()) void {
    if (builtin.os.tag == .windows) _ = timeEndPeriod(1);
}

/// Wait for ns nanoseconds *from the last call to this function*.
pub fn wait_for_interval(self: *@This(), io: std.Io, ns: u64) void {
    self.next_deadline += ns;
    const now = nano_timestamp(io);
    if (now < self.next_deadline) {
        // Sleep until we're close to the deadline.
        if (self.next_deadline - now > 1_000_000)
            std.Io.sleep(io, .fromNanoseconds(self.next_deadline - now - 1_000_000), .awake) catch {};
        // Spin for the last millisecond.
        while (nano_timestamp(io) < self.next_deadline)
            std.atomic.spinLoopHint();
    } else if (self.next_deadline < now - ns) {
        // We are very late, push the deadline forward to give us a chance to catch up.
        self.next_deadline = now;
    }
}

fn nano_timestamp(io: std.Io) i96 {
    return std.Io.Timestamp.now(io, .awake).toNanoseconds();
}
