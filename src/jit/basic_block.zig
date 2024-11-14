const builtin = @import("builtin");

pub const EnableInstrumentation = false and builtin.mode != .ReleaseFast;

offset: u32,
cycles: u32 = 0,

time_spent: if (EnableInstrumentation) i128 else void = if (EnableInstrumentation) 0 else {},
call_count: if (EnableInstrumentation) u64 else void = if (EnableInstrumentation) 0 else {},
start_addr: if (EnableInstrumentation) u32 else void = if (EnableInstrumentation) 0 else {},
len: if (EnableInstrumentation) u32 else void = if (EnableInstrumentation) 0 else {},

pub fn execute(self: *const @This(), buffer: []const u8, user_data: *anyopaque) void {
    @as(*const fn (*anyopaque) void, @ptrCast(&buffer[self.offset]))(user_data);
}
