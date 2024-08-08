pub const EnableInstrumentation = true;

buffer: []u8,
cycles: u32 = 0,

time_spent: if (EnableInstrumentation) i128 else void = if (EnableInstrumentation) 0 else {},
call_count: if (EnableInstrumentation) u64 else void = if (EnableInstrumentation) 0 else {},
start_addr: if (EnableInstrumentation) u32 else void = if (EnableInstrumentation) 0 else {},
len: if (EnableInstrumentation) u32 else void = if (EnableInstrumentation) 0 else {},

pub fn init(buffer: []u8) @This() {
    return .{
        .buffer = buffer,
    };
}

pub fn execute(self: *@This(), user_data: *anyopaque) void {
    @as(*const fn (*anyopaque) void, @ptrCast(self.buffer.ptr))(user_data);
}
