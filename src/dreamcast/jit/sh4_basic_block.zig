const builtin = @import("builtin");
const dc_config = @import("dc_config");

const Architecture = @import("x86_64.zig");

pub const EnableInstrumentation = dc_config.jit_instrumentation;

offset: u32,

cycles: if (EnableInstrumentation) u32 else void = if (EnableInstrumentation) 0 else {},
time_spent: if (EnableInstrumentation) i128 else void = if (EnableInstrumentation) 0 else {},
call_count: if (EnableInstrumentation) u64 else void = if (EnableInstrumentation) 0 else {},
start_addr: if (EnableInstrumentation) u32 else void = if (EnableInstrumentation) 0 else {},
len: if (EnableInstrumentation) u32 else void = if (EnableInstrumentation) 0 else {},

pub inline fn execute(self: *const @This(), buffer: []const u8, user_data: *anyopaque) u32 {
    @setRuntimeSafety(false);
    return @as(*const fn (*anyopaque) callconv(Architecture.CallingConvention) u32, @ptrCast(&buffer[self.offset]))(user_data);
}
