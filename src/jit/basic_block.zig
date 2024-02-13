const std = @import("std");

buffer: []u8,
cycles: u32 = 0,

pub fn init(buffer: []u8) @This() {
    return .{
        .buffer = buffer,
    };
}

pub fn execute(self: *@This(), user_data: *anyopaque) void {
    @as(*const fn (*anyopaque) void, @ptrCast(self.buffer.ptr))(user_data);
}
