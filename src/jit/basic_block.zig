const std = @import("std");

pub const BasicBlock = struct {
    buffer: []u8,
    size: u32 = 0,

    cycles: u32 = 0,
    instr_count: u32 = 0,

    pub fn init(buffer: []u8) @This() {
        return .{
            .buffer = buffer,
        };
    }

    pub fn emit(self: *@This(), value: u8) !void {
        self.buffer[self.size] = value;
        self.size += 1;
    }

    pub fn execute(self: *@This(), user_data: *anyopaque) void {
        @as(*const fn (*anyopaque) void, @ptrCast(self.buffer.ptr))(user_data);
    }
};
