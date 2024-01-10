const std = @import("std");

const BasicBlock = struct {
    buffer: []u8,
    end: u32 = undefined,

    start_address: u32,
    end_address: u32 = undefined,

    cycles: u32 = 0,

    pub fn init(buffer: []u8, start_address: u32) @This() {
        return .{
            .buffer = buffer,
            .start_address = start_address,
        };
    }
};
