const std = @import("std");

const BasicBlock = @import("basic_block.zig").BasicBlock;

const Emitter = struct {
    block: BasicBlock,
    _allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) @This() {
        return .{
            .block = BasicBlock.init(allocator),
            ._allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        self.block.deinit();
    }

    pub fn emit(self: *@This(), comptime T: type, value: T) !void {
        for (0..@sizeOf(T)) |i| {
            try self.block.emit((value >> @intCast(8 * i)) & 0xFF);
        }
    }

    pub fn native_call(self: *@This(), comptime T: type, function: T) !void {
        // mov rax, function
        try self.emit(u8, 0x48);
        try self.emit(u8, 0xB8);
        try self.emit(u64, @intFromPtr(function));
        // call rax
        try self.emit(u8, 0xFF);
        try self.emit(u8, 0xD0);
    }

    pub fn ret(self: *@This()) !void {
        try self.emit(u8, 0xC3);
    }
};
