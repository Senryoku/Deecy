const std = @import("std");

pub const Instruction = enum {
    FunctionCall,
    Load,
    Store,
    Branch,
};

pub const JITBlock = struct {
    instructions: std.ArrayList(Instruction),

    pub fn init(allocator: std.mem.Allocator) JITBlock {
        return .{
            .instructions = std.ArrayList(Instruction).init(allocator),
        };
    }

    pub fn deinit(self: *JITBlock) void {
        self.instructions.deinit();
    }

    pub fn call(self: *@This(), func: anytype) void {
        _ = self;
        _ = func;
    }
};
