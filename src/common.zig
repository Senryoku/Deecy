const std = @import("std");

var gpa = std.heap.GeneralPurposeAllocator(.{}){};
pub const GeneralAllocator = gpa.allocator();
