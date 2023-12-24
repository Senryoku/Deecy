const std = @import("std");

var gpa = std.heap.GeneralPurposeAllocator(.{}){};
pub const GeneralAllocator = gpa.allocator();

pub const addr_t = u32;
pub const byte_t = u8;
pub const word_t = u16;
pub const longword_t = u32;
