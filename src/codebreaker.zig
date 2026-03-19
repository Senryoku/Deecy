const std = @import("std");

pub const Type = enum(u8) {
    /// Writes a single byte
    u8 = 0x00,
    /// Writes a single word
    u16 = 0x01,
    /// Writes a single dword
    u32 = 0x02,
    /// Writes multiple dword
    Repeat = 0x04,
    /// Copies N bytes from a first address to another
    Copy = 0x05,
    Unknown = 0xB,
    /// Conditionally executes the next code
    Condition = 0xD,
    /// Conditionally executes the next N codes
    MultipleCondition = 0xE,
    /// Writes once before running the game
    StartWrite = 0xF,
    _,
};

pub const Condition = enum(u4) {
    Equal = 0x0,
    Different = 0x1,
    LessThan = 0x2,
    GreaterThan = 0x3,
};

pub const Code = union(Type) {
    u8: struct { address: u32, value: u8 },
    u16: struct { address: u32, value: u16 },
    u32: struct { address: u32, value: u32 },
    Repeat: struct {
        address: u32,
        value: u32,
        count: u32,
    },
    Copy: struct {
        source: u32,
        target: u32,
        count: u32,
    },
    Unknown: void,
    Condition: struct {
        condition: Condition,
        address: u32,
    },
    MultipleCondition: struct {
        condition: Condition,
        address: u32,
        count: u32,
    },
    StartWrite: void,
};

fn toss_whitespace(reader: *std.Io.Reader) !void {
    while (std.ascii.isWhitespace((try reader.peek(1))[0])) reader.toss(1);
}

// Caller owns returned slice
pub fn parse(allocator: std.mem.Allocator, reader: *std.Io.Reader) ![]Code {
    var codes: std.ArrayList(Code) = .empty;
    defer codes.deinit(allocator);
    while (true) {
        toss_whitespace(reader) catch break;
        const c_str = reader.take(8) catch break;
        if (c_str.len < 8) break;
        const control = try std.fmt.parseUnsigned(u32, c_str, 16);
        const t: Type = @enumFromInt(control >> 24);
        switch (t) {
            inline .u8, .u16, .u32 => |u| {
                const address = 0x0C000000 | control & 0x00FFFFFF;
                try toss_whitespace(reader);
                const value = try std.fmt.parseUnsigned(switch (u) {
                    .u8 => u8,
                    .u16 => u16,
                    .u32 => u32,
                    else => unreachable,
                }, try reader.take(8), 16);
                try codes.append(allocator, @unionInit(Code, @tagName(u), .{ .address = address, .value = value }));
            },
            else => return error.UnsupportedType,
            _ => return error.UnknownType,
        }
    }
    return codes.toOwnedSlice(allocator);
}

test "parse" {
    const allocator = std.testing.allocator;
    const buff =
        \\ 026625C0
        \\ 
        \\ 43700000
        \\ 
        \\ 
    ;
    var reader: std.Io.Reader = .fixed(buff);
    const codes = try parse(allocator, &reader);
    defer allocator.free(codes);
    std.debug.print("codes: {any}\n", .{codes});
}
