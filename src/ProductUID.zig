//! Unique identifier for a game version as found on discs (IP.BIN)

/// Title from IP.BIN
name: []const u8,
/// Version from IP.BIN
id: []const u8,

pub fn eql(self: @This(), other: @This()) bool {
    return std.mem.eql(u8, self.name, other.name) and std.mem.eql(u8, self.id, other.id);
}

pub fn format(self: @This(), writer: *std.Io.Writer) !void {
    try writer.print("'{s}' ({s})", .{ self.name, self.id });
}

const std = @import("std");
