bytes: u64,

pub fn fromBytes(bytes: u64) @This() {
    return .{ .bytes = bytes };
}

pub fn fromKiB(kibibytes: u54) @This() {
    return .{ .bytes = @as(u64, kibibytes) * kib };
}

pub fn fromMiB(mebibytes: u44) @This() {
    return .{ .bytes = @as(u64, mebibytes) * mib };
}

pub fn fromGiB(gibibytes: u34) @This() {
    return .{ .bytes = @as(u64, gibibytes) * gib };
}

pub fn fromTiB(tebibytes: u24) @This() {
    return .{ .bytes = @as(u64, tebibytes) * tib };
}

pub fn fromPiB(pebibytes: u14) @This() {
    return .{ .bytes = @as(u64, pebibytes) * pib };
}

pub fn fromEiB(exbibytes: u4) @This() {
    return .{ .bytes = @as(u64, exbibytes) * eib };
}

pub fn asBytes(self: @This()) u64 {
    return self.bytes;
}

pub fn asKiB(self: @This()) u54 {
    return @truncate(self.bytes >> 10);
}

pub fn asMiB(self: @This()) u44 {
    return @truncate(self.bytes >> 20);
}

pub fn asGiB(self: @This()) u34 {
    return @truncate(self.bytes >> 30);
}

pub fn asTiB(self: @This()) u24 {
    return @truncate(self.bytes >> 40);
}

pub fn asPiB(self: @This()) u14 {
    return @truncate(self.bytes >> 50);
}

pub fn asEiB(self: @This()) u4 {
    return @truncate(self.bytes >> 60);
}

pub fn format(self: @This(), writer: *std.Io.Writer) !void {
    switch (self.bytes) {
        0...kib - 1 => try writer.print("{} B", .{self.bytes}),
        kib...mib - 1 => try formatUnit(self, "KiB", kib, writer),
        mib...gib - 1 => try formatUnit(self, "MiB", mib, writer),
        gib...tib - 1 => try formatUnit(self, "GiB", gib, writer),
        tib...pib - 1 => try formatUnit(self, "TiB", tib, writer),
        pib...eib - 1 => try formatUnit(self, "PiB", pib, writer),
        else => try formatUnit(self, "EiB", eib, writer),
    }
}

fn formatUnit(self: @This(), comptime unit: []const u8, comptime factor: comptime_int, writer: *std.Io.Writer) !void {
    if (self.bytes % factor == 0)
        return writer.print("{} " ++ unit, .{self.bytes / factor});

    const f_bytes: f64 = @floatFromInt(self.bytes);

    if (100 * factor > std.math.maxInt(u64)) return switch (self.bytes) { // EiB special case.
        factor...10 * factor - 1 => try writer.print("{d:.2} " ++ unit, .{f_bytes / @as(f64, factor)}),
        else => try writer.print("{d:.1} " ++ unit, .{f_bytes / @as(f64, factor)}),
    };

    switch (self.bytes) {
        factor...10 * factor - 1 => try writer.print("{d:.2} " ++ unit, .{f_bytes / @as(f64, factor)}),
        10 * factor...100 * factor - 1 => try writer.print("{d:.1} " ++ unit, .{f_bytes / @as(f64, factor)}),
        100 * factor...1024 * factor - 1 => try writer.print("{} " ++ unit, .{self.bytes / factor}),
        else => unreachable,
    }
}

pub const zero = fromBytes(0);

const kib = 1024;
const mib = 1024 * kib;
const gib = 1024 * mib;
const tib = 1024 * gib;
const pib = 1024 * tib;
const eib = 1024 * pib;

const std = @import("std");
