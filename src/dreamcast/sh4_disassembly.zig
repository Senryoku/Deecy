const std = @import("std");
const sh4 = @import("sh4.zig");
const sh4_instructions = sh4.sh4_instructions;
const Instr = sh4.Instr;
const bit_manip = @import("bit_manip.zig");

var DisassemblyCache: [0x10000]?[]const u8 = .{null} ** 0x10000;

pub fn disassemble(opcode: Instr, allocator: std.mem.Allocator) ![]const u8 {
    if (DisassemblyCache[opcode.value]) |r|
        return r;

    var arena = std.heap.ArenaAllocator.init(allocator);
    defer arena.deinit();
    const arena_allocator = arena.allocator();

    const desc = sh4_instructions.Opcodes[sh4_instructions.JumpTable[opcode.value]];

    const Rn = try std.fmt.allocPrint(arena_allocator, "R{d}", .{opcode.nmd.n});
    const Rm = try std.fmt.allocPrint(arena_allocator, "R{d}", .{opcode.nmd.m});
    const disp = try std.fmt.allocPrint(arena_allocator, "{d}", .{opcode.nmd.d});
    const d8 = try std.fmt.allocPrint(arena_allocator, "{d}", .{@as(i8, @bitCast(opcode.nd8.d))});
    const d12 = try std.fmt.allocPrint(arena_allocator, "{d}", .{@as(i12, @bitCast(opcode.d12.d))});
    const imm = try std.fmt.allocPrint(arena_allocator, "#{d}", .{@as(i8, @bitCast(opcode.nd8.d))});
    const label8 = try std.fmt.allocPrint(arena_allocator, "{X}", .{4 + 2 * bit_manip.sign_extension_u8(opcode.nd8.d)});
    const label12 = try std.fmt.allocPrint(arena_allocator, "{X}", .{4 + 2 * bit_manip.sign_extension_u12(opcode.d12.d)});

    const n0 = try std.mem.replaceOwned(u8, arena_allocator, desc.name, "Rn", Rn);
    const n1 = try std.mem.replaceOwned(u8, arena_allocator, n0, "Rm", Rm);
    const n2 = try std.mem.replaceOwned(u8, arena_allocator, n1, "disp", disp);
    const n3 = try std.mem.replaceOwned(u8, arena_allocator, n2, "d:8", d8);
    const n4 = try std.mem.replaceOwned(u8, arena_allocator, n3, "d:12", d12);
    const n5 = try std.mem.replaceOwned(u8, arena_allocator, n4, "label:8", label8);
    const n6 = try std.mem.replaceOwned(u8, arena_allocator, n5, "label:12", label12);

    const final_buff = try std.mem.replaceOwned(u8, allocator, n6, "#imm", imm);

    DisassemblyCache[opcode.value] = final_buff;

    return final_buff;
}

pub fn free_disassembly_cache(allocator: std.mem.Allocator) void {
    for (&DisassemblyCache) |*dc| {
        if (dc.*) |d|
            allocator.free(d);
        dc.* = null;
    }
}

fn test_decoding(instruction: Instr, comptime expected: []const u8) !void {
    const dis = try disassemble(instruction, std.testing.allocator);
    std.debug.print("{b:0>16}: {s} - {s}\n", .{ instruction.value, dis, expected });
    try std.testing.expect(std.mem.eql(u8, dis, expected));
}

test "Instruction Decoding" {
    defer free_disassembly_cache(std.testing.allocator);

    try test_decoding(.{ .value = 0b1110_0000_0000_0001 }, "mov #1,R0");

    try test_decoding(.{ .value = 0b1001_0000_0000_0001 }, "mov.w @(1,PC),R0");
    try test_decoding(.{ .value = 0b1101_0000_0000_0001 }, "mov.l @(1,PC),R0");

    try test_decoding(.{ .value = 0b0110_0000_0000_0011 }, "mov R0,R0");

    try test_decoding(.{ .value = 0b0010_0000_0001_0000 }, "mov.b R1,@R0");
    try test_decoding(.{ .value = 0b0010_0000_0001_0001 }, "mov.w R1,@R0");
    try test_decoding(.{ .value = 0b0010_0000_0001_0010 }, "mov.l R1,@R0");

    try test_decoding(.{ .value = 0b0110_0000_0001_0000 }, "mov.b @R1,R0");
    try test_decoding(.{ .value = 0b0110_0000_0001_0001 }, "mov.w @R1,R0");
    try test_decoding(.{ .value = 0b0110_0000_0001_0010 }, "mov.l @R1,R0");

    try test_decoding(.{ .value = 0b0010_0000_0001_0100 }, "mov.b R1,@-R0");
    try test_decoding(.{ .value = 0b0010_0000_0001_0101 }, "mov.w R1,@-R0");
    try test_decoding(.{ .value = 0b0010_0000_0001_0110 }, "mov.l R1,@-R0");

    try test_decoding(.{ .value = 0b0110_0000_0001_0100 }, "mov.b @R1+,R0");
    try test_decoding(.{ .value = 0b0110_0000_0001_0101 }, "mov.w @R1+,R0");
    try test_decoding(.{ .value = 0b0110_0000_0001_0110 }, "mov.l @R1+,R0");

    try test_decoding(.{ .value = 0b1000_0000_0001_0010 }, "mov.b R0,@(2,R1)");
    try test_decoding(.{ .value = 0b1000_0001_0001_0010 }, "mov.w R0,@(2,R1)");

    try test_decoding(.{ .value = 0b0001_0001_0011_0010 }, "mov.l R3,@(2,R1)");

    try test_decoding(.{ .value = 0b1000_0100_0001_0010 }, "mov.b @(2,R1),R0");
    try test_decoding(.{ .value = 0b1000_0101_0001_0010 }, "mov.w @(2,R1),R0");

    try test_decoding(.{ .value = 0b0101_0011_0001_0010 }, "mov.l @(2,R1),R3");

    try test_decoding(.{ .value = 0b0000_0001_0011_0100 }, "mov.b R3,@(R0,R1)");
    try test_decoding(.{ .value = 0b0000_0001_0011_0101 }, "mov.w R3,@(R0,R1)");
    try test_decoding(.{ .value = 0b0000_0001_0011_0110 }, "mov.l R3,@(R0,R1)");

    try test_decoding(.{ .value = 0b0000_0001_0011_1100 }, "mov.b @(R0,R3),R1");
    try test_decoding(.{ .value = 0b0000_0001_0011_1101 }, "mov.w @(R0,R3),R1");
    try test_decoding(.{ .value = 0b0000_0001_0011_1110 }, "mov.l @(R0,R3),R1");

    try test_decoding(.{ .value = 0b1100_0000_0000_0001 }, "mov.b R0,@(1,GBR)");
    try test_decoding(.{ .value = 0b1100_0001_0000_0001 }, "mov.w R0,@(1,GBR)");
    try test_decoding(.{ .value = 0b1100_0010_0000_0001 }, "mov.l R0,@(1,GBR)");

    try test_decoding(.{ .value = 0b1100_0100_0000_0001 }, "mov.b @(1,GBR),R0");
    try test_decoding(.{ .value = 0b1100_0101_0000_0001 }, "mov.w @(1,GBR),R0");
    try test_decoding(.{ .value = 0b1100_0110_0000_0001 }, "mov.l @(1,GBR),R0");

    try test_decoding(.{ .value = 0b1100_0111_0000_0000 }, "mova @(0,PC),R0");
    try test_decoding(.{ .value = 0b0000_0011_0010_1001 }, "movt R3");

    try test_decoding(.{ .value = 0b1000_1011_0000_0100 }, "bf C");
    try test_decoding(.{ .value = 0b1000_1111_0000_0100 }, "bf/s C");
    try test_decoding(.{ .value = 0b1000_1001_0000_0100 }, "bt C");
    try test_decoding(.{ .value = 0b1000_1101_0000_0100 }, "bt/s C");
    try test_decoding(.{ .value = 0b1010_0000_0000_0100 }, "bra C");
    try test_decoding(.{ .value = 0b0000_0001_0010_0011 }, "braf R1");
    try test_decoding(.{ .value = 0b1011_0000_0000_0100 }, "bsr C");
    try test_decoding(.{ .value = 0b0000_0001_0000_0011 }, "bsrf R1");
    try test_decoding(.{ .value = 0b0100_0001_0010_1011 }, "jmp @R1");
    try test_decoding(.{ .value = 0b0100_0001_0000_1011 }, "jsr @R1");
    try test_decoding(.{ .value = 0b0000_0000_0000_1011 }, "rts");
}
