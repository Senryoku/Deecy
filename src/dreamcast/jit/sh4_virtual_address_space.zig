const std = @import("std");
const termcolor = @import("termcolor");
const Architecture = @import("x86_64.zig");

const log = std.log.scoped(.sh4_jit);

pub var read_8_offset: u64 = 0;
pub var read_16_offset: u64 = 0;
pub var read_32_offset: u64 = 0;
pub var read_64_offset: u64 = 0;
pub var write_8_offset: u64 = 0;
pub var write_16_offset: u64 = 0;
pub var write_32_offset: u64 = 0;
pub var write_64_offset: u64 = 0;

pub fn patch_access(fault_address: u64, space_base: u64, space_size: u64, rip: *u64) !void {
    if (fault_address >= space_base and fault_address < space_base + space_size) {
        const dc_addr: u32 = @truncate(fault_address - space_base);
        log.info("  Patching Access: @ {X} - {X:0>8}", .{ fault_address, dc_addr });

        const start_patch = rip.*;
        var end_patch = start_patch;
        const instructions = @as([*]u8, @ptrFromInt(start_patch));
        log.debug("    [-16] {X}", .{@as([*]u8, @ptrFromInt(start_patch - 16))[0..16]});
        log.debug("    [  0] {X}", .{instructions[0..16]});

        var size: enum { _8, _16, _32, _64, Unknown } = .Unknown;

        // Skip 16bit prefix
        if (@as(*u8, @ptrFromInt(end_patch)).* == 0x66) {
            end_patch += 1;
            size = ._16;
        }

        // Skip REX
        const rex = @as(*u8, @ptrFromInt(end_patch)).*;
        if (0xF0 & rex == 0x40) {
            end_patch += 1;
            if (rex & 0x08 != 0) size = ._64;
        }

        // Skip 0F prefix
        if (@as(*u8, @ptrFromInt(end_patch)).* == 0x0F)
            end_patch += 1;

        const opcode = @as(*u8, @ptrFromInt(end_patch)).*;

        const direction: enum { Read, Write } = switch (opcode) {
            0x88, 0x89 => .Write, // mov reg, r/m
            0x8A, 0x8B => .Read, // mov r/m, reg
            0xB6, 0xB7 => .Read, // movzx reg, r/m8 - movzx reg, r/m16
            0x6E => .Read, // movd xmm, r/m32
            0x7E => .Write, // movd r/m32, xmm
            else => {
                log.err("  Invalid Opcode: {X}", .{opcode});
                log.err("    {X}", .{instructions[0..16]});
                return error.InvalidOpcode;
            },
        };
        switch (opcode) {
            // mov r/m8, r8 ; mov r8, r/m8
            0x88, 0x8A => {
                if (size != .Unknown) return error.InvalidMOV;
                size = ._8;
            },
            // mov r/m, r ; mov r, r/m, other than 8bit
            0x89, 0x8B => {
                if (size == .Unknown) size = ._32;
            },
            // movzx
            0xB6 => size = ._8,
            0xB7 => size = ._16,
            // movd
            0x6E, 0x7E => {
                if (size != ._64) size = ._32;
            },
            else => {
                log.err("  Invalid Opcode: {X}", .{opcode});
                log.err("    {X}", .{instructions[0..16]});
                return error.InvalidOpcode;
            },
        }

        const modrm: Architecture.MODRM = @bitCast(@as(*u8, @ptrFromInt(end_patch + 1)).*);
        switch (modrm.mod) {
            .indirect => end_patch += 2,
            .disp8 => end_patch += 3,
            .disp32 => end_patch += 6,
            else => return error.InvalidModRM,
        }
        // Special case: Skip SIB byte
        if (modrm.r_m == 4) end_patch += 1;

        var patch_size = end_patch - start_patch;
        log.debug("  Detected {X}, {t}, {t}, patch_size={d}", .{ opcode, direction, size, patch_size });
        const call_size = 5;
        if (patch_size < call_size) {
            for (patch_size..call_size) |i|
                if (instructions[i] != 0x90) return error.MissingNopPadding;
            patch_size = call_size;
        }

        instructions[0] = 0xE8; // call rel32
        const offset_patch = @as(*align(1) i32, @ptrFromInt(start_patch + 1));
        const dest: i64 = @intCast(switch (direction) {
            .Read => switch (size) {
                ._8 => read_8_offset,
                ._16 => read_16_offset,
                ._32 => read_32_offset,
                ._64 => read_64_offset,
                .Unknown => return error.InvalidSize,
            },
            .Write => switch (size) {
                ._8 => write_8_offset,
                ._16 => write_16_offset,
                ._32 => write_32_offset,
                ._64 => write_64_offset,
                .Unknown => return error.InvalidSize,
            },
        });
        offset_patch.* = @intCast(dest - @as(i64, @intCast(start_patch + call_size)));

        if (patch_size > call_size)
            Architecture.convert_to_nops(instructions[call_size..patch_size]);

        log.debug("  Patched: [-16] {X}", .{@as([*]u8, @ptrFromInt(start_patch - 16))[0..16]});
        log.debug("           [  0] {X}", .{instructions[0..16]});
    } else return error.InvalidAddress;
}
