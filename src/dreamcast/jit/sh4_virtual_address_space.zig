const std = @import("std");
const termcolor = @import("termcolor");
const Architecture = @import("x86_64.zig");

pub fn patch_access(fault_address: u64, space_base: u64, space_size: u64, rip: *u64) !void {
    if (fault_address >= space_base and fault_address < space_base + space_size) {
        const dc_addr: u32 = @truncate(fault_address - space_base);
        std.log.scoped(.sh4_jit).debug("  Patching Access: @ {X} - {X:0>8}", .{ fault_address, dc_addr });

        const start_patch = rip.*;
        var end_patch = start_patch;

        // Skip 16bit prefix
        if (@as(*u8, @ptrFromInt(end_patch)).* == 0x66)
            end_patch += 1;

        // Skip REX
        if (0xF0 & @as(*u8, @ptrFromInt(end_patch)).* == 0x40)
            end_patch += 1;

        // Skip 0F prefix
        if (@as(*u8, @ptrFromInt(end_patch)).* == 0x0F)
            end_patch += 1;

        const modrm: Architecture.MODRM = @bitCast(@as(*u8, @ptrFromInt(end_patch + 1)).*);

        switch (modrm.mod) {
            .indirect => end_patch += 2,
            .disp8 => end_patch += 3,
            .disp32 => end_patch += 6,
            else => return error.InvalidModRM,
        }
        // Special case: Skip SIB byte
        if (modrm.r_m == 4) end_patch += 1;

        switch (@as(*u8, @ptrFromInt(end_patch)).*) {
            0xEB => end_patch += 2, // JMP rel8
            0xE9 => end_patch += 5, // JMP rel32 in 64bit mode
            else => |byte| {
                std.log.scoped(.sh4_jit).err(termcolor.red("Unhandled jump: {X:0>2}"), .{byte});
                std.log.scoped(.sh4_jit).err("  {X:0>2}", .{@as([*]u8, @ptrFromInt(end_patch))[0..16]});
                return error.InvalidJumpInstruction;
            },
        }

        // Patch out the mov and jump, we'll always execute the fallback from now on.
        Architecture.convert_to_nops(@as([*]u8, @ptrFromInt(start_patch))[0..(end_patch - start_patch)]);

        // Skip patched instructions. Not strictly necessary.
        rip.* = end_patch;
    } else return error.InvalidAddress;
}
