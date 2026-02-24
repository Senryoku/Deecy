const std = @import("std");

const Dreamcast = @import("../dreamcast.zig").Dreamcast;
const Maple = @import("../maple.zig");
const Instruction = Maple.Instruction;
const CommandWord = Maple.CommandWord;

extern fn dpp_send(dest: [*]u8, data: [*]u32, len: u32) callconv(.c) u32;

pub fn transfer(dc: *Dreamcast, data: [*]u32) u32 {
    var total_transferred_words: u32 = 0;
    var idx: u32 = 0;
    while (idx < 1024) {
        const instr: Instruction = @bitCast(data[idx]);
        idx += 1;

        switch (instr.pattern) {
            .Normal => {
                const return_addr = data[idx];
                const command: CommandWord = @bitCast(data[idx + 1]);
                const function_type = data[idx + 2];
                std.debug.print("  Dest: {X:0>8}, Command: {f}, Function: {f}\n", .{ return_addr, command, @as(Maple.FunctionCodesMask, @bitCast(function_type)) });

                var command_buffer = dc._allocator.alloc(u32, instr.transfer_length + 1) catch std.debug.panic("Failed to allocate command buffer", .{});
                defer dc._allocator.free(command_buffer);
                for (0..command_buffer.len) |i| {
                    command_buffer[i] = @byteSwap(data[idx + i + 1]);
                }
                std.debug.print("Return addr {X}\n", .{return_addr});
                const transferred_words = dpp_send(dc.ram[(return_addr & 0x03FFFFFF)..].ptr, command_buffer.ptr, @intCast(command_buffer.len));
                total_transferred_words += transferred_words;
                var ram_u32 = @as([*]align(1) u32, @ptrCast(dc.ram[(return_addr & 0x03FFFFFF)..].ptr));
                for (0..transferred_words) |i| {
                    ram_u32[i] = @byteSwap(ram_u32[i]);
                }
                if (transferred_words > 0) {
                    const returned_command: CommandWord = @bitCast(ram_u32[0]);
                    std.debug.print("Returned command: {f}\n", .{returned_command});
                }
                idx += instr.transfer_length + 2;
                std.debug.print("-----\n", .{});
            },
            .NOP, .RESET => {},
            else => {},
        }

        if (instr.end_flag == 1)
            break;
    }
    return 4 * (idx + total_transferred_words);
}
