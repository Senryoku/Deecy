const std = @import("std");

const Dreamcast = @import("../dreamcast.zig").Dreamcast;
const Maple = @import("../maple.zig");
const Instruction = Maple.Instruction;
const CommandWord = Maple.CommandWord;

const log = std.log.scoped(.dpp);

const CommandResult = enum(i16) {
    Success = 0x0A,
    Attention = 0x0B,
    Failure = 0x0F,
    Invalid = 0xFE,
    SendFailure = -1,
    Timeout = -2,
    Disconnect = -3,
    _,
};

extern fn dpp_send(dest: [*]u8, data: [*]u32, len: u32) callconv(.c) extern struct { result: CommandResult, words_transferred: u32 };
extern fn dpp_send_async(dest: [*]u8, data: [*]u32, len: u32) callconv(.c) void;
extern fn ddp_wait_all_async_complete() callconv(.c) bool;
extern fn ddp_async_reset() callconv(.c) void;
extern fn ddp_async_get_transferred_words() callconv(.c) u32;

pub fn transfer(dc: *Dreamcast, data: [*]u32) u32 {
    var total_transferred_words: u32 = 0;
    var idx: u32 = 0;

    const AsyncTest = true;

    if (AsyncTest) {
        ddp_async_reset();
    }

    while (idx < 1024) {
        const instr: Instruction = @bitCast(data[idx]);
        idx += 1;

        switch (instr.pattern) {
            .Normal => {
                const return_addr = data[idx];
                const command: CommandWord = @bitCast(data[idx + 1]);
                const function_type = data[idx + 2];
                log.debug("Dest: {X:0>8}, Command: {f}, Function: {f}", .{ return_addr, command, @as(Maple.FunctionCodesMask, @bitCast(function_type)) });

                var command_buffer = dc._allocator.alloc(u32, instr.transfer_length + 1) catch std.debug.panic("Failed to allocate command buffer", .{});
                defer dc._allocator.free(command_buffer);
                for (0..command_buffer.len) |i| {
                    command_buffer[i] = @byteSwap(data[idx + i + 1]);
                }
                if (!AsyncTest) {
                    const r = dpp_send(dc.ram[(return_addr & 0x03FFFFFF)..].ptr, command_buffer.ptr, @intCast(command_buffer.len));
                    if (r.result != .Success) {
                        log.err("dpp_send failed: {t}", .{r.result});
                        if (r.words_transferred == 0) {
                            dc.cpu.write_physical(u32, return_addr, 0xFFFFFFFF); // "No connection"
                        }
                    }
                    total_transferred_words += r.words_transferred;
                    var ram_u32 = @as([*]align(1) u32, @ptrCast(dc.ram[(return_addr & 0x03FFFFFF)..].ptr));
                    for (0..r.words_transferred) |i| {
                        ram_u32[i] = @byteSwap(ram_u32[i]);
                    }
                    if (r.words_transferred > 0) {
                        const returned_command: CommandWord = @bitCast(ram_u32[0]);
                        log.debug("  Returned command: {f}\n", .{returned_command});
                    }
                } else {
                    dpp_send_async(dc.ram[(return_addr & 0x03FFFFFF)..].ptr, command_buffer.ptr, @intCast(command_buffer.len));
                }
                idx += instr.transfer_length + 2;
            },
            .NOP, .RESET => {},
            else => {},
        }

        if (instr.end_flag == 1)
            break;
    }

    if (AsyncTest) {
        if (!ddp_wait_all_async_complete()) {
            log.err("ddp_wait_all_async_complete timedout", .{});
        }
        total_transferred_words += ddp_async_get_transferred_words();
    }

    return 4 * (idx + total_transferred_words);
}
