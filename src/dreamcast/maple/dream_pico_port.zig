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

pub fn send_command(dc: *Dreamcast, physical_port: u8, data: []const u32) u32 {
    const return_addr = data[0];
    var command: CommandWord = @bitCast(data[1]);
    // Redirect command to the physical port on the DreamPico
    const original_recipient = command.recipent_address;
    command.recipent_address &= 0x3F;
    command.recipent_address |= physical_port << 6;

    var command_buffer: [1024]u32 = undefined;
    command_buffer[0] = @bitCast(command);
    command_buffer[0] = @byteSwap(command_buffer[0]);
    for (1..data.len - 1) |i|
        command_buffer[i] = @byteSwap(data[i + 1]);

    const r = dpp_send(dc.ram[(return_addr & 0x03FFFFFF)..].ptr, &command_buffer, @intCast(data.len - 1));

    var ram_u32 = @as([*]align(1) u32, @ptrCast(dc.ram[(return_addr & 0x03FFFFFF)..].ptr));

    if (r.result != .Success) {
        log.err("dpp_send failed: {t}", .{r.result});
        if (r.words_transferred == 0) {
            ram_u32[0] = 0xFFFFFFFF; // "No connection"
            return 0;
        }
    }

    if (r.words_transferred > 0) {
        for (0..r.words_transferred) |i| ram_u32[i] = @byteSwap(ram_u32[i]);
        // Restore the original command target port
        @as(*align(1) CommandWord, @ptrCast(&ram_u32[0])).sender_address &= 0x3F;
        @as(*align(1) CommandWord, @ptrCast(&ram_u32[0])).sender_address |= original_recipient & 0xC0;
    }

    return 4 * r.words_transferred;
}
