const std = @import("std");
const termcolor = @import("termcolor");
const file = @import("log_file.zig");

pub const Output = enum { None, Console, File, Both };

var output: Output = .Console;
var last_message: struct {
    message_level: std.log.Level,
    format: []const u8,
    args_hash: u64,
} = undefined;
var count: u32 = 0;
var buffer: [128]u8 = undefined;

pub fn deinit() void {
    file.close();
}

pub const get_path = file.get_path;

pub fn set_output(out: Output) void {
    if (out == output) return;
    file.close();
    output = out;
    if (output == .File or output == .Both) {
        file.open(std.heap.page_allocator) catch {
            output = .Console;
            std.log.err("Failed to open log file. That's akward...", .{});
        };
    }
}

pub fn log(
    comptime message_level: std.log.Level,
    comptime scope: @Type(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    if (output == .None) return;

    file.lock();
    defer file.unlock();

    const output_to_console = output == .Console or output == .Both;
    const output_to_file = output == .File or output == .Both;
    if (output_to_file) std.debug.assert(file.opened());

    const args_hash = std.hash.CityHash64.hash(std.mem.asBytes(&args));

    if (message_level == last_message.message_level and
        std.mem.eql(u8, format, last_message.format) and
        args_hash == last_message.args_hash)
    {
        count +|= 1;

        if (output_to_console) {
            const stderr = std.debug.lockStderrWriter(&buffer);
            defer std.debug.unlockStderrWriter();
            nosuspend stderr.print(termcolor.grey("\r  (...x{d})"), .{count}) catch return;
        }
        return;
    }

    if (count > 1) {
        if (output_to_console) {
            const stderr = std.debug.lockStderrWriter(&buffer);
            defer std.debug.unlockStderrWriter();
            nosuspend stderr.print("\n", .{}) catch return;
        }
        if (output_to_file) {
            file.writer.print("(...x{d})\n", .{count}) catch return;
            file.writer.flush() catch return;
        }
    }

    last_message = .{
        .message_level = message_level,
        .format = format,
        .args_hash = args_hash,
    };
    count = 1;

    const level_txt = comptime message_level.asText();
    const prefix2 = if (scope == .default) ": " else "(" ++ @tagName(scope) ++ "): ";
    if (output_to_console) {
        const stderr = std.debug.lockStderrWriter(&buffer);
        defer std.debug.unlockStderrWriter();
        nosuspend stderr.print(comptime switch (message_level) {
            inline .debug, .info => level_txt ++ prefix2,
            inline .warn => termcolor.yellow(level_txt ++ prefix2),
            inline .err => termcolor.red(level_txt ++ prefix2),
        } ++ format ++ "\n", args) catch return;
    }
    if (output_to_file) {
        file.writer.print(level_txt ++ prefix2 ++ format ++ "\n", args) catch return;
        file.writer.flush() catch return;
    }
}
