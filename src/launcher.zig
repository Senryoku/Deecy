//! Fake syscall to allow program to communicate with the emulator

pub fn init(d: *Deecy) void {
    Dreamcast.SH4Module.instructions.EmulatorSyscall.set(@ptrCast(&deecy_syscall), d);
}

pub fn deinit() void {
    Dreamcast.SH4Module.instructions.EmulatorSyscall.clear();
}

const Command = enum(u32) {
    GetGameList = 0x01,
    LaunchGame = 0x02,
    RefreshGameList = 0x03,
    _,
};

fn deecy_syscall(d: *Deecy, cpu: *Dreamcast.SH4, instr: Dreamcast.SH4Module.Instr) void {
    _ = instr;
    const command: Command = @enumFromInt(cpu.R(4).*);
    handle_syscall(d, cpu, command) catch |err| {
        log.err("Error handling syscall '{t}': {t}", .{ command, err });
        cpu.R(0).* = 0xFFFFFFFF;
    };
}

fn handle_syscall(d: *Deecy, cpu: *Dreamcast.SH4, command: Command) !void {
    switch (command) {
        .GetGameList => try get_game_list(d, cpu),
        .LaunchGame => try launch_game(d, cpu),
        .RefreshGameList => try refresh_game_list(d, cpu),
        _ => return error.InvalidCommand,
    }
}

fn get_game_list(d: *Deecy, cpu: *Dreamcast.SH4) !void {
    const dest = cpu.R(5).* & 0x00FFFFFF;
    const max = cpu.R(6).*;
    const offset = cpu.R(7).*;
    log.info("GetGameList(dest={X:0>8}, max={d}, offset={d})", .{ dest, max, offset });

    d.wait_async_jobs();

    try d.ui.disc_files_mutex.lock(d.io);
    defer d.ui.disc_files_mutex.unlock(d.io);

    @memset(d.dc.ram[dest..][0 .. 256 * max], 0);

    const count = @min(d.ui.disc_files.items.len, max);

    for (d.ui.disc_files.items[0..count], 0..) |item, idx| {
        const len = @min(item.name.len, 255);
        @memcpy(d.dc.ram[dest + idx * 256 ..][0..len], item.name[0..len]);
    }
    cpu.R(0).* = count;
}

fn launch_game(d: *Deecy, cpu: *Dreamcast.SH4) !void {
    const game_index = cpu.R(5).*;
    if (game_index >= d.ui.disc_files.items.len) return error.InvalidParameter;
    selected_game = game_index;
    d._on_stop_request = launch_selected_game;
    d._stop_request = true;
    cpu.R(0).* = 0;
}

fn refresh_game_list(d: *Deecy, cpu: *Dreamcast.SH4) !void {
    log.info("RefreshGameList()", .{});
    try d.launch_async(Deecy.UI.refresh_games, .{d.ui});
    cpu.R(0).* = 0;
}

var selected_game: u32 = 0;
fn launch_selected_game(d: *Deecy) void {
    d._on_stop_request = null;
    d.load_and_start(d.ui.disc_files.items[selected_game].path) catch |err| {
        log.err("Failed to launch game: {t}", .{err});
    };
}

const std = @import("std");
const log = std.log.scoped(.launcher);

const Dreamcast = @import("dreamcast");
const Deecy = @import("./deecy.zig");
