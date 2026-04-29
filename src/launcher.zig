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
    _,
};

fn deecy_syscall(d: *Deecy, cpu: *Dreamcast.SH4, instr: Dreamcast.SH4Module.Instr) void {
    _ = instr;
    const command: Command = @enumFromInt(cpu.R(4).*);
    switch (command) {
        .GetGameList => {
            get_game_list(d, cpu) catch {
                cpu.R(0).* = 0xFFFFFFFF;
                return;
            };
        },
        .LaunchGame => {
            const game_index = cpu.R(5).*;
            if (game_index >= d.ui.disc_files.items.len) {
                cpu.R(0).* = 0xFFFFFFFF;
                return;
            }
            selected_game = game_index;
            d._on_stop_request = launch_selected_game;
            d._stop_request = true;
        },
        _ => {
            log.err("Invalid command: {t}", .{command});
            cpu.R(0).* = 0xFFFFFFFF;
        },
    }
}

fn get_game_list(d: *Deecy, cpu: *Dreamcast.SH4) !void {
    const dest = cpu.R(5).* & 0x00FFFFFF;
    const max = cpu.R(6).*;
    const offset = cpu.R(7).*;
    log.info("GetGameList(dest={X:0>8}, max={d}, offset={d})", .{ dest, max, offset });

    try d.ui.refresh_games();

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
