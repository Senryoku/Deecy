const std = @import("std");

const GDI = @import("gdi.zig").GDI;
const SH4 = @import("sh4.zig").SH4;

pub const GDROMStatus = enum(u32) {
    Busy = 0,
    Paused = 1,
    Standby = 2,
    Playing = 3,
    Seeking = 4,
    Scanning = 5,
    Open = 6,
    Empty = 7,
};

pub const GDROMCommand = enum(u32) {
    PIORead = 16,
    DMARead = 17,
    GetTOC = 18,
    GetTOC2 = 19,
    Play = 20,
    Play2 = 21,
    Pause = 22,
    Release = 23,
    Init = 24,
    Seek = 27,
    Read = 28,
    Stop = 33,
    GetSCD = 34,
    GetSession = 35,
    GetVersion = 40,
    _,
};

pub const GDROM = struct {
    status: GDROMStatus = GDROMStatus.Standby,

    disk: GDI = .{},

    command: u32 = undefined,
    params: [4]u32 = undefined,
    result: [4]u32 = undefined,

    _current_command_id: u32 = 1,

    pub fn send_command(self: *@This(), command_code: u32, params: [4]u32) u32 {
        if (self.status != GDROMStatus.Standby) return 0;

        self._current_command_id +%= 1;
        if (self._current_command_id == 0) self._current_command_id = 1;

        self.status = GDROMStatus.Busy;
        self.command = command_code;
        self.params = params;

        return self._current_command_id;
    }

    pub fn mainloop(self: *@This(), cpu: *SH4) void {
        std.debug.print("  TODO: GDROM MAINLOOP {d}\n", .{self.command});

        if (self.status != GDROMStatus.Busy) return;

        switch (@as(GDROMCommand, @enumFromInt(self.command))) {
            GDROMCommand.DMARead => {
                const lba = self.params[0];
                const size = self.params[1];
                const dest = self.params[2];

                std.debug.print("  DMARead {d} {d} {X:0>8}\n", .{ lba, size, dest });
                const read = self.disk.load_sectors(lba, size, @as([*]u8, @ptrCast(cpu._get_memory(dest)))[0 .. 2048 * size]);

                self.result[2] = read;
                self.result[3] = 0;

                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.Init => {
                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.GetVersion => {
                const dest = self.params[0];
                const version = "GDC Version 1.10 1999-03-31";
                for (0..version.len) |i| {
                    cpu.write8(@intCast(dest + i), version[i]);
                }
                cpu.write8(@intCast(dest + version.len), 0x2);
                self.status = GDROMStatus.Standby;
            },
            else => {
                std.debug.print("  Unhandled GDROM command {X:0>8} {s}\n", .{ self.command, @tagName(@as(GDROMCommand, @enumFromInt(self.command))) });
            },
        }
    }

    pub fn check_command(self: @This(), cmd_id: u32) u32 {
        if (cmd_id != self._current_command_id) return 0; // no such request active
        if (self.status != GDROMStatus.Standby) return 1; // request is still being processed
        return 2; // request has completed
    }
};
