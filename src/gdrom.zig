const std = @import("std");
const termcolor = @import("termcolor.zig");

const GDI = @import("gdi.zig").GDI;
const SH4 = @import("sh4.zig").SH4;

const MemoryRegisters = @import("MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;

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
    ReqMode = 30,
    SetMode = 31,
    Stop = 33,
    GetSCD = 34,
    GetSession = 35,
    GetVersion = 40,
    _,
};

pub const GDROM = struct {
    disk: GDI = .{},

    status: GDROMStatus = GDROMStatus.Standby,
    command: GDROMCommand = undefined,
    params: [4]u32 = undefined,
    result: [4]u32 = undefined,

    _next_command_id: u32 = 1,
    _current_command_id: u32 = 0,

    pub fn init(self: *@This()) void {
        // No idea if this is needed.
        self.status = GDROMStatus.Standby;
        self.command = undefined;
        @memset(&self.params, 0);
        @memset(&self.result, 0);
        self._next_command_id = 1;
    }

    pub fn send_command(self: *@This(), command_code: u32, params: [4]u32) u32 {
        if (self.status != GDROMStatus.Standby) return 0;

        self._current_command_id = self._next_command_id;
        self._next_command_id +%= 1;
        if (self._next_command_id == 0) self._next_command_id = 1;

        self.status = GDROMStatus.Busy;
        self.command = @enumFromInt(command_code);
        self.params = params;
        @memset(&self.result, 0);

        return self._current_command_id;
    }

    pub fn mainloop(self: *@This(), cpu: *SH4) void {
        if (self.status != GDROMStatus.Busy) {
            std.debug.print("  GDROM Mainloop - No command queued\n", .{});
            return;
        }

        std.debug.print("  GDROM Mainloop - {s}\n", .{std.enums.tagName(GDROMCommand, self.command) orelse "Unknown"});

        switch (self.command) {
            GDROMCommand.DMARead, GDROMCommand.PIORead => {
                const lba = self.params[0];
                const size = self.params[1];
                const dest = self.params[2] & 0x1FFFFFFF;

                std.debug.print("    GDROM {s} sector={d} size={d} destination=0x{X:0>8}\n", .{ @tagName(self.command), lba, size, dest });
                const byte_size = 2048 * size;
                const read = self.disk.load_sectors(lba, byte_size, @as([*]u8, @ptrCast(cpu._get_memory(dest)))[0..byte_size]);

                cpu.raise_normal_interrupt(.{ .EoD_GDROM = 1 });
                cpu.raise_external_interrupt(.{ .GDRom = 1 });

                self.result = .{ 0, 0, read, 0 };

                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.Init => {
                std.debug.print("    GDROM Command Init : TODO (Reset some stuff?)\n", .{});
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
            GDROMCommand.ReqMode => {
                const dest = self.params[0];
                std.debug.print("    GDROM ReqMode  dest=0x{X:0>8}\n", .{dest});
                cpu.write32(dest + 0, 0); // Speed
                cpu.write32(dest + 4, 0x00B4); // Standby
                cpu.write32(dest + 8, 0x19); // Read Flags
                cpu.write32(dest + 12, 0x08); // Read retry
                self.result = .{ 0, 0, 0xA, 0 };
                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.SetMode => {
                std.debug.print("    GDROM SetMode: TODO\n", .{});
                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.GetTOC2 => {
                const area = self.params[0];
                const dest = self.params[1];
                std.debug.print(termcolor.yellow("    GDROM GetTOC2: area={d} dest=0x{X:0>8}, TODO!\n"), .{ area, dest });
                if (area == 1) {
                    // High Density Area, doesn't have a TOC? (What's that thing at 0x110 in track 3?)
                } else {}
                self.status = GDROMStatus.Standby;
            },
            else => {
                std.debug.print("    Unhandled GDROM command {X:0>8} {s}\n", .{ self.command, @tagName(self.command) });
            },
        }
    }

    pub fn check_command(self: *@This(), cmd_id: u32) u32 {
        if (cmd_id != self._current_command_id) {
            @memset(&self.result, 0);
            self.result[0] = 0x5;
            return 0; // no such request active
        }
        if (self.status != GDROMStatus.Standby) return 1; // request is still being processed

        // request has completed
        self._current_command_id = 0;
        return 2;
    }
};
