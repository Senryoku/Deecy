const std = @import("std");
const termcolor = @import("termcolor.zig");

const gdrom_log = std.log.scoped(.gdrom);

const GDI = @import("gdi.zig").GDI;
const SH4 = @import("sh4.zig").SH4;
const Dreamcast = @import("dreamcast.zig").Dreamcast;

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

    pub fn init() GDROM {
        var gdrom = GDROM{};
        gdrom.reinit();
        return gdrom;
    }

    pub fn reinit(self: *@This()) void {
        self.status = GDROMStatus.Standby;
        self.command = undefined;
        @memset(&self.params, 0);
        @memset(&self.result, 0);
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

    pub fn mainloop(self: *@This(), dc: *Dreamcast) void {
        if (self.status != GDROMStatus.Busy) {
            gdrom_log.debug("  GDROM Mainloop - No command queued", .{});
            return;
        }

        gdrom_log.info("  GDROM Mainloop - {s}", .{std.enums.tagName(GDROMCommand, self.command) orelse "Unknown"});

        switch (self.command) {
            GDROMCommand.DMARead, GDROMCommand.PIORead => {
                const lba = self.params[0];
                const size = self.params[1];
                const dest = self.params[2] & 0x1FFFFFFF;

                gdrom_log.info("    GDROM {s} sector={d} size={d} destination=0x{X:0>8}", .{ @tagName(self.command), lba, size, dest });
                const byte_size = 2048 * size;
                const read = self.disk.load_sectors(lba, byte_size, @as([*]u8, @ptrCast(dc.cpu._get_memory(dest)))[0..byte_size]);

                dc.raise_normal_interrupt(.{ .EoD_GDROM = 1 });
                dc.raise_external_interrupt(.{ .GDRom = 1 });

                self.result = .{ 0, 0, read, 0 };

                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.Init => {
                gdrom_log.warn("    GDROM Command Init : TODO (Reset some stuff?)", .{});
                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.GetVersion => {
                const dest = self.params[0];
                const version = "GDC Version 1.10 1999-03-31";
                for (0..version.len) |i| {
                    dc.cpu.write8(@intCast(dest + i), version[i]);
                }
                dc.cpu.write8(@intCast(dest + version.len), 0x2);
                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.ReqMode => {
                const dest = self.params[0];
                gdrom_log.info("    GDROM ReqMode  dest=0x{X:0>8}", .{dest});
                dc.cpu.write32(dest + 0, 0); // Speed
                dc.cpu.write32(dest + 4, 0x00B4); // Standby
                dc.cpu.write32(dest + 8, 0x19); // Read Flags
                dc.cpu.write32(dest + 12, 0x08); // Read retry
                self.result = .{ 0, 0, 0xA, 0 };
                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.SetMode => {
                gdrom_log.warn("    GDROM SetMode: TODO", .{});
                self.status = GDROMStatus.Standby;
            },
            GDROMCommand.GetTOC2 => {
                const area = self.params[0];
                const dest = self.params[1];
                gdrom_log.warn(termcolor.yellow("    GDROM GetTOC2: area={d} dest=0x{X:0>8}, TODO!"), .{ area, dest });
                if (area == 1) {
                    // High Density Area, doesn't have a TOC? (What's that thing at 0x110 in track 3?)
                } else {}
                self.status = GDROMStatus.Standby;
            },
            else => {
                gdrom_log.warn("    Unhandled GDROM command {X:0>8} {s}", .{ self.command, @tagName(self.command) });
                self.status = GDROMStatus.Standby;
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
