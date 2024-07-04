const std = @import("std");
const termcolor = @import("termcolor");
const Dreamcast = @import("dreamcast.zig").Dreamcast;
const gdrom = @import("gdrom.zig");

const gdrom_hle_log = std.log.scoped(.gdrom_hle);

// GDROM HLE - Used by the BIOS syscalls

pub fn send_command(self: *gdrom.GDROM, command_code: u32, params: [4]u32) u32 {
    if (self.state != gdrom.GDROMStatus.Standby) return 0;

    self._current_command_id = self._next_command_id;
    self._next_command_id +%= 1;
    if (self._next_command_id == 0) self._next_command_id = 1;

    self.state = gdrom.GDROMStatus.Busy;
    self.hle_command = @enumFromInt(command_code);
    self.hle_params = params;
    @memset(&self.hle_result, 0);

    return self._current_command_id;
}

pub fn mainloop(self: *gdrom.GDROM, dc: *Dreamcast) void {
    if (self.state != gdrom.GDROMStatus.Busy) {
        gdrom_hle_log.debug("  GDROM Mainloop - No command queued", .{});
        return;
    }

    gdrom_hle_log.info("  GDROM Mainloop - {s}", .{std.enums.tagName(gdrom.GDROMCommand, self.hle_command) orelse "Unknown"});

    switch (self.hle_command) {
        .DMARead, .PIORead => {
            const lba = self.hle_params[0];
            const size = self.hle_params[1];
            const dest = self.hle_params[2] & 0x1FFFFFFF;

            gdrom_hle_log.info("    GDROM {s} sector={d} size={d} destination=0x{X:0>8}", .{ @tagName(self.hle_command), lba, size, dest });
            const read = self.disk.?.load_sectors(lba, size, @as([*]u8, @ptrCast(dc.cpu._get_memory(dest)))[0 .. 2352 * size]);

            dc.schedule_interrupt(.{ .EoD_GDROM = 1 }, 100_000 * size);
            dc.schedule_external_interrupt(.{ .GDRom = 1 }, 100_000 * size);

            self.hle_result = .{ 0, 0, read, 0 };

            self.state = .Standby;
        },
        .Init => {
            gdrom_hle_log.warn("    GDROM Command Init : TODO (Reset some stuff?)", .{});
            self.state = .Standby;
        },
        .GetVersion => {
            const dest = self.hle_params[0];
            const version = "GDC Version 1.10 1999-03-31";
            for (0..version.len) |i| {
                dc.cpu.write8(@intCast(dest + i), version[i]);
            }
            dc.cpu.write8(@intCast(dest + version.len), 0x2);
            self.state = .Standby;
        },
        .ReqMode => {
            const dest = self.hle_params[0];
            gdrom_hle_log.info("    GDROM ReqMode  dest=0x{X:0>8}", .{dest});
            dc.cpu.write32(dest + 0, 0); // Speed
            dc.cpu.write32(dest + 4, 0x00B4); // Standby
            dc.cpu.write32(dest + 8, 0x19); // Read Flags
            dc.cpu.write32(dest + 12, 0x08); // Read retry
            self.hle_result = .{ 0, 0, 0xA, 0 };
            self.state = .Standby;
        },
        .SetMode => {
            gdrom_hle_log.warn("    GDROM SetMode: TODO", .{});
            self.state = .Standby;
        },
        .GetTOC2 => {
            const area = self.hle_params[0];
            const dest = self.hle_params[1];

            gdrom_hle_log.info("    GDROM GetTOC2: area={d} dest=0x{X:0>8}", .{ area, dest });
            for (0..self.hle_params.len) |i| {
                gdrom_hle_log.debug("      {d}  {X:0>8}", .{ i, self.hle_params[i] });
            }

            const dest_slice = @as([*]u8, @ptrCast(dc.*.cpu._get_memory(dest & 0x1FFFFFFF)))[0..408];
            const bytes_written = self.write_toc(dest_slice, if (area == 1) .DoubleDensity else .SingleDensity);
            self.hle_result = .{ 0, 0, bytes_written, 0 };

            self.state = .Standby;
        },
        .GetSCD => {
            gdrom_hle_log.warn(termcolor.yellow("    Unimplemented GDROM command GetSCD"), .{});
            for (0..self.hle_params.len) |i| {
                gdrom_hle_log.debug("      {d}  {X:0>8}", .{ i, self.hle_params[i] });
            }

            const dest = self.hle_params[2];
            const len = self.hle_params[1];
            for (0..len) |i| {
                dc.cpu.write32(@intCast(dest + 4 * i), 0);
            }

            self.state = .Standby;
        },
        else => {
            gdrom_hle_log.warn(termcolor.yellow("    Unhandled GDROM command {any}"), .{self.hle_command});
            for (0..self.hle_params.len) |i| {
                gdrom_hle_log.debug("      {d}  {X:0>8}", .{ i, self.hle_params[i] });
            }
            self.state = .Standby;
        },
    }
}

pub fn check_command(self: *gdrom.GDROM, cmd_id: u32) u32 {
    if (cmd_id != self._current_command_id) {
        @memset(&self.hle_result, 0);
        self.hle_result[0] = 0x5;
        return 0; // no such request active
    }
    if (self.state != .Standby) return 1; // request is still being processed

    // request has completed
    self._current_command_id = 0;
    return 2;
}
