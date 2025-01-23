const std = @import("std");
const termcolor = @import("termcolor");
const Dreamcast = @import("dreamcast.zig").Dreamcast;
const GDROM = @import("gdrom.zig");

const gdrom_hle_log = std.log.scoped(.gdrom_hle);

// GDROM HLE - Used by the BIOS syscalls

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

command: GDROMCommand = @enumFromInt(0),
params: [4]u32 = .{0} ** 4,
result: [4]u32 = .{0} ** 4,

_next_command_id: u32 = 1,
_current_command_id: u32 = 0,

pub fn reset(self: *@This()) void {
    self.command = @enumFromInt(0);
    @memset(&self.params, 0);
    @memset(&self.result, 0);
    self._next_command_id = 1;
    self._current_command_id = 0;
}

pub fn send_command(dc: *Dreamcast, command_code: u32, params: [4]u32) u32 {
    if (dc.gdrom.state != .Standby) return 0;

    dc.gdrom_hle._current_command_id = dc.gdrom_hle._next_command_id;
    dc.gdrom_hle._next_command_id +%= 1;
    if (dc.gdrom_hle._next_command_id == 0) dc.gdrom_hle._next_command_id = 1;

    dc.gdrom.state = .Busy;
    dc.gdrom_hle.command = @enumFromInt(command_code);
    dc.gdrom_hle.params = params;
    @memset(&dc.gdrom_hle.result, 0);

    return dc.gdrom_hle._current_command_id;
}

pub fn mainloop(dc: *Dreamcast) void {
    if (dc.gdrom.state != .Busy) {
        gdrom_hle_log.debug("  GDROM Mainloop - No command queued", .{});
        return;
    }

    gdrom_hle_log.info("  GDROM Mainloop - {s}", .{std.enums.tagName(GDROMCommand, dc.gdrom_hle.command) orelse "Unknown"});

    switch (dc.gdrom_hle.command) {
        .DMARead, .PIORead => {
            const lba = dc.gdrom_hle.params[0];
            const size = dc.gdrom_hle.params[1];
            const dest = dc.gdrom_hle.params[2] & 0x1FFFFFFF;

            gdrom_hle_log.info("    GDROM {s} sector={d} size={d} destination=0x{X:0>8}", .{ @tagName(dc.gdrom_hle.command), lba, size, dest });
            const read = dc.gdrom.disc.?.load_sectors(lba, size, @as([*]u8, @ptrCast(dc.cpu._get_memory(dest)))[0 .. 2352 * size]);

            dc.schedule_interrupt(.{ .EoD_GDROM = 1 }, 100_000 * size);
            dc.schedule_external_interrupt(.{ .GDRom = 1 }, 100_000 * size);

            dc.gdrom_hle.result = .{ 0, 0, read, 0 };

            dc.gdrom.state = .Standby;
        },
        .Init => {
            gdrom_hle_log.warn("    GDROM Command Init : TODO (Reset some stuff?)", .{});
            dc.gdrom.state = .Standby;
        },
        .GetVersion => {
            const dest = dc.gdrom_hle.params[0];
            const version = "GDC Version 1.10 1999-03-31";
            for (0..version.len) |i|
                dc.cpu.write8(@intCast(dest + i), version[i]);
            dc.cpu.write8(@intCast(dest + version.len), 0x2);
            dc.gdrom.state = .Standby;
        },
        .ReqMode => {
            const dest = dc.gdrom_hle.params[0];
            gdrom_hle_log.info("    GDROM ReqMode  dest=0x{X:0>8}", .{dest});
            dc.cpu.write32(dest + 0, 0); // Speed
            dc.cpu.write32(dest + 4, 0x00B4); // Standby
            dc.cpu.write32(dest + 8, 0x19); // Read Flags
            dc.cpu.write32(dest + 12, 0x08); // Read retry
            dc.gdrom_hle.result = .{ 0, 0, 0xA, 0 };
            dc.gdrom.state = .Standby;
        },
        .SetMode => {
            gdrom_hle_log.warn("    GDROM SetMode: TODO", .{});
            dc.gdrom.state = .Standby;
        },
        .GetTOC2 => {
            const area = dc.gdrom_hle.params[0];
            const dest = dc.gdrom_hle.params[1];

            gdrom_hle_log.info("    GDROM GetTOC2: area={d} dest=0x{X:0>8}", .{ area, dest });
            for (0..dc.gdrom_hle.params.len) |i|
                gdrom_hle_log.debug("      {d}  {X:0>8}", .{ i, dc.gdrom_hle.params[i] });

            const dest_slice = @as([*]u8, @ptrCast(dc.*.cpu._get_memory(dest & 0x1FFFFFFF)))[0..408];
            const bytes_written = dc.gdrom.write_toc(dest_slice, if (area == 1) .DoubleDensity else .SingleDensity);
            dc.gdrom_hle.result = .{ 0, 0, bytes_written, 0 };

            dc.gdrom.state = .Standby;
        },
        .GetSCD => {
            gdrom_hle_log.warn(termcolor.yellow("    Unimplemented GDROM command GetSCD"), .{});
            for (0..dc.gdrom_hle.params.len) |i|
                gdrom_hle_log.debug("      {d}  {X:0>8}", .{ i, dc.gdrom_hle.params[i] });

            const dest = dc.gdrom_hle.params[2];
            const len = dc.gdrom_hle.params[1];
            for (0..len) |i|
                dc.cpu.write32(@intCast(dest + 4 * i), 0);

            dc.gdrom.state = .Standby;
        },
        else => {
            gdrom_hle_log.warn(termcolor.yellow("    Unhandled GDROM command {any}"), .{dc.gdrom_hle.command});
            for (0..dc.gdrom_hle.params.len) |i|
                gdrom_hle_log.debug("      {d}  {X:0>8}", .{ i, dc.gdrom_hle.params[i] });
            dc.gdrom.state = .Standby;
        },
    }
}

pub fn check_command(dc: *Dreamcast, cmd_id: u32) u32 {
    if (cmd_id != dc.gdrom_hle._current_command_id) {
        @memset(&dc.gdrom_hle.result, 0);
        dc.gdrom_hle.result[0] = 0x5;
        return 0; // no such request active
    }
    if (dc.gdrom.state != .Standby) return 1; // request is still being processed

    // request has completed
    dc.gdrom_hle._current_command_id = 0;
    return 2;
}

pub fn serialize(self: *@This(), writer: anytype) !usize {
    var bytes: usize = 0;
    bytes += try writer.write(std.mem.asBytes(&self.command));
    bytes += try writer.write(std.mem.sliceAsBytes(self.params));
    bytes += try writer.write(std.mem.sliceAsBytes(self.result));
    bytes += try writer.write(std.mem.asBytes(&self._next_command_id));
    bytes += try writer.write(std.mem.asBytes(&self._current_command_id));
    return bytes;
}

pub fn deserialize(self: *@This(), reader: anytype) !usize {
    var bytes: usize = 0;
    bytes += try reader.read(std.mem.asBytes(&self.command));
    bytes += try reader.read(std.mem.sliceAsBytes(self.params));
    bytes += try reader.read(std.mem.sliceAsBytes(self.result));
    bytes += try reader.read(std.mem.asBytes(&self._next_command_id));
    bytes += try reader.read(std.mem.asBytes(&self._current_command_id));
    return bytes;
}
