//! HLE Re-Implementations of syscalls normally provided by the Boot ROM

const std = @import("std");
const log = std.log.scoped(.syscall);

const DreamcastModule = @import("dreamcast.zig");
const Dreamcast = DreamcastModule.Dreamcast;
const HardwareRegisters = DreamcastModule.HardwareRegisters;
const HardwareRegister = HardwareRegisters.HardwareRegister;

const gdrom_hle = @import("gdrom_hle.zig");

inline fn return_from_syscall(dc: *Dreamcast) void {
    // Syscall are called using JSR, simulate a RTS/N (return from subroutine, without delay slot)
    dc.cpu.pc = dc.cpu.pr - 2;
}

pub fn syscall(dc: *Dreamcast) void {
    log.err("Unimplemented SYSCALL: 0x{X:0>8} = 0b{b:0>16}", .{ dc.cpu.pc, dc.cpu.read_physical(u16, dc.cpu.pc) });
    for (0..15) |i|
        log.err("  R{d}: {x}", .{ i, dc.cpu.R(@intCast(i)).* });
    @panic("Unimplemented SYSCALL");
}

fn unhandled_function(comptime syscall_name: []const u8, dc: *Dreamcast, function: u32) void {
    log.err("Syscall " ++ syscall_name ++ " called with unhandled function {d}.", .{function});
    dc.cpu.R(0).* = 0xFFFFFFFF;
}

pub fn syscall_sysinfo(dc: *Dreamcast) void {
    const func: enum(u32) { Init = 0, GetIcon = 2, GetId = 3, _ } = @enumFromInt(dc.cpu.R(7).*);
    switch (func) {
        .Init => {
            // Prepares the other two SYSINFO calls for use by copying the relevant data from the system flashrom into 8C000068-8C00007F. Always call this function before using the other two calls.
            @memcpy(dc.ram[0x00000068 .. 0x00000068 + 8], dc.flash.data[0x1A056 .. 0x1A056 + 8]);
            @memcpy(dc.ram[0x00000068 + 8 .. 0x00000068 + 8 + 6], dc.flash.data[0x1A000 .. 0x1A000 + 6]);
            dc.cpu.R(0).* = 0;
        },
        .GetId => {
            // Query the unique 64 bit ID number of this Dreamcast. SYSINFO_INIT must have been called first.
            // Args: none
            // Returns: A pointer to where the ID is stored as 8 contiguous bytes
            dc.cpu.R(0).* = 0x8C000068;
        },
        else => unhandled_function("SYSINFO", dc, dc.cpu.R(7).*),
    }

    return_from_syscall(dc);
}

pub fn syscall_romfont(dc: *Dreamcast) void {
    const func: enum(u32) { Address = 0, Lock = 1, Unlock = 2, _ } = @enumFromInt(dc.cpu.R(1).*);
    switch (func) {
        .Address => dc.cpu.R(0).* = 0xA0100020, // NOTE: Just an informed guess from stepping through the boot ROM.
        .Lock => dc.cpu.R(0).* = 0, // Returns: 0 if you got the mutex (unlock it with ROMFONT_UNLOCK when you're done), -1 if it was already taken by someone else.
        .Unlock => {},
        else => unhandled_function("ROMFONT", dc, dc.cpu.R(1).*),
    }
    return_from_syscall(dc);
}

pub fn syscall_flashrom(dc: *Dreamcast) void {
    switch (dc.cpu.R(7).*) {
        0 => {
            log.info("FLASHROM_INFO  (R4={X:0>8}, R5={X:0>8})", .{ dc.cpu.R(4).*, dc.cpu.R(5).* });
            // Queries the extent of a single partition in the system flashrom.
            // Args: r4 = partition number (0-4)
            //       r5 = pointer to two 32 bit integers to receive the result. The first will be the offset of the partition start, in bytes from the start of the flashrom. The second will be the size of the partition, in bytes.
            // Returns: zero if successful, -1 if no such partition exists
            switch (dc.cpu.R(4).*) {
                0, 1, 2, 3, 4 => |partition_number| {
                    const dest = dc.cpu.R(5).*;
                    const partition = DreamcastModule.Flash.get_dc_partition(@enumFromInt(partition_number));
                    dc.cpu.write_physical(u32, dest, partition.offset);
                    dc.cpu.write_physical(u32, dest + 4, partition.size);
                    dc.cpu.R(0).* = 0;
                },
                else => dc.cpu.R(0).* = 0xFFFFFFFF,
            }
        },
        1 => {
            log.info("FLASHROM_READ (R4={X:0>8}, R5={X:0>8}, R6={X:0>8})", .{ dc.cpu.R(4).*, dc.cpu.R(5).*, dc.cpu.R(6).* });
            // Read data from the system flashrom.
            // Returns: number of read bytes if successful, -1 if read failed
            const start = dc.cpu.R(4).*; // Read start position, in bytes from the start of the flashrom
            const dest = (dc.cpu.R(5).* & 0x1FFFFFFF) - 0x0C000000; // Pointer to destination buffer
            const len = dc.cpu.R(6).*; // Number of bytes to read
            @memcpy(dc.ram[dest .. dest + len], dc.flash.data[start .. start + len]);
            dc.cpu.R(0).* = len;
        },
        2 => {
            log.info("FLASHROM_WRITE (R4={X:0>8}, R5={X:0>8}, R6={X:0>8})", .{ dc.cpu.R(4).*, dc.cpu.R(5).*, dc.cpu.R(6).* });
            // Write data to the system flashrom.
            // Returns: number of written bytes if successful, -1 if write failed
            const start = dc.cpu.R(4).*; // Write start position, in bytes from the start of the flashrom
            const source = (dc.cpu.R(5).* & 0x1FFFFFFF) - 0x0C000000; // Pointer to source buffer
            const len = dc.cpu.R(6).*; // Number of bytes to write
            @memcpy(dc.flash.data[start .. start + len], dc.ram[source .. source + len]);
            dc.cpu.R(0).* = len;
        },
        3 => {
            log.warn("Unimplemented FLASHROM_DELETE (R4={X:0>8})", .{dc.cpu.R(4).*});
            // FlashROM Delete - Return a flashrom partition to all ones, so that it may be rewritten. Danger: ALL data in the entire partition will be lost.
            //     r4 = offset of the start of the partition you want to delete, in bytes from the start of the flashrom
            // Returns: zero if successful, -1 if delete failed
            dc.cpu.R(0).* = 0xFFFFFFFF;
        },
        else => unhandled_function("FLASHROM", dc, dc.cpu.R(7).*),
    }
    return_from_syscall(dc);
}

const GDFunction = enum(u32) {
    ReqCmd = 0,
    GetCmdStat = 1,
    ExecServer = 2,
    InitSystem = 3,
    GetDrvStat = 4,
    G1DmaEnd = 5,
    ReqDmaTrans = 6,
    CheckDmaTrans = 7,
    ReadAbort = 8,
    Reset = 9,
    ChangeDataType = 10,
    _,
};

/// Called when R6 = -1
const GDBootFunction = enum(u32) {
    ReInitEntry = 0,
    AddDesc = 1,
    _,
};

pub fn syscall_gdrom(dc: *Dreamcast) void {
    if (dc.cpu.R(6).* != 0) log.warn("GD syscall with non-zero R6: R6={d}", .{dc.cpu.R(6).*});

    const function: GDFunction = @enumFromInt(dc.cpu.R(7).*);
    switch (function) {
        .ReqCmd => {
            // GDROM_SEND_COMMAND
            // Enqueue a command for the GDROM subsystem to execute.
            // Args: r4 = command code
            //       r5 = pointer to parameter block for the command, can be NULL if the command does not take parameters
            // Returns: a request id (>=0) if successful, negative error code if failed
            log.info("  GDROM_SEND_COMMAND R4={d} R5={X:0>8}", .{ dc.cpu.R(4).*, dc.cpu.R(5).* });

            var params: [4]u32 = @splat(0);
            const params_addr = dc.cpu.R(5).*;
            if (params_addr != 0) {
                for (0..4) |i| {
                    params[i] = dc.cpu.read_physical(u32, @intCast(params_addr + 4 * i));
                }
            }

            dc.cpu.R(0).* = gdrom_hle.send_command(dc, dc.cpu.R(4).*, params);
        },
        .GetCmdStat => {
            // GDROM_CHECK_COMMAND
            // Check if an enqueued command has completed.
            // Args: r4 = request id
            //       r5 = pointer to four 32 bit integers to receive extended status information. The first is a generic error code.
            dc.cpu.R(0).* = @intFromEnum(gdrom_hle.check_command(dc, dc.cpu.R(4).*));
            for (0..4) |i|
                dc.cpu.write_physical(u32, @intCast(dc.cpu.R(5).* + 4 * i), dc.gdrom_hle.result[i]);
            log.info("GDROM_CHECK_COMMAND R4={d} R5={X:0>8} | Ret : {X:0>8}, Result: {X:0>8} {X:0>8} {X:0>8} {X:0>8}", .{
                dc.cpu.R(4).*,
                dc.cpu.R(5).*,
                dc.cpu.R(0).*,
                dc.gdrom_hle.result[0],
                dc.gdrom_hle.result[1],
                dc.gdrom_hle.result[2],
                dc.gdrom_hle.result[3],
            });
        },
        .ExecServer => {
            // GDROM_MAINLOOP
            // In order for enqueued commands to get processed, this function must be called a few times.
            // It can be called from a periodic interrupt, or just keep calling it manually until GDROM_CHECK_COMMAND says that your command has stopped processing.
            // Args: none
            // Returns: no return value
            gdrom_hle.mainloop(dc);
        },
        .InitSystem => {
            // GDROM_INIT
            log.debug("GDROM_INIT", .{});
            dc.gdrom.reset();
        },
        .GetDrvStat => {
            // GDROM_CHECK_DRIVE
            // Checks the general condition of the drive.
            // Args: r4 = pointer to two 32 bit integers, to receive the drive status. The first is the current drive status, the second is the type of disc inserted (if any).
            // Returns: zero if successful, nonzero if failure
            log.debug("GDROM_CHECK_DRIVE", .{});
            dc.cpu.write_physical(u32, dc.cpu.R(4).*, @intFromEnum(dc.gdrom.state));
            const disc_type: u8 = if (dc.gdrom.disc) |disc| @intFromEnum(disc.get_format()) else 0;
            dc.cpu.write_physical(u32, dc.cpu.R(4).* + 4, disc_type << 4);
            dc.cpu.R(0).* = 0;
        },
        .G1DmaEnd => {
            // DMA END?
            // According to the disassembly, maybe?:
            //   R4: Callback
            //   R5: Callback parameter
            log.info("GDROM_DMA_END (R7={d}) R4={X:0>8} R5={X:0>8}", .{
                dc.cpu.R(7).*,
                dc.cpu.R(4).*,
                dc.cpu.R(5).*,
            });
            dc.cpu.write_physical(u32, @intFromEnum(HardwareRegister.SB_ISTNRM), @bitCast(HardwareRegisters.SB_ISTNRM{ .EoD_GDROM = 1 })); // Clear interrupt
            if (dc.cpu.R(4).* != 0)
                log.warn("GDROM_DMA_END: R4={X:0>8}, is it a callback?", .{dc.cpu.R(4).*});
            // No return value, I think.
        },
        .Reset => {
            // GDROM_RESET
            log.warn("GDROM_RESET (R7={d}) : Not implemented!", .{dc.cpu.R(7).*});
        },
        .ChangeDataType => {
            // GDROM_SECTOR_MODE
            log.warn("GDROM_SECTOR_MODE  (R7={d}) : Not implemented!", .{dc.cpu.R(7).*});
            if (dc.cpu.read_physical(u32, dc.cpu.R(4).*) == 0) { // Get/Set, if 0 the mode will be set, if 1 it will be queried.
                const mode = dc.cpu.read_physical(u32, dc.cpu.R(4).* + 8) == 0;
                const sector_size_in_bytes = dc.cpu.read_physical(u32, dc.cpu.R(4).* + 12) == 0;
                _ = sector_size_in_bytes;
                _ = mode;
            } else {
                dc.cpu.write_physical(u32, dc.cpu.R(4).* + 4, 0x2000); // Constant
                dc.cpu.write_physical(u32, dc.cpu.R(4).* + 8, 0x0800); // Mode, 1024 = mode 1, 2048 = mode 2, 0 = auto detect
                dc.cpu.write_physical(u32, dc.cpu.R(4).* + 12, 0x0800); // Sector size in bytes (normally 2048)
            }
            dc.cpu.R(0).* = 0;
        },
        else => unhandled_function("GDROM", dc, dc.cpu.R(7).*),
    }
    return_from_syscall(dc);
}

pub fn syscall_misc(dc: *Dreamcast) void {
    log.debug("syscall_misc: R4={d} R5={X:0>8} R6={X:0>8} R7={X:0>8} ", .{ dc.cpu.R(4).*, dc.cpu.R(5).*, dc.cpu.R(6).*, dc.cpu.R(7).* });
    switch (dc.cpu.R(4).*) {
        0 => {
            // Normal Init
            // Looking at the disassembly, it does a bunch of stuff related to the GDROM, security checks and status check, but we probably don't care about it.
            dc.cpu.sr.imask = 0;
            dc.cpu.write_physical(u32, @intFromEnum(HardwareRegister.SB_IML2NRM), 0);
            dc.gpu._get_register(u32, .VO_BORDER_COL).* = 0x00C0BEBC; // Set border color to light grey
        },
        1 => {
            // Return to BIOS?
            log.err("syscall_misc: Return to BIOS? R4={d} R5={X:0>8} R6={X:0>8} R7={X:0>8} ", .{ dc.cpu.R(4).*, dc.cpu.R(5).*, dc.cpu.R(6).*, dc.cpu.R(7).* });
            log.err("                              PC={X:0>8} PR={X:0>8} ", .{ dc.cpu.pc, dc.cpu.pr });
            @panic("Return to BIOS?");
        },
        else => unhandled_function("MISC", dc, dc.cpu.R(4).*),
    }
    return_from_syscall(dc);
}
