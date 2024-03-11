const std = @import("std");
const termcolor = @import("termcolor.zig");

const HardwareRegisters = @import("hardware_registers.zig");
const HardwareRegister = HardwareRegisters.HardwareRegister;

const Dreamcast = @import("dreamcast.zig").Dreamcast;

const syscall_log = std.log.scoped(.syscall_log);

// HLE Re-Implementations of syscalls normally provided by the Boot ROM

inline fn return_from_syscall(dc: *Dreamcast) void {
    // Syscall are called using JSR, simulate a RTS/N (return from subroutine, without delay slot)
    dc.cpu.pc = dc.cpu.pr - 2;
}

pub fn syscall(dc: *Dreamcast) void {
    syscall_log.err("Unimplemented SYSCALL: 0x{X:0>8} = 0b{b:0>16}", .{ dc.cpu.pc, dc.cpu.read16(dc.cpu.pc) });
    for (0..15) |i| {
        syscall_log.err("  R{d}: {x}", .{ i, dc.cpu.R(@intCast(i)).* });
    }
    @panic("Unimplemented SYSCALL");
}

pub fn syscall_sysinfo(dc: *Dreamcast) void {
    switch (dc.cpu.R(7).*) {
        0 => {
            // Prepares the other two SYSINFO calls for use by copying the relevant data from the system flashrom into 8C000068-8C00007F. Always call this function before using the other two calls.

            @memcpy(dc.ram[0x00000068 .. 0x00000068 + 8], dc.flash.data[0x1A056 .. 0x1A056 + 8]);
            @memcpy(dc.ram[0x00000068 + 8 .. 0x00000068 + 8 + 6], dc.flash.data[0x1A000 .. 0x1A000 + 6]);

            dc.cpu.R(0).* = 0;
        },
        3 => {
            // Query the unique 64 bit ID number of this Dreamcast. SYSINFO_INIT must have been called first.
            // Args: none
            // Returns: A pointer to where the ID is stored as 8 contiguous bytes
            dc.cpu.R(0).* = 0x8c000068;
        },
        else => {
            syscall_log.err("  syscall_sysinfo with unhandled R7: R7={d}", .{dc.cpu.R(7).*});
            @panic("syscall_sysinfo with unhandled R7");
        },
    }

    return_from_syscall(dc);
}

pub fn syscall_romfont(dc: *Dreamcast) void {
    switch (dc.cpu.R(1).*) {
        0 => {
            // ROMFONT_ADDRESS
            dc.cpu.R(0).* = 0xA0100020; // NOTE: Just an informed guess from stepping through the boot ROM.
        },
        1 => {
            // ROMFONT_LOCK - Nothing to do, I guess?
            dc.cpu.R(0).* = 0; // Returns: 0 if you got the mutex (unlock it with ROMFONT_UNLOCK when you're done), -1 if it was already taken by someone else
        },
        2 => {
            // ROMFONT_UNLOCK - Nothing to do, I guess?
        },
        else => {
            syscall_log.err("  syscall_romfont with unhandled R1: R1={d}", .{dc.cpu.R(1).*});
            @panic("syscall_romfont with unhandled R1");
        },
    }

    return_from_syscall(dc);
}

pub fn syscall_flashrom(dc: *Dreamcast) void {
    switch (dc.cpu.R(7).*) {
        0 => {
            // Queries the extent of a single partition in the system flashrom.
            // Args: r4 = partition number (0-4)
            //       r5 = pointer to two 32 bit integers to receive the result. The first will be the offset of the partition start, in bytes from the start of the flashrom. The second will be the size of the partition, in bytes.
            // Returns: zero if successful, -1 if no such partition exists

            dc.cpu.R(0).* = 0;
            const dest = dc.cpu.R(5).*;
            switch (dc.cpu.R(4).*) {
                0 => {
                    dc.cpu.write32(dest, 0x1A000);
                    dc.cpu.write32(dest + 4, 8 * 1024);
                },
                1 => {
                    dc.cpu.write32(dest, 0x18000);
                    dc.cpu.write32(dest + 4, 8 * 1024);
                },
                2 => {
                    dc.cpu.write32(dest, 0x1C000);
                    dc.cpu.write32(dest + 4, 16 * 1024);
                },
                3 => {
                    dc.cpu.write32(dest, 0x10000);
                    dc.cpu.write32(dest + 4, 32 * 1024);
                },
                4 => {
                    dc.cpu.write32(dest, 0x00000);
                    dc.cpu.write32(dest + 4, 64 * 1024);
                },
                else => {
                    dc.cpu.R(0).* = @bitCast(@as(i32, @intCast(-1)));
                },
            }
        },
        1 => {
            // Read data from the system flashrom.
            // Args: r4 = read start position, in bytes from the start of the flashrom
            //       r5 = pointer to destination buffer
            //       r6 = number of bytes to read
            // Returns: number of read bytes if successful, -1 if read failed

            const start = dc.cpu.R(4).*;
            const dest = (dc.cpu.R(5).* & 0x1FFFFFFF) - 0x0C000000;
            const len = dc.cpu.R(6).*;
            @memcpy(dc.ram[dest .. dest + len], dc.flash.data[start .. start + len]);
            dc.cpu.R(0).* = len;
        },
        2 => {
            // Write data to the system flashrom.
            //   r4 = write start position, in bytes from the start of the flashrom
            //   r5 = pointer to source buffer
            //   r6 = number of bytes to write
            // Returns: number of written bytes if successful, -1 if write failed
            const start = dc.cpu.R(4).*;
            const source = (dc.cpu.R(5).* & 0x1FFFFFFF) - 0x0C000000;
            const len = dc.cpu.R(6).*;
            @memcpy(dc.flash.data[start .. start + len], dc.ram[source .. source + len]);
            dc.cpu.R(0).* = len;
        },
        3 => {
            // FlashROM Delete - Return a flashrom partition to all ones, so that it may be rewritten. Danger: ALL data in the entire partition will be lost.
            //     r4 = offset of the start of the partition you want to delete, in bytes from the start of the flashrom
            // Returns: zero if successful, -1 if delete failed
            const start = dc.cpu.R(4).*;
            syscall_log.warn("  Unimplemented FLASHROM_DELETE (R4={X:0>8})", .{start});
            dc.cpu.R(0).* = 0xFFFFFFFF;
        },
        else => {
            syscall_log.err("  syscall_flashrom with unhandled R7: R7={d}", .{dc.cpu.R(7).*});
            @panic("syscall_flashrom with unhandled R7");
        },
    }
    return_from_syscall(dc);
}

pub fn syscall_gdrom(dc: *Dreamcast) void {
    switch (dc.cpu.R(7).*) {
        0 => {
            // GDROM_SEND_COMMAND
            // Enqueue a command for the GDROM subsystem to execute.
            // Args: r4 = command code
            //       r5 = pointer to parameter block for the command, can be NULL if the command does not take parameters
            // Returns: a request id (>=0) if successful, negative error code if failed
            syscall_log.info("  GDROM_SEND_COMMAND R4={d} R5={X:0>8}", .{ dc.cpu.R(4).*, dc.cpu.R(5).* });

            var params: [4]u32 = .{0} ** 4;
            const params_addr = dc.cpu.R(5).*;
            if (params_addr != 0) {
                for (0..4) |i| {
                    params[i] = dc.cpu.read32(@intCast(params_addr + 4 * i));
                }
            }

            dc.cpu.R(0).* = dc.gdrom.send_command(dc.cpu.R(4).*, params);
        },
        1 => {
            // GDROM_CHECK_COMMAND
            // Check if an enqueued command has completed.
            // Args: r4 = request id
            //       r5 = pointer to four 32 bit integers to receive extended status information. The first is a generic error code.
            // Returns: 0 - no such request active
            //          1 - request is still being processed
            //          2 - request has completed (if queried again, you will get a 0)
            //          3 - request was aborted(?)
            //         -1 - request has failed (examine extended status information for cause of failure)
            dc.cpu.R(0).* = dc.gdrom.check_command(dc.cpu.R(4).*);
            for (0..4) |i| {
                dc.cpu.write32(@intCast(dc.cpu.R(5).* + 4 * i), dc.gdrom.hle_result[i]);
            }
            syscall_log.info("  GDROM_CHECK_COMMAND R4={d} R5={X:0>8} | Ret : {X:0>8}, Result: {X:0>8} {X:0>8} {X:0>8} {X:0>8}", .{
                dc.cpu.R(4).*,
                dc.cpu.R(5).*,
                dc.cpu.R(0).*,
                dc.gdrom.hle_result[0],
                dc.gdrom.hle_result[1],
                dc.gdrom.hle_result[2],
                dc.gdrom.hle_result[3],
            });
        },
        2 => {
            // GDROM_MAINLOOP
            // In order for enqueued commands to get processed, this function must be called a few times.
            // It can be called from a periodic interrupt, or just keep calling it manually until GDROM_CHECK_COMMAND says that your command has stopped processing.
            // Args: none
            // Returns: no return value
            dc.gdrom.mainloop(dc);
        },
        3 => {
            // GDROM_INIT
            syscall_log.debug("  GDROM_INIT", .{});
            dc.gdrom.reinit();
        },
        4 => {
            // GDROM_CHECK_DRIVE
            // Checks the general condition of the drive.
            // Args: r4 = pointer to two 32 bit integers, to receive the drive status. The first is the current drive status, the second is the type of disc inserted (if any).
            // Returns: zero if successful, nonzero if failure

            // TODO: We always return success (i.e. ready) for now.
            //       Get actual GDROM state and disk type.
            syscall_log.debug("  GDROM_CHECK_DRIVE", .{});
            dc.cpu.write32(dc.cpu.R(4).*, @intFromEnum(dc.gdrom.state)); // GDROM status. 0x2 => Standby.
            dc.cpu.write32(dc.cpu.R(4).* + 4, 0x80); // Disk Type. 0x80 => GDROM.
            dc.cpu.R(0).* = 0;
        },
        5 => {
            // DMA END?
            // According to the disassembly, maybe?:
            //   R4: Callback
            //   R5: Callback parameter
            syscall_log.info("  GDROM_DMA_END (R7={d}) R4={X:0>8} R5={X:0>8}", .{
                dc.cpu.R(7).*,
                dc.cpu.R(4).*,
                dc.cpu.R(5).*,
            });
            dc.cpu.write32(@intFromEnum(HardwareRegister.SB_ISTNRM), @bitCast(HardwareRegisters.SB_ISTNRM{ .EoD_GDROM = 1 })); // Clear interrupt
            if (dc.cpu.R(4).* != 0) {
                syscall_log.warn(termcolor.yellow(" GDROM_DMA_END: R4={X:0>8}, is it a callback?"), .{dc.cpu.R(4).*});
            }
            // No return value, I think.
        },
        9 => {
            // GDROM_RESET
            syscall_log.warn(termcolor.yellow("  GDROM_RESET (R7={d}) : Not implemented!"), .{dc.cpu.R(7).*});
        },
        10 => {
            // GDROM_SECTOR_MODE
            syscall_log.warn(termcolor.yellow("  GDROM_SECTOR_MODE  (R7={d}) : Not implemented!"), .{dc.cpu.R(7).*});
            if (dc.cpu.read32(dc.cpu.R(4).*) == 0) { // Get/Set, if 0 the mode will be set, if 1 it will be queried.
                const mode = dc.cpu.read32(dc.cpu.R(4).* + 8) == 0;
                const sector_size_in_bytes = dc.cpu.read32(dc.cpu.R(4).* + 12) == 0;
                _ = sector_size_in_bytes;
                _ = mode;
            } else {
                dc.cpu.write32(dc.cpu.R(4).* + 4, 0x2000); // Constant
                dc.cpu.write32(dc.cpu.R(4).* + 8, 0x0800); // Mode, 1024 = mode 1, 2048 = mode 2, 0 = auto detect
                dc.cpu.write32(dc.cpu.R(4).* + 12, 0x0800); // Sector size in bytes (normally 2048)
            }
            dc.cpu.R(0).* = 0;
        },
        else => {
            syscall_log.err("  syscall_gdrom with unhandled R7: R7={d}", .{dc.cpu.R(7).*});
            @panic("syscall_gdrom with unhandled R7");
        },
    }
    return_from_syscall(dc);
}

pub fn syscall_misc(dc: *Dreamcast) void {
    syscall_log.debug("syscall_misc: R4={d} R5={X:0>8} R6={X:0>8} R7={X:0>8} ", .{ dc.cpu.R(4).*, dc.cpu.R(5).*, dc.cpu.R(6).*, dc.cpu.R(7).* });
    switch (dc.cpu.R(4).*) {
        0 => {
            // Normal Init
            // Looking at the disassembly, it does a bunch of stuff related to the GDROM, security checks and status check, but we probably don't care about it.
            dc.cpu.sr.imask = 0;
            dc.cpu.write32(@intFromEnum(HardwareRegister.SB_IML2NRM), 0);
            dc.gpu._get_register(u32, .VO_BORDER_COL).* = 0x00C0BEBC; // Set border color to light grey
        },
        1 => {
            // Return to BIOS?
            syscall_log.err("syscall_misc: Return to BIOS? R4={d} R5={X:0>8} R6={X:0>8} R7={X:0>8} ", .{ dc.cpu.R(4).*, dc.cpu.R(5).*, dc.cpu.R(6).*, dc.cpu.R(7).* });
            syscall_log.err("                              PC={X:0>8} PR={X:0>8} ", .{ dc.cpu.pc, dc.cpu.pr });
            @panic("Return to BIOS?");
        },
        else => {
            syscall_log.err("  syscall_misc with unhandled R4: R4={d}", .{dc.cpu.R(4).*});
            @panic("syscall_misc with unhandled R4");
        },
    }
    return_from_syscall(dc);
}
