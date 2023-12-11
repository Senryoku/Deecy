const std = @import("std");

const MemoryRegister = @import("MemoryRegisters.zig").MemoryRegister;

const sh4 = @import("sh4.zig");
const SH4 = sh4.SH4;
const Instr = sh4.Instr;

// FIXME: MOVE THIS (Part of a Bus struct?)
const GDROM = @import("gdrom.zig").GDROM;
pub var gdrom: GDROM = .{};

pub fn syscall(cpu: *SH4, opcode: Instr) void {
    std.debug.print("Unimplemented SYSCALL: 0x{X:0>8} = 0b{b:0>16}\n", .{ cpu.pc, opcode.value });
    for (0..15) |i| {
        std.debug.print("  R{d}: {x}\n", .{ i, cpu.R(@intCast(i)).* });
    }
    @panic("Unimplemented SYSCALL");
}

pub fn syscall_sysinfo(cpu: *SH4, _: Instr) void {
    switch (cpu.R(7).*) {
        0 => {
            // Prepares the other two SYSINFO calls for use by copying the relevant data from the system flashrom into 8C000068-8C00007F. Always call this function before using the other two calls.

            @memcpy(cpu.ram[0x00000068 .. 0x00000068 + 8], cpu.flash[0x1A056 .. 0x1A056 + 8]);
            @memcpy(cpu.ram[0x00000068 + 8 .. 0x00000068 + 8 + 6], cpu.flash[0x1A000 .. 0x1A000 + 6]);

            cpu.R(0).* = 0;
        },
        3 => {
            // Query the unique 64 bit ID number of this Dreamcast. SYSINFO_INIT must have been called first.
            // Args: none
            // Returns: A pointer to where the ID is stored as 8 contiguous bytes
            cpu.R(0).* = 0x8c000068;
        },
        else => {
            std.debug.print("  syscall_sysinfo with unhandled R7: R7={d}\n", .{cpu.R(7).*});
            @panic("syscall_sysinfo with unhandled R7");
        },
    }

    // Ret
    cpu.pc = cpu.pr - 2;
}

pub fn syscall_romfont(cpu: *SH4, _: Instr) void {
    switch (cpu.R(7).*) {
        else => {
            std.debug.print("  syscall_romfont with unhandled R7: R7={d}\n", .{cpu.R(7).*});
            @panic("syscall_romfont with unhandled R7");
        },
    }

    // Ret
    cpu.pc = cpu.pr - 2;
}

pub fn syscall_flashrom(cpu: *SH4, _: Instr) void {
    switch (cpu.R(7).*) {
        0 => {
            // Queries the extent of a single partition in the system flashrom.
            // Args: r4 = partition number (0-4)
            //       r5 = pointer to two 32 bit integers to receive the result. The first will be the offset of the partition start, in bytes from the start of the flashrom. The second will be the size of the partition, in bytes.
            // Returns: zero if successful, -1 if no such partition exists

            cpu.R(0).* = 0;
            const dest = cpu.R(5).*;
            switch (cpu.R(4).*) {
                0 => {
                    cpu.write32(dest, 0x1A000);
                    cpu.write32(dest + 4, 8 * 1024);
                },
                1 => {
                    cpu.write32(dest, 0x18000);
                    cpu.write32(dest + 4, 8 * 1024);
                },
                2 => {
                    cpu.write32(dest, 0x1C000);
                    cpu.write32(dest + 4, 16 * 1024);
                },
                3 => {
                    cpu.write32(dest, 0x10000);
                    cpu.write32(dest + 4, 32 * 1024);
                },
                4 => {
                    cpu.write32(dest, 0x00000);
                    cpu.write32(dest + 4, 64 * 1024);
                },
                else => {
                    cpu.R(0).* = @bitCast(@as(i32, @intCast(-1)));
                },
            }
        },
        1 => {
            // Read data from the system flashrom.
            // Args: r4 = read start position, in bytes from the start of the flashrom
            //       r5 = pointer to destination buffer
            //       r6 = number of bytes to read
            // Returns: number of read bytes if successful, -1 if read failed

            const start = cpu.R(4).*;
            const len = cpu.R(6).*;
            const dest = (cpu.R(5).* & 0x1FFFFFFF) - 0x0C000000;
            @memcpy(cpu.ram[dest .. dest + len], cpu.flash[start .. start + len]);
            cpu.R(0).* = len;
        },
        else => {
            std.debug.print("  syscall_flashrom with unhandled R7: R7={d}\n", .{cpu.R(7).*});
            @panic("syscall_flashrom with unhandled R7");
        },
    }

    // Ret
    cpu.pc = cpu.pr - 2;
}

pub fn syscall_gdrom(cpu: *SH4, _: Instr) void {
    switch (cpu.R(7).*) {
        0 => {
            // GDROM_SEND_COMMAND
            // Enqueue a command for the GDROM subsystem to execute.
            // Args: r4 = command code
            //       r5 = pointer to parameter block for the command, can be NULL if the command does not take parameters
            // Returns: a request id (>=0) if successful, negative error code if failed
            std.debug.print("  TODO: GDROM_SEND_COMMAND R4={d} R5={X:0>8}\n", .{ cpu.R(4).*, cpu.R(5).* });

            var params: [4]u32 = .{0} ** 4;
            const params_addr = cpu.R(5).*;
            if (params_addr != 0) {
                params[0] = cpu.read32(params_addr);
                params[1] = cpu.read32(params_addr + 4);
                params[2] = cpu.read32(params_addr + 8);
                params[3] = cpu.read32(params_addr + 12);
            }

            cpu.R(0).* = gdrom.send_command(cpu.R(4).*, params);
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
            std.debug.print("  TODO: GDROM_CHECK_COMMAND R4={d} R5={X:0>8}\n", .{ cpu.R(4).*, cpu.R(5).* });
            cpu.R(0).* = gdrom.check_command(cpu.R(4).*);
            for (0..4) |i| {
                cpu.write32(@intCast(cpu.R(5).* + 4 * i), gdrom.result[i]);
            }
        },
        2 => {
            // GDROM_MAINLOOP
            // In order for enqueued commands to get processed, this function must be called a few times.
            // It can be called from a periodic interrupt, or just keep calling it manually until GDROM_CHECK_COMMAND says that your command has stopped processing.
            // Args: none
            // Returns: no return value
            gdrom.mainloop(cpu);
        },
        3 => {
            // GDROM_INIT
            std.debug.print("  TODO: GDROM_INIT\n", .{});
        },
        4 => {
            // GDROM_CHECK_DRIVE
            // Checks the general condition of the drive.
            // Args: r4 = pointer to two 32 bit integers, to receive the drive status. The first is the current drive status, the second is the type of disc inserted (if any).
            // Returns: zero if successful, nonzero if failure

            // TODO: We always return success (i.e. ready) for now.
            //       Get actual GDROM state and disk type.
            cpu.write32(cpu.R(4).*, 0x2); // GDROM status. 0x2 => Standby.
            cpu.write32(cpu.R(4).* + 4, 0x80); // Disk Type. 0x80 => GDROM.
            cpu.R(0).* = 0;
        },
        else => {
            std.debug.print("  syscall_gdrom with unhandled R7: R7={d}\n", .{cpu.R(7).*});
            @panic("syscall_gdrom with unhandled R7");
        },
    }
    // Ret
    cpu.pc = cpu.pr - 2;
}

pub fn syscall_misc(cpu: *SH4, _: Instr) void {
    // This comes pretty much directly from flycast, I don't think there's any official
    // documentation on these syscalls, https://mc.pp.se/dc/syscalls.html doesn't have enough info.
    std.debug.print("syscall_misc: R4={d} R5={X:0>8} R6={X:0>8} R7={X:0>8} \n", .{ cpu.R(4).*, cpu.R(5).*, cpu.R(6).*, cpu.R(7).* });
    switch (cpu.R(4).*) {
        0 => {
            // Normal Init
            cpu.write32(@intFromEnum(MemoryRegister.SB_GDSTARD), 0x0C010000); // + bootSectors * 2048;
            cpu.write32(@intFromEnum(MemoryRegister.SB_IML2NRM), 0);
            cpu.R(0).* = 0x00C0BEBC;
            // TODO: VO_BORDER_COL.full = p_sh4rcb->cntx.r[0];
        },
        1 => {
            // Return to BIOS?
            std.debug.print("syscall_misc: Return to BIOS? R4={d} R5={X:0>8} R6={X:0>8} R7={X:0>8} \n", .{ cpu.R(4).*, cpu.R(5).*, cpu.R(6).*, cpu.R(7).* });
            @panic("Return to BIOS?");
        },
        else => {
            std.debug.print("  syscall_misc with unhandled R4: R4={d}\n", .{cpu.R(4).*});
            @panic("syscall_misc with unhandled R4");
        },
    }
    // Syscall are called using JSR, simulate a RTS/N (return from subroutine, without delay slot)
    cpu.pc = cpu.pr - 2;
}
