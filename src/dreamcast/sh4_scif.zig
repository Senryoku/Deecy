//! Serial Communication Interface with FIFO (SCIF)

pub fn write(self: *SH4, comptime T: type, virtual_addr: u32, value: T) void {
    const p4_reg: SH4Module.P4Register = @enumFromInt(virtual_addr);
    log.debug("Write to {t}: {any}", .{ p4_reg, value });
    switch (p4_reg) {
        .SCFTDR2 => {
            SH4Module.check_type(&[_]type{u8}, T, "Invalid P4 Write({}) to SCFTDR2\n", .{T});

            var stdout_buffer: [128]u8 = undefined;
            var stdout_writer = std.Io.File.stdout().writer(SH4Module.Context.io, &stdout_buffer);
            const stdout = &stdout_writer.interface;

            stdout.print("\u{001b}[44m\u{001b}[97m{c}\u{001b}[0m", .{value}) catch |err| {
                log.err("Error writing serial output: {}\n", .{err});
            };
            stdout.flush() catch |err| {
                log.err("Error flushing stdout: {}\n", .{err});
            };

            // Immediately mark transfer as complete.
            const status_register = self.p4_register(P4.SCFSR2, .SCFSR2);
            status_register.tdfe = true;

            const control_register = self.p4_register(P4.SCSCR2, .SCSCR2);
            const fifo_control = self.p4_register(P4.SCFCR2, .SCFCR2);
            if (control_register.tie) {
                log.warn("Unimplemented SCIF_TXI interrupt enabled at {d} bytes.", .{fifo_control.ttrg.bytes()});
                // self.request_interrupt(Interrupt.SCIF_TXI);
            }
            return;
        },
        .SCFSR2 => {
            SH4Module.check_type(&[_]type{u16}, T, "Invalid P4 Write({}) to SCFSR2\n", .{T});
            const val: P4.SCFSR2 = @bitCast(value);
            log.debug("Write to SCFSR2: {any}", .{val});
            // Writable bits can only be cleared.
            // TODO: "Also note that in order to clear these flags they must be read as 1 beforehand."
            // self.p4_register(u16, .SCFSR2).* &= (value | 0b11111111_00001100);
            return;
        },
        .SCFCR2 => {
            // FIFO Control Register
            SH4Module.check_type(&[_]type{u16}, T, "Invalid P4 Write({}) to SCFCR2\n", .{T});
            const val: P4.SCFCR2 = @bitCast(value);
            log.debug("Write to SCFCR2: {any}", .{val});
            if (val.rfrst) log.debug("  Resetting Receive FIFO", .{});
            if (val.tfrst) log.debug("  Resetting Transmit FIFO", .{});
            // self.p4_register(P4.SCFCR2, .SCFSR2).* = val;
            return;
        },
        else => {
            self.p4_register_addr(T, virtual_addr).* = value;
        },
    }
}

const std = @import("std");
const log = std.log.scoped(.sh4_scif);
const SH4Module = @import("sh4.zig");
const SH4 = SH4Module.SH4;
const P4 = SH4Module.P4;
