//! Serial Communication Interface with FIFO (SCIF)

var stdin_intialized = false;
var stdin_buffer: [128]u8 = undefined;
var stdin_reader: std.Io.File.Reader = undefined;

pub fn read(self: *const SH4, comptime T: type, virtual_addr: u32) T {
    const p4_reg: SH4Module.P4Register = @enumFromInt(virtual_addr);
    switch (p4_reg) {
        .SCFSR2 => {
            SH4Module.check_type(&[_]type{u16}, T, "Invalid P4 Write({}) to SCFSR2\n", .{T});
            var val = self.p4_register(P4.SCFSR2, .SCFSR2).*;

            if (!stdin_intialized) {
                stdin_reader = std.Io.File.stdin().readerStreaming(SH4Module.Context.io, &stdin_buffer);
                stdin_intialized = true;
            }
            var fds = [_]std.posix.pollfd{.{
                .fd = std.Io.File.stdin().handle,
                .events = std.posix.POLL.IN,
                .revents = 0,
            }};

            const ready = std.posix.poll(&fds, 0) catch return @bitCast(val);
            if (ready > 0) {
                const byte = stdin_reader.interface.takeByte() catch unreachable;
                log.debug("Read from stdin: {}", .{byte});
                self.p4_register(u8, .SCFRDR2).* = byte;
                const control_register = self.p4_register(P4.SCFCR2, .SCFCR2).*;
                val.rdf = stdin_reader.interface.bufferedLen() >= control_register.rtrg.bytes();
                val.dr = stdin_reader.interface.bufferedLen() < control_register.rtrg.bytes();

                self.p4_register(P4.SCFSR2, .SCFSR2).* = val;
                update_interrupts(@constCast(self));
            }
            return @bitCast(val);
        },
        else => {
            return self.p4_register_addr(T, virtual_addr).*;
        },
    }
}

pub fn write(self: *SH4, comptime T: type, virtual_addr: u32, value: T) void {
    const p4_reg: SH4Module.P4Register = @enumFromInt(virtual_addr);
    log.debug("Write to {t}: {any}", .{ p4_reg, value });
    switch (p4_reg) {
        .SCFTDR2 => {
            SH4Module.check_type(&[_]type{u8}, T, "Invalid P4 Write({}) to SCFTDR2\n", .{T});

            var stdout_buffer: [128]u8 = undefined;
            var stdout_writer = std.Io.File.stdout().writer(SH4Module.Context.io, &stdout_buffer);
            const stdout = &stdout_writer.interface;

            log.debug("Serial FIFO out: {c}", .{value});
            if (false) {
                stdout.print("\u{001b}[44m\u{001b}[97m{c}\u{001b}[0m", .{value}) catch |err| {
                    log.err("Error writing serial output: {}\n", .{err});
                };
            } else {
                stdout.writeByte(value) catch |err| {
                    log.err("Error writing serial output: {}\n", .{err});
                };
            }
            stdout.flush() catch |err| {
                log.err("Error flushing stdout: {}\n", .{err});
            };

            // Immediately mark transfer as complete/FIFO not full.
            const status_register = self.p4_register(P4.SCFSR2, .SCFSR2);
            status_register.tdfe = true;
            status_register.tend = true;

            update_interrupts(self);
            return;
        },
        .SCFSR2 => {
            SH4Module.check_type(&[_]type{u16}, T, "Invalid P4 Write({}) to SCFSR2\n", .{T});
            const val: P4.SCFSR2 = @bitCast(value);
            log.debug("Write to SCFSR2: {any}", .{val});
            // Writable bits can only be cleared.
            // "Note: * Only 0 can be written, to clear the flag.""
            // TODO: "Also note that in order to clear these flags they must be read as 1 beforehand."
            const scfsr2 = self.p4_register(P4.SCFSR2, .SCFSR2);
            inline for (.{ "dr", "rdf", "brk", "tdfe", "tend", "er" }) |flag| {
                if (!@field(val, flag)) @field(scfsr2, flag) = false;
            }
            // Always mark transmission as complete/FIFO not full.
            scfsr2.tdfe = true;
            scfsr2.tend = true;
            update_interrupts(self);
            return;
        },
        .SCFCR2 => {
            // FIFO Control Register
            SH4Module.check_type(&[_]type{u16}, T, "Invalid P4 Write({}) to SCFCR2\n", .{T});
            const val: P4.SCFCR2 = @bitCast(value);
            log.debug("Write to SCFCR2: {any}", .{val});
            if (val.rfrst) log.debug("  Resetting Receive FIFO", .{});
            if (val.tfrst) log.debug("  Resetting Transmit FIFO", .{});
            self.p4_register(P4.SCFCR2, .SCFCR2).* = val;
            return;
        },
        else => {
            self.p4_register_addr(T, virtual_addr).* = value;
        },
    }
}

fn update_interrupts(self: *SH4) void {
    const status_register = self.p4_register(P4.SCFSR2, .SCFSR2);
    const line_status_register = self.p4_register(P4.SCLSR2, .SCLSR2);
    const control_register = self.p4_register(P4.SCSCR2, .SCSCR2);
    self.set_interrupt(SH4Module.Interrupt.SCIF_TXI, status_register.tdfe and control_register.tie);
    self.set_interrupt(SH4Module.Interrupt.SCIF_RXI, (status_register.rdf or status_register.dr) and control_register.rie);
    // "By setting the REIE bit to 1 while the RIE bit is cleared to 0, it is possible to output ERI and BRI interrupt requests, but not RXI interrupt requests."
    self.set_interrupt(SH4Module.Interrupt.SCIF_BRI, (status_register.brk or line_status_register.orer) and (control_register.rie or control_register.reie));
    self.set_interrupt(SH4Module.Interrupt.SCIF_ERI, status_register.er and (control_register.rie or control_register.reie));
}

const std = @import("std");
const log = std.log.scoped(.sh4_scif);
const SH4Module = @import("sh4.zig");
const SH4 = SH4Module.SH4;
const P4 = SH4Module.P4;
