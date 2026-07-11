//! Serial Communication Interface with FIFO (SCIF)

var pty: ?std.Io.File = null;
var rx_buffer: [16]u8 = undefined;
var rx_reader: std.Io.File.Reader = undefined;
var tx_buffer: [16]u8 = undefined;
var tx_writer: std.Io.File.Writer = undefined;

/// Used to prevent clearing bits from SCFSR2 before they are read.
/// FIXME: Not serialized.
var read_scfsr2: P4.SCFSR2 = .{};

const DefaultSymlink = "/tmp/deecy/scif";

/// Links the SCIF to /dev/ptmx
pub fn init(io: std.Io) !void {
    if (pty) |_| return;

    switch (builtin.os.tag) {
        .linux => {
            pty = std.Io.Dir.cwd().openFile(io, "/dev/ptmx", .{ .mode = .read_write }) catch |err|
                return log.err("Failed to open /dev/ptmx: {t}\n", .{err});

            errdefer {
                pty.?.close(SH4Module.Context.io);
                pty = null;
            }

            if (c.grantpt(pty.?.handle) < 0) return error.PtyCreationFailed;
            if (c.unlockpt(pty.?.handle) < 0) return error.PtyCreationFailed;

            const pty_name = c.ptsname(pty.?.handle);
            log.info(termcolor.green("[+]") ++ " Opened PTY: {s}", .{pty_name});

            // NOTE: Placing the symlink at the root of /tmp causes issues when trying to access it with elevated privileges
            //       (as required by `dc-tool-ser` with `-c`). This is a workaround for this issue.
            switch (std.os.linux.errno(std.os.linux.mkdir("/tmp/deecy", 0o777))) {
                .SUCCESS, .EXIST => {},
                else => |err| log.err("Failed to create /tmp/deecy: {t}.", .{err}),
            }
            switch (std.os.linux.errno(std.os.linux.unlink(DefaultSymlink))) {
                .SUCCESS, .NOENT => {},
                else => |err| log.err("Failed to unlink symlink '{s}': {t}.", .{ DefaultSymlink, err }),
            }
            switch (std.os.linux.errno(std.os.linux.symlink(pty_name, DefaultSymlink))) {
                .SUCCESS => log.info(termcolor.green("[+]") ++ " Serial interface available at '{s}'.", .{DefaultSymlink}),
                else => |err| log.err("Failed to create symlink '{s}': {t}.", .{ DefaultSymlink, err }),
            }

            rx_reader = pty.?.readerStreaming(io, &rx_buffer);
            tx_writer = pty.?.writerStreaming(io, &tx_buffer);
        },
        // TODO: Windows support?
        else => log.err("SCIF redirection is not supported on {t}.", .{builtin.os.tag}),
    }
}

pub fn deinit(io: std.Io) void {
    if (pty) |p| {
        switch (builtin.os.tag) {
            .linux => {
                switch (std.os.linux.errno(std.os.linux.unlink(DefaultSymlink))) {
                    .SUCCESS, .NOENT => {},
                    else => |err| log.err("Failed to unlink symlink '{s}': {t}.", .{ DefaultSymlink, err }),
                }
                p.close(io);
            },
            else => {},
        }
        pty = null;
    }
}

pub fn read(self: *const SH4, comptime T: type, virtual_addr: u32) T {
    const p4_reg: SH4Module.P4Register = @enumFromInt(virtual_addr);
    switch (p4_reg) {
        .SCFSR2 => {
            SH4Module.check_type(&[_]type{u16}, T, "Invalid P4 Write({}) to SCFSR2\n", .{T});
            check_rx(self);
            const status = self.p4_register(P4.SCFSR2, .SCFSR2).*;
            read_scfsr2 = status;
            return @bitCast(status);
        },
        .SCFRDR2 => {
            if (pty) |_| {
                defer check_rx(self);
                const byte = rx_reader.interface.takeByte() catch |err| {
                    log.err("Error reading from serial: {}\n", .{err});
                    return 0;
                };
                return byte;
            }
            return 0;
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

            log.debug("Serial FIFO out: {c}", .{value});

            if (pty) |_| {
                // Raw output to pseudo-terminal.
                tx_writer.interface.writeByte(value) catch |err|
                    log.err("Error writing serial output: {t}", .{err});
                tx_writer.interface.flush() catch |err|
                    log.err("Error flushing serial output: {t}", .{err});
            } else {
                // Colored output to stdout.
                var stdout_buffer: [128]u8 = undefined;
                var stdout_writer = std.Io.File.stdout().writer(SH4Module.Context.io, &stdout_buffer);
                const stdout = &stdout_writer.interface;
                stdout.print("\u{001b}[44m\u{001b}[97m{c}\u{001b}[0m", .{value}) catch |err|
                    log.err("Error writing serial output: {t}", .{err});
                stdout.flush() catch |err|
                    log.err("Error flushing stdout: {t}", .{err});
            }

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
            //   "Note: * Only 0 can be written, to clear the flag.""
            //   "Also note that in order to clear these flags they must be read as 1 beforehand."
            const status_register = self.p4_register(P4.SCFSR2, .SCFSR2);
            inline for (.{ "dr", "rdf", "brk", "tdfe", "tend", "er" }) |flag| {
                if (!@field(val, flag) and @field(read_scfsr2, flag))
                    @field(status_register, flag) = false;
            }
            // Always mark transmission as complete/FIFO not full.
            status_register.tdfe = true;
            status_register.tend = true;
            update_interrupts(self);
            return;
        },
        .SCFCR2 => {
            // FIFO Control Register
            SH4Module.check_type(&[_]type{u16}, T, "Invalid P4 Write({}) to SCFCR2\n", .{T});
            const val: P4.SCFCR2 = @bitCast(value);
            log.debug("Write to SCFCR2: {any}", .{val});
            if (val.rfrst) {
                log.debug("  Resetting Receive FIFO", .{});
                check_rx(self);
                rx_reader.interface.tossBuffered();
                check_rx(self);
            }
            if (val.tfrst) log.debug("  Resetting Transmit FIFO", .{});
            self.p4_register(P4.SCFCR2, .SCFCR2).* = val;
            return;
        },
        .SCLSR2, .SCSCR2 => {
            self.p4_register_addr(T, virtual_addr).* = value;
            update_interrupts(self);
        },
        else => {
            self.p4_register_addr(T, virtual_addr).* = value;
        },
    }
}

fn check_rx(self: *const SH4) void {
    // Check if serial receive is enabled.
    const serial_control_register = self.p4_register(P4.SCSCR2, .SCSCR2).*;
    if (!serial_control_register.re) return;

    if (pty) |p| {
        switch (builtin.os.tag) {
            .linux => {
                // Check if there is data to read.
                // FIXME: Is it the best way to do this?
                var fds = [_]std.posix.pollfd{.{
                    .fd = p.handle,
                    .events = std.posix.POLL.IN,
                    .revents = 0,
                }};
                const ready = std.posix.poll(&fds, 0) catch return;
                if ((ready > 0 and fds[0].revents & std.posix.POLL.IN != 0) or rx_reader.interface.bufferedLen() > 0)
                    on_rx(self);
            },
            else => {},
        }
    }
}

fn on_rx(self: *const SH4) void {
    // FIXME: This is only here to populate the buffer.
    const byte = rx_reader.interface.peekByte() catch |err|
        return log.err("Error peeking byte: {}\n", .{err});
    self.p4_register(u8, .SCFRDR2).* = byte;
    log.debug("Read from serial: {}", .{byte});

    const fifo_control_register = self.p4_register(P4.SCFCR2, .SCFCR2).*;
    var status_register = self.p4_register(P4.SCFSR2, .SCFSR2);
    if (rx_reader.interface.bufferedLen() >= fifo_control_register.rtrg.bytes())
        status_register.rdf = true;
    if (rx_reader.interface.bufferedLen() < fifo_control_register.rtrg.bytes())
        status_register.dr = true;

    update_interrupts(@constCast(self));
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

const c = @cImport({
    @cDefine("_XOPEN_SOURCE", "500");
    @cInclude("stdlib.h");
});

const builtin = @import("builtin");
const std = @import("std");
const log = std.log.scoped(.sh4_scif);
const termcolor = @import("termcolor");
const SH4Module = @import("sh4.zig");
const SH4 = SH4Module.SH4;
const P4 = SH4Module.P4;
