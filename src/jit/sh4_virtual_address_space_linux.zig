const std = @import("std");
const builtin = @import("builtin");
const termcolor = @import("termcolor");
const Dreamcast = @import("../dreamcast.zig").Dreamcast;
const Architecture = @import("x86_64.zig");

var GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE: ?[]align(std.mem.page_size) u8 = null;

base: []align(std.mem.page_size) u8,
mirrors: std.ArrayList([]align(std.mem.page_size) u8),
boot: std.posix.fd_t,
ram: std.posix.fd_t,
vram: std.posix.fd_t,

pub fn init(allocator: std.mem.Allocator) !@This() {
    if (GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE) |_| {
        return error.AlreadyInitialized;
    }

    var vas: @This() = .{
        .base = try std.posix.mmap(null, 0x1_0000_0000, std.posix.PROT.NONE, .{ .TYPE = .PRIVATE, .ANONYMOUS = true, .NORESERVE = true }, -1, 0),
        .mirrors = std.ArrayList([]align(std.mem.page_size) u8).init(allocator),
        .boot = try allocate_backing_memory(Dreamcast.BootSize),
        .ram = try allocate_backing_memory(Dreamcast.RAMSize),
        .vram = try allocate_backing_memory(Dreamcast.VRAMSize),
    };

    // U0/P0, P1, P2, P3
    for ([_]u32{ 0x0000_0000, 0x8000_0000, 0xA000_0000, 0xC000_0000 }) |base| {
        try vas.mirror(vas.boot, Dreamcast.BootSize, base + 0x0000_0000);
        try vas.mirror(vas.vram, Dreamcast.VRAMSize, base + 0x0400_0000);
        try vas.mirror(vas.vram, Dreamcast.VRAMSize, base + 0x0600_0000);

        for (0..4) |i| {
            try vas.mirror(vas.ram, Dreamcast.RAMSize, @intCast(base + 0x0C00_0000 + i * Dreamcast.RAMSize));
        }

        // TODO: Operand Cache? This is tricky because mirrors are smaller than the minimal page alignment.
    }

    GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE = vas.base;
    var act = std.posix.Sigaction{
        .handler = .{ .sigaction = sigsegv_handler },
        .mask = std.posix.empty_sigset,
        .flags = std.posix.SA.SIGINFO,
    };
    std.posix.sigaction(std.posix.SIG.SEGV, &act, null);

    return vas;
}

pub fn deinit(self: *@This()) void {
    for (self.mirrors.items) |item| {
        std.posix.munmap(item);
    }
    self.mirrors.deinit();
    std.posix.unlinkat(self.vram, "", 0) catch |err| std.debug.print(termcolor.yellow("Failed to unlink vram: {}\n"), .{err});
    std.posix.unlinkat(self.ram, "", 0) catch |err| std.debug.print(termcolor.yellow("Failed to unlink ram: {}\n"), .{err});
    std.posix.unlinkat(self.boot, "", 0) catch |err| std.debug.print(termcolor.yellow("Failed to unlink boot: {}\n"), .{err});
    std.posix.munmap(self.base);
}

pub fn base_addr(self: *@This()) *u8 {
    return @ptrCast(self.base.ptr);
}

fn allocate_backing_memory(size: u64) !std.posix.fd_t {
    const fd = try std.posix.memfd_create("my_shared_memory", 0);
    try std.posix.ftruncate(fd, size);
    return fd;
}

fn mirror(self: *@This(), fd: std.posix.fd_t, size: u64, offset: u64) !void {
    std.debug.assert(offset % std.mem.page_size == 0);
    const result = try std.posix.mmap(
        @alignCast(@ptrCast(self.base[offset .. offset + size])),
        size,
        std.posix.PROT.READ | std.posix.PROT.WRITE,
        .{ .TYPE = .PRIVATE, .FIXED = true },
        fd,
        0,
    );
    try self.mirrors.append(result);
}

fn sigsegv_handler(sig: i32, info: *const std.posix.siginfo_t, context_ptr: ?*anyopaque) callconv(.C) void {
    std.debug.print("sigsegv_handler\n", .{});
    const context: *std.posix.ucontext_t = @alignCast(@ptrCast(context_ptr.?));
    switch (sig) {
        std.posix.SIG.SEGV => {
            const fault_address = switch (builtin.os.tag) {
                .linux => @intFromPtr(info.fields.sigfault.addr),
                else => @compileError("Unsupported OS"),
            };
            const space_base = @intFromPtr(GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE.?.ptr);

            if (fault_address >= space_base and fault_address < space_base + GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE.?.len) {
                const addr: u32 = @truncate(fault_address - space_base);

                const start_patch = context.mcontext.gregs[std.posix.REG.RIP];
                var end_patch = start_patch;

                // Skip 16bit prefix
                if (@as(*u8, @ptrFromInt(end_patch)).* == 0x66)
                    end_patch += 1;

                var rex: Architecture.REX = .{};
                if (0xF0 & @as(*u8, @ptrFromInt(end_patch)).* == 0x40) {
                    rex = @bitCast(@as(*u8, @ptrFromInt(end_patch)).*);
                    end_patch += 1;
                }

                const mov_instruction: [*]u8 = @ptrFromInt(end_patch);
                var modrm: Architecture.MODRM = @bitCast(mov_instruction[1]);

                var access_type: enum(u1) { read = 0, write = 1 } = .read;

                if (false) {
                    switch (access_type) {
                        .read => {
                            switch (mov_instruction[0]) {
                                0x8A, 0x8B => {},
                                0x0F => {
                                    end_patch += 1;
                                    modrm = @bitCast(mov_instruction[2]);
                                },
                                else => {
                                    std.debug.print("Unhandled read: {X}\n", .{mov_instruction[0]});
                                    return signal_not_handled();
                                },
                            }
                        },
                        .write => {
                            switch (mov_instruction[0]) {
                                0x88, 0x89 => {},
                                0x0F => {
                                    end_patch += 1;
                                    modrm = @bitCast(mov_instruction[2]);
                                },
                                else => {
                                    std.debug.print("Unhandled write: {X:0>2} {X:0>2}\n", .{ mov_instruction[0], mov_instruction[1] });
                                    return signal_not_handled();
                                },
                            }
                        },
                    }
                } else {
                    switch (mov_instruction[0]) {
                        0x8A, 0x8B => access_type = .read,
                        0x88, 0x89 => access_type = .write,
                        0x0F => {
                            switch (mov_instruction[1]) {
                                0x8A, 0x8B => access_type = .read,
                                0x88, 0x89 => access_type = .write,
                                else => {
                                    std.debug.print("Unhandled instruction: {X}\n", .{mov_instruction[0]});
                                    return signal_not_handled();
                                },
                            }
                            end_patch += 1;
                            modrm = @bitCast(mov_instruction[2]);
                        },
                        else => {
                            std.debug.print("Unhandled instruction: {X}\n", .{mov_instruction[0]});
                            return signal_not_handled();
                        },
                    }
                }

                switch (modrm.mod) {
                    .indirect => end_patch += 2,
                    .disp8 => end_patch += 3,
                    .disp32 => end_patch += 6,
                    else => return signal_not_handled(),
                }
                // Special case: Skip SIB byte
                if (modrm.r_m == 4) end_patch += 1;

                switch (@as(*u8, @ptrFromInt(end_patch)).*) {
                    0xEB => end_patch += 2, // JMP rel8
                    0xE9 => end_patch += 5, // JMP rel32 in 64bit mode
                    else => {
                        std.debug.print("Unhandled jump: {X:0>2}\n", .{@as(*u8, @ptrFromInt(end_patch)).*});
                        return signal_not_handled();
                    },
                }

                std.debug.print("  Access Violation: {s: <5} @ {X} - {X:0>8}  \n", .{ @tagName(access_type), fault_address, addr });
                for ([_]usize{
                    std.posix.REG.RDI,
                    std.posix.REG.RSI,
                    std.posix.REG.RBP,
                    std.posix.REG.RSP,
                    std.posix.REG.RBX,
                    std.posix.REG.RDX,
                    std.posix.REG.RCX,
                    std.posix.REG.RAX,
                }, [_][]const u8{
                    "RDI",
                    "RSI",
                    "RBP",
                    "RSP",
                    "RBX",
                    "RDX",
                    "RCX",
                    "RAX",
                }) |reg, name| {
                    std.debug.print("  {s}: {X:0>8}\n", .{ name, context.mcontext.gregs[reg] });
                }

                // const aligned_addr: []align(std.mem.page_size) u8 = @as([*]align(std.mem.page_size) u8, @ptrFromInt(start_patch & ~(@as(u64, std.mem.page_size) - 1)))[0..std.mem.page_size];
                // std.posix.mprotect(aligned_addr, std.posix.PROT.READ | std.posix.PROT.WRITE | std.posix.PROT.EXEC) catch |err| std.debug.print(termcolor.yellow("Failed to mprotect: {}\n"), .{err});

                // Patch out the mov and jump, we'll always execute the fallback from now on.
                std.debug.print("  Was: {X:0>2}\n", .{@as([*]u8, @ptrFromInt(start_patch))[0..(end_patch - start_patch)]});
                Architecture.convert_to_nops(@as([*]u8, @ptrFromInt(start_patch))[0..(end_patch - start_patch)]);
                std.debug.print("  Now: {X:0>2}\n", .{@as([*]u8, @ptrFromInt(start_patch))[0..(end_patch - start_patch)]});

                // std.posix.mprotect(aligned_addr, std.posix.PROT.READ | std.posix.PROT.EXEC) catch |err| std.debug.print(termcolor.yellow("Failed to mprotect: {}\n"), .{err});

                std.debug.print("start_patch - 0x100: {X:0>8}, start_patch: {X:0>8}, end_patch: {X:0>8}\n", .{ start_patch - 0x100, start_patch, end_patch });
                std.debug.print("  Around: {X:0>2}\n", .{@as([*]u8, @ptrFromInt(start_patch - 0x100))[0..(end_patch + 0x200 - start_patch)]});

                // Skip patched instructions. Not strictly necessary.
                // context.mcontext.gregs[std.posix.REG.RIP] = end_patch;

                return;
            }
        },
        else => std.debug.print("  Unhandled Signal: {X}\n", .{sig}),
    }

    signal_not_handled();
}

fn signal_not_handled() void {
    // Signal outside of expected range, restore default handler and let it deal with it.
    var act = std.posix.Sigaction{
        .handler = .{ .handler = std.posix.SIG.DFL },
        .mask = std.posix.empty_sigset,
        .flags = 0,
    };
    std.posix.sigaction(std.posix.SIG.SEGV, &act, null);
}
