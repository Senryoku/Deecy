const std = @import("std");
const builtin = @import("builtin");
const termcolor = @import("termcolor");
const Dreamcast = @import("../dreamcast.zig").Dreamcast;
const Architecture = @import("x86_64.zig");
const VAS = @import("sh4_virtual_address_space.zig");

var GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE: ?[]align(std.heap.page_size_min) u8 = null;

base: []align(std.heap.page_size_min) u8,
mirrors: std.ArrayList([]align(std.heap.page_size_min) u8) = .empty,
boot: std.posix.fd_t,
ram: std.posix.fd_t,
vram: std.posix.fd_t,
aram: std.posix.fd_t,

pub fn init(allocator: std.mem.Allocator) !@This() {
    if (GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE) |_| {
        return error.AlreadyInitialized;
    }

    var vas: @This() = .{
        .base = try std.posix.mmap(null, 0x1_0000_0000, std.posix.PROT.NONE, .{ .TYPE = .PRIVATE, .ANONYMOUS = true, .NORESERVE = true }, -1, 0),
        .boot = try allocate_backing_memory("boot", Dreamcast.BootSize),
        .ram = try allocate_backing_memory("ram", Dreamcast.RAMSize),
        .vram = try allocate_backing_memory("vram", Dreamcast.VRAMSize),
        .aram = try allocate_backing_memory("aram", Dreamcast.ARAMSize),
    };

    //           U0/P0 and mirrors,                                  P1,          P2,          P3
    for ([_]u32{ 0x0000_0000, 0x2000_0000, 0x4000_0000, 0x6000_0000, 0x8000_0000, 0xA000_0000, 0xC000_0000 }) |base| {
        try vas.mirror(allocator, vas.boot, Dreamcast.BootSize, base + 0x0000_0000);
        for (0..(0x0100_0000 - 0x0080_0000) / Dreamcast.ARAMSize) |i|
            try vas.mirror(allocator, vas.aram, Dreamcast.ARAMSize, @intCast(base + 0x0080_0000 + i * Dreamcast.ARAMSize));
        try vas.mirror(allocator, vas.vram, Dreamcast.VRAMSize, base + 0x0400_0000);
        try vas.mirror(allocator, vas.vram, Dreamcast.VRAMSize, base + 0x0600_0000);
        for (0..(0x1000_0000 - 0x0C00_0000) / Dreamcast.RAMSize) |i|
            try vas.mirror(allocator, vas.ram, Dreamcast.RAMSize, @intCast(base + 0x0C00_0000 + i * Dreamcast.RAMSize));
    }

    GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE = vas.base;
    var act = std.posix.Sigaction{
        .handler = .{ .sigaction = sigsegv_handler },
        .mask = std.posix.sigemptyset(),
        .flags = std.posix.SA.SIGINFO,
    };
    std.posix.sigaction(std.posix.SIG.SEGV, &act, null);

    return vas;
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    var act = std.posix.Sigaction{
        .handler = .{ .handler = std.posix.SIG.DFL },
        .mask = std.posix.sigemptyset(),
        .flags = 0,
    };
    std.posix.sigaction(std.posix.SIG.SEGV, &act, null);

    GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE = null;

    for (self.mirrors.items) |item|
        std.posix.munmap(item);
    self.mirrors.deinit(allocator);
    std.posix.munmap(self.base);
}

pub fn base_addr(self: *@This()) *u8 {
    return @ptrCast(self.base.ptr);
}

fn allocate_backing_memory(name: []const u8, size: u64) !std.posix.fd_t {
    const fd = try std.posix.memfd_create(name, 0);
    try std.posix.ftruncate(fd, size);
    return fd;
}

fn mirror(self: *@This(), allocator: std.mem.Allocator, fd: std.posix.fd_t, size: u64, offset: u64) !void {
    std.debug.assert(offset % std.heap.page_size_min == 0);
    const result = try std.posix.mmap(
        @ptrCast(@alignCast(self.base[offset .. offset + size])),
        size,
        std.posix.PROT.READ | std.posix.PROT.WRITE,
        .{ .TYPE = .SHARED, .FIXED = true },
        fd,
        0,
    );
    try self.mirrors.append(allocator, result);
}

fn sigsegv_handler(sig: i32, info: *const std.posix.siginfo_t, context_ptr: ?*anyopaque) callconv(.c) void {
    switch (sig) {
        std.posix.SIG.SEGV => {
            const fault_address = switch (builtin.os.tag) {
                .linux => @intFromPtr(info.fields.sigfault.addr),
                else => @compileError("Unsupported OS"),
            };

            if (GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE) |base| {
                const context: *std.posix.ucontext_t = @ptrCast(@alignCast(context_ptr.?));
                VAS.patch_access(fault_address, @intFromPtr(base.ptr), base.len, &context.mcontext.gregs[std.posix.REG.RIP]) catch |err| {
                    std.log.scoped(.sh4_jit).err("Failed to patch FastMem access @{X}: {s}", .{ fault_address, @errorName(err) });
                    signal_not_handled();
                };
                return;
            } else {
                std.log.scoped(.sh4_jit).err("Virtual Address Space not initialized.", .{});
            }
        },
        else => std.log.scoped(.sh4_jit).debug("Unhandled Signal: {X}\n", .{sig}),
    }

    signal_not_handled();
}

fn signal_not_handled() void {
    // Signal outside of expected range, restore default handler and let it deal with it.
    var act = std.posix.Sigaction{
        .handler = .{ .handler = std.posix.SIG.DFL },
        .mask = std.posix.sigemptyset(),
        .flags = 0,
    };
    std.posix.sigaction(std.posix.SIG.SEGV, &act, null);
}
