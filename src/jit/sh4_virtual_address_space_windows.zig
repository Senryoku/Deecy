const std = @import("std");
const windows = std.os.windows;
const termcolor = @import("termcolor");
const Dreamcast = @import("../dreamcast.zig");
const Architecture = @import("x86_64.zig");
const VAS = @import("sh4_virtual_address_space.zig");

var GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE: ?std.os.windows.LPVOID = null;

base: std.os.windows.LPVOID = undefined,
no_access: std.ArrayList(*anyopaque), // Reads and Writes to these ranges will throw an access violation
mirrors: std.ArrayList(std.os.windows.LPVOID),
boot: std.os.windows.LPVOID = undefined,
ram: std.os.windows.LPVOID = undefined,
vram: std.os.windows.LPVOID = undefined,

pub fn init(allocator: std.mem.Allocator) !@This() {
    var vas: @This() = .{
        .no_access = std.ArrayList(*anyopaque).init(allocator),
        .mirrors = std.ArrayList(std.os.windows.LPVOID).init(allocator),
    };

    vas.boot = try allocate_backing_memory(Dreamcast.BootSize);
    vas.ram = try allocate_backing_memory(Dreamcast.RAMSize);
    vas.vram = try allocate_backing_memory(Dreamcast.VRAMSize);

    // Ask nicely for an available virtual address space.
    vas.base = try std.os.windows.VirtualAlloc(null, 0x1_0000_0000, std.os.windows.MEM_RESERVE, std.os.windows.PAGE_NOACCESS);
    // Free it immediately. We'll try to reacquire it. I think I read somewhere there's a way to avoid this race condition with a newer API, but I don't remember right now.
    std.os.windows.VirtualFree(vas.base, 0, std.os.windows.MEM_RELEASE);

    // U0/P0, P1, P2, P3
    for ([_]u32{ 0x0000_0000, 0x8000_0000, 0xA000_0000, 0xC000_0000 }) |base| {
        try vas.mirror(vas.boot, base + 0x0000_0000);

        try vas.forbid(base + Dreamcast.BootSize, base + 0x0400_0000);

        try vas.mirror(vas.vram, base + 0x0400_0000);
        try vas.forbid(base + 0x0500_0000, base + 0x0600_0000);
        try vas.mirror(vas.vram, base + 0x0600_0000);

        try vas.forbid(base + 0x0700_0000, base + 0x0C00_0000);

        for (0..4) |i| {
            try vas.mirror(vas.ram, @intCast(base + 0x0C00_0000 + i * Dreamcast.RAMSize));
        }

        try vas.forbid(base + 0x0C00_0000 + 4 * Dreamcast.RAMSize, base + 0x2000_0000);

        // TODO: Operand Cache? This is tricky because mirrors are smaller than the minimal page alignment.
    }

    // P4
    try vas.forbid(0xE0000000, 0x1_0000_0000);

    GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE = vas.base;
    _ = std.os.windows.kernel32.AddVectoredExceptionHandler(1, handle_segfault_windows);

    return vas;
}

pub fn deinit(self: *@This()) void {
    for (self.mirrors.items) |item| {
        if (windows.UnmapViewOfFile(item) == 0)
            std.log.warn(termcolor.yellow("UnmapViewOfFile Error: {}\n"), .{std.os.windows.GetLastError()});
    }
    self.mirrors.deinit();
    for (self.no_access.items) |item| {
        std.os.windows.VirtualFree(item, 0, std.os.windows.MEM_RELEASE);
    }
    self.no_access.deinit();
    std.os.windows.CloseHandle(self.vram);
    std.os.windows.CloseHandle(self.ram);
    std.os.windows.CloseHandle(self.boot);
}

pub fn base_addr(self: *@This()) *u8 {
    return @ptrCast(self.base);
}

fn allocate_backing_memory(size: std.os.windows.DWORD) !*anyopaque {
    if (windows.CreateFileMappingA(std.os.windows.INVALID_HANDLE_VALUE, null, std.os.windows.PAGE_READWRITE, 0, size, null)) |handle| {
        return handle;
    }
    return error.FileMapError;
}

fn mirror(self: *@This(), file_handle: std.os.windows.HANDLE, addr: u64) !void {
    const result = windows.MapViewOfFileEx(
        file_handle,
        windows.FILE_MAP_ALL_ACCESS,
        0,
        0, // Offset 0
        0, // Full size
        @ptrFromInt(@intFromPtr(self.base) + addr),
    );
    if (result == null) {
        std.debug.print("MapViewOfFileEx({X:0>8}) Error: {}\n", .{ addr, std.os.windows.GetLastError() });
        return error.MapError;
    }
    try self.mirrors.append(result.?);
}

fn forbid(self: *@This(), from: u32, to: u64) !void {
    try self.no_access.append(try std.os.windows.VirtualAlloc(
        @ptrFromInt(@intFromPtr(self.base) + from),
        to - from,
        std.os.windows.MEM_RESERVE,
        std.os.windows.PAGE_NOACCESS,
    ));
}

fn handle_segfault_windows(info: *std.os.windows.EXCEPTION_POINTERS) callconv(std.os.windows.WINAPI) c_long {
    switch (info.ExceptionRecord.ExceptionCode) {
        std.os.windows.EXCEPTION_ACCESS_VIOLATION => {
            const fault_address = info.ExceptionRecord.ExceptionInformation[1];

            VAS.patch_access(fault_address, @intFromPtr(GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE), 0x1_0000_0000, &info.ContextRecord.Rip) catch |err| {
                std.log.scoped(.sh4_jit).err("Failed to patch FastMem access: {s}", .{@errorName(err)});
                return windows.EXCEPTION_CONTINUE_SEARCH;
            };

            return windows.EXCEPTION_CONTINUE_EXECUTION;
        },
        else => return std.os.windows.EXCEPTION_CONTINUE_SEARCH,
    }
    return std.os.windows.EXCEPTION_CONTINUE_SEARCH;
}
