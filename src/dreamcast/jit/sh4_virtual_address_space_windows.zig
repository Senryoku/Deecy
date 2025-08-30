const std = @import("std");
const windows = @import("../host/windows.zig");
const termcolor = @import("termcolor");
const Dreamcast = @import("../dreamcast.zig").Dreamcast;
const Architecture = @import("x86_64.zig");
const VAS = @import("sh4_virtual_address_space.zig");

const log = std.log.scoped(.sh4_jit);

var GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE: ?std.os.windows.LPVOID = null;
var GLOBAL_EXCEPTION_HANDLER: ?std.os.windows.LPVOID = null;

base: std.os.windows.LPVOID = undefined,
no_access: std.ArrayList(*anyopaque) = .empty, // Reads and Writes to these ranges will throw an access violation
mirrors: std.ArrayList(std.os.windows.LPVOID) = .empty,
boot: std.os.windows.LPVOID = undefined,
ram: std.os.windows.LPVOID = undefined,
vram: std.os.windows.LPVOID = undefined,
aram: std.os.windows.LPVOID = undefined,

pub fn init(allocator: std.mem.Allocator) !@This() {
    if (GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE) |_| {
        return error.AlreadyInitialized;
    }

    var vas: @This() = .{};
    errdefer vas.deinit(allocator);

    vas.boot = try allocate_backing_memory(Dreamcast.BootSize);
    vas.ram = try allocate_backing_memory(Dreamcast.RAMSize);
    vas.vram = try allocate_backing_memory(Dreamcast.VRAMSize);
    vas.aram = try allocate_backing_memory(Dreamcast.ARAMSize);

    // Try repeatedly to map the virtual address space.
    // It can technically fail if another thread uses the VirtualAlloc/MapViewOfFile API.
    // We can do better using placeholders: https://devblogs.microsoft.com/oldnewthing/20240201-00/?p=109346
    //   But... Using VirtualAlloc2/MapViewOfFile3 from zig seems complicated right now.
    var attempts: u32 = 0;
    var mapped = false;
    while (!mapped and attempts < 10) : (attempts += 1) {
        mapped = true;
        vas.try_mapping(allocator) catch |err| {
            log.err(termcolor.red("Failed to map virtual address space: {t} (attempt {d}/10)"), .{ err, attempts + 1 });
            mapped = false;
        };
    }
    if (!mapped) return error.FailedToMapVirtualAddressSpace;

    GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE = vas.base;
    GLOBAL_EXCEPTION_HANDLER = std.os.windows.kernel32.AddVectoredExceptionHandler(1, handle_segfault_windows);

    return vas;
}

fn try_mapping(self: *@This(), allocator: std.mem.Allocator) !void {
    // Ask nicely for an available virtual address space.
    self.base = try std.os.windows.VirtualAlloc(null, 0x1_0000_0000, std.os.windows.MEM_RESERVE, std.os.windows.PAGE_NOACCESS);
    // Free it immediately. We'll try to reacquire it.
    std.os.windows.VirtualFree(self.base, 0, std.os.windows.MEM_RELEASE);

    errdefer self.release_views();

    //           U0/P0 and mirrors,                                  P1,          P2,          P3
    for ([_]u32{ 0x0000_0000, 0x2000_0000, 0x4000_0000, 0x6000_0000, 0x8000_0000, 0xA000_0000, 0xC000_0000 }) |base| {
        try self.mirror(allocator, self.boot, base + 0x0000_0000);

        try self.forbid(allocator, base + Dreamcast.BootSize, base + 0x0080_0000);
        for (0..(0x0100_0000 - 0x0080_0000) / Dreamcast.ARAMSize) |i|
            try self.mirror(allocator, self.aram, base + 0x0080_0000 + i * Dreamcast.ARAMSize);
        try self.forbid(allocator, base + 0x0100_0000, base + 0x0400_0000);

        try self.mirror(allocator, self.vram, base + 0x0400_0000); // 64-bit path
        try self.forbid(allocator, base + 0x0500_0000, base + 0x0600_0000); // 32-bit path
        try self.mirror(allocator, self.vram, base + 0x0600_0000); // 64-bit path mirror
        try self.forbid(allocator, base + 0x0700_0000, base + 0x0C00_0000); // 32-bit path mirror and more

        for (0..(0x1000_0000 - 0x0C00_0000) / Dreamcast.RAMSize) |i|
            try self.mirror(allocator, self.ram, @intCast(base + 0x0C00_0000 + i * Dreamcast.RAMSize));

        try self.forbid(allocator, base + 0x1000_0000, base + 0x2000_0000);
    }
    try self.forbid(allocator, 0xE000_0000, 0x1_0000_0000); // Forbid P4
}

fn release_views(self: *@This()) void {
    for (self.mirrors.items) |item| {
        if (windows.UnmapViewOfFile(item) == 0)
            log.warn(termcolor.yellow("UnmapViewOfFile Error: {}\n"), .{std.os.windows.GetLastError()});
    }
    self.mirrors.clearRetainingCapacity();
    for (self.no_access.items) |item| {
        std.os.windows.VirtualFree(item, 0, std.os.windows.MEM_RELEASE);
    }
    self.no_access.clearRetainingCapacity();
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    self.release_views();
    self.mirrors.deinit(allocator);
    self.no_access.deinit(allocator);
    std.os.windows.CloseHandle(self.aram);
    std.os.windows.CloseHandle(self.vram);
    std.os.windows.CloseHandle(self.ram);
    std.os.windows.CloseHandle(self.boot);
    if (std.os.windows.kernel32.RemoveVectoredExceptionHandler(GLOBAL_EXCEPTION_HANDLER.?) == 0)
        log.err(termcolor.red("Call to RemoveVectoredExceptionHandler failed.\n"), .{});
    GLOBAL_EXCEPTION_HANDLER = null;
    GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE = null;
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

fn mirror(self: *@This(), allocator: std.mem.Allocator, file_handle: std.os.windows.HANDLE, addr: u64) !void {
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
    try self.mirrors.append(allocator, result.?);
}

fn forbid(self: *@This(), allocator: std.mem.Allocator, from: u32, to: u64) !void {
    try self.no_access.append(allocator, try std.os.windows.VirtualAlloc(
        @ptrFromInt(@intFromPtr(self.base) + from),
        to - from,
        std.os.windows.MEM_RESERVE,
        std.os.windows.PAGE_NOACCESS,
    ));
}

fn handle_segfault_windows(info: *std.os.windows.EXCEPTION_POINTERS) callconv(.winapi) c_long {
    switch (info.ExceptionRecord.ExceptionCode) {
        std.os.windows.EXCEPTION_ACCESS_VIOLATION => {
            const fault_address = info.ExceptionRecord.ExceptionInformation[1];

            VAS.patch_access(fault_address, @intFromPtr(GLOBAL_VIRTUAL_ADDRESS_SPACE_BASE), 0x1_0000_0000, &info.ContextRecord.Rip) catch |err| {
                log.err("Failed to patch FastMem access @{X}: {t}", .{ fault_address, err });
                if (std.os.windows.kernel32.RemoveVectoredExceptionHandler(GLOBAL_EXCEPTION_HANDLER.?) == 0)
                    log.err(termcolor.red("Call to RemoveVectoredExceptionHandler failed.\n"), .{});
                return std.os.windows.EXCEPTION_CONTINUE_SEARCH;
            };

            return windows.EXCEPTION_CONTINUE_EXECUTION;
        },
        else => return std.os.windows.EXCEPTION_CONTINUE_SEARCH,
    }
    return std.os.windows.EXCEPTION_CONTINUE_SEARCH;
}
