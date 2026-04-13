const std = @import("std");
const builtin = @import("builtin");

pub fn allocate_executable(allocator: std.mem.Allocator, size: usize) ![]align(std.heap.page_size_min) u8 {
    const r = try allocator.alignedAlloc(u8, .fromByteUnits(std.heap.page_size_min), size);
    switch (std.posix.errno(std.posix.system.mprotect(r.ptr, size, .{ .READ = true, .WRITE = true, .EXEC = true }))) {
        .SUCCESS => {},
        .NOMEM => return error.OutOfMemory,
        else => |err| return std.posix.unexpectedErrno(err),
    }
    return r;
}

pub fn virtual_alloc(comptime element_type: type, count: usize) ![]element_type {
    switch (builtin.os.tag) {
        .windows => {
            var size: std.os.windows.SIZE_T = @sizeOf(element_type) * count;
            var base_addr: ?*anyopaque = null;
            if (std.os.windows.ntdll.NtAllocateVirtualMemory(
                std.os.windows.GetCurrentProcess(),
                @ptrCast(&base_addr),
                0,
                &size,
                .{ .RESERVE = true, .COMMIT = true },
                .{ .READWRITE = true },
            ) != .SUCCESS) return error.NtAllocateVirtualMemoryError;
            return @as([*]element_type, @ptrCast(@alignCast(base_addr)))[0..count];
        },
        .linux => {
            const memory = try std.posix.mmap(
                null,
                @sizeOf(element_type) * count,
                std.posix.PROT.READ | std.posix.PROT.WRITE,
                .{ .TYPE = .PRIVATE, .ANONYMOUS = true },
                -1,
                0,
            );
            return @as([*]element_type, @ptrCast(@alignCast(memory)))[0..count];
        },
        else => @compileError("Unsupported OS."),
    }
}

pub fn virtual_dealloc(memory: anytype) void {
    if (@typeInfo(@TypeOf(memory)) != .pointer or @typeInfo(@TypeOf(memory)).pointer.size != .slice) @compileError("virtual_dealloc expects a slice.");

    switch (builtin.os.tag) {
        .windows => {
            var base_addr: ?*anyopaque = memory.ptr;
            var size: std.os.windows.SIZE_T = 0;
            const status = std.os.windows.ntdll.NtFreeVirtualMemory(std.os.windows.GetCurrentProcess(), @ptrCast(&base_addr), &size, .{ .RELEASE = true });
            if (status != .SUCCESS) std.log.err("Failed to free virtual memory: {t}", .{status});
        },
        .linux => std.posix.munmap(@as([*]align(std.heap.page_size_min) const u8, @ptrCast(@alignCast(memory.ptr)))[0 .. memory.len * @sizeOf(std.meta.Elem(@TypeOf(memory)))]),
        else => @compileError("Unsupported OS."),
    }
}

// Zero-out a virtual allocation
pub fn reset_virtual_alloc(memory: anytype) !void {
    if (@typeInfo(@TypeOf(memory)) != .pointer or @typeInfo(@TypeOf(memory)).pointer.size != .slice) @compileError("reset_virtual_alloc expects a slice.");
    var byte_size = memory.len * @sizeOf(std.meta.Elem(@TypeOf(memory)));
    switch (builtin.os.tag) {
        .windows => {
            var base_addr: ?*anyopaque = memory.ptr;
            if (std.os.windows.ntdll.NtFreeVirtualMemory(std.os.windows.GetCurrentProcess(), @ptrCast(&base_addr), &byte_size, .{ .DECOMMIT = true }) != .SUCCESS)
                return error.NtFreeVirtualMemoryError;
            if (std.os.windows.ntdll.NtAllocateVirtualMemory(
                std.os.windows.GetCurrentProcess(),
                @ptrCast(&base_addr),
                0,
                &byte_size,
                .{ .COMMIT = true },
                .{ .READWRITE = true },
            ) != .SUCCESS)
                return error.NtAllocateVirtualMemoryError;
        },
        else => {
            // NOTE: madvise 'dontneed' could be faster and should reliably zero out the memory on private anonymous mappings on Linux, if I believe what I read here and there online.
            //       However this isn't standard or portable, maybe I should just use the fixed mmap fallback directly.
            std.posix.madvise(@ptrCast(@alignCast(memory.ptr)), byte_size, std.posix.MADV.DONTNEED) catch |madv_err| {
                std.log.warn("Failed to madvise: {t}. Fallback to mmap.", .{madv_err});
                const remmaped = try std.posix.mmap(
                    @ptrCast(@alignCast(memory.ptr)),
                    byte_size,
                    std.posix.PROT.READ | std.posix.PROT.WRITE,
                    .{ .TYPE = .PRIVATE, .ANONYMOUS = true, .FIXED = true },
                    -1,
                    0,
                );
                std.debug.assert(remmaped.ptr == @as([*]align(std.heap.page_size_min) u8, @ptrCast(@alignCast(memory.ptr))));
            };
        },
    }
}
