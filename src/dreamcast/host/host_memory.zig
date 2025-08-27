const std = @import("std");
const builtin = @import("builtin");

pub fn allocate_executable(allocator: std.mem.Allocator, size: usize) ![]align(std.heap.page_size_min) u8 {
    const r = try allocator.alignedAlloc(u8, .fromByteUnits(std.heap.page_size_min), size);
    try std.posix.mprotect(r, std.posix.PROT.READ | std.posix.PROT.WRITE | std.posix.PROT.EXEC);
    return r;
}

pub fn virtual_alloc(comptime element_type: type, count: usize) ![]element_type {
    switch (builtin.os.tag) {
        .windows => {
            const memory = try std.os.windows.VirtualAlloc(
                null,
                @sizeOf(element_type) * count,
                std.os.windows.MEM_RESERVE | std.os.windows.MEM_COMMIT,
                std.os.windows.PAGE_READWRITE,
            );
            return @as([*]element_type, @ptrCast(@alignCast(memory)))[0..count];
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
        .windows => std.os.windows.VirtualFree(memory.ptr, 0, std.os.windows.MEM_RELEASE),
        .linux => std.posix.munmap(@as([*]align(std.heap.page_size_min) const u8, @ptrCast(@alignCast(memory.ptr)))[0 .. memory.len * @sizeOf(std.meta.Elem(@TypeOf(memory)))]),
        else => @compileError("Unsupported OS."),
    }
}

// Zero-out a virtual allocation
pub fn reset_virtual_alloc(memory: anytype) !void {
    if (@typeInfo(@TypeOf(memory)) != .pointer or @typeInfo(@TypeOf(memory)).pointer.size != .slice) @compileError("reset_virtual_alloc expects a slice.");
    const byte_size = memory.len * @sizeOf(std.meta.Elem(@TypeOf(memory)));
    switch (builtin.os.tag) {
        .windows => {
            std.os.windows.VirtualFree(memory.ptr, byte_size, std.os.windows.MEM_DECOMMIT);
            std.debug.assert(try std.os.windows.VirtualAlloc(
                memory.ptr,
                byte_size,
                std.os.windows.MEM_COMMIT,
                std.os.windows.PAGE_READWRITE,
            ) == @as(*anyopaque, @ptrCast(memory.ptr)));
        },
        else => {
            // NOTE: madvise 'dontneed' could be faster and should reliably zero out the memory on private anonymous mappings on Linux, if I believe what I read here and there online.
            //       However this isn't standard or portable, maybe I should just use the fixed mmap fallback directly.
            std.posix.madvise(@ptrCast(@alignCast(memory.ptr)), byte_size, std.posix.MADV.DONTNEED) catch |madv_err| {
                std.log.warn("Failed to madvise: {s}. Fallback to mmap.", .{@errorName(madv_err)});
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
