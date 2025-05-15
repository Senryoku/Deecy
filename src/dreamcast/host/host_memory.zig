const std = @import("std");
const builtin = @import("builtin");

pub fn allocate_executable(allocator: std.mem.Allocator, size: usize) ![]align(std.heap.page_size_min) u8 {
    const r = try allocator.alignedAlloc(u8, std.heap.page_size_min, size);
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
            return @as([*]element_type, @alignCast(@ptrCast(memory)))[0..count];
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
            return @as([*]element_type, @alignCast(@ptrCast(memory)))[0..count];
        },
        else => @compileError("Unsupported OS."),
    }
}

pub fn virtual_dealloc(memory: anytype) void {
    if (@typeInfo(@TypeOf(memory)) != .pointer or @typeInfo(@TypeOf(memory)).pointer.size != .slice) @compileError("virtual_dealloc expects a slice.");

    switch (builtin.os.tag) {
        .windows => std.os.windows.VirtualFree(memory.ptr, 0, std.os.windows.MEM_RELEASE),
        .linux => std.posix.munmap(@as([*]align(std.heap.page_size_min) const u8, @alignCast(@ptrCast(memory.ptr)))[0 .. memory.len * @sizeOf(std.meta.Elem(@TypeOf(memory)))]),
        else => @compileError("Unsupported OS."),
    }
}
