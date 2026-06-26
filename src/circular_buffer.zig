const std = @import("std");
const builtin = @import("builtin");

pub fn CircularBuffer(comptime T: type) type {
    if (std.heap.pageSize() % @sizeOf(T) != 0)
        @compileError(std.fmt.comptimePrint("@sizeOf({}) must cleanly divide the page size ({d})", .{ T, std.heap.pageSize() }));
    return struct {
        capacity: usize,
        buffer: []align(std.heap.page_size_min) T,

        read_offset: usize = 0,
        write_offset: usize = 0,

        /// `capacity` * sizeof(T) must be a multiple of page size on Linux (most likely 4k) or allocation granularity on Windows (most likely 64k).
        pub fn init(capacity: usize) !@This() {
            const byte_capacity = capacity * @sizeOf(T);
            if (byte_capacity % allocation_granularity() != 0) return error.InvalidSize;
            switch (builtin.os.tag) {
                .windows => {
                    const fd = CreateFileMappingA(
                        std.os.windows.INVALID_HANDLE_VALUE,
                        null,
                        PAGE_READWRITE,
                        0,
                        @intCast(byte_capacity),
                        null,
                    ) orelse return error.CreateFileMappingError;
                    defer _ = std.os.windows.CloseHandle(fd);

                    // FIXME: Use placeholders instead of the retry loop...
                    for (0..8) |_| {
                        const reserve = VirtualAlloc(null, 2 * byte_capacity, MEM_RESERVE, PAGE_NOACCESS) orelse return error.VirtualAllocError;
                        _ = VirtualFree(reserve, 0, MEM_RELEASE);

                        const first = MapViewOfFileEx(fd, FILE_MAP_ALL_ACCESS, 0, 0, byte_capacity, reserve) orelse continue;
                        _ = MapViewOfFileEx(fd, FILE_MAP_ALL_ACCESS, 0, 0, byte_capacity, &@as([*]u8, @ptrCast(reserve))[byte_capacity]) orelse {
                            _ = UnmapViewOfFile(first);
                            continue;
                        };

                        return .{
                            .capacity = capacity,
                            .buffer = @alignCast(std.mem.bytesAsSlice(T, @as([*]u8, @ptrCast(reserve))[0 .. 2 * byte_capacity])),
                        };
                    }
                    std.log.err("Failed to allocate Circular Buffer: {}", .{std.os.windows.GetLastError()});
                    return error.MapViewOfFileExError;
                },
                .linux => {
                    const fd = try std.posix.memfd_create("Circular Buffer", 0);
                    defer _ = std.os.linux.close(fd);
                    switch (std.os.linux.errno(std.os.linux.ftruncate(fd, @intCast(byte_capacity)))) {
                        .SUCCESS => {},
                        else => return error.ftruncateError,
                    }
                    const virtual: []align(std.heap.page_size_min) u8 = @alignCast(try std.posix.mmap(null, 2 * byte_capacity, .{}, .{ .TYPE = .PRIVATE, .ANONYMOUS = true }, -1, 0));
                    errdefer std.posix.munmap(virtual);
                    _ = try std.posix.mmap(@ptrCast(@alignCast(virtual)), byte_capacity, .{ .READ = true, .WRITE = true }, .{ .TYPE = .SHARED, .FIXED = true }, fd, 0);
                    _ = try std.posix.mmap(@ptrCast(@alignCast(virtual[byte_capacity..])), byte_capacity, .{ .READ = true, .WRITE = true }, .{ .TYPE = .SHARED, .FIXED = true }, fd, 0);
                    return .{
                        .capacity = capacity,
                        .buffer = std.mem.bytesAsSlice(T, virtual),
                    };
                },
                else => @compileError("Unsupported OS"),
            }
        }

        /// Initializes the buffer with at least `capacity` elements.
        /// Actual capacity may be greater to be a multiple of the required alignement.
        pub fn initAtLeast(capacity: usize) !@This() {
            const byte_capacity = capacity * @sizeOf(T);
            return init(std.mem.alignForward(usize, byte_capacity, allocation_granularity()) / @sizeOf(T));
        }

        pub fn deinit(self: *@This()) void {
            switch (builtin.os.tag) {
                .windows => {
                    const byte_capacity = self.capacity * @sizeOf(T);
                    _ = UnmapViewOfFile(@ptrCast(self.buffer.ptr));
                    _ = UnmapViewOfFile(@ptrFromInt(@intFromPtr(self.buffer.ptr) + byte_capacity));
                },
                .linux => std.posix.munmap(std.mem.sliceAsBytes(self.buffer)),
                else => @compileError("Unsupported OS"),
            }
        }

        pub fn clear(self: *@This()) void {
            self.read_offset = 0;
            self.write_offset = 0;
        }

        pub fn push(self: *@This(), data: T) void {
            return self.pushSlice(&[1]T{data}) catch unreachable;
        }

        pub fn pushSlice(self: *@This(), data: []const T) !void {
            if (data.len > self.capacity) return error.BufferOverflow;
            @memcpy(self.buffer[self.write_offset % self.capacity ..][0..data.len], data);
            self.write_offset += data.len;
            if (self.write_offset > self.capacity) {
                self.read_offset = self.write_offset - self.capacity;
            }
            if (self.write_offset > 2 * self.capacity) {
                self.write_offset -= self.capacity;
                self.read_offset -= self.capacity;
            }
        }

        pub fn view(self: *const @This()) []const T {
            return self.buffer[self.read_offset..self.write_offset];
        }

        fn allocation_granularity() usize {
            switch (builtin.os.tag) {
                .windows => {
                    var system_info: SYSTEM_INFO = undefined;
                    GetSystemInfo(&system_info);
                    return system_info.dwAllocationGranularity;
                },
                .linux => return std.heap.pageSize(),
                else => @compileError("Unsupported OS"),
            }
        }
    };
}

pub const FILE_MAP_ALL_ACCESS: std.os.windows.DWORD = ((0x000F0000) | 0x0001 | 0x0002 | 0x0004 | 0x0008 | 0x0010);
pub const MEM_RESERVE: std.os.windows.DWORD = 0x00002000;
pub const MEM_RELEASE: std.os.windows.DWORD = 0x00008000;
pub const PAGE_NOACCESS: std.os.windows.DWORD = 0x01;
pub const PAGE_READWRITE: std.os.windows.DWORD = 0x04;

pub const SYSTEM_INFO = extern struct {
    id: extern union {
        dwOemId: std.os.windows.DWORD,
        other: extern struct {
            wProcessorArchitecture: std.os.windows.WORD,
            wReserved: std.os.windows.WORD,
        },
    },
    dwPageSize: std.os.windows.DWORD,
    lpMinimumApplicationAddress: std.os.windows.LPVOID,
    lpMaximumApplicationAddress: std.os.windows.LPVOID,
    dwActiveProcessorMask: std.os.windows.DWORD_PTR,
    dwNumberOfProcessors: std.os.windows.DWORD,
    dwProcessorType: std.os.windows.DWORD,
    dwAllocationGranularity: std.os.windows.DWORD,
    wProcessorLevel: std.os.windows.WORD,
    wProcessorRevision: std.os.windows.WORD,
};

pub extern "kernel32" fn GetSystemInfo(
    lpSystemInfo: *SYSTEM_INFO,
) callconv(.winapi) void;

pub extern "kernel32" fn CreateFileMappingA(
    hFile: std.os.windows.HANDLE,
    lpFileMappingAttributes: ?*std.os.windows.SECURITY_ATTRIBUTES,
    flProtect: std.os.windows.DWORD,
    dwMaximumSizeHigh: std.os.windows.DWORD,
    dwMaximumSizeLow: std.os.windows.DWORD,
    lpName: ?std.os.windows.LPCSTR,
) callconv(.winapi) ?std.os.windows.HANDLE;

pub extern "kernel32" fn MapViewOfFileEx(
    hFileMappingObject: std.os.windows.HANDLE,
    dwDesiredAccess: std.os.windows.DWORD,
    dwFileOffsetHigh: std.os.windows.DWORD,
    dwFileOffsetLow: std.os.windows.DWORD,
    dwNumberOfBytesToMap: std.os.windows.SIZE_T,
    lpBaseAddress: ?std.os.windows.LPVOID,
) callconv(.winapi) ?std.os.windows.LPVOID;

pub extern "kernel32" fn UnmapViewOfFile(
    lpBaseAddress: std.os.windows.LPCVOID,
) callconv(.winapi) std.os.windows.BOOL;
pub extern "kernel32" fn VirtualAlloc(
    lpAddress: ?std.os.windows.LPVOID,
    dwSize: std.os.windows.SIZE_T,
    flAllocationType: std.os.windows.DWORD,
    flProtect: std.os.windows.DWORD,
) callconv(.winapi) ?std.os.windows.LPVOID;

pub extern "kernel32" fn VirtualFree(
    lpAddress: std.os.windows.LPVOID,
    dwSize: std.os.windows.SIZE_T,
    dwFreeType: std.os.windows.DWORD,
) callconv(.winapi) std.os.windows.BOOL;

test "Circular Buffer" {
    var cb = try CircularBuffer(u64).init(std.heap.pageSize() / @sizeOf(u64));
    defer cb.deinit();

    for (0..cb.capacity) |i| {
        cb.push(i);
        try std.testing.expect(cb.view().len == i + 1);
        try expectOrdered(u64, &cb);
    }
    for (cb.capacity..4 * cb.capacity) |i| {
        cb.push(i);
        try std.testing.expect(cb.view().len == cb.capacity);
        try expectOrdered(u64, &cb);
    }
}

test "Circular Buffer - Slice length 2" {
    var cb = try CircularBuffer(u64).init(std.heap.pageSize() / @sizeOf(u64));
    defer cb.deinit();

    for (0..cb.capacity / 2) |i| {
        try cb.pushSlice(&[_]u64{ 2 * i, 2 * i + 1 });
        try std.testing.expect(cb.view().len == 2 * (i + 1));
        try expectOrdered(u64, &cb);
    }
    for (cb.capacity / 2..4 * cb.capacity / 2) |i| {
        try cb.pushSlice(&[_]u64{ 2 * i, 2 * i + 1 });
        try std.testing.expect(cb.view().len == cb.capacity);
        try expectOrdered(u64, &cb);
    }
}

test "Circular Buffer - Slice length 3" {
    var cb = try CircularBuffer(u64).init(std.heap.pageSize() / @sizeOf(u64));
    defer cb.deinit();

    for (0..cb.capacity / 3) |i| {
        try cb.pushSlice(&[_]u64{ 3 * i, 3 * i + 1, 3 * i + 2 });
        try std.testing.expect(cb.view().len == 3 * (i + 1));
        try expectOrdered(u64, &cb);
    }
    for (cb.capacity / 3..4 * cb.capacity / 3) |i| {
        try cb.pushSlice(&[_]u64{ 3 * i, 3 * i + 1, 3 * i + 2 });
        try std.testing.expect(cb.view().len == cb.capacity);
        try expectOrdered(u64, &cb);
    }
}

fn expectOrdered(comptime T: type, cb: *const CircularBuffer(T)) !void {
    var last_val = cb.view()[0];
    for (cb.view()[1..]) |val| {
        try std.testing.expect(last_val + 1 == val);
        last_val = val;
    }
}
