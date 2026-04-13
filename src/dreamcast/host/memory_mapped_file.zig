const std = @import("std");
const log = std.log.scoped(.memory_mapped_file);
const builtin = @import("builtin");

const windows = @import("windows.zig");

const View = if (builtin.os.tag == .windows) std.os.windows.HANDLE else []align(std.heap.page_size_min) const u8;

file: std.Io.File,
mapping_handle: if (builtin.os.tag == .windows) std.os.windows.HANDLE else void,
views: std.ArrayList(View) = .empty,
allocator: std.mem.Allocator,
io: std.Io,

pub fn init(allocator: std.mem.Allocator, io: std.Io, filepath: []const u8) !@This() {
    var self: @This() = .{
        .file = std.Io.Dir.cwd().openFile(io, filepath, .{}) catch {
            log.err("File not found: {s}", .{filepath});
            return error.TrackFileNotFound;
        },
        .mapping_handle = undefined,
        .allocator = allocator,
        .io = io,
    };
    if (builtin.os.tag == .windows) {
        self.mapping_handle = std.os.windows.INVALID_HANDLE_VALUE;
        self.mapping_handle = windows.CreateFileMappingA(self.file.handle, null, windows.PAGE_READONLY, 0, 0, null) orelse return error.FileMapError;
    }
    return self;
}

pub fn deinit(self: *@This()) void {
    if (builtin.os.tag != .windows) {
        for (self.views.items) |view| {
            std.posix.munmap(view);
        }
        self.file.close(self.io);
    } else {
        for (self.views.items) |view| {
            if (windows.UnmapViewOfFile(view) == .FALSE)
                log.err("UnmapViewOfFile failed: {any}\n", .{std.os.windows.GetLastError()});
        }
        if (self.mapping_handle != std.os.windows.INVALID_HANDLE_VALUE)
            std.os.windows.CloseHandle(self.mapping_handle);
        self.file.close(self.io);
    }
    self.views.deinit(self.allocator);
}

pub fn create_full_view(self: *@This()) ![]u8 {
    return try self.create_view(0, if (builtin.os.tag != .windows) (try self.file.stat()).size else 0);
}

pub fn create_view(self: *@This(), offset: u64, size: u64) ![]u8 {
    if (builtin.os.tag != .windows) {
        const alignment = std.heap.page_size_min;
        const aligned_offset = std.mem.alignBackward(u64, offset, alignment);
        const adjustment = offset - aligned_offset;
        const adjusted_size = size + adjustment;
        const r = try std.posix.mmap(null, adjusted_size, std.posix.PROT.READ, .{ .TYPE = .SHARED }, self.file.handle, aligned_offset);
        errdefer std.posix.munmap(r);
        try self.views.append(self.allocator, r);
        return r[adjustment..];
    } else {
        var map_to_end = size == 0;
        const alignment = 64 * 1024;
        const aligned_offset = std.mem.alignBackward(u64, offset, alignment);
        const adjustment = offset - aligned_offset;
        const aligned_size = if (map_to_end) 0 else std.mem.alignForward(u64, size + adjustment, alignment);

        var ptr_or_null = windows.MapViewOfFile(self.mapping_handle, windows.SECTION_MAP_READ, @truncate(aligned_offset >> 32), @truncate(aligned_offset), aligned_size);
        if (ptr_or_null == null and !map_to_end) {
            switch (std.os.windows.GetLastError()) {
                // Try mapping to the end, instead of an aligned size.
                .ACCESS_DENIED => {
                    ptr_or_null = windows.MapViewOfFile(self.mapping_handle, windows.SECTION_MAP_READ, @truncate(aligned_offset >> 32), @truncate(aligned_offset), 0);
                    map_to_end = true;
                },
                else => {},
            }
        }

        if (ptr_or_null) |ptr| {
            try self.views.append(self.allocator, ptr);

            const final_size = sz: {
                if (map_to_end) {
                    var info: windows.MEMORY_BASIC_INFORMATION = undefined;
                    if (windows.VirtualQuery(ptr, &info, @sizeOf(windows.MEMORY_BASIC_INFORMATION)) == 0) {
                        log.err("VirtualQuery failed: {}", .{std.os.windows.GetLastError()});
                        return error.VirtualQueryError;
                    }
                    break :sz if (size > 0) @min(size, info.RegionSize) else info.RegionSize;
                } else {
                    break :sz size;
                }
            };
            return @as([*]u8, @ptrCast(ptr))[adjustment .. adjustment + final_size];
        } else {
            log.err("MapViewOfFile (offset: {X:0>8}, size: {X:0>8}) failed: {}", .{ aligned_offset, aligned_size, std.os.windows.GetLastError() });
            return error.MapViewOfFileError;
        }
    }
}

pub fn file_size(self: *const @This()) !u64 {
    return (try self.file.stat(self.io)).size;
}
