const std = @import("std");
const builtin = @import("builtin");

const windows = @import("windows.zig");

const log = std.log.scoped(.memory_mapped_file);

const View = if (builtin.os.tag == .windows) std.os.windows.HANDLE else []align(std.heap.page_size_min) const u8;

file: if (builtin.os.tag == .windows) std.os.windows.HANDLE else std.fs.File,
mapping_handle: if (builtin.os.tag == .windows) std.os.windows.HANDLE else void,
views: std.ArrayList(View),

pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !@This() {
    var self: @This() = .{
        .file = undefined,
        .mapping_handle = undefined,
        .views = try .initCapacity(allocator, 1),
    };
    if (builtin.os.tag != .windows) {
        self.file = std.fs.cwd().openFile(filepath, .{}) catch {
            log.err("File not found: {s}", .{filepath});
            return error.TrackFileNotFound;
        };
    } else {
        self.file = std.os.windows.INVALID_HANDLE_VALUE;
        self.mapping_handle = std.os.windows.INVALID_HANDLE_VALUE;

        var track_file_abs_path_buffer: [std.fs.max_path_bytes + 1]u8 = @splat(0);
        const track_file_abs_path = try std.fs.cwd().realpath(filepath, &track_file_abs_path_buffer);
        const file_path_w = try std.os.windows.sliceToPrefixedFileW(null, track_file_abs_path);

        self.file = try std.os.windows.OpenFile(file_path_w.span(), .{
            .access_mask = std.os.windows.GENERIC_READ | std.os.windows.SYNCHRONIZE,
            .creation = std.os.windows.FILE_OPEN_IF,
        });

        self.mapping_handle = windows.CreateFileMappingA(self.file, null, std.os.windows.PAGE_READONLY, 0, 0, null) orelse return error.FileMapError;
    }
    return self;
}

pub fn deinit(self: *@This()) void {
    if (builtin.os.tag != .windows) {
        for (self.views.items) |view| {
            std.posix.munmap(view);
        }
        self.file.close();
    } else {
        for (self.views.items) |view| {
            if (windows.UnmapViewOfFile(view) == 0)
                log.err("UnmapViewOfFile failed: {any}\n", .{std.os.windows.GetLastError()});
        }
        if (self.mapping_handle != std.os.windows.INVALID_HANDLE_VALUE)
            std.os.windows.CloseHandle(self.mapping_handle);
        if (self.file != std.os.windows.INVALID_HANDLE_VALUE)
            std.os.windows.CloseHandle(self.file);
    }
    self.views.deinit();
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
        try self.views.append(r);
        return r[adjustment..];
    } else {
        var map_to_end = size == 0;
        const alignment = 64 * 1024;
        const aligned_offset = std.mem.alignBackward(u64, offset, alignment);
        const adjustment = offset - aligned_offset;
        const aligned_size = if (map_to_end) 0 else std.mem.alignForward(u64, size + adjustment, alignment);

        var ptr_or_null = windows.MapViewOfFile(self.mapping_handle, std.os.windows.SECTION_MAP_READ, @truncate(aligned_offset >> 32), @truncate(aligned_offset), aligned_size);
        if (ptr_or_null == null and !map_to_end) {
            switch (std.os.windows.GetLastError()) {
                // Try mapping to the end, instead of an aligned size.
                .ACCESS_DENIED => {
                    ptr_or_null = windows.MapViewOfFile(self.mapping_handle, std.os.windows.SECTION_MAP_READ, @truncate(aligned_offset >> 32), @truncate(aligned_offset), 0);
                    map_to_end = true;
                },
                else => {},
            }
        }

        if (ptr_or_null) |ptr| {
            try self.views.append(ptr);

            const final_size = sz: {
                if (map_to_end) {
                    var info: std.os.windows.MEMORY_BASIC_INFORMATION = undefined;
                    _ = try std.os.windows.VirtualQuery(ptr, &info, @sizeOf(std.os.windows.MEMORY_BASIC_INFORMATION));
                    break :sz if (size > 0) @min(size, info.RegionSize) else info.RegionSize;
                } else {
                    break :sz size;
                }
            };
            return @as([*]u8, @ptrCast(ptr))[adjustment .. adjustment + final_size];
        } else {
            log.err("MapViewOfFile (offset: {X:0>8}, size: {X:0>8}) failed: {any}", .{ aligned_offset, aligned_size, std.os.windows.GetLastError() });
            return error.MapViewOfFileError;
        }
    }
}
