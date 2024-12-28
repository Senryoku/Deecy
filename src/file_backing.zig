const std = @import("std");
const builtin = @import("builtin");

const windows = @import("windows.zig");

const View = if (builtin.os.tag == .windows) std.os.windows.HANDLE else []align(std.mem.page_size) const u8;

file: if (builtin.os.tag == .windows) std.os.windows.HANDLE else std.fs.File,
mapping_handle: if (builtin.os.tag == .windows) std.os.windows.HANDLE else void,
views: std.ArrayList(View),

pub fn init(filepath: []const u8, allocator: std.mem.Allocator) !@This() {
    var self: @This() = .{
        .file = undefined,
        .mapping_handle = undefined,
        .views = std.ArrayList(View).init(allocator),
    };
    if (builtin.os.tag != .windows) {
        self.file = std.fs.cwd().openFile(filepath, .{}) catch {
            std.debug.print("File not found: {s}\n", .{filepath});
            return error.TrackFileNotFound;
        };
    } else {
        var track_file_abs_path_buffer: [std.fs.max_path_bytes + 1]u8 = .{0} ** (std.fs.max_path_bytes + 1);
        const track_file_abs_path = try std.fs.cwd().realpath(filepath, &track_file_abs_path_buffer);
        const file_path_w = try std.os.windows.sliceToPrefixedFileW(null, track_file_abs_path);

        self.file = try std.os.windows.OpenFile(file_path_w.span(), .{
            .access_mask = std.os.windows.GENERIC_READ | std.os.windows.SYNCHRONIZE,
            .creation = std.os.windows.FILE_OPEN,
        });
        errdefer std.os.windows.CloseHandle(self.file);

        self.mapping_handle = windows.CreateFileMappingA(self.file, null, std.os.windows.PAGE_READONLY, 0, 0, null) orelse return error.FileMapError;
        errdefer std.os.windows.CloseHandle(self.mapping_handle);
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
                std.debug.print("UnmapViewOfFile failed: {any}\n", .{std.os.windows.GetLastError()});
        }
        std.os.windows.CloseHandle(self.mapping_handle);
        std.os.windows.CloseHandle(self.file);
    }
    self.views.deinit();
}

pub fn create_full_view(self: *@This()) ![]align(std.mem.page_size) const u8 {
    return try self.create_view(0, if (builtin.os.tag != .windows) (try self.file.stat()).size else 0);
}

pub fn create_view(self: *@This(), offset: u64, size: u64) ![]align(std.mem.page_size) const u8 {
    if (builtin.os.tag != .windows) {
        const r = try std.posix.mmap(null, size, std.posix.PROT.READ, .{ .TYPE = .SHARED }, self.platform_specific.file.handle, offset);
        errdefer std.posix.munmap(r);
        try self.views.append(r);
        return r;
    } else {
        const aligned_offset = std.mem.alignBackward(u64, offset, std.mem.page_size);
        const adjustment = offset - aligned_offset;
        const aligned_size = std.mem.alignForward(u64, size, std.mem.page_size);

        // FIXME: CDI: os.windows.win32error.Win32Error.MAPPED_ALIGNMENT
        const ptr = windows.MapViewOfFile(self.mapping_handle, std.os.windows.SECTION_MAP_READ, @truncate(aligned_offset >> 32), @truncate(aligned_offset), aligned_size) orelse {
            std.debug.print("MapViewOfFile (offset: {X:0>8}, size: {X:0>8}) failed: {any}\n", .{ aligned_offset, aligned_size, std.os.windows.GetLastError() });
            return error.MapViewOfFileError;
        };
        errdefer _ = windows.UnmapViewOfFile(ptr);

        try self.views.append(ptr);

        var info: std.os.windows.MEMORY_BASIC_INFORMATION = undefined;
        _ = try std.os.windows.VirtualQuery(ptr, &info, @sizeOf(std.os.windows.MEMORY_BASIC_INFORMATION));
        return @alignCast(@as([*]const u8, @ptrCast(ptr))[adjustment .. info.RegionSize + adjustment]);
    }
}
