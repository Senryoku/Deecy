const std = @import("std");

/// Combines multiple WGSL shader files into a single string.
/// Comptime in release builds, loads from disc at runtime in debug, assuming CWD is the root of the source tree.
pub fn load(comptime options: []const u8, comptime names: anytype) []const u8 {
    if (@import("builtin").mode == .Debug) {
        // Load from disc at runtime in debug.
        // NOTE: Leaky on purpose, this is only used for debugging, and should only be called on startup.
        const allocator = std.heap.c_allocator;
        var alloc_writer: std.Io.Writer.Allocating = .init(allocator);
        alloc_writer.writer.writeAll(options ++ "\n") catch std.debug.panic("shader.load: OOM.", .{});
        var buffer: [256]u8 = undefined;
        inline for (names) |name| {
            const file = std.fs.cwd().openFile("./src/shaders/" ++ name ++ ".wgsl", .{}) catch std.debug.panic("shader.load: Failed to open '{s}'.", .{name});
            defer file.close();
            var reader = file.reader(&buffer);
            _ = reader.interface.streamRemaining(&alloc_writer.writer) catch std.debug.panic("shader.load: OOM.", .{});
        }
        return alloc_writer.toOwnedSlice() catch std.debug.panic("shader.load: OOM.", .{});
    } else {
        // Comptime version for release builds.
        comptime var r: []const u8 = options ++ "\n";
        inline for (names) |name| {
            r = r ++ @embedFile("./shaders/" ++ name ++ ".wgsl");
        }
        return r;
    }
}
