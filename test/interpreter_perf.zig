const std = @import("std");
const builtin = @import("builtin");
const lz4 = @import("lz4");

const DreamcastModule = @import("dreamcast");
const Dreamcast = DreamcastModule.Dreamcast;

// Very quick and dirty perf test to avoid doing random optimisation completely blindly.

pub const std_options: std.Options = .{
    .log_level = .err,
};

// Sharing a single file accross module is a pain, easier to just paste that in for now. Keep it in sync.
pub const SaveStateHeader = extern struct {
    const Signature: [8]u8 = .{ 'D', 'E', 'E', 'C', 'Y', 'S', 'A', 'V' };
    const Version: u32 = 5;

    signature: [Signature.len]u8 = Signature,
    version: u16 = Version,
    _reserved: [2]u8 = @splat(0),
    uncompressed_size: u32,
    compressed_size: u32,

    pub fn validate(self: *const SaveStateHeader) !void {
        if (!std.mem.eql(u8, &self.signature, &SaveStateHeader.Signature)) return error.InvalidSaveState;
        if (self.version != SaveStateHeader.Version) return error.InvalidSaveStateVersion;
        if (!std.mem.eql(u8, &self._reserved, &[2]u8{ 0, 0 })) return error.InvalidSaveState;
        if (self.uncompressed_size == 0) return error.InvalidSaveState;
        if (self.compressed_size == 0) return error.InvalidSaveState;
    }
};

pub fn load_state(dc: *Dreamcast, path: []const u8) !void {
    var file = try std.fs.cwd().openFile(path, .{});
    defer file.close();

    var header: SaveStateHeader = undefined;
    const header_size = try file.read(std.mem.asBytes(&header));
    if (header_size != @sizeOf(SaveStateHeader)) return error.InvalidSaveState;
    try header.validate();

    const compressed = try file.readToEndAllocOptions(dc._allocator, 32 * 1024 * 1024, header.compressed_size, 8, null);
    defer dc._allocator.free(compressed);

    if (header.compressed_size != compressed.len) return error.UnexpectedSaveStateSize;

    const decompressed = try lz4.Standard.decompress(dc._allocator, compressed, header.uncompressed_size);
    defer dc._allocator.free(decompressed);

    var uncompressed_stream = std.io.fixedBufferStream(decompressed);
    var reader = uncompressed_stream.reader();

    try dc.reset();

    _ = try dc.deserialize(&reader);
}

// Expects pairs of game path and save state path as arguments
pub fn main() !void {
    const cycles_target = 1 * 200_000_000;
    const max_instructions = 32;

    var total_time: u64 = 0;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    var allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    _ = args.skip();

    while (args.next()) |game| {
        var dc = try Dreamcast.create(allocator);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        const save_state_path = args.next().?;

        dc.gdrom.disc = try .init(game, allocator);
        try load_state(dc, save_state_path);
        const start = try std.time.Instant.now();
        var cycles: u32 = 0;
        while (cycles < cycles_target) {
            cycles += try dc.tick(max_instructions);
        }
        const elapsed = (try std.time.Instant.now()).since(start);
        total_time += elapsed;
        std.debug.print("[Interpreter] {s}: Ran {d} cycles in {} ms\n", .{ dc.gdrom.disc.?.get_product_name() orelse "Unknown", cycles, elapsed / std.time.ns_per_ms });
    }

    std.debug.print("[Interpreter] Total: {} ms\n", .{total_time / std.time.ns_per_ms});
}
