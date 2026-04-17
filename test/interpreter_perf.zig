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

pub fn load_state(dc: *Dreamcast, io: std.Io, path: []const u8) !void {
    const file = try std.Io.Dir.cwd().readFileAllocOptions(io, path, dc._allocator, .limited(32 * 1024 * 1024), .@"8", null);
    defer dc._allocator.free(file);

    const header = std.mem.bytesToValue(SaveStateHeader, file[0..@sizeOf(SaveStateHeader)]);
    try header.validate();
    try header.validate();

    const compressed = file[@sizeOf(SaveStateHeader)..];
    if (header.compressed_size != compressed.len) return error.UnexpectedSaveStateSize;

    const decompressed = try lz4.Standard.decompress(dc._allocator, compressed, header.uncompressed_size);
    defer dc._allocator.free(decompressed);

    var reader = std.Io.Reader.fixed(decompressed);

    try dc.reset();

    _ = try dc.deserialize(&reader);
}

// Expects pairs of game path and save state path as arguments
pub fn main(init: std.process.Init) !void {
    const cycles_target = 1 * 200_000_000;
    const max_instructions = 32;

    var total_time: i64 = 0;

    var allocator = init.gpa;
    const io = init.io;

    var args = try init.minimal.args.iterateAllocator(allocator);
    defer args.deinit();

    _ = args.skip();

    while (args.next()) |game| {
        var dc = try Dreamcast.create(allocator, io);
        defer {
            dc.deinit();
            allocator.destroy(dc);
        }

        const save_state_path = args.next().?;

        dc.gdrom.disc = try .init(allocator, io, game);
        try load_state(dc, io, save_state_path);
        const start = std.Io.Timestamp.now(io, .awake);
        var cycles: u64 = 0;
        while (cycles < cycles_target)
            cycles += try dc.tick(max_instructions);
        const elapsed = start.durationTo(std.Io.Timestamp.now(io, .awake));
        total_time += elapsed.toMilliseconds();
        std.debug.print("[Interpreter] {s}: Ran {d} cycles in {} ms\n", .{ dc.gdrom.disc.?.get_product_name() orelse "Unknown", cycles, elapsed.toMilliseconds() });
    }

    std.debug.print("[Interpreter] Total: {d} ms\n", .{total_time});
}
