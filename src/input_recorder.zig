state: enum { Idle, Playing, Recording } = .Idle,
path: ?[]const u8 = null,
record: ?InputRecord = null,
/// Playing heads. One per port.
cursors: [4]usize = @splat(0),
mutex: std.Io.Mutex = .init,

pub fn deinit(self: *@This()) void {
    const d: *Deecy = self.deecy();
    if (self.path) |p| d._allocator.free(p);
    if (self.record) |*r| r.deinit(d._allocator);
    self.record = null;
}

pub fn set_path(self: *@This(), path: []const u8) !void {
    const d: *Deecy = self.deecy();
    if (self.path) |p| d._allocator.free(p);
    self.path = try d._allocator.dupe(u8, path);
}

pub fn new(self: *@This()) !void {
    const d: *Deecy = self.deecy();

    d.pause();
    try d.reset();

    self.deinit();
    self.* = .{ .state = .Recording, .record = .{} };
    if (self.record) |*r| {
        r.set_game(d.product_uid());
        for (d.dc.maple.ports, 0..) |p, idx| {
            switch (p) {
                .none => r.ports[idx] = .none,
                .emulated => |e| {
                    switch (e.main) {
                        .Controller => {
                            r.ports[idx] = .{ .controller = .{ .peripherals = .{
                                try .init(d._allocator, e.subperipherals[0]),
                                try .init(d._allocator, e.subperipherals[1]),
                            } } };
                        },
                        else => log.warn("Recording unimplemented for device {t}.", .{std.meta.activeTag(e.main)}),
                    }
                },
                .physical => log.warn("Recording unimplemented for physical devices.", .{}),
            }
        }
    }
}

pub fn save(self: *@This(), path: []const u8) !void {
    if (self.record) |r| {
        const d: *Deecy = self.deecy();
        var file = try std.Io.Dir.cwd().createFile(d.io, path, .{});
        defer file.close(d.io);
        var buffer: [2048]u8 = undefined;
        var file_writer = file.writer(d.io, &buffer);
        try r.serialize(&file_writer.interface);
        try file_writer.end();
        try self.set_path(path);
    } else return error.NoRecord;
}

pub fn load(self: *@This(), path: []const u8) !void {
    const d: *Deecy = self.deecy();

    d.pause();
    {
        var file = try std.Io.Dir.cwd().openFile(d.io, path, .{});
        defer file.close(d.io);
        var buffer: [2048]u8 = undefined;
        var file_reader = file.reader(d.io, &buffer);
        const record = try InputRecord.deserialize(d._allocator, &file_reader.interface);
        if (self.record) |*r| r.deinit(d._allocator);
        self.record = record;
    }
    try self.set_path(path);

    // TODO: Check if the loaded disc matches the one expected by the record.
    //       If not, search for it in the library and load it (after user confirmation?).

    // Update input devices to match the recording.
    // FXIME: This feels really hacky. All helper function from Deecy rely on the current config.
    inline for (self.record.?.ports, 0..) |p, port_idx| {
        d.dc.maple.ports[port_idx].deinit(d.io, d._allocator);
        switch (p) {
            .none => d.dc.maple.ports[port_idx] = .none,
            .controller => {
                d.dc.maple.ports[port_idx] = .{ .emulated = .{
                    .main = .{ .Controller = .{ .subcapabilities = .{ @bitCast(MapleModule.Controller.InputCapabilities.Standard), 0, 0 } } },
                    .on_get_condition = .{ .callback = @ptrCast(&Deecy.on_get_condition(port_idx)), .context = d },
                } };
                inline for (p.controller.peripherals, 0..) |peripheral, slot_idx| switch (peripheral) {
                    .none => {},
                    .vmu => |vmu| {
                        d.dc.maple.ports[port_idx].emulated.subperipherals[slot_idx] = .{ .VMU = try .init(d.io, d._allocator, null) };
                        d.install_vmu_callbacks(port_idx, slot_idx);
                        @memcpy(std.mem.sliceAsBytes(d.dc.maple.ports[port_idx].emulated.subperipherals[slot_idx].?.VMU.blocks), vmu.initial_state);
                    },
                };
            },
        }
    }
}

fn deecy(self: *@This()) *Deecy {
    return @alignCast(@fieldParentPtr("input_recorder", self));
}

const std = @import("std");
const log = std.log.scoped(.input_recorder);
const Deecy = @import("deecy.zig");
const InputRecord = Deecy.InputRecord;
const MapleModule = @import("dreamcast").Maple;
