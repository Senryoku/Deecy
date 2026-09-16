pub fn draw(d: *Deecy) !?enum { Save, SaveAs, Load, StartRecord, StartReplay } {
    try d.input_recording.mutex.lock(d.io);
    defer d.input_recording.mutex.unlock(d.io);

    const self = &d.input_recording.record;

    defer zgui.end();
    if (zgui.begin("Input Editor", .{})) {
        zgui.text("Status: {t}", .{d.input_recording.state});

        if (zgui.button("Save", .{}))
            return .Save;
        zgui.sameLine(.{});
        if (zgui.button("Save As...", .{}))
            return .SaveAs;
        zgui.sameLine(.{});
        if (zgui.button("Load", .{}))
            return .Load;
        zgui.sameLine(.{});

        if (zgui.button("Record", .{}))
            return .StartRecord;
        zgui.sameLine(.{});
        if (zgui.button("Replay", .{}))
            return .StartReplay;

        zgui.text("Game ID: '{s}' ({s})", .{ self.game_id.name, self.game_id.id });

        if (d.input_recording.state == .Playing) {
            zgui.text("Playing: {d}/{d}", .{ d.input_recording.cursor, self.inputs.items.len });
        } else {
            zgui.text("Entry count: {d}", .{self.inputs.items.len});
        }

        if (zgui.beginChild("Inputs", .{ .w = 0, .h = @max(300, zgui.getContentRegionAvail()[1]) })) {
            var clipper: zgui.ListClipper = .init();
            clipper.begin(@intCast(self.inputs.items.len), null);
            while (clipper.step()) {
                const start: usize = @intCast(clipper.DisplayStart);
                for (self.inputs.items[start..@intCast(clipper.DisplayEnd)], start..) |entry, idx| {
                    zgui.text("{d: >8} {d: >6}M {s} {s} {s} {s} {s} {s} {s} {s} {s} {X}", .{
                        idx,
                        entry.cycle / 1_000_000,
                        if (entry.input.buttons.a == 0) "A" else " ",
                        if (entry.input.buttons.b == 0) "B" else " ",
                        if (entry.input.buttons.x == 0) "X" else " ",
                        if (entry.input.buttons.y == 0) "Y" else " ",
                        if (entry.input.buttons.start == 0) "S" else " ",
                        if (entry.input.buttons.up == 0) "U" else " ",
                        if (entry.input.buttons.down == 0) "D" else " ",
                        if (entry.input.buttons.left == 0) "L" else " ",
                        if (entry.input.buttons.right == 0) "R" else " ",
                        entry.input.axis,
                    });
                }
            }
            clipper.end();
        }
        zgui.endChild();
    }

    return null;
}

const std = @import("std");
const zgui = @import("zgui");
const Deecy = @import("../deecy.zig");
