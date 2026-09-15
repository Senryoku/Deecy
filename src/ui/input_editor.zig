pub fn draw(d: *Deecy) !?enum { StartRecord, StartReplay } {
    try d.input_recording.mutex.lock(d.io);
    defer d.input_recording.mutex.unlock(d.io);

    defer zgui.end();
    if (zgui.begin("Input Recording", .{})) {
        zgui.text("Status: {t}", .{d.input_recording.state});
        if (zgui.button("Record", .{}))
            return .StartRecord;

        if (zgui.button("Replay", .{}))
            return .StartReplay;

        zgui.text("Entry count: {d}", .{d.input_recording.record.inputs.items.len});
        for (d.input_recording.record.inputs.items, 0..) |entry, idx| {
            zgui.text("{d: >8}, {d: >12}, {s} {s} {s} {s} {s} {s} {s} {s} {s} {X}", .{
                idx,
                entry.cycle,
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

    return null;
}

const std = @import("std");
const zgui = @import("zgui");
const Deecy = @import("../deecy.zig");
