const std = @import("std");
const zglfw = @import("zglfw");
const zgui = @import("zgui");
const Deecy = @import("../deecy.zig");

// Stupid way of waiting for any key press
var key_pressed: ?zglfw.Key = null;
var modifiers: ?zglfw.Mods = null;
fn wait_for_key_callback(_: *zglfw.Window, key: zglfw.Key, _: i32, action: zglfw.Action, mods: zglfw.Mods) callconv(.c) void {
    if (action == .press) {
        switch (key) {
            // Ignore mofifiers
            .left_shift, .right_shift, .left_alt, .right_alt, .left_control, .right_control, .left_super, .right_super => {},
            else => {
                key_pressed = key;
                modifiers = mods;
            },
        }
    }
}
fn reset() void {
    key_pressed = null;
    modifiers = null;
}

pub fn keyboard(d: *Deecy) zglfw.Key {
    const prev_callback = d.window.setKeyCallback(wait_for_key_callback);
    while (key_pressed == null) zglfw.waitEvents();
    const value = key_pressed;
    reset();
    _ = d.window.setKeyCallback(prev_callback);
    return value.?;
}

pub fn controller_button(d: *Deecy, gamepad_id: zglfw.Gamepad) ?zglfw.Gamepad.Button {
    while (true) {
        zglfw.pollEvents();
        if (zglfw.getKey(d.window, .escape) == .press) return null;

        const gamepad_state = gamepad_id.getState() catch return null;
        for (gamepad_state.buttons, 0..) |button, i| {
            if (button == .press)
                return @enumFromInt(i);
        }
        std.Thread.sleep(1_000_000);
    }
    return null;
}

pub fn controller_axis(d: *Deecy, gamepad_id: zglfw.Gamepad) ?zglfw.Gamepad.Axis {
    const initial_state = gamepad_id.getState() catch return null;
    const Threshold = 0.1;
    while (true) {
        zglfw.pollEvents();
        if (zglfw.getKey(d.window, .escape) == .press) return null;

        const gamepad_state = gamepad_id.getState() catch return null;
        for (gamepad_state.axes, 0..) |value, i| {
            if (@abs(initial_state.axes[i] - value) > Threshold) {
                return @enumFromInt(i);
            }
        }
        std.Thread.sleep(1_000_000);
    }
    return null;
}

pub fn any_button(d: *Deecy) ?union(enum) { controller: zglfw.Gamepad.Button, keyboard: struct { key: zglfw.Key, modifiers: zglfw.Mods } } {
    const prev_callback = d.window.setKeyCallback(wait_for_key_callback);
    defer {
        reset();
        _ = d.window.setKeyCallback(prev_callback);
    }
    while (true) {
        zglfw.pollEvents();
        if (key_pressed) |key| {
            if (key == .escape) return null;
            return .{ .keyboard = .{ .key = key, .modifiers = modifiers orelse .{} } };
        }

        for (d.controllers) |maybe_controller| {
            if (maybe_controller) |controller| {
                if (controller.id.isPresent()) {
                    if (controller.id.asGamepad()) |gamepad_id| {
                        const gamepad_state = gamepad_id.getState() catch continue;
                        for (gamepad_state.buttons, 0..) |button, i| {
                            if (button == .press)
                                return .{ .controller = @enumFromInt(i) };
                        }
                    }
                }
            }
        }
        std.Thread.sleep(1_000_000);
    }
    return null;
}
