const std = @import("std");
const builtin = @import("builtin");
const xinput = @import("xinput.zig");

const zglfw = @import("zglfw");

var states: [4]struct { used: bool = false, power: f32 = 0, change: f32 = 0 } = @splat(.{});

const WindowsImpl = struct {
    pub fn set_rumble(jid: zglfw.Joystick, power: f32) bool {
        // NOTE/FIXME: There is no guarantee that j.id will match the XInput device index.
        //             GLFW does not expose the plateform-specific joystick struct (it would be joystick->win32.index).
        const index: u32 = @intCast(@intFromEnum(jid));
        if (index >= 4) return false;
        const speed: u16 = @intFromFloat(std.math.clamp(power, 0, 1) * 65535);
        xinput.set_state(index, .{ .wLeftMotorSpeed = speed, .wRightMotorSpeed = speed }) catch return false;
        return true;
    }
};

const LinuxImpl = struct {
    pub fn set_rumble(jid: zglfw.Joystick, power: f32) bool {
        // TODO
        _ = jid;
        _ = power;
        return false;
    }
};

const impl = switch (builtin.os.tag) {
    .windows => WindowsImpl,
    .linux => LinuxImpl,
    else => @compileError("Unsupported OS."),
};

pub fn set_rumble(jid: zglfw.Joystick, power: f32, change: f32) bool {
    const index: u32 = @intCast(@intFromEnum(jid));
    if (index >= 4) return false;
    states[index] = .{ .used = power != 0 or change != 0, .power = power, .change = change };
    return impl.set_rumble(jid, states[index].power);
}

pub fn update_rumble(dt: f32) void {
    for (0..4) |i| {
        if (states[i].used) {
            states[i].power += states[i].change * dt;
            if (states[i].power <= 0)
                states[i] = .{};
            if (states[i].change >= 0 and states[i].power > 1)
                states[i] = .{};
            _ = impl.set_rumble(@enumFromInt(i), states[i].power);
        }
    }
}

pub fn stop_all() void {
    for (0..4) |i| {
        _ = set_rumble(@enumFromInt(i), 0, 0);
    }
}
