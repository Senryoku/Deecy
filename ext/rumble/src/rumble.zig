const rumbler = @cImport({
    @cDefine("UP_RUMBLE_IMPLEMENTATION", "1");
    @cInclude("up_rumble.h");
});

pub const max_gamepads = 4;

pub fn set_gamepad_rumble(gamepad: u32, left_rumble: f32, right_rumble: f32, duration: f32) bool {
    return rumbler.set_gamepad_rumble(gamepad, left_rumble, right_rumble, duration) == 1;
}

pub fn is_gamepad_rumbling(gamepad: u32) bool {
    return rumbler.is_gamepad_rumbling(gamepad) == 1;
}

pub fn poll_gamepads(delta_time: f32) void {
    rumbler.poll_gamepads(delta_time);
}
