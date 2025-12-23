const std = @import("std");

pub const XINPUT_GAMEPAD = extern struct {
    wButtons: std.os.windows.WORD,
    bLeftTrigger: std.os.windows.BYTE,
    bRightTrigger: std.os.windows.BYTE,
    sThumbLX: std.os.windows.SHORT,
    sThumbLY: std.os.windows.SHORT,
    sThumbRX: std.os.windows.SHORT,
    sThumbRY: std.os.windows.SHORT,
};

pub const XINPUT_STATE = extern struct {
    dwPacketNumber: std.os.windows.DWORD,
    Gamepad: XINPUT_GAMEPAD,
};

pub const XINPUT_VIBRATION = extern struct {
    wLeftMotorSpeed: std.os.windows.WORD,
    wRightMotorSpeed: std.os.windows.WORD,
};

pub extern fn XInputEnable(enable: std.os.windows.BOOL) void;
pub extern fn XInputGetState(dwUserIndex: std.os.windows.DWORD, pVibration: *XINPUT_STATE) std.os.windows.DWORD;
pub extern fn XInputSetState(dwUserIndex: std.os.windows.DWORD, pVibration: *XINPUT_VIBRATION) std.os.windows.DWORD;

pub fn get_state(index: u32) !XINPUT_STATE {
    var state: XINPUT_STATE = undefined;
    const result: std.os.windows.Win32Error = @intCast(XInputGetState(index, &state));
    switch (result) {
        .SUCCESS => return state,
        .DEVICE_NOT_CONNECTED => return error.DeviceNotConnected,
        else => return error.UnknownError,
    }
}

pub fn set_state(index: u32, state: XINPUT_VIBRATION) !void {
    const result: std.os.windows.Win32Error = @enumFromInt(XInputSetState(index, @constCast(&state)));
    switch (result) {
        .SUCCESS => return,
        .DEVICE_NOT_CONNECTED => return error.DeviceNotConnected,
        else => return error.UnknownError,
    }
}
