const std = @import("std");

const common = @import("../maple.zig");
const FunctionCodesMask = common.FunctionCodesMask;
const DeviceInfoPayload = common.DeviceInfoPayload;

const log = std.log.scoped(.maple);

const MaxKeys = 6;

pub const Capabilities = FunctionCodesMask.Keyboard;

subcapabilities: [3]u32 = .{ (FunctionDefinition{
    .keyboard_language = .@"America (US)",
    .keyboard_type = .@"101-key",
    .led_type = .{
        .num_lock = true,
        .caps_lock = true,
        .scroll_lock = true,
        .kana = false,
        .power = false,
        .shift = false,
    },
    .led_control = LEDControl.Keyboard,
}).as_u32(), 0, 0 },

pressed_keys: u8 = 0,
status: ReadFormat = .{},

pub fn get_identity(self: *const @This()) DeviceInfoPayload {
    return .{
        .FunctionCodesMask = Capabilities,
        .SubFunctionCodesMasks = self.subcapabilities,
        // FIXME: None of the following values are correct.
        .DescriptionString = "Dreamcast Keyboard             ".*,
        .StandbyConsumption = 0x01AE,
        .MaximumConsumption = 0x01F4,
    };
}

pub fn get_condition(self: *const @This(), function: u32) [3]u32 {
    _ = function;
    const status = @as([*]const u32, @ptrCast(@alignCast(&self.read())))[0..2].*;
    return .{
        Capabilities.as_u32(),
        status[0],
        status[1],
    };
}

pub fn read(self: *const @This()) ReadFormat {
    var r = self.status;
    if (self.pressed_keys > MaxKeys) {
        r.key_scan_code_array = @splat(.Rollover);
    }
    return r;
}

pub fn press_key(self: *@This(), key: KeyScanCode) void {
    switch (key) {
        .LeftControl, .RightShift, .LeftAlt, .LeftGui, .RightControl, .RightShift2, .RightAlt, .RightS3 => self.press_modifier_key(key),
        else => {
            self.pressed_keys += 1;
            if (self.pressed_keys > MaxKeys) return;
            self.status.key_scan_code_array[self.pressed_keys - 1] = key;
        },
    }
}

fn press_modifier_key(self: *@This(), key: KeyScanCode) void {
    switch (key) {
        .LeftControl => self.status.change_key_bits.left_control = true,
        .RightShift => self.status.change_key_bits.right_shift = true,
        .RightShift2 => self.status.change_key_bits.right_shift = true,
        .LeftAlt => self.status.change_key_bits.left_alt = true,
        .LeftGui => self.status.change_key_bits.left_gui = true,
        .RightControl => self.status.change_key_bits.right_control = true,
        .RightAlt => self.status.change_key_bits.right_alt = true,
        .RightS3 => self.status.change_key_bits.s2 = true, // ??
        else => log.err("Unknown modifier key: {t}", .{key}),
    }
}

pub fn release_key(self: *@This(), key: KeyScanCode) void {
    switch (key) {
        .LeftControl, .RightShift, .LeftAlt, .LeftGui, .RightControl, .RightShift2, .RightAlt, .RightS3 => self.release_modifier_key(key),
        else => {
            if (self.pressed_keys == 0) {
                log.err("Releasing key {t} when none are pressed.", .{key});
                return;
            }
            for (&self.status.key_scan_code_array, 0..) |*k, idx| {
                if (k.* == key) {
                    self.pressed_keys -= 1;
                    for (idx..self.pressed_keys) |i| {
                        self.status.key_scan_code_array[i] = self.status.key_scan_code_array[i + 1];
                    }
                    self.status.key_scan_code_array[self.pressed_keys] = .NoOperation;
                    return;
                }
            }
            if (self.pressed_keys <= MaxKeys) {
                log.err("Releasing key {t} when it wasn't pressed.", .{key});
            } else {
                // Rollover key
                self.pressed_keys -= 1;
            }
        },
    }
}

fn release_modifier_key(self: *@This(), key: KeyScanCode) void {
    switch (key) {
        .LeftControl => self.status.change_key_bits.left_control = false,
        .RightShift => self.status.change_key_bits.right_shift = false,
        .RightShift2 => self.status.change_key_bits.right_shift = false,
        .LeftAlt => self.status.change_key_bits.left_alt = false,
        .LeftGui => self.status.change_key_bits.left_gui = false,
        .RightControl => self.status.change_key_bits.right_control = false,
        .RightAlt => self.status.change_key_bits.right_alt = false,
        .RightS3 => self.status.change_key_bits.s2 = false, // ??
        else => log.err("Unknown modifier key: {t}", .{key}),
    }
}

const Language = enum(u8) {
    // Prohibited = 0,
    Japan = 0x01,
    @"America (US)" = 0x02,
    @"England (UK)" = 0x03,
    Germany = 0x04,
    France = 0x05,
    Italy = 0x06,
    Spain = 0x07,
    Sweden = 0x08,
    Switzerland = 0x09,
    Holland = 0x0A,
    Portugal = 0x0B,
    @"Latin America" = 0x0C,
    @"Canadian French" = 0x0D,
    Russia = 0x0E,
    China = 0x0F,
    Korea = 0x10,
    _, // Reserved
};

const Type = enum(u8) {
    // Prohibited = 0,
    @"89-key" = 0x01,
    @"92-key" = 0x02,
    @"101-key" = 0x03,
    @"102-key" = 0x04,
    @"104-key" = 0x05,
    @"105-key" = 0x06,
    @"106-key" = 0x07,
    @"109-key" = 0x08,
    @"87-key" = 0x09,
    @"88-key" = 0x0A,
    _, // Reserved
};

const LEDType = packed struct(u8) {
    num_lock: bool = false,
    caps_lock: bool = false,
    scroll_lock: bool = false,
    _reserved: u2 = 0,
    kana: bool = false,
    power: bool = false,
    shift: bool = false,
};

/// Determines whether the keyboard LED ON/OFF state is controlled by the host or the Keyboard function.
/// When set to be controlled by the Keyboard function, the host disregards the LD setting.
const LEDControl = enum(u1) {
    Host = 0,
    Keyboard = 1,
};

const FunctionDefinition = packed struct(u32) {
    keyboard_language: Language,
    keyboard_type: Type,
    led_type: LEDType,
    _reserved: u7 = 0,
    led_control: LEDControl,

    pub fn as_u32(self: @This()) u32 {
        return @bitCast(self);
    }
};

const ChangeKeyBits = packed struct(u8) {
    left_control: bool = false,
    left_shift: bool = false,
    left_alt: bool = false,
    left_gui: bool = false,
    right_control: bool = false,
    right_shift: bool = false,
    right_alt: bool = false,
    s2: bool = false,
};

pub const KeyScanCode = enum(u8) {
    NoOperation = 0,
    Rollover = 1,
    POSTFail = 2,
    Undefined = 3,
    A = 4,
    B = 5,
    C = 6,
    D = 7,
    E = 8,
    F = 9,
    G = 10,
    H = 11,
    I = 12,
    J = 13,
    K = 14,
    L = 15,
    M = 16,
    N = 17,
    O = 18,
    P = 19,
    Q = 20,
    R = 21,
    S = 22,
    T = 23,
    U = 24,
    V = 25,
    W = 26,
    X = 27,
    Y = 28,
    Z = 29,
    @"1" = 30,
    @"2" = 31,
    @"3" = 32,
    @"4" = 33,
    @"5" = 34,
    @"6" = 35,
    @"7" = 36,
    @"8" = 37,
    @"9" = 38,
    @"0" = 39,
    Return = 40, // Enter
    Escape = 41,
    Delete = 42, // Backspace
    Tab = 43,
    Spacebar = 44,
    Minus = 45,
    Equal = 46,
    LeftBracket = 47,
    RightBracket = 48,
    Backslash = 49,
    NonUSTilde = 50,
    Semicolon = 51,
    Quote = 52,
    Tilde = 53,
    Comma = 54,
    Period = 55,
    Slash = 56,
    CapsLock = 57,
    F1 = 58,
    F2 = 59,
    F3 = 60,
    F4 = 61,
    F5 = 62,
    F6 = 63,
    F7 = 64,
    F8 = 65,
    F9 = 66,
    F10 = 67,
    F11 = 68,
    F12 = 69,
    PrintScreen = 70,
    ScrollLock = 71,
    Pause = 72,
    Insert = 73,
    Home = 74,
    PageUp = 75,
    DeleteForward = 76,
    End = 77,
    PageDown = 78,
    RightArrow = 79,
    LeftArrow = 80,
    DownArrow = 81,
    UpArrow = 82,
    KeypadNumLock = 83,
    KeypadSlash = 84,
    KeypadAsterisk = 85,
    KeypadMinus = 86,
    KeypadPlus = 87,
    KeypadEnter = 88,
    Keypad1 = 89,
    Keypad2 = 90,
    Keypad3 = 91,
    Keypad4 = 92,
    Keypad5 = 93,
    Keypad6 = 94,
    Keypad7 = 95,
    Keypad8 = 96,
    Keypad9 = 97,
    Keypad0 = 98,
    KeypadPeriod = 99,
    NonUSPipe = 100,
    S3 = 101,
    Power = 102,
    KeypadEqual = 103,
    F13 = 104,
    F14 = 105,
    F15 = 106,
    F16 = 107,
    F17 = 108,
    F18 = 109,
    F19 = 110,
    F20 = 111,
    F21 = 112,
    F22 = 113,
    F23 = 114,
    F24 = 115,
    Execute = 116,
    Help = 117,
    Menu = 118,
    Select = 119,
    Stop = 120,
    Again = 121,
    Undo = 122,
    Cut = 123,
    Copy = 124,
    Paste = 125,
    Find = 126,
    Mute = 127,
    VolumeUp = 128,
    VolumeDown = 129,
    CapsLockFixed = 130,
    NumLockFixed = 131,
    ScrollLockFixed = 132,
    KeypadComma = 133,
    KeypadEqualSign = 134,
    InternationalKeyboard1 = 135,
    InternationalKeyboard2 = 136,
    InternationalKeyboard3 = 137,
    InternationalKeyboard4 = 138,
    InternationalKeyboard5 = 139,
    InternationalKeyboard6 = 140,
    InternationalKeyboard7 = 141,
    InternationalKeyboard8 = 142,
    InternationalKeyboard9 = 143,
    Lang1 = 144,
    Lang2 = 145,
    Lang3 = 146,
    Lang4 = 147,
    Lang5 = 148,
    Lang6 = 149,
    Lang7 = 150,
    Lang8 = 151,
    Lang9 = 152,
    AlternateErase = 153,
    SysReqOrAttention = 154,
    Cancel = 155,
    Clear = 156,
    Prior = 157,
    Return2 = 158,
    Separator = 159,
    Out = 160,
    Oper = 161,
    ClearAgain = 162,
    CrSel = 163,
    ExSel = 164,
    // [Reserved]
    LeftControl = 224,
    RightShift = 225,
    LeftAlt = 226,
    LeftGui = 227,
    RightControl = 228,
    RightShift2 = 229,
    RightAlt = 230,
    RightS3 = 231,
    _,
};

const ReadFormat = extern struct {
    change_key_bits: ChangeKeyBits = .{},
    led_information: LEDType = .{},
    key_scan_code_array: [6]KeyScanCode = @splat(.NoOperation),
};
