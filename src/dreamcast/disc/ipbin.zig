const std = @import("std");

pub const Peripherals = packed struct(u28) {
    wince: bool,
    _0: u3,
    vga: bool,
    _1: u3,
    // Supported expansions
    other_expansions: bool,
    vibration_pack: bool,
    microphone: bool,
    memory_card: bool,
    // Minimum controller requirements
    start_a_b_directions: bool,
    c_button: bool,
    d_button: bool,
    x_button: bool,
    y_button: bool,
    z_button: bool,
    expanded_direction_buttons: bool,
    analog_r_trigger: bool,
    analog_l_trigger: bool,
    analog_horizontal_controller: bool,
    analog_vertical_controller: bool,
    expanded_analog_horizontal: bool,
    expanded_analog_vertical: bool,
    // Supported peripherals
    gun: bool,
    keyboard: bool,
    mouse: bool,
};

data: []const u8,

pub fn product_id(self: @This()) []const u8 {
    return self.data[0x40..0x50];
}

pub fn product_name(self: @This()) []const u8 {
    const name = self.data[0x80..0x90];
    // Trim spaces
    var end = name.len - 1;
    while (end > 0 and name[end] == ' ') end -= 1;
    return name[0 .. end + 1];
}

pub fn japan_region(self: @This()) bool {
    return self.data[0x30] == 'J';
}

pub fn usa_region(self: @This()) bool {
    return self.data[0x31] == 'U';
}

pub fn europe_region(self: @This()) bool {
    return self.data[0x32] == 'E';
}

pub fn peripherals(self: @This()) Peripherals {
    const ascii_hex = self.data[0x38..0x3F];
    return @bitCast(std.fmt.parseInt(u28, ascii_hex, 16) catch 0);
}
