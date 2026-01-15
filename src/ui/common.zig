const zgui = @import("zgui");

pub const Icons = @import("font_awesome.zig");

/// Afterwards:
///   defer zgui.pop_red_button_style();
pub fn push_red_button_style() void {
    zgui.pushStyleColor4f(.{ .c = .{ 0.60, 0.12, 0.15, 1.0 }, .idx = .button });
    zgui.pushStyleColor4f(.{ .c = .{ 0.80, 0.20, 0.23, 1.0 }, .idx = .button_hovered });
    zgui.pushStyleColor4f(.{ .c = .{ 0.50, 0.08, 0.11, 1.0 }, .idx = .button_active });
}

pub fn pop_red_button_style() void {
    zgui.popStyleColor(.{ .count = 3 });
}

pub fn red_button(comptime fmt: [:0]const u8, args: struct { w: f32 = 0.0, h: f32 = 0.0 }) bool {
    push_red_button_style();
    defer pop_red_button_style();
    return zgui.button(fmt, .{ .w = args.w, .h = args.h });
}
