const zgui = @import("zgui");

pub const Icons = @import("font_awesome.zig");

/// Afterwards:
///   defer zgui.pop_red_button_style();
pub fn push_red_button_style() void {
    zgui.pushStyleColor4f(.{ .c = .{ 0.86, 0.12, 0.15, 1.0 }, .idx = .button });
    zgui.pushStyleColor4f(.{ .c = .{ 0.95, 0.20, 0.23, 1.0 }, .idx = .button_hovered });
    zgui.pushStyleColor4f(.{ .c = .{ 0.70, 0.08, 0.11, 1.0 }, .idx = .button_active });
}

pub fn pop_red_button_style() void {
    zgui.popStyleColor(.{ .count = 3 });
}
