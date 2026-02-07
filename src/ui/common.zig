const zgui = @import("zgui");

pub const Icons = @import("font_awesome.zig");

pub const White: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 };
pub const Grey: [4]f32 = .{ 0.5, 0.5, 0.5, 1.0 };
pub const Green: [4]f32 = .{ 0.51, 0.71, 0.212, 1.0 };
pub const Yellow: [4]f32 = .{ 0.72, 0.55, 0.13, 1.0 };
pub const Red: [4]f32 = .{ 0.60, 0.12, 0.15, 1.0 };

/// Afterwards:
///   defer zgui.pop_red_button_style();
pub fn push_red_button_style() void {
    zgui.pushStyleColor4f(.{ .c = Red, .idx = .button });
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
