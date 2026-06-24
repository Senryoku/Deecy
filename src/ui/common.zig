const std = @import("std");
const zgui = @import("zgui");

pub const Icons = @import("font_awesome.zig");

pub const White: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 };
pub const Grey: [4]f32 = .{ 0.5, 0.5, 0.5, 1.0 };
pub const Green: [4]f32 = .{ 0.51, 0.71, 0.212, 1.0 };
pub const Yellow: [4]f32 = .{ 0.72, 0.55, 0.13, 1.0 };
pub const Red: [4]f32 = .{ 0.60, 0.12, 0.15, 1.0 };

pub const DCBlueU: u32 = 0xFFC2763B;

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

/// Returns true if all the words (separated by spaces) in `query` are present in `value`
pub fn word_filter(query: []const u8, value: []const u8) bool {
    if (query.len == 0) return true;
    var words = std.mem.tokenizeScalar(u8, query, ' ');
    while (words.next()) |word| {
        if (std.mem.find(u8, value, word) == null) return false;
    }
    return true;
}

/// Return a rough score of how well `query` matches the words in `value`.
/// Assumes the inputs already passes `word_filter`.
pub fn word_filter_score(query: []const u8, value: []const u8) i32 {
    var score: i32 = 0;
    var words = std.mem.tokenizeScalar(u8, query, ' ');
    var last_index: usize = value.len;
    while (words.next()) |word| {
        if (std.mem.find(u8, value, word)) |index| {
            if (index == 0) score += 5; // Start bonus
            if (index > last_index) score += 1; // Sequence bonus
            // Actual word in query bonus
            if (index == 0 or !std.ascii.isAlphanumeric(value[index - 1])) score += 10;
            if (index + word.len >= value.len or !std.ascii.isAlphanumeric(value[index + word.len])) score += 10;
            last_index = index;
        } else return 0;
    }
    return score;
}
