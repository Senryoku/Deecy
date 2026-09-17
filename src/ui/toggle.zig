//! Toggle widget.

const AnimationSpeed: f32 = 12.0;
const TrackAspectRatio: f32 = 1.6;
const TrackPadding: f32 = 2.6;

var animation: struct {
    id: zgui.Ident = 0,
    value: f32 = 0.0,

    pub fn update(self: *@This(), target: f32) f32 {
        const framerate = @max(zgui.io.getFramerate(), 1.0);
        const step = AnimationSpeed * 1.0 / framerate;
        self.value += (target - self.value) * @min(step, 1.0);
        if (@abs(target - self.value) < 0.001) {
            self.id = 0;
            self.value = target;
        }
        return self.value;
    }
} = .{};

pub fn toggle(label: [:0]const u8, args: struct { v: *bool }) bool {
    const id = zgui.getStrId(label);
    const frame_height = zgui.getFrameHeight();
    const track_height = 0.9 * frame_height;
    const track_width = track_height * TrackAspectRatio;
    const label_size = zgui.calcTextSize(label, .{ .hide_text_after_double_hash = true });
    const width = track_width + zgui.getStyle().item_inner_spacing[0] + label_size[0];
    const position = zgui.getCursorScreenPos();

    const changed = zgui.invisibleButton(label, .{ .w = width, .h = frame_height });
    const hovered = zgui.isItemHovered(.{});
    if (hovered) zgui.setMouseCursor(.hand);
    if (changed) {
        if (animation.id != id)
            animation.value = if (args.v.*) 1.0 else 0.0;
        animation.id = id;
        args.v.* = !args.v.*;
    }

    const target: f32 = if (args.v.*) 1.0 else 0.0;
    const slide = if (id != animation.id) target else animation.update(target);

    const style = zgui.getStyle();
    // Off and On colors, interpolated during animation.
    const track_colors: [2]@Vector(4, f32) = .{ style.getColor(.frame_bg), style.getColor(.button_active) };
    const knob_colors: [2]@Vector(4, f32) = .{ if (hovered) style.getColor(.slider_grab_active) else style.getColor(.slider_grab), .{ 0.85, 0.85, 0.85, 1.0 } };
    const border_colors: [2]@Vector(4, f32) = .{ style.getColor(.frame_bg_hovered), style.getColor(.button) };
    const on: @Vector(4, f32) = @splat(slide);
    const off: @Vector(4, f32) = @splat(1.0 - slide);
    const track_color = off * track_colors[0] + on * track_colors[1];
    const knob_color = off * knob_colors[0] + on * knob_colors[1];
    const border_color = off * border_colors[0] + on * border_colors[1];

    const draw_list = zgui.getWindowDrawList();
    const track_offset_y = 0.5 * (frame_height - track_height);
    const track_min = .{ position[0], position[1] + track_offset_y };
    const track_max = .{ track_min[0] + track_width, track_min[1] + track_height };
    const radius = track_height * 0.5 - TrackPadding;
    const knob_x = track_min[0] + TrackPadding + radius + slide * (track_width - 2.0 * (TrackPadding + radius));

    draw_list.addRectFilled(.{
        .pmin = track_min,
        .pmax = track_max,
        .col = zgui.colorConvertFloat4ToU32(track_color),
        .rounding = track_height * 0.5,
    });
    draw_list.addRect(.{
        .pmin = track_min,
        .pmax = track_max,
        .col = zgui.colorConvertFloat4ToU32(border_color),
        .rounding = track_height * 0.5,
    });
    draw_list.addCircleFilled(.{
        .p = .{ knob_x, track_min[1] + track_height * 0.5 },
        .r = radius,
        .col = zgui.colorConvertFloat4ToU32(knob_color),
    });

    if (label_size[0] > 0.0) {
        const text_start = position[0] + track_width + style.item_inner_spacing[0];
        const visible_label = if (std.mem.indexOf(u8, label, "##")) |index| label[0..index] else label;
        draw_list.addTextUnformatted(.{ text_start, position[1] + style.frame_padding[1] }, zgui.colorConvertFloat4ToU32(style.getColor(.text)), visible_label);
    }

    return changed;
}

const std = @import("std");
const zgui = @import("zgui");
