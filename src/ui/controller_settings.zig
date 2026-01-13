const std = @import("std");
const zglfw = @import("zglfw");
const zgui = @import("zgui");
const zgpu = @import("zgpu");

const Deecy = @import("../deecy.zig");
const Dreamcast = @import("dreamcast");
const termcolor = @import("termcolor");
const wait_for = @import("wait_for_input.zig");

const Color = 0xFFFFFFFF;
const PressedButtonColor: u32 = 0xFFFFFFFF;
const Thickness = 2.0;
const Rounding = 2.0;
const IntendedSize = 200.0; // All the following pixel values are within a 200x200 image.

fn controller_binding_tooltip(d: *Deecy, comptime port: u8, comptime field_name: [:0]const u8) void {
    comptime var controller_field_name = field_name;
    if (!@hasField(Deecy.ControllerBindings, field_name))
        controller_field_name = controller_field_name ++ "_button";
    if (zgui.isItemHovered(.{})) {
        zgui.setMouseCursor(.hand);
        if (zgui.beginTooltip()) {
            zgui.textUnformatted(@import("helpers").title_case(field_name));
            zgui.separator();
            if (@hasField(Deecy.KeyboardBindings, field_name)) {
                if (@field(d.config.keyboard_bindings[port], field_name)) |keyboard_binding| {
                    zgui.text("Keyboard: {t}", .{keyboard_binding});
                } else {
                    zgui.text("Keyboard: None", .{});
                }
            }
            if (@field(d.config.controllers_bindings[port], controller_field_name)) |controller_binding| {
                zgui.text("Controller: {t}", .{controller_binding});
            } else {
                zgui.text("Controller: None", .{});
            }
            zgui.textDisabled("Left click to edit controller binding.", .{});
            if (@hasField(Deecy.KeyboardBindings, field_name)) {
                zgui.textDisabled("Right click to edit keyboard binding.", .{});
                zgui.textDisabled("Escape while editing to clear. Middle click to clear both.", .{});
            } else {
                zgui.textDisabled("Escape while editing or Middle click to clear.", .{});
            }
            zgui.endTooltip();
        }
    }
}

fn bind_button(d: *Deecy, comptime port: u8, comptime field_name: [:0]const u8, gamepad_id: ?zglfw.Gamepad) void {
    comptime var controller_field_name = field_name;
    if (!@hasField(Deecy.ControllerBindings, field_name) or @TypeOf(@field(d.config.controllers_bindings[port], field_name)) != ?zglfw.Gamepad.Button)
        controller_field_name = controller_field_name ++ "_button";
    if (zgui.isMouseReleased(.left)) {
        if (gamepad_id) |gamepad|
            @field(d.config.controllers_bindings[port], controller_field_name) = wait_for.controller_button(d, gamepad);
    } else if (zgui.isMouseReleased(.right)) {
        @field(d.config.keyboard_bindings[port], field_name) = wait_for.keyboard(d);
    } else if (zgui.isMouseReleased(.middle)) {
        @field(d.config.controllers_bindings[port], controller_field_name) = null;
        @field(d.config.keyboard_bindings[port], field_name) = null;
    }
}

fn bind_axis(d: *Deecy, comptime port: u8, comptime field_name: [:0]const u8, gamepad_id: ?zglfw.Gamepad) void {
    if (zgui.isMouseReleased(.left)) {
        if (gamepad_id) |gamepad|
            @field(d.config.controllers_bindings[port], field_name) = wait_for.controller_axis(d, gamepad);
    } else if (zgui.isMouseReleased(.right)) {
        if (@hasField(Deecy.KeyboardBindings, field_name))
            @field(d.config.keyboard_bindings[port], field_name) = wait_for.keyboard(d);
    } else if (zgui.isMouseReleased(.middle)) {
        @field(d.config.controllers_bindings[port], field_name) = null;
    }
}

fn stroke_cubic_curves(draw_list: zgui.DrawList, x: f32, y: f32, s: f32, color: u32, thickness: f32, control_points: []const [3][2]f32) void {
    draw_list.pathLineTo(.{ x + s * control_points[0][0][0], y + s * control_points[0][0][1] });
    for (control_points) |cp| {
        draw_list.pathBezierCubicCurveTo(.{
            .p2 = .{ x + cp[0][0] * s, y + cp[0][1] * s },
            .p3 = .{ x + cp[1][0] * s, y + cp[1][1] * s },
            .p4 = .{ x + cp[2][0] * s, y + cp[2][1] * s },
            .num_segments = 0,
        });
    }
    draw_list.pathStroke(.{ .col = color, .flags = .{}, .thickness = thickness });
}

fn draw_analog_stick(d: *Deecy, comptime port: u8, draw_list: zgui.DrawList, s: f32, p: [2]f32, bg_color: u32, comptime name: [:0]const u8, gamepad_id: ?zglfw.Gamepad, x_axis: u8, y_axis: u8) void {
    const stick_outer_size = s * 12;
    const stick_inner_size = s * 10;
    const axis_size = s * 16.0;
    const axis_thickness = s * 2.0;
    const axis_button_thickness = s * 8.0;

    zgui.pushStrId(name);
    defer zgui.popId();

    // Outer (fixed)
    draw_list.addCircleFilled(.{ .p = p, .r = stick_outer_size, .col = bg_color, .num_segments = 0 });
    draw_list.addCircle(.{ .p = p, .r = stick_outer_size, .col = Color, .num_segments = 0, .thickness = Thickness });

    // Axes
    {
        zgui.pushIntId(0);
        defer zgui.popId();
        draw_list.addLine(.{ .p1 = .{ p[0] - axis_size, p[1] }, .p2 = .{ p[0] + axis_size, p[1] }, .col = Color, .thickness = axis_thickness });
        zgui.setCursorScreenPos(.{ p[0] - axis_button_thickness / 2.0, p[1] - axis_size });
        if (zgui.invisibleButton("##StickUD", .{ .w = axis_button_thickness, .h = 2 * axis_size, .flags = .{ .mouse_button_left = true, .mouse_button_right = true, .mouse_button_middle = true } }))
            bind_axis(d, port, name ++ "_stick_up_down", gamepad_id);
        controller_binding_tooltip(d, port, name ++ "_stick_up_down");
    }
    {
        zgui.pushIntId(1);
        defer zgui.popId();
        draw_list.addLine(.{ .p1 = .{ p[0], p[1] - axis_size }, .p2 = .{ p[0], p[1] + axis_size }, .col = Color, .thickness = axis_thickness });
        zgui.setCursorScreenPos(.{ p[0] - axis_size, p[1] - axis_button_thickness / 2.0 });
        if (zgui.invisibleButton("##StickLR", .{ .w = 2 * axis_size, .h = axis_button_thickness, .flags = .{ .mouse_button_left = true, .mouse_button_right = true, .mouse_button_middle = true } }))
            bind_axis(d, port, name ++ "_stick_left_right", gamepad_id);
        controller_binding_tooltip(d, port, name ++ "_stick_left_right");
    }

    // Button directions (primarily for keyboard bindings)
    const dir_width = s * 4.0;
    inline for (.{
        .{ .p = .{ p[0] - axis_size - dir_width, p[1] - dir_width / 2.0 }, .triangle = .{
            .{ p[0] - axis_size - dir_width, p[1] },
            .{ p[0] - axis_size, p[1] + dir_width / 2.0 },
            .{ p[0] - axis_size, p[1] - dir_width / 2.0 },
        }, .field_name = name ++ "_stick_left", .pressed = 127 - @min(x_axis, 127) },
        .{ .p = .{ p[0] + axis_size, p[1] - dir_width / 2.0 }, .triangle = .{
            .{ p[0] + axis_size + dir_width, p[1] },
            .{ p[0] + axis_size, p[1] + dir_width / 2.0 },
            .{ p[0] + axis_size, p[1] - dir_width / 2.0 },
        }, .field_name = name ++ "_stick_right", .pressed = @max(x_axis, 128) - 128 },
        .{ .p = .{ p[0] - dir_width / 2.0, p[1] - axis_size - dir_width }, .triangle = .{
            .{ p[0], p[1] - axis_size - dir_width },
            .{ p[0] + dir_width / 2.0, p[1] - axis_size },
            .{ p[0] - dir_width / 2.0, p[1] - axis_size },
        }, .field_name = name ++ "_stick_up", .pressed = 127 - @min(y_axis, 127) },
        .{ .p = .{ p[0] - dir_width / 2.0, p[1] + axis_size }, .triangle = .{
            .{ p[0], p[1] + axis_size + dir_width },
            .{ p[0] + dir_width / 2.0, p[1] + axis_size },
            .{ p[0] - dir_width / 2.0, p[1] + axis_size },
        }, .field_name = name ++ "_stick_down", .pressed = @max(y_axis, 128) - 128 },
    }, 0..) |dir, dir_idx| {
        zgui.pushIntId(@intCast(dir_idx));
        defer zgui.popId();
        zgui.setCursorScreenPos(dir.p);
        if (zgui.invisibleButton("##Stick", .{ .w = dir_width, .h = dir_width, .flags = .{ .mouse_button_left = true, .mouse_button_right = true, .mouse_button_middle = true } }))
            bind_button(d, port, dir.field_name, gamepad_id);
        controller_binding_tooltip(d, port, dir.field_name);

        const color = 0x00FFFFFF | (@as(u32, @intCast(dir.pressed)) << 25);
        draw_list.addTriangleFilled(.{ .p1 = dir.triangle[0], .p2 = dir.triangle[1], .p3 = dir.triangle[2], .col = color });
        draw_list.addTriangle(.{ .p1 = dir.triangle[0], .p2 = dir.triangle[1], .p3 = dir.triangle[2], .col = Color, .thickness = Thickness });
    }

    // Inner (movable)
    const stick_x = stick_inner_size * (@as(f32, @floatFromInt(x_axis)) / 256.0 - 0.5);
    const stick_y = stick_inner_size * (@as(f32, @floatFromInt(y_axis)) / 256.0 - 0.5);
    draw_list.addCircleFilled(.{ .p = .{ p[0] + stick_x, p[1] + stick_y }, .r = stick_inner_size, .col = Color, .num_segments = 0 });
}

pub fn draw_controller_settings(d: *Deecy, comptime port: u8) !void {
    if (d.dc.maple.ports[port].main == null or d.dc.maple.ports[port].main.? != .Controller) return;

    const guest_controller = &d.dc.maple.ports[port].main.?.Controller;
    const axes: [6]u8 = guest_controller.axis;
    const buttons = guest_controller.buttons;
    var capabilities: Dreamcast.Maple.Controller.InputCapabilities = @bitCast(guest_controller.subcapabilities[0]);

    var gamepad_id: ?zglfw.Gamepad = null;
    if (d.controllers[port]) |j| {
        if (j.id.isPresent())
            gamepad_id = j.id.asGamepad();
    }

    const bg_color = zgui.colorConvertFloat4ToU32(zgui.getStyle().colors[@intFromEnum(zgui.StyleCol.window_bg)]);
    const draw_list = zgui.getWindowDrawList();
    const window_size = zgui.getContentRegionAvail();
    const s: f32 = std.math.clamp(window_size[0] / IntendedSize, 0.5, 4.0); // Scaling factor
    const x = zgui.getCursorScreenPos()[0];
    const y = zgui.getCursorScreenPos()[1];
    // Non-functional decorations
    {
        const DecorationColor = 0xFFA0A0A0;
        // Outline
        stroke_cubic_curves(draw_list, x, y, s, DecorationColor, Thickness, &[_][3][2]f32{
            .{ .{ 50.91, 34.74 }, .{ 28.98, 40.38 }, .{ 26.44, 41.46 } },
            .{ .{ 24.16, 42.73 }, .{ 19.21, 51.22 }, .{ 16.68, 65.17 } },
            .{ .{ 14.14, 79.12 }, .{ 15.41, 140.99 }, .{ 20.74, 156.2 } },
            .{ .{ 26.07, 171.41 }, .{ 34.08, 188.32 }, .{ 37.57, 189.42 } },
            .{ .{ 39.19, 189.93 }, .{ 40.11, 190.18 }, .{ 41.88, 188.91 } },
            .{ .{ 43.66, 187.64 }, .{ 50.41, 162.79 }, .{ 50.41, 162.79 } },
            .{ .{ 50.41, 162.79 }, .{ 51.07, 159.83 }, .{ 52.95, 159.49 } },
            .{ .{ 58.7, 158.46 }, .{ 84.53, 152.88 }, .{ 100.0, 153.12 } },
        });
        // Mirror
        stroke_cubic_curves(draw_list, x, y, s, DecorationColor, Thickness, &[_][3][2]f32{
            .{ .{ 149.09, 34.74 }, .{ 171.02, 40.38 }, .{ 173.56, 41.46 } },
            .{ .{ 175.84, 42.73 }, .{ 180.79, 51.22 }, .{ 183.32, 65.17 } },
            .{ .{ 185.86, 79.12 }, .{ 184.59, 140.99 }, .{ 179.26, 156.2 } },
            .{ .{ 173.93, 171.41 }, .{ 165.92, 188.32 }, .{ 162.43, 189.42 } },
            .{ .{ 160.81, 189.93 }, .{ 159.89, 190.18 }, .{ 158.12, 188.91 } },
            .{ .{ 156.34, 187.64 }, .{ 149.59, 162.79 }, .{ 149.59, 162.79 } },
            .{ .{ 149.59, 162.79 }, .{ 148.93, 159.83 }, .{ 147.05, 159.49 } },
            .{ .{ 141.3, 158.46 }, .{ 115.47, 152.88 }, .{ 100.0, 153.12 } },
        });
        // Decorative circle and end of outline
        draw_list.addCircleFilled(.{ .p = .{ x + s * 100.0, y + s * 70.0 }, .r = s * 60.0, .col = bg_color, .num_segments = 0 });
        draw_list.addCircle(.{ .p = .{ x + s * 100.0, y + s * 70.0 }, .r = s * 60.0, .col = DecorationColor, .num_segments = 0, .thickness = Thickness });
        // Decorative VMU outline
        stroke_cubic_curves(draw_list, x, y, s, DecorationColor, Thickness, &[_][3][2]f32{
            .{ .{ 70.0, 18.0 }, .{ 70.0, 50.0 }, .{ 70.0, 65.0 } },
            .{ .{ 70.0, 85.0 }, .{ 85.0, 95.0 }, .{ 100.0, 95.0 } },
        });
        stroke_cubic_curves(draw_list, x, y, s, DecorationColor, Thickness, &[_][3][2]f32{
            .{ .{ 130.0, 18.0 }, .{ 130.0, 50.0 }, .{ 130.0, 65.0 } },
            .{ .{ 130.0, 85.0 }, .{ 115.0, 95.0 }, .{ 100.0, 95.0 } },
        });
        // VMU Window
        draw_list.addRect(.{ .pmin = .{ x + s * 80.0, y + s * 40.0 }, .pmax = .{ x + s * 120.0, y + s * 72.0 }, .col = DecorationColor, .flags = .{}, .thickness = Thickness, .rounding = Rounding });
    }

    var has_right_stick = capabilities.analogVertical2 != 0;
    zgui.setCursorScreenPos(.{ x + s * 60.0, y + s * 170.0 });
    if (zgui.checkbox("Right Stick", .{ .v = &has_right_stick })) {
        if (has_right_stick) {
            capabilities.analogVertical2 = 1;
            capabilities.analogHorizontal2 = 1;
        } else {
            capabilities.analogVertical2 = 0;
            capabilities.analogHorizontal2 = 0;
        }
        guest_controller.subcapabilities[0] = @bitCast(capabilities);
        d.config.controllers[port].subcapabilities = capabilities;
    }

    const trigger_size = [2]f32{ s * 15.0, s * 20.0 };
    inline for (.{
        .{ .pos = .{ 30.0, 10.0 }, .axis = axes[1], .field_name = "left_trigger" },
        .{ .pos = .{ 155.0, 10.0 }, .axis = axes[0], .field_name = "right_trigger" },
    }, 0..) |trigger, idx| {
        zgui.pushIntId(@intCast(idx));
        defer zgui.popId();
        const value = @as(f32, @floatFromInt(trigger.axis)) / 256.0;
        const trigger_color = @as(u32, trigger.axis) << 24 | 0x00FFFFFF;
        const pmin = [2]f32{ x + s * trigger.pos[0], y + s * trigger.pos[1] };
        const pmax = [2]f32{ pmin[0] + trigger_size[0], pmin[1] + trigger_size[1] + Rounding };
        zgui.setCursorScreenPos(pmin);
        if (zgui.invisibleButton("##Trigger", .{ .w = trigger_size[0], .h = trigger_size[1], .flags = .{ .mouse_button_left = true, .mouse_button_right = true, .mouse_button_middle = true } }))
            bind_axis(d, port, trigger.field_name, gamepad_id);
        controller_binding_tooltip(d, port, trigger.field_name);
        draw_list.addRectFilled(.{ .pmin = .{ pmin[0], pmin[1] + trigger_size[1] * value }, .pmax = pmax, .col = trigger_color, .flags = .{}, .rounding = Rounding });
        draw_list.addRect(.{ .pmin = .{ pmin[0], pmin[1] + trigger_size[1] * value }, .pmax = pmax, .col = Color, .flags = .{}, .thickness = Thickness, .rounding = Rounding });
    }

    // D-Pad
    const dpad_x = x + s * 50;
    const dpad_y = y + s * 100;
    const dpad_size = s * 12;
    const dpad_width = s * 4;
    inline for (.{
        .{ .pmin = .{ dpad_x - dpad_width, dpad_y - dpad_size }, .pmax = .{ dpad_x + dpad_width, dpad_y }, .button = buttons.up, .field_name = "up" },
        .{ .pmin = .{ dpad_x - dpad_width, dpad_y + dpad_size - 2 * dpad_width }, .pmax = .{ dpad_x + dpad_width, dpad_y + dpad_size }, .button = buttons.down, .field_name = "down" },
        .{ .pmin = .{ dpad_x - dpad_size, dpad_y - dpad_width }, .pmax = .{ dpad_x, dpad_y + dpad_width }, .button = buttons.left, .field_name = "left" },
        .{ .pmin = .{ dpad_x + dpad_size - 2 * dpad_width, dpad_y - dpad_width }, .pmax = .{ dpad_x + dpad_size, dpad_y + dpad_width }, .button = buttons.right, .field_name = "right" },
    }, 0..) |direction, idx| {
        zgui.pushIntId(@intCast(idx));
        defer zgui.popId();
        zgui.setCursorScreenPos(direction.pmin);
        if (zgui.invisibleButton("##Dpad", .{ .w = 2 * dpad_width, .h = 2 * dpad_width, .flags = .{ .mouse_button_left = true, .mouse_button_right = true, .mouse_button_middle = true } }))
            bind_button(d, port, direction.field_name, gamepad_id);
        controller_binding_tooltip(d, port, direction.field_name);
        draw_list.addRectFilled(.{ .pmin = direction.pmin, .pmax = direction.pmax, .col = if (direction.button == 0) PressedButtonColor else bg_color });
    }
    draw_list.addRect(.{ .pmin = .{ dpad_x - dpad_width, dpad_y - dpad_size }, .pmax = .{ dpad_x + dpad_width, dpad_y + dpad_size }, .col = Color, .thickness = Thickness, .rounding = Rounding });
    draw_list.addRect(.{ .pmin = .{ dpad_x - dpad_size, dpad_y - dpad_width }, .pmax = .{ dpad_x + dpad_size, dpad_y + dpad_width }, .col = Color, .thickness = Thickness, .rounding = Rounding });
    draw_list.addCircleFilled(.{ .p = .{ dpad_x, dpad_y }, .r = @sqrt(2.0) * dpad_width, .col = bg_color, .num_segments = 0 });

    draw_analog_stick(d, port, draw_list, s, .{ x + s * 40, y + s * 65 }, bg_color, "left", gamepad_id, axes[2], axes[3]);
    if (has_right_stick)
        draw_analog_stick(d, port, draw_list, s, .{ x + s * 135, y + s * 110 }, bg_color, "right", gamepad_id, axes[4], axes[5]);

    const buttons_x = x + s * 158;
    const buttons_y = y + s * 78;
    const button_radius = s * 7;
    const button_distance = s * 14.0;
    inline for (.{
        .{ .p = .{ buttons_x, buttons_y + button_distance }, .button = buttons.a, .color = 0xFF2751F2, .field_name = "a" },
        .{ .p = .{ buttons_x + button_distance, buttons_y }, .button = buttons.b, .color = 0xFFEEA200, .field_name = "b" },
        .{ .p = .{ buttons_x - button_distance, buttons_y }, .button = buttons.x, .color = 0xFF07B8FF, .field_name = "x" },
        .{ .p = .{ buttons_x, buttons_y - button_distance }, .button = buttons.y, .color = 0xFF02B97E, .field_name = "y" },
    }, 0..) |button, idx| {
        zgui.pushIntId(@intCast(idx));
        defer zgui.popId();
        zgui.setCursorScreenPos(.{ button.p[0] - button_radius, button.p[1] - button_radius });
        if (zgui.invisibleButton("##FaceButton", .{ .w = 2 * button_radius, .h = 2 * button_radius, .flags = .{ .mouse_button_left = true, .mouse_button_right = true, .mouse_button_middle = true } }))
            bind_button(d, port, button.field_name, gamepad_id);
        controller_binding_tooltip(d, port, button.field_name);
        draw_list.addCircleFilled(.{ .p = button.p, .r = button_radius, .col = if (button.button == 0) PressedButtonColor else button.color, .num_segments = 0 });
        draw_list.addCircle(.{ .p = button.p, .r = button_radius, .col = Color, .num_segments = 0, .thickness = Thickness });
    }

    // Start Button (Triangle)
    const start_x = x + s * 100.0;
    const start_y = y + s * 115.0;
    const start_size = s * 20.0;
    zgui.setCursorScreenPos(.{ start_x - start_size / 2.0, start_y });
    if (zgui.invisibleButton("##Start", .{ .w = start_size, .h = start_size, .flags = .{ .mouse_button_left = true, .mouse_button_right = true, .mouse_button_middle = true } }))
        bind_button(d, port, "start", gamepad_id);
    controller_binding_tooltip(d, port, "start");
    draw_list.addTriangleFilled(.{
        .p1 = .{ start_x, start_y },
        .p2 = .{ start_x + start_size / 2.0, start_y + start_size },
        .p3 = .{ start_x - start_size / 2.0, start_y + start_size },
        .col = if (buttons.start == 0) PressedButtonColor else bg_color,
    });
    draw_list.addTriangle(.{
        .p1 = .{ start_x, start_y },
        .p2 = .{ start_x + start_size / 2.0, start_y + start_size },
        .p3 = .{ start_x - start_size / 2.0, start_y + start_size },
        .col = Color,
        .thickness = Thickness,
    });

    zgui.setCursorScreenPos(.{ x, y + s * IntendedSize });

    // FIXME/TODO: Keeping this around for now, but ideally this shouldn't be needed anymore.

    if (gamepad_id) |gamepad| {
        if (zgui.collapsingHeader("Controller Bindings", .{})) {
            zgui.indent(.{});
            defer zgui.unindent(.{});
            inline for (std.meta.fields(Deecy.ControllerBindings)) |field| {
                if (zgui.button("Edit##" ++ field.name, .{})) {
                    const maybe_button = switch (field.type) {
                        ?zglfw.Gamepad.Button => wait_for.controller_button(d, gamepad),
                        ?zglfw.Gamepad.Axis => wait_for.controller_axis(d, gamepad),
                        else => @compileError("Unexpected field type"),
                    };
                    if (maybe_button) |button|
                        @field(d.config.controllers_bindings[port], field.name) = button;
                }
                zgui.sameLine(.{});
                {
                    zgui.pushStyleColor4f(.{ .c = .{ 0.86, 0.12, 0.15, 1.0 }, .idx = .button });
                    defer zgui.popStyleColor(.{ .count = 1 });
                    if (zgui.button("X##" ++ field.name, .{}))
                        @field(d.config.controllers_bindings[port], field.name) = null;
                }
                zgui.sameLine(.{});
                if (@field(d.config.controllers_bindings[port], field.name)) |key| {
                    zgui.text("{s}: {t}", .{ field.name, key });
                } else {
                    zgui.text("{s}: None", .{field.name});
                }
            }
        }
    }
    if (zgui.collapsingHeader("Keyboard Bindings", .{})) {
        zgui.indent(.{});
        defer zgui.unindent(.{});
        inline for (std.meta.fields(Deecy.KeyboardBindings)) |field| {
            if (zgui.button("Edit##" ++ field.name, .{})) {
                @field(d.config.keyboard_bindings[port], field.name) = wait_for.keyboard(d);
            }
            zgui.sameLine(.{});
            {
                zgui.pushStyleColor4f(.{ .c = .{ 0.86, 0.12, 0.15, 1.0 }, .idx = .button });
                defer zgui.popStyleColor(.{ .count = 1 });
                if (zgui.button("X##" ++ field.name, .{}))
                    @field(d.config.keyboard_bindings[port], field.name) = null;
            }
            zgui.sameLine(.{});
            if (@field(d.config.keyboard_bindings[port], field.name)) |key| {
                zgui.text("{s}: {t}", .{ field.name, key });
            } else {
                zgui.text("{s}: None", .{field.name});
            }
        }
    }
}
