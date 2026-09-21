var allow_editing = false;

pub fn draw(d: *Deecy) !?enum { New, NewFromState, Save, SaveAs, Load, Stop, Record, Play } {
    try d.input_recording.mutex.lock(d.io);
    defer d.input_recording.mutex.unlock(d.io);

    defer zgui.end();
    if (zgui.begin("Input Editor", .{ .flags = .{ .menu_bar = true } })) {
        if (zgui.beginMenuBar()) {
            defer zgui.endMenuBar();
            if (zgui.beginMenu("File", true)) {
                defer zgui.endMenu();
                if (zgui.menuItem(Icons.FileCirclePlus ++ " New", .{}))
                    return .New;
                if (zgui.menuItem(Icons.FileCircleCheck ++ " New from current state", .{}))
                    return .NewFromState;
                if (zgui.menuItem(Icons.FileImport ++ " Open", .{}))
                    return .Load;
                zgui.separator();
                if (zgui.menuItem(Icons.FileExport ++ " Save", .{ .enabled = d.input_recording.record != null }))
                    return .Save;
                if (zgui.menuItem(Icons.FileExport ++ " Save As...", .{ .enabled = d.input_recording.record != null }))
                    return .SaveAs;
            }
        }

        zgui.textUnformattedColored(common.Yellow, Icons.TriangleExclamation);
        zgui.sameLine(.{});
        zgui.textUnformatted("Input recording is experimental.");
        if (!d.config.rewind.enabled) {
            zgui.textUnformattedColored(common.Yellow, Icons.TriangleExclamation);
            zgui.sameLine(.{});
            zgui.textUnformatted("Rewind is disabled, enabling it is recommended for advanced editing.");
        }

        zgui.text("Status: {t}", .{d.input_recording.state});

        if (d.input_recording.record) |*self| {
            _ = common.toggle("Allow edition", .{ .v = &allow_editing });
            if (d.input_recording.path) |p|
                zgui.text("Loaded: {s}", .{p});

            if (zgui.button(Icons.Square ++ " Stop", .{}))
                return .Stop;
            zgui.sameLine(.{});
            if (zgui.button(Icons.Circle ++ " Record", .{}))
                return .Record;
            zgui.sameLine(.{});
            if (zgui.button(Icons.Play ++ " Play", .{}))
                return .Play;

            zgui.text("Game ID: '{s}' ({s})", .{ self.game_id.name, self.game_id.id });

            const editable = allow_editing and !d.running;
            const right_stick = false; // TODO

            if (zgui.beginTabBar("Ports", .{})) {
                defer zgui.endTabBar();
                inline for (0..4) |port| {
                    switch (self.ports[port]) {
                        .none => {},
                        inline .controller => |c| {
                            if (zgui.beginTabItem("Port " ++ .{ "A", "B", "C", "D" }[port], .{})) {
                                defer zgui.endTabItem();
                                if (d.input_recording.state == .Playing) {
                                    zgui.text(Icons.Play ++ " Playing: {d}/{d}", .{ d.input_recording.cursors[port], c.inputs.items.len });
                                } else {
                                    zgui.text("Entry count: {d}", .{c.inputs.items.len});
                                }

                                if (zgui.beginChild("Inputs", .{ .w = 0, .h = @max(300, zgui.getContentRegionAvail()[1]) })) {
                                    var clipper: zgui.ListClipper = .init();
                                    clipper.begin(@intCast(c.inputs.items.len), null);
                                    var draw_list = zgui.getWindowDrawList();
                                    const clip_min = zgui.getWindowPos();
                                    const clip_size = zgui.getContentRegionAvail();
                                    const spacing = 10.0;
                                    draw_list.pushClipRect(.{ .pmin = clip_min, .pmax = .{ clip_min[0] + clip_size[0], clip_min[1] + clip_size[1] } });
                                    defer draw_list.popClipRect();
                                    while (clipper.step()) {
                                        const start: usize = @intCast(clipper.DisplayStart);
                                        for (c.inputs.items[start..@intCast(clipper.DisplayEnd)], start..) |*entry, idx| {
                                            zgui.pushIntId(@intCast(idx));
                                            defer zgui.popId();
                                            zgui.text("{s} {d: >8} {d: >6}M", .{
                                                if (d.input_recording.state == .Playing and start + idx == d.input_recording.cursors[port]) Icons.AngleRight else "  ",
                                                idx,
                                                entry.cycle / 1_000_000,
                                            });
                                            inline for (.{
                                                .{ "a", "A", ButtonColors.a },
                                                .{ "b", "B", ButtonColors.b },
                                                .{ "x", "X", ButtonColors.x },
                                                .{ "y", "Y", ButtonColors.y },
                                            }) |f| {
                                                zgui.sameLine(.{ .spacing = spacing });
                                                const pos = zgui.getCursorScreenPos();
                                                if (editable) {
                                                    if (zgui.invisibleButton(f[0], .{ .w = 16.0, .h = 16.0 }))
                                                        @field(entry.input.buttons, f[0]) ^= 1;
                                                    if (zgui.isItemHovered(.{})) zgui.setMouseCursor(.hand);
                                                    zgui.setCursorScreenPos(pos);
                                                }
                                                const pressed = @field(entry.input.buttons, f[0]) == 0;
                                                if (pressed) {
                                                    draw_list.addCircleFilled(.{ .p = .{ pos[0] + 4.0, pos[1] + 8.0 }, .r = 8.0, .col = f[2] });
                                                }
                                                zgui.textColored(if (pressed) common.White else .{ 0.2, 0.2, 0.2, 1.0 }, "{s}", .{f[1]});
                                            }
                                            {
                                                zgui.sameLine(.{ .spacing = spacing });
                                                const pos = zgui.getCursorScreenPos();
                                                if (editable) {
                                                    if (zgui.invisibleButton("start", .{ .w = 16.0, .h = 16.0 }))
                                                        entry.input.buttons.start ^= 1;
                                                    if (zgui.isItemHovered(.{})) zgui.setMouseCursor(.hand);
                                                    zgui.setCursorScreenPos(pos);
                                                }
                                                const pressed = entry.input.buttons.start == 0;
                                                if (pressed) {
                                                    draw_list.addTriangleFilled(.{
                                                        .p1 = .{ pos[0] - 4.0 + 8.0, pos[1] - 1.0 },
                                                        .p2 = .{ pos[0] - 4.0, pos[1] - 1.0 + 16.0 },
                                                        .p3 = .{ pos[0] - 4.0 + 16.0, pos[1] - 1.0 + 16.0 },
                                                        .col = 0xFF808080,
                                                    });
                                                }
                                                zgui.textColored(if (pressed) common.White else .{ 0.2, 0.2, 0.2, 1.0 }, "S", .{});
                                            }
                                            inline for (.{
                                                .{ "up", Icons.ArrowUp },
                                                .{ "down", Icons.ArrowDown },
                                                .{ "left", Icons.ArrowLeft },
                                                .{ "right", Icons.ArrowRight },
                                            }) |f| {
                                                zgui.sameLine(.{ .spacing = spacing });
                                                const pos = zgui.getCursorScreenPos();
                                                if (editable) {
                                                    if (zgui.invisibleButton(f[0], .{ .w = 16.0, .h = 16.0 }))
                                                        @field(entry.input.buttons, f[0]) ^= 1;
                                                    if (zgui.isItemHovered(.{})) zgui.setMouseCursor(.hand);
                                                    zgui.setCursorScreenPos(pos);
                                                }
                                                zgui.textColored(if (@field(entry.input.buttons, f[0]) == 0) common.White else .{ 0.2, 0.2, 0.2, 1.0 }, "{s}", .{f[1]});
                                            }

                                            zgui.sameLine(.{ .spacing = spacing });
                                            draw_trigger(draw_list, editable, "RT", &entry.input.axis[0], 0xFF2751F2);
                                            zgui.sameLine(.{ .spacing = spacing });
                                            draw_trigger(draw_list, editable, "LT", &entry.input.axis[1], 0xFFEEA200);
                                            zgui.sameLine(.{ .spacing = spacing });
                                            draw_stick(draw_list, editable, "L", &entry.input.axis[2], &entry.input.axis[3]);
                                            if (right_stick) {
                                                zgui.sameLine(.{ .spacing = spacing });
                                                draw_stick(draw_list, editable, "R", &entry.input.axis[4], &entry.input.axis[5]);
                                            }
                                        }
                                    }
                                    clipper.end();
                                }
                                zgui.endChild();
                            }
                        },
                    }
                }
            }
        }
    }

    return null;
}

fn draw_axis_drag(editable: bool, comptime label: [:0]const u8, value: *u8) void {
    var local_value: i32 = value.*;
    if (editable) {
        zgui.setNextItemWidth(16.0);
        zgui.pushStyleVar2f(.{ .idx = .frame_padding, .v = .{ 0.0, 0.0 } });
        defer zgui.popStyleVar(.{});
        if (zgui.dragInt(label, .{ .v = &local_value, .speed = 1.0, .min = 0, .max = 0xFF, .cfmt = "%02X" }))
            value.* = @intCast(local_value);
    } else {
        zgui.text("{X:0>2}", .{value.*});
    }
}

fn draw_trigger(draw_list: zgui.DrawList, editable: bool, comptime label: [:0]const u8, value: *u8, color: u32) void {
    zgui.textUnformatted(label);
    zgui.sameLine(.{});
    const pos = zgui.getCursorScreenPos();
    const width = 6.0;
    const height = 16.0;
    const pmax = .{ pos[0] + width, pos[1] + height };
    const fill = height * (@as(f32, @floatFromInt(value.*)) / 255.0);
    draw_list.addRectFilled(.{ .pmin = pos, .pmax = pmax, .col = 0xFF303030 });
    draw_list.addRectFilled(.{ .pmin = .{ pos[0], pmax[1] - fill }, .pmax = pmax, .col = color });
    draw_list.addRect(.{ .pmin = pos, .pmax = pmax, .col = 0xFF808080, .flags = .{}, .thickness = 1.0 });
    zgui.dummy(.{ .w = width, .h = height });
    zgui.sameLine(.{});
    draw_axis_drag(editable, "##" ++ label, value);
}

fn draw_stick(draw_list: zgui.DrawList, editable: bool, comptime label: [:0]const u8, x_value: *u8, y_value: *u8) void {
    const pos = zgui.getCursorScreenPos();
    const size = 16.0;
    const center = size / 2.0;
    const knob = 3.0;
    const x = center + (@as(f32, @floatFromInt(x_value.*)) - 128.0) * center / 128.0;
    const y = center + (@as(f32, @floatFromInt(y_value.*)) - 128.0) * center / 128.0;
    draw_list.addRectFilled(.{ .pmin = pos, .pmax = .{ pos[0] + size, pos[1] + size }, .col = 0xFF303030 });
    draw_list.addRect(.{ .pmin = pos, .pmax = .{ pos[0] + size, pos[1] + size }, .col = 0xFF606060, .flags = .{}, .thickness = 1.0 });
    draw_list.addCircle(.{ .p = .{ pos[0] + center, pos[1] + center }, .r = size / 2, .col = 0xFF808080 });
    draw_list.addLine(.{ .p1 = .{ pos[0] + center, pos[1] }, .p2 = .{ pos[0] + center, pos[1] + size }, .col = 0xFF404040, .thickness = 1.0 });
    draw_list.addLine(.{ .p1 = .{ pos[0], pos[1] + center }, .p2 = .{ pos[0] + size, pos[1] + center }, .col = 0xFF404040, .thickness = 1.0 });
    draw_list.addCircleFilled(.{ .p = .{ pos[0] + x, pos[1] + y }, .r = knob, .col = 0xFFFFFFFF });
    zgui.dummy(.{ .w = size, .h = size });
    zgui.sameLine(.{});
    draw_axis_drag(editable, "##X" ++ label, x_value);
    zgui.sameLine(.{ .spacing = 4.0 });
    draw_axis_drag(editable, "##Y" ++ label, y_value);
}

const std = @import("std");
const zgui = @import("zgui");
const common = @import("common.zig");
const Icons = common.Icons;
const Deecy = @import("../deecy.zig");
const ButtonColors = @import("controller_settings.zig").ButtonColors;
