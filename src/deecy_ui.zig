const std = @import("std");

const zglfw = @import("zglfw");
const zgui = @import("zgui");
const zguiExtra = @import("zgui_extra.zig");

const ui_log = std.log.scoped(.ui);

const nfd = @import("nfd");

const Deecy = @import("deecy.zig").Deecy;

last_error: []const u8 = "",

pub fn init(_: std.mem.Allocator) @This() {
    return .{};
}

pub fn deinit(_: *@This()) void {}

pub fn draw(self: *@This(), d: *Deecy) !void {
    if (zgui.beginMainMenuBar()) {
        if (zgui.beginMenu("File", true)) {
            if (zgui.menuItem("Load GDI", .{})) {
                const open_path = try nfd.openFileDialog("gdi", null);
                if (open_path) |path| err_brk: {
                    defer nfd.freePath(path);
                    const was_running = d.running;
                    if (was_running) d.stop();
                    d.load_disk(path) catch |err| {
                        ui_log.err("Failed to load GDI: {s}", .{@errorName(err)});
                        self.last_error = "Failed to load GDI.";
                        zgui.openPopup("ErrorPopup", .{});
                        break :err_brk;
                    };
                    d.dc.set_region(d.dc.gdrom.disk.?.get_region()) catch |err| {
                        ui_log.err("Failed to set region: {s}", .{@errorName(err)});
                        self.last_error = "Failed to set region. Did you put bios and flash files in 'data/[region]/' (e.g. '/data/us/dc_boot.bin')?";
                        zgui.openPopup("ErrorPopup", .{});
                        break :err_brk;
                    };

                    d.on_game_load() catch |err| {
                        ui_log.err("Error while setting up game: {s}", .{@errorName(err)});
                        self.last_error = "Error while setting up game.";
                        zgui.openPopup("ErrorPopup", .{});
                        break :err_brk;
                    };

                    if (was_running) try d.dc.reset();
                    d.start();
                    d.display_ui = false;
                }
            }
            if (zgui.menuItem("Swap Disk", .{ .enabled = false })) {
                // TODO! Emulate opening the tray and inserting a new disk.
            }
            zgui.separator();
            if (zgui.menuItem("Reset", .{})) {
                const was_running = d.running;
                if (was_running) d.stop();
                try d.dc.reset();
                if (was_running) d.start();
            }
            zgui.separator();
            if (zgui.menuItem("Exit", .{})) {
                d.stop();
                d.window.setShouldClose(true);
            }
            zgui.endMenu();
        }
        if (zgui.beginMenu("Settings", true)) {
            if (zgui.menuItem("Debug Menu", .{ .selected = d.config.display_debug_ui })) {
                d.config.display_debug_ui = !d.config.display_debug_ui;
            }
            zgui.endMenu();
        }
        zgui.endMainMenuBar();
    }

    if (zgui.begin("Settings", .{})) {
        if (zgui.beginTabBar("SettingsTabBar", .{})) {
            if (zgui.beginTabItem("CPU", .{})) {
                var method = d.config.cpu_throttling_method;
                if (zguiExtra.selectEnum("CPU Throttling Method", &method)) {
                    d.set_throttle_method(method);
                }
                zgui.endTabItem();
            }

            if (zgui.beginTabItem("Controls", .{})) {
                var available_controllers = std.ArrayList(struct { id: ?zglfw.Joystick.Id, name: [:0]const u8 }).init(d._allocator);
                defer available_controllers.deinit();

                try available_controllers.append(.{ .id = null, .name = "None" });

                for (0..zglfw.Joystick.maximum_supported) |idx| {
                    const jid: zglfw.Joystick.Id = @intCast(idx);
                    if (zglfw.Joystick.get(jid)) |joystick| {
                        if (joystick.asGamepad()) |gamepad| {
                            try available_controllers.append(.{ .id = jid, .name = gamepad.getName() });
                        }
                    }
                }

                inline for (0..4) |i| {
                    zgui.pushIntId(i);
                    defer zgui.popId();

                    if (i != 0) zgui.separator();

                    const number = std.fmt.comptimePrint("{d}", .{i + 1});
                    var connected: bool = d.dc.maple.ports[i].main != null;
                    if (zgui.checkbox("Connected##" ++ number, .{ .v = &connected })) {
                        if (d.dc.maple.ports[i].main != null) {
                            d.dc.maple.ports[i].main = null;
                        } else {
                            d.dc.maple.ports[i].main = .{ .Controller = .{} };
                        }
                    }
                    const name = if (d.controllers[i]) |j|
                        (if (zglfw.Joystick.get(j.id)) |joystick|
                            (if (joystick.asGamepad()) |gamepad| gamepad.getName() else "None")
                        else
                            "None")
                    else
                        "None";
                    if (zgui.beginCombo("Controller #" ++ number, .{ .preview_value = name })) {
                        for (available_controllers.items, 0..) |item, index| {
                            const idx = @as(u32, @intCast(index));
                            if (zgui.selectable(item.name, .{ .selected = d.controllers[i] != null and d.controllers[i].?.id == available_controllers.items[idx].id }))
                                d.controllers[i] = .{ .id = available_controllers.items[idx].id.? };
                        }
                        zgui.endCombo();
                    }
                    if (d.controllers[i]) |*j| {
                        _ = zgui.sliderFloat("Deadzone##" ++ number, .{ .v = &j.deadzone, .min = 0.1, .max = 1.0, .flags = .{} });
                    }
                    if (d.dc.maple.ports[i].main) |_| {
                        for (d.dc.maple.ports[i].subperipherals) |sub| {
                            if (sub) |*s| {
                                switch (s.*) {
                                    .VMU => |vmu| {
                                        zgui.text("VMU: {s}", .{vmu.backing_file_path});
                                    },
                                    else => {
                                        zgui.text("(TODO!)", .{});
                                    },
                                }
                            } else {
                                zgui.textColored(.{ 1.0, 1.0, 1.0, 0.5 }, "  (Empty)", .{});
                            }
                        }
                    }
                }

                zgui.endTabItem();
            }

            if (zgui.beginTabItem("Audio", .{})) {
                var volume = try d.audio_device.getMasterVolume();
                if (zgui.sliderFloat("Volume", .{ .v = &volume, .min = 0.0, .max = 1.0, .flags = .{} })) {
                    try d.audio_device.setMasterVolume(volume);
                }
                zgui.endTabItem();
            }

            zgui.endTabBar();
        }
    }
    zgui.end();

    const center = zgui.getMainViewport().getCenter();
    zgui.setNextWindowPos(.{ .cond = .appearing, .x = center[0], .y = center[1] });

    if (zgui.beginPopupModal("ErrorPopup", .{})) {
        zgui.text("{s}", .{self.last_error});
        if (zgui.button("OK", .{})) {
            zgui.closeCurrentPopup();
        }
        zgui.endPopup();
    }
}
