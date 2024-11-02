const std = @import("std");

const zglfw = @import("zglfw");
const zgui = @import("zgui");
const zgpu = @import("zgpu");

const ui_log = std.log.scoped(.ui);

const nfd = @import("nfd");

const Deecy = @import("deecy.zig").Deecy;
const MapleModule = @import("maple.zig");

last_error: []const u8 = "",

display_vmus: bool = true,
vmu_displays: [4][2]?struct {
    texture: zgpu.TextureHandle,
    view: zgpu.TextureViewHandle,
    dirty: bool = false,
    data: [48 * 32 / 8]u8 = .{255} ** (48 * 32 / 8),
} = .{ .{ null, null }, .{ null, null }, .{ null, null }, .{ null, null } },

gctx: *zgpu.GraphicsContext,

pub fn init(_: std.mem.Allocator, gctx: *zgpu.GraphicsContext) @This() {
    var r: @This() = .{ .gctx = gctx };
    r.create_vmu_texture(0, 0);
    return r;
}

pub fn deinit(self: *@This()) void {
    for (self.vmu_displays) |*vmu_texture| {
        if (vmu_texture[0]) |texture| {
            self.gctx.releaseResource(texture.texture);
            self.gctx.releaseResource(texture.view);
            vmu_texture[0] = null;
        }
        if (vmu_texture[1]) |texture| {
            self.gctx.releaseResource(texture.texture);
            self.gctx.releaseResource(texture.view);
            vmu_texture[1] = null;
        }
    }
}

fn create_vmu_texture(self: *@This(), controller: u8, index: u8) void {
    const tex = self.gctx.createTexture(.{
        .usage = .{ .texture_binding = true, .copy_dst = true },
        .size = .{
            .width = 48,
            .height = 32,
            .depth_or_array_layers = 1,
        },
        .format = .bgra8_unorm,
        .mip_level_count = 1,
    });
    const view = self.gctx.createTextureView(tex, .{});
    self.vmu_displays[controller][index] = .{ .texture = tex, .view = view };
    self.upload_vmu_texture(controller, index);
}

pub fn update_vmu_screen(self: *@This(), data: [*]const u8, controller: u8, index: u8) void {
    if (self.vmu_displays[controller][index] == null) return;
    @memcpy(&self.vmu_displays[controller][index].?.data, data[0 .. 48 * 32 / 8]);
    self.vmu_displays[controller][index].?.dirty = true;
}

pub fn update_vmu_screen_0_0(self: *@This(), data: [*]const u8) void {
    self.update_vmu_screen(data, 0, 0);
}

pub fn upload_vmu_texture(self: *@This(), controller: u8, index: u8) void {
    const colors = [2][3]u8{ // bgr
        .{ 152, 135, 92 }, // "white"
        .{ 104, 43, 40 }, // "black"
    };

    var tex = &self.vmu_displays[controller][index].?;
    var pixels: [4 * 48 * 32]u8 = undefined;
    for (0..32) |r| {
        const row = tex.data[6 * (31 - r) .. 6 * ((31 - r) + 1)];
        for (0..6) |c| {
            var byte = row[5 - c];
            for (0..8) |b| {
                const color: [3]u8 = colors[byte & 0x1];
                const idx = 4 * (48 * r + (8 * c + b));
                pixels[idx + 0] = color[0];
                pixels[idx + 1] = color[1];
                pixels[idx + 2] = color[2];
                pixels[idx + 3] = 255;
                byte >>= 1;
            }
        }
    }
    self.gctx.queue.writeTexture(
        .{ .texture = self.gctx.lookupResource(tex.texture).? },
        .{ .bytes_per_row = 4 * 48, .rows_per_image = 32 },
        .{ .width = 48, .height = 32 },
        u8,
        &pixels,
    );
    tex.dirty = false;
}

pub fn draw_vmus(self: *@This(), editable: bool) void {
    if (!editable and !self.display_vmus) return;

    zgui.setNextWindowSize(.{ .w = 4 * 48, .h = 2 * 4 * 32, .cond = .first_use_ever });

    zgui.pushStyleVar2f(.{ .idx = .window_padding, .v = .{ 0.0, 0.0 } });

    if (zgui.begin("VMUs", .{ .flags = .{ .no_resize = !editable, .no_move = !editable, .no_title_bar = !editable, .no_mouse_inputs = !editable, .no_nav_inputs = !editable, .no_nav_focus = !editable, .no_background = !editable } })) {
        const win_width = zgui.getWindowSize()[0];
        for (0..self.vmu_displays.len) |controller| {
            for (0..2) |index| {
                if (self.vmu_displays[controller][index] != null) {
                    const tex = &self.vmu_displays[controller][index].?;
                    if (tex.dirty)
                        self.upload_vmu_texture(@intCast(controller), @intCast(index));
                    zgui.image(self.gctx.lookupResource(tex.view).?, .{ .w = win_width, .h = win_width * 32.0 / 48.0 });
                }
            }
        }
    }
    zgui.end();

    zgui.popStyleVar(.{});
}

pub fn draw(self: *@This(), d: *Deecy) !void {
    var error_popup_to_open: [:0]const u8 = "";

    if (zgui.beginMainMenuBar()) {
        if (zgui.beginMenu("File", true)) {
            if (zgui.menuItem("Load GDI", .{})) {
                const was_running = d.running;
                if (was_running) d.stop();
                const open_path = try nfd.openFileDialog("gdi", null);
                if (open_path) |path| {
                    defer nfd.freePath(path);
                    d.load_and_start(path) catch |err| {
                        switch (err) {
                            error.MissingFlash => error_popup_to_open = "Error: Missing Flash",
                            else => {
                                ui_log.err("Failed to load GDI: {s}", .{@errorName(err)});
                                error_popup_to_open = "Unknown error";
                            },
                        }
                    };
                }
            }
            zgui.separator();
            if (zgui.menuItem("Exit", .{})) {
                d.stop();
                d.window.setShouldClose(true);
            }
            zgui.endMenu();
        }

        if (zgui.beginMenu("DC", true)) {
            if (!d.running) {
                if (zgui.menuItem("Start", .{}))
                    d.start();
            } else {
                if (zgui.menuItem("Stop", .{}))
                    d.stop();
            }
            if (zgui.menuItem("Reset", .{})) {
                const was_running = d.running;
                if (was_running) d.stop();
                try d.dc.reset();
                if (was_running) d.start();
            }
            zgui.separator();
            if (zgui.beginMenu("Region", !d.running)) {
                if (zgui.menuItem("USA", .{ .selected = d.dc.region == .USA })) {
                    try d.dc.set_region(.USA);
                }
                if (zgui.menuItem("Europe", .{ .selected = d.dc.region == .Europe })) {
                    try d.dc.set_region(.Europe);
                }
                if (zgui.menuItem("Japan", .{ .selected = d.dc.region == .Japan })) {
                    try d.dc.set_region(.Japan);
                }
                zgui.endMenu();
            }
            zgui.separator();
            if (zgui.beginMenu("Cable", true)) {
                zgui.textColored(.{ 1.0, 1.0, 1.0, 0.5 }, "  (WIP!)", .{});
                if (zgui.menuItem("VGA", .{ .selected = d.dc.cable_type == .VGA })) {
                    d.dc.cable_type = .VGA;
                }
                if (zgui.menuItem("RGB", .{ .selected = d.dc.cable_type == .RGB })) {
                    d.dc.cable_type = .RGB;
                }
                if (zgui.menuItem("Composite", .{ .selected = d.dc.cable_type == .Composite })) {
                    d.dc.cable_type = .Composite;
                }
                zgui.endMenu();
            }
            zgui.separator();
            if (zgui.beginMenu("Save States", true)) {
                inline for (0..4) |i| {
                    if (zgui.menuItem("Save Slot " ++ std.fmt.comptimePrint("{d}", .{i}), .{ .shortcut = std.fmt.comptimePrint("F{d}", .{i + 1}) })) {
                        d.save_state(i) catch |err| {
                            ui_log.err("Failed to save state: {}", .{err});
                        };
                    }
                }
                zgui.separator();
                inline for (0..4) |i| {
                    if (zgui.menuItem("Load Slot " ++ std.fmt.comptimePrint("{d}", .{i}), .{ .enabled = d.save_state_slots[i], .shortcut = std.fmt.comptimePrint("F{d}", .{i + 5}) })) {
                        d.load_state(i) catch |err| {
                            ui_log.err("Failed to load state: {}", .{err});
                        };
                    }
                }
                zgui.endMenu();
            }
            zgui.endMenu();
        }
        if (zgui.beginMenu("Drive", true)) {
            // TODO
            if (zgui.menuItem("Swap Disk", .{})) {
                const open_path = try nfd.openFileDialog("gdi", null);
                const was_running = d.running;
                if (was_running) d.stop();
                if (open_path) |path| err_brk: {
                    defer nfd.freePath(path);
                    // TODO! Emulate opening the tray and inserting a new disk.
                    d.load_disk(path) catch |err| {
                        ui_log.err("Failed to load GDI: {s}", .{@errorName(err)});
                        self.last_error = "Failed to load GDI.";
                        zgui.openPopup("ErrorPopup", .{});
                        break :err_brk;
                    };
                    d.dc.gdrom.state = .Open;
                    if (was_running)
                        d.start();
                }
            }
            if (zgui.menuItem("Open Tray", .{})) {
                d.dc.gdrom.state = .Open;
            }
            if (zgui.menuItem("Remove Disk", .{ .enabled = d.dc.gdrom.disk != null and d.dc.gdrom.state == .Open })) {
                d.dc.gdrom.disk.?.deinit();
                d.dc.gdrom.disk = null;
            }
            if (zgui.menuItem("Close Tray", .{})) {
                d.dc.gdrom.state = .Standby;
            }
            zgui.endMenu();
        }
        if (zgui.beginMenu("Settings", true)) {
            if (zgui.menuItem("Display VMUs", .{ .selected = self.display_vmus })) {
                self.display_vmus = !self.display_vmus;
            }
            zgui.separator();
            if (zgui.menuItem("Debug Menu", .{ .selected = d.config.display_debug_ui })) {
                d.config.display_debug_ui = !d.config.display_debug_ui;
            }
            zgui.endMenu();
        }
        zgui.endMainMenuBar();
    }

    if (zgui.begin("Settings", .{})) {
        if (zgui.beginTabBar("SettingsTabBar", .{})) {
            if (zgui.beginTabItem("General", .{})) {
                var method = d.config.cpu_throttling_method;
                if (zgui.comboFromEnum("CPU Throttling Method", &method)) {
                    d.set_throttle_method(method);
                }
                zgui.endTabItem();
            }

            if (zgui.beginTabItem("Renderer", .{})) {
                zgui.text("Resolution: {d}x{d}", .{ d.renderer.resolution.width, d.renderer.resolution.height });
                if (zgui.comboFromEnum("Display Mode", &d.renderer.display_mode))
                    d.renderer.update_blit_to_screen_vertex_buffer();
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
                    if (zgui.checkbox("Controller #" ++ number, .{ .v = &connected })) {
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
                    if (zgui.beginCombo("Device" ++ number, .{ .preview_value = name })) {
                        for (available_controllers.items, 0..) |item, index| {
                            if (available_controllers.items[index].id) |id| {
                                if (zgui.selectable(item.name, .{ .selected = d.controllers[i] != null and d.controllers[i].?.id == id }))
                                    d.controllers[i] = .{ .id = id };
                            } else {
                                if (zgui.selectable(item.name, .{ .selected = d.controllers[i] == null }))
                                    d.controllers[i] = null;
                            }
                        }
                        zgui.endCombo();
                    }
                    if (d.controllers[i]) |*j| {
                        _ = zgui.sliderFloat("Deadzone##" ++ number, .{ .v = &j.deadzone, .min = 0.0, .max = 1.0, .flags = .{} });
                    }

                    if (d.dc.maple.ports[i].main) |peripheral| {
                        switch (peripheral) {
                            .Controller => |controller| {
                                zgui.textColored(if (controller.buttons.a == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[A] ", .{});
                                zgui.sameLine(.{});
                                zgui.textColored(if (controller.buttons.b == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[B] ", .{});
                                zgui.sameLine(.{});
                                zgui.textColored(if (controller.buttons.x == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[X] ", .{});
                                zgui.sameLine(.{});
                                zgui.textColored(if (controller.buttons.y == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[Y] ", .{});
                                zgui.sameLine(.{});
                                zgui.textColored(if (controller.buttons.start == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[Start] ", .{});

                                zgui.textColored(if (controller.buttons.up == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[^] ", .{});
                                zgui.sameLine(.{});
                                zgui.textColored(if (controller.buttons.down == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[v] ", .{});
                                zgui.sameLine(.{});
                                zgui.textColored(if (controller.buttons.left == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[<] ", .{});
                                zgui.sameLine(.{});
                                zgui.textColored(if (controller.buttons.right == 0) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, "[>] ", .{});

                                const capabilities: MapleModule.InputCapabilities = @bitCast(MapleModule.Controller.Subcapabilities[0]);
                                var buf: [64]u8 = undefined;
                                inline for (0..6) |axis| {
                                    if (@field(capabilities, ([_][]const u8{ "analogRtrigger", "analogLtrigger", "analogHorizontal", "analogVertical", "analogHorizontal2", "analogVertical2" })[axis]) == 0) continue;
                                    const value = controller.axis[axis];
                                    const overlay = try std.fmt.bufPrintZ(&buf, "{s} {d}/255", .{ .{ "R", "L", "H", "V", "H2", "V2" }[axis], value });
                                    _ = zgui.progressBar(.{
                                        .fraction = @as(f32, @floatFromInt(value)) / 255.0,
                                        .overlay = overlay,
                                    });
                                }
                            },
                            else => {},
                        }
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

    // NOTE: Modals have to be in the same ID stack as the openPopup call :(
    //       Hence the weird workaround.
    if (error_popup_to_open.len > 0) {
        zgui.openPopup(error_popup_to_open, .{});
    }

    if (zgui.beginPopupModal("Error: Missing Flash", .{ .flags = .{ .always_auto_resize = true } })) {
        zgui.text("Failed to load flash. Did you put a copy of 'dc_flash.bin' in 'data/'?", .{});
        if (zgui.button("OK", .{})) {
            zgui.closeCurrentPopup();
        }
        zgui.endPopup();
    }
    if (zgui.beginPopupModal("Unknown error", .{})) {
        zgui.text("The console might have more information!", .{});
        if (zgui.button("OK", .{})) {
            zgui.closeCurrentPopup();
        }
        zgui.endPopup();
    }
}
