const std = @import("std");
const termcolor = @import("termcolor");

const zglfw = @import("zglfw");
const zgui = @import("zgui");
const zgpu = @import("zgpu");

const ui_log = std.log.scoped(.ui);

const nfd = @import("nfd");

const Deecy = @import("deecy.zig");
const MapleModule = @import("maple.zig");
const GDI = @import("gdi.zig");
const Colors = @import("colors.zig");
const PVRFile = @import("pvr_file.zig");

const GameFile = struct {
    path: [:0]const u8,
    name: [:0]const u8,
    texture: ?zgpu.TextureHandle,
    view: ?zgpu.TextureViewHandle,

    pub fn free(self: *@This(), allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) void {
        allocator.free(self.path);
        allocator.free(self.name);
        if (self.texture) |texture| {
            gctx.releaseResource(texture);
            self.texture = null;
        }
        if (self.view) |view| {
            gctx.releaseResource(view);
            self.view = null;
        }
    }
};

last_error: []const u8 = "",

display_vmus: bool = true,
vmu_displays: [4][2]?struct {
    texture: zgpu.TextureHandle,
    view: zgpu.TextureViewHandle,
    dirty: bool = false,
    data: [48 * 32 / 8]u8 = .{255} ** (48 * 32 / 8),
} = .{ .{ null, null }, .{ null, null }, .{ null, null }, .{ null, null } },

display_library: bool = false,
game_directory: ?[]const u8 = null,
gdi_files: std.ArrayList(GameFile),

gctx: *zgpu.GraphicsContext,
allocator: std.mem.Allocator,

pub fn init(allocator: std.mem.Allocator, gctx: *zgpu.GraphicsContext) !@This() {
    var r: @This() = .{
        .gdi_files = std.ArrayList(GameFile).init(allocator),
        .gctx = gctx,
        .allocator = allocator,
    };
    r.game_directory = try allocator.dupe(u8, "D:/DC Games/"); // TODO: Load default game directory from config (when we have one :)))
    r.create_vmu_texture(0, 0);
    try r.refresh_games();
    return r;
}

pub fn deinit(self: *@This()) void {
    for (self.gdi_files.items) |entry| entry.free(self.allocator);
    self.gdi_files.deinit();
    self.allocator.free(self.game_directory);

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

fn refresh_games(self: *@This()) !void {
    if (self.game_directory) |dir_path| {
        const start = std.time.milliTimestamp();

        for (self.gdi_files.items) |*entry| entry.free(self.allocator, self.gctx);
        self.gdi_files.clearRetainingCapacity();

        var dir = std.fs.cwd().openDir(dir_path, .{ .iterate = true }) catch |err| {
            ui_log.err(termcolor.red("Failed to open game directory: {s}"), .{@errorName(err)});
            return;
        };
        defer dir.close();
        var walker = try dir.walk(self.allocator);
        defer walker.deinit();

        const tex_buffer: []u8 = try self.allocator.alloc(u8, 1024 * 1024);
        defer self.allocator.free(tex_buffer);

        while (try walker.next()) |entry| {
            if (entry.kind == .file and std.mem.endsWith(u8, entry.path, ".gdi")) {
                const path = try std.fs.path.joinZ(self.allocator, &[_][]const u8{ dir_path, entry.path });

                var gdi = try GDI.GDI.init(path, self.allocator);
                defer gdi.deinit();

                var texture: ?zgpu.TextureHandle = null;
                var view: ?zgpu.TextureViewHandle = null;

                if (gdi.load_file("0GDTEX.PVR;1", tex_buffer)) |len| {
                    if (PVRFile.decode(self.allocator, tex_buffer[0..len])) |result| {
                        defer self.allocator.free(result.bgra);

                        texture = self.gctx.createTexture(.{
                            .usage = .{ .texture_binding = true, .copy_dst = true },
                            .size = .{
                                .width = result.width,
                                .height = result.height,
                                .depth_or_array_layers = 1,
                            },
                            .format = .bgra8_unorm,
                            .mip_level_count = 1,
                        });

                        view = self.gctx.createTextureView(texture.?, .{});

                        self.gctx.queue.writeTexture(
                            .{ .texture = self.gctx.lookupResource(texture.?).? },
                            .{ .bytes_per_row = 4 * result.width, .rows_per_image = result.height },
                            .{ .width = result.width, .height = result.height },
                            u8,
                            result.bgra,
                        );
                    } else |err| {
                        ui_log.err(termcolor.red("[{s}] Failed to decode 0GDTEX.PVR: {any}"), .{ entry.basename[0..16], err });
                    }
                } else |err| {
                    ui_log.info("[{s}] Failed to find 0GDTEX.PVR: {any}", .{ entry.basename[0..16], err });
                }

                const name = try self.allocator.dupeZ(u8, entry.basename);
                errdefer self.allocator.free(name);

                try self.gdi_files.append(.{
                    .name = name,
                    .path = path,
                    .texture = texture,
                    .view = view,
                });
            }
        }

        const end = std.time.milliTimestamp();
        ui_log.info("Checked {d} GDI files in {d}ms", .{ self.gdi_files.items.len, end - start });
    }
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

    if (d.dc.gdrom.disk == null or self.display_library)
        self.draw_game_library(d) catch |err| {
            switch (err) {
                error.MissingFlash => error_popup_to_open = "Error: Missing Flash",
                else => error_popup_to_open = "Unknown error",
            }
        };

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
                zgui.text("Curent Resolution: {d}x{d}", .{ d.renderer.resolution.width, d.renderer.resolution.height });
                var resolution: enum(u8) { Native = 1, x2 = 2, x3 = 3, x4 = 4 } = @enumFromInt(d.renderer.resolution.width / Deecy.Renderer.NativeResolution.width);
                if (zgui.comboFromEnum("Resolution", &resolution)) {
                    // NOTE: This might not be the best idea to do this here without explicit synchronization but... This has worked flawlessly so far.
                    d.renderer.resolution = .{ .width = Deecy.Renderer.NativeResolution.width * @intFromEnum(resolution), .height = Deecy.Renderer.NativeResolution.height * @intFromEnum(resolution) };
                    d.renderer.on_inner_resolution_change();
                }
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

pub fn draw_game_library(self: *@This(), d: *Deecy) !void {
    const target_width = 4 * 256 + 50;
    zgui.setNextWindowPos(.{ .x = @floatFromInt((@max(target_width, d.gctx.swapchain_descriptor.width) - target_width) / 2), .y = 24, .cond = .always });
    zgui.setNextWindowSize(.{ .w = target_width, .h = @floatFromInt(@max(48, d.gctx.swapchain_descriptor.height) - 48), .cond = .always });

    if (zgui.begin("Games", .{ .flags = .{ .no_resize = true, .no_move = true, .no_title_bar = true } })) {
        if (self.game_directory) |dir| {
            zgui.text("Directory: {s}", .{dir});
        } else {
            zgui.text("Directory: None", .{});
        }
        zgui.sameLine(.{});
        if (zgui.button("Refresh", .{})) {
            try self.refresh_games();
        }
        zgui.sameLine(.{});
        if (zgui.button("Change Directory", .{})) {
            const open_path = try nfd.openFolderDialog(null);
            if (open_path) |path| {
                defer nfd.freePath(path);
                if (self.game_directory) |old_dir| self.allocator.free(old_dir);
                self.game_directory = try self.allocator.dupe(u8, path);
                try self.refresh_games();
            }
        }

        zgui.pushStyleVar2f(.{ .idx = .frame_padding, .v = .{ 0, 0 } });
        for (self.gdi_files.items, 0..) |entry, idx| {
            var launch = false;
            zgui.beginGroup();
            zgui.pushStyleVar2f(.{ .idx = .item_spacing, .v = .{ 0, 0 } });
            zgui.pushIntId(@intCast(idx));
            launch = (zgui.button(entry.name, .{ .w = 256 })) or launch;

            zgui.pushStrId("image");
            if (entry.view) |view| {
                launch = zgui.imageButton(entry.name, self.gctx.lookupResource(view).?, .{ .w = 256, .h = 256 }) or launch;
            } else {
                launch = zgui.button(entry.name, .{ .w = 256, .h = 256 }) or launch;
            }
            zgui.popId();

            zgui.popId();
            zgui.popStyleVar(.{});
            zgui.endGroup();

            if (launch)
                try d.load_and_start(entry.path);

            if (idx % 4 != 3) zgui.sameLine(.{});
        }
        zgui.popStyleVar(.{});
    }
    zgui.end();
}
