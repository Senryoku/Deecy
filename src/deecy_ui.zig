const std = @import("std");
const termcolor = @import("termcolor");

const zglfw = @import("zglfw");
const zgui = @import("zgui");
const zgpu = @import("zgpu");

const ui_log = std.log.scoped(.ui);

const nfd = @import("nfd");

const Deecy = @import("deecy.zig");
const DreamcastModule = @import("dreamcast");
const MapleModule = DreamcastModule.Maple;
const Disc = DreamcastModule.GDROM.Disc;
const PVRFile = @import("pvr_file.zig");

const Notifications = @import("./ui/notifications.zig");

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

    pub fn sort(_: void, a: @This(), b: @This()) bool {
        return std.mem.order(u8, a.name, b.name) == .lt;
    }
};

last_error: []const u8 = "",

vmu_displays: [4][2]?struct {
    texture: zgpu.TextureHandle,
    view: zgpu.TextureViewHandle,
    dirty: bool = false,
    data: [48 * 32 / 8]u8 = .{255} ** (48 * 32 / 8),
} = .{ .{ null, null }, .{ null, null }, .{ null, null }, .{ null, null } },

display_library: bool = false,
disc_files: std.ArrayList(GameFile),
disc_files_mutex: std.Thread.Mutex = .{}, // Used during disc_files population (then assumed to be constant outside of refresh_games)

notifications: Notifications,

deecy: *Deecy,
allocator: std.mem.Allocator,

pub fn create(allocator: std.mem.Allocator, d: *Deecy) !*@This() {
    var r = try allocator.create(@This());
    r.* = .{
        .disc_files = std.ArrayList(GameFile).init(allocator),
        .notifications = Notifications.init(allocator),
        .deecy = d,
        .allocator = allocator,
    };
    r.create_vmu_texture(0, 0);
    return r;
}

pub fn destroy(self: *@This()) void {
    for (self.disc_files.items) |*entry| entry.free(self.allocator, self.deecy.gctx);
    self.disc_files.deinit();

    for (&self.vmu_displays) |*vmu_texture| {
        if (vmu_texture[0]) |texture| {
            self.deecy.gctx.releaseResource(texture.texture);
            self.deecy.gctx.releaseResource(texture.view);
            vmu_texture[0] = null;
        }
        if (vmu_texture[1]) |texture| {
            self.deecy.gctx.releaseResource(texture.texture);
            self.deecy.gctx.releaseResource(texture.view);
            vmu_texture[1] = null;
        }
    }

    self.allocator.destroy(self);
}

fn create_vmu_texture(self: *@This(), controller: u8, index: u8) void {
    const tex = self.deecy.gctx.createTexture(.{
        .usage = .{ .texture_binding = true, .copy_dst = true },
        .size = .{
            .width = 48,
            .height = 32,
            .depth_or_array_layers = 1,
        },
        .format = .bgra8_unorm,
        .mip_level_count = 1,
    });
    const view = self.deecy.gctx.createTextureView(tex, .{});
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
    self.deecy.gctx.queue.writeTexture(
        .{ .texture = self.deecy.gctx.lookupResource(tex.texture).? },
        .{ .bytes_per_row = 4 * 48, .rows_per_image = 32 },
        .{ .width = 48, .height = 32 },
        u8,
        &pixels,
    );
    tex.dirty = false;
}

pub fn draw_vmus(self: *@This(), editable: bool) void {
    if ((editable and self.deecy.config.display_debug_ui) or (!editable and !self.deecy.config.display_vmus)) return;

    zgui.setNextWindowSize(.{ .w = 4 * 48, .h = 2 * 4 * 32, .cond = .first_use_ever });
    zgui.setNextWindowPos(.{ .x = 32, .y = 32, .cond = .first_use_ever });

    zgui.pushStyleVar2f(.{ .idx = .window_padding, .v = .{ 0.0, 0.0 } });

    if (zgui.begin("VMUs", .{ .flags = .{ .no_resize = !editable, .no_move = !editable, .no_title_bar = !editable, .no_mouse_inputs = !editable, .no_nav_inputs = !editable, .no_nav_focus = !editable, .no_background = !editable, .no_docking = true } })) {
        const win_width = zgui.getWindowSize()[0];
        for (0..self.vmu_displays.len) |controller| {
            for (0..2) |index| {
                if (self.vmu_displays[controller][index] != null) {
                    const tex = &self.vmu_displays[controller][index].?;
                    if (tex.dirty)
                        self.upload_vmu_texture(@intCast(controller), @intCast(index));
                    zgui.image(self.deecy.gctx.lookupResource(tex.view).?, .{ .w = win_width, .h = win_width * 32.0 / 48.0 });
                }
            }
        }
    }
    zgui.end();

    zgui.popStyleVar(.{});
}

fn get_game_image(self: *@This(), path: []const u8) void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    var disc = Disc.init(path, allocator) catch |err| {
        ui_log.err("Failed to load disc '{s}': {s}", .{ path, @errorName(err) });
        return;
    };
    defer disc.deinit();

    const tex_buffer: []u8 = allocator.alloc(u8, 1024 * 1024) catch |err| {
        ui_log.err("Failed to allocate texture buffer: {s}", .{@errorName(err)});
        return;
    };
    defer allocator.free(tex_buffer);

    if (disc.load_file("0GDTEX.PVR;1", tex_buffer)) |len| {
        if (PVRFile.decode(allocator, tex_buffer[0..len])) |result| {
            defer result.deinit(allocator);

            self.deecy.gctx_queue_mutex.lock();
            const texture = self.deecy.gctx.createTexture(.{
                .usage = .{ .texture_binding = true, .copy_dst = true },
                .size = .{
                    .width = result.width,
                    .height = result.height,
                    .depth_or_array_layers = 1,
                },
                .format = .bgra8_unorm,
                .mip_level_count = 1,
            });
            const view = self.deecy.gctx.createTextureView(texture, .{});
            self.deecy.gctx.queue.writeTexture(
                .{ .texture = self.deecy.gctx.lookupResource(texture).? },
                .{ .bytes_per_row = 4 * result.width, .rows_per_image = result.height },
                .{ .width = result.width, .height = result.height },
                u8,
                result.bgra,
            );
            self.deecy.gctx_queue_mutex.unlock();

            {
                self.disc_files_mutex.lock();
                defer self.disc_files_mutex.unlock();
                for (self.disc_files.items) |*entry| {
                    if (std.mem.eql(u8, entry.path, path)) {
                        entry.texture = texture;
                        entry.view = view;
                        return;
                    }
                }
            }

            ui_log.err("Failed to find disc entry for '{s}'", .{path});
            self.deecy.gctx_queue_mutex.lock();
            defer self.deecy.gctx_queue_mutex.unlock();
            self.deecy.gctx.releaseResource(view);
            self.deecy.gctx.releaseResource(texture);
        } else |err| {
            ui_log.err(termcolor.red("Failed to decode 0GDTEX.PVR for '{s}': {any}"), .{ path, err });
        }
    } else |err| {
        ui_log.info("Failed to find 0GDTEX.PVR for '{s}': {any}", .{ path, err });
    }
}

pub fn refresh_games(self: *@This()) !void {
    if (self.deecy.config.game_directory) |dir_path| {
        const start = std.time.milliTimestamp();

        var threads = std.ArrayList(std.Thread).init(self.allocator);
        defer threads.deinit();

        {
            for (self.disc_files.items) |*entry| entry.free(self.allocator, self.deecy.gctx);
            self.disc_files.clearRetainingCapacity();

            var dir = std.fs.cwd().openDir(dir_path, .{ .iterate = true }) catch |err| {
                ui_log.err(termcolor.red("Failed to open game directory: {s}"), .{@errorName(err)});
                return;
            };
            defer dir.close();
            var walker = try dir.walk(self.allocator);
            defer walker.deinit();

            while (try walker.next()) |entry| {
                if (entry.kind == .file and (std.mem.endsWith(u8, entry.path, ".gdi") or std.mem.endsWith(u8, entry.path, ".cdi") or std.mem.endsWith(u8, entry.path, ".chd"))) {
                    const path = try std.fs.path.joinZ(self.allocator, &[_][]const u8{ dir_path, entry.path });

                    const name = try self.allocator.dupeZ(u8, entry.basename);
                    errdefer self.allocator.free(name);
                    {
                        self.disc_files_mutex.lock();
                        defer self.disc_files_mutex.unlock();
                        try self.disc_files.append(.{
                            .name = name,
                            .path = path,
                            .texture = null,
                            .view = null,
                        });
                    }
                    try threads.append(try std.Thread.spawn(.{}, get_game_image, .{ self, path }));
                }
            }
        }
        {
            self.disc_files_mutex.lock();
            defer self.disc_files_mutex.unlock();
            std.mem.sort(GameFile, self.disc_files.items, {}, GameFile.sort);
        }

        for (threads.items) |t| {
            t.join();
        }

        const end = std.time.milliTimestamp();
        ui_log.info("Checked {d} disc files in {d}ms", .{ self.disc_files.items.len, end - start });
    }
}

pub fn draw(self: *@This()) !void {
    const d = self.deecy;
    var error_popup_to_open: [:0]const u8 = "";

    if (zgui.beginMainMenuBar()) {
        if (zgui.beginMenu("File", true)) {
            if (zgui.menuItem("Load Disc", .{})) {
                const was_running = d.running;
                if (was_running) d.pause();
                const open_path = try nfd.openFileDialog("gdi,cdi,chd", null);
                if (open_path) |path| {
                    defer nfd.freePath(path);
                    d.load_and_start(path) catch |err| {
                        switch (err) {
                            error.MissingFlash => error_popup_to_open = "Error: Missing Flash",
                            else => {
                                ui_log.err("Failed to load disc: {s}", .{@errorName(err)});
                                error_popup_to_open = "Unknown error";
                            },
                        }
                    };
                }
            }
            zgui.separator();
            if (zgui.menuItem("Exit", .{})) {
                d.pause();
                zglfw.setWindowShouldClose(d.window, true);
            }
            zgui.endMenu();
        }

        if (zgui.beginMenu("Dreamcast", true)) {
            if (!d.running) {
                if (zgui.menuItem("Start", .{ .shortcut = "Space" }))
                    d.start();
            } else {
                if (zgui.menuItem("Pause", .{ .shortcut = "Space" }))
                    d.pause();
            }
            if (zgui.menuItem("Reset", .{}))
                try d.reset();
            if (zgui.menuItem("Stop", .{}))
                try d.stop();
            zgui.separator();

            var realtime = self.deecy.realtime;
            if (zgui.checkbox("Realtime", .{ .v = &realtime }))
                self.deecy.set_realtime(realtime);
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
            if (d.dc.gdrom.disc) |disc| {
                if (disc.get_product_name()) |name| {
                    zgui.textColored(.{ 1.0, 1.0, 1.0, 0.75 }, "Disc: {s}", .{name});
                } else {
                    zgui.textColored(.{ 1.0, 1.0, 1.0, 0.75 }, "Unknown Disc", .{});
                }
            } else {
                zgui.textColored(.{ 1.0, 1.0, 1.0, 0.5 }, "No Disc", .{});
            }
            zgui.separator();
            if (zgui.menuItem("Swap Disc", .{})) {
                const open_path = try nfd.openFileDialog("gdi,cdi,chd", null);
                const was_running = d.running;
                if (was_running) d.pause();
                if (open_path) |path| err_brk: {
                    defer nfd.freePath(path);
                    // TODO! Emulate opening the tray and inserting a new disc.
                    d.load_disc(path) catch |err| {
                        ui_log.err("Failed to load disc: {s}", .{@errorName(err)});
                        self.last_error = "Failed to load disc.";
                        zgui.openPopup("ErrorPopup", .{});
                        break :err_brk;
                    };
                    d.dc.gdrom.state = .Open;
                    if (was_running)
                        d.start();
                }
            }
            if (zgui.menuItem("Open Tray", .{ .enabled = d.dc.gdrom.state != .Open })) {
                d.dc.gdrom.state = .Open;
            }
            if (zgui.menuItem("Remove Disc", .{ .enabled = d.dc.gdrom.disc != null })) {
                const was_running = d.running;
                if (was_running) d.pause();
                d.dc.gdrom.disc.?.deinit();
                d.dc.gdrom.disc = null;
                if (d.dc.gdrom.state != .Open)
                    d.dc.gdrom.state = .Empty;
                if (was_running)
                    d.start();
            }
            if (zgui.menuItem("Close Tray", .{ .enabled = d.dc.gdrom.state == .Open })) {
                if (d.dc.gdrom.disc == null) {
                    d.dc.gdrom.state = .Empty;
                } else {
                    d.dc.gdrom.state = .Standby;
                }
            }
            zgui.endMenu();
        }
        if (zgui.beginMenu("Settings", true)) {
            if (zgui.menuItem("Display VMUs", .{ .selected = d.config.display_vmus })) {
                d.config.display_vmus = !d.config.display_vmus;
            }
            zgui.separator();
            if (zgui.menuItem("Debug Menu", .{ .selected = d.config.display_debug_ui })) {
                d.config.display_debug_ui = !d.config.display_debug_ui;
            }
            zgui.endMenu();
        }
        zgui.endMainMenuBar();
    }

    if (!d.running and d.dc.gdrom.disc == null or self.display_library)
        self.draw_game_library() catch |err| {
            // Most likely due to game loading, and not actually drawing the UI.
            switch (err) {
                error.MissingFlash => error_popup_to_open = "Error: Missing Flash",
                else => {
                    error_popup_to_open = "Unknown error";
                    ui_log.err(termcolor.red("Error: {s}"), .{@errorName(err)});
                },
            }
        };

    zgui.setNextWindowSize(.{ .w = 290, .h = 800, .cond = .first_use_ever });
    zgui.setNextWindowPos(.{ .x = 1408, .y = 32, .cond = .first_use_ever });

    if (zgui.begin("Settings", .{})) {
        if (zgui.beginTabBar("SettingsTabBar", .{})) {
            if (zgui.beginTabItem("Renderer", .{})) {
                var fullscreen = self.deecy.config.fullscreen;
                if (zgui.checkbox("Fullscreen", .{ .v = &fullscreen })) {
                    self.deecy.toggle_fullscreen();
                }
                zgui.text("Curent Resolution: {d}x{d}", .{ d.renderer.resolution.width, d.renderer.resolution.height });
                var resolution: enum(u8) { Native = 1, x2 = 2, x3 = 3, x4 = 4, x5 = 5 } = @enumFromInt(d.renderer.resolution.width / Deecy.Renderer.NativeResolution.width);
                if (zgui.comboFromEnum("Resolution", &resolution)) {
                    d.config.internal_resolution_factor = @intFromEnum(resolution);
                    // NOTE: This might not be the best idea to do this here without explicit synchronization but... This has worked flawlessly so far.
                    d.renderer.resolution = .{ .width = Deecy.Renderer.NativeResolution.width * @intFromEnum(resolution), .height = Deecy.Renderer.NativeResolution.height * @intFromEnum(resolution) };
                    d.renderer.on_inner_resolution_change();
                }
                if (zgui.comboFromEnum("Display Mode", &d.renderer.display_mode)) {
                    d.config.display_mode = d.renderer.display_mode;
                    d.renderer.update_blit_to_screen_vertex_buffer();
                }
                zgui.separator();
                _ = zgui.comboFromEnum("Present Mode (Restart required)", &d.config.present_mode);
                zgui.endTabItem();
            }

            if (zgui.beginTabItem("Controls", .{})) {
                var available_controllers = std.ArrayList(struct { id: ?zglfw.Joystick, name: [:0]const u8 }).init(d._allocator);
                defer available_controllers.deinit();

                try available_controllers.append(.{ .id = null, .name = "None" });

                for (0..zglfw.Joystick.maximum_supported) |idx| {
                    const joystick: zglfw.Joystick = @enumFromInt(idx);
                    if (joystick.isPresent()) {
                        if (joystick.asGamepad()) |gamepad| {
                            try available_controllers.append(.{ .id = joystick, .name = gamepad.getName() });
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
                        (if (j.id.isPresent())
                            (if (j.id.asGamepad()) |gamepad| gamepad.getName() else "None")
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
                if (zgui.sliderFloat("Volume", .{ .v = &d.config.audio_volume, .min = 0.0, .max = 1.0, .flags = .{} })) {
                    try d.audio_device.setMasterVolume(d.config.audio_volume);
                }
                var dsp_emulation = d.dc.aica.dsp_emulation;
                if (zgui.comboFromEnum("DSP", &dsp_emulation)) {
                    const was_running = d.running;
                    if (was_running) d.pause();
                    defer if (was_running) d.start();
                    d.dc.aica.dsp_emulation = dsp_emulation;
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
        if (zgui.button("OK", .{}))
            zgui.closeCurrentPopup();
        zgui.endPopup();
    }
    if (zgui.beginPopupModal("Unknown error", .{})) {
        zgui.text("The console might have more information!", .{});
        if (zgui.button("OK", .{}))
            zgui.closeCurrentPopup();
        zgui.endPopup();
    }
}

pub fn draw_game_library(self: *@This()) !void {
    const d = self.deecy;
    const target_width = 4 * 256 + 64;
    zgui.setNextWindowPos(.{ .x = @floatFromInt((@max(target_width, d.gctx.swapchain_descriptor.width) - target_width) / 2), .y = 32, .cond = .always });
    zgui.setNextWindowSize(.{ .w = target_width, .h = @floatFromInt(@max(48, d.gctx.swapchain_descriptor.height) - 64), .cond = .always });

    defer zgui.end();
    if (zgui.begin("Library", .{ .flags = .{ .no_resize = true, .no_move = true, .no_title_bar = true, .no_docking = true, .no_bring_to_front_on_focus = true } })) {
        if (d.config.game_directory) |dir| {
            zgui.text("Directory: {s}", .{dir});
        } else {
            zgui.text("Directory: None", .{});
        }
        zgui.sameLine(.{});
        if (zgui.button("Refresh", .{})) {
            try self.deecy.launch_async(refresh_games, .{self});
        }
        zgui.sameLine(.{});
        if (zgui.button("Change Directory", .{})) {
            const open_path = try nfd.openFolderDialog(null);
            if (open_path) |path| {
                defer nfd.freePath(path);
                if (d.config.game_directory) |old_dir| self.allocator.free(old_dir);
                d.config.game_directory = try self.allocator.dupe(u8, path);
                try self.deecy.launch_async(refresh_games, .{self});
            }
        }

        {
            self.disc_files_mutex.lock();
            defer self.disc_files_mutex.unlock();

            {
                _ = zgui.beginChild("Games", .{});
                defer zgui.endChild();

                var truncated_name: [28:0]u8 = undefined;
                zgui.pushStyleVar2f(.{ .idx = .frame_padding, .v = .{ 0, 0 } });
                zgui.pushStyleVar2f(.{ .idx = .item_spacing, .v = .{ 8.0, 8.0 } });
                defer zgui.popStyleVar(.{ .count = 2 });
                for (self.disc_files.items, 0..) |entry, idx| {
                    var launch = false;
                    {
                        zgui.beginGroup();
                        defer zgui.endGroup();
                        zgui.pushStyleVar2f(.{ .idx = .item_spacing, .v = .{ 0, 0 } });
                        defer zgui.popStyleVar(.{});
                        zgui.pushIntId(@intCast(idx));
                        defer zgui.popId();
                        @memset(&truncated_name, 0);
                        @memcpy(truncated_name[0..@min(entry.name.len, truncated_name.len - 1)], entry.name[0..@min(entry.name.len, truncated_name.len - 1)]);
                        launch = (zgui.button(&truncated_name, .{ .w = 256, .h = 24 })) or launch;

                        zgui.pushStrId("image");
                        defer zgui.popId();
                        if (entry.view) |view| {
                            launch = zgui.imageButton(entry.name, self.deecy.gctx.lookupResource(view).?, .{ .w = 256, .h = 256 }) or launch;
                        } else {
                            launch = zgui.button(entry.name, .{ .w = 256, .h = 256 }) or launch;
                        }
                    }

                    if (zgui.isItemHovered(.{ .for_tooltip = true }) and zgui.beginTooltip()) {
                        zgui.text("{s}", .{entry.name});
                        zgui.endTooltip();
                    }

                    if (launch)
                        try d.load_and_start(entry.path);

                    if (idx % 4 != 3) zgui.sameLine(.{});
                }
            }
        }
    }
}
