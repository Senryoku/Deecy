const std = @import("std");

const zglfw = @import("zglfw");
const zgpu = @import("zgpu");
const zgui = @import("zgui");
const zaudio = @import("zaudio");

const termcolor = @import("termcolor");

const DreamcastModule = @import("./dreamcast.zig");
const Dreamcast = DreamcastModule.Dreamcast;
const AICA = DreamcastModule.AICAModule.AICA;
const GDI = @import("./gdi.zig").GDI;

const Renderer = @import("./renderer.zig").Renderer;

const DeecyUI = @import("./deecy_ui.zig");
const DebugUI = @import("./debug_ui.zig");

const deecy_log = std.log.scoped(.deecy);

fn glfw_key_callback(
    window: *zglfw.Window,
    key: zglfw.Key,
    scancode: i32,
    action: zglfw.Action,
    mods: zglfw.Mods,
) callconv(.C) void {
    _ = scancode;
    _ = mods;

    const maybe_app = window.getUserPointer(Deecy);

    if (maybe_app) |app| {
        if (action == .press) {
            switch (key) {
                .escape => {
                    app.display_ui = !app.display_ui;
                },
                .space => {
                    if (app.running) {
                        app.stop();
                    } else {
                        app.start();
                    }
                },
                .d => {
                    app.config.display_debug_ui = !app.config.display_debug_ui;
                },
                .l => {
                    app.set_throttle_method(switch (app.config.cpu_throttling_method) {
                        .None => .PerFrame,
                        .PerFrame => .None,
                    });
                },
                else => {},
            }
        }
    }
}

fn glfw_drop_callback(
    window: *zglfw.Window,
    count: i32,
    paths: [*][*:0]const u8,
) callconv(.C) void {
    const maybe_app = window.getUserPointer(Deecy);
    if (maybe_app) |app| {
        if (count > 0) {
            app.load_and_start(std.mem.span(paths[0])) catch |err| {
                deecy_log.err("Failed to load disk: {}\n", .{err});
            };
        }
        if (count > 1) {
            deecy_log.warn("Drop only supports 1 file at a time :)", .{});
        }
    }
}

const assets_dir = "assets/";
const DefaultFont = @embedFile(assets_dir ++ "fonts/Hack-Regular.ttf");

pub const CPUThrottleMethod = enum { None, PerFrame };

pub fn reset_semaphore(sem: *std.Thread.Semaphore) void {
    sem.mutex.lock();
    defer sem.mutex.unlock();

    sem.permits = 0;
}

fn safe_path(path: []u8) void {
    for (path) |*c| {
        if (!((c.* >= 'a' and c.* <= 'z') or (c.* >= 'A' and c.* <= 'Z') or (c.* >= '0' and c.* <= '9') or c.* == '.' or c.* == '/')) {
            c.* = '_';
        }
    }
}

const Configuration = struct {
    per_game_vmu: bool = true,
    cpu_throttling_method: CPUThrottleMethod = .None,
    display_debug_ui: bool = false,
};

pub const Deecy = struct {
    pub const TmpDirPath = "./userdata/.tmp_deecy"; // Be careful when editing this, it will deleted on program exit!

    const ExperimentalThreadedDC = true;

    window: *zglfw.Window,
    gctx: *zgpu.GraphicsContext = undefined,
    scale_factor: f32 = 1.0,

    dc: *Dreamcast,
    renderer: Renderer = undefined,
    audio_device: *zaudio.Device = undefined,

    config: Configuration = .{},

    last_frame_timestamp: i64,
    last_n_frametimes: std.fifo.LinearFifo(i64, .Dynamic),

    running: bool = false,
    dc_thread: std.Thread = undefined,
    dc_thread_semaphore: std.Thread.Semaphore = .{},
    dc_last_frame: std.time.Instant = undefined,

    enable_jit: bool = true,
    breakpoints: std.ArrayList(u32),

    controllers: [4]?struct { id: zglfw.Joystick.Id, deadzone: f32 = 0.1 } = .{null} ** 4,

    display_ui: bool = true,
    ui: DeecyUI,
    debug_ui: DebugUI = undefined,

    _allocator: std.mem.Allocator,

    pub fn create(allocator: std.mem.Allocator) !*Deecy {
        std.fs.cwd().makeDir("userdata") catch |err| switch (err) {
            error.PathAlreadyExists => {},
            else => return err,
        };

        try zglfw.init();

        zaudio.init(allocator);

        const self = try allocator.create(Deecy);
        self.* = Deecy{
            .window = try zglfw.Window.create(640 * 2 + 2 * 320, 480 * 2 + 320, "Deecy", null),
            .dc = try Dreamcast.create(allocator),
            .last_frame_timestamp = std.time.microTimestamp(),
            .last_n_frametimes = std.fifo.LinearFifo(i64, .Dynamic).init(allocator),
            .breakpoints = std.ArrayList(u32).init(allocator),
            .ui = DeecyUI.init(allocator),
            ._allocator = allocator,
        };

        self.window.setUserPointer(self);
        _ = self.window.setKeyCallback(glfw_key_callback);
        _ = self.window.setDropCallback(glfw_drop_callback);

        self.gctx = try zgpu.GraphicsContext.create(allocator, .{
            .window = self.window,
            .fn_getTime = @ptrCast(&zglfw.getTime),
            .fn_getFramebufferSize = @ptrCast(&zglfw.Window.getFramebufferSize),
            .fn_getWin32Window = @ptrCast(&zglfw.getWin32Window),
            .fn_getX11Display = @ptrCast(&zglfw.getX11Display),
            .fn_getX11Window = @ptrCast(&zglfw.getX11Window),
            .fn_getCocoaWindow = @ptrCast(&zglfw.getCocoaWindow),
        }, .{
            .present_mode = .mailbox,
            .required_features = &[_]zgpu.wgpu.FeatureName{ .bgra8_unorm_storage, .depth32_float_stencil8 },
            .required_limits = &.{ .limits = .{ .max_texture_array_layers = 512 } },
        });

        brk_limits: {
            var device_limits: zgpu.wgpu.SupportedLimits = .{};
            var adapter_limits: zgpu.wgpu.SupportedLimits = .{};
            if (!self.gctx.device.getLimits(&device_limits)) {
                deecy_log.err("Failed to get device limits.", .{});
                break :brk_limits;
            }
            if (!self.gctx.device.getAdapter().getLimits(&adapter_limits)) {
                deecy_log.err("Failed to get adapter limits.", .{});
                break :brk_limits;
            }
            deecy_log.info("WebGPU Limits (Device/Adapter):", .{});
            inline for (std.meta.fields(zgpu.wgpu.Limits)) |field| {
                deecy_log.info("{s: >48}: {d: >10} / {d: >10}", .{ field.name, @field(device_limits.limits, field.name), @field(adapter_limits.limits, field.name) });
            }
        }

        const scale = self.window.getContentScale();
        self.scale_factor = @max(scale[0], scale[1]);

        self.renderer = try Renderer.init(self._allocator, self.gctx);
        self.dc.on_render_start = .{
            .function = @ptrCast(&Renderer.on_render_start),
            .context = &self.renderer,
        };

        var audio_device_config = zaudio.Device.Config.init(.playback);
        audio_device_config.sample_rate = DreamcastModule.AICAModule.AICA.SampleRate;
        audio_device_config.data_callback = audio_callback;
        audio_device_config.user_data = self;
        audio_device_config.period_size_in_frames = 16;
        audio_device_config.playback.format = .signed32;
        audio_device_config.playback.channels = 2;
        // std.debug.print("Audio device config: {}\n", .{audio_device_config});
        self.audio_device = try zaudio.Device.create(null, audio_device_config);

        try self.audio_device.setMasterVolume(0.2);
        try self.audio_device.start();

        try self.ui_init();

        var curr_pad: usize = 0;
        for (0..zglfw.Joystick.maximum_supported) |idx| {
            const jid: zglfw.Joystick.Id = @intCast(idx);
            if (zglfw.Joystick.get(jid)) |joystick| {
                if (joystick.asGamepad()) |_| {
                    self.controllers[curr_pad] = .{ .id = jid };
                    curr_pad += 1;
                    if (curr_pad >= 4)
                        break;
                }
            }
        }

        return self;
    }

    fn ui_init(self: *Deecy) !void {
        zgui.init(self._allocator);
        zgui.io.setConfigFlags(.{ .dock_enable = true });

        _ = zgui.io.addFontFromMemory(
            DefaultFont,
            std.math.floor(16.0 * self.scale_factor),
        );

        zgui.getStyle().scaleAllSizes(self.scale_factor);

        zgui.backend.init(
            self.window,
            self.gctx.device,
            @intFromEnum(zgpu.GraphicsContext.swapchain_format),
            @intFromEnum(zgpu.wgpu.TextureFormat.undef),
        );

        zgui.plot.init();

        self.debug_ui = try DebugUI.init(self);
    }

    fn ui_deinit(self: *Deecy) void {
        self.debug_ui.deinit();

        zgui.plot.deinit();

        zgui.backend.deinit();
        zgui.deinit();
    }

    pub fn pool_controllers(self: *Deecy) void {
        for (0..4) |controller_idx| {
            if (self.dc.maple.ports[controller_idx].main) |*guest_controller| {
                switch (guest_controller.*) {
                    .Controller => |*c| {
                        // NOTE: Hackish keyboard input for controller 1.
                        var any_keyboard_key_pressed = false;
                        if (controller_idx == 0) {
                            const keybinds: [9]struct { zglfw.Key, DreamcastModule.Maple.ControllerButtons } = .{
                                .{ .enter, .{ .start = 0 } },
                                .{ .up, .{ .up = 0 } },
                                .{ .down, .{ .down = 0 } },
                                .{ .left, .{ .left = 0 } },
                                .{ .right, .{ .right = 0 } },
                                .{ .q, .{ .a = 0 } },
                                .{ .w, .{ .b = 0 } },
                                .{ .a, .{ .x = 0 } },
                                .{ .s, .{ .y = 0 } },
                            };
                            for (keybinds) |keybind| {
                                const key_status = self.window.getKey(keybind[0]);
                                if (key_status == .press) {
                                    any_keyboard_key_pressed = true;
                                    c.press_buttons(keybind[1]);
                                } else if (key_status == .release) {
                                    c.release_buttons(keybind[1]);
                                }
                            }
                        }

                        if (!any_keyboard_key_pressed) {
                            if (self.controllers[controller_idx]) |host_controller| {
                                if (zglfw.Joystick.get(host_controller.id)) |joystick| {
                                    if (joystick.asGamepad()) |gamepad| {
                                        const gamepad_state = gamepad.getState();
                                        const gamepad_binds: [9]struct { zglfw.Gamepad.Button, DreamcastModule.Maple.ControllerButtons } = .{
                                            .{ .start, .{ .start = 0 } },
                                            .{ .dpad_up, .{ .up = 0 } },
                                            .{ .dpad_down, .{ .down = 0 } },
                                            .{ .dpad_left, .{ .left = 0 } },
                                            .{ .dpad_right, .{ .right = 0 } },
                                            .{ .a, .{ .a = 0 } },
                                            .{ .b, .{ .b = 0 } },
                                            .{ .x, .{ .x = 0 } },
                                            .{ .y, .{ .y = 0 } },
                                        };
                                        for (gamepad_binds) |keybind| {
                                            const key_status = gamepad_state.buttons[@intFromEnum(keybind[0])];
                                            if (key_status == .press) {
                                                c.press_buttons(keybind[1]);
                                            } else if (key_status == .release) {
                                                c.release_buttons(keybind[1]);
                                            }
                                        }
                                        c.axis[0] = @as(u8, @intFromFloat((gamepad_state.axes[@intFromEnum(zglfw.Gamepad.Axis.right_trigger)] * 0.5 + 0.5) * 255));
                                        c.axis[1] = @as(u8, @intFromFloat((gamepad_state.axes[@intFromEnum(zglfw.Gamepad.Axis.left_trigger)] * 0.5 + 0.5) * 255));

                                        var x_axis = gamepad_state.axes[@intFromEnum(zglfw.Gamepad.Axis.left_x)];
                                        var y_axis = gamepad_state.axes[@intFromEnum(zglfw.Gamepad.Axis.left_y)];
                                        if (@abs(x_axis) < host_controller.deadzone)
                                            x_axis = 0.0;
                                        if (@abs(y_axis) < host_controller.deadzone)
                                            y_axis = 0.0;
                                        // TODO: Remap with deadzone?
                                        x_axis = x_axis * 0.5 + 0.5;
                                        y_axis = y_axis * 0.5 + 0.5;
                                        c.axis[2] = @as(u8, @intFromFloat(std.math.ceil(x_axis * 255)));
                                        c.axis[3] = @as(u8, @intFromFloat(std.math.ceil(y_axis * 255)));
                                    }
                                } else {
                                    // Not valid anymore? Disconnected?
                                    self.controllers[controller_idx] = null;
                                }
                            }
                        }
                    },
                    else => {},
                }
            }
        }
    }

    pub fn load_and_start(self: *Deecy, path: []const u8) !void {
        self.stop();
        try self.load_disk(path);
        self.dc.set_region(self.dc.gdrom.disk.?.get_region()) catch |err| {
            switch (err) {
                error.FileNotFound => return error.BiosOrFlashMissing,
                else => return err,
            }
        };
        try self.on_game_load();
        try self.dc.reset();
        self.start();
        self.display_ui = false;
    }

    pub fn load_disk(self: *Deecy, path: []const u8) !void {
        if (std.mem.endsWith(u8, path, ".zip")) {
            var zip_file = try std.fs.cwd().openFile(path, .{});
            defer zip_file.close();
            var stream = zip_file.seekableStream();
            var iter = try std.zip.Iterator(std.fs.File.SeekableStream).init(stream);
            var filename_buf: [std.fs.max_path_bytes]u8 = undefined;
            var gdi_filename: []u8 = "";
            while (try iter.next()) |entry| {
                const filename = filename_buf[0..entry.filename_len];
                try zip_file.seekTo(entry.header_zip_offset + @sizeOf(std.zip.CentralDirectoryFileHeader));
                std.debug.assert(try stream.context.reader().readAll(filename) == filename.len);
                if (std.mem.endsWith(u8, filename, ".gdi")) {
                    gdi_filename = filename;
                    break;
                }
            }
            if (gdi_filename.len == 0) {
                std.log.err("Could not find GDI file in zip file '{s}'.", .{path});
                return error.GDIFileNotFound;
            }
            var gdi_path_buf: [std.fs.max_path_bytes]u8 = undefined;
            const tmp_gdi_path = try std.fmt.bufPrint(&gdi_path_buf, TmpDirPath ++ "/{s}", .{gdi_filename});
            std.log.info("Found GDI file: '{s}'.", .{gdi_filename});
            std.log.info("Extracting zip to '{s}'...", .{TmpDirPath});
            var tmp_dir = try std.fs.cwd().makeOpenPath(TmpDirPath, .{});
            defer tmp_dir.close();
            try std.zip.extract(tmp_dir, stream, .{});
            self.dc.gdrom.disk = try GDI.init(tmp_gdi_path, self._allocator);
        } else {
            self.dc.gdrom.disk = try GDI.init(path, self._allocator);
        }
    }

    pub fn on_game_load(self: *@This()) !void {
        if (self.config.per_game_vmu) {
            if (self.dc.gdrom.disk.?.get_product_id()) |product_id| {
                var vmu_path = std.ArrayList(u8).init(self._allocator);
                defer vmu_path.deinit();
                try vmu_path.writer().print("./userdata/{s}/vmu_0.bin", .{product_id});
                safe_path(vmu_path.items);

                if (self.dc.maple.ports[0].subperipherals[0]) |*peripheral| {
                    switch (peripheral.*) {
                        .VMU => |*vmu| vmu.deinit(self._allocator),
                        else => {},
                    }
                }
                self.dc.maple.ports[0].subperipherals[0] = .{ .VMU = try DreamcastModule.Maple.VMU.init(self._allocator, vmu_path.items) };
            }
        }
    }

    pub fn set_throttle_method(self: *Deecy, method: CPUThrottleMethod) void {
        if (method == self.config.cpu_throttling_method) return;

        switch (method) {
            .None => {
                self.dc_thread_semaphore.post(); // Make sure to wake up.
            },
            .PerFrame => {
                reset_semaphore(&self.dc_thread_semaphore);
                self.dc_last_frame = std.time.Instant.now() catch unreachable;
            },
        }
        self.config.cpu_throttling_method = method;
    }

    pub fn start(self: *Deecy) void {
        if (!self.running) {
            if (self.dc.region == .Unknown) {
                self.dc.set_region(.USA) catch {
                    @panic("Failed to set default region");
                };
            }
            self.running = true;
            if (ExperimentalThreadedDC) {
                self.dc_thread = std.Thread.spawn(.{}, dreamcast_thread_fn, .{self}) catch |err| {
                    self.running = false;
                    deecy_log.err(termcolor.red("Failed to start dreamcast thread: {s}"), .{@errorName(err)});
                    return undefined;
                };
            }
        }
    }

    pub fn stop(self: *Deecy) void {
        if (self.running) {
            self.running = false;
            if (ExperimentalThreadedDC) {
                self.dc_thread_semaphore.post();
                self.dc_thread.join();
            }
        }
    }

    pub fn destroy(self: *Deecy) void {
        self.stop();

        self.breakpoints.deinit();

        self.audio_device.destroy();

        self.renderer.deinit();

        self.dc.deinit();
        self._allocator.destroy(self.dc);

        self.ui_deinit();

        zaudio.deinit();

        self.gctx.destroy(self._allocator);

        self.window.destroy();
        zglfw.terminate();

        self._allocator.destroy(self);
    }

    pub fn draw_ui(self: *@This()) !void {
        zgui.backend.newFrame(
            self.gctx.swapchain_descriptor.width,
            self.gctx.swapchain_descriptor.height,
        );

        _ = zgui.DockSpaceOverViewport(zgui.getMainViewport(), .{ .passthru_central_node = true });

        if (self.display_ui) {
            try self.ui.draw(self);
            if (self.config.display_debug_ui)
                try self.debug_ui.draw(self);
        } else {
            zgui.setNextWindowPos(.{ .x = 0, .y = 0 });
            if (zgui.begin("##FPSCounter", .{ .flags = .{ .no_resize = true, .no_move = true, .no_background = true, .no_title_bar = true, .no_mouse_inputs = true, .no_nav_inputs = true, .no_nav_focus = true } })) {
                var sum: i128 = 0;
                for (0..self.last_n_frametimes.count) |i| {
                    sum += self.last_n_frametimes.peekItem(i);
                }
                const avg: f32 = @as(f32, @floatFromInt(sum)) / @as(f32, @floatFromInt(self.last_n_frametimes.count));
                zgui.text("FPS: {d: >4.1} ({d: >3.1}ms)", .{ 1000000.0 / avg, avg / 1000.0 });
            }
            zgui.end();
        }
    }

    pub fn one_frame(self: *Deecy) void {
        if (ExperimentalThreadedDC) {
            const target_frame_time = std.time.ns_per_s / 60; // FIXME: Adjust that based on the DC settings...

            // Internal representation of std.time.Instant is plateform dependent. To do arithmetic with it, we need to learn about it.
            // FIXME: This is not ideal... It is unknown at compile tile, on Windows at least. But it should be constant for the duration of the program, I hope.
            const static = struct {
                var frame_time: u64 = 0; // In nanoseconds
                var timestamp_diff: u64 = undefined; // In platform-dependent units
            };
            if (static.frame_time != target_frame_time) {
                static.frame_time = target_frame_time;
                const timestamp_scale = (std.time.Instant{ .timestamp = 1_000_000_000 }).since(std.time.Instant{ .timestamp = 0 });
                static.timestamp_diff = (target_frame_time * 1_000_000_000) / timestamp_scale;
            }

            if (self.running and self.config.cpu_throttling_method == .PerFrame) {
                const now = std.time.Instant.now() catch unreachable;
                if (now.since(self.dc_last_frame) >= target_frame_time) {
                    self.dc_thread_semaphore.post(); // FIXME: This will eventually overflow if the DC thread can't keep up (e.g. using the interpreter).
                    // Adding to the previous timestamp rather that using 'now' will compensate the latency between calls to one_frame().
                    self.dc_last_frame.timestamp += static.timestamp_diff;
                }
            }
        } else {
            self.run_dreamcast_until_next_frame();
        }
    }

    fn run_dreamcast_until_next_frame(self: *Deecy) void {
        var cycles: u64 = 0;
        if (!self.enable_jit) {
            while (self.running and !self.dc.gpu.vblank_signal()) {
                const max_instructions: u8 = if (self.breakpoints.items.len == 0) 16 else 1;

                cycles += self.dc.tick(max_instructions) catch unreachable;

                // Doesn't make sense to try to have breakpoints if the interpreter can execute more than one instruction at a time.
                if (max_instructions == 1) {
                    const breakpoint = for (self.breakpoints.items, 0..) |addr, index| {
                        if (addr & 0x1FFFFFFF == self.dc.cpu.pc & 0x1FFFFFFF) break index;
                    } else null;
                    if (breakpoint != null) {
                        self.running = false;
                    }
                }
            }
        } else {
            while (!self.dc.gpu.vblank_signal()) {
                cycles += self.dc.tick_jit() catch unreachable;
            }
        }
        self.dc.maple.flush_vmus(); // FIXME: Won't flush if paused!
    }

    fn dreamcast_thread_fn(self: *Deecy) void {
        deecy_log.info(termcolor.green("Dreamcast thread started."), .{});

        while (self.running) {
            if (self.config.cpu_throttling_method == .PerFrame) {
                self.dc_thread_semaphore.wait();
            }
            self.run_dreamcast_until_next_frame();
        }

        deecy_log.info(termcolor.red("Dreamcast thread stopped."), .{});
    }

    fn audio_callback(
        device: *zaudio.Device,
        output: ?*anyopaque,
        _: ?*const anyopaque, // Input
        frame_count: u32,
    ) callconv(.C) void {
        const self: *@This() = @ptrCast(@alignCast(device.getUserData()));
        const aica = &self.dc.aica;

        if (!self.running) return;

        aica.sample_mutex.lock();
        defer aica.sample_mutex.unlock();

        if (AICA.ExperimentalExternalSampleGeneration) {
            aica.generate_samples(self.dc, frame_count);
            aica.update_timers(self.dc, frame_count);
        }

        var out: [*]i32 = @ptrCast(@alignCast(output));

        var available: i64 = @as(i64, @intCast(aica.sample_write_offset)) - @as(i64, @intCast(aica.sample_read_offset));
        if (available < 0) available += aica.sample_buffer.len;
        if (available <= 0) return;

        // std.debug.print("audio_callback: frame_count={d}, available={d}\n", .{ frame_count, available });

        for (0..@min(@as(usize, @intCast(available)), 2 * frame_count)) |i| {
            out[i] = 30000 *| aica.sample_buffer[aica.sample_read_offset];
            aica.sample_read_offset = (aica.sample_read_offset + 1) % aica.sample_buffer.len;
        }
    }
};
