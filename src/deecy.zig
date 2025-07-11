const std = @import("std");
const builtin = @import("builtin");
const comptime_config = @import("config");

const zglfw = @import("zglfw");
const zgpu = @import("zgpu");
const zgui = @import("zgui");
const zaudio = @import("zaudio");

extern fn glfwSetWindowIcon(window: *zglfw.Window, count: i32, images: [*]const zglfw.Image) void;
const icon_48_data = @embedFile("assets/icon-48.rgba");
const icon_32_data = @embedFile("assets/icon-32.rgba");
const icon_16_data = @embedFile("assets/icon-small-16.rgba");
const icons = [_]zglfw.Image{
    .{ .width = 48, .height = 48, .pixels = @constCast(icon_48_data)[0..] },
    .{ .width = 32, .height = 32, .pixels = @constCast(icon_32_data)[0..] },
    .{ .width = 16, .height = 16, .pixels = @constCast(icon_16_data)[0..] },
};

const termcolor = @import("termcolor");

const DreamcastModule = @import("dreamcast");
const Dreamcast = DreamcastModule.Dreamcast;
const AICA = DreamcastModule.AICAModule.AICA;
const Disc = DreamcastModule.GDROM.Disc;
const HostPaths = DreamcastModule.HostPaths;

pub const Renderer = @import("./renderer.zig").Renderer;

pub const UI = @import("./deecy_ui.zig");
const DebugUI = @import("./debug_ui.zig");

const lz4 = @import("lz4");

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

    const maybe_app = window.getUserPointer(@This());

    if (maybe_app) |app| {
        if (zgui.io.getWantCaptureKeyboard()) return;

        if (action == .press) {
            switch (key) {
                .escape => {
                    app.display_ui = !app.display_ui;
                },
                .space => {
                    if (app.running) {
                        app.pause();
                    } else {
                        app.start();
                    }
                },
                .d => app.config.display_debug_ui = !app.config.display_debug_ui,
                .f => app.toggle_fullscreen(),
                .l => app.set_realtime(!app.realtime),
                .n => {
                    if (app.running) {
                        app.pause();
                    } else {
                        while (!app.renderer.render_start) app.run_for(128);
                    }
                },
                .F1, .F2, .F3, .F4 => {
                    const idx: usize = switch (key) {
                        .F1 => 0,
                        .F2 => 1,
                        .F3 => 2,
                        .F4 => 3,
                        else => unreachable,
                    };
                    app.save_state(idx) catch |err| {
                        deecy_log.err(termcolor.red("Failed to save state #{d}: {}"), .{ idx, err });
                    };
                },
                .F5, .F6, .F7, .F8 => {
                    const idx: usize = switch (key) {
                        .F5 => 0,
                        .F6 => 1,
                        .F7 => 2,
                        .F8 => 3,
                        else => unreachable,
                    };
                    app.load_state(idx) catch |err| {
                        deecy_log.err(termcolor.red("Failed to load state #{d}: {}"), .{ idx, err });
                    };
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
    const maybe_app = window.getUserPointer(@This());
    if (maybe_app) |app| {
        if (count > 0) {
            app.load_and_start(std.mem.span(paths[0])) catch |err| {
                deecy_log.err("Failed to load disc: {}\n", .{err});
            };
        }
        if (count > 1) {
            deecy_log.warn("Drop only supports 1 file at a time :)", .{});
        }
    }
}

const assets_dir = "assets/";
const DefaultFont = @embedFile(assets_dir ++ "fonts/Hack-Regular.ttf");

// Replaces invalid characters with underscores
fn safe_path(path: []u8) void {
    for (path) |*c| {
        if (!((c.* >= 'a' and c.* <= 'z') or (c.* >= 'A' and c.* <= 'Z') or (c.* >= '0' and c.* <= '9') or c.* == '.' or c.* == '/' or c.* == '[' or c.* == ']')) {
            c.* = '_';
        }
    }
}

fn file_exists(path: []const u8) !bool {
    const file = std.fs.cwd().openFile(path, .{}) catch |err| switch (err) {
        error.FileNotFound => return false,
        else => return err,
    };
    file.close();
    return true;
}

pub const KeyboardBindings = struct {
    a: ?zglfw.Key = null,
    b: ?zglfw.Key = null,
    x: ?zglfw.Key = null,
    y: ?zglfw.Key = null,
    up: ?zglfw.Key = null,
    down: ?zglfw.Key = null,
    left: ?zglfw.Key = null,
    right: ?zglfw.Key = null,
    start: ?zglfw.Key = null,
    left_trigger: ?zglfw.Key = null,
    right_trigger: ?zglfw.Key = null,
    left_stick_up: ?zglfw.Key = null,
    left_stick_down: ?zglfw.Key = null,
    left_stick_left: ?zglfw.Key = null,
    left_stick_right: ?zglfw.Key = null,
    right_stick_up: ?zglfw.Key = null,
    right_stick_down: ?zglfw.Key = null,
    right_stick_left: ?zglfw.Key = null,
    right_stick_right: ?zglfw.Key = null,

    const Default: KeyboardBindings = .{
        .a = .q,
        .b = .w,
        .x = .a,
        .y = .s,
        .up = .up,
        .down = .down,
        .left = .left,
        .right = .right,
        .start = .enter,
        .left_trigger = .z,
        .right_trigger = .x,
        .left_stick_up = .kp_8,
        .left_stick_down = .kp_5,
        .left_stick_left = .kp_4,
        .left_stick_right = .kp_6,
    };
};

const ControllerSettings = struct {
    enabled: bool,
    subcapabilities: DreamcastModule.Maple.InputCapabilities = DreamcastModule.Maple.DualStickControllerCapabilities,
};

const ConfigurationJSON = struct {
    per_game_vmu: ?bool = true,
    display_vmus: ?bool = true,
    game_directory: ?[]const u8 = null,
    display_debug_ui: ?bool = false,

    window_size: ?struct { width: u32 = 2 * @ceil((16.0 / 9.0 * @as(f32, @floatFromInt(Renderer.NativeResolution.height)))), height: u32 = 2 * Renderer.NativeResolution.height } = .{},
    present_mode: ?zgpu.wgpu.PresentMode = .fifo,
    fullscreen: ?bool = false,

    internal_resolution_factor: ?u32 = 1,
    display_mode: ?Renderer.DisplayMode = .Center,

    keyboard_bindings: ?[4]KeyboardBindings = .{ .Default, .{}, .{}, .{} },
    controllers: ?[4]ControllerSettings = null,

    audio_volume: ?f32 = 0.3,
    dsp_emulation: ?DreamcastModule.AICAModule.DSPEmulation = .JIT,
};

const Configuration = struct {
    per_game_vmu: bool = true,
    display_vmus: bool = true,
    game_directory: ?[]const u8 = null,
    display_debug_ui: bool = false,

    window_size: struct { width: u32 = 2 * @ceil((16.0 / 9.0 * @as(f32, @floatFromInt(Renderer.NativeResolution.height)))), height: u32 = 2 * Renderer.NativeResolution.height } = .{},
    present_mode: zgpu.wgpu.PresentMode = .fifo,
    fullscreen: bool = false,

    internal_resolution_factor: u32 = 2,
    display_mode: Renderer.DisplayMode = .Center,

    keyboard_bindings: [4]KeyboardBindings = .{ .Default, .{}, .{}, .{} },
    controllers: [4]ControllerSettings = .{ .{ .enabled = true }, .{ .enabled = true }, .{ .enabled = false }, .{ .enabled = false } },

    audio_volume: f32 = 0.3,
    dsp_emulation: DreamcastModule.AICAModule.DSPEmulation = .JIT,

    pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
        if (self.game_directory) |game_directory|
            allocator.free(game_directory);
    }
};

pub const TmpDirPath = "/.tmp_deecy"; // Be careful when editing this, it will be deleted on program exit!
pub const ConfigFile = "/config.json";

pub const MaxSaveStates = 4;

window: *zglfw.Window,
gctx: *zgpu.GraphicsContext = undefined,
gctx_queue_mutex: std.Thread.Mutex = .{}, // GPU Memory access isn't thread safe. Use this to copy to textures from another thread for example.
scale_factor: f32 = 1.0,

dc: *Dreamcast = undefined,
renderer: *Renderer = undefined,
audio_device: *zaudio.Device = undefined,

config: Configuration = .{},
toggle_fullscreen_request: bool = false,
previous_window_position: struct { x: i32 = 0, y: i32 = 0, w: i32 = 0, h: i32 = 0 } = .{},

last_frame_timestamp: i64,
last_n_frametimes: std.fifo.LinearFifo(i64, .Dynamic),

running: bool = false,
_cycles_to_run: i64 = 0,
_stop_request: bool = false,
realtime: bool = true, // By default, emulation is driven by the audio thread.
_dc_thread: ?std.Thread = null, // Used for unlimited frame rate, i.e. when realtime == false

enable_jit: bool = true,
breakpoints: std.ArrayList(u32),

controllers: [4]?struct { id: zglfw.Joystick, deadzone: f32 = 0.1 } = @splat(null),

display_ui: bool = true,
ui: *UI = undefined,
debug_ui: DebugUI = undefined,

save_state_slots: [MaxSaveStates]bool = .{ false, false, false, false },

_allocator: std.mem.Allocator,

_thread: ?std.Thread = null, // Thread for one-time, fire-and-forget, async jobs

pub fn create(allocator: std.mem.Allocator) !*@This() {
    const start_time = std.time.milliTimestamp();
    defer deecy_log.info("Deecy initialized in {d} ms", .{std.time.milliTimestamp() - start_time});

    std.fs.cwd().makePath(HostPaths.get_userdata_path()) catch |err| switch (err) {
        error.PathAlreadyExists => {},
        else => return err,
    };

    // Load user config
    // TODO: Replace by ZON when available.
    const config = config: {
        const config_path = try std.fs.path.join(allocator, &[_][]const u8{ HostPaths.get_userdata_path(), ConfigFile });
        defer allocator.free(config_path);
        if (std.fs.cwd().openFile(config_path, .{})) |file| {
            defer file.close();
            const conf_str = try file.readToEndAlloc(allocator, 1024 * 1024);
            defer allocator.free(conf_str);
            const json = try std.json.parseFromSlice(ConfigurationJSON, allocator, conf_str, .{});
            defer json.deinit();

            var conf: Configuration = .{};
            conf.display_debug_ui = json.value.display_debug_ui orelse false;
            conf.per_game_vmu = json.value.per_game_vmu orelse true;
            conf.display_vmus = json.value.display_vmus orelse true;
            if (json.value.game_directory) |game_directory|
                conf.game_directory = try allocator.dupe(u8, game_directory);
            if (json.value.window_size) |window_size| {
                conf.window_size.width = window_size.width;
                conf.window_size.height = window_size.height;
            }
            conf.present_mode = json.value.present_mode orelse .fifo;
            conf.fullscreen = json.value.fullscreen orelse false;

            conf.internal_resolution_factor = json.value.internal_resolution_factor orelse 2;
            conf.display_mode = json.value.display_mode orelse .Center;

            conf.keyboard_bindings = json.value.keyboard_bindings orelse .{ .Default, .{}, .{}, .{} };
            conf.controllers = json.value.controllers orelse .{ .{ .enabled = true }, .{ .enabled = true }, .{ .enabled = false }, .{ .enabled = false } };

            conf.audio_volume = json.value.audio_volume orelse 0.3;
            conf.dsp_emulation = json.value.dsp_emulation orelse .JIT;

            break :config conf;
        } else |_| {
            break :config Configuration{};
        }
    };

    try zglfw.init();

    const self = try allocator.create(@This());
    self.* = .{
        .window = undefined,
        .config = config,
        .last_frame_timestamp = std.time.microTimestamp(),
        .last_n_frametimes = .init(allocator),
        .breakpoints = .init(allocator),
        ._allocator = allocator,
    };

    {
        // NOTE: For some reason Joystick initialization is the longest operation in here on Windows. Start ASAP and in parallel of context creation and window creation (second longest operation).
        //       Note that glfwIsJoystickPresent() is not supposed to be thread-safe... But I haven't had any problems so far.
        var joystick_thread = try std.Thread.spawn(.{}, auto_populate_joysticks, .{self});
        defer joystick_thread.join();

        {
            const window_init_time = std.time.milliTimestamp();
            defer deecy_log.info("Window initialized in {d} ms", .{std.time.milliTimestamp() - window_init_time});

            // IDK, prevents device lost crash on Linux. See https://github.com/zig-gamedev/zig-gamedev/commit/9bd4cf860c8e295f4f0db9ec4357905e090b5b98
            zglfw.windowHint(.client_api, .no_api);

            const useWayland = if (builtin.os.tag == .windows) false else wayland_env: {
                var env_var = try std.process.getEnvMap(allocator);
                defer env_var.deinit();
                break :wayland_env std.mem.eql(u8, env_var.get("XDG_SESSION_TYPE") orelse "", "wayland");
            };
            // Hide window until the first frame is drawn to avoid a white flash. Don't forget to .show() it eventually!
            // NOTE: Crashes on Kubuntu 25.04. From what I could gather, Wayland forbid using a window before making it visible.
            //       Not sure if this is the correct fix, or the correct way to detect Wayland either.
            if (!useWayland)
                zglfw.windowHint(.visible, false);

            self.window = try zglfw.Window.create(@intCast(config.window_size.width), @intCast(config.window_size.height), "Deecy", null);
            glfwSetWindowIcon(self.window, icons.len, &icons);
            if (builtin.os.tag == .windows)
                @import("dwmapi.zig").allow_dark_mode(self.window, true);
            if (self.config.fullscreen) {
                self.config.fullscreen = false;
                self.toggle_fullscreen();
            }

            self.window.setUserPointer(self);
            _ = self.window.setKeyCallback(glfw_key_callback);
            _ = zglfw.setDropCallback(self.window, glfw_drop_callback);

            const scale = self.window.getContentScale();
            self.scale_factor = @max(scale[0], scale[1]);
        }

        {
            const gctx_start_time = std.time.milliTimestamp();
            defer deecy_log.info("Graphics context initialized in {d} ms", .{std.time.milliTimestamp() - gctx_start_time});
            self.gctx = try zgpu.GraphicsContext.create(allocator, .{
                .window = self.window,
                .fn_getTime = @ptrCast(&zglfw.getTime),
                .fn_getFramebufferSize = @ptrCast(&zglfw.Window.getFramebufferSize),
                .fn_getWin32Window = @ptrCast(&zglfw.getWin32Window),
                .fn_getX11Display = @ptrCast(&zglfw.getX11Display),
                .fn_getX11Window = @ptrCast(&zglfw.getX11Window),
                .fn_getWaylandDisplay = @ptrCast(&zglfw.getWaylandDisplay),
                .fn_getWaylandSurface = @ptrCast(&zglfw.getWaylandWindow),
                .fn_getCocoaWindow = @ptrCast(&zglfw.getCocoaWindow),
            }, .{
                .present_mode = config.present_mode,
                .required_features = &[_]zgpu.wgpu.FeatureName{ .bgra8_unorm_storage, .depth32_float_stencil8 },
                .required_limits = &.{ .limits = .{ .max_texture_array_layers = 512 } },
            });
        }

        // Now that we have our graphics context, "draw" a black frame and show the window.
        // We could wait until the first frame is actually drawn, but this give an earlier feedback...
        // Not sure what's best here.
        {
            const back_buffer_view = self.gctx.swapchain.getCurrentTextureView();
            defer back_buffer_view.release();
            _ = self.gctx.present();
            self.window.show();
        }

        if (comptime std.log.logEnabled(.debug, .deecy)) {
            brk_limits: {
                var device_limits: zgpu.wgpu.SupportedLimits = .{};
                var adapter_limits: zgpu.wgpu.SupportedLimits = .{};
                if (!self.gctx.device.getLimits(&device_limits)) {
                    deecy_log.debug("Failed to get device limits.", .{});
                    break :brk_limits;
                }
                if (!self.gctx.device.getAdapter().getLimits(&adapter_limits)) {
                    deecy_log.debug("Failed to get adapter limits.", .{});
                    break :brk_limits;
                }
                deecy_log.debug("WebGPU Limits (Device/Adapter):", .{});
                inline for (std.meta.fields(zgpu.wgpu.Limits)) |field| {
                    deecy_log.debug("{s: >48}: {d: >10} / {d: >10}", .{ field.name, @field(device_limits.limits, field.name), @field(adapter_limits.limits, field.name) });
                }
            }
        }

        try self.audio_init();
    }

    try self.ui_init(); // Init UI early to have a chance to display errors.

    // Avoid having other threads running while initialisation the Dreamcast to increase the chance of virtual_address_space initialization to succeed on Windows.
    // FIXME: See sh4_virtual_address_space_windows.zig
    {
        const dc_init_time = std.time.milliTimestamp();
        defer deecy_log.info("Dreamcast initialized in {d} ms", .{std.time.milliTimestamp() - dc_init_time});
        self.dc = Dreamcast.create(allocator) catch |err| {
            switch (err) {
                error.BiosNotFound => {
                    self.display_unrecoverable_error("Missing BIOS. Please copy your bios file as 'dc_boot.bin' to '{s}'.", .{HostPaths.get_data_path()});
                },
                else => {
                    self.display_unrecoverable_error("Error initializing Dreamcast: {s}", .{@errorName(err)});
                },
            }
            return err;
        };
        for (self.config.controllers, 0..) |c, idx| {
            if (c.enabled) self.dc.maple.ports[idx].main = .{ .Controller = .{ .subcapabilities = .{ @bitCast(c.subcapabilities), 0, 0 } } };
        }
    }

    self.renderer = try .create(self._allocator, self.gctx, config.internal_resolution_factor, config.display_mode);
    self.dc.on_render_start = .{
        .function = @ptrCast(&Renderer.on_render_start),
        .context = self.renderer,
    };

    self.ui = try .create(self._allocator, self);
    self.debug_ui = try .init(self);

    try self.check_save_state_slots();

    return self;
}

pub fn destroy(self: *@This()) void {
    self.pause();
    self.wait_async_jobs();

    self.save_config() catch |err| deecy_log.err("Error writing config: {s}", .{@errorName(err)});
    self.config.deinit(self._allocator);

    self.breakpoints.deinit();

    self.audio_device.destroy();

    self.renderer.destroy();

    self.dc.deinit();
    self._allocator.destroy(self.dc);

    self.debug_ui.deinit();
    self.ui_deinit();
    self.ui.destroy();

    zaudio.deinit();

    self.gctx.destroy(self._allocator);

    self.window.destroy();
    zglfw.terminate();

    self._allocator.destroy(self);
}

fn auto_populate_joysticks(self: *@This()) !void {
    const start_time = std.time.milliTimestamp();
    defer deecy_log.info("Joysticks initialized in {d}ms", .{std.time.milliTimestamp() - start_time});
    var curr_pad: usize = 0;
    for (0..zglfw.Joystick.maximum_supported) |idx| {
        const joystick: zglfw.Joystick = @enumFromInt(idx);
        if (joystick.isPresent()) {
            if (joystick.asGamepad()) |_| {
                self.controllers[curr_pad] = .{ .id = joystick };
                curr_pad += 1;
                if (curr_pad >= 4)
                    break;
            }
        }
    }
}

fn audio_init(self: *@This()) !void {
    const zaudio_init_time = std.time.milliTimestamp();
    defer deecy_log.info("Zaudio initialized in {d} ms", .{std.time.milliTimestamp() - zaudio_init_time});
    zaudio.init(self._allocator);

    var audio_device_config = zaudio.Device.Config.init(.playback);
    audio_device_config.sample_rate = DreamcastModule.AICAModule.AICA.SampleRate;
    audio_device_config.data_callback = audio_callback;
    audio_device_config.user_data = self;
    audio_device_config.period_size_in_frames = 16;
    audio_device_config.playback.format = .signed32;
    audio_device_config.playback.channels = 2;
    self.audio_device = try zaudio.Device.create(null, audio_device_config);

    try self.audio_device.setMasterVolume(self.config.audio_volume);
}

fn ui_init(self: *@This()) !void {
    zgui.init(self._allocator);
    zgui.io.setConfigFlags(.{ .dock_enable = true });

    _ = zgui.io.addFontFromMemory(
        DefaultFont,
        std.math.floor(16.0 * self.scale_factor),
    );

    var style = zgui.getStyle();

    style.scaleAllSizes(self.scale_factor);

    // Based on Deep Dark style by janekb04 from ImThemes
    style.alpha = 1.0;
    style.disabled_alpha = 0.6000000238418579;
    style.window_padding = .{ 8.0, 8.0 };
    style.window_rounding = 7.0;
    style.window_border_size = 1.0;
    style.window_min_size = .{ 32.0, 32.0 };
    style.window_title_align = .{ 0.0, 0.5 };
    style.window_menu_button_position = .left;
    style.child_rounding = 4.0;
    style.child_border_size = 1.0;
    style.popup_rounding = 4.0;
    style.popup_border_size = 1.0;
    style.frame_padding = .{ 6.0, 4.0 };
    style.frame_rounding = 3.0;
    style.frame_border_size = 1.0;
    style.item_spacing = .{ 6.0, 6.0 };
    style.item_inner_spacing = .{ 6.0, 6.0 };
    style.cell_padding = .{ 6.0, 6.0 };
    style.indent_spacing = 25.0;
    style.columns_min_spacing = 6.0;
    style.scrollbar_size = 15.0;
    style.scrollbar_rounding = 9.0;
    style.grab_min_size = 10.0;
    style.grab_rounding = 3.0;
    style.tab_rounding = 4.0;
    style.tab_border_size = 1.0;
    style.tab_min_width_for_close_button = 0.0;
    style.color_button_position = .right;
    style.button_text_align = .{ 0.5, 0.5 };
    style.selectable_text_align = .{ 0.0, 0.0 };

    const EULogoColor: [4]f32 = .{ 0.231, 0.463, 0.761, 1.0 };
    //const JPLogoColor: [4]f32 = .{ 0.929, 0.518, 0.192, 1.0 };
    //const USLogoColor: [4]f32 = .{ 0.816, 0.2, 0.071, 1.0 };

    style.setColor(.text, .{ 1.0, 1.0, 1.0, 1.0 });
    style.setColor(.text_disabled, .{ 0.4980392158031464, 0.4980392158031464, 0.4980392158031464, 1.0 });
    style.setColor(.window_bg, .{ 0.09803921729326248, 0.09803921729326248, 0.09803921729326248, 1.0 });
    style.setColor(.child_bg, .{ 0.0, 0.0, 0.0, 0.0 });
    style.setColor(.popup_bg, .{ 0.1882352977991104, 0.1882352977991104, 0.1882352977991104, 0.9200000166893005 });
    style.setColor(.border, .{ 0.1882352977991104, 0.1882352977991104, 0.1882352977991104, 0.2899999916553497 });
    style.setColor(.border_shadow, .{ 0.0, 0.0, 0.0, 0.239999994635582 });
    style.setColor(.frame_bg, .{ 0.0470588244497776, 0.0470588244497776, 0.0470588244497776, 0.5400000214576721 });
    style.setColor(.frame_bg_hovered, .{ 0.1882352977991104, 0.1882352977991104, 0.1882352977991104, 0.5400000214576721 });
    style.setColor(.frame_bg_active, .{ 0.2000000029802322, 0.2196078449487686, 0.2274509817361832, 1.0 });
    style.setColor(.title_bg, .{ 0.0, 0.0, 0.0, 1.0 });
    style.setColor(.title_bg_active, .{ 0.05882352963089943, 0.05882352963089943, 0.05882352963089943, 1.0 });
    style.setColor(.title_bg_collapsed, .{ 0.0, 0.0, 0.0, 1.0 });
    style.setColor(.menu_bar_bg, .{ 0.1372549086809158, 0.1372549086809158, 0.1372549086809158, 1.0 });
    style.setColor(.scrollbar_bg, .{ 0.0470588244497776, 0.0470588244497776, 0.0470588244497776, 0.5400000214576721 });
    style.setColor(.scrollbar_grab, .{ 0.3372549116611481, 0.3372549116611481, 0.3372549116611481, 0.5400000214576721 });
    style.setColor(.scrollbar_grab_hovered, .{ 0.4000000059604645, 0.4000000059604645, 0.4000000059604645, 0.5400000214576721 });
    style.setColor(.scrollbar_grab_active, .{ 0.5568627715110779, 0.5568627715110779, 0.5568627715110779, 0.5400000214576721 });
    style.setColor(.check_mark, EULogoColor);
    style.setColor(.slider_grab, .{ 0.3372549116611481, 0.3372549116611481, 0.3372549116611481, 0.5400000214576721 });
    style.setColor(.slider_grab_active, .{ 0.5568627715110779, 0.5568627715110779, 0.5568627715110779, 0.5400000214576721 });
    style.setColor(.button, .{ EULogoColor[0], EULogoColor[1], EULogoColor[2], 0.2 });
    style.setColor(.button_hovered, .{ EULogoColor[0], EULogoColor[1], EULogoColor[2], 0.35 });
    style.setColor(.button_active, .{ EULogoColor[0], EULogoColor[1], EULogoColor[2], 0.7 });
    style.setColor(.header, .{ 0.0, 0.0, 0.0, 0.5199999809265137 });
    style.setColor(.header_hovered, .{ 0.0, 0.0, 0.0, 0.3600000143051147 });
    style.setColor(.header_active, .{ 0.2000000029802322, 0.2196078449487686, 0.2274509817361832, 0.3300000131130219 });
    style.setColor(.separator, .{ 0.2784313857555389, 0.2784313857555389, 0.2784313857555389, 0.2899999916553497 });
    style.setColor(.separator_hovered, .{ 0.4392156898975372, 0.4392156898975372, 0.4392156898975372, 0.2899999916553497 });
    style.setColor(.separator_active, .{ 0.4000000059604645, 0.4392156898975372, 0.4666666686534882, 1.0 });
    style.setColor(.resize_grip, .{ 0.2784313857555389, 0.2784313857555389, 0.2784313857555389, 0.2899999916553497 });
    style.setColor(.resize_grip_hovered, .{ 0.4392156898975372, 0.4392156898975372, 0.4392156898975372, 0.2899999916553497 });
    style.setColor(.resize_grip_active, .{ 0.4000000059604645, 0.4392156898975372, 0.4666666686534882, 1.0 });
    style.setColor(.tab, .{ 0.0, 0.0, 0.0, 0.5199999809265137 });
    style.setColor(.tab_hovered, .{ 0.1372549086809158, 0.1372549086809158, 0.1372549086809158, 1.0 });
    style.setColor(.tab_selected, .{ 0.2000000029802322, 0.2000000029802322, 0.2000000029802322, 0.3600000143051147 });
    style.setColor(.tab_dimmed, .{ 0.0, 0.0, 0.0, 0.5199999809265137 });
    style.setColor(.tab_dimmed_selected, .{ 0.1372549086809158, 0.1372549086809158, 0.1372549086809158, 1.0 });
    style.setColor(.plot_lines, EULogoColor);
    style.setColor(.plot_lines_hovered, EULogoColor);
    style.setColor(.plot_histogram, EULogoColor);
    style.setColor(.plot_histogram_hovered, EULogoColor);
    style.setColor(.table_header_bg, .{ 0.0, 0.0, 0.0, 0.5199999809265137 });
    style.setColor(.table_border_strong, .{ 0.0, 0.0, 0.0, 0.5199999809265137 });
    style.setColor(.table_border_light, .{ 0.2784313857555389, 0.2784313857555389, 0.2784313857555389, 0.2899999916553497 });
    style.setColor(.table_row_bg, .{ 0.0, 0.0, 0.0, 0.0 });
    style.setColor(.table_row_bg_alt, .{ 1.0, 1.0, 1.0, 0.05999999865889549 });
    style.setColor(.text_selected_bg, .{ 0.2000000029802322, 0.2196078449487686, 0.2274509817361832, 1.0 });
    style.setColor(.drag_drop_target, .{ 0.3294117748737335, 0.6666666865348816, 0.8588235378265381, 1.0 });
    style.setColor(.nav_highlight, EULogoColor);
    style.setColor(.nav_windowing_highlight, .{ EULogoColor[0], EULogoColor[1], EULogoColor[2], 0.7 });
    style.setColor(.nav_windowing_dim_bg, .{ EULogoColor[0], EULogoColor[1], EULogoColor[2], 0.2 });
    style.setColor(.modal_window_dim_bg, .{ 0, 0, 0, 0.6 });

    zgui.backend.init(
        self.window,
        self.gctx.device,
        @intFromEnum(zgpu.GraphicsContext.swapchain_format),
        @intFromEnum(zgpu.wgpu.TextureFormat.undef),
    );

    zgui.plot.init();
}

fn ui_deinit(_: *@This()) void {
    zgui.plot.deinit();
    zgui.backend.deinit();
    zgui.deinit();
}

pub fn reset(self: *@This()) !void {
    const was_running = self.running;
    if (was_running) self.pause();
    defer {
        if (was_running) self.start();
    }

    try self.dc.reset();
    self.ui.binary_loaded = false;
    self.renderer.reset();
    self._cycles_to_run = 0;
    self.last_frame_timestamp = std.time.microTimestamp();
    self.last_n_frametimes.discard(self.last_n_frametimes.count);
    try self.check_save_state_slots();
}

pub fn stop(self: *@This()) !void {
    self.pause();
    try self.reset();
    if (self.dc.gdrom.disc) |*disc| disc.deinit();
    self.dc.gdrom.disc = null;
}

pub fn update(self: *@This()) void {
    self.poll_controllers();
    self.dc.maple.flush_vmus();
    if (self._stop_request) {
        self.pause();
        self._stop_request = false;
    }
}

pub fn poll_controllers(self: *@This()) void {
    for (0..4) |controller_idx| {
        if (self.dc.maple.ports[controller_idx].main) |*guest_controller| {
            switch (guest_controller.*) {
                .Controller => |*c| {
                    c.axis[0] = 0;
                    c.axis[1] = 0;
                    c.axis[2] = 128;
                    c.axis[3] = 128;
                    c.axis[4] = 128;
                    c.axis[5] = 128;

                    var any_keyboard_key_pressed = false;
                    const keyboard_bindings = self.config.keyboard_bindings[controller_idx];
                    inline for ([_][]const u8{ "start", "up", "down", "left", "right", "a", "b", "x", "y" }) |button_name| {
                        if (@field(keyboard_bindings, button_name)) |key| {
                            const key_status = self.window.getKey(key);
                            var button: DreamcastModule.Maple.ControllerButtons = .{};
                            @field(button, button_name) = 0;
                            if (key_status == .press) {
                                any_keyboard_key_pressed = true;
                                c.press_buttons(button);
                            } else if (key_status == .release) {
                                c.release_buttons(button);
                            }
                        }
                    }
                    if (keyboard_bindings.right_trigger) |key|
                        c.axis[0] = if (self.window.getKey(key) == .press) 255 else 0;
                    if (keyboard_bindings.left_trigger) |key|
                        c.axis[1] = if (self.window.getKey(key) == .press) 255 else 0;
                    if (keyboard_bindings.left_stick_left) |key| {
                        if (self.window.getKey(key) == .press) c.axis[2] = 0;
                    }
                    if (keyboard_bindings.left_stick_right) |key| {
                        if (self.window.getKey(key) == .press) c.axis[2] = 255;
                    }
                    if (keyboard_bindings.left_stick_up) |key| {
                        if (self.window.getKey(key) == .press) c.axis[3] = 0;
                    }
                    if (keyboard_bindings.left_stick_down) |key| {
                        if (self.window.getKey(key) == .press) c.axis[3] = 255;
                    }
                    if (keyboard_bindings.right_stick_left) |key| {
                        if (self.window.getKey(key) == .press) c.axis[4] = 0;
                    }
                    if (keyboard_bindings.right_stick_right) |key| {
                        if (self.window.getKey(key) == .press) c.axis[4] = 255;
                    }
                    if (keyboard_bindings.right_stick_up) |key| {
                        if (self.window.getKey(key) == .press) c.axis[5] = 0;
                    }
                    if (keyboard_bindings.right_stick_down) |key| {
                        if (self.window.getKey(key) == .press) c.axis[5] = 255;
                    }

                    if (c.axis[0] != 0 or c.axis[1] != 0 or c.axis[2] != 128 or c.axis[3] != 128 or c.axis[4] != 128 or c.axis[5] != 128)
                        any_keyboard_key_pressed = true;

                    if (!any_keyboard_key_pressed) {
                        if (self.controllers[controller_idx]) |host_controller| {
                            if (host_controller.id.isPresent()) {
                                if (host_controller.id.asGamepad()) |gamepad| {
                                    const gamepad_state = gamepad.getState() catch continue;
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
                                    c.axis[0] = @as(u8, @intFromFloat(std.math.clamp(gamepad_state.axes[@intFromEnum(zglfw.Gamepad.Axis.right_trigger)], 0.0, 1.0) * 255));
                                    c.axis[1] = @as(u8, @intFromFloat(std.math.clamp(gamepad_state.axes[@intFromEnum(zglfw.Gamepad.Axis.left_trigger)], 0.0, 1.0) * 255));

                                    const capabilities: DreamcastModule.Maple.InputCapabilities = @bitCast(c.subcapabilities[0]);
                                    inline for ([_]struct { host: zglfw.Gamepad.Axis, guest: u8 }{
                                        .{ .host = .left_x, .guest = 2 },
                                        .{ .host = .left_y, .guest = 3 },
                                        .{ .host = .right_x, .guest = 4 },
                                        .{ .host = .right_y, .guest = 5 },
                                    }, 0..) |binding, idx| {
                                        if (@field(capabilities, ([_][]const u8{ "analogHorizontal", "analogVertical", "analogHorizontal2", "analogVertical2" })[idx]) != 0) {
                                            var value = gamepad_state.axes[@intFromEnum(binding.host)];
                                            if (@abs(value) < host_controller.deadzone)
                                                value = 0.0;
                                            // TODO: Remap with deadzone?
                                            value = value * 0.5 + 0.5;
                                            c.axis[binding.guest] = @as(u8, @intFromFloat(std.math.ceil(value * 255)));
                                        }
                                    }
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

pub fn load_and_start(self: *@This(), path: []const u8) !void {
    self.pause();
    try self.load_disc(path);
    self.dc.set_region(self.dc.gdrom.disc.?.get_region()) catch |err| {
        switch (err) {
            error.FileNotFound => return error.MissingFlash,
            else => return err,
        }
    };
    try self.on_game_load();
    try self.dc.reset();
    self.start();
    self.display_ui = false;
}

pub fn load_disc(self: *@This(), path: []const u8) !void {
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
        const tmp_gdi_path = try std.fs.path.join(self._allocator, &[_][]const u8{ HostPaths.get_userdata_path(), TmpDirPath, gdi_filename });
        std.log.info("Found GDI file: '{s}'.", .{gdi_filename});
        std.log.info("Extracting zip to '{s}'...", .{tmp_gdi_path});
        var tmp_dir = try std.fs.cwd().makeOpenPath(tmp_gdi_path, .{});
        defer tmp_dir.close();
        try std.zip.extract(tmp_dir, stream, .{});
        self.dc.gdrom.disc = try .init(tmp_gdi_path, self._allocator);
    } else {
        self.dc.gdrom.disc = try .init(path, self._allocator);
    }
}

pub fn get_product_name(self: *@This()) ?[]const u8 {
    return if (self.dc.gdrom.disc) |*disc| disc.get_product_name() else null;
}

pub fn get_product_id(self: *@This()) ?[]const u8 {
    return if (self.dc.gdrom.disc) |*disc| disc.get_product_id() else null;
}

/// Game specific sub directory name (for VMUs, save states...)
/// Caller owns the returned string.
fn userdata_game_directory(self: *@This()) ![]const u8 {
    const product_id = self.get_product_id() orelse "default";
    const product_name = self.get_product_name() orelse "default";
    const folder_name = try std.fmt.allocPrint(self._allocator, "{s}[{s}]", .{ product_name, product_id });
    safe_path(folder_name);
    defer self._allocator.free(folder_name);
    const path = try std.fs.path.join(self._allocator, &[_][]const u8{ HostPaths.get_userdata_path(), folder_name });
    return path;
}

pub fn on_game_load(self: *@This()) !void {
    if (self.config.per_game_vmu) {
        const game_dir = try self.userdata_game_directory();
        defer self._allocator.free(game_dir);
        const vmu_path = try std.fs.path.join(self._allocator, &[_][]const u8{ game_dir, "vmu_0.bin" });
        defer self._allocator.free(vmu_path);

        if (self.dc.maple.ports[0].subperipherals[0]) |*peripheral| {
            switch (peripheral.*) {
                .VMU => |*vmu| vmu.deinit(self._allocator),
                else => {},
            }
        }
        self.dc.maple.ports[0].subperipherals[0] = .{ .VMU = try .init(self._allocator, vmu_path) };
        self.dc.maple.ports[0].subperipherals[0].?.VMU.on_screen_update = .{ .function = @ptrCast(&UI.update_vmu_screen_0_0), .userdata = self.ui };
    }
    try self.check_save_state_slots();

    var title = try std.ArrayList(u8).initCapacity(self._allocator, 64);
    defer title.deinit();
    try title.appendSlice("Deecy");
    if (self.get_product_name()) |name| {
        try title.appendSlice(" - ");
        try title.appendSlice(name);
        if (self.get_product_id()) |id| {
            try title.appendSlice(" (");
            try title.appendSlice(id);
            try title.append(')');
        }
    }
    try title.append(0);
    self.window.setTitle(title.items[0 .. title.items.len - 1 :0]);
}

// Caller owns the returned ArrayList
fn save_state_path(self: *@This(), index: usize) !std.ArrayList(u8) {
    const game_dir = try self.userdata_game_directory();
    defer self._allocator.free(game_dir);
    var save_slot_path = std.ArrayList(u8).init(self._allocator);
    try save_slot_path.writer().print("{s}/save_{d}.sav", .{ game_dir, index });
    return save_slot_path;
}

fn check_save_state_slots(self: *@This()) !void {
    for (0..self.save_state_slots.len) |i| {
        var save_slot_path = try self.save_state_path(i);
        defer save_slot_path.deinit();
        self.save_state_slots[i] = try file_exists(save_slot_path.items);
    }
}

pub fn toggle_fullscreen(self: *@This()) void {
    if (self.config.fullscreen) {
        self.config.fullscreen = false;
        self.window.setMonitor(null, self.previous_window_position.x, self.previous_window_position.y, self.previous_window_position.w, self.previous_window_position.h, 0);
    } else {
        self.previous_window_position = .{ .x = self.window.getPos()[0], .y = self.window.getPos()[1], .w = self.window.getSize()[0], .h = self.window.getSize()[1] };
        const monitor = zglfw.Monitor.getPrimary().?;
        const mode = monitor.getVideoMode() catch |err| {
            deecy_log.err(termcolor.red("Failed to get video mode: {}"), .{err});
            return;
        };
        self.window.setMonitor(monitor, 0, 0, mode.width, mode.height, mode.refresh_rate);
        self.config.fullscreen = true;
    }
}

pub fn start(self: *@This()) void {
    if (!self.running) {
        if (self.dc.region == .Unknown) {
            self.dc.set_region(.USA) catch |err| std.debug.panic("Failed to set default region: {}", .{err});
        }

        if (!self.realtime) {
            self.running = true;
            self._dc_thread = std.Thread.spawn(.{}, dc_thread_loop, .{self}) catch |err| {
                deecy_log.err(termcolor.red("Failed to spawn DC thread: {}"), .{err});
                self.running = false;
                return;
            };
        } else {
            var bw = std.io.bufferedWriter(std.io.getStdErr().writer());
            bw.flush() catch {}; // FIXME: There's an issue with running binary files on startup (on Windows) that this flush magically solves. I can't make sense of it.

            if (self.dc.aica.available_samples() <= 32)
                self.run_for(AICA.SH4CyclesPerSample * 16); // Preemptively accumulate some samples

            self.audio_device.start() catch |err| {
                deecy_log.err(termcolor.red("Failed to start audio device: {}"), .{err});
                return;
            };
            self.running = true;
        }
    }
}

pub fn pause(self: *@This()) void {
    if (self.running) {
        if (!self.realtime) {
            self.running = false;
            if (self._dc_thread) |dc_thread| dc_thread.join();
            self._dc_thread = null;
        } else {
            self.audio_device.stop() catch |err| {
                deecy_log.err(termcolor.red("Failed to stop audio device: {}"), .{err});
                return;
            };
            self.running = false;
            self.dc.maple.flush_vmus();
        }
    }
}

pub fn set_realtime(self: *@This(), realtime: bool) void {
    if (self.realtime == realtime) return;
    const was_running = self.running;
    if (was_running) self.pause();
    defer if (was_running) self.start();
    self.realtime = realtime;
}

// Used for uncapped framerate (no audio output)
pub fn dc_thread_loop(self: *@This()) void {
    while (self.running) {
        self.run_for(128);
    }
}

pub fn on_resize(self: *@This()) void {
    if (!self.config.fullscreen) {
        const ww = self.gctx.swapchain_descriptor.width;
        const wh = self.gctx.swapchain_descriptor.height;
        self.config.window_size.width = ww;
        self.config.window_size.height = wh;
    }
    self.renderer.update_blit_to_screen_vertex_buffer();
}

pub fn draw_ui(self: *@This()) !void {
    zgui.backend.newFrame(
        self.gctx.swapchain_descriptor.width,
        self.gctx.swapchain_descriptor.height,
    );

    _ = zgui.DockSpaceOverViewport(0, zgui.getMainViewport(), .{ .passthru_central_node = true });

    self.ui.draw_vmus(self.display_ui);

    if (self.display_ui) {
        try self.ui.draw();
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

    self.ui.notifications.draw();

    self.submit_ui();
}

fn submit_ui(self: *@This()) void {
    const swapchain_texv = self.gctx.swapchain.getCurrentTextureView();
    defer swapchain_texv.release();

    const commands = commands: {
        const encoder = self.gctx.device.createCommandEncoder(null);
        defer encoder.release();
        // GUI pass
        {
            const pass = zgpu.beginRenderPassSimple(encoder, .load, swapchain_texv, null, null, null);
            defer zgpu.endReleasePass(pass);
            zgui.backend.draw(pass);
        }
        break :commands encoder.finish(null);
    };
    defer commands.release();

    self.gctx.submit(&.{commands});
}

// Display an error message and wait for the user to close the window.
fn display_unrecoverable_error(self: *@This(), comptime fmt: []const u8, args: anytype) void {
    while (!self.window.shouldClose()) {
        zglfw.pollEvents();

        zgui.backend.newFrame(self.gctx.swapchain_descriptor.width, self.gctx.swapchain_descriptor.height);

        if (!zgui.isPopupOpen("Error##Modal", .{})) {
            zgui.openPopup("Error##Modal", .{});
        }

        if (zgui.beginPopupModal("Error##Modal", .{ .flags = .{ .always_auto_resize = true } })) {
            zgui.text(fmt, args);
            if (zgui.button("OK", .{})) {
                zglfw.setWindowShouldClose(self.window, true);
            }
            zgui.endPopup();
        }

        self.submit_ui();

        _ = self.gctx.present();
    }
}

fn run_for(self: *@This(), sh4_cycles: u64) void {
    self._cycles_to_run += @intCast(sh4_cycles);
    if (self.enable_jit) {
        while (self._cycles_to_run > 0) {
            self._cycles_to_run -= self.dc.tick_jit() catch |err| {
                deecy_log.err("Error running JIT: {}", .{err});
                self._stop_request = true;
                return;
            };
        }
    } else {
        const max_instructions: u8 = if (self.breakpoints.items.len == 0) 16 else 1;

        while (self.running and self._cycles_to_run > 0) {
            self._cycles_to_run -= self.dc.tick(max_instructions) catch |err| {
                deecy_log.err("Error running interpreter: {}", .{err});
                return;
            };

            // Doesn't make sense to try to have breakpoints if the interpreter can execute more than one instruction at a time.
            if (max_instructions == 1) {
                const breakpoint = for (self.breakpoints.items, 0..) |addr, index| {
                    if (addr & 0x1FFFFFFF == self.dc.cpu.pc & 0x1FFFFFFF) break index;
                } else null;
                if (breakpoint != null) {
                    self._stop_request = true; // Can't stop the audio device from inside the audio callback.
                    return;
                }
            }
        }
    }
}

fn audio_callback(
    device: *zaudio.Device,
    output: ?*anyopaque,
    _: ?*const anyopaque, // Input
    frame_count: u32,
) callconv(.C) void {
    const self: *@This() = @ptrCast(@alignCast(device.getUserData()));
    const aica = &self.dc.aica;

    if (!self.running or self._stop_request) return;

    const sh4_cycles = AICA.SH4CyclesPerSample * frame_count;
    self.run_for(sh4_cycles);

    var out: [*]i32 = @ptrCast(@alignCast(output));

    const available = aica.available_samples();
    if (available <= 0) return;

    // std.debug.print("audio_callback: frame_count={d}, available={d}\n", .{ frame_count, available });

    for (0..@min(@as(usize, @intCast(available)), 2 * frame_count)) |i| {
        out[i] = 30000 *| aica.sample_buffer[aica.sample_read_offset];
        aica.sample_read_offset = (aica.sample_read_offset + 1) % aica.sample_buffer.len;
    }
}

pub const SaveStateHeader = extern struct {
    const Signature: [8]u8 = .{ 'D', 'E', 'E', 'C', 'Y', 'S', 'A', 'V' };
    const Version: u32 = 6;

    signature: [Signature.len]u8 = Signature,
    version: u16 = Version,
    _reserved: [2]u8 = @splat(0), // Could have reserved more here, for a disc ID for example, but I don't care about backward compatibility for now.
    uncompressed_size: u32, // Allocation optimisation and easy integrity check.
    compressed_size: u32, // Same.

    pub fn validate(self: *const SaveStateHeader) !void {
        if (!std.mem.eql(u8, &self.signature, &SaveStateHeader.Signature)) return error.InvalidSaveState;
        if (self.version != SaveStateHeader.Version) return error.InvalidSaveStateVersion;
        if (!std.mem.eql(u8, &self._reserved, &[2]u8{ 0, 0 })) return error.InvalidSaveState;
        if (self.uncompressed_size == 0) return error.InvalidSaveState;
        if (self.compressed_size == 0) return error.InvalidSaveState;
    }
};

pub fn save_state(self: *@This(), index: usize) !void {
    const was_running = self.running;
    if (was_running) self.pause();
    defer {
        if (was_running) self.start();
    }

    const start_time = std.time.milliTimestamp();
    deecy_log.info("Saving State #{d}...", .{index});

    var uncompressed_array = try std.ArrayList(u8).initCapacity(self._allocator, 32 * 1024 * 1024);
    var writer = uncompressed_array.writer();
    _ = try self.dc.serialize(writer);
    _ = try writer.write(std.mem.asBytes(&self._cycles_to_run));

    deecy_log.info("  Serialized state in {d} ms. Compressing...", .{std.time.milliTimestamp() - start_time});

    try self.launch_async(compress_and_dump_save_state, .{ self, index, uncompressed_array });
}

fn compress_and_dump_save_state(self: *@This(), index: usize, uncompressed_array: std.ArrayList(u8)) !void {
    const start_time = std.time.milliTimestamp();
    defer uncompressed_array.deinit();

    const compressed = try lz4.Standard.compress(self._allocator, uncompressed_array.items);
    defer self._allocator.free(compressed);

    var save_slot_path = try self.save_state_path(index);
    defer save_slot_path.deinit();
    var file = try std.fs.cwd().createFile(save_slot_path.items, .{});
    defer file.close();
    _ = try file.write(std.mem.asBytes(&SaveStateHeader{
        .uncompressed_size = @intCast(uncompressed_array.items.len),
        .compressed_size = @intCast(compressed.len),
    }));
    _ = try file.write(compressed);

    self.save_state_slots[index] = true;

    deecy_log.info("  Saved State #{d} to '{s}' in {d}ms", .{ index, save_slot_path.items, std.time.milliTimestamp() - start_time });

    self.ui.notifications.push("State Saved", .{}, "State #{d} saved successfully.", .{index});
}

pub fn load_state(self: *@This(), index: usize) !void {
    const was_running = self.running;
    if (was_running) self.pause();
    defer {
        if (was_running) self.start();
    }

    var save_slot_path = try self.save_state_path(index);
    defer save_slot_path.deinit();

    deecy_log.info("Loading State #{d} from '{s}'...", .{ index, save_slot_path.items });

    const start_time = std.time.milliTimestamp();

    var file = try std.fs.cwd().openFile(save_slot_path.items, .{});
    defer file.close();

    var header: SaveStateHeader = undefined;
    const header_size = try file.read(std.mem.asBytes(&header));
    if (header_size != @sizeOf(SaveStateHeader)) return error.InvalidSaveState;
    try header.validate();

    const compressed = try file.readToEndAllocOptions(self._allocator, 32 * 1024 * 1024, header.compressed_size, 8, null);
    defer self._allocator.free(compressed);

    if (header.compressed_size != compressed.len) return error.UnexpectedSaveStateSize;

    const decompressed = try lz4.Standard.decompress(self._allocator, compressed, header.uncompressed_size);
    defer self._allocator.free(decompressed);

    var uncompressed_stream = std.io.fixedBufferStream(decompressed);
    var reader = uncompressed_stream.reader();

    try self.reset();

    var bytes = try self.dc.deserialize(&reader);
    bytes += try reader.read(std.mem.asBytes(&self._cycles_to_run));

    if (bytes != header.uncompressed_size)
        deecy_log.err(termcolor.red("Loading save state used {d} bytes, expected {d}"), .{ bytes, header.uncompressed_size });

    deecy_log.info("Loaded State #{d} from '{s}' in {d}ms", .{ index, save_slot_path.items, std.time.milliTimestamp() - start_time });

    self.ui.notifications.push("State Loaded", .{}, "Save State #{d} loaded successfully.", .{index});
}

fn save_config(self: *@This()) !void {
    const config_path = try std.fs.path.join(self._allocator, &[_][]const u8{ HostPaths.get_userdata_path(), ConfigFile });
    defer self._allocator.free(config_path);
    var config_file = try std.fs.cwd().createFile(config_path, .{});
    defer config_file.close();
    try std.json.stringify(self.config, .{}, config_file.writer());
}

pub fn wait_async_jobs(self: *@This()) void {
    if (self._thread) |thread| {
        thread.join();
        self._thread = null;
    }
}

pub fn launch_async(self: *@This(), func: anytype, args: anytype) !void {
    self.wait_async_jobs();
    self._thread = try std.Thread.spawn(.{}, func, args);
}
