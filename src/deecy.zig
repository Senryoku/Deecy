const std = @import("std");

const zglfw = @import("zglfw");
const zgpu = @import("zgpu");
const zgui = @import("zgui");

const common = @import("./common.zig");

const DreamcastModule = @import("./dreamcast.zig");
const Dreamcast = DreamcastModule.Dreamcast;

const Renderer = @import("./renderer.zig").Renderer;

const DebugUI = @import("./debug_ui.zig");

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
        if (key == .escape and action == .press) {
            app.debug_ui.draw_debug_ui = !app.debug_ui.draw_debug_ui;
        }
    }
}

const assets_dir = "assets/";
const DefaultFont = @embedFile(assets_dir ++ "fonts/Hack-Regular.ttf");

pub const Deecy = struct {
    window: *zglfw.Window,
    gctx: *zgpu.GraphicsContext = undefined,
    scale_factor: f32 = 1.0,

    dc: *Dreamcast,
    renderer: Renderer = undefined,

    running: bool = true,
    enable_jit: bool = true,
    breakpoints: std.ArrayList(u32),

    controllers: [4]?zglfw.Joystick.Id = .{ null, null, null, null },

    debug_ui: DebugUI = undefined,

    _allocator: std.mem.Allocator,

    pub fn create(allocator: std.mem.Allocator) !*Deecy {
        std.fs.cwd().makeDir("userdata") catch |err| switch (err) {
            error.PathAlreadyExists => {},
            else => return err,
        };

        try zglfw.init();

        const self = try allocator.create(Deecy);
        self.* = Deecy{
            .window = try zglfw.Window.create(640 * 2, 480 * 2, "Deecy", null),
            .dc = try Dreamcast.create(common.GeneralAllocator),
            .breakpoints = std.ArrayList(u32).init(allocator),
            ._allocator = allocator,
        };

        self.window.setUserPointer(self);
        _ = self.window.setKeyCallback(glfw_key_callback);

        self.gctx = try zgpu.GraphicsContext.create(common.GeneralAllocator, .{
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
        });

        self.scale_factor = scale_factor: {
            const scale = self.window.getContentScale();
            break :scale_factor @max(scale[0], scale[1]);
        };

        self.renderer = try Renderer.init(self._allocator, self.gctx);

        try self.ui_init();

        var curr_pad: usize = 0;
        for (0..zglfw.Joystick.maximum_supported) |idx| {
            const jid: zglfw.Joystick.Id = @intCast(idx);
            if (zglfw.Joystick.get(jid)) |joystick| {
                if (joystick.asGamepad()) |_| {
                    self.controllers[curr_pad] = jid;
                    curr_pad += 1;
                    if (curr_pad >= 4)
                        break;
                }
            }
        }

        return self;
    }

    fn ui_init(self: *Deecy) !void {
        zgui.init(common.GeneralAllocator);

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
                                if (zglfw.Joystick.get(host_controller)) |joystick| {
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
                                        c.axis[2] = @as(u8, @intFromFloat((gamepad_state.axes[@intFromEnum(zglfw.Gamepad.Axis.left_x)] * 0.5 + 0.5) * 255));
                                        c.axis[3] = @as(u8, @intFromFloat((gamepad_state.axes[@intFromEnum(zglfw.Gamepad.Axis.left_y)] * 0.5 + 0.5) * 255));
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

    pub fn destroy(self: *Deecy) void {
        self.breakpoints.deinit();

        self.renderer.deinit();

        self.dc.deinit();
        self._allocator.destroy(self.dc);

        self.ui_deinit();

        self.gctx.destroy(self._allocator);

        self.window.destroy();
        zglfw.terminate();

        self._allocator.destroy(self);
    }
};
