const std = @import("std");

const zglfw = @import("zglfw");
const zgpu = @import("zgpu");
const zgui = @import("zgui");

const common = @import("./common.zig");

const Dreamcast = @import("./dreamcast.zig").Dreamcast;

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

pub const Deecy = struct {
    const assets_dir = "assets/";

    window: *zglfw.Window,
    gctx: *zgpu.GraphicsContext = undefined,
    scale_factor: f32 = 1.0,

    dc: *Dreamcast,
    renderer: Renderer = undefined,

    running: bool = true,
    enable_jit: bool = true,
    breakpoints: std.ArrayList(u32),

    debug_ui: DebugUI = undefined,

    _allocator: std.mem.Allocator,

    pub fn create(allocator: std.mem.Allocator) !*Deecy {
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

        self.gctx = try zgpu.GraphicsContext.create(common.GeneralAllocator, self.window, .{
            .present_mode = .mailbox,
            .required_features = &[_]zgpu.wgpu.FeatureName{.bgra8_unorm_storage},
        });

        self.scale_factor = scale_factor: {
            const scale = self.window.getContentScale();
            break :scale_factor @max(scale[0], scale[1]);
        };

        self.renderer = try Renderer.init(self._allocator, self.gctx);

        try self.ui_init();

        return self;
    }

    fn ui_init(self: *Deecy) !void {
        zgui.init(common.GeneralAllocator);

        _ = zgui.io.addFontFromFile(
            assets_dir ++ "fonts/Hack-Regular.ttf",
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
