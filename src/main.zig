const std = @import("std");
const builtin = @import("builtin");
const config = @import("config");

const termcolor = @import("termcolor");
const custom_log = @import("custom_log.zig");

const DreamcastModule = @import("dreamcast");
const Holly = DreamcastModule.HollyModule;
const MapleModule = DreamcastModule.Maple;
const PreciseSleep = @import("precise_sleep.zig");

const zglfw = @import("zglfw");

const Deecy = @import("deecy.zig");

pub const std_options: std.Options = .{
    .log_level = .info,
    .logFn = custom_log.log,
    .log_scope_levels = &[_]std.log.ScopeLevel{
        .{ .scope = .elf, .level = .warn },
        .{ .scope = .dc, .level = .warn },
        .{ .scope = .sh4, .level = .warn },
        .{ .scope = .sh4_scif, .level = .info },
        .{ .scope = .mmu, .level = .info },
        .{ .scope = .sh4_jit, .level = .warn },
        .{ .scope = .arm_jit, .level = .info },
        .{ .scope = .x86_64_emitter, .level = .info },
        .{ .scope = .syscall, .level = .info },
        .{ .scope = .aica, .level = .info },
        .{ .scope = .dsp, .level = .info },
        .{ .scope = .holly, .level = .warn },
        .{ .scope = .gdrom, .level = .warn },
        .{ .scope = .gdrom_hle, .level = .info },
        .{ .scope = .cdi, .level = .info },
        .{ .scope = .chd, .level = .info },
        // .{ .scope = .chd_flac, .level = .debug },
        // .{ .scope = .chd_flac_frame, .level = .debug },
        // .{ .scope = .chd_flac_subframe, .level = .debug },
        // .{ .scope = .chd_flac_residual, .level = .debug },
        .{ .scope = .cue, .level = .warn },
        .{ .scope = .maple, .level = .info },
        .{ .scope = .renderer, .level = .info },
        .{ .scope = .pipeline_cache, .level = .info },
        .{ .scope = .flashrom, .level = .warn },
    },
};

fn trapa_handler(app: *anyopaque) void {
    @as(*Deecy, @ptrCast(@alignCast(app))).pause();
}

pub extern "kernel32" fn timeBeginPeriod(uPeriod: std.os.windows.UINT) callconv(.winapi) std.os.windows.UINT; // MMRESULT
pub extern "kernel32" fn timeEndPeriod(uPeriod: std.os.windows.UINT) callconv(.winapi) std.os.windows.UINT; // MMRESULT
pub extern "kernel32" fn AttachConsole(dwProcessId: std.os.windows.DWORD) callconv(.winapi) std.os.windows.BOOL;
const ATTACH_PARENT_PROCESS = ~@as(std.os.windows.DWORD, 0);

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    const allocator = init.gpa;

    custom_log.init(io, allocator);
    defer custom_log.deinit();

    try DreamcastModule.HostPaths.init(io, allocator, init.environ_map.*);
    defer DreamcastModule.HostPaths.deinit(allocator);

    defer DreamcastModule.SH4Module.SCIF.deinit(io);

    if (builtin.os.tag == .windows and config.no_console) {
        // When built with the GUI subsystem on Windows, try to attach to the console if we received any arguments.
        if (init.minimal.args.vector.len > 1)
            _ = AttachConsole(ATTACH_PARENT_PROCESS);
    }
    const wayland = wl: {
        if (builtin.os.tag == .windows) break :wl false;
        if (init.minimal.environ.getPosix("XDG_SESSION_TYPE")) |session_type| {
            break :wl std.mem.eql(u8, session_type, "wayland");
        }
        break :wl false;
    };

    var d = try Deecy.create(allocator, io, .{ .wayland = wayland });
    defer d.destroy();
    var dc = d.dc;

    var binary_path: ?[]const u8 = null;
    var ip_bin_path: ?[]const u8 = null;

    var disc_path: ?[]const u8 = null;

    var start_immediately = false;
    var force_stop = false;
    var force_render = false; // Enable to re-render every time and help capturing with RenderDoc (will mess with framebuffer emulation).
    var load_state: ?u32 = null;

    var args_iterator = try init.minimal.args.iterateAllocator(allocator);
    defer args_iterator.deinit();
    _ = args_iterator.skip();
    while (args_iterator.next()) |arg| {
        if (std.mem.startsWith(u8, arg, "-")) {
            if (std.mem.eql(u8, arg, "-b")) {
                binary_path = args_iterator.next() orelse {
                    std.log.err(termcolor.red("Expected path to binary file after -b."), .{});
                    return error.InvalidArguments;
                };
            } else if (std.mem.eql(u8, arg, "-g")) {
                disc_path = args_iterator.next() orelse {
                    std.log.err(termcolor.red("Expected path to disc file after -g."), .{});
                    return error.InvalidArguments;
                };
            } else if (std.mem.eql(u8, arg, "--launcher")) {
                try d.load_launcher();
                start_immediately = true;
            } else if (std.mem.eql(u8, arg, "-i")) {
                ip_bin_path = args_iterator.next() orelse {
                    std.log.err(termcolor.red("Expected path to IP.bin after -i."), .{});
                    return error.InvalidArguments;
                };
            } else if (std.mem.eql(u8, arg, "--vmu")) {
                const path = args_iterator.next() orelse {
                    std.log.err(termcolor.red("Expected path to VMU after --vmu."), .{});
                    return error.InvalidArguments;
                };
                try d.load_vmu(0, 0, path);
                d.config.per_game_vmu = false;
            } else if (std.mem.eql(u8, arg, "-d")) {
                dc.cpu.debug_trace = true;
            } else if (std.mem.eql(u8, arg, "--stop")) {
                force_stop = true;
            } else if (std.mem.eql(u8, arg, "--force-render")) {
                force_render = true;
            } else if (std.mem.eql(u8, arg, "--no-realtime")) {
                d.realtime = false;
            } else if (std.mem.eql(u8, arg, "--fullscreen")) {
                if (!d.config.fullscreen)
                    d.toggle_fullscreen();
            } else if (std.mem.eql(u8, arg, "--load-state")) {
                const num_str = args_iterator.next() orelse {
                    std.log.err(termcolor.red("Expected state number after --load-state."), .{});
                    return error.InvalidArguments;
                };
                load_state = std.fmt.parseInt(u32, num_str, 10) catch |err| {
                    std.log.err(termcolor.red("Invalid state number after --load-state: {t}"), .{err});
                    return error.InvalidArguments;
                };
                if (load_state.? >= Deecy.MaxSaveStates) {
                    std.log.err(termcolor.red("Invalid state number after --load-state: {d}"), .{load_state.?});
                    return error.InvalidArguments;
                }
            } else if (std.mem.eql(u8, arg, "--scif")) {
                try DreamcastModule.SH4Module.SCIF.init(io);
            } else if (std.mem.eql(u8, arg, "--attach-console")) {
                // Argument used to attach the console on Windows in GUI subsystem, see above.
            } else {
                std.log.warn(termcolor.yellow("Unknown argument: '{s}'"), .{arg});
            }
        } else {
            if (std.mem.endsWith(u8, arg, ".bin") or std.mem.endsWith(u8, arg, ".elf")) {
                binary_path = arg;
            } else if (std.mem.endsWith(u8, arg, ".gdi") or std.mem.endsWith(u8, arg, ".cdi") or std.mem.endsWith(u8, arg, ".chd") or std.mem.endsWith(u8, arg, ".cue")) {
                disc_path = arg;
            } else {
                std.log.warn("Unsupported file format: '{s}'", .{arg});
            }
        }
    }

    if (binary_path) |path| {
        try d.load_binary(path, ip_bin_path);
        start_immediately = true;
    } else if (disc_path) |path| {
        std.log.info("Loading Disc: '{s}'...", .{path});
        try d.load_disc(path);
        if (load_state) |state| try d.load_state(state);
        start_immediately = true;
        d.set_display_ui(false);
    } else if (d.config.auto_start_launcher) {
        try d.load_launcher();
        start_immediately = true;
    } else {
        try d.launch_async(Deecy.UI.refresh_games, .{d.ui});
    }

    dc.cpu.set_trapa_callback(trapa_handler, d);

    if (!force_stop and start_immediately) {
        d.wait_async_jobs();
        d.start();
    }

    // Request 1ms timer resolution on Windows.
    if (builtin.os.tag == .windows) _ = timeBeginPeriod(1);
    defer {
        if (builtin.os.tag == .windows) _ = timeEndPeriod(1);
    }

    var precise_sleep: PreciseSleep = .init(io);
    defer precise_sleep.deinit();
    var then = zglfw.getTime();
    while (!d.window.shouldClose()) {
        zglfw.pollEvents();

        const now = zglfw.getTime();
        d.update(@floatCast(now - then));
        then = now;

        if (try d.check_resize() == .Resizing) {
            // Skip rendering while resizing. Keep presenting to update window size.
            try d.gctx_queue_mutex.lock(d.io);
            defer d.gctx_queue_mutex.unlock(d.io);
            _ = d.gctx.present();
            continue;
        }

        if (d.dc.gpu.read_register(Holly.FB_R_CTRL, .FB_R_CTRL).enable) {
            // Framebuffer has been written to by the CPU.
            // Update the host texture and blit it to our render target.
            if (d.dc.gpu.dirty_framebuffer) {
                d.dc.gpu.dirty_framebuffer = false;
                try d.gctx_queue_mutex.lock(io);
                defer d.gctx_queue_mutex.unlock(io);
                d.renderer.update_framebuffer_texture(&d.dc.gpu);
                // FIXME: Yet another framebuffer hack.
                //        Skip the framebuffer blit if we recently used the PVR for rendering.
                //        Some games (like Speed Devils) renders only at 30FPS and each frame is presented twice,
                //        however we don't actually write back the framebuffer to VRAM, meaning we'd blit garbage to the screen.
                //        Plus, even if PVR writing to the framebuffer was perfectly emulated, it would still only be at native resolution.
                if (std.Io.Clock.awake.now(d.io).toMicroseconds() - d.renderer.last_frame_timestamp > 40_000) {
                    d.renderer.update_registers(&d.dc.gpu);
                    d.renderer.blit_framebuffer();
                }
            }

            d.renderer.draw(d.config.window_size.width, d.config.window_size.height, d.config.renderer.aspect_ratio); //  Blit to screen
        }

        try d.draw_ui();
        d.submit_ui();

        const resized = resized: {
            try d.gctx_queue_mutex.lock(io);
            defer d.gctx_queue_mutex.unlock(io);
            break :resized d.gctx.present() == .surface_reconfigured;
        };
        if (resized)
            d.on_resize();

        if (d.config.frame_limiter != .Off) {
            const ns_per_frame: u64 = switch (d.config.frame_limiter) {
                .Auto => d.dc.target_refresh_rate().ns_per_frame(),
                .@"120Hz" => 8_333_333,
                .@"100Hz" => 10_000_000,
                .@"60Hz" => 16_666_666,
                .@"59.94Hz" => 16_683_350,
                .@"50Hz" => 20_000_000,
                .Off => unreachable,
            };
            precise_sleep.wait_for_interval(io, ns_per_frame);
        }
    }
}
