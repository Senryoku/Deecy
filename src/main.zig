const std = @import("std");
const builtin = @import("builtin");

const common = @import("./common.zig");
const arm7 = @import("arm7");
const termcolor = @import("termcolor");
const sh4 = @import("./sh4.zig");
const P4Register = sh4.P4Register;
const sh4_disassembly = @import("./sh4_disassembly.zig");
const HardwareRegisters = @import("./hardware_registers.zig");
const DreamcastModule = @import("./dreamcast.zig");
const Dreamcast = DreamcastModule.Dreamcast;
const GDI = @import("./gdi.zig").GDI;
const Holly = @import("./holly.zig");
const MapleModule = @import("./maple.zig");

const zgui = @import("zgui");
const zgpu = @import("zgpu");
const zglfw = @import("zglfw");

const RendererModule = @import("renderer.zig");
const Renderer = RendererModule.Renderer;

const Deecy = @import("deecy.zig").Deecy;

pub fn customLog(
    comptime message_level: std.log.Level,
    comptime scope: @Type(.EnumLiteral),
    comptime format: []const u8,
    args: anytype,
) void {
    const static = struct {
        var last_message: struct {
            message_level: std.log.Level,
            // scope: @Type(.EnumLiteral),
            format: []const u8,
            args_hash: u64,
        } = undefined;
        var count: u32 = 0;
    };

    const args_hash = std.hash.CityHash64.hash(std.mem.asBytes(&args));

    if (message_level == static.last_message.message_level and
        //  scope == static.last_message.scope and
        std.mem.eql(u8, format, static.last_message.format) and
        args_hash == static.last_message.args_hash)
    {
        static.count +|= 1;
        const stderr = std.io.getStdErr().writer();
        var bw = std.io.bufferedWriter(stderr);
        const writer = bw.writer();

        std.debug.lockStdErr();
        defer std.debug.unlockStdErr();
        nosuspend {
            writer.print(termcolor.grey("\r  (...x{d})"), .{static.count}) catch return;
            bw.flush() catch return;
        }
        return;
    }

    if (static.count > 1) {
        const stderr = std.io.getStdErr().writer();
        var bw = std.io.bufferedWriter(stderr);
        const writer = bw.writer();

        std.debug.lockStdErr();
        defer std.debug.unlockStdErr();
        nosuspend {
            writer.print("\n", .{}) catch return;
            bw.flush() catch return;
        }
    }

    static.last_message = .{
        .message_level = message_level,
        //.scope = scope,
        .format = format,
        .args_hash = args_hash,
    };
    static.count = 1;

    std.log.defaultLog(message_level, scope, format, args);
}

pub const std_options: std.Options = .{
    .log_level = .info,
    .logFn = customLog,
    .log_scope_levels = &[_]std.log.ScopeLevel{
        .{ .scope = .dc, .level = .info },
        .{ .scope = .sh4, .level = .warn },
        .{ .scope = .sh4_jit, .level = .info },
        .{ .scope = .arm_jit, .level = .info },
        .{ .scope = .x86_64_emitter, .level = .info },
        .{ .scope = .syscall_log, .level = .info },
        .{ .scope = .aica, .level = .info },
        .{ .scope = .holly, .level = .info },
        .{ .scope = .gdrom, .level = .info },
        .{ .scope = .gdrom_hle_log, .level = .info },
        .{ .scope = .maple, .level = .info },
        .{ .scope = .renderer, .level = .info },
        .{ .scope = .flashrom, .level = .info },
    },
};

fn safe_path(path: []u8) void {
    for (path) |*c| {
        if (!((c.* >= 'a' and c.* <= 'z') or (c.* >= 'A' and c.* <= 'Z') or (c.* >= '0' and c.* <= '9') or c.* == '.' or c.* == '/')) {
            c.* = '_';
        }
    }
}

fn trapa_handler(app: *anyopaque) void {
    @as(*Deecy, @alignCast(@ptrCast(app))).stop();
}

const Configuration = struct {
    per_game_vmu: bool = true,
};

pub fn main() !void {
    var d = try Deecy.create(common.GeneralAllocator);
    defer d.destroy();
    var dc = d.dc;

    const config: Configuration = .{};

    var binary_path: ?[]const u8 = null;
    var ip_bin_path: ?[]const u8 = null;

    var gdi_path: ?[]const u8 = null;

    var default_vmu = true;
    var vmu_path = std.ArrayList(u8).init(common.GeneralAllocator);
    defer vmu_path.deinit();
    try vmu_path.appendSlice("./userdata/vmu_default.bin");

    var skip_bios = false;
    var start_immediately = true;

    var args = try std.process.argsWithAllocator(common.GeneralAllocator);
    defer args.deinit();
    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "-b")) {
            binary_path = args.next() orelse {
                std.log.err(termcolor.red("Expected path to binary file after -b."), .{});
                return error.InvalidArguments;
            };
        }
        if (std.mem.eql(u8, arg, "-g")) {
            gdi_path = args.next() orelse {
                std.log.err(termcolor.red("Expected path to GDI file after -g."), .{});
                return error.InvalidArguments;
            };
        }
        if (std.mem.eql(u8, arg, "-i")) {
            ip_bin_path = args.next() orelse {
                std.log.err(termcolor.red("Expected path to IP.bin after -i."), .{});
                return error.InvalidArguments;
            };
        }
        if (std.mem.eql(u8, arg, "--vmu")) {
            const path = args.next() orelse {
                std.log.err(termcolor.red("Expected path to VMU after --vmu."), .{});
                return error.InvalidArguments;
            };
            vmu_path.clearRetainingCapacity();
            try vmu_path.appendSlice(path);
            default_vmu = false;
        }
        if (std.mem.eql(u8, arg, "-d")) {
            dc.cpu.debug_trace = true;
        }
        if (std.mem.eql(u8, arg, "--skip-bios")) {
            skip_bios = true;
        }
        if (std.mem.eql(u8, arg, "--stop")) {
            start_immediately = false;
        }
    }

    if (binary_path) |path| {
        // FIXME: I'd rather be using LLE syscalls here,
        //        but at least the ROM font one requires some initialization
        //        and won't work if the boot ROM is skipped.
        dc.skip_bios(true);

        var bin_file = try std.fs.cwd().openFile(path, .{});
        defer bin_file.close();
        _ = try bin_file.readAll(dc.ram[0x10000..]);

        if (ip_bin_path) |ipb_path| {
            var ip_bin_file = try std.fs.cwd().openFile(ipb_path, .{});
            defer ip_bin_file.close();
            _ = try ip_bin_file.readAll(dc.ram[0x8000..]);
        } else {
            // Skip IP.bin
            dc.cpu.pc = 0xAC010000;
        }
    } else if (gdi_path) |path| {
        std.log.info("Loading GDI: {s}...", .{path});
        dc.gdrom.disk = try GDI.init(path, common.GeneralAllocator);

        const region = dc.gdrom.disk.?.get_region();
        std.log.info("  Detected region: {s}", .{@tagName(region)});
        if (region != .Unknown) {
            try dc.set_region(region);
        }

        if (default_vmu and config.per_game_vmu) {
            if (dc.gdrom.disk.?.get_product_id()) |product_id| {
                vmu_path.clearRetainingCapacity();
                try vmu_path.writer().print("./userdata/{s}/vmu_0.bin", .{product_id});
                safe_path(vmu_path.items);
            }
        }

        if (skip_bios) {
            dc.skip_bios(true);

            // Load 1STREAD.BIN (Actual name might change)
            const header_size: u32 = dc.gdrom.disk.?.tracks.items[2].header_size();
            const first_read_name = dc.gdrom.disk.?.tracks.items[2].data[0x60 + header_size .. 0x70 + header_size];
            const name_end = std.mem.indexOfScalar(u8, first_read_name, 0x20) orelse first_read_name.len;
            var first_read: []u8 = try common.GeneralAllocator.alloc(u8, name_end + 2);
            defer common.GeneralAllocator.free(first_read);
            @memcpy(first_read[0..name_end], first_read_name[0..name_end]);
            @memcpy(first_read[name_end .. name_end + 2], ";1");
            _ = try dc.gdrom.disk.?.load_file(first_read, dc.ram[0x00010000..]);
        }

        // FIXME: Hacks.
        // NOPs for DC Checker, skips serial check
        if (std.mem.count(u8, path, "Loop Checker version 1.00") > 0) {
            dc.cpu.write16(0x0C0196DA, 0x9);
            dc.cpu.write16(0x0C0196EC, 0x9);
        }

        // DC CHECKER for Repair v2.050
        if (std.mem.count(u8, path, "DC CHECKER for Repair v2.050") > 0) {
            dc.cpu.write16(0x0C018F54, 0x9);
            dc.cpu.write16(0x0C018F42, 0x9);
        }
    } else {
        if (skip_bios) {
            // Boot to menu
            dc.skip_bios(true);
            // Skip IP.bin (Maybe we should bundle one to load here).
            dc.cpu.pc = 0xAC010000;
        }
    }

    dc.maple.ports[0].subperipherals[0] = .{ .VMU = try MapleModule.VMU.init(common.GeneralAllocator, vmu_path.items) };

    dc.cpu.on_trapa = .{ .callback = trapa_handler, .userdata = d };

    var last_frame_timestamp = std.time.microTimestamp();
    var last_n_frametimes = std.fifo.LinearFifo(i64, .Dynamic).init(common.GeneralAllocator);

    var blit_framebuffer_from_vram = true;

    if (start_immediately)
        d.start();

    while (!d.window.shouldClose()) {
        d.one_frame();

        zglfw.pollEvents();

        zgui.backend.newFrame(
            d.gctx.swapchain_descriptor.width,
            d.gctx.swapchain_descriptor.height,
        );

        if (!d.debug_ui.draw_debug_ui) {
            zgui.setNextWindowPos(.{ .x = 0, .y = 0 });
            if (zgui.begin("##FPSCounter", .{ .flags = .{ .no_resize = true, .no_move = true, .no_background = true, .no_title_bar = true, .no_mouse_inputs = true, .no_nav_inputs = true, .no_nav_focus = true } })) {
                var sum: i128 = 0;
                for (0..last_n_frametimes.count) |i| {
                    sum += last_n_frametimes.peekItem(i);
                }
                const avg: f32 = @as(f32, @floatFromInt(sum)) / @as(f32, @floatFromInt(last_n_frametimes.count));
                zgui.text("FPS: {d: >4.1} ({d: >3.1}ms)", .{ 1000000.0 / avg, avg / 1000.0 });
            }
            zgui.end();
        }

        d.pool_controllers();

        // FIXME: I don't how to handle this correctly, copying the framebuffer from VRAM
        // is very expensive and generally useless outside of splash screen/homebrews.
        // However it is actually sometimes used in games, like Namco Museum.
        // TODO: I could start by only updating in on vblank.
        if (blit_framebuffer_from_vram) {
            d.renderer.update_framebuffer();
            d.renderer.blit_framebuffer();
        }

        const render_start = d.renderer.render_start;
        if (render_start) {
            // FIXME: Remove
            blit_framebuffer_from_vram = false;
            d.renderer.read_framebuffer_enabled = false;

            d.renderer.render_start = false;
            try d.renderer.update();

            if (last_n_frametimes.count >= 60) {
                _ = last_n_frametimes.readItem();
            }
            const now = std.time.microTimestamp();
            try last_n_frametimes.writeItem(now - last_frame_timestamp);
            last_frame_timestamp = now;
        }

        const swapchain_texv = d.gctx.swapchain.getCurrentTextureView();
        defer swapchain_texv.release();

        const always_render = builtin.mode != .ReleaseFast; // Enable to re-render every time and help capturing with RenderDoc.
        if (always_render or render_start)
            try d.renderer.render();

        d.renderer.draw(); //  Blit to screen

        try d.debug_ui.draw(d);

        const commands = commands: {
            const encoder = d.gctx.device.createCommandEncoder(null);
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

        d.gctx.submit(&.{commands});

        if (d.gctx.present() == .swap_chain_resized) {
            d.renderer.on_inner_resolution_change();
        }
    }
}

test "all tests" {
    _ = sh4;
    _ = Dreamcast;
}
