const std = @import("std");
const builtin = @import("builtin");

const common = @import("./common.zig");
const arm7 = @import("arm7");
const termcolor = @import("./termcolor.zig");
const sh4 = @import("./sh4.zig");
const P4Register = sh4.P4Register;
const sh4_disassembly = @import("./sh4_disassembly.zig");
const HardwareRegisters = @import("./hardware_registers.zig");
const Dreamcast = @import("./dreamcast.zig").Dreamcast;
const GDI = @import("./gdi.zig").GDI;
const Holly = @import("./holly.zig");
const MapleModule = @import("./maple.zig");

const zgui = @import("zgui");
const zgpu = @import("zgpu");
const zglfw = @import("zglfw");

const RendererModule = @import("renderer.zig");
const Renderer = RendererModule.Renderer;

const Deecy = @import("deecy.zig").Deecy;

pub const std_options: std.Options = .{
    .log_level = .info,
    .log_scope_levels = &[_]std.log.ScopeLevel{
        .{ .scope = .dc, .level = .err },
        .{ .scope = .sh4, .level = .info },
        .{ .scope = .sh4_jit, .level = .info },
        .{ .scope = .arm_jit, .level = .info },
        .{ .scope = .x86_64_emitter, .level = .info },
        .{ .scope = .syscall_log, .level = .info },
        .{ .scope = .aica, .level = .info },
        .{ .scope = .holly, .level = .info },
        .{ .scope = .gdrom, .level = .info },
        .{ .scope = .maple, .level = .info },
        .{ .scope = .renderer, .level = .info },
    },
};

// FIXME.
var global_application: *Deecy = undefined;
fn trapa_handler() void {
    global_application.running = false;
}

pub fn main() !void {
    var d = try Deecy.create(common.GeneralAllocator);
    defer d.destroy();
    global_application = d;
    var dc = d.dc;

    var binary_path: ?[]const u8 = null;
    var ip_bin_path: ?[]const u8 = null;

    var gdi_path: ?[]const u8 = null;

    var skip_bios = false;

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
        if (std.mem.eql(u8, arg, "-d")) {
            dc.cpu.debug_trace = true;
        }
        if (std.mem.eql(u8, arg, "--skip-bios")) {
            skip_bios = true;
        }
        if (std.mem.eql(u8, arg, "--stop")) {
            d.running = false;
        }
    }

    if (gdi_path) |path|
        dc.gdrom.disk = try GDI.init(path, common.GeneralAllocator);

    if (binary_path) |path| {
        dc.skip_bios();

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
        if (skip_bios) {
            dc.skip_bios();

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
            dc.skip_bios();
            // Skip IP.bin (Maybe we should bundle one to load here).
            dc.cpu.pc = 0xAC010000;
        }
    }

    dc.cpu.on_trapa = trapa_handler;

    var last_frame_timestamp = std.time.microTimestamp();
    var last_n_frametimes = std.fifo.LinearFifo(i64, .Dynamic).init(common.GeneralAllocator);

    var blit_framebuffer_from_vram = true;

    while (!d.window.shouldClose()) {
        zglfw.pollEvents();

        zgui.backend.newFrame(
            d.gctx.swapchain_descriptor.width,
            d.gctx.swapchain_descriptor.height,
        );

        zgui.setNextWindowPos(.{ .x = 0, .y = 0 });
        if (zgui.begin("##FPSCounter", .{ .flags = .{ .no_resize = true, .no_move = true, .no_background = true, .no_title_bar = true } })) {
            var sum: i128 = 0;
            for (0..last_n_frametimes.count) |i| {
                sum += last_n_frametimes.peekItem(i);
            }
            const avg: f32 = @as(f32, @floatFromInt(sum)) / @as(f32, @floatFromInt(last_n_frametimes.count));
            zgui.text("FPS: {d: >4.1} ({d: >3.1}ms)", .{ 1000000.0 / avg, avg / 1000.0 });
        }
        zgui.end();

        try d.debug_ui.draw(d);

        d.pool_controllers();

        if (d.running) {
            const start = try std.time.Instant.now();
            // FIXME: We break on render start for synchronization, this is not how we'll want to do it in the end.
            while (d.running and (try std.time.Instant.now()).since(start) < 16 * std.time.ns_per_ms and !dc.gpu.render_start) {
                if (!d.enable_jit) {
                    const max_instructions: u8 = if (d.breakpoints.items.len == 0) 16 else 1;

                    _ = try dc.tick(max_instructions);

                    // Doesn't make sense to try to have breakpoints if the interpreter can execute more than one instruction at a time.
                    if (max_instructions == 1) {
                        const breakpoint = for (d.breakpoints.items, 0..) |addr, index| {
                            if (addr & 0x1FFFFFFF == dc.cpu.pc & 0x1FFFFFFF) break index;
                        } else null;
                        if (breakpoint != null) {
                            d.running = false;
                        }
                    }
                } else {
                    for (0..32) |_| {
                        _ = try dc.tick_jit();
                        if (dc.gpu.render_start) break;
                    }
                }
            }
        }

        const swapchain_texv = d.gctx.swapchain.getCurrentTextureView();
        defer swapchain_texv.release();

        // FIXME: I don't how to handle this correctly, copying the framebuffer from VRAM
        // is very expensive and generally useless outside of splash screen/homebrews.
        // However it is actually sometimes used in games, like Namco Museum.
        // TODO: I could start by only updating in on vblank.
        if (blit_framebuffer_from_vram) {
            d.renderer.update_framebuffer(&dc.gpu);
            d.renderer.blit_framebuffer();
        }

        // FIXME: Find a better way to start a render.
        if (dc.gpu.render_start) {
            // FIXME: Remove
            blit_framebuffer_from_vram = false;
            d.renderer.read_framebuffer_enabled = false;

            dc.gpu.render_start = false;
            try d.renderer.update(&dc.gpu);

            if (last_n_frametimes.count >= 10) {
                _ = last_n_frametimes.readItem();
            }
            const now = std.time.microTimestamp();
            try last_n_frametimes.writeItem(now - last_frame_timestamp);
            last_frame_timestamp = now;
        }
        // FIXME: Complete render every frame to help capture debugging, this could be called only on render_start.
        try d.renderer.render();

        d.renderer.draw(); //  Blit to screen

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
            d.renderer.on_resize();
        }
    }
}

test "all tests" {
    _ = sh4;
    _ = Dreamcast;
}
