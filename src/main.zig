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
const Colors = @import("./colors.zig");
const MapleModule = @import("./maple.zig");

const zgui = @import("zgui");
const zgpu = @import("zgpu");
const zglfw = @import("zglfw");

const RendererModule = @import("renderer.zig");
const Renderer = RendererModule.Renderer;

const Deecy = @import("deecy.zig");

pub fn customLog(
    comptime message_level: std.log.Level,
    comptime scope: @Type(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    const static = struct {
        var last_message: struct {
            message_level: std.log.Level,
            // scope: @Type(.enum_literal),
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

fn trapa_handler(app: *anyopaque) void {
    @as(*Deecy, @alignCast(@ptrCast(app))).stop();
}

// FIXME: Temp PoC, do better.
var fb_mapping_available: bool = false;
fn signal_fb_mapped(status: zgpu.wgpu.BufferMapAsyncStatus, _: ?*anyopaque) void {
    switch (status) {
        .success => {},
        else => std.log.err(termcolor.red("Failed to map buffer: {s}"), .{@tagName(status)}),
    }
    fb_mapping_available = true;
}
fn write_back_fb(d: *Deecy) void {
    defer d.gctx.lookupResource(d.renderer.framebuffer_copy_buffer).?.unmap();
    const mapped_pixels = d.gctx.lookupResource(d.renderer.framebuffer_copy_buffer).?.getConstMappedRange(
        u8,
        0,
        4 * Renderer.NativeResolution.width * Renderer.NativeResolution.height,
    );
    if (mapped_pixels) |pixels| {
        d.dc.gpu.write_framebuffer(pixels);
    } else std.log.err(termcolor.red("Failed to map framebuffer"), .{});
}

const Hack = struct { addr: u32, instr: []const u16 };

const AvailableHacks = [_]struct { name: []const u8, hacks: []const Hack }{
    .{
        .name = "Loop Checker version 1.00",
        .hacks = &[_]Hack{
            .{ .addr = 0x0C0196DA, .instr = &[_]u16{0x9} },
            .{ .addr = 0x0C0196EC, .instr = &[_]u16{0x9} },
        },
    },
    .{
        .name = "DC CHECKER for Repair v2.050",
        .hacks = &[_]Hack{
            .{ .addr = 0x0C018F54, .instr = &[_]u16{0x9} },
            .{ .addr = 0x0C018F42, .instr = &[_]u16{0x9} },
        },
    },
};

var EnabledHacks: ?[]const Hack = null;

pub fn main() !void {
    defer {
        // Cleanup temprary directory, if it exists
        std.fs.cwd().deleteTree(Deecy.TmpDirPath) catch |err| std.log.err("Failed to delete temporary directory ('" ++ Deecy.TmpDirPath ++ "'): {s}", .{@errorName(err)});
    }

    var d = try Deecy.create(common.GeneralAllocator);
    defer d.destroy();
    var dc = d.dc;

    var binary_path: ?[]const u8 = null;
    var ip_bin_path: ?[]const u8 = null;

    var gdi_path: ?[]const u8 = null;

    var default_vmu = true;
    var vmu_path = std.ArrayList(u8).init(common.GeneralAllocator);
    defer vmu_path.deinit();
    try vmu_path.appendSlice("./userdata/vmu_default.bin");

    var skip_bios = false;
    var start_immediately = false;
    var force_stop = false;
    var force_render = false; // Enable to re-render every time and help capturing with RenderDoc (will mess with framebuffer emulation).

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
            force_stop = true;
        }
        if (std.mem.eql(u8, arg, "--force-render")) {
            force_render = true;
        }
    }

    dc.maple.ports[0].subperipherals[0] = .{ .VMU = try MapleModule.VMU.init(common.GeneralAllocator, vmu_path.items) };

    if (binary_path) |path| {
        try dc.set_region(.USA);

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
        start_immediately = true;
    } else if (gdi_path) |path| {
        std.log.info("Loading GDI: {s}...", .{path});

        try d.load_disk(path);

        const region = dc.gdrom.disk.?.get_region();
        std.log.info("  Detected region: {s}", .{@tagName(region)});
        if (region != .Unknown) {
            try dc.set_region(region);
        } else {
            try dc.set_region(.USA);
        }

        try d.on_game_load();

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

        for (AvailableHacks) |hack| {
            if (std.mem.count(u8, path, hack.name) > 0) {
                EnabledHacks = hack.hacks;
                break;
            }
        }

        start_immediately = true;
    } else {
        if (skip_bios) {
            try dc.set_region(.USA);
            // Boot to menu
            dc.skip_bios(true);
            // Skip IP.bin (Maybe we should bundle one to load here).
            dc.cpu.pc = 0xAC010000;
        }

        try d.launch_async(Deecy.UI.refresh_games, .{d.ui});
    }

    dc.cpu.set_trapa_callback(trapa_handler, d);

    if (!force_stop and start_immediately)
        d.start();

    while (!d.window.shouldClose()) {
        zglfw.pollEvents();
        d.poll_controllers();

        if (EnabledHacks) |hacks| {
            for (hacks) |hack| {
                var addr = hack.addr;
                for (hack.instr) |instr| {
                    dc.cpu.write16(addr, instr);
                    addr += 2;
                }
            }
        }

        d.one_frame();

        d.gctx_queue_mutex.lock();
        defer d.gctx_queue_mutex.unlock();

        // Framebuffer has been written to by the CPU.
        // Update the host texture and blit it to our render target.
        if (d.dc.gpu.dirty_framebuffer) {
            d.renderer.update_framebuffer_texture(&d.dc.gpu);
            // FIXME: Yet another framebuffer hack.
            //        Skip the framebuffer blit if we recently used the PVR for rendering.
            //        Some games (like Speed Devils) renders only at 30FPS and each frame is presented twice,
            //        however we don't actually write back the framebuffer to VRAM, meaning we'd blit garbage to the screen.
            //        Plus, even if PVR writing to the framebuffer was perfectly emulated, it would still only be at native resolution.
            if (std.time.microTimestamp() - d.last_frame_timestamp > 40_000)
                d.renderer.blit_framebuffer();
            d.dc.gpu.dirty_framebuffer = false;
        }

        const render_start = d.renderer.render_start;
        if (render_start) {
            d.renderer.render_start = false;
            try d.renderer.update(&d.dc.gpu);

            if (d.last_n_frametimes.count >= 60) {
                _ = d.last_n_frametimes.readItem();
            }
            const now = std.time.microTimestamp();
            try d.last_n_frametimes.writeItem(now - d.last_frame_timestamp);
            d.last_frame_timestamp = now;
        }

        if (force_render or render_start) {
            try d.renderer.render(&d.dc.gpu);

            if (RendererModule.ExperimentalFBWriteBack) {
                d.gctx.lookupResource(d.renderer.framebuffer_copy_buffer).?.mapAsync(
                    .{ .read = true },
                    0,
                    4 * Renderer.NativeResolution.width * Renderer.NativeResolution.height,
                    @ptrCast(&signal_fb_mapped),
                    null,
                );
                // Wait for mapping to be available. There's no synchronous way to do that AFAIK.
                // It needs to be unmapped before the next frame.
                while (!fb_mapping_available) {
                    d.gctx.device.tick();
                }
                fb_mapping_available = false;

                write_back_fb(d);
            }
        }

        if (d.dc.gpu.read_register(Holly.FB_R_CTRL, .FB_R_CTRL).enable) {
            d.renderer.draw(); //  Blit to screen
        }

        try d.draw_ui();

        if (d.gctx.present() == .swap_chain_resized) {
            d.renderer.update_blit_to_screen_vertex_buffer();
        }
    }
}

test "all tests" {
    _ = sh4;
    _ = Dreamcast;
}
