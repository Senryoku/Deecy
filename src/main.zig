const std = @import("std");
const builtin = @import("builtin");
const config = @import("config");

const termcolor = @import("termcolor");

const DreamcastModule = @import("dreamcast");
const Holly = DreamcastModule.HollyModule;
const MapleModule = DreamcastModule.Maple;
const PreciseSleep = @import("precise_sleep.zig");

const zglfw = @import("zglfw");

const Deecy = @import("deecy.zig");
const ELF = @import("elf.zig");

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

        var buffer: [64]u8 = undefined;
        const stderr = std.debug.lockStderrWriter(&buffer);
        defer std.debug.unlockStderrWriter();
        nosuspend stderr.print(termcolor.grey("\r  (...x{d})"), .{static.count}) catch return;
        return;
    }

    if (static.count > 1) {
        var buffer: [64]u8 = undefined;
        const stderr = std.debug.lockStderrWriter(&buffer);
        defer std.debug.unlockStderrWriter();
        nosuspend stderr.print("\n", .{}) catch return;
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
        .{ .scope = .elf, .level = .warn },
        .{ .scope = .dc, .level = .info },
        .{ .scope = .sh4, .level = .warn },
        .{ .scope = .mmu, .level = .info },
        .{ .scope = .sh4_jit, .level = .warn },
        .{ .scope = .arm_jit, .level = .info },
        .{ .scope = .x86_64_emitter, .level = .info },
        .{ .scope = .syscall, .level = .debug },
        .{ .scope = .aica, .level = .info },
        .{ .scope = .dsp, .level = .info },
        .{ .scope = .holly, .level = .warn },
        .{ .scope = .gdrom, .level = .warn },
        .{ .scope = .gdrom_hle, .level = .debug },
        .{ .scope = .cdi, .level = .info },
        .{ .scope = .chd, .level = .info },
        // .{ .scope = .chd_flac, .level = .debug },
        // .{ .scope = .chd_flac_frame, .level = .debug },
        // .{ .scope = .chd_flac_subframe, .level = .debug },
        // .{ .scope = .chd_flac_residual, .level = .debug },
        .{ .scope = .maple, .level = .info },
        .{ .scope = .renderer, .level = .info },
        .{ .scope = .flashrom, .level = .info },
    },
};

fn trapa_handler(app: *anyopaque) void {
    @as(*Deecy, @ptrCast(@alignCast(app))).pause();
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

pub extern "kernel32" fn timeBeginPeriod(
    uPeriod: std.os.windows.UINT,
) callconv(.winapi) std.os.windows.UINT; // MMRESULT
pub extern "kernel32" fn timeEndPeriod(
    uPeriod: std.os.windows.UINT,
) callconv(.winapi) std.os.windows.UINT; // MMRESULT

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    var allocator = gpa.allocator();

    var d = try Deecy.create(allocator);
    defer d.destroy();
    var dc = d.dc;

    var binary_path: ?[]const u8 = null;
    var ip_bin_path: ?[]const u8 = null;

    var disc_path: ?[]const u8 = null;

    var skip_bios = false;
    var start_immediately = false;
    var force_stop = false;
    var force_render = false; // Enable to re-render every time and help capturing with RenderDoc (will mess with framebuffer emulation).
    var load_state: ?u32 = null;

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();
    _ = args.skip();
    while (args.next()) |arg| {
        if (std.mem.startsWith(u8, arg, "-")) {
            if (std.mem.eql(u8, arg, "-b")) {
                binary_path = args.next() orelse {
                    std.log.err(termcolor.red("Expected path to binary file after -b."), .{});
                    return error.InvalidArguments;
                };
            } else if (std.mem.eql(u8, arg, "-g")) {
                disc_path = args.next() orelse {
                    std.log.err(termcolor.red("Expected path to disc file after -g."), .{});
                    return error.InvalidArguments;
                };
            } else if (std.mem.eql(u8, arg, "-i")) {
                ip_bin_path = args.next() orelse {
                    std.log.err(termcolor.red("Expected path to IP.bin after -i."), .{});
                    return error.InvalidArguments;
                };
            } else if (std.mem.eql(u8, arg, "--vmu")) {
                const path = args.next() orelse {
                    std.log.err(termcolor.red("Expected path to VMU after --vmu."), .{});
                    return error.InvalidArguments;
                };
                try d.load_vmu(0, 0, path);
                d.config.per_game_vmu = false;
            } else if (std.mem.eql(u8, arg, "-d")) {
                dc.cpu.debug_trace = true;
            } else if (std.mem.eql(u8, arg, "--skip-bios")) {
                skip_bios = true;
            } else if (std.mem.eql(u8, arg, "--stop")) {
                force_stop = true;
            } else if (std.mem.eql(u8, arg, "--force-render")) {
                force_render = true;
            } else if (std.mem.eql(u8, arg, "--no-realtime")) {
                d.realtime = false;
            } else if (std.mem.eql(u8, arg, "--fullscreen")) {
                if (!d.config.fullscreen) {
                    d.toggle_fullscreen();
                }
            } else if (std.mem.eql(u8, arg, "--load-state")) {
                const num_str = args.next() orelse {
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
            } else {
                std.log.warn(termcolor.yellow("Unknown argument: '{s}'"), .{arg});
            }
        } else {
            if (std.mem.endsWith(u8, arg, ".bin") or std.mem.endsWith(u8, arg, ".elf")) {
                binary_path = arg;
            } else if (std.mem.endsWith(u8, arg, ".gdi") or std.mem.endsWith(u8, arg, ".cdi") or std.mem.endsWith(u8, arg, ".chd")) {
                disc_path = arg;
            } else {
                std.log.warn(termcolor.yellow("Unsupported file format: '{s}'"), .{arg});
            }
        }
    }

    if (binary_path) |path| {
        // FIXME: I'd rather be using LLE syscalls here,
        //        but at least the ROM font one requires some initialization
        //        and won't work if the boot ROM is skipped.
        try dc.skip_bios(true);

        var entry_point: u32 = 0xAC010000;

        if (std.mem.endsWith(u8, path, ".elf")) {
            var elf_file = try std.fs.cwd().openFile(path, .{});
            defer elf_file.close();
            const buffer = try allocator.alloc(u8, 8192);
            defer allocator.free(buffer);
            var file_reader = elf_file.reader(buffer);
            var elf = try ELF.init(allocator, &file_reader);
            defer elf.deinit();

            entry_point = @intCast(elf.program_entry_offset);
            for (elf.program_headers) |ph| {
                if (ph.p_type == .Load) {
                    try elf_file.seekTo(ph.p_offset);
                    _ = try elf_file.readAll(dc.ram[(ph.p_vaddr & 0x1FFF_FFFF) - 0x0C00_0000 ..]);
                } else {
                    if (std.enums.tagName(ELF.SegmentType, ph.p_type)) |tag| {
                        std.log.scoped(.elf).warn(termcolor.yellow("Program header type {s} not supported"), .{tag});
                    } else {
                        std.log.scoped(.elf).warn(termcolor.yellow("Program header type {d} not supported"), .{@intFromEnum(ph.p_type)});
                    }
                }
            }
        } else {
            var bin_file = try std.fs.cwd().openFile(path, .{});
            defer bin_file.close();
            _ = try bin_file.readAll(dc.ram[0x10000..]);
        }

        if (ip_bin_path) |ipb_path| {
            var ip_bin_file = try std.fs.cwd().openFile(ipb_path, .{});
            defer ip_bin_file.close();
            _ = try ip_bin_file.readAll(dc.ram[0x8000..]);
        } else {
            // Skip IP.bin
            dc.cpu.pc = entry_point;
        }
        start_immediately = true;
        d.display_ui = false;
        d.ui.binary_loaded = true;
    } else if (disc_path) |path| {
        std.log.info("Loading Disc: {s}...", .{path});

        try d.load_disc(path);
        if (d.config.region == .Auto)
            try d.dc.set_region(d.dc.gdrom.disc.?.get_region());
        try d.on_game_load();

        if (skip_bios) {
            try dc.skip_bios(true);

            // Load 1STREAD.BIN (Actual name might change)
            const header_size: u32 = dc.gdrom.disc.?.get_first_data_track().?.header_size();
            const first_read_name = dc.gdrom.disc.?.get_first_data_track().?.data[0x60 + header_size .. 0x70 + header_size];
            const name_end = std.mem.indexOfScalar(u8, first_read_name, 0x20) orelse first_read_name.len;
            var first_read: []u8 = try allocator.alloc(u8, name_end + 2);
            defer allocator.free(first_read);
            @memcpy(first_read[0..name_end], first_read_name[0..name_end]);
            @memcpy(first_read[name_end .. name_end + 2], ";1");
            _ = try dc.gdrom.disc.?.load_file(first_read, dc.ram[0x00010000..]);
        }

        for (AvailableHacks) |hack| {
            if (std.mem.count(u8, path, hack.name) > 0) {
                EnabledHacks = hack.hacks;
                std.log.info("  Enabled hacks for '{s}'", .{hack.name});
                break;
            }
        }

        if (load_state) |state| {
            try d.load_state(state);
        }

        start_immediately = true;
        d.display_ui = false;
    } else {
        if (skip_bios) {
            // Boot to menu
            try dc.skip_bios(true);
            // Skip IP.bin (Maybe we should bundle one to load here).
            dc.cpu.pc = 0xAC010000;
        }

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

    var precise_sleep: PreciseSleep = .init();
    defer precise_sleep.deinit();
    var then = zglfw.getTime();
    while (!d.window.shouldClose()) {
        zglfw.pollEvents();

        const now = zglfw.getTime();
        d.update(@floatCast(now - then));
        then = now;

        if (EnabledHacks) |hacks| {
            for (hacks) |hack| {
                var addr = hack.addr;
                for (hack.instr) |instr| {
                    dc.cpu.write_physical(u16, addr, instr);
                    addr += 2;
                }
            }
        }

        // Framebuffer has been written to by the CPU.
        // Update the host texture and blit it to our render target.
        // FIXME: Hackishly forced on for .bin files
        if (binary_path != null or d.dc.gpu.dirty_framebuffer) {
            d.dc.gpu.dirty_framebuffer = false;
            d.renderer.update_framebuffer_texture(&d.dc.gpu);
            // FIXME: Yet another framebuffer hack.
            //        Skip the framebuffer blit if we recently used the PVR for rendering.
            //        Some games (like Speed Devils) renders only at 30FPS and each frame is presented twice,
            //        however we don't actually write back the framebuffer to VRAM, meaning we'd blit garbage to the screen.
            //        Plus, even if PVR writing to the framebuffer was perfectly emulated, it would still only be at native resolution.
            if (std.time.microTimestamp() - d.renderer.last_frame_timestamp > 40_000)
                d.renderer.blit_framebuffer();
        }

        const render_start = d.renderer.render_request;
        if (render_start) {
            d.gctx_queue_mutex.lock();
            defer d.gctx_queue_mutex.unlock();

            try d.renderer.update(&d.dc.gpu);
            try d.renderer.render(&d.dc.gpu, false);
            d.renderer.render_request = false;
        } else if (force_render) {
            d.gctx_queue_mutex.lock();
            defer d.gctx_queue_mutex.unlock();
            // Debug aid (see force_render). NOTE: This will break if the game renders to textures.
            try d.renderer.render(&d.dc.gpu, false);
        }

        if (d.dc.gpu.read_register(Holly.FB_R_CTRL, .FB_R_CTRL).enable) {
            d.renderer.draw(); //  Blit to screen
        }

        try d.draw_ui();
        d.submit_ui();

        const resized = resized: {
            d.gctx_queue_mutex.lock();
            defer d.gctx_queue_mutex.unlock();
            break :resized d.gctx.present() == .swap_chain_resized;
        };
        if (resized)
            d.on_resize();

        if (d.config.frame_limiter != .Off) {
            const ns_per_frame: u64 = switch (d.config.frame_limiter) {
                .Auto => if (d.dc.gpu._get_register(DreamcastModule.HollyModule.SPG_CONTROL, .SPG_CONTROL).PAL == 1) 20_000_000 else 16_666_666,
                .@"120Hz" => 8_333_333,
                .@"100Hz" => 10_000_000,
                .@"60Hz" => 16_666_666,
                .@"50Hz" => 20_000_000,
                .Off => unreachable,
            };
            precise_sleep.wait_for_interval(ns_per_frame);
        }
    }
}
