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

const assets_dir = "assets/";

pub const std_options = struct {
    pub const log_level = .info;

    pub const log_scope_levels = &[_]std.log.ScopeLevel{
        .{ .scope = .sh4, .level = .info },
        .{ .scope = .sh4_jit, .level = .info },
        .{ .scope = .arm_jit, .level = .info },
        .{ .scope = .x86_64_emitter, .level = .info },
        .{ .scope = .syscall_log, .level = .warn },
        .{ .scope = .aica, .level = .info },
        .{ .scope = .holly, .level = .info },
        .{ .scope = .gdrom, .level = .warn },
        .{ .scope = .maple, .level = .info },
        .{ .scope = .renderer, .level = .info },
    };
};

var running = true;
var draw_debug_ui = true;

fn trapa_handler() void {
    running = false;
}

fn glfw_key_callback(
    window: *zglfw.Window,
    key: zglfw.Key,
    scancode: i32,
    action: zglfw.Action,
    mods: zglfw.Mods,
) callconv(.C) void {
    _ = window;
    _ = scancode;
    _ = mods;
    if (key == .escape and action == .press) {
        draw_debug_ui = !draw_debug_ui;
    }
}

pub fn main() !void {
    std.log.info("\r  == Katana ==                             ", .{});

    var dc = try Dreamcast.create(common.GeneralAllocator);
    defer {
        dc.deinit();
        common.GeneralAllocator.destroy(dc);
    }

    var binary_path: ?[]const u8 = null;
    var ip_bin_path: ?[]const u8 = null;

    var gdi_path: ?[]const u8 = null;

    var skip_bios = false;

    var enable_jit = true;

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
            running = false;
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
        if (skip_bios)
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

    try zglfw.init();
    defer zglfw.terminate();

    const window = zglfw.Window.create(640 * 2, 480 * 2, "Katana", null) catch {
        std.log.err("Failed to create window.", .{});
        return;
    };
    defer window.destroy();

    _ = window.setKeyCallback(glfw_key_callback);

    const gctx = try zgpu.GraphicsContext.create(common.GeneralAllocator, window, .{
        .present_mode = .mailbox,
        .required_features = &[_]zgpu.wgpu.FeatureName{.bgra8_unorm_storage},
    });
    defer gctx.destroy(common.GeneralAllocator);

    const scale_factor = scale_factor: {
        const scale = window.getContentScale();
        break :scale_factor @max(scale[0], scale[1]);
    };

    var renderer = try Renderer.init(common.GeneralAllocator, gctx);
    defer renderer.deinit();

    zgui.init(common.GeneralAllocator);
    defer zgui.deinit();

    _ = zgui.io.addFontFromFile(
        assets_dir ++ "fonts/Hack-Regular.ttf",
        std.math.floor(16.0 * scale_factor),
    );

    zgui.backend.init(
        window,
        gctx.device,
        @intFromEnum(zgpu.GraphicsContext.swapchain_format),
        @intFromEnum(zgpu.wgpu.TextureFormat.undef),
    );
    defer zgui.backend.deinit();

    zgui.getStyle().scaleAllSizes(scale_factor);

    var breakpoints = std.ArrayList(u32).init(common.GeneralAllocator);
    defer breakpoints.deinit();
    dc.cpu.on_trapa = trapa_handler;

    // Debug UI State
    const vram_width = 640;
    const vram_height = 400;
    const texture = gctx.createTexture(.{
        .usage = .{ .texture_binding = true, .copy_dst = true },
        .size = .{
            .width = vram_width,
            .height = vram_height,
            .depth_or_array_layers = 1,
        },
        .format = zgpu.imageInfoToTextureFormat(
            4,
            1,
            false,
        ),
        .mip_level_count = 1,
    });
    const texture_view = gctx.createTextureView(texture, .{});

    const pixels = try common.GeneralAllocator.alloc(u8, (vram_width * vram_height) * 4);
    defer common.GeneralAllocator.free(pixels);

    var renderer_texture_views: [8]zgpu.TextureViewHandle = undefined;
    for (0..renderer_texture_views.len) |i|
        renderer_texture_views[i] = gctx.createTextureView(renderer.texture_arrays[i], .{ .dimension = .tvdim_2d, .base_array_layer = 0, .array_layer_count = 1 });

    var show_disabled_channels = false;
    //////////

    var last_frame_timestamp = std.time.microTimestamp();
    var last_n_frametimes = std.fifo.LinearFifo(i64, .Dynamic).init(common.GeneralAllocator);

    var blit_framebuffer_from_vram = true;

    while (!window.shouldClose()) {
        zglfw.pollEvents();

        zgui.backend.newFrame(
            gctx.swapchain_descriptor.width,
            gctx.swapchain_descriptor.height,
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

        if (draw_debug_ui) {
            if (zgui.begin("CPU State", .{})) {
                _ = zgui.checkbox("JIT", .{ .v = &enable_jit });
                zgui.text("PC: 0x{X:0>8} - SPC: 0x{X:0>8}", .{ dc.cpu.pc, dc.cpu.spc });
                zgui.text("PR: 0x{X:0>8}", .{dc.cpu.pr});
                zgui.text("SR: T={any}, S={any}, IMASK={d}", .{ dc.cpu.sr.t, dc.cpu.sr.s, dc.cpu.sr.imask });
                zgui.text("GBR: 0x{X:0>8}", .{dc.cpu.gbr});
                zgui.text("VBR: 0x{X:0>8}", .{dc.cpu.vbr});
                zgui.beginGroup();
                for (0..8) |i| {
                    zgui.text("R{d: <2}: 0x{X:0>8}", .{ i, dc.cpu.R(@truncate(i)).* });
                }
                zgui.endGroup();
                zgui.sameLine(.{});
                zgui.beginGroup();
                for (8..16) |i| {
                    zgui.text("R{d: <2}: 0x{X:0>8}", .{ i, dc.cpu.R(@truncate(i)).* });
                }
                zgui.endGroup();

                const range = 16; // In bytes.
                const pc = dc.cpu.pc & 0x1FFFFFFF;
                //              In RAM                                                                           In BootROM
                var addr = (if (pc >= 0x0C000000) std.math.clamp(pc, 0x0C000000 + range / 2, 0x0D000000 - range) else std.math.clamp(pc, 0x00000000 + range / 2, 0x02000000 - range)) - range / 2;
                const end_addr = addr + range;
                while (addr < end_addr) {
                    //zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == dc.cpu.pc) ">" else " ", sh4.Opcodes[sh4.JumpTable[dc.read16(@intCast(addr))]].name });
                    const disassembly = try sh4_disassembly.disassemble(.{ .value = dc.cpu.read16(@intCast(addr)) }, common.GeneralAllocator);
                    zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == pc) ">" else " ", disassembly });
                    addr += 2;
                }

                if (zgui.button(if (running) "Pause" else "Run", .{ .w = 200.0 })) {
                    running = !running;
                }

                if (zgui.button("Step", .{ .w = 200.0 })) {
                    running = false;
                    _ = try dc.tick(1);
                }
                if (zgui.button("Skip", .{ .w = 200.0 })) {
                    dc.cpu.pc += 2;
                }

                if (comptime builtin.mode == .Debug or builtin.mode == .ReleaseSafe) {
                    _ = zgui.checkbox("Debug trace", .{ .v = &dc.cpu.debug_trace });
                } else {
                    zgui.textColored(.{ 0.5, 0.5, 0.5, 1 }, "Debug trace is not available in ReleaseFast builds!", .{});
                }

                for (0..breakpoints.items.len) |i| {
                    zgui.text("Breakpoint {d}: 0x{X:0>8}", .{ i, breakpoints.items[i] });
                    zgui.sameLine(.{});
                    if (zgui.button("Remove", .{})) {
                        _ = breakpoints.orderedRemove(i);
                        break;
                    }
                }
                const static = struct {
                    var bp_addr: i32 = 0;
                };
                _ = zgui.inputInt("##breakpoint", .{ .v = &static.bp_addr, .flags = .{ .chars_hexadecimal = true } });
                zgui.sameLine(.{});
                if (zgui.button("Add Breakpoint", .{ .w = 200.0 })) {
                    try breakpoints.append(@as(u32, @intCast(static.bp_addr & 0x1FFFFFFF)));
                }

                const timers = .{
                    .{ .counter = P4Register.TCNT0, .control = P4Register.TCR0, .constant = P4Register.TCOR0 },
                    .{ .counter = P4Register.TCNT1, .control = P4Register.TCR1, .constant = P4Register.TCOR1 },
                    .{ .counter = P4Register.TCNT2, .control = P4Register.TCR2, .constant = P4Register.TCOR2 },
                };
                const TSTR = dc.cpu.read_p4_register(u32, .TSTR);
                inline for (0..3) |i| {
                    zgui.beginGroup();
                    const control = dc.cpu.read_p4_register(sh4.P4.TCR, timers[i].control);
                    zgui.text("Timer {d:0>1}: Enabled: {any}", .{ i, ((TSTR >> i) & 1) == 1 });
                    zgui.text("  0x{X:0>8}/0x{X:0>8}", .{ dc.cpu.read_p4_register(u32, timers[i].counter), dc.cpu.read_p4_register(u32, timers[i].constant) });
                    zgui.text("  TPSC {X:0>1} CKEG {X:0>1} UNIE {X:0>1} ICPE {X:0>1} UNF {X:0>1}", .{ control.tpsc, control.ckeg, control.unie, control.icpe, control.unf });
                    zgui.endGroup();
                }
            }
            zgui.end();

            if (zgui.begin("FPU", .{})) {
                zgui.text("FPUL: {d: >8.4} | {d: >8.4} | {d: >8.4}", .{ @as(f32, @bitCast(dc.cpu.fpul)), dc.cpu.fpul, @as(i32, @bitCast(dc.cpu.fpul)) });

                zgui.spacing();

                zgui.beginGroup();
                for (0..8) |i| {
                    zgui.text("FR{d: <2}: {d: >12.4}  ", .{ i, dc.cpu.FR(@truncate(i)).* });
                }
                zgui.endGroup();
                zgui.sameLine(.{});
                zgui.beginGroup();
                for (8..16) |i| {
                    zgui.text("FR{d: <2}: {d: >12.4}", .{ i, dc.cpu.FR(@truncate(i)).* });
                }
                zgui.endGroup();

                zgui.spacing();

                zgui.beginGroup();
                for (0..8) |i| {
                    zgui.text("XF{d: <2}: {d: >12.4}  ", .{ i, dc.cpu.XF(@truncate(i)).* });
                }
                zgui.endGroup();
                zgui.sameLine(.{});
                zgui.beginGroup();
                for (8..16) |i| {
                    zgui.text("XF{d: <2}: {d: >12.4}", .{ i, dc.cpu.XF(@truncate(i)).* });
                }
                zgui.endGroup();
            }
            zgui.end();

            if (zgui.begin("AICA - ARM", .{})) {
                _ = zgui.checkbox("ARM JIT", .{ .v = &dc.aica.enable_arm_jit });
                zgui.text("State: {s}", .{@tagName(dc.aica.arm7.cpsr.m)});
                zgui.text("PC: 0x{X:0>8}", .{dc.aica.arm7.pc()});
                zgui.beginGroup();
                for (0..8) |i| {
                    zgui.text("R{d: <2}: 0x{X:0>8}", .{ i, dc.aica.arm7.r[i] });
                }
                zgui.endGroup();
                zgui.sameLine(.{});
                zgui.beginGroup();
                for (8..16) |i| {
                    zgui.text("R{d: <2}: 0x{X:0>8}", .{ i, dc.aica.arm7.r[i] });
                }
                zgui.endGroup();

                const range = 32; // In bytes.
                const pc = 0x00800000 + dc.aica.arm7.pc() - 4;
                var addr = std.math.clamp(pc, 0x00800000 + range / 2, 0x00A00000 - range);
                const end_addr = addr + range;
                while (addr < end_addr) {
                    const disassembly = arm7.ARM7.disassemble(dc.aica.read_mem(u32, addr));
                    zgui.text("[{X: >6}] {s} {s}", .{ addr - 0x00800000, if (addr == pc) ">" else " ", disassembly });
                    addr += 4;
                }
            }
            zgui.end();

            if (zgui.begin("AICA", .{})) {
                _ = zgui.checkbox("Show disabled channels", .{ .v = &show_disabled_channels });
                inline for (0..64) |i| {
                    const channel = dc.aica.get_channel(@intCast(i));
                    if (show_disabled_channels or channel.play_control.key_on_bit) {
                        if (zgui.collapsingHeader("Channel " ++ std.fmt.comptimePrint("{d}", .{i}), .{ .default_open = true })) {
                            zgui.text("KeyOn: {any} - Format: {s} - Loop: {any} - Start Address: {X:0>4}", .{
                                channel.play_control.key_on_bit,
                                @tagName(channel.play_control.sample_format),
                                channel.play_control.sample_loop,
                                @as(u16, channel.play_control.start_address) << 7,
                            });
                            // TODO: Display the samples!
                            //if (zgui.plot.beginPlot("Line Plot", .{ .h = -1.0 })) {
                            //    zgui.plot.setupAxis(.x1, .{ .label = "xaxis" });
                            //    zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = 5 });
                            //    zgui.plot.setupLegend(.{ .south = true, .west = true }, .{});
                            //    zgui.plot.setupFinish();
                            //    zgui.plot.plotLineValues("y data", i32, .{ .v = &.{ 0, 1, 0, 1, 0, 1 } });
                            //    zgui.plot.plotLine("xy data", f32, .{
                            //        .xv = &.{ 0.1, 0.2, 0.5, 2.5 },
                            //        .yv = &.{ 0.1, 0.3, 0.5, 0.9 },
                            //    });
                            //    zgui.plot.endPlot();
                            //}
                        }
                    }
                }
            }
            zgui.end();

            if (zgui.begin("Memory", .{})) {
                const static = struct {
                    var start_addr: i32 = 0;
                    var edit_addr: i32 = 0;
                };
                if (zgui.inputInt("Start", .{ .v = &static.edit_addr, .step = 8, .flags = .{ .chars_hexadecimal = true } })) {
                    static.start_addr = static.edit_addr;
                }
                var addr = @max(0, @as(u32, @intCast(static.start_addr & 0x1FFFFFFF)));
                const end_addr = addr + 128;
                zgui.textColored(.{ 0.5, 0.5, 0.5, 1 }, "           00 01 02 03 04 05 06 07", .{});
                while (addr < end_addr) {
                    zgui.text("[{X:0>8}] {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}", .{
                        addr,
                        dc.cpu.read8(@intCast(addr)),
                        dc.cpu.read8(@intCast(addr + 1)),
                        dc.cpu.read8(@intCast(addr + 2)),
                        dc.cpu.read8(@intCast(addr + 3)),
                        dc.cpu.read8(@intCast(addr + 4)),
                        dc.cpu.read8(@intCast(addr + 5)),
                        dc.cpu.read8(@intCast(addr + 6)),
                        dc.cpu.read8(@intCast(addr + 7)),
                    });
                    addr += 8;
                }

                zgui.separator();

                for (0..2) |sq| {
                    for (0..8) |i| {
                        zgui.text("[SQ{d:0>1}] {X:0>8}", .{
                            sq,
                            dc.cpu.store_queues[sq][i],
                        });
                    }
                }
            }
            zgui.end();

            if (zgui.begin("Holly", .{})) {
                const ISP_BACKGND_D = dc.gpu._get_register(u32, .ISP_BACKGND_D).*;
                const ISP_BACKGND_T = dc.gpu._get_register(u32, .ISP_BACKGND_T).*;
                zgui.text("ISP_BACKGND_D: {d: >8.2} / {d: >8.2}", .{ ISP_BACKGND_D, @as(f32, @bitCast(ISP_BACKGND_D)) });
                zgui.text("ISP_BACKGND_T: {X:0>8}", .{ISP_BACKGND_T});

                const FB_C_SOF = dc.gpu._get_register(u32, .FB_C_SOF).*;
                const FB_W_CTRL = dc.gpu._get_register(Holly.FB_W_CTRL, .FB_W_CTRL).*;
                const FB_W_SOF1 = dc.gpu._get_register(u32, .FB_W_SOF1).*;
                const FB_W_SOF2 = dc.gpu._get_register(u32, .FB_W_SOF2).*;
                const FB_R_CTRL = dc.gpu._get_register(Holly.FB_R_CTRL, .FB_R_CTRL).*;
                const FB_R_SOF1 = dc.gpu._get_register(u32, .FB_R_SOF1).*;
                const FB_R_SOF2 = dc.gpu._get_register(u32, .FB_R_SOF2).*;
                zgui.text("FB_C_SOF: 0x{X:0>8}", .{FB_C_SOF});
                zgui.text("FB_W_CTRL: {any}", .{FB_W_CTRL});
                zgui.text("FB_W_SOF1: 0x{X:0>8}", .{FB_W_SOF1});
                zgui.text("FB_W_SOF2: 0x{X:0>8}", .{FB_W_SOF2});
                zgui.text("FB_R_CTRL: {any}", .{FB_R_CTRL});
                zgui.text("FB_R_SOF1: 0x{X:0>8}", .{FB_R_SOF1});
                zgui.text("FB_R_SOF2: 0x{X:0>8}", .{FB_R_SOF2});

                if (zgui.collapsingHeader("VRAM", .{})) {
                    const static = struct {
                        var start_addr: i32 = 0x04200000;
                        var format: i32 = 0x6;
                        var twiddled: bool = false;
                        var width: i32 = vram_width;
                    };
                    _ = zgui.inputInt("Start", .{ .v = &static.start_addr, .step = 0x8000, .flags = .{ .chars_hexadecimal = true } });
                    _ = zgui.inputInt("Format", .{ .v = &static.format, .step = 1, .flags = .{ .chars_hexadecimal = true } });
                    _ = zgui.inputInt("Width", .{ .v = &static.width, .step = 8 });
                    _ = zgui.checkbox("Twiddled", .{ .v = &static.twiddled });
                    static.format = std.math.clamp(static.format, 0, 0x6);
                    static.width = std.math.clamp(static.width, 8, vram_width);
                    const width: u32 = @intCast(static.width);
                    const bytes_per_pixels: u32 = if (static.format & 0b111 == 0x6) 4 else 2;
                    const start: u32 = @intCast(std.math.clamp(static.start_addr, 0x04000000, 0x04800000));
                    const end = std.math.clamp(start + bytes_per_pixels * width * vram_height, 0x04000000, 0x04800000);
                    zgui.text("(VRAM addresses: {X:0>8} - {X:0>8})", .{ start, end });
                    var i: u32 = 0;
                    var current_addr = start;
                    while (current_addr < end) {
                        const pixel_idx: u32 = if (static.twiddled) RendererModule.to_twiddled_index(@intCast(i), width) else i;
                        const addr: u32 = start + bytes_per_pixels * pixel_idx;
                        if (addr >= 0x04800000 - bytes_per_pixels) break;
                        switch (static.format & 0b111) {
                            0x0, 0x3 => { // 0555 KRGB 16 bits
                                const color: Holly.Color16 = .{ .value = dc.cpu.read16(addr) };
                                pixels[4 * i + 0] = @as(u8, @intCast(color.arbg1555.r)) << 3;
                                pixels[4 * i + 1] = @as(u8, @intCast(color.arbg1555.g)) << 3;
                                pixels[4 * i + 2] = @as(u8, @intCast(color.arbg1555.b)) << 3;
                                pixels[4 * i + 3] = 255; // FIXME: Not really.
                                i += 1;
                            },
                            0x1 => { // 565 RGB 16 bit
                                const color: Holly.Color16 = .{ .value = dc.cpu.read16(addr) };
                                pixels[4 * i + 0] = @as(u8, @intCast(color.rgb565.r)) << 3;
                                pixels[4 * i + 1] = @as(u8, @intCast(color.rgb565.g)) << 2;
                                pixels[4 * i + 2] = @as(u8, @intCast(color.rgb565.b)) << 3;
                                pixels[4 * i + 3] = 255;
                                i += 1;
                            },
                            // ARGB 32-Bits
                            0x6 => {
                                pixels[4 * i + 0] = dc.cpu.read8(@intCast(addr + 3));
                                pixels[4 * i + 1] = dc.cpu.read8(@intCast(addr + 2));
                                pixels[4 * i + 2] = dc.cpu.read8(@intCast(addr + 1));
                                pixels[4 * i + 3] = dc.cpu.read8(@intCast(addr + 0));
                                i += 1;
                            },
                            else => {
                                current_addr = end;
                                zgui.text("Unsupported packed format: 0x{X:0>1}", .{static.format & 0b111});
                            },
                        }
                        current_addr += bytes_per_pixels;
                    }

                    gctx.queue.writeTexture(
                        .{ .texture = gctx.lookupResource(texture).? },
                        .{
                            .bytes_per_row = 4 * vram_width,
                            .rows_per_image = vram_height,
                        },
                        .{ .width = width, .height = vram_height },
                        u8,
                        pixels,
                    );
                    const tex_id = gctx.lookupResource(texture_view).?;

                    zgui.image(tex_id, .{ .w = vram_width, .h = vram_height });
                }
            }
            zgui.end();

            if (zgui.begin("Renderer", .{})) {
                zgui.text("Min Depth: {d: >4.2}", .{renderer.min_depth});
                zgui.text("Max Depth: {d: >4.2}", .{renderer.max_depth});
                if (zgui.collapsingHeader("Textures", .{})) {
                    const static = struct {
                        var index: i32 = 0;
                        var scale: f32 = 512.0 / 8.0;
                        var size: i32 = 0;
                    };
                    if (zgui.inputInt("Size", .{ .v = &static.size, .step = 1 })) {
                        static.size = std.math.clamp(static.size, 0, @as(i32, @intCast(renderer_texture_views.len - 1)));
                        static.scale = @as(f32, 512) / @as(f32, @floatFromInt((@as(u32, 8) << @intCast(static.size))));
                        static.index = std.math.clamp(static.index, 0, Renderer.MaxTextures[@intCast(static.size)] - 1);
                        gctx.releaseResource(renderer_texture_views[@intCast(static.size)]);
                        renderer_texture_views[@intCast(static.size)] = gctx.createTextureView(renderer.texture_arrays[@intCast(static.size)], .{ .dimension = .tvdim_2d, .base_array_layer = @as(u32, @intCast(static.index)), .array_layer_count = 1 });
                    }
                    zgui.sameLine(.{});
                    zgui.text("{d: >3}x{d: >3}", .{ @as(u32, 8) << @intCast(static.size), @as(u32, 8) << @intCast(static.size) });
                    if (zgui.inputInt("Index", .{ .v = &static.index, .step = 1 })) {
                        static.index = std.math.clamp(static.index, 0, Renderer.MaxTextures[@intCast(static.size)] - 1);
                        gctx.releaseResource(renderer_texture_views[@intCast(static.size)]);
                        renderer_texture_views[@intCast(static.size)] = gctx.createTextureView(renderer.texture_arrays[@intCast(static.size)], .{ .dimension = .tvdim_2d, .base_array_layer = @as(u32, @intCast(static.index)), .array_layer_count = 1 });
                    }
                    if (zgui.dragFloat("Scale", .{ .v = &static.scale, .min = 1.0, .max = 8.0, .speed = 0.1 })) {
                        static.scale = std.math.clamp(static.scale, 1.0, 8.0);
                    }
                    const tex_id = gctx.lookupResource(renderer_texture_views[@intCast(static.size)]).?;
                    zgui.image(tex_id, .{ .w = static.scale * @as(f32, @floatFromInt(@as(u32, 8) << @intCast(static.size))), .h = static.scale * @as(f32, @floatFromInt(@as(u32, 8) << @intCast(static.size))) });
                    zgui.text("Parameter Control Word: {any}", .{renderer.texture_metadata[@intCast(static.size)][@intCast(static.index)].control_word});
                    zgui.text("TSP Instruction: {any}", .{renderer.texture_metadata[@intCast(static.size)][@intCast(static.index)].tsp_instruction});
                }
                if (zgui.collapsingHeader("Framebuffer Texture", .{})) {
                    const fb_tex_id = gctx.lookupResource(renderer.framebuffer_texture_view).?;
                    zgui.image(fb_tex_id, .{ .w = 640, .h = 480 });
                }
                if (zgui.collapsingHeader("Resized Framebuffer Texture", .{})) {
                    const fb_tex_id = gctx.lookupResource(renderer.resized_framebuffer_texture_view).?;
                    zgui.image(fb_tex_id, .{ .w = @floatFromInt(gctx.swapchain_descriptor.width), .h = @floatFromInt(gctx.swapchain_descriptor.height) });
                }
            }
            zgui.end();
        }

        if (dc.maple.ports[0].main) |*main_controller| {
            switch (main_controller.*) {
                .Controller => |*c| {
                    // FIXME: This shouldn't be here.
                    const keybinds: [9]struct { zglfw.Key, MapleModule.ControllerButtons } = .{
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
                    var any_keyboard_key_pressed = false;
                    for (keybinds) |keybind| {
                        const key_status = window.getKey(keybind[0]);
                        if (key_status == .press) {
                            any_keyboard_key_pressed = true;
                            c.press_buttons(keybind[1]);
                        } else if (key_status == .release) {
                            c.release_buttons(keybind[1]);
                        }
                    }
                    if (!any_keyboard_key_pressed) {
                        for (0..zglfw.Joystick.maximum_supported) |jid| {
                            if (zglfw.Joystick.get(@as(zglfw.Joystick.Id, @intCast(jid)))) |joystick| {
                                if (joystick.asGamepad()) |gamepad| {
                                    const gamepad_state = gamepad.getState();
                                    const gamepad_binds: [9]struct { zglfw.Gamepad.Button, MapleModule.ControllerButtons } = .{
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

                                    break; // FIXME: We're using the first one available and that's it for now.
                                }
                            }
                        }
                    }
                },
                else => {},
            }
        }

        if (running) {
            const start = try std.time.Instant.now();
            // FIXME: We break on render start for synchronization, this is not how we'll want to do it in the end.
            while (running and (try std.time.Instant.now()).since(start) < 16 * std.time.ns_per_ms and !dc.gpu.render_start) {
                if (!enable_jit) {
                    const max_instructions: u8 = if (breakpoints.items.len == 0) 16 else 1;

                    _ = try dc.tick(max_instructions);

                    // Doesn't make sense to try to have breakpoints if the interpreter can execute more than one instruction at a time.
                    if (max_instructions == 1) {
                        const breakpoint = for (breakpoints.items, 0..) |addr, index| {
                            if (addr & 0x1FFFFFFF == dc.cpu.pc & 0x1FFFFFFF) break index;
                        } else null;
                        if (breakpoint != null) {
                            running = false;
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

        const swapchain_texv = gctx.swapchain.getCurrentTextureView();
        defer swapchain_texv.release();

        if (blit_framebuffer_from_vram) {
            renderer.update_framebuffer(&dc.gpu);
            renderer.blit_framebuffer();
        }

        if (dc.gpu.render_start) { // FIXME: Find a better way to start a render.
            // FIXME: I don't how to handle this correctly, but copying the framebuffer from VRAM
            // is very expensive and useless outside of splash screen/homebrews.
            // I'm disabling it as soon as we start rendering normally.
            blit_framebuffer_from_vram = false;
            renderer.read_framebuffer_enabled = false;

            dc.gpu.render_start = false;
            try renderer.update(&dc.gpu);

            if (last_n_frametimes.count >= 10) {
                _ = last_n_frametimes.readItem();
            }
            const now = std.time.microTimestamp();
            try last_n_frametimes.writeItem(now - last_frame_timestamp);
            last_frame_timestamp = now;

            try renderer.render();
        }
        renderer.draw(); //  Blit to screen

        const commands = commands: {
            const encoder = gctx.device.createCommandEncoder(null);
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

        gctx.submit(&.{commands});

        if (gctx.present() == .swap_chain_resized) {
            renderer.on_resize();
        }
    }
}

test "all tests" {
    _ = sh4;
    _ = Dreamcast;
}
