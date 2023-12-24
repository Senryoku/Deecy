const std = @import("std");

const common = @import("./common.zig");
const termcolor = @import("./termcolor.zig");
const sh4 = @import("./sh4.zig");
const MemoryRegisters = @import("./MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;
const Dreamcast = @import("./dreamcast.zig").Dreamcast;
const GDI = @import("./GDI.zig").GDI;
const Holly = @import("./holly.zig");
const MapleModule = @import("./maple.zig");

const zgui = @import("zgui");
const zgpu = @import("zgpu");
const zglfw = @import("zglfw");

const RendererModule = @import("renderer.zig");
const Renderer = RendererModule.Renderer;

const assets_dir = "assets/";

// FIXME
const syscall = @import("syscall.zig");

pub const std_options = struct {
    pub const log_level = .info;

    pub const log_scope_levels = &[_]std.log.ScopeLevel{
        .{ .scope = .sh4, .level = .info },
        .{ .scope = .holy, .level = .info },
        .{ .scope = .gdrom, .level = .info },
        .{ .scope = .renderer, .level = .info },
    };
};

var running = false;

fn trapa_handler() void {
    running = false;
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

    var args = try std.process.argsWithAllocator(common.GeneralAllocator);
    defer args.deinit();
    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "-b")) {
            const v = args.next();
            if (v == null) {
                std.log.err(termcolor.red("Expected path to binary file after -b."), .{});
                return;
            }
            binary_path = v.?;
        }
        if (std.mem.eql(u8, arg, "-g")) {
            const v = args.next();
            if (v == null) {
                std.log.err(termcolor.red("Expected path to GDI file after -g."), .{});
                return;
            }
            gdi_path = v.?;
        }
        if (std.mem.eql(u8, arg, "-i")) {
            const v = args.next();
            if (v == null) {
                std.log.err(termcolor.red("Expected path to IP.bin after -i."), .{});
                return;
            }
            ip_bin_path = v.?;
        }
        if (std.mem.eql(u8, arg, "-d")) {
            dc.cpu.debug_trace = true;
        }
    }

    if (binary_path != null) {
        dc.skip_bios();

        var bin_file = try std.fs.cwd().openFile(binary_path.?, .{});
        defer bin_file.close();
        _ = try bin_file.readAll(dc.ram[0x10000..]);

        if (ip_bin_path != null) {
            var ip_bin_file = try std.fs.cwd().openFile(ip_bin_path.?, .{});
            defer ip_bin_file.close();
            _ = try ip_bin_file.readAll(dc.ram[0x8000..]);
        } else {
            // Skip IP.bin
            dc.cpu.pc = 0xAC010000;
        }
    } else if (gdi_path != null) {
        dc.skip_bios();

        try dc.gdrom.disk.init(gdi_path.?, common.GeneralAllocator);

        // Load IP.bin from disk (16 first sectors of the last track)
        // FIXME: Here we assume the last track is the 3rd.
        _ = dc.gdrom.disk.load_sectors(45150, 16 * 2048, dc.ram[0x00008000..]);

        syscall.FirstReadBINSectorSize = (try dc.gdrom.disk.load_file("1ST_READ.BIN;1", dc.ram[0x00010000..]) + 2047) / 2048;
    } else {
        // Boot to menu
        dc.cpu.skip_bios();
        // Skip IP.bin (Maybe we should bundle one to load here).
        dc.cpu.pc = 0xAC010000;
    }

    try zglfw.init();
    defer zglfw.terminate();

    const window = zglfw.Window.create(640 * 2, 480 * 2, "Katana", null) catch {
        std.log.err("Failed to create window.", .{});
        return;
    };
    defer window.destroy();

    const gctx = try zgpu.GraphicsContext.create(common.GeneralAllocator, window, .{
        .present_mode = .mailbox,
    });
    defer gctx.destroy(common.GeneralAllocator);

    const scale_factor = scale_factor: {
        const scale = window.getContentScale();
        break :scale_factor @max(scale[0], scale[1]);
    };

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
    );
    defer zgui.backend.deinit();

    zgui.getStyle().scaleAllSizes(scale_factor);

    var breakpoints = std.ArrayList(u32).init(common.GeneralAllocator);
    defer breakpoints.deinit();
    dc.cpu.on_trapa = trapa_handler;

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

    var renderer = Renderer.init(common.GeneralAllocator, gctx);
    defer renderer.deinit();

    var renderer_texture_view = gctx.createTextureView(renderer.texture_array, .{ .dimension = .tvdim_2d, .base_array_layer = 0, .array_layer_count = 1 });

    while (!window.shouldClose()) {
        zglfw.pollEvents();

        zgui.backend.newFrame(
            gctx.swapchain_descriptor.width,
            gctx.swapchain_descriptor.height,
        );

        if (zgui.begin("CPU State", .{})) {
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
                const disassembly = try sh4.disassemble(.{ .value = dc.read16(@intCast(addr)) }, common.GeneralAllocator);
                zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == pc) ">" else " ", disassembly });
                addr += 2;
            }

            if (zgui.button(if (running) "Pause" else "Run", .{ .w = 200.0 })) {
                running = !running;
            }

            if (zgui.button("Step", .{ .w = 200.0 })) {
                dc.tick();
            }

            _ = zgui.checkbox("Debug trace", .{ .v = &dc.cpu.debug_trace });

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
                try breakpoints.append(@as(u32, @intCast(static.bp_addr)) & 0x1FFFFFFF);
            }
        }
        zgui.end();

        if (zgui.begin("FPU", .{})) {
            zgui.text("FPUL: {d: >8.2} | {d: >8.2} | {d: >8.2}", .{ @as(f32, @bitCast(dc.cpu.fpul)), dc.cpu.fpul, @as(i32, @bitCast(dc.cpu.fpul)) });

            zgui.spacing();

            zgui.beginGroup();
            for (0..8) |i| {
                zgui.text("FR{d: <2}: {d: >12.2}  ", .{ i, dc.cpu.FR(@truncate(i)).* });
            }
            zgui.endGroup();
            zgui.sameLine(.{});
            zgui.beginGroup();
            for (8..16) |i| {
                zgui.text("FR{d: <2}: {d: >12.2}", .{ i, dc.cpu.FR(@truncate(i)).* });
            }
            zgui.endGroup();

            zgui.spacing();

            zgui.beginGroup();
            for (0..8) |i| {
                zgui.text("XF{d: <2}: {d: >12.2}  ", .{ i, dc.cpu.XF(@truncate(i)).* });
            }
            zgui.endGroup();
            zgui.sameLine(.{});
            zgui.beginGroup();
            for (8..16) |i| {
                zgui.text("XF{d: <2}: {d: >12.2}", .{ i, dc.cpu.XF(@truncate(i)).* });
            }
            zgui.endGroup();
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
                    dc.read8(@intCast(addr)),
                    dc.read8(@intCast(addr + 1)),
                    dc.read8(@intCast(addr + 2)),
                    dc.read8(@intCast(addr + 3)),
                    dc.read8(@intCast(addr + 4)),
                    dc.read8(@intCast(addr + 5)),
                    dc.read8(@intCast(addr + 6)),
                    dc.read8(@intCast(addr + 7)),
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
            zgui.text("ISP_BACKGND_D: {d: >8.2} / {d: >8.2}", .{ ISP_BACKGND_D, @as(f32, @bitCast(ISP_BACKGND_D)) });

            const FB_W_CTRL: u32 = dc.gpu._get_register(u32, .FB_W_CTRL).*;
            const FB_W_SOF1: u32 = dc.gpu._get_register(u32, .FB_W_SOF1).*;
            const FB_W_SOF2: u32 = dc.gpu._get_register(u32, .FB_W_SOF2).*;
            const FB_R_CTRL: u32 = dc.gpu._get_register(u32, .FB_R_CTRL).*;
            const FB_R_SOF1: u32 = dc.gpu._get_register(u32, .FB_R_SOF1).*;
            const FB_R_SOF2: u32 = dc.gpu._get_register(u32, .FB_R_SOF2).*;
            zgui.text("FB_W_CTRL: 0x{X:0>8}", .{FB_W_CTRL});
            zgui.text("FB_W_SOF1: 0x{X:0>8}", .{FB_W_SOF1});
            zgui.text("FB_W_SOF2: 0x{X:0>8}", .{FB_W_SOF2});
            zgui.text("FB_R_CTRL: 0x{X:0>8}", .{FB_R_CTRL});
            zgui.text("FB_R_SOF1: 0x{X:0>8}", .{FB_R_SOF1});
            zgui.text("FB_R_SOF2: 0x{X:0>8}", .{FB_R_SOF2});

            if (zgui.collapsingHeader("Framebuffer?", .{})) {
                const static = struct {
                    var start_addr: i32 = 0x04200000;
                    var format: i32 = 0x6;
                };
                _ = zgui.inputInt("Start", .{ .v = &static.start_addr, .step = 0x8000, .flags = .{ .chars_hexadecimal = true } });
                _ = zgui.inputInt("Format", .{ .v = &static.format, .step = 1, .flags = .{ .chars_hexadecimal = true } });
                static.format = std.math.clamp(static.format, 0, 0x6);
                const bytes_per_pixels: u32 = if (static.format & 0b111 == 0x6) 4 else 2;
                var start: u32 = @intCast(std.math.clamp(static.start_addr, 0x04000000, 0x04800000));
                const end = std.math.clamp(start + bytes_per_pixels * vram_width * vram_height, 0x04000000, 0x04800000);
                zgui.text("(VRAM addresses: {X:0>8} - {X:0>8})", .{ start, end });
                var i: usize = 0;
                while (start < end) {
                    switch (static.format & 0b111) {
                        0x0, 0x3 => { // 0555 KRGB 16 bits
                            const color: Holly.Color16 = .{
                                .value = dc.read16(@intCast(start)),
                            };
                            pixels[4 * i + 0] = @as(u8, @intCast(color.arbg1555.r)) << 3;
                            pixels[4 * i + 1] = @as(u8, @intCast(color.arbg1555.g)) << 3;
                            pixels[4 * i + 2] = @as(u8, @intCast(color.arbg1555.b)) << 3;
                            pixels[4 * i + 3] = 255; // FIXME: Not really.
                            start += 2;
                            i += 1;
                        },
                        0x1 => { // 565 RGB 16 bit
                            const color: Holly.Color16 = .{
                                .value = dc.read16(@intCast(start)),
                            };
                            pixels[4 * i + 0] = @as(u8, @intCast(color.rgb565.r)) << 3;
                            pixels[4 * i + 1] = @as(u8, @intCast(color.rgb565.g)) << 2;
                            pixels[4 * i + 2] = @as(u8, @intCast(color.rgb565.b)) << 3;
                            pixels[4 * i + 3] = 255; // FIXME: Not really.
                            start += 2;
                            i += 1;
                        },
                        // ARGB 32-Bits
                        0x6 => {
                            pixels[4 * i + 0] = dc.read8(@intCast(start + 3));
                            pixels[4 * i + 1] = dc.read8(@intCast(start + 2));
                            pixels[4 * i + 2] = dc.read8(@intCast(start + 1));
                            pixels[4 * i + 3] = dc.read8(@intCast(start + 0));
                            start += 4;
                            i += 1;
                        },
                        else => {
                            start = end;
                            zgui.text("Unsupported packed format: 0x{X:0>1}", .{static.format & 0b111});
                        },
                    }
                }

                gctx.queue.writeTexture(
                    .{ .texture = gctx.lookupResource(texture).? },
                    .{
                        .bytes_per_row = 4 * vram_width,
                        .rows_per_image = vram_height,
                    },
                    .{ .width = vram_width, .height = vram_height },
                    u8,
                    pixels,
                );
                const tex_id = gctx.lookupResource(texture_view).?;

                zgui.image(tex_id, .{ .w = vram_width, .h = vram_height });
            }

            if (zgui.collapsingHeader("Renderer Textures", .{})) {
                const static = struct {
                    var index: i32 = 0;
                    var scale: f32 = 1;
                };
                if (zgui.inputInt("Index", .{ .v = &static.index, .step = 1 })) {
                    gctx.releaseResource(renderer_texture_view);
                    static.index = std.math.clamp(static.index, 0, 255);
                    renderer_texture_view = gctx.createTextureView(renderer.texture_array, .{ .dimension = .tvdim_2d, .base_array_layer = @as(u32, @intCast(static.index)), .array_layer_count = 1 });
                }
                if (zgui.dragFloat("Scale", .{ .v = &static.scale, .min = 1.0, .max = 8.0, .speed = 0.1 })) {
                    static.scale = std.math.clamp(static.scale, 1.0, 8.0);
                }
                const tex_id = gctx.lookupResource(renderer_texture_view).?;
                zgui.image(tex_id, .{ .w = static.scale * 1024, .h = static.scale * 1024 });
            }
        }
        zgui.end();

        // FIXME: Just testing things, as always!
        const keybinds: [7]struct { zglfw.Key, MapleModule.ControllerButtons } = .{
            .{ .enter, .{ .start = 0 } },
            .{ .up, .{ .up = 0 } },
            .{ .down, .{ .down = 0 } },
            .{ .left, .{ .left = 0 } },
            .{ .right, .{ .right = 0 } },
            .{ .q, .{ .a = 0 } },
            .{ .w, .{ .b = 0 } },
        };
        for (keybinds) |keybind| {
            const key_status = window.getKey(keybind[0]);
            if (key_status == .press) {
                dc.maple.ports[0].press_buttons(keybind[1]);
            } else if (key_status == .release) {
                dc.maple.ports[0].release_buttons(keybind[1]);
            }
        }

        if (running) {
            const start = try std.time.Instant.now();
            // FIXME: We break on render start for synchronization, this is not how we'll want to do it in the end.
            while (running and (try std.time.Instant.now()).since(start) < 16 * std.time.ns_per_ms and !dc.gpu.render_start) {
                dc.tick();

                // Crude outlier values checking
                if (false) {
                    if (std.math.lossyCast(f32, dc.cpu.fpul) >= 4294966000.000) {
                        std.debug.print("Weird: FPUL = {d}\n", .{std.math.lossyCast(f32, dc.cpu.fpul)});
                        running = false;
                    }
                    for (0..16) |i| {
                        if (dc.cpu.FR(@intCast(i)).* >= 4294966000.000) {
                            std.debug.print("Weird: FR({d}) = {d}\n", .{ i, dc.cpu.FR(@intCast(i)).* });
                            running = false;
                        }
                    }
                }

                const breakpoint = for (breakpoints.items, 0..) |addr, index| {
                    if (addr & 0x1FFFFFFF == dc.cpu.pc & 0x1FFFFFFF) break index;
                } else null;
                if (breakpoint != null) {
                    running = false;
                }
            }
        }

        const swapchain_texv = gctx.swapchain.getCurrentTextureView();
        defer swapchain_texv.release();

        if (dc.gpu.render_start) { // FIXME: Find a better way to start a render.
            dc.gpu.render_start = false;
            try renderer.update(&dc.gpu);
        }
        renderer.draw(); // Draw to a texture and reuse it instead of re drawing everytime?

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
