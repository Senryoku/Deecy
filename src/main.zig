const std = @import("std");

const common = @import("./common.zig");
const sh4 = @import("./sh4.zig");
const MemoryRegisters = @import("./MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;
const GDI = @import("./GDI.zig").GDI;

const zgui = @import("zgui");
const zgpu = @import("zgpu");
const zglfw = @import("zglfw");

const assets_dir = "assets/";

const Color = packed union {
    value: u16,
    arbg1555: packed struct(u16) {
        b: u5,
        g: u5,
        r: u5,
        a: u1,
    },
    rgb565: packed struct(u16) {
        b: u5,
        g: u6,
        r: u5,
    },
    argb4444: packed struct(u16) {
        b: u4,
        g: u4,
        r: u4,
        a: u4,
    },
};

const YUV422 = packed struct(u32) {
    v: u8,
    y1: u8,
    u: u8,
    y0: u8,
};

const RGBA = packed struct(u32) {
    a: u8,
    b: u8,
    g: u8,
    r: u8,
};

fn yuv_to_rgba(yuv: YUV422) [2]RGBA {
    const v = @as(f32, @floatFromInt(yuv.v)) - 128.0;
    const u = @as(f32, @floatFromInt(yuv.u)) - 128.0;
    const y0 = @as(f32, @floatFromInt(yuv.y0));
    const y1 = @as(f32, @floatFromInt(yuv.y1));
    return .{ .{
        .r = @intFromFloat(std.math.clamp(y0 + (11 / 8) * v, 0, 255)),
        .g = @intFromFloat(std.math.clamp(y0 - 0.25 * (11 / 8) * u - 0.5 * (11 / 8) * v, 0, 255)),
        .b = @intFromFloat(std.math.clamp(y0 + 1.25 * (11 / 8) * u, 0, 255)),
        .a = 255,
    }, .{
        .r = @intFromFloat(std.math.clamp(y1 + (11 / 8) * v, 0, 255)),
        .g = @intFromFloat(std.math.clamp(y1 - 0.25 * (11 / 8) * u - 0.5 * (11 / 8) * v, 0, 255)),
        .b = @intFromFloat(std.math.clamp(y1 + 1.25 * (11 / 8) * u, 0, 255)),
        .a = 255,
    } };
}

// FIXME
const syscall = @import("syscall.zig");

pub fn main() !void {
    std.debug.print("\r  == Katana ==                             \n", .{});

    var cpu = try sh4.SH4.init(common.GeneralAllocator);
    defer cpu.deinit();

    cpu.init_boot();

    var gdrom = &syscall.gdrom; // FIXME
    //try gdrom.disk.init("./bin/[GDI] Virtua Tennis (EU)/Virtua Tennis v1.001 (2000)(Sega)(PAL)(M4)[!].gdi", common.GeneralAllocator);
    //try gdrom.disk.init("./bin/[GDI] ChuChu Rocket!/ChuChu Rocket! v1.007 (2000)(Sega)(NTSC)(US)(en-ja)[!].gdi", common.GeneralAllocator);
    //try gdrom.disk.init("./bin/[GDI] Sonic Adventure (PAL)/Sonic Adventure v1.003 (1999)(Sega)(PAL)(M5)[!].gdi", common.GeneralAllocator);
    try gdrom.disk.init("./bin/GigaWing (USA)/GigaWing v1.000 (2000)(Capcom)(US)[!].gdi", common.GeneralAllocator);
    defer gdrom.disk.deinit();

    // Load IP.bin from disk (16 first sectors of the last track)
    // FIXME: Here we assume the last track is the 3rd.
    _ = gdrom.disk.load_sectors(45150, 16 * 2048, cpu.ram[0x00008000..]);

    syscall.FirstReadBINSectorSize = (try gdrom.disk.load_file("1ST_READ.BIN;1", cpu.ram[0x00010000..]) + 2047) / 2048;

    //while (true) {
    //    if (cpu.pc == 0x8C001008)
    //        cpu.debug_trace = true;
    //    cpu.execute();
    //}

    try zglfw.init();
    defer zglfw.terminate();

    const window = zglfw.Window.create(1024, 800, "Katana", null) catch {
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

    var running = false;
    var breakpoints = std.ArrayList(u32).init(common.GeneralAllocator);
    defer breakpoints.deinit();

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

    while (!window.shouldClose()) {
        zglfw.pollEvents();

        zgui.backend.newFrame(
            gctx.swapchain_descriptor.width,
            gctx.swapchain_descriptor.height,
        );

        // Set the starting window position and size to custom values
        zgui.setNextWindowPos(.{ .x = 20.0, .y = 20.0, .cond = .first_use_ever });
        zgui.setNextWindowSize(.{ .w = -1.0, .h = -1.0, .cond = .first_use_ever });

        if (zgui.begin("CPU State", .{})) {
            zgui.text("PC: 0x{X:0>8} - SPC: 0x{X:0>8}", .{ cpu.pc, cpu.spc });
            zgui.text("PR: 0x{X:0>8}", .{cpu.pr});
            zgui.text("SR: T={any}, S={any}, IMASK={d}", .{ cpu.sr.t, cpu.sr.s, cpu.sr.imask });
            zgui.text("GBR: 0x{X:0>8}", .{cpu.gbr});
            zgui.text("VBR: 0x{X:0>8}", .{cpu.vbr});
            zgui.beginGroup();
            for (0..8) |i| {
                zgui.text("R{d: <2}: 0x{X:0>8}", .{ i, cpu.R(@truncate(i)).* });
            }
            zgui.endGroup();
            zgui.sameLine(.{});
            zgui.beginGroup();
            for (8..16) |i| {
                zgui.text("R{d: <2}: 0x{X:0>8}", .{ i, cpu.R(@truncate(i)).* });
            }
            zgui.endGroup();

            var addr = @max(0, @min(0x100000000 - 16, cpu.pc - 8));
            const end_addr = @min(0xFFFFFFFFF, @min(0x100000000 - 16, addr) + 16);
            while (addr < end_addr) {
                //zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == cpu.pc) ">" else " ", sh4.Opcodes[sh4.JumpTable[cpu.read16(@intCast(addr))]].name });
                const disassembly = try sh4.disassemble(.{ .value = cpu.read16(@intCast(addr)) }, common.GeneralAllocator);
                zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == cpu.pc) ">" else " ", disassembly });
                addr += 2;
            }

            if (zgui.button(if (running) "Pause" else "Run", .{ .w = 200.0 })) {
                running = !running;
            }

            if (zgui.button("Step", .{ .w = 200.0 })) {
                cpu.execute();
            }

            _ = zgui.checkbox("Debug trace", .{ .v = &cpu.debug_trace });

            for (0..breakpoints.items.len) |i| {
                zgui.text("Breakpoint {d}: 0x{X:0>8}", .{ i, breakpoints.items[i] });
            }
            const static = struct {
                var bp_addr: i32 = 0;
            };
            _ = zgui.inputInt("##breakpoint", .{ .v = &static.bp_addr, .flags = .{ .chars_hexadecimal = true } });
            zgui.sameLine(.{});
            if (zgui.button("Add Breakpoint", .{ .w = 200.0 })) {
                try breakpoints.append(@intCast(static.bp_addr));
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
                    cpu.read8(@intCast(addr)),
                    cpu.read8(@intCast(addr + 1)),
                    cpu.read8(@intCast(addr + 2)),
                    cpu.read8(@intCast(addr + 3)),
                    cpu.read8(@intCast(addr + 4)),
                    cpu.read8(@intCast(addr + 5)),
                    cpu.read8(@intCast(addr + 6)),
                    cpu.read8(@intCast(addr + 7)),
                });
                addr += 8;
            }
        }
        zgui.end();

        if (zgui.begin("VRAM", .{})) {
            const FB_W_CTRL: u32 = cpu.read32(0x005F8048);
            const FB_W_SOF1: u32 = cpu.read32(0x005F8060);
            const FB_W_SOF2: u32 = cpu.read32(0x005F8064);
            zgui.text("FB_W_CTRL: 0x{X:0>8}", .{FB_W_CTRL});
            zgui.text("FB_W_SOF1: 0x{X:0>8}", .{FB_W_SOF1});
            zgui.text("FB_W_SOF2: 0x{X:0>8}", .{FB_W_SOF2});

            if (zgui.collapsingHeader("Framebuffer?", .{})) {
                var start: u32 = 0x05200000;
                const bytes_per_pixels: u32 = if (FB_W_CTRL & 0b111 == 0x6) 4 else 2;
                const end = start + bytes_per_pixels * vram_width * vram_height;
                var i: usize = 0;
                while (start < end) {
                    switch (FB_W_CTRL & 0b111) {
                        0x0, 0x3 => { // 0555 KRGB 16 bits
                            const color: Color = .{
                                .value = cpu.read16(@intCast(start)),
                            };
                            pixels[4 * i + 0] = @as(u8, @intCast(color.arbg1555.r)) << 3;
                            pixels[4 * i + 1] = @as(u8, @intCast(color.arbg1555.g)) << 3;
                            pixels[4 * i + 2] = @as(u8, @intCast(color.arbg1555.b)) << 3;
                            pixels[4 * i + 3] = 255; // FIXME: Not really.
                            start += 4;
                            i += 1;
                        },
                        0x1 => { // 565 RGB 16 bit
                            const color: Color = .{
                                .value = cpu.read16(@intCast(start)),
                            };
                            pixels[4 * i + 0] = @as(u8, @intCast(color.rgb565.r)) << 3;
                            pixels[4 * i + 1] = @as(u8, @intCast(color.rgb565.g)) << 2;
                            pixels[4 * i + 2] = @as(u8, @intCast(color.rgb565.b)) << 3;
                            pixels[4 * i + 3] = 255; // FIXME: Not really.
                            start += 4;
                            i += 1;
                        },

                        //const yuv: YUV422 = @bitCast(cpu.read32(@intCast(start)));
                        //const rgba = yuv_to_rgba(yuv);
                        //pixels[4 * i + 0] = @as(u8, @intCast(rgba[0].r));
                        //pixels[4 * i + 1] = @as(u8, @intCast(rgba[0].g));
                        //pixels[4 * i + 2] = @as(u8, @intCast(rgba[0].b));
                        //pixels[4 * i + 3] = 255;
                        //pixels[4 * (i + 1) + 0] = @as(u8, @intCast(rgba[1].r));
                        //pixels[4 * (i + 1) + 1] = @as(u8, @intCast(rgba[1].g));
                        //pixels[4 * (i + 1) + 2] = @as(u8, @intCast(rgba[1].b));
                        //pixels[4 * (i + 1) + 3] = 255;
                        //start += 4;
                        //i += 2;

                        //const palette_data: u8 = @bitCast(cpu.read8(@intCast(start)));
                        //pixels[4 * i + 0] = @as(u8, @intCast(palette_data));
                        //pixels[4 * i + 1] = @as(u8, @intCast(palette_data));
                        //pixels[4 * i + 2] = @as(u8, @intCast(palette_data));
                        //pixels[4 * i + 3] = 255;
                        //start += 1;
                        //i += 1;

                        // ARGB 32-Bits
                        0x6 => {
                            pixels[4 * i + 0] = cpu.read8(@intCast(start + 3));
                            pixels[4 * i + 1] = cpu.read8(@intCast(start + 2));
                            pixels[4 * i + 2] = cpu.read8(@intCast(start + 1));
                            pixels[4 * i + 3] = cpu.read8(@intCast(start + 0));
                            start += 4;
                            i += 1;
                        },
                        else => {
                            start = end;
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

            if (zgui.collapsingHeader("VRAM", .{})) {
                const static = struct {
                    var start: i32 = 0;
                    var format: i32 = 0x6;
                };

                const bytes_per_pixels: u32 = if (static.format == 0x6) 4 else 2;

                _ = zgui.inputInt("Start", .{ .v = &static.start, .step = @intCast(bytes_per_pixels * vram_width), .flags = .{ .chars_hexadecimal = true } });
                static.start = @max(0, @min(static.start, @as(i32, @intCast(cpu.gpu.vram.len)) - @as(i32, @intCast(bytes_per_pixels * vram_width * vram_height))));
                _ = zgui.inputInt("Format", .{ .v = &static.format });
                static.format = @max(0, @min(static.format, 0x9));

                var start: i32 = static.start;
                const end: i32 = start + @as(i32, @intCast(bytes_per_pixels * vram_width * vram_height));
                var i: usize = 0;
                while (start < end) {
                    switch (static.format) {
                        0x0, 0x3 => { // 0555 KRGB 16 bits
                            const color: Color = .{
                                .value = cpu.gpu.vram[@intCast(start)],
                            };
                            pixels[4 * i + 0] = @as(u8, @intCast(color.arbg1555.r)) << 3;
                            pixels[4 * i + 1] = @as(u8, @intCast(color.arbg1555.g)) << 3;
                            pixels[4 * i + 2] = @as(u8, @intCast(color.arbg1555.b)) << 3;
                            pixels[4 * i + 3] = 255; // FIXME: Not really.
                            start += 4;
                            i += 1;
                        },
                        0x1 => { // 565 RGB 16 bit
                            const color: Color = .{
                                .value = cpu.gpu.vram[@intCast(start)],
                            };
                            pixels[4 * i + 0] = @as(u8, @intCast(color.rgb565.r)) << 3;
                            pixels[4 * i + 1] = @as(u8, @intCast(color.rgb565.g)) << 2;
                            pixels[4 * i + 2] = @as(u8, @intCast(color.rgb565.b)) << 3;
                            pixels[4 * i + 3] = 255; // FIXME: Not really.
                            start += 4;
                            i += 1;
                        },
                        // ARGB 32-Bits
                        0x6 => {
                            pixels[4 * i + 0] = cpu.gpu.vram[@as(u32, @intCast(start)) + 3];
                            pixels[4 * i + 1] = cpu.gpu.vram[@as(u32, @intCast(start)) + 2];
                            pixels[4 * i + 2] = cpu.gpu.vram[@as(u32, @intCast(start)) + 1];
                            pixels[4 * i + 3] = cpu.gpu.vram[@as(u32, @intCast(start)) + 0];
                            start += 4;
                            i += 1;
                        },
                        else => {
                            start = end;
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
        }
        zgui.end();

        if (running) {
            for (0..10000) |_| {
                cpu.execute();
                const breakpoint = for (breakpoints.items, 0..) |addr, index| {
                    if (addr & 0x1FFFFFFF == cpu.pc & 0x1FFFFFFF) break index;
                } else null;
                if (breakpoint != null) {
                    running = false;
                    break;
                }
            }
        }

        const swapchain_texv = gctx.swapchain.getCurrentTextureView();
        defer swapchain_texv.release();

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
        _ = gctx.present();
    }
}

test "all tests" {
    _ = sh4;
}
