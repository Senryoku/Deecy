const std = @import("std");

const common = @import("./common.zig");
const termcolor = @import("./termcolor.zig");
const sh4 = @import("./sh4.zig");
const MemoryRegisters = @import("./MemoryRegisters.zig");
const MemoryRegister = MemoryRegisters.MemoryRegister;
const GDI = @import("./GDI.zig").GDI;
const Holly = @import("./holly.zig");

const zgui = @import("zgui");
const zgpu = @import("zgpu");
const zglfw = @import("zglfw");

const RendererModule = @import("./Renderer.zig");
const Renderer = RendererModule.Renderer;

const assets_dir = "assets/";

// FIXME
const syscall = @import("syscall.zig");

pub fn main() !void {
    std.debug.print("\r  == Katana ==                             \n", .{});

    var load_binary = false;
    var binary_path: ?[]const u8 = null;

    var args = try std.process.argsWithAllocator(common.GeneralAllocator);
    defer args.deinit();
    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "-b")) {
            const v = args.next();
            if (v == null) {
                std.debug.print(termcolor.red("Expected path to binary file after -b.\n"), .{});
                return;
            }
            load_binary = true;
            binary_path = v.?;
        }
    }

    var cpu = try sh4.SH4.init(common.GeneralAllocator);
    defer cpu.deinit();

    var gdrom = &syscall.gdrom; // FIXME
    defer gdrom.disk.deinit();

    if (load_binary) {
        var bin_file = try std.fs.cwd().openFile(binary_path.?, .{});
        defer bin_file.close();
        _ = try bin_file.readAll(cpu.ram[0x10000..]);

        cpu.init_boot();
        cpu.pc = 0xAC010000;
    } else {
        cpu.init_boot();

        //try gdrom.disk.init("./bin/[GDI] Virtua Tennis (EU)/Virtua Tennis v1.001 (2000)(Sega)(PAL)(M4)[!].gdi", common.GeneralAllocator);
        //try gdrom.disk.init("./bin/[GDI] ChuChu Rocket!/ChuChu Rocket! v1.007 (2000)(Sega)(NTSC)(US)(en-ja)[!].gdi", common.GeneralAllocator);
        try gdrom.disk.init("./bin/[GDI] Sonic Adventure (PAL)/Sonic Adventure v1.003 (1999)(Sega)(PAL)(M5)[!].gdi", common.GeneralAllocator);
        //try gdrom.disk.init("./bin/GigaWing (USA)/GigaWing v1.000 (2000)(Capcom)(US)[!].gdi", common.GeneralAllocator);

        // Load IP.bin from disk (16 first sectors of the last track)
        // FIXME: Here we assume the last track is the 3rd.
        _ = gdrom.disk.load_sectors(45150, 16 * 2048, cpu.ram[0x00008000..]);

        syscall.FirstReadBINSectorSize = (try gdrom.disk.load_file("1ST_READ.BIN;1", cpu.ram[0x00010000..]) + 2047) / 2048;
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

    var vram_texture = gctx.createTexture(.{
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
    var vram_texture_view = gctx.createTextureView(vram_texture, .{});

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

            const range = 16; // In bytes.
            const pc = cpu.pc & 0x1FFFFFFF;
            //              In RAM                                                                           In BootROM
            var addr = (if (pc >= 0x0C000000) std.math.clamp(pc, 0x0C000000 + range / 2, 0x0D000000 - range) else std.math.clamp(pc, 0x00000000 + range / 2, 0x02000000 - range)) - range / 2;
            const end_addr = addr + range;
            while (addr < end_addr) {
                //zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == cpu.pc) ">" else " ", sh4.Opcodes[sh4.JumpTable[cpu.read16(@intCast(addr))]].name });
                const disassembly = try sh4.disassemble(.{ .value = cpu.read16(@intCast(addr)) }, common.GeneralAllocator);
                zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == pc) ">" else " ", disassembly });
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
                            const color: Holly.Color16 = .{
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
                            const color: Holly.Color16 = .{
                                .value = cpu.read16(@intCast(start)),
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

            if (zgui.collapsingHeader("VRAM", .{})) {
                const static = struct {
                    var start: i32 = 0;
                    var format: i32 = 0x6;
                    var display_width: i32 = vram_width;
                    var scale: i32 = 1;
                    var twiddle: bool = true;
                };

                const bytes_per_pixels: u32 = if (static.format == 0x6) 4 else 2;

                _ = zgui.inputInt("Start", .{ .v = &static.start, .step = @intCast(bytes_per_pixels * vram_width), .step_fast = @intCast(16 * bytes_per_pixels * vram_width), .flags = .{ .chars_hexadecimal = true } });
                static.start = @max(0, @min(static.start, @as(i32, @intCast(cpu.gpu.vram.len)) - @as(i32, @intCast(bytes_per_pixels * vram_width * vram_height))));
                _ = zgui.inputInt("Format", .{ .v = &static.format });
                static.format = @max(0, @min(static.format, 0x9));
                if (zgui.inputInt("Width", .{ .v = &static.display_width, .step = 1 })) {
                    static.display_width = @max(8, @min(static.display_width, vram_width));
                    gctx.releaseResource(vram_texture);
                    vram_texture = gctx.createTexture(.{
                        .usage = .{ .texture_binding = true, .copy_dst = true },
                        .size = .{
                            .width = @intCast(static.display_width),
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
                    gctx.releaseResource(vram_texture_view);
                    vram_texture_view = gctx.createTextureView(vram_texture, .{});
                }
                _ = zgui.inputInt("Scale", .{ .v = &static.scale, .step = 1 });
                static.scale = @max(1, @min(static.scale, 8));
                _ = zgui.checkbox("Twiddle", .{ .v = &static.twiddle });

                var start: i32 = static.start;
                const end: i32 = start + @as(i32, @intCast(bytes_per_pixels * vram_width * vram_height));
                var i: usize = 0;

                if (static.twiddle) {
                    for (0..vram_height) |y| {
                        for (0..vram_width) |x| {
                            switch (static.format) {
                                0x0, 0x3 => { // 0555 KRGB 16 bits
                                    const color: Holly.Color16 = .{
                                        .value = @as(*u16, @alignCast(@ptrCast(&cpu.gpu.vram[@as(u32, @intCast(start)) + 2 * RendererModule.zorder_curve(@intCast(x), @intCast(y))]))).*,
                                    };
                                    pixels[y * vram_width + x + 0] = @as(u8, @intCast(color.arbg1555.r)) << 3;
                                    pixels[y * vram_width + x + 1] = @as(u8, @intCast(color.arbg1555.g)) << 3;
                                    pixels[y * vram_width + x + 2] = @as(u8, @intCast(color.arbg1555.b)) << 3;
                                    pixels[y * vram_width + x + 3] = 255; // FIXME: Not really.
                                },
                                0x1 => { // 565 RGB 16 bit
                                    const color: Holly.Color16 = .{
                                        .value = @as(*u16, @alignCast(@ptrCast(&cpu.gpu.vram[@as(u32, @intCast(start)) + 2 * RendererModule.zorder_curve(@intCast(x), @intCast(y))]))).*,
                                    };
                                    pixels[y * vram_width + x + 0] = @as(u8, @intCast(color.rgb565.r)) << 3;
                                    pixels[y * vram_width + x + 1] = @as(u8, @intCast(color.rgb565.g)) << 2;
                                    pixels[y * vram_width + x + 2] = @as(u8, @intCast(color.rgb565.b)) << 3;
                                    pixels[y * vram_width + x + 3] = 255; // FIXME: Not really.
                                },
                                0x2 => { // 4444 ARGB 16 bit
                                    const color: Holly.Color16 = .{
                                        .value = @as(*u16, @alignCast(@ptrCast(&cpu.gpu.vram[@as(u32, @intCast(start)) + 2 * RendererModule.zorder_curve(@intCast(x), @intCast(y))]))).*,
                                    };
                                    pixels[y * vram_width + x + 0] = @as(u8, @intCast(color.argb4444.r)) << 4;
                                    pixels[y * vram_width + x + 1] = @as(u8, @intCast(color.argb4444.g)) << 4;
                                    pixels[y * vram_width + x + 2] = @as(u8, @intCast(color.argb4444.b)) << 4;
                                    pixels[y * vram_width + x + 3] = @as(u8, @intCast(color.argb4444.a)) << 4;
                                },
                                0x6 => {
                                    pixels[y * vram_width + x + 0] = cpu.gpu.vram[@as(u32, @intCast(start)) + 4 * RendererModule.zorder_curve(@intCast(x), @intCast(y)) + 3];
                                    pixels[y * vram_width + x + 1] = cpu.gpu.vram[@as(u32, @intCast(start)) + 4 * RendererModule.zorder_curve(@intCast(x), @intCast(y)) + 2];
                                    pixels[y * vram_width + x + 2] = cpu.gpu.vram[@as(u32, @intCast(start)) + 4 * RendererModule.zorder_curve(@intCast(x), @intCast(y)) + 1];
                                    pixels[y * vram_width + x + 3] = cpu.gpu.vram[@as(u32, @intCast(start)) + 4 * RendererModule.zorder_curve(@intCast(x), @intCast(y)) + 0];
                                },
                                else => {
                                    break;
                                },
                            }
                        }
                    }
                } else {
                    while (start < end) {
                        switch (static.format) {
                            0x0, 0x3 => { // 0555 KRGB 16 bits
                                const color: Holly.Color16 = .{
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
                                const color: Holly.Color16 = .{
                                    .value = cpu.gpu.vram[@intCast(start)],
                                };
                                pixels[4 * i + 0] = @as(u8, @intCast(color.rgb565.r)) << 3;
                                pixels[4 * i + 1] = @as(u8, @intCast(color.rgb565.g)) << 2;
                                pixels[4 * i + 2] = @as(u8, @intCast(color.rgb565.b)) << 3;
                                pixels[4 * i + 3] = 255; // FIXME: Not really.
                                start += 4;
                                i += 1;
                            },
                            0x2 => { // 4444 ARGB 16 bit
                                const color: Holly.Color16 = .{
                                    .value = cpu.gpu.vram[@intCast(start)],
                                };
                                pixels[4 * i + 0] = @as(u8, @intCast(color.argb4444.r)) << 4;
                                pixels[4 * i + 1] = @as(u8, @intCast(color.argb4444.g)) << 4;
                                pixels[4 * i + 2] = @as(u8, @intCast(color.argb4444.b)) << 4;
                                pixels[4 * i + 3] = @as(u8, @intCast(color.argb4444.a)) << 4;
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
                }

                gctx.queue.writeTexture(
                    .{ .texture = gctx.lookupResource(vram_texture).? },
                    .{
                        .bytes_per_row = @intCast(4 * static.display_width),
                        .rows_per_image = vram_height,
                    },
                    .{ .width = @intCast(static.display_width), .height = @intCast(vram_height) },
                    u8,
                    pixels,
                );
                const tex_id = gctx.lookupResource(vram_texture_view).?;

                zgui.image(tex_id, .{ .w = @floatFromInt(static.scale * static.display_width), .h = @floatFromInt(static.scale * vram_height) });
            }
        }
        zgui.end();

        // FIXME: Just testing things, as always!
        const enter = window.getKey(.enter);
        if (enter == .press) {
            cpu.maple.ports[0].set_inputs(.{ .a = 0 });
        } else if (enter == .release) {
            cpu.maple.ports[0].set_inputs(.{ .a = 1 });
        }

        if (running) {
            const start = try std.time.Instant.now();
            while ((try std.time.Instant.now()).since(start) < 16 * std.time.ns_per_ms) {
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

        if (cpu.gpu.render_start) { // FIXME: Find a better way to start a render.
            cpu.gpu.render_start = false;
            try renderer.update(&cpu.gpu);
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
}
