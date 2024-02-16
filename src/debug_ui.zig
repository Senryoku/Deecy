const std = @import("std");
const builtin = @import("builtin");
const zgpu = @import("zgpu");
const zgui = @import("zgui");

const P4Register = @import("./sh4.zig").P4Register;

const sh4 = @import("./sh4.zig");
const sh4_disassembly = sh4.sh4_disassembly;
const arm7 = @import("arm7");
const Holly = @import("./holly.zig");

const RendererModule = @import("renderer.zig");

const Deecy = @import("deecy.zig").Deecy;

const vram_width: u32 = 640;
const vram_height: u32 = 480;

draw_debug_ui: bool = true,

show_disabled_channels: bool = false,

vram_texture: zgpu.TextureHandle = undefined,
vram_texture_view: zgpu.TextureViewHandle = undefined,
renderer_texture_views: [8]zgpu.TextureViewHandle = undefined,

pixels: []u8 = undefined,

_allocator: std.mem.Allocator,
_gctx: *zgpu.GraphicsContext,

pub fn init(d: *Deecy) !@This() {
    var self = @This(){
        ._allocator = d._allocator,
        ._gctx = d.gctx,
    };

    self.vram_texture = d.gctx.createTexture(.{
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
    self.vram_texture_view = d.gctx.createTextureView(self.vram_texture, .{});

    self.pixels = try self._allocator.alloc(u8, (vram_width * vram_height) * 4);

    for (0..self.renderer_texture_views.len) |i|
        self.renderer_texture_views[i] = d.gctx.createTextureView(d.renderer.texture_arrays[i], .{
            .dimension = .tvdim_2d,
            .base_array_layer = 0,
            .array_layer_count = 1,
        });

    return self;
}

pub fn deinit(self: *@This()) void {
    for (self.renderer_texture_views) |view|
        self._gctx.releaseResource(view);

    self._gctx.releaseResource(self.vram_texture_view);
    self._gctx.releaseResource(self.vram_texture);

    self._allocator.free(self.pixels);
}

pub fn draw(self: *@This(), d: *Deecy) !void {
    if (self.draw_debug_ui) {
        var dc = d.dc;

        if (zgui.begin("CPU State", .{})) {
            _ = zgui.checkbox("JIT", .{ .v = &d.enable_jit });
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
                const disassembly = try sh4_disassembly.disassemble(.{ .value = dc.cpu.read16(@intCast(addr)) }, self._allocator);
                zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == pc) ">" else " ", disassembly });
                addr += 2;
            }

            if (zgui.button(if (d.running) "Pause" else "Run", .{ .w = 200.0 })) {
                d.running = !d.running;
            }

            if (zgui.button("Step", .{ .w = 200.0 })) {
                d.running = false;
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

            for (0..d.breakpoints.items.len) |i| {
                zgui.text("Breakpoint {d}: 0x{X:0>8}", .{ i, d.breakpoints.items[i] });
                zgui.sameLine(.{});
                if (zgui.button("Remove", .{})) {
                    _ = d.breakpoints.orderedRemove(i);
                    break;
                }
            }
            const static = struct {
                var bp_addr: i32 = 0;
            };
            _ = zgui.inputInt("##breakpoint", .{ .v = &static.bp_addr, .flags = .{ .chars_hexadecimal = true } });
            zgui.sameLine(.{});
            if (zgui.button("Add Breakpoint", .{ .w = 200.0 })) {
                try d.breakpoints.append(@as(u32, @intCast(static.bp_addr & 0x1FFFFFFF)));
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
            _ = zgui.checkbox("Show disabled channels", .{ .v = &self.show_disabled_channels });
            inline for (0..64) |i| {
                const channel = dc.aica.get_channel(@intCast(i));
                if (self.show_disabled_channels or channel.play_control.key_on_bit) {
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
            zgui.text("FB_W_CTRL: 0x{X:0>8} - {any}", .{ @as(u32, @bitCast(FB_W_CTRL)), FB_W_CTRL });
            zgui.text("FB_W_SOF1: 0x{X:0>8}", .{FB_W_SOF1});
            zgui.text("FB_W_SOF2: 0x{X:0>8}", .{FB_W_SOF2});
            zgui.text("FB_R_CTRL: 0x{X:0>8} - {any}", .{ @as(u32, @bitCast(FB_R_CTRL)), FB_R_CTRL });
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
                static.width = std.math.clamp(static.width, 8, @as(i32, @intCast(vram_width)));
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
                            self.pixels[4 * i + 0] = @as(u8, @intCast(color.arbg1555.r)) << 3;
                            self.pixels[4 * i + 1] = @as(u8, @intCast(color.arbg1555.g)) << 3;
                            self.pixels[4 * i + 2] = @as(u8, @intCast(color.arbg1555.b)) << 3;
                            self.pixels[4 * i + 3] = 255; // FIXME: Not really.
                            i += 1;
                        },
                        0x1 => { // 565 RGB 16 bit
                            const color: Holly.Color16 = .{ .value = dc.cpu.read16(addr) };
                            self.pixels[4 * i + 0] = @as(u8, @intCast(color.rgb565.r)) << 3;
                            self.pixels[4 * i + 1] = @as(u8, @intCast(color.rgb565.g)) << 2;
                            self.pixels[4 * i + 2] = @as(u8, @intCast(color.rgb565.b)) << 3;
                            self.pixels[4 * i + 3] = 255;
                            i += 1;
                        },
                        // ARGB 32-Bits
                        0x6 => {
                            self.pixels[4 * i + 0] = dc.cpu.read8(@intCast(addr + 3));
                            self.pixels[4 * i + 1] = dc.cpu.read8(@intCast(addr + 2));
                            self.pixels[4 * i + 2] = dc.cpu.read8(@intCast(addr + 1));
                            self.pixels[4 * i + 3] = dc.cpu.read8(@intCast(addr + 0));
                            i += 1;
                        },
                        else => {
                            current_addr = end;
                            zgui.text("Unsupported packed format: 0x{X:0>1}", .{static.format & 0b111});
                        },
                    }
                    current_addr += bytes_per_pixels;
                }

                d.gctx.queue.writeTexture(
                    .{ .texture = d.gctx.lookupResource(self.vram_texture).? },
                    .{
                        .bytes_per_row = 4 * vram_width,
                        .rows_per_image = vram_height,
                    },
                    .{ .width = width, .height = vram_height },
                    u8,
                    self.pixels,
                );
                const tex_id = d.gctx.lookupResource(self.vram_texture_view).?;

                zgui.image(tex_id, .{ .w = vram_width, .h = vram_height });
            }
        }
        zgui.end();

        if (zgui.begin("Renderer", .{})) {
            zgui.text("Min Depth: {d: >4.2}", .{d.renderer.min_depth});
            zgui.text("Max Depth: {d: >4.2}", .{d.renderer.max_depth});
            if (zgui.collapsingHeader("Textures", .{})) {
                const static = struct {
                    var index: i32 = 0;
                    var scale: f32 = 512.0 / 8.0;
                    var size: i32 = 0;
                };
                if (zgui.inputInt("Size", .{ .v = &static.size, .step = 1 })) {
                    static.size = std.math.clamp(static.size, 0, @as(i32, @intCast(self.renderer_texture_views.len - 1)));
                    static.scale = @as(f32, 512) / @as(f32, @floatFromInt((@as(u32, 8) << @intCast(static.size))));
                    static.index = std.math.clamp(static.index, 0, RendererModule.Renderer.MaxTextures[@intCast(static.size)] - 1);
                    d.gctx.releaseResource(self.renderer_texture_views[@intCast(static.size)]);
                    self.renderer_texture_views[@intCast(static.size)] = d.gctx.createTextureView(d.renderer.texture_arrays[@intCast(static.size)], .{ .dimension = .tvdim_2d, .base_array_layer = @as(u32, @intCast(static.index)), .array_layer_count = 1 });
                }
                zgui.sameLine(.{});
                zgui.text("{d: >3}x{d: >3}", .{ @as(u32, 8) << @intCast(static.size), @as(u32, 8) << @intCast(static.size) });
                if (zgui.inputInt("Index", .{ .v = &static.index, .step = 1 })) {
                    static.index = std.math.clamp(static.index, 0, RendererModule.Renderer.MaxTextures[@intCast(static.size)] - 1);
                    d.gctx.releaseResource(self.renderer_texture_views[@intCast(static.size)]);
                    self.renderer_texture_views[@intCast(static.size)] = d.gctx.createTextureView(d.renderer.texture_arrays[@intCast(static.size)], .{ .dimension = .tvdim_2d, .base_array_layer = @as(u32, @intCast(static.index)), .array_layer_count = 1 });
                }
                if (zgui.dragFloat("Scale", .{ .v = &static.scale, .min = 1.0, .max = 8.0, .speed = 0.1 })) {
                    static.scale = std.math.clamp(static.scale, 1.0, 8.0);
                }
                const tex_id = d.gctx.lookupResource(self.renderer_texture_views[@intCast(static.size)]).?;
                zgui.image(tex_id, .{ .w = static.scale * @as(f32, @floatFromInt(@as(u32, 8) << @intCast(static.size))), .h = static.scale * @as(f32, @floatFromInt(@as(u32, 8) << @intCast(static.size))) });
                zgui.text("Parameter Control Word: {any}", .{d.renderer.texture_metadata[@intCast(static.size)][@intCast(static.index)].control_word});
                zgui.text("TSP Instruction: {any}", .{d.renderer.texture_metadata[@intCast(static.size)][@intCast(static.index)].tsp_instruction});
            }
            if (zgui.collapsingHeader("Framebuffer Texture", .{})) {
                const fb_tex_id = d.gctx.lookupResource(d.renderer.framebuffer_texture_view).?;
                zgui.image(fb_tex_id, .{ .w = 640, .h = 480 });
            }
            if (zgui.collapsingHeader("Resized Framebuffer Texture", .{})) {
                const fb_tex_id = d.gctx.lookupResource(d.renderer.resized_framebuffer_texture_view).?;
                zgui.image(fb_tex_id, .{ .w = @floatFromInt(d.gctx.swapchain_descriptor.width), .h = @floatFromInt(d.gctx.swapchain_descriptor.height) });
            }
        }
        zgui.end();

        if (zgui.begin("Debug", .{})) {
            if (zgui.button("Trigger RenderDoneVideo Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .RenderDoneVideo = 1 });
            if (zgui.button("Trigger RenderDoneISP Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .RenderDoneISP = 1 });
            if (zgui.button("Trigger RenderDoneTSP Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .RenderDoneTSP = 1 });
            if (zgui.button("Trigger VBlankIn Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .VBlankIn = 1 });
            if (zgui.button("Trigger VBlankOut Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .VBlankOut = 1 });
            if (zgui.button("Trigger HBlankIn Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .HBlankIn = 1 });
            if (zgui.button("Trigger EoT_YUV Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoT_YUV = 1 });
            if (zgui.button("Trigger EoT_OpaqueList Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoT_OpaqueList = 1 });
            if (zgui.button("Trigger EoT_OpaqueModifierVolumeList Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoT_OpaqueModifierVolumeList = 1 });
            if (zgui.button("Trigger EoT_TranslucentList Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoT_TranslucentList = 1 });
            if (zgui.button("Trigger EoT_TranslucentModifierVolumeList Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoT_TranslucentModifierVolumeList = 1 });
            if (zgui.button("Trigger EoD_PVR Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_PVR = 1 });
            if (zgui.button("Trigger EoD_Maple Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_Maple = 1 });
            if (zgui.button("Trigger MapleVBlankOver Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .MapleVBlankOver = 1 });
            if (zgui.button("Trigger EoD_GDROM Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_GDROM = 1 });
            if (zgui.button("Trigger EoD_AICA Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_AICA = 1 });
            if (zgui.button("Trigger EoD_EXT1 Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_EXT1 = 1 });
            if (zgui.button("Trigger EoD_EXT2 Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_EXT2 = 1 });
            if (zgui.button("Trigger EoD_DEV Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_DEV = 1 });
            if (zgui.button("Trigger EoD_CH2 Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_CH2 = 1 });
            if (zgui.button("Trigger EoD_PVRSort Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_PVRSort = 1 });
            if (zgui.button("Trigger EoD_PunchThroughList Interrupt", .{}))
                dc.raise_normal_interrupt(.{ .EoD_PunchThroughList = 1 });

            zgui.separator();

            if (zgui.button("Trigger GDRom Interrupt", .{}))
                dc.raise_external_interrupt(.{ .GDRom = 1 });
            if (zgui.button("Trigger AICA Interrupt", .{}))
                dc.raise_external_interrupt(.{ .AICA = 1 });
            if (zgui.button("Trigger Modem Interrupt", .{}))
                dc.raise_external_interrupt(.{ .Modem = 1 });
            if (zgui.button("Trigger ExternalDevice Interrupt", .{}))
                dc.raise_external_interrupt(.{ .ExternalDevice = 1 });
        }
        zgui.end();
    }
}
