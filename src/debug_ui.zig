const std = @import("std");
const builtin = @import("builtin");
const zgpu = @import("zgpu");
const zgui = @import("zgui");
const zglfw = @import("zglfw");

const P4Register = @import("./sh4.zig").P4Register;
const HardwareRegisters = @import("./hardware_registers.zig");

const sh4 = @import("./sh4.zig");
const sh4_disassembly = sh4.sh4_disassembly;
const BasicBlock = @import("jit/basic_block.zig");
const arm7 = @import("arm7");
const Holly = @import("./holly.zig");
const AICAModule = @import("./aica.zig");

const Colors = @import("colors.zig");
const Color16 = Colors.Color16;
const fRGBA = Colors.fRGBA;

const RendererModule = @import("renderer.zig");

const Deecy = @import("deecy.zig").Deecy;

const vram_width: u32 = 640;
const vram_height: u32 = 480;

show_disabled_channels: bool = false,

vram_texture: zgpu.TextureHandle = undefined,
vram_texture_view: zgpu.TextureViewHandle = undefined,
renderer_texture_views: [8][512]zgpu.TextureViewHandle = undefined,

// Strip Debug Display
selected_strip_focus: bool = false, // Element has been clicked and remain in focus
selected_strip_list: Holly.ListType = .Opaque,
selected_strip_index: u32 = 0xFFFFFFFF,

// Vertex Debug Display
selected_vertex_focus: bool = false,
selected_vertex: ?[2]f32 = null,

selected_volume_focus: bool = false,
selected_volume_list: Holly.ListType = .OpaqueModifierVolume,
selected_volume_index: ?u32 = null,

pixels: []u8 = undefined,

audio_channels: [64]struct {
    amplitude_envelope: struct { start_time: i64 = 0, xv: std.ArrayList(u32) = undefined, yv: std.ArrayList(u32) = undefined } = .{},
} = .{.{}} ** 64,

_allocator: std.mem.Allocator,
_gctx: *zgpu.GraphicsContext,

fn printable_ascii(c: u8) u8 {
    if (c >= 0x20 and c <= 0x7E)
        return c;
    return '.';
}

fn display(self: anytype) void {
    zgui.indent(.{});
    defer zgui.unindent(.{});

    const info = @typeInfo(@TypeOf(self));
    comptime var max_length = 0;
    inline for (info.Struct.fields) |field| {
        max_length = @max(max_length, field.name.len);
    }
    inline for (info.Struct.fields) |field| {
        // Hide "private" (or hidden) fields - Meaning those starting with an underscore.
        if (!std.mem.startsWith(u8, field.name, "_")) {
            switch (@typeInfo(field.type)) {
                .Enum => zgui.text("{s: <" ++ std.fmt.comptimePrint("{d}", .{max_length}) ++ "} {s}", .{ field.name, @tagName(@field(self, field.name)) }),
                .Struct => {
                    if (zgui.collapsingHeader(field.name ++ " (" ++ @typeName(field.type) ++ ")", .{})) {
                        display(@field(self, field.name));
                    }
                },
                else => zgui.text("{s: <" ++ std.fmt.comptimePrint("{d}", .{max_length}) ++ "} {any}", .{ field.name, @field(self, field.name) }),
            }
        }
    }
}

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

    for (0..self.renderer_texture_views.len) |i| {
        for (0..RendererModule.Renderer.MaxTextures[i]) |j| {
            self.renderer_texture_views[i][j] = d.gctx.createTextureView(d.renderer.texture_arrays[i], .{
                .dimension = .tvdim_2d,
                .base_array_layer = @intCast(j),
                .array_layer_count = 1,
            });
        }
    }

    for (0..self.audio_channels.len) |i| {
        self.audio_channels[i].amplitude_envelope.xv = std.ArrayList(u32).init(self._allocator);
        self.audio_channels[i].amplitude_envelope.yv = std.ArrayList(u32).init(self._allocator);
    }

    return self;
}

pub fn deinit(self: *@This()) void {
    for (0..self.audio_channels.len) |i| {
        self.audio_channels[i].amplitude_envelope.xv.deinit();
        self.audio_channels[i].amplitude_envelope.yv.deinit();
    }

    for (self.renderer_texture_views) |views| {
        for (views) |view|
            self._gctx.releaseResource(view);
    }

    self._gctx.releaseResource(self.vram_texture_view);
    self._gctx.releaseResource(self.vram_texture);

    self._allocator.free(self.pixels);
}

fn add(a: [2]f32, b: [2]f32) [2]f32 {
    return .{ a[0] + b[0], a[1] + b[1] };
}

fn mul(a: [2]f32, b: [2]f32) [2]f32 {
    return .{ a[1] * b[0], a[0] * b[1] };
}

fn reset_hover(self: *@This()) void {
    if (!self.selected_strip_focus) {
        self.selected_strip_index = 0xFFFFFFFF;
    }
    if (!self.selected_vertex_focus) {
        self.selected_vertex = null;
    }
    if (!self.selected_volume_focus) {
        self.selected_volume_index = null;
    }
}

fn textHighlighted(b: bool, comptime fmt: []const u8, args: anytype) void {
    zgui.textColored(if (b) .{ 1.0, 1.0, 1.0, 1.0 } else .{ 1.0, 1.0, 1.0, 0.5 }, fmt, args);
}

fn compare_blocks(_: void, a: BasicBlock, b: BasicBlock) std.math.Order {
    return std.math.order(a.time_spent, b.time_spent);
}
fn compare_blocks_desc(_: void, a: BasicBlock, b: BasicBlock) bool {
    return a.time_spent > b.time_spent;
}

pub fn draw(self: *@This(), d: *Deecy) !void {
    var dc = d.dc;

    self.reset_hover();

    if (zgui.begin("CPU State", .{})) {
        _ = zgui.checkbox("JIT", .{ .v = &d.enable_jit });
        zgui.sameLine(.{});
        if (zgui.button("Clear Cache", .{})) {
            const was_running = d.running;
            if (was_running)
                d.stop();
            try dc.sh4_jit.block_cache.reset();
            if (was_running)
                d.start();
        }
        zgui.text("PC: {X:0>8} - SPC: 0x{X:0>8}", .{ dc.cpu.pc, dc.cpu.spc });
        zgui.text("PR: {X:0>8}", .{dc.cpu.pr});
        zgui.text("SR: ", .{});
        zgui.sameLine(.{});
        textHighlighted(dc.cpu.sr.t, "[T] ", .{});
        zgui.sameLine(.{});
        textHighlighted(dc.cpu.sr.s, "[S] ", .{});
        zgui.sameLine(.{});
        textHighlighted(dc.cpu.sr.bl, "[BL] ", .{});
        zgui.text("IMASK: {d: >2} ", .{dc.cpu.sr.imask});
        zgui.text("GBR: {X:0>8}", .{dc.cpu.gbr});
        zgui.text("VBR: {X:0>8}", .{dc.cpu.vbr});
        zgui.beginGroup();
        for (0..8) |i| {
            zgui.text("R{d: <2}: {X:0>8}", .{ i, dc.cpu.R(@truncate(i)).* });
        }
        zgui.endGroup();
        zgui.sameLine(.{});
        zgui.beginGroup();
        for (8..16) |i| {
            zgui.text("R{d: <2}: {X:0>8}", .{ i, dc.cpu.R(@truncate(i)).* });
        }
        zgui.endGroup();

        const range = 16; // In bytes.
        const pc = dc.cpu.pc & 0x1FFFFFFF;
        //              In RAM                                                                           In BootROM
        var addr = (if (pc >= 0x0C000000) std.math.clamp(pc, 0x0C000000 + range / 2, 0x0D000000 - range) else std.math.clamp(pc, 0x00000000 + range / 2, 0x02000000 - range)) - range / 2;
        const end_addr = addr + range;
        while (addr < end_addr) {
            const disassembly = try sh4_disassembly.disassemble(.{ .value = dc.cpu.read16(@intCast(addr)) }, self._allocator);
            zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == pc) ">" else " ", disassembly });
            addr += 2;
        }

        if (zgui.button(if (d.running) "Pause" else "Run", .{ .w = 200.0 })) {
            if (d.running) {
                d.stop();
            } else {
                d.start();
            }
        }

        zgui.beginDisabled(.{ .disabled = d.running });
        if (zgui.button("Step", .{ .w = 200.0 })) {
            _ = try dc.tick(1);
        }
        if (zgui.button("Skip", .{ .w = 200.0 })) {
            dc.cpu.pc += 2;
        }
        zgui.endDisabled();

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
            const enabled = ((TSTR >> i) & 1) == 1;
            textHighlighted(enabled, "Timer {d:0>1}: {X:0>8} / {X:0>8}", .{
                i,
                dc.cpu.read_p4_register(u32, timers[i].counter),
                dc.cpu.read_p4_register(u32, timers[i].constant),
            });
            if (i == 2) {
                textHighlighted(enabled, "  TPSC {X:0>1} CKEG {X:0>1} UNIE {X:0>1} UNF {X:0>1} ICPE {X:0>1} ICPF {X:0>1}", .{ control.tpsc, control.ckeg, control.unie, control.unf, control.icpe, control.icpf });
            } else {
                textHighlighted(enabled, "  TPSC {X:0>1} CKEG {X:0>1} UNIE {X:0>1} UNF {X:0>1}", .{ control.tpsc, control.ckeg, control.unie, control.unf });
            }
            zgui.endGroup();
        }

        zgui.beginGroup();
        zgui.text("IPRA: {X:0>4}", .{dc.cpu.read_p4_register(u16, .IPRA)});
        display(dc.cpu.read_p4_register(sh4.P4.IPRA, .IPRA));
        zgui.endGroup();
        zgui.sameLine(.{});
        zgui.beginGroup();
        zgui.text("IPRB: {X:0>4}", .{dc.cpu.read_p4_register(u16, .IPRB)});
        display(dc.cpu.read_p4_register(sh4.P4.IPRB, .IPRB));
        zgui.endGroup();
        zgui.sameLine(.{});
        zgui.beginGroup();
        zgui.text("IPRC: {X:0>4}", .{dc.cpu.read_p4_register(u16, .IPRC)});
        display(dc.cpu.read_p4_register(sh4.P4.IPRC, .IPRC));
        zgui.endGroup();
    }
    zgui.end();

    if (zgui.begin("CPU JIT", .{})) {
        zgui.text("Block statistics", .{});
        if (BasicBlock.EnableInstrumentation) {
            const max = 50;
            const static = struct {
                var top: std.PriorityQueue(BasicBlock, void, compare_blocks) = undefined;
                var sorted: [max]usize = .{0} ** max;
                var initialized: bool = false;
            };
            zgui.beginDisabled(.{ .disabled = d.running });
            if (zgui.button("Refresh", .{})) {
                if (!static.initialized) {
                    static.top =
                        std.PriorityQueue(BasicBlock, void, compare_blocks).init(dc._allocator, {});
                    static.initialized = true;
                } else {
                    while (static.top.count() > 0) _ = static.top.remove();
                }
                for (0..dc.sh4_jit.block_cache.blocks.len) |i| {
                    if (dc.sh4_jit.block_cache.blocks[i]) |block| {
                        if (block.call_count > 0 and (static.top.count() < max or static.top.peek().?.time_spent < block.time_spent)) {
                            static.top.add(block) catch unreachable;
                        }
                        if (static.top.count() > max) {
                            _ = static.top.remove();
                        }
                    }
                }
                std.mem.sort(BasicBlock, static.top.items, {}, comptime compare_blocks_desc);
            }
            zgui.sameLine(.{});
            if (zgui.button("Reset", .{})) {
                for (0..dc.sh4_jit.block_cache.blocks.len) |i| {
                    if (dc.sh4_jit.block_cache.blocks[i]) |*block| {
                        block.time_spent = 0;
                        block.call_count = 0;
                    }
                }
            }
            zgui.endDisabled();

            for (static.top.items) |block| {
                zgui.text("Block {X:0>6} ({d}, {d}): {d}ms - {d}ns ({d})", .{
                    block.start_addr,
                    block.len,
                    block.cycles,
                    @divTrunc(block.time_spent, 1_000_000),
                    @divTrunc(block.time_spent, block.call_count),
                    block.call_count,
                });
                for (0..block.len) |i| {
                    const addr: u32 = block.start_addr + @as(u32, @intCast(2 * i));
                    const instr = dc.cpu.read16(addr);
                    const op = sh4.sh4_instructions.Opcodes[sh4.sh4_instructions.JumpTable[instr]];
                    zgui.text("{s} {X:0>6}: {s}", .{ if (op.use_fallback()) "!" else " ", addr, try sh4_disassembly.disassemble(@bitCast(instr), dc._allocator) });
                }
            }
        } else {
            zgui.text("JIT Instrumentation was disabled for this build.", .{});
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
        zgui.sameLine(.{});
        _ = zgui.checkbox("Debug Trace", .{ .v = &dc.aica.arm_debug_trace });
        zgui.sameLine(.{});
        if (zgui.button("Dump Memory", .{})) {
            dc.aica.dump_wave_memory();
        }
        zgui.text("State: {s} - Run.: {any}", .{ @tagName(dc.aica.arm7.cpsr.m), dc.aica.arm7.running });
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
        const pc = 0x00800000 + @max(4, dc.aica.arm7.pc() & dc.aica.arm7.memory_address_mask) - 4;
        var addr = std.math.clamp(pc - range / 2, 0x00800000, 0x00A00000 - range);
        const end_addr = addr + range;
        while (addr < end_addr) {
            const disassembly = arm7.ARM7.disassemble(dc.aica.read_mem(u32, addr));
            zgui.text("[{X: >6}] {s} {s}", .{ addr - 0x00800000, if (addr == pc) ">" else " ", disassembly });
            addr += 4;
        }
    }
    zgui.end();

    if (zgui.begin("AICA", .{})) {
        zgui.text("Master Volume: {X}", .{dc.aica.debug_read_reg(u32, .MasterVolume) & 0xF});
        zgui.text("Ring buffer address: 0x{X:0>8}", .{dc.aica.debug_read_reg(u32, .RingBufferAddress)});
        const channel_info_req = dc.aica.debug_read_reg(AICAModule.ChannelInfoReq, .ChannelInfoReq);
        zgui.text("Channel Select: {d: >2} ({s})", .{ channel_info_req.monitor_select, if (channel_info_req.amplitude_or_filter_select == 1) "filter" else "amplitude" });
        zgui.text("SCIEB: {X:0>8}, SCIPD: {X:0>8}", .{ dc.aica.debug_read_reg(u32, .SCIEB), dc.aica.debug_read_reg(u32, .SCIPD) });
        zgui.text("MCIEB: {X:0>8}, MCIPD: {X:0>8}", .{ dc.aica.debug_read_reg(u32, .MCIEB), dc.aica.debug_read_reg(u32, .MCIPD) });
        zgui.text("INTRequest: {X:0>8}", .{dc.aica.debug_read_reg(u32, .INTRequest)});
        if (zgui.collapsingHeader("Timers", .{ .default_open = true })) {
            const timer_registers = [_]AICAModule.AICARegister{ .TACTL_TIMA, .TBCTL_TIMB, .TCCTL_TIMC };
            inline for (0..3) |i| {
                const number = std.fmt.comptimePrint("{d}", .{i});
                const timer = dc.aica.debug_read_reg(AICAModule.TimerControl, timer_registers[i]);
                const mask: u32 = @as(u32, 1) << @intCast(6 + i);
                const interrupt_enabled = (dc.aica.debug_read_reg(u32, .SCIEB) & mask) != 0;
                if (interrupt_enabled) {
                    zgui.textColored(.{ 0.0, 1.0, 0.0, 1.0 }, "!", .{});
                } else {
                    zgui.textColored(.{ 1.0, 0.0, 0.0, 1.0 }, ".", .{});
                }
                zgui.sameLine(.{});
                zgui.text("Timer " ++ number ++ ": Prescale: {X:0>1} - Value: {X:0>2}", .{ timer.prescale, timer.value });
            }
        }
        if (zgui.plot.beginPlot("Sample Buffer", .{ .flags = zgui.plot.Flags.canvas_only })) {
            //zgui.plot.setupAxis(.x1, .{ .label = "time" });
            zgui.plot.setupAxisLimits(.y1, .{ .min = std.math.minInt(i16), .max = std.math.maxInt(i16) });
            // zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = 10_000 });
            // zgui.plot.setupAxisLimits(.y1, .{ .min = 0, .max = 0x400 });
            // zgui.plot.setupLegend(.{ .south = false, .west = false }, .{});
            zgui.plot.setupFinish();
            zgui.plot.plotLineValues("samples", i32, .{ .v = &dc.aica.sample_buffer });
            zgui.plot.plotLine("sample_read_offset", i32, .{ .xv = &[_]i32{ @intCast(dc.aica.sample_read_offset), @intCast(dc.aica.sample_read_offset) }, .yv = &[_]i32{ 0, std.math.maxInt(i16) } });
            zgui.plot.plotLine("sample_write_offset", i32, .{ .xv = &[_]i32{ @intCast(dc.aica.sample_write_offset), @intCast(dc.aica.sample_write_offset) }, .yv = &[_]i32{ 0, -std.math.maxInt(i16) } });
            zgui.plot.endPlot();
        }
        _ = zgui.checkbox("Show disabled channels", .{ .v = &self.show_disabled_channels });
        inline for (0..64) |i| {
            const channel = dc.aica.get_channel_registers(@intCast(i));
            zgui.pushPtrId(channel);
            defer zgui.popId();
            const state = dc.aica.channel_states[i];
            const time = std.time.milliTimestamp();
            if (self.show_disabled_channels or state.playing or channel.play_control.key_on_bit) {
                const number = std.fmt.comptimePrint("{d}", .{i});
                if (zgui.collapsingHeader("Channel " ++ number, .{ .default_open = true })) {
                    _ = zgui.checkbox("Mute (Debug)", .{ .v = &dc.aica.channel_states[i].debug.mute });
                    const start_addr = channel.sample_address();

                    zgui.textColored(if (channel.play_control.key_on_bit) .{ 0.0, 1.0, 0.0, 1.0 } else .{ 1.0, 0.0, 0.0, 1.0 }, "KeyOn: {s: >3}", .{if (channel.play_control.key_on_bit) "Yes" else "No"});
                    zgui.sameLine(.{});
                    zgui.text(" - {s} - ", .{@tagName(channel.play_control.sample_format)});
                    zgui.sameLine(.{});
                    zgui.textColored(if (channel.play_control.sample_loop) .{ 0.0, 1.0, 0.0, 1.0 } else .{ 1.0, 0.0, 0.0, 1.0 }, "Loop: {s: >3}", .{if (channel.play_control.sample_loop) "Yes" else "No"});
                    zgui.text("Addr: {X: >6} - Loop: {X:0>4} - {X:0>4}", .{
                        start_addr,
                        channel.loop_start,
                        channel.loop_end,
                    });
                    zgui.text("FNS: {X:0>3} - Oct: {X:0>2}", .{ channel.sample_pitch_rate.fns, channel.sample_pitch_rate.oct });
                    zgui.text("DIPAN: {X:0>2} - DISDL: {X:0>1}", .{ channel.direct_pan_vol_send.pan, channel.direct_pan_vol_send.volume });
                    zgui.text("DSP Vol: {X:0>1} - DSP Chan: {X:0>1}", .{ channel.dps_channel_send.level, channel.dps_channel_send.channel });
                    zgui.textColored(if (state.playing) .{ 0.0, 1.0, 0.0, 1.0 } else .{ 1.0, 0.0, 0.0, 1.0 }, "{s: >7}", .{if (state.playing) "Playing" else "Stopped"});
                    zgui.sameLine(.{});
                    zgui.text("PlayPos: {X: >6} - ", .{state.play_position});
                    zgui.sameLine(.{});
                    zgui.textColored(if (state.loop_end_flag) .{ 0.0, 1.0, 0.0, 1.0 } else .{ 1.0, 0.0, 0.0, 1.0 }, "LoodEnd: {s: >3}", .{if (state.loop_end_flag) "Yes" else "No"});
                    const effective_rate = AICAModule.AICAChannelState.compute_effective_rate(channel, switch (state.amp_env_state) {
                        .Attack => channel.amp_env_1.attack_rate,
                        .Decay => channel.amp_env_1.decay_rate,
                        .Sustain => channel.amp_env_1.sustain_rate,
                        .Release => channel.amp_env_2.release_rate,
                    });
                    zgui.textColored(if (!channel.env_settings.voff) .{ 0.0, 1.0, 0.0, 1.0 } else .{ 1.0, 0.0, 0.0, 1.0 }, "AmpEnv ", .{});
                    zgui.sameLine(.{});
                    zgui.text("{s: >7} - level: {X: >4} - rate: {X: >2}", .{ @tagName(state.amp_env_state), state.amp_env_level, effective_rate });
                    zgui.textColored(if (!channel.env_settings.lpoff) .{ 0.0, 1.0, 0.0, 1.0 } else .{ 1.0, 0.0, 0.0, 1.0 }, "FilEnv ", .{});
                    zgui.sameLine(.{});
                    zgui.text("{s: >7} - level: {X: >4}", .{ @tagName(state.filter_env_state), state.filter_env_level });
                    zgui.text("ALFOS {X: >1} ALFOWS {X: >1} PLFOS {X: >1} PLFOWS {X: >1} F {X: >2} R {any}", .{
                        channel.lfo_control.amplitude_modulation_depth,
                        @intFromEnum(channel.lfo_control.amplitude_modulation_waveform),
                        channel.lfo_control.pitch_modulation_depth,
                        @intFromEnum(channel.lfo_control.pitch_modulation_waveform),
                        channel.lfo_control.frequency,
                        channel.lfo_control.reset,
                    });
                    var loop_size = if (channel.play_control.sample_loop) channel.loop_end - channel.loop_start else channel.loop_end;
                    if (loop_size == 0) loop_size = 2048;
                    if (channel.play_control.sample_format == .i16) {
                        if (zgui.plot.beginPlot("Samples##" ++ number, .{ .flags = zgui.plot.Flags.canvas_only })) {
                            zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = @floatFromInt(loop_size / 2) });
                            zgui.plot.setupAxisLimits(.y1, .{ .min = std.math.minInt(i16), .max = std.math.maxInt(i16) });
                            zgui.plot.setupFinish();
                            zgui.plot.plotLineValues("samples", i16, .{ .v = @as([*]const i16, @alignCast(@ptrCast(&dc.aica.wave_memory[start_addr])))[0..loop_size] });
                            zgui.plot.plotLine("play_position", i32, .{ .xv = &[_]i32{ @intCast(state.play_position), @intCast(state.play_position) }, .yv = &[_]i32{ -std.math.maxInt(i16), std.math.maxInt(i16) } });
                            zgui.plot.endPlot();
                        }
                    } else if (channel.play_control.sample_format == .i8) {
                        if (zgui.plot.beginPlot("Samples##" ++ number, .{ .flags = zgui.plot.Flags.canvas_only })) {
                            zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = @floatFromInt(loop_size) });
                            zgui.plot.setupAxisLimits(.y1, .{ .min = std.math.minInt(i8), .max = std.math.maxInt(i8) });
                            zgui.plot.setupFinish();
                            zgui.plot.plotLineValues("samples", i8, .{ .v = @as([*]const i8, @alignCast(@ptrCast(&dc.aica.wave_memory[start_addr])))[0..loop_size] });
                            zgui.plot.plotLine("play_position", i32, .{ .xv = &[_]i32{ @intCast(state.play_position), @intCast(state.play_position) }, .yv = &[_]i32{ -std.math.maxInt(i8), std.math.maxInt(i8) } });
                            zgui.plot.endPlot();
                        }
                    }
                    if (d.running) {
                        if (time - self.audio_channels[i].amplitude_envelope.start_time > 10_000) {
                            self.audio_channels[i].amplitude_envelope.xv.clearRetainingCapacity();
                            self.audio_channels[i].amplitude_envelope.yv.clearRetainingCapacity();
                        }
                        if (self.audio_channels[i].amplitude_envelope.xv.items.len > 0) {
                            try self.audio_channels[i].amplitude_envelope.xv.append(@intCast(time - self.audio_channels[i].amplitude_envelope.start_time));
                        } else {
                            self.audio_channels[i].amplitude_envelope.start_time = time;
                            try self.audio_channels[i].amplitude_envelope.xv.append(0);
                        }
                        try self.audio_channels[i].amplitude_envelope.yv.append(state.amp_env_level);
                    } else {
                        if (self.audio_channels[i].amplitude_envelope.xv.items.len > 0)
                            self.audio_channels[i].amplitude_envelope.start_time = time - self.audio_channels[i].amplitude_envelope.xv.items[self.audio_channels[i].amplitude_envelope.xv.items.len - 1];
                    }
                    if (zgui.plot.beginPlot("Envelope##" ++ number, .{ .flags = zgui.plot.Flags.canvas_only })) {
                        zgui.plot.setupAxis(.x1, .{ .label = "time" });
                        zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = 10_000 });
                        zgui.plot.setupAxisLimits(.y1, .{ .min = 0, .max = 0x400 });
                        // zgui.plot.setupLegend(.{ .south = false, .west = false }, .{});
                        zgui.plot.setupFinish();
                        zgui.plot.plotLine("attenuation", u32, .{ .xv = self.audio_channels[i].amplitude_envelope.xv.items, .yv = self.audio_channels[i].amplitude_envelope.yv.items });
                        zgui.plot.endPlot();
                    }
                }
            } else {
                self.audio_channels[i].amplitude_envelope.xv.clearRetainingCapacity();
                self.audio_channels[i].amplitude_envelope.yv.clearRetainingCapacity();
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
            zgui.text("[{X:0>8}] {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}  {c}{c}{c}{c}{c}{c}{c}{c}", .{
                addr,
                dc.cpu.read8(@intCast(addr)),
                dc.cpu.read8(@intCast(addr + 1)),
                dc.cpu.read8(@intCast(addr + 2)),
                dc.cpu.read8(@intCast(addr + 3)),
                dc.cpu.read8(@intCast(addr + 4)),
                dc.cpu.read8(@intCast(addr + 5)),
                dc.cpu.read8(@intCast(addr + 6)),
                dc.cpu.read8(@intCast(addr + 7)),
                printable_ascii(dc.cpu.read8(@intCast(addr))),
                printable_ascii(dc.cpu.read8(@intCast(addr + 1))),
                printable_ascii(dc.cpu.read8(@intCast(addr + 2))),
                printable_ascii(dc.cpu.read8(@intCast(addr + 3))),
                printable_ascii(dc.cpu.read8(@intCast(addr + 4))),
                printable_ascii(dc.cpu.read8(@intCast(addr + 5))),
                printable_ascii(dc.cpu.read8(@intCast(addr + 6))),
                printable_ascii(dc.cpu.read8(@intCast(addr + 7))),
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
        if (zgui.collapsingHeader("SPG", .{ .frame_padding = true })) {
            zgui.indent(.{});
            zgui.text("SPG_HBLANK_INT: {X:0>8}", .{dc.gpu._get_register(u32, .SPG_HBLANK_INT).*});
            zgui.text("SPG_VBLANK_INT: {X:0>8}", .{dc.gpu._get_register(u32, .SPG_VBLANK_INT).*});
            zgui.text("SPG_CONTROL:    {X:0>8}", .{dc.gpu._get_register(u32, .SPG_CONTROL).*});
            zgui.text("SPG_HBLANK:     {X:0>8}", .{dc.gpu._get_register(u32, .SPG_HBLANK).*});
            zgui.text("SPG_VBLANK:     {X:0>8}", .{dc.gpu._get_register(u32, .SPG_VBLANK).*});
            zgui.text("SPG_WIDTH:      {X:0>8}", .{dc.gpu._get_register(u32, .SPG_WIDTH).*});
            zgui.text("SPG_STATUS:     {X:0>8}", .{dc.gpu._get_register(u32, .SPG_STATUS).*});
            zgui.unindent(.{});
        }
        const ISP_BACKGND_D = dc.gpu._get_register(u32, .ISP_BACKGND_D).*;
        const ISP_BACKGND_T = dc.gpu._get_register(u32, .ISP_BACKGND_T).*;
        zgui.text("ISP_BACKGND_D: {d: >8.2} / {d: >8.2}", .{ ISP_BACKGND_D, @as(f32, @bitCast(ISP_BACKGND_D)) });
        zgui.text("ISP_BACKGND_T: {X:0>8}", .{ISP_BACKGND_T});

        const FB_C_SOF = dc.gpu._get_register(u32, .FB_C_SOF).*;
        const FB_W_CTRL = dc.gpu._get_register(Holly.FB_W_CTRL, .FB_W_CTRL).*;
        const FB_W_SOF1 = dc.gpu._get_register(u32, .FB_W_SOF1).*;
        const FB_W_SOF2 = dc.gpu._get_register(u32, .FB_W_SOF2).*;
        const FB_W_LINESTRIDE = dc.gpu._get_register(u32, .FB_W_LINESTRIDE).*;
        const FB_R_CTRL = dc.gpu._get_register(Holly.FB_R_CTRL, .FB_R_CTRL).*;
        const FB_R_SOF1 = dc.gpu._get_register(u32, .FB_R_SOF1).*;
        const FB_R_SOF2 = dc.gpu._get_register(u32, .FB_R_SOF2).*;
        const FB_R_SIZE = dc.gpu._get_register(Holly.FB_R_SIZE, .FB_R_SIZE).*;
        const FB_X_CLIP = dc.gpu._get_register(Holly.FB_CLIP, .FB_X_CLIP).*;
        const FB_Y_CLIP = dc.gpu._get_register(Holly.FB_CLIP, .FB_Y_CLIP).*;
        const VO_CONTROL = dc.gpu._get_register(Holly.VO_CONTROL, .VO_CONTROL).*;
        zgui.text("FB_C_SOF:  0x{X:0>8}", .{FB_C_SOF});
        zgui.text("FB_W_CTRL: 0x{X:0>8} - {any}", .{ @as(u32, @bitCast(FB_W_CTRL)), FB_W_CTRL });
        zgui.text("FB_W_SOF1: 0x{X:0>8}", .{FB_W_SOF1});
        zgui.text("FB_W_SOF2: 0x{X:0>8}", .{FB_W_SOF2});
        zgui.text("FB_W_LINESTRIDE: 0x{X:0>8}", .{FB_W_LINESTRIDE});
        zgui.text("FB_X_CLIP: [{d}, {d}]", .{ FB_X_CLIP.min, FB_X_CLIP.max });
        zgui.text("FB_Y_CLIP: [{d}, {d}]", .{ FB_Y_CLIP.min, FB_Y_CLIP.max });
        zgui.text("FB_R_CTRL: 0x{X:0>8} - {any}", .{ @as(u32, @bitCast(FB_R_CTRL)), FB_R_CTRL });
        zgui.text("FB_R_SOF1: 0x{X:0>8}", .{FB_R_SOF1});
        zgui.text("FB_R_SOF2: 0x{X:0>8}", .{FB_R_SOF2});
        zgui.text("FB_R_SOF2: 0x{X:0>8} - {any}", .{ @as(u32, @bitCast(FB_R_SIZE)), FB_R_SIZE });
        zgui.text("VO_CONTROL: {any}", .{VO_CONTROL});

        var buffer: [256]u8 = .{0} ** 256;

        // NOTE: We're looking at the last list used during a START_RENDER.
        if (zgui.collapsingHeader("Polygons", .{ .frame_padding = true })) {
            zgui.indent(.{});
            inline for (.{ Holly.ListType.Opaque, Holly.ListType.Translucent, Holly.ListType.PunchThrough }) |list_type| {
                const list = d.renderer.ta_lists.get_list(list_type);
                const name = @tagName(@as(Holly.ListType, list_type));
                const header = try std.fmt.bufPrintZ(&buffer, name ++ " ({d})###" ++ name, .{list.vertex_strips.items.len});

                if (zgui.collapsingHeader(header, .{})) {
                    zgui.text("Strips: {d}, Vertices: {d}", .{ list.vertex_strips.items.len, list.vertex_parameters.items.len });
                    for (list.vertex_strips.items, 0..) |strip, idx| {
                        const strip_header = try std.fmt.bufPrintZ(&buffer, "  {s} ({d}) - {s}###strip_{d}", .{
                            @tagName(strip.polygon.tag()),
                            strip.vertex_parameter_count,
                            @tagName(strip.polygon.tsp_instruction().texture_shading_instruction),
                            idx,
                        });
                        {
                            zgui.beginGroup();
                            const node_open = zgui.treeNodeFlags(strip_header, .{
                                .open_on_double_click = true,
                                .open_on_arrow = true,
                            });
                            if (zgui.isItemClicked(.left) and !zgui.isItemToggledOpen()) {
                                self.selected_strip_focus = true;
                                self.selected_strip_list = list_type;
                                self.selected_strip_index = @intCast(idx);
                            }
                            if (node_open) {
                                if (idx < list.vertex_strips.items.len) {
                                    self.display_strip_info(&d.renderer, &list.vertex_strips.items[idx]);
                                    for (strip.vertex_parameter_index..(strip.vertex_parameter_index + strip.vertex_parameter_count)) |i| {
                                        if (i < list.vertex_parameters.items.len)
                                            self.display_vertex_data(&list.vertex_parameters.items[i]);
                                    }
                                }
                                zgui.treePop();
                            }
                            zgui.endGroup();
                        }
                        if (zgui.isItemClicked(.right)) {
                            self.selected_strip_focus = false;
                        }
                        if (!self.selected_strip_focus and zgui.isItemHovered(.{})) {
                            self.selected_strip_list = list_type;
                            self.selected_strip_index = @intCast(idx);
                        }
                    }
                }
            }
            zgui.unindent(.{});
        }
        if (zgui.collapsingHeader("Modifier Volumes", .{ .frame_padding = true })) {
            zgui.indent(.{});
            // NOTE: By the time we get there, the renderer took the volumes for itself (rather than copying them).
            {
                const list = d.renderer.ta_lists.opaque_modifier_volumes;
                const header = try std.fmt.bufPrintZ(&buffer, "Opaque ({d})###OMV", .{list.items.len});
                if (zgui.collapsingHeader(header, .{})) {
                    for (list.items, 0..) |vol, idx| {
                        zgui.text("  {any}", .{vol});
                        if (zgui.isItemClicked(.left)) {
                            self.selected_volume_focus = true;
                            self.selected_volume_list = .OpaqueModifierVolume;
                            self.selected_volume_index = @intCast(idx);
                        }
                        if (zgui.isItemClicked(.right)) {
                            self.selected_volume_focus = false;
                        }
                        if (!self.selected_volume_focus and zgui.isItemHovered(.{})) {
                            self.selected_volume_list = .OpaqueModifierVolume;
                            self.selected_volume_index = @intCast(idx);
                        }
                    }
                }
            }
            {
                const list = d.renderer.ta_lists.translucent_modifier_volumes;
                const header = try std.fmt.bufPrintZ(&buffer, "Translucent ({d})###TMV", .{list.items.len});
                if (zgui.collapsingHeader(header, .{})) {
                    for (list.items, 0..) |vol, idx| {
                        zgui.text("  {any}", .{vol});
                        if (zgui.isItemClicked(.left)) {
                            self.selected_volume_focus = true;
                            self.selected_volume_list = .TranslucentModifierVolume;
                            self.selected_volume_index = @intCast(idx);
                        }
                        if (zgui.isItemClicked(.right)) {
                            self.selected_volume_focus = false;
                        }
                        if (!self.selected_volume_focus and zgui.isItemHovered(.{})) {
                            self.selected_volume_list = .TranslucentModifierVolume;
                            self.selected_volume_index = @intCast(idx);
                        }
                    }
                }
            }
            zgui.unindent(.{});
        }

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
                        const color: Color16 = .{ .value = dc.cpu.read16(addr) };
                        self.pixels[4 * i + 0] = @as(u8, @intCast(color.arbg1555.r)) << 3;
                        self.pixels[4 * i + 1] = @as(u8, @intCast(color.arbg1555.g)) << 3;
                        self.pixels[4 * i + 2] = @as(u8, @intCast(color.arbg1555.b)) << 3;
                        self.pixels[4 * i + 3] = 255; // FIXME: Not really.
                        i += 1;
                    },
                    0x1 => { // 565 RGB 16 bit
                        const color: Color16 = .{ .value = dc.cpu.read16(addr) };
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
        zgui.text("PT_ALPHA_REF: {d: >4.2}", .{d.renderer.pt_alpha_ref});
        zgui.text("FPU_SHAD_SCALE: {d: >4.2}", .{d.renderer.fpu_shad_scale});
        if (zgui.collapsingHeader("Fog", .{})) {
            zgui.text("Fog Density: {d: >4.2}", .{d.renderer.fog_density});
            _ = zgui.colorEdit4("Fog Pal", .{ .col = @as([*]f32, @ptrCast(&d.renderer.fog_col_pal))[0..4] });
            _ = zgui.colorEdit4("Fog Vert", .{ .col = @as([*]f32, @ptrCast(&d.renderer.fog_col_vert))[0..4] });
            for (0..128) |i| {
                zgui.text("Fog {d: >3}: {X:0>4}", .{ i, d.renderer.fog_lut[i] });
            }
        }
        if (zgui.collapsingHeader("Textures", .{})) {
            const static = struct {
                var index: i32 = 0;
                var scale: f32 = 512.0 / 8.0;
                var size: i32 = 0;
            };
            if (zgui.button("Clear Texture Cache", .{})) {
                for (0..d.renderer.texture_metadata.len) |size_index| {
                    for (0..d.renderer.texture_metadata[size_index].len) |i| {
                        d.renderer.texture_metadata[size_index][i].status = .Invalid;
                    }
                }
            }
            if (zgui.inputInt("Size", .{
                .v = &static.size,
                .step = 1,
            })) {
                static.size = std.math.clamp(static.size, 0, @as(i32, @intCast(self.renderer_texture_views.len - 1)));
                static.scale = @as(f32, 512) / @as(f32, @floatFromInt((@as(u32, 8) << @intCast(static.size))));
                static.index = std.math.clamp(static.index, 0, RendererModule.Renderer.MaxTextures[@intCast(static.size)] - 1);
            }
            zgui.sameLine(.{});
            zgui.text("{d: >3}x{d: >3}", .{ @as(u32, 8) << @intCast(static.size), @as(u32, 8) << @intCast(static.size) });
            if (zgui.inputInt("Index", .{ .v = &static.index, .step = 1 })) {
                static.index = std.math.clamp(static.index, 0, RendererModule.Renderer.MaxTextures[@intCast(static.size)] - 1);
            }
            if (zgui.dragFloat("Scale", .{ .v = &static.scale, .min = 1.0, .max = 8.0, .speed = 0.1 })) {
                static.scale = std.math.clamp(static.scale, 1.0, 8.0);
            }
            const tex_id = d.gctx.lookupResource(self.renderer_texture_views[@intCast(static.size)][@intCast(static.index)]).?;
            zgui.image(tex_id, .{ .w = static.scale * @as(f32, @floatFromInt(@as(u32, 8) << @intCast(static.size))), .h = static.scale * @as(f32, @floatFromInt(@as(u32, 8) << @intCast(static.size))) });
            if (zgui.collapsingHeader("Parameter Control Word", .{ .default_open = true })) {
                const control_word = d.renderer.texture_metadata[@intCast(static.size)][@intCast(static.index)].control_word;
                display(control_word);
            }
            if (zgui.collapsingHeader("TSP Instruction", .{ .default_open = true })) {
                const tsp_instruction = d.renderer.texture_metadata[@intCast(static.size)][@intCast(static.index)].tsp_instruction;
                display(tsp_instruction);
            }
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

    if (zgui.begin("Interrupts", .{})) {
        inline for (@typeInfo(HardwareRegisters.SB_ISTNRM).Struct.fields) |field| {
            if (zgui.button("Trigger " ++ field.name ++ " Interrupt", .{})) {
                comptime var val: HardwareRegisters.SB_ISTNRM = .{};
                @field(val, field.name) = 1;
                dc.raise_normal_interrupt(val);
            }
        }

        zgui.separator();

        inline for (@typeInfo(HardwareRegisters.SB_ISTEXT).Struct.fields) |field| {
            if (zgui.button("Trigger " ++ field.name ++ " Interrupt", .{})) {
                comptime var val: HardwareRegisters.SB_ISTEXT = .{};
                @field(val, field.name) = 1;
                dc.raise_external_interrupt(val);
            }
        }
    }
    zgui.end();

    self.draw_overlay(d);
}

fn draw_overlay(self: *@This(), d: *Deecy) void {
    const draw_list = zgui.getBackgroundDrawList();
    // TODO: We only support the "Centered" display mode for now.
    const size = [2]f32{
        @floatFromInt(d.renderer.resolution.width),
        @floatFromInt(d.renderer.resolution.height),
    };
    const scale = [2]f32{ size[0] / 640.0, size[1] / 480.0 }; // Scale of inner render compared to native DC resolution.
    const min = [2]f32{
        @as(f32, @floatFromInt(self._gctx.swapchain_descriptor.width)) / 2.0 - size[0] / 2.0,
        @as(f32, @floatFromInt(self._gctx.swapchain_descriptor.height)) / 2.0 - size[1] / 2.0,
    };

    if (self.selected_strip_list == .Opaque or self.selected_strip_list == .PunchThrough or self.selected_strip_list == .Translucent) {
        const list = d.renderer.ta_lists.get_list(self.selected_strip_list);
        if (self.selected_strip_index < list.vertex_strips.items.len) {
            const parameters = list.vertex_parameters.items;
            const strip = &list.vertex_strips.items[self.selected_strip_index];
            switch (strip.polygon) {
                .Sprite => |_| {
                    for (strip.vertex_parameter_index..strip.vertex_parameter_index + strip.vertex_parameter_count) |i| {
                        const pos = parameters[i].sprite_positions();
                        draw_list.addTriangle(.{
                            .p1 = add(mul(scale, pos[0][0..2].*), min),
                            .p2 = add(mul(scale, pos[3][0..2].*), min),
                            .p3 = add(mul(scale, pos[1][0..2].*), min),
                            .col = 0xFFFF00FF,
                            .thickness = 1.0,
                        });
                        draw_list.addTriangle(.{
                            .p1 = add(mul(scale, pos[3][0..2].*), min),
                            .p2 = add(mul(scale, pos[2][0..2].*), min),
                            .p3 = add(mul(scale, pos[1][0..2].*), min),
                            .col = 0xFFFF00FF,
                            .thickness = 1.0,
                        });
                    }
                },
                else => {
                    for (strip.vertex_parameter_index..strip.vertex_parameter_index + strip.vertex_parameter_count - 2) |i| {
                        const p1 = add(mul(scale, parameters[i].position()[0..2].*), min);
                        const p2 = add(mul(scale, parameters[i + 1].position()[0..2].*), min);
                        const p3 = add(mul(scale, parameters[i + 2].position()[0..2].*), min);
                        draw_list.addTriangle(.{ .p1 = p1, .p2 = p2, .p3 = p3, .col = 0xFFFF00FF, .thickness = 1.0 });
                    }
                },
            }
        }
    }
    if (self.selected_vertex) |vertex| {
        draw_list.addCircleFilled(.{ .p = add(mul(scale, vertex), min), .r = 5.0, .col = 0xFF4000FF });
    }
    if (self.selected_volume_index) |idx| {
        const list = switch (self.selected_volume_list) {
            .OpaqueModifierVolume => d.renderer.ta_lists.opaque_modifier_volumes.items,
            .TranslucentModifierVolume => d.renderer.ta_lists.translucent_modifier_volumes.items,
            else => unreachable,
        };
        if (idx < list.len) {
            const volume = list[idx];
            for (0..volume.triangle_count) |i| {
                const p1 = add(mul(scale, d.renderer.modifier_volume_vertices.items[3 * volume.first_triangle_index + 3 * i + 0][0..2].*), min);
                const p2 = add(mul(scale, d.renderer.modifier_volume_vertices.items[3 * volume.first_triangle_index + 3 * i + 1][0..2].*), min);
                const p3 = add(mul(scale, d.renderer.modifier_volume_vertices.items[3 * volume.first_triangle_index + 3 * i + 2][0..2].*), min);
                draw_list.addTriangle(.{ .p1 = p1, .p2 = p2, .p3 = p3, .col = 0xFF0000FF, .thickness = 1.0 });
            }
        }
    }
}

fn display_strip_info(self: *@This(), renderer: *const RendererModule.Renderer, strip: *const Holly.VertexStrip) void {
    zgui.pushPtrId(strip);
    defer zgui.popId();

    var buffer: [128]u8 = undefined;

    const control_word = strip.polygon.control_word();
    const isp_tsp = strip.polygon.isp_tsp_instruction();
    const tsp = strip.polygon.tsp_instruction();
    // TODO: Display some actually useful information :)
    {
        const header = std.fmt.bufPrintZ(&buffer, "Control Word:    {X:0>8}##ControlWord", .{@as(u32, @bitCast(control_word))}) catch unreachable;
        if (zgui.collapsingHeader(header, .{})) {
            display(control_word);
        }
    }
    {
        const header = std.fmt.bufPrintZ(&buffer, "ISP TSP:         {X:0>8}##ISPTSP", .{@as(u32, @bitCast(isp_tsp))}) catch unreachable;
        if (zgui.collapsingHeader(header, .{})) {
            display(isp_tsp);
        }
    }
    {
        const header = std.fmt.bufPrintZ(&buffer, "TSP:             {X:0>8}##TSP", .{@as(u32, @bitCast(tsp))}) catch unreachable;
        if (zgui.collapsingHeader(header, .{})) {
            display(tsp);
        }
    }
    if (control_word.obj_control.texture == 1) {
        const texture_control_word = strip.polygon.texture_control();
        zgui.text("Texture Control: {X:0>8}", .{@as(u32, @bitCast(texture_control_word))});
        if (renderer.get_texture_view(texture_control_word, tsp)) |texture| {
            const view = renderer._gctx.lookupResource(self.renderer_texture_views[texture.size_index][texture.index]).?;
            zgui.image(view, .{
                .w = @floatFromInt(tsp.get_u_size()),
                .h = @floatFromInt(tsp.get_v_size()),
            });
        }
    }
    if (strip.polygon.area1_texture_control()) |area1_texture_control| {
        zgui.text("Area1 Tex.:   {X:0>8}", .{@as(u32, @bitCast(area1_texture_control))});
    }
    if (strip.polygon.area1_tsp_instruction()) |area1_tsp| {
        zgui.text("Area1 TSP:    {X:0>8}", .{@as(u32, @bitCast(area1_tsp))});
    }
    if (strip.polygon.base_color()) |base_color| {
        var local = base_color;
        _ = zgui.colorEdit4("Base Color", .{ .col = &local, .flags = .{ .float = true } });
    }
    if (strip.polygon.offset_color()) |offset_color| {
        var local = offset_color;
        _ = zgui.colorEdit4("Offset Color", .{ .col = &local, .flags = .{ .float = true } });
    }
}

fn display_vertex_data(self: *@This(), vertex: *const Holly.VertexParameter) void {
    zgui.pushPtrId(vertex);
    defer zgui.popId();

    if (vertex.tag() == .SpriteType0 or vertex.tag() == .SpriteType1) {
        const sprite_positions = vertex.sprite_positions();
        for (0..3) |i| {
            zgui.text("Pos: {d: >3.2} | {d: >3.2} | {d: >3.2}", .{ sprite_positions[i][0], sprite_positions[i][1], sprite_positions[i][2] });
        }
        // TODO: UVs
        // TODO: Overlay on hover?
        return;
    }

    var position = vertex.position();
    var base_color: ?fRGBA = null;
    var offset_color: ?fRGBA = null;
    var base_intensity: ?f32 = null;
    var offset_intensity: ?f32 = null;
    var uv: ?[2]f32 = null;

    switch (vertex.*) {
        .Type0 => |v| {
            base_color = fRGBA.from_packed(v.base_color, true);
        },
        .Type1 => |v| {
            base_color = .{ .r = v.base_color.r, .g = v.base_color.g, .b = v.base_color.b, .a = v.base_color.a };
        },
        .Type2 => |v| {
            base_intensity = v.base_intensity;
        },
        .Type3 => |v| {
            base_color = fRGBA.from_packed(v.base_color, true);
            offset_color = fRGBA.from_packed(v.offset_color, true);
            uv = .{ v.u, v.v };
        },
        .Type4 => |v| {
            base_color = fRGBA.from_packed(v.base_color, true);
            offset_color = fRGBA.from_packed(v.offset_color, true);
            uv = .{ @bitCast(@as(u32, v.uv.u) << 16), @bitCast(@as(u32, v.uv.v) << 16) };
        },
        .Type5 => |v| {
            base_color = .{ .r = v.base_color.r, .g = v.base_color.g, .b = v.base_color.b, .a = v.base_color.a };
            offset_color = .{ .r = v.offset_color.r, .g = v.offset_color.g, .b = v.offset_color.b, .a = v.offset_color.a };
            uv = .{ v.u, v.v };
        },
        .Type6 => |v| {
            base_color = .{ .r = v.base_color.r, .g = v.base_color.g, .b = v.base_color.b, .a = v.base_color.a };
            offset_color = .{ .r = v.offset_color.r, .g = v.offset_color.g, .b = v.offset_color.b, .a = v.offset_color.a };
            uv = .{ @bitCast(@as(u32, v.uv.u) << 16), @bitCast(@as(u32, v.uv.v) << 16) };
        },
        .Type7 => |v| {
            base_intensity = v.base_intensity;
            offset_intensity = v.offset_intensity;
            uv = .{ v.u, v.v };
        },
        else => {},
    }

    zgui.beginGroup();
    const node_open = zgui.treeNodeFlags(@tagName(vertex.tag()), .{
        .open_on_double_click = true,
        .open_on_arrow = true,
    });
    if (zgui.isItemClicked(.left) and !zgui.isItemToggledOpen()) {
        self.selected_vertex_focus = true;
        self.selected_vertex = position[0..2].*;
    }
    if (node_open) {
        _ = zgui.inputFloat3("pos", .{ .v = &position, .flags = .{ .read_only = true } });
        if (base_color) |*color| {
            _ = zgui.colorEdit4("Base Color", .{ .col = @ptrCast(color), .flags = .{ .float = true } });
        }
        if (offset_color) |*color| {
            _ = zgui.colorEdit4("Offset Color", .{ .col = @ptrCast(color), .flags = .{ .float = true } });
        }
        if (base_intensity) |*intensity| {
            _ = zgui.inputFloat("Base Intensity", .{ .v = intensity, .flags = .{ .read_only = true } });
        }
        if (offset_intensity) |*intensity| {
            _ = zgui.inputFloat("Offset Intensity", .{ .v = intensity, .flags = .{ .read_only = true } });
        }
        if (uv) |*uvs| {
            _ = zgui.inputFloat2("UV", .{ .v = uvs, .flags = .{ .read_only = true } });
        }
        zgui.treePop();
    }

    zgui.endGroup();
    if (zgui.isItemClicked(.right)) {
        self.selected_vertex_focus = false;
    }
    if (!self.selected_vertex_focus and zgui.isItemHovered(.{})) {
        self.selected_vertex = position[0..2].*;
    }
}
