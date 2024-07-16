const std = @import("std");
const builtin = @import("builtin");
const zgpu = @import("zgpu");
const zgui = @import("zgui");
const zglfw = @import("zglfw");

const P4Register = @import("./sh4.zig").P4Register;
const HardwareRegisters = @import("./hardware_registers.zig");

const sh4 = @import("./sh4.zig");
const sh4_disassembly = sh4.sh4_disassembly;
const arm7 = @import("arm7");
const Holly = @import("./holly.zig");
const AICAModule = @import("./aica.zig");

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

audio_channels: [64]struct {
    amplitude_envelope: struct { start_time: u32 = 0, xv: std.ArrayList(u32) = undefined, yv: std.ArrayList(u32) = undefined } = .{},
} = .{.{}} ** 64,

_allocator: std.mem.Allocator,
_gctx: *zgpu.GraphicsContext,

fn printable_ascii(c: u8) u8 {
    if (c >= 0x20 and c <= 0x7E)
        return c;
    return '.';
}

fn display(self: anytype) void {
    const info = @typeInfo(@TypeOf(self));
    comptime var max_length = 0;
    inline for (info.Struct.fields) |field| {
        max_length = @max(max_length, field.name.len);
    }
    inline for (info.Struct.fields) |field| {
        zgui.text("{s: <" ++ std.fmt.comptimePrint("{d}", .{max_length}) ++ "} {any}", .{ field.name, @field(self, field.name) });
    }
}

fn zguiSelectEnum(comptime name: [:0]const u8, target: anytype) bool {
    var modified = false;
    if (zgui.beginCombo(name, .{ .preview_value = @tagName(target.*) })) {
        inline for (std.meta.fields(@TypeOf(target.*))) |mode| {
            const value: @TypeOf(target.*) = @enumFromInt(mode.value);
            if (zgui.selectable(mode.name, .{ .selected = target.* == value })) {
                target.* = value;
                modified = true;
            }
        }
        zgui.endCombo();
    }
    return modified;
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

    for (0..self.renderer_texture_views.len) |i|
        self.renderer_texture_views[i] = d.gctx.createTextureView(d.renderer.texture_arrays[i], .{
            .dimension = .tvdim_2d,
            .base_array_layer = 0,
            .array_layer_count = 1,
        });

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

    for (self.renderer_texture_views) |view|
        self._gctx.releaseResource(view);

    self._gctx.releaseResource(self.vram_texture_view);
    self._gctx.releaseResource(self.vram_texture);

    self._allocator.free(self.pixels);
}

pub fn draw(self: *@This(), d: *Deecy) !void {
    if (self.draw_debug_ui) {
        var dc = d.dc;

        _ = zgui.DockSpaceOverViewport(zgui.getMainViewport(), .{ .passthru_central_node = true });

        if (zgui.begin("Settings", .{})) {
            var volume = try d.audio_device.getMasterVolume();
            if (zgui.sliderFloat("Volume", .{ .v = &volume, .min = 0.0, .max = 1.0, .flags = .{} })) {
                try d.audio_device.setMasterVolume(volume);
            }
            _ = zguiSelectEnum("CPU Throttling Method", &d.cpu_throttling_method);
        }
        zgui.end();

        if (zgui.begin("Controls", .{})) {
            var available_controllers = std.ArrayList(struct { id: ?zglfw.Joystick.Id, name: [:0]const u8 }).init(self._allocator);
            defer available_controllers.deinit();

            try available_controllers.append(.{ .id = null, .name = "None" });

            for (0..zglfw.Joystick.maximum_supported) |idx| {
                const jid: zglfw.Joystick.Id = @intCast(idx);
                if (zglfw.Joystick.get(jid)) |joystick| {
                    if (joystick.asGamepad()) |gamepad| {
                        try available_controllers.append(.{ .id = jid, .name = gamepad.getName() });
                    }
                }
            }

            inline for (0..4) |i| {
                var connected: bool = d.dc.maple.ports[i].main != null;
                if (zgui.checkbox("Connected##" ++ std.fmt.comptimePrint("{d}", .{i + 1}), .{ .v = &connected })) {
                    if (d.dc.maple.ports[i].main != null) {
                        d.dc.maple.ports[i].main = null;
                    } else {
                        d.dc.maple.ports[i].main = .{ .Controller = .{} };
                    }
                }
                const name = if (d.controllers[i]) |jid|
                    (if (zglfw.Joystick.get(jid)) |joystick|
                        (if (joystick.asGamepad()) |gamepad| gamepad.getName() else "None")
                    else
                        "None")
                else
                    "None";
                if (zgui.beginCombo("Controller #" ++ std.fmt.comptimePrint("{d}", .{i + 1}), .{ .preview_value = name })) {
                    for (available_controllers.items, 0..) |item, index| {
                        const idx = @as(u32, @intCast(index));
                        if (zgui.selectable(item.name, .{ .selected = d.controllers[i] == available_controllers.items[idx].id }))
                            d.controllers[i] = available_controllers.items[idx].id;
                    }
                    zgui.endCombo();
                }
            }
        }
        zgui.end();

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
            zgui.text("SCIEB: {any}", .{dc.aica.debug_read_reg(AICAModule.InterruptBits, .SCIEB)});
            zgui.text("INTRequest: {any}", .{dc.aica.debug_read_reg(u32, .INTRequest)});
            if (zgui.collapsingHeader("Timers", .{ .default_open = true })) {
                const timer_registers = [_]AICAModule.AICARegister{ .TACTL_TIMA, .TBCTL_TIMB, .TCCTL_TIMC };
                inline for (0..3) |i| {
                    const number = std.fmt.comptimePrint("{d}", .{i});
                    const timer = dc.aica.debug_read_reg(AICAModule.TimerControl, timer_registers[i]);
                    zgui.text("Timer " ++ number ++ ": Prescale: {X:0>1} - Value: {X:0>2}", .{ timer.prescale, timer.value });
                    const mask: u32 = @as(u32, 1) << @intCast(6 + i);
                    zgui.text("Interrupt Enabled: {s}", .{if ((dc.aica.debug_read_reg(u32, .SCIEB) & mask) != 0) "Yes" else "No"});
                }
            }
            if (zgui.plot.beginPlot("Sample Buffer", .{ .flags = zgui.plot.Flags.canvas_only })) {
                //zgui.plot.setupAxis(.x1, .{ .label = "time" });
                zgui.plot.setupAxisLimits(.y1, .{ .min = std.math.minInt(i16), .max = std.math.maxInt(i16) });
                // zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = 10_000 });
                // zgui.plot.setupAxisLimits(.y1, .{ .min = 0, .max = 0x400 });
                // zgui.plot.setupLegend(.{ .south = false, .west = false }, .{});
                zgui.plot.setupFinish();
                zgui.plot.plotLine("sample_read_offset", i32, .{ .xv = &[_]i32{ @intCast(dc.aica.sample_read_offset), @intCast(dc.aica.sample_read_offset) }, .yv = &[_]i32{ 0, std.math.maxInt(i16) } });
                zgui.plot.plotLine("sample_write_offset", i32, .{ .xv = &[_]i32{ @intCast(dc.aica.sample_write_offset), @intCast(dc.aica.sample_write_offset) }, .yv = &[_]i32{ 0, -std.math.maxInt(i16) } });
                zgui.plot.plotLineValues("samples", i32, .{ .v = &dc.aica.sample_buffer });
                zgui.plot.endPlot();
            }
            _ = zgui.checkbox("Show disabled channels", .{ .v = &self.show_disabled_channels });
            inline for (0..64) |i| {
                const channel = dc.aica.get_channel_registers(@intCast(i));
                zgui.pushPtrId(channel);
                defer zgui.popId();
                const state = dc.aica.channel_states[i];
                const time: u32 = @truncate(@as(u64, @intCast(std.time.milliTimestamp())));
                if (self.show_disabled_channels or state.playing or channel.play_control.key_on_bit) {
                    const number = std.fmt.comptimePrint("{d}", .{i});
                    if (zgui.collapsingHeader("Channel " ++ number, .{ .default_open = true })) {
                        _ = zgui.checkbox("Mute (Debug)", .{ .v = &dc.aica.channel_states[i].debug.mute });
                        const start_addr = channel.sample_address();
                        zgui.text("KeyOn: {any} - Format: {s} - Loop: {any}", .{
                            channel.play_control.key_on_bit,
                            @tagName(channel.play_control.sample_format),
                            channel.play_control.sample_loop,
                        });
                        zgui.text("Start Address: {X: >8} - Loop: {X:0>4} - {X:0>4}", .{
                            start_addr,
                            channel.loop_start,
                            channel.loop_end,
                        });
                        zgui.text("FNS: {X:0>3} - Oct: {X:0>2}", .{ channel.sample_pitch_rate.fns, channel.sample_pitch_rate.oct });
                        zgui.textColored(if (state.playing) .{ 0.0, 1.0, 0.0, 1.0 } else .{ 1.0, 0.0, 0.0, 1.0 }, "Playing: {s: >3}", .{if (state.playing) "Yes" else "No"});
                        zgui.sameLine(.{});
                        zgui.text("PlayPos: {d: >10} - LoopEnd: {s: >3}", .{ state.play_position, if (state.loop_end_flag) "Yes" else "No" });
                        const effective_rate = AICAModule.AICAChannelState.compute_effective_rate(channel, switch (state.amp_env_state) {
                            .Attack => channel.amp_env_1.attack_rate,
                            .Decay => channel.amp_env_1.decay_rate,
                            .Sustain => channel.amp_env_1.sustain_rate,
                            .Release => channel.amp_env_2.release_rate,
                        });
                        zgui.text("AmpEnv    {s: >7} - level: {X: >5} - rate: {X: >5}", .{ @tagName(state.amp_env_state), state.amp_env_level, effective_rate });
                        zgui.text("FilterEnv {s: >7} - level: {X: >5}", .{ @tagName(state.filter_env_state), state.filter_env_level });
                        var loop_size = if (channel.play_control.sample_loop) channel.loop_end - channel.loop_start else channel.loop_end;
                        if (loop_size == 0) loop_size = 2048;
                        if (channel.play_control.sample_format == .i16) {
                            if (zgui.plot.beginPlot("Samples##" ++ number, .{ .flags = zgui.plot.Flags.canvas_only })) {
                                zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = @floatFromInt(loop_size) });
                                zgui.plot.setupAxisLimits(.y1, .{ .min = std.math.minInt(i16), .max = std.math.maxInt(i16) });
                                zgui.plot.setupFinish();
                                zgui.plot.plotLineValues("samples", i16, .{ .v = @as([*]const i16, @alignCast(@ptrCast(&dc.aica.wave_memory[start_addr])))[0..loop_size] });
                                zgui.plot.endPlot();
                            }
                        } else if (channel.play_control.sample_format == .i8) {
                            if (zgui.plot.beginPlot("Samples##" ++ number, .{ .flags = zgui.plot.Flags.canvas_only })) {
                                zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = @floatFromInt(loop_size) });
                                zgui.plot.setupAxisLimits(.y1, .{ .min = std.math.minInt(i8), .max = std.math.maxInt(i8) });
                                zgui.plot.setupFinish();
                                zgui.plot.plotLineValues("samples", i8, .{ .v = @as([*]const i8, @alignCast(@ptrCast(&dc.aica.wave_memory[start_addr])))[0..loop_size] });
                                zgui.plot.endPlot();
                            }
                        }
                        if (time - self.audio_channels[i].amplitude_envelope.start_time > 10_000) {
                            self.audio_channels[i].amplitude_envelope.xv.clearRetainingCapacity();
                            self.audio_channels[i].amplitude_envelope.yv.clearRetainingCapacity();
                        }
                        if (self.audio_channels[i].amplitude_envelope.xv.items.len > 0) {
                            try self.audio_channels[i].amplitude_envelope.xv.append(time - self.audio_channels[i].amplitude_envelope.start_time);
                        } else {
                            self.audio_channels[i].amplitude_envelope.start_time = time;
                            try self.audio_channels[i].amplitude_envelope.xv.append(0);
                        }
                        try self.audio_channels[i].amplitude_envelope.yv.append(state.amp_env_level);
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

            var buffer: [256]u8 = .{0} ** 256;

            if (zgui.collapsingHeader("Polygons", .{ .frame_padding = true })) {
                zgui.indent(.{});
                inline for (.{ Holly.ListType.Opaque, Holly.ListType.Translucent, Holly.ListType.PunchThrough }) |list_type| {
                    const list = &dc.gpu.ta_display_lists[@intFromEnum(list_type)];
                    const name = @tagName(@as(Holly.ListType, list_type));
                    const header = try std.fmt.bufPrintZ(&buffer, name ++ " ({d})###" ++ name, .{list.vertex_strips.items.len});

                    if (zgui.collapsingHeader(header, .{})) {
                        zgui.text("Strips: {d}, Vertices: {d}", .{ list.vertex_strips.items.len, list.vertex_parameters.items.len });
                        for (list.vertex_strips.items) |strip| {
                            zgui.text("  {any}", .{strip.polygon});
                        }
                    }
                }
                zgui.unindent(.{});
            }
            if (zgui.collapsingHeader("Modifier Volumes", .{ .frame_padding = true })) {
                zgui.indent(.{});
                {
                    const header = try std.fmt.bufPrintZ(&buffer, "Opaque ({d})###OMV", .{d.renderer.opaque_modifier_volumes.items.len});
                    if (zgui.collapsingHeader(header, .{})) {
                        for (d.renderer.opaque_modifier_volumes.items) |vol| {
                            zgui.text("  {any}", .{vol});
                        }
                    }
                }
                //{
                //    const header = try std.fmt.bufPrintZ(&buffer, "Translucent ({d})###TMV", .{d.renderer.translucent_modifier_volumes.items.len});
                //    if (zgui.collapsingHeader(header, .{})) {
                //        for (d.renderer.translucent_modifier_volumes.items) |vol| {
                //            zgui.text("  {any}", .{vol});
                //        }
                //    }
                //}
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
            if (zguiSelectEnum("Display Mode", &d.renderer.display_mode))
                d.renderer.update_blit_to_screen_vertex_buffer();

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

        if (zgui.begin("Debug", .{})) {
            if (zgui.button("Clear SH4 JIT Cache", .{}))
                try dc.sh4_jit.block_cache.reset();

            zgui.separator();

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
    }
}
