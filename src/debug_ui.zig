const std = @import("std");
const builtin = @import("builtin");
const zgpu = @import("zgpu");
const zgui = @import("zgui");
const zglfw = @import("zglfw");

const DreamcastModule = @import("dreamcast");
const arm7 = DreamcastModule.AICAModule.arm7;

const HardwareRegisters = DreamcastModule.HardwareRegisters;

const SH4Module = DreamcastModule.SH4Module;
const P4Register = SH4Module.P4Register;
const sh4_disassembly = SH4Module.disassembly;
const BasicBlock = DreamcastModule.SH4JITModule.BasicBlock;
const Holly = DreamcastModule.HollyModule;
const AICAModule = DreamcastModule.AICAModule;

const Colors = DreamcastModule.HollyModule.Colors;
const Color16 = Colors.Color16;
const fRGBA = Colors.fRGBA;

const RendererModule = @import("renderer.zig");

const Deecy = @import("deecy.zig");

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
selected_strip_pass_idx: usize = 0,

// Vertex Debug Display
selected_vertex_focus: bool = false,
selected_vertex: ?[2]f32 = null,

draw_wireframe: bool = false,
draw_list_wireframe: [3]bool = @splat(true),
list_wireframe_colors: [3]u32 = .{ 0xFF4B19E6, 0xFF4BB43C, 0xFFD86343 },

selected_volume_focus: bool = false,
selected_volume_list: Holly.ListType = .OpaqueModifierVolume,
selected_volume_index: ?u32 = null,
selected_volume_pass_idx: usize = 0,

selected_texture: struct {
    index: i32 = 0,
    scale: f32 = 512.0 / 8.0,
    size: i32 = 0,
} = .{},

pixels: []u8 = undefined,

audio_channels: [64]struct {
    amplitude_envelope: struct { start_time: i64 = 0, xv: std.ArrayList(u32) = undefined, yv: std.ArrayList(u32) = undefined } = .{},
} = @splat(.{}),

dsp_inputs: [16]struct { start_time: i64 = 0, xv: std.ArrayList(i32) = undefined, yv: std.ArrayList(i32) = undefined } = @splat(.{}),
dsp_outputs: [16]struct { start_time: i64 = 0, xv: std.ArrayList(i32) = undefined, yv: std.ArrayList(i32) = undefined } = @splat(.{}),

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
    inline for (info.@"struct".fields) |field| {
        max_length = @max(max_length, field.name.len);
    }
    inline for (info.@"struct".fields) |field| {
        // Hide "private" (or hidden) fields - Meaning those starting with an underscore.
        if (!std.mem.startsWith(u8, field.name, "_")) {
            switch (@typeInfo(field.type)) {
                .@"enum" => zgui.text("{s: <" ++ std.fmt.comptimePrint("{d}", .{max_length}) ++ "} {s}", .{ field.name, @tagName(@field(self, field.name)) }),
                .@"struct" => {
                    if (zgui.collapsingHeader(field.name ++ " (" ++ @typeName(field.type) ++ ")", .{})) {
                        display(@field(self, field.name));
                    }
                },
                else => zgui.text("{s: <" ++ std.fmt.comptimePrint("{d}", .{max_length}) ++ "} {any}", .{ field.name, @field(self, field.name) }),
            }
        }
    }
}

const White: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 };
const Grey: [4]f32 = .{ 0.5, 0.5, 0.5, 1.0 };
const Green: [4]f32 = .{ 0.51, 0.71, 0.212, 1.0 };
const Red: [4]f32 = .{ 0.973, 0.443, 0.408, 1.0 };

fn inline_bool(value: bool, comptime fmt: []const u8) void {
    inline_colored(value, fmt, .{});
}

fn inline_colored(value: bool, comptime fmt: []const u8, args: anytype) void {
    zgui.sameLine(.{});
    colored(value, fmt, args);
}

fn colored(value: bool, comptime fmt: []const u8, args: anytype) void {
    zgui.textColored(if (value) Green else Red, fmt, args);
}

fn text_highlighted(b: bool, comptime fmt: []const u8, args: anytype) void {
    zgui.textColored(if (b) White else Grey, fmt, args);
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
            self.renderer_texture_views[i][j] = d.gctx.createTextureView(d.renderer.texture_arrays[i].texture, .{
                .dimension = .tvdim_2d,
                .base_array_layer = @intCast(j),
                .array_layer_count = 1,
            });
        }
    }

    for (0..self.audio_channels.len) |i| {
        self.audio_channels[i].amplitude_envelope.xv = .init(self._allocator);
        self.audio_channels[i].amplitude_envelope.yv = .init(self._allocator);
    }
    for (0..16) |i| {
        self.dsp_inputs[i].xv = .init(self._allocator);
        self.dsp_inputs[i].yv = .init(self._allocator);
        self.dsp_outputs[i].xv = .init(self._allocator);
        self.dsp_outputs[i].yv = .init(self._allocator);
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

    for (0..16) |i| {
        self.dsp_inputs[i].xv.deinit();
        self.dsp_inputs[i].yv.deinit();
        self.dsp_outputs[i].xv.deinit();
        self.dsp_outputs[i].yv.deinit();
    }

    self._gctx.releaseResource(self.vram_texture_view);
    self._gctx.releaseResource(self.vram_texture);

    self._allocator.free(self.pixels);
}

fn add(a: [2]f32, b: [2]f32) [2]f32 {
    return .{ a[0] + b[0], a[1] + b[1] };
}

fn mul(a: [2]f32, b: [2]f32) [2]f32 {
    return .{ a[0] * b[0], a[1] * b[1] };
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

fn compare_blocks(_: void, a: BasicBlock, b: BasicBlock) std.math.Order {
    return std.math.order(a.time_spent, b.time_spent);
}
fn compare_blocks_desc(_: void, a: BasicBlock, b: BasicBlock) bool {
    return a.time_spent > b.time_spent;
}

fn display_tlb(comptime name: [:0]const u8, tlbs: []SH4Module.mmu.TLBEntry) void {
    if (zgui.beginTable(name, .{
        .column = 7,
        .flags = .{
            .sizing = .fixed_fit,
            .row_bg = true,
            .borders = .all,
        },
    })) {
        zgui.tableSetupColumn("#", .{});
        zgui.tableSetupColumn("ASID", .{});
        zgui.tableSetupColumn("VPN", .{});
        zgui.tableSetupColumn("PPN", .{});
        zgui.tableSetupColumn("Size", .{});
        zgui.tableSetupColumn("SH", .{});
        zgui.tableSetupColumn("PR", .{});
        zgui.tableHeadersRow();
        for (tlbs, 0..) |tlb, i| {
            const color: [4]f32 = if (tlb.valid()) White else Grey;
            zgui.tableNextRow(.{});
            _ = zgui.tableSetColumnIndex(0);
            zgui.textColored(color, "{d: >2}", .{i});
            _ = zgui.tableSetColumnIndex(1);
            zgui.textColored(color, "{X: >2}", .{tlb.asid});
            _ = zgui.tableSetColumnIndex(2);
            zgui.textColored(color, "{X:0>8}", .{@as(u32, tlb.vpn) << 10});
            _ = zgui.tableSetColumnIndex(3);
            zgui.textColored(color, "{X:0>8}", .{tlb._ppn});
            _ = zgui.tableSetColumnIndex(4);
            zgui.textColored(color, "{s}", .{switch (tlb.sz) {
                0 => "1KB",
                1 => "4KB",
                2 => "64KB",
                3 => "1MB",
            }});
            _ = zgui.tableSetColumnIndex(5);
            zgui.textColored(if (!tlb.valid()) color else if (tlb.sh) Green else Red, "{s}", .{if (tlb.sh) "O" else "X"});
            _ = zgui.tableSetColumnIndex(6);
            zgui.textColored(color, "{s}", .{switch (tlb.pr) {
                .ReadOnly => "R",
                .ReadWrite => "RW",
                .PrivilegedReadOnly => "R*",
                .PrivilegedReadWrite => "RW*",
            }});
        }
        zgui.endTable();
    }
}

pub fn draw(self: *@This(), d: *Deecy) !void {
    var dc = d.dc;

    self.reset_hover();

    if (zgui.begin("SH4", .{})) {
        if (zgui.button("Dump DC State", .{})) {
            const was_running = d.running;
            if (was_running)
                d.pause();
            var file = try std.fs.cwd().createFile("logs/dc_dump.bin", .{});
            defer file.close();
            _ = try dc.serialize(file.writer());
            if (was_running)
                d.start();
        }
        zgui.text("PC: {X:0>8} - SPC: 0x{X:0>8}", .{ dc.cpu.pc, dc.cpu.spc });
        zgui.text("PR: {X:0>8}", .{dc.cpu.pr});
        zgui.text("SR: {s}", .{if (dc.cpu.sr.md == 1) "Priv" else "User"});
        zgui.sameLine(.{});
        text_highlighted(dc.cpu.sr.t, " [T]", .{});
        zgui.sameLine(.{});
        text_highlighted(dc.cpu.sr.s, " [S]", .{});
        zgui.sameLine(.{});
        text_highlighted(dc.cpu.sr.bl, " [BL]", .{});
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
            if (addr < DreamcastModule.Dreamcast.BootSize or (addr >= 0x0C000000 and addr < 0x0D000000)) {
                const disassembly = sh4_disassembly.disassemble(.{ .value = dc.cpu.read_physical(u16, @intCast(addr)) }, self._allocator);
                zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == pc) ">" else " ", disassembly });
            } else {
                zgui.text("[{X:0>8}] {s} Out of range (TODO: Handle MMU)", .{ addr, if (addr == pc) ">" else " " });
            }
            addr += 2;
        }

        if (zgui.button(if (d.running) "Pause" else "Run", .{ .w = 200.0 })) {
            if (d.running) {
                d.pause();
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
            zgui.pushIntId(@intCast(i));
            defer zgui.popId();
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
            const control = dc.cpu.read_p4_register(SH4Module.P4.TCR, timers[i].control);
            const enabled = ((TSTR >> i) & 1) == 1;
            text_highlighted(enabled, "Timer {d:0>1}: {X:0>8} / {X:0>8}", .{
                i,
                dc.cpu.read_p4_register(u32, timers[i].counter),
                dc.cpu.read_p4_register(u32, timers[i].constant),
            });
            if (i == 2) {
                text_highlighted(enabled, "  TPSC {X:0>1} CKEG {X:0>1} UNIE {X:0>1} UNF {X:0>1} ICPE {X:0>1} ICPF {X:0>1}", .{ control.tpsc, control.ckeg, control.unie, control.unf, control.icpe, control.icpf });
            } else {
                text_highlighted(enabled, "  TPSC {X:0>1} CKEG {X:0>1} UNIE {X:0>1} UNF {X:0>1}", .{ control.tpsc, control.ckeg, control.unie, control.unf });
            }
            zgui.endGroup();
        }

        zgui.beginGroup();
        zgui.text("IPRA: {X:0>4}", .{dc.cpu.read_p4_register(u16, .IPRA)});
        display(dc.cpu.read_p4_register(SH4Module.P4.IPRA, .IPRA));
        zgui.endGroup();
        zgui.sameLine(.{});
        zgui.beginGroup();
        zgui.text("IPRB: {X:0>4}", .{dc.cpu.read_p4_register(u16, .IPRB)});
        display(dc.cpu.read_p4_register(SH4Module.P4.IPRB, .IPRB));
        zgui.endGroup();
        zgui.sameLine(.{});
        zgui.beginGroup();
        zgui.text("IPRC: {X:0>4}", .{dc.cpu.read_p4_register(u16, .IPRC)});
        display(dc.cpu.read_p4_register(SH4Module.P4.IPRC, .IPRC));
        zgui.endGroup();

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

    if (zgui.begin("SH4 MMU", .{})) {
        const MMUCR = dc.cpu.read_p4_register(SH4Module.mmu.MMUCR, .MMUCR);
        const PTEH = dc.cpu.read_p4_register(SH4Module.mmu.PTEH, .PTEH);
        const PTEL = dc.cpu.read_p4_register(SH4Module.mmu.PTEL, .PTEL);
        const PTEA = dc.cpu.read_p4_register(SH4Module.mmu.PTEA, .PTEA);
        const TTB = dc.cpu.read_p4_register(u32, .TTB);
        const TEA = dc.cpu.read_p4_register(u32, .TEA);
        if (MMUCR.at) zgui.textColored(Green, "Enabled", .{}) else zgui.textColored(Red, "Disabled", .{});
        zgui.sameLine(.{});
        zgui.text(" (Level: {s})", .{@tagName(dc.cpu._mmu_state)});
        if (MMUCR.sv) zgui.textColored(Green, "Single virtual memory mode", .{}) else zgui.textColored(Red, "Multiple virtual memory mode", .{});
        if (MMUCR.sqmd == 0) zgui.textColored(Green, "Store queue User mode", .{}) else zgui.textColored(Red, "Store queue Privileged mode", .{});
        zgui.text("URC: {X: >2}, URB: {X: >2}, LRUI: {b:0>6}", .{ MMUCR.urc, MMUCR.urb, MMUCR.lrui });
        if (zgui.collapsingHeader("Registers", .{ .default_open = false })) {
            zgui.indent(.{});
            defer zgui.unindent(.{});
            zgui.text("PTEH: {X:0>8}", .{@as(u32, @bitCast(PTEH))});
            display(PTEH);
            zgui.text("PTEL: {X:0>8}", .{@as(u32, @bitCast(PTEL))});
            display(PTEL);
            zgui.text("PTEA: {X:0>8}", .{@as(u32, @bitCast(PTEA))});
            display(PTEA);
            zgui.text("TTB: {X:0>8}", .{TTB});
            zgui.text("TEA: {X:0>8}", .{TEA});
        }
        if (zgui.collapsingHeader("ITLB", .{ .default_open = true })) {
            display_tlb("ITLB", dc.cpu.itlb);
        }
        if (zgui.collapsingHeader("UTLB", .{ .default_open = true })) {
            display_tlb("UTLB", dc.cpu.utlb);
        }
    }
    zgui.end();

    if (zgui.begin("SH4 JIT", .{})) {
        _ = zgui.checkbox("Enable", .{ .v = &d.enable_jit });
        zgui.sameLine(.{});
        if (zgui.button("Clear Cache", .{})) {
            const was_running = d.running;
            if (was_running)
                d.pause();
            try dc.sh4_jit.reset();
            if (was_running)
                d.start();
        }
        _ = zgui.checkbox("Enable Idle Skip", .{ .v = &d.dc.sh4_jit.idle_skip_enabled });
        var idle_skip_cycles: i32 = @intCast(d.dc.sh4_jit.idle_skip_cycles);
        if (zgui.inputInt("Idle Skip Cycles", .{ .v = &idle_skip_cycles })) d.dc.sh4_jit.idle_skip_cycles = @intCast(idle_skip_cycles);
        zgui.text("Cache Size: {d}KiB", .{dc.sh4_jit.block_cache.cursor / 1024});
        zgui.separator();
        if (BasicBlock.EnableInstrumentation) {
            if (zgui.collapsingHeader("Block statistics", .{ .default_open = true })) {
                const max = 50;
                const static = struct {
                    var top: std.PriorityQueue(BasicBlock, void, compare_blocks) = undefined;
                    var sorted: [max]usize = @splat(0);
                    var initialized: bool = false;
                };
                zgui.beginDisabled(.{ .disabled = d.running });
                if (zgui.button("Refresh", .{})) {
                    if (!static.initialized) {
                        static.top = .init(dc._allocator, {});
                        static.initialized = true;
                    } else {
                        while (static.top.count() > 0) _ = static.top.remove();
                    }
                    for (0..dc.sh4_jit.block_cache.blocks.len) |i| {
                        if (dc.sh4_jit.block_cache.blocks[i].offset > 0) {
                            const block = dc.sh4_jit.block_cache.blocks[i];
                            if (block.call_count > 0 and (static.top.count() < max or static.top.peek().?.time_spent < block.time_spent)) {
                                try static.top.add(block);
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
                    const was_running = d.running;
                    if (was_running)
                        try d.stop();
                    try dc.sh4_jit.reset();
                    if (was_running)
                        d.start();
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
                        const instr = dc.cpu.read_physical(u16, addr);
                        const op = SH4Module.instructions.Opcodes[SH4Module.instructions.JumpTable[instr]];
                        zgui.text("{s} {X:0>6}: {s}", .{ if (op.use_fallback()) "!" else " ", addr, sh4_disassembly.disassemble(@bitCast(instr), dc._allocator) });
                    }
                }
            }
        } else {
            zgui.textColored(Grey, "JIT instrumentation disabled in this build.", .{});
            zgui.textColored(Grey, "Compile with '-Djit_instrumentation=true' to enable (slow).", .{});
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
        var jit_enabled = dc.aica.enable_arm_jit;
        if (zgui.checkbox("ARM JIT", .{ .v = &jit_enabled })) {
            const was_running = d.running;
            if (was_running)
                d.pause();
            try dc.aica.arm_jit.reset();
            dc.aica.enable_arm_jit = jit_enabled;
            if (was_running)
                d.start();
        }
        zgui.sameLine(.{});
        _ = zgui.checkbox("Debug Trace", .{ .v = &dc.aica.arm_debug_trace });
        if (zgui.button("Dump Memory", .{})) {
            dc.aica.dump_wave_memory();
        }
        zgui.sameLine(.{});
        if (zgui.button("Dump Regs", .{})) {
            dc.aica.dump_registers();
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
        var addr = std.math.clamp(pc - range / 2, 0x00800000, 0x00A00000 - range) & 0xFFFFFFFC;
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
        const cdda_out_left = dc.aica.get_reg(AICAModule.DSPOutputMixer, .CDDAOutputLeft).*;
        zgui.text("CDDA Left:  Level: {X}, Pan: {X}", .{ cdda_out_left.efsdl, cdda_out_left.efpan });
        const cdda_out_right = dc.aica.get_reg(AICAModule.DSPOutputMixer, .CDDAOutputRight).*;
        zgui.text("CDDA Right: Level: {X}, Pan: {X}", .{ cdda_out_right.efsdl, cdda_out_right.efpan });
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
                    zgui.textColored(Green, "!", .{});
                } else {
                    zgui.textColored(Red, ".", .{});
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
            zgui.plot.plotLineValues("samples", i32, .{ .v = dc.aica.sample_buffer });
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

                    inline_colored(channel.play_control.key_on_bit, "KeyOn: {s: >3}", .{if (channel.play_control.key_on_bit) "Yes" else "No"});
                    zgui.text(" - {s} - ", .{@tagName(channel.play_control.sample_format)});
                    zgui.sameLine(.{});
                    colored(channel.play_control.sample_loop, "Loop: {s: >3}", .{if (channel.play_control.sample_loop) "Yes" else "No"});
                    zgui.text("Addr: {X: >6} - Loop: {X:0>4} - {X:0>4}", .{
                        start_addr,
                        channel.loop_start,
                        channel.loop_end,
                    });
                    zgui.text("FNS: {X:0>3} - Oct: {X:0>2}", .{ channel.sample_pitch_rate.fns, channel.sample_pitch_rate.oct });
                    zgui.text("DIPAN: {X:0>2} - DISDL: {X:0>1}", .{ channel.direct_pan_vol_send.pan, channel.direct_pan_vol_send.volume });
                    zgui.text("DSP Vol: {X:0>1} - DSP Chan: {X:0>1}", .{ channel.dps_channel_send.level, channel.dps_channel_send.channel });
                    inline_colored(state.playing, "{s: >7}", .{if (state.playing) "Playing" else "Stopped"});
                    zgui.text("PlayPos: {X: >6} - ", .{state.play_position});
                    zgui.sameLine(.{});
                    inline_colored(state.loop_end_flag, "LoodEnd: {s: >3}", .{if (state.loop_end_flag) "Yes" else "No"});
                    const effective_rate = AICAModule.AICAChannelState.compute_effective_rate(channel, switch (state.amp_env_state) {
                        .Attack => channel.amp_env_1.attack_rate,
                        .Decay => channel.amp_env_1.decay_rate,
                        .Sustain => channel.amp_env_1.sustain_rate,
                        .Release => channel.amp_env_2.release_rate,
                    });
                    inline_colored(!channel.env_settings.voff, "AmpEnv ", .{});
                    zgui.text("{s: >7} - level: {X: >4} - rate: {X: >2}", .{ @tagName(state.amp_env_state), state.amp_env_level, effective_rate });
                    inline_colored(!channel.env_settings.lpoff, "FilEnv ", .{});
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

    if (false) {
        if (zgui.begin("Scheduler", .{})) {
            const cycle: i64 = @intCast(d.dc._global_cycles);
            zgui.text("Global Cycle: {d}", .{cycle});
            // NOTE: This is not thread safe, and I don't want to introduce synchronization for a debugging view.
            //       Disabled by default, use it at your own risk :)
            var it = d.dc.scheduled_events.iterator();
            while (it.next()) |event| {
                zgui.text("[{d: >10}] {?} {any}", .{ @as(i64, @intCast(event.trigger_cycle)) - cycle, event.interrupt, event.event });
            }
        }
        zgui.end();
    }

    if (zgui.begin("AICA - DSP", .{})) {
        const time = std.time.milliTimestamp();
        const MaxSamples = 10_000;
        if (zgui.collapsingHeader("Coeff.", .{ .default_open = false })) {
            zgui.indent(.{});
            defer zgui.unindent(.{});
            inline for (0..128) |i| {
                const number = std.fmt.comptimePrint("{d: >3}", .{i});
                zgui.text("Coeff #" ++ number ++ ": {d: >6}", .{dc.aica.dsp.read_coef(i)});
            }
        }
        if (zgui.collapsingHeader("DSP Inputs (MIXS)", .{ .default_open = false })) {
            inline for (0..16) |i| {
                const number = std.fmt.comptimePrint("{d}", .{i});
                zgui.text("MIXS #" ++ number, .{});
                if (d.running) {
                    if (time - self.dsp_inputs[i].start_time > MaxSamples) {
                        self.dsp_inputs[i].xv.clearRetainingCapacity();
                        self.dsp_inputs[i].yv.clearRetainingCapacity();
                    }
                    if (self.dsp_inputs[i].xv.items.len > 0) {
                        try self.dsp_inputs[i].xv.append(@intCast(time - self.dsp_inputs[i].start_time));
                    } else {
                        self.dsp_inputs[i].start_time = time;
                        try self.dsp_inputs[i].xv.append(0);
                    }
                    try self.dsp_inputs[i].yv.append(@intCast(@as(i20, @bitCast(dc.aica.dsp.read_mixs(i)))));
                } else {
                    if (self.dsp_inputs[i].xv.items.len > 0)
                        self.dsp_inputs[i].start_time = time - self.dsp_inputs[i].xv.items[self.dsp_inputs[i].xv.items.len - 1];
                }
                if (zgui.plot.beginPlot("DSPInput##" ++ number, .{ .flags = zgui.plot.Flags.canvas_only, .h = 128.0 })) {
                    zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = MaxSamples });
                    zgui.plot.setupAxisLimits(.y1, .{ .min = @floatFromInt(std.math.minInt(i16)), .max = @floatFromInt(std.math.maxInt(i16)) });
                    zgui.plot.setupFinish();
                    zgui.plot.plotLine("input", i32, .{ .xv = self.dsp_inputs[i].xv.items, .yv = self.dsp_inputs[i].yv.items });
                    zgui.plot.endPlot();
                }
            }
        }
        if (zgui.collapsingHeader("DSP Outputs (EFREG)", .{ .default_open = true })) {
            inline for (0..16) |i| {
                const number = std.fmt.comptimePrint("{d}", .{i});
                const mix = dc.aica.get_dsp_mix_register(@intCast(i)).*;
                zgui.text("EFREG #" ++ number ++ " (EFSDL: {X}, EFPAN: {X})", .{ mix.efsdl, mix.efpan });
                if (d.running) {
                    if (time - self.dsp_outputs[i].start_time > MaxSamples) {
                        self.dsp_outputs[i].xv.clearRetainingCapacity();
                        self.dsp_outputs[i].yv.clearRetainingCapacity();
                    }
                    if (self.dsp_outputs[i].xv.items.len > 0) {
                        try self.dsp_outputs[i].xv.append(@intCast(time - self.dsp_outputs[i].start_time));
                    } else {
                        self.dsp_outputs[i].start_time = time;
                        try self.dsp_outputs[i].xv.append(0);
                    }
                    try self.dsp_outputs[i].yv.append(@intCast(dc.aica.dsp.read_efreg(i)));
                } else {
                    if (self.dsp_outputs[i].xv.items.len > 0)
                        self.dsp_outputs[i].start_time = time - self.dsp_outputs[i].xv.items[self.dsp_outputs[i].xv.items.len - 1];
                }
                if (zgui.plot.beginPlot("DSPOutput##" ++ number, .{ .flags = zgui.plot.Flags.canvas_only, .h = 128.0 })) {
                    zgui.plot.setupAxisLimits(.x1, .{ .min = 0, .max = MaxSamples });
                    zgui.plot.setupAxisLimits(.y1, .{ .min = @floatFromInt(std.math.minInt(i20)), .max = @floatFromInt(std.math.maxInt(i20)) });
                    zgui.plot.setupFinish();
                    zgui.plot.plotLine("Output", i32, .{ .xv = self.dsp_outputs[i].xv.items, .yv = self.dsp_outputs[i].yv.items });
                    zgui.plot.endPlot();
                }
            }
        }
    }
    zgui.end();

    if (zgui.begin("RAM", .{})) {
        const Range = 128;
        const static = struct {
            var start_addr: u32 = 0x0C000000;
            var edit_addr: i32 = 0x0C000000;
        };
        if (zgui.inputInt("Start", .{ .v = &static.edit_addr, .step = 8, .flags = .{ .chars_hexadecimal = true } })) {
            static.start_addr = @intCast(@max(0x0C000000, @min(static.edit_addr & 0x1FFFFFF8, 0x0D000000 - Range)));
        }
        var addr = static.start_addr;
        const end_addr = addr + Range;
        zgui.textColored(.{ 0.5, 0.5, 0.5, 1 }, "           00 01 02 03 04 05 06 07", .{});
        while (addr < end_addr) {
            zgui.text("[{X:0>8}] {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}  {c}{c}{c}{c}{c}{c}{c}{c}", .{
                addr,
                dc.cpu.read_physical(u8, @intCast(addr)),
                dc.cpu.read_physical(u8, @intCast(addr + 1)),
                dc.cpu.read_physical(u8, @intCast(addr + 2)),
                dc.cpu.read_physical(u8, @intCast(addr + 3)),
                dc.cpu.read_physical(u8, @intCast(addr + 4)),
                dc.cpu.read_physical(u8, @intCast(addr + 5)),
                dc.cpu.read_physical(u8, @intCast(addr + 6)),
                dc.cpu.read_physical(u8, @intCast(addr + 7)),
                printable_ascii(dc.cpu.read_physical(u8, @intCast(addr))),
                printable_ascii(dc.cpu.read_physical(u8, @intCast(addr + 1))),
                printable_ascii(dc.cpu.read_physical(u8, @intCast(addr + 2))),
                printable_ascii(dc.cpu.read_physical(u8, @intCast(addr + 3))),
                printable_ascii(dc.cpu.read_physical(u8, @intCast(addr + 4))),
                printable_ascii(dc.cpu.read_physical(u8, @intCast(addr + 5))),
                printable_ascii(dc.cpu.read_physical(u8, @intCast(addr + 6))),
                printable_ascii(dc.cpu.read_physical(u8, @intCast(addr + 7))),
            });
            addr += 8;
        }
    }
    zgui.end();

    if (zgui.begin("Holly", .{})) {
        if (zgui.collapsingHeader("SPG Registers", .{ .frame_padding = true })) {
            zgui.indent(.{});
            zgui.text("SPG_HBLANK_INT: {X:0>8} - {any}", .{ dc.gpu._get_register(u32, .SPG_HBLANK_INT).*, dc.gpu._get_register(Holly.SPG_HBLANK_INT, .SPG_HBLANK_INT).* });
            zgui.text("SPG_VBLANK_INT: {X:0>8} - {any}", .{ dc.gpu._get_register(u32, .SPG_VBLANK_INT).*, dc.gpu._get_register(Holly.SPG_VBLANK_INT, .SPG_VBLANK_INT).* });
            zgui.text("SPG_CONTROL:    {X:0>8} - {any}", .{ dc.gpu._get_register(u32, .SPG_CONTROL).*, dc.gpu._get_register(Holly.SPG_CONTROL, .SPG_CONTROL).* });
            zgui.text("SPG_HBLANK:     {X:0>8} - {any}", .{ dc.gpu._get_register(u32, .SPG_HBLANK).*, dc.gpu._get_register(Holly.SPG_HBLANK, .SPG_HBLANK).* });
            zgui.text("SPG_VBLANK:     {X:0>8} - {any}", .{ dc.gpu._get_register(u32, .SPG_VBLANK).*, dc.gpu._get_register(Holly.SPG_VBLANK, .SPG_VBLANK).* });
            zgui.text("SPG_WIDTH:      {X:0>8} - {any}", .{ dc.gpu._get_register(u32, .SPG_WIDTH).*, dc.gpu._get_register(Holly.SPG_WIDTH, .SPG_WIDTH).* });
            zgui.text("SPG_STATUS:     {X:0>8} - {any}", .{ dc.gpu._get_register(u32, .SPG_STATUS).*, dc.gpu._get_register(Holly.SPG_STATUS, .SPG_STATUS).* });
            zgui.text("SPG_LOAD:       {X:0>8} - {any}", .{ dc.gpu._get_register(u32, .SPG_LOAD).*, dc.gpu._get_register(Holly.SPG_LOAD, .SPG_LOAD).* });
            zgui.unindent(.{});
        }
        const ISP_BACKGND_D = dc.gpu._get_register(u32, .ISP_BACKGND_D).*;
        const ISP_BACKGND_T = dc.gpu._get_register(u32, .ISP_BACKGND_T).*;
        zgui.text("ISP_BACKGND_D: {d: >8.2} / {d: >8.2}", .{ ISP_BACKGND_D, @as(f32, @bitCast(ISP_BACKGND_D)) });
        zgui.text("ISP_BACKGND_T: {X:0>8}", .{ISP_BACKGND_T});

        const FB_C_SOF = dc.gpu.read_register(u32, .FB_C_SOF);
        const FB_W_SOF1 = dc.gpu.read_register(u32, .FB_W_SOF1);
        const FB_W_SOF2 = dc.gpu.read_register(u32, .FB_W_SOF2);
        const FB_W_LINESTRIDE = dc.gpu.read_register(u32, .FB_W_LINESTRIDE);
        const FB_R_SOF1 = dc.gpu.read_register(u32, .FB_R_SOF1);
        const FB_R_SOF2 = dc.gpu.read_register(u32, .FB_R_SOF2);
        const FB_X_CLIP = dc.gpu.read_register(Holly.FB_CLIP, .FB_X_CLIP);
        const FB_Y_CLIP = dc.gpu.read_register(Holly.FB_CLIP, .FB_Y_CLIP);
        zgui.text("FB_C_SOF:  0x{X:0>8}", .{FB_C_SOF});
        if (zgui.collapsingHeader("FB_W_CTRL", .{ .default_open = false }))
            display(dc.gpu.read_register(Holly.FB_W_CTRL, .FB_W_CTRL));
        zgui.text("FB_W_SOF1: 0x{X:0>8}", .{FB_W_SOF1});
        zgui.text("FB_W_SOF2: 0x{X:0>8}", .{FB_W_SOF2});
        zgui.text("FB_W_LINESTRIDE: 0x{X:0>8}", .{FB_W_LINESTRIDE});
        if (zgui.collapsingHeader("FB_R_CTRL", .{ .default_open = false }))
            display(dc.gpu.read_register(Holly.FB_R_CTRL, .FB_R_CTRL));
        zgui.text("FB_R_SOF1: 0x{X:0>8}", .{FB_R_SOF1});
        zgui.text("FB_R_SOF2: 0x{X:0>8}", .{FB_R_SOF2});
        zgui.text("FB_CLIP:  X=[{d}, {d}], Y=[{d}, {d}]", .{ FB_X_CLIP.min, FB_X_CLIP.max, FB_Y_CLIP.min, FB_Y_CLIP.max });
        if (zgui.collapsingHeader("FB_R_SIZE", .{ .default_open = false }))
            display(dc.gpu.read_register(Holly.FB_R_SIZE, .FB_R_SIZE));
        if (zgui.collapsingHeader("VO_CONTROL", .{ .default_open = false }))
            display(dc.gpu.read_register(Holly.VO_CONTROL, .VO_CONTROL));
        if (zgui.collapsingHeader("FPU_PARAM_CFG", .{ .default_open = false }))
            display(dc.gpu.read_register(Holly.FPU_PARAM_CFG, .FPU_PARAM_CFG));
        if (zgui.collapsingHeader("SCALER_CTL", .{ .default_open = false }))
            display(dc.gpu.read_register(Holly.SCALER_CTL, .SCALER_CTL));
        if (zgui.collapsingHeader("TA Registers", .{ .default_open = false })) {
            zgui.indent(.{});
            defer zgui.unindent(.{});
            zgui.text("TA_OL_BASE:  0x{X:0>8}", .{dc.gpu.read_register(u32, .TA_OL_BASE)});
            zgui.text("TA_ISP_BASE:  0x{X:0>8}", .{dc.gpu.read_register(u32, .TA_ISP_BASE)});
            zgui.text("TA_OL_LIMIT:  0x{X:0>8}", .{dc.gpu.read_register(u32, .TA_OL_LIMIT)});
            zgui.text("TA_ISP_LIMIT:  0x{X:0>8}", .{dc.gpu.read_register(u32, .TA_ISP_LIMIT)});
            zgui.text("TA_NEXT_OPB:  0x{X:0>8}", .{dc.gpu.read_register(u32, .TA_NEXT_OPB)});
            zgui.text("TA_ITP_CURRENT:  0x{X:0>8}", .{dc.gpu.read_register(u32, .TA_ITP_CURRENT)});
            if (zgui.collapsingHeader("TA_ALLOC_CTRL", .{ .default_open = false }))
                display(dc.gpu.read_register(Holly.TA_ALLOC_CTRL, .TA_ALLOC_CTRL));
            const ta_glob_tile_clip = dc.gpu.read_register(Holly.TA_GLOB_TILE_CLIP, .TA_GLOB_TILE_CLIP);
            zgui.text("TA_GLOB_TILE_CLIP: X={d}, Y={d}", .{ ta_glob_tile_clip.tile_x_num, ta_glob_tile_clip.tile_y_num });
            zgui.text("TA_YUV_TEX_BASE:  0x{X:0>8}", .{dc.gpu.read_register(u32, .TA_YUV_TEX_BASE)});
            zgui.text("TA_YUV_TEX_CNT:  0x{X:0>8}", .{dc.gpu.read_register(u32, .TA_YUV_TEX_CNT)});
            if (zgui.collapsingHeader("TA_YUV_TEX_CTRL", .{ .default_open = false }))
                display(dc.gpu.read_register(Holly.TA_YUV_TEX_CTRL, .TA_YUV_TEX_CTRL));
        }

        if (zgui.collapsingHeader("Half Offset", .{ .frame_padding = true })) {
            const HALF_OFFSET = dc.gpu.read_register(Holly.HALF_OFFSET, .HALF_OFFSET);
            zgui.text("FPU Pixel Sampling Position: {s}", .{@tagName(HALF_OFFSET.fpu_pixel_sampling_position)});
            zgui.text("TSP Pixel Sampling Position: {s}", .{@tagName(HALF_OFFSET.tsp_pixel_sampling_position)});
            zgui.text("TSP Texel Sampling Position: {s}", .{@tagName(HALF_OFFSET.tsp_texel_sampling_position)});
        }

        if (zgui.collapsingHeader("Region Array", .{ .frame_padding = true })) {
            zgui.indent(.{});
            defer zgui.unindent(.{});
            const region_base = dc.gpu.read_register(u32, .REGION_BASE);
            zgui.text("REGION_BASE:  0x{X:0>8}", .{region_base});
            const static = struct {
                var view_all: bool = false;
            };
            _ = zgui.checkbox("View All", .{ .v = &static.view_all });
            var region = dc.gpu.get_region_array_data_config(0);
            var idx: usize = 1;
            while (idx < 512) : (idx += 1) {
                zgui.text("[{d:<3}] {d:<2} {d:<2}", .{ idx, region.settings.tile_x_position, region.settings.tile_y_position });
                inline_bool(region.settings.flush_accumulate, "Flush-Accumulate ");
                inline_bool(region.settings.z_clear == .Clear, "Z-Clear ");
                inline_bool(region.settings.pre_sort, "Pre-Sort ");
                inline_bool(region.settings.last_region, "Last-Region ");
                zgui.text("      ", .{});
                inline_colored(!region.opaque_list_pointer.empty, "{X:<8} ", .{region.opaque_list_pointer.pointer_to_object_list});
                inline_colored(!region.opaque_modifier_volume_pointer.empty, "{X:<8} ", .{region.opaque_modifier_volume_pointer.pointer_to_object_list});
                inline_colored(!region.translucent_list_pointer.empty, "{X:<8} ", .{region.translucent_list_pointer.pointer_to_object_list});
                inline_colored(!region.translucent_modifier_volume_pointer.empty, "{X:<8} ", .{region.translucent_modifier_volume_pointer.pointer_to_object_list});
                inline_colored(!region.punch_through_list_pointer.empty, "{X:<8} ", .{region.punch_through_list_pointer.pointer_to_object_list});
                if (!static.view_all and region.settings.last_region) break;
                region = dc.gpu.get_region_array_data_config(idx);
            }
        }

        if (zgui.collapsingHeader("Wireframe", .{ .frame_padding = true })) {
            _ = zgui.checkbox("Draw wireframe", .{ .v = &self.draw_wireframe });
            {
                zgui.indent(.{});
                defer zgui.unindent(.{});
                _ = zgui.checkbox("Draw Opaque Wireframe", .{ .v = &self.draw_list_wireframe[0] });
                _ = zgui.checkbox("Draw Translucent Wireframe", .{ .v = &self.draw_list_wireframe[1] });
                _ = zgui.checkbox("Draw Punchthrough Wireframe", .{ .v = &self.draw_list_wireframe[2] });
            }
        }

        var buffer: [256]u8 = @splat(0);

        // NOTE: We're looking at the last list used during a START_RENDER.
        for (d.renderer.render_passes.items, 0..) |render_pass, pass_idx| {
            zgui.text("Pass #{d}: ZClear={any}, Pre-Sort={any}", .{ pass_idx, render_pass.z_clear, render_pass.pre_sort });
            zgui.indent(.{});
            defer zgui.unindent(.{});
            zgui.pushIntId(@intCast(pass_idx));
            defer zgui.popId();
            if (zgui.collapsingHeader("Polygons", .{ .frame_padding = true })) {
                zgui.indent(.{});
                inline for (.{ Holly.ListType.Opaque, Holly.ListType.Translucent, Holly.ListType.PunchThrough }) |list_type| {
                    const list = d.renderer.ta_lists_to_render.items[pass_idx].get_list(list_type);
                    const name = @tagName(@as(Holly.ListType, list_type));
                    const header = try std.fmt.bufPrintZ(&buffer, name ++ " ({d})###" ++ name, .{list.vertex_strips.items.len});

                    if (zgui.collapsingHeader(header, .{})) {
                        zgui.pushIntId(@intFromEnum(list_type));
                        defer zgui.popId();
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
                                    self.selected_strip_pass_idx = pass_idx;
                                }
                                if (node_open) {
                                    if (idx < list.vertex_strips.items.len) {
                                        self.display_strip_info(d.renderer, &list.vertex_strips.items[idx]);
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
                                self.selected_strip_pass_idx = pass_idx;
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
                    const list = d.renderer.ta_lists_to_render.items[pass_idx].opaque_modifier_volumes;
                    const header = try std.fmt.bufPrintZ(&buffer, "Opaque ({d})###OMV", .{list.items.len});
                    if (zgui.collapsingHeader(header, .{})) {
                        for (list.items, 0..) |vol, idx| {
                            zgui.text("  {any}", .{vol});
                            if (zgui.isItemClicked(.left)) {
                                self.selected_volume_focus = true;
                                self.selected_volume_list = .OpaqueModifierVolume;
                                self.selected_volume_index = @intCast(idx);
                                self.selected_volume_pass_idx = pass_idx;
                            }
                            if (zgui.isItemClicked(.right)) {
                                self.selected_volume_focus = false;
                            }
                            if (!self.selected_volume_focus and zgui.isItemHovered(.{})) {
                                self.selected_volume_list = .OpaqueModifierVolume;
                                self.selected_volume_index = @intCast(idx);
                                self.selected_volume_pass_idx = pass_idx;
                            }
                        }
                    }
                }
                {
                    const list = d.renderer.ta_lists_to_render.items[pass_idx].translucent_modifier_volumes;
                    const header = try std.fmt.bufPrintZ(&buffer, "Translucent ({d})###TMV", .{list.items.len});
                    if (zgui.collapsingHeader(header, .{})) {
                        for (list.items, 0..) |vol, idx| {
                            zgui.text("  {any}", .{vol});
                            if (zgui.isItemClicked(.left)) {
                                self.selected_volume_focus = true;
                                self.selected_volume_list = .TranslucentModifierVolume;
                                self.selected_volume_index = @intCast(idx);
                                self.selected_volume_pass_idx = pass_idx;
                            }
                            if (zgui.isItemClicked(.right)) {
                                self.selected_volume_focus = false;
                            }
                            if (!self.selected_volume_focus and zgui.isItemHovered(.{})) {
                                self.selected_volume_list = .TranslucentModifierVolume;
                                self.selected_volume_index = @intCast(idx);
                                self.selected_volume_pass_idx = pass_idx;
                            }
                        }
                    }
                }
                zgui.unindent(.{});
            }
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
                        const color: Color16 = .{ .value = dc.cpu.read_physical(u16, addr) };
                        self.pixels[4 * i + 0] = @as(u8, @intCast(color.argb1555.r)) << 3;
                        self.pixels[4 * i + 1] = @as(u8, @intCast(color.argb1555.g)) << 3;
                        self.pixels[4 * i + 2] = @as(u8, @intCast(color.argb1555.b)) << 3;
                        self.pixels[4 * i + 3] = 255; // FIXME: Not really.
                        i += 1;
                    },
                    0x1 => { // 565 RGB 16 bit
                        const color: Color16 = .{ .value = dc.cpu.read_physical(u16, addr) };
                        self.pixels[4 * i + 0] = @as(u8, @intCast(color.rgb565.r)) << 3;
                        self.pixels[4 * i + 1] = @as(u8, @intCast(color.rgb565.g)) << 2;
                        self.pixels[4 * i + 2] = @as(u8, @intCast(color.rgb565.b)) << 3;
                        self.pixels[4 * i + 3] = 255;
                        i += 1;
                    },
                    // ARGB 32-Bits
                    0x6 => {
                        self.pixels[4 * i + 0] = dc.cpu.read_physical(u8, @intCast(addr + 3));
                        self.pixels[4 * i + 1] = dc.cpu.read_physical(u8, @intCast(addr + 2));
                        self.pixels[4 * i + 2] = dc.cpu.read_physical(u8, @intCast(addr + 1));
                        self.pixels[4 * i + 3] = dc.cpu.read_physical(u8, @intCast(addr + 0));
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
        if (zgui.collapsingHeader("Texture Cache", .{})) {
            if (zgui.button("Clear Texture Cache", .{})) {
                for (0..d.renderer.texture_metadata.len) |size_index| {
                    for (0..d.renderer.texture_metadata[size_index].len) |i| {
                        d.renderer.texture_metadata[size_index][i].status = .Invalid;
                    }
                }
            }
            if (zgui.inputInt("Size", .{
                .v = &self.selected_texture.size,
                .step = 1,
            })) {
                self.selected_texture.size = std.math.clamp(self.selected_texture.size, 0, @as(i32, @intCast(self.renderer_texture_views.len - 1)));
                self.selected_texture.scale = @as(f32, 512) / @as(f32, @floatFromInt((@as(u32, 8) << @intCast(self.selected_texture.size))));
                self.selected_texture.index = std.math.clamp(self.selected_texture.index, 0, RendererModule.Renderer.MaxTextures[@intCast(self.selected_texture.size)] - 1);
            }
            zgui.sameLine(.{});
            zgui.text("{d: >3}x{d: >3}", .{ @as(u32, 8) << @intCast(self.selected_texture.size), @as(u32, 8) << @intCast(self.selected_texture.size) });
            if (zgui.inputInt("Index", .{ .v = &self.selected_texture.index, .step = 1 })) {
                self.selected_texture.index = std.math.clamp(self.selected_texture.index, 0, RendererModule.Renderer.MaxTextures[@intCast(self.selected_texture.size)] - 1);
            }
            if (zgui.dragFloat("Scale", .{ .v = &self.selected_texture.scale, .min = 1.0, .max = 8.0, .speed = 0.1 })) {
                self.selected_texture.scale = std.math.clamp(self.selected_texture.scale, 1.0, 8.0);
            }
            const tex_id = d.gctx.lookupResource(self.renderer_texture_views[@intCast(self.selected_texture.size)][@intCast(self.selected_texture.index)]).?;
            zgui.image(tex_id, .{ .w = self.selected_texture.scale * @as(f32, @floatFromInt(@as(u32, 8) << @intCast(self.selected_texture.size))), .h = self.selected_texture.scale * @as(f32, @floatFromInt(@as(u32, 8) << @intCast(self.selected_texture.size))) });
            if (zgui.collapsingHeader("Parameter Control Word", .{ .default_open = true })) {
                const control_word = d.renderer.texture_metadata[@intCast(self.selected_texture.size)][@intCast(self.selected_texture.index)].control_word;
                display(control_word);
            }
            if (zgui.collapsingHeader("TSP Instruction", .{ .default_open = true })) {
                const tsp_instruction = d.renderer.texture_metadata[@intCast(self.selected_texture.size)][@intCast(self.selected_texture.index)].tsp_instruction;
                display(tsp_instruction);
            }
        }
        if (zgui.collapsingHeader("Framebuffer Texture", .{})) {
            const fb_tex_id = d.gctx.lookupResource(d.renderer.framebuffer.view).?;
            zgui.image(fb_tex_id, .{ .w = 640, .h = 480 });
        }
        if (zgui.collapsingHeader("Resized Framebuffer Texture", .{})) {
            const fb_tex_id = d.gctx.lookupResource(d.renderer.resized_framebuffer.view).?;
            zgui.image(fb_tex_id, .{ .w = @floatFromInt(d.gctx.swapchain_descriptor.width), .h = @floatFromInt(d.gctx.swapchain_descriptor.height) });
        }
    }
    zgui.end();

    if (zgui.begin("Interrupts", .{})) {
        inline for (@typeInfo(HardwareRegisters.SB_ISTNRM).@"struct".fields) |field| {
            if (!std.mem.startsWith(u8, field.name, "_")) {
                if (zgui.button("Trigger " ++ field.name ++ " Interrupt", .{})) {
                    comptime var val: HardwareRegisters.SB_ISTNRM = .{};
                    @field(val, field.name) = 1;
                    dc.raise_normal_interrupt(val);
                }
            }
        }

        zgui.separator();

        inline for (@typeInfo(HardwareRegisters.SB_ISTEXT).@"struct".fields) |field| {
            if (!std.mem.startsWith(u8, field.name, "_")) {
                if (zgui.button("Trigger " ++ field.name ++ " Ext Interrupt", .{})) {
                    comptime var val: HardwareRegisters.SB_ISTEXT = .{};
                    @field(val, field.name) = 1;
                    dc.raise_external_interrupt(val);
                }
            }
        }
    }
    zgui.end();

    self.draw_overlay(d);
}

fn draw_strip(draw_list: zgui.DrawList, min: [2]f32, scale: [2]f32, display_list: *const Holly.DisplayList, index: u32, color: u32) void {
    if (index < display_list.vertex_strips.items.len) {
        const parameters = display_list.vertex_parameters.items;
        const strip = &display_list.vertex_strips.items[index];
        switch (strip.polygon) {
            .Sprite => |_| {
                for (strip.vertex_parameter_index..strip.vertex_parameter_index + strip.vertex_parameter_count) |i| {
                    const pos = parameters[i].sprite_positions();
                    draw_list.addTriangle(.{
                        .p1 = add(mul(scale, pos[0][0..2].*), min),
                        .p2 = add(mul(scale, pos[3][0..2].*), min),
                        .p3 = add(mul(scale, pos[1][0..2].*), min),
                        .col = color,
                        .thickness = 1.0,
                    });
                    draw_list.addTriangle(.{
                        .p1 = add(mul(scale, pos[3][0..2].*), min),
                        .p2 = add(mul(scale, pos[2][0..2].*), min),
                        .p3 = add(mul(scale, pos[1][0..2].*), min),
                        .col = color,
                        .thickness = 1.0,
                    });
                }
            },
            else => {
                if (strip.vertex_parameter_count >= 3) {
                    for (strip.vertex_parameter_index..strip.vertex_parameter_index + strip.vertex_parameter_count - 2) |i| {
                        const p1 = add(mul(scale, parameters[i].position()[0..2].*), min);
                        const p2 = add(mul(scale, parameters[i + 1].position()[0..2].*), min);
                        const p3 = add(mul(scale, parameters[i + 2].position()[0..2].*), min);
                        draw_list.addTriangle(.{ .p1 = p1, .p2 = p2, .p3 = p3, .col = color, .thickness = 1.0 });
                    }
                }
            },
        }
    }
}

fn draw_overlay(self: *@This(), d: *Deecy) void {
    const draw_list = zgui.getBackgroundDrawList();

    const native_resolution = [2]f32{ 640.0, 480.0 };
    const aspect_ratio = native_resolution[0] / native_resolution[1];
    const window_size = [2]f32{ @floatFromInt(self._gctx.swapchain_descriptor.width), @floatFromInt(self._gctx.swapchain_descriptor.height) };
    const resolution = [2]f32{ @floatFromInt(d.renderer.resolution.width), @floatFromInt(d.renderer.resolution.height) };

    const size = switch (d.renderer.display_mode) {
        .Center, .Fit => if (d.renderer.display_mode == .Fit or resolution[0] > window_size[0] or resolution[1] > window_size[1])
            (if (window_size[0] / window_size[1] < aspect_ratio) [2]f32{
                window_size[0],
                window_size[0] / aspect_ratio,
            } else [2]f32{
                aspect_ratio * window_size[1],
                window_size[1],
            })
        else
            resolution,
        .Stretch => window_size,
    };
    const scaler_ctl = d.dc.gpu.read_register(Holly.SCALER_CTL, .SCALER_CTL);
    // Scale of inner render compared to native DC resolution.
    const scale = [2]f32{
        scaler_ctl.get_x_scale_factor() * size[0] / native_resolution[0],
        scaler_ctl.get_y_scale_factor() * size[1] / native_resolution[1],
    };
    const min = switch (d.renderer.display_mode) {
        .Stretch => [2]f32{ 0, 0 },
        else => [2]f32{
            window_size[0] / 2.0 - size[0] / 2.0,
            window_size[1] / 2.0 - size[1] / 2.0,
        },
    };

    if (self.draw_wireframe) {
        for (d.renderer.ta_lists_to_render.items) |list| {
            for ([_]*const Holly.DisplayList{ &list.opaque_list, &list.translucent_list, &list.punchthrough_list }, 0..) |l, idx| {
                if (self.draw_list_wireframe[idx]) {
                    for (0..l.vertex_strips.items.len) |strip_idx| {
                        draw_strip(draw_list, min, scale, l, @intCast(strip_idx), self.list_wireframe_colors[idx]);
                    }
                }
            }
        }
    }

    if (self.selected_strip_list == .Opaque or self.selected_strip_list == .PunchThrough or self.selected_strip_list == .Translucent) {
        const list = d.renderer.ta_lists_to_render.items[self.selected_strip_pass_idx].get_list(self.selected_strip_list);
        draw_strip(draw_list, min, scale, list, self.selected_strip_index, 0xFFFF00FF);
    }
    if (self.selected_vertex) |vertex| {
        draw_list.addCircleFilled(.{ .p = add(mul(scale, vertex), min), .r = 5.0, .col = 0xFF4000FF });
    }
    if (self.selected_volume_index) |idx| {
        const list = switch (self.selected_volume_list) {
            .OpaqueModifierVolume => d.renderer.ta_lists_to_render.items[self.selected_volume_pass_idx].opaque_modifier_volumes.items,
            .TranslucentModifierVolume => d.renderer.ta_lists_to_render.items[self.selected_volume_pass_idx].translucent_modifier_volumes.items,
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
        if (zgui.collapsingHeader("Texture Control", .{}))
            display(texture_control_word);
        zgui.text("Texture size: {d}x{d}", .{ tsp.get_u_size(), tsp.get_v_size() });
        if (renderer.get_texture_view(texture_control_word, tsp)) |texture| {
            const view = renderer._gctx.lookupResource(self.renderer_texture_views[texture.size_index][texture.index]).?;
            zgui.image(view, .{
                .w = @floatFromInt(tsp.get_u_size()),
                .h = @floatFromInt(tsp.get_v_size()),
            });
            if (zgui.isItemClicked(.left)) {
                self.selected_texture.size = @intCast(texture.size_index);
                self.selected_texture.index = @intCast(texture.index);
            }
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
            uv = .{ v.uv.u_as_f32(), v.uv.v_as_f32() };
        },
        .Type5 => |v| {
            base_color = .{ .r = v.base_color.r, .g = v.base_color.g, .b = v.base_color.b, .a = v.base_color.a };
            offset_color = .{ .r = v.offset_color.r, .g = v.offset_color.g, .b = v.offset_color.b, .a = v.offset_color.a };
            uv = .{ v.u, v.v };
        },
        .Type6 => |v| {
            base_color = .{ .r = v.base_color.r, .g = v.base_color.g, .b = v.base_color.b, .a = v.base_color.a };
            offset_color = .{ .r = v.offset_color.r, .g = v.offset_color.g, .b = v.offset_color.b, .a = v.offset_color.a };
            uv = .{ v.uv.u_as_f32(), v.uv.v_as_f32() };
        },
        .Type7 => |v| {
            base_intensity = v.base_intensity;
            offset_intensity = v.offset_intensity;
            uv = .{ v.u, v.v };
        },
        .Type8 => |v| {
            base_intensity = v.base_intensity;
            offset_intensity = v.offset_intensity;
            uv = .{ v.uv.u_as_f32(), v.uv.v_as_f32() };
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
