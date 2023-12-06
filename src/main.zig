const std = @import("std");

const common = @import("./common.zig");
const sh4 = @import("./sh4.zig");

const zgui = @import("zgui");
const zgpu = @import("zgpu");
const zglfw = @import("zglfw");

const assets_dir = "assets/";

pub fn main() !void {
    std.debug.print("\r  == Katana ==                             \n", .{});

    var cpu: sh4.SH4 = .{};
    try cpu.init();
    defer cpu.deinit();

    const IPbin_file = try std.fs.cwd().openFile("./bin/IP.bin", .{});
    defer IPbin_file.close();
    const IPbin = try IPbin_file.readToEndAlloc(common.GeneralAllocator, 0x10000);
    defer common.GeneralAllocator.free(IPbin);
    cpu.load_IP_bin(IPbin);
    cpu.init_boot();

    while (true) {
        if (cpu.pc == 0x8C001008)
            cpu.debug_trace = true;
        cpu.execute();
    }

    try zglfw.init();
    defer zglfw.terminate();

    const window = zglfw.Window.create(800, 600, "Katana", null) catch {
        std.log.err("Failed to create window.", .{});
        return;
    };
    defer window.destroy();

    const gctx = try zgpu.GraphicsContext.create(common.GeneralAllocator, window, .{});
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
            zgui.text("PC: 0x{X:0>8}", .{cpu.pc});
            zgui.text("SR: {any}", .{cpu.sr});

            var addr = @max(0, cpu.pc - 8);
            const end_addr = @min(0xFFFFFFFFF, addr + 16);
            while (addr < end_addr) {
                zgui.text("[{X:0>8}] {s} {s}", .{ addr, if (addr == cpu.pc) ">" else " ", sh4.Opcodes[sh4.JumpTable[cpu.read16(@intCast(addr))]].name });
                addr += 2;
            }

            if (zgui.button(if (running) "Pause" else "Run", .{ .w = 200.0 })) {
                running = !running;
            }

            if (zgui.button("Step", .{ .w = 200.0 })) {
                cpu.execute();
            }

            for (0..breakpoints.items.len) |i| {
                zgui.text("Breakpoint {d}: 0x{X:0>8}", .{ i, breakpoints.items[i] });
            }
            const static = struct {
                var bp_addr: i32 = 0;
            };
            _ = zgui.inputInt("##breakpoint", .{ .v = &static.bp_addr });
            zgui.sameLine(.{});
            if (zgui.button("Add Breakpoint", .{ .w = 200.0 })) {
                try breakpoints.append(@intCast(static.bp_addr));
            }
        }
        zgui.end();

        if (zgui.begin("Memory", .{})) {
            const static = struct {
                var start_addr: i32 = 0;
            };
            _ = zgui.inputInt("Start", .{ .v = &static.start_addr });
            var addr = @max(0, @rem(static.start_addr, 8));
            const end_addr = addr + 32;
            while (addr < end_addr) {
                zgui.text("{X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}", .{
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

        if (running) {
            for (0..10000) |_| {
                cpu.execute();
                const breakpoint = for (breakpoints.items, 0..) |addr, index| {
                    if (addr == cpu.pc) break index;
                } else null;
                if (breakpoint != null) {
                    running = false;
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
