const std = @import("std");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) !void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    const termcolor_module = b.createModule(.{ .root_source_file = b.path("src/termcolor.zig") });
    const helpers_module = b.createModule(.{ .root_source_file = b.path("src/helpers.zig") });

    const arm7 = b.dependency("arm7", .{});
    const arm7_module = arm7.module("arm7");

    const mmu = b.option(bool, "mmu", "Enable Full MMU Emulation (default: true)") orelse true;
    const fast_mem = b.option(bool, "fast_mem", "Enable FastMem (default: true)") orelse true;
    const jit_instrumentation = b.option(bool, "jit_instrumentation", "Enable JIT instrumentation (Slow, default: false)") orelse false;
    const data_path = b.option([]const u8, "data_path", "Path to the data directory (Copy your bios and flash files here, default: './data')") orelse "./data";
    const userdata_path = b.option([]const u8, "userdata_path", "Path to the userdata directory (default: './userdata')") orelse "./userdata";
    const use_appdata_dir = b.option(bool, "use_appdata_dir", "Prepend the platform specific AppData directory to data_path and userdata_path (default: false)") orelse false;

    const dc_options = b.addOptions();
    dc_options.addOption(bool, "mmu", mmu);
    dc_options.addOption(bool, "fast_mem", fast_mem);
    dc_options.addOption(bool, "jit_instrumentation", jit_instrumentation);
    const path_options = b.addOptions();
    path_options.addOption(bool, "use_appdata_dir", use_appdata_dir);
    path_options.addOption([]const u8, "data_path", data_path);
    path_options.addOption([]const u8, "userdata_path", userdata_path);

    const dc_module = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/dreamcast/dreamcast.zig"),
        .imports = &.{
            .{ .name = "termcolor", .module = termcolor_module },
            .{ .name = "helpers", .module = helpers_module },
            .{ .name = "arm7", .module = arm7_module },
        },
    });
    dc_module.addOptions("dc_config", dc_options);
    dc_module.addOptions("path_config", path_options);

    const ziglz4 = b.dependency("ziglz4", .{
        .target = target,
        .optimize = .ReleaseFast,
    });
    const nfd = b.dependency("nfdzig", .{
        .target = target,
        .optimize = .ReleaseFast,
    });

    const deecy_options = b.addOptions();
    // None right now.

    const deecy_module = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
        .imports = &.{
            .{ .name = "termcolor", .module = termcolor_module },
            .{ .name = "helpers", .module = helpers_module },
            .{ .name = "dreamcast", .module = dc_module },
            .{ .name = "lz4", .module = ziglz4.module("zig-lz4") },
            .{ .name = "nfd", .module = nfd.module("nfd") },
        },
        // For some reason, on Windows, ___chkstk_ms takes up to 10% of the DC thread.
        // This is an attempts at getting rid of it, but doesn't seem functional as of zig 0.14.0-dev.2577+271452d22
        // See https://github.com/ziglang/zig/issues/20724
        // Also https://nullprogram.com/blog/2024/02/05/ for more info.
        .stack_check = false,
    });
    deecy_module.addOptions("config", deecy_options);

    const exe = b.addExecutable(.{
        .name = "Deecy",
        .root_module = deecy_module,
        .use_llvm = true, // NOTE: zgpu doesn't work correctly with the self-hosted backend (zig 0.15.1), leading to a crash in Linux in debug mode. Forcing LLVM use fixes the issue.
    });
    exe.addWin32ResourceFile(.{ .file = b.path("src/assets/resource.rc") });

    // Check target for IDE support
    const exe_check = b.addExecutable(.{
        .name = "DeecyCheck",
        .root_module = deecy_module,
    });

    switch (target.result.os.tag) {
        .windows => {
            // DwmSetWindowAttribute
            deecy_module.linkSystemLibrary("dwmapi", .{});
            // Windows Multimedia API for timeBeginPeriod
            deecy_module.linkSystemLibrary("winmm", .{});
        },
        else => {},
    }

    { // zig-gamedev
        const zgui = b.dependency("zgui", .{
            .shared = false,
            .with_implot = true,
            .backend = .glfw_wgpu,
        });
        const zgui_mod = zgui.module("root");
        // zgui_mod.linkLibrary(b.dependency("webgpu_dawn", .{}).artifact("dawn"));
        deecy_module.addImport("zgui", zgui_mod);
        deecy_module.linkLibrary(zgui.artifact("imgui"));

        const zglfw = b.dependency("zglfw", .{});
        deecy_module.addImport("zglfw", zglfw.module("root"));
        deecy_module.linkLibrary(zglfw.artifact("glfw"));

        const zpool = b.dependency("zpool", .{});
        deecy_module.addImport("zpool", zpool.module("root"));

        const zgpu = @import("zgpu");
        if (target.result.os.tag == .windows) {
            zgpu.installDxcFrom(exe, "zgpu");
        }
        const zgpu_dep = b.dependency("zgpu", .{ .max_num_bindings_per_group = 12 });
        const zgpu_module = zgpu_dep.module("root");
        deecy_module.addImport("zgpu", zgpu_module);
        // deecy_module.linkLibrary(zgpu_dep.artifact("zdawn"));

        try zgpu.linkDawn(b, "zgpu", exe);
        // try @import("webgpu_dawn").link(b, "webgpu_dawn", exe);
        // const dawn = b.dependency("webgpu_dawn", .{});
        // deecy_module.addLibraryPath(dawn.path("./build/src/dawn/native"));
        // deecy_module.linkSystemLibrary("webgpu_dawn", .{});

        const zaudio = b.dependency("zaudio", .{});
        deecy_module.addImport("zaudio", zaudio.module("root"));
        deecy_module.linkLibrary(zaudio.artifact("miniaudio"));
    }

    // This declares intent for the executable to be installed into the
    // standard location when the user invokes the "install" step (the default
    // step when running `zig build`).
    b.installArtifact(exe);

    const check = b.step("check", "Check if Deecy compiles");
    check.dependOn(&exe_check.step);

    // This *creates* a Run step in the build graph, to be executed when another
    // step is evaluated that depends on it. The next line below will establish
    // such a dependency.
    const run_cmd = b.addRunArtifact(exe);

    // By making the run step depend on the install step, it will be run from the
    // installation directory rather than directly from within the cache directory.
    // This is not necessary, however, if the application depends on other installed
    // files, this ensures they will be present and in the expected location.
    run_cmd.step.dependOn(b.getInstallStep());

    // This allows the user to pass arguments to the application in the build
    // command itself, like this: `zig build run -- arg1 arg2 etc`
    if (b.args) |args| run_cmd.addArgs(args);

    // This creates a build step. It will be visible in the `zig build --help` menu,
    // and can be selected like this: `zig build run`
    // This will evaluate the `run` step rather than the default, which is "install".
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    {
        const interpreter_perf = b.addExecutable(.{
            .name = "InterpreterPerf",
            .root_module = b.createModule(.{
                .root_source_file = b.path("test/interpreter_perf.zig"),
                .target = target,
                .optimize = .ReleaseFast, // Note: This ignores the optimization level set by the user.
            }),
        });
        interpreter_perf.root_module.addImport("termcolor", termcolor_module);
        interpreter_perf.root_module.addImport("lz4", ziglz4.module("zig-lz4"));
        interpreter_perf.root_module.addImport("dreamcast", dc_module);

        const interpreter_pref_install = b.addInstallArtifact(interpreter_perf, .{});

        const run_interpreter_perf_tests = b.addRunArtifact(interpreter_perf);
        const interpreter_perf_step = b.step("interpreter_perf", "Run interpreter performance tests");
        interpreter_perf_step.dependOn(&run_interpreter_perf_tests.step);
        interpreter_perf_step.dependOn(&interpreter_pref_install.step);
        if (b.args) |args| run_interpreter_perf_tests.addArgs(args);

        const perf_install_step = b.step("interpreter_perf_install", "Install the interpreter performance tests");
        perf_install_step.dependOn(&interpreter_pref_install.step);
    }
    {
        const jit_perf = b.addExecutable(.{
            .name = "JITPerf",
            .root_module = b.createModule(.{
                .root_source_file = b.path("test/jit_perf.zig"),
                .target = target,
                .optimize = .ReleaseFast, // Note: This ignores the optimization level set by the user.
            }),
        });
        jit_perf.root_module.addImport("termcolor", termcolor_module);
        jit_perf.root_module.addImport("lz4", ziglz4.module("zig-lz4"));
        jit_perf.root_module.addImport("dreamcast", dc_module);

        const jit_pref_install = b.addInstallArtifact(jit_perf, .{});

        const run_jit_perf_tests = b.addRunArtifact(jit_perf);
        const perf_run_step = b.step("perf", "Run performance tests");
        perf_run_step.dependOn(&run_jit_perf_tests.step);
        perf_run_step.dependOn(&jit_pref_install.step);
        if (b.args) |args| run_jit_perf_tests.addArgs(args);

        const perf_install_step = b.step("perf_install", "Install the JIT performance tests");
        perf_install_step.dependOn(&jit_pref_install.step);
    }

    // ----- Tests ------

    // Creates a step for unit testing. This only builds the test executable
    // but does not run it.
    const unit_tests = b.addTest(.{ .root_module = dc_module });
    const run_unit_tests = b.addRunArtifact(unit_tests);

    // Similar to creating the run step earlier, this exposes a `test` step to
    // the `zig build --help` menu, providing a way for the user to request
    // running the unit tests.
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_unit_tests.step);

    const sh4_tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("test/sh4_SingleStepTests.zig"),
            .target = target,
            .optimize = optimize,
        }),
        // NOTE: The ftrv XMTRX,FVn test fails with the self-hosted backend, I guess there are differences in float handling.
        //       I don't know if the test is reliable in the first place, but llvm is used the releases anyway.
        .use_llvm = true,
    });
    sh4_tests.root_module.addImport("termcolor", termcolor_module);
    sh4_tests.root_module.addImport("dreamcast", dc_module);
    const run_sh4_tests = b.addRunArtifact(sh4_tests);
    const sh4_test_step = b.step("sh4_test", "Run sh4 tests");
    sh4_test_step.dependOn(&run_sh4_tests.step);
}
