const std = @import("std");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    const arm7 = b.dependency("arm7", .{});
    const arm7_module = arm7.module("arm7");

    const termcolor_module = b.createModule(.{ .root_source_file = b.path("src/termcolor.zig") });

    const dc_module = b.createModule(.{
        .root_source_file = b.path("src/dreamcast.zig"),
        .imports = &.{
            .{ .name = "arm7", .module = arm7_module },
            .{ .name = "termcolor", .module = termcolor_module },
        },
    });

    const deecy_module = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
        .imports = &.{
            .{ .name = "arm7", .module = arm7_module },
            .{ .name = "termcolor", .module = termcolor_module },
        },
        // For some reason, on Windows, ___chkstk_ms takes up to 10% of the DC thread.
        // This is an attempts at getting rid of it, but doesn't seem functional as of zig 0.14.0-dev.2577+271452d22
        // See https://github.com/ziglang/zig/issues/20724
        // Also https://nullprogram.com/blog/2024/02/05/ for more info.
        .stack_check = false,
    });

    const exe = b.addExecutable(.{
        .name = "Deecy",
        .root_module = deecy_module,
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
        },
        else => {},
    }

    { // zig-gamedev
        const zgui = b.dependency("zgui", .{
            .shared = false,
            .with_implot = true,
            .backend = .glfw_wgpu,
        });
        deecy_module.addImport("zgui", zgui.module("root"));
        deecy_module.linkLibrary(zgui.artifact("imgui"));

        @import("zgpu").addLibraryPathsTo(exe);
        @import("zgpu").addLibraryPathsTo(exe_check);

        const zglfw = b.dependency("zglfw", .{});
        deecy_module.addImport("zglfw", zglfw.module("root"));
        deecy_module.linkLibrary(zglfw.artifact("glfw"));

        const zpool = b.dependency("zpool", .{});
        deecy_module.addImport("zpool", zpool.module("root"));

        const zgpu = b.dependency("zgpu", .{ .max_num_bindings_per_group = 12 });
        deecy_module.addImport("zgpu", zgpu.module("root"));
        deecy_module.linkLibrary(zgpu.artifact("zdawn"));

        const zaudio = b.dependency("zaudio", .{});
        deecy_module.addImport("zaudio", zaudio.module("root"));
        deecy_module.linkLibrary(zaudio.artifact("miniaudio"));
    }

    const ziglz4 = b.dependency("zig-lz4", .{
        .target = target,
        .optimize = optimize,
    });
    deecy_module.addImport("lz4", ziglz4.module("zig-lz4"));

    const nfd = b.dependency("nfd", .{});
    const nfd_mod = nfd.module("nfd");
    deecy_module.addImport("nfd", nfd_mod);

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
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    // This creates a build step. It will be visible in the `zig build --help` menu,
    // and can be selected like this: `zig build run`
    // This will evaluate the `run` step rather than the default, which is "install".
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const interpreter_perf = b.addExecutable(.{
        .name = "InterpreterPerf",
        .root_source_file = b.path("test/interpreter_perf.zig"),
        .target = target,
        .optimize = .ReleaseFast, // Note: This ignores the optimization level set by the user.
    });
    interpreter_perf.root_module.addImport("termcolor", termcolor_module);
    interpreter_perf.root_module.addImport("dreamcast", dc_module);
    const run_perf_tests = b.addRunArtifact(interpreter_perf);
    const interpreter_perf_step = b.step("interpreter_perf", "Run interpreter performance tests");
    interpreter_perf_step.dependOn(&run_perf_tests.step);

    const jit_perf = b.addExecutable(.{
        .name = "JITPerf",
        .root_source_file = b.path("test/jit_perf.zig"),
        .target = target,
        .optimize = .ReleaseFast, // Note: This ignores the optimization level set by the user.
    });
    jit_perf.root_module.addImport("termcolor", termcolor_module);
    jit_perf.root_module.addImport("dreamcast", dc_module);

    const jit_pref_install = b.addInstallArtifact(jit_perf, .{});

    const run_jit_perf_tests = b.addRunArtifact(jit_perf);
    const jit_perf_step = b.step("jit_perf", "Run JIT performance tests");
    jit_perf_step.dependOn(&run_jit_perf_tests.step);
    jit_perf_step.dependOn(&jit_pref_install.step);

    const perf_step = b.step("perf", "Run performance tests");
    perf_step.dependOn(&run_jit_perf_tests.step);
    perf_step.dependOn(&jit_pref_install.step);

    const pref_install = b.addInstallArtifact(interpreter_perf, .{});
    const perf_install_step = b.step("perf_install", "Install the performance tests");
    perf_install_step.dependOn(&pref_install.step);
    perf_install_step.dependOn(&jit_pref_install.step);

    // ----- Tests ------

    // Creates a step for unit testing. This only builds the test executable
    // but does not run it.
    const unit_tests = b.addTest(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    unit_tests.root_module.addImport("arm7", arm7_module);
    unit_tests.root_module.addImport("termcolor", termcolor_module);
    const run_unit_tests = b.addRunArtifact(unit_tests);

    // Similar to creating the run step earlier, this exposes a `test` step to
    // the `zig build --help` menu, providing a way for the user to request
    // running the unit tests.
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_unit_tests.step);

    const sh4_tests = b.addTest(.{ .root_source_file = b.path("test/sh4_SingleStepTests.zig"), .target = target, .optimize = optimize });
    const sh4_module = b.createModule(.{ .root_source_file = b.path("src/sh4.zig") });
    sh4_module.addImport("arm7", arm7_module);
    sh4_module.addImport("termcolor", termcolor_module);
    sh4_tests.root_module.addImport("sh4", sh4_module);
    sh4_tests.root_module.addImport("termcolor", termcolor_module);
    const run_sh4_tests = b.addRunArtifact(sh4_tests);
    const sh4_test_step = b.step("sh4_test", "Run sh4 tests");
    sh4_test_step.dependOn(&run_sh4_tests.step);
}
