const std = @import("std");

pub fn link(exe: *std.build.LibExeObjStep) void {
    exe.linkSystemLibrary("xinput1_4");
}

pub const pkg = std.build.Pkg{
    .name = "up_rumble",
    .source = .{ .path = thisDir() ++ "/src/rumble.zig" },
};

inline fn thisDir() []const u8 {
    return comptime std.fs.path.dirname(@src().file) orelse ".";
}

// How to use:

// Go on your build.zig and add it:

//const up_rumble = @import("some_path_you_put_it/up_rumble/build.zig");
//
//pub fn build(b: *std.build.Builder) void {
//    ...
//    exe.addPackage(up_rumble.pkg);
//
//    up_rumble.link(exe);
//}

// Then

// const rumble = @import("up_rumble");

// Read src/up_rumble.h for morsels of information.