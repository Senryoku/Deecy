const std = @import("std");
const termcolor = @import("termcolor");
const dwmapi_log = std.log.scoped(.deecy);

const zglfw = @import("zglfw");
extern fn glfwGetWin32Window(window: *zglfw.Window) u32;

const dwmapi = @cImport({
    @cInclude("dwmapi.h");
});

const HResult = packed struct(u32) {
    code: u16,
    facility: u11,
    r: u1,
    N: u1,
    C: u1,
    R: u1,
    S: u1,
};

pub fn allow_dark_mode(window: *zglfw.Window, allow: bool) void {
    const use_dark_mode: dwmapi.BOOL = if (allow) dwmapi.TRUE else dwmapi.FALSE;
    const result = dwmapi.DwmSetWindowAttribute(glfwGetWin32Window(window), dwmapi.DWMWA_USE_IMMERSIVE_DARK_MODE, &use_dark_mode, @sizeOf(dwmapi.BOOL));
    if (result != dwmapi.S_OK) {
        const hresult: HResult = @bitCast(result);
        dwmapi_log.err(termcolor.red("Failed to allow dark mode: {any}"), .{hresult});
    }
}
