const std = @import("std");

pub const EXCEPTION_CONTINUE_EXECUTION: c_long = -1;
pub const EXCEPTION_EXECUTE_HANDLER: c_long = 1;

// Some additional bindings for Windows APIs

pub extern "kernel32" fn OpenFileMappingA(
    dwDesiredAccess: std.os.windows.DWORD,
    bInheritHandle: bool,
    lpName: std.os.windows.LPCSTR,
) callconv(std.os.windows.WINAPI) ?std.os.windows.HANDLE;

pub extern "kernel32" fn CreateFileMappingA(
    hFile: std.os.windows.HANDLE,
    lpFileMappingAttributes: ?*std.os.windows.SECURITY_ATTRIBUTES,
    flProtect: std.os.windows.DWORD,
    dwMaximumSizeHigh: std.os.windows.DWORD,
    dwMaximumSizeLow: std.os.windows.DWORD,
    lpName: ?std.os.windows.LPCSTR,
) callconv(std.os.windows.WINAPI) ?std.os.windows.HANDLE;

pub extern "kernel32" fn MapViewOfFile(
    hFileMappingObject: std.os.windows.HANDLE,
    dwDesiredAccess: std.os.windows.DWORD,
    dwFileOffsetHigh: std.os.windows.DWORD,
    dwFileOffsetLow: std.os.windows.DWORD,
    dwNumberOfBytesToMap: std.os.windows.SIZE_T,
) callconv(std.os.windows.WINAPI) ?std.os.windows.LPVOID;

pub extern "kernel32" fn UnmapViewOfFile(
    lpBaseAddress: std.os.windows.LPCVOID,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
