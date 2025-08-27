const std = @import("std");

pub const EXCEPTION_CONTINUE_EXECUTION: c_long = -1;
pub const EXCEPTION_EXECUTE_HANDLER: c_long = 1;

pub const FILE_MAP_WRITE: std.os.windows.DWORD = 2;
pub const FILE_MAP_READ: std.os.windows.DWORD = 4;
pub const FILE_MAP_ALL_ACCESS: std.os.windows.DWORD = ((0x000F0000) | 0x0001 | 0x0002 | 0x0004 | 0x0008 | 0x0010);

// Some additional bindings for Windows APIs

pub extern "kernel32" fn OpenFileMappingA(
    dwDesiredAccess: std.os.windows.DWORD,
    bInheritHandle: bool,
    lpName: std.os.windows.LPCSTR,
) callconv(.winapi) ?std.os.windows.HANDLE;

pub extern "kernel32" fn CreateFileMappingA(
    hFile: std.os.windows.HANDLE,
    lpFileMappingAttributes: ?*std.os.windows.SECURITY_ATTRIBUTES,
    flProtect: std.os.windows.DWORD,
    dwMaximumSizeHigh: std.os.windows.DWORD,
    dwMaximumSizeLow: std.os.windows.DWORD,
    lpName: ?std.os.windows.LPCSTR,
) callconv(.winapi) ?std.os.windows.HANDLE;

pub extern "kernel32" fn MapViewOfFile(
    hFileMappingObject: std.os.windows.HANDLE,
    dwDesiredAccess: std.os.windows.DWORD,
    dwFileOffsetHigh: std.os.windows.DWORD,
    dwFileOffsetLow: std.os.windows.DWORD,
    dwNumberOfBytesToMap: std.os.windows.SIZE_T,
) callconv(.winapi) ?std.os.windows.LPVOID;

pub extern "kernel32" fn MapViewOfFileEx(
    hFileMappingObject: std.os.windows.HANDLE,
    dwDesiredAccess: std.os.windows.DWORD,
    dwFileOffsetHigh: std.os.windows.DWORD,
    dwFileOffsetLow: std.os.windows.DWORD,
    dwNumberOfBytesToMap: std.os.windows.SIZE_T,
    lpBaseAddress: ?std.os.windows.LPVOID,
) callconv(.winapi) ?std.os.windows.LPVOID;

pub extern "kernel32" fn UnmapViewOfFile(
    lpBaseAddress: std.os.windows.LPCVOID,
) callconv(.winapi) std.os.windows.BOOL;

pub extern "kernel32" fn VirtualProtect(
    lpAddress: std.os.windows.LPVOID,
    dwSize: std.os.windows.SIZE_T,
    flNewProtect: std.os.windows.DWORD,
    lpflOldProtect: *std.os.windows.DWORD,
) callconv(.winapi) bool;
