const std = @import("std");

pub const EXCEPTION_CONTINUE_EXECUTION: c_long = -1;
pub const EXCEPTION_EXECUTE_HANDLER: c_long = 1;

pub const FILE_MAP_WRITE: std.os.windows.DWORD = 2;
pub const FILE_MAP_READ: std.os.windows.DWORD = 4;
pub const FILE_MAP_ALL_ACCESS: std.os.windows.DWORD = ((0x000F0000) | 0x0001 | 0x0002 | 0x0004 | 0x0008 | 0x0010);

pub const MEM_COMMIT: std.os.windows.DWORD = 0x00001000;
pub const MEM_RESERVE: std.os.windows.DWORD = 0x00002000;
pub const MEM_RESET: std.os.windows.DWORD = 0x00080000;
pub const MEM_RESET_UNDO: std.os.windows.DWORD = 0x1000000;

pub const MEM_DECOMMIT: std.os.windows.DWORD = 0x00004000;
pub const MEM_RELEASE: std.os.windows.DWORD = 0x00008000;

pub const MEM_LARGE_PAGES: std.os.windows.DWORD = 0x20000000;
pub const MEM_PHYSICAL: std.os.windows.DWORD = 0x00400000;
pub const MEM_TOP_DOWN: std.os.windows.DWORD = 0x00100000;
pub const MEM_WRITE_WATCH: std.os.windows.DWORD = 0x00200000;

pub const PAGE_NOACCESS: std.os.windows.DWORD = 0x01;
pub const PAGE_READONLY: std.os.windows.DWORD = 0x02;
pub const PAGE_READWRITE: std.os.windows.DWORD = 0x04;
pub const PAGE_WRITECOPY: std.os.windows.DWORD = 0x08;

pub const SECTION_QUERY: std.os.windows.ULONG = 1;
pub const SECTION_MAP_WRITE: std.os.windows.ULONG = 2;
pub const SECTION_MAP_READ: std.os.windows.ULONG = 4;
pub const SECTION_MAP_EXECUTE: std.os.windows.ULONG = 8;
pub const SECTION_EXTEND_SIZE: std.os.windows.ULONG = 16;
pub const SECTION_MAP_EXECUTE_EXPLICIT: std.os.windows.ULONG = 32;

pub const MEMORY_BASIC_INFORMATION = extern struct {
    BaseAddress: std.os.windows.PVOID,
    AllocationBase: std.os.windows.PVOID,
    AllocationProtect: std.os.windows.DWORD,
    PartitionId: std.os.windows.WORD,
    RegionSize: std.os.windows.SIZE_T,
    State: std.os.windows.DWORD,
    Protect: std.os.windows.DWORD,
    Type: std.os.windows.DWORD,
};

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

pub extern "kernel32" fn VirtualAlloc(
    lpAddress: ?std.os.windows.LPVOID,
    dwSize: std.os.windows.SIZE_T,
    flAllocationType: std.os.windows.DWORD,
    flProtect: std.os.windows.DWORD,
) callconv(.winapi) ?std.os.windows.LPVOID;

pub extern "kernel32" fn VirtualFree(
    lpAddress: std.os.windows.LPVOID,
    dwSize: std.os.windows.SIZE_T,
    dwFreeType: std.os.windows.DWORD,
) callconv(.winapi) std.os.windows.BOOL;

pub extern "kernel32" fn VirtualQuery(
    lpAddress: std.os.windows.LPCVOID,
    lpBuffer: *MEMORY_BASIC_INFORMATION,
    dwLength: std.os.windows.SIZE_T,
) callconv(.winapi) std.os.windows.SIZE_T;
