const std = @import("std");
const windows = @import("windows.zig");
const x86_64 = @import("jit/x86_64.zig");

fn handleSegfaultWindows(info: *std.os.windows.EXCEPTION_POINTERS) callconv(std.os.windows.WINAPI) c_long {
    switch (info.ExceptionRecord.ExceptionCode) {
        std.os.windows.EXCEPTION_ACCESS_VIOLATION => {
            const access_type: enum(u1) { read = 0, write = 1 } = @enumFromInt(info.ExceptionRecord.ExceptionInformation[0]);
            const fault_address = info.ExceptionRecord.ExceptionInformation[1];

            std.debug.print("  Access Violation: {s} @ {X}\n", .{ @tagName(access_type), fault_address });

            const instruction: [*]u8 = @ptrFromInt(info.ContextRecord.Rip);
            //std.debug.print("   Instr: {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}\n", .{
            //    instruction[0],
            //    instruction[1],
            //    instruction[2],
            //    instruction[3],
            //    instruction[4],
            //    instruction[5],
            //    instruction[6],
            //    instruction[7],
            //});
            const modrm: x86_64.MODRM = @bitCast(instruction[1]);
            //std.debug.print("     MODRM: {any}\n", .{modrm});

            switch (access_type) {
                .read => {
                    // TODO: Skip instruction
                    switch (instruction[0]) {
                        0x8A => {
                            switch (modrm.reg_opcode) {
                                0 => info.ContextRecord.Rax = 42,
                                1 => info.ContextRecord.Rcx = 43,
                                else => return std.os.windows.EXCEPTION_CONTINUE_SEARCH,
                            }
                            switch (modrm.mod) {
                                .indirect => info.ContextRecord.Rip += 2,
                                .disp8 => info.ContextRecord.Rip += 3,
                                .disp32 => info.ContextRecord.Rip += 6,
                                else => return std.os.windows.EXCEPTION_CONTINUE_SEARCH,
                            }
                        },
                        else => {
                            std.debug.print("REX?", .{});
                            return std.os.windows.EXCEPTION_CONTINUE_SEARCH;
                        },
                    }
                },
                .write => {
                    switch (instruction[0]) {
                        // TODO: Skip instruction
                        0xC6 => {
                            switch (modrm.mod) {
                                .indirect => info.ContextRecord.Rip += 3,
                                .disp8 => info.ContextRecord.Rip += 4,
                                .disp32 => info.ContextRecord.Rip += 7,
                                else => return std.os.windows.EXCEPTION_CONTINUE_SEARCH,
                            }
                        },
                        else => {
                            std.debug.print("REX?", .{});
                            return std.os.windows.EXCEPTION_CONTINUE_SEARCH;
                        },
                    }
                },
            }
            return windows.EXCEPTION_CONTINUE_EXECUTION; // Not defined in std
        },
        else => {
            std.debug.print("  Unhandled Exception: {}\n", .{info.ExceptionRecord.ExceptionCode});
            return std.os.windows.EXCEPTION_CONTINUE_SEARCH;
        },
    }
    return std.os.windows.EXCEPTION_CONTINUE_SEARCH;
}

pub fn main() !void {
    const reserved_memory = try std.os.windows.VirtualAlloc(
        null,
        0x1_0000_0000,
        std.os.windows.MEM_RESERVE,
        //std.os.windows.MEM_RESERVE | std.os.windows.MEM_COMMIT,
        std.os.windows.PAGE_NOACCESS,
        // std.os.windows.PAGE_READWRITE,
    );
    std.os.windows.VirtualFree(reserved_memory, 0, std.os.windows.MEM_RELEASE);
    var slice: [*]u8 = @as([*]u8, @ptrCast(reserved_memory));

    const boot = try std.os.windows.VirtualAlloc(
        @ptrFromInt(@intFromPtr(reserved_memory) + 0x0000_0000),
        0x20_0000,
        std.os.windows.MEM_RESERVE | std.os.windows.MEM_COMMIT,
        std.os.windows.PAGE_READWRITE,
    );
    var boot_slice: []u8 = @as([*]u8, @ptrCast(boot))[0..0x20_0000];

    const RAMSize = 0x100_0000;

    //const ram = try std.os.windows.VirtualAlloc(@ptrFromInt(@intFromPtr(reserved_memory) + 0x0C00_0000), RAMSize, std.os.windows.MEM_COMMIT, std.os.windows.PAGE_READWRITE);

    const ram_handle = windows.CreateFileMappingA(
        std.os.windows.INVALID_HANDLE_VALUE,
        null,
        std.os.windows.PAGE_READWRITE,
        0,
        RAMSize,
        null,
    ).?;

    //var lpflOldProtect: std.os.windows.DWORD = undefined;
    //_ = try std.os.windows.VirtualProtect(
    //    @ptrFromInt(@intFromPtr(reserved_memory) + 0x0C00_0000),
    //    RAMSize,
    //    std.os.windows.PAGE_READWRITE,
    //    &lpflOldProtect,
    //);

    for (0..4) |i| {
        const addr: *u64 = @ptrFromInt(@intFromPtr(reserved_memory) + 0x0C00_0000 + i * RAMSize);
        //_ = try std.os.windows.VirtualAlloc(
        //    addr,
        //    RAMSize,
        //    std.os.windows.MEM_RESERVE | std.os.windows.MEM_COMMIT,
        //    std.os.windows.PAGE_READWRITE,
        //);
        //if (!windows.VirtualProtect(
        //    addr,
        //    RAMSize,
        //    std.os.windows.PAGE_READWRITE,
        //    &lpflOldProtect,
        //)) {
        //    std.debug.print("VirtualProtect Error: {}\n", .{std.os.windows.GetLastError()});
        //}
        //std.os.windows.VirtualFree(addr, RAMSize, std.os.windows.MEM_RELEASE);
        const result = windows.MapViewOfFileEx(
            ram_handle,
            windows.FILE_MAP_ALL_ACCESS,
            0,
            0,
            RAMSize,
            addr,
        );

        if (result == null) {
            std.debug.print("MapViewOfFileEx({}, {X:0>8}) Error: {}\n", .{ i, addr, std.os.windows.GetLastError() });
        }
    }
    const ram: [*]u8 = @ptrFromInt(@intFromPtr(reserved_memory) + 0x0C00_0000);
    var ram_slice: []u8 = @as([*]u8, @ptrCast(ram))[0..RAMSize];

    _ = std.os.windows.kernel32.AddVectoredExceptionHandler(1, handleSegfaultWindows);

    slice[0xAA_AAAA] = 0x42;
    slice[0xBB_BBBB] = 0x64;

    std.debug.print("Base: {X:0>2} {X:0>2}\n", .{ slice[0xAA_AAAA], slice[0xBB_BBBB] });

    std.debug.print("RAM slice:  {X:0>2} {X:0>2}\n", .{ ram_slice[0], ram_slice[1] });
    std.debug.print("RAM 0x0C00_0000:  {X:0>2} {X:0>2}\n", .{ slice[0x0C00_0000], slice[0x0C00_0001] });
    std.debug.print("RAM 0x0D00_0000: {X:0>2} {X:0>2}\n", .{ slice[0x0D00_0000], slice[0x0D00_0001] });

    ram_slice[0] = 1;
    ram_slice[1] = 2;

    std.debug.print("RAM slice:  {X:0>2} {X:0>2}\n", .{ ram_slice[0], ram_slice[1] });
    std.debug.print("RAM 0x0C00_0000:  {X:0>2} {X:0>2}\n", .{ slice[0x0C00_0000], slice[0x0C00_0001] });
    std.debug.print("RAM 0x0D00_0000: {X:0>2} {X:0>2}\n", .{ slice[0x0D00_0000], slice[0x0D00_0001] });

    slice[0x0C00_0000] = 0x42;
    slice[0x0C00_0001] = 0x64;

    std.debug.print("RAM slice:  {X:0>2} {X:0>2}\n", .{ ram_slice[0], ram_slice[1] });
    std.debug.print("RAM 0x0C00_0000:  {X:0>2} {X:0>2}\n", .{ slice[0x0C00_0000], slice[0x0C00_0001] });
    std.debug.print("RAM 0x0D00_0000: {X:0>2} {X:0>2}\n", .{ slice[0x0D00_0000], slice[0x0D00_0001] });

    boot_slice[0] = 3;
    boot_slice[1] = 4;

    std.debug.print("Boot:  {X:0>2} {X:0>2}\n", .{ boot_slice[0], boot_slice[1] });

    slice[0] = 0x42;
    slice[1] = 0x64;

    std.debug.print("Boot:  {X:0>2} {X:0>2}\n", .{ boot_slice[0], boot_slice[1] });

    slice[0x0D00_0000] = 0x98;
    slice[0x0D00_0001] = 0x99;
    std.debug.print("RAM Mirror:  {X:0>2} {X:0>2}\n", .{ slice[0x0D00_0000], slice[0x0D00_0001] });
}
