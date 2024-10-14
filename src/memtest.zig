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
            std.debug.print("   Instr: {X:0>2} {X:0>2} {X:0>2} {X:0>2} {X:0>2}\n", .{ instruction[0], instruction[1], instruction[2], instruction[3], instruction[4] });
            const modrm: x86_64.MODRM = @bitCast(instruction[1]);
            std.debug.print("     MODRM: {any}\n", .{modrm});

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
    const reserved_memory = try std.os.windows.VirtualAlloc(null, 0x1_0000_0000, std.os.windows.MEM_RESERVE, std.os.windows.PAGE_NOACCESS);
    defer std.os.windows.VirtualFree(reserved_memory, 0, std.os.windows.MEM_RELEASE);
    var slice: [*]u8 = @as([*]u8, @ptrCast(reserved_memory));

    const ram = try std.os.windows.VirtualAlloc(@ptrFromInt(@intFromPtr(reserved_memory) + 0x0C00_0000), 0x0100_0000, std.os.windows.MEM_COMMIT, std.os.windows.PAGE_READWRITE);
    var ram_slice: []u8 = @as([*]u8, @ptrCast(ram))[0..0x0100_0000];

    _ = std.os.windows.kernel32.AddVectoredExceptionHandler(1, handleSegfaultWindows);

    slice[0] = 93;
    slice[1] = 94;

    std.debug.print("Base: {X:0>2} {X:0>2}\n", .{ slice[0], slice[1] });

    ram_slice[0] = 0;
    ram_slice[1] = 1;

    std.debug.print("RAM: {X:0>2} {X:0>2}\n", .{ ram_slice[0], ram_slice[1] });
}
