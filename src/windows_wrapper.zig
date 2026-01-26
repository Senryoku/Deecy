const std = @import("std");

// Wrapper calling Deecy.exe to get proper console interaction when launched from a terminal on Windows.
// Windows forces to select the GUI or Console subsystem at compilation time, you can't have a executable
// that will behave correctly in both modes.
// You can reattach to the console from a GUI executable to get logs, but it won't block the prompt,
// this solution is the best I could find, even if it requires a second executable and a second process.

pub fn main() void {
    const cmd_line = std.os.windows.peb().ProcessParameters.CommandLine;
    const raw_cmd = cmd_line.Buffer.?[0 .. cmd_line.Length / 2 :0];

    var end_of_exe: usize = 0;
    if (raw_cmd[0] == '"') {
        if (std.mem.indexOfScalar(u16, raw_cmd[1..], '"')) |quote| {
            end_of_exe = quote + 2;
        } else end_of_exe = raw_cmd.len;
    } else {
        end_of_exe = std.mem.indexOfScalar(u16, raw_cmd, ' ') orelse raw_cmd.len;
    }
    const args = raw_cmd[end_of_exe..];

    const exe_flag = std.unicode.utf8ToUtf16LeStringLiteral("Deecy.exe --attach-console ");

    const cmd_size = exe_flag.len + args.len + 1;
    if (cmd_size > 8192) std.os.windows.kernel32.ExitProcess(1);

    var buf: [8192]u16 = undefined;
    const new_cmd = buf[0..cmd_size];
    @memcpy(new_cmd[0..exe_flag.len], exe_flag);
    @memcpy(new_cmd[exe_flag.len .. exe_flag.len + args.len], args);
    new_cmd[new_cmd.len - 1] = 0;

    var si = std.mem.zeroInit(std.os.windows.STARTUPINFOW, .{ .cb = @sizeOf(std.os.windows.STARTUPINFOW) });
    var pi: std.os.windows.PROCESS_INFORMATION = undefined;
    if (std.os.windows.kernel32.CreateProcessW(
        null, // Executable Name (passed via the command line)
        new_cmd[0.. :0], // Command Line
        null, // Process Attributes
        null, // Thread Attributes
        std.os.windows.TRUE, // Inherit Handles
        .{}, // Flags
        null, // Environment
        null, // Current Directory
        &si,
        &pi,
    ) == 0) {
        std.os.windows.kernel32.ExitProcess(1);
    }

    _ = std.os.windows.kernel32.WaitForSingleObject(pi.hProcess, std.os.windows.INFINITE);
    var exit_code: std.os.windows.DWORD = 0;
    _ = std.os.windows.kernel32.GetExitCodeProcess(pi.hProcess, &exit_code);

    _ = std.os.windows.CloseHandle(pi.hProcess);
    _ = std.os.windows.CloseHandle(pi.hThread);

    std.os.windows.kernel32.ExitProcess(exit_code);
}
