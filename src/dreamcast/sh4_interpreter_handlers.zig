const sh4 = @import("sh4.zig");
const Opcodes = @import("sh4_instructions.zig").Opcodes;

const JITCallingConvention = @import("jit/x86_64.zig").CallingConvention;

/// Generate a unique function for each possible instruction.
/// Saves an indirection and lets the compiler optimize them further.
/// Slows down compilation by a lot.
pub const Enable = false;

const ForceInline = false; // Turn this on for a slight performance boost, stupidly long compile times and bloated binary :)

fn InstructionHandler(comptime idx: usize, comptime instruction: u16) type {
    const instr: sh4.Instr = comptime @bitCast(instruction);
    return struct {
        fn handler(cpu: *sh4.SH4) callconv(JITCallingConvention) void {
            @setRuntimeSafety(false);
            if (comptime ForceInline) {
                @call(.always_inline, comptime Opcodes[idx].fn_, .{ cpu, comptime instr });
            } else {
                Opcodes[idx].fn_(cpu, comptime instr);
            }
            cpu.add_cycles(comptime Opcodes[idx].issue_cycles);
        }
    };
}

pub const InstructionHandlers = t: {
    if (!Enable) break :t {};

    @setEvalBranchQuota(0xFFFFFFFF);

    var table: [0x10000]fn (*sh4.SH4) callconv(JITCallingConvention) void = undefined;

    for (1..0x10000) |i| {
        for (2..Opcodes.len) |idx| {
            if ((i & ~Opcodes[idx].mask) == Opcodes[idx].code) {
                table[i] = InstructionHandler(idx, @intCast(i)).handler;
                break;
            }
        }
    }

    break :t table;
};
