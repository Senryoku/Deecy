const std = @import("std");

const sh4 = @import("sh4.zig");
const interpreter = @import("sh4_interpreter.zig");

const sh4_jit = @import("jit/sh4_jit.zig");
const IRBlock = @import("jit/ir_block.zig").IRBlock;

// FIXME: This slows down compile time a lot. From 46sec to 1.2min in ReleaseSafe and 15sec to 41sec in Debug.
// pub const JumpTable: [0x10000]u8 = t: {
//     @setEvalBranchQuota(0xFFFFFFFF);
//
//     var table: [0x10000]u8 = .{1} ** 0x10000;
//
//     table[0] = 0; // NOP
//     for (1..0x10000) |i| {
//         for (2..Opcodes.len) |idx| {
//             if ((i & ~Opcodes[idx].mask) == Opcodes[idx].code) {
//                 //if (table[i] != 1) {
//                 //    std.debug.print("{b:0>16}: Matches {s} but already set to {s}\n", .{ i, Opcodes[idx].name, Opcodes[JumpTable[i]].name });
//                 //    @panic("Duplicate matching instruction.");
//                 //}
//                 table[i] = @intCast(idx);
//                 break;
//             }
//         }
//     }
//
//     break :t table;
// };

pub var JumpTable: [0x10000]u8 = .{1} ** 0x10000;

pub fn init_table() void {
    if (JumpTable[0] == 0) return;

    JumpTable[0] = 0; // NOP
    for (1..0x10000) |i| {
        for (2..Opcodes.len) |idx| {
            if ((i & ~Opcodes[idx].mask) == Opcodes[idx].code) {
                JumpTable[i] = @intCast(idx);
                break;
            }
        }
    }
}

const CacheAccess = struct {
    r0: bool = false,
    rn: bool = false,
    rm: bool = false,
    pc: bool = false,
};

pub const OpcodeDescription = struct {
    code: u16,
    mask: u16,
    fn_: *const fn (*sh4.SH4, sh4.Instr) anyerror!void,
    name: []const u8,
    is_branch: bool = false,
    privileged: bool = false,
    issue_cycles: u5 = 1,
    latency_cycles: u5 = 1,

    jit_emit_fn: *const fn (*IRBlock, *sh4_jit.JITContext, sh4.Instr) anyerror!bool = sh4_jit.interpreter_fallback_cached,

    access: struct { r: CacheAccess = .{}, w: CacheAccess = .{} } = .{}, // Used only by interpreter fallback, it's not defined for all instructions.

    pub fn use_fallback(self: *const @This()) bool {
        return self.jit_emit_fn == sh4_jit.interpreter_fallback or self.jit_emit_fn == sh4_jit.interpreter_fallback_branch or self.jit_emit_fn == sh4_jit.interpreter_fallback_cached;
    }
};

pub const Opcodes = [_]OpcodeDescription{
    // NOTE: According to MetalliC, not all technically invalid instructions causes a exception, some are treated as NOP. This includes the SH2 DSP opcodes for examples.
    //       One case where it matters is WinCE games, which might include 0 opcodes in their delay slots (maybe due to a compiler bug), hence the following special case.
    .{ .code = 0b0000000000000000, .mask = 0b1111111111111111, .fn_ = interpreter.nop, .name = "nop", .jit_emit_fn = sh4_jit.nop },
    .{ .code = 0b0000000000000000, .mask = 0b0000000000000000, .fn_ = interpreter.unknown, .name = "Unknown opcode", .latency_cycles = 1 },
    // Fake opcodes to catch emulated syscalls
    .{ .code = 0b0000000000010000, .mask = 0b0000000000000000, .fn_ = interpreter.syscall_sysinfo, .name = "Syscall Sysinfo", .issue_cycles = 1, .latency_cycles = 0, .jit_emit_fn = sh4_jit.interpreter_fallback_branch },
    .{ .code = 0b0000000000100000, .mask = 0b0000000000000000, .fn_ = interpreter.syscall_romfont, .name = "Syscall ROMFont", .issue_cycles = 1, .latency_cycles = 0, .jit_emit_fn = sh4_jit.interpreter_fallback_branch },
    .{ .code = 0b0000000000110000, .mask = 0b0000000000000000, .fn_ = interpreter.syscall_flashrom, .name = "Syscall FlashROM", .issue_cycles = 1, .latency_cycles = 0, .jit_emit_fn = sh4_jit.interpreter_fallback_branch },
    .{ .code = 0b0000000001000000, .mask = 0b0000000000000000, .fn_ = interpreter.syscall_gdrom, .name = "Syscall GDROM", .issue_cycles = 1, .latency_cycles = 0, .jit_emit_fn = sh4_jit.interpreter_fallback_branch },
    .{ .code = 0b0000000001010000, .mask = 0b0000000000000000, .fn_ = interpreter.syscall_unknown, .name = "Syscall", .issue_cycles = 1, .latency_cycles = 0, .jit_emit_fn = sh4_jit.interpreter_fallback_branch },
    .{ .code = 0b0000000001100000, .mask = 0b0000000000000000, .fn_ = interpreter.syscall_misc, .name = "Syscall Misc.", .issue_cycles = 1, .latency_cycles = 0, .jit_emit_fn = sh4_jit.interpreter_fallback_branch },

    .{ .code = 0b0110000000000011, .mask = 0b0000111111110000, .fn_ = interpreter.mov_Rm_Rn, .name = "mov Rm,Rn", .jit_emit_fn = sh4_jit.mov_Rm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1110000000000000, .mask = 0b0000111111111111, .fn_ = interpreter.mov_imm_Rn, .name = "mov #imm,Rn", .jit_emit_fn = sh4_jit.mov_imm_Rn, .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b1100011100000000, .mask = 0b0000000011111111, .fn_ = interpreter.mova_atDispPC_R0, .name = "mova @(d:8,PC),R0", .jit_emit_fn = sh4_jit.mova_atDispPC_R0, .access = .{ .r = .{ .pc = true }, .w = .{ .r0 = true } } },
    .{ .code = 0b1001000000000000, .mask = 0b0000111111111111, .fn_ = interpreter.movw_atDispPC_Rn, .name = "mov.w @(d:8,PC),Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.movw_atDispPC_Rn, .access = .{ .r = .{ .pc = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1101000000000000, .mask = 0b0000111111111111, .fn_ = interpreter.movl_atDispPC_Rn, .name = "mov.l @(d:8,PC),Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.movl_atDispPC_Rn, .access = .{ .r = .{ .pc = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000000000, .mask = 0b0000111111110000, .fn_ = interpreter.movb_atRm_Rn, .name = "mov.b @Rm,Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.mov_atRm_Rn(8), .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000000001, .mask = 0b0000111111110000, .fn_ = interpreter.movw_atRm_Rn, .name = "mov.w @Rm,Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.mov_atRm_Rn(16), .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000000010, .mask = 0b0000111111110000, .fn_ = interpreter.movl_atRm_Rn, .name = "mov.l @Rm,Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.mov_atRm_Rn(32), .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0010000000000000, .mask = 0b0000111111110000, .fn_ = interpreter.movb_Rm_atRn, .name = "mov.b Rm,@Rn", .jit_emit_fn = sh4_jit.mov_Rm_atRn(8), .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0010000000000001, .mask = 0b0000111111110000, .fn_ = interpreter.movw_Rm_atRn, .name = "mov.w Rm,@Rn", .jit_emit_fn = sh4_jit.mov_Rm_atRn(16), .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0010000000000010, .mask = 0b0000111111110000, .fn_ = interpreter.movl_Rm_atRn, .name = "mov.l Rm,@Rn", .jit_emit_fn = sh4_jit.mov_Rm_atRn(32), .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0110000000000100, .mask = 0b0000111111110000, .fn_ = interpreter.movb_atRmInc_Rn, .name = "mov.b @Rm+,Rn", .jit_emit_fn = sh4_jit.mov_atRmInc_Rn(8), .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true, .rm = true } } }, // TODO: or 2
    .{ .code = 0b0110000000000101, .mask = 0b0000111111110000, .fn_ = interpreter.movw_atRmInc_Rn, .name = "mov.w @Rm+,Rn", .jit_emit_fn = sh4_jit.mov_atRmInc_Rn(16), .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true, .rm = true } } }, // TODO: or 2
    .{ .code = 0b0110000000000110, .mask = 0b0000111111110000, .fn_ = interpreter.movl_atRmInc_Rn, .name = "mov.l @Rm+,Rn", .jit_emit_fn = sh4_jit.mov_atRmInc_Rn(32), .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true, .rm = true } } }, // TODO: or 2
    .{ .code = 0b0010000000000100, .mask = 0b0000111111110000, .fn_ = interpreter.movb_Rm_atDecRn, .name = "mov.b Rm,@-Rn", .jit_emit_fn = sh4_jit.mov_Rm_atDecRn(8), .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0010000000000101, .mask = 0b0000111111110000, .fn_ = interpreter.movw_Rm_atDecRn, .name = "mov.w Rm,@-Rn", .jit_emit_fn = sh4_jit.mov_Rm_atDecRn(16), .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0010000000000110, .mask = 0b0000111111110000, .fn_ = interpreter.movl_Rm_atDecRn, .name = "mov.l Rm,@-Rn", .jit_emit_fn = sh4_jit.mov_Rm_atDecRn(32), .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1000010000000000, .mask = 0b0000000011111111, .fn_ = interpreter.movb_atDispRm_R0, .name = "mov.b @(disp,Rm),R0", .latency_cycles = 2, .jit_emit_fn = sh4_jit.movb_atDispRm_R0, .access = .{ .r = .{ .rm = true }, .w = .{ .r0 = true } } },
    .{ .code = 0b1000010100000000, .mask = 0b0000000011111111, .fn_ = interpreter.movw_atDispRm_R0, .name = "mov.w @(disp,Rm),R0", .latency_cycles = 2, .jit_emit_fn = sh4_jit.movw_atDispRm_R0, .access = .{ .r = .{ .rm = true }, .w = .{ .r0 = true } } },
    .{ .code = 0b0101000000000000, .mask = 0b0000111111111111, .fn_ = interpreter.movl_atDispRm_Rn, .name = "mov.l @(disp,Rm),Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.movl_atDispRm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1000000000000000, .mask = 0b0000000011111111, .fn_ = interpreter.movb_R0_atDispRm, .name = "mov.b R0,@(disp,Rm)", .jit_emit_fn = sh4_jit.movb_R0_atDispRm, .access = .{ .r = .{ .r0 = true, .rm = true }, .w = .{} } },
    .{ .code = 0b1000000100000000, .mask = 0b0000000011111111, .fn_ = interpreter.movw_R0_atDispRm, .name = "mov.w R0,@(disp,Rm)", .jit_emit_fn = sh4_jit.movw_R0_atDispRm, .access = .{ .r = .{ .r0 = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0001000000000000, .mask = 0b0000111111111111, .fn_ = interpreter.movl_Rm_atDispRn, .name = "mov.l Rm,@(disp,Rn)", .jit_emit_fn = sh4_jit.movl_Rm_atDispRn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0000000000001100, .mask = 0b0000111111110000, .fn_ = interpreter.movb_atR0Rm_Rn, .name = "mov.b @(R0,Rm),Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.movb_atR0Rm_Rn, .access = .{ .r = .{ .r0 = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000001101, .mask = 0b0000111111110000, .fn_ = interpreter.movw_atR0Rm_Rn, .name = "mov.w @(R0,Rm),Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.movw_atR0Rm_Rn, .access = .{ .r = .{ .r0 = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000001110, .mask = 0b0000111111110000, .fn_ = interpreter.movl_atR0Rm_Rn, .name = "mov.l @(R0,Rm),Rn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.movl_atR0Rm_Rn, .access = .{ .r = .{ .r0 = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000000100, .mask = 0b0000111111110000, .fn_ = interpreter.movb_Rm_atR0Rn, .name = "mov.b Rm,@(R0,Rn)", .jit_emit_fn = sh4_jit.movb_Rm_atR0Rn, .access = .{ .r = .{ .r0 = true, .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0000000000000101, .mask = 0b0000111111110000, .fn_ = interpreter.movw_Rm_atR0Rn, .name = "mov.w Rm,@(R0,Rn)", .jit_emit_fn = sh4_jit.movw_Rm_atR0Rn, .access = .{ .r = .{ .r0 = true, .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0000000000000110, .mask = 0b0000111111110000, .fn_ = interpreter.movl_Rm_atR0Rn, .name = "mov.l Rm,@(R0,Rn)", .jit_emit_fn = sh4_jit.movl_Rm_atR0Rn, .access = .{ .r = .{ .r0 = true, .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b1100010000000000, .mask = 0b0000000011111111, .fn_ = interpreter.movb_atDispGBR_R0, .name = "mov.b @(d:8,GBR),R0", .jit_emit_fn = sh4_jit.mov_atDispGBR_R0(8), .latency_cycles = 2, .access = .{ .r = .{}, .w = .{ .r0 = true } } },
    .{ .code = 0b1100010100000000, .mask = 0b0000000011111111, .fn_ = interpreter.movw_atDispGBR_R0, .name = "mov.w @(d:8,GBR),R0", .jit_emit_fn = sh4_jit.mov_atDispGBR_R0(16), .latency_cycles = 2, .access = .{ .r = .{}, .w = .{ .r0 = true } } },
    .{ .code = 0b1100011000000000, .mask = 0b0000000011111111, .fn_ = interpreter.movl_atDispGBR_R0, .name = "mov.l @(d:8,GBR),R0", .jit_emit_fn = sh4_jit.mov_atDispGBR_R0(32), .latency_cycles = 2, .access = .{ .r = .{}, .w = .{ .r0 = true } } },
    .{ .code = 0b1100000000000000, .mask = 0b0000000011111111, .fn_ = interpreter.movb_R0_atDispGBR, .name = "mov.b R0,@(d:8,GBR)", .jit_emit_fn = sh4_jit.mov_R0_atDispGBR(8), .access = .{ .r = .{ .r0 = true }, .w = .{} } },
    .{ .code = 0b1100000100000000, .mask = 0b0000000011111111, .fn_ = interpreter.movw_R0_atDispGBR, .name = "mov.w R0,@(d:8,GBR)", .jit_emit_fn = sh4_jit.mov_R0_atDispGBR(16), .access = .{ .r = .{ .r0 = true }, .w = .{} } },
    .{ .code = 0b1100001000000000, .mask = 0b0000000011111111, .fn_ = interpreter.movl_R0_atDispGBR, .name = "mov.l R0,@(d:8,GBR)", .jit_emit_fn = sh4_jit.mov_R0_atDispGBR(32), .access = .{ .r = .{ .r0 = true }, .w = .{} } },
    .{ .code = 0b0000000000101001, .mask = 0b0000111100000000, .fn_ = interpreter.movt_Rn, .name = "movt Rn", .jit_emit_fn = sh4_jit.movt_Rn, .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000001000, .mask = 0b0000111111110000, .fn_ = interpreter.swapb, .name = "swap.b Rm,Rn", .jit_emit_fn = sh4_jit.swapb_Rm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000001001, .mask = 0b0000111111110000, .fn_ = interpreter.swapw, .name = "swap.w Rm,Rn", .jit_emit_fn = sh4_jit.swapw_Rm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0010000000001101, .mask = 0b0000111111110000, .fn_ = interpreter.xtrct_Rm_Rn, .name = "xtrct Rm,Rn", .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0011000000001100, .mask = 0b0000111111110000, .fn_ = interpreter.add_Rm_Rn, .name = "add Rm,Rn", .jit_emit_fn = sh4_jit.add_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0111000000000000, .mask = 0b0000111111111111, .fn_ = interpreter.add_imm_Rn, .name = "add #imm,Rn", .jit_emit_fn = sh4_jit.add_imm_Rn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0011000000001110, .mask = 0b0000111111110000, .fn_ = interpreter.addc_Rm_Rn, .name = "addc Rm,Rn", .jit_emit_fn = sh4_jit.addc_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0011000000001111, .mask = 0b0000111111110000, .fn_ = interpreter.addv_Rm_Rn, .name = "addv Rm,Rn", .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1000100000000000, .mask = 0b0000000011111111, .fn_ = interpreter.cmpeq_imm_R0, .name = "cmp/eq #imm,R0", .jit_emit_fn = sh4_jit.cmpeq_imm_R0, .access = .{ .r = .{ .r0 = true }, .w = .{} } },
    .{ .code = 0b0011000000000000, .mask = 0b0000111111110000, .fn_ = interpreter.cmpeq_Rm_Rn, .name = "cmp/eq Rm,Rn", .jit_emit_fn = sh4_jit.cmpeq_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0011000000000010, .mask = 0b0000111111110000, .fn_ = interpreter.cmphs_Rm_Rn, .name = "cmp/hs Rm,Rn", .jit_emit_fn = sh4_jit.cmphs_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0011000000000011, .mask = 0b0000111111110000, .fn_ = interpreter.cmpge_Rm_Rn, .name = "cmp/ge Rm,Rn", .jit_emit_fn = sh4_jit.cmpge_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0011000000000110, .mask = 0b0000111111110000, .fn_ = interpreter.cmphi_Rm_Rn, .name = "cmp/hi Rm,Rn", .jit_emit_fn = sh4_jit.cmphi_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0011000000000111, .mask = 0b0000111111110000, .fn_ = interpreter.cmpgt_Rm_Rn, .name = "cmp/gt Rm,Rn", .jit_emit_fn = sh4_jit.cmpgt_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0100000000010101, .mask = 0b0000111100000000, .fn_ = interpreter.cmppl_Rn, .name = "cmp/pl Rn", .jit_emit_fn = sh4_jit.cmppl_Rn, .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000000010001, .mask = 0b0000111100000000, .fn_ = interpreter.cmppz_Rn, .name = "cmp/pz Rn", .jit_emit_fn = sh4_jit.cmppz_Rn, .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0010000000001100, .mask = 0b0000111111110000, .fn_ = interpreter.cmpstr_Rm_Rn, .name = "cmp/str Rm,Rn", .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0010000000000111, .mask = 0b0000111111110000, .fn_ = interpreter.div0s_Rm_Rn, .name = "div0s Rm,Rn", .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0000000000011001, .mask = 0b0000000000000000, .fn_ = interpreter.div0u, .name = "div0u", .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b0011000000000100, .mask = 0b0000111111110000, .fn_ = interpreter.div1, .name = "div1 Rm,Rn", .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0011000000001101, .mask = 0b0000111111110000, .fn_ = interpreter.dmulsl_Rm_Rn, .name = "dmuls.l Rm,Rn", .issue_cycles = 2, .latency_cycles = 4, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0011000000000101, .mask = 0b0000111111110000, .fn_ = interpreter.dmulul_Rm_Rn, .name = "dmulu.l Rm,Rn", .issue_cycles = 2, .latency_cycles = 4, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0100000000010000, .mask = 0b0000111100000000, .fn_ = interpreter.dt_Rn, .name = "dt Rn", .jit_emit_fn = sh4_jit.dt_Rn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000001110, .mask = 0b0000111111110000, .fn_ = interpreter.extsb_Rm_Rn, .name = "exts.b Rm,Rn", .jit_emit_fn = sh4_jit.extsb_Rm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000001111, .mask = 0b0000111111110000, .fn_ = interpreter.extsw_Rm_Rn, .name = "exts.w Rm,Rn", .jit_emit_fn = sh4_jit.extsw_Rm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000001100, .mask = 0b0000111111110000, .fn_ = interpreter.extub_Rm_Rn, .name = "extu.b Rm,Rn", .jit_emit_fn = sh4_jit.extub_Rm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000001101, .mask = 0b0000111111110000, .fn_ = interpreter.extuw_Rm_Rn, .name = "extu.w Rm,Rn", .jit_emit_fn = sh4_jit.extuw_Rm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000001111, .mask = 0b0000111111110000, .fn_ = interpreter.macl_atRmInc_atRnInc, .name = "mac.l @Rm+,@Rn+", .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.macl_atRmInc_atRnInc, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true, .rm = true } } },
    .{ .code = 0b0100000000001111, .mask = 0b0000111111110000, .fn_ = interpreter.macw_atRmInc_atRnInc, .name = "mac.w @Rm+,@Rn+", .issue_cycles = 2, .latency_cycles = 4, .jit_emit_fn = sh4_jit.macw_atRmInc_atRnInc, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true, .rm = true } } },
    .{ .code = 0b0000000000000111, .mask = 0b0000111111110000, .fn_ = interpreter.mull_Rm_Rn, .name = "mul.l Rm,Rn", .issue_cycles = 2, .latency_cycles = 4, .jit_emit_fn = sh4_jit.mull_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0010000000001111, .mask = 0b0000111111110000, .fn_ = interpreter.mulsw_Rm_Rn, .name = "muls.w Rm,Rn", .issue_cycles = 2, .latency_cycles = 4, .jit_emit_fn = sh4_jit.mulsw_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0010000000001110, .mask = 0b0000111111110000, .fn_ = interpreter.muluw_Rm_Rn, .name = "mulu.w Rm,Rn", .issue_cycles = 2, .latency_cycles = 4, .jit_emit_fn = sh4_jit.muluw_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b0110000000001011, .mask = 0b0000111111110000, .fn_ = interpreter.neg_Rm_Rn, .name = "neg Rm,Rn", .jit_emit_fn = sh4_jit.neg_Rm_Rn, .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0110000000001010, .mask = 0b0000111111110000, .fn_ = interpreter.negc_Rm_Rn, .name = "negc Rm,Rn", .access = .{ .r = .{ .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0011000000001000, .mask = 0b0000111111110000, .fn_ = interpreter.sub_Rm_Rn, .name = "sub Rm,Rn", .jit_emit_fn = sh4_jit.sub_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0011000000001010, .mask = 0b0000111111110000, .fn_ = interpreter.subc_Rm_Rn, .name = "subc Rm,Rn", .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0011000000001011, .mask = 0b0000111111110000, .fn_ = interpreter.unimplemented, .name = "subv Rm,Rn", .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0010000000001001, .mask = 0b0000111111110000, .fn_ = interpreter.and_Rm_Rn, .name = "and Rm,Rn", .jit_emit_fn = sh4_jit.and_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1100100100000000, .mask = 0b0000000011111111, .fn_ = interpreter.and_imm_R0, .name = "and #imm,R0", .jit_emit_fn = sh4_jit.and_imm_R0, .access = .{ .r = .{ .r0 = true }, .w = .{ .r0 = true } } },
    .{ .code = 0b1100110100000000, .mask = 0b0000000011111111, .fn_ = interpreter.andb_imm_atR0GBR, .name = "and.b #imm,@(R0,GBR)", .issue_cycles = 4, .latency_cycles = 4, .access = .{ .r = .{ .r0 = true }, .w = .{} } },
    .{ .code = 0b0110000000000111, .mask = 0b0000111111110000, .fn_ = interpreter.not_Rm_Rn, .name = "not Rm,Rn", .jit_emit_fn = sh4_jit.not_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0010000000001011, .mask = 0b0000111111110000, .fn_ = interpreter.or_Rm_Rn, .name = "or Rm,Rn", .jit_emit_fn = sh4_jit.or_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1100101100000000, .mask = 0b0000000011111111, .fn_ = interpreter.or_imm_R0, .name = "or #imm,R0", .jit_emit_fn = sh4_jit.or_imm_R0, .access = .{ .r = .{ .r0 = true }, .w = .{ .r0 = true } } },
    .{ .code = 0b1100111100000000, .mask = 0b0000000011111111, .fn_ = interpreter.orb_imm_atR0GBR, .name = "or.b #imm,@(R0,GBR)", .issue_cycles = 4, .latency_cycles = 4, .access = .{ .r = .{ .r0 = true }, .w = .{} } },
    .{ .code = 0b0100000000011011, .mask = 0b0000111100000000, .fn_ = interpreter.tasb_atRn, .name = "tas.b @Rn", .issue_cycles = 5, .latency_cycles = 5, .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0010000000001000, .mask = 0b0000111111110000, .fn_ = interpreter.tst_Rm_Rn, .name = "tst Rm,Rn", .jit_emit_fn = sh4_jit.tst_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{} } },
    .{ .code = 0b1100100000000000, .mask = 0b0000000011111111, .fn_ = interpreter.tst_imm_R0, .name = "tst #imm,R0", .jit_emit_fn = sh4_jit.tst_imm_R0, .access = .{ .r = .{ .r0 = true }, .w = .{} } },
    .{ .code = 0b1100110000000000, .mask = 0b0000000011111111, .fn_ = interpreter.tstb_imm_atR0GBR, .name = "tst.b #imm,@(R0,GBR)", .issue_cycles = 3, .latency_cycles = 3, .access = .{ .r = .{ .r0 = true }, .w = .{} } },
    .{ .code = 0b0010000000001010, .mask = 0b0000111111110000, .fn_ = interpreter.xorRmRn, .name = "xor Rm,Rn", .jit_emit_fn = sh4_jit.xor_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1100101000000000, .mask = 0b0000000011111111, .fn_ = interpreter.xorImmR0, .name = "xor #imm,R0", .jit_emit_fn = sh4_jit.xor_imm_R0, .access = .{ .r = .{ .r0 = true }, .w = .{ .r0 = true } } },
    .{ .code = 0b1100111000000000, .mask = 0b0000000011111111, .fn_ = interpreter.xorb_imm_atR0GBR, .name = "xor.b #imm,@(R0,GBR)", .issue_cycles = 4, .latency_cycles = 4, .access = .{ .r = .{ .r0 = true }, .w = .{} } },

    .{ .code = 0b0100000000100100, .mask = 0b0000111100000000, .fn_ = interpreter.rotcl_Rn, .name = "rotcl Rn", .jit_emit_fn = sh4_jit.rotcl_Rn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000100101, .mask = 0b0000111100000000, .fn_ = interpreter.rotcr_Rn, .name = "rotcr Rn", .jit_emit_fn = sh4_jit.rotcr_Rn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000000100, .mask = 0b0000111100000000, .fn_ = interpreter.rotl_Rn, .name = "rotl Rn", .jit_emit_fn = sh4_jit.rotl_Rn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000000101, .mask = 0b0000111100000000, .fn_ = interpreter.rotr_Rn, .name = "rotr Rn", .jit_emit_fn = sh4_jit.rotr_Rn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000001100, .mask = 0b0000111111110000, .fn_ = interpreter.shad_Rm_Rn, .name = "shad Rm,Rn", .jit_emit_fn = sh4_jit.shad_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000100000, .mask = 0b0000111100000000, .fn_ = interpreter.shal_Rn, .name = "shal Rn", .jit_emit_fn = sh4_jit.shal_Rn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000100001, .mask = 0b0000111100000000, .fn_ = interpreter.shar_Rn, .name = "shar Rn", .jit_emit_fn = sh4_jit.shar_Rn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000001101, .mask = 0b0000111111110000, .fn_ = interpreter.shld_Rm_Rn, .name = "shld Rm,Rn", .jit_emit_fn = sh4_jit.shld_Rm_Rn, .access = .{ .r = .{ .rn = true, .rm = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000000000, .mask = 0b0000111100000000, .fn_ = interpreter.shll, .name = "shll Rn", .jit_emit_fn = sh4_jit.shll, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000001000, .mask = 0b0000111100000000, .fn_ = interpreter.shll2, .name = "shll2 Rn", .jit_emit_fn = sh4_jit.shll2, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000011000, .mask = 0b0000111100000000, .fn_ = interpreter.shll8, .name = "shll8 Rn", .jit_emit_fn = sh4_jit.shll8, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000101000, .mask = 0b0000111100000000, .fn_ = interpreter.shll16, .name = "shll16 Rn", .jit_emit_fn = sh4_jit.shll16, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000000001, .mask = 0b0000111100000000, .fn_ = interpreter.shlr, .name = "shlr Rn", .jit_emit_fn = sh4_jit.shlr, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000001001, .mask = 0b0000111100000000, .fn_ = interpreter.shlr2, .name = "shlr2 Rn", .jit_emit_fn = sh4_jit.shlr2, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000011001, .mask = 0b0000111100000000, .fn_ = interpreter.shlr8, .name = "shlr8 Rn", .jit_emit_fn = sh4_jit.shlr8, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000101001, .mask = 0b0000111100000000, .fn_ = interpreter.shlr16, .name = "shlr16 Rn", .jit_emit_fn = sh4_jit.shlr16, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },

    .{ .code = 0b1000101100000000, .mask = 0b0000000011111111, .fn_ = interpreter.bf_label, .name = "bf label:8", .is_branch = true, .jit_emit_fn = sh4_jit.bf_label },
    .{ .code = 0b1000111100000000, .mask = 0b0000000011111111, .fn_ = interpreter.bfs_label, .name = "bf/s label:8", .is_branch = true, .jit_emit_fn = sh4_jit.bfs_label },
    .{ .code = 0b1000100100000000, .mask = 0b0000000011111111, .fn_ = interpreter.bt_label, .name = "bt label:8", .is_branch = true, .jit_emit_fn = sh4_jit.bt_label },
    .{ .code = 0b1000110100000000, .mask = 0b0000000011111111, .fn_ = interpreter.bts_label, .name = "bt/s label:8", .is_branch = true, .jit_emit_fn = sh4_jit.bts_label },
    .{ .code = 0b1010000000000000, .mask = 0b0000111111111111, .fn_ = interpreter.bra_label, .name = "bra label:12", .is_branch = true, .jit_emit_fn = sh4_jit.bra_label },
    .{ .code = 0b0000000000100011, .mask = 0b0000111100000000, .fn_ = interpreter.braf_Rn, .name = "braf Rn", .is_branch = true, .issue_cycles = 2, .latency_cycles = 3, .jit_emit_fn = sh4_jit.braf_Rn },
    .{ .code = 0b1011000000000000, .mask = 0b0000111111111111, .fn_ = interpreter.bsr_label, .name = "bsr label:12", .is_branch = true, .latency_cycles = 2, .jit_emit_fn = sh4_jit.bsr_label },
    .{ .code = 0b0000000000000011, .mask = 0b0000111100000000, .fn_ = interpreter.bsrf_Rn, .name = "bsrf Rn", .is_branch = true, .issue_cycles = 2, .latency_cycles = 3, .jit_emit_fn = sh4_jit.bsrf_Rn },
    .{ .code = 0b0100000000101011, .mask = 0b0000111100000000, .fn_ = interpreter.jmp_atRn, .name = "jmp @Rn", .is_branch = true, .issue_cycles = 2, .latency_cycles = 3, .jit_emit_fn = sh4_jit.jmp_atRn },
    .{ .code = 0b0100000000001011, .mask = 0b0000111100000000, .fn_ = interpreter.jsr_Rn, .name = "jsr @Rn", .is_branch = true, .issue_cycles = 2, .latency_cycles = 3, .jit_emit_fn = sh4_jit.jsr_rn },
    .{ .code = 0b0000000000001011, .mask = 0b0000000000000000, .fn_ = interpreter.rts, .name = "rts", .is_branch = true, .issue_cycles = 2, .latency_cycles = 3, .jit_emit_fn = sh4_jit.rts },

    .{ .code = 0b0000000000101000, .mask = 0b0000000000000000, .fn_ = interpreter.clrmac, .name = "clrmac", .latency_cycles = 3, .jit_emit_fn = sh4_jit.clrmac, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b0000000001001000, .mask = 0b0000000000000000, .fn_ = interpreter.clrs, .name = "clrs", .jit_emit_fn = sh4_jit.clrs, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b0000000000001000, .mask = 0b0000000000000000, .fn_ = interpreter.clrt, .name = "clrt", .jit_emit_fn = sh4_jit.clrt, .access = .{ .r = .{}, .w = .{} } },

    // NOTE: These two might switch register banks.
    .{ .code = 0b0100000000001110, .mask = 0b0000111100000000, .fn_ = interpreter.ldc_Rn_SR, .name = "ldc Rn,SR", .privileged = true, .issue_cycles = 4, .latency_cycles = 4, .jit_emit_fn = sh4_jit.ldc_Rn_SR, .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000000000111, .mask = 0b0000111100000000, .fn_ = interpreter.ldcl_atRnInc_SR, .name = "ldc.l @Rn+,SR", .privileged = true, .issue_cycles = 4, .latency_cycles = 4, .jit_emit_fn = sh4_jit.ldcl_atRnInc_SR, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000011110, .mask = 0b0000111100000000, .fn_ = interpreter.ld_Rn_Reg("gbr"), .name = "ldc Rn,GBR", .issue_cycles = 3, .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldc_Rn_Reg("gbr"), .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000000101110, .mask = 0b0000111100000000, .fn_ = interpreter.ld_Rn_Reg("vbr"), .name = "ldc Rn,VBR", .privileged = true, .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldc_Rn_Reg("vbr"), .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000001001110, .mask = 0b0000111100000000, .fn_ = interpreter.ld_Rn_Reg("spc"), .name = "ldc Rn,SPC", .privileged = true, .issue_cycles = 3, .jit_emit_fn = sh4_jit.ldc_Rn_Reg("spc"), .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000000111110, .mask = 0b0000111100000000, .fn_ = interpreter.ld_Rn_Reg("ssr"), .name = "ldc Rn,SSR", .privileged = true, .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldc_Rn_Reg("ssr"), .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000011111010, .mask = 0b0000111100000000, .fn_ = interpreter.ld_Rn_Reg("dbr"), .name = "ldc Rn,DBR", .privileged = true, .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldc_Rn_Reg("dbr"), .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000000001010, .mask = 0b0000111100000000, .fn_ = interpreter.ld_Rn_Reg("mach"), .name = "lds Rn,MACH", .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldc_Rn_Reg("mach"), .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000000011010, .mask = 0b0000111100000000, .fn_ = interpreter.ld_Rn_Reg("macl"), .name = "lds Rn,MACL", .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldc_Rn_Reg("macl"), .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000000101010, .mask = 0b0000111100000000, .fn_ = interpreter.ld_Rn_Reg("pr"), .name = "lds Rn,PR", .issue_cycles = 2, .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldc_Rn_Reg("pr"), .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000010001110, .mask = 0b0000111101110000, .fn_ = interpreter.ldc_Rn_Rm_BANK, .name = "ldc Rn,Rm_BANK", .privileged = true, .latency_cycles = 3, .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0100000000010111, .mask = 0b0000111100000000, .fn_ = interpreter.ldl_atRnInc_Reg("gbr"), .name = "ldc.l @Rn+,GBR", .issue_cycles = 3, .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldcl_atRnInc_Reg("gbr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000100111, .mask = 0b0000111100000000, .fn_ = interpreter.ldl_atRnInc_Reg("vbr"), .name = "ldc.l @Rn+,VBR", .privileged = true, .jit_emit_fn = sh4_jit.ldcl_atRnInc_Reg("vbr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000110111, .mask = 0b0000111100000000, .fn_ = interpreter.ldl_atRnInc_Reg("ssr"), .name = "ldc.l @Rn+,SSR", .privileged = true, .jit_emit_fn = sh4_jit.ldcl_atRnInc_Reg("ssr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000001000111, .mask = 0b0000111100000000, .fn_ = interpreter.ldl_atRnInc_Reg("spc"), .name = "ldc.l @Rn+,SPC", .privileged = true, .jit_emit_fn = sh4_jit.ldcl_atRnInc_Reg("spc"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000011110110, .mask = 0b0000111100000000, .fn_ = interpreter.ldl_atRnInc_Reg("dbr"), .name = "ldc.l @Rn+,DBR", .privileged = true, .jit_emit_fn = sh4_jit.ldcl_atRnInc_Reg("dbr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000000110, .mask = 0b0000111100000000, .fn_ = interpreter.ldl_atRnInc_Reg("mach"), .name = "lds.l @Rn+,MACH", .jit_emit_fn = sh4_jit.ldcl_atRnInc_Reg("mach"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000010110, .mask = 0b0000111100000000, .fn_ = interpreter.ldl_atRnInc_Reg("macl"), .name = "lds.l @Rn+,MACL", .jit_emit_fn = sh4_jit.ldcl_atRnInc_Reg("macl"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000100110, .mask = 0b0000111100000000, .fn_ = interpreter.ldl_atRnInc_Reg("pr"), .name = "lds.l @Rn+,PR", .jit_emit_fn = sh4_jit.ldcl_atRnInc_Reg("pr"), .issue_cycles = 2, .latency_cycles = 2, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000010000111, .mask = 0b0000111101110000, .fn_ = interpreter.ldcl_atRnInc_Rm_BANK, .name = "ldc.l @Rn+,Rm_BANK", .privileged = true, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000111000, .mask = 0b0000000000000000, .fn_ = interpreter.ldtlb, .name = "ldtlb", .privileged = true, .access = .{ .r = .{}, .w = .{} } },

    //.{ .code = 0b0000000011000011, .mask = 0b0000111100000000, .fn_ = interpreter.movcal_R0_atRn, .name = "movca.l R0,@Rn", .jit_emit_fn = sh4_jit.movcal_R0_atRn, .latency_cycles = 3, .access = .{ .r = .{ .r0 = true, .rn = true }, .w = .{} } },
    .{ .code = 0b0000000011000011, .mask = 0b0000111100000000, .fn_ = interpreter.movcal_R0_atRn, .name = "movca.l R0,@Rn", .latency_cycles = 3, .access = .{ .r = .{ .r0 = true, .rn = true }, .w = .{} } },
    .{ .code = 0b0000000000001001, .mask = 0b0000000000000000, .fn_ = interpreter.nop, .name = "nop", .latency_cycles = 0, .jit_emit_fn = sh4_jit.nop },
    .{ .code = 0b0000000010010011, .mask = 0b0000111100000000, .fn_ = interpreter.ocbi_atRn, .name = "ocbi @Rn", .jit_emit_fn = sh4_jit.nop, .access = .{ .r = .{ .rn = true } } },
    .{ .code = 0b0000000010100011, .mask = 0b0000111100000000, .fn_ = interpreter.ocbp_atRn, .name = "ocbp @Rn", .jit_emit_fn = sh4_jit.nop, .access = .{ .r = .{ .rn = true } } },
    // .{ .code = 0b0000000010110011, .mask = 0b0000111100000000, .fn_ = interpreter.ocbwb_atRn, .name = "ocbwb @Rn", .jit_emit_fn = sh4_jit.nop, .access = .{ .r = .{ .rn = true } } },
    .{ .code = 0b0000000010110011, .mask = 0b0000111100000000, .fn_ = interpreter.ocbwb_atRn, .name = "ocbwb @Rn", .access = .{ .r = .{ .rn = true } } },
    .{ .code = 0b0000000010000011, .mask = 0b0000111100000000, .fn_ = interpreter.pref_atRn, .name = "pref @Rn", .access = .{ .r = .{ .rn = true } } },
    .{ .code = 0b0000000000101011, .mask = 0b0000000000000000, .fn_ = interpreter.rte, .name = "rte", .is_branch = true, .privileged = true, .issue_cycles = 5, .latency_cycles = 5, .jit_emit_fn = sh4_jit.rte },
    .{ .code = 0b0000000001011000, .mask = 0b0000000000000000, .fn_ = interpreter.sets, .name = "sets", .jit_emit_fn = sh4_jit.sets, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b0000000000011000, .mask = 0b0000000000000000, .fn_ = interpreter.sett, .name = "sett", .jit_emit_fn = sh4_jit.sett, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b0000000000011011, .mask = 0b0000000000000000, .fn_ = interpreter.sleep, .name = "sleep", .privileged = true, .issue_cycles = 4, .latency_cycles = 4, .access = .{ .r = .{}, .w = .{} } },

    .{ .code = 0b0000000000000010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("sr"), .name = "stc SR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stc_Reg_Rn("sr"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000010010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("gbr"), .name = "stc GBR,Rn", .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stc_Reg_Rn("gbr"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000100010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("vbr"), .name = "stc VBR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stc_Reg_Rn("vbr"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000111010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("sgr"), .name = "stc SGR,Rn", .privileged = true, .issue_cycles = 3, .latency_cycles = 3, .jit_emit_fn = sh4_jit.stc_Reg_Rn("sgr"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000110010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("ssr"), .name = "stc SSR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stc_Reg_Rn("ssr"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000001000010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("spc"), .name = "stc SPC,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stc_Reg_Rn("spc"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000011111010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("dbr"), .name = "stc DBR,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stc_Reg_Rn("dbr"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000001010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("mach"), .name = "sts MACH,Rn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.stc_Reg_Rn("mach"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000011010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("macl"), .name = "sts MACL,Rn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.stc_Reg_Rn("macl"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000000101010, .mask = 0b0000111100000000, .fn_ = interpreter.stc_Reg_Rn("pr"), .name = "sts PR,Rn", .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stc_Reg_Rn("pr"), .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0000000010000010, .mask = 0b0000111101110000, .fn_ = interpreter.stc_Rm_BANK_Rn, .name = "stc Rm_BANK,Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000000011, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("sr"), .name = "stc.l SR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("sr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000010011, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("gbr"), .name = "stc.l GBR,@-Rn", .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("gbr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000100011, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("vbr"), .name = "stc.l VBR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("vbr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000110010, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("sgr"), .name = "stc.l SGR,@-Rn", .privileged = true, .issue_cycles = 3, .latency_cycles = 3, .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("sgr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000110011, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("ssr"), .name = "stc.l SSR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("ssr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000001000011, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("spc"), .name = "stc.l SPC,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("spc"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000011110010, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("dbr"), .name = "stc.l DBR,@-Rn", .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("dbr"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000000010, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("mach"), .name = "sts.l MACH,@-Rn", .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("mach"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000010010, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("macl"), .name = "sts.l MACL,@-Rn", .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("macl"), .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000000100010, .mask = 0b0000111100000000, .fn_ = interpreter.stcl_Reg_atDecRn("pr"), .name = "sts.l PR,@-Rn", .jit_emit_fn = sh4_jit.stcl_Reg_atDecRn("pr"), .issue_cycles = 2, .latency_cycles = 2, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000010000011, .mask = 0b0000111101110000, .fn_ = interpreter.stcl_Rm_BANK_atDecRn, .name = "stc.l Rm_BANK,@-Rn", .jit_emit_fn = sh4_jit.stcl_Rm_BANK_atDecRn, .privileged = true, .issue_cycles = 2, .latency_cycles = 2, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1100001100000000, .mask = 0b0000000011111111, .fn_ = interpreter.trapa_imm, .name = "trapa #imm", .issue_cycles = 7, .latency_cycles = 7, .access = .{ .r = .{}, .w = .{} } },

    .{ .code = 0b1111000000001100, .mask = 0b0000111111110000, .fn_ = interpreter.fmov_FRm_FRn, .name = "fmov FRm,FRn", .latency_cycles = 0, .jit_emit_fn = sh4_jit.fmov_FRm_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000001000, .mask = 0b0000111111110000, .fn_ = interpreter.fmovs_atRm_FRn, .name = "fmov.s @Rm,FRn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.fmovs_atRm_FRn, .access = .{ .r = .{ .rm = true }, .w = .{} } },
    .{ .code = 0b1111000000001010, .mask = 0b0000111111110000, .fn_ = interpreter.fmovs_FRm_atRn, .name = "fmov.s FRm,@Rn", .jit_emit_fn = sh4_jit.fmovs_FRm_atRn, .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b1111000000001001, .mask = 0b0000111111110000, .fn_ = interpreter.fmovs_atRmInc_FRn, .name = "fmov.s @Rm+,FRn", .jit_emit_fn = sh4_jit.fmovs_atRmInc_FRn, .access = .{ .r = .{ .rm = true }, .w = .{ .rm = true } } },
    .{ .code = 0b1111000000001011, .mask = 0b0000111111110000, .fn_ = interpreter.fmovs_FRm_atDecRn, .name = "fmov.s FRm,@-Rn", .jit_emit_fn = sh4_jit.fmovs_FRm_atDecRn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1111000000000110, .mask = 0b0000111111110000, .fn_ = interpreter.fmovs_atR0Rm_FRn, .name = "fmov.s @(R0,Rm),FRn", .jit_emit_fn = sh4_jit.fmovs_atR0Rm_FRn, .latency_cycles = 2, .access = .{ .r = .{ .r0 = true, .rm = true }, .w = .{} } },
    .{ .code = 0b1111000000000111, .mask = 0b0000111111110000, .fn_ = interpreter.fmovs_FRm_atR0Rn, .name = "fmov.s FRm,@(R0,Rn)", .jit_emit_fn = sh4_jit.fmovs_FRm_atR0Rn, .access = .{ .r = .{ .r0 = true, .rn = true }, .w = .{} } },

    // Handled by single precision version - Switched by SR register sz flag
    //.{ .code = 0b1111000000001100, .mask = 0b0000111011100000, .fn_ = fmov_DRm_DRn, .name = "fmov DRm,DRn", .latency_cycles = 0 },
    //.{ .code = 0b1111000100001100, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fmov DRm,XDn", .latency_cycles = 0 },
    //.{ .code = 0b1111000000011100, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fmov XDm,DRn", .latency_cycles = 0 },
    //.{ .code = 0b1111000100011100, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fmov XDm,XDn", .latency_cycles = 0 },
    //.{ .code = 0b1111000000001000, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @Rm,DRn", .latency_cycles = 2 },
    //.{ .code = 0b1111000100001000, .mask = 0b0000111011110000, .fn_ = fmovd_atRm_XDn, .name = "fmov.d @Rm,XDn", .latency_cycles = 2 },
    //.{ .code = 0b1111000000001010, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d DRm,@Rn", .latency_cycles = 1 },
    //.{ .code = 0b1111000000011010, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d XDm,@Rn", .latency_cycles = 1 },
    //.{ .code = 0b1111000000001001, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @Rm+,DRn" },
    //.{ .code = 0b1111000100001001, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @Rm+,XDn" },
    //.{ .code = 0b1111000000001011, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d DRm,@-Rn" },
    //.{ .code = 0b1111000000011011, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d XDm,@-Rn" },
    //.{ .code = 0b1111000000000110, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @(R0,Rm),DRn", .latency_cycles = 2 },
    //.{ .code = 0b1111000100000110, .mask = 0b0000111011110000, .fn_ = unimplemented, .name = "fmov.d @(R0,Rm),XDn", .latency_cycles = 2 },
    //.{ .code = 0b1111000000000111, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d DRm,@(R0,Rn)", .latency_cycles = 1 },
    //.{ .code = 0b1111000000010111, .mask = 0b0000111111100000, .fn_ = unimplemented, .name = "fmov.d XDm,@(R0,Rn)", .latency_cycles = 1 },

    .{ .code = 0b1111000010001101, .mask = 0b0000111100000000, .fn_ = interpreter.fldi0_FRn, .name = "fldi0 FRn", .latency_cycles = 0, .jit_emit_fn = sh4_jit.fldi0_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000010011101, .mask = 0b0000111100000000, .fn_ = interpreter.fldi1_FRn, .name = "fldi1 FRn", .latency_cycles = 0, .jit_emit_fn = sh4_jit.fldi1_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000011101, .mask = 0b0000111100000000, .fn_ = interpreter.flds_FRn_FPUL, .name = "flds FRm,FPUL", .latency_cycles = 0, .jit_emit_fn = sh4_jit.flds_FRn_FPUL, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000001101, .mask = 0b0000111100000000, .fn_ = interpreter.fsts_FPUL_FRn, .name = "fsts FPUL,FRn", .latency_cycles = 0, .jit_emit_fn = sh4_jit.fsts_FPUL_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000001011101, .mask = 0b0000111100000000, .fn_ = interpreter.fabs_FRn, .name = "fabs FRn", .latency_cycles = 0, .jit_emit_fn = sh4_jit.fabs_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000001001101, .mask = 0b0000111100000000, .fn_ = interpreter.fneg_FRn, .name = "fneg FRn", .latency_cycles = 0, .jit_emit_fn = sh4_jit.fneg_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000000000, .mask = 0b0000111111110000, .fn_ = interpreter.fadd_FRm_FRn, .name = "fadd FRm,FRn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.fadd_FRm_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000000001, .mask = 0b0000111111110000, .fn_ = interpreter.fsub_FRm_FRn, .name = "fsub FRm,FRn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.fsub_FRm_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000000010, .mask = 0b0000111111110000, .fn_ = interpreter.fmul_FRm_FRn, .name = "fmul FRm,FRn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.fmul_FRm_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000001110, .mask = 0b0000111111110000, .fn_ = interpreter.fmac_FR0_FRm_FRn, .name = "fmac FR0,FRm,FRn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.fmac_FR0_FRm_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000000011, .mask = 0b0000111111110000, .fn_ = interpreter.fdiv_FRm_FRn, .name = "fdiv FRm,FRn", .latency_cycles = 11, .jit_emit_fn = sh4_jit.fdiv_FRm_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000001101101, .mask = 0b0000111100000000, .fn_ = interpreter.fsqrt_FRn, .name = "fsqrt FRn", .latency_cycles = 11, .jit_emit_fn = sh4_jit.fsqrt_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000000100, .mask = 0b0000111111110000, .fn_ = interpreter.fcmp_eq_FRm_FRn, .name = "fcmp/eq FRm,FRn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.fcmp_eq_FRm_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000000101, .mask = 0b0000111111110000, .fn_ = interpreter.fcmp_gt_FRm_FRn, .name = "fcmp/gt FRm,FRn", .latency_cycles = 2, .jit_emit_fn = sh4_jit.fcmp_gt_FRm_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000101101, .mask = 0b0000111100000000, .fn_ = interpreter.float_FPUL_FRn, .name = "float FPUL,FRn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.float_FPUL_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000000111101, .mask = 0b0000111100000000, .fn_ = interpreter.ftrc_FRn_FPUL, .name = "ftrc FRn,FPUL", .latency_cycles = 3, .jit_emit_fn = sh4_jit.ftrc_FRn_FPUL, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000011101101, .mask = 0b0000111100000000, .fn_ = interpreter.fipr_FVm_FVn, .name = "fipr FVm,FVn", .latency_cycles = 4, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000111111101, .mask = 0b0000110000000000, .fn_ = interpreter.ftrv_XMTRX_FVn, .name = "ftrv XMTRX,FVn", .latency_cycles = 5, .access = .{ .r = .{}, .w = .{} } },
    // Undocumented opcodes - Supposed to be exclusive to the SH4A, but the SH7091 seem to have them.
    .{ .code = 0b1111000001111101, .mask = 0b0000111100000000, .fn_ = interpreter.fsrra_FRn, .name = "fsrra FRn", .jit_emit_fn = sh4_jit.fsrra_FRn, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000011111101, .mask = 0b0000111000000000, .fn_ = interpreter.fsca_FPUL_DRn, .name = "fsca FPUL,DRn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.fsca_FPUL_DRn, .access = .{ .r = .{}, .w = .{} } },

    // Handled by single precision version - Switched by SR register pr flag
    //.{ .code = 0b1111000001011101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fabs DRn" },
    //.{ .code = 0b1111000001001101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fneg DRn" },
    //.{ .code = 0b1111000000000000, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fadd DRm,DRn", .latency_cycles = 7 },
    //.{ .code = 0b1111000000000001, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fsub DRm,DRn", .latency_cycles = 7 },
    //.{ .code = 0b1111000000000010, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fmul DRm,DRn", .latency_cycles = 7 },
    //.{ .code = 0b1111000000000011, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fdiv DRm,DRn", .latency_cycles = 24 },
    //.{ .code = 0b1111000001101101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "fsqrt DRn", .latency_cycles = 23 },
    //.{ .code = 0b1111000000000100, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fcmp/eq DRm,DRn", .latency_cycles = 3 },
    //.{ .code = 0b1111000000000101, .mask = 0b0000111011100000, .fn_ = unimplemented, .name = "fcmp/gt DRm,DRn", .latency_cycles = 3 },
    //.{ .code = 0b1111000000101101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "float FPUL,DRn", .latency_cycles = 3 },
    //.{ .code = 0b1111000000111101, .mask = 0b0000111000000000, .fn_ = unimplemented, .name = "ftrc DRm,FPUL", .latency_cycles = 4 },
    .{ .code = 0b1111000010111101, .mask = 0b0000111000000000, .fn_ = interpreter.fcnvds_DRn_FPUL, .name = "fcnvds DRn,FPUL", .latency_cycles = 4, .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111000010101101, .mask = 0b0000111000000000, .fn_ = interpreter.fcnvsd_FPUL_DRn, .name = "fcnvsd FPUL,DRn", .latency_cycles = 3, .access = .{ .r = .{}, .w = .{} } },

    .{ .code = 0b0100000001101010, .mask = 0b0000111100000000, .fn_ = interpreter.lds_Rn_FPSCR, .name = "lds Rn,FPSCR", .latency_cycles = 4, .jit_emit_fn = sh4_jit.lds_rn_FPSCR, .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0000000001101010, .mask = 0b0000111100000000, .fn_ = interpreter.sts_FPSCR_Rn, .name = "sts FPSCR,Rn", .latency_cycles = 3, .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0100000001100110, .mask = 0b0000111100000000, .fn_ = interpreter.ldsl_atRnInc_FPSCR, .name = "lds.l @Rn+,FPSCR", .latency_cycles = 3, .jit_emit_fn = sh4_jit.ldsl_atRnInc_FPSCR, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000001100010, .mask = 0b0000111100000000, .fn_ = interpreter.stsl_FPSCR_atDecRn, .name = "sts.l FPSCR,@-Rn", .jit_emit_fn = sh4_jit.stsl_FPSCR_atDecRn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000001011010, .mask = 0b0000111100000000, .fn_ = interpreter.lds_Rn_FPUL, .name = "lds Rn,FPUL", .jit_emit_fn = sh4_jit.lds_Rn_FPUL, .access = .{ .r = .{ .rn = true }, .w = .{} } },
    .{ .code = 0b0000000001011010, .mask = 0b0000111100000000, .fn_ = interpreter.sts_FPUL_Rn, .name = "sts FPUL,Rn", .latency_cycles = 3, .jit_emit_fn = sh4_jit.sts_FPUL_Rn, .access = .{ .r = .{}, .w = .{ .rn = true } } },
    .{ .code = 0b0100000001010110, .mask = 0b0000111100000000, .fn_ = interpreter.ldsl_atRnInc_FPUL, .name = "lds.l @Rn+,FPUL", .jit_emit_fn = sh4_jit.ldsl_atRnInc_FPUL, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b0100000001010010, .mask = 0b0000111100000000, .fn_ = interpreter.stsl_FPUL_atDecRn, .name = "sts.l FPUL,@-Rn", .jit_emit_fn = sh4_jit.stsl_FPUL_atDecRn, .access = .{ .r = .{ .rn = true }, .w = .{ .rn = true } } },
    .{ .code = 0b1111101111111101, .mask = 0b0000000000000000, .fn_ = interpreter.frchg, .name = "frchg", .access = .{ .r = .{}, .w = .{} } },
    .{ .code = 0b1111001111111101, .mask = 0b0000000000000000, .fn_ = interpreter.fschg, .name = "fschg", .jit_emit_fn = sh4_jit.fschg, .access = .{ .r = .{}, .w = .{} } },
};
