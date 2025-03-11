const std = @import("std");
const builtin = @import("builtin");

const termcolor = @import("termcolor");

const sh4 = @import("sh4.zig");
const SH4 = sh4.SH4;
const Instr = sh4.Instr;
const sh4_log = sh4.sh4_log;

const sh4_instructions = @import("sh4_instructions.zig");

const syscall = @import("syscall.zig");

const bit_manip = @import("bit_manip.zig");
const zero_extend = bit_manip.zero_extend;
const sign_extension_u8 = bit_manip.sign_extension_u8;
const sign_extension_u12 = bit_manip.sign_extension_u12;
const sign_extension_u16 = bit_manip.sign_extension_u16;
const as_i32 = bit_manip.as_i32;

const Experimental = struct {
    // https://godbolt.org/#g:!((g:!((g:!((h:codeEditor,i:(filename:'1',fontScale:14,fontUsePx:'0',j:1,lang:zig,selection:(endColumn:2,endLineNumber:13,positionColumn:2,positionLineNumber:13,selectionStartColumn:2,selectionStartLineNumber:13,startColumn:2,startLineNumber:13),source:'const+std+%3D+@import(%22std%22)%3B%0A%0Aexport+fn+fpir_fma(FVn:+@Vector(4,+f32),+FVm:+@Vector(4,+f32))+f32+%7B%0A++++var+tmp:+f32+%3D+FVn%5B0%5D+*+FVm%5B0%5D%3B%0A++++tmp+%3D+@mulAdd(f32,+FVn%5B1%5D,+FVm%5B1%5D,+tmp)%3B%0A++++tmp+%3D+@mulAdd(f32,+FVn%5B2%5D,+FVm%5B2%5D,+tmp)%3B%0A++++tmp+%3D+@mulAdd(f32,+FVn%5B3%5D,+FVm%5B3%5D,+tmp)%3B%0A++++return+tmp%3B%0A%7D%0A%0Aexport+fn+fpir_reduce(FVn:+@Vector(4,+f32),+FVm:+@Vector(4,+f32))+f32+%7B%0A++++return+@reduce(.Add,+FVn+*+FVm)%3B%0A%7D'),l:'5',n:'1',o:'Zig+source+%231',t:'0')),header:(),k:50,l:'4',m:100,n:'0',o:'',s:0,t:'0'),(g:!((g:!((h:compiler,i:(compiler:ztrunk,filters:(b:'0',binary:'1',binaryObject:'1',commentOnly:'0',debugCalls:'1',demangle:'0',directives:'0',execute:'1',intel:'0',libraryCode:'0',trim:'1',verboseDemangling:'0'),flagsViewOpen:'1',fontScale:14,fontUsePx:'0',j:1,lang:zig,libs:!(),options:'-OReleaseSafe',overrides:!(),selection:(endColumn:1,endLineNumber:1,positionColumn:1,positionLineNumber:1,selectionStartColumn:1,selectionStartLineNumber:1,startColumn:1,startLineNumber:1),source:1),l:'5',n:'0',o:'+zig+trunk+(Editor+%231)',t:'0')),header:(),k:50,l:'4',m:87.26851851851852,n:'0',o:'',s:0,t:'0'),(g:!((h:output,i:(compilerName:'zig+trunk',editorid:1,fontScale:14,fontUsePx:'0',j:1,wrap:'1'),l:'5',n:'0',o:'Output+of+zig+trunk+(Compiler+%231)',t:'0')),l:'4',m:12.731481481481477,n:'0',o:'',s:0,t:'0')),k:50,l:'3',n:'0',o:'',t:'0')),l:'2',n:'0',o:'',t:'0')),version:4
    const fpir: enum { FMA, Reduce } = .FMA;
};

pub fn unknown(cpu: *SH4, opcode: Instr) !void {
    std.debug.print("Unknown opcode: 0x{X:0>4} 0b{b:0>16}\n", .{ opcode.value, opcode.value });
    std.debug.print("  CPU State: PC={X:0>8}\n", .{cpu.pc});
    @panic("Unknown opcode");
}

pub fn nop(_: *SH4, _: Instr) !void {}

pub fn unimplemented(_: *SH4, opcode: Instr) !void {
    std.debug.panic("Unimplemented opcode: {s}\n", .{sh4_instructions.Opcodes[sh4_instructions.JumpTable[@as(u16, @bitCast(opcode))]].name});
}

pub fn mov_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.m).*;
}

pub fn mov_imm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nd8.n).* = @bitCast(sign_extension_u8(opcode.nd8.d));
}

test "mov #imm,Rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0 } });
    try std.testing.expect(cpu.R(0).* == 0);
    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0xFF } });
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFF);
    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 1, .d = 1 } });
    try std.testing.expect(cpu.R(1).* == 1);
    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 2, .d = 2 } });
    try std.testing.expect(cpu.R(2).* == 2);
    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 3, .d = 3 } });
    try std.testing.expect(cpu.R(3).* == 3);

    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 15, .d = 0 } }); // mov #0,R15
    try std.testing.expect(cpu.R(15).* == 0);
    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 15, .d = 1 } }); // mov #1,R15
    try std.testing.expect(cpu.R(15).* == 1);
}

// Stores the effective address of the source operand into general register R0.
// The 8-bit displacement is zero-extended and quadrupled. Consequently, the relative interval from the operand is PC + 1020 bytes.
// The PC is the address four bytes after this instruction, but the lowest two bits of the PC are fixed at 00.
// TODO: If this instruction is executed in a delay slot, a slot illegal instruction exception will be generated.
pub fn mova_atDispPC_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* = (cpu.pc & 0xFFFFFFFC) + 4 + (zero_extend(opcode.nd8.d) << 2);
}

// Stores immediate data, sign-extended to longword, in general register Rn.
// The data is stored from memory address (PC + 4 + displacement * 2).
// The 8-bit displacement is multiplied by two after zero-extension, and so the relative distance from the table is in the range up to PC + 4 + 510 bytes.
// The PC value is the address of this instruction.
pub fn movw_atDispPC_Rn(cpu: *SH4, opcode: Instr) !void {
    const n = opcode.nd8.n;
    const d = zero_extend(opcode.nd8.d) << 1;
    cpu.R(n).* = @bitCast(sign_extension_u16(try cpu.read(u16, cpu.pc + 4 + d)));
}

// The 8-bit displacement is multiplied by four after zero-extension, and so the relative distance from the operand is in the range up to PC + 4 + 1020 bytes.
// The PC value is the address of this instruction. A value with the lower 2 bits adjusted to 00 is used in address calculation.
pub fn movl_atDispPC_Rn(cpu: *SH4, opcode: Instr) !void {
    const n = opcode.nd8.n;
    const d = zero_extend(opcode.nd8.d) << 2;
    cpu.R(n).* = try cpu.read(u32, (cpu.pc & 0xFFFFFFFC) + 4 + d);
}

pub fn movb_atRm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(try cpu.read(u8, cpu.R(opcode.nmd.m).*)));
}

test "mov Rm,Rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0 } }); // mov #0,R0
    try std.testing.expect(cpu.R(0).* == 0);
    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 1, .d = 1 } }); // mov #1,R1
    try std.testing.expect(cpu.R(1).* == 1);
    mov_Rm_Rn(&cpu, .{ .nmd = .{ .n = 0, .m = 1 } }); // mov R1,R0
    try std.testing.expect(cpu.R(0).* == 1);
}

pub fn movw_atRm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(try cpu.read(u16, cpu.R(opcode.nmd.m).*)));
}

pub fn movl_atRm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = try cpu.read(u32, cpu.R(opcode.nmd.m).*);
}

pub fn movb_Rm_atRn(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u8, cpu.R(opcode.nmd.n).*, @truncate(cpu.R(opcode.nmd.m).*));
}

pub fn movw_Rm_atRn(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u16, cpu.R(opcode.nmd.n).*, @truncate(cpu.R(opcode.nmd.m).*));
}

pub fn movl_Rm_atRn(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u32, cpu.R(opcode.nmd.n).*, cpu.R(opcode.nmd.m).*);
}

pub fn movb_atRmInc_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(try cpu.read(u8, cpu.R(opcode.nmd.m).*)));
    if (opcode.nmd.n != opcode.nmd.m) {
        cpu.R(opcode.nmd.m).* += 1;
    }
}

// The loaded data is sign-extended to 32 bit before being stored in the destination register.
pub fn movw_atRmInc_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(try cpu.read(u16, cpu.R(opcode.nmd.m).*)));
    if (opcode.nmd.n != opcode.nmd.m) {
        cpu.R(opcode.nmd.m).* += 2;
    }
}

pub fn movl_atRmInc_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = try cpu.read(u32, cpu.R(opcode.nmd.m).*);
    if (opcode.nmd.n != opcode.nmd.m) {
        cpu.R(opcode.nmd.m).* += 4;
    }
}

inline fn mov_Rm_atDecRn(comptime T: type, cpu: *SH4, opcode: Instr) !void {
    const val: T = @truncate(cpu.R(opcode.nmd.m).*);
    cpu.R(opcode.nmd.n).* -%= @sizeOf(T);
    try cpu.write(T, cpu.R(opcode.nmd.n).*, val);
}

pub fn movb_Rm_atDecRn(cpu: *SH4, opcode: Instr) !void {
    try mov_Rm_atDecRn(u8, cpu, opcode);
}

pub fn movw_Rm_atDecRn(cpu: *SH4, opcode: Instr) !void {
    try mov_Rm_atDecRn(u16, cpu, opcode);
}

pub fn movl_Rm_atDecRn(cpu: *SH4, opcode: Instr) !void {
    try mov_Rm_atDecRn(u32, cpu, opcode);
}

pub fn movb_atDispRm_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* = @bitCast(sign_extension_u8(try cpu.read(u8, cpu.R(opcode.nmd.m).* + opcode.nmd.d)));
}

// The 4-bit displacement is multiplied by two after zero-extension, enabling a range up to +30 bytes to be specified.
// The loaded data is sign-extended to 32 bit before being stored in the destination register.
pub fn movw_atDispRm_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* = @bitCast(sign_extension_u16(try cpu.read(u16, cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 1))));
}

pub fn movl_at_dispRm_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* = try cpu.read(u32, cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 2));
}

// Transfers the source operand to the destination. The 4-bit displacement is multiplied by four after zero-extension, enabling a range up to +60 bytes to be specified.
pub fn movl_atDispRm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = try cpu.read(u32, cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 2));
}

// The 4-bit displacement is only zero-extended, so a range up to +15 bytes can be specified.
pub fn movb_R0_atDispRm(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u8, cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d)), @truncate(cpu.R(0).*));
}
// The 4-bit displacement is multiplied by two after zero-extension, enabling a range up to +30 bytes to be specified.
pub fn movw_R0_atDispRm(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u16, cpu.R(opcode.nmd.m).* + (zero_extend(opcode.nmd.d) << 1), @truncate(cpu.R(0).*));
}

// Transfers the source operand to the destination.
// The 4-bit displacement is multiplied by four after zero-extension, enabling a range up to +60 bytes to be specified.
pub fn movl_Rm_atDispRn(cpu: *SH4, opcode: Instr) !void {
    const d = zero_extend(opcode.nmd.d) << 2;
    try cpu.write(u32, cpu.R(opcode.nmd.n).* +% d, cpu.R(opcode.nmd.m).*);
}

pub fn movb_atR0Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(try cpu.read(u8, cpu.R(opcode.nmd.m).* +% cpu.R(0).*)));
}

pub fn movw_atR0Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(try cpu.read(u16, cpu.R(opcode.nmd.m).* +% cpu.R(0).*)));
}

pub fn movl_atR0Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = try cpu.read(u32, cpu.R(opcode.nmd.m).* +% cpu.R(0).*);
}

pub fn movb_Rm_atR0Rn(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u8, cpu.R(opcode.nmd.n).* +% cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
}
pub fn movw_Rm_atR0Rn(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u16, cpu.R(opcode.nmd.n).* +% cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
}
pub fn movl_Rm_atR0Rn(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u32, cpu.R(opcode.nmd.n).* +% cpu.R(0).*, @truncate(cpu.R(opcode.nmd.m).*));
}

pub fn movb_atDispGBR_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* = @bitCast(sign_extension_u8(try cpu.read(u8, cpu.gbr + zero_extend(opcode.nd8.d))));
}
pub fn movw_atDispGBR_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* = @bitCast(sign_extension_u16(try cpu.read(u16, cpu.gbr + (zero_extend(opcode.nd8.d) << 1))));
}
pub fn movl_atDispGBR_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* = try cpu.read(u32, cpu.gbr + (zero_extend(opcode.nd8.d) << 2));
}

pub fn movb_R0_atDispGBR(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u8, cpu.gbr + opcode.nd8.d, @truncate(cpu.R(0).*));
}
pub fn movw_R0_atDispGBR(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u16, cpu.gbr + (zero_extend(opcode.nd8.d) << 1), @truncate(cpu.R(0).*));
}
pub fn movl_R0_atDispGBR(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u32, cpu.gbr + (zero_extend(opcode.nd8.d) << 2), cpu.R(0).*);
}

pub fn movt_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = if (cpu.sr.t) 1 else 0;
}

// Swaps the upper and lower parts of the contents of general register Rm and stores the result in Rn.
// The 8 bits from bit 15 to bit 8 of Rm are swapped with the 8 bits from bit 7 to bit 0.
// The upper 16 bits of Rm are transferred directly to the upper 16 bits of Rn.
pub fn swapb(cpu: *SH4, opcode: Instr) !void {
    const val = cpu.R(opcode.nmd.m).*;
    const l: u16 = @truncate(val);
    cpu.R(opcode.nmd.n).* = (val & 0xFFFF0000) | @byteSwap(l);
}
pub fn swapw(cpu: *SH4, opcode: Instr) !void {
    const val = cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = val << 16 | val >> 16;
}

test "swapb" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0xAABBCCDD;
    swapb(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0xAABBDDCC);
}

test "swapw" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0xAABBCCDD;
    swapw(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0xCCDDAABB);
}

pub fn xtrct_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = ((cpu.R(opcode.nmd.m).* << 16) & 0xFFFF0000) | ((cpu.R(opcode.nmd.n).* >> 16) & 0x0000FFFF);
}

pub fn add_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* +%= cpu.R(opcode.nmd.m).*;
}

pub fn add_imm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nd8.n).* = @bitCast(@as(i32, @bitCast(cpu.R(opcode.nd8.n).*)) +% sign_extension_u8(opcode.nd8.d));
}

test "add imm rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0;
    add_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 1 } });
    try std.testing.expect(cpu.R(0).* == 1);

    cpu.R(0).* = 0;
    add_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0xFF } });
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFF);

    cpu.R(0).* = 0xFFFFFFFF;
    add_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0xFF } });
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFE);

    cpu.R(0).* = 0xFFFFFFFF;
    add_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 1 } });
    try std.testing.expect(cpu.R(0).* == 0);

    cpu.R(0).* = 0x12345678;
    add_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0 } });
    try std.testing.expect(cpu.R(0).* == 0x12345678);

    cpu.R(0).* = 0x12345678;
    add_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 1 } });
    try std.testing.expect(cpu.R(0).* == 0x12345679);

    cpu.R(0).* = 0x12345678;
    add_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 0xFF } });
    try std.testing.expect(cpu.R(0).* == 0x12345677);
}

pub fn addc_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    if (comptime false) {
        // Pure zig version, keeping it around just in case.
        const sum, const overflow = @addWithOverflow(cpu.R(opcode.nmd.n).*, cpu.R(opcode.nmd.m).*);
        cpu.R(opcode.nmd.n).*, const carry_overflow = @addWithOverflow(sum, (if (cpu.sr.t) @as(u32, @intCast(1)) else 0));
        cpu.sr.t = overflow == 1 or carry_overflow == 1;
    } else {
        const rn = cpu.R(opcode.nmd.n);
        const rm = cpu.R(opcode.nmd.m);

        rn.* = asm volatile (
            \\ bt $0, %edx
            \\ adc %ecx, %eax
            : [ret] "={eax}" (-> u32),
            : [_] "{edx}" (cpu.sr.t),
              [_] "{eax}" (rn.*),
              [_] "{ecx}" (rm.*),
            : "edx", "eax"
        );
        cpu.sr.t = asm volatile (
            \\ setb %dl
            : [ret] "={dl}" (-> bool),
            :
            : "dl"
        );
    }
}

// Adds together the contents of general registers Rn and Rm and stores the result in Rn. If overflow occurs, the T bit is set.
pub fn addv_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const rn: i32 = @bitCast(cpu.R(opcode.nmd.n).*);
    const rm: i32 = @bitCast(cpu.R(opcode.nmd.m).*);
    const r = @addWithOverflow(rn, rm);
    cpu.R(opcode.nmd.n).* = @bitCast(r[0]);
    cpu.sr.t = @bitCast(r[1]);
}

// Compares general register R0 and the sign-extended 8-bit immediate data and sets the T bit if the values are equal.
// If they are not equal the T bit is cleared. The contents of R0 are not changed.
pub fn cmpeq_imm_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (cpu.R(0).* == @as(u32, @bitCast(sign_extension_u8(opcode.nd8.d))));
}
pub fn cmpeq_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (cpu.R(opcode.nmd.n).* == cpu.R(opcode.nmd.m).*);
}
// The values for the comparison are interpreted as unsigned integer values
pub fn cmphs_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (cpu.R(opcode.nmd.n).* >= cpu.R(opcode.nmd.m).*);
}
pub fn cmphi_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (cpu.R(opcode.nmd.n).* > cpu.R(opcode.nmd.m).*);
}
// The values for the comparison are interpreted as signed integer values.
pub fn cmpge_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (as_i32(cpu.R(opcode.nmd.n).*) >= as_i32(cpu.R(opcode.nmd.m).*));
}
pub fn cmpgt_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (as_i32(cpu.R(opcode.nmd.n).*) > as_i32(cpu.R(opcode.nmd.m).*));
}

// The value in Rn for the comparison is interpreted as signed integer.
pub fn cmppl_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (as_i32(cpu.R(opcode.nmd.n).*) > 0);
}
pub fn cmppz_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (as_i32(cpu.R(opcode.nmd.n).*) >= 0);
}
// Compares general registers Rn and Rm, and sets the T bit if any of the 4 bytes in Rn are equal to the corresponding byte in Rm.
pub fn cmpstr_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const l = cpu.R(opcode.nmd.n).*;
    const r = cpu.R(opcode.nmd.m).*;

    // cpu.sr.t = (l & 0xFF000000 == r & 0xFF000000) or (l & 0x00FF0000 == r & 0x00FF0000) or (l & 0x0000FF00 == r & 0x0000FF00) or (l & 0x000000FF == r & 0x000000FF);

    const vl: @Vector(4, u8) = std.mem.asBytes(&l).*;
    const vr: @Vector(4, u8) = std.mem.asBytes(&r).*;
    cpu.sr.t = @reduce(.Or, vl == vr);
}

// Performs initial settings for signed division.
// This instruction is followed by a DIV1 instruction that executes 1-digit division, for example, and repeated division steps are executed to find the quotient.
pub fn div0s_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.q = (0x80000000 & cpu.R(opcode.nmd.n).*) != 0;
    cpu.sr.m = (0x80000000 & cpu.R(opcode.nmd.m).*) != 0;
    cpu.sr.t = cpu.sr.q != cpu.sr.m;
}

// Performs initial settings for unsigned division.
// This instruction is followed by a DIV1 instruction that executes 1-digit division, for example,
// and repeated division steps are executed to find the quotient.
pub fn div0u(cpu: *SH4, _: Instr) !void {
    cpu.sr.m = false;
    cpu.sr.q = false;
    cpu.sr.t = false;
}

// Performs 1-digit division (1-step division) of the 32-bit contents of general register Rn (dividend) by the contents of Rm (divisor)
pub fn div1(cpu: *SH4, opcode: Instr) !void {
    const pRn = cpu.R(opcode.nmd.n);

    const prev_q = cpu.sr.q;
    var Q = (0x80000000 & pRn.*) != 0;

    const Rm = cpu.R(opcode.nmd.m).*;

    pRn.* <<= 1;
    pRn.* |= if (cpu.sr.t) 1 else 0;

    const prev_Rn = pRn.*;

    if (!prev_q) {
        if (!cpu.sr.m) {
            pRn.* -%= Rm;
            Q = Q != (pRn.* > prev_Rn); // XOR
        } else {
            pRn.* +%= Rm;
            Q = !Q != (pRn.* < prev_Rn);
        }
    } else {
        if (!cpu.sr.m) {
            pRn.* +%= Rm;
            Q = Q != (pRn.* < prev_Rn);
        } else {
            pRn.* -%= Rm;
            Q = !Q != (pRn.* > prev_Rn);
        }
    }
    cpu.sr.q = Q;
    cpu.sr.t = (Q == cpu.sr.m);
}

test "div1" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0b00111110111110001001111010110000;
    cpu.R(1).* = 0b00000011100100011000110101000110;

    div0u(&cpu, .{ .value = 0b0000000000011001 });
    try std.testing.expect(!cpu.sr.m);
    try std.testing.expect(!cpu.sr.q);
    try std.testing.expect(!cpu.sr.t);
    div1(&cpu, .{ .value = 0b0011_0000_0001_0100 });
    try std.testing.expect(cpu.R(0).* == 0b01111101111100010011110101100000 - 0b00000011100100011000110101000110);
    try std.testing.expect(cpu.sr.t);
}

test "div1 r1 (32 bits) / r0 (16 bits) = r1 (16 bits)" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    const dividend = 0b00111110111110001001111010110000;
    const divisor = 0b00000000000000000100110001100010;

    cpu.R(0).* = divisor;
    cpu.R(1).* = dividend;

    shll16(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    div0u(&cpu, .{ .value = 0b0000000000011001 });

    for (0..16) |_| {
        div1(&cpu, .{ .nmd = .{ ._ = 0b0011, .n = 1, .m = 0, .d = 0b0100 } });
    }
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = 0b0100, .n = 1, .m = 0b0010, .d = 0b0100 } });
    extuw_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 1, .d = undefined } });

    try std.testing.expect(cpu.R(1).* == dividend / divisor);
}

test "div1 r3:r1 (64 bits) / r4 (32 bits) = r1 (32 bits)  (unsigned)" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    // Example from KallistiOS

    const dividend: u64 = 0x1743D174;

    const dividend_low: u32 = @truncate(dividend);
    const dividend_high: u32 = @truncate(dividend >> 32);
    const divisor: u32 = 0x0000000A;

    cpu.R(1).* = dividend_low;
    cpu.R(3).* = dividend_high;
    cpu.R(4).* = divisor;

    div0u(&cpu, .{ .value = 0b0000000000011001 });

    for (0..32) |_| {
        rotcl_Rn(&cpu, .{ .nmd = .{ ._ = 0b0100, .n = 1, .m = 0b0010, .d = 0b0100 } }); // rotcl R1
        div1(&cpu, .{ .nmd = .{ ._ = 0b0011, .n = 3, .m = 4, .d = 0b0100 } }); // div1 R4,R3
    }
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = 0b0100, .n = 1, .m = 0b0010, .d = 0b0100 } }); // rotcl R1

    try std.testing.expect(cpu.R(1).* == @as(u32, @truncate(dividend / divisor)));
}

test "r2 (32 bits) / r0 (32 bits) = r2 (32 bits)  (signed)" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    // Example from http://shared-ptr.com/sh_insns.html

    for ([_][2]i32{
        .{ 21685454, 354 },
        .{ 21685454, -354 },
        .{ -21685454, -354 },
        .{ -21685454, 354 },
        .{ 4544721, 576 },
        .{ 4544721, -576 },
        .{ -4544721, -576 },
        .{ -4544721, 576 },
        .{ 3574, 258765 },
        .{ 3574, -258765 },
        .{ -3574, -258765 },
        .{ -3574, 258765 },
    }) |pair| {
        const dividend: i32 = pair[0];
        const divisor: i32 = pair[1];

        cpu.R(2).* = @bitCast(dividend);
        cpu.R(0).* = @bitCast(divisor);

        // mov     r2,r3
        mov_Rm_Rn(&cpu, .{ .nmd = .{ .n = 3, .m = 2 } });
        // rotcl   r3
        rotcl_Rn(&cpu, .{ .nmd = .{ .n = 3, .m = undefined } });
        // subc    r1,r1     ! Dividend sign-extended to 64 bits (r1:r2)
        subc_Rm_Rn(&cpu, .{ .nmd = .{ .n = 1, .m = 1 } });
        // mov     #0,r3
        mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 3, .d = 0 } });
        // subc    r3,r2     ! If dividend is negative, subtract 1 to convert to one's complement notation
        subc_Rm_Rn(&cpu, .{ .nmd = .{ .n = 2, .m = 3 } });
        // div0s   r0,r1     ! Flag initialization
        div0s_Rm_Rn(&cpu, .{ .nmd = .{ .n = 1, .m = 0 } });

        // .rept 32
        for (0..32) |_| {
            // rotcl   r2        ! Repeat 32 times
            rotcl_Rn(&cpu, .{ .nmd = .{ .n = 2 } });
            // div1    r0,r1
            div1(&cpu, .{ .nmd = .{ .n = 1, .m = 0 } });
            // .endr
        }

        // rotcl   r2        ! r2 = quotient (one's complement notation)
        rotcl_Rn(&cpu, .{ .nmd = .{ .n = 2 } });
        // addc    r3,r2     ! If MSB of quotient is 1, add 1 to convert to two's complement notation
        addc_Rm_Rn(&cpu, .{ .nmd = .{ .n = 2, .m = 3 } });
        //                   ! r2 = quotient (two's complement notation)

        try std.testing.expect(@as(i32, @bitCast(cpu.R(2).*)) == @divTrunc(dividend, divisor));
    }
}

pub fn dmulsl_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const rn: i32 = @bitCast(cpu.R(opcode.nmd.n).*);
    const rm: i32 = @bitCast(cpu.R(opcode.nmd.m).*);

    const r: u64 = @bitCast(@as(i64, rn) * @as(i64, rm));
    cpu.mach = @truncate(r >> 32);
    cpu.macl = @truncate(r);
}

pub fn dmulul_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const r = @as(u64, cpu.R(opcode.nmd.n).*) * @as(u64, cpu.R(opcode.nmd.m).*);
    cpu.mach = @truncate(r >> 32);
    cpu.macl = @truncate(r);
}

// Decrements the contents of general register Rn by 1 and compares the result with zero.
// If the result is zero, the T bit is set to 1. If the result is nonzero, the T bit is cleared to 0.
pub fn dt_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* -%= 1;
    cpu.sr.t = (cpu.R(opcode.nmd.n).* == 0);
}

pub fn extsb_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u8(@truncate(cpu.R(opcode.nmd.m).*)));
}
pub fn extsw_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @bitCast(sign_extension_u16(@truncate(cpu.R(opcode.nmd.m).*)));
}
// Zero-extends the contents of general register Rm and stores the result in Rn. 0 is transferred to Rn bits 8 to 31.
pub fn extub_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.m).* & 0xFF;
}
pub fn extuw_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = cpu.R(opcode.nmd.m).* & 0xFFFF;
}

pub fn macl_atRmInc_atRnInc(cpu: *SH4, opcode: Instr) !void {
    const rn: i32 = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.n).*));
    const rm: i32 = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.m).*));

    var m: i64 = @as(i64, rn) * @as(i64, rm);
    m += @as(i64, @bitCast(@as(u64, cpu.mach) << 32 | @as(u64, cpu.macl)));

    const bits: u64 = @bitCast(m);
    if (cpu.sr.s) {
        // FIXME: Not sure about this
        cpu.mach = (cpu.mach & 0xFFFF0000) & (@as(u32, @truncate(bits >> 32)) & 0x0000FFFF);
        @panic("Unimplemented");
    } else {
        cpu.mach = @truncate(bits >> 32);
    }
    cpu.macl = @truncate(bits);

    cpu.R(opcode.nmd.n).* += 4;
    cpu.R(opcode.nmd.m).* += 4;
}

pub fn macw_atRmInc_atRnInc(cpu: *SH4, opcode: Instr) !void {
    const rn: i16 = @bitCast(try cpu.read(u16, cpu.R(opcode.nmd.n).*));
    const rm: i16 = @bitCast(try cpu.read(u16, cpu.R(opcode.nmd.m).*));

    var m: i64 = @as(i32, rn) * @as(i32, rm);
    m += @as(i64, @bitCast(@as(u64, cpu.mach) << 32 | @as(u64, cpu.macl)));

    const bits: u64 = @bitCast(m);
    if (cpu.sr.s) {
        @panic("Unimplemented");
    } else {
        cpu.mach = @truncate(bits >> 32);
    }
    cpu.macl = @truncate(bits);

    cpu.R(opcode.nmd.n).* += 2;
    cpu.R(opcode.nmd.m).* += 2;
}

pub fn mull_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.macl = cpu.R(opcode.nmd.n).* *% cpu.R(opcode.nmd.m).*;
}
// Performs 16-bit multiplication of the contents of general registers Rn and Rm, and stores the 32-bit result in the MACL register.
// The multiplication is performed as a signed arithmetic operation.
pub fn mulsw_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const n: i16 = @bitCast(@as(u16, @truncate(cpu.R(opcode.nmd.n).*)));
    const m: i16 = @bitCast(@as(u16, @truncate(cpu.R(opcode.nmd.m).*)));
    cpu.macl = @bitCast(@as(i32, @intCast(n)) * @as(i32, @intCast(m)));
}

// Performs 16-bit multiplication of the contents of general registers Rn and Rm, and stores the 32-bit result in the MACL register.
// The multiplication is performed as an unsigned arithmetic operation. The contents of MACH are not changed
pub fn muluw_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.macl = @as(u32, @intCast(@as(u16, @truncate(cpu.R(opcode.nmd.n).*)))) * @as(u32, @intCast(@as(u16, @truncate(cpu.R(opcode.nmd.m).*))));
}

pub fn neg_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = 0 -% cpu.R(opcode.nmd.m).*;
}

test "neg Rm,Rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 1;
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == -1);

    cpu.R(0).* = @bitCast(@as(i32, -1));
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == 1);

    cpu.R(0).* = 1337;
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == -1337);

    cpu.R(0).* = @bitCast(@as(i32, -1337));
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == 1337);

    cpu.R(0).* = 0x180;
    neg_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 0, .d = undefined } });
    try std.testing.expect(as_i32(cpu.R(1).*) == -0x180);
}

pub fn negc_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const tmp = 0 -% cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = tmp -% (if (cpu.sr.t) @as(u32, 1) else 0);
    cpu.sr.t = (0 < tmp);
    if (tmp < cpu.R(opcode.nmd.n).*)
        cpu.sr.t = true;
}

test "negc Rm,Rn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    // Examples from the manual

    cpu.R(1).* = 1;
    cpu.sr.t = false;
    negc_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 1, .m = 1, .d = undefined } });
    try std.testing.expect(cpu.R(1).* == 0xFFFFFFFF);
    try std.testing.expect(cpu.sr.t);

    cpu.R(0).* = 0;
    cpu.sr.t = true;
    negc_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = 0, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0xFFFFFFFF);
    try std.testing.expect(cpu.sr.t);

    cpu.R(1).* = @bitCast(@as(i32, -1));
    cpu.sr.t = true;
    negc_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = 1, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0);
    try std.testing.expect(cpu.sr.t);

    cpu.R(1).* = @bitCast(@as(i32, -1));
    cpu.sr.t = false;
    negc_Rm_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = 1, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 1);
    try std.testing.expect(cpu.sr.t);
}

pub fn sub_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* -%= cpu.R(opcode.nmd.m).*;
}
pub fn subc_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const prev_Rn = cpu.R(opcode.nmd.n).*;
    const diff = prev_Rn -% cpu.R(opcode.nmd.m).*;
    cpu.R(opcode.nmd.n).* = diff -% (if (cpu.sr.t) @as(u32, 1) else 0);
    cpu.sr.t = (prev_Rn < diff);
    if (diff < cpu.R(opcode.nmd.n).*)
        cpu.sr.t = true;
}

pub fn and_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* &= cpu.R(opcode.nmd.m).*;
}
pub fn and_imm_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* &= zero_extend(opcode.nd8.d);
}
pub fn andb_imm_atR0GBR(cpu: *SH4, opcode: Instr) !void {
    const temp = try cpu.read(u8, cpu.gbr +% cpu.R(0).*) & opcode.nd8.d;
    try cpu.write(u8, cpu.gbr +% cpu.R(0).*, temp);
}

pub fn not_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = ~cpu.R(opcode.nmd.m).*;
}
pub fn or_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* |= cpu.R(opcode.nmd.m).*;
}
pub fn or_imm_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* |= zero_extend(opcode.nd8.d);
}
pub fn orb_imm_atR0GBR(cpu: *SH4, opcode: Instr) !void {
    const val: u8 = try cpu.read(u8, cpu.gbr +% cpu.R(0).*) | opcode.nd8.d;
    try cpu.write(u8, cpu.gbr +% cpu.R(0).*, val);
}

pub fn tasb_atRn(cpu: *SH4, opcode: Instr) !void {
    // Reads byte data from the address specified by general register Rn, and sets the T bit to 1 if the data is 0, or clears the T bit to 0 if the data is not 0.
    // Then, data bit 7 is set to 1, and the data is written to the address specified by Rn.
    const tmp = try cpu.read(u8, cpu.R(opcode.nmd.n).*);
    cpu.sr.t = (tmp == 0);
    try cpu.write(u8, cpu.R(opcode.nmd.n).*, tmp | 0x80);
}
pub fn tst_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (cpu.R(opcode.nmd.n).* & cpu.R(opcode.nmd.m).*) == 0;
}
pub fn tst_imm_R0(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (cpu.R(0).* & zero_extend(opcode.nd8.d)) == 0;
}
pub fn tstb_imm_atR0GBR(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = (try cpu.read(u8, cpu.gbr +% cpu.R(0).*) & opcode.nd8.d) == 0;
}

pub fn xorRmRn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* ^= cpu.R(opcode.nmd.m).*;
}
pub fn xorImmR0(cpu: *SH4, opcode: Instr) !void {
    cpu.R(0).* ^= zero_extend(opcode.nd8.d);
}
pub fn xorb_imm_atR0GBR(cpu: *SH4, opcode: Instr) !void {
    try cpu.write(u8, cpu.gbr +% cpu.R(0).*, try cpu.read(u8, cpu.gbr +% cpu.R(0).*) ^ opcode.nd8.d);
}

pub fn rotcl_Rn(cpu: *SH4, opcode: Instr) !void {
    const tmp = ((cpu.R(opcode.nmd.n).* & 0x80000000) != 0);
    cpu.R(opcode.nmd.n).* <<= 1;
    if (cpu.sr.t) {
        cpu.R(opcode.nmd.n).* |= 0x00000001;
    } else {
        cpu.R(opcode.nmd.n).* &= 0xFFFFFFFE;
    }
    cpu.sr.t = tmp;
}
test "rotcl" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0;
    cpu.sr.t = false;
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0;
    cpu.sr.t = true;
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 1);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b01;
    cpu.sr.t = false;
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b10);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b01;
    cpu.sr.t = true;
    rotcl_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b11);
    try std.testing.expect(!cpu.sr.t);
}

pub fn rotcr_Rn(cpu: *SH4, opcode: Instr) !void {
    const tmp = (cpu.R(opcode.nmd.n).* & 0x00000001) == 1;
    cpu.R(opcode.nmd.n).* >>= 1;
    if (cpu.sr.t) {
        cpu.R(opcode.nmd.n).* |= 0x80000000;
    } else {
        cpu.R(opcode.nmd.n).* &= 0x7FFFFFFF;
    }
    cpu.sr.t = tmp;
}

test "rotcr" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();

    cpu.R(0).* = 0;
    cpu.sr.t = false;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0;
    cpu.sr.t = true;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b1000_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b01;
    cpu.sr.t = false;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0);
    try std.testing.expect(cpu.sr.t);

    cpu.R(0).* = 0b01;
    cpu.sr.t = true;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b1000_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(cpu.sr.t);

    cpu.R(0).* = 0b1000_0000_0000_0000_0000_0000_0000_0000;
    cpu.sr.t = false;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b0100_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b1000_0000_0000_0000_0000_0000_0000_0000;
    cpu.sr.t = true;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b1100_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(!cpu.sr.t);

    cpu.R(0).* = 0b1000_0000_0000_0000_0000_0000_0000_0001;
    cpu.sr.t = false;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b0100_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(cpu.sr.t);

    cpu.R(0).* = 0b1000_0000_0000_0000_0000_0000_0000_0001;
    cpu.sr.t = true;
    rotcr_Rn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    try std.testing.expect(cpu.R(0).* == 0b1100_0000_0000_0000_0000_0000_0000_0000);
    try std.testing.expect(cpu.sr.t);
}

pub fn rotl_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 0x80000000) != 0);
    cpu.R(opcode.nmd.n).* = std.math.rotl(u32, cpu.R(opcode.nmd.n).*, 1);
}
pub fn rotr_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 1) == 1);
    cpu.R(opcode.nmd.n).* = std.math.rotr(u32, cpu.R(opcode.nmd.n).*, 1);
}
// Arithmetically shifts the contents of general register Rn. General register Rm specifies the shift direction and the number of bits to be shifted.
pub fn shad_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const shift = cpu.R(opcode.nmd.m).*;
    const sign = shift & 0x80000000;

    if (sign == 0) {
        cpu.R(opcode.nmd.n).* <<= @intCast(shift & 0x1F);
    } else if (shift & 0x1F == 0) {
        if (cpu.R(opcode.nmd.n).* & 0x80000000 == 0) {
            cpu.R(opcode.nmd.n).* = 0;
        } else {
            cpu.R(opcode.nmd.n).* = 0xFFFFFFFF;
        }
    } else {
        if (cpu.R(opcode.nmd.n).* & 0x80000000 == 0) {
            cpu.R(opcode.nmd.n).* >>= @intCast(((~shift) & 0x1F) + 1);
        } else {
            cpu.R(opcode.nmd.n).* = ~((~cpu.R(opcode.nmd.n).*) >> @intCast(((~shift) & 0x1F) + 1));
        }
    }
}
// Arithmetically shifts the contents of general register Rn one bit to the left and stores the result in Rn. The bit shifted out of the operand is transferred to the T bit
pub fn shal_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 0x80000000) != 0);
    cpu.R(opcode.nmd.n).* <<= 1;
}
// Arithmetically shifts the contents of general register Rn one bit to the right and stores the result in Rn. The bit shifted out of the operand is transferred to the T bit.
pub fn shar_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 1) == 1);

    const tmp = ((cpu.R(opcode.nmd.n).* & 0x80000000) != 0);

    cpu.R(opcode.nmd.n).* >>= 1;

    if (tmp) {
        cpu.R(opcode.nmd.n).* |= 0x80000000;
    } else {
        cpu.R(opcode.nmd.n).* &= 0x7FFFFFFF;
    }
}

pub fn shld_Rm_Rn(cpu: *SH4, opcode: Instr) !void {
    const sign = cpu.R(opcode.nmd.m).* & 0x80000000;
    if (sign == 0) {
        cpu.R(opcode.nmd.n).* <<= @intCast(cpu.R(opcode.nmd.m).* & 0x1F);
    } else if ((cpu.R(opcode.nmd.m).* & 0x1F) == 0) {
        cpu.R(opcode.nmd.n).* = 0;
    } else {
        cpu.R(opcode.nmd.n).* >>= @intCast(((~cpu.R(opcode.nmd.m).*) & 0x1F) + 1);
    }
}
pub fn shll(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 0x80000000) != 0);
    cpu.R(opcode.nmd.n).* <<= 1;
}
pub fn shll2(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* <<= 2;
}
pub fn shll8(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* <<= 8;
}
pub fn shll16(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* <<= 16;
}
pub fn shlr(cpu: *SH4, opcode: Instr) !void {
    cpu.sr.t = ((cpu.R(opcode.nmd.n).* & 1) == 1);
    cpu.R(opcode.nmd.n).* >>= 1;
}
pub fn shlr2(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* >>= 2;
}
pub fn shlr8(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* >>= 8;
}
pub fn shlr16(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* >>= 16;
}

pub inline fn d8_disp(addr: u32, opcode: sh4.Instr) u32 {
    const displacement = bit_manip.sign_extension_u8(opcode.nd8.d);
    var pc: i32 = @intCast(addr & 0x1FFFFFFF);
    pc += 4 + (displacement * 2);
    return (addr & 0xE0000000) | @as(u32, @bitCast(pc & 0x1FFFFFFF));
}

inline fn d8_label(cpu: *SH4, opcode: Instr) void {
    cpu.pc = d8_disp(cpu.pc, opcode);
    // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    cpu.pc -= 2;
}

pub inline fn d12_disp(addr: u32, opcode: Instr) u32 {
    const displacement = sign_extension_u12(opcode.d12.d);
    var pc: i32 = @intCast(addr & 0x1FFFFFFF);
    pc += 4 + (displacement * 2);
    return (addr & 0xE0000000) | @as(u32, @bitCast(pc & 0x1FFFFFFF));
}

inline fn d12_label(cpu: *SH4, opcode: Instr) void {
    cpu.pc = d12_disp(cpu.pc, opcode);
    // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    cpu.pc -= 2;
}

inline fn execute_delay_slot(cpu: *SH4, addr: u32) void {
    // TODO: If the instruction at addr is a branch instruction, raise a Slot illegal instruction exception

    // Set PC to the instruction that triggered the delay slot in case an exception is raised inside the delay slot.
    const current_pc = cpu.pc;
    cpu.pc = addr - 2;
    cpu._execute(addr);
    // Restore PC, unless an exception occured and PC was overwritten.
    if (cpu.pc == addr - 2)
        cpu.pc = current_pc;
}

pub fn bf_label(cpu: *SH4, opcode: Instr) !void {
    if (!cpu.sr.t) {
        d8_label(cpu, opcode);
    }
}
pub fn bfs_label(cpu: *SH4, opcode: Instr) !void {
    const delay_slot = cpu.pc + 2;
    if (!cpu.sr.t) {
        d8_label(cpu, opcode);
    } else { // Don't execute the delay slot twice.
        cpu.pc += 4;
        cpu.pc -= 2; // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    }
    execute_delay_slot(cpu, delay_slot);
}
pub fn bt_label(cpu: *SH4, opcode: Instr) !void {
    if (cpu.sr.t) {
        d8_label(cpu, opcode);
    }
}
pub fn bts_label(cpu: *SH4, opcode: Instr) !void {
    const delay_slot = cpu.pc + 2;
    if (cpu.sr.t) {
        d8_label(cpu, opcode);
    } else { // Don't execute the delay slot twice.
        cpu.pc += 4;
        cpu.pc -= 2; // -2 to account for the generic, inavoidable pc advancement of the current implementation.
    }
    execute_delay_slot(cpu, delay_slot);
}
pub fn bra_label(cpu: *SH4, opcode: Instr) !void {
    const delay_slot = cpu.pc + 2;
    d12_label(cpu, opcode);
    execute_delay_slot(cpu, delay_slot);
}
pub fn braf_Rn(cpu: *SH4, opcode: Instr) !void {
    const delay_slot = cpu.pc + 2;
    cpu.pc +%= 4 + cpu.R(opcode.nmd.n).*;
    cpu.pc -= 2; // execute will allready add +2
    execute_delay_slot(cpu, delay_slot);
}
pub fn bsr_label(cpu: *SH4, opcode: Instr) !void {
    const delay_slot = cpu.pc + 2;
    cpu.pr = cpu.pc + 4;
    d12_label(cpu, opcode);
    execute_delay_slot(cpu, delay_slot);
}
pub fn bsrf_Rn(cpu: *SH4, opcode: Instr) !void {
    const delay_slot = cpu.pc + 2;
    cpu.pr = cpu.pc + 4;
    // Note: The Boot ROM seem to intentionally wrap around the address.
    cpu.pc +%= 4 + cpu.R(opcode.nmd.n).*;
    cpu.pc -= 2; // execute will allready add +2
    execute_delay_slot(cpu, delay_slot);
}
pub fn jmp_atRn(cpu: *SH4, opcode: Instr) !void {
    const delay_slot = cpu.pc + 2;
    cpu.pc = cpu.R(opcode.nmd.n).*;
    cpu.pc -= 2; // -2 to account for the standard +2
    execute_delay_slot(cpu, delay_slot);
}
// Makes a delayed branch to the subroutine procedure at the specified address after execution of the following instruction.
// Return address (PC + 4) is saved in PR, and a branch is made to the address indicated by general register Rm. JSR is used in combination with RTS for subroutine procedure calls.
// Note: As this is a delayed branch instruction, the instruction following this instruction is executed before the branch destination instruction.
// Interrupts are not accepted between this instruction and the following instruction.
// If the following instruction is a branch instruction, it is identified as a slot illegal instruction.
pub fn jsr_Rn(cpu: *SH4, opcode: Instr) !void {
    const delay_slot = cpu.pc + 2;
    cpu.pr = cpu.pc + 4;
    cpu.pc = cpu.R(opcode.nmd.n).*;
    cpu.pc -= 2; // -2 to account for the standard +2
    execute_delay_slot(cpu, delay_slot);
}
pub fn rts(cpu: *SH4, _: Instr) !void {
    const delay_slot = cpu.pc + 2;
    cpu.pc = cpu.pr;
    cpu.pc -= 2; // execute will add +2
    execute_delay_slot(cpu, delay_slot);
}

pub fn clrmac(cpu: *SH4, _: Instr) !void {
    cpu.mach = 0;
    cpu.macl = 0;
}
pub fn clrs(cpu: *SH4, _: Instr) !void {
    cpu.sr.s = false;
}
pub fn clrt(cpu: *SH4, _: Instr) !void {
    cpu.sr.t = false;
}

pub fn ldc_Rn_SR(cpu: *SH4, opcode: Instr) !void {
    cpu.set_sr(@bitCast(cpu.R(opcode.nmd.n).*));
}

test "ldc Rn,SR" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.pc = 0x0C000000;

    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 0, .d = 3 } }); // mov #3,R0
    try std.testing.expect(cpu.R(0).* == 0b000000011);
    ldc_Rn_SR(&cpu, .{ .nmd = .{ .n = 0 } }); // ldc R0,SR
    try std.testing.expect(@as(u32, @bitCast(cpu.sr)) == 0b00000011 & 0x700083F3);

    cpu.set_sr(.{});

    mov_imm_Rn(&cpu, .{ .nd8 = .{ .n = 15, .d = 3 } }); // mov #3,R15
    try std.testing.expect(cpu.R(15).* == 0b00000011);
    ldc_Rn_SR(&cpu, .{ .nmd = .{ .n = 15 } }); // ldc R15,SR
    try std.testing.expect(@as(u32, @bitCast(cpu.sr)) == 0b00000011 & 0x700083F3);
}

pub fn ldcl_atRnInc_SR(cpu: *SH4, opcode: Instr) !void {
    const addr = cpu.R(opcode.nmd.n).*;
    cpu.R(opcode.nmd.n).* += 4;
    cpu.set_sr(@bitCast(try cpu.read(u32, addr)));
}
pub fn ldc_Rn_GBR(cpu: *SH4, opcode: Instr) !void {
    cpu.gbr = cpu.R(opcode.nmd.n).*;
}
pub fn ldcl_atRnInc_GBR(cpu: *SH4, opcode: Instr) !void {
    cpu.gbr = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn ldc_Rn_VBR(cpu: *SH4, opcode: Instr) !void {
    cpu.vbr = cpu.R(opcode.nmd.n).*;
}
pub fn ldcl_atRnInc_VBR(cpu: *SH4, opcode: Instr) !void {
    cpu.vbr = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn ldc_Rn_SSR(cpu: *SH4, opcode: Instr) !void {
    cpu.ssr = cpu.R(opcode.nmd.n).*;
}
pub fn ldcl_atRnInc_SSR(cpu: *SH4, opcode: Instr) !void {
    cpu.ssr = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn ldc_Rn_SPC(cpu: *SH4, opcode: Instr) !void {
    cpu.spc = cpu.R(opcode.nmd.n).*;
}
pub fn ldcl_atRnInc_SPC(cpu: *SH4, opcode: Instr) !void {
    cpu.spc = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn ldc_Rn_DBR(cpu: *SH4, opcode: Instr) !void {
    cpu.dbr = cpu.R(opcode.nmd.n).*;
}
pub fn ldcl_atRnInc_DBR(cpu: *SH4, opcode: Instr) !void {
    cpu.dbr = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn ldc_Rn_Rm_BANK(cpu: *SH4, opcode: Instr) !void {
    cpu.r_bank[opcode.nmd.m & 0b0111] = cpu.R(opcode.nmd.n).*;
}
pub fn ldcl_atRnInc_Rm_BANK(cpu: *SH4, opcode: Instr) !void {
    cpu.r_bank[opcode.nmd.m & 0b0111] = try cpu.read(u32, cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn lds_Rn_MACH(cpu: *SH4, opcode: Instr) !void {
    cpu.mach = cpu.R(opcode.nmd.n).*;
}
pub fn ldsl_atRnInc_MACH(cpu: *SH4, opcode: Instr) !void {
    cpu.mach = try cpu.read(u32, cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn lds_Rn_MACL(cpu: *SH4, opcode: Instr) !void {
    cpu.macl = cpu.R(opcode.nmd.n).*;
}
pub fn ldsl_atRnInc_MACL(cpu: *SH4, opcode: Instr) !void {
    cpu.macl = try cpu.read(u32, cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn lds_Rn_PR(cpu: *SH4, opcode: Instr) !void {
    cpu.pr = cpu.R(opcode.nmd.n).*;
}
pub fn ldsl_atRnInc_PR(cpu: *SH4, opcode: Instr) !void {
    cpu.pr = try cpu.read(u32, cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}

pub fn ldtlb(cpu: *SH4, _: Instr) !void {
    const urc = cpu.read_p4_register(sh4.mmu.MMUCR, .MMUCR).urc;
    const pteh = cpu.read_p4_register(sh4.mmu.PTEH, .PTEH);
    const ptel = cpu.read_p4_register(sh4.mmu.PTEL, .PTEL);
    const ptea = cpu.read_p4_register(sh4.mmu.PTEA, .PTEA);

    cpu.utlb[urc] = .{
        .asid = pteh.asid,
        .vpn = pteh.vpn,

        .ppn = ptel.ppn,
        .v = ptel.v,
        .sz = ptel.sz(),
        .pr = ptel.pr,
        .c = ptel.c,
        .d = ptel.d,
        .sh = ptel.sh,
        .wt = ptel.wt,

        .sa = ptea.sa,
        .tc = ptea.tc,
    };

    sh4_log.info("ldtlb : utlb[{d}] = {any}", .{ urc, cpu.utlb[urc] });

    if (cpu._dc) |dc| dc.sh4_jit.invalidate(cpu.utlb[urc].first_address(), cpu.utlb[urc].first_address() + cpu.utlb[urc].size());
}

pub fn movcal_R0_atRn(cpu: *SH4, opcode: Instr) !void {
    // Stores the contents of general register R0 in the memory location indicated by effective address Rn.
    // If write-back is selected for the accessed memory, and a cache miss occurs, the cache block will be allocated but an
    // R0 data write will be performed to that cache block without performing a block read. Other cache block contents are undefined.

    const addr = cpu.R(opcode.nmd.n).*;
    const data = cpu.R(0).*;
    if (addr & (@as(u32, 1) << 25) != 0) {
        // DCA3 Hack
        std.debug.assert(builtin.is_test or (addr & 3) == 0);
        const index: u32 = (addr / 32) & 255;
        const offset: u32 = (addr & 31) / @sizeOf(u32);

        cpu._operand_cache_state.addr[index] = addr & ~@as(u32, 31);
        cpu._operand_cache_state.addr[index] &= 0x1FFFFFFF; // FIXME: Not sure about that.
        @memset(cpu.operand_cache_lines()[index][0..], 0);
        cpu.operand_cache_lines()[index][offset] = data;
        cpu._operand_cache_state.dirty[index] = true;

        sh4_log.debug("movcal_R0_atRn addr={X:0>8}, data={X:0>8}, index={X:0>8}, offset={X:0>8}", .{ addr, data, index, offset });
    } else {
        try cpu.write(u32, addr, data);
    }
}
pub fn lds_Rn_FPSCR(cpu: *SH4, opcode: Instr) !void {
    cpu.set_fpscr(cpu.R(opcode.nmd.n).*);
}
pub fn sts_FPSCR_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = @as(u32, @bitCast(cpu.fpscr)) & 0x003FFFFF;
}
pub fn ldsl_atRnInc_FPSCR(cpu: *SH4, opcode: Instr) !void {
    cpu.set_fpscr(try cpu.read(u32, cpu.R(opcode.nmd.n).*));
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn stsl_FPSCR_atDecRn(cpu: *SH4, opcode: Instr) !void {
    const tmp = cpu.R(opcode.nmd.n).* - 4; // Use a temporary in case of exception.
    try cpu.write(u32, tmp, @as(u32, @bitCast(cpu.fpscr)) & 0x003FFFFF);
    cpu.R(opcode.nmd.n).* = tmp;
}
pub fn lds_Rn_FPUL(cpu: *SH4, opcode: Instr) !void {
    cpu.fpul = cpu.R(opcode.nmd.n).*;
}
pub fn sts_FPUL_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = cpu.fpul;
}
pub fn ldsl_atRnInc_FPUL(cpu: *SH4, opcode: Instr) !void {
    cpu.fpul = try cpu.read(u32, cpu.R(opcode.nmd.n).*);
    cpu.R(opcode.nmd.n).* += 4;
}
pub fn stsl_FPUL_atDecRn(cpu: *SH4, opcode: Instr) !void {
    const tmp = cpu.R(opcode.nmd.n).* - 4; // Use a temporary in case of exception.
    try cpu.write(u32, tmp, cpu.fpul);
    cpu.R(opcode.nmd.n).* = tmp;
}

// Inverts the FR bit in floating-point register FPSCR.
pub fn frchg(cpu: *SH4, _: Instr) !void {
    std.debug.assert(cpu.fpscr.pr == 0);
    var new_fpscr = cpu.fpscr;
    new_fpscr.fr +%= 1;
    cpu.set_fpscr(@bitCast(new_fpscr));
}
pub fn fschg(cpu: *SH4, _: Instr) !void {
    std.debug.assert(cpu.fpscr.pr == 0);
    cpu.fpscr.sz +%= 1;
}

pub fn ocbi_atRn(_: *SH4, _: Instr) !void {
    const static = struct {
        var once = true;
    };
    if (static.once) {
        static.once = false;
        sh4_log.warn("Note: ocbi @Rn not implemented", .{});
    }
}

pub fn ocbp_atRn(_: *SH4, _: Instr) !void {
    // Accesses data using the contents indicated by effective address Rn.
    // If the cache is hit and there is unwritten information (U bit = 1),
    // the corresponding cache block is written back to external memory and
    // that block is invalidated (the V bit is cleared to 0).
    // If there is no unwritten information (U bit = 0), the block is simply
    // invalidated. No operation is performed in the case of a cache miss
    // or an access to a non-cache area.

    const static = struct {
        var once = true;
    };
    if (static.once) {
        static.once = false;
        sh4_log.warn("Note: obcp @Rn not implemented", .{});
    }
}

pub fn ocbwb_atRn(cpu: *SH4, opcode: Instr) !void {
    // Accesses data using the contents indicated by effective address Rn. If the cache is hit and there is unwritten information (U bit = 1),
    // the corresponding cache block is written back to external memory and that block is cleaned (the U bit is cleared to 0).
    // In other cases (i.e. in the case of a cache miss or an access to a non-cache area, or if the block is already clean), no operation is performed.
    const addr = cpu.R(opcode.nmd.n).*;
    if (addr & (@as(u32, 1) << 25) != 0) {
        // DCA3 Hack
        const index = (addr / 32) & 255;
        sh4_log.debug("  ocbwb {X:0>8}, index={X:0>2}, OIX_CACHE[index]={X:0>8}, dirty={any}, OIX_ADDR[index]={X:0>8}", .{ addr, index, cpu.operand_cache_lines()[index], cpu._operand_cache_state.dirty[index], cpu._operand_cache_state.addr[index] });
        if (cpu._operand_cache_state.addr[index] != ((addr & 0x1FFF_FFFF) & ~@as(u32, 31)))
            sh4_log.warn("  (ocbwb_atRn) Expected OIX_ADDR[index] = {X:0>8}, got {X:0>8}\n", .{ cpu._operand_cache_state.addr[index], (addr & 0x1FFF_FFFF) & ~@as(u32, 31) });
        if (cpu._operand_cache_state.dirty[index]) {
            const target = cpu._operand_cache_state.addr[index] & 0x1FFF_FFFF;
            if (cpu._dc) |dc| {
                const LMMode = dc.read_hw_register(u32, if (target >= 0x11000000 and target < 0x12000000) .SB_LMMODE0 else .SB_LMMODE1);
                const access_32bit = LMMode != 0;
                dc.gpu.write_ta(target, &cpu.operand_cache_lines()[index], if (access_32bit) .b32 else .b64);
            }
            cpu._operand_cache_state.dirty[index] = false;
        }
    }
}

// Reads a 32-byte data block starting at a 32-byte boundary into the operand cache.
// The lower 5 bits of the address specified by Rn are masked to zero.
// This instruction is also used to trigger a Store Queue write-back operation if the specified address points to the Store Queue area.
pub fn pref_atRn(cpu: *SH4, opcode: Instr) !void {
    const addr = cpu.R(opcode.nmd.n).*;
    if (addr & 0xEC000000 == 0xE0000000) {
        const sq_addr: sh4.StoreQueueAddr = @bitCast(addr);
        std.debug.assert(sq_addr.spec == 0b111000);

        //               The full address also includes the sq bit.
        var ext_addr = (addr & 0x03FFFFE0) | (((cpu.read_p4_register(u32, if (sq_addr.sq == 0) .QACR0 else .QACR1) & 0b11100) << 24));

        if (cpu._mmu_enabled) {
            // The SQ area (H'E000 0000 to H'E3FF FFFF) is set in VPN of the UTLB, and the transfer
            // destination external memory address in PPN. The ASID, V, SZ, SH, PR, and D bits have the
            // same meaning as for normal address translation, but the C and WT bits have no meaning
            // with regard to this page. Since burst transfer is prohibited for PCMCIA areas, the SA and TC
            // bits also have no meaning.
            // When a prefetch instruction is issued for the SQ area, address translation is performed and
            // external memory address bits [28:10] are generated in accordance with the SZ bit
            // specification. For external memory address bits [9:5], the address prior to address translation
            // is generated in the same way as when the MMU is off. External memory address bits [4:0]
            // are fixed at 0. Transfer from the SQs to external memory is performed to this address.

            if (sh4.ExperimentalFullMMUSupport) {
                if (addr & 3 != 0 or (cpu.sr.md == 0 and cpu.read_p4_register(sh4.mmu.MMUCR, .MMUCR).sqmd == 1)) {
                    sh4_log.warn("DataAddressErrorRead exception in pref instruction: {X:0>8}", .{addr});
                    return error.DataAddressErrorRead;
                }

                const entry = cpu.utlb_lookup(addr) catch |err| {
                    sh4_log.warn("{s} exception in pref instruction: {X:0>8}", .{ @errorName(err), addr });
                    cpu.report_address_exception(addr);
                    return switch (err) {
                        error.TLBMiss => return error.DataTLBMissRead, // This is a bit weird, but the manual explicitly states that this is treated as a read access (20.3.1).
                        else => std.debug.panic("Unhandled {s} exception in pref instruction: {X:0>8}", .{ @errorName(err), addr }),
                    };
                };
                if (!entry.d) return error.InitialPageWrite;
                ext_addr = entry.translate(addr);
                ext_addr &= 0xFFFFFFE0;
            } else {
                //  This is the simplified version for Ikaruga and other similar games, not a general solution.
                const vpn: u22 = @truncate(addr >> 10);
                for (cpu.utlb) |entry| {
                    if (entry.match(false, 0, vpn)) {
                        ext_addr = (@as(u32, entry.ppn) << 10) | (addr & 0xFFFE0);
                        break;
                    }
                }
            }
        }
        sh4_log.debug("pref @R{d}={X:0>8} : Store queue write back to {X:0>8}", .{ opcode.nmd.n, addr, ext_addr });

        // pref is often used to send commands to the GPU, we can optimize this use case.
        switch (ext_addr) {
            0x10000000...0x107FFFFF, 0x12000000...0x127FFFFF => if (cpu._dc) |dc| dc.gpu.write_ta_fifo_polygon_path_command(cpu.store_queues[sq_addr.sq]),
            0x10800000...0x10FFFFFF, 0x12800000...0x12FFFFFF => if (cpu._dc) |dc| dc.gpu.write_ta_fifo_yuv_converter_path_partial(std.mem.sliceAsBytes(&cpu.store_queues[sq_addr.sq])),
            0x11000000...0x11FFFFFF, 0x13000000...0x13FFFFFF => {
                if (cpu._dc) |dc| {
                    const LMMode = dc.read_hw_register(u32, if (ext_addr >= 0x11000000 and ext_addr < 0x12000000) .SB_LMMODE0 else .SB_LMMODE1);
                    dc.gpu.write_ta_direct_texture_path(addr, if (LMMode != 0) .b32 else .b64, cpu.store_queues[sq_addr.sq]);
                }
            },
            // AICA Memory           System RAM               Texture Memory
            0x00800000...0x009FFFFF, 0x0C000000...0x0FFFFFFF, 0x04000000...0x04FFFFFF, 0x06000000...0x06FFFFFF => {
                @setRuntimeSafety(false);
                const dst: *@Vector(8, u32) = @alignCast(@ptrCast(cpu._get_memory(ext_addr)));
                dst.* = cpu.store_queues[sq_addr.sq];
            },
            // Texture memory, 32bit path
            0x05000000...0x05FFFFFF, 0x07000000...0x07FFFFFF => {
                inline for (0..8) |i|
                    cpu._dc.?.gpu.write_vram(u32, @intCast(ext_addr + 4 * i), cpu.store_queues[sq_addr.sq][i]);
            },
            else => {
                sh4_log.warn("pref: Slow store queue write back to {X:0>8}", .{ext_addr});
                inline for (0..8) |i|
                    cpu.write_physical(u32, @intCast(ext_addr + 4 * i), cpu.store_queues[sq_addr.sq][i]);
            },
        }
    } else {
        const static = struct {
            var once = true;
        };
        if (static.once) {
            static.once = false;
            sh4_log.warn("Note: pref @Rn not implemented outside of store queue operation.", .{});
        }
    }
}

// Returns from an exception or interrupt handling routine by restoring the PC and SR values. Delayed jump.
pub fn rte(cpu: *SH4, _: Instr) !void {
    const delay_slot = cpu.pc + 2;
    const spc = cpu.spc;

    // NOTE:
    // In an RTE delay slot, status register (SR) bits are referenced as follows. In instruction access, the
    // MD bit is used before modification, and in data access, the MD bit is accessed after
    // modification. The other bits - S, T, M, Q, FD, BL, and RB - after modification are used for
    // delay slot instruction execution. The STC and STC.L SR instructions access all SR bits after
    // modification.

    // NOTE: This is how reicast handles it, and thus helps us passing some sh4 unit tests.
    // However I'm not sure this is the **correct** way to do it!
    const old_sr = cpu.sr;
    cpu.sr = @bitCast(cpu.ssr); // Intended! Update SR without triggering side effects like bank swithing.

    execute_delay_slot(cpu, delay_slot);

    cpu.sr = @bitCast(old_sr); // Intended!
    cpu.set_sr(@bitCast(cpu.ssr)); // Actually bank change.

    cpu.pc = spc;
    cpu.pc -%= 2; // Execute will add 2
}

pub fn sets(cpu: *SH4, _: Instr) !void {
    cpu.sr.s = true;
}

pub fn sett(cpu: *SH4, _: Instr) !void {
    cpu.sr.t = true;
}

pub fn sleep(cpu: *SH4, _: Instr) !void {
    const standby_control_register = cpu.read_p4_register(u8, .STBCR);
    if ((standby_control_register & 0b1000_0000) == 0b1000_0000) {
        const standby_control_register_2 = cpu.read_p4_register(u8, .STBCR2);
        if ((standby_control_register_2 & 0b1000_0000) == 0b1000_0000) {
            cpu.execution_state = .DeepSleep;
        } else {
            cpu.execution_state = .Standby;
        }
    } else {
        cpu.execution_state = .Sleep;
    }
    // std.debug.print("\u{001B}[33mSleep State: .{s}\u{001B}[0m\n", .{@tagName(cpu.execution_state)});
}

pub fn stc_Reg_Rn(comptime reg: []const u8) *const fn (cpu: *SH4, opcode: Instr) anyerror!void {
    return (struct {
        pub fn handler(cpu: *SH4, opcode: Instr) !void {
            cpu.R(opcode.nmd.n).* = @bitCast(@field(cpu, reg));
        }
    }).handler;
}

pub fn stc_Rm_BANK_Rn(cpu: *SH4, opcode: Instr) !void {
    cpu.R(opcode.nmd.n).* = cpu.r_bank[opcode.nmd.m & 0b0111];
}

pub fn stcl_Reg_atDecRn(comptime reg: []const u8) *const fn (cpu: *SH4, opcode: Instr) anyerror!void {
    return (struct {
        pub fn handler(cpu: *SH4, opcode: Instr) !void {
            // Use a temporary in case the write raises an exception.
            const tmp = cpu.R(opcode.nmd.n).* - 4;
            try cpu.write(u32, tmp, @bitCast(@field(cpu, reg)));
            cpu.R(opcode.nmd.n).* = tmp;
        }
    }).handler;
}

pub fn stcl_Rm_BANK_atDecRn(cpu: *SH4, opcode: Instr) !void {
    // Use a temporary in case the write raises an exception.
    const tmp = cpu.R(opcode.nmd.n).* - 4;
    try cpu.write(u32, tmp, cpu.r_bank[opcode.nmd.m & 0b0111]);
    cpu.R(opcode.nmd.n).* = tmp;
}

pub fn trapa_imm(cpu: *SH4, opcode: Instr) !void {
    sh4_log.debug("TRAPA #0x{X}\n", .{opcode.nd8.d});
    // Hijack this instruction for debugging purposes.
    if (SH4.EnableTRAPACallback) if (cpu.on_trapa) |c|
        c.callback(c.userdata);
}

pub fn fmov_FRm_FRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = cpu.FR(opcode.nmd.m).*;
    } else {
        if (opcode.nmd.n & 0x1 == 0 and opcode.nmd.m & 0x1 == 0) {
            // fmov DRm,DRn
            cpu.getDRPtr(opcode.nmd.n).* = cpu.getDRPtr(opcode.nmd.m).*;
        } else if (opcode.nmd.n & 0x1 == 1 and opcode.nmd.m & 0x1 == 0) {
            // fmov DRm,XDn
            cpu.getXDPtr(opcode.nmd.n).* = cpu.getDRPtr(opcode.nmd.m).*;
        } else if (opcode.nmd.n & 0x1 == 0 and opcode.nmd.m & 0x1 == 1) {
            // fmov XDm,DRn
            cpu.getDRPtr(opcode.nmd.n).* = cpu.getXDPtr(opcode.nmd.m).*;
        } else if (opcode.nmd.n & 0x1 == 1 and opcode.nmd.m & 0x1 == 1) {
            // fmov XDm,XDn
            cpu.getXDPtr(opcode.nmd.n).* = cpu.getXDPtr(opcode.nmd.m).*;
        }
    }
}
pub fn fmovs_atRm_FRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.m).*));
    } else {
        if (opcode.nmd.n & 0x1 == 0) {
            // fmov.d @Rm,DRn
            cpu.getDRPtr(opcode.nmd.n).* = @bitCast(try cpu.read(u64, cpu.R(opcode.nmd.m).*));
        } else {
            // fmov.d @Rm,XDn
            cpu.getXDPtr(opcode.nmd.n).* = @bitCast(try cpu.read(u64, cpu.R(opcode.nmd.m).*));
        }
    }
}
pub fn fmovs_FRm_atRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.sz == 0) {
        try cpu.write(u32, cpu.R(opcode.nmd.n).*, @bitCast(cpu.FR(opcode.nmd.m).*));
    } else {
        if (opcode.nmd.m & 0x1 == 0) {
            // fmov.d DRm,@Rn
            try cpu.write(u64, cpu.R(opcode.nmd.n).*, @bitCast(cpu.getDRPtr(opcode.nmd.m).*));
        } else {
            // fmov.d XDm,@Rn
            try cpu.write(u64, cpu.R(opcode.nmd.n).*, @bitCast(cpu.getXDPtr(opcode.nmd.m).*));
        }
    }
}
pub fn fmovs_atRmInc_FRn(cpu: *SH4, opcode: Instr) !void {
    // Single-precision
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @bitCast(try cpu.read(u32, cpu.R(opcode.nmd.m).*));
        cpu.R(opcode.nmd.m).* += 4;
    } else { // Double-precision
        if (opcode.nmd.n & 0x1 == 0) {
            // fmov.d @Rm+,DRn
            cpu.getDRPtr(opcode.nmd.n).* = @bitCast(try cpu.read(u64, cpu.R(opcode.nmd.m).*));
        } else {
            // fmov.d @Rm+,XDn
            cpu.getXDPtr(opcode.nmd.n).* = @bitCast(try cpu.read(u64, cpu.R(opcode.nmd.m).*));
        }
        cpu.R(opcode.nmd.m).* += 8;
    }
}
pub fn fmovs_FRm_atDecRn(cpu: *SH4, opcode: Instr) !void {
    // Single-precision
    if (cpu.fpscr.sz == 0) {
        const tmp = cpu.R(opcode.nmd.n).* - 4;
        try cpu.write(u32, tmp, @bitCast(cpu.FR(opcode.nmd.m).*));
        cpu.R(opcode.nmd.n).* = tmp;
    } else { // Double-precision
        const tmp = cpu.R(opcode.nmd.n).* - 8;
        if (opcode.nmd.m & 0x1 == 0) {
            // fmov.d DRm,@-Rn
            try cpu.write(u64, tmp, @bitCast(cpu.getDRPtr(opcode.nmd.m).*));
        } else {
            // fmov.d XDm,@-Rn
            try cpu.write(u64, tmp, @bitCast(cpu.getXDPtr(opcode.nmd.m).*));
        }
        cpu.R(opcode.nmd.n).* = tmp;
    }
}
pub fn fmovs_atR0Rm_FRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.sz == 0) {
        cpu.FR(opcode.nmd.n).* = @bitCast(try cpu.read(u32, cpu.R(0).* +% cpu.R(opcode.nmd.m).*));
    } else {
        if (opcode.nmd.n & 0x1 == 0) {
            // fmov.d @(R0,Rm),DRn
            cpu.getDRPtr(opcode.nmd.n).* = @bitCast(try cpu.read(u64, cpu.R(0).* +% cpu.R(opcode.nmd.m).*));
        } else {
            // fmov.d @(R0,Rm),XDn
            cpu.getXDPtr(opcode.nmd.n).* = @bitCast(try cpu.read(u64, cpu.R(0).* +% cpu.R(opcode.nmd.m).*));
        }
    }
}
pub fn fmovs_FRm_atR0Rn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.sz == 0) {
        try cpu.write(u32, cpu.R(0).* +% cpu.R(opcode.nmd.n).*, @bitCast(cpu.FR(opcode.nmd.m).*));
    } else {
        if (opcode.nmd.m & 0x1 == 0) {
            // fmov.d DRm,@(R0,Rn)
            try cpu.write(u64, cpu.R(0).* +% cpu.R(opcode.nmd.n).*, @bitCast(cpu.getDRPtr(opcode.nmd.m).*));
        } else {
            // fmov.d XDm,@(R0,Rn)
            try cpu.write(u64, cpu.R(0).* +% cpu.R(opcode.nmd.n).*, @bitCast(cpu.getXDPtr(opcode.nmd.m).*));
        }
    }
}

const FloatType = enum {
    SignalingNaN,
    QuietNaN,
    PositiveInfinity,
    PositiveNormalized,
    PositiveDenormalized,
    PositiveZero,
    NegativeZero,
    NegativeDenormalized,
    NegativeNormalized,
    NegativeInfinity,
};

pub fn data_type_of(f: f32) FloatType {
    return switch (@as(u32, @bitCast(f))) {
        0x7FFFFFFF...0x7FC00000 => .SignalingNaN,
        0x7FBFFFFF...0x7F800001 => .QuietNaN,
        0x7F800000 => .PositiveInfinity,
        0x7F7FFFFF...0x00800000 => .PositiveNormalized,
        0x007FFFFF...0x00000001 => .PositiveDenormalized,
        0x00000000 => .PositiveZero,
        0x80000000 => .NegativeZero,
        0x80000001...0x807FFFFF => .NegativeDenormalized,
        0x80800000...0xFF7FFFFF => .NegativeNormalized,
        0xFF800000 => .NegativeInfinity,
        0xFF800001...0xFFBFFFFF => .QuietNaN,
        0xFFC00000...0xFFFFFFFF => .SignalingNaN,
    };
}

pub fn fldi0_FRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.pr == 1) @panic("Illegal instruction");
    cpu.FR(opcode.nmd.n).* = 0.0;
}
pub fn fldi1_FRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.pr == 1) @panic("Illegal instruction");
    cpu.FR(opcode.nmd.n).* = 1.0;
}

test "fldi1_FRn" {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.fpscr.pr = 0;
    fldi1_FRn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });
    std.debug.assert(@as(u32, @bitCast(cpu.FR(0).*)) == 0x3F800000);
}

pub fn flds_FRn_FPUL(cpu: *SH4, opcode: Instr) !void {
    cpu.fpul = @bitCast(cpu.FR(opcode.nmd.n).*);
}

pub fn fsts_FPUL_FRn(cpu: *SH4, opcode: Instr) !void {
    cpu.FR(opcode.nmd.n).* = @bitCast(cpu.fpul);
}

pub fn fabs_FRn(cpu: *SH4, opcode: Instr) !void {
    // This instruction operates only on the high part and thus the operation performed for double and single precision setting is the same. It is not necessary to adjust the FPSRC.PR setting before this instruction.
    cpu.FR(opcode.nmd.n).* = @bitCast(@as(u32, @bitCast(cpu.FR(opcode.nmd.n).*)) & @as(u32, 0x7FFFFFFF));
}

pub fn fneg_FRn(cpu: *SH4, opcode: Instr) !void {
    // See fabs FRn
    cpu.FR(opcode.nmd.n).* = @bitCast(@as(u32, @bitCast(cpu.FR(opcode.nmd.n).*)) ^ @as(u32, 0x80000000));
}

pub fn fadd_FRm_FRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.pr == 0) {
        // TODO: Handle exceptions
        // if(!cpu.fpscr.dn and (n is denorm or m  is denorm)) ...
        cpu.FR(opcode.nmd.n).* += cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.setDR(opcode.nmd.n, cpu.getDR(opcode.nmd.n) + cpu.getDR(opcode.nmd.m));
    }
}
pub fn fsub_FRm_FRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.pr == 0) {
        cpu.FR(opcode.nmd.n).* -= cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.setDR(opcode.nmd.n, cpu.getDR(opcode.nmd.n) - cpu.getDR(opcode.nmd.m));
    }
}
pub fn fmul_FRm_FRn(cpu: *SH4, opcode: Instr) !void {
    // FIXME: There's a lot more to do here.
    if (cpu.fpscr.pr == 0) {
        cpu.FR(opcode.nmd.n).* *= cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.setDR(opcode.nmd.n, cpu.getDR(opcode.nmd.n) * cpu.getDR(opcode.nmd.m));
    }
}
pub fn fmac_FR0_FRm_FRn(cpu: *SH4, opcode: Instr) !void {
    std.debug.assert(cpu.fpscr.pr == 0);
    cpu.FR(opcode.nmd.n).* = @mulAdd(f32, cpu.FR(0).*, cpu.FR(opcode.nmd.m).*, cpu.FR(opcode.nmd.n).*);
}
pub fn fdiv_FRm_FRn(cpu: *SH4, opcode: Instr) !void {
    // FIXME: There's a lot more to do here.
    if (cpu.fpscr.pr == 0) {
        cpu.FR(opcode.nmd.n).* /= cpu.FR(opcode.nmd.m).*;
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.setDR(opcode.nmd.n, cpu.getDR(opcode.nmd.n) / cpu.getDR(opcode.nmd.m));
    }
}
pub fn fsqrt_FRn(cpu: *SH4, opcode: Instr) !void {
    if (cpu.fpscr.pr == 0) {
        if (cpu.FR(opcode.nmd.n).* < 0) {
            cpu.FR(opcode.nmd.n).* = std.math.nan(f32);
        } else {
            cpu.FR(opcode.nmd.n).* = @sqrt(cpu.FR(opcode.nmd.n).*);
        }
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        if (cpu.getDR(opcode.nmd.n) < 0) {
            cpu.setDR(opcode.nmd.n, std.math.nan(f64));
        } else {
            cpu.setDR(opcode.nmd.n, @sqrt(cpu.getDR(opcode.nmd.n)));
        }
    }
}
pub fn fcmp_gt_FRm_FRn(cpu: *SH4, opcode: Instr) !void {
    // TODO: Special float values checks?
    if (cpu.fpscr.pr == 0) {
        cpu.sr.t = (cpu.FR(opcode.nmd.n).* > cpu.FR(opcode.nmd.m).*);
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.sr.t = cpu.getDR(opcode.nmd.n) > cpu.getDR(opcode.nmd.m);
    }
}
pub fn fcmp_eq_FRm_FRn(cpu: *SH4, opcode: Instr) !void {
    // TODO: Special float values checks?
    if (cpu.fpscr.pr == 0) {
        cpu.sr.t = (cpu.FR(opcode.nmd.n).* == cpu.FR(opcode.nmd.m).*);
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        std.debug.assert(opcode.nmd.m & 0x1 == 0);
        cpu.sr.t = cpu.getDR(opcode.nmd.n) == cpu.getDR(opcode.nmd.m);
    }
}
pub fn float_FPUL_FRn(cpu: *SH4, opcode: Instr) !void {
    // NOTE: Experimentation shows the FPUL is treated as signed, at least here. I don't know if this is ALWAYS the case, or not.
    if (cpu.fpscr.pr == 0) {
        cpu.FR(opcode.nmd.n).* = @floatFromInt(as_i32(cpu.fpul));
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);
        cpu.setDR(opcode.nmd.n, @floatFromInt(as_i32(cpu.fpul)));
    }
}

pub fn ftrc_FRn_FPUL(cpu: *SH4, opcode: Instr) !void {
    // Converts the single-precision floating-point number in FRm to a 32-bit integer, and stores the result in FPUL.
    // NOTE: I have no evidence that the conversion should be to a signed integer or not here, however,
    //       it makes sense to be symetrical with float FPUL,FRn, which is signed, I'm pretty sure.

    // NOTE/FIXME: The overflow behavior is different between SH4 and x86. Might want to look into that. Thanks Raziel!
    //        SH4 wants 0x7F800000 if positive, 0xFF800000 if negative.

    // NOTE/TODO: If FPU exceptions are enabled, any out of range result (0x80000000 or 0x7FFFFFFF) will cause an exception instead.

    if (cpu.fpscr.pr == 0) {
        const f = cpu.FR(opcode.nmd.n).*;
        if (std.math.isNan(f)) { // This helps matching reicast behaviour, and thus passing tests, but I'm not sure if this accurate.
            cpu.fpul = 0;
        } else {
            const u: u32 = @bitCast(f);
            if ((u & 0x80000000) == 0) {
                if (u > 0x7F800000) {
                    cpu.fpul = 0x80000000;
                } else if (u > 0x4EFFFFFF) {
                    cpu.fpul = 0x7FFFFF80; // Some sources say it should be 0x7FFFFFFF
                } else {
                    cpu.fpul = @bitCast(std.math.lossyCast(i32, f));
                }
            } else {
                if ((u & 0x7FFFFFFF) > (0xCF000000 & 0x7FFFFFFF)) {
                    cpu.fpul = 0x80000000;
                } else {
                    cpu.fpul = @bitCast(std.math.lossyCast(i32, f));
                }
            }
        }
    } else {
        std.debug.assert(opcode.nmd.n & 0x1 == 0);

        const f = cpu.getDR(opcode.nmd.n);
        const u: u64 = @bitCast(f);
        if ((u & 0x80000000_00000000) == 0) {
            if (u > 0x7FF00000_00000000) {
                cpu.fpul = 0x80000000;
            } else if (u >= 0x41E0000000000000) {
                cpu.fpul = 0x7FFFFFFF;
            } else {
                cpu.fpul = @bitCast(std.math.lossyCast(i32, f));
            }
        } else {
            if ((u & 0x7FFFFFFFFFFFFFFF) >= (0xC1E0000000200000 & 0x7FFFFFFFFFFFFFFF)) {
                cpu.fpul = 0x80000000;
            } else {
                cpu.fpul = @bitCast(std.math.lossyCast(i32, f));
            }
        }
    }
}

pub fn fipr_FVm_FVn(cpu: *SH4, opcode: Instr) !void {
    // Computes the dot product of FVn and FVm and stores it into pub fn+3.
    const n = opcode.nmd.n & 0b1100;
    const m = (opcode.nmd.n << 2) & 0b1100;
    // FIXME: Not accurate to the actual hardware implementation.

    const FVn: @Vector(4, f32) = @as([*]f32, @ptrCast(cpu.FR(n)))[0..4].*;
    const FVm: @Vector(4, f32) = @as([*]f32, @ptrCast(cpu.FR(m)))[0..4].*;
    switch (Experimental.fpir) {
        .FMA => {
            // This passes units tests.
            var tmp: f32 = FVn[0] * FVm[0];
            tmp = @mulAdd(f32, FVn[1], FVm[1], tmp);
            tmp = @mulAdd(f32, FVn[2], FVm[2], tmp);
            tmp = @mulAdd(f32, FVn[3], FVm[3], tmp);
            cpu.FR(n + 3).* = tmp;
        },
        .Reduce => {
            // This produces slightly shorter assembly.
            @setFloatMode(.optimized); // Strict: 94/500 fails, Optimized: 117/500 fails.
            cpu.FR(n + 3).* = @reduce(.Add, FVn * FVm);
        },
    }
}

pub fn test_fipr(n: [4]f32, m: [4]f32) !void {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.fpscr.pr = 0;

    for (0..4) |i| {
        cpu.FR(@intCast(i)).* = n[i];
    }

    for (0..4) |i| {
        cpu.FR(@intCast(4 + i)).* = m[i];
    }

    fipr_FVm_FVn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0b0001, .m = undefined, .d = undefined } });

    try std.testing.expect(cpu.FR(3).* == n[0] * m[0] + n[1] * m[1] + n[2] * m[2] + n[3] * m[3]);
}

test "fipr" {
    try test_fipr(.{ 1, 2, 3, 4 }, .{ 4, 3, 2, 1 });
    try test_fipr(.{ 0, 0, 0, 0 }, .{ 0, 0, 0, 0 });
    try test_fipr(.{ 1.5, 2.5, 3.5, 4.5 }, .{ 4.5, 3.5, 2.5, 1.5 });
    try test_fipr(.{ 1, 2, 3, 4 }, .{ 0, 0, 0, 0 });
}

pub fn ftrv_XMTRX_FVn(cpu: *SH4, opcode: Instr) !void {
    std.debug.assert(cpu.fpscr.pr == 0);
    // NOTE: Doesn't handle exceptions.
    const n = opcode.nmd.n & 0b1100;

    // NOTE: Should always set these flags. Fails the unit tests though :')
    // cpu.fpscr.inexact = true;
    // cpu.fpscr.cause_inexact = true;

    @setFloatMode(.optimized);

    const pFVn = @as([*]f32, @ptrCast(cpu.FR(n + 0)))[0..4];
    const FVn: @Vector(4, f32) = pFVn.*;
    const pXMTRX = @as([*]f32, @ptrCast(cpu.XF(0)))[0..16];
    const XMTRX = [4]@Vector(4, f32){
        pXMTRX[0..4].*,
        pXMTRX[4..8].*,
        pXMTRX[8..12].*,
        pXMTRX[12..16].*,
    };
    const FRs = .{
        @shuffle(f32, FVn, undefined, [4]i32{ 0, 0, 0, 0 }),
        @shuffle(f32, FVn, undefined, [4]i32{ 1, 1, 1, 1 }),
        @shuffle(f32, FVn, undefined, [4]i32{ 2, 2, 2, 2 }),
        @shuffle(f32, FVn, undefined, [4]i32{ 3, 3, 3, 3 }),
    };
    const r = FRs[0] * XMTRX[0] + FRs[1] * XMTRX[1] + FRs[2] * XMTRX[2] + FRs[3] * XMTRX[3];
    pFVn.* = .{ r[0], r[1], r[2], r[3] };
}

// Column major matrix
pub fn test_ftrv(v: [4]f32, m: [4 * 4]f32, r: [4]f32) !void {
    var cpu = try SH4.init(std.testing.allocator, null);
    defer cpu.deinit();
    cpu.fpscr.pr = 0;

    for (0..4) |i| {
        cpu.FR(@intCast(i)).* = v[i];
    }
    for (0..16) |i| {
        cpu.XF(@intCast(i)).* = m[i];
    }

    ftrv_XMTRX_FVn(&cpu, .{ .nmd = .{ ._ = undefined, .n = 0, .m = undefined, .d = undefined } });

    for (0..4) |i| {
        try std.testing.expect(cpu.FR(@intCast(i)).* == r[i]);
    }
}

test "ftrv XMTRX_FVn" {
    try test_ftrv(.{ 1.0, 2.0, 3.0, 4.0 }, .{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    }, .{ 1.0, 2.0, 3.0, 4.0 });

    try test_ftrv(.{ 1.0, 2.0, 3.0, 4.0 }, .{
        2, 0, 0, 0,
        0, 2, 0, 0,
        0, 0, 2, 0,
        0, 0, 0, 2,
    }, .{ 2.0, 4.0, 6.0, 8.0 });

    try test_ftrv(.{ 1.0, 2.0, 3.0, 4.0 }, .{
        2, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
    }, .{ 2.0, 0.0, 0.0, 0.0 });
}

pub fn fsrra_FRn(cpu: *SH4, opcode: Instr) !void {
    std.debug.assert(cpu.fpscr.pr == 0);
    if (cpu.FR(opcode.nmd.n).* < 0) {
        cpu.FR(opcode.nmd.n).* = std.math.nan(f32);
    } else {
        cpu.FR(opcode.nmd.n).* = 1.0 / @sqrt(cpu.FR(opcode.nmd.n).*);
    }
}

pub fn fsca_FPUL_DRn(cpu: *SH4, opcode: Instr) !void {
    std.debug.assert(cpu.fpscr.pr == 0);
    std.debug.assert(opcode.nmd.n & 1 == 0);

    const fraction = cpu.fpul & 0x0000_FFFF;
    const angle = 2 * std.math.pi * @as(f32, @floatFromInt(fraction)) / 0x10000;

    cpu.FR(opcode.nmd.n).* = @sin(angle);
    cpu.FR(opcode.nmd.n + 1).* = @cos(angle);
}

pub fn fcnvds_DRn_FPUL(cpu: *SH4, opcode: Instr) !void {
    std.debug.assert(cpu.fpscr.pr == 1);
    std.debug.assert(opcode.nmd.n & 1 == 0);
    cpu.fpul = @bitCast(@as(f32, @floatCast(cpu.getDR(opcode.nmd.n))));
}

pub fn fcnvsd_FPUL_DRn(cpu: *SH4, opcode: Instr) !void {
    std.debug.assert(cpu.fpscr.pr == 1);
    std.debug.assert(opcode.nmd.n & 1 == 0);
    cpu.setDR(opcode.nmd.n, @floatCast(@as(f32, @bitCast(cpu.fpul))));
}

pub fn syscall_sysinfo(cpu: *SH4, _: Instr) !void {
    syscall.syscall_sysinfo(cpu._dc.?);
}

pub fn syscall_romfont(cpu: *SH4, _: Instr) !void {
    syscall.syscall_romfont(cpu._dc.?);
}

pub fn syscall_flashrom(cpu: *SH4, _: Instr) !void {
    syscall.syscall_flashrom(cpu._dc.?);
}

pub fn syscall_gdrom(cpu: *SH4, _: Instr) !void {
    syscall.syscall_gdrom(cpu._dc.?);
}

pub fn syscall_misc(cpu: *SH4, _: Instr) !void {
    syscall.syscall_misc(cpu._dc.?);
}

pub fn syscall_unknown(cpu: *SH4, _: Instr) !void {
    syscall.syscall(cpu._dc.?);
}
