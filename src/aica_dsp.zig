const std = @import("std");
const log = std.log.scoped(.dsp);

const AICAModule = @import("aica.zig");

// HEAVILY based on Neill Corlett's Yamaha AICA notes, most of the comments come directly from there.

const Instruction = packed struct(u64) {
    _0_6: u7,
    NXADR: bool,
    ADREB: bool,
    MASA: u6,
    NOFL: bool,
    // -----
    BSEL: bool,
    ZERO: bool,
    NEGB: bool,
    YRL: u1,
    SHFT: u2,
    FRCL: u1,
    ADRL: bool,
    EWA: u4,
    EWT: bool,
    MRD: bool,
    MWT: bool,
    TABLE: u1,
    // -----
    _32: u1,
    IWA: u5,
    IWT: u1,
    IRA: u6,
    YSEL: u2,
    XSEL: u1,
    // -----
    _48: u1,
    TWA: u7,
    TWT: u1,
    TRA: u7,
};

fn saturate(comptime T: type, value: anytype) T {
    std.debug.assert(@bitSizeOf(T) < @bitSizeOf(@TypeOf(value)));
    return @truncate(@max(@min(value, @as(@TypeOf(value), std.math.maxInt(T))), @as(@TypeOf(value), std.math.minInt(T))));
}

// TODO
// To convert from 16-bit float to 24-bit integer (on read):
//  - Take the mantissa and shift it left by 11
//  - Make bit 23 be the sign bit
//  - Make bit 22 be the reverse of the sign bit
//  - Shift right (signed) by the exponent
fn convert_from_16bit_float(value: u16) i24 {
    _ = value;
    return 0;
}

// To convert from 24-bit integer to 16-bit float (on write):
// - While the top 2 bits are the same, shift left
//   The number of times you have to shift becomes the exponent
// - Shift right (signed) by 11
// - Set bits 14-11 to the exponent value
fn convert_to_16bit_float(value: i24) u16 {
    const sign_bit: u1 = @truncate(@as(u24, @bitCast(value)) >> 23);
    var u: u24 = @bitCast(value);

    var exponent: u4 = 0;
    while (exponent < 15 and u & @as(u24, 0b11 << 22) == 0 or u & @as(u24, 0b11 << 22) == @as(u24, 0b11) << 22) {
        u <<= 1;
        exponent += 1;
    }
    var val: u16 = @truncate(@as(u24, @bitCast(value >> 11)));
    val &= 0x3FF;
    val |= @as(u16, exponent) << 11;
    val |= @as(u16, sign_bit) << 15;

    return val;
}

/// 16-bit unsigned register which is decremented on every sample
MDEC_CT: u16 = 1,

temp_word: [4]u16 = .{0} ** 4,

_regs: []u32, // Memory backing for internal registers
_memory: []u8,
_ring_buffer: *const AICAModule.RingBufferAddress,

_dirty_mpro: bool = false,

pub fn init(ring_buffer: *const AICAModule.RingBufferAddress, registers: []u32, memory: []u8) @This() {
    return .{ ._ring_buffer = ring_buffer, ._regs = registers, ._memory = memory };
}

pub inline fn read_register(self: *@This(), comptime T: type, local_addr: u32) T {
    return switch (T) {
        u8 => @as([*]const u8, @ptrCast(&self._regs[0]))[local_addr],
        u32 => self._regs[(local_addr) / 4],
        else => @compileError("Invalid value type"),
    };
}

pub inline fn write_register(self: *@This(), comptime T: type, local_addr: u32, value: T) void {
    if (local_addr >= 0x0400 and local_addr < 0x0C00)
        self._dirty_mpro = true;

    switch (T) {
        u8 => @as([*]u8, @ptrCast(self._regs.ptr))[local_addr] = value,
        u32 => self._regs[local_addr / 4] = value & 0xFFFF,
        else => @compileError("Invalid value type"),
    }
}

fn read(self: *@This(), comptime T: type, addr: u32) T {
    log.debug("Read({any}): {X}", .{ T, addr });
    return std.mem.bytesAsValue(T, self._memory[addr & 0x1FFFFF ..]).*;
}

fn write(self: *@This(), comptime T: type, addr: u32, value: T) void {
    log.debug("Write({any}): {X} = {X}", .{ T, addr, value });
    std.mem.bytesAsValue(T, self._memory[addr & 0x1FFFFF ..]).* = value;
}

/// 0x3000-0x31FF: Coefficients (COEF), 128 registers, 13 bits each
///               0x3000: bits 15-3 = COEF(0)
///               0x3004: bits 15-3 = COEF(1)
///               ...
///               0x31FC: bits 15-3 = COEF(127)
///               You could interpret these as 16-bit signed (2's complement)
///               coefficients with the lowest 3 bits are always 0.
///               Each of the 128 COEFs is used by the corresponding instruction (MPRO).
fn read_coef(self: *@This(), idx: usize) u13 {
    return @truncate(self._regs[idx] >> 3);
}

/// 0x3200-0x32FF: External memory addresses (MADRS), 64 registers, 16 bits each
///                0x3200: bits 15-0 = MADRS(0)
///                ...
///                0x3204: bits 15-0 = MADRS(1)
///                0x32FC: bits 15-0 = MADRS(63)
///                These are memory offsets that refer to locations in the
///                external ringbuffer.  Every increment of a MADRS register
///                represents 2 bytes.
fn _madrs(self: *@This(), idx: usize) *u16 {
    return @ptrCast(&self._regs[0x200 / 4 + idx]);
}

/// 0x3400-0x3BFF: DSP program (MPRO), 128 registers, 64 bits each
///                0x3400: bits 15-0 = bits 63-48 of first instruction
///                0x3404: bits 15-0 = bits 47-32 of first instruction
///                0x3408: bits 15-0 = bits 31-16 of first instruction
///                0x340C: bits 15-0 = bits 15-0  of first instruction
///                0x3410: bits 15-0 = bits 63-48 of second instruction
///                ...
///                0x3BFC: bits 15-0 = bits 15-0  of last instruction
const MPRO_base = 0x400 / 4;
fn read_mpro(self: *@This(), idx: usize) Instruction {
    const parts = [4]u16{
        @truncate(self._regs[MPRO_base + 4 * idx + 0]),
        @truncate(self._regs[MPRO_base + 4 * idx + 1]),
        @truncate(self._regs[MPRO_base + 4 * idx + 2]),
        @truncate(self._regs[MPRO_base + 4 * idx + 3]),
    };
    return @bitCast((@as(u64, parts[0]) << 48) | (@as(u64, parts[1]) << 32) | (@as(u64, parts[2]) << 16) | (@as(u64, parts[3]) << 0));
}

/// 0x4000-0x43FF: Temp buffer (TEMP), 128 registers, 24 bits each
///                0x4000: bits 7-0  = bits 7-0 of TEMP(0)
///                0x4004: bits 15-0 = bits 23-8 of TEMP(0)
///                0x4008: bits 7-0  = bits 7-0 of TEMP(1)
///                ...
///                0x43FC: bits 15-0 = bits 23-8 of TEMP(127)
///                The temp buffer is configured as a ring buffer, so pointers
///                referring to it decrement by 1 each sample.
const TEMP_base = 0x1000 / 4;
fn read_temp(self: *@This(), idx: usize) u24 {
    const lo = self._regs[TEMP_base + 2 * idx];
    const hi = self._regs[TEMP_base + 2 * idx + 1];
    return @truncate(((hi & 0xFFFF) << 8) | (lo & 0xFF));
}
fn write_temp(self: *@This(), idx: usize, value: u24) void {
    self._regs[TEMP_base + 2 * idx] = value & 0xFF;
    self._regs[TEMP_base + 2 * idx + 1] = (value >> 8) & 0xFFFF;
}

// 0x4400-0x44FF: Memory data (MEMS), 32 registers, 24 bits each
//                0x4400: bits 7-0  = bits 7-0 of MEMS(0)
//                0x4404: bits 15-0 = bits 23-8 of MEMS(0)
//                0x4408: bits 7-0  = bits 7-0 of MEMS(1)
//                ...
//                0x44FC: bits 15-0 = bits 23-8 of MEMS(31)
//                Used for holding data that was read out of the ringbuffer.
const MEMS_base: u32 = 0x1400 / 4;
fn read_mems(self: *@This(), idx: usize) u24 {
    const lo = self._regs[MEMS_base + 2 * idx];
    const hi = self._regs[MEMS_base + 2 * idx + 1];
    return @truncate(((hi & 0xFFFF) << 8) | (lo & 0xFF));
}
fn write_mems(self: *@This(), idx: usize, value: u24) void {
    self._regs[MEMS_base + 2 * idx] = value & 0xFF;
    self._regs[MEMS_base + 2 * idx + 1] = (value >> 8) & 0xFFFF;
}

// 0x4500-0x457F: Mixer input data (MIXS), 16 registers, 20 bits each
//                0x4500: bits 3-0  = bits 3-0 of MIXS(0)
//                0x4504: bits 15-0 = bits 19-4 of MIXS(0)
//                0x4508: bits 3-0  = bits 3-0 of MIXS(1)
//                ...
//                0x457C: bits 15-0 = bits 19-4 of MIXS(15)
//                These are the 16 send buses coming from the 64 main channels.
const MIXS_base: u32 = 0x1500 / 4;
fn read_mixs(self: *@This(), idx: usize) u20 {
    const low = self._regs[MIXS_base + 2 * idx];
    const high = self._regs[MIXS_base + 2 * idx + 1];
    return @truncate(((high & 0xFFFF) << 4) | (low & 0xF));
}
fn write_mixs(self: *@This(), idx: usize, value: u24) void {
    self._regs[MIXS_base + 2 * idx] = value & 0xFF;
    self._regs[MIXS_base + 2 * idx + 1] = (value >> 8) & 0xFFFF;
}
pub fn add_mixs(self: *@This(), idx: usize, value: u24) void {
    self.write_mixs(idx, self.read_mixs(idx) +| value);
}

// 0x4580-0x45BF: Effect output data (EFREG), 16 registers, 16 bits each
//                0x4580: bits 15-0 = EFREG(0)
//                0x4584: bits 15-0 = EFREG(1)
//                ...
//                0x45BC: bits 15-0 = EFREG(15)
//                These are the 16 sound outputs.
fn _efreg(self: *@This(), idx: usize) *u16 {
    return @ptrCast(&self._regs[@as(u32, 0x1580 / 4) + idx]);
}
pub fn read_efreg(self: *@This(), idx: usize) u16 {
    return self._efreg(idx).*;
}

// 0x45C0-0x45C7: External input data stack (EXTS), 2 registers, 16 bits each
//                0x45C0: bits 15-0 = EXTS(0)
//                0x45C4: bits 15-0 = EXTS(1)
//                These come from CDDA left and right, respectively.
fn _exts(self: *@This(), idx: usize) *u16 {
    return @ptrCast(&self._regs[@as(u32, 0x15C0 / 4) + idx]);
}

pub fn set_exts(self: *@This(), idx: usize, value: u16) void {
    self._exts(idx).* = value;
}

pub fn generate_sample(self: *@This()) void {
    // 26-bit signed addend
    var B: i26 = 0;
    // 24-bit signed multiplicand
    var X: i24 = 0;
    // 13-bit signed multiplier
    var Y: i13 = 0;
    // 26-bit signed accumulator
    var ACC: i26 = 0;
    // 24-bit signed shifted output value
    var SHIFTED: i24 = 0;
    // 24-bit signed latch register
    var Y_REG: i24 = 0;
    // 13-bit fractional address
    var FRC_REG: u13 = 0;
    // 12-bit whole address
    var ADRS_REG: u12 = 0;

    for (0..128) |step| {
        const instruction = self.read_mpro(@intCast(step));

        // If the source register is less than 24 bits, it's promoted to 24-bit by shifting left.
        const INPUTS: u24 = switch (instruction.IRA) {
            0x00...0x1F => |reg| self.read_mems(reg),
            0x20...0x2F => |reg| @as(u24, self.read_mixs(reg - 0x20)) << 4,
            0x30...0x31 => |reg| @as(u24, self._exts(@intCast(reg - 0x30)).*) << 8,
            else => 0,
        };

        if (instruction.IWT == 1) {
            // If IWT is set, then the memory data from a MRD (either 2 or 3 instructions ago) is copied into the MEMS register indicated by IWA (0x00-0x1F).
            // If NOFL was set 2 instructions ago (only), the value is simply promoted from 16 to 24-bit by shifting left.
            // Otherwise, it's converted from 16-bit float format to 24-bit integer.  (See float notes)
            std.debug.assert(step >= 2);
            const value: u24 = if (self.read_mpro(@intCast(step - 2)).NOFL)
                @truncate(@as(u24, self.temp_word[(step - 2) % self.temp_word.len]) << 8)
            else
                convert_to_16bit_float(@intCast(@as(i16, @bitCast(self.temp_word[(step - 2) % self.temp_word.len]))));
            self.write_mems(instruction.IWA, value);
        }

        // - B selection
        // A 26-bit value (B) is read from one of two sources:
        //
        // If BSEL=0: The TEMP register indicated by ((TRA + MDEC_CT) & 0x7F).
        //            This value is sign-extended from 24 to 26 bits, not shifted.
        // If BSEL=1: The accumulator (ACC), already 26 bits.
        //
        // If NEGB=1, this value is then made negative. (B becomes 0-B)
        // If ZERO=1, this value becomes zero, regardless of all other bits.
        B = if (instruction.BSEL)
            @intCast(@as(i24, @bitCast(self.read_temp((instruction.TRA + self.MDEC_CT) & 0x7F))))
        else
            ACC;
        if (instruction.NEGB)
            B = ~B +% 1;
        if (instruction.ZERO)
            B = 0;

        //- X selection
        //   A 24-bit value (X) is read from one of two sources:
        //     If XSEL=0: One of the TEMP registers indicated by ((TRA + MDEC_CT) & 0x7F).
        //     If XSEL=1: The INPUTS register.
        X = if (instruction.XSEL == 0)
            @bitCast(self.read_temp((instruction.TRA + self.MDEC_CT) & 0x7F))
        else
            @bitCast(INPUTS);

        // - Y selection
        //   A 13-bit value (Y) is read from one of four sources:
        //     If YSEL=0: Y becomes FRC_REG
        //     If YSEL=1: Y becomes this instruction's COEF
        //     If YSEL=2: Y becomes bits 23-11 of Y_REG
        //     If YSEL=3: Y becomes bits 15-4 of Y_REG, ZERO-EXTENDED to 13 bits.
        Y = switch (instruction.YSEL) {
            0 => @bitCast(FRC_REG),
            1 => @bitCast(self.read_coef(step)),
            2 => @truncate(Y_REG >> 11),
            3 => @truncate(Y_REG >> 4),
        };

        // - Y latch
        //   If YRL is set, Y_REG becomes INPUTS.
        if (instruction.YRL == 1)
            Y_REG = @bitCast(INPUTS);

        // - Shift of previous accumulator
        //   A 24-bit value (SHIFTED) is set to one of the following:
        //     If SHFT=0: SHIFTED becomes ACC, with saturation
        //     If SHFT=1: SHIFTED becomes ACC*2, with saturation
        //     If SHFT=2: SHIFTED becomes ACC*2, with the highest bits simply cut off
        //     If SHFT=3: SHIFTED becomes ACC, with the highest bits simply cut off
        //
        //     If saturation is enabled, SHIFTED is clipped to the range
        //     -0x800000...0x7FFFFF.
        SHIFTED = switch (instruction.SHFT) {
            0 => saturate(i24, ACC),
            1 => saturate(i24, ACC *| 2),
            2 => @truncate(ACC *% 2),
            3 => @truncate(ACC),
        };

        //  - Multiply and accumulate
        //   A 26-bit value (ACC) becomes ((X * Y) >> 12) + B.
        //   The multiplication is signed.  I don't think the addition includes saturation.
        ACC = X;
        ACC *%= Y;
        ACC >>= 12;
        ACC +%= B;

        // - Temp write
        //  If TWT is set, the value SHIFTED is written to the temp buffer address indicated by ((TWA + MDEC_CT) & 0x7F).
        if (instruction.TWT == 1)
            self.write_temp((instruction.TWA + self.MDEC_CT) & 0x7F, @bitCast(SHIFTED));

        //  - Fractional address latch
        //   If FRCL is set, FRC_REG (13-bit) becomes one of the following:
        //     If SHFT=3: FRC_REG = lowest 12 bits of SHIFTED, ZERO-EXTENDED to 13 bits.
        //     Other values of SHFT: FRC_REG = bits 23-11 of SHIFTED.
        if (instruction.FRCL == 1) {
            const s = @as(u24, @bitCast(SHIFTED));
            FRC_REG = switch (instruction.SHFT) {
                3 => @as(u12, @truncate(s)),
                else => @truncate(s >> 11),
            };
        }

        // - Memory operations
        //
        //   If either MRD or MWT are set, we are performing a memory operation (on the
        //   external ringbuffer) and we'll need to compute an address.
        //
        //   Start with the 16-bit value of the MADRS register indicated by MASA
        //   (0x00-0x3F).
        var addr: u32 = self._madrs(instruction.MASA).*;
        //   If TABLE=0, add the 16-bit value MDEC_CT.
        if (instruction.TABLE == 0)
            addr +%= self.MDEC_CT;
        //   If ADREB=1, add the 12-bit value ADRS_REG, zero-extended.
        if (instruction.ADREB)
            addr +%= ADRS_REG;
        //   If NXADR=1, add 1.
        if (instruction.NXADR)
            addr +%= 1;
        //   If TABLE=0, mod the address so it's within the proper ringbuffer size.
        //     For a 8Kword  ringbuffer: AND by 0x1FFF
        //     For a 16Kword ringbuffer: AND by 0x3FFF
        //     For a 32Kword ringbuffer: AND by 0x7FFF
        //     For a 64Kword ringbuffer: AND by 0xFFFF
        //   If TABLE=1, simply AND the address by 0xFFFF.
        if (instruction.TABLE == 0)
            switch (self._ring_buffer.size) {
                .@"8k" => addr &= 0x1FFF,
                .@"16k" => addr &= 0x3FFF,
                .@"32k" => addr &= 0x7FFF,
                .@"64k" => addr &= 0xFFFF,
            }
        else
            addr &= 0xFFFF;
        //   Shift the address left by 1 (so it's referring to a word offset).
        addr <<= 1;
        //   Add (RBP<<11).
        addr +%= @as(u32, self._ring_buffer.pointer) << 11;
        //   This is the address, in main memory, you'll be reading or writing.

        //   If MRD is set, read the 16-bit word at the calculated address and put it
        //   in a temporary place.  It will then be accessible by the instruction either
        //   2 or 3 steps ahead.  That instruction will have to use a IWT to get the
        //   result.  Don't perform any conversions yet; let the IWT handle it later on.
        if (instruction.MRD)
            self.temp_word[step % self.temp_word.len] = self.read(u16, addr);

        //   If MWT is set, write the value of SHIFTED into memory at the calculated
        //   address.  If NOFL=1, simply shift it right by 8 to make it 16-bit.
        //   Otherwise, convert from 24-bit integer to 16-bit float (see float notes).
        if (instruction.MWT) {
            const value: u16 = if (instruction.NOFL) @truncate(@as(u24, @bitCast(SHIFTED)) >> 8) else convert_to_16bit_float(SHIFTED);
            self.write(u16, addr, value);
        }

        //- Address latch
        //   If ADRL is set, ADRS_REG (12-bit) becomes one of the following:
        //     If SHFT=3: ADRS_REG = INPUTS >> 16 (signed shift with sign extension).
        //     Other values of SHFT: ADRS_REG = bits 23-12 of SHIFTED.
        if (instruction.ADRL)
            ADRS_REG = @truncate(if (instruction.SHFT == 3) @as(u24, @bitCast(@divTrunc(@as(i24, @bitCast(INPUTS)), 65_536))) else @as(u24, @bitCast(SHIFTED)) >> 12);

        // - Effect output write
        //   If EWT is set, write (SHIFTED >> 8) into the EFREG register specified by EWA (0x0-0xF).
        if (instruction.EWT) self._efreg(instruction.EWA).* = @as(u16, @truncate(@as(u24, @bitCast(SHIFTED)) >> 8));
    }

    self.MDEC_CT -= 1;
    if (self.MDEC_CT == 0)
        self.MDEC_CT = @intCast(self._ring_buffer.size_in_samples() - 1);
}
