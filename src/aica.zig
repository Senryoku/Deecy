const std = @import("std");
const termcolor = @import("termcolor");

const aica_log = std.log.scoped(.aica);

const arm7 = @import("arm7");
const ARM7JIT = @import("jit/arm_jit.zig").ARM7JIT;

const Dreamcast = @import("dreamcast.zig").Dreamcast;

// Yamaha AICA Audio Chip
// Most notable source outside of official docs: Neill Corlett's Yamaha AICA notes

const SampleFormat = enum(u2) {
    i16 = 0, // 16-bit signed little-endian
    i8 = 1, // 8-bit signed
    ADPCM = 2, // 4-bit Yamaha ADPCM
    Invalid = 3,
};

const EnvelopeState = enum(u2) {
    Attack = 0,
    Decay = 1,
    Sustain = 2,
    Release = 3,
};

const PlayControl = packed struct(u32) {
    start_address: u7, // Highest bits of the address
    sample_format: SampleFormat,
    sample_loop: bool,
    noise_enabled: bool,
    _0: u3 = 0,
    key_on_bit: bool,
    key_on_execute: bool,
    _1: u16 = 0,

    const Mask: u32 = 0x47FF; // NOTE: key_on_execute is not saved and will always be read as 0.
};

pub const AmpEnv1 = packed struct(u32) {
    attack_rate: u5,
    _: u1 = 0,
    decay_rate: u5,
    sustain_rate: u5,
    _0: u16,
};

pub const AmpEnv2 = packed struct(u32) {
    release_rate: u5,
    decay_level: u5,
    key_rate_scaling: u4,
    link: u1, // LPSLNK - If this bit is set, then the envelope transitions to the decay state when the sample loop start address is exceeded.
    _: u17 = 0,
};

pub const SamplePitchRate = packed struct(u32) {
    fns: u11,
    oct: u4, // octave (OCT), signed value, -0x8 to 0x7
    _: u17 = 0,
};

pub const LFOControl = packed struct(u32) {
    amplitude_modulation_depth: u3, // (ALFOS)
    amplitude_modulation_waveform: u2, // (ALFOWS)
    pitch_modulation_depth: u3, // (PLFOS)
    pitch_modulation_waveform: u2, // (PLFOWS)
    frequency: u5, // (LFOF)
    reset: u1, // (LFORE) 1=on, 0=off If set, the LFO phase is reset at the start of EACH SAMPLE LOOP.
    _: u16,
};

pub const LPF1Volume = packed struct(u32) {
    q: u5, // filter resonance value Q = (0.75*value)-3, from -3 to 20.25 db
    lpoff: bool, // 1 = turn off lowpass filter.
    voff: bool, // if this bit is set to 1, the constant attenuation, envelope, and LFO volumes will not take effect. however, the note will still end when the envelope level reaches zero in the release state.
    _: u1, // unknown [SAVED]
    constant_attenuation: u8, // this value *4 seems to be added to the envelope attenuation (as in, 0x00-0xFF here corresponds to 0x000-0x3FF when referring to the envelope attenuation)
    _r: u16,
};

pub const LPF7 = packed struct(u32) {
    lpf_decay_rate: u5,
    _0: u3,
    lpf_attack_rate: u5,
    _1: u3,
    _2: u16,

    pub const Mask: u32 = 0x1F1F;
};

pub const LPF8 = packed struct(u32) {
    lpf_release_rate: u5,
    _0: u3,
    lpf_sustain_rate: u5,
    _1: u3,
    _2: u16,

    pub const Mask: u32 = 0x1F1F;
};

// NOTE: Only the lower 16bits of each registers are actually used.
pub const AICAChannel = packed struct(u576) {
    play_control: PlayControl,
    sample_address_low: u32, // lowest 16 bits of sample start address (SA) (in bytes)
    loop_start: u32,
    loop_end: u32,
    amp_env_1: AmpEnv1,
    amp_env_2: AmpEnv2,
    sample_pitch_rate: SamplePitchRate,
    lfo_control: LFOControl,
    dps_channel_send: u32,
    direct_pan_vol_send: u32,
    lpf1_volume: LPF1Volume,
    lpf2: u32, // Bits 0-12: Filter value
    lpf3: u32, // Bits 0-12: Filter value
    lpf4: u32, // Bits 0-12: Filter value
    lpf5: u32, // Bits 0-12: Filter value
    lpf6: u32, // Bits 0-12: Filter value
    lpf7: LPF7,
    lpf8: LPF8,

    pub fn sample_address(self: *const AICAChannel) u32 {
        return (@as(u32, self.play_control.start_address) << 16) | (self.sample_address_low & 0xFFFF);
    }
};

// Address of AICA registers. Add 0x00700000 for access from SH4 and 0x00800000 for access from ARM7
pub const AICARegister = enum(u32) {
    MasterVolume = 0x00002800,
    RingBufferAddress = 0x00002804,
    MIDIInput = 0x00002808,
    ChannelInfoReq = 0x0000280C,
    PlayStatus = 0x00002810,
    PlayPosition = 0x00002814,

    IC_TEST = 0x00002880,
    DMEA = 0x00002884,
    DGATE = 0x00002888,
    DDIR_DEXE = 0x0000288C,

    TACTL_TIMA = 0x00002890,
    TBCTL_TIMB = 0x00002894,
    TCCTL_TIMC = 0x00002898,

    SCIEB = 0x0000289C,
    SCIPD = 0x000028A0,
    SCIRE = 0x000028A4,

    SCILV0 = 0x000028A8,
    SCILV1 = 0x000028AC,
    SCILV2 = 0x000028B0,

    MCIEB = 0x000028B4, // Main CPU Interrupt Enable
    MCIPD = 0x000028B8, // Main CPU Interrupt Pending
    MCIRE = 0x000028BC, // Main CPU Interrupt Reset

    ARMRST = 0x00002C00,
    INTRequest = 0x00002D00, // Named "L" in the manual. Only 3 bits are actually used.
    INTClear = 0x00002D04, // Named "M" in the manual. Write 1 to clear INTRequest.

    RTC_High = 0x00010000,
    RTC_Low = 0x00010004,
    RTC_Write_Enable = 0x00010008,

    _,
};

pub const InterruptBits = packed struct(u32) {
    ext: u1 = 0,
    _0: u2 = 0,
    MIDI_input: u1 = 0,
    dma: u1 = 0,
    scpu: u1 = 0,
    timer_a: u1 = 0,
    timer_b: u1 = 0,
    timer_c: u1 = 0,
    MIDI_output: u1 = 0,
    one_sample_interval: u1 = 0,
    _1: u21 = 0,
};

pub const SB_ADSUSP = packed struct(u32) {
    // DMA Suspend Request (Write Only
    // 0: Continues DMA transfer without going to the suspended state. Or, bit 2 of the SB_ADTSEL register is "0"
    // 1: Suspends and terminates DMA transfer
    smd_suspend_request: u1 = 0,

    _reserved1: u3 = 0,

    // DMA Suspend or DMA Stop (Read Only)
    // 0: DMA transfer is in progress, or bit 2 of the SB_ADTSEL register is "0"
    // 1: DMA transfer has ended, or is stopped due to a suspend
    dma_suspend: u1 = 1,

    // DMA Request Input State (Read Only)
    // 0: The DMA transfer request is high (transfer not possible), or bit 2 of the SB_ADTSEL register is "0"
    // 1: The DMA transfer request is low (transfer possible)
    dma_request_input_state: u1 = 1,

    _: u26 = 0,
};

const PlayStatus = packed struct(u32) {
    env_level: u13 = 0x1FFF,
    env_state: EnvelopeState = .Release,
    loop_end_flag: bool = false,
    _: u16 = 0,
};

const ChannelInfoReq = packed struct(u32) {
    MIDI_output_buffer: u8,
    monitor_select: u6,
    amplitude_or_filter_select: u1,
    _: u17 = 0,
};

pub const TimerControl = packed struct(u32) {
    value: u8 = 0,
    prescale: u3 = 0,
    _: u21 = 0,
};

const AICARegisterStart = 0x00700000;

pub const AICAChannelState = struct {
    playing: bool = false,

    loop_end_flag: bool = false,
    play_position: u16 = 0,

    amp_env_level: u16 = 0x1FFF,
    amp_env_state: EnvelopeState = .Release,

    filter_env_level: u16 = 0x1FFF,
    filter_env_state: EnvelopeState = .Release,

    prev_sample: i32 = 0,
    curr_sample: i32 = 0,

    fractional_play_position: u32 = 0,

    adpcm_state: struct {
        step: i32 = 0x7F,
        step_loopstart: i32 = 0,
        prev: i32 = 0,
        prev_loopstart: i32 = 0,
        loop_init: bool = false,
    } = .{},

    sample_buffer: [2048]i32 = [_]i32{0} ** 2048,
    sample_read_offset: usize = 0,
    sample_write_offset: usize = 0,

    pub fn key_on(self: *AICAChannelState, registers: *const AICAChannel) void {
        if (self.amp_env_state != .Release) return;
        self.playing = true;
        self.loop_end_flag = false;
        self.play_position = 0; // Looping channels also start at 0
        self.amp_env_level = 0x280;
        self.amp_env_state = .Attack;
        self.filter_env_level = @truncate(registers.lpf2);
        self.filter_env_state = .Attack;

        self.adpcm_state = .{};
        self.sample_read_offset = 0;
        self.sample_write_offset = 0;
    }

    pub fn key_off(self: *AICAChannelState) void {
        self.* = .{};
    }

    pub fn compute_effective_rate(registers: *const AICAChannel, rate: u32) u32 {
        var effective_rate: i32 = 2 * @as(i32, @intCast(rate));
        if (registers.amp_env_2.key_rate_scaling < 0xF) {
            effective_rate += 2 * registers.amp_env_2.key_rate_scaling;

            // NOTE: In Neill Corlett's notes, this is also multiplied by 2, but not in Highly_Theoretical sources.
            const oct: i32 = @intCast(@as(i4, @bitCast(registers.sample_pitch_rate.oct)));
            effective_rate += 2 * oct;

            effective_rate += ((registers.sample_pitch_rate.fns >> 9) & 1);
        }
        return @max(0, @min(0x3C, effective_rate));
    }

    // Returns true if the channel should advance in the enveloppe calculation for the current sample.
    // This is briefly described in Neill Corlett's notes, but the pattern between effective rate 0x2 and 0x30
    // is still kinda obscure to me, so this implementation is directly adapted from Highly_Theoretical sources (GPLv3).
    pub fn env_should_advance(effective_rate: u32, sample: u32) bool {
        if (effective_rate <= 0x01) return false;
        if (effective_rate >= 0x30) return (sample & 1) == 0;
        const shift: u5 = @truncate(12 - ((effective_rate - 1) >> 2));
        const pattern: u32 = (effective_rate - 1) & 3;
        if ((sample & ((@as(u32, 1) << shift) - 1)) != 0) return false;
        const bitplace = (sample >> shift) & 0x7;
        return (@as(u32, 0xFFFDDDD5) >> @intCast(pattern * 8 + bitplace)) & 1 != 0;
    }

    pub fn compute_adpcm(self: *AICAChannelState, adpcm_sample: u4) i32 {
        var val = @divTrunc(self.adpcm_state.step * ADPCMDiff[adpcm_sample & 7], 8);
        if (val >= 0x7FFF) val = 0x7FFF;
        val *= @as(i32, 1) - ((adpcm_sample >> 2) & 2);
        val += self.adpcm_state.prev;
        val = std.math.clamp(val, -0x8000, 0x7FFF);
        self.adpcm_state.step = (self.adpcm_state.step + ADPCMScale[adpcm_sample & 7]) >> 8;
        self.adpcm_state.step = std.math.clamp(self.adpcm_state.step, 0x007F, 0x6000);
        self.adpcm_state.prev = val;
        return val;
    }

    const ADPCMScale: [8]i32 = .{ 0xE6, 0xE6, 0xE6, 0xE6, 0x133, 0x199, 0x200, 0x266 };

    const ADPCMDiff: [8]i32 = .{ 1, 3, 5, 7, 9, 11, 13, 15 };
};

// AICA User Manual p. 23
// Effective Rate => Transition Time (ms)
const AEGAttack = [_]f64{
    std.math.inf(f64), std.math.inf(f64), 8100.0, 6900.0, 6000.0, 4800.0, 4000.0, 3400.0, 3000.0, 2400.0, 2000.0, 1700.0, 1500.0, 1200.0, 1000.0, 860.0, 760.0, 600.0, 500.0, 430.0, 380.0, 300.0, 250.0, 220.0, 190.0, 150.0, 130.0, 110.0, 95.0, 76.0, 63.0, 55.0, 47.0, 38.0, 31.0, 27.0, 24.0, 19.0, 15.0, 13.0, 12.0, 9.4, 7.9, 6.8, 6.0, 4.7, 3.8, 3.4, 3.0, 2.4, 2.0, 1.8, 1.6, 1.3, 1.1, 0.93, 0.85, 0.65, 0.53, 0.44, 0.40, 0.35, 0.0, 0.0,
};
// Effective Rate => Transition Time (ms)
const AEGDecay = [_]f64{
    std.math.inf(f64), std.math.inf(f64), 118200.0, 101300.0, 88600.0, 70900.0, 59100.0, 50700.0, 44300.0, 35500.0, 29600.0, 25300.0, 22200.0, 17700.0, 14800.0, 12700.0, 11100.0, 8900.0, 7400.0, 6300.0, 5500.0, 4400.0, 3700.0, 3200.0, 2800.0, 2200.0, 1800.0, 1600.0, 1400.0, 1100.0, 920.0, 790.0, 690.0, 550.0, 460.0, 390.0, 340.0, 270.0, 230.0, 200.0, 170.0, 140.0, 110.0, 98.0, 85.0, 68.0, 57.0, 49.0, 43.0, 34.0, 28.0, 25.0, 22.0, 18.0, 14.0, 12.0, 11.0, 8.5, 7.1, 6.1, 5.4, 4.3, 3.6, 3.1,
};
// Effective Rate => Transition Time (ms)
const FEGTransitionTime = [_]f64{
    std.math.inf(f64), std.math.inf(f64), 118200.0, 101300.0, 88600.0, 70900.0, 59100.0, 50700.0, 44300.0, 35500.0, 29600.0, 25300.0, 22200.0, 17700.0, 14800.0, 12700.0, 11100.0, 8900.0, 7400.0, 6300.0, 5500.0, 4400.0, 3700.0, 3200.0, 2800.0, 2200.0, 1800.0, 1600.0, 1400.0, 1100.0, 920.0, 790.0, 690.0, 550.0, 460.0, 390.0, 340.0, 270.0, 230.0, 200.0, 170.0, 140.0, 110.0, 98.0, 85.0, 68.0, 57.0, 49.0, 43.0, 34.0, 28.0, 25.0, 22.0, 18.0, 14.0, 12.0, 11.0, 8.5, 7.1, 6.1, 5.4, 4.3, 3.6, 3.1,
};

const EnvelopeAttackShift = [_][4]u4{
    .{ 4, 4, 4, 4 },
    .{ 3, 4, 4, 4 },
    .{ 3, 4, 3, 4 },
    .{ 3, 3, 3, 4 },
    .{ 3, 3, 3, 3 },
    .{ 2, 3, 3, 3 },
    .{ 2, 3, 2, 3 },
    .{ 2, 2, 2, 3 },
    .{ 2, 2, 2, 2 },
    .{ 1, 2, 2, 2 },
    .{ 1, 2, 1, 2 },
    .{ 1, 1, 1, 2 },
    .{ 1, 1, 1, 1 },
};
const EnvelopeDecayValue = [_][4]u4{
    .{ 1, 1, 1, 1 },
    .{ 2, 1, 1, 1 },
    .{ 2, 1, 2, 1 },
    .{ 2, 2, 2, 1 },
    .{ 2, 2, 2, 2 },
    .{ 4, 2, 2, 2 },
    .{ 4, 2, 4, 2 },
    .{ 4, 4, 4, 2 },
    .{ 4, 4, 4, 4 },
    .{ 8, 4, 4, 4 },
    .{ 8, 4, 8, 4 },
    .{ 8, 8, 8, 4 },
    .{ 8, 8, 8, 8 },
};

// Memory Map
// SH4 Side             Internal
// 00800000 - 00FFFFFF  00000000 - 007FFFFF  DRAM_AREA*
// 00700000 - 007027FF  00800000 - 008027FF  CHANNEL_DATA
// 00702800 - 00702FFF  00802800 - 00802FFF  COMMON_DATA
// 00703000 - 00707FFF  00803000 - 00807FFF  DSP_DATA
// 00710000 - 00710008        N/A            RTC_REGISTERS

pub const AICA = struct {
    const ARM7CycleRatio = 8;
    const SH4CyclesPerSample = @divTrunc(200_000_000, 44100); // FIXME

    arm7: arm7.ARM7 = undefined,
    enable_arm_jit: bool = false,
    arm_jit: ARM7JIT = undefined,

    regs: []u32 = undefined, // All registers are 32-bit afaik
    wave_memory: []u8 align(4) = undefined,

    channel_states: [64]AICAChannelState = .{.{}} ** 64,

    rtc_write_enabled: bool = false,

    _arm_cycles_counter: i32 = 0,
    _timer_cycles_counter: u32 = 0,
    _timer_counters: [3]u32 = .{0} ** 3,

    _samples_counter: u32 = 0,

    _allocator: std.mem.Allocator = undefined,

    // NOTE: Call setup_arm after!
    pub fn init(allocator: std.mem.Allocator) !AICA {
        var r = AICA{
            .regs = try allocator.alloc(u32, 0x8000 / 4),
            .wave_memory = try allocator.alloc(u8, 0x200000),
            ._allocator = allocator,
        };
        @memset(r.regs, 0);
        @memset(r.wave_memory, 0);
        r.arm7 = arm7.ARM7.init(r.wave_memory, 0x1FFFFF, 0x800000);
        r.arm_jit = try ARM7JIT.init(allocator, r.arm7.memory_address_mask);

        r.get_reg(u32, .MasterVolume).* = 0x10;

        r.get_reg(u32, .SCIEB).* = 0x40;
        r.get_reg(u32, .SCILV0).* = 0x18;
        r.get_reg(u32, .SCILV1).* = 0x50;
        r.get_reg(u32, .SCILV2).* = 0x08;

        return r;
    }

    pub fn setup_arm(self: *@This()) void {
        self.arm7.on_external_read8 = .{
            .callback = @ptrCast(&@This().read8_from_arm),
            .data = self,
        };
        self.arm7.on_external_read32 = .{
            .callback = @ptrCast(&@This().read32_from_arm),
            .data = self,
        };
        self.arm7.on_external_write8 = .{
            .callback = @ptrCast(&@This().write8_from_arm),
            .data = self,
        };
        self.arm7.on_external_write32 = .{
            .callback = @ptrCast(&@This().write32_from_arm),
            .data = self,
        };
    }

    pub fn deinit(self: *AICA) void {
        self.arm_jit.deinit();
        self._allocator.free(self.regs);
        self._allocator.free(self.wave_memory);
    }

    // Read/Write from main CPU

    pub fn read_mem(self: *const AICA, comptime T: type, addr: u32) T {
        std.debug.assert(addr >= 0x00800000 and addr < 0x01000000);
        return @as(*T, @alignCast(@ptrCast(&self.wave_memory[(addr - 0x00800000) % self.wave_memory.len]))).*;
    }

    pub fn write_mem(self: *AICA, comptime T: type, addr: u32, value: T) void {
        std.debug.assert(addr >= 0x00800000 and addr < 0x01000000);
        self.arm_jit.block_cache.signal_write(addr - 0x00800000);

        switch (addr) {
            else => {},
        }
        // FIXME: No idea if this actually wraps around. Dev kit had 8MB of RAM instead of the final 2MB.
        @as(*T, @alignCast(@ptrCast(&self.wave_memory[(addr - 0x00800000) % self.wave_memory.len]))).* = value;
    }

    pub fn get_channel_registers(self: *const AICA, number: u8) *const AICAChannel {
        std.debug.assert(number < 64);
        return @alignCast(@ptrCast(&self.regs[0x80 / 4 * @as(u32, number)]));
    }

    fn get_reg(self: *const AICA, comptime T: type, reg: AICARegister) *T {
        return @as(*T, @alignCast(@ptrCast(&self.regs[@intFromEnum(reg) / 4])));
    }

    pub fn debug_read_reg(self: *const AICA, comptime T: type, reg: AICARegister) T {
        return @as(*T, @alignCast(@ptrCast(&self.regs[@intFromEnum(reg) / 4]))).*;
    }

    pub fn read_register(self: *AICA, comptime T: type, addr: u32) T {
        const local_addr = addr & 0x0000FFFF;

        //aica_log.debug("Read AICA register at 0x{X:0>8} (0x{X:0>8}) = 0x{X:0>8}", .{ addr, local_addr, self.regs[local_addr / 4] });
        if (local_addr < 0x2000) {}

        switch (local_addr) {
            @intFromEnum(AICARegister.MasterVolume) => return 0x10,
            @intFromEnum(AICARegister.MasterVolume) + 1 => return 0x00,
            @intFromEnum(AICARegister.PlayStatus)...@intFromEnum(AICARegister.PlayStatus) + 1 => {
                const req = self.get_reg(ChannelInfoReq, .ChannelInfoReq);
                var chan = &self.channel_states[req.monitor_select];
                const status: PlayStatus = .{
                    .env_level = @truncate(if (req.amplitude_or_filter_select == 0) chan.amp_env_level else chan.filter_env_level),
                    .env_state = if (req.amplitude_or_filter_select == 0) chan.amp_env_state else chan.filter_env_state,
                    .loop_end_flag = chan.loop_end_flag,
                };
                chan.loop_end_flag = false; // Cleared on read.
                if (T == u8 and local_addr & 1 == 1) {
                    return @truncate(@as(u32, @bitCast(status)) >> 8);
                } else {
                    return @truncate(@as(u32, @bitCast(status)));
                }
            },
            @intFromEnum(AICARegister.PlayPosition)...@intFromEnum(AICARegister.PlayPosition) + 1 => {
                const req = self.get_reg(ChannelInfoReq, .ChannelInfoReq);
                const pos = self.channel_states[req.monitor_select].play_position;
                if (T == u8 and local_addr & 1 == 1) {
                    return @truncate(pos >> 8);
                } else {
                    return if (T == u8) @truncate(pos) else pos;
                }
            },
            else => {},
        }

        return switch (T) {
            u8 => @as([*]const u8, @ptrCast(&self.regs[0]))[local_addr],
            u32 => self.regs[local_addr / 4],
            else => @compileError("Invalid value type"),
        };
    }

    pub fn write_register(self: *AICA, comptime T: type, addr: u32, value: T) void {
        const local_addr = addr & 0x0000FFFF;
        aica_log.debug("Write to AICA Register at 0x{X:0>8} = 0x{X:0>8}", .{ addr, value });

        // Channel registers
        if (local_addr < 0x2000) {
            switch (local_addr & 0x7F) {
                0x00 => { // Play control
                    switch (T) {
                        u8 => @as([*]u8, @ptrCast(self.regs.ptr))[local_addr] = value,
                        u32 => {
                            self.regs[local_addr / 4] = value & PlayControl.Mask;

                            const val: PlayControl = @bitCast(value);
                            aica_log.info("Play control: 0x{X:0>8} = 0x{X:0>8}\n  {any}", .{ addr, value, val });
                            if (val.key_on_execute) self.key_on_execute();
                        },
                        else => @compileError("Invalid value type"),
                    }
                    return;
                },
                0x01 => { // Play control - High byte
                    switch (T) {
                        u8 => {
                            @as([*]u8, @ptrCast(self.regs.ptr))[local_addr] = value;
                            const val: PlayControl = @bitCast(@as(u32, value) << 8);
                            if (val.key_on_execute) self.key_on_execute();
                        },
                        else => unreachable,
                    }
                },
                else => {},
            }
        } else {
            std.debug.assert(local_addr % 4 == 0 or T == u8);
            const reg_addr = local_addr - (local_addr % 4);
            const low_byte = T == u32 or (T == u8 and local_addr % 4 == 0);
            switch (@as(AICARegister, @enumFromInt(reg_addr))) {
                .MasterVolume => {
                    aica_log.warn(termcolor.yellow("Write to Master Volume = 0x{X:0>8}"), .{value});
                },
                .DDIR_DEXE => { // DMA transfer direction / DMA transfer start
                    if (T == u8)
                        @as([*]u8, @ptrCast(&self.regs))[local_addr] = value & 0xFC;
                    if (T == u32)
                        self.regs[local_addr / 4] = value & 0xFFFFFFFC;
                    if (low_byte and value & 1 == 1) {
                        aica_log.info(termcolor.green("DMA Start"), .{});
                        @panic("TODO AICA DMA");
                    }
                    return;
                },
                .SCIEB => {
                    aica_log.info("Write to AICA Register SCIEB = {any}", .{if (T == u32) @as(InterruptBits, @bitCast(value)) else value});
                    if (low_byte) {
                        self.get_reg(u32, .SCIEB).* = value & @as(T, @truncate(0x7F9));
                    } else {
                        self.get_reg(u32, .SCIEB).* = ((@as(u32, value) << 8) & 0x7FF) | (self.get_reg(u32, .SCIEB).* & 0xFF);
                    }
                    self.check_interrupts();
                    return;
                },
                .SCIPD => {
                    aica_log.info("Write to AICA Register SCIPD = {any}", .{if (T == u32) @as(InterruptBits, @bitCast(value)) else value});
                    if (low_byte) {
                        self.get_reg(u32, .SCIPD).* |= (value & (@as(u32, 1) << 5)); // Set scpu interrupt
                        self.check_interrupts();
                    }
                    return;
                },
                .SCIRE => { // Clear interrupt(s)
                    aica_log.info("Write to AICA Register SCIRE = {any}", .{if (T == u32) @as(InterruptBits, @bitCast(value)) else value});
                    if (low_byte) {
                        self.get_reg(u32, .SCIPD).* &= ~value;
                    } else {
                        self.get_reg(u32, .SCIPD).* &= ~(@as(u32, value) << 8);
                    }
                    return;
                },
                .MCIPD => {
                    if (T == u32) {
                        aica_log.warn("Write to AICA Register MCIPD = 0x{X:0>8}", .{value});
                        if (@as(InterruptBits, @bitCast(value)).scpu == 1 and self.get_reg(InterruptBits, .MCIEB).*.scpu == 1) {
                            aica_log.warn(termcolor.green("SCPU interrupt"), .{});
                            // TODO!
                        }
                    } else aica_log.err("Write8 to AICA Register MCIPD = 0x{X:0>8}", .{value});
                },
                .MCIRE => { // Clear interrupt(s)
                    if (low_byte) {
                        self.get_reg(u32, .MCIPD).* &= ~value;
                    } else {
                        self.get_reg(u32, .MCIPD).* &= ~(@as(u32, value) << 8);
                    }
                },
                .ARMRST => {
                    if (low_byte) {
                        aica_log.info("ARM reset : {d}", .{value & 1});

                        // Not sure if actually necessary, but some homebrew could let this set when interrupted
                        // during a FIQ (I guess at least) and then fail to reset it.
                        self.get_reg(u32, .INTRequest).* = 0;

                        @memset(&self.arm7.r, 0);
                        @memset(&self.arm7.r_fiq_8_12, 0);
                        @memset(&self.arm7.r_usr, 0);
                        @memset(&self.arm7.r_fiq, 0);
                        @memset(&self.arm7.r_svc, 0);
                        @memset(&self.arm7.r_irq, 0);
                        @memset(&self.arm7.r_abt, 0);
                        @memset(&self.arm7.r_und, 0);
                        self._arm_cycles_counter = 0;
                        self.arm7.reset(value & 1 == 0);
                        if (value & 1 == 0 and self.wave_memory[0] == 0x00000000) {
                            aica_log.err(termcolor.red("  No code uploaded to ARM7, ignoring reset. FIXME: This is a hack."), .{});
                            self.arm7.running = false;
                        }
                    }
                },
                .INTRequest => {
                    aica_log.warn(termcolor.yellow("Write to AICA Register INTRequest = {X:0>8}"), .{value});
                },
                .INTClear => {
                    aica_log.info("Write to AICA Register INTClear = {X:0>8}", .{value});
                    if (low_byte and value & 1 == 1) {
                        self.get_reg(u32, .INTRequest).* = 0;
                        self.check_interrupts();
                    }
                    return;
                },
                else => {},
            }
        }

        switch (T) {
            u8 => @as([*]u8, @ptrCast(self.regs.ptr))[local_addr] = value,
            u32 => self.regs[local_addr / 4] = value & 0xFFFF, // Only half of each u32 register is actually used.
            else => @compileError("Invalid value type"),
        }
    }

    pub fn read_rtc_register(self: *const AICA, addr: u32) u32 {
        _ = self;
        std.debug.assert(addr >= 0x00710000);
        const utc = std.time.timestamp();
        const dc_timestamp: u32 = @intCast(utc + (20 * 365 + 5) * 24 * 60 * 60); // Dreamcast epoch is January 1, 1950 00:00
        // TODO: Handle timezone.
        return switch (addr - 0x00710000) {
            0x00 => (dc_timestamp >> 16) & 0x0000FFFFF,
            0x04 => dc_timestamp & 0x0000FFFFF,
            else => @panic("Read to unimplemented RTC register."),
        };
    }

    pub fn write_rtc_register(self: *AICA, addr: u32, value: u32) void {
        // RTC[31:0] is normally write protected, but can be written when a "1" is written to the EN bit.
        // Furthermore, when RTC[15:0] is written, the counter below one second is cleared. When RTC[31:16] is
        // written, write protection is enabled again.
        switch (addr - 0x00710000) {
            0x00 => {},
            0x04 => {
                self.rtc_write_enabled = false;
            },
            0x08 => {
                self.rtc_write_enabled = value == 1;
            },
            else => {
                @panic("Write to unimplemented RTC register.");
            },
        }
    }

    fn arm_debug_dump(self: *const @This()) void {
        var s = @constCast(self);
        std.debug.print("PC: {X:0>8}\n", .{s.arm7.pc() - 8});
        std.debug.print("LR: {X:0>8}\n", .{s.arm7.lr()});
        std.debug.print("SP: {X:0>8}\n", .{s.arm7.sp()});

        for (0..8) |i| {
            std.debug.print("R{d}: {X:0>8}   R{d}: {X:0>8}\n", .{ i, s.arm7.r[i], i + 8, s.arm7.r[i + 8] });
        }

        for (0..16) |i| {
            const o: u32 = @truncate(8 + 4 * 16 - 4 * i);
            std.debug.print("   [{X:0>8}] {X:0>8} {s}\n", .{ s.arm7.pc() - o, self.arm7.read(u32, s.arm7.pc() - o), arm7.ARM7.disassemble(self.arm7.read(u32, s.arm7.pc() - o)) });
        }
        std.debug.print(" > [{X:0>8}] {X:0>8} {s}\n", .{ s.arm7.pc() - 8, s.arm7.read(u32, s.arm7.pc() - 8), arm7.ARM7.disassemble(self.arm7.read(u32, s.arm7.pc() - 8)) });
        std.debug.print("   [{X:0>8}] {X:0>8} {s}\n", .{ s.arm7.pc() - 4, s.arm7.read(u32, s.arm7.pc() - 4), arm7.ARM7.disassemble(self.arm7.read(u32, s.arm7.pc() - 4)) });
        std.debug.print("   [{X:0>8}] {X:0>8} {s}\n", .{ s.arm7.pc() - 0, s.arm7.read(u32, s.arm7.pc() - 0), arm7.ARM7.disassemble(self.arm7.read(u32, s.arm7.pc() - 0)) });
    }

    pub fn read8_from_arm(self: *AICA, addr: u32) u8 {
        if (!(addr >= 0x00800000 and addr < 0x00808000)) {
            aica_log.err(termcolor.red("AICA read8 from ARM out of bounds address: 0x{X:0>8}"), .{addr});
            self.arm_debug_dump();
            @panic("AICA read8 from ARM out of bounds address");
        }
        return self.read_register(u8, addr);
    }

    pub fn read32_from_arm(self: *AICA, addr: u32) u32 {
        if (!(addr >= 0x00800000 and addr < 0x00808000)) {
            aica_log.err(termcolor.red("AICA read32 from ARM out of bounds address: 0x{X:0>8}"), .{addr});
            self.arm_debug_dump();
            @panic("AICA read32 from ARM out of bounds address");
        }
        return self.read_register(u32, addr);
    }

    pub fn write8_from_arm(self: *AICA, addr: u32, value: u8) void {
        if (!(addr >= 0x00800000 and addr < 0x00808000)) {
            std.debug.print("AICA write8 from ARM: 0x{X:0>8} = 0x{X:0>8}\n", .{ addr, value });
            self.arm_debug_dump();
            @panic("AICA write8 from ARM out of bounds address");
        }
        self.write_register(u8, addr, value);
    }
    pub fn write32_from_arm(self: *AICA, addr: u32, value: u32) void {
        if (!(addr >= 0x00800000 and addr < 0x00808000)) {
            std.debug.print("AICA write32 from ARM: 0x{X:0>8} = 0x{X:0>8}\n", .{ addr, value });
            self.arm_debug_dump();
            @panic("AICA write32 from ARM out of bounds address");
        }
        self.write_register(u32, addr, value);
    }

    fn check_sh4_interrupt(self: *AICA, dc: *Dreamcast) void {
        if ((self.get_reg(u32, .MCIPD).* & self.get_reg(u32, .MCIEB).*) != 0) {
            dc.raise_external_interrupt(.{ .AICA = 1 });
        }
    }

    fn check_interrupts(self: *AICA) void {
        if (self.get_reg(u32, .INTRequest).* != 0) return;

        const enabled = self.get_reg(u32, .SCIEB).*;
        const pending = self.get_reg(u32, .SCIPD).* & enabled & 0x7F9;

        if (pending != 0) {
            for (0..11) |i| {
                if (pending & (@as(u32, 1) << @intCast(i)) != 0) {
                    const bit = @min(7, i); // Interrupts higher than 7 share the same INTReq number.
                    self.get_reg(u32, .INTRequest).* =
                        (((@as(u32, self.get_reg(u32, .SCILV0).*) >> bit) & 1) << 0) |
                        (((@as(u32, self.get_reg(u32, .SCILV1).*) >> bit) & 1) << 1) |
                        (((@as(u32, self.get_reg(u32, .SCILV2).*) >> bit) & 1) << 2);
                    break;
                }
            }
            self.arm7.fast_interrupt_request();
        }
    }

    // Key on execute: Execute a key on for every channel this the KeyOn bit enabled.
    fn key_on_execute(self: *AICA) void {
        aica_log.info(termcolor.green("Key On Execute"), .{});
        for (0..64) |i| {
            const regs = self.get_channel_registers(@intCast(i));
            if (regs.play_control.key_on_bit) {
                self.channel_states[i].key_on(regs);
            } else {
                self.channel_states[i].key_off();
            }
        }
    }

    pub fn update(self: *AICA, dc: *Dreamcast, sh4_cycles: u32) !void {
        self._timer_cycles_counter += @intCast(sh4_cycles);
        const sample_count = @divTrunc(self._timer_cycles_counter, SH4CyclesPerSample);

        if (sample_count > 0) {
            self.get_reg(u32, .SCIPD).* |= @as(u32, 1) << @bitOffsetOf(InterruptBits, "one_sample_interval");

            // Avoid overflow in channel update. FIXME in another way?
            if (self._samples_counter +% sample_count < self._samples_counter) {
                self._samples_counter &= 0xFFFF;
            }

            for (0..64) |i| {
                self.update_channel(@intCast(i), sample_count);
            }
            self._samples_counter +%= sample_count;

            self._timer_cycles_counter = self._timer_cycles_counter % SH4CyclesPerSample;
            const timer_registers = [_]AICARegister{ .TACTL_TIMA, .TBCTL_TIMB, .TCCTL_TIMC };
            for (0..3) |i| {
                var timer = self.get_reg(TimerControl, timer_registers[i]);
                self._timer_counters[i] += sample_count;
                const scaled = @as(u32, 1) << timer.prescale;
                if (self._timer_counters[i] >= scaled) {
                    self._timer_counters[i] -= scaled;
                    if (timer.value == 0xFF) {
                        const mask: u32 = @as(u32, 1) << @intCast(6 + i);

                        self.get_reg(u32, .SCIPD).* |= mask;
                        self.get_reg(u32, .MCIPD).* |= mask;

                        timer.value = 0;
                    } else timer.value += 1;
                }
            }

            self.check_sh4_interrupt(dc);
            self.check_interrupts();
        }

        if (self.arm7.running) {
            self._arm_cycles_counter += @intCast(sh4_cycles);
            if (self.enable_arm_jit) {
                if (self._arm_cycles_counter >= ARM7CycleRatio)
                    self._arm_cycles_counter -= ARM7CycleRatio * try self.arm_jit.run_for(&self.arm7, @divFloor(self._arm_cycles_counter, ARM7CycleRatio));
            } else {
                // FIXME: We're not actually counting ARM7 cycles here (unless all instructions are 1 cycle :^)).
                while (self._arm_cycles_counter >= ARM7CycleRatio) {
                    self._arm_cycles_counter -= ARM7CycleRatio;
                    // aica_log.info("arm7: ({s}) [{X:0>4}] {X:0>8} - {s: <22} - SP:{X:0>8} - LR:{X:0>8}", .{ @tagName(self.arm7.cpsr.m), @max(4, self.arm7.pc() & self.arm7.memory_address_mask) - 4, self.arm7.instruction_pipeline[0], arm7.ARM7.disassemble(self.arm7.instruction_pipeline[0]), self.arm7.sp(), self.arm7.lr() });
                    arm7.interpreter.tick(&self.arm7);
                    if (self.arm7.pc() >= 0x00200000) {
                        aica_log.warn("arm7: PC out of bounds: {X:0>8}, stopping.", .{self.arm7.pc()});
                        self.arm7.running = false;
                        break;
                    }
                }
            }
        }
    }

    pub fn update_channel(self: *@This(), channel_number: u8, samples: u32) void {
        var state = &self.channel_states[channel_number];
        if (!state.playing) return;

        const registers = self.get_channel_registers(channel_number);

        // FIXME: Too slow now?
        var base_play_position_inc: u32 = registers.sample_pitch_rate.fns ^ 0x400;
        if ((registers.sample_pitch_rate.oct & 8) == 0) {
            base_play_position_inc <<= registers.sample_pitch_rate.oct;
        } else {
            base_play_position_inc >>= @as(u5, 16) - registers.sample_pitch_rate.oct;
        }
        if (registers.play_control.sample_format == .ADPCM and registers.sample_pitch_rate.oct >= 0xA)
            base_play_position_inc <<= 1;

        for (self._samples_counter..self._samples_counter + samples) |i| {
            if (state.amp_env_level > 0x3BF) {
                state.amp_env_level = 0x1FFF;
                state.loop_end_flag = true;
                break;
            }

            if (state.play_position == registers.loop_start) {
                if (registers.amp_env_2.link == 1 and state.amp_env_state == .Attack)
                    state.amp_env_state = .Decay;
                if (!state.adpcm_state.loop_init) {
                    state.adpcm_state.step_loopstart = state.adpcm_state.step;
                    state.adpcm_state.prev_loopstart = state.adpcm_state.prev;
                    state.adpcm_state.loop_init = true;
                }
            }

            // Advance amplitude envelope
            {
                const effective_rate = AICAChannelState.compute_effective_rate(registers, switch (state.amp_env_state) {
                    .Attack => registers.amp_env_1.attack_rate,
                    .Decay => registers.amp_env_1.decay_rate,
                    .Sustain => registers.amp_env_1.sustain_rate,
                    .Release => registers.amp_env_2.release_rate,
                });
                if (AICAChannelState.env_should_advance(effective_rate, @intCast(i))) {
                    const idx = if (effective_rate < 0x30) 0 else effective_rate - 0x30;
                    switch (state.amp_env_state) {
                        .Attack => {
                            const diff = ((state.amp_env_level >> EnvelopeAttackShift[idx][i % 4]) + 1);
                            if (state.amp_env_level < diff) {
                                state.amp_env_level = 0;
                                state.amp_env_state = .Decay;
                            } else {
                                state.amp_env_level -= diff;
                            }
                        },
                        .Decay => {
                            state.amp_env_level += EnvelopeDecayValue[idx][i % 4];
                            if ((state.amp_env_level >> 5) >= registers.amp_env_2.decay_level) {
                                state.amp_env_state = .Sustain;
                            }
                        },
                        .Sustain, .Release => {
                            state.amp_env_level += EnvelopeDecayValue[idx][i % 4];
                            // FIXME: Not sure about this at all
                            if (state.amp_env_level >= 0x3FF) {
                                state.key_off();
                            }
                        },
                    }
                }
            }
            // Advance low pass filter envelope
            {
                const effective_rate = AICAChannelState.compute_effective_rate(registers, switch (state.filter_env_state) {
                    .Attack => registers.lpf2 & 0x1FFF,
                    .Decay => registers.lpf3 & 0x1FFF,
                    .Sustain => registers.lpf4 & 0x1FFF,
                    .Release => registers.lpf5 & 0x1FFF,
                });
                if (AICAChannelState.env_should_advance(effective_rate, @intCast(i))) {
                    const idx = if (effective_rate < 0x30) 0 else effective_rate - 0x30;
                    const decay = EnvelopeDecayValue[idx][i % 4];
                    const target: u16 = @truncate(switch (state.filter_env_state) {
                        .Attack => registers.lpf3 & 0x1FFF,
                        .Decay => registers.lpf4 & 0x1FFF,
                        .Sustain => registers.lpf5 & 0x1FFF,
                        else => 0,
                    });
                    if (state.filter_env_level < target) {
                        state.filter_env_level += decay;
                        state.filter_env_level = @min(state.filter_env_level, target);
                    } else if (state.filter_env_level > target) {
                        if (state.filter_env_level > decay) {
                            state.filter_env_level -= decay;
                        } else {
                            state.filter_env_level = 0;
                        }
                        state.filter_env_level = @max(state.filter_env_level, target);
                    } else {
                        switch (state.filter_env_state) {
                            .Attack => state.filter_env_state = .Decay,
                            .Decay => state.filter_env_state = .Sustain,
                            .Sustain => state.filter_env_state = .Release,
                            .Release => {},
                        }
                    }
                }
            }

            // Interpolate samples
            const f: i32 = @intCast((state.fractional_play_position >> 4) & 0x3FFF);
            const sample = @divTrunc((state.curr_sample * f) + (state.prev_sample * (0x4000 - f)), 0x4000);
            state.sample_buffer[state.sample_write_offset] = sample;
            state.sample_write_offset = (state.sample_write_offset + 1) % state.sample_buffer.len;

            state.fractional_play_position += base_play_position_inc;
            while (state.fractional_play_position >= 0x40000) {
                state.fractional_play_position -= 0x40000;

                state.prev_sample = state.curr_sample;
                const sample_ram = self.wave_memory[registers.sample_address()..];
                state.curr_sample = switch (registers.play_control.sample_format) {
                    .i16 => @as([*]const i16, @alignCast(@ptrCast(sample_ram.ptr)))[state.play_position],
                    .i8 => @as(i32, @intCast(@as([*]const i8, @alignCast(@ptrCast(sample_ram.ptr)))[state.play_position])) << 8,
                    .ADPCM => adpcm: {
                        // 4 bits per sample
                        var s: u8 = @intCast(@as([*]const u8, @alignCast(@ptrCast(sample_ram.ptr)))[state.play_position >> 1]);
                        if (state.play_position & 1 == 1)
                            s >>= 4;
                        break :adpcm state.compute_adpcm(@truncate(s));
                    },
                    else => 0,
                };

                state.play_position +%= 1;

                if (state.play_position == registers.loop_end & 0xFFFF) {
                    state.loop_end_flag = true;
                    if (registers.play_control.sample_loop) {
                        state.play_position = @truncate(registers.loop_start);
                        state.adpcm_state.step = state.adpcm_state.step_loopstart;
                        state.adpcm_state.prev = state.adpcm_state.prev_loopstart;
                    } else {
                        state.play_position = 0;
                        state.playing = false;
                        return;
                    }
                }
            }
        }
    }

    pub fn start_dma(self: *AICA, dc: *Dreamcast) void {
        const enabled = dc.read_hw_register(u32, .SB_ADEN);
        if (enabled == 0) return;

        var aica_addr = dc.read_hw_register(u32, .SB_ADSTAG);
        const root_bus_addr = dc.read_hw_register(u32, .SB_ADSTAR);
        const len_reg = dc.read_hw_register(u32, .SB_ADLEN);
        const len_in_bytes = len_reg & 0x7FFFFFFF;
        const direction = dc.read_hw_register(u32, .SB_ADDIR);
        aica_log.debug(" AICA G2-DMA Start!", .{});
        aica_log.debug("   AICA Address: 0x{X:0>8}", .{aica_addr});
        aica_log.debug("   Root Bus Address: 0x{X:0>8}", .{root_bus_addr});
        aica_log.debug("   Length: 0x{X:0>8} (0x{X:0>8})", .{ len_reg, len_in_bytes });
        aica_log.debug("   Direction: 0x{X:0>8}", .{direction});
        aica_log.debug("   Trigger Select: 0x{X:0>8}", .{dc.read_hw_register(u32, .SB_ADTSEL)});
        aica_log.debug("   Enable: 0x{X:0>8}", .{enabled});

        // 0x02800000-0x02A00000 is a mirror of 0x02000000-0x02A00000, I think.
        // It comes up in Loop Checker G2 DMA test.
        if (aica_addr >= 0x02800000 and aica_addr < 0x02A00000) {
            aica_addr -= 0x02000000;
        }

        // NOTE: Wrong start adresses should raise exception, but we don't emulate them, yet.
        if (aica_addr < 0x00800000 or aica_addr >= 0x00A00000) {
            aica_log.err(termcolor.red("AICA DMA out of bounds! AICA Address: 0x{X:0>8}. Canceled."), .{aica_addr});
            return;
        }
        if (root_bus_addr < 0x0C000000 or root_bus_addr >= 0x0D000000) {
            aica_log.err(termcolor.red("AICA DMA out of bounds! Root Bus Address: 0x{X:0>8}. Canceled."), .{root_bus_addr});
            return;
        }

        // FIXME: I have no idea how to correctly handle this error case.
        if (aica_addr + len_in_bytes >= 0x00A00000) {
            aica_log.err(termcolor.red("AICA DMA out of bounds! AICA Address: 0x{X:0>8}, length: 0x{X:0>8} => 0x{X:0>8}"), .{ aica_addr, len_in_bytes, aica_addr + len_in_bytes });
            return;
        }
        if (root_bus_addr + len_in_bytes >= 0x0D000000) {
            aica_log.err(termcolor.red("AICA DMA out of bounds! Root Bus Address: 0x{X:0>8}, length: 0x{X:0>8} => 0x{X:0>8}"), .{ root_bus_addr, len_in_bytes, root_bus_addr + len_in_bytes });
            return;
        }

        const physical_root_addr = dc.cpu._get_memory(root_bus_addr);
        const physical_aica_addr = &self.wave_memory[aica_addr - 0x00800000];

        const len_in_u32 = len_in_bytes / 4;

        // TODO: This might raise some exceptions, if the addresses are wrong.

        if (direction == 0) {
            // DMA transfer from the Root Bus to a G2 device
            const src = physical_root_addr;
            const dst = physical_aica_addr;
            @memcpy(@as([*]u32, @ptrCast(@alignCast(dst)))[0..len_in_u32], @as([*]u32, @ptrCast(@alignCast(src)))[0..len_in_u32]);
        } else {
            // DMA transfer from a G2 device to the Root Bus
            const src = physical_aica_addr;
            const dst = physical_root_addr;
            @memcpy(@as([*]u32, @ptrCast(@alignCast(dst)))[0..len_in_u32], @as([*]u32, @ptrCast(@alignCast(src)))[0..len_in_u32]);
        }

        // Signals the DMA is in progress
        dc.hw_register(u32, .SB_ADST).* = 1;
        dc.hw_register(u32, .SB_ADSUSP).* &= 0b101111; // Clear "DMA Suspend or DMA Stop"

        // Schedule the end of the transfer interrupt
        dc.schedule_event(.{ .function = @ptrCast(&end_dma), .context = self }, 10 * len_in_bytes); // FIXME: Compute the actual cycle count.
    }

    fn end_dma(_: *AICA, dc: *Dreamcast) void {
        const suspended = dc.read_hw_register(u32, .SB_ADSUSP);
        if ((suspended & 1) == 1) {
            // The DMA is suspended, wait.
            // FIXME: This is probably not how it's supposed to work.
            return;
        }

        const len_reg = dc.read_hw_register(u32, .SB_ADLEN);
        const dma_end = (len_reg & 0x80000000) != 0; // DMA Transfer End//Restart
        const len = len_reg & 0x7FFFFFFF;
        // When a transfer ends, the DMA enable register is set to "0".
        if (dma_end)
            dc.hw_register(u32, .SB_ADEN).* = 0;

        dc.hw_register(u32, .SB_ADSTAR).* += len;
        dc.hw_register(u32, .SB_ADSTAG).* += len;
        dc.hw_register(u32, .SB_ADLEN).* = 0;
        dc.hw_register(u32, .SB_ADST).* = 0;

        // Set "DMA Suspend or DMA Stop" bit in SB_ADSUSP
        dc.hw_register(u32, .SB_ADSUSP).* |= 0b010000;

        dc.raise_normal_interrupt(.{ .EoD_AICA = 1 });
        dc.raise_external_interrupt(.{ .AICA = 1 });
    }
};
