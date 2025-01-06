const std = @import("std");
const termcolor = @import("termcolor");

const aica_log = std.log.scoped(.aica);

const arm7 = @import("arm7");
const ARM7JIT = @import("jit/arm_jit.zig").ARM7JIT;

const DreamcastModule = @import("dreamcast.zig");
const Dreamcast = DreamcastModule.Dreamcast;
const HardwareRegisters = @import("hardware_registers.zig");

// Yamaha AICA Audio Chip
// Most notable source outside of official docs: Neill Corlett's Yamaha AICA notes

const SampleFormat = enum(u2) {
    i16 = 0, // 16-bit signed little-endian
    i8 = 1, // 8-bit signed
    ADPCM = 2, // 4-bit Yamaha ADPCM
    ADPCMStream = 3,
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

    const Mask: u32 = 0x0000_47FF; // NOTE: key_on_execute is not saved and will always be read as 0.
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

pub const Waveform = enum(u2) {
    Sawtooth = 0,
    Square = 1,
    Triangle = 2,
    Noise = 3,
};

pub const LFOControl = packed struct(u32) {
    amplitude_modulation_depth: u3, // (ALFOS)
    amplitude_modulation_waveform: Waveform, // (ALFOWS)
    pitch_modulation_depth: u3, // (PLFOS)
    pitch_modulation_waveform: Waveform, // (PLFOWS)
    frequency: u5, // (LFOF)
    reset: bool, // (LFORE) 1=on, 0=off If set, the LFO phase is reset at the start of EACH SAMPLE LOOP.
    _: u16,
};

pub const DSPChannelSend = packed struct(u32) {
    channel: u4, // DSP send channel, 0x0-0xF
    // "This affects which DSP MIXS register will receive this channel's output.
    //  I have verified that bit 19 of MIXS corresponds to bit 15 of a
    //  single 16-bit sample played at the maximum possible volume."
    level: u4, // DSP send level, 0x0-0xF
    // Scales the output of this channel to one of the effect send buses.
    // 0xF is full volume (no attenuation), every level beneath that adds
    // 3dB, and 0x0 means no output.
    _0: u8 = 0,
    _1: u16,
};

pub const DirectPanVolSend = packed struct(u32) {
    pan: u5, // Direct send pan (DIPAN)
    // 0x00 or 0x10 is center, 0x0F is full right, 0x1F is full left
    // 0x00-0x0F: each step beyond 0x00 attenuates the left side by 3db
    //            (right side remains at same volume)
    // 0x10-0x1F: each step beyond 0x10 attenuates the right side by 3db
    //            (left side remains at same volume)
    _0: u3,
    volume: u4, // Direct send volume (DISDL) - "Affects how much of this channel is being sent directly to the dry output.  0xF is full volume (no attenuation), every level beneath that adds 3dB, and 0x0 means no output.""
    _1: u4,
    _2: u16,
};

pub const EnvSettings = packed struct(u32) {
    q: u5, // filter resonance value Q = (0.75*value)-3, from -3 to 20.25 db
    lpoff: bool, // 1 = turn off lowpass filter.
    voff: bool, // if this bit is set to 1, the constant attenuation, envelope, and LFO volumes will not take effect. however, the note will still end when the envelope level reaches zero in the release state.
    _: u1, // unknown [SAVED]
    constant_attenuation: u8, // (TL) this value *4 seems to be added to the envelope attenuation (as in, 0x00-0xFF here corresponds to 0x000-0x3FF when referring to the envelope attenuation)
    _r: u16,
};

pub const LPFRates1 = packed struct(u32) {
    lpf_decay_rate: u5,
    _0: u3,
    lpf_attack_rate: u5,
    _1: u3,
    _2: u16,

    pub const Mask: u32 = 0x1F1F;
};

pub const LPFRates2 = packed struct(u32) {
    lpf_release_rate: u5,
    _0: u3,
    lpf_sustain_rate: u5,
    _1: u3,
    _2: u16,

    pub const Mask: u32 = 0x1F1F;
};

pub const DSPOutputMixer = packed struct(u32) {
    efpan: u5, // Effect output pan. This works the same as the direct output pan register.
    _0: u3,
    efsdl: u4, // Effect output level. 0xF is no attenuation (full volume), and each value below that increases the attenuation by 3dB.
    _1: u4,
    _2: u16,
};

// NOTE: Only the lower 16bits of each registers are actually used.
pub const AICAChannel = packed struct(u576) {
    play_control: PlayControl, //             0x00
    sample_address_low: u32, //               0x04 Lowest 16 bits of sample start address (SA) (in bytes)
    loop_start: u32, //                       0x08
    loop_end: u32, //                         0x0C
    amp_env_1: AmpEnv1, //                    0x10
    amp_env_2: AmpEnv2, //                    0x14
    sample_pitch_rate: SamplePitchRate, //    0x18
    lfo_control: LFOControl, //               0x1C
    dps_channel_send: DSPChannelSend, //      0x20
    direct_pan_vol_send: DirectPanVolSend, // 0x24
    env_settings: EnvSettings, //             0x28 Also Named 'LPF1Volume'
    flv0: u32, //                 Bits 0-12   0x2C Cutoff frequency at the time of attack start.                  Also named lpf2
    flv1: u32, //                 Bits 0-12   0x30 Cutoff frequency at the time of attack end (dacay start time). Also named lpf3
    flv2: u32, //                 Bits 0-12   0x34 Cutoff frequency at the time of decay end (sustain start time) Also named lpf4
    flv3: u32, //                 Bits 0-12   0x38 Cutoff frequency at the time of KOFF.                          Also named lpf5
    flv4: u32, //                 Bits 0-12   0x3C Cutoff frequency after release.                                Also named lpf6
    lpf_rates_1: LPFRates1, //                0x40                                                                Also named lpf7
    lpf_rates_2: LPFRates2, //                0x44                                                                Also named lpf8

    pub fn sample_address(self: *const AICAChannel) u32 {
        return (@as(u32, self.play_control.start_address) << 16) | (self.sample_address_low & 0xFFFF);
    }
};

pub const MasterVolume = packed struct(u32) {
    volume: u4 = 0x0,
    version: u4 = 0x1, // version information for the AICA chip
    dac_18b: bool, // DAC18B: 0: Makes the digital output a 16-bit DAC interface. 1: Makes the digital output an 18-bit DAC interface
    mem_8mb: bool, // MEM8MB: This register specifies the size of the memory that is used for wave memory. 0:16Mbit_SDRAM, 1:64Mbit_SDRAM
    _: u5,
    mono: bool, // 0: Enables the panpot information, 1: Disables the panpot information.
    // (Note) If the panpot information has been disabled, a sound that is on only one channel doubles in volume, so it is necessary to lower MVOL
    _0: u16,
};

// Address of AICA registers. Add 0x00700000 for access from SH4 and 0x00800000 for access from ARM7
pub const AICARegister = enum(u32) {
    DSPOutputMixer0 = 0x00002000,
    // ...
    DSPOutputMixer15 = 0x0000203C,

    CDDAOutputLeft = 0x00002040,
    CDDAOutputRight = 0x00002044,

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

fn bitfield_format(self: anytype, writer: anytype) !void {
    var first = true;
    try writer.writeAll("(");
    inline for (@typeInfo(InterruptBits).@"struct".fields) |field| {
        if (@field(self, field.name) == 1) {
            if (!first) try writer.writeAll(" | ");
            try writer.writeAll(field.name);
            first = false;
        }
    }
    try writer.writeAll(")");
}

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

    pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
        try bitfield_format(self, writer);
    }
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

const MIDIInput = packed struct(u32) {
    mibuf: u8, // MIDI Input Buffer (MIBUF) - Data coming in from the MIDI port in a 4-byte FIFO.
    miemp: u1, // MIDI Input FIFO is empty
    miful: u1, // MIDI Input FIFO is full
    miovf: u1, // MIDI Input FIFO overflow
    moemp: u1, // MIDI Output FIFO is empty
    moful: u1, // MIDI Output FIFO is full
    _: u19 = 0,
};

pub const ChannelInfoReq = packed struct(u32) {
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

    lfo_phase: u32 = 0,

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

    // Unnecessary, but useful for debugging
    debug: struct {
        mute: bool = false, // Don't output sample (but run normally otherwise)
    } = .{},

    pub fn key_on(self: *AICAChannelState, registers: *const AICAChannel) void {
        if (self.amp_env_state != .Release) return;
        self.playing = true;
        self.loop_end_flag = false;
        self.play_position = 0; // Looping channels also start at 0
        self.amp_env_level = 0x280;
        self.amp_env_state = .Attack;
        self.filter_env_level = @truncate(registers.flv0);
        self.filter_env_state = .Attack;

        self.prev_sample = 0;
        self.curr_sample = 0;

        self.fractional_play_position = 0;

        self.adpcm_state = .{};
    }

    pub fn key_off(self: *AICAChannelState) void {
        self.playing = false;
        self.amp_env_state = .Release;
        self.amp_env_level = 0x3FF;
        self.filter_env_state = .Release;
        self.filter_env_level = 0x3FF;
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
        val = @min(val, 0x7FFF);
        val *= @as(i32, 1) - ((adpcm_sample >> 2) & 2);
        val += self.adpcm_state.prev;
        val = std.math.clamp(val, -0x8000, 0x7FFF);
        self.adpcm_state.step = (self.adpcm_state.step * ADPCMScale[adpcm_sample & 7]) >> 8;
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

fn apply_pan_attenuation(sample: i32, level: u4, pan: u5) struct { left: i32, right: i32 } {
    const att = 0xF - level;
    var left_att: u8 = att;
    var right_att: u8 = att;
    // PAN == 0x00 and 0x10 means center (no attenuation)
    const pan_att = pan & 0xF;
    // 5th bit selects the side to attenuate
    if (pan & 0x10 == 0x10) {
        right_att += pan_att;
    } else {
        left_att += pan_att;
    }

    var left: i32 = 0;
    var right: i32 = 0;
    if (left_att < 0xF)
        left = @divTrunc(sample, std.math.pow(i32, 2, left_att));
    if (right_att < 0xF)
        right = @divTrunc(sample, std.math.pow(i32, 2, right_att));
    return .{ .left = left, .right = right };
}

// Memory Map
// SH4 Side             Internal
// 00800000 - 00FFFFFF  00000000 - 007FFFFF  DRAM_AREA*
// 00700000 - 007027FF  00800000 - 008027FF  CHANNEL_DATA
// 00702800 - 00702FFF  00802800 - 00802FFF  COMMON_DATA
// 00703000 - 00707FFF  00803000 - 00807FFF  DSP_DATA
// 00710000 - 00710008        N/A            RTC_REGISTERS

pub const AICA = struct {
    pub const ExperimentalExternalSampleGeneration = true; // Runs sample generation in a separate thread.
    pub const ExperimentalThreadedARM = ExperimentalExternalSampleGeneration and true; // Runs the ARM core in the same thread used to generate samples, synched with sample generation, rather than with the SH4.

    pub const SampleRate = 44100;

    pub const ARM7CycleRatio = 66;
    pub const SH4CyclesPerSample = @divTrunc(200_000_000, SampleRate);

    arm7: arm7.ARM7 = undefined,
    enable_arm_jit: bool = true,
    arm_jit: ARM7JIT = undefined,

    regs: []u32, // All registers are 32-bit afaik
    wave_memory: []u8 align(4),

    channel_states: [64]AICAChannelState = .{.{}} ** 64,

    sample_mutex: std.Thread.Mutex = .{},
    sample_buffer: [2048]i32 = [_]i32{0} ** 2048,
    sample_read_offset: usize = 0,
    sample_write_offset: usize = 0,

    rtc_write_enabled: bool = false,

    arm_debug_trace: bool = false,

    _arm_cycles_counter: i32 = 0,
    _timer_cycles_counter: u32 = 0,
    _timer_counters: [3]u32 = .{0} ** 3,

    _samples_counter: u32 = 0,

    _allocator: std.mem.Allocator,

    // NOTE: Call setup_arm after!
    pub fn init(allocator: std.mem.Allocator) !AICA {
        var r = AICA{
            .regs = try allocator.alloc(u32, 0x8000 / 4),
            .wave_memory = try allocator.alloc(u8, 0x200000),
            ._allocator = allocator,
        };
        r.arm7 = arm7.ARM7.init(r.wave_memory, 0x1FFFFF, 0x800000);
        r.arm_jit = try ARM7JIT.init(allocator, r.arm7.memory_address_mask);

        try r.reset();

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

    pub fn reset(self: *@This()) !void {
        @memset(self.regs, 0);
        @memset(self.wave_memory, 0);

        self.channel_states = .{.{}} ** 64;

        self.sample_read_offset = 0;
        self.sample_write_offset = 0;

        self.rtc_write_enabled = false;

        self._arm_cycles_counter = 0;
        self._timer_cycles_counter = 0;
        self._timer_counters = .{0} ** 3;

        self._samples_counter = 0;

        self.get_reg(u32, .MasterVolume).* = 0x10;

        self.get_reg(u32, .SCIEB).* = 0x40;
        self.get_reg(u32, .SCILV0).* = 0x18;
        self.get_reg(u32, .SCILV1).* = 0x50;
        self.get_reg(u32, .SCILV2).* = 0x08;

        try self.arm_jit.reset();
    }

    // Read/Write from main CPU

    pub fn read_mem(self: *const AICA, comptime T: type, addr: u32) T {
        std.debug.assert(addr >= 0x00800000 and addr < 0x01000000);
        return @as(*T, @alignCast(@ptrCast(&self.wave_memory[(addr - 0x00800000) % self.wave_memory.len]))).*;
    }

    pub fn write_mem(self: *AICA, comptime T: type, addr: u32, value: T) void {
        std.debug.assert(addr >= 0x00800000 and addr < 0x01000000);
        const local_addr = addr - 0x00800000;
        const flush_cache = self.enable_arm_jit and local_addr >= self.arm_jit.block_cache.min_address and local_addr <= self.arm_jit.block_cache.max_address and self.read_mem(T, addr) != value;
        const lock = flush_cache and ExperimentalThreadedARM;
        if (lock) self.sample_mutex.lock();
        defer if (lock) self.sample_mutex.unlock();
        if (flush_cache)
            self.arm_jit.block_cache.signal_write(local_addr);

        // FIXME: No idea if this actually wraps around. Dev kit had 8MB of RAM instead of the final 2MB.
        @as(*T, @alignCast(@ptrCast(&self.wave_memory[(local_addr) % self.wave_memory.len]))).* = value;
    }

    pub fn get_channel_registers(self: *const AICA, number: u8) *const AICAChannel {
        std.debug.assert(number < 64);
        return @alignCast(@ptrCast(&self.regs[0x80 / 4 * @as(u32, number)]));
    }

    pub fn get_reg(self: *const AICA, comptime T: type, reg: AICARegister) *T {
        return @as(*T, @alignCast(@ptrCast(&self.regs[@intFromEnum(reg) / 4])));
    }

    fn get_dsp_mix_register(self: *const AICA, channel: u4) *DSPOutputMixer {
        return @as(*DSPOutputMixer, @alignCast(@ptrCast(&self.regs[(@as(u32, 0x2000) + 4 * @as(u32, channel)) / 4])));
    }

    pub fn debug_read_reg(self: *const AICA, comptime T: type, reg: AICARegister) T {
        return @as(*T, @alignCast(@ptrCast(&self.regs[@intFromEnum(reg) / 4]))).*;
    }

    pub fn read_register(self: *AICA, comptime T: type, addr: u32) T {
        aica_log.debug("Read({any}) to AICA Register at 0x{X:0>8}", .{ T, addr });

        const local_addr = addr & 0x0000FFFF;
        if (local_addr % 4 > 1) {
            aica_log.warn(termcolor.yellow("Read({any}) to non-existent (not 4 bytes aligned) AICA register at 0x{X:0>8}"), .{ T, addr });
            return 0;
        }

        //aica_log.debug("Read AICA register at 0x{X:0>8} (0x{X:0>8}) = 0x{X:0>8}", .{ addr, local_addr, self.regs[local_addr / 4] });
        if (local_addr < 0x2000) {}

        std.debug.assert(local_addr % 4 == 0 or T == u8);
        const reg_addr = local_addr - (local_addr % 4);
        const high_byte = T == u8 and local_addr % 4 == 1;

        switch (@as(AICARegister, @enumFromInt(reg_addr))) {
            //.MasterVolume => return if (!high_byte) 0x10 else 0,
            .MIDIInput => {
                var val = self.get_reg(MIDIInput, .MIDIInput);
                val.mibuf = 0;
                val.miemp = 1;
                val.miful = 0;
                val.miovf = 0;
                val.moemp = 1;
                val.moful = 0;
                val._ = 0;
            },
            .PlayStatus => {
                const req = self.get_reg(ChannelInfoReq, .ChannelInfoReq);
                var chan = &self.channel_states[req.monitor_select];
                var status: PlayStatus = .{
                    .env_level = @truncate(if (req.amplitude_or_filter_select == 0) chan.amp_env_level else chan.filter_env_level),
                    .env_state = chan.amp_env_state,
                    .loop_end_flag = chan.loop_end_flag,
                };
                if (status.env_level >= 0x3C0) status.env_level = 0x1FFF;
                if (T == u32 or high_byte)
                    chan.loop_end_flag = false; // Cleared on read.
                if (!high_byte) {
                    return @truncate(@as(u32, @bitCast(status)));
                } else {
                    return @truncate(@as(u32, @bitCast(status)) >> 8);
                }
            },
            .PlayPosition => {
                const req = self.get_reg(ChannelInfoReq, .ChannelInfoReq);
                const pos: u32 = self.channel_states[req.monitor_select].play_position;
                if (!high_byte) {
                    return @truncate(pos);
                } else {
                    return @truncate(pos >> 8);
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
        if (local_addr % 4 > 1) {
            aica_log.warn(termcolor.yellow("Out of bounds Write({any}) to AICA Register at 0x{X:0>8} = 0x{X:0>8}"), .{ T, addr, value });
            return;
        }

        // Channel registers
        if (local_addr < 0x2000) {
            switch (local_addr & 0x7F) {
                0x00 => { // Play control
                    switch (T) {
                        u8 => @as([*]u8, @ptrCast(self.regs.ptr))[local_addr] = value,
                        u32 => {
                            self.regs[local_addr / 4] = value & PlayControl.Mask;

                            const val: PlayControl = @bitCast(value);
                            aica_log.debug("Play control: 0x{X:0>8} = 0x{X:0>8}\n  {any}", .{ addr, value, val });
                            if (val.key_on_execute) self.key_on_execute();
                        },
                        else => @compileError("Invalid value type"),
                    }
                    return;
                },
                0x01 => { // Play control - High byte
                    switch (T) {
                        u8 => {
                            @as([*]u8, @ptrCast(self.regs.ptr))[local_addr] = value & @as(u8, @truncate(PlayControl.Mask >> 8));
                            const val: PlayControl = @bitCast(@as(u32, value) << 8);
                            if (val.key_on_execute) self.key_on_execute();
                        },
                        else => unreachable,
                    }
                    return;
                },
                else => {},
            }
        } else {
            std.debug.assert(local_addr % 4 == 0 or T == u8);
            const reg_addr = local_addr - (local_addr % 4);
            const high_byte = T == u8 and local_addr % 4 == 1;
            switch (@as(AICARegister, @enumFromInt(reg_addr))) {
                .MasterVolume => {
                    aica_log.debug("Write({any}) to Master Volume (0x{X:0>8}) = 0x{X:0>8}", .{ T, addr, value });
                },
                .DDIR_DEXE => { // DMA transfer direction / DMA transfer start
                    if (T == u8)
                        @as([*]u8, @ptrCast(&self.regs))[local_addr] = value & 0xFC;
                    if (T == u32)
                        self.regs[local_addr / 4] = value & 0xFFFFFFFC;
                    if (!high_byte and value & 1 == 1) {
                        aica_log.info(termcolor.green("DMA Start"), .{});
                        @panic("TODO AICA DMA");
                    }
                    return;
                },
                .SCIEB => {
                    aica_log.info("Write to AICA Register SCIEB = {any}", .{if (T == u32) @as(InterruptBits, @bitCast(value)) else value});
                    if (!high_byte) {
                        self.get_reg(u32, .SCIEB).* = value & @as(T, @truncate(0x7F9));
                    } else {
                        self.get_reg(u32, .SCIEB).* = ((@as(u32, value) << 8) & 0x7FF) | (self.get_reg(u32, .SCIEB).* & 0xFF);
                    }
                    self.check_interrupts();
                    return;
                },
                .SCIPD => {
                    aica_log.info("Write to AICA Register SCIPD = {any}", .{if (T == u32) @as(InterruptBits, @bitCast(value)) else value});
                    if (!high_byte) {
                        self.get_reg(u32, .SCIPD).* |= (value & (@as(u32, 1) << 5)); // Set scpu interrupt
                        self.check_interrupts();
                    }
                    return;
                },
                .SCIRE => { // Clear interrupt(s)
                    aica_log.debug("Write to AICA Register SCIRE = {any}", .{if (T == u32) @as(InterruptBits, @bitCast(value)) else value});
                    if (!high_byte) {
                        self.get_reg(u32, .SCIPD).* &= ~value;
                    } else {
                        self.get_reg(u32, .SCIPD).* &= ~(@as(u32, value) << 8);
                    }
                    self.check_interrupts();
                    return;
                },
                .MCIPD => {
                    if (!high_byte) {
                        self.get_reg(u32, .MCIPD).* |= (value & (@as(u32, 1) << 5)); // Set scpu interrupt
                        // FIXME: We should check for interrupts immediately here.
                        // self.check_sh4_interrupts(dc);
                    }
                    return;
                },
                .MCIRE => { // Clear interrupt(s)
                    if (!high_byte) {
                        self.get_reg(u32, .MCIPD).* &= ~value;
                    } else {
                        self.get_reg(u32, .MCIPD).* &= ~(@as(u32, value) << 8);
                    }
                },
                .ARMRST => {
                    if (!high_byte) {
                        aica_log.info("ARM reset : {d}", .{value & 1});

                        // Not sure if actually necessary, but some homebrew could let this set when interrupted
                        // during a FIQ (I guess at least) and then fail to reset it.
                        self.get_reg(u32, .INTRequest).* = 0;
                        self.check_interrupts();

                        self._arm_cycles_counter = 0;
                        self.arm7.reset(value & 1 == 0);
                        self.arm_jit.reset() catch unreachable;
                        if (value & 1 == 0 and self.wave_memory[0] == 0x00000000) {
                            aica_log.err(termcolor.red("  No code uploaded to ARM7, ignoring reset. FIXME: This is a hack."), .{});
                            self.arm7.running = false;
                        }
                    }
                },
                .INTRequest => {
                    aica_log.warn(termcolor.yellow("Write to AICA Register INTRequest = {X:0>8}"), .{value});
                    return;
                },
                .INTClear => {
                    aica_log.debug("Write to AICA Register INTClear = {X:0>8}", .{value});
                    if (!high_byte and value & 1 == 1) {
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

    pub fn timestamp() u32 {
        const utc = std.time.timestamp();
        // TODO: Handle timezone?
        return @intCast(utc + (20 * 365 + 5) * 24 * 60 * 60); // Dreamcast epoch is January 1, 1950 00:00
    }

    pub fn read_rtc_register(self: *const AICA, addr: u32) u32 {
        _ = self;
        std.debug.assert(addr >= 0x00710000);
        const dc_timestamp: u32 = timestamp();
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

    pub fn dump_wave_memory(self: *const @This()) void {
        const path = "logs/wave_memory_dump.bin";
        const file = std.fs.cwd().createFile(path, .{}) catch unreachable;
        defer file.close();
        _ = file.write(self.wave_memory) catch unreachable;
        aica_log.err("[+] Wrote wave memory dump to '{s}'.", .{path});
    }

    pub fn read8_from_arm(self: *AICA, addr: u32) u8 {
        std.debug.assert(addr & self.arm7.external_memory_address_mask != 0);
        return self.read_register(u8, addr);
    }

    pub fn read32_from_arm(self: *AICA, addr: u32) u32 {
        std.debug.assert(addr & self.arm7.external_memory_address_mask != 0);
        return self.read_register(u32, addr);
    }

    pub fn write8_from_arm(self: *AICA, addr: u32, value: u8) void {
        std.debug.assert(addr & self.arm7.external_memory_address_mask != 0);
        self.write_register(u8, addr, value);
    }
    pub fn write32_from_arm(self: *AICA, addr: u32, value: u32) void {
        std.debug.assert(addr & self.arm7.external_memory_address_mask != 0);
        self.write_register(u32, addr, value);
    }

    fn check_sh4_interrupt(self: *AICA, dc: *Dreamcast) void {
        if ((self.get_reg(u32, .MCIPD).* & self.get_reg(u32, .MCIEB).*) != 0) {
            dc.raise_external_interrupt(.{ .AICA = 1 });
        } else {
            dc.clear_external_interrupt(.{ .AICA = 1 });
        }
    }

    fn check_interrupts(self: *AICA) void {
        if (self.get_reg(u32, .INTRequest).* == 0) {
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
            }
        }
        self.arm7.fiq_signaled = self.get_reg(u32, .INTRequest).* != 0;
    }

    // Key on execute: Execute a key on for every channel this the KeyOn bit enabled.
    fn key_on_execute(self: *AICA) void {
        aica_log.debug(termcolor.green("Key On Execute"), .{});
        for (0..64) |i| {
            const regs = self.get_channel_registers(@intCast(i));
            if (regs.play_control.key_on_bit) {
                self.channel_states[i].key_on(regs);
            } else {
                self.channel_states[i].key_off();
            }
        }
    }

    pub fn generate_samples(self: *AICA, dc: *Dreamcast, sample_count: u32) void {
        if (sample_count > 0) {
            self.get_reg(InterruptBits, .SCIPD).one_sample_interval = 1;

            // Master Volume attenuation: -3dB (halfs the volume) for each attenuation level
            // FIXME: Really not sure I'm handling this correctly!
            //   Register Value | Volume
            // -----------------|----------
            //        0         | -MAXdB
            //        1         |  -42dB
            //        2         |  -39dB
            //        ...       |
            //        0xD       |   -6dB
            //        0xE       |   -3dB
            //        0xF       |    0dB
            {
                @memset(self.sample_buffer[self.sample_write_offset..@min(self.sample_write_offset + 2 * sample_count, self.sample_buffer.len)], 0);
                if (self.sample_write_offset + 2 * sample_count > self.sample_buffer.len)
                    @memset(self.sample_buffer[0 .. (self.sample_write_offset + 2 * sample_count) % self.sample_buffer.len], 0);

                for (0..64) |i| {
                    self.update_channel(@intCast(i), sample_count);
                }

                // Stream from GD-ROM
                const left_out = self.get_reg(DSPOutputMixer, .CDDAOutputLeft).*;
                const right_out = self.get_reg(DSPOutputMixer, .CDDAOutputRight).*;
                for (0..sample_count) |i| {
                    const samples = dc.gdrom.get_cdda_samples();
                    // I guess each channel can be independently redirected. That's a little weird, but mmh, ok.
                    const left_sample = apply_pan_attenuation(samples[0], left_out.efsdl, left_out.efpan);
                    self.sample_buffer[(self.sample_write_offset + 2 * i + 0) % self.sample_buffer.len] +|= left_sample.left;
                    self.sample_buffer[(self.sample_write_offset + 2 * i + 1) % self.sample_buffer.len] +|= left_sample.right;
                    const right_sample = apply_pan_attenuation(samples[1], right_out.efsdl, right_out.efpan);
                    self.sample_buffer[(self.sample_write_offset + 2 * i + 0) % self.sample_buffer.len] +|= right_sample.left;
                    self.sample_buffer[(self.sample_write_offset + 2 * i + 1) % self.sample_buffer.len] +|= right_sample.right;
                }

                const attenuation = 0xF - (self.get_reg(i32, .MasterVolume).* & 0x0F);
                if (attenuation == 0xF) {
                    for (0..sample_count) |i| {
                        self.sample_buffer[(self.sample_write_offset + 2 * i + 0) % self.sample_buffer.len] = 0;
                        self.sample_buffer[(self.sample_write_offset + 2 * i + 1) % self.sample_buffer.len] = 0;
                    }
                } else {
                    const factor = std.math.pow(i32, 2, attenuation);
                    for (0..sample_count) |i| {
                        // zig doesn't have a arithmetic shift right :(
                        self.sample_buffer[(self.sample_write_offset + 2 * i + 0) % self.sample_buffer.len] = @divTrunc(self.sample_buffer[(self.sample_write_offset + 2 * i + 0) % self.sample_buffer.len], factor);
                        self.sample_buffer[(self.sample_write_offset + 2 * i + 1) % self.sample_buffer.len] = @divTrunc(self.sample_buffer[(self.sample_write_offset + 2 * i + 1) % self.sample_buffer.len], factor);
                    }
                }

                self.sample_write_offset = (self.sample_write_offset + 2 * sample_count) % self.sample_buffer.len;
                self._samples_counter +%= sample_count;
            }
        }
    }

    pub fn update_timers(self: *AICA, dc: *Dreamcast, sample_count: u32) void {
        if (sample_count == 0) return;

        const timer_registers = [_]AICARegister{ .TACTL_TIMA, .TBCTL_TIMB, .TCCTL_TIMC };
        for (0..3) |i| {
            var timer = self.get_reg(TimerControl, timer_registers[i]);
            self._timer_counters[i] += sample_count;
            const scaled = @as(u32, 1) << timer.prescale;
            while (self._timer_counters[i] >= scaled) {
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

    pub fn update(self: *AICA, dc: *Dreamcast, sh4_cycles: u32) !void {
        if (!ExperimentalExternalSampleGeneration or !ExperimentalThreadedARM) {
            self.sample_mutex.lock();
            defer self.sample_mutex.unlock();

            if (!ExperimentalExternalSampleGeneration) {
                self._timer_cycles_counter += @intCast(sh4_cycles);
                const sample_count = @divTrunc(self._timer_cycles_counter, SH4CyclesPerSample);
                self._timer_cycles_counter = self._timer_cycles_counter % SH4CyclesPerSample;
                self.generate_samples(dc, sample_count);
                self.update_timers(dc, sample_count); // NOTE: When using ExperimentalExternalSampleGeneration, not sure if I should update the timer externally too or not.
            }

            try self.run_arm(sh4_cycles);
        }
    }

    pub fn run_arm(self: *AICA, sh4_cycles: u32) !void {
        if (self.arm7.running) {
            self._arm_cycles_counter += @intCast(sh4_cycles);
            if (self.enable_arm_jit) {
                if (self._arm_cycles_counter >= ARM7CycleRatio)
                    self._arm_cycles_counter -= ARM7CycleRatio * try self.arm_jit.run_for(&self.arm7, @divFloor(self._arm_cycles_counter, ARM7CycleRatio));
            } else {
                // FIXME: We're not actually counting ARM7 cycles here (unless all instructions are 1 cycle :^)).
                while (self._arm_cycles_counter >= ARM7CycleRatio) {
                    self._arm_cycles_counter -= ARM7CycleRatio;

                    if (self.arm_debug_trace) {
                        aica_log.info("arm7: ({s}) [{X:0>4}] {X:0>8} - {s: <20} - {X:0>8} - {X:0>8}", .{ @tagName(self.arm7.cpsr.m), self.arm7.pc() - 4, self.arm7.instruction_pipeline[0], arm7.ARM7.disassemble(self.arm7.instruction_pipeline[0]), self.arm7.sp(), self.arm7.lr() });
                    }

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
        if (state.amp_env_level >= 0x3FF) {
            state.amp_env_level = 0x3FF;
            return;
        }

        const registers = self.get_channel_registers(channel_number);

        const sample_length = 0x40000;
        // "For FNS = 0 (and OCT = 0), the tone matches the sampling source."
        var base_play_position_inc: u32 = sample_length | (@as(u32, registers.sample_pitch_rate.fns) << 8);
        if ((registers.sample_pitch_rate.oct & 8) == 0) {
            base_play_position_inc <<= registers.sample_pitch_rate.oct;
        } else {
            base_play_position_inc >>= @as(u5, 16) - registers.sample_pitch_rate.oct;
        }
        // "Values in parentheses are +1 octave for ADPCM" (p.25)
        if (registers.play_control.sample_format == .ADPCM and registers.sample_pitch_rate.oct >= 0x2 and registers.sample_pitch_rate.oct <= 0x7)
            base_play_position_inc <<= 1;

        for (0..samples) |sample_number| {
            const i = self._samples_counter +% sample_number;

            if (state.play_position == registers.loop_start) {
                if (registers.amp_env_2.link == 1 and state.amp_env_state == .Attack)
                    state.amp_env_state = .Decay;
                if (registers.lfo_control.reset) {
                    state.lfo_phase = 0;
                }
                if (!state.adpcm_state.loop_init) {
                    state.adpcm_state.step_loopstart = state.adpcm_state.step;
                    state.adpcm_state.prev_loopstart = state.adpcm_state.prev;
                    state.adpcm_state.loop_init = true;
                }
            }

            // Advance amplitude envelope
            {
                const effective_rate = AICAChannelState.compute_effective_rate(
                    registers,
                    switch (state.amp_env_state) {
                        .Attack => registers.amp_env_1.attack_rate,
                        .Decay => registers.amp_env_1.decay_rate,
                        .Sustain => registers.amp_env_1.sustain_rate,
                        .Release => registers.amp_env_2.release_rate,
                    },
                );
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
                                state.amp_env_level = 0x3FF;
                            }
                        },
                    }
                }
            }
            // Advance low pass filter envelope
            {
                const effective_rate = AICAChannelState.compute_effective_rate(
                    registers,
                    switch (state.filter_env_state) {
                        .Attack => registers.lpf_rates_1.lpf_attack_rate,
                        .Decay => registers.lpf_rates_1.lpf_decay_rate,
                        .Sustain => registers.lpf_rates_2.lpf_sustain_rate,
                        .Release => registers.lpf_rates_2.lpf_release_rate,
                    },
                );
                if (AICAChannelState.env_should_advance(effective_rate, @intCast(i))) {
                    const idx = if (effective_rate < 0x30) 0 else effective_rate - 0x30;
                    const decay = EnvelopeDecayValue[idx][i % 4];
                    const target: u16 = @truncate(switch (state.filter_env_state) {
                        .Attack => registers.flv1 & 0x1FFF,
                        .Decay => registers.flv2 & 0x1FFF,
                        .Sustain => registers.flv3 & 0x1FFF,
                        else => 0,
                    });
                    if (state.filter_env_level < target) {
                        state.filter_env_level +|= decay;
                        state.filter_env_level = @min(state.filter_env_level, target);
                    } else if (state.filter_env_level > target) {
                        if (state.filter_env_level > decay) {
                            state.filter_env_level -|= decay;
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
            if (!state.debug.mute) {
                const disdl = registers.direct_pan_vol_send.volume; // Attenuation level when output to the DAC. I guess that means when bypassing the DSP?
                const dipan = if (self.get_reg(MasterVolume, .MasterVolume).mono) 0 else registers.direct_pan_vol_send.pan;
                // Direct send to the DAC
                if (disdl != 0) { // 0 means full attenuation, not send.
                    const s = apply_pan_attenuation(sample, disdl, dipan);
                    self.sample_buffer[(2 * i + 0) % self.sample_buffer.len] +|= s.left;
                    self.sample_buffer[(2 * i + 1) % self.sample_buffer.len] +|= s.right;
                }
                // TODO: DSP!
                if (registers.dps_channel_send.level != 0) {
                    const channel = registers.dps_channel_send.channel;
                    const channel_mix = self.get_dsp_mix_register(channel);
                    // TEMP: Bypassing the DSP and outputting directly. Some sound without DSP effects is better that nothing for now.
                    const att: u4 = 0xF - registers.dps_channel_send.level;
                    const attenuated = @divTrunc(sample, std.math.pow(i32, 2, att));

                    const s = apply_pan_attenuation(attenuated, channel_mix.efsdl, channel_mix.efpan);

                    self.sample_buffer[(2 * i + 0) % self.sample_buffer.len] +|= s.left;
                    self.sample_buffer[(2 * i + 1) % self.sample_buffer.len] +|= s.right;
                }
            }

            state.fractional_play_position += base_play_position_inc;
            while (state.fractional_play_position >= sample_length) {
                state.fractional_play_position -= sample_length;

                state.prev_sample = state.curr_sample;
                const sample_ram = self.wave_memory[registers.sample_address()..];
                state.curr_sample = switch (registers.play_control.sample_format) {
                    .i16 => @as([*]const i16, @alignCast(@ptrCast(sample_ram.ptr)))[state.play_position],
                    .i8 => @as(i32, @intCast(@as([*]const i8, @alignCast(@ptrCast(sample_ram.ptr)))[state.play_position])) << 8,
                    // FIXME: ADPCMStream, how does it work?
                    .ADPCM, .ADPCMStream => adpcm: {
                        // 4 bits per sample
                        var s: u8 = @intCast(@as([*]const u8, @alignCast(@ptrCast(sample_ram.ptr)))[state.play_position >> 1]);
                        if (state.play_position & 1 == 1)
                            s >>= 4;
                        break :adpcm state.compute_adpcm(@truncate(s));
                    },
                };

                if (!registers.env_settings.lpoff) {
                    // TODO! Resonant Low Pass Filter
                }

                const lfo_phase_speed = [_]u32{ 0x3FC00, 0x37C00, 0x2FC00, 0x27C00, 0x1FC00, 0x1BC00, 0x17C00, 0x13C00, 0x0FC00, 0x0BC00, 0x0DC00, 0x09C00, 0x07C00, 0x06C00, 0x05C00, 0x04C00, 0x03C00, 0x03400, 0x02C00, 0x02400, 0x01C00, 0x01800, 0x01400, 0x01000, 0x00C00, 0x00A00, 0x00800, 0x00600, 0x00400, 0x00300, 0x00200, 0x00100 };
                state.lfo_phase +%= @intCast(@as(u64, 0x100000000) / lfo_phase_speed[registers.lfo_control.frequency]);

                // Apply amplitude envelope
                if (!registers.env_settings.voff) {
                    var attenuation: u32 = registers.env_settings.constant_attenuation;
                    attenuation <<= 2;
                    attenuation +|= state.amp_env_level;
                    if (registers.lfo_control.amplitude_modulation_depth != 0) {
                        // Low Frequency Oscillator amplitude modulation
                        // FIXME: This wasn't tested at all.
                        const att: u8 = @truncate(switch (registers.lfo_control.amplitude_modulation_waveform) {
                            .Sawtooth => state.lfo_phase >> 24,
                            .Square => (state.lfo_phase >> 31) * 0xFF,
                            .Triangle => if (state.lfo_phase & 0x80000000 == 0)
                                ((state.lfo_phase >> 23) & 0xFF)
                            else
                                (0xFF - ((state.lfo_phase >> 23) & 0xFF)),
                            .Noise => 0, // TODO
                        });
                        attenuation +|= @as(u32, 2) * (att >> (7 - registers.lfo_control.amplitude_modulation_depth));
                    }
                    if (attenuation >= 0x3C0) {
                        state.curr_sample = 0;
                    } else {
                        // (every 0x40 on the envelope attenuation level is 3dB)
                        state.curr_sample = @divTrunc(state.curr_sample, std.math.pow(i32, 2, @as(i32, @bitCast(attenuation)) >> 6));
                    }
                }

                if (registers.lfo_control.pitch_modulation_depth > 0) {
                    // TODO: Low Frequency Oscillator (LFO) pitch modulation
                }

                state.play_position +%= 1;

                if (state.play_position >= registers.loop_end & 0xFFFF) {
                    state.loop_end_flag = true;
                    if (registers.play_control.sample_loop) {
                        state.play_position = @truncate(registers.loop_start);
                        // NOTE: ADPCM long stream mode expects the user to correctly handle this themselves.
                        if (registers.play_control.sample_format == .ADPCM) {
                            state.adpcm_state.step = state.adpcm_state.step_loopstart;
                            state.adpcm_state.prev = state.adpcm_state.prev_loopstart;
                        }
                    } else {
                        state.playing = false;
                        state.amp_env_level = 0x3FF;
                        state.play_position = @truncate(registers.loop_end);
                        return;
                    }
                }
            }
        }
    }

    // TODO: Move these DMA functions to the Dreamcast module? Make them generic for the other G2 DMAs ?
    // NOTE: AD: AICA-DMA (ch0), E1: Ext1-DMA (ch1), E2: Ext2-DMA (ch2), DD: Dev-DMA (ch3)

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
        var adsusp = dc.hw_register(HardwareRegisters.SB_SUSP, .SB_ADSUSP);
        adsusp.dma_request_input_state = 0;
        adsusp.dma_suspend_or_dma_stop = 0;

        // Debug registers. NOTE: These are supposed to update during the transfer.
        dc.hw_register(u32, .SB_ADSTARD).* = root_bus_addr;
        dc.hw_register(u32, .SB_ADSTAGD).* = aica_addr;
        dc.hw_register(u32, .SB_ADLEND).* = len_in_bytes; // Remaining length.

        // Schedule the end of the transfer interrupt

        // G2 Bus: 25MHz 16bits - 200MHz (SH4 Clock) / 25MHz / 2 = 4 cycles per byte.
        // Used as theorical max speed.
        dc.schedule_event(.{ .function = @ptrCast(&end_dma), .context = self }, 4 * len_in_bytes);
    }

    pub fn end_dma(_: *AICA, dc: *Dreamcast) void {
        const len_reg = dc.read_hw_register(u32, .SB_ADLEN);
        const dma_end = (len_reg & 0x80000000) != 0; // DMA Transfer End/Restart
        const len = len_reg & 0x7FFFFFFF;
        // When a transfer ends, the DMA enable register is set to "0".
        if (dma_end)
            dc.hw_register(u32, .SB_ADEN).* = 0;

        dc.hw_register(u32, .SB_ADST).* = 0;

        // Debug registers. Used to keep track of the transfer state.
        dc.hw_register(u32, .SB_ADSTARD).* += len;
        dc.hw_register(u32, .SB_ADSTAGD).* += len;
        dc.hw_register(u32, .SB_ADLEND).* = 0; // Remaining length. "this value returns to its original setting immediately after DMA terminates." Does "original setting" mean "value at the start of the transfer", or 0?

        var adsusp = dc.hw_register(HardwareRegisters.SB_SUSP, .SB_ADSUSP);
        adsusp.dma_request_input_state = 1;
        adsusp.dma_suspend_or_dma_stop = 1;

        dc.raise_normal_interrupt(.{ .EoD_AICA = 1 });
    }

    pub fn serialize(self: *const @This(), writer: anytype) !usize {
        var bytes: usize = 0;
        bytes += try self.arm7.serialize(writer);
        bytes += try writer.write(std.mem.sliceAsBytes(self.regs[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self.wave_memory[0..]));
        bytes += try writer.write(std.mem.sliceAsBytes(self.channel_states[0..]));
        bytes += try writer.write(std.mem.asBytes(&self.rtc_write_enabled));
        bytes += try writer.write(std.mem.asBytes(&self._arm_cycles_counter));
        bytes += try writer.write(std.mem.asBytes(&self._timer_cycles_counter));
        bytes += try writer.write(std.mem.sliceAsBytes(self._timer_counters[0..]));
        bytes += try writer.write(std.mem.asBytes(&self._samples_counter));

        bytes += try writer.write(std.mem.asBytes(&self.sample_read_offset));
        bytes += try writer.write(std.mem.asBytes(&self.sample_write_offset));
        if (self.sample_read_offset > self.sample_write_offset) {
            bytes += try writer.write(std.mem.sliceAsBytes(self.sample_buffer[0..self.sample_write_offset]));
            bytes += try writer.write(std.mem.sliceAsBytes(self.sample_buffer[self.sample_read_offset..]));
        } else {
            bytes += try writer.write(std.mem.sliceAsBytes(self.sample_buffer[self.sample_read_offset..self.sample_write_offset]));
        }

        return bytes;
    }

    pub fn deserialize(self: *@This(), reader: anytype) !usize {
        self.sample_mutex.lock();
        defer self.sample_mutex.unlock();

        var bytes: usize = 0;
        bytes += try self.arm7.deserialize(reader);
        bytes += try reader.read(std.mem.sliceAsBytes(self.regs[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self.wave_memory[0..]));
        bytes += try reader.read(std.mem.sliceAsBytes(self.channel_states[0..]));
        bytes += try reader.read(std.mem.asBytes(&self.rtc_write_enabled));
        bytes += try reader.read(std.mem.asBytes(&self._arm_cycles_counter));
        bytes += try reader.read(std.mem.asBytes(&self._timer_cycles_counter));
        bytes += try reader.read(std.mem.sliceAsBytes(self._timer_counters[0..]));
        bytes += try reader.read(std.mem.asBytes(&self._samples_counter));

        bytes += try reader.read(std.mem.asBytes(&self.sample_read_offset));
        bytes += try reader.read(std.mem.asBytes(&self.sample_write_offset));
        if (self.sample_read_offset > self.sample_write_offset) {
            bytes += try reader.read(std.mem.sliceAsBytes(self.sample_buffer[0..self.sample_write_offset]));
            bytes += try reader.read(std.mem.sliceAsBytes(self.sample_buffer[self.sample_read_offset..]));
        } else {
            bytes += try reader.read(std.mem.sliceAsBytes(self.sample_buffer[self.sample_read_offset..self.sample_write_offset]));
        }

        return bytes;
    }
};
