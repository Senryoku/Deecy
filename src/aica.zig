const std = @import("std");
const termcolor = @import("termcolor.zig");

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

const PlayControl = packed struct(u32) {
    start_address: u7, // Highest bits of the address
    sample_format: SampleFormat,
    sample_loop: bool,
    noise_enabled: bool,
    _0: u3 = 0,
    key_on_bit: bool,
    key_on_execute: bool,
    _1: u16 = 0,
};

const ChannelInfoReq = packed struct(u32) {
    MIDIOutputBuffer: u8,
    MonitorSelect: u6,
    AmplitudeOrFilterSelect: u1,
    _: u17 = 0,
};

const EnvelopeState = enum(u2) {
    Attack = 0,
    Decay = 1,
    Sustain = 2,
    Release = 3,
};

const PlayStatus = packed struct(u32) {
    EnvelopeLevel: u13 = 0,
    EnvelopeState: EnvelopeState = .Attack,
    LoopEndFlag: u1 = 0,
    _: u16 = 0,
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
    link: u1,
    _: u17 = 0,
};

// NOTE: Only the lower 16bits of each registers are actually used.
pub const AICAChannel = packed struct(u576) {
    play_control: PlayControl,
    sample_address: u32,
    loop_start: u32,
    loop_end: u32,
    amp_env_1: AmpEnv1,
    amp_env_2: AmpEnv2,
    sample_pitch_rate: u32,
    lfo_control: u32,
    dps_channel_send: u32,
    direct_pan_vol_send: u32,
    lpf1_volume: u32,
    lpf2_volume: u32,
    lpf3_volume: u32,
    lpf4_volume: u32,
    lpf5_volume: u32,
    lpf6_volume: u32,
    lpf7_volume: u32,
    lpf8_volume: u32,
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
    INTRequest = 0x00002D00,
    INTClear = 0x00002D004,

    RTC_High = 0x00010000,
    RTC_Low = 0x00010004,
    RTC_Write_Enable = 0x00010008,

    _,
};

pub const InterruptBits = packed struct(u32) {
    Ext: u1 = 0,
    _0: u2 = 0,
    MIDI_input: u1 = 0,
    DMA: u1 = 0,
    SCPU: u1 = 0,
    TimerA: u1 = 0,
    TimerB: u1 = 0,
    TimerC: u1 = 0,
    MIDI_output: u1 = 0,
    One_sample_interval: u1 = 0,
    _1: u21 = 0,
};

pub const SB_ADSUSP = packed struct(u32) {
    // DMA Suspend Request (Write Only
    // 0: Continues DMA transfer without going to the suspended state. Or, bit 2 of the SB_ADTSEL register is "0"
    // 1: Suspends and terminates DMA transfer
    DMASuspendRequest: u1 = 0,

    _reserved1: u3 = 0,

    // DMA Suspend or DMA Stop (Read Only)
    // 0: DMA transfer is in progress, or bit 2 of the SB_ADTSEL register is "0"
    // 1: DMA transfer has ended, or is stopped due to a suspend
    DMASuspend: u1 = 1,

    // DMA Request Input State (Read Only)
    // 0: The DMA transfer request is high (transfer not possible), or bit 2 of the SB_ADTSEL register is "0"
    // 1: The DMA transfer request is low (transfer possible)
    DMARequestInputState: u1 = 1,

    _: u26 = 0,
};

const TimerControl = packed struct(u32) {
    value: u8 = 0,
    prescale: u3 = 0,
    _: u21 = 0,
};

const AICARegisterStart = 0x00700000;

// Some potential memory-mapped registers, I'm really not sure what to do with this yet.
const AICAMemoryRegister = enum(u32) {
    REG_BUS_REQUEST = 0x00802808,
    REG_ARM_INT_ENABLE = 0x0080289c,
    REG_ARM_INT_SEND = 0x008028a0,
    REG_ARM_INT_RESET = 0x008028a4,
    REG_ARM_FIQ_BIT_0 = 0x008028a8,
    REG_ARM_FIQ_BIT_1 = 0x008028ac,
    REG_ARM_FIQ_BIT_2 = 0x008028b0,
    REG_SH4_INT_ENABLE = 0x008028b4,
    REG_SH4_INT_SEND = 0x008028b8,
    REG_SH4_INT_RESET = 0x008028bc,
    REG_ARM_FIQ_CODE = 0x00802d00,
    REG_ARM_FIQ_ACK = 0x00802d04,
};

pub const AICAChannelState = struct {
    playing: bool = false,
    status: PlayStatus = .{},
    play_position: u32 = 0,
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

// Memory Map
// SH4 Side             Internal
// 00800000 - 00FFFFFF  00000000 - 007FFFFF  DRAM_AREA*
// 00700000 - 007027FF  00800000 - 008027FF  CHANNEL_DATA
// 00702800 - 00702FFF  00802800 - 00802FFF  COMMON_DATA
// 00703000 - 00707FFF  00803000 - 00807FFF  DSP_DATA
// 00710000 - 00710008        N/A            RTC_REGISTERS

pub const DisableARMCore: bool = false; // FIXME: Temp debug. NOTE: The hacks are still needed for some stuff. For example the Soulcalibur intro goes a little further when the AICA is disabled...

pub const AICA = struct {
    const ARM7CycleRatio = 8;
    const SH4CyclesPerSample = 4535; // FIXME

    arm7: arm7.ARM7 = undefined,
    enable_arm_jit: bool = true,
    arm_jit: ARM7JIT = undefined,

    regs: []u32 = undefined, // All registers are 32-bit afaik
    wave_memory: []u8 align(4) = undefined,

    channel_states: [64]AICAChannelState = .{.{}} ** 64,

    rtc_write_enabled: bool = false,

    _arm_cycles_counter: i32 = 0,
    _timer_cycles_counter: u32 = 0,
    _timer_counters: [3]u32 = .{0} ** 3,

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
        r.arm_jit = try ARM7JIT.init(allocator);

        r.get_reg(u32, .MasterVolume).* = 0x10;

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

    pub fn read_mem(self: *const AICA, comptime T: type, addr: u32) T {
        // FIXME: Hopefully remove this when we have a working AICA (I mean, one can dream.)
        if (DisableARMCore) {
            switch (addr) {
                0x0080005C => {
                    return 0x1; // Hack for an infinite loop in Power Stone, no idea what this value is supposed to be.
                },
                0x00800104, 0x008001C4, 0x00800164, 0x00800224 => {
                    return 0x0090; // Crazy Taxi will hang indefinitely here during the demo if this is zero.
                },
                0x00800284, 0x00800288 => {
                    return 0x0090; // Same thing when trying to start an arcade game.
                },
                else => {},
            }
        }
        return @as(*T, @alignCast(@ptrCast(&self.wave_memory[(addr - 0x00800000) % self.wave_memory.len]))).*;
    }

    pub fn write_mem(self: *AICA, comptime T: type, addr: u32, value: T) void {
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

        switch (@as(AICARegister, @enumFromInt(local_addr))) {
            .MasterVolume => return 0x10,
            .PlayStatus => {
                // TODO:
                const req = self.get_reg(ChannelInfoReq, .ChannelInfoReq);
                const channel = self.get_channel_registers(req.MonitorSelect);
                _ = channel;
                const status = self.channel_states[req.MonitorSelect].status;
                self.channel_states[req.MonitorSelect].status.LoopEndFlag = 0;
                return @truncate(@as(u32, @bitCast(status)));
            },
            .PlayPosition => {
                const req = self.get_reg(ChannelInfoReq, .ChannelInfoReq);
                return @truncate(self.channel_states[req.MonitorSelect].play_position);
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
            const channel = local_addr / 0x80;
            _ = channel;
            switch (local_addr & 0x7F) {
                0x00 => { // Play control
                    // Key on execute: Execute a key on for every channel this the KeyOn bit enabled.
                    if (value & 0x1 == 1) {
                        for (0..64) |i| {
                            if (self.get_channel_registers(@intCast(i)).play_control.key_on_bit) {
                                self.channel_states[i].playing = true;
                                self.channel_states[i].play_position = 0;
                                self.channel_states[i].status.EnvelopeLevel = 0x280;
                                self.channel_states[i].status.EnvelopeState = .Attack;
                                self.channel_states[i].status.LoopEndFlag = 0;
                            } else {
                                self.channel_states[i].status.EnvelopeState = .Release;
                            }
                        }

                        switch (T) {
                            u8 => @as([*]u8, @ptrCast(self.regs.ptr))[local_addr] = value & 0xFE,
                            u32 => self.regs[local_addr / 4] = value & 0xFFFFFFFE,
                            else => @compileError("Invalid value type"),
                        }
                        return;
                    }
                },
                else => {},
            }
        }

        switch (@as(AICARegister, @enumFromInt(local_addr))) {
            .MasterVolume => {
                aica_log.warn(termcolor.yellow("Write to Master Volume = 0x{X:0>8}"), .{value});
            },
            .DDIR_DEXE => { // DMA transfer direction / DMA transfer start
                if (T == u8)
                    @as([*]u8, @ptrCast(&self.regs))[local_addr] = value & 0xFC;
                if (T == u32)
                    self.regs[local_addr / 4] = value & 0xFFFFFFFC;
                if (value & 1 == 1) {
                    aica_log.info(termcolor.green("DMA Start"), .{});
                    @panic("TODO AICA DMA");
                }
                return;
            },
            .SCIPD => {
                if (T == u32) {
                    aica_log.info("Write to AICA Register SCIPD = {any}", .{@as(InterruptBits, @bitCast(value))});
                } else {
                    aica_log.info("Write to AICA Register SCIPD = {any}", .{value});
                }
            },
            .SCIRE => { // Clear interrupt(s)
                self.get_reg(u32, .SCIPD).* &= ~value;
            },
            .MCIPD => {
                if (T == u32) {
                    aica_log.warn("Write to AICA Register MCIPD = 0x{X:0>8}", .{value});
                    if (@as(InterruptBits, @bitCast(value)).SCPU == 1 and self.get_reg(InterruptBits, .MCIEB).*.SCPU == 1) {
                        aica_log.warn(termcolor.green("SCPU interrupt"), .{});
                        // TODO!
                    }
                } else aica_log.err("Write8 to AICA Register MCIPD = 0x{X:0>8}", .{value});
            },
            .MCIRE => { // Clear interrupt(s)
                self.get_reg(u32, .MCIPD).* &= ~value;
            },
            .ARMRST => {
                aica_log.info("ARM reset : {d}", .{value & 1});
                self.arm7.reset(value & 1 == 0);
                if (value & 1 == 0 and self.wave_memory[0] == 0x00000000) {
                    aica_log.err(termcolor.red("  No code uploaded to ARM7, ignoring reset. FIXME: This is a hack."), .{});
                    self.arm7.running = false;
                }
            },
            .INTClear => {
                aica_log.warn(termcolor.yellow("Write to AICA Register INTClear = {X:0>8}"), .{value});
            },
            else => {},
        }
        switch (T) {
            u8 => @as([*]u8, @ptrCast(self.regs.ptr))[local_addr] = value,
            u32 => self.regs[local_addr / 4] = value,
            else => @compileError("Invalid value type"),
        }
    }

    pub fn read_rtc_register(self: *const AICA, addr: u32) u32 {
        _ = self;
        std.debug.assert(addr >= 0x00710000);
        return switch (addr - 0x00710000) {
            0x00 => (@as(u32, @intCast(std.time.timestamp())) >> 16) & 0x0000FFFFF,
            0x04 => @as(u32, @intCast(std.time.timestamp())) & 0x0000FFFFF,
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
                @panic("Read to unimplemented RTC register.");
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

    pub fn update(self: *AICA, dc: *Dreamcast, cycles: u32) !void {
        self._timer_cycles_counter += @intCast(cycles);
        const sample_count = @divTrunc(self._timer_cycles_counter, SH4CyclesPerSample);

        if (sample_count > 0) {
            if (self.get_reg(InterruptBits, .SCIEB).*.One_sample_interval == 1) {
                self.get_reg(u32, .INTRequest).* = (@as(u32, self.get_reg(InterruptBits, .SCILV0).*.One_sample_interval) << 0) |
                    (@as(u32, self.get_reg(InterruptBits, .SCILV1).*.One_sample_interval) << 1) |
                    (@as(u32, self.get_reg(InterruptBits, .SCILV2).*.One_sample_interval) << 2);
                self.arm7.fast_interrupt_request();
            }

            for (0..64) |i| {
                self.update_channel(@intCast(i), sample_count);
            }

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

                        check_sh4_interrupt(self, dc);

                        if ((self.get_reg(u32, .SCIEB).* & mask) != 0) {
                            aica_log.debug("Timer {d} interrupt.", .{i});
                            if (i == 0) {
                                self.get_reg(u32, .INTRequest).* = (@as(u32, self.get_reg(InterruptBits, .SCILV0).*.TimerA) << 0) |
                                    (@as(u32, self.get_reg(InterruptBits, .SCILV1).*.TimerA) << 1) |
                                    (@as(u32, self.get_reg(InterruptBits, .SCILV2).*.TimerA) << 2);
                            } else {
                                // Timer B and C share the same INTReq number.
                                self.get_reg(u32, .INTRequest).* = (@as(u32, self.get_reg(InterruptBits, .SCILV0).*.TimerB) << 0) |
                                    (@as(u32, self.get_reg(InterruptBits, .SCILV1).*.TimerB) << 1) |
                                    (@as(u32, self.get_reg(InterruptBits, .SCILV2).*.TimerB) << 2);
                            }
                            self.arm7.fast_interrupt_request();
                        }
                        timer.value = 0;
                    } else timer.value += 1;
                }
            }
        }

        if (self.arm7.running and !DisableARMCore) {
            self._arm_cycles_counter += @intCast(cycles);
            if (self.enable_arm_jit) {
                if (self._arm_cycles_counter >= ARM7CycleRatio)
                    self._arm_cycles_counter -= ARM7CycleRatio * try self.arm_jit.run_for(&self.arm7, @divFloor(self._arm_cycles_counter, ARM7CycleRatio));
            } else {
                // FIXME: We're not actually counting ARM7 cycles here (unless all instructions are 1 cycle :^)).
                while (self._arm_cycles_counter >= ARM7CycleRatio) {
                    self._arm_cycles_counter -= ARM7CycleRatio;
                    // aica_log.info("arm7: ({any}) [{X:0>4}] {X:0>8} - {s: <20} - {X:0>8} - {X:0>8}", .{ self.arm7.cpsr.m, self.arm7.pc() - 4, self.arm7.instruction_pipeline[0], arm7.ARM7.disassemble(self.arm7.instruction_pipeline[0]), self.arm7.sp(), self.arm7.lr() });
                    arm7.interpreter.tick(&self.arm7);
                }
            }
        }
    }

    pub fn update_channel(self: *@This(), channel_number: u8, samples: u32) void {
        const registers = self.get_channel_registers(channel_number);
        var state = &self.channel_states[channel_number];

        if (!state.playing) return;

        state.play_position +%= samples;

        switch (state.status.EnvelopeState) {
            .Attack => {
                if (state.status.EnvelopeLevel < registers.amp_env_1.attack_rate) {
                    state.status.EnvelopeLevel = 0;
                    state.status.EnvelopeState = .Decay;
                } else {
                    state.status.EnvelopeLevel -= registers.amp_env_1.attack_rate;
                }
            },
            .Decay => {
                state.status.EnvelopeLevel += registers.amp_env_1.decay_rate;
                if (state.status.EnvelopeLevel >= registers.amp_env_2.decay_level) {
                    state.status.EnvelopeLevel = registers.amp_env_2.decay_level;
                    state.status.EnvelopeState = .Sustain;
                }
            },
            .Sustain => {
                state.status.EnvelopeLevel += registers.amp_env_1.sustain_rate;
                if (state.status.EnvelopeLevel >= 0x3FF) {
                    state.status.EnvelopeLevel = 0x3FF;
                    state.status.EnvelopeState = .Release;
                }
            },
            .Release => {
                state.status.EnvelopeLevel += registers.amp_env_2.release_rate;
                if (state.status.EnvelopeLevel >= 0x3FF) {
                    state.status.EnvelopeLevel = 0x3FF;
                }
            },
        }

        if (state.status.EnvelopeState == .Release) {
            state.playing = false;
            state.status.EnvelopeLevel = 0x1FFF;
            state.status.LoopEndFlag = 1;
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
