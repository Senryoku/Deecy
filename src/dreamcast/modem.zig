const std = @import("std");
const log = std.log.scoped(.modem);

// Rockwell RCVDL56DPGL/SP

const Register = enum(u32) {
    ModemID0 = 0x60_0000,
    ModemID1 = 0x60_0004,

    pub fn as_u32(self: @This()) u32 {
        return @intFromEnum(self);
    }
};

const CountryCode = enum(u8) {
    Reserved = 0,
    Japan = 1,
    USA = 2,
    _,
};

const DeviceType = enum(u4) {
    @"33.6Kbps" = 0,
    _,
};

const MakerCode = enum(u4) {
    SEGA = 0,
    Rockwell = 1,
    _,
};

const R01 = packed struct(u8) {
    /// Received Parity Bit. This status bit is only valid when parity is enabled (PEN = 1), and
    /// word size is set for 8 bits per character (WDSZ = 11). In this case, the parity bit
    /// received (or ninth data bit) will be available at this location. The host must read this bit
    /// before reading the received data buffer (RBUFFER).
    rxp: u1,
    /// Receiver FIFO Half Full. When set, status bit RXHF indicates that there are 8 or more
    /// bytes in the 16-byte Receiver FIFO buffer. When reset, RXHF indicates that there are
    /// less than 8 bytes in the Receiver FIFO buffer. (TPDM = 1)
    /// An interrupt mask is available to allow an interrupt request to be generated when
    /// RXHF transitions from reset to set or from set to reset (the interrupt will occur as the
    /// FIFO fills above the half-full point and as the FIFO empties below the half-full point)
    /// (see Function 17 in Section 4).
    rxhf: u1,
    /// Transmitter FIFO Half Full. When set, status bit TXHF indicates that there are 8 or
    /// more bytes in the 16-byte Transmitter FIFO buffer. When reset, TXHF indicates that
    /// there are less than 8 bytes in the Transmitter FIFO buffer. (TPDM = 1, FIFOEN = 1)
    /// An interrupt mask is available to allow an interrupt request to be generated when TXHF
    /// transitions from reset to set or from set to reset (the interrupt will occur as the FIFO
    /// fills above the half-full point and as it empties below the half-full point) (see Section 4,
    /// Function 17).
    txhf: u1,
    _: u2 = 0,
    /// Voice Pause. Control bit VPAUSE enables (1) or disables (0) the voice "pause." When
    /// VPAUSE is enabled, voice data is not output to the host
    vpause: bool,
    /// Speaker Volume Control
    volume: enum(u2) {
        Off = 0, // Speaker off
        High = 1, // Speaker attenuation = 0 dB (high volume)
        Medium = 2, // Speaker attenuation = 6 dB (medium volume)
        Low = 3, // Speaker attenuation = 12 dB (low volume)
    },
};

const R02 = packed struct(u8) {
    _0: packed union {
        _0: packed struct(u6) {
            v54pe: u1,
            v54ae: u1,
            v54te: u1,
            rtsde: u1,
            _: u1 = 0,
            s511: u1,
        },
        _1: packed struct(u6) {
            /// Coder No. of Bits. Defines the number of bits per sample (2, 3, or 4) used by the
            /// ADPCM coder. (ADPCM receive mode only.)
            codbits: u2,
            _: u2 = 0,
            /// Coder Enable. In receive voice mode (CONF bits = ACh, 80h, 81h, 83h, or 86h, and
            /// RXV = 1), control bit CDEN = 1 selects ADPCM receive mode. In this mode, the MDP
            /// performs ADPCM coding and places the coder output into the Voice Receive Buffer
            /// (VBUFR). CDEN = 0 selects receive pass-through mode (see RXV bit).
            cden: u1,
            /// Decoder Enable. In transmit voice mode (CONF bits = ACh, 80h, 81h, 83h, or 86h
            /// and TXV = 1), control bit DCDEN = 1 selects ADPCM transmit voice mode. In this
            /// mode, the MDP performs ADPCM decoding on the contents of the Voice Transmit
            /// Buffer (VBUFT). DCDEN = 0 selects transmit voice pass-through mode (see TXV bit).
            dcden: u1,
        },
    },
    /// Squarer Disable (Tone Detector C). When control bit SQDIS is set, the squarer in
    /// front of tone detector C is disabled; when reset, the squarer is enabled. Disabling the
    /// squarer cascades the prefilter and filter C creating an 8th-order filter.
    sqdis: u1,
    /// Tone Detectors Enable. When control bit TDE is set, tone detectors A, B, and C are
    /// enabled; when reset, tone detectors are disabled.
    tde: u1,
};

const R03 = packed struct(u8) {
    gts: u1,
    gte: u1,
    _: u2 = 0,
    rlsde: u1,
    srcen: u1,
    sept: u1,
    ept: u1,
};

const R04 = packed struct(u8) {
    strn: u1,
    tdo: u1,
    nzien_vagc: u1,
    _0: u1 = 0,
    /// FIFO Enable. When control bit FIFOEN = 1, the host can input up to 16 bytes of data
    /// through TBUFFER, or voice samples through VBUFT, using the TDBE bit as a
    /// software interrupt or the TXRQ signal (DMAE = 1) as a DMA request interrupt. In
    /// HDLC, by default, if the host underruns the Transmit FIFO, the MDP will append a
    /// CRC. If bit 3A5h: 6 is set, the MDP will instead abort the frame and provide an abort
    /// code of 41h in ABCODE.
    /// The Receive FIFO is always enabled. The host may wait up to 16 byte-times before
    /// reading the data in RBUFFER. The RDBF bit or RXRQ signal (DMAE = 1) signals the
    /// availability of receive data to the host. The trigger level for RDBF bit/RXRQ signal is
    /// host programmable in DSP RAM. (See Section 4 for a detailed description about FIFO
    /// operation.) (TPDM = 1)
    fifoen: bool = false,
    _1: u2 = 0,
    rb: u1,
};

const R05 = packed struct(u8) {
    _0: u1 = 0,
    stoff: u1,
    _1: u1 = 0,
    ceq: u1,
    txsq: u1,
    _2: u3 = 0,
};

const R06 = packed struct(u8) {
    wdsz_decbits: u1,
    stb: u1,
    pen: u1,
    hdlc: u1,
    _0: u1 = 0,
    exos: u1,
    _1: u1 = 0,
};

const R07 = packed struct(u8) {
    /// Mark Hold. When control bit MHLD is set, the transmitter's digital input data is
    /// clamped to a mark. When MHLD is reset, the transmitter's input is taken from TXD or
    /// TBUFFER (see TPDM).
    mhld: u1,
    /// Relay A Activate. When control bit RA is set, the ~OHRC output (~RLYA) is active;
    /// when reset, the ~OHRC output is off (high).
    ra: u1,
    _0: u1 = 0,
    l3act: u1,
    _1: u1 = 0,
    l2act: u1,
    /// Remote Digital Loopback
    rdl: u1,
    /// Remote Digital Loopback Response Enable
    rdle: u1,
};

const R08 = packed struct(u8) {
    rts: u1,
    rtrn: u1,
    v54p: u1,
    v54a: u1,
    v54t: u1,
    v21s: u1,
    tpdm: u1,
    asyn: u1,
};

const R1D = packed struct(u8) {
    meaddh: u4,
    memcr: bool,
    memw: bool,
    _: u1,
    meacc: u1,
};

modemID0: CountryCode = .Reserved,
modemID1: packed struct(u8) { device_type: DeviceType = .@"33.6Kbps", maker_code: MakerCode = .Rockwell } = .{},
registers: []align(4) u8,
dsp_ram: []align(4) u8,

pub fn init(allocator: std.mem.Allocator) !@This() {
    return .{
        .registers = try allocator.allocWithOptions(u8, 0x20, 4, null),
        .dsp_ram = try allocator.allocWithOptions(u8, 0x1000, 4, null),
    };
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    allocator.free(self.registers);
    allocator.free(self.dsp_ram);
}

pub fn reset(self: *@This()) void {
    @memset(self.registers, 0);
    self.modemID0 = .Reserved;
    self.modemID1 = .{};

    self.controller_self_test();
    self.dsp_self_test();

    self.registers[0x07] = @bitCast(R07{ .mhld = 0, .ra = 0, .l3act = 0, .l2act = 0, .rdl = 0, .rdle = 1 });

    self.registers[0x1E] = 0x08;
}

fn controller_self_test(self: *@This()) void {
    // Controller Self-Test
    self.registers[0x1D] = 0x3C;
    self.registers[0x1C] = 0xEA;
    self.registers[0x1B] = 0x36;
    self.registers[0x1A] = 0x55;
    self.registers[0x19] = 0x4C;
    self.registers[0x18] = 0x5F;
    self.registers[0x17] = 0x32;
    self.registers[0x16] = 0x38;
    self.registers[0x15] = 0x01;
    self.registers[0x14] = 0x08;
    self.registers[0x13] = 0x36;
    self.registers[0x12] = 0x34;
    self.registers[0x11] = 0x41;
    self.registers[0x10] = 0x42;
}

fn dsp_self_test(self: *@This()) void {
    // DSP Self-Test
    self.registers[0x1B] = 0x0C;
    self.registers[0x1A] = 0x02;
    self.registers[0x19] = 0xEE;
    self.registers[0x18] = 0x46;
    self.registers[0x17] = 0xA6;
    self.registers[0x16] = 0xB2;
    self.registers[0x15] = 0x32;
    self.registers[0x14] = 0x38;
    self.registers[0x13] = 0x36;
    self.registers[0x12] = 0x34;
    self.registers[0x11] = 0x41;
    self.registers[0x10] = 0x20;
}

fn memory_address(self: *@This()) u16 {
    return @as(u16, self.registers[0x1D] & 0x0F) << 8 | self.registers[0x1C];
}

pub fn read(self: *@This(), comptime T: type, addr: u32) T {
    std.debug.assert(addr >= 0x00600000 and addr <= 0x006007FF);
    log.debug("Read({any}): {X}", .{ T, addr });
    switch (addr & 0x7FF) {
        0x000 => return @intFromEnum(self.modemID0),
        0x004 => return @as(u8, @bitCast(self.modemID1)),
        0x400...0x47F => {
            const reg = (addr & 0x7F) >> 2;
            log.debug("  Reading register {X}", .{reg});
            if (reg == 0x18) {
                // Memory Data LSB
                return self.dsp_ram[self.memory_address()];
            } else if (reg == 0x19) {
                // Memory Data MSB
                return self.dsp_ram[self.memory_address() + 1];
            }
            return self.registers[reg];
        },
        0x480 => return 0,
        else => log.err("Invalid Read({any}): @{X}", .{ T, addr }),
    }
    return 0;
}

pub fn write(self: *@This(), comptime T: type, addr: u32, value: T) void {
    std.debug.assert(addr >= 0x00600000 and addr <= 0x006007FF);
    log.debug("Write({any}): {X} = {X}", .{ T, addr, value });
    switch (addr & 0x7FF) {
        0x000 => self.modemID0 = @enumFromInt(@as(u8, @truncate(value))),
        0x004 => self.modemID1 = @bitCast(@as(u8, @truncate(value))),
        0x400...0x47F => {
            log.debug("  Writing register {X} = {X}", .{ (addr & 0x7F) >> 2, value });
            self.registers[(addr & 0x7F) >> 2] = @truncate(value);
        },
        0x480 => {
            if (value == 1) self.reset();
            return;
        },
        else => log.err("Invalid Write({any}): @{X} = {X}", .{ T, addr, value }),
    }
}
