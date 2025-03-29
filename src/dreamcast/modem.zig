const std = @import("std");
const log = std.log.scoped(.modem);

// Rockwell RCVDL56DPGL/SP

const Callback = struct {
    function: *const fn (*anyopaque) void,
    context: *anyopaque,

    pub fn call(self: @This()) void {
        self.function(self.context);
    }
};

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

const Registers = packed struct(u256) {
    // R00
    rbuffer: u8,

    // R01
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
    _0: u2 = 0,
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

    // R02
    _dunno: packed union {
        _dunno_0: packed struct(u6) {
            v54pe: u1,
            v54ae: u1,
            v54te: u1,
            rtsde: u1,
            _: u1 = 0,
            s511: u1,
        },
        _dunno_1: packed struct(u6) {
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

    // R03
    gts: u1,
    gte: u1,
    _1: u2 = 0,
    rlsde: u1,
    srcen: u1,
    sept: u1,
    ept: u1,

    // R04
    strn: u1,
    tdo: u1,
    nzien_vagc: u1,
    _2: u1 = 0,
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
    _3: u2 = 0,
    rb: u1,

    // R05
    _4: u1 = 0,
    /// Soft Turn Off
    stoff: u1,
    _5: u1 = 0,
    /// Compromise Equalizer Enable
    ceq: u1,
    /// Transmitter Squelch
    txsq: u1,
    _6: u3 = 0,

    // R06
    wdsz_decbits: u2,
    stb: u1,
    pen: u1,
    hdlc: u1,
    _7: u1 = 0,
    exos: u1,
    _8: u1 = 0,

    // R07
    /// Mark Hold. When control bit MHLD is set, the transmitter's digital input data is
    /// clamped to a mark. When MHLD is reset, the transmitter's input is taken from TXD or
    /// TBUFFER (see TPDM).
    mhld: u1,
    /// Relay A Activate. When control bit RA is set, the ~OHRC output (~RLYA) is active;
    /// when reset, the ~OHRC output is off (high).
    ra: u1,
    _9: u1 = 0,
    l3act: u1,
    _A: u1 = 0,
    l2act: u1,
    /// Remote Digital Loopback
    rdl: u1,
    /// Remote Digital Loopback Response Enable
    rdle: u1,

    // R08
    /// Request to Send
    /// When control bit RTS is set, the MDP transmits any data on TXD
    /// (TPDM = 0) or TBUFFER (TPDM = 1) when CTS becomes active.
    rts: u1,
    /// Retrain.
    rtrn: u1,
    v54p: u1,
    v54a: u1,
    v54t: u1,
    v21s: u1,
    /// Transmitter Parallel Data Mode. When control bit TPDM is set, the MDP accepts
    /// data for transmission from the TBUFFER (10h) rather than the TXD input
    tpdm: u1,
    /// Asynchronous/Synchronous. When configuration bit ASYN is set, asynchronous
    /// mode is selected; when 0, synchronous mode is selected
    asyn: u1,

    // R09
    dtr: u1,
    rrtse: u1,
    data: u1,
    ll: u1,
    org: u1,
    dtmf: u1,
    cc: u1,
    nv25: u1,

    // R0A
    syncd: u1,
    flags: u1,
    crcs: u1,
    oe: u1,
    fe: u1,
    pe: u1,
    flagdt: u1,
    pnsuc: u1,

    // R0B
    eqmat: u1,
    disdet: u1,
    _B: u1,
    atbel: u1,
    atv25: u1,
    tonec: u1,
    toneb: u1,
    tonea: u1,

    // R0C
    rseq: u1,
    rxfne: u1,
    sndet: u1,
    sdet: u1,
    ccdet: u1,
    cadet: u1,
    acdet: u1,
    aadet: u1,

    // R0D
    _C: u1,
    txfnf: u1,
    _D: u1,
    u1det: u1,
    scr1: u1,
    s1det: u1,
    pndet: u1,
    p2det: u1,

    // R0E
    speed: u5,
    rredt: u1,
    brkd: u1,
    rtdet: u1,

    // R0F
    v54dt: u1,
    rtsdt: u1,
    tm: u1,
    ri: u1,
    dsr: u1,
    cts: u1,
    fed: u1,
    rlsd: u1,

    // R10
    tbuffer: u8,

    // R11
    txp: u1,
    teof: u1,
    v23hdx: u1,
    rxv: u1,
    txv: u1,
    parsl: u2,
    brks: u1,

    // R12
    conf: u8,

    // R13
    txclk: u2,
    rth: u2,
    tlvl: u4,

    // R14
    abcode: u8,

    // R15
    earc: u1,
    exl3: u1,
    rren: u1,
    auto: u1,
    hwrwk: u1,
    rdwk: u1,
    _E: u1,
    sleep: u1,

    // R16
    /// Secondary Receive Data Buffer
    secrxb: u8,

    // R17
    /// Secondary Transmit  Data Buffer
    sectxb: u8,

    // R18
    /// Memory Access Data LSB B7-B0
    medal: u8,

    // R19
    /// Memory Access Data MSB B15-B8
    medam: u8,

    // R1A
    secen: u1,
    scibe: u1,
    scobf: u1,
    _F: u1,
    dmae: u1,
    rion: u1,
    rien: u1,
    sfres: u1,

    // R1B
    dtmfw: u4,
    dtmfd: u1,
    ots: u1,
    dtdet: u1,
    edet: u1,

    // R1C
    /// Memory Access Address Low B7-B0
    meaddl: u8,

    // R1D
    meaddh: u4,
    memcr: bool,
    memw: bool,
    _10: u1,
    meacc: u1,

    // R1E
    rdbf: u1,
    _11: u1,
    rdbie: u1,
    /// Transmit Data Buffer Empty. When set, status bit TDBE signifies that the MDP has
    /// read TBUFFER (10h) and the host can write new data into TBUFFER
    tdbe: u1,
    _12: u1,
    tdbie: u1,
    rdbia: u1,
    tdbia: u1,

    // R1F
    newc: u1,
    _13: u1,
    ncie: u1,
    news: u1,
    nsie: u1,
    _14: u1,
    ncia: u1,
    nsia: u1,
};

modemID0: CountryCode = .Reserved,
modemID1: packed struct(u8) { device_type: DeviceType = .@"33.6Kbps", maker_code: MakerCode = .Rockwell } = .{},
registers: Registers = undefined,
dsp_ram: []align(4) u8,

on_irq: ?Callback = null,

pub fn init(allocator: std.mem.Allocator, on_irq: ?Callback) !@This() {
    return .{
        .dsp_ram = try allocator.allocWithOptions(u8, 0x1000, 4, null),
        .on_irq = on_irq,
    };
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    allocator.free(self.dsp_ram);
}

pub fn reset(self: *@This()) void {
    @memset(std.mem.asBytes(&self.registers), 0);
    self.modemID0 = .Reserved;
    self.modemID1 = .{};

    self.controller_self_test();
    self.dsp_self_test();

    self.registers.rdle = 1;
}

fn reg(self: *@This()) []u8 {
    return std.mem.asBytes(&self.registers);
}

fn controller_self_test(self: *@This()) void {
    self.reg()[0x1D] = 0x3C;
    self.reg()[0x1C] = 0xEA;
    self.reg()[0x1B] = 0x36;
    self.reg()[0x1A] = 0x55;
    self.reg()[0x19] = 0x4C;
    self.reg()[0x18] = 0x5F;
    self.reg()[0x17] = 0x32;
    self.reg()[0x16] = 0x38;
    self.reg()[0x15] = 0x01;
    self.reg()[0x14] = 0x08;
    self.reg()[0x13] = 0x36;
    self.reg()[0x12] = 0x34;
    self.reg()[0x11] = 0x41;
    self.reg()[0x10] = 0x42;
    self.registers.tdbe = 1;
}

fn dsp_self_test(self: *@This()) void {
    self.reg()[0x1B] = 0x0C;
    self.reg()[0x1A] = 0x02;
    self.reg()[0x19] = 0xEE;
    self.reg()[0x18] = 0x46;
    self.reg()[0x17] = 0xA6;
    self.reg()[0x16] = 0xB2;
    self.reg()[0x15] = 0x32;
    self.reg()[0x14] = 0x38;
    self.reg()[0x13] = 0x36;
    self.reg()[0x12] = 0x34;
    self.reg()[0x11] = 0x41;
    self.reg()[0x10] = 0x20;
    self.registers.tdbe = 1;
}

fn memory_address(self: *@This()) u16 {
    return @as(u16, self.registers.meaddh) << 8 | self.registers.meaddl;
}

fn memory_access_data(self: *@This()) u16 {
    return @as(u16, self.registers.medam) << 8 | self.registers.medal;
}

fn read_register(self: *@This(), addr: u32) u8 {
    std.debug.assert(addr < 0x20);
    log.debug("  Reading register {X}", .{addr});
    switch (addr) {
        0x10 => {
            self.registers.tdbe = 0;
        },
        0x18 => {
            // Memory Data LSB
            return self.dsp_ram[self.memory_address()];
        },
        0x19 => {
            // Memory Data MSB
            return self.dsp_ram[self.memory_address() + 1];
        },
        else => {},
    }
    return self.reg()[addr];
}

pub fn read(self: *@This(), comptime T: type, addr: u32) T {
    std.debug.assert(addr >= 0x00600000 and addr <= 0x006007FF);
    log.debug("Read({any}): {X}", .{ T, addr });
    switch (addr & 0x7FF) {
        0x000 => return @intFromEnum(self.modemID0),
        0x004 => return @as(u8, @bitCast(self.modemID1)),
        0x400...0x47F => return self.read_register((addr & 0x7F) >> 2),
        0x480 => return 0,
        else => log.err("Invalid Read({any}): @{X}", .{ T, addr }),
    }
    return 0;
}

fn write_register(self: *@This(), addr: u32, value: u8) void {
    std.debug.assert(addr < 0x20);
    log.debug("  Writing register {X} = {X}", .{ addr, value });
    switch (addr) {
        0x1D => {
            // DSP RAM Write Procedure
            //   1. Set MEMW to inform the DSP that a RAM write will occur when MEACC is set.
            //   2. Load the RAM address into the MEADDH and MEADDL registers.
            //   3. Write the desired data into the interface memory RAM data registers MEDAM and/or MEDAL.
            //   4. Set MEACC to signal the DSP to perform the RAM write.
            //   5. When the DSP has transferred the contents of the interface memory RAM data registers into RAM, the MDP resets the
            //      MEACC bit and sets the NEWS bit to indicate DSP RAM write completion. If the NSIE bit is a 1, IRQ is asserted and
            //      NSIA is set to inform the host that setting of the NEWS bit is the source of the interrupt request.
            //   6. Upon the completion of IRQ servicing, write a 0 into the NEWS bit to clear the NSIA bit and to negate IRQ if no other
            //      interrupt requests are pending.
            self.reg()[addr] = value;
            if (self.registers.meacc == 1) {
                if (self.registers.memw) {
                    @as(*u16, @alignCast(@ptrCast(&self.dsp_ram[self.memory_address()]))).* = self.memory_access_data();
                }

                self.registers.meacc = 0;
                self.registers.news = 1;
                if (self.registers.nsie == 1) {
                    self.registers.nsia = 1;
                    self.assert_irq();
                }
            }
            return;
        },
        0x1F => {
            self.reg()[addr] = value;
            if (self.registers.news == 0) {
                self.registers.nsia = 0;
                self.clear_irq();
            }
            return;
        },
        else => {},
    }
    self.reg()[addr] = value;
}

pub fn write(self: *@This(), comptime T: type, addr: u32, value: T) void {
    std.debug.assert(addr >= 0x00600000 and addr <= 0x006007FF);
    log.debug("Write({any}): {X} = {X}", .{ T, addr, value });
    switch (addr & 0x7FF) {
        0x000 => self.modemID0 = @enumFromInt(@as(u8, @truncate(value))),
        0x004 => self.modemID1 = @bitCast(@as(u8, @truncate(value))),
        0x400...0x47F => self.write_register((addr & 0x7F) >> 2, @truncate(value)),
        0x480 => {
            if (value == 1) self.reset();
            return;
        },
        else => log.err("Invalid Write({any}): @{X} = {X}", .{ T, addr, value }),
    }
}

fn assert_irq(self: *@This()) void {
    if (self.on_irq) |cb| cb.call();
}
fn clear_irq(_: *@This()) void {}
