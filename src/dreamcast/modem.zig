const std = @import("std");
const log = std.log.scoped(.modem);

const DreamcastModule = @import("dreamcast.zig");
const Dreamcast = DreamcastModule.Dreamcast;

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
    rbuffer: u8 = undefined,

    // R01
    /// Received Parity Bit. This status bit is only valid when parity is enabled (PEN = 1), and
    /// word size is set for 8 bits per character (WDSZ = 11). In this case, the parity bit
    /// received (or ninth data bit) will be available at this location. The host must read this bit
    /// before reading the received data buffer (RBUFFER).
    rxp: u1 = 0,
    /// Receiver FIFO Half Full. When set, status bit RXHF indicates that there are 8 or more
    /// bytes in the 16-byte Receiver FIFO buffer. When reset, RXHF indicates that there are
    /// less than 8 bytes in the Receiver FIFO buffer. (TPDM = 1)
    /// An interrupt mask is available to allow an interrupt request to be generated when
    /// RXHF transitions from reset to set or from set to reset (the interrupt will occur as the
    /// FIFO fills above the half-full point and as the FIFO empties below the half-full point)
    /// (see Function 17 in Section 4).
    rxhf: u1 = 0,
    /// Transmitter FIFO Half Full. When set, status bit TXHF indicates that there are 8 or
    /// more bytes in the 16-byte Transmitter FIFO buffer. When reset, TXHF indicates that
    /// there are less than 8 bytes in the Transmitter FIFO buffer. (TPDM = 1, FIFOEN = 1)
    /// An interrupt mask is available to allow an interrupt request to be generated when TXHF
    /// transitions from reset to set or from set to reset (the interrupt will occur as the FIFO
    /// fills above the half-full point and as it empties below the half-full point) (see Section 4,
    /// Function 17).
    txhf: u1 = 0,
    _0: u2 = 0,
    /// Voice Pause. Control bit VPAUSE enables (1) or disables (0) the voice "pause." When
    /// VPAUSE is enabled, voice data is not output to the host
    vpause: bool = false,
    /// Speaker Volume Control
    volume: enum(u2) {
        Off = 0, // Speaker off
        High = 1, // Speaker attenuation = 0 dB (high volume)
        Medium = 2, // Speaker attenuation = 6 dB (medium volume)
        Low = 3, // Speaker attenuation = 12 dB (low volume)
    } = .Off,

    // R02
    _dunno: packed union {
        _dunno_0: packed struct(u6) {
            v54pe: u1 = 0,
            v54ae: u1 = 0,
            v54te: u1 = 0,
            rtsde: u1 = 0,
            _: u1 = 0,
            s511: u1 = 0,
        },
        _dunno_1: packed struct(u6) {
            /// Coder No. of Bits. Defines the number of bits per sample (2, 3, or 4) used by the
            /// ADPCM coder. (ADPCM receive mode only.)
            codbits: u2 = undefined,
            _: u2 = 0,
            /// Coder Enable. In receive voice mode (CONF bits = ACh, 80h, 81h, 83h, or 86h, and
            /// RXV = 1), control bit CDEN = 1 selects ADPCM receive mode. In this mode, the MDP
            /// performs ADPCM coding and places the coder output into the Voice Receive Buffer
            /// (VBUFR). CDEN = 0 selects receive pass-through mode (see RXV bit).
            cden: u1 = 0,
            /// Decoder Enable. In transmit voice mode (CONF bits = ACh, 80h, 81h, 83h, or 86h
            /// and TXV = 1), control bit DCDEN = 1 selects ADPCM transmit voice mode. In this
            /// mode, the MDP performs ADPCM decoding on the contents of the Voice Transmit
            /// Buffer (VBUFT). DCDEN = 0 selects transmit voice pass-through mode (see TXV bit).
            dcden: u1 = 0,
        },
    } = .{ ._dunno_0 = .{} },
    /// Squarer Disable (Tone Detector C). When control bit SQDIS is set, the squarer in
    /// front of tone detector C is disabled; when reset, the squarer is enabled. Disabling the
    /// squarer cascades the prefilter and filter C creating an 8th-order filter.
    sqdis: u1 = 0,
    /// Tone Detectors Enable. When control bit TDE is set, tone detectors A, B, and C are
    /// enabled; when reset, tone detectors are disabled.
    tde: u1 = 1,

    // R03
    gts: u1 = 0,
    gte: u1 = 0,
    _1: u2 = 0,
    /// RLSD Enable. When control bit RLSDE is set, the ~RLSD pin reflects the RLSD bit
    /// state. When RLSDE is reset, the ~RLSD pin is clamped OFF and data is clamped to a
    /// mark regardless of the state of the RLSD bit.
    rlsde: u1 = 1,
    /// Secondary Rate Change Enable
    srcen: u1 = 0,
    /// Short Echo Protector Tone
    sept: u1 = 0,
    /// Echo Protector Tone Enable
    ept: u1 = 0,

    // R04
    /// Short Train Select
    strn: u1 = 0,
    tod: u1 = 0, // ?? Not in docs?
    /// Voice AGC. In receive voice mode (RXV = 1), the host can enable (VAGC = 1) or
    /// disable (VAGC = 0) the voice AGC.
    nzien_vagc: u1 = 0,
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
    /// Relay B Activate. When control bit RB is set, the ~TALK output (~RLYB) is active;
    /// when reset, the ~TALK output is off (high).
    rb: u1 = 0,

    // R05
    _4: u1 = 0,
    /// Soft Turn Off
    stoff: u1 = 0,
    _5: u1 = 0,
    /// Compromise Equalizer Enable
    ceq: u1 = 1,
    /// Transmitter Squelch
    txsq: u1 = 0,
    _6: u3 = 0,

    // R06
    wdsz_decbits: u2 = 0,
    stb: u1 = 0,
    /// Parity Enable. When control bit PEN is set, parity is enabled in asynchronous mode.
    /// This bit must be configured appropriately before the ASYN bit changes from a 0 to a 1
    /// for asynchronous mode. (K56flex, V.34, V.32 bis, V.32, V.22, V.22 bis, Bell 212A)
    pen: u1 = 0,
    /// HDLC Select. When control bit HDLC is set, HDLC operation is enabled. When HDLC
    /// is reset, HDLC operation is disabled. The ASYN bit must be 0 and the TDBE bit must
    /// be 1 prior to setting HDLC to a 1. The HDLC bit is valid only in synchronous parallel
    /// data mode (ASYN = 0 and TPDM = 1). Not valid in FSK modes except V.21 channel 2.
    /// RTS must be off before switching in or out of HDLC mode while in DATA mode.
    hdlc: u1 = 0,
    _7: u1 = 0,
    /// Extended Overspeed.
    exos: u1 = 0,
    _8: u1 = 0,

    // R07
    /// Mark Hold. When control bit MHLD is set, the transmitter's digital input data is
    /// clamped to a mark. When MHLD is reset, the transmitter's input is taken from TXD or
    /// TBUFFER (see TPDM).
    mhld: u1 = 0,
    /// Relay A Activate. When control bit RA is set, the ~OHRC output (~RLYA) is active;
    /// when reset, the ~OHRC output is off (high).
    ra: u1 = 0,
    _9: u1 = 0,
    /// Loop 3 Activate
    l3act: u1 = 0,
    _A: u1 = 0,
    /// Loop 2 Activate
    l2act: u1 = 0,
    /// Remote Digital Loopback
    rdl: u1 = 0,
    /// Remote Digital Loopback Response Enable
    rdle: u1 = 1,

    // R08
    /// Request to Send
    /// When control bit RTS is set, the MDP transmits any data on TXD
    /// (TPDM = 0) or TBUFFER (TPDM = 1) when CTS becomes active.
    rts: u1 = 0,
    /// Retrain.
    rtrn: u1 = 0,
    v54p: u1 = 0,
    v54a: u1 = 0,
    v54t: u1 = 0,
    v21s: u1 = 0,
    /// Transmitter Parallel Data Mode. When control bit TPDM is set, the MDP accepts
    /// data for transmission from the TBUFFER (10h) rather than the TXD input
    tpdm: u1 = 0,
    /// Asynchronous/Synchronous. When configuration bit ASYN is set, asynchronous
    /// mode is selected; when 0, synchronous mode is selected
    asyn: u1 = 0,

    // R09
    /// Data Terminal Ready
    dtr: u1 = 0,
    rrtse: u1 = 0,
    data: u1 = 1,
    /// Leased Line
    ll: u1 = 0,
    /// Originate
    org: u1 = 0,
    /// DTMF Select
    dtmf: u1 = 1,
    cc: u1 = 0,
    nv25: u1 = 0,

    // R0A
    /// Sync Pattern Detected
    syncd: u1 = 0,
    flags: u1 = 0,
    crcs: u1 = 0,
    oe: u1 = 0,
    fe: u1 = 0,
    pe: u1 = 0,
    flagdt: u1 = undefined,
    pnsuc: u1 = 0,

    // R0B
    eqmat: u1 = 0,
    disdet: u1 = undefined,
    _B: u1 = 0,
    atbel: u1 = undefined,
    atv25: u1 = undefined,
    tonec: u1 = undefined,
    toneb: u1 = undefined,
    tonea: u1 = undefined,

    // R0C
    rseq: u1 = 0,
    rxfne: u1 = undefined,
    sndet: u1 = undefined,
    sdet: u1 = undefined,
    ccdet: u1 = undefined,
    cadet: u1 = undefined,
    acdet: u1 = undefined,
    aadet: u1 = undefined,

    // R0D
    _C: u1 = 0,
    txfnf: u1 = undefined,
    _D: u1 = 0,
    u1det: u1 = undefined,
    scr1: u1 = undefined,
    s1det: u1 = undefined,
    pndet: u1 = undefined,
    p2det: u1 = undefined,

    // R0E
    speed: u5 = undefined,
    rredt: u1 = undefined,
    brkd: u1 = undefined,
    rtdet: u1 = undefined,

    // R0F
    v54dt: u1 = 0,
    rtsdt: u1 = undefined,
    tm: u1 = undefined,
    ri: u1 = undefined,
    dsr: u1 = undefined,
    cts: u1 = undefined,
    fed: u1 = undefined,
    rlsd: u1 = undefined,

    // R10
    tbuffer: u8 = 0,

    // R11
    txp: u1 = 0,
    teof: u1 = 0,
    v23hdx: u1 = 0,
    rxv: u1 = 0,
    txv: u1 = 0,
    parsl: u2 = 0,
    brks: u1 = 0,

    // R12
    conf: u8 = 0x76, // FIXME: Is it 76 hex or dec?

    // R13
    txclk: u2 = 0,
    rth: u2 = 0,
    tlvl: u4 = 9,

    // R14
    abcode: u8 = 0,

    // R15
    earc: u1 = 0,
    exl3: u1 = 0,
    rren: u1 = 0,
    auto: u1 = 0,
    hwrwk: u1 = 1,
    rdwk: u1 = 0,
    _E: u1 = 0,
    sleep: u1 = 0,

    // R16
    /// Secondary Receive Data Buffer
    secrxb: u8 = undefined,

    // R17
    /// Secondary Transmit  Data Buffer
    sectxb: u8 = undefined,

    // R18
    /// Memory Access Data LSB B7-B0
    medal: u8 = 0,

    // R19
    /// Memory Access Data MSB B15-B8
    medam: u8 = 0,

    // R1A
    /// Secondary Channel Enable
    secen: u1 = 0,
    scibe: u1 = undefined,
    scobf: u1 = undefined,
    _F: u1 = 0,
    /// DMA Signals Enabled
    dmae: u1 = 0,
    rion: u1 = 0,
    rien: u1 = 0,
    /// Soft Reset. When control bit SFRES is set to a 1, the MDP will perform power-on
    /// reset processing. The NEWC bit will be reset to a 0 by the MDP upon completion of the
    /// reset processing. NEWC must be set to initiate the reset. Wait for NEWC to clear
    /// before accessing the MDP.
    sfres: u1 = 0,

    // R1B
    dtmfw: u4 = undefined,
    dtmfd: u1 = undefined,
    ots: u1 = undefined,
    dtdet: u1 = undefined,
    edet: u1 = undefined,

    // R1C
    /// Memory Access Address Low B7-B0
    meaddl: u8 = 0,

    // R1D
    meaddh: u4 = 0,
    memcr: bool = false,
    memw: bool = false,
    _10: u1 = 0,
    meacc: u1 = 0,

    // R1E
    /// Receive Data Buffer Full. When set, status bit RDBF signifies that the receiver wrote
    /// valid data into RBUFFER (00h). This condition can also cause IRQ to be asserted. The
    /// host reading RBUFFER resets the RDBF and RDBIA bits. (See RDBIE and RDBIA.)
    rdbf: u1 = undefined,
    _11: u1 = 0,
    /// Receive Data Buffer Interrupt Enable. When control bit RDBIE is set (interrupt
    /// enabled), the MDP will assert IRQ and set the RDBIA bit when RDBF is set by the
    /// MDP. When RDBIE is reset (interrupt disabled), RDBF has no effect on IRQ or RDBIA.
    /// (See RDBF and RDBIA.)
    rdbie: u1 = 0,
    /// Transmit Data Buffer Empty. When set, status bit TDBE signifies that the MDP has
    /// read TBUFFER (10h) and the host can write new data into TBUFFER
    tdbe: u1 = 1,
    _12: u1 = 0,
    /// Transmit Data Buffer Interrupt Enable. When control bit TDBIE is set (interrupt
    /// enabled), the MDP will assert IRQ and set the TDBIA bit when TDBE is set by the
    /// MDP. When TDBIE is reset (interrupt disabled), TDBE has no effect on IRQ or TDBIA.
    /// (See TDBE and TDBIA.)
    tdbie: u1 = 0,
    /// Receive Data Buffer Interrupt Active.
    rdbia: u1 = undefined,
    /// Transmit Data Buffer Interrupt Active
    tdbia: u1 = undefined,

    // R1F
    /// New Configuration. Control bit NEWC must be set by the host after the host changes
    /// the configuration code in CONF (12h) or changes any of the following controls bits:
    /// CEQ (05h:3), DTMF (09h:5), GTE (03h:1), GTS (03h:0), L3ACT (07h:3), LL (09h:3),
    /// ORG(09h:4), RTH (13h:2,3), RXV (11h:3), SFRES (1Ah:7), SLEEP (15h:7), TLVL
    /// (13h:7-4), TXCLK (13h:1-0), TXV (11h:4), V21S (08h:5), or V23HDX (11h:2). This
    /// informs the MDP to implement the new configuration. The MDP resets the NEWC bit
    /// when the configuration change is implemented. A configuration change can also cause
    /// IRQ to be asserted. NOTE: NEWC should not be set once a connection is being
    /// attempted or completed. (See NCIE and NCIA.)
    newc: u1 = 0,
    _13: u1 = 0,
    ncie: u1 = 0,
    news: u1 = undefined,
    nsie: u1 = 0,
    _14: u1 = 0,
    ncia: u1 = undefined,
    nsia: u1 = undefined,
};

pub const State = enum {
    Starting,
    ControllerSelfTest,
    DSPSelfTest,
    Normal,
};

pub const Event = enum {
    ControllerSelfTestEnd,
    DSPTest,
    DSPTestEnd,
};

state: State = .Starting,
modemID0: CountryCode = .Reserved,
modemID1: packed struct(u8) { device_type: DeviceType = .@"33.6Kbps", maker_code: MakerCode = .Rockwell } = .{},
registers: Registers = .{},
dsp_ram: []align(4) u8,

_dc: *Dreamcast,

pub fn init(allocator: std.mem.Allocator, dc: *Dreamcast) !@This() {
    return .{
        .dsp_ram = try allocator.allocWithOptions(u8, 0x1000, 4, null),
        ._dc = dc,
    };
}

pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
    allocator.free(self.dsp_ram);
}

pub fn reset(self: *@This()) void {
    log.info("Hard Reset", .{});
    self.modemID0 = .Reserved;
    self.modemID1 = .{};
    self.registers = .{};
    @memset(self.dsp_ram, 0);
}

fn soft_reset(self: *@This()) void {
    log.info("Soft Reset", .{});

    self.controller_self_test();
}

fn reg(self: *@This()) []u8 {
    return std.mem.asBytes(&self.registers);
}

pub fn on_event(self: *@This(), event: Event) void {
    log.debug("Event: {s}", .{@tagName(event)});
    switch (event) {
        .ControllerSelfTestEnd => self.controller_self_test_end(),
        .DSPTest => self.dsp_self_test(),
        .DSPTestEnd => self.dsp_self_test_end(),
    }
}

fn controller_self_test(self: *@This()) void {
    log.info("Controller Self Test", .{});
    self.reg()[0x1D] = 0xEA;
    self.reg()[0x1C] = 0x3C;
    self.reg()[0x1B] = 0x55;
    self.reg()[0x1A] = 0x36;
    self.reg()[0x19] = 0x5F;
    self.reg()[0x18] = 0x4C;
    self.reg()[0x17] = 0x38;
    self.reg()[0x16] = 0x35;
    self.reg()[0x15] = 0x08;
    self.reg()[0x14] = 0x01;
    self.reg()[0x13] = 0x37;
    self.reg()[0x12] = 0x30;
    self.reg()[0x11] = 0x42;
    self.reg()[0x00] = 0x41;
    self.state = .ControllerSelfTest;
    self._dc.schedule_event(.{ .Modem = .ControllerSelfTestEnd }, 100_000);
}

fn controller_self_test_end(self: *@This()) void {
    if (self.state != .ControllerSelfTest) return;

    self.registers.tdbe = 1;

    // When bit 1Eh:3 is reset or 5 ms has expired, the information is cleared and the DSP self-test is performed. If the register 10
    // is not read by the host, bit 1Eh:3 will not be reset
    // FIXME: KallistiOS reads it twice? Doesn't move to DSP test immediately?
    self._dc.schedule_event(.{ .Modem = .DSPTest }, 1_000_000);
}

fn dsp_self_test(self: *@This()) void {
    if (self.state != .ControllerSelfTest) return;

    log.info("DSP Self Test", .{});
    self.reg()[0x1B] = 0xF0;
    self.reg()[0x1A] = 0x83;
    self.reg()[0x19] = 0x46;
    self.reg()[0x18] = 0xEE;
    self.reg()[0x17] = 0x00;
    self.reg()[0x16] = 0xFA;
    self.reg()[0x15] = 0x0A;
    self.reg()[0x14] = 0x09;
    self.reg()[0x13] = 0x37;
    self.reg()[0x12] = 0x30;
    self.reg()[0x11] = 0x20;
    self.reg()[0x00] = 0x41;
    self.registers.tdbe = 1;
    self.registers.newc = 0;
    self.state = .DSPSelfTest;
    // The host should read the test results within 5 ms of bit 1Eh:3 being set.
    // When bit 1Eh:3 is reset or 5 ms has expired since DSP test completion, the information is cleared and MDP initialization
    // continues.
    self._dc.schedule_event(.{ .Modem = .DSPTestEnd }, 2_000_000); // FIXME: 5ms should be 1_000_000, but seems too short for KallistiOS?
}

fn dsp_self_test_end(self: *@This()) void {
    if (self.state != .DSPSelfTest) return;

    self.registers = .{};
    self.registers.newc = 0;
    self.state = .Normal;

    // FIXME: KallistiOS waits for that.
    self.registers.tonea = 1;
    self.registers.toneb = 1;
    self.registers.tonec = 1;
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
            // if (self.state == .ControllerSelfTest) {
            //     self._dc.schedule_event(.{ .Modem = .DSPTest }, 100_000);
            // }
            // if (self.state == .DSPSelfTest) {
            //     self._dc.schedule_event(.{ .Modem = .DSPTestEnd }, 100_000);
            // }
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

fn read_tbuffer(self: *@This()) void {
    self.registers.tdbe = 1;
    if (self.registers.tdbie == 1) {
        self.registers.tdbia = 1;
        self.assert_irq();
    }
}

fn write_register(self: *@This(), addr: u32, value: u8) void {
    std.debug.assert(addr < 0x20);
    log.debug("  Writing register {X} = {X}", .{ addr, value });
    switch (addr) {
        0x10 => {
            // TBUFFER
            self.reg()[0x10] = value;
            log.debug(" Write to TBUFFER: {X} ({c})", .{ value, value });
            self.registers.tdbe = 0;
            // Delay?
            self.read_tbuffer();
            return;
        },
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
                    log.debug("Write to DSP RAM: {X} = {X}", .{ self.memory_address(), self.memory_access_data() });
                    if (self.memory_address() & 1 == 0) {
                        @as(*u16, @alignCast(@ptrCast(&self.dsp_ram[self.memory_address()]))).* = self.memory_access_data();
                    } else {
                        self.dsp_ram[self.memory_address()] = @truncate(self.memory_access_data());
                        self.dsp_ram[self.memory_address() + 1] = @truncate(self.memory_access_data() >> 8);
                    }
                } else {
                    log.debug("Read from DSP RAM: {X} = {X}", .{ self.memory_address(), self.memory_access_data() });
                    // DSP RAM Read Procedure
                    //   1. Reset MEMW to inform the DSP that a RAM read will occur when MEACC is set.
                    //   2. Load the RAM address code into the MEADDH and MEADDL registers.
                    //   3. Set MEACC to signal the DSP to perform the RAM read.
                    //   4. When the DSP has transferred the contents of RAM into the interface memory RAM data registers MEDAM and/or
                    //       MEDAL, the MDP resets the MEACC bit and sets the NEWS bit to indicate DSP RAM read completion. If the NSIE bit is
                    //       a 1, IRQ is asserted and NSIA is set to inform the host that setting of the NEWS bit is the source of the interrupt
                    //       request.
                    //   5. Upon the completion of IRQ servicing, write a 0 into the NEWS bit to clear the NSIA bit and to negate IRQ if no other
                    //      interrupt requests are pending.
                    self.registers.medal = self.dsp_ram[self.memory_address()];
                    self.registers.medam = self.dsp_ram[self.memory_address() + 1];
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
            if (self.registers.newc == 1 and self.registers.sfres == 1) {
                self.registers.newc = 0;
                self.registers.sfres = 0;
                self.soft_reset();
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
            if (value == 1) {
                self.reset();
            } else {
                @memset(std.mem.asBytes(&self.registers), 0);
            }
            return;
        },
        else => log.err("Invalid Write({any}): @{X} = {X}", .{ T, addr, value }),
    }
}

fn assert_irq(self: *@This()) void {
    self._dc.raise_external_interrupt(.{ .Modem = 1 });
}
fn clear_irq(_: *@This()) void {}

pub fn serialize(self: @This(), writer: anytype) !usize {
    // TODO
    const bytes: usize = 0;
    _ = self;
    _ = writer;
    return bytes;
}

pub fn deserialize(self: @This(), reader: anytype) !usize {
    // TODO
    const bytes: usize = 0;
    _ = self;
    _ = reader;
    return bytes;
}
