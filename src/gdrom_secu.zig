// This comes directly from MAME source (mame/machine/gdrom.cpp)
//   license:BSD-3-Clause
//   copyright-holders:smf, Angelo Salese

// Also copying their finding on these commands here so it's easily available.

// Officially not documented security-related packet commands:
//
// SYS_CHK_SECU (70h) Media security check
//    Parameters: 1byte - R0011111, R - "recheck" (it seems actual security check performed automatically at drive power on or when disc was inserted, so normally this and next command returns result of already performed check.
//                                      however, when MSB R bit is 1 will be forced media security recheck)
//    Result: none
// SYS_REQ_SECU (71h) Request security data
//    Parameters: 1byte - always 0x1f
//    Result: a bit less than 1Kbyte chunk of data (length vary each time), contains real command reply obfuscated, which is extracted by such code:
//
//   u8 reply[], real_reply[];
//   for (u32 i = 0, offset = 0; i < length; i++)
//   {
//     offset++;
//     u32 skip = reply[offset] - 2; // normally skip value is < 0x10, might be used to identify real reply length
//     offset += skip;
//     real_reply[i] = reply[offset++];
//   }
//
//   Real reply is 43 byte:
//    struct sec_reply {
//        u8 check_result;       // 0x1f - valid GD-ROM, 0x10 - valid Mil-CD (in this case following char[] fields is empty, 0x00-filled)
//        char key_id[10];       // presumable Disc ID (T-xxxxxx or HDR-xxxxx etc)
//        char key_maker_id[16]; // presumable "SEGA ENTERPRISES"
//        char hard_id[16];      // presumable "SEGA SEGAKATANA "
//    };
//
//    *_id fields names came from Dev.box "Checker BIOS" disassembly, contents meaning is guesswork because all the reply dumps we have now was dumped with Mil-CD disc inserted but not GD-ROM.
//    Presumable these data somehow encoded in GD-ROM disc HD area Lead-in (or Security Ring area ?), and compared with data in LD area IP.BIN by GD-drive firmware,
//    as described in Sega patent EP0935242A1 https://patents.google.com/patent/EP0935242A1
//
//    Dreamcast BIOS code verify only 1st result byte, if it's 5th bit (0x10) == 1.
//    Naomi DIMM firmware verify if result byte equal to 0x1f.
//
// SYS_CHG_COMD (72h) ??? Authentication for next command ?
//    Parameters: 1byte, probably key/password, in retail Dreamcast - 5th byte of unit SN# (located in flash ROM at 1A05Ah), 0 in Dev.box checker BIOS.
//    Result: none
//
// SYS_REQ_COMD (73h) Request command list
//    Parameters: none
//    Result: chunk of data where obfuscated real reply, see command 71.
//            real result: 14 bytes - codes of all regular (not security) packet commands supported by drive (00 10 11 12 13 14 15 16 20 21 22 30 31 40).
//
//  Dreamcast BIOS SysCalls contain commands 0x72/73 routine, but it seems not used at practice.

pub const GDROMCommand71Reply = [_]u8{
    0x96, 0x0B, 0x45, 0xF0, 0x7E, 0xFF, 0x3D, 0x06, 0x4D, 0x7D, 0x10, 0xBF, 0x07, 0x00, 0x73, 0xCF,
    0x9C, 0x00, 0xBC, 0x0C, 0x1C, 0xAF, 0x1C, 0x30, 0xE7, 0xA7, 0x03, 0xA8, 0x98, 0x00, 0xBD, 0x0F,
    0xBD, 0x5B, 0xAA, 0x50, 0x23, 0x39, 0x31, 0x10, 0x0E, 0x69, 0x13, 0xE5, 0x00, 0xD2, 0x0D, 0x66,
    0x54, 0xBF, 0x5F, 0xFD, 0x37, 0x74, 0xF4, 0x5B, 0x22, 0x00, 0xC6, 0x09, 0x0F, 0xCA, 0x93, 0xE8,
    0xA4, 0xAB, 0x00, 0x61, 0x0E, 0x2E, 0xE1, 0x4B, 0x76, 0x8B, 0x6A, 0xA5, 0x9C, 0xE6, 0x23, 0xC4,
    0x00, 0x4B, 0x06, 0x1B, 0x91, 0x01, 0x00, 0xE2, 0x0D, 0xCF, 0xCA, 0x38, 0x3A, 0xB9, 0xE7, 0x91,
    0xE5, 0xEF, 0x4B, 0x00, 0xD6, 0x09, 0xD3, 0x68, 0x3E, 0xC4, 0xAF, 0x2D, 0x00, 0x2A, 0x0D, 0xF9,
    0xFC, 0x78, 0xED, 0xAE, 0x99, 0xB3, 0x32, 0x5A, 0xE7, 0x00, 0x4C, 0x0A, 0x22, 0x97, 0x5B, 0x82,
    0x06, 0x7A, 0x4C, 0x00, 0x42, 0x0E, 0x57, 0x78, 0x46, 0xF5, 0x20, 0xFC, 0x6B, 0xCB, 0x01, 0x5B,
    0x86, 0x00, 0xE4, 0x0E, 0xB2, 0x26, 0xCD, 0x71, 0xE3, 0xA5, 0x33, 0x06, 0x8E, 0x9A, 0x50, 0x00,
    0x07, 0x07, 0xF5, 0x34, 0xEF, 0xE6, 0x00, 0x32, 0x0F, 0x13, 0x41, 0x59, 0x56, 0x0F, 0x02, 0x38,
    0x2A, 0x64, 0x2A, 0x07, 0x3E, 0x00, 0x52, 0x11, 0x2A, 0x1D, 0x5F, 0x76, 0x66, 0xA0, 0xB2, 0x2F,
    0x97, 0xC7, 0x5E, 0x6E, 0x52, 0xE2, 0x00, 0x58, 0x09, 0xCA, 0x89, 0xA5, 0xDF, 0x0A, 0xDE, 0x00,
    0x50, 0x06, 0x49, 0xB8, 0xB4, 0x00, 0x77, 0x05, 0x24, 0xE8, 0x00, 0xBB, 0x0C, 0x91, 0x89, 0xA2,
    0x8B, 0x62, 0xDE, 0x6A, 0xC6, 0x60, 0x00, 0xE7, 0x0F, 0x0F, 0x11, 0x96, 0x55, 0xD2, 0xBF, 0xE6,
    0x48, 0x0B, 0x5C, 0xAB, 0xDC, 0x00, 0xBA, 0x0A, 0x30, 0xD7, 0x48, 0x0E, 0x78, 0x63, 0x0C, 0x00,
    0xD2, 0x0D, 0xFB, 0x8A, 0xA3, 0xFE, 0xF8, 0x3A, 0xDD, 0x88, 0xA9, 0x4B, 0x00, 0xA2, 0x0A, 0x75,
    0x5D, 0x0D, 0x37, 0x24, 0xC5, 0x9D, 0x00, 0xF7, 0x0B, 0x25, 0xEF, 0xDB, 0x41, 0xE0, 0x52, 0x3E,
    0x4E, 0x00, 0xB7, 0x03, 0x00, 0xE5, 0x11, 0xB9, 0xDE, 0x5A, 0x57, 0xCF, 0xB9, 0x1A, 0xFC, 0x7F,
    0x26, 0xEE, 0x7B, 0xCD, 0x2B, 0x00, 0x4B, 0x08, 0xB8, 0x09, 0x70, 0x6A, 0x9F, 0x00, 0x4B, 0x11,
    0x8C, 0x15, 0x87, 0xA3, 0x05, 0x4F, 0x37, 0x8E, 0x63, 0xDE, 0xEF, 0x39, 0xFC, 0x4B, 0x00, 0xAB,
    0x10, 0x0B, 0x91, 0xAA, 0x0F, 0xE1, 0xE9, 0xAE, 0x69, 0x3A, 0xF8, 0x03, 0x69, 0xD2, 0x00, 0xE2,
    0x07, 0xC1, 0x5C, 0x3D, 0x82, 0x00, 0xA9, 0x08, 0x68, 0xC4, 0xAD, 0x2E, 0xD1, 0x00, 0xF7, 0x0E,
    0xC6, 0x47, 0xC8, 0xCD, 0x8E, 0x7C, 0x00, 0x5C, 0x95, 0xB9, 0xF4, 0x00, 0xE3, 0x04, 0x5B, 0x00,
    0x74, 0x07, 0x65, 0xC7, 0x84, 0x8E, 0x00, 0xC6, 0x07, 0x61, 0x80, 0x44, 0x3F, 0x00, 0xC8, 0x0E,
    0x72, 0x78, 0x47, 0xD3, 0xC2, 0x4D, 0xAF, 0xC0, 0x54, 0x13, 0x31, 0x00, 0xF7, 0x0D, 0x48, 0xD8,
    0xE2, 0x92, 0x9F, 0x7F, 0x2F, 0x44, 0x68, 0x33, 0x00, 0x0D, 0x10, 0xAB, 0xFE, 0xEA, 0x8E, 0x19,
    0x81, 0xF8, 0x6F, 0x7C, 0xDE, 0xE1, 0xB3, 0x06, 0x00, 0x4D, 0x11, 0x66, 0xAE, 0x4C, 0xF9, 0xB7,
    0x2F, 0xEE, 0xB0, 0x8E, 0x7E, 0xE1, 0x8D, 0x95, 0x6F, 0x00, 0xF4, 0x0D, 0x88, 0x9D, 0xCA, 0xE3,
    0xC4, 0xB2, 0x47, 0xBB, 0xA0, 0x69, 0x00, 0xF3, 0x0B, 0x48, 0x17, 0x41, 0x64, 0xA0, 0x0E, 0x71,
    0x82, 0x00, 0x34, 0x1E, 0x18, 0x4D, 0x85, 0x80, 0x4C, 0xA9, 0x0B, 0x66, 0x9B, 0x75, 0x13, 0x61,
    0x70, 0x27, 0x81, 0x7A, 0x02, 0xCD, 0x57, 0xAB, 0xDF, 0x02, 0x93, 0x52, 0x83, 0xDF, 0x48, 0xA8,
    0xA6, 0x9E, 0x74, 0x6F, 0x89, 0x03, 0x28, 0x25, 0x52, 0x96, 0xFF, 0x67, 0x7A, 0xD8, 0x3C, 0xB1,
    0x2C, 0x46, 0x84, 0xEF, 0xE1, 0xC1, 0xC6, 0xC9, 0xDC, 0x96, 0xAA, 0xA9, 0xC4, 0x82, 0x58, 0x27,
    0x57, 0x75, 0x67, 0x34, 0xFB, 0x3B, 0x25, 0xBF, 0xFB, 0x3B, 0xF6, 0x13, 0xEC, 0x96, 0xE5, 0x16,
    0x26, 0xFD, 0xA8, 0xDA, 0x1B, 0xC6, 0x50, 0x7F, 0x47, 0xFF, 0x08, 0x55, 0x08, 0xED, 0x00, 0x93,
    0x9B, 0xC4, 0x71, 0x67, 0xEC, 0xA6, 0xCC, 0x16, 0x20, 0x87, 0x47, 0x07, 0xA6, 0x00, 0x79, 0x5D,
    0x4F, 0xAB, 0xA1, 0x6F, 0x7A, 0x6B, 0x27, 0xC4, 0xDA, 0xA3, 0xC3, 0x94, 0x4F, 0x7F, 0xF3, 0xE5,
    0x1B, 0x6F, 0xCC, 0xE5, 0xF0, 0xE5, 0x9D, 0xC9, 0xAE, 0xFD, 0x39, 0xAC, 0x4C, 0xE5, 0x58, 0x83,
    0x25, 0x65, 0x92, 0x74, 0x9E, 0x81, 0xA0, 0xB6, 0xA9, 0x02, 0x9B, 0x07, 0xB6, 0xE7, 0x79, 0x57,
    0xD9, 0x4A, 0xCE, 0xFA, 0xB4, 0x94, 0x05, 0xCC, 0x86, 0x3C, 0xDD, 0x06, 0xCD, 0xA6, 0x24, 0x24,
    0xFA, 0xC1, 0xF9, 0x48, 0xC9, 0x0C, 0x6C, 0xC4, 0x96, 0x82, 0x17, 0xF6, 0x31, 0x09, 0xC4, 0xE2,
    0x77, 0xFD, 0xCF, 0x46, 0x18, 0xB2, 0x5F, 0x01, 0x6B, 0xD1, 0x7B, 0x56, 0xB8, 0x94, 0x4A, 0xE5,
    0x6C, 0x19, 0xF0, 0xC0, 0xB6, 0x70, 0x93, 0xF7, 0xD3, 0xD1, 0x2B, 0x6E, 0x7C, 0x53, 0x6D, 0x85,
    0xD1, 0x0C, 0x8B, 0x77, 0xEE, 0x90, 0xDA, 0x15, 0x55, 0xE0, 0x58, 0x09, 0x56, 0xFC, 0x31, 0x9F,
    0xAF, 0x46, 0xCB, 0xC3, 0x8D, 0x71, 0x75, 0xF2, 0x2C, 0xC3, 0xBB, 0xA1, 0xC4, 0xCF, 0x27, 0x56,
    0x7C, 0x9B, 0xFE, 0xAF, 0x3E, 0x4E, 0xB4, 0xCD, 0x6A, 0xAA, 0xF5, 0xF3, 0xE3, 0x22, 0x82, 0xE1,
    0xA5, 0x68, 0xB3, 0xDB, 0x8F, 0x9E, 0x5E, 0x7B, 0x90, 0xF0, 0x79, 0x3F, 0x52, 0x8C, 0x61, 0x88,
    0x76, 0xAE, 0x14, 0x63, 0x19, 0x0F, 0x1D, 0xCE, 0xA1, 0x63, 0x10, 0xB2, 0xE2, 0xD7, 0x94, 0xB1,
    0x33, 0xCB, 0x28, 0x85, 0x7D, 0x9B, 0xF5, 0xF4, 0x25, 0x50, 0x9B, 0xDB, 0x35, 0xA5, 0xB0, 0x9C,
    0x09, 0x92, 0xE3, 0x31, 0x40, 0xAB, 0x4D, 0xF4, 0x35, 0xE8, 0xB3, 0x0A, 0x21, 0xC3, 0x86, 0x9C,
    0xCB, 0x29, 0xA4, 0x77, 0x57, 0xBC, 0xD8, 0xDA, 0xA5, 0x82, 0x80, 0xE8, 0xCF, 0x72, 0x81, 0xAD,
    0x2E, 0x28, 0xFF, 0xD8, 0xB6, 0xD1, 0x2B, 0x97, 0x00, 0xFF, 0xE1, 0x06, 0x44, 0x39, 0x1C, 0x4B,
    0xAB, 0x19, 0x5B, 0x4D, 0xD6, 0x3E, 0x1B, 0x5C, 0x64, 0xBB, 0x32, 0x68, 0xF5, 0x7C, 0xC9, 0x9E,
    0xE8, 0xB4, 0x29, 0x1B, 0x7F, 0x4D, 0x80, 0x80, 0x7E, 0x8B, 0x1C, 0x0A, 0xE6, 0x9A, 0xBF, 0x49,
    0x1E, 0xC5, 0xB6, 0x67, 0x7D, 0x05, 0xE4, 0x90, 0x40, 0x4B, 0xAF, 0x9B, 0x52, 0xDE, 0x17, 0x80,
    0x81, 0x56, 0xEA, 0x3A, 0x53, 0x82, 0x8C, 0x62, 0xFB, 0x96, 0x97, 0x6F, 0xC1, 0x16, 0x78, 0xD4,
    0x7B, 0xE7, 0xB9, 0x5A, 0x2A, 0xEB, 0x87, 0x68, 0x33, 0xD3, 0x31, 0x45, 0xFA, 0xFE, 0xF4, 0x1C,
    0x90, 0x86, 0x73, 0x77, 0xD9, 0xA9, 0xD1, 0x4A, 0x4A, 0xCF, 0xAE, 0x23, 0xDB, 0xF9, 0x09, 0xD8,
    0x18, 0xDC, 0x6A, 0x0D, 0xE4, 0x19, 0x8C, 0x65, 0xC6, 0x64, 0xC7, 0xDC, 0xA9, 0xE3, 0x91, 0xB1,
    0x4C, 0xC8, 0xC1, 0x9E, 0x3B, 0x7F, 0xCB, 0xA3, 0xCF, 0xDD, 0xF0, 0x1D, 0x07, 0x6E, 0xDC, 0xCE,
    0x0D, 0xCD, 0x7E, 0x1E, 0x55, 0x11, 0x8B, 0xDF, 0x3A, 0xAB, 0xB6, 0x3B, 0x6E, 0x52, 0x7F, 0xA7,
    0x00, 0xD1, 0x33, 0xBE, 0xF2, 0x9B, 0xFC, 0x4A, 0xCF, 0x9D, 0x8F, 0xC6, 0xC4, 0x7B, 0xDA, 0xE7,
    0x2A, 0x1C, 0x26, 0x6E,
};
