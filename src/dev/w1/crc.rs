//! The Dallas/Maxim CRCs the parts frame their streams with.
//!
//! Ported from hw/1wire/crc.c in jamesbraid/qemu-sgi-origami.

/// CRC-8 (x^8 + x^5 + x^4 + 1, reflected), as in a ROM's last byte. A
/// ROM's CRC over all eight bytes is zero.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        let mut b = b;
        for _ in 0..8 {
            let mix = (crc ^ b) & 1;
            crc >>= 1;
            if mix != 0 {
                crc ^= 0x8c;
            }
            b >>= 1;
        }
    }
    crc
}

/// CRC-16 (x^16 + x^15 + x^2 + 1, reflected) from `seed`, as DS2505-class
/// parts and SGI's board records compute it. Parts send it inverted.
pub fn crc16(seed: u16, data: &[u8]) -> u16 {
    let mut crc = seed;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xa001 } else { crc >> 1 };
        }
    }
    crc
}
