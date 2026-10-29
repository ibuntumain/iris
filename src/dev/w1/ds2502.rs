//! DS2501 (512 bits, two 32-byte pages) and DS2502 (1 Kbit, four pages)
//! add-only memories: Read Memory (0xf0), Read Data/Generate CRC (0xc3)
//! and Read Status (0xaa). Writes are refused.
//!
//! Ported from hw/1wire/ds2502.c in jamesbraid/qemu-sgi-origami.

use super::crc::crc8;
use super::Part;

const DATA_SIZE: usize = 128;
const PAGE: usize = 32;
/// The status field's last byte, factory-programmed to 0x00; the seven
/// before it are erased.
const STATUS_FACTORY: u16 = 7;

pub struct Ds2502 {
    rom: [u8; 8],
    /// Every part answers a 128-byte address space; a DS2501 programs only
    /// its first two pages and reads the rest erased.
    data: [u8; DATA_SIZE],
    programmable: usize,
}

impl Ds2502 {
    fn build(family: u8, serial: u64, image: &[u8], pages: usize) -> Ds2502 {
        let programmable = pages * PAGE;
        let mut data = [0xff; DATA_SIZE];
        let n = image.len().min(programmable);
        data[..n].copy_from_slice(&image[..n]);
        let mut rom = [0u8; 8];
        rom[0] = family;
        rom[1..7].copy_from_slice(&serial.to_le_bytes()[..6]);
        rom[7] = crc8(&rom[..7]);
        Ds2502 { rom, data, programmable }
    }

    /// A DS2502 (128 bytes) with family code `family`, the 48-bit
    /// `serial`, and `image` at the start of its memory (the rest erased,
    /// 0xff).
    pub fn new(family: u8, serial: u64, image: &[u8]) -> Ds2502 {
        Ds2502::build(family, serial, image, 4)
    }

    /// A DS2501 (64 bytes), otherwise as `new`.
    pub fn ds2501(family: u8, serial: u64, image: &[u8]) -> Ds2502 {
        Ds2502::build(family, serial, image, 2)
    }

    /// How many bytes of the memory can hold an image (64 or 128).
    pub fn programmable(&self) -> usize {
        self.programmable
    }

    /// The memory as Read Memory sees it, all 128 addresses.
    pub fn memory(&self) -> &[u8] {
        &self.data
    }
}

impl Part for Ds2502 {
    fn rom(&self) -> [u8; 8] {
        self.rom
    }

    fn reply(&self, cmd: u8, ta: u16) -> Option<Vec<u8>> {
        let mut out = Vec::new();
        match cmd {
            0xf0 | 0xc3 => {
                // A start above the top has its high bits forced to zero,
                // and the echoed CRC-8 covers the address as masked, so a
                // master that asked above the top sees its own check fail.
                let a = ta as usize & (DATA_SIZE - 1);
                out.push(crc8(&[cmd, a as u8, 0]));
                if cmd == 0xf0 {
                    // Flat to the end of memory, then one CRC-8 over what
                    // was read.
                    out.extend_from_slice(&self.data[a..]);
                    out.push(crc8(&self.data[a..]));
                } else {
                    // A CRC-8 after every 32-byte page, the first page
                    // possibly short.
                    let mut at = a;
                    while at < DATA_SIZE {
                        let end = (at / PAGE + 1) * PAGE;
                        out.extend_from_slice(&self.data[at..end]);
                        out.push(crc8(&self.data[at..end]));
                        at = end;
                    }
                }
            }
            0xaa => {
                // The status field is not masked: a start past its end
                // reads erased bytes with no closing CRC.
                out.push(crc8(&[cmd, ta as u8, (ta >> 8) as u8]));
                if ta <= STATUS_FACTORY {
                    let status: Vec<u8> =
                        (ta..=STATUS_FACTORY).map(|i| if i == STATUS_FACTORY { 0x00 } else { 0xff }).collect();
                    out.extend_from_slice(&status);
                    out.push(crc8(&status));
                }
            }
            // Write Memory (0x0f), Write Status (0x55) and anything else.
            _ => return None,
        }
        Some(out)
    }
}
