//! DS2505 (16 Kbit, 64 pages of 32 bytes) add-only memory: Read Memory
//! (0xf0), Read Status (0xaa) and Extended Read Memory (0xa5). Writes are
//! refused.
//!
//! Ported from hw/1wire/ds2505.c in jamesbraid/qemu-sgi-origami.

use super::crc::{crc16, crc8};
use super::Part;

const DATA_SIZE: usize = 2048;
const PAGE: usize = 32;
const STATUS_PAGE: usize = 8;

pub struct Ds2505 {
    rom: [u8; 8],
    data: [u8; DATA_SIZE],
}

impl Ds2505 {
    /// A DS2505 with family code `family`, the 48-bit `serial`, and `image`
    /// at the start of its memory (the rest erased, 0xff).
    pub fn new(family: u8, serial: u64, image: &[u8]) -> Ds2505 {
        let mut data = [0xff; DATA_SIZE];
        let n = image.len().min(DATA_SIZE);
        data[..n].copy_from_slice(&image[..n]);
        let mut rom = [0u8; 8];
        rom[0] = family;
        rom[1..7].copy_from_slice(&serial.to_le_bytes()[..6]);
        rom[7] = crc8(&rom[..7]);
        Ds2505 { rom, data }
    }

    /// The memory as Read Memory sees it.
    pub fn memory(&self) -> &[u8] {
        &self.data
    }
}

/// A CRC-16 as the part sends it: inverted, low byte first.
fn inverted(crc: u16) -> [u8; 2] {
    (!crc).to_le_bytes()
}

impl Part for Ds2505 {
    fn rom(&self) -> [u8; 8] {
        self.rom
    }

    fn reply(&self, cmd: u8, ta: u16) -> Option<Vec<u8>> {
        // Data and status share an 11-bit address: a start above the top
        // has its high bits forced to zero, and the CRC-16 seed covers the
        // address as masked. This class echoes no header CRC.
        let a = ta as usize & (DATA_SIZE - 1);
        let hdr = [cmd, a as u8, (a >> 8) as u8];
        let mut out = Vec::new();
        match cmd {
            0xf0 => {
                out.extend_from_slice(&self.data[a..]);
                let c = crc16(crc16(0, &hdr), &self.data[a..]);
                out.extend_from_slice(&inverted(c));
            }
            0xaa => {
                // All status memory is erased: no write protection and no
                // redirection. Each 8-byte page ends in a CRC-16, the first
                // one covering the command.
                let mut at = a;
                let mut c = crc16(0, &hdr);
                while at < DATA_SIZE {
                    let end = (at / STATUS_PAGE + 1) * STATUS_PAGE;
                    let page = vec![0xff; end - at];
                    out.extend_from_slice(&page);
                    c = crc16(c, &page);
                    out.extend_from_slice(&inverted(c));
                    c = 0;
                    at = end;
                }
            }
            0xa5 => {
                let mut at = a;
                let mut first = true;
                while at < DATA_SIZE {
                    let end = (at / PAGE + 1) * PAGE;
                    // Page redirection byte: erased, not redirected. The
                    // part never acts on it; only the master does.
                    out.push(0xff);
                    let seed = if first { crc16(0, &hdr) } else { 0 };
                    out.extend_from_slice(&inverted(crc16(seed, &[0xff])));
                    out.extend_from_slice(&self.data[at..end]);
                    out.extend_from_slice(&inverted(crc16(0, &self.data[at..end])));
                    first = false;
                    at = end;
                }
            }
            // Write Memory, Speed Write, Write Status and anything else.
            _ => return None,
        }
        Some(out)
    }
}
