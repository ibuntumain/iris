//! The 1-wire (MicroLAN) bus and the Dallas parts SGI machines carry on it:
//! one implementation for every master (the Octane's HEART, Bridge, IOC3
//! and IMPACT HQ4).
//!
//! The bus is slot-level and instant. A master hands it one bit slot at a
//! time (`Bus::reset`, `Bus::slot`) and gets back the level on the wire;
//! the time a slot takes is the master's business, not the bus's. Parts
//! answer through the `Part` trait: a 64-bit ROM and a reply stream per
//! memory function command. `slot` turns the low-drive time of SGI's
//! MicroLAN register into a reset or a bit slot.

pub mod bus;
pub mod crc;
pub mod ds2502;
pub mod ds2505;
pub mod sgi;
pub mod slot;

pub use bus::{Bus, Part};
pub use ds2502::Ds2502;
pub use ds2505::Ds2505;

#[cfg(test)]
mod tests;
