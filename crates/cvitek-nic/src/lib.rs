#![no_std]
#![allow(dead_code)]

extern crate alloc;
#[macro_use]
extern crate log;

mod cvitek_defs;
mod cvitek_mac_dev;

pub use cvitek_mac_dev::CvitekNicDevice;
pub use cvitek_mac_dev::CvitekNicTraits;
pub use cvitek_mac_dev::Packet;
// pub use cvitek_main::Dma;

pub use cvitek_phy::{CvitekPhyDevice, CvitekPhyTraits};


pub struct RxBuffer {
    pub packet: Packet,
}

impl RxBuffer {
    /// Return packet as &[u8].
    pub fn packet(&self) -> &[u8] {
        self.packet.as_bytes()
    }

    /// Return mutuable packet as &mut [u8].
    pub fn packet_mut(&mut self) -> &mut [u8] {
        self.packet.as_mut_bytes()
    }
}


pub struct TxBuffer {
    pub packet: Packet,
}

impl TxBuffer {
    /// Returns an unmutuable packet buffer.
    pub fn packet(&self) -> &[u8] {
        self.packet.as_bytes()
    }

    /// Returns a mutuable packet buffer.
    pub fn packet_mut(&mut self) -> &mut [u8] {
        self.packet.as_mut_bytes()
    }
}