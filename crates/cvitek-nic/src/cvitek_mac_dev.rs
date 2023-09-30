use super::cvitek_defs::*;
use alloc::boxed::Box;
use alloc::slice;
use alloc::sync::Arc;
use alloc::{collections::VecDeque, vec::Vec};
use core::marker::PhantomData;
use core::ptr::{NonNull, read_volatile, write_volatile};
use core::time::Duration;
use core::{mem, ptr};

use super::cvitek_defs::*;
pub struct CvitekNicDevice<A: CvitekNicTraits> {
    reg_base_addr:usize,
    iobase_pa: usize,
    iobase_va: usize,
    rx_rings: RxRing<A>,
    tx_rings: TxRing<A>,
    phantom: PhantomData<A>,
}

impl <A: CvitekNicTraits> CvitekNicDevice<A> {
    pub fn new(iobase_pa: usize) -> Self {
        let rx_ring = RxRing::<A>::new();
        let tx_ring = TxRing::<A>::new();
        let iobase_va = A::phys_to_virt(iobase_pa);
        let mut nic = CvitekNicDevice::<A> {
            iobase_pa,
            iobase_va,
            reg_base_addr:GMAC0_REG_BASE_ADDR,
            rx_rings: rx_ring,
            tx_rings: tx_ring,
            phantom: PhantomData,
        };
        nic.init();
        nic
    }

    pub fn init(&mut self) {


        // alloc rx_ring and tx_ring
        self.rx_rings.init_dma_desc_rings();
        self.tx_rings.init_dma_desc_rings();
        info!("init tx and rxring\n");
    }
    pub fn read_mac_address(&self) -> [u8; 6]
    {
        let mut ret:[u8;6]=[0; 6];
        unsafe{
            let hi=read_volatile((self.reg_base_addr + GMAC_REG_MACADDR0HI) as *mut u32);
            let lo=read_volatile((self.reg_base_addr + GMAC_REG_MACADDR0LO) as *mut u32);
            ret[0]=(lo & 0xff) as u8;
            ret[1]=((lo>>8)& 0xff) as u8;
            ret[2]=((lo>>16)& 0xff) as u8;
            ret[3]=((lo>>24)& 0xff) as u8;
            ret[4]=(hi& 0xff) as u8;
            ret[5]=((hi>>8)& 0xff) as u8;
        }
        ret
    }


    pub fn get_tx_idx(&self) -> usize {
        let idx = self.tx_rings.idx;
        idx
    }

    pub fn receive(&mut self) -> Option<Packet> {
        let mut rx_rings = &mut self.rx_rings;
        let rd_dma = &mut rx_rings.rd;

        let mut status = 0;
        let mut idx = rx_rings.idx;
        let mut clean_idx = 0;

        let rd = rd_dma.read_volatile(idx).unwrap();
        let rdes0 = rd.rdes0;
        let rdes1 = rd.rdes1;
        let rdes2 = rd.rdes2;
        let rdes3 = rd.rdes3;

        status = rdes0 & (1 << 31);

        if status >> 31 == 1 {
            info!("dma own");
            return None;
        }

        // good frame
        // clean_idx = idx;
        let frame_len = rdes3 & EMAC_RDES3_PL;

        // get data from skb
        let skb_va = rx_rings.skbuf[idx] as *mut u8;
        let packet = Packet::new(skb_va, frame_len as usize);

        // alloc new skbuf
        // let (skb_new_va, skb_new_pa) = A::dma_alloc_pages(1);
        // rx_rings.set_idx_addr_owner(clean_idx, skb_new_pa);
        // rx_rings.skbuf[idx] = skb_new_va as _;

        rx_rings.idx = (idx + 1) % 512;
        return Some(packet);
    }

    pub fn transmit(&mut self, packet: Packet) {
        let packet_va = packet.skb_va as usize;
        let packet_pa = A::virt_to_phys(packet_va);
        let packet_len = packet.len as usize;
        let tx_rings: &mut TxRing<A> = &mut self.tx_rings;
        let idx: usize = tx_rings.idx;

        tx_rings.set_idx_addr_owner(idx, true, true, false, true, packet_pa, packet_len);

        tx_rings.idx = (idx + 1) % 512;

        tx_rings.set_tail_ptr(self.iobase_va);
    }

}

#[derive(Copy, Clone, Debug)]
#[repr(C, packed)]
pub struct RxDes {
    pub rdes0: u32,
    pub rdes1: u32,
    pub rdes2: u32,
    pub rdes3: u32,
}

#[derive(Copy, Clone, Debug)]
#[repr(C, packed)]
pub struct TxDes {
    pub tdes0: u32,
    pub tdes1: u32,
    pub tdes2: u32,
    pub tdes3: u32,
}

pub struct RxRing<A: CvitekNicTraits> {
    pub idx: usize,
    pub skbuf: Vec<usize>,
    pub rd: Dma<RxDes>,
    phantom: PhantomData<A>,
}

impl<A:CvitekNicTraits> RxRing<A> {
    pub fn new() -> Self {
        let count = 512;
        let size = mem::size_of::<RxDes>() * count;
        let pa = 0x89000000 as usize;
        let va = A::phys_to_virt(pa);

        info!("rx des  pa: {:#x?} end {:#x?}", pa, pa + size);
        let rd_dma = Dma::new(va as _, pa, count);
        let skbuf = Vec::new();
        Self {
            rd: rd_dma,
            phantom: PhantomData,
            idx: 0,
            skbuf: skbuf,
        }
    }

    pub fn init_dma_desc_rings(&mut self) {
        info!("rx set_idx_addr_owner");
        
    }

    /// Release the next RDes to the DMA engine
    pub fn set_idx_addr_owner(&mut self, idx: usize, skb_phys_addr: usize) {
        let mut rd = RxDes {
            rdes0: 0,
            rdes1: 0,
            rdes2: 0,
            rdes3: 0,
        };

        // dwmac_desc_set_addr
        rd.rdes0 = skb_phys_addr as u32;
        rd.rdes1 = ((skb_phys_addr >> 32) & 0xFF) as u32;

        // dwmac_set_rx_owner
        rd.rdes3 |= RDES3_OWN | RDES3_BUFFER1_VALID_ADDR;
        // rd.rdes3 |= RDES3_INT_ON_COMPLETION_EN;

        self.rd.write_volatile(idx, &rd);

        /*unsafe {
            core::arch::asm!("dsb sy");
        }*/
    }

    pub fn set_head_tail_ptr(&mut self, iobase: usize) {
        
        let rd_addr = self.rd.phy_addr as usize;

        unsafe {
            write_volatile(
                (iobase + DMA_CHAN_RX_BASE_ADDR_HI) as *mut u32,
                (rd_addr >> 32) as u32,
            );
            write_volatile(
                (iobase + DMA_CHAN_RX_BASE_ADDR) as *mut u32,
                (rd_addr & 0xFFFFFFFF) as u32,
            );

            write_volatile(
                (iobase + DMA_CHAN_RX_END_ADDR) as *mut u32,
                rd_addr as u32 + (511 * core::mem::size_of::<RxDes>()) as u32,
            );
        }
    }
}

pub struct TxRing<A: CvitekNicTraits> {
    pub idx: usize,
    pub skbuf: Vec<usize>,
    pub td: Dma<TxDes>,
    phantom: PhantomData<A>,
}

impl<A: CvitekNicTraits> TxRing<A> {
    pub fn new() -> Self {
        let count = 512;

        let size = mem::size_of::<TxDes>() * count;
        let pa = 0x89000000 + 0x3000 as usize;
        let va = A::phys_to_virt(pa);

        info!("tx des  pa: {:#x?} end {:#x?}", pa, pa + size);
        let td_dma: Dma<TxDes> = Dma::new(va as _, pa, count);
        
        let skbuf = Vec::new();
        Self {
            td: td_dma,
            phantom: PhantomData,
            idx: 0,
            skbuf: skbuf,
        }
    }
    pub fn init_dma_desc_rings(&mut self) {
        info!("tx set_idx_addr_owner");
    }

    pub fn set_idx_addr_owner(
        &mut self,
        idx: usize,
        fs: bool,
        ls: bool,
        csum: bool,
        own: bool,
        skb_phys_addr: usize,
        len: usize,
    ) {
        let skb_va = A::phys_to_virt(skb_phys_addr);
        self.skbuf.push(skb_va);
        let td = self.td.read_volatile(idx).unwrap();

        assert!(td.tdes3 & EMAC_DES3_OWN == 0);

        let mut td = TxDes {
            tdes0: 0,
            tdes1: 0,
            tdes2: 0,
            tdes3: 0,
        };

        td.tdes0 = skb_phys_addr as u32; // Buffer 1
        td.tdes1 = ((skb_phys_addr >> 32) & 0xff) as u32; // Not used

        td.tdes2 = (len as u32) & EMAC_TDES2_B1L;

        td.tdes3 |= EMAC_DES3_FD; // FD: Contains first buffer of packet
        td.tdes3 |= EMAC_DES3_LD; // LD: Contains last buffer of packet
        td.tdes3 |= EMAC_DES3_OWN; // Give the DMA engine ownership
        td.tdes3 |= (len as u32) & EMAC_TDES3_PL;

        self.td.write_volatile(idx, &td);
    }

    pub fn set_tail_ptr(&mut self, iobase: usize) {
        let td_addr = self.td.phy_addr as usize;
        let idx = self.idx;
        info!("tx set_tail_ptr idx:{:?}", idx);
        unsafe {
            write_volatile(
                (iobase + DMA_CHAN_TX_END_ADDR) as *mut u32,
                td_addr as u32 + (idx * core::mem::size_of::<TxDes>()) as u32,
            );
        }
    }
}

pub struct Dma<T> {
    pub count: usize,
    pub phy_addr: usize,
    pub cpu_addr: *mut T,
}

impl<T> Dma<T> {
    pub fn new(cpu_addr: *mut T, phy_addr: usize, count: usize) -> Self {
        Self {
            count: count,
            phy_addr: phy_addr,
            cpu_addr: cpu_addr,
        }
    }

    pub fn read_volatile(&self, index: usize) -> Option<T> {
        if index >= self.count {
            // pr_info!("read_volatile index:{:?} count:{:?}", index, self.count);
            return None;
        }

        let ptr = self.cpu_addr.wrapping_add(index);

        // SAFETY: We just checked that the index is within bounds.
        Some(unsafe { ptr.read() })
    }

    pub fn write_volatile(&self, index: usize, value: &T) -> bool
    where
        T: Copy,
    {
        if index >= self.count {
            // pr_info!("read_volatile index:{:?} count:{:?}", index, self.count);
            return false;
        }

        let ptr = self.cpu_addr.wrapping_add(index);

        // SAFETY: We just checked that the index is within bounds.
        unsafe { ptr.write(*value) };
        true
    }
}

pub struct Packet {
    pub skb_va: *mut u8,
    pub len: usize,
}

impl Packet {
    pub fn new(skb_va: *mut u8, len: usize) -> Self {
        Self {
            skb_va: skb_va,
            len: len,
        }
    }

    /// Returns all data in the buffer, not including header.
    pub fn as_bytes(&self) -> &[u8] {
        unsafe { slice::from_raw_parts(self.skb_va, self.len) }
    }

    /// Returns all data in the buffer with the mutuable reference,
    /// not including header.
    pub fn as_mut_bytes(&mut self) -> &mut [u8] {
        unsafe { slice::from_raw_parts_mut(self.skb_va, self.len) }
    }
}

pub trait CvitekNicTraits {
    fn phys_to_virt(pa: usize) -> usize {
        pa
    }
    fn virt_to_phys(va: usize) -> usize {
        va
    }
    fn dma_alloc_pages(pages: usize) -> (usize, usize);

    fn dma_free_pages(vaddr: usize, pages: usize);

    fn mdelay(m_times: usize);
}