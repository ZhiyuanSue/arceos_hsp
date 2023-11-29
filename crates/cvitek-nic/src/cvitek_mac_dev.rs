use super::cvitek_defs::*;
use alloc::boxed::Box;
use alloc::slice;
use alloc::sync::Arc;
use alloc::{collections::VecDeque, vec::Vec};
use core::marker::PhantomData;
use core::ptr::{NonNull, read_volatile, write_volatile};
use core::time::Duration;
use core::{mem, ptr};

pub type IrqHandler = fn();

use cvitek_phy::{CvitekPhyDevice, CvitekPhyTraits};

use super::cvitek_defs::*;
pub struct CvitekNicDevice<A: CvitekNicTraits,B: CvitekPhyTraits> {
    iobase_pa: usize,
    iobase_va: usize,
    rx_rings: RxRing<A>,
    tx_rings: TxRing<A>,
    phy:CvitekPhyDevice<B>,
    phantom: PhantomData<A>,
}

pub fn receive_irq_handler()
{
    info!("receive a package");
}

impl <A: CvitekNicTraits,B: CvitekPhyTraits> CvitekNicDevice<A,B> {
    pub fn new(iobase_pa: usize,phy:CvitekPhyDevice<B>) -> Self {
        let rx_ring = RxRing::<A>::new();
        let tx_ring = TxRing::<A>::new();
        let iobase_va = A::phys_to_virt(iobase_pa);
        let mut nic = CvitekNicDevice::<A,B> {
            iobase_pa,
            iobase_va,
            rx_rings: rx_ring,
            tx_rings: tx_ring,
            phy:phy,
            phantom: PhantomData,
        };
        A::register_irq(GMAC_IRQ,receive_irq_handler);
        nic.init();
        nic
    }

    pub fn init(&mut self) {
        // reset mac
        info!("try to reset dma!");
        let start_time:usize =A::current_time();
        unsafe{
            let mut val=read_volatile((self.iobase_va+GMAC_DMA_REG_BUS_MODE) as *mut u32);
            write_volatile((self.iobase_va+GMAC_DMA_REG_BUS_MODE) as *mut u32, val | DMAMAC_SRST);
            val=read_volatile((self.iobase_va+GMAC_DMA_REG_BUS_MODE) as *mut u32);
            while (val &DMAMAC_SRST)!=0{
                val=read_volatile((self.iobase_va+GMAC_DMA_REG_BUS_MODE) as *mut u32);
                if (A::current_time()-start_time)>=CONFIG_MDIO_TIMEOUT {
                    info!("DMA reset timeout\n");
                }
                A::mdelay(100);
            }
        }
        info!("finish try to reset dma!");
        // alloc rx_ring and tx_ring
        self.rx_rings.init_dma_desc_rings(0x9000_0000);
        self.tx_rings.init_dma_desc_rings(0x9001_0000);

        unsafe{
            // rx
            write_volatile((self.iobase_va + GMAC_DMA_REG_RXDESCLISTADDR) as *mut u32, 0x89000000);
            // tx
            write_volatile((self.iobase_va + GMAC_DMA_REG_TXDESCLISTADDR) as *mut u32, 0x89000000 + 0x3000);

            let macid_lo = 0xddccbbaa;
            let macid_hi = 0x00000605;
            write_volatile((self.iobase_va  + GMAC_REG_MACADDR0HI) as *mut u32, macid_hi);
            write_volatile((self.iobase_va  + GMAC_REG_MACADDR0LO) as *mut u32, macid_lo);

            write_volatile((self.iobase_va ) as *mut u32, 0x207);
            
            stmmac_set_mac(self.iobase_va , true);

            write_volatile((self.iobase_va+GMAC_DMA_REG_OPMODE) as *mut u32, 0x2202906);
            write_volatile((self.iobase_va+GMAC_DMA_REG_BUS_MODE) as *mut u32, 0x3900800);
            write_volatile((self.iobase_va+GMAC_REG_FLOWCONTROL) as *mut u32, 0xffff000e);
            write_volatile((self.iobase_va+GMAC_REG_CONF) as *mut u32, 0x41cc0c);
        }
        
        info!("init tx and rxring\n");
    }
    pub fn read_mac_address(&self) -> [u8; 6]
    {
        let mut ret:[u8;6]=[0; 6];
        unsafe{
            let hi=read_volatile((self.iobase_va + GMAC_REG_MACADDR0HI) as *mut u32);
            let lo=read_volatile((self.iobase_va + GMAC_REG_MACADDR0LO) as *mut u32);
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

        info!("run into the receive");
        for i in 0..10 {
            A::mdelay(2000);

            for i in 0..5 {
                let rd = self.rx_rings.rd.read_volatile(i).unwrap();
                let rdes0 = rd.txrx_status;
                let rdes1 = rd.dmamac_cntl;
                let rdes2 = rd.dmamac_addr;
                let rdes3 = rd.dmamac_cntl;
                let mut status = rdes0 & (1 << 31);
                if status >> 31 == 1 {
                    info!("dma own");
                    return None;
                }

                log::info!("rd {:x?}", rd);
            }
            let value: u32 = unsafe{
                read_volatile((self.iobase_va + 0x104c) as *mut u32)
            };
            log::info!("Current Host rx descriptor -----{:#x?}", value);
        }

        /*-------------*/

        let mut rx_rings = &mut self.rx_rings;
        let rd_dma = &mut rx_rings.rd;

        let mut status = 0;
        let mut idx = rx_rings.idx;
        let mut clean_idx = 0;

        let rd = rd_dma.read_volatile(idx).unwrap();
        let rdes0 = rd.txrx_status;
        let rdes1 = rd.dmamac_cntl;
        let rdes2 = rd.dmamac_addr;
        let rdes3 = rd.dmamac_cntl;

        status = rdes0 & (1 << 31);

        if status >> 31 == 1 {
            info!("dma own");
            return None;
        }

        // good frame
        // clean_idx = idx;
        let frame_len = rdes1 ;

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

        let x: &mut [u8] = &mut [
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xaa, 0xbb, 0xcc, 0xdd, 0x05, 0x06, 0x08, 0x06, 0x00,0x01, 
            0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xaa, 0xbb, 0xcc, 0xdd, 0x05, 0x06, 
            0xc0, 0xa8, 0x01, 0x16, 
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
            0xc0, 0xa8, 0x01, 0x08, 
            0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ];

        let tskb_start = 0x9001_0000 as usize;

        for i in 0..512{
            let buff_addr = tskb_start + 0x1000 * i;
            let raw_pointer = x.as_mut_ptr();
            let packet_pa: usize = tskb_start + 0x1000 * i;
            let packet_va = A::phys_to_virt(packet_pa);
            let buff = packet_va as *mut u8;
            unsafe {
                core::ptr::copy_nonoverlapping(raw_pointer as *const u8, buff as *mut u8, 0x2a);
            }
            let mut td = self.tx_rings.td.read_volatile(i).unwrap();

            td.dmamac_addr = buff_addr as u32;
            td.dmamac_cntl = 0x60000156;
            // td.txrx_status |= 1 << 29;
            // td.txrx_status |= 1 << 28;
            td.txrx_status |= 1 << 31;
            self.tx_rings.td.write_volatile(i, &td);
            unsafe{
                core::arch::asm!("fence	ow,ow");
            }
            unsafe{
                write_volatile((self.iobase_va + DMA_XMT_POLL_DEMAND) as *mut u32, 0x1);
            }
            A::mdelay(10000);   
            let value = unsafe{
                read_volatile((self.iobase_va + 0x1048) as *mut u32)
            };


            log::info!("Current Host tx descriptor -----{:#x?}", value);

            let intr_status = unsafe{
                read_volatile((self.iobase_va + 0x1014) as *mut u32)
            };
            let state = (intr_status & 0x00700000) >> 20;
            log::info!("tx state{:?}", state);

            loop{
                let mut td = self.tx_rings.td.read_volatile(i).unwrap();
                log::info!("td {:x?}", td);    
                if td.txrx_status & ( 1 << 31) == 0{
                    break;
                }
            }
        }
    }

}

#[derive(Copy, Clone, Debug)]
#[repr(C, packed)]
pub struct Des {
    pub txrx_status: u32,
    pub dmamac_cntl: u32,
    pub dmamac_addr: u32,
    pub dmamac_next: u32,
}

pub struct RxRing<A: CvitekNicTraits> {
    pub idx: usize,
    pub skbuf: Vec<usize>,
    pub rd: Dma<Des>,
    phantom: PhantomData<A>,
}

impl<A:CvitekNicTraits> RxRing<A> {
    pub fn new() -> Self {
        let count = CONFIG_RX_DESCR_NUM;
        let size = mem::size_of::<Des>() * count;
        let pa = 0x89000000 as usize;
        let va = A::phys_to_virt(pa);

        info!("rx des  pa: {:#x?} end {:#x?}", pa, pa + size);
        let rd_dma = Dma::new(va as _, pa, count);
        let skbuf: Vec<usize> = Vec::new();
        Self {
            rd: rd_dma,
            phantom: PhantomData,
            idx: 0,
            skbuf: skbuf,
        }
    }

    pub fn init_dma_desc_rings(&mut self, rskb_start: usize) {
        info!("rx init dma_desc_rings");
        for i in 0..16 {
            let buff_addr = rskb_start + 0x1000 * i;
            self.set_idx_addr_owner(i, buff_addr);
        }        
    }
    /// Release the next RDes to the DMA engine
    pub fn set_idx_addr_owner(&mut self, idx: usize, skb_phys_addr: usize) {
        let mut rd = Des {
            txrx_status: 0,
            dmamac_cntl: 0,
            dmamac_addr: 0,
            dmamac_next: 0,
        };
        rd.txrx_status = 1 << 31 as u32;

        rd.dmamac_cntl |= 0x600;

        rd.dmamac_addr = skb_phys_addr as u32;

        self.rd.write_volatile(idx, &rd);

    }

    pub fn set_head_tail_ptr(&mut self, iobase: usize) {
        
        let rd_addr = self.rd.phy_addr as usize;

    }
}

pub struct TxRing<A: CvitekNicTraits> {
    pub idx: usize,
    pub skbuf: Vec<usize>,
    pub td: Dma<Des>,
    phantom: PhantomData<A>,
}

impl<A: CvitekNicTraits> TxRing<A> {
    pub fn new() -> Self {
        let count = 512;

        let size = mem::size_of::<Des>() * count;
        let pa = 0x89000000 + 0x3000 as usize;
        let va = A::phys_to_virt(pa);

        info!("tx des  pa: {:#x?} end {:#x?}", pa, pa + size);
        let td_dma: Dma<Des> = Dma::new(va as _, pa, count);
        
        let skbuf = Vec::new();
        Self {
            td: td_dma,
            phantom: PhantomData,
            idx: 0,
            skbuf: skbuf,
        }
    }
    pub fn init_dma_desc_rings(&mut self, tskb_start: usize) {
        info!("tx set_idx_addr_owner");

        for i in 0..512 {
            self.init_tx_desc(i,  false);
        }
        self.init_tx_desc(511,  true);

    }

    pub fn init_tx_desc(&mut self, idx: usize, end:bool) {
        let mut td: Des = Des {
            txrx_status: 0,
            dmamac_cntl: 0,
            dmamac_addr: 0,
            dmamac_next: 0,
        };

        td.txrx_status &= !(1 << 31);


        if end {
            td.txrx_status |= 1 << 21;
        }
    
        self.td.write_volatile(idx, &td);
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

        let mut td = Des {
            txrx_status: 0,
            dmamac_cntl: 0,
            dmamac_addr: 0,
            dmamac_next: 0,
        };

        td.txrx_status = skb_phys_addr as u32; // Buffer 1
        td.dmamac_cntl = ((skb_phys_addr >> 32) & 0xff) as u32; // Not used


        self.td.write_volatile(idx, &td);
    }

    pub fn set_tail_ptr(&mut self, iobase: usize) {
        let td_addr = self.td.phy_addr as usize;
        let idx = self.idx;
        info!("tx set_tail_ptr idx:{:?}", idx);
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

    fn current_time() -> usize;

    fn register_irq(irq_num: usize, handler: IrqHandler) -> bool;
}
pub fn stmmac_set_mac(ioaddr: usize, enable: bool) {
    let old_val: u32;
    let mut value: u32;

    old_val = unsafe { read_volatile(ioaddr as *mut u32) };
    value = old_val;

    if enable {
        value |= MAC_ENABLE_RX | MAC_ENABLE_TX;
    } else {
        value &= !(MAC_ENABLE_TX | MAC_ENABLE_RX);
    }

    if value != old_val {
        unsafe { write_volatile(ioaddr as *mut u32, value) }
    }
}


pub fn dump_reg(ioaddr: usize) {
    info!("------------------------------ dumpreg ------------------------------");
    for i in 0..23 {
        let value = unsafe { read_volatile((ioaddr + 0x00001000 + i * 4) as *mut u32) };
        info!("reg {:?} = {:#x?}", i, value);
    }
    info!("------------------------------ end dumpreg ------------------------------");
}