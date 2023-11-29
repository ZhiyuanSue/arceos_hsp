use alloc::sync::Arc;
use cvitek_nic::Packet;
use cvitek_phy::CvitekPhyTraits;
use driver_common::BaseDriverOps;
use driver_common::DevError;
use driver_common::DevResult;

use crate::NetDriverOps;
use crate::RxBuf;
use alloc::boxed::Box;
use core::any::Any;
use core::any::TypeId;

use core::marker::PhantomData;
use core::ptr;
use core::ptr::{read_volatile, write_volatile};

use crate::TxBuf;
use super::CvitekNicDevice;
pub use cvitek_phy::CvitekPhyDevice;

unsafe impl<A: CvitekNicTraits,B: CvitekPhyTraits> Sync for CvitekNic<A,B> {}
unsafe impl<A: CvitekNicTraits,B: CvitekPhyTraits> Send for CvitekNic<A,B> {}

pub use super::CvitekNicTraits;

pub struct CvitekNic<A,B>
where
    A: CvitekNicTraits,
    B: CvitekPhyTraits,
{
    device: CvitekNicDevice<A,B>,
    phantom: PhantomData<A>,
}

pub (crate) const GMAC_REG_BASE_ADDR: usize = 0x0407_0000;

impl <A,B> CvitekNic<A,B> 
where
    A: CvitekNicTraits,
    B: CvitekPhyTraits,
{
    pub fn init(traits_impl: A,phy: CvitekPhyDevice<B>) -> Self {
        let device = CvitekNicDevice::new(GMAC_REG_BASE_ADDR,phy);
        Self {
            device,
            phantom: PhantomData,
        }
    }
}

impl <A:CvitekNicTraits,B: CvitekPhyTraits> BaseDriverOps for CvitekNic<A,B> {
    fn device_name(&self) -> &str {
        "cvitek_nic"
    }

    fn device_type(&self) -> driver_common::DeviceType {
        driver_common::DeviceType::Net
    }
}

impl<A:CvitekNicTraits,B: CvitekPhyTraits> NetDriverOps for CvitekNic<A,B> {
    fn mac_address(&self) -> crate::EthernetAddress {
        crate::EthernetAddress(self.device.read_mac_address())
    }

    fn tx_queue_size(&self) -> usize {
        16
    }

    fn rx_queue_size(&self) -> usize {
        16
    }

    fn can_receive(&self) -> bool {
        true
    }

    fn can_transmit(&self) -> bool {
        true
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        Ok(())
    }

    fn fill_rx_buffers(&mut self, buf_pool: &crate::NetBufPool) -> DevResult {
        Ok(())
    }

    fn prepare_tx_buffer(&self, tx_buf: &mut crate::NetBuf, packet_len: usize) -> DevResult {
        Ok(())
    }

    fn alloc_tx_buffer(&self, size: usize) -> DevResult<TxBuf> {
        use cvitek_nic::TxBuffer;

        let idx = self.device.get_tx_idx();
        let skb_pa = 0x91000000 + idx * 0x1000;
        let skb_va = A::phys_to_virt(skb_pa);
        let new_va = skb_va + idx * 0x1000;
        let packet = Packet::new(new_va as *mut u8, size);
        let tx_buffer: TxBuffer = TxBuffer { packet };

        Ok(TxBuf::CvitekNic(tx_buffer))
    }

    fn recycle_rx_buffer(&mut self, rx_buf: crate::NetBufPtr) -> DevResult {
        Ok(())
    }

    fn transmit(&mut self, tx_buf: TxBuf) -> DevResult {
        match tx_buf {
            TxBuf::CvitekNic(tx_buf) => {
                self.device.transmit(tx_buf.packet);
                Ok(())
            }
            TxBuf::Virtio(_) => Err(DevError::BadState),
        }
    }

    fn receive(&mut self) -> DevResult<RxBuf> {
        use cvitek_nic::RxBuffer;
        if let Some(packet) = self.device.receive() {
            info!("rxbuf.packet = {:x?}", packet.as_bytes());
            let rxbuf = RxBuffer { packet };
            Ok(RxBuf::CvitekNic(rxbuf))
        } else {
            Err(DevError::Again)
        }
    }
}
