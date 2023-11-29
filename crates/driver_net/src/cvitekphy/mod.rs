pub use super::CvitekPhyTraits;
pub use super::CvitekPhyDevice;
use driver_common::BaseDriverOps;
use core::marker::PhantomData;
use crate::PhyDriverOps;

pub (crate) const GMAC_REG_BASE_ADDR: usize = 0x04070000;
pub struct CvitekPhy<A>
where
    A: CvitekPhyTraits,
{
    pub device: CvitekPhyDevice<A>,
    phantom: PhantomData<A>,
}

unsafe impl<A: CvitekPhyTraits> Sync for CvitekPhy<A> {}
unsafe impl<A: CvitekPhyTraits> Send for CvitekPhy<A> {}


impl<A> CvitekPhy<A>
where
    A: CvitekPhyTraits,
{
    pub fn init(traits_impl: A) -> Self {
        let device = CvitekPhyDevice::new(GMAC_REG_BASE_ADDR);
        CvitekPhy { 
            device: device,
            phantom: PhantomData
        }
    }
}


impl <A:CvitekPhyTraits> BaseDriverOps for CvitekPhy<A> {
    fn device_name(&self) -> &str {
        "cvitek_phy"
    }

    fn device_type(&self) -> driver_common::DeviceType {
        driver_common::DeviceType::Phy
    }
}

impl <A:CvitekPhyTraits> PhyDriverOps for CvitekPhy<A> {
    fn configure(&self) {
        info!("CvitekPhy configure");
        self.device.configure();
    }
    fn start(&self) {
        info!("CvitekPhy start");
        self.device.start();
    }
    fn stop(&self) {
        info!("CvitekPhy stop");
        self.device.stop();
    }
}
