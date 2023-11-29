//! Defines types and probe methods of all supported devices.

#![allow(unused_imports)]

use crate::AxDeviceEnum;
use driver_common::DeviceType;
use driver_net::CvitekNicTraits;

#[cfg(feature = "virtio")]
use crate::virtio::{self, VirtIoDevMeta};

#[cfg(feature = "bus-pci")]
use driver_pci::{DeviceFunction, DeviceFunctionInfo, PciRoot};

pub use super::dummy::*;

pub trait DriverProbe {
    fn probe_global() -> Option<AxDeviceEnum> {
        None
    }

    #[cfg(bus = "mmio")]
    fn probe_mmio(_mmio_base: usize, _mmio_size: usize) -> Option<AxDeviceEnum> {
        None
    }

    #[cfg(bus = "pci")]
    fn probe_pci(
        _root: &mut PciRoot,
        _bdf: DeviceFunction,
        _dev_info: &DeviceFunctionInfo,
    ) -> Option<AxDeviceEnum> {
        None
    }
}

#[cfg(net_dev = "virtio-net")]
register_net_driver!(
    <virtio::VirtIoNet as VirtIoDevMeta>::Driver,
    <virtio::VirtIoNet as VirtIoDevMeta>::Device
);

#[cfg(block_dev = "virtio-blk")]
register_block_driver!(
    <virtio::VirtIoBlk as VirtIoDevMeta>::Driver,
    <virtio::VirtIoBlk as VirtIoDevMeta>::Device
);

#[cfg(display_dev = "virtio-gpu")]
register_display_driver!(
    <virtio::VirtIoGpu as VirtIoDevMeta>::Driver,
    <virtio::VirtIoGpu as VirtIoDevMeta>::Device
);

cfg_if::cfg_if! {
    if #[cfg(block_dev = "ramdisk")] {
        pub struct RamDiskDriver;
        register_block_driver!(RamDiskDriver, driver_block::ramdisk::RamDisk);

        impl DriverProbe for RamDiskDriver {
            fn probe_global() -> Option<AxDeviceEnum> {
                // TODO: format RAM disk
                Some(AxDeviceEnum::from_block(
                    driver_block::ramdisk::RamDisk::new(0x100_0000), // 16 MiB
                ))
            }
        }
    }
}
cfg_if::cfg_if! {
    if #[cfg(phy_dev = "cvitekphy")] {
        use super::CvitekPhyTraitsImpl;
        pub struct CvitekPhyDriver;
        register_phy_driver!(CvitekPhyDriver, driver_net::cvitekphy::CvitekPhy<CvitekPhyTraitsImpl>);

        impl DriverProbe for CvitekPhyDriver {
            fn probe_global() -> Option<AxDeviceEnum> {
                use driver_net::cvitekphy::CvitekPhy;
                let cvitek_phy = CvitekPhy::init(CvitekPhyTraitsImpl);
                return Some(AxDeviceEnum::from_phy(cvitek_phy));
            }
        }
    }
}
cfg_if::cfg_if! {
    if #[cfg(net_dev = "cviteknic")] {
        use super::CvitekNicTraitsImpl;
        pub struct CvitekNicDriver;
        register_net_driver!(CvitekNicDriver, driver_net::cviteknic::CvitekNic<CvitekNicTraitsImpl,CvitekPhyTraitsImpl>);

        impl DriverProbe for CvitekNicDriver {
            fn probe_global() -> Option<AxDeviceEnum> {
                use driver_net::cviteknic::CvitekNic;
                use driver_net::cvitekphy::CvitekPhy;
                let cvitek_phy = CvitekPhy::init(CvitekPhyTraitsImpl).device;
                let cvitek_nic = CvitekNic::init(CvitekNicTraitsImpl,cvitek_phy);
                return Some(AxDeviceEnum::from_net(cvitek_nic));
            }
        }
    }
}