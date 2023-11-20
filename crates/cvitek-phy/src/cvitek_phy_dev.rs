pub use core::marker::PhantomData;
use smoltcp::phy;

use super::cvitek_defs::*;
use core::ptr::{write_volatile,read_volatile};
pub struct CvitekPhyDevice<A: CvitekPhyTraits>
{
    base_addr:usize,
    phy_addr:u32,
    phantom: PhantomData<A>,
}
impl<A: CvitekPhyTraits> CvitekPhyDevice<A>
{
    pub fn new(base_addr:usize)->Self{
        let mut phymask:u32= 0xffffffff;
        let phy_if_mode= 0;
        let mut phy= CvitekPhyDevice { 
            base_addr:A::phys_to_virt(base_addr),
            phy_addr:0,
            phantom: PhantomData 
        };
        phy.eth_config();
        let _ =phy.get_phy_by_mask(phymask);
        phy.reset();
        phy.configure();
        phy
    }
    pub fn configure(&self)
    {
        let mut val_base:u32=0;
        let mut val:u32=0;
        let mut tmp_val:u32=0;
        info!("CvitekPhy configure");
        unsafe{ write_volatile(A::phys_to_virt(0x03009804) as *mut u32, 0x0001) };
        //MII-page5
        unsafe{ write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0500) };
        unsafe{ write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x0c00) };
        unsafe{ write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x0c7e) };
        unsafe{ write_volatile(A::phys_to_virt(0x03009800) as *mut u32, 0x0906) };
        //MII-page5
        unsafe{ write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0500) };
        val_base= unsafe { read_volatile( A::phys_to_virt(EPHY_EFUSE_VALID_BIT_BASE as usize) as *mut u32 ) };
        val = val_base & EPHY_EFUSE_TXITUNE_FLAG;
        if val == EPHY_EFUSE_TXITUNE_FLAG
        {
            tmp_val=unsafe{read_volatile(A::phys_to_virt(0x03051024) as *mut u32)};
            val= ( (tmp_val >> 24) & 0xFF ) | ( ((tmp_val >> 16)& 0xFF) << 8 );
            self.clrsetbits(0x03009064, 0xFFFF, val);
        }
        else {
            unsafe{write_volatile(A::phys_to_virt(0x03009064) as *mut u32, 0x5a5a);}
        }
        val_base= unsafe { read_volatile( A::phys_to_virt(EPHY_EFUSE_VALID_BIT_BASE as usize) as *mut u32 ) };
        val = val_base & EPHY_EFUSE_EXECHORC_FLAG;
        if val == EPHY_EFUSE_EXECHORC_FLAG
        {
            tmp_val = unsafe{ read_volatile(A::phys_to_virt(0x03051024) as *mut u32)};
            val = (( tmp_val >> 8 ) & 0xFF)<<8;
            self.clrsetbits(0x03009054, 0xFF00, val);
        }
        else {
            unsafe{write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0x0000);}
        }
        val_base= unsafe { read_volatile( A::phys_to_virt(EPHY_EFUSE_VALID_BIT_BASE as usize) as *mut u32 ) };
        val = val_base & EPHY_EFUSE_TXRXTERM_FLAG;
        if val == EPHY_EFUSE_TXRXTERM_FLAG
        {
            tmp_val = unsafe{ read_volatile(A::phys_to_virt(0x03051020) as *mut u32)};
            val = (((tmp_val>>28) & 0xF) << 4) | ( ((tmp_val>>24) & 0xF )<<8 )  ;
            self.clrsetbits(0x03009058, 0xFF0, val);
        }
        else {
            unsafe{write_volatile(A::phys_to_virt(0x03009058) as *mut u32, 0x0bb0);}
        }
        unsafe{
            write_volatile(A::phys_to_virt(0x0300905c) as *mut u32, 0x0c10);
            write_volatile(A::phys_to_virt(0x03009068) as *mut u32, 0x0003);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0x0000);
            //MII-page16
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x1000);
            write_volatile(A::phys_to_virt(0x03009068) as *mut u32, 0x1000);
            write_volatile(A::phys_to_virt(0x0300906c) as *mut u32, 0x3020);
            write_volatile(A::phys_to_virt(0x03009070) as *mut u32, 0x5040);
            write_volatile(A::phys_to_virt(0x03009074) as *mut u32, 0x7060);
            write_volatile(A::phys_to_virt(0x03009058) as *mut u32, 0x1708);
            write_volatile(A::phys_to_virt(0x0300905c) as *mut u32, 0x3827);
            write_volatile(A::phys_to_virt(0x03009060) as *mut u32, 0x5748);
            write_volatile(A::phys_to_virt(0x03009064) as *mut u32, 0x7867);
            //MII-page17
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x1100);
            write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x9080);
            write_volatile(A::phys_to_virt(0x03009044) as *mut u32, 0xb0a0);
            write_volatile(A::phys_to_virt(0x03009048) as *mut u32, 0xd0c0);
            write_volatile(A::phys_to_virt(0x0300904c) as *mut u32, 0xf0e0);
            write_volatile(A::phys_to_virt(0x03009050) as *mut u32, 0x9788);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0xb8a7);
            write_volatile(A::phys_to_virt(0x03009058) as *mut u32, 0xd7c8);
            write_volatile(A::phys_to_virt(0x0300905c) as *mut u32, 0xf7c8);
            //MII-page5
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0500);
            val= read_volatile(A::phys_to_virt(0x03009040) as *mut u32) | 0x0001;
            write_volatile(A::phys_to_virt(0x03009040) as *mut u32, val);
            //MII-page10
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0a00);
            write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x2000);
            write_volatile(A::phys_to_virt(0x03009044) as *mut u32, 0x3832);
            write_volatile(A::phys_to_virt(0x03009048) as *mut u32, 0x3132);
            write_volatile(A::phys_to_virt(0x0300904c) as *mut u32, 0x2d2f);
            write_volatile(A::phys_to_virt(0x03009050) as *mut u32, 0x2c2d);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0x1b2b);
            write_volatile(A::phys_to_virt(0x03009058) as *mut u32, 0x94a0);
            write_volatile(A::phys_to_virt(0x0300905c) as *mut u32, 0x8990);
            write_volatile(A::phys_to_virt(0x03009060) as *mut u32, 0x8788);
            write_volatile(A::phys_to_virt(0x03009064) as *mut u32, 0x8485);
            write_volatile(A::phys_to_virt(0x03009068) as *mut u32, 0x8283);
            write_volatile(A::phys_to_virt(0x0300906c) as *mut u32, 0x8182);
            write_volatile(A::phys_to_virt(0x03009070) as *mut u32, 0x0081);
            //MII-page11
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0b00);
            write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x5252);
            write_volatile(A::phys_to_virt(0x03009044) as *mut u32, 0x5252);
            write_volatile(A::phys_to_virt(0x03009048) as *mut u32, 0x4B52);
            write_volatile(A::phys_to_virt(0x0300904c) as *mut u32, 0x3D47);
            write_volatile(A::phys_to_virt(0x03009050) as *mut u32, 0xAA99);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0x989E);
            write_volatile(A::phys_to_virt(0x03009058) as *mut u32, 0x9395);
            write_volatile(A::phys_to_virt(0x0300905C) as *mut u32, 0x9091);
            write_volatile(A::phys_to_virt(0x03009060) as *mut u32, 0x8E8F);
            write_volatile(A::phys_to_virt(0x03009064) as *mut u32, 0x8D8E);
            write_volatile(A::phys_to_virt(0x03009068) as *mut u32, 0x8C8C);
            write_volatile(A::phys_to_virt(0x0300906c) as *mut u32, 0x8B8B);
            write_volatile(A::phys_to_virt(0x03009070) as *mut u32, 0x008A);
            //MII-page13
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0d00);
            write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x1E0A);
            write_volatile(A::phys_to_virt(0x03009044) as *mut u32, 0x3862);
            write_volatile(A::phys_to_virt(0x03009048) as *mut u32, 0x1E62);
            write_volatile(A::phys_to_virt(0x0300904c) as *mut u32, 0x2A08);
            write_volatile(A::phys_to_virt(0x03009050) as *mut u32, 0x244C);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0x1A44);
            write_volatile(A::phys_to_virt(0x03009058) as *mut u32, 0x061C);
            //MII-page14
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0e00);
            write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x2D30);
            write_volatile(A::phys_to_virt(0x03009044) as *mut u32, 0x3470);
            write_volatile(A::phys_to_virt(0x03009048) as *mut u32, 0x0648);
            write_volatile(A::phys_to_virt(0x0300904c) as *mut u32, 0x261C);
            write_volatile(A::phys_to_virt(0x03009050) as *mut u32, 0x3160);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0x2D5E);
            //MII-page15
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0f00);
            write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x2922);
            write_volatile(A::phys_to_virt(0x03009044) as *mut u32, 0x366E);
            write_volatile(A::phys_to_virt(0x03009048) as *mut u32, 0x0752);
            write_volatile(A::phys_to_virt(0x0300904c) as *mut u32, 0x2556);
            write_volatile(A::phys_to_virt(0x03009050) as *mut u32, 0x2348);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0x0C30);
            //MII-page16
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x1000);
            write_volatile(A::phys_to_virt(0x03009040) as *mut u32, 0x1E08);
            write_volatile(A::phys_to_virt(0x03009044) as *mut u32, 0x3868);
            write_volatile(A::phys_to_virt(0x03009048) as *mut u32, 0x1462);
            write_volatile(A::phys_to_virt(0x0300904c) as *mut u32, 0x1A0E);
            write_volatile(A::phys_to_virt(0x03009050) as *mut u32, 0x305E);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0x2F62);

            write_volatile(A::phys_to_virt(0x030010E0) as *mut u32, 0x05);
            write_volatile(A::phys_to_virt(0x030010E4) as *mut u32, 0x05);

            write_volatile(A::phys_to_virt(0x050270b0) as *mut u32, 0x11111111);
            write_volatile(A::phys_to_virt(0x050270b4) as *mut u32, 0x11111111);
            //MII-page1
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0100);
            val= read_volatile(A::phys_to_virt(0x03009068) as *mut u32) & !0x0f00;
            write_volatile(A::phys_to_virt(0x03009068) as *mut u32, val);
            //MII-page0
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0000);
            write_volatile(A::phys_to_virt(0x03009008) as *mut u32, 0x0043);
            write_volatile(A::phys_to_virt(0x0300900c) as *mut u32, 0x5649);
            //MII-page19
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x1300);
            write_volatile(A::phys_to_virt(0x03009058) as *mut u32, 0x0012);
            write_volatile(A::phys_to_virt(0x0300905c) as *mut u32, 0x6848);
            //MII-page18
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x1200);
            write_volatile(A::phys_to_virt(0x03009048) as *mut u32, 0x0808);
            write_volatile(A::phys_to_virt(0x0300904c) as *mut u32, 0x0808);

            write_volatile(A::phys_to_virt(0x03009050) as *mut u32, 0x32f8);
            write_volatile(A::phys_to_virt(0x03009054) as *mut u32, 0xf8dc);
            //MII-page0
            write_volatile(A::phys_to_virt(0x0300907c) as *mut u32, 0x0000);
            write_volatile(A::phys_to_virt(0x03009800) as *mut u32, 0x090E);

            write_volatile(A::phys_to_virt(0x03009804) as *mut u32, 0x0000);
        }
        info!("finish configure cvitekphy");
    }
    pub fn phy_read(&self,phy_addr:u8,reg_addr:u8) -> Result<u16,i32>{
        let mut miiaddr=(( phy_addr as u32) << MIIADDRSHIFT) & MIIADDRMASK;
        miiaddr |= ((reg_addr as u32)<<MIIREGSHIFT) & MIIREGMASK;
        miiaddr |= MII_CLKRANGE_150_250M;
        miiaddr |= MII_BUSY;
        unsafe{
            write_volatile((self.base_addr+GMAC_REG_MIIADDR) as *mut u32, miiaddr);
        }
        let start_time:usize =A::current_time();
        while (A::current_time()-start_time) < CONFIG_MDIO_TIMEOUT  {
            let mut val= unsafe{read_volatile((self.base_addr+GMAC_REG_MIIADDR) as *mut u32)};
            if (val & MII_BUSY)==0{
                unsafe{
                    val=read_volatile((self.base_addr+GMAC_REG_MIIDATA) as *mut u32);
                }
                return Ok(val as u16);
            }
            A::mdelay(1);
        }
        Err(-1)
    }
    pub fn phy_write(&self,phy_addr:u8,reg_addr:u8,data:u16) -> Result<i32,i32>{
        let mut miiaddr=(( phy_addr as u32) << MIIADDRSHIFT) & MIIADDRMASK;
        miiaddr |= ((reg_addr as u32)<<MIIREGSHIFT) & MIIREGMASK | MII_WRITE;
        miiaddr |= MII_CLKRANGE_150_250M;
        miiaddr |= MII_BUSY;

        unsafe{
            write_volatile((self.base_addr+GMAC_REG_MIIDATA) as *mut u32, data as u32);
            write_volatile((self.base_addr+GMAC_REG_MIIADDR) as *mut u32, miiaddr);
        }
        let start_time:usize =A::current_time();
        while (A::current_time()-start_time) < CONFIG_MDIO_TIMEOUT  {
            let mut val= unsafe{read_volatile((self.base_addr+GMAC_REG_MIIADDR) as *mut u32)};
            if (val & MII_BUSY)==0{
                return Ok(0);
            }
            A::mdelay(1);
        }

        Err(-1)
    }
    /*in alios, the start and update link only read the phy regs,so I do nothing*/
    pub fn start(&self){

    }
    pub fn stop(&self){
        
    }
    fn reset(&self){
        let mut timeout=600;
        let mut data:u16=0;
        let data=self.phy_read(self.phy_addr as u8, CVI_MII_BMCR).unwrap();
        let mut ret =self.phy_write(self.phy_addr as u8, CVI_MII_BMCR, data|CVI_BMCR_RESET);
        match ret {
            Ok(_r) => {},
            Err(_r) =>{
                info!("PHY soft reset failed\n");
                return;
            }
        }
        'loop_check:loop
        {
            let ret =self.phy_read(self.phy_addr as u8, CVI_MII_BMCR);
            match ret {
                Ok(r) => {
                    if timeout==0 || (r & CVI_BMCR_RESET)==0
                    {
                        break 'loop_check;
                    }
                },
                Err(_r) =>{
                    info!("PHY soft reset failed\n");
                    return;
                }
            }
            A::mdelay(1);
            timeout-=1;
        }
        let ret =self.phy_read(self.phy_addr as u8, CVI_MII_BMCR);
        match ret {
            Ok(r) => {
                if r & CVI_BMCR_RESET != 0
                {
                    info!("PHY soft reset timeout\n");
                }
                return;
            },
            Err(_r) =>{
                info!("PHY soft reset failed\n");
                return;
            }
        }
    }
    fn get_phy_by_mask(&mut self,phy_mask: u32 ) -> Result<i32, i32>
    {
        let mut mask:u32=phy_mask;
        let mut phy_id:u32=0xffffffff;
        while mask != 0 {
            let addr:i32=ffs(phy_mask)-1;
            let ret=self.read_phy_id(addr as u8);
            match ret{
                Ok(phy_id)=>{
                    if (phy_id & 0x1fffffff)!=0x1fffffff {
                        self.phy_addr=addr as u32;
                        return Ok(0);
                    }
                },
                Err(_e) => {}
            }
            mask &= !(1 << addr);
        }
        
        Err(-1)
    }
    fn read_phy_id(&self,phy_addr:u8) -> Result<u32,i32>
    {
        let mut data:u16=0;
        let mut id:u32=0;

        let mut res=self.phy_read(phy_addr, CVI_MII_PHYSID1);

        match res {
            Ok(d) =>{ data =d;},
            Err(ret) => { return Err(ret); }
        }
        id = data as u32;
        id = (id & 0xffff) << 16;

        res=self.phy_read(phy_addr, CVI_MII_PHYSID2);
        
        match res {
            Ok(d) =>{ data =d;},
            Err(ret) => { return Err(ret); }
        }

        id |= data as u32 & 0xffff;

        Ok(id)
    }
    fn clrsetbits(&self,physical_addr: u32,clear:u32, set:u32){
        unsafe{ 
            let mut val:u32=read_volatile(A::phys_to_virt(physical_addr as usize) as *mut u32);
            val &= !clear;
            write_volatile(A::phys_to_virt(physical_addr as usize) as *mut u32, val); 
        }
    }
    fn eth_config(&self)
    {
        unsafe{
            let val:u32=read_volatile(A::phys_to_virt(ETH_PHY_BASE & ETH_PHY_INIT_MASK) as *mut u32);
            let mut tmp_value=((val as usize)|ETH_PHY_SHUTDOWN)&ETH_PHY_RESET;
            write_volatile(A::phys_to_virt(ETH_PHY_BASE) as *mut u32, tmp_value as u32);
            A::mdelay(1);
            tmp_value=((val as usize)&ETH_PHY_POWERUP)&ETH_PHY_RESET;
            write_volatile(A::phys_to_virt(ETH_PHY_BASE) as *mut u32, tmp_value as u32);
            A::mdelay(20);
            tmp_value=((val as usize)&ETH_PHY_POWERUP)|ETH_PHY_RESET;
            write_volatile(A::phys_to_virt(ETH_PHY_BASE) as *mut u32, tmp_value as u32);
            A::mdelay(1);
        }
    }
}
pub trait CvitekPhyTraits {
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
}

pub fn ffs(phy_mask:u32)->i32{
    
    let table:[u8;256] =
	[
	  0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
	  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8
    ];
    let i:i32=phy_mask as i32;
    let mut a:u32=0;
    let x:u32 = i as u32 & (-i) as u32;

    if x<=0xffff{
        if x <= 0xff {
            a=0;
        }
        else {
            a=8;
        }
    }
    else{
        if x <= 0xffffff{
            a=16;
        }
        else {
            a=24;
        }
    }
    let index:usize=(x>>a) as usize;
    return (table[index] as u32 + a) as i32;
}