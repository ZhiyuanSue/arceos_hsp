use riscv::register::satp;

use axconfig::{PHYS_VIRT_OFFSET, TASK_STACK_SIZE};

use core::ptr::read_volatile;
use crate::console::putchar;

/*
const SBI_CONSOLE_GETCHAR: usize = 1;

unsafe fn console_getchar() {
    return sbi_call(SBI_CONSOLE_GETCHAR);
}
fn sbi_call(which: usize) {
    unsafe {
        // 加了下面一行才可以在华山派上输出
        core::arch::asm!("ecall", in("x17") which);
    }
}*/

// 下面是最新版本的代码，通过利用arceos本身提供的函数进行输出，同时保证了华山派输出的正确性
// 一直没明白的是为什么需要先手动输出才能进行正常输出
unsafe fn console_putchar() {
    // 输出 hello from sbi
    putchar(10);
    putchar(72);
    putchar(69);
    putchar(76);
    putchar(76);
    putchar(79);
    putchar(32);
    putchar(70);
    putchar(82);
    putchar(79);
    putchar(77);
    putchar(32);
    putchar(83);
    putchar(66);
    putchar(73);
    putchar(10);
}

#[link_section = ".bss.stack"]
static mut BOOT_STACK: [u8; TASK_STACK_SIZE] = [0; TASK_STACK_SIZE];

#[link_section = ".data.boot_page_table"]
static mut BOOT_PT_SV39: [u64; 512] = [0; 512];

unsafe fn init_boot_page_table() {
    read_volatile(0x7000_0000 as *mut u32);
    // 0x8000_0000..0xc000_0000, VRWX_GAD, 1G block
    BOOT_PT_SV39[2] = (0x80000 << 10) | (0x6 << 60) | 0xef;
    // 0xffff_ffc0_8000_0000..0xffff_ffc0_c000_0000, VRWX_GAD, 1G block
    BOOT_PT_SV39[0x102] = (0x80000 << 10) | (0x6 << 60) | 0xef;
}

unsafe fn init_mmu() {
    let page_table_root = BOOT_PT_SV39.as_ptr() as usize;
    satp::set(satp::Mode::Sv39, 0, page_table_root >> 12);
    riscv::asm::sfence_vma_all();
}

/// The earliest entry point for the primary CPU.
#[naked]
#[no_mangle]
#[link_section = ".text.boot"]
unsafe extern "C" fn _start() -> ! {
    // PC = 0x8020_0000
    // a0 = hartid
    // a1 = dtb
    core::arch::asm!("
        mv      s0, a0                  // save hartid
        mv      s1, a1                  // save DTB pointer
        la      sp, {boot_stack}
        li      t0, {boot_stack_size}
        add     sp, sp, t0              // setup boot stack

        call    {init_boot_page_table}
        call    {console_putchar}
        call    {init_mmu}              // setup boot page table and enabel MMU

        li      s2, {phys_virt_offset}  // fix up virtual high address
        add     sp, sp, s2

        mv      a0, s0
        mv      a1, s1
        la      a2, {entry}
        add     a2, a2, s2
        jalr    a2                      // call rust_entry(hartid, dtb)
        j       .",
        phys_virt_offset = const PHYS_VIRT_OFFSET,
        boot_stack_size = const TASK_STACK_SIZE,
        boot_stack = sym BOOT_STACK,
        init_boot_page_table = sym init_boot_page_table,
        init_mmu = sym init_mmu,
        entry = sym super::rust_entry,
        console_putchar = sym console_putchar,
        options(noreturn),
    )
}

/// The earliest entry point for secondary CPUs.
#[cfg(feature = "smp")]
#[naked]
#[no_mangle]
#[link_section = ".text.boot"]
unsafe extern "C" fn _start_secondary() -> ! {
    // a0 = hartid
    // a1 = SP
    core::arch::asm!("
        mv      s0, a0                  // save hartid
        mv      sp, a1                  // set SP

        call    {init_mmu}              // setup boot page table and enabel MMU

        li      s1, {phys_virt_offset}  // fix up virtual high address
        add     a1, a1, s1
        add     sp, sp, s1

        mv      a0, s0
        la      a1, {entry}
        add     a1, a1, s1
        jalr    a1                      // call rust_entry_secondary(hartid)
        j       .",
        phys_virt_offset = const PHYS_VIRT_OFFSET,
        init_mmu = sym init_mmu,
        entry = sym super::rust_entry_secondary,
        options(noreturn),
    )
}
