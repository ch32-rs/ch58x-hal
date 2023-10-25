//! rt for CH58x

use core::arch::global_asm;

#[export_name = "error: riscv-rt appears more than once in the dependency graph"]
#[doc(hidden)]
pub static __ONCE__: () = ();

#[doc(hidden)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    reserved: usize,
}

#[doc(hidden)]
#[no_mangle]
#[allow(unused_variables, non_snake_case)]
pub fn DefaultInterruptHandler() {
    loop {
        // Prevent this from turning into a UDF instruction
        // see rust-lang/rust#28728 for details
        continue;
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub enum Interrupt {
    NonMaskableInt = 2,
    HardFault = 3,
    EcallM = 5,
    EcallU = 8,
    BreakPoint = 9,
    SysTick = 12,
    Software = 14,
    ///16 - TMR0_IRQHandler
    TMR0 = 16,
    ///17 - GPIOA_IRQHandler
    GPIOA = 17,
    ///18 - GPIOB_IRQHandler
    GPIOB = 18,
    ///19 - SPI0_IRQHandler
    SPI0 = 19,
    ///21 - LLE_IRQHandler
    BLEL = 20,
    ///20 - BB_IRQHandler
    BLEB = 21,
    ///22 - USB_IRQHandler
    USB = 22,
    ///24 - TMR1_IRQHandler
    TMR1 = 24,
    ///25 - TMR2_IRQHandler
    TMR2 = 25,
    ///26 - UART0_IRQHandler
    UART0 = 26,
    ///27 - UART1_IRQHandler
    UART1 = 27,
    ///28 - RTC_IRQHandler
    RTC = 28,
    ///29 - ADC_IRQHandler
    ADC = 29,
    ///30 - I2C_IRQHandler
    I2C = 30,
    ///31 - PPWMX_SPI1_IRQHandler
    PWMX = 31,
    ///32 - TMR3_IRQHandler
    TMR3 = 32,
    ///33 - UART2_IRQHandler
    UART2 = 33,
    ///34 - UART3_IRQHandler
    UART3 = 34,
    ///35 - WDOG_BAT_IRQHandler
    WDOG_BAT = 35,
}

// Overwrites PAC's interrupt handlers
extern "C" {
    fn Reset() -> !;

    fn NonMaskableInt();

    fn HardFault();

    fn EcallM();

    fn EcallU();

    fn BreakPoint();

    fn SysTick();

    fn Software();

    // External interrupts
    fn TMR0();
    fn GPIOA();
    fn GPIOB();
    fn SPI0();
    fn BLEB();
    fn BLEL();
    fn USB();
    fn TMR1();
    fn TMR2();
    fn UART0();
    fn UART1();
    fn RTC();
    fn ADC();
    fn I2C();
    fn PWMX();
    fn TMR3();
    fn UART2();
    fn UART3();
    fn WDOG_BAT();
}

#[doc(hidden)]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Vector; 36] = [
    Vector { reserved: 0 },
    Vector { reserved: 0 }, // reset?
    // 2: Non-Maskable Interrupt.
    Vector {
        handler: NonMaskableInt,
    },
    // 3: Hard Fault Interrupt.
    Vector { handler: HardFault },
    Vector { reserved: 0 }, // magic value from official SDK
    // 5: ECALL-M
    Vector { handler: EcallM },
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    // 8: ECALL-U
    Vector { handler: EcallU },
    // 9: Breakpoint
    Vector { handler: BreakPoint },
    // 10-11
    Vector { reserved: 0 },
    Vector { reserved: 0 },
    // 12
    Vector { handler: SysTick },
    Vector { reserved: 0 },
    Vector { handler: Software },
    Vector { reserved: 0 },
    // External interrupts
    // 15
    Vector { handler: TMR0 },
    Vector { handler: GPIOA },
    Vector { handler: GPIOB },
    Vector { handler: SPI0 },
    Vector { handler: BLEB },
    Vector { handler: BLEL },
    Vector { handler: USB },
    Vector { reserved: 0 },
    Vector { handler: TMR1 },
    Vector { handler: TMR2 },
    Vector { handler: UART0 },
    Vector { handler: UART1 },
    Vector { handler: RTC },
    Vector { handler: ADC },
    Vector { handler: I2C },
    Vector { handler: PWMX },
    Vector { handler: TMR3 },
    Vector { handler: UART2 },
    Vector { handler: UART3 },
    Vector { reserved: 0xaaaaaaaa },
];

macro_rules! cfg_global_asm {
    {@inner, [$($x:tt)*], } => {
        global_asm!{$($x)*}
    };
    (@inner, [$($x:tt)*], #[cfg($meta:meta)] $asm:literal, $($rest:tt)*) => {
        #[cfg($meta)]
        cfg_global_asm!{@inner, [$($x)* $asm,], $($rest)*}
        #[cfg(not($meta))]
        cfg_global_asm!{@inner, [$($x)*], $($rest)*}
    };
    {@inner, [$($x:tt)*], $asm:literal, $($rest:tt)*} => {
        cfg_global_asm!{@inner, [$($x)* $asm,], $($rest)*}
    };
    {$($asms:tt)*} => {
        cfg_global_asm!{@inner, [], $($asms)*}
    };
}

cfg_global_asm! {
    "
    .section    .init,\"ax\"
    .global _start
    .align  1
//    .option norvc
_start:
    j handle_reset
    ",
    "
    .section    .handle_reset,\"ax\",@progbits
    .weak   handle_reset
    .align  1
handle_reset:
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop
    la sp, _stack_top
    ",
    // load highcode from flash to ram
    "
    la a0, _highcode_lma
    la a1, _highcode_vma_start
    la a2, _highcode_vma_end
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b
    ",
    // load data from flash to ram
    "
2:
    la a0, _data_lma
    la a1, _data_vma
    la a2, _edata
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b
2:
    ",
    // clear bss section
    "
    la a0, _sbss
    la a1, _ebss
    bgeu a0, a1, 2f
1:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, 1b
2:
    ",
    // corecfgr: 流水线控制位 & 动态预测控制位
    // corecfgr: Pipeline control bit & Dynamic prediction control
    "
    li t0, 0x1f
    csrw 0xbc0, t0",
    // 打开嵌套中断、硬件压栈功能
    // intsyscr: Open nested interrupts and hardware stack functions
    // 0x3 both nested interrupts and hardware stack
    // 0x1 only hardware stack
    "
    li t0, 0x3
    csrw 0x804, t0",
    // Set mpp=3, return to machine mode
    // or use 0x88 to set mpp=0, return to user mode
    "
    li t0, 0x1888
    csrs mstatus, t0
    la t0, __vector_base
    ",
    // 配置向量表模式为绝对地址模式
    "
    ori t0, t0, 3
    csrw mtvec, t0
    ",
    "
    la t0, main
    csrw mepc, t0

    mret
    ",
}
