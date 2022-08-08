#![no_std]
#![no_main]

use nb::block;
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{prelude::*, stm32};

#[entry]
fn main() -> ! {
    let pdev = stm32::Peripherals::take().unwrap();

    let mut flash = pdev.FLASH.constrain();
    let rcc = pdev.RCC.constrain();
    let mut gpiob = pdev.GPIOB.split();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(32.MHz())
        .pclk1(16.MHz())
        .freeze(&mut flash.acr);

    let mut delay = pdev.TIM2.delay_us(&clocks);
    let mut tmr = pdev.TIM3.counter_hz(&clocks);
    tmr.start(115_200.Hz()).unwrap();

    // use 5V tolerant pins to test with UART-to-USB connector
    let tx = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);
    let rx = gpiob.pb11.into_floating_input(&mut gpiob.crh);

    let mut serial = bitbang_hal::serial::Serial::new(tx, rx, tmr);

    loop {
        for byte in b"Hello, World!\r\n" {
            block!(serial.write(*byte)).unwrap();
        }

        delay.delay_ms(1000u16);
    }
}
