#![no_std]
#![no_main]

use nb::block;
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{prelude::*, stm32};

use bitbang_hal::spi::MODE_0;
use bitbang_hal::spi::SPI;

#[entry]
fn main() -> ! {
    let pdev = stm32::Peripherals::take().unwrap();

    let mut flash = pdev.FLASH.constrain();
    let rcc = pdev.RCC.constrain();
    let mut gpioa = pdev.GPIOA.split();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(32.MHz())
        .pclk1(16.MHz())
        .freeze(&mut flash.acr);

    let mut delay = pdev.TIM2.delay_us(&clocks);
    let mut tmr = pdev.TIM3.counter_hz(&clocks);
    tmr.start(6.MHz()).unwrap();

    let miso = gpioa.pa0.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
    let sck = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);

    let mut spi = SPI::new(MODE_0, miso, mosi, sck, tmr);

    loop {
        for byte in b"Hello, World!" {
            block!(spi.send(*byte)).unwrap();
        }

        delay.delay_ms(1000u16);
    }
}
