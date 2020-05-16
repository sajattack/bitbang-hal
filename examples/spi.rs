#![no_std]
#![no_main]

use nb::block;
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::{prelude::*, stm32};

use bitbang_hal::spi::MODE_0;
use bitbang_hal::spi::SPI;

#[entry]
fn main() -> ! {
    let pdev = stm32::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();

    let mut flash = pdev.FLASH.constrain();
    let mut rcc = pdev.RCC.constrain();
    let mut gpioa = pdev.GPIOA.split(&mut rcc.apb2);

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(32.mhz())
        .pclk1(16.mhz())
        .freeze(&mut flash.acr);

    let mut delay = Delay::new(core.SYST, clocks);
    let tmr = Timer::tim3(pdev.TIM3, &clocks, &mut rcc.apb1).start_count_down(6.mhz());

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
