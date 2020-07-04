#![no_std]
#![no_main]

use nb::block;
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::{prelude::*, stm32};

#[entry]
fn main() -> ! {
    let pdev = stm32::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();

    let mut flash = pdev.FLASH.constrain();
    let mut rcc = pdev.RCC.constrain();
    let mut gpiob = pdev.GPIOB.split(&mut rcc.apb2);

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(32.mhz())
        .pclk1(16.mhz())
        .freeze(&mut flash.acr);

    let mut delay = Delay::new(core.SYST, clocks);
    let tmr = Timer::tim3(pdev.TIM3, &clocks, &mut rcc.apb1).start_count_down(115_200.hz());

    // use 5V tolerant pins to test with UART-to-USB connector
    let tx = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);
    let rx = gpiob.pb11.into_floating_input(&mut gpiob.crh);

    let mut serial = bitbang_hal::serial::Serial::new(tx, rx, tmr);

    loop {
        for byte in b"Hello, World!\r\n" {
            block!(serial.try_write(*byte)).unwrap();
        }

        delay.delay_ms(1000u16);
    }
}
