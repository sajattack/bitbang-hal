#![no_std]
#![no_main]

#[allow(unused)]
use panic_halt;

use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::timer::TimerCounter;
use hal::{entry, CorePeripherals, Peripherals};
use metro_m4 as hal;
use nb::block;

use bitbang_hal::spi::MODE_0;
use bitbang_hal::spi::SPI;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );

    let gclk0 = clocks.gclk0();
    let timer_clock = clocks.tc2_tc3(&gclk0).unwrap();
    let mut timer = TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.MCLK);
    timer.start(6.mhz()); // results in a SPI frequency of 3MHz

    let mut pins = hal::Pins::new(peripherals.PORT);
    let miso = pins.miso.into_pull_up_input(&mut pins.port);
    let mosi = pins.mosi.into_push_pull_output(&mut pins.port);
    let sck = pins.sck.into_push_pull_output(&mut pins.port);

    let mut spi = SPI::new(MODE_0, miso, mosi, sck, timer);
    let mut delay = Delay::new(core.SYST, &mut clocks);

    loop {
        for byte in b"Hello, World!" {
            block!(spi.send(*byte)).unwrap();
        }
        delay.delay_ms(1000u16);
    }
}
