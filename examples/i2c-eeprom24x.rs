#![no_std]
#![no_main]

use nb::block;
use panic_halt as _;

use eeprom24x::Eeprom24x;
use eeprom24x::SlaveAddr;

use cortex_m_rt::entry;
use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::{prelude::*, stm32};

#[entry]
fn main() -> ! {
    let pdev = stm32::Peripherals::take().unwrap();

    let mut flash = pdev.FLASH.constrain();
    let mut rcc = pdev.RCC.constrain();
    let mut gpioa = pdev.GPIOA.split(&mut rcc.apb2);

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(32.mhz())
        .pclk1(16.mhz())
        .freeze(&mut flash.acr);

    let mut delay = Timer::tim2(pdev.TIM2, &clocks, &mut rcc.apb1).start_count_down(10.hz());
    let tmr = Timer::tim3(pdev.TIM3, &clocks, &mut rcc.apb1).start_count_down(200.khz());
    let scl = gpioa.pa1.into_open_drain_output(&mut gpioa.crl);
    let sda = gpioa.pa2.into_open_drain_output(&mut gpioa.crl);

    let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);
    let mut eeprom = Eeprom24x::new_24x04(i2c, SlaveAddr::default());

    // check high memory addresses: 1 bit passed as a part of i2c addr
    let addrs: [u32; 4] = [0x100, 0x10F, 0x1F0, 0x1EE];
    let byte = 0xe5;

    for addr in addrs.iter() {
        eeprom.write_byte(*addr, byte).unwrap();
        // need to wait before next write
        block!(delay.wait()).ok();
    }

    loop {
        for addr in addrs.iter() {
            let _ = eeprom.read_byte(*addr).unwrap();
            block!(delay.wait()).ok();
        }
    }
}
