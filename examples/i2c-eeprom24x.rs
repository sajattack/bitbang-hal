#![no_std]
#![no_main]

use nb::block;
use panic_halt as _;

use eeprom24x::Eeprom24x;
use eeprom24x::SlaveAddr;

use cortex_m_rt::entry;
use stm32f1xx_hal::{prelude::*, stm32};

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

    let mut delay = pdev.TIM2.counter_hz(&clocks);
    delay.start(10.Hz()).unwrap();
    let mut tmr = pdev.TIM3.counter_hz(&clocks);
    tmr.start(200.kHz()).unwrap();

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
