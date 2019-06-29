pub use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3};

use embedded_hal::spi::{FullDuplex, Mode, Phase::*, Polarity::*};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

#[derive(Debug)]
pub enum Error {
    NoData,
}

#[derive(Debug)]
pub enum BitOrder {
    MSBFirst,
    LSBFirst,
}

impl Default for BitOrder {
    /// Default bit order: MSB first
    fn default() -> Self {
        BitOrder::MSBFirst
    }
}

/// A Full-Duplex SPI implementation, takes 3 pins, and a timer running at 2x
/// the desired SPI frequency.
pub struct SPI<Miso, Mosi, Sck, Timer>
where 
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Timer: CountDown + Periodic,
{
    mode: Mode,
    miso: Miso,
    mosi: Mosi,
    sck: Sck,
    timer: Timer,
    read_val: Option<u8>, 
    bit_order: BitOrder,
}

impl <Miso, Mosi, Sck, Timer> SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Timer: CountDown + Periodic,
{
    pub fn new(
        mode: Mode,
        miso: Miso,
        mosi: Mosi,
        sck: Sck,
        timer: Timer,
    ) -> Self {
        SPI {
            mode: mode,
            miso: miso,
            mosi: mosi,
            sck: sck,
            timer: timer,
            read_val: None,
            bit_order: BitOrder::default(),
        }
    }

    pub fn set_bit_order(&mut self, order: BitOrder) {
        self.bit_order = order;
    }

    fn read_bit(&mut self) {
        if self.miso.is_high() {
            self.read_val = Some((self.read_val.unwrap_or(0) << 1) | 1);
        } else {
            self.read_val = Some(self.read_val.unwrap_or(0) << 1);
        }
    }
}

impl<Miso, Mosi, Sck, Timer> FullDuplex<u8> for SPI<Miso, Mosi, Sck, Timer>
where 
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Timer: CountDown + Periodic
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        match self.read_val {
            Some(val) => Ok(val),
            None => Err(nb::Error::Other(Error::NoData))
        }
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        for bit in 0..8 {
            let out_bit = match self.bit_order {
                BitOrder::MSBFirst => (byte >> (7 - bit)) & 0b1,
                BitOrder::LSBFirst => (byte >> bit) & 0b1,
            };

            if out_bit == 1 {
                self.mosi.set_high(); 
            } else {
                self.mosi.set_low();
            }

            if self.mode.phase == CaptureOnFirstTransition {
                if self.mode.polarity == IdleLow {
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_high();
                    self.read_bit();
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_low();
                } else {
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_low();
                    self.read_bit();
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_high();
                }
            } else {
                if self.mode.polarity == IdleLow {
                    self.sck.set_high();
                    block!(self.timer.wait()).ok(); 
                    self.read_bit();
                    self.sck.set_low();
                    block!(self.timer.wait()).ok(); 
                } else {
                    self.sck.set_low();
                    block!(self.timer.wait()).ok(); 
                    self.read_bit();
                    self.sck.set_high();
                    block!(self.timer.wait()).ok(); 
                }
            }
        }
        Ok(())
    }
}
impl<Miso, Mosi, Sck, Timer> 
    embedded_hal::blocking::spi::transfer::Default<u8> 
    for SPI<Miso, Mosi, Sck, Timer>
where 
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Timer: CountDown + Periodic
{}
impl<Miso, Mosi, Sck, Timer> 
    embedded_hal::blocking::spi::write::Default<u8> 
    for SPI<Miso, Mosi, Sck, Timer>
where 
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Timer: CountDown + Periodic
{}
