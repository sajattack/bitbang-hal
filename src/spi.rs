use embedded_hal::spi::{FullDuplex, Mode, Phase::*, Polarity::*};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

#[derive(Debug)]
pub enum Error {
    Unimplemented,
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
    read_val: u8, 
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
            read_val: 0,
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
        Ok(self.read_val)
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        let mut data_out = byte;
        for _bit in 0..8 {
            let out_bit = (data_out >> 7) & 1;
            if out_bit == 1 {
                self.mosi.set_high(); 
            } else {
                self.mosi.set_low();
            }
            if self.miso.is_high() {
                self.read_val = (self.read_val << 1) | 1
            } else {
                self.read_val = self.read_val << 1
            }
            if self.mode.phase == CaptureOnFirstTransition {
                if self.mode.polarity == IdleLow {
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_high();
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_low();
                } else {
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_low();
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_high();
                }
            } else {
                if self.mode.polarity == IdleLow {
                    self.sck.set_high();
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_low();
                    block!(self.timer.wait()).ok(); 
                } else {
                    self.sck.set_low();
                    block!(self.timer.wait()).ok(); 
                    self.sck.set_high();
                    block!(self.timer.wait()).ok(); 
                }
            }
            data_out <<= 1;
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
