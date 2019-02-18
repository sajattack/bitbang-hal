use embedded_hal::spi::FullDuplex;
use embedded_hal::spi::Mode;
use embedded_hal::spi::Phase::*;
use embedded_hal::spi::Polarity::*;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::blocking::delay::DelayUs;
use crate::time::Hertz;
use nb;

#[derive(Debug)]
pub enum Error {
    Unimplemented,
}

pub struct SPI<Miso, Mosi, Sck, Delay>
where 
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Delay: DelayUs<u32>
{
    mode: Mode,
    miso: Miso,
    mosi: Mosi,
    sck: Sck,
    delay: Delay,
    half_delay_us: u32,
}

impl <Miso, Mosi, Sck, Delay> SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Delay: DelayUs<u32>
{
    pub fn new<F: Into<Hertz>>(
        freq: F,
        mode: Mode,
        miso: Miso,
        mosi: Mosi,
        sck: Sck,
        delay: Delay,
    ) -> Self {
        let hertz = freq.into().0;
        let mut half_delay_us = 500_000 / hertz; 
        // round up the delay (lower the baudrate) 
        if 500_000 % hertz != 0 {
            half_delay_us += 1;
        }
        SPI {
            mode: mode,
            miso: miso,
            mosi: mosi,
            sck: sck,
            delay: delay,
            half_delay_us: half_delay_us
        }
    }
}

impl<Miso, Mosi, Sck, Delay> FullDuplex<u8> for SPI<Miso, Mosi, Sck, Delay>
where 
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Delay: DelayUs<u32>
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        self.mosi.set_low();
        let mut data_in: u8 = 0;
        for _bit in 0..8 {
            if self.mode.phase == CaptureOnFirstTransition {
                if self.mode.polarity == IdleLow {
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_high();
                    if self.miso.is_high() {
                        data_in = (data_in << 1) | 1
                    } else {
                        data_in = data_in << 1
                    }
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_low();
                } else {
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_low();
                    if self.miso.is_high() {
                        data_in = (data_in << 1) | 1
                    } else {
                        data_in = data_in << 1
                    }
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_high();
                }
            } else {
                if self.mode.polarity == IdleLow {
                    self.sck.set_high();
                    self.delay.delay_us(self.half_delay_us);
                    if self.miso.is_high() {
                        data_in = (data_in << 1) | 1
                    } else {
                        data_in = data_in << 1
                    }
                    self.sck.set_low();
                    self.delay.delay_us(self.half_delay_us);
                } else {
                    self.sck.set_low();
                    self.delay.delay_us(self.half_delay_us);
                    if self.miso.is_high() {
                        data_in = (data_in << 1) | 1
                    } else {
                        data_in = data_in << 1
                    }
                    self.sck.set_high();
                    self.delay.delay_us(self.half_delay_us);
                }
            }
        }
        Ok(data_in)
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
            if self.mode.phase == CaptureOnFirstTransition {
                if self.mode.polarity == IdleLow {
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_high();
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_low();
                } else {
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_low();
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_high();
                }
            } else {
                if self.mode.polarity == IdleLow {
                    self.sck.set_high();
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_low();
                    self.delay.delay_us(self.half_delay_us);
                } else {
                    self.sck.set_low();
                    self.delay.delay_us(self.half_delay_us);
                    self.sck.set_high();
                    self.delay.delay_us(self.half_delay_us);
                }
            }
            data_out <<= 1;
        }
        Ok(())
    }
}
