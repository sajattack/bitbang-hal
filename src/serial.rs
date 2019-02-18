use embedded_hal::digital::{OutputPin, InputPin};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::serial;
use embedded_hal::serial::{Write, Read};
use crate::time::Hertz;

pub struct Serial<TX, /*RX,*/ Delay> 
where 
    TX: OutputPin,
    RX: InputPin,
    Delay: DelayUs<u32>,
{
    delay_time: u32,
    tx: TX,
    rx: RX,
    delay: Delay,
}

impl <TX, RX, Delay> Serial <TX, RX, Delay>
where 
    TX: OutputPin,
    RX: InputPin,
    Delay: DelayUs<u32>
{
    pub fn new<F: Into<Hertz>>(
        baud: F,
        tx: TX,
        rx: RX,
        delay: Delay
    ) -> Self {
       let delay_time = 1_000_000 / (baud.into().0);
          Serial {
              delay_time: delay_time,
              tx: tx,
              rx: rx,
              delay: delay
        }
    }
}

impl <TX, RX, Delay> serial::Write<u8> for Serial <TX, RX, Delay>
where 
    TX: OutputPin,
    RX: InputPin,
    Delay: DelayUs<u32> 
{

    type Error = ();

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        let mut out_byte = byte;
        self.tx.set_low(); // start bit
        self.delay.delay_us(self.delay_time);
        for _bit in 0..8 {
            if out_byte & 1 == 1 {
                self.tx.set_high();
            } else {
                self.tx.set_low();
            }
            out_byte >>= 1;
            self.delay.delay_us(self.delay_time);
        }
        self.tx.set_high(); // stop bit
        self.delay.delay_us(self.delay_time);
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        Ok(())
    }
}
