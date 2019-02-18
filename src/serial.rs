use embedded_hal::digital::{OutputPin, InputPin};
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::serial;
use crate::time::Hertz;

pub struct Serial<TX, RX, Delay> 
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
        let mut data_out = byte;
        self.tx.set_low(); // start bit
        self.delay.delay_us(self.delay_time);
        for _bit in 0..8 {
            if data_out & 1 == 1 {
                self.tx.set_high();
            } else {
                self.tx.set_low();
            }
            data_out >>= 1;
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

impl <TX, RX, Delay> serial::Read<u8> for Serial <TX, RX, Delay>
where 
    TX: OutputPin,
    RX: InputPin,
    Delay: DelayUs<u32> 
{

    type Error = ();

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut data_in = 0;
        // wait for start bit
        while self.rx.is_high() {}
        // catch the middle of the first bit
        self.delay.delay_us((self.delay_time as f32 * 1.5) as u32); 
        for _bit in 0..8 {
            data_in <<= 1;
            if self.rx.is_high() {
               data_in |= 1
            }
            self.delay.delay_us(self.delay_time);
        }
        // wait for stop bit
        self.delay.delay_us(self.delay_time);
        Ok(data_in)
    }
}
