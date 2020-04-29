//! Serial communication (USART)

use embedded_hal::digital::v2::{OutputPin, InputPin};
use embedded_hal::timer::{CountDown, Periodic};
use embedded_hal::serial;
use nb::block;

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
}

pub struct Serial<TX, RX, Timer> 
where 
    TX: OutputPin,
    RX: InputPin,
    Timer: CountDown + Periodic,
{
    tx: TX,
    rx: RX,
    timer: Timer, 
}

impl <TX, RX, Timer, E> Serial<TX, RX, Timer>
where 
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic 
{
    pub fn new(
        tx: TX,
        rx: RX,
        timer: Timer 
    ) -> Self {
          Serial {
              tx,
              rx,
              timer
        }
    }
}

impl <TX, RX, Timer, E> serial::Write<u8> for Serial <TX, RX, Timer>
where 
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic
{
    type Error = crate::serial::Error<E>;

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        let mut data_out = byte;
        self.tx.set_low().map_err(Error::Bus)?; // start bit
        block!(self.timer.wait()).ok(); 
        for _bit in 0..8 {
            if data_out & 1 == 1 {
                self.tx.set_high().map_err(Error::Bus)?;
            } else {
                self.tx.set_low().map_err(Error::Bus)?;
            }
            data_out >>= 1;
            block!(self.timer.wait()).ok(); 
        }
        self.tx.set_high().map_err(Error::Bus)?; // stop bit
        block!(self.timer.wait()).ok(); 
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        Ok(())
    }
}

impl <TX, RX, Timer, E> serial::Read<u8> for Serial <TX, RX, Timer>
where 
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic 
{
    type Error = crate::serial::Error<E>;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut data_in = 0;
        // wait for start bit
        while self.rx.is_high().map_err(Error::Bus)? {}
        block!(self.timer.wait()).ok(); 
        for _bit in 0..8 {
            data_in <<= 1;
            if self.rx.is_high().map_err(Error::Bus)? {
               data_in |= 1
            }
            block!(self.timer.wait()).ok(); 
        }
        // wait for stop bit
        block!(self.timer.wait()).ok(); 
        Ok(data_in)
    }
}
