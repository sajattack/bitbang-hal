//! Serial communication (USART)
//!
//! This implementation consumes the following hardware resources:
//! - Periodic timer to mark clock cycles
//! - Output GPIO pin for transmission (TX)
//! - Input GPIO pin for reception (RX)
//!
//! The timer must be configured to twice the desired communication frequency.
//!

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::serial;
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

/// Serial communication error type
#[derive(Debug)]
pub enum Error<E> {
    /// Bus error
    Bus(E),
}

/// Bit banging serial communication (USART) device
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

impl<TX, RX, Timer, E> Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic,
{
    /// Create instance
    pub fn new(tx: TX, rx: RX, timer: Timer) -> Self {
        Serial { tx, rx, timer }
    }

    #[inline]
    fn wait_for_timer(&mut self) {
        block!(self.timer.try_wait()).ok();
    }
}

impl<TX, RX, Timer, E> serial::Write<u8> for Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic,
{
    type Error = crate::serial::Error<E>;

    fn try_write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        let mut data_out = byte;
        self.tx.try_set_low().map_err(Error::Bus)?; // start bit
        self.wait_for_timer();
        for _bit in 0..8 {
            if data_out & 1 == 1 {
                self.tx.try_set_high().map_err(Error::Bus)?;
            } else {
                self.tx.try_set_low().map_err(Error::Bus)?;
            }
            data_out >>= 1;
            self.wait_for_timer();
        }
        self.tx.try_set_high().map_err(Error::Bus)?; // stop bit
        self.wait_for_timer();
        Ok(())
    }

    fn try_flush(&mut self) -> nb::Result<(), Self::Error> {
        Ok(())
    }
}

impl<TX, RX, Timer, E> serial::Read<u8> for Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic,
{
    type Error = crate::serial::Error<E>;

    fn try_read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut data_in = 0;
        // wait for start bit
        while self.rx.try_is_high().map_err(Error::Bus)? {}
        self.wait_for_timer();
        for _bit in 0..8 {
            data_in <<= 1;
            if self.rx.try_is_high().map_err(Error::Bus)? {
                data_in |= 1
            }
            self.wait_for_timer();
        }
        // wait for stop bit
        self.wait_for_timer();
        Ok(data_in)
    }
}
