//! Serial communication (USART)
//!
//! This implementation consumes the following hardware resources:
//! - Periodic timer to mark clock cycles
//! - Output GPIO pin for transmission (TX)
//! - Input GPIO pin for reception (RX)
//!
//! The timer must be configured to twice the desired communication frequency.
//!

use core::fmt;
use core::fmt::Debug;
use embedded_hal::{
    digital::{InputPin, OutputPin},
};
use embedded_hal_async::{
    delay::DelayNs,
};
use embedded_io_async::ErrorKind;
use fugit::{Duration, TimerRate};

/// Serial communication error type
#[derive(Debug)]
pub enum Error<E> {
    /// Bus error
    Bus(E),
}

impl<E: core::fmt::Debug> embedded_io_async::Error for Error<E> {
    fn kind(&self) -> ErrorKind {
        match self {
            Error::Bus(_) => ErrorKind::Other
        }
    }
}

/// Bit banging serial communication (USART) device
pub struct Serial<TX, RX, Timer>
where
    TX: OutputPin,
    RX: InputPin,
    Timer: DelayNs,
{
    tx: TX,
    rx: RX,
    timer: Timer,
    clock_duration: Duration<u32, 1, 1_000_000>,
}

impl<TX, RX, Timer, E> Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: DelayNs,
{
    /// Create instance
    pub fn new(tx: TX, rx: RX, timer: Timer, frequency: TimerRate<u32, 1_000_000>) -> Self {
        Serial {
            tx,
            rx,
            timer,
            clock_duration: frequency.into_duration(),
        }
    }

    #[inline]
    async fn wait_for_timer(&mut self) {
        self.timer.delay_ns(self.clock_duration.to_nanos()).await;
    }
}

impl<TX, RX, Timer, E> embedded_io_async::ErrorType for Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: DelayNs,
    E: fmt::Debug
{
    type Error = crate::serial::Error<E>;
}


impl<TX, RX, Timer, E> Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: DelayNs,
    E: core::fmt::Debug
{
    async fn write_byte(&mut self, byte: u8) -> Result<(), crate::serial::Error<E>> {
        let mut data_out = byte;
        self.tx.set_low().map_err(Error::Bus)?; // start bit
        self.wait_for_timer().await;
        for _bit in 0..8 {
            if data_out & 1 == 1 {
                self.tx.set_high().map_err(Error::Bus)?;
            } else {
                self.tx.set_low().map_err(Error::Bus)?;
            }
            data_out >>= 1;
            self.wait_for_timer().await;
        }
        self.tx.set_high().map_err(Error::Bus)?; // stop bit
        self.wait_for_timer().await;
        Ok(())
    }

    async fn read_byte(&mut self) -> Result<u8, crate::serial::Error<E>> {
        let mut data_in = 0;
        // wait for start bit
        while self.rx.is_high().map_err(Error::Bus)? {}
        self.wait_for_timer().await;
        for _bit in 0..8 {
            data_in <<= 1;
            if self.rx.is_high().map_err(Error::Bus)? {
                data_in |= 1
            }
            self.wait_for_timer().await;
        }
        // wait for stop bit
        self.wait_for_timer().await;
        Ok(data_in)
    }
}
impl<TX, RX, Timer, E> embedded_io_async::Write for Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: DelayNs,
    E: core::fmt::Debug
{
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for byte in buf {
            self.write_byte(*byte).await?;
        }
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<TX, RX, Timer, E> embedded_io_async::Read for Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: DelayNs,
    E: core::fmt::Debug
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        for byte in buf.iter_mut() {
            *byte = self.read_byte().await?;
        }
        Ok(buf.len())
    }
}
