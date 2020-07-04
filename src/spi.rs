//! Serial Peripheral Interface
//!
//! This implementation consumes the following hardware resources:
//! - Periodic timer to mark clock cycles
//! - Output GPIO pin for clock signal (SCLK)
//! - Output GPIO pin for data transmission (Master Output Slave Input - MOSI)
//! - Input GPIO pin for data reception (Master Input Slave Output - MISO)
//!
//! The timer must be configured to twice the desired communication frequency.
//!
//! SS/CS (slave select) must be handled independently.
//!
//! MSB-first and LSB-first bit orders are supported.
//!

pub use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3};

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{FullDuplex, Mode, Polarity};
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

/// Error type
#[derive(Debug)]
pub enum Error<E> {
    /// Communication error
    Bus(E),
    /// Attempted read without input data
    NoData,
}

/// Transmission bit order
#[derive(Debug)]
pub enum BitOrder {
    /// Most significant bit first
    MSBFirst,
    /// Least significant bit first
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

impl<Miso, Mosi, Sck, Timer, E> SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: CountDown + Periodic,
{
    /// Create instance
    pub fn new(mode: Mode, miso: Miso, mosi: Mosi, sck: Sck, timer: Timer) -> Self {
        let mut spi = SPI {
            mode,
            miso,
            mosi,
            sck,
            timer,
            read_val: None,
            bit_order: BitOrder::default(),
        };

        match mode.polarity {
            Polarity::IdleLow => spi.sck.try_set_low(),
            Polarity::IdleHigh => spi.sck.try_set_high(),
        }
        .unwrap_or(());

        spi
    }

    /// Set transmission bit order
    pub fn set_bit_order(&mut self, order: BitOrder) {
        self.bit_order = order;
    }

    /// Allows for an access to the timer type.
    /// This can be used to change the speed.
    ///
    /// In closure you get ownership of the timer
    /// so you can destruct it and build it up again if necessary.
    ///
    /// # Example
    ///
    /// ```Rust
    ///spi.access_timer(|mut timer| {
    ///    timer.set_freq(4.mhz());
    ///    timer
    ///});
    ///```
    ///
    pub fn access_timer<F>(&mut self, f: F)
    where
        F: FnOnce(Timer) -> Timer,
    {
        // Create a zeroed timer.
        // This is unsafe, but its safety is guaranteed, though, because the zeroed timer is never used.
        let timer = unsafe { core::mem::zeroed() };
        // Get the timer in the struct.
        let timer = core::mem::replace(&mut self.timer, timer);
        // Give the timer to the closure and put the result back into the struct.
        self.timer = f(timer);
    }

    fn read_bit(&mut self) -> nb::Result<(), crate::spi::Error<E>> {
        let is_miso_high = self.miso.try_is_high().map_err(Error::Bus)?;
        let shifted_value = self.read_val.unwrap_or(0) << 1;
        if is_miso_high {
            self.read_val = Some(shifted_value | 1);
        } else {
            self.read_val = Some(shifted_value);
        }
        Ok(())
    }

    #[inline]
    fn set_clk_high(&mut self) -> Result<(), crate::spi::Error<E>> {
        self.sck.try_set_high().map_err(Error::Bus)
    }

    #[inline]
    fn set_clk_low(&mut self) -> Result<(), crate::spi::Error<E>> {
        self.sck.try_set_low().map_err(Error::Bus)
    }

    #[inline]
    fn wait_for_timer(&mut self) {
        block!(self.timer.try_wait()).ok();
    }
}

impl<Miso, Mosi, Sck, Timer, E> FullDuplex<u8> for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: CountDown + Periodic,
{
    type Error = crate::spi::Error<E>;

    #[inline]
    fn try_read(&mut self) -> nb::Result<u8, Self::Error> {
        match self.read_val {
            Some(val) => Ok(val),
            None => Err(nb::Error::Other(crate::spi::Error::NoData)),
        }
    }

    fn try_send(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        for bit_offset in 0..8 {
            let out_bit = match self.bit_order {
                BitOrder::MSBFirst => (byte >> (7 - bit_offset)) & 0b1,
                BitOrder::LSBFirst => (byte >> bit_offset) & 0b1,
            };

            if out_bit == 1 {
                self.mosi.try_set_high().map_err(Error::Bus)?;
            } else {
                self.mosi.try_set_low().map_err(Error::Bus)?;
            }

            match self.mode {
                MODE_0 => {
                    self.wait_for_timer();
                    self.set_clk_high()?;
                    self.read_bit()?;
                    self.wait_for_timer();
                    self.set_clk_low()?;
                }
                MODE_1 => {
                    self.set_clk_high()?;
                    self.wait_for_timer();
                    self.read_bit()?;
                    self.set_clk_low()?;
                    self.wait_for_timer();
                }
                MODE_2 => {
                    self.wait_for_timer();
                    self.set_clk_low()?;
                    self.read_bit()?;
                    self.wait_for_timer();
                    self.set_clk_high()?;
                }
                MODE_3 => {
                    self.set_clk_low()?;
                    self.wait_for_timer();
                    self.read_bit()?;
                    self.set_clk_high()?;
                    self.wait_for_timer();
                }
            }
        }

        Ok(())
    }
}

impl<Miso, Mosi, Sck, Timer, E> embedded_hal::blocking::spi::transfer::Default<u8>
    for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: CountDown + Periodic,
{
}

impl<Miso, Mosi, Sck, Timer, E> embedded_hal::blocking::spi::write::Default<u8>
    for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Timer: CountDown + Periodic,
{
}
