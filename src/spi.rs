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

use core::cmp::max;
pub use embedded_hal_async::spi::{MODE_0, MODE_1, MODE_2, MODE_3};
use embedded_hal_async::{delay::DelayNs, spi::SpiBus};

use embedded_hal::{
    digital::{InputPin, OutputPin},
    spi::ErrorType,
};
use embedded_hal_async::spi::{Mode, Polarity};
use fugit::RateExtU32;

/// Error type
#[derive(Debug)]
pub enum Error<E> {
    /// Communication error
    Bus(E),
    /// Attempted read without input data
    NoData,
}

impl<E: core::fmt::Debug> embedded_hal::spi::Error for Error<E> {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match self {
            Error::Bus(_) => embedded_hal::spi::ErrorKind::Other,
            Error::NoData => embedded_hal::spi::ErrorKind::Other,
        }
    }
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

/// Configuration of the SPI Interface
pub struct SpiConfig {
    mode: Mode,
    bit_order: BitOrder,
    /// Value used when still receiving bits from MISO, but not value is available
    /// in buffer to be written to MOSI. This value is used instead. Usually `0x00`
    empty_write_value: u8,
    /// controls the clock speed. `f = 2 / half_period_duration_ns`
    half_period_duration_ns: u32,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: Default::default(),
            empty_write_value: 0x00,
            half_period_duration_ns: 10, // 100 kHz.
        }
    }
}

/// A Full-Duplex SPI implementation, takes 3 pins, and a timer running at 2x
/// the desired SPI frequency.
pub struct SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Delay: DelayNs,
{
    miso: Miso,
    mosi: Mosi,
    sck: Sck,
    delay: Delay,
    config: SpiConfig,
}

impl<Miso, Mosi, Sck, Delay, E> SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin<Error=E>,
    Mosi: OutputPin<Error=E>,
    Sck: OutputPin<Error=E>,
    Delay: DelayNs,
{
    /// Create instance
    pub fn new(
        mode: Mode,
        miso: Miso,
        mosi: Mosi,
        sck: Sck,
        delay: Delay,
        spi_config: SpiConfig,
    ) -> Self {
        let mut spi = SPI {
            miso,
            mosi,
            sck,
            delay,
            config: spi_config,
        };

        match mode.polarity {
            Polarity::IdleLow => spi.sck.set_low(),
            Polarity::IdleHigh => spi.sck.set_high(),
        }
            .unwrap_or(());

        spi
    }

    /// Set transmission bit order
    pub fn with_bit_order(&mut self, order: BitOrder) {
        self.config.bit_order = order;
    }

    async fn read_bit(&mut self, read_val: &mut u8) -> Result<(), crate::spi::Error<E>> {
        let is_miso_high = self.miso.is_high().map_err(Error::Bus)?;
        let shifted_value = *read_val << 1;
        if is_miso_high {
            *read_val = shifted_value | 1;
        } else {
            *read_val = shifted_value;
        }
        Ok(())
    }

    #[inline]
    fn set_clk_high(&mut self) -> Result<(), crate::spi::Error<E>> {
        self.sck.set_high().map_err(Error::Bus)
    }

    #[inline]
    fn set_clk_low(&mut self) -> Result<(), crate::spi::Error<E>> {
        self.sck.set_low().map_err(Error::Bus)
    }

    #[inline]
    async fn wait_for_timer(&mut self) {
        self.delay
            .delay_ns(self.config.half_period_duration_ns)
            .await;
    }

    #[inline]
    async fn rw_byte(
        &mut self,
        clock_out: u8,
        read_in: &mut u8,
    ) -> Result<(), crate::spi::Error<E>> {
        for bit_offset in 0..8 {
            let out_bit = match self.config.bit_order {
                BitOrder::MSBFirst => (clock_out >> (7 - bit_offset)) & 0b1,
                BitOrder::LSBFirst => (clock_out >> bit_offset) & 0b1,
            };

            println!("Write bit {} of {:b}", out_bit, clock_out);

            if out_bit == 1 {
                self.mosi.set_high().map_err(Error::Bus)?;
            } else {
                self.mosi.set_low().map_err(Error::Bus)?;
            }

            match self.config.mode {
                MODE_0 => {
                    self.wait_for_timer().await;
                    self.set_clk_high()?;
                    self.read_bit(read_in).await?;
                    self.wait_for_timer().await;
                    self.set_clk_low()?;
                }
                MODE_1 => {
                    self.set_clk_high()?;
                    self.wait_for_timer().await;
                    self.read_bit(read_in).await?;
                    self.set_clk_low()?;
                    self.wait_for_timer().await;
                }
                MODE_2 => {
                    self.wait_for_timer().await;
                    self.set_clk_low()?;
                    self.read_bit(read_in).await?;
                    self.wait_for_timer().await;
                    self.set_clk_high()?;
                }
                MODE_3 => {
                    self.set_clk_low()?;
                    self.wait_for_timer().await;
                    self.read_bit(read_in).await?;
                    self.set_clk_high()?;
                    self.wait_for_timer().await;
                }
            };
        }
        Ok(())
    }
}

impl<Miso, Mosi, Sck, Timer, E> ErrorType for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error=E>,
    Mosi: OutputPin<Error=E>,
    Sck: OutputPin<Error=E>,
    Timer: DelayNs,
    E: core::fmt::Debug,
{
    type Error = crate::spi::Error<E>;
}

impl<Miso, Mosi, Sck, Timer, E> SpiBus<u8> for SPI<Miso, Mosi, Sck, Timer>
where
    Miso: InputPin<Error=E>,
    Mosi: OutputPin<Error=E>,
    Sck: OutputPin<Error=E>,
    Timer: DelayNs,
    E: core::fmt::Debug,
{
    #[inline]
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            self.rw_byte(self.config.empty_write_value, word).await?;
        }
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut ignored_write = 0u8;
        for byte in words {
            self.rw_byte(*byte, &mut ignored_write).await?;
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut ignored_write = 0u8;
        for i in 0..max(read.len(), write.len()) {
            let read_in_byte = read.get_mut(i).unwrap_or(&mut ignored_write);
            let clock_out_byte = write
                .get(i)
                .copied()
                .unwrap_or(self.config.empty_write_value);
            self.rw_byte(clock_out_byte, read_in_byte).await?;
        }

        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut current_read_byte = 0u8;
        for clock_out_byte in words {
            self.rw_byte(*clock_out_byte, &mut current_read_byte)
                .await?;
            *clock_out_byte = current_read_byte;
        }

        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // we always flush. Nothing to do here.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;
    use embedded_hal_async::spi::MODE_0;
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };

    fn waveform(string: &str) -> Vec<PinState> {
        let mut transactions = Vec::new();
        let mut last_state = None;
        let mut last_action = string.chars().next().unwrap();
        for step in string.chars() {
            let step = if step == '.' { last_action } else { step };
            match step {
                '0' => transactions.push(PinState::Low),
                '1' => transactions.push(PinState::High),
                'p' | 'P' => {
                    let next_state = if last_state == Some(PinState::Low) {
                        PinState::High
                    } else {
                        PinState::Low
                    };
                    transactions.push(next_state);
                    last_state = Some(next_state);
                }
                'n' | 'N' => {
                    let next_state = if last_state == Some(PinState::High) {
                        PinState::Low
                    } else {
                        PinState::High
                    };
                    transactions.push(next_state);
                    last_state = Some(next_state);
                }
                _ => panic!("Invalid binary literal"),
            };
            last_action = step;
        }
        transactions
    }

    #[test]
    fn test_states() {
        let res = waveform("p..");
        assert_eq!(res, vec![PinState::Low, PinState::High, PinState::Low]);

        let res = waveform("P..");
        assert_eq!(res, vec![PinState::Low, PinState::High, PinState::Low]);

        let res = waveform("n..");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::High]);

        let res = waveform("N..");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::High]);

        let res = waveform("n.0");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::Low]);
    }

    fn input_waveform(string: &str) -> Vec<PinTransaction> {
        waveform(string)
            .into_iter()
            .map(PinTransaction::get)
            .collect()
    }

    fn output_waveform(string: &str) -> Vec<PinTransaction> {
        waveform(string)
            .into_iter()
            .map(PinTransaction::set)
            .collect()
    }

    #[tokio::test]
    async fn test_spi_read_single_byte() {
        let miso = PinMock::new(&input_waveform("10101010"));
        // write default value (0x00) to mosi
        let mosi = PinMock::new(&output_waveform("00000000"));
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let mut spi = SPI::new(MODE_0, miso, mosi, sck, delay, SpiConfig::default());
        let mut data = [0x00];
        spi.read(&mut data).await.expect("SPI read failed");

        spi.mosi.done();
        spi.miso.done();
        spi.sck.done();
        assert_eq!(data[0], 0b10101010);
    }

    #[tokio::test]
    async fn test_spi_write_single_byte() {
        // this is ignored when reading
        let miso = PinMock::new(&input_waveform("00000000"));
        let mosi = PinMock::new(&output_waveform("01010101"));
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let mut spi = SPI::new(MODE_0, miso, mosi, sck, delay, SpiConfig::default());
        let data = [0b01010101]; // 10101010 in binary
        spi.write(&data).await.expect("SPI write failed");

        // Verify that all transactions were completed
        spi.mosi.done();
        spi.sck.done();
        spi.miso.done();
    }

    /// Based on https://www.analog.com/en/resources/analog-dialogue/articles/introduction-to-spi-interface.html
    /// Figure https://www.analog.com/en/_/media/images/analog-dialogue/en/volume-52/number-3/articles/introduction-to-spi-interface/205973_fig_02.png?la=en&rev=c19f52f7fc014bbda34df6bf7c2a18fe&sc_lang=en
    #[tokio::test]
    async fn analog_com_example_figure_2() {
        let miso = PinMock::new(&input_waveform("10111010"));
        let mosi = PinMock::new(&output_waveform("10100101"));
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let mut spi = SPI::new(MODE_0, miso, mosi, sck, delay, SpiConfig::default());
        let mut read_data = [0x00];
        let write_data = [0xA5];

        spi.transfer(&mut read_data, &write_data)
            .await
            .expect("SPI transfer failed");

        spi.miso.done();
        spi.mosi.done();
        spi.sck.done();
        assert_eq!(read_data[0], 0xBA); // Received bits in opposite phase
    }
}
