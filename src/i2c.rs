/*!
# Synchronous implementation of embedded-hal I2C traits based on GPIO bitbang

This implementation consumes the following hardware resources:
- A periodic timer to mark clock cycles
- Two GPIO pins for SDA and SCL lines.

Note that the current implementation does not support I2C clock stretching.

## Hardware requirements

1. Configure GPIO pins as Open-Drain outputs.
2. Configure timer frequency to be twice the desired I2C clock frequency.

## Blue Pill example

Here is a sample code for LM75A I2C temperature sensor
on Blue Pill or any other stm32f1xx board:

```no_run
  use stm32f1xx_hal as hal;
  use hal::{prelude::*, timer::Timer, stm32};
  use lm75::{Lm75, SlaveAddr};
  use bitbang_hal;
  use fugit::*;

  // ...

  let pdev = stm32::Peripherals::take().unwrap();

  let mut flash = pdev.FLASH.constrain();
  let mut rcc = pdev.RCC.constrain();
  let mut gpioa = pdev.GPIOA.split(&mut rcc.apb2);

  let clocks = rcc
      .cfgr
      .use_hse(8.mhz())
      .sysclk(32.mhz())
      .pclk1(16.mhz())
      .freeze(&mut flash.acr);

  let tmr = Timer::tim3(pdev.TIM3, &clocks, &mut rcc.apb1).start_count_down(200.khz());
  let scl = gpioa.pa1.into_open_drain_output(&mut gpioa.crl);
  let sda = gpioa.pa2.into_open_drain_output(&mut gpioa.crl);

  let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);
  let mut sensor = Lm75::new(i2c, SlaveAddr::default());
  let temp = sensor.read_temperature().unwrap();

  //...
```
*/

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation, SevenBitAddress};
use embedded_hal::i2c::NoAcknowledgeSource::Unknown;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use fugit::{Duration, TimerRate};

/// I2C error
#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    /// GPIO error
    Bus(E),
    /// No ack received
    NoAck(NoAcknowledgeSource),
    /// Invalid input
    InvalidData,
}

impl<E : core::fmt::Debug> embedded_hal::i2c::Error for Error<E> {
    fn kind(&self) -> ErrorKind {
        match self {
            Error::Bus(_) => ErrorKind::Bus,
            Error::NoAck(source) => ErrorKind::NoAcknowledge(*source),
            Error::InvalidData => ErrorKind::Other
        }
    }
}

/// Bit banging I2C device
pub struct I2cBB<SCL, SDA, DELAY>
where
    SCL: OutputPin,
    SDA: OutputPin + InputPin,
    DELAY: DelayNs,
{
    scl: SCL,
    sda: SDA,
    delay: DELAY,
    clock_duration: Duration<u32, 1, 1_000_000>,
}

impl<SCL, SDA, DELAY, E> I2cBB<SCL, SDA, DELAY>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    DELAY: DelayNs,
{
    /// Create instance
    pub fn new(scl: SCL, sda: SDA, delay: DELAY, frequency: TimerRate<u32, 1_000_000>) -> Self {
        I2cBB {
            scl,
            sda,
            delay,
            clock_duration: frequency.into_duration(),
        }
    }

    /// Send a raw I2C start.
    ///
    /// **This is a low-level control function.** For normal I2C devices,
    /// please use the embedded-hal traits [Read], [Write], or
    /// [WriteRead].
    pub async fn raw_i2c_start(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.set_scl_high()?;
        self.set_sda_high()?;
        self.wait_for_clk().await;

        self.set_sda_low()?;
        self.wait_for_clk().await;

        self.set_scl_low()?;
        self.wait_for_clk().await;

        Ok(())
    }

    /// Send a raw I2C stop.
    ///
    /// **This is a low-level control function.** For normal I2C devices,
    /// please use the embedded-hal traits [Read], [Write], or
    /// [WriteRead].
    pub async fn raw_i2c_stop(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.set_scl_high()?;
        self.wait_for_clk().await;

        self.set_sda_high()?;
        self.wait_for_clk().await;

        Ok(())
    }

    async fn i2c_is_ack(&mut self) -> Result<bool, crate::i2c::Error<E>> {
        self.set_sda_high()?;
        self.set_scl_high()?;
        self.wait_for_clk().await;

        let ack = self.sda.is_low().map_err(Error::Bus)?;

        self.set_scl_low()?;
        self.set_sda_low()?;
        self.wait_for_clk().await;

        Ok(ack)
    }

    async fn i2c_read_byte(&mut self, should_send_ack: bool) -> Result<u8, crate::i2c::Error<E>> {
        let mut byte: u8 = 0;

        self.set_sda_high()?;

        for bit_offset in 0..8 {
            self.set_scl_high()?;
            self.wait_for_clk().await;

            if self.sda.is_high().map_err(Error::Bus)? {
                byte |= 1 << (7 - bit_offset);
            }

            self.set_scl_low()?;
            self.wait_for_clk().await;
        }

        if should_send_ack {
            self.set_sda_low()?;
        } else {
            self.set_sda_high()?;
        }

        self.set_scl_high()?;
        self.wait_for_clk().await;

        self.set_scl_low()?;
        self.set_sda_low()?;
        self.wait_for_clk().await;

        Ok(byte)
    }

    async fn i2c_write_byte(&mut self, byte: u8) -> Result<(), crate::i2c::Error<E>> {
        for bit_offset in 0..8 {
            let out_bit = (byte >> (7 - bit_offset)) & 0b1;

            if out_bit == 1 {
                self.set_sda_high()?;
            } else {
                self.set_sda_low()?;
            }

            self.set_scl_high()?;
            self.wait_for_clk().await;

            self.set_scl_low()?;
            self.set_sda_low()?;
            self.wait_for_clk().await;
        }

        Ok(())
    }

    /// Read raw bytes from the slave.
    ///
    /// **This is a low-level control function.** For normal I2C devices,
    /// please use the embedded-hal traits [Read], [Write], or
    /// [WriteRead].
    #[inline]
    pub async fn raw_read_from_slave(&mut self, input: &mut [u8]) -> Result<(), crate::i2c::Error<E>> {
        for i in 0..input.len() {
            let should_send_ack = i != (input.len() - 1);
            input[i] = self.i2c_read_byte(should_send_ack).await?;
        }
        Ok(())
    }

    /// Send raw bytes to the slave.
    ///
    /// **This is a low-level control function.** For normal I2C devices,
    /// please use the embedded-hal traits [Read], [Write], or
    /// [WriteRead].
    #[inline]
    pub async fn raw_write_to_slave(&mut self, output: &[u8]) -> Result<(), crate::i2c::Error<E>> {
        for byte in output {
            self.i2c_write_byte(*byte).await?;
            self.check_ack(NoAcknowledgeSource::Data).await?;
        }
        Ok(())
    }

    #[inline]
    fn set_scl_high(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.scl.set_high().map_err(Error::Bus)
    }

    #[inline]
    fn set_scl_low(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.scl.set_low().map_err(Error::Bus)
    }

    #[inline]
    fn set_sda_high(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.sda.set_high().map_err(Error::Bus)
    }

    #[inline]
    fn set_sda_low(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.sda.set_low().map_err(Error::Bus)
    }

    #[inline]
    async fn wait_for_clk(&mut self) {
        self.delay.delay_ns(self.clock_duration.to_nanos()).await;
    }

    #[inline]
    async fn check_ack(&mut self, source: NoAcknowledgeSource) -> Result<(), crate::i2c::Error<E>> {
        if !self.i2c_is_ack().await? {
            Err(Error::NoAck(source))
        } else {
            Ok(())
        }
    }
}

impl<SCL, SDA, CLK, E> ErrorType for I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: DelayNs,
    E: core::fmt::Debug
{
    type Error = crate::i2c::Error<E>;
}

impl<SCL, SDA, CLK, E> I2c for I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: DelayNs,
    E: core::fmt::Debug
{
    async fn read(&mut self, addr: u8, input: &mut [u8]) -> Result<(), Self::Error> {
        if input.is_empty() {
            return Ok(());
        }

        // ST
        self.raw_i2c_start().await?;

        // SAD + R
        self.i2c_write_byte((addr << 1) | 0x1).await?;
        self.check_ack(NoAcknowledgeSource::Address).await?;

        self.raw_read_from_slave(input).await?;

        // SP
        self.raw_i2c_stop().await
    }

    async fn write(&mut self, addr: u8, output: &[u8]) -> Result<(), Self::Error> {
        // ST
        self.raw_i2c_start().await?;

        // SAD + W
        self.i2c_write_byte((addr << 1) | 0x0).await?;
        self.check_ack(NoAcknowledgeSource::Address).await?;

        self.raw_write_to_slave(output).await?;

        // SP
        self.raw_i2c_stop().await?;

        Ok(())
    }

    async fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        #[derive(Eq, PartialEq, Copy, Clone)]
        enum OpType {
            Read,
            Write,
        }
        let mut previous_op_type: Option<OpType> = None;
        for operation in operations.iter_mut() {
            if previous_op_type != Some(OpType::Read) {
                // ST or SR for subsequent frames
                self.raw_i2c_start().await?;
            }

            match operation {
                Operation::Read(buffer) => {
                    // SAD + R
                    self.i2c_write_byte((address << 1) | 0x1).await?;
                    self.raw_read_from_slave(buffer).await?;
                    self.check_ack(NoAcknowledgeSource::Data).await?;

                    previous_op_type = Some(OpType::Read);
                }
                Operation::Write(bytes) => {
                    // SAD + W
                    self.i2c_write_byte((address << 1) | 0x0).await?;

                    self.raw_write_to_slave(bytes).await?;

                    self.check_ack(NoAcknowledgeSource::Data).await?;

                    previous_op_type = Some(OpType::Write);
                }
            };
        }

        // SP
        self.raw_i2c_stop().await
    }
}
