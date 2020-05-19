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

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    /// GPIO error
    Bus(E),
    /// No ack received
    NoAck,
    /// Invalid input
    InvalidData,
}

/// Bit banging I2C device
pub struct I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin,
    SDA: OutputPin + InputPin,
    CLK: CountDown + Periodic,
{
    scl: SCL,
    sda: SDA,
    clk: CLK,
}

impl<SCL, SDA, CLK, E> I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: CountDown + Periodic,
{
    /// Create instance
    pub fn new(scl: SCL, sda: SDA, clk: CLK) -> Self {
        I2cBB { scl, sda, clk }
    }

    fn i2c_start(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.set_scl_high()?;
        self.set_sda_high()?;
        self.wait_for_clk();

        self.set_sda_low()?;
        self.wait_for_clk();

        self.set_scl_low()?;
        self.wait_for_clk();

        Ok(())
    }

    fn i2c_stop(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.set_scl_high()?;
        self.wait_for_clk();

        self.set_sda_high()?;
        self.wait_for_clk();

        Ok(())
    }

    fn i2c_is_ack(&mut self) -> Result<bool, crate::i2c::Error<E>> {
        self.set_sda_high()?;
        self.set_scl_high()?;
        self.wait_for_clk();

        let ack = self.sda.is_low().map_err(Error::Bus)?;

        self.set_scl_low()?;
        self.set_sda_low()?;
        self.wait_for_clk();

        Ok(ack)
    }

    fn i2c_read_byte(&mut self, should_send_ack: bool) -> Result<u8, crate::i2c::Error<E>> {
        let mut byte: u8 = 0;

        self.set_sda_high()?;

        for bit_offset in 0..8 {
            self.set_scl_high()?;
            self.wait_for_clk();

            if self.sda.is_high().map_err(Error::Bus)? {
                byte |= 1 << (7 - bit_offset);
            }

            self.set_scl_low()?;
            self.wait_for_clk();
        }

        if should_send_ack {
            self.set_sda_low()?;
        } else {
            self.set_sda_high()?;
        }

        self.set_scl_high()?;
        self.wait_for_clk();

        self.set_scl_low()?;
        self.set_sda_low()?;
        self.wait_for_clk();

        Ok(byte)
    }

    fn i2c_write_byte(&mut self, byte: u8) -> Result<(), crate::i2c::Error<E>> {
        for bit_offset in 0..8 {
            let out_bit = (byte >> (7 - bit_offset)) & 0b1;

            if out_bit == 1 {
                self.set_sda_high()?;
            } else {
                self.set_sda_low()?;
            }

            self.set_scl_high()?;
            self.wait_for_clk();

            self.set_scl_low()?;
            self.set_sda_low()?;
            self.wait_for_clk();
        }

        Ok(())
    }

    #[inline]
    fn read_from_slave(&mut self, input: &mut [u8]) -> Result<(), crate::i2c::Error<E>> {
        for i in 0..input.len() {
            let should_send_ack = i != (input.len() - 1);
            input[i] = self.i2c_read_byte(should_send_ack)?;
        }
        Ok(())
    }

    #[inline]
    fn write_to_slave(&mut self, output: &[u8]) -> Result<(), crate::i2c::Error<E>> {
        for byte in output {
            self.i2c_write_byte(*byte)?;
            self.check_ack()?;
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
    fn wait_for_clk(&mut self) {
        block!(self.clk.wait()).ok();
    }

    #[inline]
    fn check_ack(&mut self) -> Result<(), crate::i2c::Error<E>> {
        if !self.i2c_is_ack()? {
            Err(Error::NoAck)
        } else {
            Ok(())
        }
    }
}

impl<SCL, SDA, CLK, E> Write for I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: CountDown + Periodic,
{
    type Error = crate::i2c::Error<E>;

    fn write(&mut self, addr: u8, output: &[u8]) -> Result<(), Self::Error> {
        if output.is_empty() {
            return Ok(());
        }

        // ST
        self.i2c_start()?;

        // SAD + W
        self.i2c_write_byte((addr << 1) | 0x0)?;
        self.check_ack()?;

        self.write_to_slave(output)?;

        // SP
        self.i2c_stop()
    }
}

impl<SCL, SDA, CLK, E> Read for I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: CountDown + Periodic,
{
    type Error = crate::i2c::Error<E>;

    fn read(&mut self, addr: u8, input: &mut [u8]) -> Result<(), Self::Error> {
        if input.is_empty() {
            return Ok(());
        }

        // ST
        self.i2c_start()?;

        // SAD + R
        self.i2c_write_byte((addr << 1) | 0x1)?;
        self.check_ack()?;

        self.read_from_slave(input)?;

        // SP
        self.i2c_stop()
    }
}

impl<SCL, SDA, CLK, E> WriteRead for I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: CountDown + Periodic,
{
    type Error = crate::i2c::Error<E>;

    fn write_read(&mut self, addr: u8, output: &[u8], input: &mut [u8]) -> Result<(), Self::Error> {
        if output.is_empty() || input.is_empty() {
            return Err(Error::InvalidData);
        }

        // ST
        self.i2c_start()?;

        // SAD + W
        self.i2c_write_byte((addr << 1) | 0x0)?;
        self.check_ack()?;

        self.write_to_slave(output)?;

        // SR
        self.i2c_start()?;

        // SAD + R
        self.i2c_write_byte((addr << 1) | 0x1)?;
        self.check_ack()?;

        self.read_from_slave(input)?;

        // SP
        self.i2c_stop()
    }
}
