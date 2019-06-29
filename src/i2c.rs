/*!
  # Synchronous implementation of embedded-hal I2C traits based on GPIO bitbang


  This implementation consumes the following hardware resources:
  periodic timer to mark clock cycles and
  two gpio pins for SDA and SCL lines.

  ## Hardware requirements

  1. Configure gpio pins as Open-Drain outputs
  2. Configure timer frequency to be twice as required i2c clock

  ## Blue Pill example

  Here is a sample code for LM75A I2C temperature sensor
  on Blue Pill or any other stm32f1xx board:

  ```rust
   extern crate stm32f1xx_hal as hal;
   use hal::prelude::*;
   use hal::timer::Timer;

   extern crate lm75;
   use lm75::{Lm75, SlaveAddr};

   use bitbang_hal;

   ...

   let dp = hal::stm32::Peripherals::take().unwrap();
   let mut rcc = dp.RCC.constrain();
   let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

   let mut flash = dp.FLASH.constrain();
   let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(32.mhz())
        .pclk1(16.mhz())
        .freeze(&mut flash.acr);

    let tmr = Timer::tim3(dp.TIM3, 200.khz(), clocks, &mut rcc.apb1);
    let scl = gpioa.pa1.into_open_drain_output(&mut gpioa.crl);
    let sda = gpioa.pa2.into_open_drain_output(&mut gpioa.crl);

    let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);
    let mut sensor = Lm75::new(i2c, SlaveAddr::default());
    let temp = sensor.read_temperature().unwrap();

    ...

  ```

*/

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

/// I2C error
#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    /// gpio error
    Bus(E),
    /// No ack received
    NoAck,
    /// Invalid input
    InvalidData,
}

/// I2C structure
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
    pub fn new(scl: SCL, sda: SDA, clk: CLK) -> Self {
        I2cBB { scl, sda, clk }
    }

    fn i2c_start(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.scl.set_high().map_err(Error::Bus)?;
        self.sda.set_high().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        self.sda.set_low().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        self.sda.set_low().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        Ok(())
    }

    fn i2c_stop(&mut self) -> Result<(), crate::i2c::Error<E>> {
        self.sda.set_high().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        self.sda.set_high().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        Ok(())
    }

    fn i2c_is_ack(&mut self) -> Result<bool, crate::i2c::Error<E>> {
        self.sda.set_high().map_err(Error::Bus)?;
        self.sda.set_high().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        let ack = self.sda.is_low().map_err(Error::Bus)?;

        self.sda.set_low().map_err(Error::Bus)?;
        self.sda.set_low().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        Ok(ack)
    }

    fn i2c_read_byte(&mut self, ack: bool) -> Result<u8, crate::i2c::Error<E>> {
        let mut byte: u8 = 0;

        self.sda.set_high().map_err(Error::Bus)?;

        for i in 0..8 {
            self.sda.set_high().map_err(Error::Bus)?;
            block!(self.clk.wait()).ok();

            if self.sda.is_high().map_err(Error::Bus)? {
                byte |= 1 << (7 - i);
            }

            self.sda.set_low().map_err(Error::Bus)?;
            block!(self.clk.wait()).ok();
        }

        if ack {
            self.sda.set_low().map_err(Error::Bus)?;
        } else {
            self.sda.set_high().map_err(Error::Bus)?;
        }

        self.sda.set_high().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        self.sda.set_low().map_err(Error::Bus)?;
        self.sda.set_low().map_err(Error::Bus)?;
        block!(self.clk.wait()).ok();

        Ok(byte)
    }

    fn i2c_write_byte(&mut self, byte: u8) -> Result<(), crate::i2c::Error<E>> {
        for bit in 0..8 {
            let val = (byte >> (7 - bit)) & 0b1;

            if val == 1 {
                self.sda.set_high().map_err(Error::Bus)?;
            } else {
                self.sda.set_low().map_err(Error::Bus)?;
            }

            self.sda.set_high().map_err(Error::Bus)?;
            block!(self.clk.wait()).ok();

            self.sda.set_low().map_err(Error::Bus)?;
            self.sda.set_low().map_err(Error::Bus)?;
            block!(self.clk.wait()).ok();
        }

        Ok(())
    }
}

impl<SCL, SDA, CLK, E> Write for I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: CountDown + Periodic,
{
    type Error  = crate::i2c::Error<E>;

    fn write(&mut self, addr: u8, output: &[u8]) -> Result<(), Self::Error> {
        if output.is_empty() {
            return Ok(());
        }

        // ST
        self.i2c_start()?;

        // SAD + W
        self.i2c_write_byte((addr << 1) | 0x0)?;
        if !self.i2c_is_ack()? {
            return Err(Error::NoAck);
        }

        // write output to slave
        for byte in output {
            self.i2c_write_byte(*byte)?;

            if !self.i2c_is_ack()? {
                return Err(Error::NoAck);
            }
        }

        // SP
        self.i2c_stop()?;

        Ok(())
    }
}

impl<SCL, SDA, CLK, E> Read for I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: CountDown + Periodic,
{
    type Error  = crate::i2c::Error<E>;

    fn read(&mut self, addr: u8, input: &mut [u8]) -> Result<(), Self::Error> {
        if input.is_empty() {
            return Ok(());
        }

        // ST
        self.i2c_start()?;

        // SAD + R
        self.i2c_write_byte((addr << 1) | 0x1)?;
        if !self.i2c_is_ack()? {
            return Err(Error::NoAck);
        }

        // read bytes from slave
        for i in 0..input.len() {
            input[i] = self.i2c_read_byte(i != (input.len() - 1))?;
        }

        // SP
        self.i2c_stop()?;

        Ok(())
    }
}

impl<SCL, SDA, CLK, E> WriteRead for I2cBB<SCL, SDA, CLK>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    CLK: CountDown + Periodic,
{
    type Error  = crate::i2c::Error<E>;

    fn write_read(&mut self, addr: u8, output: &[u8], input: &mut [u8]) -> Result<(), Self::Error> {
        if output.is_empty() || input.is_empty() {
            return Err(Error::InvalidData);
        }

        // ST
        self.i2c_start()?;

        // SAD + W
        self.i2c_write_byte((addr << 1) | 0x0)?;
        if !self.i2c_is_ack()? {
            return Err(Error::NoAck);
        }

        // write output to slave
        for byte in output {
            self.i2c_write_byte(*byte)?;

            if !self.i2c_is_ack()? {
                return Err(Error::NoAck);
            }
        }

        // SR
        self.i2c_start()?;

        // SAD + R
        self.i2c_write_byte((addr << 1) | 0x1)?;
        if !self.i2c_is_ack()? {
            return Err(Error::NoAck);
        }

        // read output from slave
        for i in 0..input.len() {
            input[i] = self.i2c_read_byte(i != (input.len() - 1))?;
        }

        // SP
        self.i2c_stop()?;

        Ok(())
    }
}
