use core::time::Duration;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::serial;
use embedded_hal::timer::{CountDown, Periodic};
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
	rate: u64,
	rx_delay_centering: u64,
	rx_delay_intrabit: u64,
	rx_delay_stopbit: u64,
	tx_delay: u64,
	timeout: u64,
}

impl<TX, RX, Timer, E> Serial<TX, RX, Timer>
where
	TX: OutputPin<Error = E>,
	RX: InputPin<Error = E>,
	Timer: CountDown + Periodic,
	Timer::Time: From<Duration>,
{
	pub fn new(tx: TX, rx: RX, timer: Timer) -> Self {
		Serial {
			tx,
			rx,
			timer,
			rate: 0,
			rx_delay_centering: 0,
			rx_delay_intrabit: 0,
			rx_delay_stopbit: 0,
			tx_delay: 0,
			timeout: 0,
		}
	}

	#[allow(dead_code)]
	pub fn destroy(self) -> (TX, RX, Timer)
	where
		TX: OutputPin<Error = E>,
		RX: InputPin<Error = E>,
		Timer: CountDown + Periodic,
	{
		(self.tx, self.rx, self.timer)
	}

	pub fn set_rate(&mut self, rate: u64) {
		self.rate = rate;
		self.rx_delay_centering = 100_000_000 / rate;
		self.rx_delay_intrabit = 1_000_000_000 / rate - 500;
		self.rx_delay_stopbit = 10_000_000 / rate;
		self.tx_delay = 1_000_000_000 / rate;
	}

	pub fn set_timeout(&mut self, timeout: u64) { self.timeout = timeout; }

	#[inline]
	pub fn wait_time(&mut self, nanoseconds: u64) {
		self.set_timer(nanoseconds);
		self.wait();
	}

	#[inline]
	fn set_timer(&mut self, nanoseconds: u64) {
		self.timer.start(Duration::from_nanos(nanoseconds));
	}

	#[inline]
	fn wait(&mut self) { block!(self.timer.wait()).unwrap(); }

	pub fn try_read(&mut self) -> nb::Result<u8, ()> {
		let mut data_in = 0;
		let mut select_bit = 1u8;
		// Wait for start bit
		let mut ctr = 0u64;
		while self.rx.is_high().map_err(|_| ())? {
			ctr += 1;
			if ctr >= self.timeout {
				return Err(nb::Error::Other(()));
			}
		}
		self.wait_time(self.rx_delay_centering);
		self.wait_time(self.rx_delay_intrabit);
		for _ in 0..8 {
			if self.rx.is_high().map_err(|_| ())? {
				data_in |= select_bit;
			}
			else {
				data_in &= !select_bit;
			}
			select_bit <<= 1;
			self.wait();
		}
		// Wait for stop bit
		self.wait_time(self.rx_delay_stopbit);
		Ok(data_in)
	}
}

impl<TX, RX, Timer, E> serial::Write<u8> for Serial<TX, RX, Timer>
where
	TX: OutputPin<Error = E>,
	RX: InputPin<Error = E>,
	Timer: CountDown + Periodic,
	Timer::Time: From<Duration>,
{
	type Error = Error<E>;

	fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
		let mut select_bit = 1u8;
		// Start bit
		self.tx.set_low().map_err(Error::Bus)?;
		self.wait_time(self.tx_delay);
		for _ in 0..8 {
			if byte & select_bit != 0 {
				self.tx.set_high().map_err(Error::Bus)?;
			}
			else {
				self.tx.set_low().map_err(Error::Bus)?;
			}
			select_bit <<= 1;
			self.wait();
		}
		// Stop bit
		self.tx.set_high().map_err(Error::Bus)?;
		self.wait();
		Ok(())
	}

	fn flush(&mut self) -> nb::Result<(), Self::Error> { Ok(()) }
}

impl<TX, RX, Timer, E> serial::Read<u8> for Serial<TX, RX, Timer>
where
	TX: OutputPin<Error = E>,
	RX: InputPin<Error = E>,
	Timer: CountDown + Periodic,
	Timer::Time: From<Duration>,
{
	type Error = Error<E>;

	fn read(&mut self) -> nb::Result<u8, Self::Error> {
		let mut data_in = 0;
		let mut select_bit = 1u8;
		// Wait for start bit
		while self.rx.is_high().map_err(Error::Bus)? {}
		self.wait_time(self.rx_delay_centering);
		self.wait_time(self.rx_delay_intrabit);
		for _ in 0..8 {
			if self.rx.is_high().map_err(Error::Bus)? {
				data_in |= select_bit;
			}
			else {
				data_in &= !select_bit;
			}
			select_bit <<= 1;
			self.wait();
		}
		// Wait for stop bit
		self.wait_time(self.rx_delay_stopbit);
		Ok(data_in)
	}
}
