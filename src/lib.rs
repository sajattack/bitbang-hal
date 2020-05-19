//! This is a [bit banging] implementation of the [`embedded-hal`] traits.
//!
//! [bit banging]: https://en.wikipedia.org/wiki/Bit_banging
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! ## Usage examples
//!
//! See usage examples in the examples folder in the crate sources

#![no_std]
#[deny(missing_docs)]

pub mod i2c;
pub mod serial;
pub mod spi;
