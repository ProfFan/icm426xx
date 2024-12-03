# `icm426xx` - ICM-426xx 6-axis motion sensor driver in Rust

[![crates.io](https://img.shields.io/crates/v/icm426xx.svg)](https://crates.io/crates/icm426xx)
[![docs.rs](https://docs.rs/icm426xx/badge.svg)](https://docs.rs/icm426xx)

This is a platform agnostic Rust driver for the ICM-426xx 6-axis motion sensor using the `embedded-hal` traits.

Currently supported devices:

- ICM-42688-P

We support both the I2C and SPI interface, but currently only the SPI interface is tested. PRs are welcome!

Similarly, we support both the async and blocking interface, but currently only the async interface is tested.

Only the FIFO-based, 20-bit data mode is supported. Please open an issue if you need support for other modes.

## Usage

```rust
use async_std::prelude::*; // Just for the runtime
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};
use embedded_hal_mock::eh1::digital::Mock as PinMock;
use embedded_hal_mock::eh1::digital::{State as PinState, Transaction as PinTransaction};
use embedded_hal_mock::eh1::delay::NoopDelay as Delay;
#[async_std::main]
async fn main() {
    let spi = SpiMock::new(&[]);
    let mut pin = PinMock::new(&[PinTransaction::set(PinState::High)]);
    let spidev =
        embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, pin.clone()).unwrap();

    let mut icm = icm426xx::ICM42688::new(spidev);
    let mut icm = icm.initialize(Delay).await.unwrap();
    let mut bank = icm.ll().bank::<{ icm426xx::register_bank::BANK0 }>();

    // print WHO_AM_I register
    let who_am_i = bank.who_am_i().async_read().await;
    loop {
        let fifo_count = icm.read_fifo_count().await;
        let mut fifo_buffer = [0u32; 128];
        let num_read = icm.read_fifo(&mut fifo_buffer).await.unwrap();
    }
}
```

## CHANGELOG

- 0.3.2: Fixed the initialization sequence to match the datasheet.

## LICENSE

MIT OR Apache-2.0
