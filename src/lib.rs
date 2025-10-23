#![no_std]
#![cfg_attr(not(doctest), doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md")))]

pub mod fifo;
pub mod ll;
pub mod ready;
pub mod register_bank;
pub mod uninitialized;

// Reexports.
pub use fifo::{Sample, Timestamp};
pub use uninitialized::{
    Config, InterruptMode, InterruptPolarity, OutputDataRate,
};

#[derive(Debug)]
pub struct Uninitialized;

/// Indicates that the `ICM42688` instance is ready to be used
#[derive(Debug)]
pub struct Ready;

/// Represents all possible errors that can occur in the icm426xx driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<BusError> {
    /// An error occurred on the underlying communication bus.
    Bus(BusError),

    /// The 'Who Am I' check failed during initialization.
    /// This indicates either the wrong device or a communication issue.
    /// The stored u8 indicates the returned response from the device.
    WhoAmIMismatch(u8),
}

/// Helper trait to convert bus errors into our top-level Error::Bus variant.
impl<BusError> From<BusError> for Error<BusError> {
    fn from(error: BusError) -> Self {
        Error::Bus(error)
    }
}

/// ICM42688 top-level driver
///
/// Usage:
///
/// ```rust,ignore
/// # use async_std::prelude::*; // Just for the runtime
/// # use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};
/// # use embedded_hal_mock::eh1::digital::Mock as PinMock;
/// # use embedded_hal_mock::eh1::digital::{State as PinState, Transaction as PinTransaction};
/// # use embedded_hal_mock::eh1::delay::NoopDelay as Delay;
/// # #[async_std::main]
/// async fn main() {
///     let spi = SpiMock::new(&[]);
///     let mut pin = PinMock::new(&[PinTransaction::set(PinState::High)]);
///     let spidev =
///         embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, pin.clone()).unwrap();
///     let mut icm = icm426xx::ICM42688::new(spidev);
///     let mut icm = icm.initialize(Delay).await.unwrap();
///     let mut bank = icm.ll().bank::<{ icm426xx::register_bank::BANK0 }>();
///
///     // print WHO_AM_I register
///     let who_am_i = bank.who_am_i().async_read().await;
///     loop {
///         let fifo_count = icm.read_fifo_count().await;
///         let mut fifo_buffer = [0u32; 128];
///         let num_read = icm.read_fifo(&mut fifo_buffer).await.unwrap();
///     }
/// }
/// ```
pub struct ICM42688<SPI, State> {
    ll: crate::ll::ICM42688<SPI>,
    _state: State,
}
