#![no_std]
#![cfg_attr(not(doctest), doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md")))]

use core::marker::PhantomData;

pub mod fifo;
pub mod ll;
pub mod ready;
pub mod register_bank;
pub mod uninitialized;

// Reexports.
pub use fifo::{Sample, Timestamp};
pub use uninitialized::{Config, InterruptMode, InterruptPolarity, OutputDataRate};

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

    /// The code has been to slow in reading samples out of the FIFO. The
    /// user is recommended to mitigate by restarting the device if this
    /// happens.
    FifoOverflow,
}

/// Helper trait to convert bus errors into our top-level Error::Bus variant.
impl<BusError> From<BusError> for Error<BusError> {
    fn from(error: BusError) -> Self {
        Error::Bus(error)
    }
}

mod sealed {
    pub trait Sealed {}
}

/// Trait describing a specific device variant in the ICM-426xx family.
///
/// This trait is sealed and cannot be implemented outside of this crate.
pub trait Device: sealed::Sealed {
    /// Expected WHO_AM_I register value.
    const WHO_AM_I: u8;
    /// Full-scale gyroscope range in degrees per second (for 20-bit FIFO mode).
    const GYRO_FULL_SCALE_DPS: f32;
    /// Full-scale accelerometer range in g (for 20-bit FIFO mode).
    const ACCEL_FULL_SCALE_G: f32;
}

/// Marker type for the ICM-42688-P variant.
///
/// WHO_AM_I = 0x47, gyro ±2000 dps, accel ±16g.
pub struct Icm42688p;

impl sealed::Sealed for Icm42688p {}
impl Device for Icm42688p {
    const WHO_AM_I: u8 = 0x47;
    const GYRO_FULL_SCALE_DPS: f32 = 2000.0;
    const ACCEL_FULL_SCALE_G: f32 = 16.0;
}

/// Marker type for the ICM-42686-P variant.
///
/// WHO_AM_I = 0x44, gyro ±4000 dps, accel ±32g.
pub struct Icm42686p;

impl sealed::Sealed for Icm42686p {}
impl Device for Icm42686p {
    const WHO_AM_I: u8 = 0x44;
    const GYRO_FULL_SCALE_DPS: f32 = 4000.0;
    const ACCEL_FULL_SCALE_G: f32 = 32.0;
}

/// Type alias for the ICM-42686-P variant.
pub type ICM42686<SPI, State> = ICM42688<SPI, State, Icm42686p>;

/// ICM42688 top-level driver
///
/// The `D` type parameter selects the device variant and defaults to
/// [`Icm42688p`]. Use [`Icm42686`] (a type alias) for the ICM-42686-P.
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
pub struct ICM42688<SPI, State, D: Device = Icm42688p> {
    ll: crate::ll::ICM42688<SPI>,
    _state: State,
    _device: PhantomData<D>,
}
