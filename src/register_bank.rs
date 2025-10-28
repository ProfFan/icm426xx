//! Register bank module
//!
//! The ICM42688P IMU has 5 register banks, and access to the registers is
//! controlled by the BANK_SEL register. This module provides a type-safe
//! interface to the register banks.
#![allow(non_camel_case_types)]
#![allow(unused)]
#![allow(clippy::eq_op)]
#![allow(clippy::identity_op)]

use core::{fmt, marker::PhantomData};

#[cfg(not(feature = "async"))]
use embedded_hal::spi;

#[cfg(feature = "async")]
use embedded_hal_async::spi as async_spi;

use paste::paste;

// #[feature(adt_const_params)] is not stable yet
// #[derive(ConstParamTy, Debug, Clone, Copy, PartialEq, Eq)]
// pub enum RegisterBank {
//     Bank0,
//     Bank1,
//     Bank2,
//     Bank3,
//     Bank4,
// }
pub type RegisterBank = u8;
pub const BANK0: RegisterBank = 0x00;
pub const BANK1: RegisterBank = 0x01;
pub const BANK2: RegisterBank = 0x02;
pub const BANK3: RegisterBank = 0x03;
pub const BANK4: RegisterBank = 0x04;

#[derive(Debug, PartialEq, Eq)]
pub struct Registers<'b, BUS, const BANK: RegisterBank> {
    bus: &'b mut BUS,
}

impl<'b, BUS, const BANK: RegisterBank> Registers<'b, BUS, BANK> {
    /// Create a new instance of `Registers`
    ///
    /// Requires the BUS peripheral and the chip select pin that are connected
    /// to the Registers.
    pub fn new(bus: &'b mut BUS) -> Self {
        Registers { bus }
    }

    /// Direct access to the BUS bus
    pub fn bus(&mut self) -> &mut BUS {
        self.bus
    }

    /// Current register bank
    ///
    /// This method is used to get the current register bank.
    #[inline(always)]
    pub fn current_bank(&self) -> RegisterBank {
        BANK
    }
}

/// Provides access to a register
///
/// You can get an instance for a given register using one of the methods on
/// [`Registers`].
pub struct RegAccessor<'s, 'b, R, BUS, const BANK: RegisterBank>(
    &'s mut Registers<'b, BUS, BANK>,
    PhantomData<R>,
);

#[cfg(not(feature = "async"))]
impl<R, BUS, const BANK: RegisterBank> RegAccessor<'_, '_, R, BUS, BANK>
where
    BUS: spi::SpiDevice<u8>,
{
    /// Read from the register
    pub fn read(&mut self) -> Result<R::Read, BUS::Error>
    where
        R: Register + Readable,
    {
        let mut r = R::read();
        let buffer = R::buffer(&mut r);

        init_header::<R>(false, buffer);
        self.0.bus.transfer_in_place(buffer)?;

        Ok(r)
    }

    /// Write to the register
    pub fn write<F>(&mut self, f: F) -> Result<(), BUS::Error>
    where
        R: Register + Writable,
        F: FnOnce(&mut R::Write) -> &mut R::Write,
    {
        let mut w = R::write();
        f(&mut w);

        let buffer = R::buffer(&mut w);
        init_header::<R>(true, buffer);

        BUS::write(self.0.bus, buffer)?;

        Ok(())
    }

    /// Modify the register
    pub fn modify<F>(&mut self, f: F) -> Result<(), BUS::Error>
    where
        R: Register + Readable + Writable,
        F: for<'r> FnOnce(&mut R::Read, &'r mut R::Write) -> &'r mut R::Write,
    {
        let mut r = self.read()?;
        let mut w = R::write();

        <R as Writable>::buffer(&mut w).copy_from_slice(<R as Readable>::buffer(&mut r));

        f(&mut r, &mut w);

        let buffer = <R as Writable>::buffer(&mut w);
        init_header::<R>(true, buffer);

        BUS::write(self.0.bus, buffer)?;

        Ok(())
    }
}

/// Provide async access to a register
#[cfg(feature = "async")]
impl<R, BUS, const BANK: RegisterBank> RegAccessor<'_, '_, R, BUS, BANK>
where
    BUS: async_spi::SpiDevice<u8>,
{
    /// Read from the register
    pub async fn async_read(&mut self) -> Result<R::Read, BUS::Error>
    where
        R: Register + Readable,
    {
        let mut r = R::read();
        let buffer = R::buffer(&mut r);

        init_header::<R>(false, buffer);
        async_spi::SpiDevice::transfer_in_place(&mut self.0.bus, buffer).await?;

        Ok(r)
    }

    /// Write to the register
    pub async fn async_write<F>(&mut self, f: F) -> Result<(), BUS::Error>
    where
        R: Register + Writable,
        F: FnOnce(&mut R::Write) -> &mut R::Write,
    {
        let mut w = R::write();
        f(&mut w);

        let buffer = R::buffer(&mut w);
        let _ = init_header::<R>(true, buffer);

        async_spi::SpiDevice::write(&mut self.0.bus, buffer).await?;

        Ok(())
    }

    /// Modify the register
    pub async fn async_modify<F>(&mut self, f: F) -> Result<(), BUS::Error>
    where
        R: Register + Readable + Writable,
        F: for<'r> FnOnce(&mut R::Read, &'r mut R::Write) -> &'r mut R::Write,
    {
        let mut r = self.async_read().await?;
        let mut w = R::write();

        <R as Writable>::buffer(&mut w).copy_from_slice(<R as Readable>::buffer(&mut r));

        f(&mut r, &mut w);

        let buffer = <R as Writable>::buffer(&mut w);
        let _ = init_header::<R>(true, buffer);

        async_spi::SpiDevice::write(&mut self.0.bus, buffer).await?;

        Ok(())
    }
}

/// Initializes the SPI message header
///
/// Initializes the SPI message header for accessing a given register, writing
/// the header directly into the provided buffer. Returns the length of the
/// header that was written.
fn init_header<R: Register>(write: bool, buffer: &mut [u8]) -> usize {
    buffer[0] = (((!write as u8) << 7) & 0x80) | R::ID;

    1
}

/// Implemented for all registers
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
pub trait Register {
    /// The register index
    const ID: u8;

    /// The length of the register
    const LEN: usize;
}

/// Marker trait for registers that can be read from
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
pub trait Readable {
    /// The type that is used to read from the register
    type Read;

    /// Return the read type for this register
    fn read() -> Self::Read;

    /// Return the read type's internal buffer
    fn buffer(r: &mut Self::Read) -> &mut [u8];
}

/// Marker trait for registers that can be written to
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
pub trait Writable {
    /// The type that is used to write to the register
    type Write;

    /// Return the write type for this register
    fn write() -> Self::Write;

    /// Return the write type's internal buffer
    fn buffer(w: &mut Self::Write) -> &mut [u8];
}

/// Generates register implementations
macro_rules! impl_register {
    (
        $bank: ident,
        $(
            $id:expr,
            $len:expr,
            $rw:tt,
            $name:ident($name_lower:ident) {
            #[$doc:meta]
            $(
                $field:ident,
                $first_bit:expr,
                $last_bit:expr,
                $ty:ty;
                #[$field_doc:meta]
            )*
            }
        )*
    ) => {
        paste! {
            pub mod [<$bank:lower>] {
                use super::*;

                $(
                    #[$doc]
                    #[allow(non_camel_case_types)]
                    pub struct $name;

                    impl Register for $name {
                        const ID:     u8    = $id;
                        const LEN:    usize = $len;
                    }

                    impl $name {
                        const HEADER_LEN: usize = 1; // Always one byte for the header
                    }

                    #[$doc]
                    pub mod $name_lower {
                        use core::fmt;


                        const HEADER_LEN: usize = super::$name::HEADER_LEN;


                        /// Used to read from the register
                        pub struct R(pub(crate) [u8; HEADER_LEN + $len]);

                        impl R {
                            $(
                                #[$field_doc]
                                pub fn $field(&self) -> $ty {
                                    use core::mem::size_of;
                                    use crate::register_bank::FromBytes;

                                    // The index (in the register data) of the first
                                    // byte that contains a part of this field.
                                    const START: usize = $first_bit / 8;

                                    // The index (in the register data) of the byte
                                    // after the last byte that contains a part of this
                                    // field.
                                    const END: usize = $last_bit  / 8 + 1;

                                    // The number of bytes in the register data that
                                    // contain part of this field.
                                    const LEN: usize = END - START;

                                    // Get all bytes that contain our field. The field
                                    // might fill out these bytes completely, or only
                                    // some bits in them.
                                    let mut bytes = [0; LEN];
                                    bytes[..LEN].copy_from_slice(
                                        &self.0[START+HEADER_LEN .. END+HEADER_LEN]
                                    );

                                    // Before we can convert the field into a number and
                                    // return it, we need to shift it, to make sure
                                    // there are no other bits to the right of it. Let's
                                    // start by determining the offset of the field
                                    // within a byte.
                                    const OFFSET_IN_BYTE: usize = $first_bit % 8;

                                    if OFFSET_IN_BYTE > 0 {
                                        // Shift the first byte. We always have at least
                                        // one byte here, so this always works.
                                        bytes[0] >>= OFFSET_IN_BYTE;

                                        // If there are more bytes, let's shift those
                                        // too.
                                        // We need to allow exceeding bitshifts in this
                                        // loop, as we run into that if `OFFSET_IN_BYTE`
                                        // equals `0`. Please note that we never
                                        // actually encounter that at runtime, due to
                                        // the if condition above.
                                        let mut i = 1;
                                        #[allow(arithmetic_overflow)]
                                        while i < LEN {
                                            bytes[i - 1] |=
                                                bytes[i] << 8 - OFFSET_IN_BYTE;
                                            bytes[i] >>= OFFSET_IN_BYTE;
                                            i += 1;
                                        }
                                    }

                                    // If the field didn't completely fill out its last
                                    // byte, we might have bits from unrelated fields
                                    // there. Let's erase those before doing the final
                                    // conversion into the field's data type.
                                    const SIZE_IN_BITS: usize =
                                        $last_bit - $first_bit + 1;
                                    const BITS_ABOVE_FIELD: usize =
                                        8 - (SIZE_IN_BITS % 8);
                                    const SIZE_IN_BYTES: usize =
                                        (SIZE_IN_BITS - 1) / 8 + 1;
                                    const LAST_INDEX: usize =
                                        SIZE_IN_BYTES - 1;
                                    if BITS_ABOVE_FIELD < 8 {
                                        // Need to allow exceeding bitshifts to make the
                                        // compiler happy. They're never actually
                                        // encountered at runtime, due to the if
                                        // condition.
                                        #[allow(arithmetic_overflow)]
                                        {
                                            bytes[LAST_INDEX] <<= BITS_ABOVE_FIELD;
                                            bytes[LAST_INDEX] >>= BITS_ABOVE_FIELD;
                                        }
                                    }

                                    // Now all that's left is to convert the bytes into
                                    // the field's type. Please note that methods for
                                    // converting numbers to/from bytes are coming to
                                    // stable Rust, so we might be able to remove our
                                    // custom infrastructure here. Tracking issue:
                                    // https://github.com/rust-lang/rust/issues/52963
                                    let bytes = if bytes.len() > size_of::<$ty>() {
                                        &bytes[..size_of::<$ty>()]
                                    }
                                    else {
                                        &bytes
                                    };
                                    <$ty as FromBytes>::from_bytes(bytes)
                                }
                            )*
                        }

                        impl fmt::Debug for R {
                            fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
                                write!(f, "0x")?;
                                for i in (0 .. $len).rev() {
                                    write!(f, "{:02x}", self.0[HEADER_LEN + i])?;
                                }

                                Ok(())
                            }
                        }


                        /// Used to write to the register
                        pub struct W(pub(crate) [u8; HEADER_LEN + $len]);

                        impl W {
                            $(
                                #[$field_doc]
                                pub fn $field(&mut self, value: $ty) -> &mut Self {
                                    use crate::register_bank::ToBytes;

                                    // Convert value into bytes
                                    let source = <$ty as ToBytes>::to_bytes(value);

                                    // Now, let's figure out where the bytes are located
                                    // within the register array.
                                    const START:          usize = $first_bit / 8;
                                    const END:            usize = $last_bit  / 8 + 1;
                                    const OFFSET_IN_BYTE: usize = $first_bit % 8;

                                    // Also figure out the length of the value in bits.
                                    // That's going to come in handy.
                                    const LEN: usize = $last_bit - $first_bit + 1;


                                    // We need to track how many bits are left in the
                                    // value overall, and in the value's current byte.
                                    let mut bits_left         = LEN;
                                    let mut bits_left_in_byte = 8;

                                    // We also need to track how many bits have already
                                    // been written to the current target byte.
                                    let mut bits_written_to_byte = 0;

                                    // Now we can take the bytes from the value, shift
                                    // them, mask them, and write them into the target
                                    // array.
                                    let mut source_i  = 0;
                                    let mut target_i  = START;
                                    while target_i < END {
                                        // Values don't always end at byte boundaries,
                                        // so we need to mask the bytes when writing to
                                        // the slice.
                                        // Let's start out assuming we can write to the
                                        // whole byte of the slice. This will be true
                                        // for the middle bytes of our value.
                                        let mut mask = 0xff;

                                        // Let's keep track of the offset we're using to
                                        // write to this byte. We're going to need it.
                                        let mut offset_in_this_byte = 0;

                                        // If this is the first byte we're writing to
                                        // the slice, we need to remove the lower bits
                                        // of the mask.
                                        if target_i == START {
                                            mask <<= OFFSET_IN_BYTE;
                                            offset_in_this_byte = OFFSET_IN_BYTE;
                                        }

                                        // If this is the last byte we're writing to the
                                        // slice, we need to remove the higher bits of
                                        // the mask. Please note that we could be
                                        // writing to _both_ the first and the last
                                        // byte.
                                        if target_i == END - 1 {
                                            let shift =
                                                8 - bits_left - offset_in_this_byte;
                                            mask <<= shift;
                                            mask >>= shift;
                                        }

                                        mask <<= bits_written_to_byte;

                                        // Read the value from `source`
                                        let value = source[source_i]
                                            >> 8 - bits_left_in_byte
                                            << offset_in_this_byte
                                            << bits_written_to_byte;

                                        // Zero the target bits in the slice, then write
                                        // the value.
                                        self.0[HEADER_LEN + target_i] &= !mask;
                                        self.0[HEADER_LEN + target_i] |= value & mask;

                                        // The number of bits that were expected to be
                                        // written to the target byte.
                                        let bits_needed = mask.count_ones() as usize;

                                        // The number of bits we actually wrote to the
                                        // target byte.
                                        let bits_used = bits_needed.min(
                                            bits_left_in_byte - offset_in_this_byte
                                        );

                                        bits_left -= bits_used;
                                        bits_written_to_byte += bits_used;

                                        // Did we use up all the bits in the source
                                        // byte? If so, we can move on to the next one.
                                        if bits_left_in_byte > bits_used {
                                            bits_left_in_byte -= bits_used;
                                        }
                                        else {
                                            bits_left_in_byte =
                                                8 - (bits_used - bits_left_in_byte);

                                            source_i += 1;
                                        }

                                        // Did we write all the bits in the target byte?
                                        // If so, we can move on to the next one.
                                        if bits_used == bits_needed {
                                            target_i += 1;
                                            bits_written_to_byte = 0;
                                        }
                                    }

                                    self
                                }
                            )*
                        }
                    }

                    impl_rw!($rw, $name, $name_lower, $len);
                )*
            }

            impl<'b, BUS> Registers<'b, BUS, $bank> {
                $(
                    #[$doc]
                    pub fn $name_lower(&mut self) -> RegAccessor<'_, 'b, [<$bank:lower>]::$name, BUS, $bank> {
                        RegAccessor(self, PhantomData)
                    }
                )*
            }
        }
    }
}

// Helper macro, used internally by `impl_register!`
macro_rules! impl_rw {
    (RO, $name:ident, $name_lower:ident, $len:expr) => {
        impl_rw!(@R, $name, $name_lower, $len);
    };
    (RW, $name:ident, $name_lower:ident, $len:expr) => {
        impl_rw!(@R, $name, $name_lower, $len);
        impl_rw!(@W, $name, $name_lower, $len);
    };

    (@R, $name:ident, $name_lower:ident, $len:expr) => {
        impl Readable for $name {
            type Read = $name_lower::R;

            fn read() -> Self::Read {
                $name_lower::R([0; Self::HEADER_LEN + $len])
            }

            fn buffer(r: &mut Self::Read) -> &mut [u8] {
                &mut r.0
            }
        }
    };
    (@W, $name:ident, $name_lower:ident, $len:expr) => {
        impl Writable for $name {
            type Write = $name_lower::W;

            fn write() -> Self::Write {
                $name_lower::W([0; Self::HEADER_LEN + $len])
            }

            fn buffer(w: &mut Self::Write) -> &mut [u8] {
                &mut w.0
            }
        }
    };
}
impl_register! {
    BANK0,
    0x11, 1, RW, DEVICE_CONFIG(device_config) { /// Device configuration register
        soft_reset_config, 0, 0, u8;  /// Software reset configuration (default 0, normal)
        reserved_0, 1, 3, u8; /// Reserved (0)
        spi_mode, 4, 4, u8;  /// SPI mode selection (default 0, mode 0 and 3)
        reserved_1, 5, 7, u8; /// Reserved (1)
    }
    0x13, 1, RW, DRIVE_CONFIG(drive_config) { /// Drive configuration register
        i2c_slew_rate, 0, 2, u8; /// Controls slew rate for output pin 14 in I2C mode only (default 1)
        spi_slew_rate, 3, 5, u8; /// Controls slew rate for output pin 14 in SPI or I3CSM mode, and for all other output pins (default 5)
        reserved_0, 6, 7, u8; /// Reserved (0)
    }
    0x14, 1, RW, INT_CONFIG(int_config) { /// Interrupt configuration register
        int1_polarity,  0, 0, u8;  /// INT1 interrupt polarity (default 0, active low)
        int1_drive_circuit, 1, 1, u8;  /// INT1 drive circuit (default 0, open drain)
        int1_mode,  2, 2, u8;  /// INT1 interrupt mode (default 0, pulsed mode)
        int2_polarity,  3, 3, u8;  /// INT2 interrupt polarity (default 0, active low)
        int2_drive_circuit, 4, 4, u8;  /// INT2 drive circuit (default 0, open drain)
        int2_mode,  5, 5, u8;  /// INT2 interrupt mode (default 0, pulsed mode)
        reserved_0, 6, 7, u8; /// Reserved (0)
    }
    0x16, 1, RW, FIFO_CONFIG(fifo_config) { /// FIFO configuration register
        fifo_mode, 6, 7, u8;  /// FIFO mode selection. 00: Bypass Mode (default) 01: Stream-to-FIFO Mode 10: STOP-on-FULL Mode 11: STOP-on-FULL Mode
        reserved_0, 0, 5, u8; /// Reserved (0)
    }
    0x1D, 1, RO, TEMP_DATA1(temp_data1) { /// Temperature data output register 1
        temp_data_15_8, 0, 7, u8; /// Upper byte of temperature data
    }
    0x1E, 1, RO, TEMP_DATA0(temp_data0) { /// Temperature data output register 0
        temp_data_7_0, 0, 7, u8; /// Lower byte of temperature data
    }
    0x1F, 1, RO, ACCEL_DATA_X1(accel_data_x1) { /// Accelerometer data output register X 1
        accel_data_x_15_8, 0, 7, u8; /// Upper byte of Accel X-axis data
    }
    0x20, 1, RO, ACCEL_DATA_X0(accel_data_x0) { /// Accelerometer data output register X 0
        accel_data_x_7_0, 0, 7, u8; /// Lower byte of Accel X-axis data
    }
    0x21, 1, RO, ACCEL_DATA_Y1(accel_data_y1) { /// Accelerometer data output register Y 1
        accel_data_y_15_8, 0, 7, u8; /// Upper byte of Accel Y-axis data
    }
    0x22, 1, RO, ACCEL_DATA_Y0(accel_data_y0) { /// Accelerometer data output register Y 0
        accel_data_y_7_0, 0, 7, u8; /// Lower byte of Accel Y-axis data
    }
    0x23, 1, RO, ACCEL_DATA_Z1(accel_data_z1) { /// Accelerometer data output register Z 1
        accel_data_z_15_8, 0, 7, u8; /// Upper byte of Accel Z-axis data
    }
    0x24, 1, RO, ACCEL_DATA_Z0(accel_data_z0) { /// Accelerometer data output register Z 0
        accel_data_z_7_0, 0, 7, u8; /// Lower byte of Accel Z-axis data
    }
    0x25, 1, RO, GYRO_DATA_X1(gyro_data_x1) { /// Gyroscope data output register X 1
        gyro_data_x_15_8, 0, 7, u8; /// Upper byte of Gyro X-axis data
    }
    0x26, 1, RO, GYRO_DATA_X0(gyro_data_x0) { /// Gyroscope data output register X 0
        gyro_data_x_7_0, 0, 7, u8; /// Lower byte of Gyro X-axis data
    }
    0x27, 1, RO, GYRO_DATA_Y1(gyro_data_y1) { /// Gyroscope data output register Y 1
        gyro_data_y_15_8, 0, 7, u8; /// Upper byte of Gyro Y-axis data
    }
    0x28, 1, RO, GYRO_DATA_Y0(gyro_data_y0) { /// Gyroscope data output register Y 0
        gyro_data_y_7_0, 0, 7, u8; /// Lower byte of Gyro Y-axis data
    }
    0x29, 1, RO, GYRO_DATA_Z1(gyro_data_z1) { /// Gyroscope data output register Z 1
        gyro_data_z_15_8, 0, 7, u8; /// Upper byte of Gyro Z-axis data
    }
    0x2A, 1, RO, GYRO_DATA_Z0(gyro_data_z0) { /// Gyroscope data output register Z 0
        gyro_data_z_7_0, 0, 7, u8; /// Lower byte of Gyro Z-axis data
    }
    0x2B, 1, RO, TMST_FSYNCH(tmst_fsynch) { /// Time stamp data output register FSYNC High
        tmst_fsync_data_ui_15_8, 0, 7, u8; /// Stores the upper byte of the time delta from the rising edge of FSYNC to the latest ODR until the UI Interface reads the FSYNC tag in the status register
    }
    0x2C, 1, RO, TMST_FSYNCL(tmst_fsyncl) { /// Time stamp data output register FSYNC Low
        tmst_fsync_data_ui_7_0, 0, 7, u8; /// Stores the lower byte of the time delta from the rising edge of FSYNC to the latest ODR until the UI Interface reads the FSYNC tag in the status register
    }
    0x2D, 1, RO, INT_STATUS(int_status) { /// Interrupt status register
        agc_rdy_int, 0, 0, u8;  /// This bit automatically sets to 1 when an AGC Ready interrupt is generated. The bit clears to 0 after the register has been read.
        fifo_full_int, 1, 1, u8;  /// This bit automatically sets to 1 when the FIFO buffer is full.  The bit clears to 0 after the register has been read.
        fifo_ths_int, 2, 2, u8;  /// This bit automatically sets to 1 when the FIFO buffer reaches the threshold value.  The bit clears to 0 after the register has been read.
        data_rdy_int, 3, 3, u8;  /// This bit automatically sets to 1 when a Data Ready interrupt is generated.  The bit clears to 0 after the register has been read.
        reset_done_int, 4, 4, u8;  /// This bit automatically sets to 1 when software reset is complete.  The bit clears to 0 after the register has been read.
        pll_rdy_int, 5, 5, u8;  /// This bit automatically sets to 1 when a PLL Ready interrupt is generated.  The bit clears to 0 after the register has been read.
        ui_fsync_int, 6, 6, u8;  /// This bit automatically sets to 1 when a UI FSYNC interrupt is generated.  The bit clears to 0 after the register has been read.
    }
    0x2E, 1, RO, FIFO_COUNTH(fifo_counth) { /// FIFO count register High
        fifo_count_15_8, 0, 7, u8; /// High Bits, count indicates the number of records or bytes available in FIFO according to FIFO_COUNT_REC setting. Reading this byte latches the data for both FIFO_COUNTH, and FIFO_COUNTL.
    }
    0x2F, 1, RO, FIFO_COUNTL(fifo_countl) { /// FIFO count register Low
        fifo_count_7_0, 0, 7, u8; /// Low Bits, count indicates the number of records or bytes available in FIFO according to FIFO_COUNT_REC setting. Note:  Must read FIFO_COUNTH to latch new data for both FIFO_COUNTH and FIFO_COUNTL.
    }
    0x30, 1, RO, FIFO_DATA(fifo_data) { /// FIFO data register
        fifo_data, 0, 7, u8; /// FIFO data port
    }
    0x31, 1, RO, APEX_DATA0(apex_data0) { /// APEX data output register 0
        step_cnt_7_0, 0, 7, u8; /// Pedometer Output: Lower byte of Step Count measured by pedometer
    }
    0x32, 1, RO, APEX_DATA1(apex_data1) { /// APEX data output register 1
        step_cnt_15_8, 0, 7, u8; /// Pedometer Output: Upper byte of Step Count measured by pedometer
    }
    0x33, 1, RO, APEX_DATA2(apex_data2) { /// APEX data output register 2
        step_cadence, 0, 7, u8; /// Pedometer Output: Walk/run cadency in number of samples.  Format is u6.2.  e.g. At 50Hz ODR and 2Hz walk frequency, the cadency is 25 samples. The register will output 100.
    }
    0x34, 1, RO, APEX_DATA3(apex_data3) { /// APEX data output register 3
        reserved_0, 3, 7, u8;  /// Reserved (0)
        dmp_idle, 2, 2, u8;  /// 0: Indicates DMP is running 1: Indicates DMP is idle
        activity_class, 0, 1, u8; /// Pedometer Output: Detected activity 00: Unknown 01: Walk 10: Run 11: Reserved
    }
    0x35, 1, RO, APEX_DATA4(apex_data4) { /// APEX data output register 4
        reserved_0, 5, 7, u8;  /// Reserved (0)
        tap_dir, 0, 0, u8;  /// Tap Detection Output: Polarity of tap pulse 0: Current accelerometer value – Previous accelerometer value is a positive value 1: Current accelerometer value – Previous accelerometer value is a negative value or zero
        tap_axis, 1, 2, u8;  /// Tap Detection Output: Represents the accelerometer axis on which tap energy is concentrated 00: X-axis 01: Y-axis 10: Z-axis 11: Reserved
        tap_num, 3, 4, u8;  /// Tap Detection Output: Number of taps in the current Tap event 00: No tap 01: Single tap 10: Double tap 11: Reserved
    }
    0x36, 1, RO, APEX_DATA5(apex_data5) { /// APEX data output register 5
        reserved_0, 6, 7, u8;  /// Reserved (0)
        double_tap_timing, 0, 5, u8; /// DOUBLE_TAP_TIMING measures the time interval between the two taps when double tap is detected.  It counts every 16 accelerometer samples as one unit between the 2 tap pulses. Therefore, the value is related to the accelerometer ODR.
    }
    0x37, 1, RO, INT_STATUS2(int_status2) { /// Interrupt status register 2
        wom_x_int, 0, 0, u8;  /// Wake on Motion Interrupt on X-axis, clears on read
        wom_y_int, 1, 1, u8;  /// Wake on Motion Interrupt on Y-axis, clears on read
        wom_z_int, 2, 2, u8;  /// Wake on Motion Interrupt on Z-axis, clears on read
        smd_int, 3, 3, u8;  /// Significant Motion Detection Interrupt, clears on read
        reserved, 4, 7, u8;  /// Reserved
    }
    0x38, 1, RO, INT_STATUS3(int_status3) { /// Interrupt status register 3
        tap_det_int, 0, 0, u8;  /// Tap Detection Interrupt, clears on read
        sleep_int, 1, 1, u8;  /// Sleep Event Interrupt, clears on read
        wake_int, 2, 2, u8;  /// Wake Event Interrupt, clears on read
        tilt_det_int, 3, 3, u8;  /// Tilt Detection Interrupt, clears on read
        step_cnt_ovf_int, 4, 4, u8;  /// Step Count Overflow Interrupt, clears on read
        step_det_int, 5, 5, u8;  /// Step Detection Interrupt, clears on read
        reserved_1, 6, 7, u8;  /// Reserved (0)
    }
    0x4B, 1, RW, SIGNAL_PATH_RESET(signal_path_reset) { /// Signal path reset register
        reserved_0, 0, 0, u8;  /// Reserved (0)
        fifo_flush, 1, 1, u8;  /// When set to 1, FIFO will get flushed.
        tmst_strobe, 2, 2, u8;  /// When this bit is set to 1, the time stamp counter is latched into the time stamp register. This is a write on clear bit.
        abort_and_reset, 3, 3, u8;  /// When this bit is set to 1, the signal path is reset by restarting the ODR counter and signal path controls
        reserved_1, 4, 4, u8;  /// Reserved (0)
        dmp_mem_reset_en, 5, 5, u8;  /// When this bit is set to 1, the DMP memory is reset
        dmp_init_en, 6, 6, u8;  /// When this bit is set to 1, the DMP is enabled
        reserved_2, 7, 7, u8;  /// Reserved (0)
    }
    0x4C, 1, RW, INTF_CONFIG0(intf_config0) { /// Interface configuration register 0
        ui_sifs_cfg, 0, 1, u8;  /// 0x: Reserved 10: Disable SPI 11: Disable I2C
        reserved_0, 2, 3, u8;  /// Reserved (0)
        sensor_data_endian, 4, 4, u8;  /// 0: Sensor data is reported in Little Endian format 1: Sensor data is reported in Big Endian format (default)
        fifo_count_endian, 5, 5, u8;  /// 0: FIFO count is reported in Little Endian format 1: FIFO count is reported in Big Endian format (default)
        fifo_count_rec, 6, 6, u8;  /// 0: FIFO count is reported in bytes 1: FIFO count is reported in records (1 record = 16 bytes for header + gyro + accel + temp sensor data + time stamp, or 8 bytes for header + gyro/accel + temp sensor data, or 20 bytes for header + gyro + accel + temp sensor data + time stamp + 20-bit extension data)
        fifo_hold_last_data_en, 7, 7, u8;  /// This bit selects the treatment of invalid samples.  See Invalid Data Generation note below this register description.  Setting this bit to 0:  In order to signal an invalid sample, and to differentiate it from a valid sample based on values only:  Sense Registers:
    }
    0x4D, 1, RW, INTF_CONFIG1(intf_config1) { /// Interface configuration register 1
        clkssel, 0, 1, u8;  /// 00: Always select internal RC oscillator 01: Select PLL when available, else select RC oscillator (default) 10: Reserved 11: Disable all clocks
        rtc_mode, 2, 2, u8;  /// 0: No input RTC clock is required 1: RTC clock input is required
        accel_lp_clk_sel, 3, 3, u8;  /// 0: Accelerometer LP mode uses Wake Up oscillator clock 1: Accelerometer LP mode uses RC oscillator clock
        reserved_2, 4, 5, u8;  /// Reserved (0)
        afsr, 6, 7, u8; /// AFSR (undocumented, default 10: AFSR enabled, 01: AFSR disabled)
    }
    0x4E, 1, RW, PWR_MGMT0(pwr_mgmt0) { /// Power management register 0
        accel_mode, 0, 1, u8;  /// 00: Turns accelerometer off (default) 01: Turns accelerometer off 10: Places accelerometer in Low Power (LP) Mode 11: Places accelerometer in Low Noise (LN) Mode  When transitioning from OFF to any of the other modes, do not issue any register writes for 200µs.
        gyro_mode, 2, 3, u8;  /// 00: Turns gyroscope off (default) 01: Places gyroscope in Standby Mode 10: Reserved 11: Places gyroscope in Low Noise (LN) Mode  Gyroscope needs to be kept ON for a minimum of 45ms. When transitioning from OFF to any of the other modes, do not issue any register writes for 200µs.
        idle, 4, 4, u8;  /// If this bit is set to 1, the RC oscillator is powered on even if Accel and Gyro are powered off.  Nominally this bit is set to 0, so when Accel and Gyro are powered off,  the chip will go to OFF state, since the RC oscillator will also be powered off
        temp_dis, 5, 5, u8;  /// 0: Temperature sensor is enabled (default) 1: Temperature sensor is disabled
        reserved_0, 6, 7, u8;  /// Reserved (0)
    }
    0x4F, 1, RW, GYRO_CONFIG0(gyro_config0) { /// Gyroscope configuration register 0
        gyro_odr, 0, 3, u8;  /// Gyroscope ODR selection for UI interface output 0000: Reserved 0001: 32kHz 0010: 16kHz 0011: 8kHz 0100: 4kHz 0101: 2kHz 0110: 1kHz (default) 0111: 200Hz  1000: 100Hz 1001: 50Hz 1010: 25Hz 1011: 12.5Hz 1100: Reserved 1101: Reserved 1110: Reserved 1111: 500Hz
        reserved_0, 4, 4, u8;  /// Reserved (0)
        gyro_fs_sel, 5, 7, u8;  /// Full scale select for gyroscope UI interface output 000: ±2000dps (default) 001: ±1000dps 010: ±500dps 011: ±250dps 100: ±125dps 101: ±62.5dps 110: ±31.25dps 111: ±15.625dps
    }
    0x50, 1, RW, ACCEL_CONFIG0(accel_config0) { /// Accelerometer configuration register 0
        accel_odr, 0, 3, u8;  /// Accelerometer ODR selection for UI interface output 0000: Reserved 0001: 32kHz (LN mode) 0010: 16kHz (LN mode) 0011: 8kHz (LN mode) 0100: 4kHz (LN mode) 0101: 2kHz (LN mode) 0110: 1kHz (LN mode) (default) 0111: 200Hz (LP or LN mode)  1000: 100Hz (LP or LN mode) 1001: 50Hz (LP or LN mode) 1010: 25Hz (LP or LN mode) 1011: 12.5Hz (LP or LN mode) 1100: 6.25Hz (LP mode) 1101: 3.125Hz (LP mode) 1110: 1.5625Hz (LP mode) 1111: 500Hz (LP or LN mode)
        reserved_0, 4, 4, u8;  /// Reserved (0)
        accel_fs_sel, 5, 7, u8;  /// Full scale select for accelerometer UI interface output 000: ±16g (default) 001: ±8g 010: ±4g 011: ±2g 100: Reserved 101: Reserved 110: Reserved 111: Reserved
    }
    0x51, 1, RW, GYRO_CONFIG1(gyro_config1) { /// Gyroscope configuration register 1
        gyro_dec2_m2_ord, 0, 1, u8;  /// Selects order of GYRO DEC2_M2 Filter 00: Reserved 01: Reserved 10: 3rd Order 11: Reserved
        gyro_ui_filt_ord, 2, 3, u8;  /// Selects order of GYRO UI filter 00: 1st Order 01: 2nd Order 10: 3rd Order 11: Reserved
        reserved_0, 4, 4, u8;  /// Reserved (0)
        temp_filt_bw, 5, 7, u8;  /// Sets the bandwidth of the temperature signal DLPF 000: DLPF BW = 4000Hz; DLPF Latency = 0.125ms (default) 001: DLPF BW = 170Hz; DLPF Latency = 1ms 010: DLPF BW = 82Hz; DLPF Latency = 2ms 011: DLPF BW = 40Hz; DLPF Latency = 4ms 100: DLPF BW = 20Hz; DLPF Latency = 8ms 101: DLPF BW = 10Hz; DLPF Latency = 16ms 110: DLPF BW = 5Hz; DLPF Latency = 32ms 111: DLPF BW = 5Hz; DLPF Latency = 32ms
    }
    0x52, 1, RW, GYRO_ACCEL_CONFIG0(gyro_accel_config0) { /// Gyroscope/Accelerometer configuration register 0
        gyro_ui_filt_bw, 0, 3, u8;  /// LN Mode: Bandwidth for Gyro LPF 0 BW=ODR/2 1 BW=max(400Hz, ODR)/4 (default) 2 BW=max(400Hz, ODR)/5 3 BW=max(400Hz, ODR)/8 4 BW=max(400Hz, ODR)/10 5 BW=max(400Hz, ODR)/16 6 BW=max(400Hz, ODR)/20 7 BW=max(400Hz, ODR)/40 8 to 13:  Reserved 14 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)  15 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)
        accel_ui_filt_bw, 4, 7, u8;  /// LN Mode: Bandwidth for Accel LPF 0 BW=ODR/2 1 BW=max(400Hz, ODR)/4 (default) 2 BW=max(400Hz, ODR)/5 3 BW=max(400Hz, ODR)/8 4 BW=max(400Hz, ODR)/10 5 BW=max(400Hz, ODR)/16 6 BW=max(400Hz, ODR)/20 7 BW=max(400Hz, ODR)/40 8 to 13: Reserved 14 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)  15 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)  LP Mode: 0 Reserved 1 1x AVG filter (default) 2 to 5 Reserved 6 16x AVG filter 7 to 15 Reserved
    }
    0x53, 1, RW, ACCEL_CONFIG1(accel_config1) { /// Accelerometer configuration register 1
        reserved_0, 0, 0, u8;  /// Reserved (0)
        accel_dec2_m2_ord, 1, 2, u8;  /// Order of Accelerometer DEC2_M2 filter 00: Reserved 01: Reserved 10: 3rd order 11: Reserved
        accel_ui_filt_ord, 3, 4, u8;  /// Selects order of ACCEL UI filter 00: 1st Order 01: 2nd Order 10: 3rd Order 11: Reserved
        reserved_1, 5, 7, u8;  /// Reserved (0)
    }
    0x54, 1, RW, TMST_CONFIG(tmst_config) { /// Time stamp configuration register
        tmst_en, 0, 0, u8;  /// 0: Time Stamp register disable 1: Time Stamp register enable (default)
        tmst_fsync_en, 1, 1, u8;  /// Time Stamp register FSYNC enable (default). When set to 1, the contents of the Timestamp feature of FSYNC is enabled. The user also needs to select  FIFO_TMST_FSYNC_EN in order to propagate the timestamp value to the  FIFO.
        tmst_delta_en, 2, 2, u8;  /// Time Stamp delta enable: When set to 1, the time stamp field contains the  measurement of time since  the last occurrence of ODR.
        tmst_res, 3, 3, u8;  /// Time Stamp resolution:  When set to 0 (default), time stamp resolution is 1 µs.  When set to 1: If RTC is disabled, resolution is 16µs. If RTC is enabled, resolution is 1 RTC clock period
        tmst_to_regs_en, 4, 4, u8;  /// 0: TMST_VALUE\[19:0\] read always returns 0s 1: TMST_VALUE\[19:0\] read returns timestamp value
        reserved_1, 5, 7, u8;  /// Reserved (0)
    }
    0x56, 1, RW, APEX_CONFIG0(apex_config0) { /// APEX configuration register 0
        dmp_odr, 0, 1, u8;  /// 00: 25Hz 01: Reserved 10: 50Hz 11: Reserved
        reserved_0, 2, 2, u8;  /// Reserved (0)
        r2w_en, 3, 3, u8;  /// 0: Raise to Wake/Sleep not enabled 1: Raise to Wake/Sleep enabled
        tilt_enable, 4, 4, u8;  /// 0: Tilt Detection not enabled 1: Tilt Detection enabled
        ped_enable, 5, 5, u8;  /// 0: Pedometer not enabled 1: Pedometer enabled
        tap_enable, 6, 6, u8;  /// 0: Tap Detection not enabled 1: Tap Detection enabled when accelerometer ODR is set to one of the ODR values supported by Tap Detection (200Hz, 500Hz, 1kHz)
        dmp_power_save, 7, 7, u8;  /// 0: DMP power save mode not active 1: DMP power save mode active (default)
    }
    0x57, 1, RW, SMD_CONFIG(smd_config) { /// SMD configuration register
        smd_mode, 0, 1, u8;  /// 00: SMD disabled 01: Reserved 10: SMD short (1 sec wait) An SMD event is detected when two WOM are detected 1 sec apart 11: SMD long (3 sec wait) An SMD event is detected when two WOM are detected 3 sec apart
        wom_mode, 2, 2, u8;  /// 0: Initial sample is stored. Future samples are compared to initial sample 1: Compare current sample to previous sample
        wom_int_mode, 3, 3, u8;  /// 0: Set WoM interrupt on the OR of all enabled accelerometer thresholds 1: Set WoM interrupt on the AND of all enabled accelerometer threshold
        reserved_0, 4, 7, u8;  /// Reserved (0)
    }
    0x5F, 1, RW, FIFO_CONFIG1(fifo_config1) { /// FIFO configuration register 1
        fifo_accel_en, 0, 0, u8;  /// Enable accelerometer packets to go to FIFO
        fifo_gyro_en, 1, 1, u8;  /// Enable gyroscope packets to go to FIFO
        fifo_temp_en, 2, 2, u8;  /// Enable temperature sensor packets to go to FIFO
        fifo_tmst_fsync_en, 3, 3, u8;  /// Must be set to 1 for all FIFO use cases when FSYNC is used.
        fifo_hires_en, 4, 4, u8;  /// Enable 3 bytes of extended 20-bits accel, gyro data + 1 byte of extended 16-bit temperature sensor data to be placed into the FIFO
        fifo_wm_gt_th, 5, 5, u8;  /// Trigger FIFO watermark interrupt on every ODR (DMA write) if FIFO_COUNT ≥ FIFO_WM_TH
        fifo_resume_partial_rd, 6, 6, u8;  /// 0: Partial FIFO read disabled, requires re-reading of the entire FIFO 1: FIFO read can be partial, and resume from last read point
        reserved_0, 7, 7, u8;  /// Reserved (0)
    }
    0x60, 1, RW, FIFO_CONFIG2(fifo_config2) { /// FIFO configuration register 2
        fifo_wm_7_0, 0, 7, u8; /// Lower bits of FIFO watermark.  Generate interrupt when the FIFO reaches or exceeds FIFO_WM size in bytes or records according to FIFO_COUNT_REC setting.  Interrupt only fires once.  This register should be set to non-zero value, before choosing this interrupt source.
    }
    0x61, 1, RW, FIFO_CONFIG3(fifo_config3) { /// FIFO configuration register 3
        reserved_0, 4, 7, u8;  /// Reserved (0)
        fifo_wm_11_8, 0, 3, u8; /// Upper bits of FIFO watermark.  Generate interrupt when the FIFO reaches or exceeds FIFO_WM size in bytes or records according to FIFO_COUNT_REC setting.  Interrupt only fires once.  This register should be set to non-zero value, before choosing this interrupt source.
    }
    0x62, 1, RW, FSYNC_CONFIG(fsync_config) { /// FSYNC configuration register
        fsync_polarity, 0, 0, u8;  /// 0: Start from rising edge of FSYNC pulse to measure FSYNC interval. 1: Start from falling edge of FSYNC pulse to measure FSYNC interval
        fsync_ui_flag_clear_sel, 1, 1, u8;  /// 0: Clear FSYNC flag when UI sensor register is updated. 1: Clear FSYNC flag when UI interface reads the sensor register LSB of FSYNC tagged axis
        reserved, 2, 3, u8;  /// Reserved (0)
        fsync_ui_sel, 4, 6, u8; /// Select FSYNC tag axis. 000: Disable; 001: TEMP_OUT LSB; 010: GYRO_XOUT LSB; 011: GYRO_YOUT LSB; 100: GYRO_ZOUT LSB; 101: ACCEL_XOUT LSB; 110: ACCEL_YOUT LSB; 111: ACCEL_ZOUT LSB
    }
    0x63, 1, RW, INT_CONFIG0(int_config0) { /// INT pin / interrupt configuration register
        fifo_full_int_clear, 0, 1, u8;  /// 00/01: Clear on status bit read; 10: Clear on FIFO 1 byte read; 11: Clear on both
        fifo_ths_int_clear, 2, 3, u8;  /// 00/01: Clear on status bit read; 10: Clear on FIFO 1 byte read; 11: Clear on both
        ui_drdy_int_clear, 4, 5, u8;  /// 00/01: Clear on status bit read; 10: Clear on FIFO 1 byte read; 11: Clear on both
        reserved_0, 6, 7, u8;  /// Reserved (0)
    }
    0x64, 1, RW, INT_CONFIG1(int_config1) { /// INT pin / interrupt configuration register
        reserved_0, 0, 3, u8;  /// Reserved (0)
        int_async_reset, 4, 4, u8;  /// NOTE: User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation!
        int_tdeassert_disable, 5, 5, u8;  /// Interrupt de-assertion duration. 0: minimum of 100µs (DO NOT USE with ODR >= 4kHz); 1: disabled (can be used with any ODR)
    }
    0x65, 1, RW, INT_SOURCE0(int_source0) { /// INT pin / interrupt source register
        ui_agc_rdy_int1_en, 0, 0, u8;  /// Enable interrupt generation on UI AGC ready status
        fifo_full_int1_en, 1, 1, u8;  /// Enable interrupt generation on FIFO full status
        fifo_ths_int1_en, 2, 2, u8;  /// Enable interrupt generation on FIFO threshold status
        ui_drdy_int1_en, 3, 3, u8;  /// Enable interrupt generation on UI data ready status
        reset_done_int1_en, 4, 4, u8;  /// Enable interrupt generation on reset done status
        pll_rdy_int1_en, 5, 5, u8;  /// Enable interrupt generation on PLL ready status
        ui_fsync_int1_en, 6, 6, u8;  /// Enable interrupt generation on UI FSYNC status
        reserved_0, 7, 7, u8;  /// Reserved (0)
    }
    0x70, 1, RW, SELF_TEST_CONFIG(self_test_config) { /// Self-test configuration register
        en_gx_st, 0, 0, u8;  /// Enable gyroscope X-axis self-test (default 0, disabled)
        en_gy_st, 1, 1, u8;  /// Enable gyroscope Y-axis self-test (default 0, disabled)
        en_gz_st, 2, 2, u8;  /// Enable gyroscope Z-axis self-test (default 0, disabled)
        en_ax_st, 3, 3, u8;  /// Enable accelerometer X-axis self-test (default 0, disabled)
        en_ay_st, 4, 4, u8;  /// Enable accelerometer Y-axis self-test (default 0, disabled)
        en_az_st, 5, 5, u8;  /// Enable accelerometer Z-axis self-test (default 0, disabled)
        accel_st_power, 6, 6, u8;  /// Accelerometer self-test power mode (default 0, normal). Set to 0 after self-test is complete.
        reserved_0, 7, 7, u8;  /// Reserved (0)
    }
    0x75, 1, RO, WHO_AM_I(who_am_i) { /// Who am I register
        value, 0, 7, u8;  /// Who am I value
    }
    0x76, 1, RW, REG_BANK_SEL(reg_bank_sel) { /// Register bank selection register
        bank_sel, 0, 2, u8;  /// Register bank selection
        reserved_0, 3, 7, u8;  /// Reserved (0)
    }
}

impl_register! {
    BANK1,
    0x0B, 1, RW, GYRO_CONFIG_STATIC2(gyro_config_static2) { /// Gyroscope static configuration register
        gyro_nf_dis, 0, 0, u8;  /// Gyroscope notch filter disable
        gyro_aaf_dis, 1, 1, u8;  /// Gyroscope anti-aliasing filter disable
        reserved_0, 2, 7, u8;  /// Reserved (0)
    }
    0x0C, 1, RW, GYRO_CONFIG_STATIC3(gyro_config_static3) { /// Gyroscope static configuration register
        gyro_aaf_delt, 0, 5, u8;  /// Gyroscope anti-alias filter bandwidth selection, see Section 5.2 of the datasheet
        reserved_0, 6, 7, u8;  /// Reserved (0)
    }
    0x0D, 1, RW, GYRO_CONFIG_STATIC4(gyro_config_static4) { /// Gyroscope static configuration register
        gyro_aaf_deltsqr_7_0, 0, 7, u8;  /// Gyroscope anti-alias filter bandwidth selection, see Section 5.2 of the datasheet
    }
    0x0E, 1, RW, GYRO_CONFIG_STATIC5(gyro_config_static5) { /// Gyroscope static configuration register
        gyro_aaf_deltsqr_11_8, 0, 3, u8;  /// Gyroscope anti-alias filter bandwidth selection, see Section 5.2 of the datasheet
        gyro_aaf_bitshift, 4, 7, u8;  /// Gyroscope anti-alias filter bandwidth selection, see Section 5.2 of the datasheet
    }
    0x76, 1, RW, REG_BANK_SEL(reg_bank_sel) { /// Register bank selection register
        bank_sel, 0, 2, u8;  /// Register bank selection
        reserved_0, 3, 7, u8;  /// Reserved (0)
    }
    0x7B, 1, RW, INTF_CONFIG5(intf_config5) { /// Interface configuration register
        reserved_0, 0, 0, u8;  /// Reserved (0)
        pin9_function, 1, 2, u8;  /// Pin 9 function selection. 00: INT2; 01: FSYNC; 10: CLKIN, 11: Reserved
        reserved_1, 3, 7, u8;  /// Reserved (0)
    }
}

impl_register! {
    BANK2,
    0x03, 1, RW, ACCEL_CONFIG_STATIC2(accel_config_static2) { /// Accelerometer static configuration register
        accel_aaf_dis, 0, 0, u8;  /// Accelerometer notch filter disable
        accel_aaf_delt, 1, 6, u8;  /// Control for accelerometer anti-alias filter bandwidth selection, see Section 5.2 of the datasheet
        reserved, 7, 7, u8;  /// Reserved (0)
    }
    0x04, 1, RW, ACCEL_CONFIG_STATIC3(accel_config_static3) { /// Accelerometer static configuration register
        accel_aaf_deltsqr_7_0, 0, 7, u8;  /// Control for accelerometer anti-alias filter bandwidth selection, see Section 5.2 of the datasheet
    }
    0x05, 1, RW, ACCEL_CONFIG_STATIC4(accel_config_static4) { /// Accelerometer static configuration register
        accel_aaf_deltsqr_11_8, 0, 3, u8;  /// Control for accelerometer anti-alias filter bandwidth selection, see Section 5.2 of the datasheet
        accel_aaf_bitshift, 4, 7, u8;  /// Control for accelerometer anti-alias filter bandwidth selection, see Section 5.2 of the datasheet
    }
    0x76, 1, RW, REG_BANK_SEL(reg_bank_sel) { /// Register bank selection register
        bank_sel, 0, 2, u8;  /// Register bank selection
        reserved_0, 3, 7, u8;  /// Reserved (0)
    }
}

/// Internal trait used by `impl_registers!`
trait FromBytes {
    fn from_bytes(bytes: &[u8]) -> Self;
}

/// Internal trait used by `impl_registers!`
trait ToBytes {
    type Bytes;

    fn to_bytes(self) -> Self::Bytes;
}

/// Internal macro used to implement `FromBytes`/`ToBytes`
macro_rules! impl_bytes {
    ($($ty:ty,)*) => {
        $(
            impl FromBytes for $ty {
                fn from_bytes(bytes: &[u8]) -> Self {
                    let mut val = 0;

                    for (i, &b) in bytes.iter().enumerate() {
                        val |= (b as $ty) << (i * 8);
                    }

                    val
                }
            }

            impl ToBytes for $ty {
                type Bytes = [u8; ::core::mem::size_of::<$ty>()];

                fn to_bytes(self) -> Self::Bytes {
                    let mut bytes = [0; ::core::mem::size_of::<$ty>()];

                    for (i, b) in bytes.iter_mut().enumerate() {
                        let shift = 8 * i;
                        let mask  = 0xff << shift;

                        *b = ((self & mask) >> shift) as u8;
                    }

                    bytes
                }
            }
        )*
    }
}

impl_bytes! {
    u8,
    u16,
    u32,
    u64,
    u128,
}
