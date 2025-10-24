#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice;

#[cfg(not(feature = "async"))]
use embedded_hal::spi::SpiDevice;

use crate::{
    fifo::{FifoPacket4, Sample},
    register_bank::{bank0::int_status, Register},
    Error, Ready, ICM42688,
};

impl<SPI> ICM42688<SPI, Ready>
where
    SPI: SpiDevice,
{
    #[cfg(feature = "async")]
    pub async fn reset_fifo(&mut self) -> Result<(), SPI::Error> {
        let mut bank0 = self.ll.bank::<0>();
        bank0
            .signal_path_reset()
            .async_modify(|_, w| w.fifo_flush(1))
            .await?;
        Ok(())
    }

    #[cfg(not(feature = "async"))]
    pub fn reset_fifo(&mut self) -> Result<(), SPI::Error> {
        let mut bank0 = self.ll.bank::<0>();
        bank0.signal_path_reset().modify(|_, w| w.fifo_flush(1))?;
        Ok(())
    }

    #[cfg(feature = "async")]
    pub async fn read_fifo_count(&mut self) -> Result<u16, SPI::Error> {
        let mut bank0 = self.ll.bank::<0>();
        let count_h = bank0.fifo_counth().async_read().await?.fifo_count_15_8();
        let count_l = bank0.fifo_countl().async_read().await?.fifo_count_7_0();
        Ok(((count_h as u16) << 8) | count_l as u16)
    }

    #[cfg(not(feature = "async"))]
    pub fn read_fifo_count(&mut self) -> Result<u16, SPI::Error> {
        let mut bank0 = self.ll.bank::<0>();
        let count_h = bank0.fifo_counth().read()?.fifo_count_15_8();
        let count_l = bank0.fifo_countl().read()?.fifo_count_7_0();
        Ok(((count_h as u16) << 8) | count_l as u16)
    }

    /// Read data from the FIFO
    ///
    /// NOTE: Only the 4-byte packet mode is supported
    ///
    /// Buffer must hold at least
    #[cfg(feature = "async")]
    pub async fn read_fifo(
        &mut self,
        buffer: &mut [u32],
    ) -> Result<usize, SPI::Error> {
        /// We read INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, and then the data in
        /// one go
        const INT_STATUS_ADDR: u8 = crate::register_bank::bank0::INT_STATUS::ID;

        let buffer = bytemuck::cast_slice_mut::<u32, u8>(buffer);

        buffer[0] = INT_STATUS_ADDR | 0x80; // Read bit set

        self.ll.bus.transfer_in_place(buffer).await?;

        // Buffer now contains [0, INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, DATA,
        // DATA, ...] We need to check the FIFO_COUNT and then return the number
        // of samples read
        let fifo_count = ((buffer[2] as u16) << 8) | (buffer[3] as u16);

        Ok(fifo_count as usize / 20)
    }

    #[cfg(feature = "async")]
    /// Reads and parses a single sample from the FIFO.
    ///
    /// This function performs an efficient single SPI transaction to read the
    /// interrupt status, FIFO count, and the next available FIFO packet. It
    /// then parses this packet into a [`Sample`].
    ///
    /// # Returns
    ///
    /// - `Ok(Some((sample, more_data)))`: On a successful read and parse.
    ///   - `sample`: The [`Sample`] containing sensor data.
    ///   - `more_data`: A boolean flag that is `true` if the FIFO contains more
    ///     packets to be read.
    /// - `Ok(None)`: If the FIFO was empty or the packet read was invalid
    ///   (e.g., an "empty" marker).
    /// - `Err(Error::Bus(_))`: If a communication error occurs on the SPI bus.
    pub async fn read_sample(
        &mut self,
    ) -> Result<Option<(Sample, bool)>, Error<SPI::Error>> {
        // We read INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, and then the data in
        // one go. The leading byte in the buffer is used to signal register
        // address, it's output doesn't contain data on return.
        let mut buffer = [0u8; 4 + core::mem::size_of::<FifoPacket4>()];
        buffer[0] = crate::register_bank::bank0::INT_STATUS::ID | 0x80;
        self.ll.bus.transfer_in_place(&mut buffer).await?;
        // Buffer now contains [0, INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, DATA,
        // DATA, ...]
        let int_status = int_status::R([buffer[1], 1]);
        if int_status.fifo_full_int() != 0 {
            return Err(Error::FifoOverflow);
        }
        let p = bytemuck::from_bytes::<FifoPacket4>(&buffer[4..24]);
        let fifo_count = ((buffer[2] as u16) << 8) | (buffer[3] as u16);
        Ok(Self::sample_from_packet4(p).map(|sample| {
            (
                sample,
                fifo_count > core::mem::size_of::<FifoPacket4>() as u16,
            )
        }))
    }

    #[cfg(not(feature = "async"))]
    /// Reads and parses a single sample from the FIFO.
    ///
    /// This function performs an efficient single SPI transaction to read the
    /// interrupt status, FIFO count, and the next available FIFO packet. It
    /// then parses this packet into a [`Sample`].
    ///
    /// # Returns
    ///
    /// - `Ok(Some((sample, more_data)))`: On a successful read and parse.
    ///   - `sample`: The [`Sample`] containing sensor data.
    ///   - `more_data`: A boolean flag that is `true` if the FIFO contains more
    ///     packets to be read.
    /// - `Ok(None)`: If the FIFO was empty or the packet read was invalid
    ///   (e.g., an "empty" marker).
    /// - `Err(Error::Bus(_))`: If a communication error occurs on the SPI bus.
    pub fn read_sample(
        &mut self,
    ) -> Result<Option<(Sample, bool)>, Error<SPI::Error>> {
        // We read INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, and then the data in
        // one go. The leading byte in the buffer is used to signal register
        // address, it's output doesn't contain data on return.
        let mut buffer = [0u8; 4 + core::mem::size_of::<FifoPacket4>()];
        buffer[0] = crate::register_bank::bank0::INT_STATUS::ID | 0x80;
        self.ll.bus.transfer_in_place(&mut buffer)?;
        // Buffer now contains [0, INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, DATA,
        // DATA, ...]
        let int_status = int_status::R([buffer[1], 1]);
        if int_status.fifo_full_int() != 0 {
            return Err(Error::FifoOverflow);
        }
        let p = bytemuck::from_bytes::<FifoPacket4>(&buffer[4..24]);
        let fifo_count = ((buffer[2] as u16) << 8) | (buffer[3] as u16);
        Ok(Self::sample_from_packet4(p).map(|sample| {
            (
                sample,
                fifo_count > core::mem::size_of::<FifoPacket4>() as u16,
            )
        }))
    }

    fn sample_from_packet4(p: &FifoPacket4) -> Option<Sample> {
        (p.fifo_header().header_msg().value() == 0).then(|| {
            const FULL_1SIDE_RANGE: f32 = (1 << 19) as f32;
            let gyro = (p.fifo_header().has_gyro().value() != 0).then(|| {
                let gx = p.gyro_data_x();
                let gy = p.gyro_data_y();
                let gz = p.gyro_data_z();
                // Packet 4 => full scale reading.
                // The signed 20-bit quantity corresponds to ±2000 degrees per
                // second. 1 degree = π/180 radians.
                const FULL_SCALE_DPS: f32 = 2000.0;
                let scale = core::f32::consts::PI / 180.0 * FULL_SCALE_DPS
                    / FULL_1SIDE_RANGE;
                (gx as f32 * scale, gy as f32 * scale, gz as f32 * scale)
            });
            let accel = (p.fifo_header().has_accel().value() != 0).then(|| {
                let ax = p.accel_data_x();
                let ay = p.accel_data_y();
                let az = p.accel_data_z();
                // Packet 4 => full scale reading.
                // The signed 20-bit quantity corresponds to ±16g.
                const STD_GRAVITY: f32 = 9.80665;
                const FULL_SCALE_G: f32 = 16.0;
                let scale = STD_GRAVITY * FULL_SCALE_G / FULL_1SIDE_RANGE;
                (ax as f32 * scale, ay as f32 * scale, az as f32 * scale)
            });
            // Temperature is always provided.
            // Using formula as stated in data sheet.
            const TEMP_SENSITIVITY_LSB_PER_CELSIUS: f32 = 2.07;
            let temperature_celsius = p.temperature_raw() as f32
                / TEMP_SENSITIVITY_LSB_PER_CELSIUS
                + 25.0;

            Sample {
                accel,
                gyro,
                temperature_celsius,
                timestamp: p.timestamp(),
            }
        })
    }

    #[cfg(not(feature = "async"))]
    pub fn read_fifo(
        &mut self,
        buffer: &mut [u32],
    ) -> Result<usize, SPI::Error> {
        /// We read INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, and then the data in
        /// one go
        const INT_STATUS_ADDR: u8 = crate::register_bank::bank0::INT_STATUS::ID;

        let buffer = bytemuck::cast_slice_mut::<u32, u8>(buffer);

        buffer[0] = INT_STATUS_ADDR | 0x80; // Read bit set

        self.ll.bus.transfer_in_place(buffer)?;

        // Buffer now contains [0, INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, DATA,
        // DATA, ...] We need to check the FIFO_COUNT and then return the number
        // of samples read
        let fifo_count = ((buffer[2] as u16) << 8) | (buffer[3] as u16);

        Ok(fifo_count as usize / 20)
    }

    /// Direct low level access to the underlying peripheral
    pub fn ll(&mut self) -> &mut crate::ll::ICM42688<SPI> {
        &mut self.ll
    }

    pub fn release(self) -> SPI {
        self.ll.release()
    }
}
