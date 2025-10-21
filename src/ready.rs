#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice;

#[cfg(not(feature = "async"))]
use embedded_hal::spi::SpiDevice;

use crate::{
    fifo::{FifoPacket4, Sample},
    register_bank::Register,
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
    pub async fn read_sample(&mut self) -> Result<Sample, Error<SPI::Error>> {
        // Read 1 extra packet in case the user has been slow in reading out samples.
        let mut buffer = [0u8; 44];
        // We read INT_STATUS, FIFO_COUNT_H, FIFO_COUNT_L, and then the data in
        // one go.
        buffer[0] = crate::register_bank::bank0::INT_STATUS::ID | 0x80;
        self.ll.bus.transfer_in_place(&mut buffer).await?;
        let p = bytemuck::from_bytes::<FifoPacket4>(&buffer[4..24]);
        Self::sample_from_packet4(p).ok_or(Error::NoSampleRead)
    }

    #[cfg(not(feature = "async"))]
    pub fn read_sample(&mut self) -> Result<Sample, Error<SPI::Error>> {
        let mut buffer = [0u8; 44];
        buffer[0] = crate::register_bank::bank0::INT_STATUS::ID | 0x80;
        self.ll.bus.transfer_in_place(&mut buffer)?;
        let p = bytemuck::from_bytes::<FifoPacket4>(&buffer[4..24]);
        Self::sample_from_packet4(p).ok_or(Error::NoSampleRead)
    }

    fn sample_from_packet4(p: &FifoPacket4) -> Option<Sample> {
        (p.fifo_header().header_msg().value() == 0).then(|| {
            let gyro = (p.fifo_header().has_gyro().value() != 0).then(|| {
                let gx = p.gyro_data_x();
                let gy = p.gyro_data_y();
                let gz = p.gyro_data_z();
                // Packet 4 => full scale reading.
                // The signed 20-bit quantity corresponds to ±2000 degrees per
                // second. 1 degree = π/180 radians.
                let scale = core::f32::consts::PI / 180.0 * 2000.0f32
                    / (1 << 19) as f32;
                (gx as f32 * scale, gy as f32 * scale, gz as f32 * scale)
            });
            let accel = (p.fifo_header().has_accel().value() != 0).then(|| {
                let ax = p.accel_data_x();
                let ay = p.accel_data_y();
                let az = p.accel_data_z();
                // Packet 4 => full scale reading.
                // The signed 20-bit quantity corresponds to ±16g.
                let std_gravity = 9.80665;
                let scale = std_gravity * 16.0f32 / (1 << 19) as f32;
                (ax as f32 * scale, ay as f32 * scale, az as f32 * scale)
            });
            // Temperature is always provided.
            let temp_scale = 1f32 / 2.07;
            let temperature_celsius =
                p.temperature_raw() as f32 * temp_scale + 25.0;
            let timestamp = (p.fifo_header().has_timestamp_fsync().value()
                != 0)
                .then_some(p.timestamp());

            Sample {
                accel,
                gyro,
                temperature_celsius,
                timestamp,
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
