use bilge::prelude::*;
use bytemuck::{AnyBitPattern, NoUninit};

#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq)]
pub struct FifoHeader {
    odr_changed_gyro: u1, /* 1: The ODR for gyro is different for this gyro
                           * data packet compared to the previous gyro
                           * packet */
    odr_changed_accel: u1, /* 1: The ODR for accel is different for this
                            * accel data packet
                            * compared to the previous accel
                            * packet */
    has_timestamp_fsync: u2, // 10: Packet contains ODR Timestamp
    has_20bit: u1,           /* 1: Packet has a new and valid sample of
                              * extended 20-bit data for gyro and/or accel */
    has_gyro: u1, /* 1: Packet is sized so that gyro data have location in
                   * the packet, FIFO_GYRO_EN must be 1 */
    has_accel: u1, /* 1: Packet is sized so that accel data have location in
                    * the packet, FIFO_ACCEL_EN must be 1 */
    header_msg: u1, // 1: FIFO is empty
}

#[cfg(feature = "defmt")]
impl defmt::Format for FifoHeader {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "FifoHeader {{ header_msg: {}, has_accel: {}, has_gyro: {}, has_20bit: {}, has_timestamp_fsync: {}, odr_changed_accel: {}, odr_changed_gyro: {} }}",
            self.header_msg().value(),
            self.has_accel().value(),
            self.has_gyro().value(),
            self.has_20bit().value(),
            self.has_timestamp_fsync().value(),
            self.odr_changed_accel().value(),
            self.odr_changed_gyro().value(),
        );
    }
}

#[derive(Debug, Clone, Copy, PartialEq, NoUninit, AnyBitPattern, Default)]
#[repr(C)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoPacket4 {
    pub fifo_header: u8,
    pub accel_data_x1: u8,      // Accel X [19:12]
    pub accel_data_x0: u8,      // Accel X [11:4]
    pub accel_data_y1: u8,      // Accel Y [19:12]
    pub accel_data_y0: u8,      // Accel Y [11:4]
    pub accel_data_z1: u8,      // Accel Z [19:12]
    pub accel_data_z0: u8,      // Accel Z [11:4]
    pub gyro_data_x1: u8,       // Gyro X [19:12]
    pub gyro_data_x0: u8,       // Gyro X [11:4]
    pub gyro_data_y1: u8,       // Gyro Y [19:12]
    pub gyro_data_y0: u8,       // Gyro Y [11:4]
    pub gyro_data_z1: u8,       // Gyro Z [19:12]
    pub gyro_data_z0: u8,       // Gyro Z [11:4]
    pub temp_data1: u8,         // Temperature[15:8]
    pub temp_data0: u8,         // Temperature[7:0]
    pub timestamp_h: u8,        // TimeStamp[15:8]
    pub timestamp_l: u8,        // TimeStamp[7:0]
    pub ext_accel_x_gyro_x: u8, // Accel X [3:0] Gyro X [3:0]
    pub ext_accel_y_gyro_y: u8, // Accel Y [3:0] Gyro Y [3:0]
    pub ext_accel_z_gyro_z: u8, // Accel Z [3:0] Gyro Z [3:0]
}

impl FifoPacket4 {
    pub fn fifo_header(&self) -> FifoHeader {
        FifoHeader::from(self.fifo_header)
    }

    #[inline]
    fn convert_parts_to_20bit(
        &self,
        high_8: u8,
        low_8: u8,
        ext_low_4: u8,
    ) -> i32 {
        let high_12 = (high_8 as u32) << 12;
        let low_12 = (low_8 as u32) << 4;
        let ext_4 = (ext_low_4 & 0xF) as u32;
        let value = high_12 | low_12 | ext_4;

        // Sign extend the 20-bit value
        // If the 20th bit is set, then the value is negative
        let sign_extended = if value & 0x80000 != 0 {
            value | 0xFFF00000
        } else {
            value
        };

        sign_extended as i32
    }

    pub fn accel_data_x(&self) -> i32 {
        let ext_accel_x = (self.ext_accel_x_gyro_x & 0xF0) >> 4;
        self.convert_parts_to_20bit(
            self.accel_data_x1,
            self.accel_data_x0,
            ext_accel_x,
        )
    }

    pub fn accel_data_y(&self) -> i32 {
        let ext_accel_y = (self.ext_accel_y_gyro_y & 0xF0) >> 4;
        self.convert_parts_to_20bit(
            self.accel_data_y1,
            self.accel_data_y0,
            ext_accel_y,
        )
    }

    pub fn accel_data_z(&self) -> i32 {
        let ext_accel_z = (self.ext_accel_z_gyro_z & 0xF0) >> 4;
        self.convert_parts_to_20bit(
            self.accel_data_z1,
            self.accel_data_z0,
            ext_accel_z,
        )
    }

    pub fn gyro_data_x(&self) -> i32 {
        let ext_gyro_x = self.ext_accel_x_gyro_x & 0x0F;
        self.convert_parts_to_20bit(
            self.gyro_data_x1,
            self.gyro_data_x0,
            ext_gyro_x,
        )
    }

    pub fn gyro_data_y(&self) -> i32 {
        let ext_gyro_y = self.ext_accel_y_gyro_y & 0x0F;
        self.convert_parts_to_20bit(
            self.gyro_data_y1,
            self.gyro_data_y0,
            ext_gyro_y,
        )
    }

    pub fn gyro_data_z(&self) -> i32 {
        let ext_gyro_z = self.ext_accel_z_gyro_z & 0x0F;
        self.convert_parts_to_20bit(
            self.gyro_data_z1,
            self.gyro_data_z0,
            ext_gyro_z,
        )
    }

    pub fn temperature_raw(&self) -> u16 {
        ((self.temp_data1 as u16) << 8) | self.temp_data0 as u16
    }

    pub fn timestamp(&self) -> u16 {
        ((self.timestamp_h as u16) << 8) | self.timestamp_l as u16
    }
}

// Assert that the size of the struct is 20 bytes
const _SIZE_CHECK: usize =
    (core::mem::size_of::<FifoPacket4>() == 20) as usize - 1;
