// Indicates that the `ICM42688` instance is not initialized yet

#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs;

#[cfg(not(feature = "async"))]
use embedded_hal::delay::DelayNs;

use crate::{Error, Ready, Uninitialized, ICM42688};

/// Determines how the interrupt signal is generated.
///
/// - `Pulsed`: The interrupt signal is a short pulse when the event occurs.
/// - `Latched`: The interrupt signal remains active until it is cleared by
///   software.
pub enum InterruptMode {
    /// Interrupt signal is a short pulse when the event occurs. This
    /// configuration can lead to race conditions if a configured interrupt
    /// handler isn't always installed.
    Pulsed,
    /// Interrupt signal remains active until cleared by software.
    Latched,
}
/// Sets the active level of the interrupt signal.
///
/// - `ActiveHigh`: Interrupt is signaled by a high logic level.
/// - `ActiveLow`: Interrupt is signaled by a low logic level.
pub enum InterruptPolarity {
    /// Interrupt is signaled by a high logic level.
    ActiveHigh,
    /// Interrupt is signaled by a low logic level.
    ActiveLow,
}

/// Sensor output data rate. The code uses the same rate for both gyro and
/// accelerometer.
#[derive(Copy, Clone)]
pub enum OutputDataRate {
    Hz32000,
    Hz16000,
    Hz8000,
    Hz4000,
    Hz2000,
    Hz1000,
    Hz500,
    Hz200,
    Hz100,
    Hz50,
    Hz25,
    Hz12_5,
}

struct AafConfig {
    delt: u8,
    delt_sqr: u16,
    bitshift: u8,
}

/// Configuration for the ICM42688 sensor during initialization.
///
/// This struct is used to set up various parameters of the sensor, such as
/// interrupt behavior and the output data rate. An instance with default values
/// can be created with `Config::default()`.
pub struct Config {
    /// The mode of the INT1 interrupt pin.
    pub int1_mode: InterruptMode,
    /// The polarity of the INT1 interrupt pin.
    pub int1_polarity: InterruptPolarity,
    /// The output data rate for both the accelerometer and gyroscope.
    pub rate: OutputDataRate,
    /// Configures the nature of timestamps in [`Sample`]s.
    ///
    /// See `Timestamp::OdrTimestamp` for more details.
    ///
    /// - `true`: Timestamps are absolute. Each consecutive sample's timestamp
    ///   will monotonically increase, wrapping on the `u16` boundary.
    /// - `false` (default): Timestamps represent the delta of time since the
    ///   last Output Data Rate (ODR) event. This corresponds to the
    ///   `TMST_DELTA_EN` register setting.
    pub timestamps_are_absolute: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            int1_mode: InterruptMode::Latched,
            int1_polarity: InterruptPolarity::ActiveHigh,
            rate: OutputDataRate::Hz200,
            timestamps_are_absolute: false,
        }
    }
}

impl<SPI> ICM42688<SPI, Uninitialized> {
    pub fn new(spi: SPI) -> Self {
        ICM42688 {
            ll: crate::ll::ICM42688::new(spi),
            _state: Uninitialized,
        }
    }

    fn int_config_bits(config: &Config) -> (u8, u8) {
        let mode = match config.int1_mode {
            InterruptMode::Pulsed => 0,
            InterruptMode::Latched => 1,
        };
        let polarity = match config.int1_polarity {
            InterruptPolarity::ActiveHigh => 1,
            InterruptPolarity::ActiveLow => 0,
        };
        (mode, polarity)
    }

    fn gyro_accel_odr(odr: OutputDataRate) -> u8 {
        match odr {
            OutputDataRate::Hz32000 => 0b0001,
            OutputDataRate::Hz16000 => 0b0010,
            OutputDataRate::Hz8000 => 0b0011,
            OutputDataRate::Hz4000 => 0b0100,
            OutputDataRate::Hz2000 => 0b0101,
            OutputDataRate::Hz1000 => 0b0110,
            OutputDataRate::Hz500 => 0b1111,
            OutputDataRate::Hz200 => 0b0111,
            OutputDataRate::Hz100 => 0b1000,
            OutputDataRate::Hz50 => 0b1001,
            OutputDataRate::Hz25 => 0b1010,
            OutputDataRate::Hz12_5 => 0b1011,
        }
    }

    fn get_aaf_config(odr: OutputDataRate) -> AafConfig {
        let create_config = |delt: u8, delt_sqr: u16, bitshift: u8| -> AafConfig {
            AafConfig {
                delt,
                delt_sqr,
                bitshift,
            }
        };
        match odr {
            // Nyquist: 4000 Hz or higher. Highest available BW is 3979 Hz.
            OutputDataRate::Hz32000 | OutputDataRate::Hz16000 | OutputDataRate::Hz8000 => {
                create_config(63, 3968, 3)
            }
            // Nyquist: 2000 Hz. Choose BW: 1962 Hz.
            OutputDataRate::Hz4000 => create_config(37, 1376, 4),
            // Nyquist: 1000 Hz. Choose BW: 997 Hz.
            OutputDataRate::Hz2000 => create_config(21, 440, 6),
            // Nyquist: 500 Hz. Choose BW: 488 Hz.
            OutputDataRate::Hz1000 => create_config(11, 122, 8),
            // Nyquist: 250 Hz. Choose BW: 213 Hz.
            OutputDataRate::Hz500 => create_config(5, 25, 10),
            // Nyquist: 100 Hz. Choose BW: 84 Hz.
            OutputDataRate::Hz200 => create_config(2, 4, 13),
            // Nyquist: 50 Hz or lower. Choose BW: 42 Hz. For lower rates this
            // is higher than Nyquist, but this is the limit of the hardware.
            OutputDataRate::Hz100
            | OutputDataRate::Hz50
            | OutputDataRate::Hz25
            | OutputDataRate::Hz12_5 => create_config(1, 1, 15),
        }
    }

    #[cfg(feature = "async")]
    pub async fn initialize(
        mut self,
        mut delay: impl DelayNs,
        config: Config,
    ) -> Result<ICM42688<SPI, Ready>, Error<SPI::Error>>
    where
        SPI: embedded_hal_async::spi::SpiDevice,
    {
        use crate::Ready;

        let mut bank0 = self.ll.bank::<0>();

        // Soft reset the device
        //
        // This is required to ensure the device is in a known state
        bank0
            .device_config()
            .async_modify(|_, w| w.soft_reset_config(1))
            .await?;

        // Wait 1ms for the device to reset
        delay.delay_ms(1).await;

        // Read the WHO_AM_I register to verify the device is present
        let who_am_i = bank0.who_am_i().async_read().await?.value();
        if who_am_i != 0x47 {
            return Err(Error::WhoAmIMismatch(who_am_i));
        }

        let (int1_mode, int1_polarity) = Self::int_config_bits(&config);
        bank0
            .int_config()
            .async_modify(|_, w| {
                w.int1_drive_circuit(1)
                    .int1_mode(int1_mode)
                    .int1_polarity(int1_polarity)
            })
            .await?;
        bank0
            .fifo_config()
            .async_modify(|_, w| w.fifo_mode(11))
            .await?;
        bank0
            .intf_config0()
            .async_modify(|_, w| w.fifo_count_endian(1).sensor_data_endian(1).ui_sifs_cfg(11))
            .await?;
        // Disable AFSR (undocumented adaptive scale change)
        bank0
            .intf_config1()
            .async_modify(|_, w| w.afsr(0b01))
            .await?;

        let odr = Self::gyro_accel_odr(config.rate);
        bank0
            .gyro_config0()
            .async_modify(|_, w| w.gyro_fs_sel(0b000).gyro_odr(odr))
            .await?;

        bank0
            .accel_config0()
            .async_modify(|_, w| w.accel_fs_sel(0b000).accel_odr(odr))
            .await?;

        bank0
            .gyro_config1()
            .async_modify(|_, w| w.gyro_ui_filt_ord(0b0))
            .await?;

        bank0
            .gyro_accel_config0()
            .async_modify(|_, w| w.accel_ui_filt_bw(0b00).gyro_ui_filt_bw(0b00))
            .await?;

        bank0
            .accel_config1()
            .async_modify(|_, w| w.accel_ui_filt_ord(0b0))
            .await?;
        bank0
            .tmst_config()
            .async_modify(|_, w| {
                w.tmst_en(1)
                    .tmst_delta_en((!config.timestamps_are_absolute) as u8)
                    .tmst_to_regs_en(1)
                    .tmst_res(1)
                    .tmst_fsync_en(0)
            })
            .await?;
        bank0
            .fifo_config1()
            .async_modify(|_, w| {
                w.fifo_wm_gt_th(1)
                    .fifo_hires_en(1)
                    .fifo_temp_en(1)
                    .fifo_gyro_en(1)
                    .fifo_accel_en(1)
                    .fifo_tmst_fsync_en(0)
            })
            .await?;
        // Watermark interrupt at 1 "Packet 4" (20 bytes).
        bank0
            .fifo_config2()
            .async_modify(|_, w| w.fifo_wm_7_0(20))
            .await?;
        bank0
            .fifo_config3()
            .async_modify(|_, w| w.fifo_wm_11_8(0))
            .await?;
        // Clear interrupt on first FIFO byte read.
        bank0
            .int_config0()
            .async_modify(|_, w| w.fifo_ths_int_clear(0b10))
            .await?;
        // Data sheet: For register INT_CONFIG1 (bank 0 register 0x64) bit 4
        // INT_ASYNC_RESET, user should change setting to 0 from default setting
        // of 1, for proper INT1 and INT2 pin operation.
        bank0
            .int_config1()
            .async_modify(|_, w| w.int_async_reset(0))
            .await?;
        bank0
            .int_source0()
            .async_modify(|_, w| w.fifo_ths_int1_en(1))
            .await?;

        let aaf = Self::get_aaf_config(config.rate);

        // Set bank 1
        bank0.reg_bank_sel().async_write(|r| r.bank_sel(1)).await?;
        self.ll.set_bank(1);
        let mut bank1 = self.ll.bank::<1>();
        bank1
            .gyro_config_static2()
            .async_modify(|_, w| w.gyro_nf_dis(0).gyro_aaf_dis(0))
            .await?;

        bank1
            .gyro_config_static3()
            .async_modify(|_, w| w.gyro_aaf_delt(aaf.delt))
            .await?;

        bank1
            .gyro_config_static4()
            .async_modify(|_, w| w.gyro_aaf_deltsqr_7_0(aaf.delt_sqr as u8))
            .await?;

        bank1
            .gyro_config_static5()
            .async_modify(|_, w| {
                w.gyro_aaf_deltsqr_11_8((aaf.delt_sqr >> 8) as u8)
                    .gyro_aaf_bitshift(aaf.bitshift)
            })
            .await?;

        bank1
            .intf_config5()
            .async_modify(|_, w| w.pin9_function(0b00)) // Default, INT2
            .await?;

        // Set bank 2
        bank1.reg_bank_sel().async_write(|r| r.bank_sel(2)).await?;
        self.ll.set_bank(2);
        let mut bank2 = self.ll.bank::<2>();
        bank2
            .accel_config_static2()
            .async_modify(|_, w| w.accel_aaf_dis(0).accel_aaf_delt(aaf.delt))
            .await?;

        bank2
            .accel_config_static3()
            .async_modify(|_, w| w.accel_aaf_deltsqr_7_0(aaf.delt_sqr as u8))
            .await?;

        bank2
            .accel_config_static4()
            .async_modify(|_, w| {
                w.accel_aaf_deltsqr_11_8((aaf.delt_sqr >> 8) as u8)
                    .accel_aaf_bitshift(aaf.bitshift)
            })
            .await?;

        // Set bank 0
        bank2.reg_bank_sel().async_write(|r| r.bank_sel(0)).await?;
        self.ll.set_bank(0);

        // Only enable gyro and accel when all registers are written
        // Refer to Section 12.9 of the datasheet
        self.ll
            .bank::<0>()
            .pwr_mgmt0()
            .async_modify(|_, w| w.gyro_mode(0b11).accel_mode(0b11).temp_dis(0))
            .await?;

        // Delay for 200us per the datasheet after writing to PWR_MGMT0
        delay.delay_us(200).await;

        Ok(ICM42688 {
            ll: self.ll,
            _state: Ready,
        })
    }

    #[cfg(not(feature = "async"))]
    pub fn initialize(
        mut self,
        mut delay: impl DelayNs,
        config: Config,
    ) -> Result<ICM42688<SPI, Ready>, Error<SPI::Error>>
    where
        SPI: embedded_hal::spi::SpiDevice,
    {
        use crate::Ready;

        let mut bank0 = self.ll.bank::<0>();

        // Soft reset the device
        //
        // This is required to ensure the device is in a known state
        bank0
            .device_config()
            .modify(|_, w| w.soft_reset_config(1))?;

        // Wait 1ms for the device to reset
        delay.delay_ms(1);

        // Read the WHO_AM_I register to verify the device is present
        let who_am_i = bank0.who_am_i().read()?.value();
        if who_am_i != 0x47 {
            return Err(Error::WhoAmIMismatch(who_am_i));
        }

        let (int1_mode, int1_polarity) = Self::int_config_bits(&config);
        bank0.int_config().modify(|_, w| {
            w.int1_drive_circuit(1)
                .int1_polarity(int1_polarity)
                .int1_mode(int1_mode)
        })?;
        bank0.fifo_config().modify(|_, w| w.fifo_mode(11))?;
        bank0
            .intf_config0()
            .modify(|_, w| w.fifo_count_endian(1).sensor_data_endian(1).ui_sifs_cfg(11))?;
        // Disable AFSR (undocumented adaptive scale change)
        bank0.intf_config1().modify(|_, w| w.afsr(0b01))?;

        let odr = Self::gyro_accel_odr(config.rate);
        bank0
            .gyro_config0()
            .modify(|_, w| w.gyro_fs_sel(0b000).gyro_odr(odr))?;

        bank0
            .accel_config0()
            .modify(|_, w| w.accel_fs_sel(0b000).accel_odr(odr))?;

        bank0
            .gyro_config1()
            .modify(|_, w| w.gyro_ui_filt_ord(0b0))?;

        bank0
            .gyro_accel_config0()
            .modify(|_, w| w.accel_ui_filt_bw(0b00).gyro_ui_filt_bw(0b00))?;

        bank0
            .accel_config1()
            .modify(|_, w| w.accel_ui_filt_ord(0b0))?;
        bank0.tmst_config().modify(|_, w| {
            w.tmst_en(1)
                .tmst_delta_en((!config.timestamps_are_absolute) as u8)
                .tmst_to_regs_en(1)
                .tmst_res(1)
                .tmst_fsync_en(0)
        })?;
        bank0.fifo_config1().modify(|_, w| {
            w.fifo_wm_gt_th(1)
                .fifo_hires_en(1)
                .fifo_temp_en(1)
                .fifo_gyro_en(1)
                .fifo_accel_en(1)
                .fifo_tmst_fsync_en(0)
        })?;
        // Watermark interrupt at 1 "Packet 4" (20 bytes).
        bank0.fifo_config2().modify(|_, w| w.fifo_wm_7_0(20))?;
        bank0.fifo_config3().modify(|_, w| w.fifo_wm_11_8(0))?;
        // Clear interrupt on first FIFO byte read.
        bank0
            .int_config0()
            .modify(|_, w| w.fifo_ths_int_clear(0b10))?;
        // Data sheet: For register INT_CONFIG1 (bank 0 register 0x64) bit 4
        // INT_ASYNC_RESET, user should change setting to 0 from default setting
        // of 1, for proper INT1 and INT2 pin operation.
        bank0.int_config1().modify(|_, w| w.int_async_reset(0))?;
        bank0.int_source0().modify(|_, w| w.fifo_ths_int1_en(1))?;

        let aaf = Self::get_aaf_config(config.rate);
        // Set bank 1
        bank0.reg_bank_sel().write(|r| r.bank_sel(1))?;
        self.ll.set_bank(1);
        let mut bank1 = self.ll.bank::<1>();
        bank1
            .gyro_config_static2()
            .modify(|_, w| w.gyro_nf_dis(0).gyro_aaf_dis(0))?;

        bank1
            .gyro_config_static3()
            .modify(|_, w| w.gyro_aaf_delt(aaf.delt))?;

        bank1
            .gyro_config_static4()
            .modify(|_, w| w.gyro_aaf_deltsqr_7_0(aaf.delt_sqr as u8))?;

        bank1.gyro_config_static5().modify(|_, w| {
            w.gyro_aaf_deltsqr_11_8((aaf.delt_sqr >> 8) as u8)
                .gyro_aaf_bitshift(aaf.bitshift)
        })?;

        bank1.intf_config5().modify(|_, w| w.pin9_function(0b00))?; // Default, INT2

        bank1.reg_bank_sel().write(|r| r.bank_sel(2))?;

        // Set bank 2
        self.ll.set_bank(2);
        let mut bank2 = self.ll.bank::<2>();
        bank2
            .accel_config_static2()
            .modify(|_, w| w.accel_aaf_dis(0).accel_aaf_delt(aaf.delt))?;

        bank2
            .accel_config_static3()
            .modify(|_, w| w.accel_aaf_deltsqr_7_0(aaf.delt_sqr as u8))?;

        bank2.accel_config_static4().modify(|_, w| {
            w.accel_aaf_deltsqr_11_8((aaf.delt_sqr >> 8) as u8)
                .accel_aaf_bitshift(aaf.bitshift)
        })?;

        // Set bank 0
        bank2.reg_bank_sel().write(|r| r.bank_sel(0))?;
        self.ll.set_bank(0);
        self.ll
            .bank::<0>()
            .pwr_mgmt0()
            .modify(|_, w| w.gyro_mode(0b11).accel_mode(0b11).temp_dis(0))?;

        // Delay for 200us per the datasheet after writing to PWR_MGMT0
        delay.delay_us(200);

        Ok(ICM42688 {
            ll: self.ll,
            _state: Ready,
        })
    }

    /// Direct low level access to the underlying peripheral
    pub fn ll(&mut self) -> &mut crate::ll::ICM42688<SPI> {
        &mut self.ll
    }
}
