// Indicates that the `ICM42688` instance is not initialized yet

#[cfg(feature = "async")]
use embedded_hal_async::delay::DelayNs;

#[cfg(not(feature = "async"))]
use embedded_hal::delay::DelayNs;

use crate::{Error, Ready, Uninitialized, ICM42688};

impl<SPI> ICM42688<SPI, Uninitialized> {
    pub fn new(spi: SPI) -> Self {
        ICM42688 {
            ll: crate::ll::ICM42688::new(spi),
            _state: Uninitialized,
        }
    }

    #[cfg(feature = "async")]
    pub async fn initialize(
        mut self,
        mut delay: impl DelayNs,
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

        bank0
            .int_config()
            .async_modify(|_, w| w.int1_mode(1).int1_drive_circuit(1))
            .await?;
        bank0
            .fifo_config()
            .async_modify(|_, w| w.fifo_mode(11))
            .await?;
        bank0
            .intf_config0()
            .async_modify(|_, w| {
                w.fifo_count_endian(1).sensor_data_endian(1).ui_sifs_cfg(11)
            })
            .await?;
        // Disable AFSR (undocumented adaptive scale change)
        bank0
            .intf_config1()
            .async_modify(|_, w| w.afsr(0b01))
            .await?;

        bank0
            .gyro_config0()
            .async_modify(|_, w| w.gyro_fs_sel(0b000).gyro_odr(0b0110))
            .await?;

        bank0
            .accel_config0()
            .async_modify(|_, w| w.accel_fs_sel(0b000).accel_odr(0b0110))
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
                    .tmst_delta_en(1)
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
        bank0
            .fifo_config2()
            .async_modify(|_, w| w.fifo_wm_7_0(0b00000000))
            .await?;
        bank0
            .fifo_config3()
            .async_modify(|_, w| w.fifo_wm_11_8(0b0000))
            .await?;
        bank0
            .int_config0()
            .async_modify(|_, w| w.fifo_ths_int_clear(0b10))
            .await?;
        bank0
            .int_config1()
            .async_modify(|_, w| w.int_async_reset(0))
            .await?;
        bank0
            .int_source0()
            .async_modify(|_, w| w.fifo_ths_int1_en(1))
            .await?;

        bank0.reg_bank_sel().async_write(|r| r.bank_sel(1)).await?;

        self.ll.set_bank(1);

        // Set bank 1
        let mut bank1 = self.ll.bank::<1>();

        bank1
            .gyro_config_static2()
            .async_modify(|_, w| w.gyro_nf_dis(0).gyro_aaf_dis(0))
            .await?;

        bank1
            .gyro_config_static3()
            .async_modify(|_, w| w.gyro_aaf_delt(13))
            .await?;

        bank1
            .gyro_config_static4()
            .async_modify(|_, w| w.gyro_aaf_deltsqr_7_0(170))
            .await?;

        bank1
            .gyro_config_static5()
            .async_modify(|_, w| {
                w.gyro_aaf_deltsqr_11_8(0).gyro_aaf_bitshift(8)
            })
            .await?;

        bank1
            .intf_config5()
            .async_modify(|_, w| w.pin9_function(0b00)) // Default, INT2
            .await?;

        bank1.reg_bank_sel().async_write(|r| r.bank_sel(2)).await?;

        self.ll.set_bank(2);

        // Set bank 2
        let mut bank2 = self.ll.bank::<2>();

        bank2
            .accel_config_static2()
            .async_modify(|_, w| w.accel_aaf_dis(0).accel_aaf_delt(13))
            .await?;

        bank2
            .accel_config_static3()
            .async_modify(|_, w| w.accel_aaf_deltsqr_7_0(170))
            .await?;

        bank2
            .accel_config_static4()
            .async_modify(|_, w| {
                w.accel_aaf_deltsqr_11_8(0).accel_aaf_bitshift(8)
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
            .async_modify(|_, w| w.gyro_mode(0b11).accel_mode(0b11))
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

        bank0
            .int_config()
            .modify(|_, w| w.int1_mode(1).int1_drive_circuit(1))?;
        bank0.fifo_config().modify(|_, w| w.fifo_mode(11))?;
        bank0.intf_config0().modify(|_, w| {
            w.fifo_count_endian(1).sensor_data_endian(1).ui_sifs_cfg(11)
        })?;
        // Disable AFSR (undocumented adaptive scale change)
        bank0.intf_config1().modify(|_, w| w.afsr(0b01))?;

        bank0
            .gyro_config0()
            .modify(|_, w| w.gyro_fs_sel(0b000).gyro_odr(0b0110))?;

        bank0
            .accel_config0()
            .modify(|_, w| w.accel_fs_sel(0b000).accel_odr(0b0110))?;

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
                .tmst_delta_en(1)
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
        bank0
            .fifo_config2()
            .modify(|_, w| w.fifo_wm_7_0(0b00000000))?;
        bank0.fifo_config3().modify(|_, w| w.fifo_wm_11_8(0b0000))?;
        bank0
            .int_config0()
            .modify(|_, w| w.fifo_ths_int_clear(0b10))?;
        bank0.int_config1().modify(|_, w| w.int_async_reset(0))?;
        bank0.int_source0().modify(|_, w| w.fifo_ths_int1_en(1))?;

        bank0.reg_bank_sel().write(|r| r.bank_sel(1))?;

        self.ll.set_bank(1);

        // Set bank 1
        let mut bank1 = self.ll.bank::<1>();

        bank1
            .gyro_config_static2()
            .modify(|_, w| w.gyro_nf_dis(0).gyro_aaf_dis(0))?;

        bank1
            .gyro_config_static3()
            .modify(|_, w| w.gyro_aaf_delt(13))?;

        bank1
            .gyro_config_static4()
            .modify(|_, w| w.gyro_aaf_deltsqr_7_0(170))?;

        bank1
            .gyro_config_static5()
            .modify(|_, w| w.gyro_aaf_deltsqr_11_8(0).gyro_aaf_bitshift(8))?;

        bank1
      .intf_config5()
      .modify(|_, w| w.pin9_function(0b00)) // Default, INT2
      ?;

        bank1.reg_bank_sel().write(|r| r.bank_sel(2))?;

        self.ll.set_bank(2);

        // Set bank 2
        let mut bank2 = self.ll.bank::<2>();

        bank2
            .accel_config_static2()
            .modify(|_, w| w.accel_aaf_dis(0).accel_aaf_delt(13))?;

        bank2
            .accel_config_static3()
            .modify(|_, w| w.accel_aaf_deltsqr_7_0(170))?;

        bank2
            .accel_config_static4()
            .modify(|_, w| w.accel_aaf_deltsqr_11_8(0).accel_aaf_bitshift(8))?;

        // Set bank 0
        bank2.reg_bank_sel().write(|r| r.bank_sel(0))?;

        self.ll.set_bank(0);

        self.ll
            .bank::<0>()
            .pwr_mgmt0()
            .modify(|_, w| w.gyro_mode(0b11).accel_mode(0b11))?;

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
