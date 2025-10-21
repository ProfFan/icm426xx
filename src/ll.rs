use crate::register_bank::{RegisterBank, Registers, BANK0};

pub struct ICM42688<BUS> {
    pub(crate) bus: BUS,
    current_bank: RegisterBank,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BankSelectionError;

impl core::fmt::Display for BankSelectionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Bank selection error")
    }
}

// Only available in nightly
// impl core::error::Error for BankSelectionError {}

pub trait BankSelectable {
    fn set_bank(
        &mut self,
        bank: RegisterBank,
    ) -> Result<(), BankSelectionError>;
}

impl<BUS> ICM42688<BUS> {
    pub fn new(bus: BUS) -> Self {
        ICM42688 {
            bus,
            current_bank: BANK0,
        }
    }

    pub fn set_bank(&mut self, bank: RegisterBank) {
        self.current_bank = bank;
    }

    pub fn get_bank(&self) -> RegisterBank {
        self.current_bank
    }

    pub fn bank<const BANK: RegisterBank>(
        &mut self,
    ) -> Registers<'_, BUS, BANK> {
        if self.current_bank != BANK {
            panic!("Bank mismatch")
        }
        Registers::new(&mut self.bus)
    }

    /// Get a reference to the bus
    pub fn bus(&mut self) -> &mut BUS {
        &mut self.bus
    }

    /// Release the bus from the ICM42688 instance
    pub fn release(self) -> BUS {
        self.bus
    }
}

#[cfg(test)]
mod test {
    extern crate alloc;
    use super::*;
    use crate::register_bank::{BANK0, BANK1};
    use alloc::vec;

    use embedded_hal_mock::eh1::digital::Mock as PinMock;
    use embedded_hal_mock::eh1::digital::{
        State as PinState, Transaction as PinTransaction,
    };
    use embedded_hal_mock::eh1::spi::{
        Mock as SpiMock, Transaction as SpiTransaction,
    };

    #[test]
    fn test_bank_noop() {
        let expectations: &[SpiTransaction<u8>] = &[];

        let spi = SpiMock::new(expectations);
        let mut pin = PinMock::new(&[PinTransaction::set(PinState::High)]);
        let spidev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(
            spi,
            pin.clone(),
        )
        .unwrap();
        let mut icm = ICM42688::new(spidev);
        let _ = icm.bank::<BANK0>();

        let mut spidev = icm.release();
        spidev.bus_mut().done();
        pin.done();
    }

    #[test]
    #[should_panic]
    fn test_bank_noop_wrong() {
        let expectations: &[SpiTransaction<u8>] = &[];

        let spi = SpiMock::new(expectations);
        let mut pin = PinMock::new(&[PinTransaction::set(PinState::High)]);
        let spidev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(
            spi,
            pin.clone(),
        )
        .unwrap();
        let mut icm = ICM42688::new(spidev);
        let _ = icm.bank::<BANK1>();

        let mut spidev = icm.release();
        spidev.bus_mut().done();
        pin.done();
    }

    #[cfg(not(feature = "async"))]
    #[test]
    fn test_who_am_i() {
        let expectations: &[SpiTransaction<u8>] = &[
            SpiTransaction::transfer_in_place(
                vec![0x75 | 0x80, 0x00],
                vec![0x12, 0x47],
            ),
            SpiTransaction::flush(),
        ];

        let spi = SpiMock::new(expectations);
        let mut pin = PinMock::new(&[
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ]);
        let spidev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(
            spi,
            pin.clone(),
        )
        .unwrap();
        let mut icm = ICM42688::new(spidev);
        let mut bank = icm.bank::<BANK0>();
        let whoami = bank.who_am_i().read().unwrap().value();
        assert_eq!(whoami, 0x47);

        let mut spidev = icm.release();
        spidev.bus_mut().done();
        pin.done();
    }

    #[cfg(feature = "async")]
    #[async_std::test]
    async fn test_who_am_i_async() {
        let expectations: &[SpiTransaction<u8>] = &[
            SpiTransaction::transfer_in_place(
                vec![0x75 | 0x80, 0x00],
                vec![0x12, 0x47],
            ),
            SpiTransaction::flush(),
        ];

        let spi = SpiMock::new(expectations);
        let mut pin = PinMock::new(&[
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ]);
        let spidev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(
            spi,
            pin.clone(),
        )
        .unwrap();
        let mut icm = ICM42688::new(spidev);
        let mut bank = icm.bank::<BANK0>();
        let whoami = bank.who_am_i().async_read().await.unwrap().value();
        assert_eq!(whoami, 0x47);

        let mut spidev = icm.release();
        spidev.bus_mut().done();
        pin.done();
    }
}
