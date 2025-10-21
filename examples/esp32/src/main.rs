//! This example runs on the ESP32-S3-DevKitC-1 board, wired to a ICM-42688-P
//! via SPI It demonstrates simple IMU data read functionality.
#![no_std]
#![no_main]

use bytemuck::{cast_slice, from_bytes};
use defmt::Debug2Format;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use esp_backtrace as _;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::spi::master::Spi;
use esp_hal::spi::{master::Config, Mode};
use esp_hal::time::Rate;
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
use esp_println as _;
use icm426xx::fifo::FifoPacket4;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    // Set up ESP32
    let peripherals = esp_hal::init(
        esp_hal::Config::default().with_cpu_clock(CpuClock::max()),
    );
    let timer_group = TimerGroup::new(peripherals.TIMG0);

    esp_hal_embassy::init(timer_group.timer1);

    // Initialize SPI
    let cs =
        Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO6)
    .with_miso(peripherals.GPIO7)
    .into_async();

    let spi_bus = Mutex::<CriticalSectionRawMutex, _>::new(spi);
    let spi_device =
        embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(
            &spi_bus, cs,
        );

    let icm = icm426xx::ICM42688::new(spi_device);
    let mut icm = icm
        .initialize(Delay, icm426xx::uninitialized::Config::default())
        .await
        .unwrap();
    let mut bank = icm.ll().bank::<{ icm426xx::register_bank::BANK0 }>();

    let who_am_i = bank.who_am_i().async_read().await.unwrap();
    defmt::info!("ICM WHO_AM_I = 0x{:02X}", &Debug2Format(&who_am_i));

    loop {
        // 1) Prepare a u32 buffer
        let mut raw_words = [0u32; 128];

        // 2) Read as many 32-bit words as we can
        let num_words = icm.read_fifo(&mut raw_words).await.unwrap();

        // 3) Reinterpret those words as bytes
        let raw_bytes: &[u8] = cast_slice(&raw_words[..num_words]);

        // 4) How many full 20-byte packets did we get?
        let packet_size = core::mem::size_of::<FifoPacket4>();

        // 5) Iterate each 20-byte chunk
        for (i, chunk) in raw_bytes.chunks_exact(packet_size).enumerate() {
            // SAFELY reify it as a FifoPacket4
            let pkt: &FifoPacket4 = from_bytes(chunk);

            // Pull out the signed 20-bit values
            let gx_raw = pkt.gyro_data_x();
            let gy_raw = pkt.gyro_data_y();
            let gz_raw = pkt.gyro_data_z();

            // Scale to human units:
            let gyro_lsb = 250.0f32 / (1 << 19) as f32;

            // Uncomment the following block to acquire and print raw
            // acceleration values
            /*
            let ax_raw = pkt.accel_data_x();
            let ay_raw = pkt.accel_data_y();
            let az_raw = pkt.accel_data_z();

            let accel_lsb = 2.0f32 / (1 << 19) as f32;
            let ax_g = ax_raw as f32 * accel_lsb;
            let ay_g = ay_raw as f32 * accel_lsb;
            let az_g = az_raw as f32 * accel_lsb;
            defmt::info!("Pkt[{}] Accel (g): x={}, y={}, z={}", i, ax_g, ay_g, az_g);
             */

            let gx_dps = gx_raw as f32 * gyro_lsb;
            let gy_dps = gy_raw as f32 * gyro_lsb;
            let gz_dps = gz_raw as f32 * gyro_lsb;

            defmt::info!(
                "Pkt[{=usize:02}] Gyro  (°/s): x={}, y={}, z={}",
                i,
                gx_dps as i32,
                gy_dps as i32,
                gz_dps as i32
            );
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::info!("Panic: {}", info);

    loop {}
}
