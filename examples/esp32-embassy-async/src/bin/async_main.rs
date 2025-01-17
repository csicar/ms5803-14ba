#![no_std]
#![no_main]

use alloc::fmt::format;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::prelude::*;
use {defmt_rtt as _, esp_backtrace as _};

use ms5803_14ba::{PressureSensorDriver, DEFAULT_SENSOR_ADDRESS};

extern crate alloc;

#[main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER)
        .split::<esp_hal::timer::systimer::Target>();
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    // TODO: Spawn some tasks
    let _ = spawner;

    // Get an Device Specific I2c instance (Here: esp-rs)
    let i2c = esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config {
            frequency: 10.kHz(),
            timeout: None,
        },
    )
    .with_sda(peripherals.GPIO6)
    .with_scl(peripherals.GPIO1)
    .into_async();

    // Setup the driver
    let mut sensor = PressureSensorDriver::new(i2c, embassy_time::Delay, DEFAULT_SENSOR_ADDRESS);

    // Reset and init sensor
    sensor.init().await.unwrap();

    loop {
        // Real value
        let measurement = sensor.read().await.unwrap();
        info!("Sensor t = {}°C  p = {}bar", measurement.celsius(), measurement.bar());
        Timer::after(Duration::from_secs(1)).await;
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.22.0/examples/src/bin
}
