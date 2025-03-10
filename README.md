# `embedded-hal` async and sync driver for the MS5803-14BA pressure sensor

This library provides a driver for the [MS5803-14BA](https://cdn.sparkfun.com/datasheets/Sensors/Weather/ms5803_14ba.pdf) pressure sensor using the `embedded-hal` async abstractions over I2C and Delay.

<img src="https://www.sparkfun.com/media/catalog/product/cache/a793f13fd3d678cea13d28206895ba0c/1/2/12909-01a.jpg" width="50%">


## Usage

```rust
use ms5803_14ba::{PressureSensorDriver, DEFAULT_SENSOR_ADDRESS};

// Get an Device Specific I2c instance (Here: esp-rs)
let i2c = I2c::new(
    peripherals.I2C0,
    i2c::master::Config {
        frequency: 40.kHz(),
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
    let result = format!(
        "Sensor t = {:.2}°C  p = {:.4}bar",
        measurement.celsius(),
        measurement.bar(),
    );
    info!("{=str}", &result);
    Timer::after(Duration::from_secs(1)).await;
}
```

which logs:

```log
INFO  Sensor t = 19.46°C  p = 1.0187bar
INFO  Sensor t = 19.46°C  p = 1.0190bar
...
```

more examples can be found [here](./examples/)

The examples use an esp32c3 with embassy. You can use any other `embedded-hal` impl though.

Pins are connected like this:

esp32-c3 pin | ms5803 sensor
------------ | -------------
GPIO6        | SDA
GPIO1        | SCL
3V3          | 3V3
GND          | GND