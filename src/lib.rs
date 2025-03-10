#![cfg_attr(not(test), no_std)]

#[cfg(feature = "sync")]
use embedded_hal::delay::DelayNs;
#[cfg(feature="sync")]
use embedded_hal::i2c::I2c;

#[cfg(not(feature="sync"))]
use embedded_hal_async::delay::DelayNs;
#[cfg(not(feature="sync"))]
use embedded_hal_async::i2c::I2c;
use maybe_async::maybe_async;

// As per https://cdn.sparkfun.com/datasheets/Sensors/Weather/ms5803_14ba.pdf ; Page 6
pub const DEFAULT_SENSOR_ADDRESS: u8 = 0x76;
pub const SECONDARY_SENSOR_ADDRESS: u8 = 0x77;
const RESET_COMMAND: u8 = 0x1E;
const CONVERT_D1: u8 = 0x48;
const CONVERT_D2: u8 = 0x58;
const ADC_READ: u8 = 0x00;
const PROM_READ: u8 = 0xA0;

macro_rules! defmt {
    ($body:expr) => {
        #[cfg(feature = "logging")]
        {
            use defmt::*;

            $body;
        }
        #[cfg(not(feature = "logging"))]
        {
            // No-op
        }
    };
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature measured in hundredths of a degree celsius. E.g. `2145` represents 21.45 °C 
    pub temperature: i64,
    /// Pressure measured in tenth on a millibar. E.g. `10245` represents 1024.5 mbar or 1.0245 bar
    pub pressure: i64,
}

impl Measurement {
    /// Pressure in millibar
    pub fn mbar(&self) -> f32 {
        self.pressure as f32 / 10.0
    }

    /// Pressure in bar
    pub fn bar(&self) -> f32 {
        self.mbar() / 1000.0
    }

    /// Temperature in °C
    pub fn celsius(&self) -> f32 {
        self.temperature as f32 / 100.0
    }

    /// Temperature in °F
    pub fn fahrenheit(&self) -> f32 {
        self.celsius() * 9.0 / 5.0 + 32.0
    }
}

pub struct PressureSensorDriver<I2cImpl: I2c, DelayImpl: DelayNs> {
    i2c: I2cImpl,
    delay: DelayImpl,
    calibration: [u16; 8],
    sensor_address: u8,
}

#[maybe_async]
impl<I2cImpl: I2c, DelayImpl: DelayNs> PressureSensorDriver<I2cImpl, DelayImpl> {
    pub fn new(i2c: I2cImpl, delay: DelayImpl, sensor_address: u8) -> Self {
        PressureSensorDriver {
            i2c,
            delay,
            calibration: [0u16; 8],
            sensor_address,
        }
    }

    pub async fn init(&mut self) -> Result<(), I2cImpl::Error> {
        defmt!(trace!("Resetting Sensor"));
        self.i2c
            .write(self.sensor_address, &[RESET_COMMAND])
            .await?;

        for i in 0..8 {
            self.delay.delay_ms(3).await;

            let mut buffer = [0u8; 2];
            let addr = PROM_READ + i * 2;
            self.i2c
                .write_read(self.sensor_address, &[addr], &mut buffer)
                .await?;

            let v = u16::from_be_bytes(buffer);
            self.calibration[i as usize] = v;
        }
        defmt!(trace!(
            "Received Calibration Coefficients {}",
            self.calibration
        ));

        Ok(())
    }

    pub async fn read(&mut self) -> Result<Measurement, I2cImpl::Error> {
        // OSR = 4096 -> need to wait at least 9ms
        self.delay.delay_ms(10).await;

        self.i2c.write(self.sensor_address, &[CONVERT_D1]).await?;
        self.delay.delay_ms(10).await;

        // Read ADC result
        let mut pressure_raw = [0u8; 3];
        self.i2c
            .write_read(self.sensor_address, &[ADC_READ], &mut pressure_raw)
            .await?;
        let d1 = u32::from_be_bytes([0, pressure_raw[0], pressure_raw[1], pressure_raw[2]]);
        self.delay.delay_ms(10).await;

        // Start a temperature conversion (D2)
        self.i2c.write(self.sensor_address, &[CONVERT_D2]).await?;
        self.delay.delay_ms(10).await; // Wait for conversion

        // Read ADC result
        let mut temperature_raw = [0u8; 3];
        self.i2c
            .write_read(self.sensor_address, &[ADC_READ], &mut temperature_raw)
            .await?;
        let d2: u32 = u32::from_be_bytes([
            0,
            temperature_raw[0],
            temperature_raw[1],
            temperature_raw[2],
        ]);

        let _reserved = self.calibration[0];
        // Pressure sensitivity | SENS T1
        let c1 = self.calibration[1];
        // Pressure offset | OFF T1
        let c2 = self.calibration[2];
        // Temperature coefficient of pressure sensitivity | TCS
        let c3 = self.calibration[3];
        // Temperature coefficient of pressure offset | TCO
        let c4 = self.calibration[4];
        // Reference temperature | T REF
        let c5 = self.calibration[5];
        // Temperature coefficient of the temperature | TEMPSENS
        let c6 = self.calibration[6];
        let _serial_code_and_crc = self.calibration[7];

        // Difference between actual and reference temperature
        let d_t = (d2 as i64) - c5 as i64 * 2i64.pow(8);
        
        // Actual temperature (-40…85°C with 0.01°C resolution)
        let temperature: i64 = 2_000 + (d_t * c6 as i64) / 2i64.pow(23);

        // Offset at actual temperature
        let off = c2 as i64 * 2i64.pow(16) + (c4 as i64 * d_t) / 2i64.pow(7);

        // Sensitivity at actual temperature
        let sens = c1 as i64 * 2i64.pow(15) + (c3 as i64 * d_t) / 2i64.pow(8);

        //
        // SECOND ORDER TEMPERATURE COMPENSATION
        //
        let mut off2;
        let mut sens2;
        let t2;
        if temperature < 2000 {
            // Low temperature
            t2 = 3 * (d_t).pow(2) / 2i64.pow(33);
            off2 = 3 * (temperature - 2000).pow(2) / 2;
            sens2 = 5 * (temperature - 2000).pow(2) / 2i64.pow(3);
            if temperature < -1500 {
                off2 -= 7 * (temperature + 1500).pow(2);
                sens2 += 4 * (temperature + 1500).pow(2);
            }
        } else {
            // High temperature
            t2 = 7 * (d_t).pow(2) / 2i64.pow(37);
            off2 = (temperature - 2000).pow(2) / 2i64.pow(4);
            sens2 = 0;
        }
        defmt!(trace!(
            "Without second order values {} {} {}",
            temperature,
            off,
            sens
        ));
        // Use second order correction values
        let temperature = temperature - t2;
        let off = off - off2;
        let sens = sens - sens2;
        defmt!(trace!(
            "Corrected with second order values {} {} {}",
            temperature,
            off,
            sens
        ));

        // Temperature compensated pressure (0…14bar with 0.1mbar resolution)
        let pressure: i64 = (d1 as i64 * sens / 2i64.pow(21) - off) / 2i64.pow(15);

        defmt!(trace!(
            "Got Temperature {} and Pressure {}",
            temperature,
            pressure
        ));

        // TODO second degree correction
        Ok(Measurement {
            temperature,
            pressure,
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    /// Mock test with the example values given in the data sheet
    #[tokio::test]
    async fn i2c_example_from_datasheet() -> Result<(), ()> {
        // assert_eq!(46546u16.to_be_bytes(), [0xB5, 0xF2]);
        let expectations = [
            // Reset
            I2cTransaction::write(0x76, vec![0x1e]),
            // Read calibration values
            I2cTransaction::write_read(0x76, vec![0xa0], vec![0x13, 0x37]),
            I2cTransaction::write_read(0x76, vec![0xa2], 46546u16.to_be_bytes().to_vec()),
            I2cTransaction::write_read(0x76, vec![0xa4], 42845u16.to_be_bytes().to_vec()),
            I2cTransaction::write_read(0x76, vec![0xa6], 29751u16.to_be_bytes().to_vec()),
            I2cTransaction::write_read(0x76, vec![0xa8], 29457u16.to_be_bytes().to_vec()),
            I2cTransaction::write_read(0x76, vec![0xaa], 32745u16.to_be_bytes().to_vec()),
            I2cTransaction::write_read(0x76, vec![0xac], 29059u16.to_be_bytes().to_vec()),
            I2cTransaction::write_read(0x76, vec![0xae], 0x1337u16.to_be_bytes().to_vec()),
        ];
        let i2c = I2cMock::new(&expectations);

        let mut sensor = PressureSensorDriver::new(
            i2c,
            embedded_hal_mock::eh1::delay::StdSleep,
            DEFAULT_SENSOR_ADDRESS,
        );

        sensor.init().await.unwrap();

        // Read sensor
        sensor.i2c.update_expectations(&[
            // -> Read D1
            I2cTransaction::write(0x76, vec![0x48]),
            I2cTransaction::write_read(0x76, vec![0x00], 4311550u32.to_be_bytes()[1..].to_vec()),
            // -> Read D2
            I2cTransaction::write(0x76, vec![0x58]),
            I2cTransaction::write_read(0x76, vec![0x00], 8387300u32.to_be_bytes()[1..].to_vec()),
        ]);

        let measurement = sensor.read().await.unwrap();

        assert_eq!(measurement.temperature, 2015);
        assert_eq!(measurement.pressure, 10005);

        sensor.i2c.done();

        Ok(())
    }

    #[test]
    fn measurement_conversions_are_correct() {
        let measurement = Measurement { temperature: 2015, pressure: 10005 };
        assert_eq!(measurement.bar(), 1.0005);
        assert_eq!(measurement.mbar(), 1000.5);
        assert_eq!(measurement.celsius(), 20.15);
        assert_eq!(measurement.fahrenheit(), 68.27);
    }
}
