#![no_std]

// As per https://cdn.sparkfun.com/datasheets/Sensors/Weather/ms5803_14ba.pdf ; Page 6
pub const DEFAULT_SENSOR_ADDRESS: u8 = 0x76;
pub const SECONDARY_SENSOR_ADDRESS: u8 = 0x77;
const RESET_COMMAND: u8 = 0x1E;
const CONVERT_D1: u8 = 0x48;
const CONVERT_D2: u8 = 0x58;
const ADC_READ: u8 = 0x00;
const PROM_READ: u8 = 0xA0;

use embedded_hal_async::delay::DelayNs;

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
pub struct PressureSensorDriver<I2cImpl: I2c, DelayImpl: DelayNs> {
    i2c: I2cImpl,
    delay: DelayImpl,
    calibration: [u16; 8],
    sensor_address: u8,
}

use embedded_hal_async::i2c::I2c;

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


        // OSR = 4096 -> need to wait at least 9ms
        self.delay.delay_ms(10).await;
        self.i2c.write(self.sensor_address, &[CONVERT_D1]).await?;

        Ok(())
    }

    pub async fn read(&mut self) -> Result<(i32, i64), I2cImpl::Error> {
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
        let c1 = self.calibration[1];
        let c2 = self.calibration[2];
        let c3 = self.calibration[3];
        let c4 = self.calibration[4];
        let c5 = self.calibration[5];
        let c6 = self.calibration[6];
        let _serial_code_and_crc = self.calibration[7];

        // Difference between actual and reference temperature
        let d_t = (d2 as i32) - c5 as i32 * 2i32.pow(8);

        // Actual temperature (-40…85°C with 0.01°C resolution)
        let temperature = 2_000 + (d_t * c6 as i32) / 2i32.pow(23);

        // Offset at actual temperature
        let off = c2 as i64 * 2i64.pow(16) + (c4 as i64 * d_t as i64) / 2i64.pow(7);

        // Sensitivity at actual temperature
        let sens = c1 as i64 * 2i64.pow(15) + (c3 as i64 * d_t as i64) / 2i64.pow(8);

        // Temperature compensated pressure (0…14bar with 0.1mbar resolution)
        let p: i64 = (d1 as i64 * sens / 2i64.pow(21) - off) / 2i64.pow(15);

        defmt!(trace!("Got Temperature {} and Pressure {}", temperature, p));

        // TODO second degree correction
        Ok((temperature, p))
    }
}
