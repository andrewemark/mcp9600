#![cfg_attr(not(test), no_std)]

use core::convert::TryFrom;
use embedded_hal::blocking::i2c;
use fixed::types::I12F4;

mod util;

pub type MCP9600Result<T> = Result<T, Mcp9600Error>;

const NUM_RETRIES: usize = 5;

#[derive(Debug, PartialEq)]
pub enum Mcp9600Error {
    Read,
    Write,
    Address,
    OutOfRetries,
    ThermocoupleTypeEnumFromInt,
    FilterSettingEnumFromInt,
}

/// The I2C slave address for an MCP9600 device
pub struct Address(u8);
impl Address {
    /// Construct address
    ///
    /// The I2C slave address of the MCP9600 must be between 0 and 7, inclusive.
    pub fn new(address: u8) -> MCP9600Result<Self> {
        if address > 7 {
            Err(Mcp9600Error::Address)
        } else {
            // The address byte, when later transmitted with a command, must have
            // bits 5 and 6 set (p17 of datasheet).
            Ok(Self(0x60 | address))
        }
    }
}

struct Register<const R: usize, const W: usize> {
    reg_addr: u8,
}
impl<const R: usize, const W: usize> Register<R, W> {
    fn read<I>(&self, mcp: &mut MCP9600<I>) -> MCP9600Result<[u8; R]>
    where
        I: i2c::Write + i2c::Read,
    {
        let tx_buf = [self.reg_addr];
        let mut rx_buf = [0u8; R];

        // See MCP9600 errata document for why we must retry
        for i in 1..=NUM_RETRIES {
            mcp.i2c
                .write(mcp.address.0, &tx_buf)
                .map_err(|_| Mcp9600Error::Write)?;

            if let Err(_) = mcp.i2c.read(mcp.address.0, &mut rx_buf) {
                //defmt::warn!("Read failed! Try number: {}", i);
                if i == NUM_RETRIES {
                    return Err(Mcp9600Error::OutOfRetries);
                }
                continue;
            } else {
                break;
            }
        }
        Ok(rx_buf)
    }

    fn write<I>(&self, mcp: &mut MCP9600<I>, data: [u8; R]) -> MCP9600Result<()>
    where
        I: i2c::Write + i2c::Read,
    {
        // Pack data into tx buf for sending
        let mut tx_buf = [0u8; W];
        tx_buf[0] = self.reg_addr;
        for (tx_byte, &data_byte) in tx_buf[1..].iter_mut().zip(data.iter()) {
            *tx_byte = data_byte
        }

        mcp.i2c
            .write(mcp.address.0, &tx_buf)
            .map_err(|_| Mcp9600Error::Write)?;
        Ok(())
    }
}

// TOOD: use typestates to encode which register are read, write, read/write

const THERMOCOUPLE_TEMP_REG: Register<2, 3> = Register { reg_addr: 0x00 };
const JUNCTIONS_TEMP_DELTA_REG: Register<2, 3> = Register { reg_addr: 0x01 };
const COLD_JUNCTION_TEMP_REG: Register<2, 3> = Register { reg_addr: 0x02 };
// const RAW_ADC_DATA_REG: Register<3, 4> = Register { reg_addr: 0x03 };

// const STATUS_REG: Register<1, 2> = Register { reg_addr: 0x04 };
const THERMOCOUPLE_CONFIG_REG: Register<1, 2> = Register { reg_addr: 0x05 };
// const DEVICE_CONFIG_REG: Register<1, 2> = Register { reg_addr: 0x06 };

// const ALERT_1_CONFIG_REG: Register<1, 2> = Register { reg_addr: 0x08 };
// const ALERT_2_CONFIG_REG: Register<1, 2> = Register { reg_addr: 0x09 };
// const ALERT_3_CONFIG_REG: Register<1, 2> = Register { reg_addr: 0x0A };
// const ALERT_4_CONFIG_REG: Register<1, 2> = Register { reg_addr: 0x0B };
//
// const ALERT_1_HYSTERESIS_REG: Register<1, 2> = Register { reg_addr: 0x0C };
// const ALERT_2_HYSTERESIS_REG: Register<1, 2> = Register { reg_addr: 0x0D };
// const ALERT_3_HYSTERESIS_REG: Register<1, 2> = Register { reg_addr: 0x0E };
// const ALERT_4_HYSTERESIS_REG: Register<1, 2> = Register { reg_addr: 0x0F };
//
// const ALERT_1_LIMIT_REG: Register<2, 3> = Register { reg_addr: 0x10 };
// const ALERT_2_LIMIT_REG: Register<2, 3> = Register { reg_addr: 0x11 };
// const ALERT_3_LIMIT_REG: Register<2, 3> = Register { reg_addr: 0x12 };
// const ALERT_4_LIMIT_REG: Register<2, 3> = Register { reg_addr: 0x13 };
//
// const DEVICE_INFO_REG: Register<2, 3> = Register { reg_addr: 0x20 };

/// Thermocouple type
#[repr(u8)]
pub enum ThermocoupleType {
    K = 0,
    J = 1,
    T = 2,
    N = 3,
    S = 4,
    E = 5,
    B = 6,
    R = 7,
}
impl TryFrom<u8> for ThermocoupleType {
    type Error = Mcp9600Error;

    fn try_from(v: u8) -> MCP9600Result<ThermocoupleType> {
        match v {
            x if x == Self::K as u8 => Ok(Self::K),
            x if x == Self::J as u8 => Ok(Self::J),
            x if x == Self::T as u8 => Ok(Self::T),
            x if x == Self::N as u8 => Ok(Self::N),
            x if x == Self::S as u8 => Ok(Self::S),
            x if x == Self::E as u8 => Ok(Self::E),
            x if x == Self::B as u8 => Ok(Self::B),
            x if x == Self::R as u8 => Ok(Self::R),
            _ => Err(Mcp9600Error::ThermocoupleTypeEnumFromInt),
        }
    }
}

/// Temperature filter settings
///
/// See page 31 of datasheet for equation
#[repr(u8)]
pub enum FilterSetting {
    Off = 0,
    L1 = 1,
    L2 = 2,
    L3 = 3,
    L4 = 4,
    L5 = 5,
    L6 = 6,
    L7 = 7,
}
impl TryFrom<u8> for FilterSetting {
    type Error = Mcp9600Error;

    fn try_from(v: u8) -> MCP9600Result<FilterSetting> {
        match v {
            x if x == Self::Off as u8 => Ok(Self::Off),
            x if x == Self::L1 as u8 => Ok(Self::L1),
            x if x == Self::L2 as u8 => Ok(Self::L2),
            x if x == Self::L3 as u8 => Ok(Self::L3),
            x if x == Self::L4 as u8 => Ok(Self::L4),
            x if x == Self::L4 as u8 => Ok(Self::L4),
            x if x == Self::L5 as u8 => Ok(Self::L5),
            x if x == Self::L6 as u8 => Ok(Self::L6),
            x if x == Self::L7 as u8 => Ok(Self::L7),
            _ => Err(Mcp9600Error::FilterSettingEnumFromInt),
        }
    }
}

/// MCP9600 driver
pub struct MCP9600<I> {
    i2c: I,
    address: Address,
}

impl<I> MCP9600<I>
where
    I: i2c::Read + i2c::Write,
{
    pub fn init(address: Address, i2c: I) -> MCP9600Result<Self> {
        let sensor = Self { i2c, address };
        Ok(sensor)
    }

    /// Reads the thermocouple temperature in degrees Celsius.
    ///
    /// This temperature is at the so-called hot junction (using the thermocouple).
    /// This temperature is cold-junction compensated and error-corrected.
    /// See pg. 26 of datasheet
    pub fn temperature(&mut self) -> MCP9600Result<I12F4> {
        let rx_buf = THERMOCOUPLE_TEMP_REG.read(self)?;
        let temperature = I12F4::from_be_bytes(rx_buf);
        Ok(temperature)
    }

    /// Reads the thermocouple temperature in degrees Celsius without cold-junction compensation
    ///
    /// This is the hot junction temperature with error correction but without cold-junction
    /// compensation.
    /// See pg. 27 of datasheet
    pub fn temperature_delta(&mut self) -> MCP9600Result<I12F4> {
        let rx_buf = JUNCTIONS_TEMP_DELTA_REG.read(self)?;
        let temperature = I12F4::from_be_bytes(rx_buf);
        Ok(temperature)
    }

    /// Reads the ambient temperature in degrees Celsius
    ///
    /// The chip integrates an ambient temperature sensor, which it uses as the cold-junction
    /// temperature. That temperature is obtained with this method.
    /// See pg. 28 of datasheet
    pub fn ambient_temperature(&mut self) -> MCP9600Result<I12F4> {
        // TODO: write test for conversion; split into method
        // TODO: test this method
        let mut rx_buf = COLD_JUNCTION_TEMP_REG.read(self)?;
        let msb = &mut rx_buf[0];
        // From the datashet, the MS nibble is all sign bits.
        // Clear MS nibble except for the most significant (sign) bit so we properly convert
        // to fixed point.
        *msb = *msb & 0x8F;
        let temperature = I12F4::from_be_bytes(rx_buf);
        Ok(temperature)
    }

    /// Configures the sensor's thermocouple type and/or filter settings.
    ///
    /// See pg. 31 of datasheet
    fn configure_thermocouple(
        &mut self,
        thermocouple_type: Option<ThermocoupleType>,
        filter_setting: Option<FilterSetting>,
    ) -> MCP9600Result<()> {
        let mut config = 0u8;
        if thermocouple_type.is_none() && filter_setting.is_none() {
            return Ok(());
        }
        if thermocouple_type.is_none() || filter_setting.is_none() {
            config = self.raw_thermocouple_configuration()?;
        }
        if let Some(ttype) = thermocouple_type {
            util::set_ms_nibble(&mut config, ttype as u8)
        }
        if let Some(fset) = filter_setting {
            util::set_ls_nibble(&mut config, fset as u8)
        }
        THERMOCOUPLE_CONFIG_REG.write(self, [config; 1])?;
        Ok(())
    }

    /// Reads the sensor's thermocouple configuration register
    ///
    /// See pg. 31 of datasheet
    fn raw_thermocouple_configuration(&mut self) -> MCP9600Result<u8> {
        Ok(THERMOCOUPLE_CONFIG_REG.read(self)?[0])
    }

    /// Reads the sensor's thermocouple filter setting
    ///
    /// See pg. 31 of datasheet
    pub fn filter_setting(&mut self) -> MCP9600Result<FilterSetting> {
        let config = self.raw_thermocouple_configuration()?;
        // LS nibble is the filter settings
        Ok(FilterSetting::try_from(config & 0x0F)?)
    }

    /// Set thermocouple filter setting
    ///
    /// See pg. 31 of datasheet
    pub fn configure_filter(&mut self, setting: FilterSetting) -> MCP9600Result<()> {
        Ok(self.configure_thermocouple(None, Some(setting))?)
    }

    /// Reads the sensor's thermocouple type setting
    pub fn thermocouple_type(&mut self) -> MCP9600Result<ThermocoupleType> {
        let config = self.raw_thermocouple_configuration()?;
        // MS nibble is the thermocouple type
        Ok(ThermocoupleType::try_from(config >> 4)?)
    }

    /// Set thermocouple type
    pub fn set_thermocouple_type(&mut self, t_type: ThermocoupleType) -> MCP9600Result<()> {
        Ok(self.configure_thermocouple(Some(t_type), None)?)
    }

    /// Destroys driver and releases i2c bus
    pub fn destroy(self) -> I {
        self.i2c
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::i2c;

    // When setting both the type and filter setting, we simply write the config
    #[test]
    fn set_thermocouple_configuration() {
        let address = Address(0);
        let expectations = vec![i2c::Transaction::write(address.0, vec![0x05, 0x13])];
        let mock = i2c::Mock::new(&expectations);

        let mut sensor = MCP9600::init(address, mock).unwrap();
        sensor
            .configure_thermocouple(Some(ThermocoupleType::J), Some(FilterSetting::L3))
            .unwrap();

        let mut mock = sensor.destroy();
        mock.done(); // verify expectations
    }

    #[test]
    fn read_thermocouple_configuration() {
        let address = Address(0);
        let expectations = vec![
            i2c::Transaction::write(address.0, vec![0x05]),
            i2c::Transaction::read(address.0, vec![0x13]),
        ];
        let mock = i2c::Mock::new(&expectations);

        let mut sensor = MCP9600::init(address, mock).unwrap();
        let config = sensor.raw_thermocouple_configuration().unwrap();
        assert_eq!(config, 0x13);
        let mut mock = sensor.destroy();
        mock.done();
    }

    // When setting the thermocouple type, we read, modify, then write
    #[test]
    fn set_thermocouple_type() {
        let address = Address(0);
        let expectations = vec![
            // Send register address 5
            i2c::Transaction::write(address.0, vec![0x05]),
            // Read back Type K, filter level 3
            i2c::Transaction::read(address.0, vec![0x03]),
            // Send Type J, filter level 3 retained
            i2c::Transaction::write(address.0, vec![0x05, 0x13]),
        ];
        let mock = i2c::Mock::new(&expectations);

        let mut sensor = MCP9600::init(address, mock).unwrap();
        sensor
            .configure_thermocouple(Some(ThermocoupleType::J), None)
            .unwrap();

        let mut mock = sensor.destroy();
        mock.done();
    }

    // If neither type nor filter setting is provided, we write nothing to the device
    #[test]
    fn retain_thermocouple_configuration() {
        let address = Address(0);
        let expectations = vec![];
        let mock = i2c::Mock::new(&expectations);

        let mut sensor = MCP9600::init(address, mock).unwrap();
        sensor.configure_thermocouple(None, None).unwrap();

        let mut mock = sensor.destroy();
        mock.done();
    }

    #[test]
    fn read_temperature() {
        let address = Address(0);
        let expectations = vec![
            // Send register address 0
            i2c::Transaction::write(address.0, vec![0x00]),
            // Read back 16degC + 1degC = 17degC
            i2c::Transaction::read(address.0, vec![0x01, 0x10]),
            // Send register address 0
            i2c::Transaction::write(address.0, vec![0x00]),
            // Read back -17 (twos compliment)
            i2c::Transaction::read(address.0, vec![0xFE, 0xF0]),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut sensor = MCP9600::init(address, mock).unwrap();

        let temp = sensor.temperature().unwrap();
        let expected = I12F4::from_num(17);
        assert_eq!(temp, expected);

        let temp = sensor.temperature().unwrap();
        let expected = I12F4::from_num(-17);
        assert_eq!(temp, expected);

        let mut mock = sensor.destroy();
        mock.done();
    }
}
