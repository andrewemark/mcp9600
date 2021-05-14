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
}

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
            // bits 6 and 7 set (p17 of datasheet).
            Ok(Self(0x60 | address))
        }
    }
}

pub struct Register<const R: usize> {
    reg_addr: u8,
}
impl<const R: usize> Register<R> {
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

    // There are cases where we need to write two bytes to a register.
    // Is this possible with const generics?
    // We would write [self.address, data[1], data[0]] to the i2c bus
    fn write<I>(&self, mcp: &mut MCP9600<I>, data: u8) -> MCP9600Result<()>
    where
        I: i2c::Write + i2c::Read,
    {
        let tx_buf = [self.reg_addr, data];
        mcp.i2c
            .write(mcp.address.0, &tx_buf)
            .map_err(|_| Mcp9600Error::Write)?;
        Ok(())
    }
}

// pub struct MCP9600DoubleByteRegister {
//     address: u8,
// }
// impl MCP9600DoubleByteRegister {
//     fn write<I: i2c::Write + i2c::Read>(
//         &self,
//         mcp: &mut MCP9600<I>,
//         data: [u8; 2],
//     ) -> MCP9600Result<()> {
//         // MSB goes out first; Big Endian for transmission
//         let tx_buf = [self.address, data[1], data[0]];
//         mcp.i2c
//             .write(mcp.address.0, &tx_buf)
//             .map_err(|_| Mcp9600Error::Write)?;
//         Ok(())
//     }
// }

const THERMOCOUPLE_CONFIG_REG: Register<1> = Register {
    reg_addr: 0b0000_0101,
};
const HOT_JUNCTION_TEMP_REG: Register<2> = Register {
    reg_addr: 0b0000_0000,
};
const DELTA_TEMP_REG: Register<2> = Register {
    reg_addr: 0b0000_0001,
};

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
    /// This temperature is at the so-called hot junction. This temperature is cold-junction
    /// compensated and error-corrected.
    pub fn temperature(&mut self) -> MCP9600Result<I12F4> {
        let rx_buf = HOT_JUNCTION_TEMP_REG.read(self)?;
        let temperature = I12F4::from_be_bytes(rx_buf);
        Ok(temperature)
    }

    /// Reads the thermocouple temperature in degrees Celsius without cold-junction compensation
    pub fn temperature_delta(&mut self) -> MCP9600Result<I12F4> {
        let rx_buf = DELTA_TEMP_REG.read(self)?;
        let temperature = I12F4::from_be_bytes(rx_buf);
        Ok(temperature)
    }

    /// Configures the sensor's thermocouple type and/or filter settings.
    pub fn configure_thermocouple(
        &mut self,
        thermocouple_type: Option<ThermocoupleType>,
        filter_setting: Option<FilterSetting>,
    ) -> MCP9600Result<()> {
        let mut config = 0u8;
        if thermocouple_type.is_none() && filter_setting.is_none() {
            return Ok(());
        }
        if thermocouple_type.is_none() || filter_setting.is_none() {
            config = self.thermocouple_configuration()?;
        }
        if let Some(ttype) = thermocouple_type {
            util::set_ms_nibble(&mut config, ttype as u8)
        }
        if let Some(fset) = filter_setting {
            util::set_ls_nibble(&mut config, fset as u8)
        }
        THERMOCOUPLE_CONFIG_REG.write(self, config)?;
        Ok(())
    }

    fn thermocouple_configuration(&mut self) -> MCP9600Result<u8> {
        Ok(THERMOCOUPLE_CONFIG_REG.read(self)?[0])
    }

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
        let config = sensor.thermocouple_configuration().unwrap();
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
}
