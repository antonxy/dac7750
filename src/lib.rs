#![no_std]

//! Driver for SPI communication with the Texas Instruments DAC7750/DAC8750
//! current output digital-analog converter.
//!
//! It should also be compatible with the equivalent Analog Devices AD5410/AD5420.
//!
//! Also very similar and probably easily adaptable to Texas Instruments DAC7760/DAC8760 current
//! and voltage output DAC.

extern crate embedded_hal as hal;

use core::fmt::Debug;
use core::convert::TryInto;

use hal::blocking::spi;

use modular_bitfield_msb::prelude::*;


#[derive(BitfieldSpecifier, Debug)]
#[bits = 3]
#[allow(non_camel_case_types)]
/// Range of output in mA
pub enum Range {
    /// 4-20 mA
    mA_4_20 = 5,
    /// 0-20 mA
    mA_0_20 = 6,
    /// 0-24 mA
    mA_0_24 = 7
}

#[derive(BitfieldSpecifier, Debug)]
#[bits = 4]
pub enum SlewUpdateRate {
    Hz258_065 = 0,
    Hz200_000 = 1,
    Hz153_845 = 2,
    Hz131_145 = 3,
    Hz115_940 = 4,
    Hz69_565 = 5,
    Hz37_560 = 6,
    Hz25_805 = 7,
    Hz20_150 = 8,
    Hz16_030 = 9,
    Hz10_295 = 10,
    Hz8_280 = 11,
    Hz6_900 = 12,
    Hz5_530 = 13,
    Hz4_240 = 14,
    Hz3_300 = 15,
}

#[derive(BitfieldSpecifier, Debug)]
#[bits = 3]
pub enum SlewStepSize {
    S1 = 0,
    S2 = 1,
    S4 = 2,
    S8 = 3,
    S16 = 4,
    S32 = 5,
    S64 = 6,
    S128 = 7,
}

/// Control register
///
/// Fields:
///
/// | Name   | Description                              |
/// |--------|------------------------------------------|
/// | rext   | External current setting resistor enable |
/// | outen  | Output enable                            |
/// | srclk  | Slew update rate                         |
/// | srstep | Slew step size                           |
/// | sren   | Slew enable                              |
/// | dcne   | Daisy-chain enable                       |
/// | range  | Output range                             |
///
/// # Examples
/// ```
/// Control::new()
///  .with_outen(true)
///  .with_range(Range::mA_4_20)
/// ```
#[bitfield(bits = 16)]
#[derive(Clone, Copy, Debug)]
pub struct Control {
    #[skip] __: B2,
    /// External current setting resistor enable
    pub rext: bool,
    /// Output enable
    pub outen: bool,
    /// Slew update rate
    pub srclk: SlewUpdateRate,
    /// Slew step size
    pub srstep: SlewStepSize,
    /// Slew enable
    pub sren: bool,
    /// Daisy-chain enable
    pub dcne: bool,
    /// Output range
    pub range: Range,
}

/// Status register
///
/// Fields:
///
/// | Name    | Description                              |
/// |---------|------------------------------------------|
/// | crc_flt | CRC fault                                |
/// | wd_flt  | Watchdog fault                           |
/// | i_flt   | Current output fault                     |
/// | sr_on   | Slewing active                           |
/// | t_flt   | Temperature fault                        |
///
/// # Examples
/// ```
/// Status::new().i_flt()
/// ```
#[bitfield(bits = 16)]
#[derive(Clone, Copy, Debug)]
pub struct Status {
    #[skip] __: B11,
    /// CRC fault
    pub crc_flt: bool,
    /// Watchdog fault
    pub wd_flt: bool,
    /// Current output fault
    pub i_flt: bool,
    /// Slewing active
    pub sr_on: bool,
    /// Temperature fault
    pub t_flt: bool,
}

pub struct DAC7750<SPI>
{
    spi: SPI
}

//TODO spi access should be in critical section probably
//so that nothing can interrupt while we are transmitting
//but does a cs do that?
//or does it just prevent interrupt by routines which need the same resources?
//that would anyway not be a problem here since we took the whole SPI device
impl<E, SPI> DAC7750<SPI>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>
{
    /// Takes an spi interface and sends a reset command to the dac
    ///
    /// As SPI configuration is hardware dependent is has to be done by the caller.
    /// For example for STM32 SPI should be initialized as following:
    ///
    /// ```rust
    /// let sck = gpioc.pc10.into_alternate_af6();
    /// let miso = gpioc.pc11.into_alternate_af6();
    /// let mosi = gpioc.pc12.into_alternate_af6();
    /// let cs = gpioa.pa15.into_alternate_af6();
    /// let config = hal::spi::Config::new(hal::spi::MODE_0)
    ///     .hardware_cs(hal::spi::HardwareCS {
    ///         mode: hal::spi::HardwareCSMode::FrameTransaction,
    ///         assertion_delay: 0.0,
    ///         polarity: hal::spi::Polarity::IdleHigh,
    ///     });
    /// let spi = ctx.device.SPI3.spi((sck, miso, mosi, cs), config, 1.mhz(), ccdr.peripheral.SPI3, &ccdr.clocks);
    /// ```
    pub fn new(spi: SPI) -> Self {
        let mut dac = Self {
            spi
        };
        dac.reset();
        dac
    }

    fn transmit(&mut self, data: &[u8; 3]) {
        self.spi.write(data).ok();
    }

    fn read<'a>(&mut self, data: &'a mut [u8;3]) -> &'a[u8; 3] {
        self.spi.write(data).ok();
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        self.spi.transfer(data).ok().unwrap().try_into().unwrap()
    }

    pub fn reset(&mut self) {
        self.transmit(&[0x56, 0x0, 0x1]);
    }

    pub fn read_status(&mut self) -> Status {
        let mut data = [0x02, 0x0, 0x0];
        let r = self.read(&mut data);
        Status::from_bytes(r[1..].try_into().unwrap())
    }

    pub fn set_value(&mut self, data: u16) {
        let mut tr = [0x01, 0x0, 0x0];
        tr[1..3].copy_from_slice(&data.to_be_bytes());
        self.transmit(&tr);
    }

    pub fn read_value(&mut self) -> u16 {
        let mut data = [0x02, 0x0, 0x01];
        let res = self.read(&mut data);
        u16::from_be_bytes(res[1..].try_into().unwrap())
    }

    pub fn set_control(&mut self, ctrl: Control) {
        let mut tr = [0x55, 0x0, 0x0];
        tr[1..3].copy_from_slice(&ctrl.into_bytes());
        self.transmit(&tr);
    }

    pub fn read_control(&mut self) -> Control {
        let mut data = [0x02, 0x0, 0x02];
        let res = self.read(&mut data);
        Control::from_bytes(res[1..].try_into().unwrap())
    }
}
