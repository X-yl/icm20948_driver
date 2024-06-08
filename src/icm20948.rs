// Copyright (c) 2022, Zachary D. Olkin.
// This code is provided under the MIT license.

use defmt::{Format, Formatter};

/// The i2c module holds all of the driver implementations when using an I2C bus to communicate with the device
pub mod i2c;
/// The SPI module holds all of the driver implementations when using an SPI bus to communicate with the device
pub mod spi;

const READ_REG: bool = true;
const WRITE_REG: bool = false;

const INT_ENABLED: bool = true;
const INT_NOT_ENABLED: bool = false;

const ACCEL_SEN_0: u16 = 16_384;
const ACCEL_SEN_1: u16 = 8_192;
const ACCEL_SEN_2: u16 = 4_096;
const ACCEL_SEN_3: u16 = 2_048;

const GYRO_SEN_0: f32 = 131.0;
const GYRO_SEN_1: f32 = 65.5;
const GYRO_SEN_2: f32 = 32.8;
const GYRO_SEN_3: f32 = 16.4;

const REG_BANK_0: u8 = 0x00;
//const REG_BANK_1: u8 = 0x10;
const REG_BANK_2: u8 = 0x20;
//const REG_BANK_3: u8 = 0x30;

/// States of the accelerometer: On or Off
#[derive(PartialEq, Format)]
pub enum AccStates {
    /// On
    AccOn,
    /// Off
    AccOff,
}

/// States of the gyro: On or Off
#[derive(PartialEq, Format)]
pub enum GyroStates {
    /// On
    GyroOn,
    /// Off
    GyroOff,
}

/// States of the magnetometer: On or Off
#[derive(PartialEq, Format)]
pub enum MagStates {
    /// On
    MagOn,
    /// Off
    MagOff,
}

/// Accelerometer sensitivity options as specified in the data sheet in g's.
pub enum AccSensitivity {
    /// 2g of sensitivity
    Sen2g,
    /// 4g of sensitivity
    Sen4g,
    /// 8g of sensitivity
    Sen8g,
    /// 16g of sensitivity
    Sen16g,
}

/// Gyro sensitivity options as specified in the data sheet in degrees per second (dps).
pub enum GyroSensitivity {
    /// 250 dps of sensitivity
    Sen250dps,
    /// 500 dps of sensitivity
    Sen500dps,
    /// 1000 dps of sensitivity
    Sen1000dps,
    /// 2000 dps of sensitivity
    Sen2000dps,
}

/// Accelerometer Low Pass Filter (LPF) options as specified in the data sheet.
/// All values are in the 3DB BW of the LPF.
pub enum AccLPF {
    /// 246.0Hz 3DB BW
    BW246,
    /// 111.4Hz 3DB BW
    BW111,
    /// 50.4Hz 3DB BW
    BW50,
    /// 23.9Hz 3DB BW
    BW24,
    /// 11.5Hz 3DB BW
    BW11,
    /// 5.7Hz 3DB BW
    BW6,
    /// 473.0Hz 3DB BW
    Bw473,
}

/// Gyro Low Pass Filter (LPF) options as specified in the data sheet.
/// All values are in the 3DB BW of the LPF.
pub enum GyroLPF {
    /// 196.6Hz 3DB BW
    BW197,
    /// 151.8Hz 3DB BW
    BW152,
    /// 119.5Hz 3DB BW
    BW119,
    /// 51.2Hz 3DB BW
    BW51,
    /// 23.9Hz 3DB BW
    BW24,
    /// 11.6Hz 3DB BW
    BW12,
    /// 5.7Hz 3DB BW
    BW6,
    /// 361.4Hz 3DB BW
    BW361,
}

/// The possible errors that the driver can return.
///
/// The `BusError` option is for when a HAL function using either the SPI or I2C bus fails.
/// This may be caused by a number of reasons. For example, using the wrong 7-bit address with the I2C bus will cause a bus error.
///
/// `InvalidInput` is for when an input to a driver function is unacceptable.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum IcmError<E> {
    /// An error occurred when using the bus
    BusError(E),
    /// An invalid input was passed to the function
    InvalidInput,
}

#[allow(dead_code)]
enum RegistersBank0 {
    Wai,
    PwrMgmt1,
    PwrMgmt2,
    IntEnable1,
    AccelXOutH,
    AccelXOutL,
    AccelYOutH,
    AccelYOutL,
    AccelZOutH,
    AccelZOutL,
    GyroXOutH,
    GyroXOutL,
    GyroYOutH,
    GyroYOutL,
    GyroZOutH,
    GyroZOutL,
    RegBankSel,
    FifoCountH,
    FifoCountL,
    HwFixDisable,
    FifoPrioritySel,
    FifoRW,
    FifoRst,
    UserCtrl,
    FifoEn2,
    MemStartAddr,
    MemRW,
    MemBankSel,
}
/*
enum RegistersBank1 {
    XAOffsH,
    XAOffsL,
    YAOffsH,
    YAOffsL,
    ZAOffsH,
    ZAOffsL,
}*/

enum RegistersBank2 {
    GyroConfig1,
    AccelConfig,
    AccelSmplrtDiv1,
    AccelSmplrtDiv2,
    GyroSmplrtDiv,
    OdrAlignEn,
    PrgmStrtAddrH,
}

impl<E> From<E> for IcmError<E> {
    fn from(error: E) -> Self {
        IcmError::BusError(error)
    }
}

#[cfg(feature = "defmt")]
impl<E: Format> Format for IcmError<E> {
    fn format(&self, fmt: Formatter) {
        match &*self {
            IcmError::BusError(e) => defmt::write!(fmt, "Bus Error: {}", e),
            IcmError::InvalidInput => defmt::write!(fmt, "Invalid input in the function"),
        }
    }
}

impl RegistersBank0 {
    fn get_addr(&self, is_read: bool) -> u8 {
        (match *self {
            RegistersBank0::Wai => 0x0,
            RegistersBank0::PwrMgmt1 => 0x06,
            RegistersBank0::PwrMgmt2 => 0x07,
            RegistersBank0::IntEnable1 => 0x11,
            RegistersBank0::AccelXOutH => 0x2D,
            RegistersBank0::AccelXOutL => 0x2E,
            RegistersBank0::AccelYOutH => 0x2F,
            RegistersBank0::AccelYOutL => 0x30,
            RegistersBank0::AccelZOutH => 0x31,
            RegistersBank0::AccelZOutL => 0x32,
            RegistersBank0::GyroXOutH => 0x33,
            RegistersBank0::GyroXOutL => 0x34,
            RegistersBank0::GyroYOutH => 0x35,
            RegistersBank0::GyroYOutL => 0x36,
            RegistersBank0::GyroZOutH => 0x37,
            RegistersBank0::GyroZOutL => 0x38,
            RegistersBank0::HwFixDisable => 0x75,
            RegistersBank0::RegBankSel => 0x7F,
            RegistersBank0::MemStartAddr => 0x7C,
            RegistersBank0::MemRW => 0x7D,
            RegistersBank0::MemBankSel => 0x7E,
            RegistersBank0::FifoPrioritySel => 0x26,
            RegistersBank0::FifoCountH => 0x70,
            RegistersBank0::FifoCountL => 0x71,
            RegistersBank0::FifoRW => 0x72,
            RegistersBank0::UserCtrl => 0x03,
            RegistersBank0::FifoEn2 => 0x67,
            RegistersBank0::FifoRst => 0x68,
        }) | if is_read { 1 << 7 } else { 0 }
    }
}

impl RegistersBank2 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            match *self {
                RegistersBank2::GyroSmplrtDiv => 1 << 7 | 0x00,
                RegistersBank2::GyroConfig1 => 1 << 7 | 0x01,
                RegistersBank2::AccelSmplrtDiv1 => 1 << 7 | 0x10,
                RegistersBank2::AccelSmplrtDiv2 => 1 << 7 | 0x11,
                RegistersBank2::AccelConfig => 1 << 7 | 0x14,
                RegistersBank2::OdrAlignEn => 1 << 7 | 0x09,
                RegistersBank2::PrgmStrtAddrH => 1 << 7 | 0x50
            }
        } else {
            match *self {
                RegistersBank2::GyroSmplrtDiv => 0x00,
                RegistersBank2::GyroConfig1 => 0x01,
                RegistersBank2::AccelSmplrtDiv1 => 0x10,
                RegistersBank2::AccelSmplrtDiv2 => 0x11,
                RegistersBank2::AccelConfig => 0x14,
                RegistersBank2::OdrAlignEn => 0x09,
                RegistersBank2::PrgmStrtAddrH => 0x50
            }
        }
    }
}

/*
impl RegistersBank1 {
    fn get_addr(&self, is_read: bool) -> u8 {
        if is_read {
            match *self {
                RegistersBank1::XAOffsH => 1 << 7 | 0x14,
                RegistersBank1::XAOffsL => 1 << 7 | 0x15,
                RegistersBank1::YAOffsH => 1 << 7 | 0x17,
                RegistersBank1::YAOffsL => 1 << 7 | 0x18,
                RegistersBank1::ZAOffsH => 1 << 7 | 0x1A,
                RegistersBank1::ZAOffsL => 1 << 7 | 0x1B,
            }
        } else {
            match *self {
                RegistersBank1::XAOffsH => 0x14,
                RegistersBank1::XAOffsL => 0x15,
                RegistersBank1::YAOffsH => 0x17,
                RegistersBank1::YAOffsL => 0x18,
                RegistersBank1::ZAOffsH => 0x1A,
                RegistersBank1::ZAOffsL => 0x1B,
            }
        }
    }
}*/

/// No DMP available.
/// Use `enableDMP` to enable the dmp.
pub struct NoDmp;

#[cfg(feature = "dmp")]
pub mod dmp {
    use core::{
        array::TryFromSliceError,
        convert::{TryFrom, TryInto},
    };

    use bitflags::bitflags;
    use nalgebra::{Quaternion, Vector3};
    use num_traits::float::FloatCore;
    use num_traits::FromBytes;

    pub(crate) const FIRMWARE: &'static [u8] = include_bytes!("../icm20498_dmp_firmware.bin");

    #[derive(Debug, Clone, Eq, PartialEq)]
    pub enum Sensor {
        Accel,
        Gyro,
    }

    /// DMP available.
    pub struct Dmp;

    /// Registers for the DMP.
    /// See this "proprietary" and "confidential" document for more information:
    /// https://www.cdiweb.com/datasheets/invensense/application_note-programming_sequence_for_dmp_hw_functions_icm_20648.pdf#%5B%7B%22num%22%3A103%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C62%2C689%2C0%5D
    #[allow(dead_code)]
    #[derive(Debug, Clone, Eq, PartialEq)]
    pub(crate) enum DmpRegisters {
        DataOutCtl1 = 0x40,
        DataOutCtl2 = 0x42,
        DataIntrCtl = 0x4c,
        MotionEventCtl = 0x4e,
        DataRdyStatus = 0x8a,
        OdrAccel = 0xbe,
        OdrGyro = 0xba,
        OdrCpass = 0xb6,
        OdrAls = 0xb2,
        OdrQuat6 = 0xac,
        OdrQuat9 = 0xa8,
        OdrPquat6 = 0xa4,
        OdrGeomag = 0xa0,
        OdrPressure = 0xbc,
        OdrGyroCalibr = 0xb8,
        OdrCpassCalibr = 0xb4,
        OdrCntrAccel = 0x9e,
        OdrCntrGyro = 0x9a,
        OdrCntrCpass = 0x96,
        OdrCntrAls = 0x92,
        OdrCntrQuat6 = 0x8c,
        OdrcntrQuat9 = 0x88,
        OdrCntrPquat6 = 0x84,
        OdrCntrGeomag = 0x80,
        OdrCntrPressure = 0x9c,
        OdrCntrGyroCalibr = 0x98,
        OdrCntrCpassCalibr = 0x94,
        BmBatchCntr = 0x1b0,
        BmBatchThld = 0x13c,
        BmBatchMask = 0x15e,
        CPassMtx00 = 0x170,
        CPassMtx01 = 0x174,
        CPassMtx02 = 0x178,
        CPassMtx10 = 0x17c,
        CPassMtx11 = 0x180,
        CPassMtx12 = 0x184,
        CPassMtx20 = 0x188,
        CPassMtx21 = 0x18c,
        CPassMtx22 = 0x190,
        FifoWatermark = 0x1fe,
        YroBiasX = 0x8b4,
        YroBiasY = 0x8b8,
        YroBiasZ = 0x8bc,
        CcelBiasX = 0x6e4,
        CcelBiasY = 0x6e8,
        CcelBiasZ = 0x6ec,
        PassBiasX = 0x7e4,
        PassBiasY = 0x7e8,
        PassBiasZ = 0x7ec,
        CcScale = 0x1e0,
        CcScale2 = 0x4f4,
        EdstdStepctr = 0x360,
        EdstdTimectr = 0x3c4,
        AcRate = 0x30a,
        CcelCalRate = 0x5e4,
        CcelAlphaVar = 0x5b0,
        CcelAVar = 0x5c0,
        PassTimeBuffer = 0x70e,
        CcelOnlyGain = 0x10c,
        YroSf = 0x130,
    }

    bitflags! {
       #[derive(Debug, Clone, Eq, PartialEq, Default)]
       pub struct Header: u16 {
           const Header2              = 0x0008;
           const Accel                = 0x8000;
           const Gyro                 = 0x4000;
           const Compass              = 0x2000;
           const ALS                  = 0x1000;
           const Quaternion6          = 0x0800;
           const Quaternion9          = 0x0400;
           const PedometerQuaternion6 = 0x0200;
           const Geomagrv             = 0x0100;
           const Pressure             = 0x0080;
           const CalibratedGyro       = 0x0040;
           const CalibratedCompass    = 0x0020;
           const StepDetector         = 0x0010;
        }
    }

    bitflags! {
        #[derive(Debug, Clone, Eq, PartialEq, Default)]
        pub struct Header2: u16 {
            const AccelAccuracy       = 0x4000;
            const GyroAccuracy        = 0x2000;
            const CompassAccuracy     = 0x1000;
            const Fsync               = 0x0800;
            const Pickup              = 0x0400;
            const ActivityRecognition = 0x0080;
            const SecondaryOnOff      = 0x0040;
        }
    }

    #[derive(Debug, Clone, Eq, PartialEq)]
    pub struct Gyro {
        pub values: Vector3<i16>,
        pub bias: Vector3<i16>,
    }

    #[derive(Debug, Clone, PartialEq)]
    pub struct Quaternion9 {
        pub values: Quaternion<f32>,
        pub accuracy: i16,
    }

    #[derive(Debug, Clone, Eq, PartialEq)]
    pub enum Accuracy {
        Zero = 0,
        One = 1,
        Two = 2,
        Three = 3,
    }

    impl TryFrom<u8> for Accuracy {
        type Error = ();

        fn try_from(value: u8) -> Result<Self, Self::Error> {
            match value {
                0 => Ok(Accuracy::Zero),
                1 => Ok(Accuracy::One),
                2 => Ok(Accuracy::Two),
                3 => Ok(Accuracy::Three),
                _ => Err(()),
            }
        }
    }

    #[derive(Debug, Clone, PartialEq, Default)]
    pub struct Packet {
        pub header: Header,
        pub header2: Option<Header2>,
        pub accel: Option<Vector3<i16>>,
        pub gyro: Option<Gyro>,
        pub compass: Option<Vector3<i16>>,
        pub als: Option<[u8; ALS_BYTES]>,
        pub quaternion6: Option<Quaternion<f32>>,
        pub quaternion9: Option<Quaternion9>,
        pub pedometer_quaternion6: Option<Quaternion<f32>>,
        pub geomagrv: Option<Quaternion9>,
        pub pressure: Option<u32>,
        pub temperature: Option<u32>,
        pub gyro_calibrated: Option<Vector3<i32>>,
        pub compass_calibrated: Option<Vector3<i32>>,
        pub pedometer_time: Option<u32>,
        pub accel_accuracy: Option<Accuracy>,
        pub gyro_accuracy: Option<Accuracy>,
        pub fsync_delay_time: Option<u16>,
        pub pickup: Option<bool>,
        // TODO: Activity recognition
        // TODO: Secondary on/off
    }

    #[derive(Debug, Clone, Eq, PartialEq)]
    pub enum ParseError {
        TooShort,
        InvalidHeader,
        InvalidAccuracy,
    }

    pub struct Parser<'a> {
        current: &'a [u8],
    }

    impl<'a> Iterator for Parser<'a> {
        type Item = Result<Packet, ParseError>;

        fn next(&mut self) -> Option<Self::Item> {
            (move || {
                if self.current.is_empty() {
                    return Ok(None);
                }
                let header = self.consume_num::<u16>()?;
                let header = Header::from_bits(header).ok_or(ParseError::InvalidHeader)?;

                let header2 = if header.contains(Header::Header2) {
                    let header2 = self.consume_num::<u16>()?;
                    Some(Header2::from_bits(header2).ok_or(ParseError::InvalidHeader)?)
                } else {
                    None
                };

                let accel = if header.contains(Header::Accel) {
                    let x = self.consume_num::<i16>()?;
                    let y = self.consume_num::<i16>()?;
                    let z = self.consume_num::<i16>()?;
                    Some(Vector3::new(x, y, z))
                } else {
                    None
                };

                let gyro = if header.contains(Header::Gyro) {
                    let x = self.consume_num::<i16>()?;
                    let y = self.consume_num::<i16>()?;
                    let z = self.consume_num::<i16>()?;
                    let bx = self.consume_num::<i16>()?;
                    let by = self.consume_num::<i16>()?;
                    let bz = self.consume_num::<i16>()?;
                    Some(Gyro {
                        values: Vector3::new(x, y, z),
                        bias: Vector3::new(bx, by, bz),
                    })
                } else {
                    None
                };

                let compass = if header.contains(Header::Compass) {
                    let x = self.consume_num::<i16>()?;
                    let y = self.consume_num::<i16>()?;
                    let z = self.consume_num::<i16>()?;
                    Some(Vector3::new(x, y, z))
                } else {
                    None
                };

                let als = if header.contains(Header::ALS) {
                    let mut data = [0; ALS_BYTES];
                    data.copy_from_slice(self.consume(ALS_BYTES)?);
                    Some(data)
                } else {
                    None
                };

                let quaternion6 = if header.contains(Header::Quaternion6) {
                    let i = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let j = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let k = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let w = libm::sqrtf(1f32 - i.powi(2) - j.powi(2) - k.powi(2));
                    Some(Quaternion::new(w, i, j, k))
                } else {
                    None
                };

                let quaternion9 = if header.contains(Header::Quaternion9) {
                    let i = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let j = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let k = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let w = libm::sqrtf(1f32 - i.powi(2) - j.powi(2) - k.powi(2));
                    let accuracy = self.consume_num::<i16>()?;
                    Some(Quaternion9 {
                        values: Quaternion::new(w, i, j, k),
                        accuracy,
                    })
                } else {
                    None
                };

                let pedometer_quaternion6 = if header.contains(Header::PedometerQuaternion6) {
                    let i = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let j = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let k = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let w = libm::sqrtf(1f32 - i.powi(2) - j.powi(2) - k.powi(2));
                    Some(Quaternion::new(w, i, j, k))
                } else {
                    None
                };

                let geomagrv = if header.contains(Header::Geomagrv) {
                    let x = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let y = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let z = self.consume_num::<i32>()? as f32 / 2f32.powi(30);
                    let w = libm::sqrtf(1f32 - x.powi(2) - y.powi(2) - z.powi(2));
                    let accuracy = self.consume_num::<i16>()?;

                    Some(Quaternion9 {
                        values: Quaternion::new(w, x, y, z),
                        accuracy,
                    })
                } else {
                    None
                };

                let pressure = if header.contains(Header::Pressure) {
                    let mut pres = [0; 4];
                    pres[1..].copy_from_slice(self.consume(3)?);
                    Some(u32::from_be_bytes(pres))
                } else {
                    None
                };

                let temperature = if header.contains(Header::Pressure) {
                    let mut temp = [0; 4];
                    temp[1..].copy_from_slice(self.consume(3)?);
                    Some(u32::from_be_bytes(temp))
                } else {
                    None
                };

                let gyro_calibrated = if header.contains(Header::CalibratedGyro) {
                    let x = self.consume_num::<i32>()?;
                    let y = self.consume_num::<i32>()?;
                    let z = self.consume_num::<i32>()?;
                    Some(Vector3::new(x, y, z))
                } else {
                    None
                };

                let compass_calibrated = if header.contains(Header::CalibratedCompass) {
                    let x = self.consume_num::<i32>()?;
                    let y = self.consume_num::<i32>()?;
                    let z = self.consume_num::<i32>()?;
                    Some(Vector3::new(x, y, z))
                } else {
                    None
                };

                let pedometer_time = if header.contains(Header::StepDetector) {
                    Some(self.consume_num::<u32>()?)
                } else {
                    None
                };

                let accel_accuracy = if header2
                    .as_ref()
                    .is_some_and(|header2| header2.contains(Header2::AccelAccuracy))
                {
                    Some(
                        (self.consume_num::<u16>()? as u8)
                            .try_into()
                            .map_err(|_| ParseError::InvalidAccuracy)?,
                    )
                } else {
                    None
                };

                let gyro_accuracy = if header2
                    .as_ref()
                    .is_some_and(|header2| header2.contains(Header2::GyroAccuracy))
                {
                    Some(
                        (self.consume_num::<u16>()? as u8)
                            .try_into()
                            .map_err(|_| ParseError::InvalidAccuracy)?,
                    )
                } else {
                    None
                };

                let fsync_delay_time = if header2
                    .as_ref()
                    .is_some_and(|header2| header2.contains(Header2::Fsync))
                {
                    Some(self.consume_num::<u16>()?)
                } else {
                    None
                };

                let pickup = if header2
                    .as_ref()
                    .is_some_and(|header2| header2.contains(Header2::Pickup))
                {
                    Some(self.consume_num::<u16>()? != 0)
                } else {
                    None
                };

                Ok(Some(Packet {
                    header,
                    header2,
                    accel,
                    gyro,
                    compass,
                    als,
                    quaternion6,
                    quaternion9,
                    pedometer_quaternion6,
                    geomagrv,
                    pressure,
                    temperature,
                    gyro_calibrated,
                    compass_calibrated,
                    pedometer_time,
                    accel_accuracy,
                    gyro_accuracy,
                    fsync_delay_time,
                    pickup,
                }))
            })()
            .transpose()
        }
    }

    impl<'a> Parser<'a> {
        pub fn new(data: &'a [u8]) -> Self {
            Self {
                current: data,
            }
        }

        fn consume(&mut self, len: usize) -> Result<&'a [u8], ParseError> {
            if let Some((head, tail)) = self.current.split_at_checked(len) {
                self.current = tail;
                Ok(head)
            } else {
                Err(ParseError::TooShort)
            }
        }

        fn consume_num<N: FromBytes>(&mut self) -> Result<N, ParseError>
        where
            for<'b> &'b <N as FromBytes>::Bytes: TryFrom<&'b [u8], Error = TryFromSliceError>,
        {
            if let Some((head, tail)) = self.current.split_at_checked(core::mem::size_of::<N>()) {
                self.current = tail;
                Ok(N::from_be_bytes(head.try_into().unwrap()))
            } else {
                Err(ParseError::TooShort)
            }
        }
    }

    pub trait Captures<U> {}
    impl<T: ?Sized, U> Captures<U> for T {}

    impl Packet {
        pub fn parse<'a>(
            data: &'a [u8],
        ) -> impl Iterator<Item = Result<Packet, ParseError>> + Captures<&'a ()> {
            Parser::new(data)
        }
    }

    pub const ALS_BYTES: usize = 8;
}
