use common::components::{GenericMotorId, MotorRawSignalRange};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum LocalMotorId {
    PwmChannel(PwmChannel),
    DcChannel(DcChannel),
}

impl LocalMotorId {
    pub fn default_signal_range(&self) -> MotorRawSignalRange {
        match self {
            LocalMotorId::PwmChannel(pwm_channel) => pwm_channel.default_signal_range(),
            LocalMotorId::DcChannel(dc_channel) => dc_channel.default_signal_range(),
        }
    }
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct PwmChannel(u8);

impl PwmChannel {
    pub fn new(id: u8) -> Self {
        assert!((0..16).contains(&id), "Pwm Channel {id} is invalid");

        Self(id)
    }

    pub fn id(&self) -> u8 {
        self.0
    }

    pub fn default_signal_range(&self) -> MotorRawSignalRange {
        MotorRawSignalRange {
            min: 1100,
            center: 1500,
            max: 1900,
        }
    }
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct DcChannel(u8);

impl DcChannel {
    pub fn new(id: u8) -> Self {
        assert!((0..4).contains(&id), "Dc Channel {id} is invalid");

        Self(id)
    }

    pub fn id(&self) -> u8 {
        self.0
    }

    pub fn default_signal_range(&self) -> MotorRawSignalRange {
        MotorRawSignalRange {
            min: i16::MIN as _,
            center: 0,
            max: i16::MAX as _,
        }
    }
}

impl From<LocalMotorId> for GenericMotorId {
    fn from(value: LocalMotorId) -> Self {
        GenericMotorId(match value {
            LocalMotorId::PwmChannel(pwm_channel) => pwm_channel.id(),
            LocalMotorId::DcChannel(dc_channel) => dc_channel.id() | 0x80,
        })
    }
}

impl From<GenericMotorId> for LocalMotorId {
    fn from(value: GenericMotorId) -> Self {
        let motor_type = value.0 >> 7;
        let id = value.0 & 0x7F;

        if motor_type == 0 {
            LocalMotorId::PwmChannel(PwmChannel::new(id))
        } else {
            LocalMotorId::DcChannel(DcChannel::new(id))
        }
    }
}
