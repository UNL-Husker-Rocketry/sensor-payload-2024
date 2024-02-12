#![no_std]

pub use bincode;

use bincode::Encode;
use bincode::encode_into_slice;

/// A standard packet for transmission of basic telemetry
#[derive(Encode, Clone, Copy, Debug)]
pub struct Packet {
    /// Time in hr, min, sec, ms format
    pub time: Time,

    /// Latitude in degrees * 1,000,000
    pub lat: i32,

    /// Longitude in degrees * 1,000,000
    pub lon: i32,

    /// Altitude in meters
    pub alt: u16,

    /// Temp in Celsius
    pub temp: u16,

    /// Pressure in millibars * 10
    pub pres: u16,

    /// Acceleration in G * 10 for X, Y, Z
    pub accel: Accel,
}

impl Packet {
    pub fn to_buffer(&self) -> ([u8; 255], u8) {
        let mut bytes = [0; 255];
        let size = encode_into_slice(
            self,
            &mut bytes,
            bincode::config::standard()
        ).unwrap();
        (bytes, size as u8)
    }
}

/// Time
#[derive(Encode, Clone, Copy, Debug, Default)]
pub struct Time {
    pub hours: u8,
    pub minutes: u8,
    pub seconds: u8,
    pub milliseconds: u16,
}

// Acceleration in G * 10
#[derive(Encode, Clone, Copy, Debug)]
pub struct Accel {
    pub x: u8,
    pub y: u8,
    pub z: u8,
}
