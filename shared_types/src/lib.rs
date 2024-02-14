#![no_std]

use packed_struct::prelude::*;

/// A standard packet for transmission of basic telemetry
#[derive(Clone, Copy, Debug)]
#[derive(PackedStruct)]
#[packed_struct(bit_numbering="msb0", size_bytes=22)]
pub struct Packet {
    /// Time in hr, min, sec, µs format
    #[packed_field(bits="0..=37")]
    pub time: Time,

    /// Latitude in degrees * 1,000,000
    ///
    /// -90.000000deg - 90.000000deg
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="29")]
    pub lat: i32,

    /// Longitude in degrees * 1,000,000
    ///
    /// -180.000000deg - 180.000000deg
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="29")]
    pub lon: i32,

    /// Altitude in meters
    ///
    /// -262144m - 262144m
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="19")]
    pub alt: i32,

    /// Temperature in Kelvin
    ///
    /// 0K - 4096K
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="12")]
    pub temp: u16,

    /// Pressure in millibars * 10
    ///
    /// 0.0Mb - 6553.6Mb
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="16")]
    pub pres: u16,

    /// Acceleration in G * 100 for X, Y, Z
    ///
    /// 0.0g - 204.8g
    #[packed_field(size_bits="33")]
    pub accel: Accel,
}

impl Packet {
    pub fn as_buffer(&self) -> ([u8; 255], u8) {
        let mut bytes = [0; 255];

        let size = self.pack().iter().len();
        self.pack_to_slice(&mut bytes).unwrap();

        (bytes, size as u8)
    }

    pub fn from_buffer(buffer: &[u8]) -> Result<Self, PackingError> {
        Self::unpack_from_slice(buffer)
    }
}

/// Time
#[derive(Clone, Copy, Debug, Default)]
#[derive(PackedStruct)]
#[packed_struct(bit_numbering="msb0")]
pub struct Time {
    /// The number of hours
    #[packed_field(bits="0..=5")]
    pub hours: u8,

    /// The number of minutes
    #[packed_field(size_bits="6")]
    pub minutes: u8,

    /// The number of seconds
    #[packed_field(size_bits="6")]
    pub seconds: u8,

    /// The number of microseconds within a second
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="20")]
    pub microseconds: u32,
}

/// Acceleration in G * 100
#[derive(Clone, Copy, Debug)]
#[derive(PackedStruct)]
#[packed_struct(bit_numbering="msb0")]
pub struct Accel {
    #[packed_field(endian="lsb")]
    #[packed_field(bits="0..=11")]
    pub x: i16,
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="11")]
    pub y: i16,
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="11")]
    pub z: i16,
}
