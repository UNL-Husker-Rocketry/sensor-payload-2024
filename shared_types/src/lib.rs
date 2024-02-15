#![no_std]

use core::fmt::Display;

use packed_struct::prelude::*;

/// A standard packet for transmission of basic telemetry
#[derive(Clone, Copy, Debug)]
#[derive(PackedStruct)]
#[packed_struct(bit_numbering="msb0")]
pub struct Packet {
    /// Time in hr, min, sec, µs format
    #[packed_field(bits="0..=37")]
    pub time: Time,

    /// Latitude in degrees * 1,000,000
    ///
    /// -90.000000deg - 90.000000deg
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="28")]
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

    /// Acceleration in G * 10 for X
    ///
    /// -102.4g - 102.4g
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="11")]
    pub accel_x: i16,

    /// Acceleration in G * 10 for Y
    ///
    /// -102.4g - 102.4g
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="11")]
    pub accel_y: i16,

    /// Acceleration in G * 10 for Z
    ///
    /// -102.4g - 102.4g
    #[packed_field(endian="lsb")]
    #[packed_field(size_bits="11")]
    pub accel_z: i16,
}

impl Packet {
    /// Returns the [Packet] as a bit-packed buffer of 255 bytes,
    /// along with the size of the data within the buffer.
    pub fn as_buffer(&self) -> ([u8; 255], u8) {
        let mut byte_temp = [0; 22];
        let mut bytes = [0; 255];

        self.pack_to_slice(&mut byte_temp).unwrap();

        for (i, byte) in byte_temp.iter().enumerate() {
            bytes[i] = *byte;
        }

        (bytes, byte_temp.len() as u8)
    }

    /// Takes in a buffer and unpacks the bit-packing to return a
    /// new [Packet] with the original data.
    pub fn from_buffer(buffer: &[u8]) -> Result<Self, PackingError> {
        let mut small_buf = [0; 22];
        for i in 0..22 {
            small_buf[i] = buffer[i]
        }

        Self::unpack_from_slice(&small_buf)
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

impl Display for Time {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}:{}:{}.{}", self.hours, self.minutes, self.seconds, self.microseconds)
    }
}
