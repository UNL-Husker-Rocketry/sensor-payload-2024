#![no_std]

use core::fmt::Display;
use packed_struct::prelude::*;
use bincode::{Encode, Decode, error::DecodeError};
use crc::{Crc, CRC_8_BLUETOOTH};

pub const CRC_CALC: Crc<u8> = Crc::<u8>::new(&CRC_8_BLUETOOTH);

/// A standard packet for transmission of basic telemetry
#[derive(Default, Clone, Copy, Debug, Encode, Decode, PackedStruct)]
#[packed_struct(bit_numbering="msb0")]
pub struct Packet {
    /// Time in hr, min, sec, counter format
    ///
    /// The counter value indicates which packet within the second it is, from
    /// 0 - 4 if the packet is sent 5 times per second.
    #[packed_field(bits="0..=20")]
    pub time: Time,

    /// Latitude in degrees * 1,000,000
    ///
    /// -90.000000deg - 90.000000deg
    #[packed_field(endian="lsb", size_bits="28")]
    pub lat: i32,

    /// Longitude in degrees * 1,000,000
    ///
    /// -180.000000deg - 180.000000deg
    #[packed_field(endian="lsb", size_bits="29")]
    pub lon: i32,

    /// Altitude in meters
    ///
    /// -262,144m - 262,144m
    #[packed_field(endian="lsb", size_bits="19")]
    pub alt: i32,

    /// Temperature in Celsius * 10
    ///
    /// -1638.4C - 1638.4C
    #[packed_field(endian="lsb", size_bits="15")]
    pub temp: i16,

    /// Pressure in millibars * 10,000
    ///
    /// 0.0000Mb - 1677.7216Mb
    #[packed_field(endian="lsb", size_bits="24")]
    pub pres: u32,

    /// CRC of the packet to ensure integrity
    #[packed_field(size_bits="8")]
    pub crc: u8,
}

impl Packet {
    /// Returns the [Packet] as a bit-packed buffer of 255 bytes,
    /// along with the size of the data within the buffer.
    pub fn as_buffer(&self) -> ([u8; 255], u8) {
        let mut bytes = [0; 255];

        let packed = self.pack().unwrap();

        for (i, byte) in packed.iter().enumerate() {
            bytes[i] = *byte;
        }

        (bytes, packed.len() as u8)
    }

    /// Returns the [Packet] as an unpacked byte array
    pub fn as_bytes(&self) -> [u8; 0x18] {
        let mut array = [0u8; 0x18];
        bincode::encode_into_slice(self, &mut array[0..0x18], bincode::config::legacy()).unwrap();
        array
    }

    /// Takes in a buffer and unpacks the bit-packing to return a
    /// new [Packet] with the original data.
    pub fn from_buffer(buffer: &[u8]) -> Result<Self, PackingError> {
        let mut small_buf = [0; 0x12];
        small_buf.copy_from_slice(&buffer[..0x12]);

        Self::unpack_from_slice(&small_buf)
    }

    /// Turns a slice into a [Packet]
    pub fn from_bytes(buffer: &[u8]) -> Result<Self, DecodeError> {
        let result = bincode::decode_from_slice(buffer, bincode::config::legacy())?;
        Ok(result.0)
    }

    /// Sets the CRC field of the packet using its own data
    pub fn set_crc(&mut self) {
        let bytes = &self.pack().unwrap()[0..0x11];

        self.crc = CRC_CALC.checksum(bytes);
    }

    /// Checks the CRC field of the packet against its current data,
    /// returning false if the data is different
    pub fn validate(&self) -> bool {
        let bytes = &self.pack().unwrap()[0..0x11];

        let new_crc = CRC_CALC.checksum(bytes);

        new_crc == self.crc
    }
}

/// Time
#[derive(Clone, Copy, Debug, Default, Encode, Decode, PackedStruct)]
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
    #[packed_field(size_bits="3")]
    pub counter: u8,
}

impl Display for Time {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:02}:{:02}:{:02}.{:06}", self.hours, self.minutes, self.seconds, self.counter)
    }
}
