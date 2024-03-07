#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts, gpio::{self, AnyPin}, i2c::{self, I2c}, peripherals::{I2C1, SPI1, UART1, USB}, rtc::{DateTime, Rtc}, spi::{Blocking, Spi}, uart::{self, Async, UartRx}, usb::{self, Driver}
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::{Delay, Timer, Instant};
use gpio::{Level, Output};
use log::info;
use {defmt_rtt as _, panic_probe as _};
use shared_types::{Packet, Time};

use bmp388::BMP388;
use mma8x5x::{Mma8x5x, Measurement};
use nmea0183::{ParseResult, Parser, GGA, coords::Hemisphere};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART1_IRQ => uart::InterruptHandler<UART1>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

/// Set up a task to blink the LED
#[embassy_executor::task]
async fn blink_led(mut led: Output<'static, AnyPin>) {
    loop {
        led.set_high();
        Timer::after_secs(1).await;

        led.set_low();
        Timer::after_secs(1).await;
    }
}

static TRANSMIT_CHANNEL: Channel<ThreadModeRawMutex, Packet, 1> = Channel::new();
static GPS_CHANNEL: Channel<ThreadModeRawMutex, GGA, 1> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Set up USB serial logging and the LED blink tasks
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();
    let led = Output::new(AnyPin::from(p.PIN_13), Level::Low);
    spawner.spawn(blink_led(led)).unwrap();

    // Wait for a bit for everything to start up
    // NOTE: this along with the USB logging should be removed
    // before being used for real
    Timer::after_secs(2).await;

    // Set up the UART (GPS)
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 9600;
    let uart_rx = UartRx::new(p.UART1, p.PIN_5, Irqs, p.DMA_CH1, uart_config);
    let gps_receiver = GPS_CHANNEL.receiver();
    spawner.spawn(gps_reader(uart_rx, GPS_CHANNEL.sender())).unwrap();

    // Set up I2C stuff
    let sda = p.PIN_2;
    let scl = p.PIN_3;
    let i2c = i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, crate::i2c::Config::default());
    let i2c_bus: &'static _ = shared_bus::new_cortexm!(I2c<I2C1, i2c::Async> = i2c).unwrap();

    // Accelerometer
    let mut accel = Mma8x5x::new_mma8451(i2c_bus.acquire_i2c(), mma8x5x::SlaveAddr::default());
    let _ = accel.set_scale(mma8x5x::GScale::G8);
    let mut accel = match accel.into_active() {
        Ok(device) => Some(device),
        Err(_) => {
            error("Accelerometer is disabled");
            None
        },
    };

    // Temperature/Pressure
    let mut bmp = BMP388::new(i2c_bus.acquire_i2c()).unwrap();

    // Set up all the pins needed for the LoRa module
    // Documentation here: https://learn.adafruit.com/feather-rp2040-rfm95/pinouts
    // And here: https://github.com/mr-glt/sx127x_lora
    let miso    = p.PIN_8;
    let mosi    = p.PIN_15;
    let clk     = p.PIN_14;
    let rfm_cs  = AnyPin::from(p.PIN_16); // Chip Select
    let rfm_rst = AnyPin::from(p.PIN_17); // Reset

    // Set up the SPI interface
    let mut config = embassy_rp::spi::Config::default();
    config.frequency = 20_000;
    let spi = Spi::new_blocking(p.SPI1, clk, mosi, miso, config);

    // Set up Chip Select and Reset
    let cs = Output::new(rfm_cs, Level::Low);
    let reset = Output::new(rfm_rst, Level::Low);

    // Set up the LoRa transmitter
    let transmit_sender = TRANSMIT_CHANNEL.sender();
    spawner.spawn(transmitter(TRANSMIT_CHANNEL.receiver(), spi, cs, reset, 17)).unwrap();

    // Start the real time clock with a zeroed-out datetime
    // We could replace this with reading from the GPS unit
    let mut rtc = Rtc::new(p.RTC);
    let now = DateTime {
        year: 2000,
        month: 1,
        day: 1,
        day_of_week: embassy_rp::rtc::DayOfWeek::Saturday,
        hour: 0,
        minute: 0,
        second: 0,
    };
    match rtc.set_datetime(now) {
        Ok(_) => (),
        Err(_) => error("Failed to set datetime"),
    };

    // Stuff for the GPS
    let mut lat = 0;
    let mut lon = 0;
    let mut alt = 0;

    // A timer for keeping track of sub-second time
    let mut timer = Instant::now();
    let mut second = 0;

    loop {
        // Grab a new acceleration value
        let accel_val = match accel {
            Some(ref mut accel) => accel.read().unwrap_or_default(),
            None => Measurement::default(),
        };

        let mut realtime_now = rtc.now().unwrap(); // This unwrap is safe; it must be running
        if let Ok(gga) = gps_receiver.try_receive() { // Got new GPS data, update things which need updating
            let now = DateTime {
                year: 2000,
                month: 1,
                day: 1,
                day_of_week: embassy_rp::rtc::DayOfWeek::Saturday,
                hour: gga.time.hours,
                minute: gga.time.minutes,
                second: gga.time.seconds as u8,
            };
            match rtc.set_datetime(now) {
                Ok(_) => (),
                Err(_) => error("Failed to set datetime"),
            };
            timer = Instant::now();
            realtime_now = rtc.now().unwrap();

            // Set all the position data
            alt = gga.altitude.meters as i32;
            lat = (to_decimal(
                gga.latitude.degrees,
                gga.latitude.minutes,
                gga.latitude.seconds,
                gga.latitude.hemisphere
            ) * 1_000_000.0) as i32;
            lon = (to_decimal(
                gga.longitude.degrees,
                gga.longitude.minutes,
                gga.longitude.seconds,
                gga.longitude.hemisphere
            ) * 1_000_000.0) as i32;
        } else if second != realtime_now.second { // The second must have advanced, update the subsecond timer
            timer = Instant::now();
            second = realtime_now.second
        }

        let press_temp = bmp.sensor_values().unwrap();

        info!("{:?}", press_temp);

        // Build the packet
        let conditions = Packet {
            time: Time {
                hours: realtime_now.hour,
                minutes: realtime_now.minute,
                seconds: realtime_now.second,
                microseconds: (timer.elapsed().as_micros() % 1_000_000) as u32,
            },
            lat,
            lon,
            alt,
            temp: (press_temp.temperature - 273.15) as u16,
            pres: (press_temp.pressure * 10.0) as u16,
            accel_x: (accel_val.x * 10.0) as i16,
            accel_y: (accel_val.y * 10.0) as i16,
            accel_z: (accel_val.z * 10.0) as i16,
        };

        // Send the data via the second running task
        match transmit_sender.try_send(conditions) {
            Ok(_) => (),
            Err(_) => info!("Packet could not be sent to task")
        };

        Timer::after_millis(100).await;
    }
}

/// This function is for sending packets with the transmitter asynchronously
#[embassy_executor::task]
async fn transmitter(
    channel_rec: Receiver<'static, ThreadModeRawMutex, Packet, 1>,
    spi:    Spi<'static, SPI1, Blocking>,
    cs:     Output<'static, AnyPin>,
    reset:  Output<'static, AnyPin>,
    power:  i32,
) {
    // Initialize the LoRa module and then set the transmit power
    // 915 is the frequency (in MHz), 17 is the transmission power (in dB)
    let mut lora = match sx127x_lora::LoRa::new(spi, cs, reset, 915, Delay) {
        Ok(module) => module,
        Err(_) => {
            return
        },
    };

    let _ = lora.set_tx_power(power, 1);
    let _ = lora.set_crc(true);

    loop {
        let message = channel_rec.receive().await;

        // Create the bytes of the packet
        let packet = message.as_buffer();

        // Make sure it isn't already transmitting
        let mut transmit_status = lora.transmitting();
        let time = embassy_time::Instant::now();
        while transmit_status.is_ok_and(|x| x)
            && time.elapsed().as_millis() < 1000
        {
            Timer::after_millis(2).await;
            transmit_status = lora.transmitting();
        }

        if time.elapsed().as_millis() > 1000 {
            return
        }

        // Transmit the item
        let transmit = lora.transmit_payload(packet.0, packet.1 as usize);

        // Make sure it didn't fail
        if transmit.is_err() {
            info!("Err!")
        }
    }
}

#[embassy_executor::task]
async fn gps_reader(
    mut rx: UartRx<'static, UART1, Async>,
    channel_sender: Sender<'static, ThreadModeRawMutex, GGA, 1>,
) {
    info!("Begin Reading GPS Module...");
    let mut parser = Parser::new();
    loop {
        let mut buf = [0; 128];
        let _ = rx.read(&mut buf).await; // Wait for new data from the GPS module

        for result in parser.parse_from_bytes(&buf) {
            match result {
                Ok(ParseResult::GGA(Some(gga))) => {
                    let _ = channel_sender.try_send(gga);
                },
                Ok(_) => (),
                Err(_) => (),
            }
        }
    }
}

fn to_decimal(hours: u8, minutes: u8, seconds: f32, hemisphere: Hemisphere) -> f64 {
    let mut deg = hours as f64 + (minutes as f64 / 60.0) + (seconds as f64 / 3600.0);
    if hemisphere == Hemisphere::South
        || hemisphere == Hemisphere::West
    {
        deg = -deg;
    }

    deg
}

fn error(err: &str) {
    info!("ERR: {}", err);
}
