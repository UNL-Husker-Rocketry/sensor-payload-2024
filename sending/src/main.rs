#![no_std]
#![no_main]

// Main board control stuff
use core::cell::RefCell;
use embassy_embedded_hal::{
    adapter::BlockingAsync,
    shared_bus::blocking::i2c::I2cDevice
};
use embassy_sync::{
    channel::{Channel, Receiver, Sender},
    blocking_mutex::{raw::ThreadModeRawMutex, NoopMutex},
};
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{AnyPin, Level, Output},
    i2c,
    peripherals::{I2C1, SPI1, UART0, UART1, USB},
    rtc::{DateTime, Rtc},
    spi::{Blocking, Spi},
    uart,
    usb::{self, Driver},
    watchdog::*
};
use embassy_time::{with_timeout, Delay, Duration, Timer};

// Packet and time structs
use shared_types::{Packet, Time};

// Error handling and printing
use log::info;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// Sensor data processing
use bmp388::{Oversampling, BMP388};
use nmea0183::{
    coords::Hemisphere,
    ParseResult, Parser, GGA
};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART1_IRQ => uart::InterruptHandler<UART1>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    dev_init("USB logging");
    embassy_usb_logger::run!(8192, log::LevelFilter::Info, driver);
}

/// Set up a task to blink the LED
#[embassy_executor::task]
async fn blink_led(mut led: Output<'static, AnyPin>) {
    dev_init("Debug blinker");
    loop {
        led.toggle();
        Timer::after_secs(1).await;
    }
}

static I2C_BUS: StaticCell<NoopMutex<RefCell<i2c::I2c<'_, I2C1, i2c::Blocking>>>> = StaticCell::new();
static TRANSMIT_CHANNEL: Channel<ThreadModeRawMutex, Packet, 1> = Channel::new();
static GPS_CHANNEL: Channel<ThreadModeRawMutex, GGA, 1> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut watchdog = Watchdog::new(p.WATCHDOG);

    // Set up USB serial logging and the LED blink tasks
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();
    let led = Output::new(AnyPin::from(p.PIN_13), Level::Low);
    spawner.spawn(blink_led(led)).unwrap();

    //Timer::after_millis(2000).await; // Add this delay back in if you want serial status
    info!("\x1b[0;35mSensor Payload Firmware v0.2.1 start!\x1b[0m");

    // Set up the UART (GPS)
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 9600;
    let uart_tx = uart::UartTx::new(p.UART0, p.PIN_0, p.DMA_CH0, uart_config);
    let uart_rx = uart::UartRx::new(p.UART1, p.PIN_5, Irqs, p.DMA_CH1, uart_config);
    let gps_receiver = GPS_CHANNEL.receiver();
    spawner.spawn(gps_reader(uart_rx, uart_tx, GPS_CHANNEL.sender())).unwrap();

    // Set up all the pins needed for the LoRa module SPI
    // Documentation here: https://learn.adafruit.com/feather-rp2040-rfm95/pinouts
    // And here: https://github.com/mr-glt/sx127x_lora
    let miso    = p.PIN_8;
    let mosi    = p.PIN_15;
    let clk     = p.PIN_14;
    let rfm_cs  = AnyPin::from(p.PIN_16); // Chip Select
    let rfm_rst = AnyPin::from(p.PIN_17); // Reset

    // Set up the SPI interface
    let mut spi_config = embassy_rp::spi::Config::default();
    spi_config.frequency = 20_000;
    let spi = Spi::new_blocking(p.SPI1, clk, mosi, miso, spi_config);
    let cs = Output::new(rfm_cs, Level::Low);
    let reset = Output::new(rfm_rst, Level::Low);

    // Set up the LoRa transmitter
    let transmit_sender = TRANSMIT_CHANNEL.sender();
    spawner.spawn(transmitter(TRANSMIT_CHANNEL.receiver(), spi, cs, reset, 20)).unwrap();

    // Set up I2C stuff
    let sda = p.PIN_2;
    let scl = p.PIN_3;
    let i2c_main = i2c::I2c::new_blocking(p.I2C1, scl, sda, i2c::Config::default());
    let i2c_bus = NoopMutex::new(RefCell::new(i2c_main));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    // Temperature/Pressure
    let mut bmp = match BMP388::new(
        BlockingAsync::new(I2cDevice::new(i2c_bus)),
        bmp388::Addr::Secondary as u8,
        &mut Delay
    ).await {
        Ok(device) => Some(device),
        Err(err) => {
            info!("{:?}", err);
            error("Temperature and Pressure (BMP388) are disabled");
            None
        },
    };
    if let Some(ref mut dev) = bmp {
        let _ = dev.set_power_control(bmp388::PowerControl {
            pressure_enable: true,
            temperature_enable: true,
            mode: bmp388::PowerMode::Normal
        }).await;
        let _ = dev.set_sampling_rate(bmp388::SamplingRate::ms160).await;
        let _ = dev.set_oversampling(bmp388::config::OversamplingConfig {
            osr_pressure: Oversampling::x32,
            osr_temperature: Oversampling::x2
        }).await;
        let _ = dev.set_filter(bmp388::Filter::c3);
        for _ in 0..10 {
            let _ = dev.sensor_values().await;
        }
        dev_init("Temp/Pressure (BMP388)");
    }

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
    let mut counter = 0;
    let mut second = 0;

    begin("Main Loop");
    watchdog.start(Duration::from_millis(1_000));
    loop {
        // Get the new temperature data
        let pres_temp_val = match bmp {
            Some(ref mut dev) => match dev.sensor_values().await {
                Ok(val) => val,
                Err(_) => {
                    error("Failed to read BMP388!");
                    bmp388::SensorData {
                        pressure: 0.0,
                        temperature: 0.0,
                    }
                },
            },
            None => bmp388::SensorData {
                pressure: 0.0,
                temperature: 0.0,
            },
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
        }

        // The second must have advanced, update the subsecond timer
        counter += 1;
        if second != realtime_now.second {
            counter = 0;
            second = realtime_now.second
        }

        // Build the packet
        let mut conditions = Packet {
            time: Time {
                hours: realtime_now.hour,
                minutes: realtime_now.minute,
                seconds: realtime_now.second,
                counter,
            },
            lat,
            lon,
            alt,
            temp: (pres_temp_val.temperature * 10.0) as i16,
            pres: (pres_temp_val.pressure * 100.0) as u32,
            crc: 0,
        };

        conditions.set_crc();
        match transmit_sender.try_send(conditions) {
            Ok(_) => (),
            Err(_) => error("Transmission failed."),
        };

        watchdog.feed();
        Timer::after_millis(200).await;
    }
}

#[embassy_executor::task]
async fn gps_reader(
    mut rx: uart::UartRx<'static, UART1, uart::Async>,
    mut tx: uart::UartTx<'static, UART0, uart::Async>,
    channel_sender: Sender<'static, ThreadModeRawMutex, GGA, 1>,
) {
    let mut buf = [0; 128];
    let _ = tx.write(b"$PMTK220,200*2C\r\n").await;
    let _ = tx.write(b"$PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n").await;

    dev_init("GPS (MTK3339)");
    let mut parser = Parser::new().sentence_only(nmea0183::Sentence::GGA);
    begin("GPS Loop");
    loop {
        // Wait for new data from the GPS module
        match with_timeout(Duration::from_secs(5), rx.read(&mut buf)).await {
            Ok(_) => (),
            Err(_) => error("GPS timed out"),
        };

        for result in parser.parse_from_bytes(&buf) {
            match result {
                Ok(ParseResult::GGA(Some(gga))) => {
                    let _ = channel_sender.try_send(gga);
                },
                Ok(_) => (),
                Err(_) => warn("GPS data could not be parsed"),
            }
        }
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
    // 915 is the frequency (in MHz)
    let mut lora = match sx127x_lora::LoRa::new(spi, cs, reset, 915, Delay) {
        Ok(module) => module,
        Err(_) => return,
    };
    dev_init("LoRa (Sx127x)");

    let _ = lora.set_tx_power(power, 0);
    let _ = lora.set_ocp(240);

    begin("Transmission Loop");
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
            Timer::after_millis(20).await;
            transmit_status = lora.transmitting();
        }

        // Transmit the item
        let transmit = lora.transmit_payload(packet.0, packet.1 as usize);

        // Make sure it didn't fail
        if transmit.is_err() {
            error("Failed to transmit")
        }
    }
}

/// Convert hr/min/sec to decimal format
fn to_decimal(hours: u8, minutes: u8, seconds: f32, hemisphere: Hemisphere) -> f64 {
    let mut deg = hours as f64 + (minutes as f64 / 60.0) + (seconds as f64 / 3600.0);
    if hemisphere == Hemisphere::South
        || hemisphere == Hemisphere::West
    {
        deg = -deg;
    }

    deg
}

#[inline]
fn info(info: &str) {
    info!("INFO: {}", info);
}

#[inline]
fn error(err: &str) {
    info!("\x1b[0;31mERR:\x1b[0m {}", err);
}

#[inline]
fn warn(warn: &str) {
    info!("\x1b[0;33mWARN:\x1b[0m {}", warn);
}

#[inline]
fn dev_init(dev: &str) {
    info!("\x1b[0;92mINIT:\x1b[0m {}", dev);
}

#[inline]
fn begin(thing: &str) {
    info!("\x1b[0;34mBEGIN:\x1b[0m {}", thing);
}
