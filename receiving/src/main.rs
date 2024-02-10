#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{self, AnyPin},
    peripherals::USB,
    spi::{self, Spi},
    usb::{Driver, InterruptHandler},
};
use embassy_time::{Delay, Timer};
use gpio::{Level, Output};
use log::info;
use {defmt_rtt as _, panic_probe as _};
use heapless::{String, Vec};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Set up USB serial logging and the LED blink
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();
    let led = Output::new(AnyPin::from(p.PIN_13), Level::Low);
    spawner.spawn(blink_led(led)).unwrap();

    // Wait for a bit for everything to start up, this
    // along with the serial logging should be removed
    // before being used for real
    Timer::after_secs(2).await;

    // Set up all the pins needed for the LoRa module
    // Documentation here: https://learn.adafruit.com/feather-rp2040-rfm95/pinouts
    // And here: https://github.com/mr-glt/sx127x_lora
    let miso = p.PIN_8;
    let mosi = p.PIN_15;
    let clk = p.PIN_14;
    let rfm_cs = p.PIN_16; // Chip Select
    let rfm_rst = p.PIN_17; // Reset

    // Set up the SPI interface
    let mut config = spi::Config::default();
    config.frequency = 20_000;
    let spi = Spi::new_blocking(p.SPI1, clk, mosi, miso, config);

    // Set up Chip Select and Reset
    let cs = Output::new(rfm_cs, Level::Low);
    let reset = Output::new(rfm_rst, Level::Low);

    // Actually initialize the LoRa module and then set the transmit power
    // 915 is the frequency (in MHz), 5 is the power (in dB)
    let mut lora =
        sx127x_lora::LoRa::new(spi, cs, reset, 915, Delay).expect("Could not initalize module!");
    lora.set_tx_power(5, 1).expect("Could not set power");

    loop {
        let poll = lora.poll_irq(Some(20));

        match poll {
            Ok(size) =>{
                info!("with Payload: ");
                let buffer = lora.read_packet().unwrap();
                let mut result: Vec<u8, 255> = Vec::new();
                result.extend_from_slice(&buffer[0..size]).unwrap();
                let string: String<255> = String::from_utf8(result).unwrap();
                info!("{:?}", string);
            },
            Err(_) => info!("Timeout"),
        }

        Timer::after_millis(1000).await;
    }
}
