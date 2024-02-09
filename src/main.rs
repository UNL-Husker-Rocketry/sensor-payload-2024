#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{self, AnyPin},
    peripherals::{USB, SPI1},
    spi::{self, Spi, Blocking},
    usb::{Driver, InterruptHandler},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver},
};
use embassy_time::{Delay, Timer};
use gpio::{Level, Output};
use log::{info, warn};
use {defmt_rtt as _, panic_probe as _};

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

#[embassy_executor::task]
async fn transmitter(
    channel_rec: Receiver<'static, ThreadModeRawMutex, (u8, [u8; 255]), 1>,

    spi:    Spi<'static, SPI1, Blocking>,
    cs:     Output<'static, AnyPin>,
    reset:  Output<'static, AnyPin>,
) {
    // Actually initialize the LoRa module and then set the transmit power
    // 915 is the frequency (in MHz), 5 is the power (in dB)
    let mut lora =
        sx127x_lora::LoRa::new(spi, cs, reset, 915, Delay).expect("Could not initalize module!");
    lora.set_tx_power(17, 1).expect("Could not set power");

    loop {
        let message = channel_rec.receive().await;

        // Make sure it isn't already transmitting
        while lora.transmitting().unwrap() {
            Timer::after_millis(1).await;
        }

        // Transmit something
        let transmit = lora.transmit_payload(message.1, message.0 as usize);

        // Make sure it didn't fail
        match transmit {
            Ok(_) => (),
            Err(_) => warn!("Transmission failed."),
        }
    }
}

static CHANNEL: Channel<ThreadModeRawMutex, (u8, [u8; 255]), 1> = Channel::new();

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
    let rfm_cs = AnyPin::from(p.PIN_16); // Chip Select
    let rfm_rst = AnyPin::from(p.PIN_17); // Reset

    // Set up the SPI interface
    let mut config = spi::Config::default();
    config.frequency = 20_000;
    let spi = Spi::new_blocking(p.SPI1, clk, mosi, miso, config);

    // Set up Chip Select and Reset
    let cs = Output::new(rfm_cs, Level::Low);
    let reset = Output::new(rfm_rst, Level::Low);

    // Set up the LoRa transmitter in another thread
    let channel_sender = CHANNEL.sender();
    spawner.spawn(transmitter(CHANNEL.receiver(), spi, cs, reset)).unwrap();

    // This is the general scheme to stick a message in a buffer
    let message = "Hello, world!";
    let mut buffer = [0; 255];
    for (i, c) in message.chars().enumerate() {
        buffer[i] = c as u8
    }

    loop {
        info!("Sending...");
        channel_sender.send((255, [0; 255])).await;
        info!("Sent!");
        Timer::after_secs(1).await;
    }
}
