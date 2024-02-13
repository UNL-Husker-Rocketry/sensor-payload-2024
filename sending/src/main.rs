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
use log::info;
use {defmt_rtt as _, panic_probe as _};

use shared_types::{Packet, Time, Accel};

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

/// This function is for sending packets with the transmitter asynchronously
#[embassy_executor::task]
async fn transmitter(
    channel_rec: Receiver<'static, ThreadModeRawMutex, Packet, 1>,

    spi:    Spi<'static, SPI1, Blocking>,
    cs:     Output<'static, AnyPin>,
    reset:  Output<'static, AnyPin>,
) {
    // Actually initialize the LoRa module and then set the transmit power
    // 915 is the frequency (in MHz), 17 is the transmission power (in dB)
    let mut lora = match sx127x_lora::LoRa::new(spi, cs, reset, 915, Delay) {
        Ok(module) => module,
        Err(_) => {
            return
        },
    };

    match lora.set_tx_power(17, 1) {
        Ok(_) => (),
        Err(_) => {
            return
        },
    };

    loop {
        let message = channel_rec.receive().await;

        // Create the bytes of the packet
        let packet = message.to_buffer();

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
        match transmit {
            Ok(_) => (),
            Err(_) => (),
        }
    }
}

static CHANNEL: Channel<ThreadModeRawMutex, Packet, 1> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Set up USB serial logging and the LED blink tasks
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();
    let led = Output::new(AnyPin::from(p.PIN_25), Level::Low);
    spawner.spawn(blink_led(led)).unwrap();

    // Wait for a bit for everything to start up
    // NOTE: this along with the serial logging should be removed
    // before being used for real
    Timer::after_secs(2).await;

    // Set up all the pins needed for the LoRa module
    // Documentation here: https://learn.adafruit.com/feather-rp2040-rfm95/pinouts
    // And here: https://github.com/mr-glt/sx127x_lora
    let miso    = p.PIN_8;
    let mosi    = p.PIN_15;
    let clk     = p.PIN_14;
    let rfm_cs  = AnyPin::from(p.PIN_16); // Chip Select
    let rfm_rst = AnyPin::from(p.PIN_17); // Reset

    // Set up the SPI interface
    let mut config = spi::Config::default();
    config.frequency = 20_000;
    let spi = Spi::new_blocking(p.SPI1, clk, mosi, miso, config);

    // Set up Chip Select and Reset
    let cs = Output::new(rfm_cs, Level::Low);
    let reset = Output::new(rfm_rst, Level::Low);

    // Set up the LoRa transmitter
    let transmit_sender = CHANNEL.sender();
    spawner.spawn(transmitter(CHANNEL.receiver(), spi, cs, reset)).unwrap();

    loop {
        // An example packet, in reality it will be compiled from incoming sensor data
        let conditions = Packet {
            time: Time {    // 10:13:23.132831
                hours: 10,
                minutes: 13,
                seconds: 23,
                microseconds: 132831,
            },
            lat: 40806862,  //  40.806862 degrees
            lon: -96681679, // -96.681679 degrees
            alt: 2209,      // 2,209 meters
            temp: 10,       // 10 degrees C
            pres: 7949,     // 794.9 millibars
            accel: Accel {
                x: 0,       // 0g
                y: 2,       // 0.2g
                z: 13,      // 1.3g
            },
        };

        info!("{:?}", conditions);

        // Send the data via the second running task
        match transmit_sender.try_send(conditions) {
            Ok(_) => (),
            Err(_) => info!("Packet could not be sent to task")
        };

        Timer::after_secs(1).await;
    }
}
