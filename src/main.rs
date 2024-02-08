#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{self, AnyPin},
    peripherals::USB,
    usb::{Driver, InterruptHandler},
};
use embassy_time::Timer;
use gpio::{Level, Output};
use log::info;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Set up USB logging
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    Timer::after_secs(1).await;

    let mut led = Output::new(AnyPin::from(p.PIN_13), Level::Low);

    loop {
        info!("Test");

        led.set_high();
        Timer::after_millis(100).await;

        led.set_low();
        Timer::after_millis(100).await;
    }
}
