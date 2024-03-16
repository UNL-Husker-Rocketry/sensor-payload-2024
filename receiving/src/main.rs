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
use embassy_sync::{channel::{Channel, Receiver, Sender}, blocking_mutex::raw::ThreadModeRawMutex};
use embassy_time::{Delay, Timer};
use embassy_usb::{msos::{windows_version, self}, Builder, types::InterfaceNumber, Config, control::{OutResponse, RequestType, Recipient, Request, InResponse}, Handler};
use gpio::{Level, Output};
use shared_types::Packet;
use {defmt_rtt as _, panic_probe as _};

// This is a randomly generated GUID to allow clients on Windows to find our device
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{0371DD6C-AEF4-4B59-9A74-82D5A220D226}"];

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

struct Req;
/// A channel to request data from the receiver
static REQUEST_CHANNEL: Channel<ThreadModeRawMutex, Req, 1> = Channel::new();

/// A channel to get the data returned from the receiver
static RECEIVE_CHANNEL: Channel<ThreadModeRawMutex, Option<Packet>, 1> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Set up USB serial logging and the LED blink
    let mut led = Output::new(AnyPin::from(p.PIN_13), Level::High);
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(usb_task(driver)).unwrap();

    // Wait for a bit for everything to start up, this
    // along with the serial logging should be removed
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

    // Actually initialize the LoRa module
    let mut lora =
        sx127x_lora::LoRa::new(spi, cs, reset, 915, Delay).expect("Could not initalize module!");

    let data_request = REQUEST_CHANNEL.receiver();
    let data_send = RECEIVE_CHANNEL.sender();

    loop {
        // Wait for a request to receive a new packet
        data_request.receive().await;
        led.toggle();

        // Actually receive the new packet
        let poll = lora.poll_irq(Some(20));

        match poll {
            Ok(_) => {
                let buffer = lora.read_packet().unwrap();
                let packet = match Packet::from_buffer(&buffer) {
                    Ok(packet) => packet,
                    Err(_) => {
                        let _ = data_send.try_send(None);
                        continue;
                    },
                };
                let _ = data_send.try_send(Some(packet));
            },
            Err(_) => {
                let _ = data_send.try_send(None);
            },
        }
    }
}

#[embassy_executor::task]
async fn usb_task(driver: Driver<'static, USB>) {
    // Create embassy-usb Config
    let mut config = Config::new(0x5e1f,  0x1e55);
    config.manufacturer = Some("UNL Aerospace");
    config.product = Some("LoRa Receiver");
    config.serial_number = Some("00000001");
    config.max_power = 500;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut handler = ControlHandler {
        if_num: InterfaceNumber(0),
        data_channel: RECEIVE_CHANNEL.receiver(),
        request_channel: REQUEST_CHANNEL.sender()
    };

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    // Add a vendor-specific function (class 0xFF), and corresponding interface,
    // that uses our custom handler.
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let _alt = interface.alt_setting(0xFF, 0, 0, None);
    handler.if_num = interface.interface_number();
    drop(function);
    builder.handler(&mut handler);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    usb.run().await;
}

struct ControlHandler<'a> {
    if_num: InterfaceNumber,
    data_channel: Receiver<'a, ThreadModeRawMutex, Option<Packet>, 1>,
    request_channel: Sender<'a, ThreadModeRawMutex, Req, 1>
}

impl Handler for ControlHandler<'_> {
    /// Respond to HostToDevice control messages, where the host sends us a command and
    /// optionally some data, and we can only acknowledge or reject it.
    fn control_out<'a>(&'a mut self, req: Request, _buf: &'a [u8]) -> Option<OutResponse> {
        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }

        // Ignore requests to other interfaces.
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        if req.request != 100 && req.value != 1 {
            return None;
        }

        if self.request_channel.try_send(Req).is_ok() {
            Some(OutResponse::Accepted)
        } else {
            Some(OutResponse::Rejected)
        }
    }

    /// Respond to DeviceToHost control messages, where the host requests some data from us.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None
        }

        // Ignore requests to other interfaces.
        if req.index != self.if_num.0 as u16 {
            return None
        }

        if req.request != 200 && req.value != 1 && req.length != 0x20 {
            return None
        }

        //...and try to get it
        let result = match self.data_channel.try_receive() {
            Ok(data) => data,
            Err(_) => return Some(InResponse::Rejected),
        };

        if result.is_none() {
            return Some(InResponse::Rejected)
        }

        let res_slice = result.unwrap().as_bytes();
        buf[..0x20].copy_from_slice(&res_slice[0..0x20]);

        if !result.unwrap().check_crc() {
            return Some(InResponse::Rejected)
        }

        Some(InResponse::Accepted(&buf[..0x20]))
    }
}
