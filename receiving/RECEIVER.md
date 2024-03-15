# Receiver Information
The receiver appears as a USB device with the Vendor ID `0x5e1f` and the Product ID `0x1e55`.

It exposes a single Interface, `0`, and has 2 control transfer commands:
 1. Control transfer out, request type Vendor, recipient Interface, request 100, value 1
    - Asks the device to receive a new packet from the rocket
 2. Control transfer in, request type Vendor, recipient Interface, request 200, value 1
    - Asks for the data received from the last request. #1 must be requested before #2.
