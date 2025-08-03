# Motion Capture Finger Tracking

Bluetooth code for tracking finger bend using flex sensors on arduino-compatible hardware. The current output format supports Mocap Fusion serial communication, and placeholder code for the OpenGlove driver. Uses the Adafruit bluefruit library.

The dev board used is the base model Seeed Studio XIAO nRF52840: https://wiki.seeedstudio.com/XIAO_BLE/

## Hardware

- 3x XIAO nRF52840
  - One for each hand, wirelessly transmitting over BLE (peripherals)
  - One connected to the PC, receiving the data from the peripherals and writing the payload to serial (central)

For each hand:

- 1x XIAO nRF52840 (noted above)
- 5x flex sensors (either 55mm SEN-27315, or 95mm SEN-27314)
- 1x 3.7V 1100mAH li-ion battery
- 5x resistors (see notes for rating)

## Notes

The flex sensors datasheet lists the flat resistance as 12kOhm (for 55mm) with a 4x increase in resistance at maximum bend. At 3.3V, this can be paired with a 16kOhm resistor to give the largest change in voltage in a voltage divider circuit.

The XIAO board has a 12-bit ADC so it can measure 0-4095 on the analog pins. These 12-bit values are then bit-packed for all fingers into a byte array to send via a BLE characteristic representing the left or right hand. The size of five 12-bit values bit-packed into 1 byte values is 8 bytes. It appears to work fine transmitting at 120fps (8ms/packet).

The central board filters connections to a known custom 16-bit bluetooth service UUID that the hands are advertising. Upon connection, the central detects a custom left hand UUID characteristic, and/or a custom right hand UUID characteristic. This allows a single transmitter to send both left and right hand data, or two separate transmitters to send data for each hand. Currently, the peripheral code is written to handle a single hand only. The XIAO boards only have 6 analog pins, so having code to send all 10 analog channels would require a board that has >= 10 analog pins, or an additional ADC connected to multiplex an existing analog pin. The latter would work for the XIAO board as the digital pins are free to use with a another ADC, but is currently beyond the needs for the project.

The XIAO board has a built-in li-ion battery management system, so it can charge the connected li-ion battery via it's USB-C port. When not charging, the battery then powers the board for wireless tracking.
