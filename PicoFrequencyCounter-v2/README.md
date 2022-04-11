This method can be used to calibrate Si5351 in a cost-effective manner.

- 'Install' MicroPython on Pi Pico.

  Push and hold the BOOTSEL button and plug your Pico into the USB port of your
  Raspberry Pi or other computer. Release the BOOTSEL button after your Pico is
  connected. It will mount as a Mass Storage Device called RPI-RP2. Drag and drop
  the MicroPython UF2 file onto the RPI-RP2 volume.

  (Text borrowed from [this URL](https://www.raspberrypi.com/documentation/microcontrollers/micropython.html))

- [Optional] Install `ampy`

  ```
  pip3 install adafruit-ampy
  ```

  Run the frequency counting program.

  ```
  ampy --port /serial/port run main.py
  ```

- Alternate: Use `Thonny IDE` to upload `main.py` to Pi Pico.


Connections:

Connect the input-frequency to Pin 20 (GP15). Connect the input-frequency GND
to any of the GND pins of Pico (e.g. Pin 18).

This `v2` version also optionally configures a GPS module (>= NEO-6M) connected
to the Pico board - see https://github.com/kholia/uBlox7_TimePulse for more
details.
