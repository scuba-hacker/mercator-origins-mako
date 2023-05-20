# mercator-origins-mako

Mako is the guidance and nav beating heart of Mercator Origins. He is the core of this project and proudly enables all the goodness to happen!

Mako receives GPS ASCII location messages from Lemon (running in the float) and presents navigation and guidance information to the Diver. He also uploads a binary-encoded telemetry message to Lemon after every location message is received (currently a 500ms duty cycle). Mako also currently has hardcoded lat/long location info for a small selection of features of interest at Wraysbury Dive Lake, near Staines. UK

Mako carries the following sensors as of May 2023:

* I2C Tilt-compensated Digital Compass - comprising a 3D magnetometer and a 6 degrees of freedom accelerometer.
* I2C Temperature, Humidity and Air Pressure Sensor - measures the internal environment of the Diver Console.
* I2C RGB Colour, LUX (light intensity) and Gesture Sensor - goodness that is yet to be enabled.
* Two GoPro external physical buttons that are (with hacker custom built button mounts) mapped down to two GPIO pin on Mako.
* See the M5 Stick C Plus spec and the ESP32 spec for all the extra sensors that these two devices provide as part of their package. (including 3D gyroscope for absolute attitude determination.)

Mako also does some trickery to enable an extra output on the M5 Stick C Plus which has only 3 GPIO pins through its HAT header pins. The Infra-Red transmitter (part of the M5 Stick) has a phototransistor placed in front of it to enable a Transmission UART serial stream to carry the telemetry data back up to Lemon. True hacking, which I love!

