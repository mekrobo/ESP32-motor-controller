# ESP32 TB9054 Motor Controller
This library is for custom dual channel, H-bridge, brushed DC motor controller with encoder quadrature decoder, current sensing and onboard ESP32 micro-controller for control, automation and robotics application. It can also support external H-bridges drivers connected with ESP32 devkit with/without encoder. It may required certain modification.

## Application
Robotics application such as ground robots, self-balancing robot and automation application etc.

## Specification
- **Motor Driver :** Dual channel, H-bridge.
- **Control Input :** PWM with frequency from $1-10\ khz$.
- **Operating Voltage :** $\mathrm{V_{BAT}}\ =\ 4.5\ \text{to}\ 28\ V$, $\mathrm{V_{CC}}\ =\ 4.5\ \text{to}\ 5.5\ V$, $\mathrm{V_{DDIO}}\ =\ 3.0\ \text{to}\ 5.5\ V$.
- **Operating Temperature :** $Ta\ =\ -40\ \text{to}\ 125^o\mathrm{C}$
- **Protection :** Stop output for supply under voltage, over current, over temperature.
- **Sensing :** High voltage side output current monitoring.
- **Encoder Decoder :** Onboard quadrature decoder of dual channel digital encoder for motor rpm measurement with onboard power supply to encoder sensor.
- **Interfacing :** USB serial interface of micro-controller with computer for two-way communication firmware upload, control, communication and sensors data streaming.
- **Control :** Onboard PID control of motor RPM with onboard speed and current sensing.

# Getting Started
## Installation
### Arduin ESP32 Setup

Install Arduino-ESP32 support following the step by step instruction in any of the follwing links.
- **ESPRESSIF Official :** https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
- **Unofficial link:** https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/

### Library
Download this repository and convert it to .zip file. In Arduino IDE import this library. Steps to import $${\color{cyan} Sketch >Include\ Library > Add\ .ZIP\ Library\cdots}$$ and choose the downloaded .zip file. You can see the confirmation if the library import is successful.

<!-- <img src="figure\LibraryImport.png" alt="Library Import" width="450"> -->

### Dependency
Then install the dependency library ESP32Encoder from Arduino IDE (Version 0.10.2 has been tested). Steps to install $${\color{cyan} Sketch >Include\ Library >Manage\ Libraries\cdots}$$

<!-- <img src="figure\ESP32Encoder.png" alt="Library Import" width="360"> -->

### Examples
After successful installation of the libraries and dependencies. The examples are available from the Menu. $${\color{cyan} File >Examples > ESP32\ MotorController}$$

<img src="figure\Examples.png" alt="Library Import" width="360">

Four examples are available:

- **PWM Sweep for Single Channel:** [PWMChannel1.ino](examples/PWMChannel1/PWMChannel1.ino)
- **PWM Sweep for Dual Channel:** [PWM.ino](examples\PWM\PWM.ino)
- **RPM Sweep for Single Channel:** [RPMControlChannel1.ino](examples/RPMControlChannel1/RPMControlChannel1.ino)
- **RPM Sweep for Dual Channel:** [RPMControl.ino](examples/RPMControl/RPMControl.ino) 

## Connection
[Connection Diagram](doc/ConnectionDiagram.md)
## Hardware Support
- Custom Controller using ESP32.
- ESP32 devkit with separate H bridge motor driver.

# References
[1]  Toshiba [TB9054FTG](https://toshiba.semicon-storage.com/ap-en/semiconductor/product/automotive-devices/detail.TB9054FTG.html) dual channel H-bridge Integrated Circuit (IC) controlls the motor.

[2] The motor encoder decoding has been implemented using [ESP32Encoder](https://github.com/madhephaestus/ESP32Encoder) Arduino based library that uses the ESP32 pulse counter hardware peripheral.

[https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html)

# Owner
[MekRobo Technology](https://mekrobo.in/)

See license in license.txt
