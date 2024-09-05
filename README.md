
# I2S, I2C, and CAN Bus Integration for BMW E90/E87

This project is an **attempt to hack the BMW CD73 professional radio** found in E90 and E87 models to accept **I2S audio input**, as well as interface via **I2C** and **CAN bus** protocols. The goal is to extend the radio's audio and control capabilities.

## Features
- **I2S Audio Input**: Inject high-quality digital audio into the BMW CD73 radio system.
- **I2C Communication**: Control and interact with peripheral devices using the I2C protocol.
- **CAN Bus Integration**: Communicate with the car's CAN bus network to enable media controls and other functionalities.
- **Embedded System Development**: Developed using **PlatformIO** and C++ for cross-platform compatibility and advanced functionality.

## Tech Stack
- **C++**: Programming language for implementing protocol communication and controls.
- **PlatformIO**: Build system for embedded development.
- **I2S, I2C, CAN bus**: Protocols used to interface with the hardware components and BMW CD73 radio.
- **BMW CD73 Professional Radio**: Factory radio unit in BMW E90 and E87 models.

## Installation
1. Clone the repository:
   \`\`\`bash
   git clone https://github.com/llilakoblock/i2s-i2c-canbus-bmw-e90-e87.git
   \`\`\`
2. Open the project in **PlatformIO** or another supported development environment.
3. Connect the hardware interfaces (I2S, I2C, CAN bus) to the BMW CD73 radio system.
4. Build and upload the firmware using PlatformIO:
   \`\`\`bash
   platformio run --target upload
   \`\`\`

## Hardware Requirements
- BMW CD73 professional radio unit.
- Microcontroller with I2C and I2S support.
- CAN bus interface module for communication with the car's network.

## Usage
1. Flash the firmware to the microcontroller.
2. Inject digital audio signals into the CD73 radio using I2S.
3. Control peripherals and interface with external devices using I2C.
4. Use CAN bus for additional vehicle functionalities like media control or diagnostics.

## Future Enhancements
- **Audio Processing**: Support for more advanced audio processing.
- **Vehicle Diagnostics**: Integrate additional CAN bus functionalities for car diagnostics.
- **Extended Controls**: Further integration with other vehicle systems, such as HVAC or lighting controls.

## Contributing
Contributions are welcome! If you'd like to contribute, feel free to fork the repository, submit pull requests, or raise issues. Contributions can include code improvements, new features, bug fixes, or enhanced documentation.

## License
This project is licensed under the **MIT License**.
