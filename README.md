# Environment-Monitoring-Sensor-System
This project is a real-time environmental monitoring sensor system and data logger built with an Arduino Due, capable of measuring temperature, humidity, pressure, and light intensity using various sensors.
The system is programmed through the use of an SDI-12 line on Serial1

## Collaborators
- [Jayden Kong](https://github.com/jjv7)
- [Nok Soen Enoch Chan](https://github.com/Hex-Itsuka)
- [Cao Minh Vu](https://github.com/Aragami1408)
- Dilan Augustin
- Aryan Sivakumar

## Components
- Arduino Due
- Fermion: 1.8" 128x160 IPS TFT LCD Display with MicroSD Card Slot (SPI)
- DS1307 - Real Time Clock Module (I2C)
- BME280 - Temperature, humidity, and barometric pressure sensor (I2C)
- BH1750FVI - Digital Light intensity Sensor (I2C)

## Libraries
This project utilises the following libraries:
- [DueTimer (1.4.7) by IvanSeidel](https://github.com/ivanseidel/DueTimer)
- [Adafruit BME280 Library (2.2.4) by Adafruit](https://github.com/adafruit/Adafruit_BME280_Library)
- [BH1750 (1.3.0) by Christopher Laws](https://github.com/claws/BH1750)
- [Adafruit ImageReader Library (2.8.1) by Adafruit](https://github.com/adafruit/Adafruit_ImageReader)
- [RTCLib (2.1.1) by Adafruit](https://github.com/adafruit/RTClib)
- [SdFat (2.2.2) by Bill Greiman](https://github.com/greiman/SdFat)

## Installation
1. Clone the repository

2. Install the required libraries:
   You can install the required libraries through the Arduino Library Manager:
    - Open the Arduino IDE.
    - Go to Sketch > Include Library > Manage Libraries.
    - Search for each of the following libraries and install them:
      - DueTimer by Ivan Seidel
      - Adafruit BME280 Library by Adafruit
      - BH1750 by Christopher Laws
      - Adafruit ImageReader Library by Adafruit
      - RTCLib by Adafruit
      - SdFat by Bill Greiman

3. Connect the components:
   Ensure all the sensors and the display are properly connected to the Arduino Due as per the pin configuration specified in the code.

4. Upload the code to your Arduino Due:
   - Open the Arduino IDE.
   - Load the 'Project_Final.ino' file.
   - Select Tools > Board > Arduino Due (Programming Port).
   - Select the appropriate port under Tools > Port.
   - Click on the Upload button.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
