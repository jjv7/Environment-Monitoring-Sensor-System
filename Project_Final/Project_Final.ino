#include <sam.h>
#include <DueTimer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <RTClib.h>
#include <SdFat.h>

// TFT display setup
#define TFT_CS 10
#define TFT_RST 6
#define TFT_DC 7
#define TFT_SCLK 13
#define TFT_MOSI 11

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

//BME280 Setup
Adafruit_BME280 bme;
#define SeaLevel_HPA (1013.25)

//BH1750 Setup
BH1750 lightMeter(0x23);

//SDI-12 Setup
#define DIRO 8

// RTC setup
RTC_DS1307 rtc;

DateTime now;
char timeLog[9];

// SD card setup
SdFs sd;
FsFile file;

const uint8_t SD_CS_PIN = A3;
const uint8_t SOFT_MISO_PIN = 12;
const uint8_t SOFT_MOSI_PIN = 11;
const uint8_t SOFT_SCK_PIN = 13;

SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)

#define WDT_KEY (0xA5)

void watchdogSetup(void) {
  WDT->WDT_MR = WDT_MR_WDD(0xFFF) |   // Set WDD to max value (4096)
                WDT_MR_WDFIEN |       // Enables watchdog interrupt
                WDT_MR_WDV(256 * 7);  // Set interrupt to trigger after ~7s
  
  NVIC_SetPriority(WDT_IRQn, 0);      // Set watchdog interrupt as highest priority
  NVIC_EnableIRQ(WDT_IRQn);           // Enable interrupt request for watchdog in NVIC
}

// Global variables setup
String command;
int deviceAddress = 0;
String deviceIdentification = "a14ENG20009104547242";

volatile float temp, pressure, humidity, lux = 0.0;

int scalersTemp[] = { 0, 21, 42, 64, 85 };
int scalersPressure[] = { 0, 275, 550, 825, 1100 };
int scalersHumidity[] = { 0, 25, 50, 75, 100 };
int scalersLux[] = { 0, 14, 27, 41, 55 };

const int PB1 = 2;
const int PB2 = 3;
const int PB3 = 4;
const int PB4 = 5;

volatile int menuSelect = 0;
volatile bool runningContinuous = false;
volatile bool takeContMeasurement = false;
volatile bool refreshDisplay = false;
volatile bool processCommand = false;

volatile int sensorsReady = 0;
volatile int samplingDuration = 0;
volatile int samplingPeriod = 10;
volatile int samplesTaken = 0;
volatile float sumTemp, sumPressure, sumHumidity, sumLux = 0.0;
volatile float avgTemp, avgPressure, avgHumidity, avgLux = 0.0;

char errorMessage[70] = " ERROR: SD CARD NOT FOUND\n\n CHECK SD\n\n USER MUST RESET SYSTEM";
bool errorLocatedInSetup = true;


enum MenuScreen {
  MAINMENU,
  BME,
  BH
};

enum MeasureSelect {
  NONE,
  ALL,
  BME280,
  BH1750,
  TEMP,
  PRESSURE,
  HUMIDITY
};

int selectContMeasure = NONE;

void setup() {
  //Arduino IDE Serial Monitor
  Serial.begin(9600);

  // ================= TFT ==================
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  // ================== SD ==================
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("SD card initialization failed!");
    sd.initErrorHalt();
    while (1)
      ;
  }

  // Open/create a file for writing
  if (!file.open("SensorData.txt", O_RDWR | O_CREAT)) {
    sd.errorHalt(F("open failed"));
  }
  file.open("SensorData.txt", O_RDWR);
  file.seek(0);
  file.truncate();
  file.close();

  // ================ BME280 ================
  strcpy(errorMessage, " ERROR: BME280 NOT FOUND\n\n CHECK WIRING\n\n USER MUST RESET SYSTEM");

  bme.begin();
  if (!bme.begin()) {
    Serial.println("BME280 NOT CONNECTED");
    while (1)
      ;
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED);

  // ================ BH1750 ================
  strcpy(errorMessage, " ERROR: BH1750 NOT FOUND\n\n CHECK WIRING\n\n USER MUST RESET SYSTEM");

  Wire.begin();
  lightMeter.begin();
  if (!lightMeter.begin()) {
    Serial.println("BH1750 NOT CONNECTED");
    while (1)
      ;
  }

  // ================= RTC ==================
  strcpy(errorMessage, " ERROR: RTC NOT FOUND\n\n CHECK WIRING\n\n USER MUST RESET SYSTEM");

  rtc.begin();
  rtc.adjust(DateTime(2024, 5, 7, 0, 0, 0));
  if (!rtc.begin()) {
    Serial.println("RTC NOT CONNECTED");
    while (1)
      ;
  }

  

  // =============== buttons ================
  pinMode(PB1, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP);
  pinMode(PB4, INPUT_PULLUP);

  attachInterrupt(PB1, BMEmenu, RISING);
  attachInterrupt(PB2, BHmenu, RISING);
  attachInterrupt(PB3, ExitToMainMenu, RISING);
  attachInterrupt(PB4, StopCont, RISING);

  menuSelect = 0;  // Interrupt triggers once, this is to reset the value

  // =========== timer interrupts ===========
  Timer3.attachInterrupt(TriggerContMeasurement);
  Timer4.attachInterrupt(DoSumMeasurement);
  Timer5.attachInterrupt(ReceiveCommand);
  Timer5.setFrequency(10).start();  // Program will check for bytes received over SDI-12, 10 times a second

  // ================ SDI-12 ================
  Serial1.begin(1200, SERIAL_7E1);  //SDI-12 UART, configures serial port for 7 data bits, even parity, and 1 stop bit
  pinMode(DIRO, OUTPUT);            //DIRO Pin

  //HIGH to Receive from SDI-12
  digitalWrite(DIRO, HIGH);

  strcpy(errorMessage, " ERROR: SYSTEM TIMED OUT\n\n ERROR LOGGED TO SD\n\n USER MUST RESET SYSTEM");
  errorLocatedInSetup = false;
}

void ReceiveCommand() {
  while (Serial1.available()) {
    //Receive SDI-12 over UART
    int byte = Serial1.read();                                      //Reads incoming communication in bytes

    if (byte == 33) {  
      processCommand = true;                                        //If byte is command terminator (!)
    } else {
      if (byte != 0) {                                              //do not add start bit (0)
        command += char(byte);                                      //append byte to command string
      }
    }
  }
}

void DoSumMeasurement() {
  // check that the number of samples taken is less than the expected amount to be taken. We multiply samplingDuration by 10 to convert it to deciseconds so that it works with samplingPeriod
  if (samplesTaken < (samplingDuration * 10) / samplingPeriod) {
    // take samples depending on what sensors are available
    switch (sensorsReady) {
      case 1:
        sumLux += lightMeter.readLightLevel();
        break;
      case 3:
        AddBmeToSum();
        break;
      case 4:
        AddBmeToSum();
        sumLux += lightMeter.readLightLevel();
        break;
      default:
        Timer4.stop();
        return;
    }
    Serial.println("sample");
    samplesTaken++;
    // once all of the samples have been taken, stop the timer and take the average of all of the parameters.
    if (samplesTaken >= (samplingDuration * 10) / samplingPeriod) {
      Timer4.stop();
      // has a 1 +  in the denominator to avoid dividing by 0. This also accounts for the first sample taken during the sensor check.
      avgTemp = sumTemp / (1 + samplingDuration * 10 / samplingPeriod);
      avgPressure = (sumPressure / (1 + samplingDuration * 10 / samplingPeriod)) / 100.0;
      avgHumidity = sumHumidity / (1 + samplingDuration * 10 / samplingPeriod);
      avgLux = sumLux / (1 + samplingDuration * 10 / samplingPeriod);
    }
  }
}

// Timer interrupt for continuous measurements
void TriggerContMeasurement() {
  if (runningContinuous) {
    takeContMeasurement = true;
  }
}

// switch to BME280
void BMEmenu() {
  if (menuSelect != BME) {
    menuSelect = BME;
    refreshDisplay = true;
  }
}

// switch to BH1750
void BHmenu() {
  if (menuSelect != BH) {
    menuSelect = BH;
    refreshDisplay = true;
  }
}

// switch to main menu
void ExitToMainMenu() {
  if (menuSelect != MAINMENU) {
    menuSelect = MAINMENU;
    refreshDisplay = true;
  }
}

// stop continuous measurements
void StopCont() {
  runningContinuous = false;
  Timer3.stop();
}

void loop() {
  WDT->WDT_CR = WDT_CR_KEY(WDT_KEY) | WDT_CR_WDRSTT;                // Feed watchdog (reset timer)

  if (processCommand) {
    SDI12Receive(command);
    command = "";                                                   //reset command string
    processCommand = false;
  }
  if (takeContMeasurement) {
    DoContMeasurement();
  }
  if (refreshDisplay) {
    DoRefreshDisplay();
  }
  // if (!runningContinuous) {
  //   Timer3.stop();
  // }
}

void SDI12Receive(String input) {
  //convert device address to string
  String address = String(deviceAddress);

  //Determines if the command is addressed for this device
  if (String(input.charAt(0)) == address) {

    // Address change
    if ((String(input.charAt(1)) == "A") && isdigit(input.charAt(2)) && input.length() == 3) {
      deviceAddress = input.charAt(2) - '0';
      address = String(deviceAddress);
      SDI12Send(address);
      return;
    }

    // Send identification
    if ((String(input.charAt(1)) == "I") && input.length() == 2) {
      String ident = address.charAt(0) + deviceIdentification.substring(1);  // passes in the device address, then passes in the device ID skipping the placeholder address character
      SDI12Send(ident);
      return;
    }

    // Start Measurement, Check that the length is either 2,3,5. Check that the characters after "M" are numbers.
    if ((String(input.charAt(1)) == "M") && (input.length() == 2 || (input.length() == 3 && isdigit(input.charAt(2)) || (input.length() == 5 && isdigit(input.charAt(4)) && isdigit(input.charAt(3)) && isdigit(input.charAt(2)))))) {
      String result = address;
      sensorsReady = 0; // reset all variables to default values
      samplingDuration = 0;
      samplingPeriod = 10; // period is in deciseconds (1/10 of a second)
      samplesTaken = 0;
      sumTemp = 0;
      sumPressure = 0;
      sumHumidity = 0; 
      sumLux = 0;

      // By default will be 000
      char ttt[3];
      if (input.length() == 3) {
        //aMb!, b = samplingDuration
        samplingDuration = input.charAt(2) - '0';
        samplesTaken = 0;
        samplingPeriod = 10; // default sampling period is 1 second (10 deciseconds)
        if (samplingDuration != 0) {
          Timer4.setPeriod(samplingPeriod * 100000).start(); // setPeriod is in microseconds so its 100 000 * deciseconds
        }
      } else if (input.length() == 5) {
        //aMbcd!, d = samplingDuration, bc = sampling period in deciseconds
        samplingDuration = input.charAt(4) - '0';
        samplingPeriod = ((input.charAt(3) - '0')) + (input.charAt(2) - '0') * 10; // scale b by 10 to put it in the tens digit
        samplesTaken = 0;
        // check that the samplingDuration is a perfect multiple of the samplingPeriod. Otherwise, the average will be off due to program thinking we were sampling for longer.
        // we do this by checking the the remainder is 0. This is why the period is in deciseconds, because remainder doesn't work with floats and float precision is bad.
        // again, we multiply samplingDuration by 10 so that it's also in deciseconds.
        if (samplingDuration != 0 && (10 * samplingDuration) % samplingPeriod == 0) {
          Timer4.setPeriod(samplingPeriod * 100000).start();
        } else {
          // error message if it's not a perfect multiple.
          Serial.println("Sampling duration should be perfectly divisible by Sampling Period");
          return;
        }
      } else {
        samplingDuration = 0;
        samplingPeriod = 10;
      }
      //say that the result will be ready in samplingDuration seconds
      sprintf(ttt, "%03d", samplingDuration);
      result.concat(ttt);


      char n[1];
      sensorsReady = 0;
      // upon checking the sensors, we take one sample so that 0M! still takes a sample. So when taking samples at a 0.5 second period over 5 seconds, we actually take 11 samples
      // 1 sample during sensor check + (5/0.5 = 10 samples during timer interrupt)
      if (bme.takeForcedMeasurement()) {
        if (bme.readTemperature() < 180.0f && bme.readPressure() > -15415.64f && bme.readHumidity() != 100.00f) {
          sensorsReady += 3;
          AddBmeToSum();
        }
      }

      if (lightMeter.measurementReady() && lightMeter.readLightLevel() >= 0) {
        sensorsReady += 1;
        sumLux = lightMeter.readLightLevel();
      }
      sprintf(n, "%d", sensorsReady);
      result.concat(n);

      if (samplingDuration == 0) {
        avgTemp = sumTemp;
        avgPressure = sumPressure / 100.0;
        avgHumidity = sumHumidity;
        avgLux = sumLux;
      }

      SDI12Send(result);
      return;
    }

    // Send Data
    if ((String(input.charAt(1)) == "D") && input.length() == 3 && isdigit(input.charAt(2))) {
      String result = address;

      int commandIndex = input.charAt(2) - '0';

      if (commandIndex > 5 || commandIndex < 0) {
        return;
      }

      TransferAveragesToGlobal();
      lux = avgLux;

      switch (commandIndex) {
        case 0:
          result = BMEconcatAverages(result);
          result.concat("+");
          result.concat(String(avgLux));
          break;
        case 1:
          result = BMEconcatAverages(result);
          break;
        case 2:
          result.concat("+");
          result.concat(String(avgLux));
          break;
        case 3:
          // Sending temp measurement to SDI12
          result.concat(Sign(avgTemp));
          result.concat(String(avgTemp));
          break;
        case 4:
          // Sending pressure measurement to SDI12
          result.concat(Sign(avgPressure));
          result.concat(String(avgPressure));
          break;
        case 5:
          // Sending humidity measurement to SDI12
          result.concat(Sign(avgHumidity));
          result.concat(String(avgHumidity));
          break;
        default:
          Serial.println("Command index not supported");
          break;
      }
      SaveToSD(file);

      if (menuSelect != MAINMENU) {
        refreshDisplay = true;
      }

      SDI12Send(result);
      return;
    }

    // Continuous Measurements
    if ((String(input.charAt(1)) == "R") && input.length() == 3) {
      runningContinuous = true;
      Timer3.setFrequency(0.4).start();

      int commandIndex = input.charAt(2) - '0';

      if (commandIndex >= 0 && commandIndex <= 5) {
        selectContMeasure = commandIndex + 1;
      } else {
        runningContinuous = false;
        Timer3.stop();
        Serial.println("Command index not supported");
      }
    }
  } else if (String(input.charAt(0)) == "?" && input.length() == 1) {
    SDI12Send(address);
  }
}

void SDI12Send(String message) {
  Serial.print("message: ");
  Serial.println(message);

  digitalWrite(DIRO, LOW);
  delay(100);
  Serial1.print(message + String("\r\n"));
  Serial1.flush();  //wait for print to finish
  Serial1.end();
  Serial1.begin(1200, SERIAL_7E1);
  digitalWrite(DIRO, HIGH);
  //secondruntoken = 0;
}

void DoRefreshDisplay() {
  tft.fillScreen(ST77XX_BLACK);  // fillrecting a smaller section of the screen may be faster, but we will use this for now
  UpdateDisplay();
  refreshDisplay = false;
}

void DoContMeasurement() {
  String result = String(deviceAddress);

  switch (selectContMeasure) {
    case ALL:
      if (bme.takeForcedMeasurement()) {
        if (bme.readTemperature() < 180.0f && bme.readPressure() > -15415.64f && bme.readHumidity() != 100.00f) {
          TransferBmeToGlobal();
          result = BMEconcat(result);
        }
      }

      // LightMeter will return -1 or -2 as error codes if something is wrong
      if (lightMeter.measurementReady() && lightMeter.readLightLevel() >= 0) {
        result.concat("+");
        lux = lightMeter.readLightLevel();
        result.concat(String(lux));
      }
      break;
    case BME280:
      if (bme.takeForcedMeasurement()) {
        if (bme.readTemperature() < 180.0f && bme.readPressure() > -15415.64f && bme.readHumidity() != 100.00f) {
          TransferBmeToGlobal();
          result = BMEconcat(result);
        }
      }
      break;
    case BH1750:
      // LightMeter will return -1 or -2 as error codes if something is wrong
      if (lightMeter.measurementReady() && lightMeter.readLightLevel() >= 0) {
        result.concat("+");
        lux = lightMeter.readLightLevel();
        result.concat(String(lux));
      }
      break;
    case TEMP:
      if (bme.takeForcedMeasurement()) {
        if (bme.readTemperature() < 180.0f) {
          result.concat(Sign(temp));
          result.concat(String(temp));
        }
      }
      break;
    case PRESSURE:
      if (bme.takeForcedMeasurement()) {
        if (bme.readPressure() > -15415.64f) {
          result.concat(Sign(pressure));
          result.concat(String(pressure));
        }
      }
      break;
    case HUMIDITY:
      if (bme.takeForcedMeasurement()) {
        result.concat(Sign(humidity));
        result.concat(String(humidity));
      }
      break;
  }

  SDI12Send(result);
  SaveToSD(file);
  if (menuSelect != MAINMENU) {
    refreshDisplay = true;
  }
  takeContMeasurement = false;
}

void TransferAveragesToGlobal() {
  temp = avgTemp;
  pressure = avgPressure;
  humidity = avgHumidity;
}

void TransferBmeToGlobal() {
  temp = bme.readTemperature();
  pressure = bme.readPressure() / 100.0;
  humidity = bme.readHumidity();
}

void AddBmeToSum() {
  sumTemp += bme.readTemperature();
  sumHumidity += bme.readHumidity();
  sumPressure += bme.readPressure();
}


String Sign(float input) {
  if (input < 0) {
    return "";
  } else {
    return "+";
  }
}

String BMEconcat(String input) {
  input.concat(Sign(temp));
  input.concat(String(temp));
  input.concat(Sign(pressure));
  input.concat(String(pressure));
  input.concat(Sign(humidity));
  input.concat(String(humidity));
  return input;
}

String BMEconcatAverages(String input) {
  input.concat(Sign(avgTemp));
  input.concat(String(avgTemp));
  input.concat(Sign(avgPressure));
  input.concat(String(avgPressure));
  input.concat(Sign(avgHumidity));
  input.concat(String(avgHumidity));
  return input;
}

void DrawMainMenu() {
  tft.setTextSize(2);
  tft.setCursor(0, 10);
  tft.setTextColor(ST77XX_CYAN);
  tft.println(" Buttons:\n");
  tft.setTextColor(ST77XX_WHITE);
  tft.println(" 1. BME280");
  tft.println(" 2. BH1750");
  tft.println(" 3. Main Menu");
  tft.println(" 4. Stop C.M.");  // Stop continuous measurement
}


void DrawData(float data, float low, float high, int pos, uint16_t color, String text, int scalers[5]) {
  int width = map(data, low, high, 0, tft.width() - 50);
  tft.setCursor(5, pos);
  tft.setTextSize(1);
  tft.println(text);
  tft.fillRect(25, pos + 15, width, 10, color);
  tft.drawFastVLine(53, pos + 15, 10, ST77XX_WHITE);
  tft.drawFastVLine(80, pos + 15, 10, ST77XX_WHITE);
  tft.drawFastVLine(107, pos + 15, 10, ST77XX_WHITE);
  tft.setCursor(24, 0);
  tft.drawRect(24, pos + 14, tft.width() - 49, 12, color);
  for (int i = 0; i < 5; i++) {
    tft.setCursor(21 + (i * 27), pos + 27);
    tft.print(scalers[i]);
  }
}

void DrawGraphScreen() {
  switch (menuSelect) {
    case BME:
      DrawData(temp, 0, 85, 5, ST77XX_RED, "Temperature (Celsius)", scalersTemp);
      DrawData(pressure, 0, 1100, tft.height() * 0.33 + 5, ST77XX_GREEN, "Pressure (hPa)", scalersPressure);
      DrawData(humidity, 0, 100, tft.height() * 0.66 + 5, ST77XX_BLUE, "Humidity (%)", scalersHumidity);
      break;
    case BH:
      DrawData(lux, 0, 54612, 5, ST77XX_MAGENTA, "Light (klx)", scalersLux);
      break;
    default:
      Serial.println("In the void");
  }
}

// Chooses which screen to draw
// Clearing the screen will be separate so we can implement a more efficient method if we get there
void UpdateDisplay() {
  if (menuSelect == MAINMENU) {
    DrawMainMenu();
  } else {
    DrawGraphScreen();
  }
}

void SaveToSD(File file) {
  now = rtc.now();
  sprintf(timeLog, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  file.open("SensorData.txt", FILE_WRITE);
  file.seekEnd();
  switch (menuSelect) {
    case BME:
      file.println("----------BME280----------");
      file.println(timeLog);
      file.print("Temperature: ");
      file.print(temp);
      file.println("°C");
      file.print("Pressure: ");
      file.print(pressure);
      file.println("hPa");
      file.print("Humidity: ");
      file.print(humidity);
      file.println("%");
      break;
    case BH:
      file.println("----------BH1750----------");
      file.println(timeLog);
      file.print("Brightness: ");
      file.print(lux);
      file.println("lx");
      break;
  }

  file.close();
}

void WDT_Handler(void) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 5);
  tft.setTextSize(1);
  tft.write(errorMessage);
  if (!errorLocatedInSetup) {
    file.open("SensorData.txt", FILE_WRITE);
    file.seekEnd();
    file.println("-----------ERROR----------");
    file.println("System timed out in loop");
    file.close();
  }
  while (1)
    ;
}
