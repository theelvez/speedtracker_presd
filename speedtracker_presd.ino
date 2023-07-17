#include <Adafruit_PN532.h>
#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>
#include <U8g2lib.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <esp_wifi.h>
#include <esp_http_client.h>

// Touch Sensor definitions
#define BUTTON_INPUT_PIN              32

// Status LED definitions
#define LED_BLUE_OUTPUT_PIN           14
#define LED_RED_OUTPUT_PIN            25
#define LED_GREEN_OUTPUT_PIN          27
#define LED_VCC_PIN                   26 

// BAMF special values
#define BAMF_X_POSITION               120
#define BAMF_Y_POSITION               6
#define BAMF_CHAR                     "B"

#define RUN_CONFIG_FILE_PATH          "/run_config.txt"
#define RUN_CONFIG_TEMP_FILE_PATH     "/tmp_run_config.txt"
#define RUN_DATA_FILE_PATH            "/run_data.txt"

// BAMF special values
#define BAMF_X_POSITION               120
#define BAMF_Y_POSITION               6
#define BAMF_CHAR                     "B"

void* currentScreen = NULL;

//
// Lat Lng pair
//
struct GpsCoordinate {
  double latitude;
  double longitude;
};

//
// Race information data structure
//
typedef struct _RUN_INFORMATION {
  String device_id;
  GpsCoordinate finishLine_left;
  GpsCoordinate finishLine_right;
  double high_speed;
  double bamf_speed;
  String UploadServerIP;
  String UploadSSID;
  String UploadPassword;
} RUN_INFORMATION;

RUN_INFORMATION runInformation;

//
// Speed tracking data structure definition
//
typedef struct _SPEEDTRACKER_INFO {
  double latitude;
  double longitude;
  double mph;  
} SPEEDTRACKER_INFO;


//
// Maximum number of SPEEDTRACKER_INFO objects in the SPEEDTRACKER_INFO array.
//
#define SPEEDTRACKER_INFO_MAX_ENTRIES   2000

//
// The array of SPEEDTRACKER_INFO objects used to track the speed/location info for a race
//
SPEEDTRACKER_INFO stInfo[SPEEDTRACKER_INFO_MAX_ENTRIES]; // Array of the speedtracker info entries

//
// Index of the current entry in the SPEEDTRACKER_INFO object array
//
uint stInfoCurrentIndex = 0; // Current index of the speedtracker array

void ledEnableRed() {
  ledDisable(LED_GREEN_OUTPUT_PIN);
  ledDisable(LED_BLUE_OUTPUT_PIN);
  ledEnable(LED_RED_OUTPUT_PIN);
}

void ledEnableGreen() {
  ledDisable(LED_RED_OUTPUT_PIN);
  ledDisable(LED_BLUE_OUTPUT_PIN);
  ledEnable(LED_GREEN_OUTPUT_PIN);
}

void ledEnableBlue() {
  ledDisable(LED_GREEN_OUTPUT_PIN);
  ledDisable(LED_RED_OUTPUT_PIN);
  ledEnable(LED_BLUE_OUTPUT_PIN);
}

void ledDisableAll() {
  ledDisable(LED_GREEN_OUTPUT_PIN);
  ledDisable(LED_RED_OUTPUT_PIN);
  ledDisable(LED_BLUE_OUTPUT_PIN);
}

void ledEnable(uint8_t ledNumber) {
  digitalWrite(ledNumber, LOW);
}

void ledDisable(uint8_t ledNumber) {
  digitalWrite(ledNumber, HIGH);
}

SFE_UBLOX_GNSS myGNSS;

//
// PN532 Globals
//
Adafruit_PN532 nfc(1, -1, &Wire);

//
// U8G2 library I2C SSD1306 display object
//
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

//
// Speed tracking related globals
//
volatile bool touchDetected = false; // Flag that is set when touch sensor is activated
volatile bool speed_tracking_active = false; // Flag to indicate that the location and speed are being tracked
unsigned long lastMillis = 0; // Counter to track logging times
const unsigned long debounceDelay = 150; // Debounce delay in milliseconds
volatile unsigned long lastDebounceTime = 0; // Stores the last time the button was pressed

//
// ISRs
//
void IRAM_ATTR buttonInterrupt() {
  // Check if the debounce delay has passed since the last button press
  if ((millis() - lastDebounceTime) > debounceDelay) {
    touchDetected = true;
    lastDebounceTime = millis(); // Update the last debounce time
  }
}

//
// GPS helper routines
//


double convertToDegrees(int32_t coordinate) {
  double degrees = coordinate / 10000000.0;
  return degrees;
}

double convertMmPerSecToMph(double speedInMmPerSec) {
    double conversionFactor = 0.00223694;
    return speedInMmPerSec * conversionFactor;
}

double getGPSSpeed() {
  return convertMmPerSecToMph(myGNSS.getGroundSpeed());
}

double getLongitudeDegrees() {
  return convertToDegrees(myGNSS.getLongitude());
}

double getLatitudeDegrees() {
  return convertToDegrees(myGNSS.getLatitude());
}


GpsCoordinate finishLineLeft = {47.54406, -122.04987};
GpsCoordinate finishLineRight = {47.54439, -122.04957};
GpsCoordinate vehiclePreviousPosition = {0, 0};
GpsCoordinate vehicleCurrentPosition = {0, 0};


// Use your new struct in the function
bool crossFinishLine(struct GpsCoordinate finishLineLeft, struct GpsCoordinate finishLineRight, struct GpsCoordinate vehiclePreviousPosition, struct GpsCoordinate vehicleCurrentPosition) {
  double denominator = ((finishLineRight.latitude - finishLineLeft.latitude) * (vehicleCurrentPosition.longitude - vehiclePreviousPosition.longitude) - (finishLineRight.longitude - finishLineLeft.longitude) * (vehicleCurrentPosition.latitude - vehiclePreviousPosition.latitude));
  double numerator1 = ((finishLineRight.longitude - finishLineLeft.longitude) * (vehiclePreviousPosition.latitude - finishLineLeft.latitude) - (finishLineRight.latitude - finishLineLeft.latitude) * (vehiclePreviousPosition.longitude - finishLineLeft.longitude));
  double numerator2 = ((vehicleCurrentPosition.longitude - vehiclePreviousPosition.longitude) * (vehiclePreviousPosition.latitude - finishLineLeft.latitude) - (vehicleCurrentPosition.latitude - vehiclePreviousPosition.latitude) * (vehiclePreviousPosition.longitude - finishLineLeft.longitude));

  // Detect coincident lines (has a problem, read below)
  if (denominator == 0) return numerator1 == 0 && numerator2 == 0;

  double r = numerator1 / denominator;
  double s = numerator2 / denominator;

  return (r >= 0 && r <= 1) && (s >= 0 && s <= 1);
}

void drawAttentionScreen(String attentionString) {

  currentScreen = (void*)&drawAttentionScreen;

  // Clear the display
  u8g2.clearBuffer();

  // Set text size and position
  u8g2.setFont(u8g2_font_ncenB12_tf);
  u8g2.drawStr(10, 10, attentionString.c_str());

  // Check bamf status
  if (runInformation.bamf_speed > 0) {
    u8g2.drawStr(BAMF_X_POSITION, BAMF_Y_POSITION, BAMF_CHAR);
  }

  // Clear the display
  u8g2.sendBuffer();
}

void drawGPSLockScreen(String device_id) {
  currentScreen = (void*)&drawAttentionScreen;
  
  ledEnableRed();

  u8g2.setFont(u8g2_font_fub11_tr); // set font size to 8

  // Ready section
  u8g2.setFont(u8g2_font_fub17_tf); // set font size to 30

  // Check bamf status
  if (runInformation.bamf_speed > 0) {
    u8g2.drawStr(BAMF_X_POSITION, BAMF_Y_POSITION, BAMF_CHAR);
  }

  int16_t x = u8g2.getDisplayWidth();
  char* message = "Acquiring GPS ...";

  while (myGNSS.getFixType() != 3) {
    u8g2.clearBuffer(); // clear the buffer
    u8g2.setFont(u8g2_font_fub11_tr); // set font size to 8
    u8g2.drawStr(0, 20, device_id.c_str()); // draw device id
    u8g2.setFont(u8g2_font_fub17_tf); // set font size to 30
    u8g2.drawStr(x, 55, message); // draw "Acquiring GPS ..."
    u8g2.sendBuffer(); // send the buffer to the display
    delay (100);
    x -= 5;

    if (x < -int(u8g2.getDisplayWidth() + u8g2.getDisplayWidth())) {
      x = u8g2.getDisplayWidth();
    }
  }

  ledEnableGreen();
}



void drawReadyScreen(String device_id) {
  u8g2.clearBuffer(); // clear the buffer
  
  // Top section
  u8g2.setFont(u8g2_font_fub11_tr); // set font size to 8
  u8g2.drawStr(0, 20, device_id.c_str()); // draw full name
  
  // Ready section
  u8g2.setFont(u8g2_font_fub17_tr); // set font size to 30
  u8g2.drawStr(0, 55, "Go to Start"); // draw "Ready"

  // Check bamf status
  if (runInformation.bamf_speed > 0) {
    u8g2.drawStr(BAMF_X_POSITION, BAMF_Y_POSITION, BAMF_CHAR);
  }
  
  u8g2.sendBuffer(); // send the buffer to the display
}

void drawMainScreen(String deviceId, double speed) {
  u8g2.clearBuffer(); // clear the buffer
  
  // Top section
  u8g2.setFont(u8g2_font_ncenB08_tr); // set font size to 8
  u8g2.drawStr(0, 10, deviceId.c_str()); // draw full name
  
  // Speed section
  u8g2.setFont(u8g2_font_fub20_tr); // set font size to 30
  char speedStr[7];
  sprintf(speedStr, "%06.2f", speed); // format speed with two decimal places and leading zeros if needed
  u8g2.drawStr(0, 60, speedStr); // draw speed
  u8g2.setFont(u8g2_font_fub14_tr); // set font size to 30  
  u8g2.drawStr(87, 60, "mph"); // draw "mph" after speed

  // Check bamf status
  if (runInformation.bamf_speed > 0) {
    u8g2.drawStr(BAMF_X_POSITION, BAMF_Y_POSITION, BAMF_CHAR);
  }
  
  u8g2.sendBuffer(); // send the buffer to the display
}

void drawBootingScreen() {
  u8g2.clearBuffer(); // clear the buffer

  // draw booting message
  u8g2.setFont(u8g2_font_profont22_tr); // set font size to 22
  u8g2.drawStr(0, 60, "Booting..."); // draw "Booting..." message
  
  u8g2.sendBuffer(); // send the buffer to the display


  u8g2.setFont(u8g2_font_ncenB08_tr); // set font size to 8
  
  // draw progress bar
  //u8g2.drawBox(0, 30, 128, 4); // draw progress bar outline
  for(int i = 0; i < 128; i += 4) {
    u8g2.drawBox(i, 30, 2, 4); // draw progress bar animation
    u8g2.sendBuffer(); // send the buffer to the display
    delay(50); // wait for 50 ms
  }
}

void drawSummaryScreen(double topSpeed) {
  u8g2.clearBuffer(); // clear the buffer
  
  // Top section
  u8g2.setFont(u8g2_font_ncenB08_tr); // set font size to 8
  u8g2.drawStr(0, 10, "Top Speed:"); // draw "Top Speed:"

  // Check bamf status
  if (runInformation.bamf_speed > 0) {
    u8g2.drawStr(BAMF_X_POSITION, BAMF_Y_POSITION, BAMF_CHAR);
  }

  // Speed section
  u8g2.setFont(u8g2_font_fub20_tr);
  char speedStr[7];
  sprintf(speedStr, "%06.2f", topSpeed); // add leading zeros if needed
  u8g2.drawStr(0, 60, speedStr); // draw speed
  u8g2.setFont(u8g2_font_fub14_tr);
  u8g2.drawStr(87, 60, "mph"); // draw "mph" after speed
  u8g2.sendBuffer(); // send the buffer to the display
  
  delay(10000); // wait for 10 seconds
}


void processRunState()
{  
  double mph = getGPSSpeed();
  double latitude = getLatitudeDegrees();
  double longitude = getLongitudeDegrees();

  if (((millis() - lastMillis) > 1000) ||
      (lastMillis <= 0)) {
    lastMillis = millis();

    if (mph < 2.0) {
      mph = 0;
    }

    // Save the current mph and location info and advance the index
    stInfo[stInfoCurrentIndex].latitude = latitude;
    stInfo[stInfoCurrentIndex].longitude = longitude;
    stInfo[stInfoCurrentIndex].mph = mph;

    Serial.printf("processRunState::%10.7f,%10.7f,%06.2f\n", stInfo[stInfoCurrentIndex].latitude, stInfo[stInfoCurrentIndex].longitude, stInfo[stInfoCurrentIndex].mph);

    stInfoCurrentIndex++;

    // Check for overflow
    if (stInfoCurrentIndex >= SPEEDTRACKER_INFO_MAX_ENTRIES) {
      saveSpeedTrackerInfoToSD();
      stInfoCurrentIndex = 0;      
    }
  }

  

  drawMainScreen(runInformation.device_id, mph);

  // Update the max speed if needed
  if (mph > runInformation.high_speed) {
    runInformation.high_speed = mph;
  }
}


//Function to save the stInfo array to a JSON file on the SD card
void saveSpeedTrackerInfoToSD() {
  
  const char* filename = RUN_DATA_FILE_PATH;

  //char filedata[SPEEDTRACKER_INFO_MAX_ENTRIES * (sizeof(SPEEDTRACKER_INFO) + 3)];

  // Open the file for writing
  File file = SD.open(filename, FILE_APPEND);
  if (!file) {
    Serial.printf("Failed to open file %s for writing.", filename);
    Serial.println();
    return;
  } else {
    Serial.printf("Opened file %s for writing.", filename);                        
    Serial.println();
  }

  // Write out the high speed
  file.printf("High Speed: %06.2f\n", runInformation.high_speed);

  // Add stInfo array data to the JSON array
  for (int i = 0; i < SPEEDTRACKER_INFO_MAX_ENTRIES; i++) {
    if (stInfo[i].latitude) {
      file.printf("%10.7f,%10.7f,%10.7f\n", stInfo[i].latitude, stInfo[i].longitude, stInfo[i].mph);
      Serial.printf("%10.7f,%10.7f,%10.7f\n", stInfo[i].latitude, stInfo[i].longitude, stInfo[i].mph);
    }
  }

  // Close the file
  file.close();

  // Clear the stInfo array by setting all elements to zero
  memset(stInfo, 0, sizeof(stInfo));

  Serial.println("Data saved to SD card successfully.");
}




//
// Read the run config from the config file
//
bool loadRunConfig() {
 
  // Open file
  const char* configFileName = RUN_CONFIG_FILE_PATH;  

   File configFile = SD.open(configFileName, FILE_READ);
   if (!configFile) {
     // Config file does not exist or cannot be read
     // Use default values for configuration settings
     Serial.printf("Failed to open file %s\n", configFileName);
     return false;
   } else {
     Serial.printf("Opened file %s\n", configFileName);
   }
  
   // Read in config settings from file
   while (configFile.available()) {
     // Read in line from file
    String line = configFile.readStringUntil('\n');

    if (line.endsWith("\r")) {
      line = line.substring(0, line.length() - 1);
    }

    Serial.println(line);
    
    // Split line at equal sign (=)
    int equalsIndex = line.indexOf('=');
    if (equalsIndex == -1) {
      // Invalid format
      continue;
    }
  
    // Extract key-value pair
    String key = line.substring(0, equalsIndex);
    String value = line.substring(equalsIndex + 1);

    line.toLowerCase();
   
    // Store key-value pair in global variables
    if (key == "device_id") {
      // Get MAC address
      uint8_t baseMac[6];
      // Get base MAC address
      esp_read_mac(baseMac, ESP_MAC_WIFI_STA);

      // Convert MAC address to string
      char macStr[13] = { 0 };
      sprintf(macStr, "%2X%2X%2X%2X%2X%2X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);

      runInformation.device_id = macStr;
    } else if (key == "finishLine_left_lat") {
      runInformation.finishLine_left.latitude = value.toDouble();
    } else if (key == "finishLine_left_lng") {
      runInformation.finishLine_left.longitude = value.toDouble();
    } else if (key == "finishLine_right_lat") {
      runInformation.finishLine_right.latitude = value.toDouble();
    } else if (key == "finishLine_right_lng") {
      runInformation.finishLine_right.longitude = value.toDouble();
    } else if (key == "bamf_speed") {
      runInformation.bamf_speed = value.toDouble();
    } else if (key == "UploadServerIP") {
      runInformation.UploadServerIP = value;
    } else if (key == "UploadSSID") {
      runInformation.UploadSSID = value;
    } else if (key == "UploadPassword") {
      runInformation.UploadPassword = value;
    } else {
      // Unknown key
    }
  }

   // Close file
  configFile.close();

  return true;
}

/*String readCardData(uint16_t timeout) {
  uint8_t success;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t uidLength;
  String cardData;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, timeout);

  if (success) {
    uint8_t pageBuffer[4];

    // Read 4 bytes (1 page) at a time
    for (uint8_t page = 4; page < 129; page++) {

      if (nfc.ntag2xx_ReadPage(page, pageBuffer)) {
        for (uint8_t i = 0; i < 4; i++) {
          if ((pageBuffer[i] > 31) && (pageBuffer[i] < 127)) {
            if (pageBuffer[i] != 13) {
              cardData += (char)pageBuffer[i];
            }
          Serial.printf("Data: %s\n", cardData.c_str());          }
        }
      } else {
        //Serial.println("Failed to read page data");
        //Serial.printf("readCardData() returning %s\n", cardData.c_str());
        return cardData;
      }
    }
  }
  //Serial.printf("readCardData() returning %s\n", cardData.c_str());
  return cardData;
}*/

String parseCardData(String rawConfig) {
  int start = 0, end = 0;
  String parsedData;

  rawConfig.toLowerCase();

  while(start != -1) {
    start = rawConfig.indexOf("?", end);
    if(start != -1) {
      end = rawConfig.indexOf(">", start+1);
      if(end != -1) {
        Serial.println(rawConfig.substring(start+1, end));
        parsedData += rawConfig.substring(start+1, end) + '\n';
      } else {
        break;
      }
    }
  }
  return parsedData;
}


String readCardData(uint16_t timeout) {
  uint8_t success;
  uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
  uint8_t uidLength;
  String cardData;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, timeout);

  if (success) {
    uint8_t pageBuffer[4];

    // Read 4 bytes (1 page) at a time
    for (uint8_t page = 4; page < 129; page++) {

      if (nfc.ntag2xx_ReadPage(page, pageBuffer)) {
        for (uint8_t i = 0; i < 4; i++) {
          if ((pageBuffer[i] > 31) && (pageBuffer[i] < 127)) {
            if (pageBuffer[i] != 13) {
              cardData += (char)pageBuffer[i];
            }
          //Serial.printf("Data: %s\n", cardData.c_str());          
          }
        }
      } else {
        //Serial.println("Failed to read page data");
        //Serial.printf("readCardData() returning %s\n", cardData.c_str());
        return parseCardData(cardData);
      }
    }
  }
  //Serial.printf("readCardData() returning %s\n", cardData.c_str());
  return parseCardData(cardData);
}

void setup()
{
  Serial.begin(115200);

  //
  // Set the clock rate to 100kHz to stabelize the I2C bus
  //
  Wire.setClock(100000);

  //
  // Initialize the display
  //
  if(!u8g2.begin()) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  } else {
    Serial.println(F("SSD1306 allocation success"));
  }
  
  //
  // Draw the initial boot screen
  //
  drawBootingScreen();

  //
  // Initialize the GNSS unit
  //
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));

    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(10); //Produce two solutions per second
  myGNSS.setAutoPVT(true); //Tell the GNSS to "send" each solution


  
  //
  // Set LED outputs
  //
  pinMode(LED_GREEN_OUTPUT_PIN, OUTPUT);
  pinMode(LED_BLUE_OUTPUT_PIN, OUTPUT);
  pinMode(LED_RED_OUTPUT_PIN, OUTPUT);
  
  pinMode(LED_VCC_PIN, OUTPUT);
  digitalWrite(LED_VCC_PIN, HIGH);
  
  //
  // Initial state is the LED red
  //
  ledDisableAll();

  //
  // Initialize SD card if it's not already
  //
  while (!SD.begin(5)/*, SPI, 400000, "/sd", 10, false)*/) {
    Serial.println("Failed to initialize SD card.");
    u8g2.clearBuffer(); // clear the buffer
    u8g2.setFont(u8g2_font_fub14_tr);
    u8g2.drawStr(0, 20, "Insert SD card...");
    u8g2.sendBuffer(); // send the buffer to the display
    delay(5000);
  } 
  
  Serial.println("Initialized SD card.");
  
  //
  // Print the block of text
  //
  nfc.begin();

  //
  // Check for successful initialization of the NFC reader
  //
  if (!nfc.getFirmwareVersion()) {
    Serial.print("Failed to initialize PN532 board. Halting...");
    while (1); // halt
  } else {
    Serial.println("Initialize PN532 board");
  }

  //
  // Configure the PN532 reader 
  //
  nfc.SAMConfig();

  //
  // Set the touchPin as an INPUT
  //
  pinMode(BUTTON_INPUT_PIN, INPUT_PULLDOWN);

  //
  // Attach the interrupt to the touchPin to trigger the interrupt when the pin goes from LOW to HIGH
  //
  attachInterrupt(digitalPinToInterrupt(BUTTON_INPUT_PIN), buttonInterrupt, FALLING);

  if (!loadRunConfig()) {
    u8g2.drawStr(10, 10, "No run info on SD card");
  }

  if (myGNSS.getFixType() != 3) {
    drawGPSLockScreen(runInformation.device_id);
  }

  drawReadyScreen(runInformation.device_id);

}
// Declaration of your functions
void bamf_speed(String parameter) {
    Serial.printf("Running bamf_speed(%s) function", parameter.c_str());
    runInformation.bamf_speed = parameter.toDouble();
}

void start_run(String parameter) {
  Serial.println("Running start_run function");

  if (speed_tracking_active != false) {
    // Reset globals      
    speed_tracking_active = false;
    lastMillis = 0;
    ledEnableBlue();

    if (!runInformation.bamf_speed) {
      drawSummaryScreen(runInformation.high_speed);
    } else {
      drawSummaryScreen(runInformation.bamf_speed);
    }
    
    saveSpeedTrackerInfoToSD();
    drawReadyScreen(runInformation.device_id);
    runInformation.high_speed = 0;

  } else {
    speed_tracking_active = true;    
    ledEnableBlue();      
  }
}

void end_run(String parameter) {
  Serial.println("Running start_run function");
  if (speed_tracking_active) {
    start_run("");
  }
}

void show_high(String parameter) {
    Serial.println("Running show_high function");
    
    if (!runInformation.bamf_speed) {
      drawSummaryScreen(runInformation.high_speed);
    } else {
      drawSummaryScreen(runInformation.bamf_speed);
    } 
}

void reset_sd(String parameter) {
    Serial.println("Running reset_sd function");
    // Your function's code here
}

void run_diagnostics(String parameter) {
    Serial.println("Running run_diagnostics function");
    // Your function's code here
}

void force_upload(String parameter) {
    Serial.println("Running force_upload function");
    // Your function's code here
}

// Function to parse the command and call the appropriate function with parameter
void checkAndExecuteCommand(String command) {
    // Find the index of '=' character
    int index = command.indexOf('=');
    
    // Get the function name and trim it
    String functionName = command;
    String parameter = "";
    
    // If '=' character is found, separate the function name and parameter
    if(index != -1) {
        functionName = command.substring(0, index);
        parameter = command.substring(index + 1);
    }
    
    functionName.trim();
    parameter.trim();

    if(functionName == "bamf_speed") {
        bamf_speed(parameter);
    }
    else if(functionName == "start_run") {
        start_run(parameter);
    }
    else if(functionName == "end_run") {
        end_run(parameter);
    }
    else if(functionName == "show_high") {
        show_high(parameter);
        drawReadyScreen(runInformation.device_id);
    }
    else if(functionName == "reset_sd") {
        reset_sd(parameter);
    }
    else if(functionName == "run_diagnostics") {
        run_diagnostics(parameter);
    }
    else if(functionName == "force_upload") {
        force_upload(parameter);
    }

}


void loop()
{
    // Check if the touch sensor was pressed
  if (touchDetected) {
    touchDetected = false;
    start_run("");  
  }

  if (speed_tracking_active) {
    if (myGNSS.getFixType() == 3) 
    {
      processRunState();
      //Serial.printf("Fix Type %d\n", myGNSS.getFixType());
    } else {
      drawGPSLockScreen(runInformation.device_id);
    }
  }

  String cmdString = readCardData(300);
  if (!cmdString.isEmpty()) {
    checkAndExecuteCommand(cmdString);
  }

  delay(50);
}