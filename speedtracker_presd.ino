#include <Adafruit_PN532.h>
#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>
#include <U8g2lib.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <WiFi.h>
#include <HTTPClient.h>

// // Status LED definitions
// const int LED_RED_OUTPUT_PIN = 32;
// const int LED_GREEN_OUTPUT_PIN = 25;
// const int LED_BLUE_OUTPUT_PIN = 26;
// const int LED_VCC_PIN = 33;

// Kevin special device LED pin mappings
const int LED_RED_OUTPUT_PIN = 25;
const int LED_GREEN_OUTPUT_PIN = 27;
const int LED_BLUE_OUTPUT_PIN = 14;
const int LED_VCC_PIN = 26;

// BAMF special values
#define BAMF_X_POSITION               120
#define BAMF_Y_POSITION               6
#define BAMF_CHAR                     "B"

#define RUN_CONFIG_FILE_PATH          "/run_config.txt"
#define RUN_CONFIG_TEMP_FILE_PATH     "/tmp_run_config.txt"
#define RUN_DATA_FILE_PATH            "/run_data_"   //will concat device id and .txt

#define RUN_DATA_UPLOAD_PATH          "/upload_run_data"
#define RUN_RESULT_UPLOAD_PATH        "/upload_run_result"

// BAMF special values
#define BAMF_X_POSITION               120
#define BAMF_Y_POSITION               6
#define BAMF_CHAR                     "B"


// Enum type to represent each of the screens
enum Screen {
    ATTENTION_SCREEN,
    GPS_LOCK_SCREEN,
    READY_SCREEN,
    MAIN_SCREEN,
    BOOTING_SCREEN,
    SUMMARY_SCREEN
};

// Global variable to store the current screen
Screen currentScreen;

// Function to redraw the current screen
void redrawCurrentScreen(String attention_string, String device_id, double speed, double topSpeed) {
    switch (currentScreen) {
        case ATTENTION_SCREEN:
            drawAttentionScreen(attention_string);
            break;
        case GPS_LOCK_SCREEN:
            drawGPSLockScreen(device_id);
            break;
        case READY_SCREEN:
            drawReadyScreen(device_id);
            break;
        case MAIN_SCREEN:
            drawMainScreen(device_id, speed);
            break;
        case BOOTING_SCREEN:
            drawBootingScreen();
            break;
        case SUMMARY_SCREEN:
            drawSummaryScreen(topSpeed);
            break;
    }
}

String RunDataFileName;
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
  String upload_server_ip;
  String upload_server_ssid;
  String upload_server_password;
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

SFE_UBLOX_GNSS myGNSS;

String getDeviceFullName() {
  return runInformation.device_id + '-' + myGNSS.getSIV();
}

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
//volatile bool touchDetected = false; // Flag that is set when touch sensor is activated
volatile bool speed_tracking_active = false; // Flag to indicate that the location and speed are being tracked
unsigned long lastMillis = 0; // Counter to track logging times
//const unsigned long debounceDelay = 150; // Debounce delay in milliseconds
//volatile unsigned long lastDebounceTime = 0; // Stores the last time the button was pressed

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


//
// Detect finish line crossing by linear bisection method (draw a line from left to right finish line and see if the line from previous to current position intersects with it)
//
bool crossFinishLine(struct GpsCoordinate finishLineLeft, struct GpsCoordinate finishLineRight, struct GpsCoordinate vehiclePreviousPosition, struct GpsCoordinate vehicleCurrentPosition) {
  double denominator = ((finishLineRight.latitude - finishLineLeft.latitude) * (vehicleCurrentPosition.longitude - vehiclePreviousPosition.longitude) - (finishLineRight.longitude - finishLineLeft.longitude) * (vehicleCurrentPosition.latitude - vehiclePreviousPosition.latitude));
  double numerator1 = ((finishLineRight.longitude - finishLineLeft.longitude) * (vehiclePreviousPosition.latitude - finishLineLeft.latitude) - (finishLineRight.latitude - finishLineLeft.latitude) * (vehiclePreviousPosition.longitude - finishLineLeft.longitude));
  double numerator2 = ((vehicleCurrentPosition.longitude - vehiclePreviousPosition.longitude) * (vehiclePreviousPosition.latitude - finishLineLeft.latitude) - (vehicleCurrentPosition.latitude - vehiclePreviousPosition.latitude) * (vehiclePreviousPosition.longitude - finishLineLeft.longitude));

  if (denominator == 0) return numerator1 == 0 && numerator2 == 0;

  double r = numerator1 / denominator;
  double s = numerator2 / denominator;

  return (r >= 0 && r <= 1) && (s >= 0 && s <= 1);
}

void drawAttentionScreen(String attentionString) {

  u8g2.clearBuffer(); // clear the buffer
 
  u8g2.setFont(u8g2_font_fub11_tr); // set font size to 8

  char speedStr[20];
  
 
  if (!runInformation.bamf_speed) {
      sprintf(speedStr, "%s - %06.2f mph", runInformation.device_id, runInformation.high_speed); // add leading zeros if needed
    } else {
      sprintf(speedStr, "%s - %06.2f mph", runInformation.device_id, runInformation.bamf_speed); // add leading zeros if needed
    } 

  u8g2.drawStr(0, 20, speedStr);
   
  u8g2.drawStr(0, 55, attentionString.c_str());

  // Check bamf status
  if (runInformation.bamf_speed > 0) {
    u8g2.drawStr(BAMF_X_POSITION, BAMF_Y_POSITION, BAMF_CHAR);
  }

  u8g2.sendBuffer();
}



void drawGPSLockScreen(String device_id) {

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

  while (!myGNSS.getGnssFixOk() || (myGNSS.getSIV() < 4) || (myGNSS.getSIV() > 50) || (myGNSS.getFixType() < 3)) {
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

 if (speed_tracking_active) {
      ledEnableBlue(); 
    } else {
      ledEnableGreen();
    }
}



void drawReadyScreen(String device_id) {
  currentScreen = READY_SCREEN;

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
  currentScreen = MAIN_SCREEN;
  u8g2.clearBuffer(); // clear the buffer
  
  // Top section
  u8g2.setFont(u8g2_font_ncenB08_tr); // set font size to 8
  u8g2.drawStr(0, 10, deviceId.c_str()); // draw full name
  
  // Speed section
  u8g2.setFont(u8g2_font_fub20_tr); // set font size to 30
  char speedStr[7] = {0};
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
  currentScreen = SUMMARY_SCREEN;
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

#define HISTORY_SIZE 20

struct GpsCoordinate history[HISTORY_SIZE];
int historyIndex = 0;

void addValueToPositionHistory(struct GpsCoordinate value) {
  history[historyIndex] = value;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

struct GpsCoordinate GetPositionFromHistory(int zeroBasedIndex)
{
  int actualIndex = (HISTORY_SIZE + historyIndex - zeroBasedIndex) % HISTORY_SIZE;
  return history[actualIndex];
}

// returns true if the finish line has been crossed, false otherwise
bool processRunState()
{  
  double mph = getGPSSpeed();
  double latitude = getLatitudeDegrees();
  double longitude = getLongitudeDegrees();

  struct GpsCoordinate curPosition = { latitude, longitude };
  struct GpsCoordinate prevPosition = GetPositionFromHistory(5); // get the position 5 samples ago
  addValueToPositionHistory(curPosition);

  if (mph > 300) {
    mph = 0;
  }

  if (mph < 3.00) {
    drawMainScreen(getDeviceFullName(), 0.00);
  } else {
    drawMainScreen(getDeviceFullName(), mph);
  }

  if (((millis() - lastMillis) > 500) ||
      (lastMillis <= 0)) {
    lastMillis = millis();

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

  if (mph < 3.00) {
    drawMainScreen(getDeviceFullName(), 0.00);
  } else {
    drawMainScreen(getDeviceFullName(), mph);
  }

  // Update the max speed if needed
  if (mph > runInformation.high_speed) {
    runInformation.high_speed = mph;
  }

  return crossFinishLine(
      runInformation.finishLine_left, 
      runInformation.finishLine_right,
      prevPosition,
      curPosition);
}


//Function to save the stInfo array to a JSON file on the SD card
void saveSpeedTrackerInfoToSD() {
  
    //char filedata[SPEEDTRACKER_INFO_MAX_ENTRIES * (sizeof(SPEEDTRACKER_INFO) + 3)];

  // Open the file for writing
  File file = SD.open(RunDataFileName, FILE_APPEND);
  if (!file) {
    Serial.printf("Failed to open file %s for writing.", RunDataFileName);
    Serial.println();
    return;
  } else {
    Serial.printf("Opened file %s for writing.", RunDataFileName);                        
    Serial.println();
  }

  // Add stInfo array data to the JSON array
  for (int i = 0; i < SPEEDTRACKER_INFO_MAX_ENTRIES; i++) {
    if (stInfo[i].latitude) {
      file.printf("%s,%10.7f,%10.7f,%10.7f\n", runInformation.device_id.c_str(), stInfo[i].latitude, stInfo[i].longitude, stInfo[i].mph);
      Serial.printf("%s,%10.7f,%10.7f,%10.7f\n", runInformation.device_id.c_str(), stInfo[i].latitude, stInfo[i].longitude, stInfo[i].mph);
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
      if (value.isEmpty()) {
          
        // Get MAC address
        uint8_t baseMac[6];
        // Get base MAC address
        esp_read_mac(baseMac, ESP_MAC_WIFI_STA);

        // Convert MAC address to string
        char macStr[13] = { 0 };
        sprintf(macStr, "%2X%2X%2X%2X%2X%2X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);

        runInformation.device_id = macStr;
      } else {
        runInformation.device_id = value;
      }
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
    } else if (key == "upload_server_ip") {
      runInformation.upload_server_ip = value;
    } else if (key == "upload_server_password") {
      runInformation.upload_server_password = value;
    } else if (key == "upload_server_ssid") {
      runInformation.upload_server_ssid = value;
    } else {
      // Unknown key
    }
  }

   // Close file
  configFile.close();

  return true;
}

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
  myGNSS.setNavigationFrequency(2); //Produce two solutions per second
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

  if (!loadRunConfig()) {
    u8g2.drawStr(10, 10, "No run info on SD card");
  }

  //
  // Test GPS connection
  //
 
  drawGPSLockScreen(getDeviceFullName());
  
  ledEnable(LED_GREEN_OUTPUT_PIN); 
  drawReadyScreen(getDeviceFullName());

  RunDataFileName = RUN_DATA_FILE_PATH + runInformation.device_id + ".txt";
}

String getRunData() {
  // Open the file
  File file = SD.open(RunDataFileName);  // Replace with your file path
  String fileData;

  if (file) {
    // Read and print each line of the file
    while (file.available()) {
      fileData += file.readStringUntil('\n') + '\n';
    }

    // Close the file
    file.close();
  }
  return fileData;
}

bool uploadRunResultsAndData() {
  int httpResponseCode = 0;

  Serial.printf("SSID:%s PW:%s\n", runInformation.upload_server_ssid.c_str(), runInformation.upload_server_password.c_str());
  drawAttentionScreen("WiFi...");
  WiFi.begin(runInformation.upload_server_ssid, runInformation.upload_server_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  bool successfulUpload = false;
  int attemptCount = 0;
  

  while (!successfulUpload)
  {
    attemptCount++;
    drawAttentionScreen("Results " + attemptCount + "(" + WiFi.status() + "," + httpResponseCode + ")");
    delay(1000);

    // Check WiFi connection status
    if(WiFi.status() == WL_CONNECTED){   
      HTTPClient http;
      
      // Specify destination for HTTP request
      http.begin(runInformation.upload_server_ip + RUN_RESULT_UPLOAD_PATH);

      // Specify content-type header as text/plain
      http.addHeader("Content-Type", "text/plain");
      
      // Send HTTP POST request
      httpResponseCode = http.POST(runInformation.device_id + ", " + runInformation.high_speed);
      if (httpResponseCode >= 200 && httpResponseCode <= 299)
      {
        successfulUpload = true;
      }

      // Free resources
      http.end();
    }
  }

  successfulUpload = false;
  attemptCount = 0;

  while (!successfulUpload)
  {
    attemptCount++;
    drawAttentionScreen("Data " + attemptCount + "(" + WiFi.status() + "," + httpResponseCode + ")");
    delay(1000);

    // Check WiFi connection status
    if(WiFi.status() == WL_CONNECTED){   
      HTTPClient http;
      
      // Specify destination for HTTP request
      http.begin(runInformation.upload_server_ip + RUN_DATA_UPLOAD_PATH);

      // Specify content-type header as text/plain
      http.addHeader("Content-Type", "text/plain");
      
      // Send HTTP POST request
      httpResponseCode = http.POST(getRunData());

    
      if (httpResponseCode >= 200 && httpResponseCode <= 299) {
        drawAttentionScreen("Run Data uploaded...");
        Serial.println("Data uploaded..");
        successfulUpload = true;
      } else {
        Serial.printf("Error %d uploading data\n", httpResponseCode);
        drawAttentionScreen("Error uploading" + httpResponseCode + "\n");
      }
      
      // Free resources
      http.end();
    }
  }

  redrawCurrentScreen("", getDeviceFullName(), 0.00, runInformation.high_speed);
  return (httpResponseCode == 200);
}

void device_id(String parameter) {
    Serial.printf("Running device_id(%s) function", parameter.c_str());
    runInformation.device_id = parameter;
    redrawCurrentScreen("", getDeviceFullName(), 0.00, runInformation.high_speed);
}

void bamf_speed(String parameter) {
    Serial.printf("Running bamf_speed(%s) function", parameter.c_str());
    runInformation.bamf_speed = parameter.toDouble();
    redrawCurrentScreen("",getDeviceFullName() , 0.00, runInformation.high_speed);
}

void start_run(String parameter) {
  Serial.println("Running start_run function");

  if (speed_tracking_active != true) {
    runInformation.high_speed = 0;
    speed_tracking_active = true;    
    ledEnableBlue();      
  }
}

void end_run(String parameter) {
  Serial.println("Running end_run function");
  if (speed_tracking_active != false) {
    // Reset globals      
    speed_tracking_active = false;
    lastMillis = 0;
    ledEnableGreen();

    if (!runInformation.bamf_speed) {
      drawSummaryScreen(runInformation.high_speed);
    } else {
      drawSummaryScreen(runInformation.bamf_speed);
    }
    
    saveSpeedTrackerInfoToSD();

    uploadRunResultsAndData();

    drawReadyScreen(getDeviceFullName());
  } 
}

void abort_run(String parameter) {
  Serial.println("Running abort_run function");

  if (speed_tracking_active) {
    runInformation.high_speed = 0;
    speed_tracking_active = false;    
    ledDisableAll();      
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
    uploadRunResultsAndData();
    // Your function's code here
}

void upload_server_ip(String parameter) {
    Serial.println("Running upload_server_ip function");
    runInformation.upload_server_ip = parameter;
    drawAttentionScreen("IP - " + parameter);
    redrawCurrentScreen("", getDeviceFullName(), 0.00, runInformation.high_speed);
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
    else if(functionName == "device_id") {
        device_id(parameter);
    }
    else if(functionName == "start_run") {
        start_run(parameter);
    }
    else if(functionName == "end_run") {
        end_run(parameter);
    }
    else if(functionName == "show_high") {
        show_high(parameter);
        drawReadyScreen(getDeviceFullName());
    }
    else if(functionName == "abort_run") {
        abort_run(parameter);
        ledEnableGreen();
        drawReadyScreen(getDeviceFullName());
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
    else if(functionName == "upload_server_ip") {
        upload_server_ip(parameter);
    }

}

void loop()
{

  if (speed_tracking_active) {
    if (myGNSS.getFixType() >= 3) 
    {
      bool isFinishLineCrossed = processRunState();
      if (isFinishLineCrossed)
      {
        end_run("");
      }
      //Serial.printf("Fix Type %d\n", myGNSS.getFixType());
    } else {
      drawGPSLockScreen(getDeviceFullName());
      ledEnableBlue(); 
    }
  }

  String cmdString = readCardData(50);

  if (!cmdString.isEmpty()) {
    checkAndExecuteCommand(cmdString);
  }
}