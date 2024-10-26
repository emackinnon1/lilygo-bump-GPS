// Select your modem
// for https updates
#define TINY_GSM_MODEM_SIM7000SSL
// for http updates
// #define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
#define SerialAT Serial1

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>

#include "secrets.h"

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
#define LOGGING  // <- Logging is for the HTTP library

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// set GSM PIN, if any
#define GSM_PIN ""

// flag to force SSL client authentication, if needed
#define TINY_GSM_SSL_CLIENT_AUTHENTICATION

String FINALLATI = "0", FINALLOGI = "0", FINALSPEED = "0", FINALALT = "0", FINALACCURACY = "0", FINALIGNITION = "false", ignition = "false";
float battery = 0.0;

// Set your APN Details / GPRS credentials
const char apn[]      = "hologram";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server details
const int  port       = 443;
const char server[]   = "unique-home-assistant-nabu-casa-url.ui.nabu.casa"; // your unique nabu casa url
const char resource[] = "/api/states/input_boolean.lilygo_bump_status";
const char batteryResource[] = "/api/states/sensor.lilygo_battery";

const int traccarPort        = 5055;
const char traccarServer[]   = "XX.XX.XX.XX"; // your unique traccar server ip
String myTraccarID = "lilygobumpgps";
// const char server[]   = "httpbin.org"; // B - WORKING POST
// const char resource[] = "/post"; // B - WORKING POST
// const char resource[] = "/get"; // B - WORKING GET
// String resource = "/post"; // B - WORKING POST

TinyGsm modem(SerialAT);

// for bump updates
TinyGsmClientSecure haClient(modem);
// for traccar updates
TinyGsmClient traccarClient(modem);


// LilyGO T-SIM7000G Pinout
#define UART_BAUD           115200
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13
#define LED_PIN             12
#define BAT_ADC             35
#define INT_PIN             32
#define ALARM_PIN           33

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 1500 // 25 minutes

Adafruit_MPU6050 mpu;

esp_sleep_wakeup_cause_t WAKEUP_REASON;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void wakeup_routine(){
  // esp_sleep_wakeup_cause_t wakeup_reason;

  // wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(WAKEUP_REASON)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : SerialMon.println("Wakeup caused by external signal using RTC_IO"); parse_movement_data(); sendBumpStatus(); sendBatteryData(); break;
    case ESP_SLEEP_WAKEUP_EXT1 : SerialMon.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : SerialMon.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : SerialMon.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : SerialMon.println("Wakeup caused by ULP program"); break;
    default : SerialMon.printf("Wakeup was not caused by deep sleep: %d\n",WAKEUP_REASON); break;
  }
}

void sound_alarm(int count, int delay_amount) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ALARM_PIN, HIGH);   // turn the alarm relay on (HIGH is the voltage level
    delay(delay_amount);           // wait for a period of time
    digitalWrite(ALARM_PIN, LOW);    // turn the alarm relay off by making the voltage LOW
    delay(delay_amount);  
  }
  digitalWrite(ALARM_PIN, LOW);
}


void parse_movement_data() {

  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    SerialMon.print("AccelX:");
    SerialMon.print(a.acceleration.x);
    SerialMon.print(",");
    SerialMon.print("AccelY:");
    SerialMon.print(a.acceleration.y);
    SerialMon.print(",");
    SerialMon.print("AccelZ:");
    SerialMon.print(a.acceleration.z);
    SerialMon.print(", ");
    SerialMon.print("GyroX:");
    SerialMon.print(g.gyro.x);
    SerialMon.print(",");
    SerialMon.print("GyroY:");
    SerialMon.print(g.gyro.y);
    SerialMon.print(",");
    SerialMon.print("GyroZ:");
    SerialMon.print(g.gyro.z);
    SerialMon.println("");
    
    flash_led(5, 100);
  }
}

void flash_led(int count, int delay_amount) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level
    delay(delay_amount);           // wait for a period of time
    digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
    delay(delay_amount);  
  }
  digitalWrite(LED_PIN, LOW);
}

// ============================ ALTERNATIVE BATTERY READING ============================
// void sortData(float data[], int8_t size) {             // Sorts all data small to large
//   float swapper;
//   for (int8_t i = 0; i < (size - 1); i++) {
//     for (int8_t o = 0; o < (size - (i + 1)); o++) {
//       if (data[o] > data[o + 1]) {
//         swapper = data[o];
//         data[o] = data[o + 1];
//         data[o + 1] = swapper;
//       }
//     }
//   }
// }

// float mapBatt(float x, float in_min, float in_max, float out_min, float out_max) {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

// float read_bat() {         // reads and returns the battery voltage
//   int BP = 0;
//   float v_bat = 0;
//   const int VReads = 15;   
//   float voltageBuffer[VReads];
//   uint32_t Read_buffer = 0;
//   for (int x = 0; x < VReads; x++) {
//     for (int i = 0 ; i < VReads; i++) {
//       voltageBuffer[i] = (uint32_t)analogRead(BAT_ADC);
//     }
//     sortData(voltageBuffer, VReads);
//     Read_buffer += (voltageBuffer[(VReads - 1) / 2]);
//   }
//   v_bat = (((float)(Read_buffer / VReads) / 4096) * 3600 * 2) / 1000;
//   BP = mapBatt(v_bat,2.5,4.2,0,100);     // get battery voltage as a percentage 0-100%
//   if (BP < 0) { BP = 0; }
//   return BP;
// }

// ============================ ALTERNATIVE BATTERY READING ============================

float ReadBattery() {
  float vref = 1.100;
  uint16_t volt = analogRead(BAT_ADC);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

void modemPowerOn(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500);
  digitalWrite(PWR_PIN, HIGH);
}

void modemRestart(){
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

void enableGPS(void) {
  SerialMon.println("Start positioning. Make sure to locate outdoors.");
  SerialMon.println("The blue indicator light flashes to indicate positioning.");
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.print("Something went wrong when enabling GPS.");
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
}

void disableGPS(void) {
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
}

void sendData(float lat, float lon, float speed, float alt, float accuracy, float battery) {
  HttpClient traccarHttp(traccarClient, traccarServer, traccarPort);

  String FINALLATI = "0", FINALLOGI = "0", FINALSPEED = "0", FINALALT = "0", FINALACCURACY = "0", FINALBAT = "0", FINALIGNITION = "false", FINALBATLEVEL = "";
  FINALLATI = String(lat, 8);
  FINALLOGI = String(lon, 8);
  FINALSPEED = String(speed, 2);
  FINALALT = String(alt, 0);
  FINALACCURACY = String(accuracy, 2);
  FINALIGNITION = String(ignition);
  if (battery == 0) {
    FINALBATLEVEL = "";
    FINALBAT = "";
    FINALIGNITION = "true";
  } else {
    float batterylevel = (((float)battery - 3) / 1.2) * 100;
    FINALBAT = String(battery, 2);
    if (batterylevel > 100) {
      batterylevel = 100;
    }
    FINALBATLEVEL = String(batterylevel, 0);
    FINALIGNITION = "false";
  }

  SerialMon.print(F("Performing HTTPS POST request to Traccar... "));
  int err = traccarHttp.post("/?id=" + myTraccarID + "&lat=" + FINALLATI + "&lon=" + FINALLOGI + "&accuracy=" + FINALACCURACY + "&altitude=" + FINALALT + "&speed=" + FINALSPEED + "&battery=" + FINALBAT + "&ignition=" + FINALIGNITION + "&batteryLevel=" + FINALBATLEVEL);
  Serial.println("/?id=" + myTraccarID + "&lat=" + FINALLATI + "&lon=" + FINALLOGI + "&accuracy=" + FINALACCURACY + "&altitude=" + FINALALT + "&speed=" + FINALSPEED + "&battery=" + FINALBAT + "&ignition=" + FINALIGNITION + "&batteryLevel=" + FINALBATLEVEL);
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    delay(1000);
  }

  int status = traccarHttp.responseStatusCode();
  SerialMon.print("Status code: ");
  SerialMon.println(status);
  if (!status) {
    delay(10000);
  }

  // Shutdown
  traccarHttp.stop();
  SerialMon.println(F("Server disconnected bye bye will connect soon"));
}

void dispatchGPSData() {
  // sends the GPS location 
  enableGPS();

  float lat, lon, speed, alt, accuracy;
  int vsat, usat, year, month, day, hour, min, sec;
  SerialMon.println("get GPS attempt");
  while (1) {
    battery = ReadBattery();
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {
      SerialMon.println("get GPS worked");
      SerialMon.printf("lat:%f lon:%f\n", lat, lon);
      sendData(lat, lon, speed, alt, accuracy, battery);
      break;
    } else {
      Serial.print("get GPS reattempting...");
      Serial.println(millis());
    }
  }

  if (battery == 0) {
    delay(5000);
  } else {
    int count = 0;
    while ((battery > 0) and (count < 90)) {
      battery = ReadBattery();
      delay(1000);
      count++;
    }
  }
}

void sendBatteryData() {
  // read battery
  battery = ReadBattery();
  String FINALBATLEVEL = "", FINALBAT = "0";

  if (battery == 0) {
    delay(5000);
  } else {
    int count = 0;
    while ((battery > 0) and (count < 90)) {
      battery = ReadBattery();
      delay(1000);
      count++;
    }
  }

  if (battery == 0) {
    FINALBATLEVEL = "";
    FINALBAT = "";
  } else {
    float batterylevel = (((float)battery - 3) / 1.2) * 100;
    FINALBAT = String(battery, 2);
    if (batterylevel > 100) {
      batterylevel = 100;
    }
    FINALBATLEVEL = String(batterylevel, 0);
  }

  // send battery status to Home Assistant
  SerialMon.print("Battery level: ");
  SerialMon.print(FINALBATLEVEL);
  SerialMon.print("Battery voltage: ");
  SerialMon.print(FINALBAT);
  String jsonBody = "{\"state\": \"" + FINALBATLEVEL +"\"}";
  postHomeAssistant(batteryResource, jsonBody);
}

void postHomeAssistant(const char* resourceToPost, String body) {
  HttpClient haHttp(haClient, server, port);
  String contentType = "application/json";

  SerialMon.print(F("Performing HTTPS POST request to HA... "));
  SerialMon.print("Resource: ");
  SerialMon.println(resourceToPost);
  haHttp.connectionKeepAlive();  // Currently, this is needed for HTTPS
  haHttp.beginRequest();

  int err = haHttp.post(resourceToPost);
  haHttp.sendHeader("Authorization", BEARER_TOKEN);
  haHttp.sendHeader(F("Content-Type"), F("application/json"));
  haHttp.sendHeader(F("Content-Length"), body.length());
  haHttp.beginBody();
  haHttp.println(String(body));
  haHttp.endRequest();

  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    delay(10000);
    return;
  }

  int status = haHttp.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if (!status) {
    delay(1000);
    return;
  }

  int length = haHttp.contentLength();
  if (length >= 0) {
    SerialMon.print(F("Content length is: "));
    SerialMon.println(length);
  }
  if (haHttp.isResponseChunked()) {
    SerialMon.println(F("The response is chunked"));
  }
  String responsebody = haHttp.responseBody();
  SerialMon.println(F("Response:"));
  SerialMon.println(responsebody);

  int responseCode = haHttp.responseStatusCode();
  SerialMon.println(F("Response status:"));
  SerialMon.println(responseCode);

  SerialMon.print(F("Body length is: "));
  SerialMon.println(responsebody.length());

  // Shutdown
  haHttp.stop();
  SerialMon.println(F("Server disconnected"));
}

void sendBumpStatus() {
  String jsonBody = "{\"state\": \"on\"}";
  postHomeAssistant(resource, jsonBody);


  // HttpClient haHttp(haClient, server, port);
  // String contentType = "application/json";
  // String json = "{\"state\": \"on\"}";

  // SerialMon.print(F("Performing HTTPS POST request to HA... "));
  // haHttp.connectionKeepAlive();  // Currently, this is needed for HTTPS
  // haHttp.beginRequest();

  // int err = haHttp.post(resource);
  // haHttp.sendHeader("Authorization", BEARER_TOKEN);
  // haHttp.sendHeader(F("Content-Type"), F("application/json"));
  // haHttp.sendHeader(F("Content-Length"), json.length());
  // haHttp.beginBody();
  // haHttp.println(String("{\"state\": \"on\"}"));
  // haHttp.endRequest();

  // if (err != 0) {
  //   SerialMon.println(F("failed to connect"));
  //   delay(10000);
  //   return;
  // }

  // int status = haHttp.responseStatusCode();
  // SerialMon.print(F("Response status code: "));
  // SerialMon.println(status);
  // if (!status) {
  //   delay(1000);
  //   return;
  // }

  // int length = haHttp.contentLength();
  // if (length >= 0) {
  //   SerialMon.print(F("Content length is: "));
  //   SerialMon.println(length);
  // }
  // if (haHttp.isResponseChunked()) {
  //   SerialMon.println(F("The response is chunked"));
  // }
  // String responsebody = haHttp.responseBody();
  // SerialMon.println(F("Response:"));
  // SerialMon.println(responsebody);

  // int responseCode = haHttp.responseStatusCode();
  // SerialMon.println(F("Response status:"));
  // SerialMon.println(responseCode);

  // SerialMon.print(F("Body length is: "));
  // SerialMon.println(responsebody.length());

  // // Shutdown
  // haHttp.stop();
  // SerialMon.println(F("Server disconnected"));
}


void setup() {
  // Set Serial Monitor baud rate
  SerialMon.begin(115200);
  delay(10);

  // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  pinMode(ALARM_PIN, OUTPUT);
  WAKEUP_REASON = esp_sleep_get_wakeup_cause();
  if (WAKEUP_REASON == ESP_SLEEP_WAKEUP_EXT0) {
    sound_alarm(3, 300);
  }
  flash_led(3, 300);

  modemPowerOn();

  SerialMon.println("Wait...");

  // Set GSM module baud rate and Pins
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  if (!modem.init()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }

  Serial.println("Make sure your LTE antenna has been connected to the SIM interface on the board.");
  delay(10000);


  // MPU6050 setup
  SerialMon.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    SerialMon.println("Failed to find MPU6050 chip");
      flash_led(10, 100);
      // esp_restart();
  } else {
    SerialMon.println("MPU6050 Found!");
  }

  //setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(10);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

void loop() {
  modem.gprsConnect(apn, gprsUser, gprsPass);

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }
  wakeup_routine();

  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));

  // --------POWER DOWN--------
  // Try to power-off (modem may decide to restart automatically)
  // To turn off modem completely, please use Reset/Enable pins
  modem.sendAT("+CPOWD=1");
  if (modem.waitResponse(10000L) != 1) {
    DBG("+CPOWD=1");
  }
  // The following command does the same as the previous lines
  Serial.println("Poweroff and go to sleep.");
  flash_led(3, 500);
  modem.poweroff();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 0);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  esp_deep_sleep_start();
}
