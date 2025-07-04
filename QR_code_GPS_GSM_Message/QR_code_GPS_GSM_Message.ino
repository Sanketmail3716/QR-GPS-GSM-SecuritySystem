// GSM, GPS , Qrcode, buzzer are interface 
// add message 
#include <HardwareSerial.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "quirc.h"
#include <SoftwareSerial.h>

// -------- GPS Setup --------
HardwareSerial gpsSerial(1);
const int RXPin = 2;
#define BUZZER_PIN 13
#define SWITCH_PIN 12   // GPIO12 connected to push-button
#define LED_PIN 4       // GPIO4 connected to onboard flash LEDmode:DIO, clock div:1load:0x3fff0030,len:4888load:0x40010000,len:0load:0x0f352000,len:259981760ets Jul 29 2019 12:21:46rst:0x7 (TG0WDT_SYS_RESET),boot:0x33 (SPI_FAST_FLASH_BOOT)configsip: 0, SPIWP:0xeeclk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00mode:DIO, clock div:1load:0x3fff0030,len:4888load:0x40010000,len:0load:0x0f352000,len:259981760ets Jul 29 2019 12:21:46rst:0x7 (TG0WDT_SYS_RESET),boot:0x33 (SPI_FAST_FLASH_BOOT)

struct GPSRawData {
  double latitude;
  char latitudeDir;
  double longitude;
  char longitudeDir;
  int satellites;
  double altitude;
  int hours, minutes, seconds;
  int day, month, year;
  String timestamp;
};

struct GPSData {
  double latitude;
  double longitude;
  String timestamp;
};

String incomingData = "";
bool messageReceived = false;                                                   
bool gpsDataValid = false;
GPSData latestGPSData;
GPSRawData latestGPSRawData;

// -------- SIM800L Setup --------
#define SerialMon Serial
#define MODEM_TX 14  // ESP32-CAM TX → SIM800L RX
#define MODEM_RX 15  // ESP32-CAM RX ← SIM800L TX
SoftwareSerial SerialAT(MODEM_RX, MODEM_TX);
const char phoneNumber[] = "8010084268";
bool smsSent = false;

// -------- Camera Setup --------
TaskHandle_t QRCodeReader_Task;

#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

struct quirc *q = NULL;
uint8_t *image = NULL;  
camera_fb_t * fb = NULL;
struct quirc_code code;
struct quirc_data data;
quirc_decode_error_t err;
String QRCodeResult = "";

// -------- Function Prototypes --------
void QRCodeReader(void * pvParameters);
void dumpData(const struct quirc_data *data);
void sendSMS(const char* message);
void sendAT(const char* cmd);
void processGPSData(String raw);
void parseGPGGA(String gpgga);
void parseGPRMC(String gprmc);
void convertAndPrintLocalDateTime();
double nmeaToDecimal(String nmeaCoord);
void sendThankYou(const char* number);

void setup() {
  SerialMon.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, RXPin);
  SerialMon.println("System Starting...");

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Prevent brownout

 

  // SIM800 init
  SerialAT.begin(9600);
  delay(2000);
  sendAT("AT");
  sendAT("AT+CMGF=1");
  sendAT("AT+CSCS=\"GSM\"");
  sendAT("AT+CNMI=1,2,0,0,0");
  pinMode(BUZZER_PIN, OUTPUT);
  
  pinMode(SWITCH_PIN, INPUT_PULLUP);
 // Use internal pull-up, button connects to GND
  pinMode(LED_PIN, OUTPUT);          // LED as output


  // Camera config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    ESP.restart();
  }

  // Start QR Task
  xTaskCreatePinnedToCore(
    QRCodeReader,
    "QRCodeReader_Task",
    10000,
    NULL,
    1,
    &QRCodeReader_Task,
    0
  );

  SerialMon.println("Setup Complete.");
}

void loop() {
 // Switch
 int switchState = digitalRead(SWITCH_PIN);  // Read button state

if (switchState == LOW) {  // If button is pressed (active LOW)
  digitalWrite(LED_PIN, HIGH);  // Turn on LED
  Serial.println("Button Pressed - LED ON");
  sendSMS("🔔 Access request: Someone is requesting entry. Please reply with 'GRANTED' to approve.");
} else {
  digitalWrite(LED_PIN, LOW);   // Turn off LED
}


  delay(100); // Small delay to avoid serial flooding


 // Switch Completee




// change 


 if (SerialAT.available()) {
    char c = SerialAT.read();
    incomingData += c;
    SerialMon.write(c); // Show incoming on Serial Monitor

    // Check for "GRANTED" in message
    if (incomingData.indexOf("GRANTED") != -1) {
        digitalWrite(LED_PIN, HIGH);  
      SerialMon.println("\n✅ GRANTED message received!");

      sendThankYou(phoneNumber);
      incomingData = "";  // Clear buffer
    }

    // Reset buffer if it gets too long
    if (incomingData.length() > 300) {
      incomingData = "";
    }
  }



// changes complte


  static String gpsData = "";
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gpsData += c;
    if (c == '\n') {
      processGPSData(gpsData);
      gpsData = "";
    }
  }
}

void sendThankYou(const char* number) {
  SerialMon.print("Sending Thank You to ");
  SerialMon.println(number);

  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(number);
  SerialAT.println("\"");
  delay(1000);  // Wait for '>'

  SerialAT.print("Thank you");
  SerialAT.write(26); // CTRL+Z to send
  delay(5000);

  SerialMon.println("✅ Thank You message sent!");
}


void QRCodeReader(void * pvParameters){
  while(1){
    q = quirc_new();
    if (q == NULL) continue;

    fb = esp_camera_fb_get();
    if (!fb) continue;

    quirc_resize(q, fb->width, fb->height);
    image = quirc_begin(q, NULL, NULL);
    memcpy(image, fb->buf, fb->len);
    quirc_end(q);

    int count = quirc_count(q);
    if (count > 0) {
      quirc_extract(q, 0, &code);
      err = quirc_decode(&code, &data);

      if (!err) {
        dumpData(&data);
      } else {
        Serial.println("❌ QR Decode Failed");
      }
    }

    esp_camera_fb_return(fb);
    fb = NULL;
    image = NULL;
    quirc_destroy(q);
  }
}

void dumpData(const struct quirc_data *data)
{
  QRCodeResult = String((const char *)data->payload);
  QRCodeResult.trim();
  Serial.printf("QR Code: %s\n", QRCodeResult.c_str());

  if (QRCodeResult == "Sanket") {
    Serial.println("✅ Access Granted");
    digitalWrite(LED_PIN, HIGH); 
        
     digitalWrite(LED_PIN, LOW); 
    delay(5000);  
    smsSent = false;

    // Send GPS data when QR code is valid (Access granted)
    if (gpsDataValid) {  
      String gpsMessage = "Latitude: " + String(latestGPSData.latitude, 6) + 
                           "\nLongitude: " + String(latestGPSData.longitude, 6) + 
                           "\nTimestamp: " + latestGPSData.timestamp;
      sendSMS(gpsMessage.c_str());  // Send GPS coordinates
    }

    // Send an alert SMS for access granted condition
    sendSMS("Alert: Access granted, QR code scanned.");
    
  } else {
    Serial.println("❌ Access Denied");
    
    // Send GPS data even if access is denied
    if (gpsDataValid) {
      String gpsMessage = "Latitude: " + String(latestGPSData.latitude, 6) + 
                           "\nLongitude: " + String(latestGPSData.longitude, 6) + 
                           "\nTimestamp: " + latestGPSData.timestamp;
      sendSMS(gpsMessage.c_str());  // Send GPS coordinates
       
    }

    // Send an alert SMS for access denied condition
    if (!smsSent) {
        digitalWrite(BUZZER_PIN, HIGH);  // Turn buzzer ON
        delay(5000);
        digitalWrite(BUZZER_PIN, LOW);   // Turn buzzer OFF
        delay(5000);
      sendSMS("Car theft alert! Unauthorized QR code scan.");
      smsSent = true;  // Prevent multiple alert messages
    }
  }
}

void sendSMS(const char* message) {
  SerialMon.println("Sending SMS...");

  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(phoneNumber);
  SerialAT.println("\"");
  delay(1000);

  SerialAT.print(message);
  SerialAT.write(26);
  delay(5000);

  SerialMon.println("SMS sent!");
}

void sendAT(const char* cmd) {
  SerialAT.println(cmd);
  delay(1000);
  while (SerialAT.available()) {
    SerialMon.write(SerialAT.read());
  }
}

void processGPSData(String raw) {
  if (raw.startsWith("$GPGGA")) {
    parseGPGGA(raw);
    convertAndPrintLocalDateTime();
  } else if (raw.startsWith("$GPRMC")) {
    parseGPRMC(raw);
  }
}

void parseGPGGA(String gpgga) {
  String tokens[15];
  int tokenIndex = 0, startIndex = 0;
  for (int i = 0; i < gpgga.length(); i++) {
    if (gpgga[i] == ',' || gpgga[i] == '*') {
      tokens[tokenIndex++] = gpgga.substring(startIndex, i);
      startIndex = i + 1;
    }
  }

  if (tokenIndex > 1) {
    String utcTime = tokens[1];
    latestGPSRawData.hours = utcTime.substring(0, 2).toInt();
    latestGPSRawData.minutes = utcTime.substring(2, 4).toInt();
    latestGPSRawData.seconds = utcTime.substring(4, 6).toInt();
    latestGPSRawData.latitude = nmeaToDecimal(tokens[2]);
    latestGPSData.latitude = latestGPSRawData.latitude;
    latestGPSRawData.latitudeDir = tokens[3].charAt(0);
    latestGPSRawData.longitude = nmeaToDecimal(tokens[4]);
    latestGPSData.longitude = latestGPSRawData.longitude;
    latestGPSRawData.longitudeDir = tokens[5].charAt(0);
    latestGPSRawData.satellites = tokens[7].toInt();
    latestGPSRawData.altitude = tokens[9].toDouble();

    gpsDataValid = latestGPSRawData.satellites >= 4 &&
                   (latestGPSData.latitude != 0 || latestGPSData.longitude != 0);
  }
}

void parseGPRMC(String gprmc) {
  String tokens[15];
  int tokenIndex = 0, startIndex = 0;
  for (int i = 0; i < gprmc.length(); i++) {
    if (gprmc[i] == ',' || gprmc[i] == '*') {
      tokens[tokenIndex++] = gprmc.substring(startIndex, i);
      startIndex = i + 1;
    }
  }

  if (tokenIndex > 9) {
    String utcDate = tokens[9];
    latestGPSRawData.day = utcDate.substring(0, 2).toInt();
    latestGPSRawData.month = utcDate.substring(2, 4).toInt();
    latestGPSRawData.year = 2000 + utcDate.substring(4, 6).toInt();
  }
}

void convertAndPrintLocalDateTime() {
  int offsetHours = 5;
  int offsetMinutes = 30;
  latestGPSRawData.minutes += offsetMinutes;
  latestGPSRawData.hours += offsetHours;

  if (latestGPSRawData.minutes >= 60) {
    latestGPSRawData.minutes -= 60;
    latestGPSRawData.hours++;
  }

  if (latestGPSRawData.hours >= 24) {
    latestGPSRawData.hours -= 24;
    latestGPSRawData.day++;
  }

  char timeBuffer[20];
  snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
           latestGPSRawData.year, latestGPSRawData.month, latestGPSRawData.day,
           latestGPSRawData.hours, latestGPSRawData.minutes, latestGPSRawData.seconds);

  latestGPSRawData.timestamp = String(timeBuffer);
  latestGPSData.timestamp = latestGPSRawData.timestamp;
}

double nmeaToDecimal(String nmeaCoord) {
  if (nmeaCoord == "") return 0.0;
  double decimal = nmeaCoord.substring(0, nmeaCoord.indexOf('.') - 2).toDouble();
  double minutes = nmeaCoord.substring(nmeaCoord.indexOf('.') - 2).toDouble();
  return decimal + (minutes / 60);
}
