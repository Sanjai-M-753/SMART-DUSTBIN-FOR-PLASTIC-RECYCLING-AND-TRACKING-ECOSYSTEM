/* Full ESP32 sketch: HX711 + LoadCell + GPS (NEO-6M) + SIM800L (HTTP POST to Firebase) + I2C LCD
   Wiring (summary):
   - HX711 DT -> GPIO23, SCK -> GPIO22, VCC->5V, GND->GND
   - Load cell -> HX711 (red,black,green,white)
   - GPS TX -> GPIO16 (ESP RX), GPS RX -> GPIO17 (ESP TX)
   - SIM800L TX -> GPIO27, SIM800L RX -> GPIO26 ; SIM800L VCC -> 4.0-4.2V supply, GND -> GND
   - LCD (I2C) SDA -> GPIO21, SCL -> GPIO22, VCC->5V, GND->GND
   - Common ground required
*/

#include <HX711.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>

// ---------- PINS ----------
#define LOADCELL_DOUT 23
#define LOADCELL_SCK  22

// GPS on Serial1: RX=16, TX=17
#define GPS_RX_PIN 16  // GPS TX -> ESP32 RX (16)
#define GPS_TX_PIN 17  // GPS RX -> ESP32 TX (17)

// GSM on Serial2: RX=27, TX=26
#define GSM_RX_PIN 27  // SIM800L TX -> ESP32 RX (27)
#define GSM_TX_PIN 26  // SIM800L RX -> ESP32 TX (26)

// ---------- HARDWARE OBJECTS ----------
HX711 scale;
LiquidCrystal_I2C lcd(0x27, 16, 2); // change 0x27 if your module uses 0x3F
TinyGPSPlus gps;

// Use hardware serial ports on ESP32:
HardwareSerial SerialGPS(1); // UART1 for GPS
HardwareSerial SerialGSM(2); // UART2 for SIM800L

// ---------- CONFIG (edit these) ----------
const char* FIREBASE_HOST = "your-project-id-default-rtdb.firebaseio.com"; // WITHOUT https:// and without trailing slash
const char* FIREBASE_PATH = "/plasticData.json"; // we'll POST to http://<host><path>
const char* APN = "your_apn_here"; // <-- set your operator APN (e.g., "airtelgprs" or "internet")
const char* APN_USER = ""; // usually empty
const char* APN_PASS = ""; // usually empty

float calibration_factor = -7050.0; // <-- adjust after calibration

// threshold to send a record (grams)
const float SEND_THRESHOLD = 5.0;

// ---------- UTILS ----------
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 30UL * 1000UL; // minimum 30 seconds between posts to avoid spamming

// Function to send AT and wait for response (returns response as String)
String sendAT(String cmd, unsigned long timeout = 2000) {
  Serial.print("AT> "); Serial.println(cmd);
  while (SerialGSM.available()) SerialGSM.read(); // flush GSM rx

  SerialGSM.println(cmd);
  unsigned long t = millis();
  String resp = "";
  while (millis() - t < timeout) {
    while (SerialGSM.available()) {
      char c = SerialGSM.read();
      resp += c;
    }
  }
  Serial.print("AT< "); Serial.println(resp);
  return resp;
}

// helper: wait for substring in GSM response (timeout ms)
bool waitResponseContains(const String &s, unsigned long timeout = 3000) {
  unsigned long t = millis();
  String buff = "";
  while (millis() - t < timeout) {
    while (SerialGSM.available()) {
      buff += (char)SerialGSM.read();
      if (buff.indexOf(s) >= 0) {
        Serial.print("Found: "); Serial.println(s);
        return true;
      }
    }
  }
  Serial.print("WaitResp failed for: "); Serial.println(s);
  return false;
}

// Setup GPRS (SAPBR)
bool gprsOpen() {
  sendAT("AT"); delay(200);
  sendAT("ATE0"); // echo off
  delay(200);

  // Set bearer profile
  sendAT("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", 1500);
  delay(200);
  String cmdAPN = String("AT+SAPBR=3,1,\"APN\",\"") + APN + "\"";
  sendAT(cmdAPN, 1500);
  delay(200);
  if (String(APN_USER).length() > 0) {
    String cmdAPNUser = String("AT+SAPBR=3,1,\"USER\",\"") + APN_USER + "\"";
    sendAT(cmdAPNUser, 1500);
    delay(200);
    String cmdAPNPass = String("AT+SAPBR=3,1,\"PWD\",\"") + APN_PASS + "\"";
    sendAT(cmdAPNPass, 1500);
    delay(200);
  }
  sendAT("AT+SAPBR=1,1", 5000); // open bearer
  delay(2000);
  if (!waitResponseContains("OK", 5000)) return false;
  sendAT("AT+SAPBR=2,1", 2000);
  return true;
}

void gprsClose() {
  sendAT("AT+SAPBR=0,1", 2000);
}

// Send HTTP POST using SIM800L built-in HTTP AT commands
bool httpPostToFirebase(const String &jsonPayload) {
  // initialize HTTP service
  sendAT("AT+HTTPTERM", 1000); // ignore error
  delay(200);
  sendAT("AT+HTTPINIT", 2000);
  delay(200);
  sendAT("AT+HTTPPARA=\"CID\",1", 1000);
  delay(200);

  // Build URL: http://<host><path>
  String url = String("http://") + FIREBASE_HOST + FIREBASE_PATH;
  String cmdUrl = String("AT+HTTPPARA=\"URL\",\"") + url + "\"";
  sendAT(cmdUrl, 2000);
  delay(200);

  // Content type
  sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000);
  delay(200);

  // Data length and timeout (ms)
  int len = jsonPayload.length();
  String httpDataCmd = String("AT+HTTPDATA=") + String(len) + ",10000";
  sendAT(httpDataCmd, 2000);
  delay(200);
  // Now device expects the data block
  SerialGSM.print(jsonPayload);
  delay(500);

  // Perform HTTP POST
  sendAT("AT+HTTPACTION=1", 10000);
  // Wait for +HTTPACTION: 1,200,xxx
  if (!waitResponseContains("+HTTPACTION: 1,200", 15000)) {
    // Optionally read response for debugging
    sendAT("AT+HTTPREAD", 3000);
    sendAT("AT+HTTPTERM", 1000);
    return false;
  }

  // Read response (optional)
  sendAT("AT+HTTPREAD", 3000);
  delay(200);
  sendAT("AT+HTTPTERM", 1000);
  return true;
}

// wrapper to POST JSON: open GPRS if needed, then send
bool sendJsonViaGSM(const String &json) {
  Serial.println("Opening GPRS...");
  if (!gprsOpen()) {
    Serial.println("GPRS open failed");
    return false;
  }
  Serial.println("GPRS opened. Sending HTTP POST...");
  bool ok = httpPostToFirebase(json);
  gprsClose();
  Serial.print("Post result: "); Serial.println(ok ? "OK" : "FAILED");
  return ok;
}

// ---------- setup -----------
void setup() {
  Serial.begin(9600);
  // init hardware serials
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // GPS baud 9600
  SerialGSM.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN); // SIM800L default AT at 115200 (some modules 9600)

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");
  delay(800);

  // HX711
  scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
  scale.set_scale(calibration_factor);
  scale.tare();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Ready");
  Serial.println("Setup complete.");
}

// ---------- main loop ----------
void loop() {
  // Read GPS continuously
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    gps.encode(c);
  }

  // Read weight
  float weight = 0.0;
  if (scale.is_ready()) {
    weight = scale.get_units(5); // average 5 samples
  } else {
    Serial.println("Scale not ready");
  }

  // Get best available lat/lng
  double lat = 0.0, lng = 0.0;
  bool haveGPS = false;
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
    haveGPS = true;
  }

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("W:");
  lcd.print(weight, 2);
  lcd.print("g");
  lcd.setCursor(0,1);
  if (haveGPS) {
    lcd.print(String(lat, 5));
    lcd.print(",");
    lcd.print(String(lng, 5));
  } else {
    lcd.print("GPS: waiting");
  }

  Serial.print("Weight: "); Serial.print(weight,2); Serial.print(" g  ");
  if (haveGPS) {
    Serial.print("Lat: "); Serial.print(lat,6); Serial.print(" Lng: "); Serial.print(lng,6);
  } else {
    Serial.print("GPS: no fix");
  }
  Serial.println();

  // Decide to send (weight threshold and time interval)
  unsigned long now = millis();
  if (weight > SEND_THRESHOLD && (now - lastSend > SEND_INTERVAL_MS)) {
    // Build timestamp string (use GPS time if available)
    String timestr = "";
    if (gps.time.isValid() && gps.date.isValid()) {
      char buf[32];
      sprintf(buf, "%02d:%02d:%02d %02d/%02d/%04d",
              gps.time.hour(), gps.time.minute(), gps.time.second(),
              gps.date.day(), gps.date.month(), gps.date.year());
      timestr = String(buf);
    } else {
      timestr = String(millis()/1000) + "s";
    }

    // Build JSON payload
    String json = "{";
    json += "\"weight\":" + String(weight, 2) + ",";
    if (haveGPS) {
      json += "\"lat\":" + String(lat, 6) + ",";
      json += "\"lon\":" + String(lng, 6) + ",";
    } else {
      json += "\"lat\":null,";
      json += "\"lon\":null,";
    }
    json += "\"time\":\"" + timestr + "\"";
    json += "}";

    Serial.print("JSON: "); Serial.println(json);

    // Attempt to send via GSM
    bool ok = sendJsonViaGSM(json);
    if (ok) {
      lastSend = now;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Sent OK");
      lcd.setCursor(0,1);
      lcd.print(weight); lcd.print(" g");
    } else {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Send Failed");
    }
    delay(2000);
  }

  delay(500);
}
