#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

// ==== Wi-Fi Credentials ====
const char* ssid             = "CAESAR";
const char* password         = "Nova1234567";
const char* googleScriptURL  = "https://script.google.com/macros/s/AKfycbx_348bHuOhsDMRMCMeOZqxRue5xAvo-69OzB_aZvPvEqlAQ4ICj5o0xwcnB2xOP9AJ/exec";

// ==== Servo Pins ====
const int baseServoPin    = 18;
const int gripperServoPin = 17;
Servo baseServo;
Servo gripperServo;

// ==== Bin Sorting Angles ====
const int M1 = 0;
const int M2 = 45;
const int C1 = 180;
const int C2 = 135;
int miloCount = 0;
int chivitaCount = 0;

// ==== Gripper Servo Angles ====
const int gripperOpen  = 0;
const int gripperClose = 70;

// ==== LCD Setup ====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== Ultrasonic Sensor Pins ====
const int trigPin = 14;
const int echoPin = 27;

// ==== Conveyor Motor Pins ====
const int stepPin   = 25;
const int dirPin    = 26;
const int enablePin = 33;
bool conveyorRunning = true;

// ==== Feedback ====
const int greenLED  = 13;
const int redLED    = 4;
const int buzzerPin = 32;

// ==== Detection Storage ====
String lastDetectedObject = "";
String lastDetectedConfidence = "";
bool hasValidDetection = false;

// ==== Upload Queue ====
bool hasPendingUpload = false;
String pendingObject = "";
String pendingConfidence = "";
unsigned long lastUploadTime = 0;

// ==== Distance Sampling ====
unsigned long lastDistanceCheck = 0;
const unsigned long distanceCheckInterval = 50;

// —————————————————————————————————————————————————————————————————————————
// URL-encode function
String urlEncode(const String &str) {
  String enc;
  for (char c : str) {
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') enc += c;
    else if (c == ' ') enc += "%20";
    else {
      char buf[4];
      sprintf(buf, "%%%02X", (unsigned char)c);
      enc += buf;
    }
  }
  return enc;
}

// Measure ultrasonic distance
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  if (distance < 0 || distance > 100) return -1;
  return distance;
}

// Move conveyor one step
void moveConveyor() {
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(150);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(150);
}

// Buzzer feedback
void soundBuzzer(int ms) {
  digitalWrite(buzzerPin, HIGH);
  delay(ms);
  digitalWrite(buzzerPin, LOW);
}

// Update LCD
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.printf("Milo:%d   ", miloCount);
  lcd.setCursor(0, 1);
  lcd.printf("Chivita:%d", chivitaCount);
}

// Perform sorting
void sortToBin(const String &obj) {
  int angle = 90;
  conveyorRunning = false;
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, HIGH);
  soundBuzzer(500);

  if (obj.equalsIgnoreCase("Milo")) {
    miloCount++;
    int slot = (miloCount - 1) % 2;
    angle = (slot == 0) ? M1 : M2;
  } else {
    chivitaCount++;
    int slot = (chivitaCount - 1) % 2;
    angle = (slot == 0) ? C1 : C2;
  }

  gripperServo.write(gripperClose);
  delay(500);
  baseServo.write(angle);
  delay(700);
  gripperServo.write(gripperOpen);
  delay(500);
  baseServo.write(90);
  delay(700);

  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
  conveyorRunning = true;
  Serial.println("READY");
}

// —————————————————————————————————————————————————————————————————————————
// Setup
void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(greenLED, HIGH);
  digitalWrite(redLED, LOW);

  baseServo.attach(baseServoPin);
  gripperServo.attach(gripperServoPin);
  gripperServo.write(gripperOpen);
  baseServo.write(90);

  lcd.init();
  lcd.backlight();
  updateLCD();

  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 20) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Wi-Fi connected");
    soundBuzzer(3000);
  } else {
    Serial.println("\n⚠️ Wi-Fi failed");
  }
}

// —————————————————————————————————————————————————————————————————————————
// Main loop
void loop() {
  // 1. Read serial detection
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    int a = line.indexOf(',');
    int b = line.lastIndexOf(',');
    if (a > 0 && b > 0 && a != b) {
      lastDetectedObject = line.substring(0, a);
      lastDetectedConfidence = line.substring(a + 1, b);
      hasValidDetection = true;
      Serial.printf("🔍 Stored: %s @ %s\n", lastDetectedObject.c_str(), lastDetectedConfidence.c_str());
    } else {
      Serial.println("⚠️ Invalid serial data: " + line);
    }
  }

  // 2. Check ultrasonic
  unsigned long now = millis();
  if (now - lastDistanceCheck >= distanceCheckInterval) {
    long dist = getDistance();
    if (dist > 0 && dist <= 1.5 && hasValidDetection && conveyorRunning) {
      Serial.printf("📏 Detected at %ld cm → %s\n", dist, lastDetectedObject.c_str());
      sortToBin(lastDetectedObject);
      updateLCD();

      // Queue data for upload
      pendingObject = lastDetectedObject;
      pendingConfidence = lastDetectedConfidence;
      hasPendingUpload = true;
      lastUploadTime = millis();

      hasValidDetection = false;
    }
    lastDistanceCheck = now;
  }

  // 3. Handle upload after sorting
  if (hasPendingUpload && (millis() - lastUploadTime >= 100)) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      String url = googleScriptURL;
      url += "?object=" + urlEncode(pendingObject);
      url += "&accuracy=" + urlEncode(pendingConfidence);

      Serial.println("📤 Sending to Google Sheets:");
      Serial.println(url);

      http.begin(url);
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
      int code = http.GET();
      if (code > 0) {
        Serial.printf("✅ Sent: %d - %s\n", code, http.getString().c_str());
      } else {
        Serial.printf("❌ Error: %s\n", http.errorToString(code).c_str());
      }
      http.end();
    } else {
      Serial.println("⚠️ No Wi-Fi for upload");
    }
    hasPendingUpload = false;
  }

  // 4. Keep conveyor running
  if (conveyorRunning) {
    digitalWrite(greenLED, HIGH);
    moveConveyor();
  }

  delay(1);
}
