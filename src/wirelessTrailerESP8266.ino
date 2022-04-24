/* Trailer for RC engine sound & LED controller for Arduino ESP8266. Based on the code written by TheDIYGuy999*/
/* https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32 */

const float codeVersion = 0.5; // Software revision

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Esp.h>
#include <TickerScheduler.h>

//#define DEBUG_MODE

// PINs used for LED lights and coupler switch
#define TAILLIGHT_PIN 14 // (D5) Red tail- & brake-lights (combined)
#define INDICATOR_L_PIN 5 // (D1) Orange left indicator (turn signal) light (onboard LED)
#define INDICATOR_R_PIN 4 // (D2) Orange right indicator (turn signal) light
#define REVERSING_LIGHT_PIN 2 // (D4) White reversing light
#define SIDELIGHT_PIN 13 // (D7) Side lights

#define COUPLER_SWITCH_PIN 12 // (D6) This NO switch is closed, if the trailer is coupled to the 5th wheel. Connected between Pin 16 and GND.

// Used for analogWrite()
#define PWM_MIN 0
#define PWM_MAX 1023

#define SWITCH_DETECT_MS 500 // Detect state of the coupler switch every 0.5s

TickerScheduler ts(1);  // Scheduler used for switch periodic detection

// This struct contains the data transmitted from the main controller unit
typedef struct struct_message {
  uint8_t tailLight;
  uint8_t sideLight;
  uint8_t reversingLight;
  uint8_t indicatorL;
  uint8_t indicatorR;
} struct_message;

struct_message trailerData;

bool trailerCoupled;  // This is true, when the trailer is coupled (NO switch closed to GND)

#ifdef DEBUG_MODE

// Used for debugging purposes
const unsigned long printDebugDelayMillis = 3000;  // 3s delay between debug messages
volatile unsigned long lastDebugMillis = millis();
volatile uint16_t espNowMessagesReceived = 0;

#endif


// Callback function that will be run when LEDs lights data is received
void onDataReceive(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

#ifdef DEBUG_MODE

  espNowMessagesReceived++;

#endif

  memcpy(&trailerData, incomingData, sizeof(trailerData));

  if (trailerCoupled) {
    showLEDs();
  } else {
    turnOffLEDs();
  }
  
}


// ESP-NOW setup for receiving LEDs lights data from main controller (usually on the truck)
void setupEspNow() {

  WiFi.disconnect();

  if (WiFi.SSID() != "") {
    ESP.eraseConfig();
  }

  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(0);

  // Init ESP-NOW and return if not successfull
  if (esp_now_init() != 0) {
    Serial.printf("Error initializing ESP-NOW! I give up.\n");
    return;
  }

  // Function to call when receiving data
  esp_now_register_recv_cb(onDataReceive);
}


// Main setup, runs only once
void setup() {

  pinMode(TAILLIGHT_PIN, OUTPUT);
  pinMode(INDICATOR_L_PIN, OUTPUT);
  pinMode(INDICATOR_R_PIN, OUTPUT);
  pinMode(REVERSING_LIGHT_PIN, OUTPUT);
  pinMode(SIDELIGHT_PIN, OUTPUT);
  pinMode(COUPLER_SWITCH_PIN, INPUT_PULLUP);

  Serial.begin(115200); // USB serial (mainly for DEBUG)

  // Short LEDs test
  turnOnLEDs();
  delay(2000);
  turnOffLEDs();

  Serial.printf("Wireless ESP-NOW Trailer Client for ESP8266 version %.2f\n", codeVersion);
  Serial.printf("https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32\n");
  Serial.printf("CPU Clock: %i Mhz, Free RAM: %i Byte, Free flash memory: %i Byte\n", ESP.getCpuFreqMHz(), ESP.getFreeHeap(), ESP.getFreeSketchSpace());
  Serial.printf("Last reset reason: %s\n", ESP.getResetReason().c_str());
  Serial.printf("Trailer MAC address: %s\n\n", WiFi.macAddress().c_str());
  Serial.printf("Add or replace the MAC address of this device in '10_adjustmentsTrailer.h' file of the main controller:\n");
  {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("  uint8_t broadcastAddress1[] = { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X };\n\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  setupEspNow();

  ts.add(0, SWITCH_DETECT_MS, [&](void *) { switchDetect(); }, nullptr, true); // Add scheduler for switch detection every SWITCH_DETECT_MS ms

}


// Detect state of coupler switch (NO to GND)
void switchDetect() {

  static unsigned long switchMillis;

  if (digitalRead(COUPLER_SWITCH_PIN) == LOW) {
    switchMillis = millis();
  }

  trailerCoupled = (millis() - switchMillis <= 1000); // 1s delay, if not coupled (yet)

}


// Show LEDs using the received ESP-NOW data
void showLEDs() {

  analogWrite(TAILLIGHT_PIN, map(trailerData.tailLight, 0, UINT8_MAX, PWM_MIN, PWM_MAX));
  analogWrite(INDICATOR_L_PIN, map(trailerData.indicatorL, 0, UINT8_MAX, PWM_MIN, PWM_MAX));
  analogWrite(INDICATOR_R_PIN, map(trailerData.indicatorR, 0, UINT8_MAX, PWM_MIN, PWM_MAX));
  analogWrite(REVERSING_LIGHT_PIN, map(trailerData.reversingLight, 0, UINT8_MAX, PWM_MIN, PWM_MAX));
  analogWrite(SIDELIGHT_PIN, map(trailerData.sideLight, 0, UINT8_MAX, PWM_MIN, PWM_MAX));

}


// Set all LEDs to a common PWM value
void setLEDsPWM(uint16_t pwmValue) {

  analogWrite(TAILLIGHT_PIN, pwmValue);
  analogWrite(INDICATOR_L_PIN, pwmValue);
  analogWrite(INDICATOR_R_PIN, pwmValue);
  analogWrite(REVERSING_LIGHT_PIN, pwmValue);
  analogWrite(SIDELIGHT_PIN, pwmValue);

}


void turnOffLEDs() {
  setLEDsPWM(PWM_MIN);
}


void turnOnLEDs() {
  setLEDsPWM(PWM_MAX);
}


// Main loop running forever
void loop() {

  ts.update();  // TickerScheduler

#ifdef DEBUG_MODE

  if (millis() - lastDebugMillis > 3000) {
    Serial.printf("DEBUG_MODE:\n");
    Serial.printf("Coupler switch        : %s\n", trailerCoupled ? "closed" : "open");
    Serial.printf("Taillights (TL)       : %i\n", trailerData.tailLight);
    Serial.printf("Sidelights (SL)       : %i\n", trailerData.sideLight);
    Serial.printf("Reversing Lights (REV): %i\n", trailerData.reversingLight);
    Serial.printf("Indicator L (IL)      : %i\n", trailerData.indicatorL);
    Serial.printf("Indicator R (IL)      : %i\n", trailerData.indicatorR);
    Serial.printf("ESP-NOW message count : %i (last %is)\n\n", espNowMessagesReceived, (uint8_t)(printDebugDelayMillis/1000.0));

    lastDebugMillis = millis();
    espNowMessagesReceived = 0;
  }

#endif

}
