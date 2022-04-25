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

#define SWITCH_DETECT_MS 500 // Detect state of the coupler switch every 0.5 s

#define READ_BATTERY_VOLTAGE_MS 10000  // Read battery voltage every 10 s

TickerScheduler ts(2);  // Scheduler used for switch periodic detection

// This struct contains the data transmitted from the main controller unit
typedef struct struct_message {
  uint8_t tailLight;
  uint8_t sideLight;
  uint8_t reversingLight;
  uint8_t indicatorL;
  uint8_t indicatorR;
} struct_message;

struct_message trailerData;

const float batteryVoltageOffset = -0.135;
float batteryVoltage;

bool trailerCoupled;  // This is true, when the trailer is coupled (NO switch closed to GND)

#ifdef DEBUG_MODE

// Used for debugging purposes
const unsigned long printDebugDelayMillis = 3000;  // 3s delay between debug messages
volatile unsigned long lastDebugMillis = millis();
volatile uint16_t espNowMessagesReceived = 0;

int adcRawValue;
float adcVoltValue;

#endif

class LedLight {
  public:
    LedLight(uint8_t pin) {
      pinMode(pin, OUTPUT);
      _pin = pin;
      _pwmValue = PWM_MIN;
    }

    void on() {
      _pwmValue = PWM_MAX;
      _writeOut();
    }

    uint16_t pwm() {
      return _pwmValue;
    }

    void pwm(uint8_t pwmValue) {
      long newValue = map(pwmValue, 0, UINT8_MAX, PWM_MIN, PWM_MAX);
      if (newValue != _pwmValue) {
        _pwmValue = newValue;
        _writeOut();
      }
    }

    void pwm(uint16_t pwmValue) {
      if (pwmValue >= PWM_MIN) {
        if (pwmValue > PWM_MAX) {
          pwmValue = PWM_MAX;
        }
        if (pwmValue != _pwmValue) {
          _pwmValue = pwmValue;
          _writeOut();
        }
      }
    }

    void off() {
      _pwmValue = PWM_MIN;
      _writeOut();
    }

    bool isOn() {
      return _pwmValue > 0;
    }

    bool isOff() {
      return _pwmValue == 0;
    }

    void toggle() {
      if (isOn()) {
        off();
      } else {
        on();
      }
    }

  private:
    uint8_t _pin;
    uint16_t _pwmValue;

    void _writeOut() {
      analogWrite(_pin, _pwmValue);
    }
};


// Lights available on the trailer
LedLight tailLight(TAILLIGHT_PIN);
LedLight indicatorL(INDICATOR_L_PIN);
LedLight indicatorR(INDICATOR_R_PIN);
LedLight reversingLight(REVERSING_LIGHT_PIN);
LedLight sideLight(SIDELIGHT_PIN);


// Read 18650 lipo battery voltage
void readBatteryVoltage() {

  int raw = analogRead(A0);
  float volt = raw / 1023.0;
  batteryVoltage = (volt * 4.2) + batteryVoltageOffset;

#ifdef DEBUG_MODE

  adcRawValue = raw;
  adcVoltValue = volt;

#endif

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
void showLights() {

  tailLight.pwm(trailerData.tailLight);
  indicatorL.pwm(trailerData.indicatorL);
  indicatorR.pwm(trailerData.indicatorR);
  reversingLight.pwm(trailerData.reversingLight);
  sideLight.pwm(trailerData.sideLight);

}


void turnOffLights() {

  tailLight.off();
  indicatorL.off();
  indicatorR.off();
  reversingLight.off();
  sideLight.off();

}


void turnOnLights() {

  tailLight.on();
  indicatorL.on();
  indicatorR.on();
  reversingLight.on();
  sideLight.on();

}

// Callback function that will be run when LEDs lights data is received
void onDataReceive(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

#ifdef DEBUG_MODE

  espNowMessagesReceived++;

#endif

  memcpy(&trailerData, incomingData, sizeof(trailerData));

  if (trailerCoupled) {
    showLights();
  } else {
    turnOffLights();
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

  pinMode(COUPLER_SWITCH_PIN, INPUT);
  pinMode(A0, INPUT);

  Serial.begin(115200); // USB serial (mainly for DEBUG)

  // Short LEDs test
  indicatorL.on();
  indicatorR.on();
  delay(2000);
  indicatorL.off();
  indicatorR.off();

  Serial.printf("Wireless ESP-NOW Trailer Client for ESP8266 version %.1f\n", codeVersion);
  Serial.printf("https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32\n");
  Serial.printf("CPU Clock: %i Mhz, Free RAM: %i Byte, Free flash memory: %i Byte\n", ESP.getCpuFreqMHz(), ESP.getFreeHeap(), ESP.getFreeSketchSpace());
  Serial.printf("Last reset reason: %s\n", ESP.getResetReason().c_str());
  Serial.printf("Trailer MAC address: %s\n", WiFi.macAddress().c_str());
  readBatteryVoltage();
  Serial.printf("Battery voltage: %.2f V\n\n", batteryVoltage);
  Serial.printf("Add or replace the MAC address of this device in '10_adjustmentsTrailer.h' file of the main controller:\n");
  {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("  uint8_t broadcastAddress1[] = { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X };\n\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  setupEspNow();

  ts.add(0, SWITCH_DETECT_MS, [&](void *) { switchDetect(); }, nullptr, true); // Add scheduler for switch detection every SWITCH_DETECT_MS ms
  ts.add(1, READ_BATTERY_VOLTAGE_MS, [&](void *) { readBatteryVoltage(); }, nullptr, true); // Add scheduler for reading battery voltage every READ_BATTERY_VOLTAGE_MS ms

}


// Main loop running forever
void loop() {

  ts.update();  // TickerScheduler

#ifdef DEBUG_MODE

  if (millis() - lastDebugMillis > 3000) {
    Serial.printf("DEBUG_MODE:\n");
    Serial.printf("Battery voltage       : %.2f V (adcVoltValue: %.4f, batteryVoltageOffset: %.4f, adcRawValue: %i)\n", batteryVoltage, adcVoltValue, batteryVoltageOffset, adcRawValue);
    Serial.printf("Coupler switch        : %s\n", trailerCoupled ? "closed" : "open");
    Serial.printf("Taillights (TL)       : %i (received value: %i)\n", tailLight.pwm(), trailerData.tailLight);
    Serial.printf("Sidelights (SL)       : %i (received value: %i)\n", sideLight.pwm(), trailerData.sideLight);
    Serial.printf("Reversing Lights (REV): %i (received value: %i)\n", reversingLight.pwm(), trailerData.reversingLight);
    Serial.printf("Indicator L (IL)      : %i (received value: %i)\n", indicatorL.pwm(), trailerData.indicatorL);
    Serial.printf("Indicator R (IL)      : %i (received value: %i)\n", indicatorR.pwm(), trailerData.indicatorR);
    Serial.printf("ESP-NOW message count : %i (last %is)\n\n", espNowMessagesReceived, (uint8_t)(printDebugDelayMillis/1000.0));

    lastDebugMillis = millis();
    espNowMessagesReceived = 0;
  }

#endif

}
