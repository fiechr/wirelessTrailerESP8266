/*
Trailer for RC engine sound & LED controller for Arduino ESP8266 by neo2001.
Based on the code written by TheDIYGuy999: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32
*/

/*
Sketch for a ESP8266 to switch the lights on the trailer according to the data, received
via ESP-NOW, from the ESP32 main controller (usually mounted on the truck).

Please note, that this sketch does NOT support connecting servos or ESCs (for legs, ramps, winches), only lights!

Pressing the "FLash" (or "Boot") labeled button on the dev kit PCB will cycle through all lights
and then return back to receiving ESP-NOW data.

The trailer presence switch, if open, disables the LEDs entirely until coupled (switch closed).
Bridge the contacts if there isn't a switch on your trailer!

TODO: Blinking if forgot to turn off

*/

const float CODE_VERSION = 0.6; // Software revision

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Esp.h>
#include <TickerScheduler.h>
#include <EasyButton.h>

//#define LONG_LIGHTS_TEST  // If set, turn on all the lights in sequence, on startup, instead of just the indicators.
#define LIGHTS_TEST_ON_MILLIS 500
#define LIGHTS_TEST_WAIT_MILLIS 500

#define DEBUG_MODE  // Print additional debug messages to the serial monitor.

// PINs used for LED lights and coupler switch
#define TAILLIGHT_PIN 14 // (D5) Red tail- & brake-lights (combined)
#define INDICATOR_L_PIN 5 // (D1) Orange left indicator (turn signal) light
#define INDICATOR_R_PIN 4 // (D2) Orange right indicator (turn signal) light
#define REVERSING_LIGHT_PIN 2 // (D4) White reversing light (also connected to onboard LED)
#define SIDELIGHT_PIN 13 // (D7) Side lights

#define FLASH_BUTTON_PIN 0 // (D3) "Flash" labeled button for switching running mode (testing lights).
#define COUPLER_SWITCH_PIN 12 // (D6) This switch is closed, if the trailer is coupled to the 5th wheel. Connected between this PIN and GND.

// Used for analogWrite()
#define PWM_MIN 0
#define PWM_MAX 1023

#define SWITCH_DETECT_MS 500 // Detect state of the coupler switch every 0.5 s.

#define READ_BATTERY_VOLTAGE_MS 10000  // Read battery voltage every 10 s.

// Possible states/values for the runningMode variable below.
enum RunningMode {
  MODE_RECEIVING,
  MODE_TEST_INDICATORS_LEFT,
  MODE_TEST_INDICATORS_RIGHT,
  MODE_TEST_INDICATORS_BOTH,
  MODE_TEST_TAILLIGHT,
  MODE_TEST_REVERSINGLIGHT,
  MODE_TEST_SIDELIGHT,
  MODE_TEST_ALL_LIGHTS,
  NUM_RUNNING_MODES
};

volatile RunningMode runningMode;  // Will store the current running mode.

volatile bool espNowEnabled;

TickerScheduler ts(2);  // Scheduler used for switch periodic detection.

EasyButton flashButton(FLASH_BUTTON_PIN);

// This struct is used as a container of the data transmitted from the main controller unit.
typedef struct TrailerData {
  uint8_t tailLight;
  uint8_t sideLight;
  uint8_t reversingLight;
  uint8_t indicatorL;
  uint8_t indicatorR;
} TrailerData;

TrailerData trailerData;

const float BATTERY_VOLTAGE_OFFSET = -0.135;
float batteryVoltage;

bool trailerCoupled;  // This is true, when the trailer is coupled (NO switch closed to GND)
#define COUPLER_SWITCH_UNCOUPLED_DELAY 1500  // Time switch needs to be open to be considered "uncoupled"

#ifdef DEBUG_MODE

// Used for debugging purposes
const unsigned long PRINT_DEBUG_DELAY_MILLIS = 5000;  // 5 s delay between debug output message block
volatile uint16_t espNowMessagesReceived = 0;

int adcRawValue;
float adcVoltValue;

#endif

// Used for managing the LED lights available
// Each object represents a distinct type of light (like tail lights, for example)
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

    void off() {
      _pwmValue = PWM_MIN;
      _writeOut();
    }

    // Ugly, blocking, but seems ok in this case
    void on(unsigned long onMillis) {
      on();
      delay(onMillis);
      off();
    }

    // Ugly, blocking, but seems ok in this case
    void on(unsigned long onMillis, unsigned long waitMillis) {
      on();
      delay(onMillis);
      off();
      delay(waitMillis);
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
        if (pwmValue > PWM_MAX) {
          pwmValue = PWM_MAX;
        }
        if (pwmValue != _pwmValue) {
          _pwmValue = pwmValue;
          _writeOut();
        }
    }

    bool isOn() {
      return (_pwmValue > 0);
    }

    bool isOff() {
      return (_pwmValue == 0);
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


// Lights available on the trailer.
LedLight tailLight(TAILLIGHT_PIN);
LedLight indicatorL(INDICATOR_L_PIN);
LedLight indicatorR(INDICATOR_R_PIN);
LedLight reversingLight(REVERSING_LIGHT_PIN);
LedLight sideLight(SIDELIGHT_PIN);


// Used by EasyButton library for the "Flash" button.
void IRAM_ATTR flashButtonInterrupt() {

  flashButton.read();

}


// Called when switch pin changes state.
// void IRAM_ATTR onCouplerSwitchChangeInterrupt() {

//   couplerSwitchDetect();

// }


// Read 18650 battery voltage
// This works, by connecting an additional 100 kΩ resistor between the battery positive (+) and the A0 pin.
// Also, there needs to be a voltage divider of a 220 kΩ and 100 kΩ already in place (which many dev boards have).
// Setup: [Battery +] -----(100 kΩ)----- [A0 Pin] -----(220 kΩ)----- [ADC] -----(100 kΩ)----- [GND]
// Note: The ADC of the ESP8266 is very basic and not very precise, don't expect too much!
void readBatteryVoltage() {

  int raw = analogRead(A0);
  float volt = raw / 1023.0;
  batteryVoltage = (volt * 4.2) + BATTERY_VOLTAGE_OFFSET;

#ifdef DEBUG_MODE

  adcRawValue = raw;
  adcVoltValue = volt;

#endif

}


// Detect state of coupler switch (NO to GND).
// When the trailer is coupled to the truck, the switch needs to be closed (LOW).
void couplerSwitchDetect() {

  static unsigned long switchMillis = millis();

  if (digitalRead(COUPLER_SWITCH_PIN) == LOW) {
    switchMillis = millis();
  }

  trailerCoupled = ((millis() - switchMillis) <= COUPLER_SWITCH_UNCOUPLED_DELAY);  // Introduce some delay before uncoupled state.

}


// When the onboard "Flash" or "Boot" labeled button is pressed.
void onFlashButtonPressed() {
#ifdef DEBUG_MODE

  Serial.printf("'Flash'/'Boot' button was pressed.\n\n");

#endif

  runningMode = (RunningMode)((runningMode + 1) % NUM_RUNNING_MODES);  // Switch to next running mode or reset back to zero.

}


// Turn on/pwm/off LEDs using the received ESP-NOW data.
void showLights() {

  tailLight.pwm(trailerData.tailLight);
  indicatorL.pwm(trailerData.indicatorL);
  indicatorR.pwm(trailerData.indicatorR);
  reversingLight.pwm(trailerData.reversingLight);
  sideLight.pwm(trailerData.sideLight);

}


// Turn off all lights at once.
void turnOffLights() {

  tailLight.off();
  indicatorL.off();
  indicatorR.off();
  reversingLight.off();
  sideLight.off();

}


// Turn on all lights at the same time.
void turnOnLights() {

  tailLight.on();
  indicatorL.on();
  indicatorR.on();
  reversingLight.on();
  sideLight.on();

}


// Shortly turn on indicator lights, first left, then right.
void shortLightsTest() {

  indicatorL.on(LIGHTS_TEST_ON_MILLIS);
  indicatorR.on(LIGHTS_TEST_ON_MILLIS, LIGHTS_TEST_WAIT_MILLIS);

}


// Shortly turn on every light in sequence.
void longLightsTest() {

  shortLightsTest();
  indicatorL.on();
  indicatorR.on();
  delay(LIGHTS_TEST_ON_MILLIS);
  turnOffLights();
  tailLight.on(LIGHTS_TEST_ON_MILLIS, LIGHTS_TEST_WAIT_MILLIS);
  reversingLight.on(LIGHTS_TEST_ON_MILLIS, LIGHTS_TEST_WAIT_MILLIS);
  sideLight.on(LIGHTS_TEST_ON_MILLIS);

}


// Callback function that will be run when LEDs lights data is received
void onTrailerDataReceive(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

#ifdef DEBUG_MODE

  espNowMessagesReceived++;

#endif

  memcpy(&trailerData, incomingData, sizeof(trailerData));
  showLights();

}


// Enable WiFi adapter and ESP-NOW
void setupEspNow() {

  WiFi.disconnect();

  if (WiFi.SSID() != "") {
    ESP.eraseConfig();
  }

  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(0);

  // Init ESP-NOW and return if not successfull
  if (esp_now_init() == 0) {
    Serial.printf("ESP-NOW enabled.\n\n");
    espNowEnabled = true;
  } else {
    Serial.printf("Error initializing ESP-NOW!\n\n");
    //espNowEnabled = false;
    return;
  }

}


// Disable ESP-NOW and turn WiFi adapter off
void disableEspNow() {

  esp_now_unregister_recv_cb();
  if (esp_now_deinit() == 0) {
    Serial.printf("ESP-NOW disabled.\n\n");
    espNowEnabled = false;
  } else {
    Serial.printf("Error disabling ESP-NOW!\n\n");
    //espNowEnabled = true;  // Initial state!?
  }
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);

}


// Pause receiving (and showing) lights data, but leave ESP-NOW enabled
void pauseReceiving() {
  if (espNowEnabled) {
    esp_now_unregister_recv_cb();
    delay(300);
  
#ifdef DEBUG_MODE

    Serial.printf("Pausing processing ESP-NOW data.\n\n");

#endif

  }

}


// Continue receiving (and showing) lights data
void continueReceiving() {
  if (espNowEnabled) {
    esp_now_register_recv_cb(onTrailerDataReceive);
  
#ifdef DEBUG_MODE

    Serial.printf("Starting/Continuing processing ESP-NOW data.\n\n");

#endif

  }

}


// Main setup, runs only once
void setup() {

  pinMode(COUPLER_SWITCH_PIN, INPUT_PULLUP);  // Additional 10k pull up resistor in place
  pinMode(FLASH_BUTTON_PIN, INPUT_PULLUP);  // "Flash" button to user for other stuff
  pinMode(A0, INPUT);  // Connected to Battery + using a 100 kΩ resistor (see function above).

  couplerSwitchDetect();  // Call once to get initial state for "trailerCoupled" variable.
  // attachInterrupt(digitalPinToInterrupt(COUPLER_SWITCH_PIN), onCouplerSwitchChangeInterrupt, CHANGE);

  espNowEnabled = false;  // Set default state

  Serial.begin(115200);  // USB serial monitor (mainly for DEBUG)

#ifdef LONG_LIGHTS_TEST

  longLightsTest();

#else

  shortLightsTest();

#endif

  Serial.printf("Wireless ESP-NOW Trailer Client for ESP8266 version %.1f\n", CODE_VERSION);
  Serial.printf("https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32\n");
  Serial.printf("CPU Clock: %i Mhz, Free RAM: %i Byte, Free flash memory: %i Byte\n", ESP.getCpuFreqMHz(), ESP.getFreeHeap(), ESP.getFreeSketchSpace());
  Serial.printf("Last reset reason: %s\n", ESP.getResetReason().c_str());
  Serial.printf("Trailer MAC address: %s\n", WiFi.macAddress().c_str());
  readBatteryVoltage();  // Read once, since periodic task is not running yet.
  Serial.printf("Battery voltage: %.2f V\n\n", batteryVoltage);
  Serial.printf("Add or replace the MAC address of this device in '10_adjustmentsTrailer.h' file of the main controller:\n");
  {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("  uint8_t broadcastAddress1[] = { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X };\n\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  // Use "Flash" labeled button for other purposes
  flashButton.begin();
  flashButton.onPressed(onFlashButtonPressed);
  if (flashButton.supportsInterrupt()) {
    flashButton.enableInterrupt(flashButtonInterrupt);

#ifdef DEBUG_MODE

    Serial.printf("Interrupt attached to 'Flash'/'Boot' button.\n\n");

#endif

  }

  runningMode = MODE_RECEIVING;

  ts.add(0, SWITCH_DETECT_MS, [&](void *) { couplerSwitchDetect(); }, nullptr, true); // Add scheduler for switch detection every SWITCH_DETECT_MS ms
  ts.add(1, READ_BATTERY_VOLTAGE_MS, [&](void *) { readBatteryVoltage(); }, nullptr, true); // Add scheduler for reading battery voltage every READ_BATTERY_VOLTAGE_MS ms

}


// Main loop running forever
void loop() {

  static RunningMode runningModeBefore = NUM_RUNNING_MODES;
  static bool trailerCoupledBefore = !trailerCoupled;

#ifdef DEBUG_MODE

  static unsigned long lastDebugMillis = millis();

#endif

  ts.update();  // TickerScheduler

  if (trailerCoupled != trailerCoupledBefore) {  // If state of trailer changed

#ifdef DEBUG_MODE

    Serial.printf("Trailer state changed from %s to %s.\n\n", trailerCoupledBefore ? "coupled" : "uncoupled", trailerCoupled ? "coupled" : "uncoupled");

#endif

    trailerCoupledBefore = trailerCoupled;

    if (trailerCoupled) {
      setupEspNow();
    } else {
      disableEspNow();
    }

  }

  if (runningMode != runningModeBefore) {  // If running mode changed

#ifdef DEBUG_MODE

    Serial.printf("Running mode changed from %d to %d.\n\n", runningModeBefore, runningMode);

#endif

    runningModeBefore = runningMode;

    switch(runningMode) {

      case MODE_RECEIVING:
        turnOffLights();
        continueReceiving();
        break;

      case MODE_TEST_INDICATORS_LEFT:
        pauseReceiving();
        turnOffLights();
        indicatorL.on();
        break;

      case MODE_TEST_INDICATORS_RIGHT:
        pauseReceiving();
        turnOffLights();
        indicatorR.on();
        break;

      case MODE_TEST_INDICATORS_BOTH:
        pauseReceiving();
        turnOffLights();
        indicatorL.on();
        indicatorR.on();
        break;

      case MODE_TEST_TAILLIGHT:
        pauseReceiving();
        turnOffLights();
        tailLight.on();
        break;

      case MODE_TEST_REVERSINGLIGHT:
        pauseReceiving();
        turnOffLights();
        reversingLight.on();
        break;

      case MODE_TEST_SIDELIGHT:
        pauseReceiving();
        turnOffLights();
        sideLight.on();
        break;

      case MODE_TEST_ALL_LIGHTS:
        pauseReceiving();
        turnOnLights();
        break;

      default:  // Theoretically, this should never be reached, because button resets to 0 first.
        runningMode = MODE_RECEIVING;

    }

  }

#ifdef DEBUG_MODE

  if (millis() - lastDebugMillis > PRINT_DEBUG_DELAY_MILLIS) {
    Serial.printf("DEBUG_MODE:\n");
    Serial.printf("Current running mode  : %d\n", runningMode);
    Serial.printf("Coupler switch        : %s\n", (digitalRead(COUPLER_SWITCH_PIN) == LOW) ? "closed" : "open");
    Serial.printf("Trailer coupled state : %s\n", trailerCoupled ? "coupled" : "uncoupled");
    Serial.printf("ESP-NOW state:        : %s\n", espNowEnabled ? "enabled" : "disabled");
    Serial.printf("ESP-NOW message count : %i (last %is)\n", espNowMessagesReceived, (uint8_t)(PRINT_DEBUG_DELAY_MILLIS/1000.0));
    Serial.printf("Battery voltage       : %.2f V (adcVoltValue: %.4f, BATTERY_VOLTAGE_OFFSET: %.4f, adcRawValue: %i)\n\n", batteryVoltage, adcVoltValue, BATTERY_VOLTAGE_OFFSET, adcRawValue);

    Serial.printf("Taillights (TL)       : %i (received value: %i)\n", tailLight.pwm(), trailerData.tailLight);
    Serial.printf("Sidelights (SL)       : %i (received value: %i)\n", sideLight.pwm(), trailerData.sideLight);
    Serial.printf("Reversing Lights (REV): %i (received value: %i)\n", reversingLight.pwm(), trailerData.reversingLight);
    Serial.printf("Indicator L (IL)      : %i (received value: %i)\n", indicatorL.pwm(), trailerData.indicatorL);
    Serial.printf("Indicator R (IL)      : %i (received value: %i)\n\n", indicatorR.pwm(), trailerData.indicatorR);


    lastDebugMillis = millis();
    espNowMessagesReceived = 0;
  }

#endif

}
