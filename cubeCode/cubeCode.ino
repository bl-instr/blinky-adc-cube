#include <BlinkyPicoW.h>

// --- Configuration Constants ---
constexpr int BLINKY_DIAG    = 0;
constexpr int CUBE_DIAG      = 0;
constexpr int COMM_LED_PIN   = 2;
constexpr int RST_BUTTON_PIN = 3;
constexpr int NUMCHAN        = 2;

constexpr int ADC_PINS[NUMCHAN] = {A0, A1};
constexpr float ADC_REF_VOLTS   = 3.0f;
constexpr float ADC_MAX_COUNTS  = 4096.0f;

// --- Data Structures ---
struct CubeSetting {
  uint16_t publishInterval;
  uint16_t nsamples;
};

struct CubeReading {
  float adc[NUMCHAN];
  float bandWidth;
};

struct CubeArm {
  bool adc[NUMCHAN];
  bool bandWidth = true;
};

// --- Global Variables ---
CubeSetting setting;
CubeReading reading;
CubeReading readingLow;
CubeReading readingHigh;
CubeArm readingArm;

unsigned long lastPublishTime = 0;
uint32_t digCount = 0;

// --- Helper Functions ---
template <typename T>
inline bool outsideLimits(T current, T low, T high) {
  return (current < low) || (current > high);
}

inline float readADCInVolts(int pin) {
  return ADC_REF_VOLTS * (static_cast<float>(analogRead(pin)) / ADC_MAX_COUNTS);
}

// --- Setup Functions ---
void setupBlinky() {
  if (BLINKY_DIAG > 0) {
    Serial.begin(9600);
  }

  // MQTT & Hardware Configuration
  BlinkyPicoW.setMqttKeepAlive(15);
  BlinkyPicoW.setMqttSocketTimeout(4);
  BlinkyPicoW.setMqttPort(1883);
  BlinkyPicoW.setMqttLedFlashMs(100);
  BlinkyPicoW.setHdwrWatchdogMs(8000);

  BlinkyPicoW.begin(BLINKY_DIAG, COMM_LED_PIN, RST_BUTTON_PIN, true, sizeof(setting), sizeof(reading));
}

void setupCube() {
  if (BLINKY_DIAG < 1 && CUBE_DIAG > 0) {
    Serial.begin(9600);
  }

  analogReadResolution(12);
  setting.publishInterval = 2000;
  setting.nsamples = 2;

  for (int i = 0; i < NUMCHAN; ++i) {
    reading.adc[i] = readADCInVolts(ADC_PINS[i]);
    readingArm.adc[i] = true;
  }

  digCount = 1;
  lastPublishTime = millis();
}

// --- Core Loop Functions ---
void loopCube() {
  const unsigned long now = millis();

  // 1. Regular Timed Publishing
  if ((now - lastPublishTime) >= setting.publishInterval) {
    const float safeInterval = (setting.publishInterval > 0) ? static_cast<float>(setting.publishInterval) : 1.0f;
    const float safeSamples  = (setting.nsamples > 0) ? static_cast<float>(setting.nsamples) : 1.0f;

    reading.bandWidth = 500.0f * (static_cast<float>(digCount) / safeInterval) / safeSamples;
    lastPublishTime = now;

    if (BlinkyPicoW.publishCubeData(reinterpret_cast<uint8_t*>(&setting), reinterpret_cast<uint8_t*>(&reading), false)) {
      for (int i = 0; i < NUMCHAN; ++i) {
        if (!outsideLimits(reading.adc[i], readingLow.adc[i], readingHigh.adc[i])) {
          readingArm.adc[i] = true;
        }
      }
      digCount = 0;
    }
  }

  // 2. ADC Sampling & Threshold Breach Checks
  for (int i = 0; i < NUMCHAN; ++i) {
    const float newAdc = readADCInVolts(ADC_PINS[i]);
    const float nsamples = (setting.nsamples > 0) ? static_cast<float>(setting.nsamples) : 1.0f;

    reading.adc[i] += (newAdc - reading.adc[i]) / nsamples;

    if (outsideLimits(reading.adc[i], readingLow.adc[i], readingHigh.adc[i])) {
      if (readingArm.adc[i]) {
        const bool published = BlinkyPicoW.publishCubeData(
          reinterpret_cast<uint8_t*>(&setting), 
          reinterpret_cast<uint8_t*>(&reading), 
          true
        );
        
        readingArm.adc[i] = !published;
        if (published) {
          lastPublishTime = now;
        }
      }
    }
  }

  ++digCount;

  // 3. Check for New MQTT Settings
  const bool newSettings = BlinkyPicoW.retrieveCubeSetting(
    reinterpret_cast<uint8_t*>(&setting), 
    reinterpret_cast<uint8_t*>(&readingLow), 
    reinterpret_cast<uint8_t*>(&readingHigh)
  );

  if (newSettings) {
    if (setting.publishInterval < 500) setting.publishInterval = 500;
    if (setting.nsamples < 1)         setting.nsamples = 1;

    for (int i = 0; i < NUMCHAN; ++i) {
      reading.adc[i] = readADCInVolts(ADC_PINS[i]);
    }
  }
}
