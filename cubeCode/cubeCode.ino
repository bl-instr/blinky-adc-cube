#define BLINKY_DIAG         0
#define COMM_LED_PIN       16
#define RST_BUTTON_PIN     15
#include <BlinkyPicoW.h>

struct CubeSetting
{
  uint16_t publishInterval;
  uint16_t nsamples;
};
CubeSetting setting;

struct CubeReading
{
  uint16_t adc0;
  uint16_t adc1;
  uint16_t bandWidth;
};
CubeReading reading;


unsigned long lastPublishTime;
float fadc0;
float fadc1;
int digCount;


void setupBlinky()
{
  if (BLINKY_DIAG > 0) Serial.begin(9600);

  BlinkyPicoW.setMqttKeepAlive(15);
  BlinkyPicoW.setMqttSocketTimeout(4);
  BlinkyPicoW.setMqttPort(1883);
  BlinkyPicoW.setMqttLedFlashMs(100);
  BlinkyPicoW.setHdwrWatchdogMs(8000);

  BlinkyPicoW.begin(BLINKY_DIAG, COMM_LED_PIN, RST_BUTTON_PIN, true, sizeof(setting), sizeof(reading));
}

void setupCube()
{
  analogReadResolution(12);
  setting.publishInterval = 2000;
  setting.nsamples = 2;

  reading.adc0 = 0;
  reading.adc1 = 0;
  fadc0 = (float) analogRead(A0);
  fadc1 = (float) analogRead(A1);
  digCount = 1;

  lastPublishTime = millis(); 
}
void loopCube()
{
  unsigned long now = millis();
  if ((now - lastPublishTime) > setting.publishInterval)
  {
    float fbandwidth = 500.0 * ( ((float) digCount) / ((float) setting.publishInterval) ) / ((float) setting.nsamples);
    reading.bandWidth = (uint16_t) fbandwidth;   
    lastPublishTime = now;
    reading.adc0 = (uint16_t) (fadc0 * 8);
    reading.adc1 = (uint16_t) (fadc1 * 8);
    boolean successful = BlinkyPicoW.publishCubeData((uint8_t*) &setting, (uint8_t*) &reading, false);
    digCount = 0;
  }
  fadc0 = fadc0 +(((float) analogRead(A0)) - fadc0) / ((float) setting.nsamples);
  fadc1 = fadc1 +(((float) analogRead(A1)) - fadc1) / ((float) setting.nsamples);
  ++digCount;

  boolean newSettings = BlinkyPicoW.retrieveCubeSetting((uint8_t*) &setting);
  if (newSettings)
  {
    if (setting.publishInterval < 500) setting.publishInterval = 500;
    if (setting.nsamples < 1) setting.nsamples = 1;
    fadc0 = (float) analogRead(A0);
    fadc0 = (float) analogRead(A1);
  }
}
