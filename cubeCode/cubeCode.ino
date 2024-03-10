boolean printDiagnostics = false;

union CubeData
{
  struct
  {
    int16_t state;
    int16_t watchdog;
    int16_t publishInterval;
    int16_t nsamples;
    int16_t signal1;
    int16_t signal2;
    int16_t signal3;
  };
  byte buffer[14];
};
CubeData cubeData;

#include "BlinkyPicoWCube.h"


int commLEDPin = 2;
int commLEDBright = 255; 
int resetButtonPin = 3;

unsigned long lastPublishTime;
unsigned long publishInterval = 2000;

void setupServerComm()
{
  // Optional setup to overide defaults
  if (printDiagnostics) Serial.begin(115200);
  BlinkyPicoWCube.setChattyCathy(printDiagnostics);
  BlinkyPicoWCube.setWifiTimeoutMs(20000);
  BlinkyPicoWCube.setWifiRetryMs(20000);
  BlinkyPicoWCube.setMqttRetryMs(3000);
  BlinkyPicoWCube.setResetTimeoutMs(10000);
  BlinkyPicoWCube.setHdwrWatchdogMs(8000);
  BlinkyPicoWCube.setBlMqttKeepAlive(8);
  BlinkyPicoWCube.setBlMqttSocketTimeout(4);
  BlinkyPicoWCube.setMqttLedFlashMs(10);
  BlinkyPicoWCube.setWirelesBlinkMs(100);
  BlinkyPicoWCube.setMaxNoMqttErrors(5);
  BlinkyPicoWCube.setMaxNoConnectionAttempts(5);
  
  // Must be included
  BlinkyPicoWCube.init(commLEDPin, commLEDBright, resetButtonPin);
}
float signal1;
float signal2;
float signal3;

void setupCube()
{
  analogReadResolution(12);
  lastPublishTime = millis();
  cubeData.state = 1;
  cubeData.nsamples = 10;
  cubeData.watchdog = 0;
  cubeData.signal1 = 0;
  cubeData.signal2 = 0;
  cubeData.signal3 = 0;

  cubeData.publishInterval = (int16_t) publishInterval;

  signal1 = (float) analogRead(A2);
  signal2 = (float) analogRead(A1);
  signal3 = (float) analogRead(A0);
}

void cubeLoop()
{
  unsigned long nowTime = millis();
  signal1 = signal1 +(((float) analogRead(A2)) - signal1) / ((float) cubeData.nsamples);
  signal2 = signal2 +(((float) analogRead(A1)) - signal2) / ((float) cubeData.nsamples);
  signal3 = signal3 +(((float) analogRead(A0)) - signal3) / ((float) cubeData.nsamples);
 
  if ((nowTime - lastPublishTime) > publishInterval)
  {
    lastPublishTime = nowTime;
    cubeData.watchdog = cubeData.watchdog + 1;
    if (cubeData.watchdog > 32760) cubeData.watchdog= 0;
    cubeData.signal1 = (int16_t) signal1;
    cubeData.signal2 = (int16_t) signal2;
    cubeData.signal3 = (int16_t) signal3;
    BlinkyPicoWCube::publishToServer();
    if (printDiagnostics)
    {
      Serial.print("Signals: ");
      Serial.print(cubeData.signal1);
      Serial.print(", ");
      Serial.print(cubeData.signal2);
      Serial.print(", ");
      Serial.println(cubeData.signal3);
    }
  }  
}


void handleNewSettingFromServer(uint8_t address)
{
  switch(address)
  {
    case 0:
      break;
    case 1:
      break;
    case 2:
      if (cubeData.publishInterval < 500) cubeData.publishInterval = 500;
      publishInterval = (unsigned long) cubeData.publishInterval;
      break;
    case 3:
      if (cubeData.nsamples < 1) cubeData.nsamples = 1;
      signal1 = (float) analogRead(A2);
      signal2 = (float) analogRead(A1);
      signal3 = (float) analogRead(A0);
      break;
    case 4:
      break;
    default:
      break;
  }
}
