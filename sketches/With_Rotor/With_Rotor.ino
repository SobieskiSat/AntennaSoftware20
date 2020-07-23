#include "Declarations.h"

void setup()
{
  // Setup arduino
  SerialUSB.begin(115200);  //rotor -PC communication
  Serial.begin(115200);     //rotor - remote communication

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //altitudeFromPressure = altFromPressure();

  //Init antenna rotor after it's calibrated
  //waits till the rotor is placed in (0,0) position
  while(!rotorCalibrated) getSerial();
  rotorInit();

  delay(1000);

  duplex_setup();

}

void loop()
{
  steppers_run();

  getSerial();    //we might want to add stepper_run() here if it moves slowly

  duplex_loop();
}
