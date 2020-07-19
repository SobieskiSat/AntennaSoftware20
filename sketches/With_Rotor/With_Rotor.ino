#include "Declarations.h"

void setup()
{
  // Setup arduino
  SerialUSB.begin(115200);  //rotor -PC communication
  Serial.begin(115200);     //rotor - remote communication

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

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

  SerialUSB.println("<p" + String(pressure, 2) +       // Prints received values via Serial to PC
                    "Pt" + String(temperature, 3) +   // format is like: x012345X
                    "Tn" + String(latitude, 7) +      // 'x' and 'X' are bounds for value
                    "Ne" + String(longitude, 7) +     // eg. p1023.97P sends pressure
                    "Ea" + String(altitude, 2) +
                    "Az" + String(yaw, 1) +
                    "Zy" + String(pitch, 1) +
                    "Yx" + String(roll, 1) +
                    "Xr" + String(radio.rssi) +
                    "Ro" + String(operationModeFB) +
                    "Os" + String(smallSPS, 2) +         //approximation to be corrected
                    "Sb" + String(bigSPS, 2) +
                    "Bv" + String(stepsToAngle(stepperV.currentPosition())) +
                    "Vh" + String(stepsToAngle(stepperH.currentPosition())-HORIZONTA_CALIBRATION_OFFSET) +
                    "H>");
  delay(40);
}
