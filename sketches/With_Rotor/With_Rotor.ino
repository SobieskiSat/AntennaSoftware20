#include <Radiolot.h>
#include "config.h"

#include <MultiStepper.h>
#include <AccelStepper.h>
#include "rotor_definitions.h"

bool led_state = true;// Used for blinking when packet received

SX1278 radio;         // Instance of SX1278 LoRa

uint8_t incoming_count = 5; // Count of incoming packets in duplex, (eg. 5 received for 1 transmitted)
                            // satellite sends this number of packets and then listens for packet which has to be sent from antenna

String fragment;      // Used in parsing PC Serial messages, contains values in Strings
String serialUSB_packet; // Serial messages buffer (from PC)
String serial_packet;    // Serial messages buffer (from rotor remote)
bool readingUSB_packet = false; //indicators for receiving serial commands from PC and rotor remote
bool reading_packet = false;    //true if the packet is not fully received yet
                                //false if the opening '<' character was not received yet

uint8_t toSend[2];    // Buffer to be sent via radio

// Transmitted variables (recieved via Serial)
uint8_t servo;        // 0 (0b0) - off, 1 (0b1) - on [first BIT of transmitted package]
uint8_t motors;       // 0 (0b00) - off, 2 (0b10) - on [second BIT of transmitted package]
float angle;          // angle in degrees [second byte of transmitted package]

// Transmitted variables (recieved via Serial) (for antenna rotor)
float horizontalAngle; //relative angle for the rotor to rotate in a given plane
float verticalAngle;   //applied for primitive manual steering

// Received variables
float pressure;
float temperature;
float latitude;
float longitude;
float yaw;
float pitch;
float roll;


//creates an object of AccelStepper class for each of the rotors
AccelStepper stepperV(AccelStepper::DRIVER, STEPPERV_STEP_PIN, STEPPERV_DIR_PIN);
AccelStepper stepperH(AccelStepper::DRIVER, STEPPERH_STEP_PIN, STEPPERH_DIR_PIN);

void setup()
{
   //Init antenna rotor
  rotorInit();

  // Setup arduino
  SerialUSB.begin(115200);
  Serial.begin(115200);
  //SPI.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);

  // Setup radio
  /*radio.config = sx1278_default_config;
  radio.useDio0IRQ = false;
  SX1278_init(&radio); */

  // Setup transmitted variables
  angle = 0;
  servo = 0;
  motors = 0;

  //Setup transmitted rotor variables
  verticalAngle = 0;
  horizontalAngle = 0;
}

void loop()
{
  stepperV.run();                                     //tries to make a vertical / horizontal step
  stepperH.run();                                     //these have to be called as often as possible

  /*SX1278_receive(&radio);                             // Start listening for packet (waits until received)
  decodePacket();                                     // Updates received variables from packet
  SerialUSB.println("p" + String(pressure, 2) +       // Prints received values via Serial to PC
                    "Pt" + String(temperature, 1) +   // format is like: x012345X
                    "Tx" + String(latitude, 7) +      // 'x' and 'X' are bounds for value
                    "Xy" + String(longitude, 7) +     // eg. p1023.97P sends pressure
                    "Ya" + String(yaw, 1) +
                    "Ab" + String(pitch, 1) +
                    "Bc" + String(roll, 1) +
                    "Cr" + String(radio.rssi) + "R");
                    */

  digitalWrite(LED_BUILTIN, led_state); led_state = !led_state; // toggle LED

  stepperV.run();                                     //tries to make a vertical / horizontal step
  stepperH.run();                                     //these have to be called as often as possible

  getSerial();

  /*if (radio.rxDone)                                         // If received packet is OK
  {
    if (radio.rxBuffer[radio.rxLen - 1] == incoming_count)  // If packet number (last byte of packet) is equal to count of incoming packets
    {                                                       // send packet then (satellite will listen for a while)
                                                            // (counts must be configured equal on both radios for duplex to work)
      getSerial();                                          // Receive data from Serial to be sent to satellite

      toSend[0] = motors + servo;                           // Prepare packet, two bytes for motor and servo state
      toSend[1] = (uint8_t)(angle * (255.0 / 360.0));       // Convert float to byte and place it as 2nd byte
      delay(10);                                            // [!!] Satellite seems to have problems with instantaneous reply so wait for a while
      SX1278_transmit(&radio, toSend, 2);                   // Transmit packet

      //SX1278_transmit(&radio, (uint8_t*)"Duplex!", 7);    // Dummy packet
    }

    radio.rxDone = false;                                   // Remove packet received flag
  } */
}

void getSerial()
{
  char incoming;
  while (SerialUSB.available() > 0)                         // If there are bytes waiting to be read from Serial
  {
    incoming = SerialUSB.read();                            // Get one character
    if (readingUSB_packet) serialUSB_packet += incoming;          // Append character to packet

    if (incoming == '<')                                    // '<' character opens the packet
    {
      readingUSB_packet = true;                                // We want now to save characters
      serialUSB_packet = "";
      serialUSB_packet += incoming;
    }
    else if (incoming == '>') {
      readingUSB_packet = false;       // '>' character ends the packet
      parseSerial(serialUSB_packet);
    }
  }

  //parseSerial();

  while (Serial.available() > 0)                            // If there are bytes waiting to be read from Serial
  {
    digitalWrite(LED_BUILTIN, HIGH);
    incoming = Serial.read();                               // Get one character
    if (reading_packet) serial_packet += incoming;          // Append character to packet

    if (incoming == '<')                                    // '<' character opens the packet
    {
      reading_packet = true;                                // We want now to save characters
      serial_packet = "";
      serial_packet += incoming;
    }
    else if (incoming == '>') {
      reading_packet = false;         // '>' character ends the packet
      parseSerial(serial_packet);     //parses serial data immidiately when one packet is received
      //digitalWrite(LED_BUILTIN, LOW);
    }
    digitalWrite(LED_BUILTIN, led_state); led_state = !led_state;
    //SerialUSB.println(incoming);
    //SerialUSB.println(":inc pac:");
    //SerialUSB.println(serial_packet);
  }

  //parseSerial();                    //parsing data here might overall take longer than in while loop
}

// Parses Serial message, eg. "s1Sm0M" -> servo = 1, motors = 0
// takes an argument stating which string should be parsed
void parseSerial(String serial_packet_choice)
{
  fragment = cutFragment('s', 'S', serial_packet_choice);
  if (fragment != "bad") servo = (uint8_t)fragment.toInt();
  //SerialUSB.println("Servo: " + String(servo));

  fragment = cutFragment('m', 'M', serial_packet_choice);
  if (fragment != "bad") motors = (uint8_t)fragment.toInt() * 2;
  //SerialUSB.println("Motors: " + String(servo));

  fragment = cutFragment('a', 'A', serial_packet_choice);
  if (fragment != "bad") angle = fragment.toFloat();
  //SerialUSB.println("Angle: " + String(servo));

  fragment = cutFragment('v', 'V', serial_packet_choice);
  if (fragment != "bad") {
    verticalAngle = fragment.toFloat();
    stepperV.move(angleToSteps(verticalAngle));
  //SerialUSB.println("Rotor V: " + String(verticalAngle));
  }

  fragment = cutFragment('h', 'H', serial_packet_choice);
  if (fragment != "bad") {
     horizontalAngle = fragment.toFloat();
     stepperH.move(angleToSteps(horizontalAngle));
     //horizontalAngle = 0;
     //SerialUSB.println("Rotor H: " + String(verticalAngle));
  }
  //serial_packet = "";
  fragment = "";
}

String cutFragment(char openChar, char closeChar, String serial_packet)   // Cuts out variable strings from Serial message, eg. for args 'a', 'A': "<s1Sm0Ma230A>" -> "230"
{
  if (serial_packet.indexOf('<') >= 0 && serial_packet.indexOf('>') >= 0 &&           // Validate packet (should contain bounding characters '<' '>')
      serial_packet.indexOf(openChar) >= 0 && serial_packet.indexOf(closeChar) >= 0)  // Validate requested fragment (check for both bounding letters of variable)
  {
    return (serial_packet.substring(serial_packet.indexOf(openChar) + 1, serial_packet.indexOf(closeChar)));  // Cut fragment out
  }
  return "bad";   // Return this if fragment wasn't found in packet
}

static void decodePacket()    // Converts bytes from received radio package to variables (depentent on format of package)
{
  // This format: [0:3](pressure * 1000hPa), [4:7](temperature * 10*C), [8:11](latitude * 10^7), [12:15](longitude * 10^7), [16:18](euler angles mapped to byte)
  pressure = (float)(radio.rxBuffer[0] + (radio.rxBuffer[1] << 8) + (radio.rxBuffer[2] << 16) + (radio.rxBuffer[3] << 24)) / 1000.0;
  temperature = (float)(radio.rxBuffer[4] + (radio.rxBuffer[5] << 8) + (radio.rxBuffer[6] << 16) + (radio.rxBuffer[7] << 24)) / 10.0;
  latitude = (float)(radio.rxBuffer[8] + (radio.rxBuffer[9] << 8) + (radio.rxBuffer[10] << 16) + (radio.rxBuffer[11] << 24)) / 10000000.0;
  longitude = (float)(radio.rxBuffer[12] + (radio.rxBuffer[13] << 8) + (radio.rxBuffer[14] << 16) + (radio.rxBuffer[15] << 24)) / 10000000.0;
  yaw = (float)(radio.rxBuffer[16]) * (360.0 / 255.0);
  pitch = (float)(radio.rxBuffer[17]) * (360.0 / 255.0);
  roll = (float)(radio.rxBuffer[18]) * (360.0 / 255.0);
}

// Performance and debug
//SerialUSBprintln("[LoRa] Packet received, " + (String)bitrate + "b/s, " + (String)packrate + "P/s");
//SerialUSBprintln(String((int)radio.rxBuffer[radio.rxLen - 1]));


void rotorInit(){
  //Sets up speed, acceleration, enable pins for steppers
  stepperV.setEnablePin(STEPPERV_ENABLE_PIN);
  stepperV.setPinsInverted(false, false, true);
  stepperV.setMaxSpeed(3*1600.0);
  stepperV.setAcceleration(400.0);
  stepperV.enableOutputs();

  stepperH.setEnablePin(STEPPERH_ENABLE_PIN);
  stepperH.setPinsInverted(false, false, true);
  stepperH.setMaxSpeed(3*1600.0);
  stepperH.setAcceleration(500.0);
  stepperH.enableOutputs();
}

//Returns the amount of steps to be executed in order to move the rotor a given angles
int angleToSteps(float angle){
  return (int)microStepRate*400*127/13*angle/360;
}
