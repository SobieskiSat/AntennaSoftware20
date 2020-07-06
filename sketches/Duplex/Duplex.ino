#include <Radiolot.h>
#include "config.h"

bool led_state = true;// Used for blinking when packet received

SX1278 radio;         // Instance of SX1278 LoRa 

uint8_t incoming_count = 5; // Count of incoming packets in duplex, (eg. 5 received for 1 transmitted)
                            // satellite sends this number of packets and then listens for packet which has to be sent from antenna

String fragment;      // Used in parsing PC Serial messages, contains values in Strings
String serial_packet; // Serial messages buffer (from PC)
uint8_t toSend[2];    // Buffer to be sent via radio
bool transmitting;    // Flag to be set during transmission

// Transmitted variables (recieved via Serial)
uint8_t servo;        // 0 (0b0) - off, 1 (0b1) - on [first BIT of transmitted package]
uint8_t motors;       // 0 (0b00) - off, 2 (0b10) - on [second BIT of transmitted package]
float angle;          // angle in degrees [second byte of transmitted package]

// Received variables
float pressure;
float temperature;
float latitude;
float longitude;
float yaw;
float pitch;
float roll;
uint8_t packetNumber;

void setup()
{
  // Setup arduino
  SerialUSB.begin(115200);
  SPI.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);

  // Setup radio
  radio.config = sx1278_default_config;
  radio.useDio0IRQ = true;
  SX1278_init(&radio);
  SX1278_receive(&radio);

  // Setup transmitted variables
  angle = 0;
  servo = 0;
  motors = 0;
  packetNumber = 0;
  transmitting = false;
}

void loop()
{
  if (radio.useDio0IRQ)
  {
    if (check_LoRa_INT())                                   // Manually check for interrupt
    {
      SX1278_dio0_IRQ(&radio);                              // Finish packet reception/transmission routine
      digitalWrite(LED_BUILTIN, led_state); led_state = !led_state; // toggle LED

      if (!transmitting && radio.rxDone)                                    // Prints data to PC but only when wans't transmitting during previous routine
      {
        decodePacket();                                     // Updates received variables from packet
        SerialUSB.println("p" + String(pressure, 2) +       // Prints received values via Serial to PC
                          "Pt" + String(temperature, 1) +   // format is like: x012345X
                          "Tx" + String(latitude, 7) +      // 'x' and 'X' are bounds for value
                          "Xy" + String(longitude, 7) +     // eg. p1023.97P sends pressure
                          "Ya" + String(yaw, 1) +
                          "Ab" + String(pitch, 1) +
                          "Bc" + String(roll, 1) +
                          "Cr" + String(radio.rssi) + "R");
        radio.rxDone = false;
      }
      else { ; } // (Maybe to implement) Sends transmitted data to PC
      transmitting = false;

      if (packetNumber == incoming_count)  // If packet number (last byte of packet) is equal to count of incoming packets
      {                                                       // send packet then (satellite will listen for a while)
        transmitting = true;                                  // (counts must be configured equal on both radios for duplex to work)
        getSerial();                                          // Receive data from Serial to be sent to satellite
        
        toSend[0] = motors + servo;                           // Prepare packet, two bytes for motor and servo state
        toSend[1] = (uint8_t)(angle * (255.0 / 360.0));       // Convert float to byte and place it as 2nd byte
        delay(10);                                            // [!!] Satellite seems to have problems with instantaneous reply so wait for a while 
        SX1278_transmit(&radio, toSend, 2);                   // Transmit packet
        packetNumber = 0;
      }
      else
      {
        SX1278_receive(&radio);                                       // Start listening for packet
      }
    }
  }
}

bool check_LoRa_INT()
{
	return (radio.pendingIRQ && digitalRead(radio.dio0) == HIGH);
}

void getSerial()
{
  char incoming;
  bool reading_packet = false;
  while (SerialUSB.available() > 0)                         // If there are bytes waiting to be read from Serial
  {
    incoming = SerialUSB.read();                            // Get one character
    if (reading_packet) serial_packet += incoming;          // Append character to packet
    
    if (incoming == '<')                                    // '<' character opens the packet
    {
      reading_packet = true;                                // We want now to save characters
      serial_packet = "";
      serial_packet += incoming;
    }
    else if (incoming == '>') reading_packet = false;       // '>' character ends the packet
  }
  
  parseSerial();
}

void parseSerial()  // Parses Serial message, eg. "s1Sm0M" -> servo = 1, motors = 0
{
  fragment = cutFragment('s', 'S');
  if (fragment != "bad") servo = (uint8_t)fragment.toInt();
  //SerialUSBprintln("Servo: " + String(servo));

  fragment = cutFragment('m', 'M');
  if (fragment != "bad") motors = (uint8_t)fragment.toInt() * 2;
  //SerialUSBprintln("Motors: " + String(servo));

  fragment = cutFragment('a', 'A');
  if (fragment != "bad") angle = fragment.toFloat();
  //SerialUSBprintln("Angle: " + String(servo));
}

String cutFragment(char openChar, char closeChar)   // Cuts out variable strings from Serial message, eg. for args 'a', 'A': "<s1Sm0Ma230A>" -> "230"
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
  packetNumber = radio.rxBuffer[radio.rxLen - 1];
}

// Performance and debug 
//SerialUSBprintln("[LoRa] Packet received, " + (String)bitrate + "b/s, " + (String)packrate + "P/s");
//SerialUSBprintln(String((int)radio.rxBuffer[radio.rxLen - 1]));
