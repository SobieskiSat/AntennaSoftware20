/***********************************/
/*****General purpose functions*****/
/***********************************/


// reads data from buffers and sends them for further processing, if the packet is complete
void getSerial()
{
  char incoming;
  while (SerialUSB.available() > 0)                         // If there are bytes waiting to be read from Serial
  {
    incoming = SerialUSB.read();                            // Get one character
    if (readingUSB_packet) serialUSB_packet += incoming;    // Append character to packet

    if (incoming == '<')                                    // '<' character opens the packet
    {
      readingUSB_packet = true;                             // We want now to save characters
      serialUSB_packet = "";
      serialUSB_packet += incoming;
    }
    else if (incoming == '>') {
      readingUSB_packet = false;       // '>' character ends the packet
      parseSerial(serialUSB_packet);   //parses serial data immidiately when one packet is received
    }
  }

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
    }
    digitalWrite(LED_BUILTIN, led_state); led_state = !led_state;
  }
}

////////////////////////////////////////////////////////////////////////////////

// Parses Serial message, eg. "s1Sm0M" -> servo = 1, motors = 0
// takes an argument stating which string should be parsed
//with current code it always takes a single packets
//funcion sped up by ending exectution after successful parsing
void parseSerial(String serial_packet_choice)
{
  //rotor vertical - remote
  fragment = cutFragment('q', 'Q', serial_packet_choice);
  if (fragment != "bad") {
    verticalAngle = fragment.toFloat();
    moveIfPossibleVertical(angleToSteps(verticalAngle));
    //stepperV.move(angleToSteps(verticalAngle));
    return;
  //SerialUSB.println("Rotor V: " + String(verticalAngle));
  }

  //rotor horizontal - remote
  fragment = cutFragment('w', 'W', serial_packet_choice);
  if (fragment != "bad") {
      horizontalAngle= fragment.toFloat();
     stepperH.move(angleToSteps(horizontalAngle));
     return;
     //horizontalAngle = 0;
     //SerialUSB.println("Rotor H: " + String(verticalAngle));
  }

  //servo
  fragment = cutFragment('s', 'S', serial_packet_choice);
  if (fragment != "bad") {servo = (uint8_t)fragment.toInt(); return;}
  //SerialUSB.println("Servo: " + String(servo));

  //motors
  fragment = cutFragment('m', 'M', serial_packet_choice);
  if (fragment != "bad") {motors = (uint8_t)fragment.toInt() * 2; return;}
  //SerialUSB.println("Motors: " + String(servo));

  //angle
  fragment = cutFragment('d', 'D', serial_packet_choice);
  if (fragment != "bad") {angle = fragment.toFloat(); return;}
  //SerialUSB.println("Angle: " + String(servo));

  //rotor calibrated - remote
  fragment = cutFragment('c', 'C', serial_packet_choice);
  if (fragment != "bad") {rotorCalibrated = true; return;}

  //setting initial rotor position in space
  fragment = cutFragment('j', 'J', serial_packet_choice);
  if (fragment != "bad") {rotor_lat = fragment.toFloat(); return;}

  fragment = cutFragment('k', 'K', serial_packet_choice);
  if (fragment != "bad") {rotor_lon = fragment.toFloat(); return;}

  fragment = cutFragment('l', 'L', serial_packet_choice);           //alt from BMP in the future?
  if (fragment != "bad") {rotor_alt = fragment.toFloat(); return;}

  fragment = cutFragment('o', 'O', serial_packet_choice);           //updats operationMode which will be sent to our satellite
  if (fragment != "bad") {operationMode = fragment.toInt(); return;}

  fragment = cutFragment('n', 'N', serial_packet_choice);           //updats latitudeTarget which will be sent to our satellite
  if (fragment != "bad") {latitudeTarget = fragment.toFloat(); return;}

  fragment = cutFragment('e', 'E', serial_packet_choice);           //updats longitudeTarget which will be sent to our satellite
  if (fragment != "bad") {longitudeTarget = fragment.toFloat(); return;}

  fragment = cutFragment('a', 'A', serial_packet_choice);           //updats altitudeTarget which will be sent to our satellite
  if (fragment != "bad") {altitudeTarget = fragment.toFloat(); return;}

  //moveToIfPossible(angleToSteps(bearing(latitude, longitude, rotor_lat, rotor_lon)));

  fragment = "";
}



////////////////////////////////////////////////////////////////////////////////

String cutFragment(char openChar, char closeChar, String serial_packet)   // Cuts out variable strings from Serial message, eg. for args 'a', 'A': "<s1Sm0Ma230A>" -> "230"
{
  if (serial_packet.indexOf('<') >= 0 && serial_packet.indexOf('>') >= 0 &&           // Validate packet (should contain bounding characters '<' '>')
      serial_packet.indexOf(openChar) >= 0 && serial_packet.indexOf(closeChar) >= 0)  // Validate requested fragment (check for both bounding letters of variable)
  {
    return (serial_packet.substring(serial_packet.indexOf(openChar) + 1, serial_packet.indexOf(closeChar)));  // Cut fragment out
  }
  return "bad";   // Return this if fragment wasn't found in packet
}

////////////////////////////////////////////////////////////////////////////////

void sendUSBPacket(char openChar, char closeChar, String serial_packet){
  SerialUSB.print("<" + openChar + serial_packet + closeChar + ">");
}
