/******************************************/
/*****Functions for radio transmission*****/
/******************************************/


void duplex_setup()
{
  SPI.begin();
  // Setup radio
  radio.config = sx1278_default_config;
  radio.useDio0IRQ = true;
  SX1278_init(&radio);
  SX1278_receive(&radio);
}

////////////////////////////////////////////////////////////////////////////////

bool check_LoRa_INT()
{
  return (radio.pendingIRQ && digitalRead(radio.dio0) == HIGH);
}

////////////////////////////////////////////////////////////////////////////////

void duplex_loop()
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
                          "Cr" + String(radio.rssi) +
                          "Ro" + String(operationMode) +
                          "Os" + String(smallSPS, 2) +         //approximation to be corrected
                          "Sb" + String(bigSPS, 2) +
                          "Bv" + String(stepperV.currentPosition()) +
                          "Vh" + String(stepperH.currentPosition()) +
                          "H");

        //sets a new position after a new packet is received
        stepperH.moveTo(angleToSteps(RotorGPSHorizontalAngle(latitude, longitude, rotor_lat, rotor_lon)));
        moveToIfPossibleVertical(angleToSteps(RotorGPSVerticalAngle()));

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
        //delay(10);                                            // [!!] Satellite seems to have problems with instantaneous reply so wait for a while
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

////////////////////////////////////////////////////////////////////////////////

static void decodePacket()    // Converts bytes from received radio package to variables (depentent on format of package)
{
  //after decoding the packet it sends the data to the computer
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
