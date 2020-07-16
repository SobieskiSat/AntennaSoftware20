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

      if (!transmitting && radio.rxDone)                    // Prints data to PC but only when wans't transmitting during previous routine
      {
        decodePacket();                                     // Updates received variables from packet
        SerialUSB.println("p" + String(pressure, 2) +       // Prints received values via Serial to PC
                          "Pt" + String(temperature, 1) +   // format is like: x012345X
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

      if (packetNumber == incoming_count-1)  // If packet number (last byte of packet) is equal to count of incoming packets
      {                                                       // send packet then (satellite will listen for a while)
        transmitting = true;                                  // (counts must be configured equal on both radios for duplex to work)
        getSerial();                                          // Receive data from Serial to be sent to satellite

        preparePacket();
        SX1278_transmit(&radio, radio.txBuffer, radio.txLen); // Transmit packet
        packetNumber = 0;
      }
      else
      {
        SX1278_receive(&radio);                               // Start listening for packet
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

static void preparePacket()
{
  uint32_t temv = 0;
  // format: MOT-1/8, SER-1/8, OPMOD-6/8, ANG-1, LAT-4, LON-4, ALT-2
  radio.txBuffer[0] = ((uint8_t)(motors) << 0) |
                      ((uint8_t)(servo) << 1) |
                      ((uint8_t)(operationMode << 2));

  radio.txBuffer[1] = (uint8_t)(angle * 255.0 / 360.0);
  radio.txLen = 2;

  if (operationMode == 1)
  {
    floatToBytes(&(latitudeTarget), radio.txBuffer + 3);
    floatToBytes(&(longitudeTarget), radio.txBuffer + 7);

    temv = (uint32_t)(altitudeTarget * 10);
    memcpy(radio.txBuffer + 11, (uint8_t*)&temv, 2);
    radio.txLen = 13;
  }
}

static void decodePacket()    // Converts bytes from received radio package to variables (depentent on format of package)
{
  if (radio.rxLen == 23)
  {
    // format: TEMP-2, PRES-3, LAT-4, LON-4, ALT-2, YAW-1, PITCH-1, ROLL-1, SPS1-1, SPS10-1, OPMODE-1, PN-1
    uint32_t temv = 0;

    memcpy((uint8_t*)&temv, radio.rxBuffer + 0, 2);  // 0:1
    temperature = ((float)(temv) / 1000.0) - 10;

    pressure = (float)(temv) / 10000.0;
    memcpy((uint8_t*)&temv, radio.rxBuffer + 2, 3);  // 2:5

    bytesToFloat(radio.rxBuffer + 6, &latitude);   // 6:9
    bytesToFloat(radio.rxBuffer + 10, &longitude); // 10:13

    altitude = (float)(temv) / 10;
    memcpy((uint8_t*)&temv, radio.rxBuffer + 14, 2); // 14:15

    yaw = (float)(radio.rxBuffer[16]) * 360.0 / 255.0;    // 16
    pitch = (float)(radio.rxBuffer[17]) * 360.0 / 255.0;  // 17
    roll = (float)(radio.rxBuffer[18]) * 360.0 / 255.0;   // 18

    /*
    smallSPS = (float)(radio.rxBuffer[19]) * __ / 255.0;  // 19
    bigSPS = (float)(radio.rxBuffer[20]) * __ / 255.0;   // 20
    */

    operationModeFB = radio.rxBuffer[21];  // 21
    packetNumber = radio.rxBuffer[22];    // 22
  }
}

static inline void floatToBytes(float* value, uint8_t* buffer) { for (uint8_t i = 0; i < 4; i++) buffer[i] = *((uint8_t*)(value) + i); }
static inline void bytesToFloat(uint8_t* buffer, float* value) { for (uint8_t i = 0; i < 4; i++) *((uint8_t*)(value) + i) = buffer[i]; }
