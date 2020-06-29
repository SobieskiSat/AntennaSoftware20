#include <Radiolot.h>

static SX1278_config radio_config =
{
  434000000,  //frequency in Hz, resolution: 61.035Hz
  SX1278_POWER_17DBM,
  SX1278_SF_7,
  SX1278_CR_4_5,
  SX1278_BW_125KHZ,
  SX1278_CRC_DIS,
  100 // rxTimeout = val * 1.024ms (for SF=7, BW=125K) [rxTimeout = val * (2^(SF) / BW)]
};

SX1278 radio;
uint8_t sendBuffer[SX1278_MAX_PACKET];
uint8_t packetLen = 0;
uint32_t lastSend = 0;
float bitrate = 0;
float packrate = 0;

uint8_t counter = 0;
bool led_state = true;

void setup()
{
  SerialUSB.begin(115200);
  SPI.begin();
  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);

  radio.config = radio_config;
  radio.useDio0IRQ = false;
  
  SX1278_init(&radio);
}

void loop()
{
  memset(sendBuffer, 0x00, SX1278_MAX_PACKET);  // clear send buffer
  // prepare packet
  packetLen = 2;
  sendBuffer[0] = counter;
  sendBuffer[1] = 255 - counter;

  SX1278_transmit(&radio, sendBuffer, packetLen);  // transmit packet

  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;

  if (radio.txDone) // if transmit successful
  {
    packrate = 1000/(float)(millis() - lastSend);
    bitrate = packrate * packetLen * 8;
    SerialUSB.println("[LoRa] Packet sent, " + (String)bitrate + "b/s, " + (String)packrate + "P/s");
    
    radio.txDone = false;
    //SerialUSB.println((String)sendBuffer[0] + "\t" + (String)sendBuffer[1]);
  }
  
  lastSend = millis();  
  counter++;
  if (counter >= 255) counter = 0;
}
