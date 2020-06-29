#ifndef COMPONENTS_SX1278_H_
#define COMPONENTS_SX1278_H_

#include <stdbool.h>

#include <Arduino.h>
#include <SPI.h>

#define SX1278_MAX_PACKET	256
#define LR_VALIDATE_CRCERROR 1

#define LR_DIO0_Pin 12
#define LR_NSS_Pin 10

typedef enum _SX1278_MODE {
	SLEEP, STANDBY, TX, RX
} SX1278_Mode;

typedef struct
{
	uint32_t frequency;
	uint8_t power;
	uint8_t spreadingFactor;
	uint8_t codingRate;
	uint8_t bandWidth;
	uint8_t crc;
	uint16_t rxTimeoutSymb;
} SX1278_config;

typedef struct
{
	SX1278_config config;
	SX1278_Mode mode;

	//int reset;
	int dio0;
	int nss;

	uint8_t irqStatus;
	bool txDone;
	bool rxTimeout;
	bool rxDone;
	bool crcError;
	int rssi;
	bool newPacket;

	bool useDio0IRQ;
	bool pendingIRQ;

	uint8_t txLen;
	uint8_t rxLen;
	uint8_t rxBuffer[SX1278_MAX_PACKET];

} SX1278;

bool SX1278_init(SX1278* inst);
bool SX1278_transmit(SX1278* inst, uint8_t* txBuffer, uint8_t length);
bool SX1278_receive(SX1278* inst);

void SX1278_tx_mode(SX1278* inst);
void SX1278_tx_input(SX1278* inst, uint8_t* txBuffer, uint8_t length);
void SX1278_tx_push(SX1278* inst);
bool SX1278_tx_finish(SX1278* inst);

void SX1278_rx_mode(SX1278* inst);
bool SX1278_rx_get_packet(SX1278* inst);

void SX1278_sleep(SX1278* inst);
void SX1278_standby(SX1278* inst);
void SX1278_reset(SX1278* inst);

bool SX1278_dio0_IRQ(SX1278* inst);
void SX1278_update_IRQ_status(SX1278* inst);
void SX1278_clearLoRaIrq(SX1278* inst);

int SX1278_getRSSI(SX1278* inst);

#define SX1278_DEFAULT_TIMEOUT	3000

#define SX1278_CR_4_5	0x01
#define SX1278_CR_4_6 	0x02
#define SX1278_CR_4_7 	0x03
#define SX1278_CR_4_8	0x04

#define SX1278_CRC_DIS	0x00
#define SX1278_CRC_EN	0x01

#define SX1278_POWER_20DBM		0xFF
#define SX1278_POWER_17DBM		0xFC
#define SX1278_POWER_14DBM		0xF9
#define SX1278_POWER_11DBM		0xF6

#define SX1278_SF_6		6
#define SX1278_SF_7		7
#define SX1278_SF_8		8
#define SX1278_SF_9		9
#define SX1278_SF_10		10
#define SX1278_SF_11		11
#define SX1278_SF_12		12

#define SX1278_BW_7_8KHZ	0
#define SX1278_BW_10_4KHZ	1
#define SX1278_BW_15_6KHZ	2
#define SX1278_BW_20_8KHZ	3
#define SX1278_BW_31_2KHZ	4
#define SX1278_BW_41_7KHZ	5
#define SX1278_BW_62_5KHZ	6
#define SX1278_BW_125KHZ	7
#define SX1278_BW_250KHZ	8
#define SX1278_BW_500KHZ	9

#define LR_RegFifo                                  0x00
// Common settings
#define LR_RegOpMode                                0x01
#define LR_RegFrMsb                                 0x06
#define LR_RegFrMid                                 0x07
#define LR_RegFrLsb                                 0x08
// Tx settings
#define LR_RegPaConfig                              0x09
#define LR_RegPaRamp                                0x0A
#define LR_RegOcp                                   0x0B
// Rx settings
#define LR_RegLna                                   0x0C
// LoRa registers
#define LR_RegFifoAddrPtr                           0x0D
#define LR_RegFifoTxBaseAddr                        0x0E
#define LR_RegFifoRxBaseAddr                        0x0F
#define LR_RegFifoRxCurrentaddr                     0x10
#define LR_RegIrqFlagsMask                          0x11
#define LR_RegIrqFlags                              0x12
#define LR_RegRxNbBytes                             0x13
#define LR_RegRxHeaderCntValueMsb                   0x14
#define LR_RegRxHeaderCntValueLsb                   0x15
#define LR_RegRxPacketCntValueMsb                   0x16
#define LR_RegRxPacketCntValueLsb                   0x17
#define LR_RegModemStat                             0x18
#define LR_RegPktSnrValue                           0x19
#define LR_RegPktRssiValue                          0x1A
#define LR_RegRssiValue                             0x1B
#define LR_RegHopChannel                            0x1C
#define LR_RegModemConfig1                          0x1D
#define LR_RegModemConfig2                          0x1E
#define LR_RegSymbTimeoutLsb                        0x1F
#define LR_RegPreambleMsb                           0x20
#define LR_RegPreambleLsb                           0x21
#define LR_RegPayloadLength                         0x22
#define LR_RegMaxPayloadLength                      0x23
#define LR_RegHopPeriod                             0x24
#define LR_RegFifoRxByteAddr                        0x25
// I/O settings
#define REG_LR_DIOMAPPING1                          0x40
#define REG_LR_DIOMAPPING2                          0x41
// Version
#define REG_LR_VERSION                              0x42
// Additional settings
#define REG_LR_PLLHOP                               0x44
#define REG_LR_TCXO                                 0x4B
#define REG_LR_PADAC                                0x4D
#define REG_LR_FORMERTEMP                           0x5B
#define REG_LR_AGCREF                               0x61
#define REG_LR_AGCTHRESH1                           0x62
#define REG_LR_AGCTHRESH2                           0x63
#define REG_LR_AGCTHRESH3                           0x64
// IRQ Status settings
#define IRQ_LR_RXTIMEOUT							0x80
#define IRQ_LR_RXDONE								0X40
#define IRQ_LR_CRCERROR								0X20
#define IRQ_LR_VALIDHEAD							0X10
#define IRQ_LR_TXDONE								0X08
#define IRQ_LR_CADDONE								0X04
#define IRQ_LR_FHSSCHANGE							0X02
#define IRQ_LR_CADDETECTED							0X01

#endif /* COMPONENTS_SX1278_H_ */
