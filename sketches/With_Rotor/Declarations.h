#ifndef DECLARATIONS_H
#define DECLARATIONS_H


#include <Radiolot.h>
#include "config.h"

#include <MultiStepper.h>
#include <AccelStepper.h>
#include "rotor_definitions.h"

#define SENDPACKETSAMOUNT 2

#define BAD "@" //char or string to be returned, if a serial message is invalid
//replaced "bad", because it's shorter and speeds up parsing process
//We can possibly delete checking for presence of '<' and '>' characters, since they are not included

bool led_state = true;      // Used for blinking when packet received

SX1278 radio;               // Instance of SX1278 LoRa

uint8_t packetNumber = 0;
uint8_t incoming_count = 5; // Count of incoming packets in duplex, (eg. 5 received for 1 transmitted)
                            // satellite sends this number of packets and then listens for packet which has to be sent from antenna

uint8_t toSend[SENDPACKETSAMOUNT];                  // Buffer to be sent via radio
bool transmitting = false;          // Flag to be set during transmission

String fragment;            // Used in parsing PC Serial messages, contains values in Strings
String serialUSB_packet;    // Serial messages buffer (from PC)
String serial_packet;       // Serial messages buffer (from rotor remote)
bool readingUSB_packet = false; //indicators for receiving serial commands from PC and rotor remote via UART
bool reading_packet = false;    //true if the packet is not fully received yet
                                //false if the opening '<' character has not been received yet

// Transmitted variables (recieved via Serial)
uint8_t servo = 0;        // 0 (0b0) - off, 1 (0b1) - on [first BIT of transmitted package]
uint8_t motors = 0;       // 0 (0b00) - off, 2 (0b10) - on [second BIT of transmitted package]
float angle = 0;          // angle in degrees [second byte of transmitted package]

float latitudeTarget;
float longitudeTarget;
float altitudeTarget;

// Transmitted variables (recieved via Serial) (for antenna rotor)
float horizontalAngle = 0; //relative angle for the rotor to rotate in a given plane
float verticalAngle = 0;   //applied for primitive manual steering
bool rotorCalibrated = false; // true if the positive calibration feedback
                              // has been received from the remote

// Position of antenna rotor
// Are sent from PC in setup phase
float rotor_lat = 50.053530;  //in degrees
float rotor_lon = 19.935201;  //in degrees
float rotor_alt = 261.0;      //in meters

// Variables limiting rotor's vertical movements in software
int limitUp;
int limitDown;

// Received variables
float pressure;
float temperature;
float latitude;
float longitude;
//float altitude
float yaw;
float pitch;
float roll;

float smallSPS;
float bigSPS;
float operationMode;


//float presssBMP = 985; // was needed for tests
                      //might be used for altitude for rotor


//creates an object of AccelStepper class for each of the rotors
AccelStepper stepperV(AccelStepper::DRIVER, STEPPERV_STEP_PIN, STEPPERV_DIR_PIN);   // stepper for vertical movement
AccelStepper stepperH(AccelStepper::DRIVER, STEPPERH_STEP_PIN, STEPPERH_DIR_PIN);   // stepper for horizontal movement

#endif
