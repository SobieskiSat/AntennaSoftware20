#ifndef DECLARATIONS_H
#define DECLARATIONS_H


#include <Radiolot.h>
#include "config.h"

#include <MultiStepper.h>
#include <AccelStepper.h>
#include "rotor_definitions.h"

#define DATA_SERIAL_MILLIS 1000

#define BAD "@" //char or string to be returned, if a serial message is invalid
//replaced "bad", because it's shorter and speeds up parsing process
//We can possibly delete checking for presence of '<' and '>' characters, since they are not included

long previousSerialMillis = 0;

bool led_state = true;      // Used for blinking when packet received

SX1278 radio;               // Instance of SX1278 LoRa

uint8_t packetNumber = 0;
uint8_t incoming_count = 5; // Count of incoming packets in duplex, (eg. 5 received for 1 transmitted)
                            // satellite sends this number of packets and then listens for packet which has to be sent from antenna

bool transmitting = false;          // Flag to be set during transmission

String fragment;            // Used in parsing PC Serial messages, contains values in Strings
String serialUSB_packet;    // Serial messages buffer (from PC)
String serial_packet;       // Serial messages buffer (from rotor remote)
bool readingUSB_packet = false; //indicators for receiving serial commands from PC and rotor remote via UART
bool reading_packet = false;    //true if the packet is not fully received yet
                                //false if the opening '<' character has not been received yet

// Transmitted variables (recieved via Serial)
bool servo = 0;        // 0 (0b0) - off, 1 (0b1) - on [first BIT of transmitted package]
bool motors = 0;       // 0 (0b00) - off, 2 (0b10) - on [second BIT of transmitted package]
uint8_t operationMode = 31;
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
// Are sent from PC in setup phase      Everything is in Leszno center
float rotor_lat = 51.8335479;  //in degrees
float rotor_lon = 16.5317483;  //in degrees
float rotor_alt = 69.0;      //in meters, in Leszno

// Variables limiting rotor's vertical movements in software
int limitUp;
int limitDown;

// Received variables
float pressure = 1004.4; //in Leszno pressure
float temperature;
float latitude;
float longitude;
float altitude;
float yaw;
float pitch;
float roll;

float smallSPS;
float bigSPS;
uint8_t operationModeFB;  // FB - feedback

float altitudeFromPressure; //for starters CHANGE!!!


//float presssBMP = 985; // was needed for tests
                      //might be used for altitude for rotor
long lastRotorRefresh = 0;

//creates an object of AccelStepper class for each of the rotors
AccelStepper stepperV(AccelStepper::DRIVER, STEPPERV_STEP_PIN, STEPPERV_DIR_PIN);   // stepper for vertical movement
AccelStepper stepperH(AccelStepper::DRIVER, STEPPERH_STEP_PIN, STEPPERH_DIR_PIN);   // stepper for horizontal movement

#endif
