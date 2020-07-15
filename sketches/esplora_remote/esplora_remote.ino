#include <Esplora.h>

//INITIAL COMMENTS:

//if programming / flashing the chip does not work, try removing the wire
//connecting buttons' ground rail to arduino ground
//I really don't know why it works then, but it works lol

//STANDARIZE MOTOR AND SERVO ENABLE SIGNALS
//As for now I have no idea what 0 and 1 means in any of these situations
//We have to make it simple to use and understand

#define verticalMove "5"
#define horizontalMove "5"
#define angleMove 1

#define inputLen 2

#define calibrated "<c1C>"

#define HL "<q-" horizontalMove "Q>"
#define HR "<q" horizontalMove "Q>"

#define VU "<w-" verticalMove "W>"
#define VD "<w" verticalMove "W>"

#define SON "<s0S>"   //not sure in which order
#define SOFF "<s1S>"

#define MON "<m0M>"   //not sure in which order
#define MOFF "<m1M>"

uint8_t inputs[inputLen] = {14, 15};
/*inputs = {
  [0] left
  [1] down
  [2] right
  [3] up
  [4] +angle
  [5] -angle
  [6] motors on / off
  [7] servo on / off
  [8] rotor calibration
} */

int angle = 0;    //initial heading angle
bool servo = 0;   //initial servo state
bool motor = 0;   //initial motor state

bool previousServo;
bool previousMotor;

// becomes true after first calibration button pressed
// we might distinguish two distinctive commands to send
// first: enable horizontal stepper, it is already calibrated
// second: enable vertical stepper, it is already calibrated
// now it's just a single "CALIBRATION DONE, SIR!"
// after the button is pressed the second time
bool halfCalibrated = false;

int valueX;
int valueY;

bool is_low(uint8_t i);
void sendCommand(String command);

void setup(){
  for(int i=0; i<inputLen; i++) {
    pinMode(inputs[i], INPUT_PULLUP);
  }
  Serial1.begin(115200);
  //SerialUSB.begin(115200);
}

void loop() {
  if(Esplora.readJoystickX() > 300) sendCommand(HL);       //move left etc...
  if(Esplora.readJoystickY() > 300) sendCommand(VD);
  if(Esplora.readJoystickX() < -300) sendCommand(HR);
  if(Esplora.readJoystickY() < -300) sendCommand(VU);

  if(!Esplora.readButton(SWITCH_RIGHT)){                        //increment motor angle
    if(angle >= 360) angle = 0;
    angle += angleMove;

    sendCommand("<d" + String(angle) + "D>");
  }
  if(!Esplora.readButton(SWITCH_LEFT)){                        //decrement motor angle
    if(angle <= 0) angle = 360;
    angle -= angleMove;
    sendCommand("<d" + String(angle) + "D>");
  }

/*
  if(is_low(6)) {
    previousMotor = motor;
    motor = 0;              //will have to take into consideration initial settings, what we consider as ON
    if(motor != previousMotor)
      sendCommand(MON);       //turn motors on or off
  }
  else {
    previousMotor = motor;
    motor = 1;
    if(motor != previousMotor)
      sendCommand(MOFF);
  }

  if(is_low(7)) {
    previousServo = servo;
    servo = 0;
    if(servo != previousServo)
      sendCommand(SON);       //change servo state
  }
  else {
    previousServo = servo;
    servo = 1;
    if(servo != previousServo)
      sendCommand(SOFF);
  }
  */

  if(!Esplora.readButton(SWITCH_DOWN))
    if(halfCalibrated) sendCommand(calibrated); //send signal, that the rotor in its (0,0) position
    else {halfCalibrated = true; delay(500);}
}

bool is_low(uint8_t i){
  if(digitalRead(inputs[i]) == LOW) return true;
  else return false;
}

void sendCommand(String command){
  Serial1.print(command);
  //SerialUSB.println(command);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}
