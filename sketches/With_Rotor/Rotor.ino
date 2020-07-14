/**************************************/
/*****Functions for rotor steering*****/
/**************************************/

//STEPPER CONNECTIONS:
//Connect the stepper wires to the goldpins in the middle
//so that the metal endings seen through holes in the plug face outwards

//initiates AccelStepper class objects
void rotorInit(){
  //Sets up speed, acceleration, enable pins for steppers
  if(!rotorCalibrated) return;
  stepperV.setEnablePin(STEPPERV_ENABLE_PIN);
  stepperV.setPinsInverted(false, false, true);
  stepperV.setMaxSpeed(STEPPERV_MAX_SPEED);
  stepperV.setAcceleration(STEPPERV_MAX_ACCELERATION);
  stepperV.enableOutputs();

  stepperH.setEnablePin(STEPPERH_ENABLE_PIN);
  stepperH.setPinsInverted(false, false, true);
  stepperH.setMaxSpeed(STEPPERH_MAX_SPEED);
  stepperH.setAcceleration(STEPPERH_MAX_ACCELERATION);
  stepperH.enableOutputs();

  stepperV.setCurrentPosition(0);
  stepperH.setCurrentPosition(angleToSteps(180));

  limitUp = angleToSteps(LIMIT_UP);     //limits in vertical movement in steps
  limitDown = angleToSteps(LIMIT_DOWN);

  rotorCalibrated = false; // back to false so that the rotor isn't initiated in loop every time
}

////////////////////////////////////////////////////////////////////////////////

//Returns the amount of steps to be executed in order to move the rotor a given angles
int angleToSteps(float angle){
  return (int)microStepRate*400*127/13*angle/360;
}

////////////////////////////////////////////////////////////////////////////////

// checks if the ordered movement doesn't exceed limitations in vertical axis
// it will make a movement if the target position doesn't exceed limit
void moveIfPossibleVertical(int movement){
  if(stepperV.targetPosition() >= limitUp && movement > 0) return;
  else if (stepperV.targetPosition() <= limitDown && movement < 0) return;
  else stepperV.move(movement);
}

/*void moveToIfPossibleHorizontal(int movement){ //movement is absolute desired position in steps
  if(stepperH.currentPosition() + movement > limitUp) {stepperH.moveTo(limitUp); return;}
  else if (stepperH.currentPosition() + movement < limitDown) {stepperH.moveTo(limitDown); return;}
  else stepperH.moveTo(movement);
}*/

void moveToIfPossibleVertical(int movement){ //movement is absolute desired position in steps
  if(movement >= limitUp) {stepperV.moveTo(limitUp); return;}
  else if (movement <= limitDown) {stepperV.moveTo(limitDown); return;}
  else stepperV.moveTo(movement);
}

////////////////////////////////////////////////////////////////////////////////

void steppers_run(){
  stepperV.run();                                     //tries to make a vertical / horizontal step
  stepperH.run();                                     //these have to be called as often as possible
}



////////////////////////////////////////////////////////////////////////////////


float RotorGPSHorizontalAngle(float lat,float lon,float lat2,float lon2){

    //lat = your current gps latitude.
    //lon = your current gps longitude.
    //lat2 = your destiny gps latitude.
    //lon2 = your destiny gps longitude.

    float teta1 = lat*M_PI/180;
    float teta2 = lat2*M_PI/180;
    float delta1 = (lat2-lat)*M_PI/180;
    float delta2 = (lon2-lon)*M_PI/180;

    //==================Heading Formula Calculation================//

    float y = sin(delta2) * cos(teta2);
    float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    float brng = atan2(y,x);
    brng = brng*180/M_PI;// radians to degrees
    brng = fmod((brng + 360), 360);
   // std::cout<< brng;
    return brng;
}

////////////////////////////////////////////////////////////////////////////////

static float altFromPressure(){
    return (44331.5 - 4946.62 * pow((pressure*100),(0.190263)))-rotor_alt;
}

  //////////////////////////////////////////////////////////////////////////////


static long double toRadians(const long double degree)
{
    // cmath library in C++
    // defines the constant
    // M_PI as the value of
    // pi accurate to 1e-30
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

long double distance(long double lat1, long double long1,
                     long double lat2, long double long2)
{
    // Convert the latitudes
    // and longitudes
    // from degree to radians.
    lat1 = toRadians(lat1);
    long1 = toRadians(long1);
    lat2 = toRadians(lat2);
    long2 = toRadians(long2);

    // Haversine Formula
    long double dlong = long2 - long1;
    long double dlat = lat2 - lat1;

    long double ans = pow(sin(dlat / 2), 2) +
                          cos(lat1) * cos(lat2) *
                          pow(sin(dlong / 2), 2);

    ans = 2 * asin(sqrt(ans));

    // Radius of Earth in
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    long double R = 6371;

    // Calculate the result
    ans = ans * R;

    return ans * 1000; //km to m
}

float RotorGPSVerticalAngle(){
  return (float)((atan(altFromPressure() / distance(latitude, longitude, rotor_lat, rotor_lon)))/M_PI*180);
}
