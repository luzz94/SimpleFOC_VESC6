#include "GenericSensor.h"


/*
  GenericSensor( float (*readCallback)() )
  - readCallback - pointer to the function which reads the sensor angle.
*/

GenericSensor::GenericSensor(float (*readCallback)(), void (*initCallback)()){
  // if function provided add it to the 
  if(readCallback != nullptr) this->readCallback = readCallback;
  if(initCallback != nullptr) this->initCallback = initCallback;
}

void GenericSensor::init(){
  // if init callback specified run it
  if(initCallback != nullptr) this->initCallback();
  this->Sensor::init(); // call base class init
}

void GenericSensor::update() {
    float val = getSensorAngle();
    angle_prev_ts = _micros();
    float d_angle = val - angle_prev;
    // if overflow happened track it as full rotation
    //if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

/*
	Shaft angle calculation
*/
float GenericSensor::getSensorAngle(){
  return this->readCallback();
}