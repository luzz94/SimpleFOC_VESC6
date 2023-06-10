#include <Arduino.h>
#include <board_vesc_6.h>
#include <SimpleFOC.h>

const int motorPolePairs = 7;
float pot_value = 0;
float pot_value_deg = 0;
float pot_value_deg_prev = 0;
float pot_value_rad = 0;
float pot_value_rad_prev = 0;
float discard_tr = 0.5; 
int discard_nu = 5;
int discard_av = discard_nu;
uint32_t serial_cnt = 0;

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readMySensorCallback(){
 // read my sensor
  analogReadResolution(12);
  pot_value = analogRead(ADC_15);
  pot_value_deg = fmap(pot_value,800.0,3265.0,-10.0,190);
  //pot_value_rad = fmap(pot_value,800.0,3265.0,-(_PI/10),(_PI - 0.5));
  // return the angle value in radians in between 0 and 2PI

if ((abs(pot_value_deg - pot_value_deg_prev) > discard_tr) && (discard_av > 0))
{
  pot_value_deg = pot_value_deg_prev;
  discard_av--;
  if (discard_av <= 0) discard_av = 0;
}
else
{
  pot_value_deg_prev = pot_value_deg;
  discard_av = discard_nu;
}

 return pot_value_deg;
}

void initMySensorCallback(){
  pinMode(ADC_15, INPUT);  
}

BLDCMotor motor = BLDCMotor(motorPolePairs);
BLDCDriver6PWM driver(H1, L1, H2, L2, H3, L3, EN_GATE);
InlineCurrentSense currentSense = InlineCurrentSense(0.0005, 200, CURRENT_1, CURRENT_2, CURRENT_3);

GenericSensor reduction_sensor = GenericSensor(readMySensorCallback, initMySensorCallback);

// encoder instance
HallSensor hall_sensor = HallSensor(HALL_1, HALL_2, HALL_3, motorPolePairs);

// Interrupt routine intialisation
// channel A and B callbacks
void doA() {hall_sensor.handleA();}
void doB() {hall_sensor.handleB();}
void doC() {hall_sensor.handleC();}


float target = 0;                       // angle set point variable
Commander command = Commander(Serial);        // instantiate the commander

//CAN Bus Communication Instance
CANDriver canD = CANDriver(CAN_RX, CAN_TX);
CANCommander canCommand = CANCommander(canD);

void doCommander(char* cmd) { command.motor(&motor, cmd); }
void doCommanderCAN(char* cmd) { canCommand.motor(&motor, cmd); }

void setup() {

  hall_sensor.init();                              // initialize sensor hardware
  hall_sensor.enableInterrupts(doA, doB, doC);

  // initialize sensor hardware
  reduction_sensor.init();

  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = 26;           // power supply voltage [V]
  driver.voltage_limit = 20; // Max DC voltage allowed - default voltage_power_supply
  driver.dead_zone = 0.03; // daad_zone [0,1] - default 0.02 - 2%

  driver.init();

  motor.phase_resistance = 0.053; // [Ohm]
  motor.KV_rating = 140;
  //motor.voltage_limit = 1.5; // [V] - if phase
  motor.current_limit = 200;      // [Amps] - if phase resistance defined
  motor.velocity_limit = 20; // [rad/s] 5 rad/s cca 50rpm

  // link the motor to the sensors
  //motor.linkSensor(&hall_sensor);
  motor.linkVelSensor(&hall_sensor);
  motor.linkPosSensor(&reduction_sensor);
  // link the motor and the driver
  motor.linkDriver(&driver);

  
  // aligning voltage [V]
  // motor.voltage_sensor_align = 1.5;
  // index search velocity [rad/s]
  // motor.velocity_index_search = 10;

  currentSense.init();      // current sensing
  //currentSense.driverAlign(&driver, motor.voltage_sensor_align);
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // contoller configuration
  // default parameters in defaults.h

  // Q axis
  // PID parameters - default 
  motor.PID_current_q.P = 0.1f;                       // 3    - Arduino UNO/MEGA
  motor.PID_current_q.I = 1;                    // 300  - Arduino UNO/MEGA
  motor.PID_current_q.D = 0;
  motor.PID_current_q.limit = 0.1f; 
  motor.PID_current_q.output_ramp = 1e3;                  // 1000 - Arduino UNO/MEGA
  // Low pass filtering - default 
  motor.LPF_current_q.Tf = 0.1f;                         // 0.01 - Arduino UNO/MEGA

  // D axis
  // PID parameters - default 
  motor.PID_current_d.P = 1;                       // 3    - Arduino UNO/MEGA
  motor.PID_current_d.I = 10;                    // 300  - Arduino UNO/MEGA
  motor.PID_current_d.D = 0;
  motor.PID_current_d.limit = 0.1f; 
  motor.PID_current_d.output_ramp = 1e3;                  // 1000 - Arduino UNO/MEGA
  // Low pass filtering - default 
  motor.LPF_current_d.Tf = 0.1f;                         // 0.01 - Arduino UNO/MEGA


  motor.PID_velocity.P = 0.8f;
  motor.PID_velocity.I = 0.2f;
  motor.PID_velocity.D = 0.1f;
  motor.PID_velocity.output_ramp = 1e3;
  motor.LPF_velocity.Tf = 0.2f;

/*
  // Position parameter (hall sensors)
  motor.P_angle.P = 6;
  motor.P_angle.I = 0.8;
  motor.P_angle.D = 0.5;
  motor.P_angle.output_ramp = 1e4; // default 1e6 rad/s^2
      // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.1f;
*/

  // Position parameter (Gearbox sensor - 64 ratio )
  motor.P_angle.P = 150;
  motor.P_angle.I = 10;
  motor.P_angle.D = 0;
  motor.P_angle.output_ramp = 1e4; // default 1e6 rad/s^2
      // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.1f;

  // use monitoring with serial
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  SimpleFOCDebug::enable(NULL);


  motor.monitor_variables = _MON_TARGET | _MON_ANGLE; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE
  motor.monitor_downsample = 25;
  motor.motion_downsample = 1;
    // initialize motor

  Serial.println(F("Motor init."));
  motor.init();

  // align encoder and start FOC
  // Serial.println(F("Motor align."));

  motor.initFOC(3.14, Direction::CW);
  //motor.initFOC();
  Serial.println(F("Init FOC"));

  // add target command T
  command.add('M', onMotor, "my motor");
  canCommand.add('M', doCommanderCAN, (char*)"motor");

  Serial.println(F("Motor ready."));
  _delay(1000);
}

void loop() {

serial_cnt++;

  motor.loopFOC();
  motor.move();


  // CAN Bus Communication
  canCommand.runWithCAN();
  
  // user communication
  command.run();

  reduction_sensor.update();

  if(pot_value_deg > 180) motor.move(0);
  if(pot_value_deg < 0) motor.move(0);
  
  //motor.monitor();

if( serial_cnt > motor.monitor_downsample ) serial_cnt = 0;

if( serial_cnt == motor.monitor_downsample )
  {
    
    Serial.print("Data:");
    Serial.print(" ");
    Serial.print(pot_value_deg);
    Serial.print(" ");
    Serial.print(motor.shaft_velocity);
    Serial.println(" ");

    /*
    Serial.print(">shaft velocity:");
    Serial.println(motor.shaft_velocity);
    //Serial.print(">vel hall sensor calc:");
    //Serial.println(hall_sensor.vel_cal);
    Serial.print(">pulse_diff:");
    Serial.println(hall_sensor.pulse_diff);

    Serial.print(">target:");
    Serial.println(motor.target);
    //Serial.print(">shaft_angle:");
    //Serial.println(motor.shaft_angle);
    Serial.print(">pos:");
    Serial.println(pot_value_deg);

    
    Serial.print(">current a:");
    Serial.println(currentSense.getPhaseCurrents().a);
    Serial.print(">current b:");
    Serial.println(currentSense.getPhaseCurrents().b);
    Serial.print(">current c:");
    Serial.println(currentSense.getPhaseCurrents().c);
    
   
    Serial.print(">iq:");
    Serial.println(motor.current.q);
    Serial.print(">id:");
    Serial.println(motor.current.d);
    */
  }
}
