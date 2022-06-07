#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>

#include "utils.h"
#include "PIDController.h"
#include "IMUAdapter.h"
#include "Radio.h"
#include "SerialComunication.h"

#define FLOAT_SIZE 8 // bytes

const float RAD2DEG = 180.0/PI;
const float DEG2RAD = PI/180.0;


enum Prop {LF, RF, LB, RB};
const int motor_pins[4] = {36, 34, 35, 37};

Servo motors[4];

IMUAdapter imu;
Radio radio;
SerialComunication esp;

PIDController controllers[3] = {
    PIDController(12, 0.2, 4.20),
    PIDController(12, 0.2, 4.20),
    PIDController(10.85, 0.1, 0.65)};

bool motorsActive = true;
float rot[3];  // [phi, theta, psi] mesuarement
float rotd[3]; // [phid, thetad, psid] desired
float pwm[4];

void EEPROM_writeFloat(int ee, float value)
{
  byte *p = (byte *)(void *)&value;
  for (unsigned int i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
}

float EEPROM_readFloat(int ee)
{
  float value = 0.0;
  byte *p = (byte *)(void *)&value;
  for (unsigned int i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return value;
}

void printPIDOut(float pid_out[])
{
  Serial.print(" Roll_pid:");
  Serial.print(pid_out[0]);
  Serial.print(" Pitch_pid:");
  Serial.print(pid_out[1]);
  Serial.print(" Yaw_pid:");
  Serial.print(pid_out[2]);
  Serial.print(" ");
}

void printRotRef(float rotd[])
{
  Serial.print(" Roll_ref:");
  Serial.print(rotd[0]*180/PI);
  Serial.print(" Pitch_ref:");
  Serial.print(rotd[1]*180/PI);
  Serial.print(" Yaw_ref:");
  Serial.print(rotd[2]*180/PI);
  Serial.print(" ");
}

void printPWM(float pwm[])
{
  Serial.print("pwm_L_F:");
  Serial.print(pwm[0]);
  Serial.print(" ");
  Serial.print("pwm_R_F:");
  Serial.print(pwm[1]);
  Serial.print(" ");
  Serial.print("pwm_L_B:");
  Serial.print(pwm[2]);
  Serial.print(" ");
  Serial.print("pwm_R_B:");
  Serial.print(pwm[3]);
}

void turnMotorsOff()
{
  for (int i = 0; i < 4; i++)
    motors[i].writeMicroseconds(1000);
}

void motorIputConversion(float pwm[], float Uz, float Uphi, float Utheta, float Upsi)
{
  pwm[Prop::LF] = 115 + Uz + Uphi - Utheta + Upsi;
  pwm[Prop::RF] = 115 + Uz - Uphi - Utheta - Upsi;
  pwm[Prop::LB] = 115 + Uz + Uphi + Utheta - Upsi;
  pwm[Prop::RB] = 115 + Uz - Uphi + Utheta + Upsi;
}

void readRotRef(float rotd[]) {
  rotd[0] = mapfloat((float)radio.inputs[ROLL], 1000, 2000, -10*DEG2RAD, 10*DEG2RAD);
  rotd[1] = mapfloat((float)radio.inputs[PITCH], 1000, 2000, -10*DEG2RAD, 10*DEG2RAD);
  rotd[2] = mapfloat((float)radio.inputs[YAW], 1000, 2000, -45*DEG2RAD, 45*DEG2RAD);
}

void filterRotRef(float rotd[]) {
  float temp;
  static float rotd_prev[2][3] = {{0, 0, 0}, {0, 0, 0}};
  for (int i = 0; i < 3; i++) {
    temp = rotd[i];
    rotd[i] = 0.4*rotd[i] + 0.3*rotd_prev[0][i] + 0.2*rotd_prev[1][i];
    rotd_prev[0][i] = temp;
    rotd_prev[1][i] = rotd_prev[0][i];
  }
}

void loadPIDParams() {
  for (int i = 0; i < 3; i++) {
    controllers[i].Kp = EEPROM_readFloat(FLOAT_SIZE*(3*i+0));
    controllers[i].Kd = EEPROM_readFloat(FLOAT_SIZE*(3*i+1));
    controllers[i].Ki = EEPROM_readFloat(FLOAT_SIZE*(3*i+2));
  }
}

void savePIDParams() {
  for (int i = 0; i < 3; i++) {
    EEPROM_writeFloat(FLOAT_SIZE*(3*i+0), controllers[i].Kp);
    EEPROM_writeFloat(FLOAT_SIZE*(3*i+1), controllers[i].Kd);
    EEPROM_writeFloat(FLOAT_SIZE*(3*i+2), controllers[i].Ki);
  }
}

void printPIDParams() {
  for (int i = 1; i < 3; i++) {
    // Serial.print("Controller ");
    // Serial.print(i+1);
    Serial.print(" Kp: ");
    Serial.print(controllers[i].Kp);
    Serial.print(" Kd: ");
    Serial.print(controllers[i].Kd);
    Serial.print(" Ki: ");
    Serial.print(controllers[i].Ki);
    // Serial.print("\n");
  }
}

void handleESPMsg(char msgType, float data[], int len) {
  Serial.print("MsgType: ");
  Serial.write(msgType);
  Serial.println();
  if (msgType < 70) {
    switch (msgType)
    {
    case MsgType::GET_PID_PARAMS:
      Serial2.write(MsgType::RESP_PID_PARAMS);
      for (int i = 0; i < 3; i++) {
        Serial2.print(controllers[i].Kp);
        Serial2.print(controllers[i].Ki);
        Serial2.print(controllers[i].Kd);
      }
      Serial2.write(MSG_END_CHAR);
      break;
    case MsgType::GET_ROT:
      Serial2.write(MsgType::RESP_ROT);
      for (int i = 0; i < 3; i++)
        Serial2.print(rot[i]);
      Serial2.write(MSG_END_CHAR);
      break;
    case MsgType::GET_PWM:
      Serial2.write(MsgType::RESP_PWM);
      for (int i = 0; i < 4; i++)
        Serial2.print(pwm[i]);
      Serial2.write(MSG_END_CHAR);
      break;
    case MsgType::GET_ROT_AND_PWM:
      Serial2.write(MsgType::RESP_ROT_AND_PWM);
      for (int i = 0; i < 3; i++)
        Serial2.print(rot[i]);
      for (int i = 0; i < 4; i++)
        Serial2.print(pwm[i]);
      Serial2.write(MSG_END_CHAR);
      break;

    default:
      break;
    }
  } else if (msgType < 80) {
    switch ((msgType-70)%3)
    {
    case 0:
      controllers[(msgType - 70)/3].Kp = data[0];
      break;
    case 1:
      controllers[(msgType - 70)/3].Ki = data[0];
      break;
    case 2:
      controllers[(msgType - 70)/3].Kd = data[0];
      break;
    
    default:
      break;
    }
  }
}


void setup()
{
  Serial.begin(115200);
  radio.begin();
  imu.begin();
  
  Serial2.begin(115200);
  esp.begin(handleESPMsg);

  // attach motors to pins
  for (int i = 0; i < 4; i++)
    motors[i].attach(motor_pins[i]);

  turnMotorsOff();

  delay(1000);

  if (radio.inputs[RadioInput::FLY_MODE] < 1200)
    loadPIDParams();

  printPIDParams();
  Serial.println();

  delay(2000);
}

void loop()
{
  static float dt, currentTime, timePrev;

  currentTime = millis();
  dt = (currentTime - timePrev) / 1000;
  timePrev = currentTime;

  imu.update();
  imu.getRPY(rot);

  readRotRef(rotd);
  filterRotRef(rotd);

  float pid_out[3];
  for (int i = 0; i < 3; i++)
  {
    pid_out[i] = 20*controllers[i].calc(rot[i], rotd[i], dt);
    //pid_out[i] = sat(pid_out[i], -400, 400);
  }

  motorIputConversion(
      pwm,
      radio.inputs[RadioInput::THROTTLE],
      pid_out[0],
      pid_out[1],
      pid_out[2]);

  for (int i = 0; i < 4; i++)
    pwm[i] = sat(pwm[i], 1100, 2000);

  if (motorsActive)
  {
    Serial.println();
    for (int i = 0; i < 4; i++)
      motors[i].writeMicroseconds(pwm[i]);
  }
    
  if (motorsActive) 
  {
    if (radio.inputs[RadioInput::THROTTLE] < 1100 &&
        radio.inputs[RadioInput::ARM] < 1500) {
      motorsActive = false;
      turnMotorsOff();
      savePIDParams();
    }
  }
  else if (radio.inputs[RadioInput::THROTTLE] < 1100 &&
      radio.inputs[RadioInput::ARM] > 1500) {
    motorsActive = true;
  }

  // comunicate with esp32
  while (Serial2.available()) {
    esp.updateBuffer(Serial2.read());
  }

  // imu.printRotation();
  // printPIDOut(pid_out);
  // printRotRef(rotd);
  // printPWM(pwm);
  // radio.printInputs();
  // printPIDParams();
  // radio.printInput(6);
  // radio.printInput(7);
  // Serial.print("Motor_activate:");
  // Serial.print(mot_activated);
  // Serial.print(" Kp:");
  // Serial.print(controllers[2].Kp);
  // Serial.print(" Kd:");
  // Serial.print(controllers[2].Kd);
  // Serial.println();
}

ISR(PCINT0_vect)
{
  radio.update();
}