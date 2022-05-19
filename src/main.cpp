#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>

#include "utils.h"
#include "PIDController.h"
#include "IMUAdapter.h"
#include "Radio.h"

#define FLOAT_SIZE 8 // bytes

const float RAD2DEG = 180.0/PI;
const float DEG2RAD = PI/180.0;

/*------------------*/
enum Prop {LF, RF, LB, RB};
const int motor_pins[4] = {36, 34, 35, 37};

Servo motors[4];

IMUAdapter imu;
Radio radio;

PIDController controllers[3] = {
    PIDController(12, 0.2, 4.20),
    PIDController(12, 0.2, 4.20),
    PIDController(10.85, 0.1, 0.65)};

int mot_activated = 1;
float rot[3];  // [phi, theta, psi] mesuarement
float rotd[3]; // [phid, thetad, psid] desired

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

void update_controller_params(char byte1, char byte2)
{
  int selector = map(radio.inputs[FLY_MODE], 1000, 2000, 0, 6);
  Serial.print("selector: ");
  Serial.print(selector);
  int value = ((byte1 & 0x0f) << 8) + byte2;
  Serial.println(" ");
  // Serial.print(" value: ");
  // Serial.println(value);
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

void updatePIDParams() {
  long selector = lroundf(mapfloat(radio.inputs[FLY_MODE], 1000, 2000, 0, 2));

  float aux = mapfloat(radio.inputs[PARAM_ADJUST_1], 980, 1470, -0.1, 0.1);
  aux = abs(aux) < 0.1*0.15 ? 0 : aux;
  float value1 = aux*abs(aux);
  aux = mapfloat(radio.inputs[PARAM_ADJUST_2], 1080, 1900, -0.1, 0.1);
  aux = abs(aux) < 0.1*0.15 ? 0 : aux;
  float value2 = aux*abs(aux);

  switch (selector)
  {
  case 0:
    controllers[0].Kp += value1;
    controllers[0].Kp = sat(controllers[0].Kp, 0, 50);
    controllers[1].Kp += value1;
    controllers[1].Kp = sat(controllers[1].Kp, 0, 50);
    
    controllers[2].Kp += value2;
    controllers[2].Kp = sat(controllers[2].Kp, 0, 50);
    break;
  case 1:
    controllers[0].Kd += 0.4*value1;
    controllers[0].Kd = sat(controllers[0].Kd, 0, 20);
    controllers[1].Kd += 0.4*value1;
    controllers[1].Kd = sat(controllers[1].Kd, 0, 20);
    
    controllers[2].Kd += 0.4*value2;
    controllers[2].Kd = sat(controllers[2].Kd, 0, 20);
    break;
  case 2:
    controllers[0].Ki += 0.05*value1;
    controllers[0].Ki = sat(controllers[0].Ki, 0, 3);
    controllers[1].Ki += 0.05*value1;
    controllers[1].Ki = sat(controllers[1].Ki, 0, 3);
    
    controllers[2].Ki += 0.05*value2;
    controllers[2].Ki = sat(controllers[2].Ki, 0, 3);
    break;
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

void setup()
{
  Serial.begin(9600);
  radio.begin();
  imu.begin();

  for (int i = 0; i < 4; i++)
    motors[i].attach(motor_pins[i]);

  turnMotorsOff();

  delay(3000);

  if (radio.inputs[RadioInput::FLY_MODE] < 1200)
    loadPIDParams();
  printPIDParams();
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

  float pwm[4];
  motorIputConversion(
      pwm,
      radio.inputs[RadioInput::THROTTLE],
      pid_out[0],
      pid_out[1],
      pid_out[2]);

  for (int i = 0; i < 4; i++)
    pwm[i] = sat(pwm[i], 1100, 2000);

  if (mot_activated == 1)
  {
    Serial.println();
    for (int i = 0; i < 4; i++)
      motors[i].writeMicroseconds(pwm[i]);
  }
    
  if (mot_activated) 
  {
    if (radio.inputs[RadioInput::THROTTLE] < 1100 &&
        radio.inputs[RadioInput::ARM] < 1500) {
      mot_activated = 0;
      turnMotorsOff();
      savePIDParams();
    }
  }
  else if (radio.inputs[RadioInput::THROTTLE] < 1100 &&
      radio.inputs[RadioInput::ARM] > 1500) {
    mot_activated = 1;
  }

  updatePIDParams();

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