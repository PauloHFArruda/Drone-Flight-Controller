#include "IMUAdapter.h"

#include <BMI160Gen.h>

#define GYRO_RANGE 250
#define ACC_OFFSET 2048
#define ACC_TO_G 1.0/16834.0
#define GYRO_TO_RAD_S 3.1415/180.0/131.2

IMUAdapter::IMUAdapter() {

}

void readAccelerometer(float acc[]) {
    long acc_raw[3];

    BMI160.readAccelerometer(acc_raw[0], acc_raw[1], acc_raw[2]);

    for (int i = 0; i < 3; i++)
        acc[i] = (float)acc_raw[i]/16834.0;
}

void readGyro(float gyro[]) {
    int gyro_raw[3];

    BMI160.readGyro(gyro_raw[0], gyro_raw[1], gyro_raw[2]);

    for (int i = 0; i < 3; i++)
        gyro[i] = (float)gyro_raw[i]*(3.1415/180.0)/131.2;
}

void IMUAdapter::begin() {
    BMI160.begin(BMI160GenClass::I2C_MODE);
    BMI160.setGyroRange(GYRO_RANGE);
    BMI160.autoCalibrateGyroOffset();

    float acc[3];
    readAccelerometer(acc);

    fusion.setup(acc[0], acc[1], acc[2]);

}

void IMUAdapter::update() {
    float acc[3], gyro[3];
    readAccelerometer(acc);
    readGyro(gyro);

    fusion.update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
}

void IMUAdapter::getRPY(float rot[]) {
    rot[0] = fusion.roll();
    rot[1] = fusion.pitch();
    rot[2] = fusion.yaw();
}

void IMUAdapter::printRotation() {
    Serial.print("Roll:");
    Serial.print(fusion.roll() * 180.0/3.1415);
    Serial.print(" Pitch:");
    Serial.print(fusion.pitch() * 180.0/3.1415);
    Serial.print(" Yaw:");
    Serial.print(fusion.yaw() * 180.0/3.1415);
    Serial.print(" ");
}