#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Configuration
const uint32_t BAUDRATE = 115200;
const uint16_t LOOP_RATE_HZ = 100;
const uint32_t LOOP_PERIOD_MS = 1000 / LOOP_RATE_HZ;

uint32_t sequence = 0;

void setup()
{
    Serial.begin(BAUDRATE);
    Wire.begin();

    if (!bno.begin())
    {
        Serial.println("ERROR,BNO055_NOT_FOUND");

        while (1);
    }

    delay(1000);

    // Use external crystal if available
    bno.setExtCrystalUse(true);

    Serial.println("IMU_READY");
}

void loop()
{
    // Read Sensors

    imu::Quaternion quat = bno.getQuat();

    sensors_event_t gyroEvent;
    sensors_event_t accelEvent;
    sensors_event_t linearEvent;
    sensors_event_t gravityEvent;
    sensors_event_t magEvent;

    bno.getEvent(
        &gyroEvent,
        Adafruit_BNO055::VECTOR_GYROSCOPE);

    bno.getEvent(
        &accelEvent,
        Adafruit_BNO055::VECTOR_ACCELEROMETER);

    bno.getEvent(
        &linearEvent,
        Adafruit_BNO055::VECTOR_LINEARACCEL);

    bno.getEvent(
        &gravityEvent,
        Adafruit_BNO055::VECTOR_GRAVITY);

    bno.getEvent(
        &magEvent,
        Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Calibration

    uint8_t sysCal;
    uint8_t gyroCal;
    uint8_t accelCal;
    uint8_t magCal;

    bno.getCalibration(
        &sysCal,
        &gyroCal,
        &accelCal,
        &magCal);

    // Temperature

    int8_t temp = bno.getTemp();

    // Data Packet Header

    Serial.print("IMU,");

    Serial.print(sequence++);
    Serial.print(",");

    Serial.print(millis());
    Serial.print(",");

    // Quaternion

    Serial.print(quat.w(),6);
    Serial.print(",");

    Serial.print(quat.x(),6);
    Serial.print(",");

    Serial.print(quat.y(),6);
    Serial.print(",");

    Serial.print(quat.z(),6);
    Serial.print(",");

    // Gyroscope (rad/s)

    Serial.print(gyroEvent.gyro.x,6);
    Serial.print(",");

    Serial.print(gyroEvent.gyro.y,6);
    Serial.print(",");

    Serial.print(gyroEvent.gyro.z,6);
    Serial.print(",");

    // Accelerometer (m/s²)

    Serial.print(accelEvent.acceleration.x,6);
    Serial.print(",");

    Serial.print(accelEvent.acceleration.y,6);
    Serial.print(",");

    Serial.print(accelEvent.acceleration.z,6);
    Serial.print(",");


    // Linear Acceleration

    Serial.print(linearEvent.acceleration.x,6);
    Serial.print(",");

    Serial.print(linearEvent.acceleration.y,6);
    Serial.print(",");

    Serial.print(linearEvent.acceleration.z,6);
    Serial.print(",");

    // Gravity

    Serial.print(gravityEvent.acceleration.x,6);
    Serial.print(",");

    Serial.print(gravityEvent.acceleration.y,6);
    Serial.print(",");

    Serial.print(gravityEvent.acceleration.z,6);
    Serial.print(",");

    // Magnetometer (micro T)

    Serial.print(magEvent.magnetic.x,6);
    Serial.print(",");

    Serial.print(magEvent.magnetic.y,6);
    Serial.print(",");

    Serial.print(magEvent.magnetic.z,6);
    Serial.print(",");

    // Temperature

    Serial.print(temp);
    Serial.print(",");

    // Calibration

    Serial.print(sysCal);
    Serial.print(",");

    Serial.print(gyroCal);
    Serial.print(",");

    Serial.print(accelCal);
    Serial.print(",");

    Serial.println(magCal);

    delay(LOOP_PERIOD_MS);
}