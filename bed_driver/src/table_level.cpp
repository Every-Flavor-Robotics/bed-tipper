#include <Adafruit_AHRS.h>
#include <Adafruit_LSM6DS33.h>
#include <Arduino.h>
#include <joybridge_receiver.h>
#include <motorgo_plink.h>

#define RAD_TO_DEG (57.2957795131)

// Create an instance of the MotorGo Plink driver
MotorGo::MotorGoPlink motorgo_plink;

// Get easy references to the motor/enconder channels
MotorGo::MotorChannel& motor1 = motorgo_plink.ch1;
MotorGo::MotorChannel& motor2 = motorgo_plink.ch2;
MotorGo::MotorChannel& motor3 = motorgo_plink.ch3;
MotorGo::MotorChannel& motor4 = motorgo_plink.ch4;

MotorGo::ChannelConfiguration ch1_config;
MotorGo::ChannelConfiguration ch2_config;
MotorGo::ChannelConfiguration ch3_config;
MotorGo::ChannelConfiguration ch4_config;

JoyBridge::JoyBridgeReceiver receiver;

// IMU objects
Adafruit_LSM6DS33 lsm6ds;
bool imu_initialized = false;
sensors_event_t accel_event, gyro_event, temp;
unsigned long last_imu_read = 0;
Adafruit_Madgwick filter;

// Table Leveling Algorithm
// Four linear actuators attached to four corners of the table
// We want one "real" foot on the ground always
// The other three feet will be adjusted to level the table

// First, choose the corner that is highest
// Next, lower the other three corners together, until we pass level in both
// axes
//

void setup()
{
  Serial.begin(115200);

  if (!imu_initialized)
  {
    // Start I2C bus
    Wire1.begin(HIDDEN_SDA, HIDDEN_SCL, 400000);
    bool lsm6ds_success = lsm6ds.begin_I2C(0x6a, &Wire1);
    // bool lis3mdl_success = lis3mdl.begin_I2C(0x1C, &Wire1);

    imu_initialized = true;
  }

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_416_HZ);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_416_HZ);

  ch1_config.power_supply_voltage = 12.0;
  ch2_config.power_supply_voltage = 12.0;
  ch3_config.power_supply_voltage = 12.0;
  ch4_config.power_supply_voltage = 12.0;

  ch1_config.voltage_limit = 12.0;
  ch2_config.voltage_limit = 12.0;
  ch3_config.voltage_limit = 12.0;
  ch4_config.voltage_limit = 12.0;

  motor1.init(ch1_config);
  motor2.init(ch2_config);
  motor3.init(ch3_config);
  motor4.init(ch4_config);

  if (!receiver.begin("bed-tipper"))
  {
    Serial.println("Receiver initialization failed!");
    while (true) delay(1000);
  }

  // Configure stopping behavior for the motors
  motor1.set_brake();
  motor2.set_brake();

  motor3.set_brake();
  motor4.set_brake();

  motor1.enable();
  motor2.enable();
  motor3.enable();
  motor4.enable();
}

long last_print = millis();
void loop()
{
  lsm6ds.getEvent(&accel_event, &gyro_event, &temp);

  filter.updateIMU(RAD_TO_DEG * gyro_event.gyro.x,
                   RAD_TO_DEG * gyro_event.gyro.y,
                   RAD_TO_DEG * gyro_event.gyro.z, accel_event.acceleration.x,
                   accel_event.acceleration.y, accel_event.acceleration.z,
                   (micros() - last_imu_read) / 1000000.0f);

  last_imu_read = micros();

  //

  //   receiver.loop();

  //   if (receiver.isConnected())
  //   {
  //     JoyBridge::JoystickData data = receiver.getJoystickData();
  //     if (abs(data.left_y) > 0.1)
  //     {
  //       motor1.set_power(data.left_y);
  //       motor2.set_power(data.left_y);
  //     }
  //     else
  //     {
  //       motor1.set_power(0);
  //       motor2.set_power(0);
  //     }

  //     if (abs(data.right_y) > 0.1)
  //     {
  //       motor3.set_power(data.right_x);
  //       motor4.set_power(data.right_x);
  //     }
  //     else
  //     {
  //       motor3.set_power(0);
  //       motor4.set_power(0);
  //     }
  //   }

  //   Print the gravity vector every 50 ms
  if (millis() - last_print > 50)
  {
    Serial.print("Gravity Vector: ");
    float x, y, z;
    // Compute the magnitude
    filter.getGravityVector(&x, &y, &z);
    float square_mag = x * x + y * y + z * z;

    // Print gravity vector with fixed width and precision
    char buf[64];
    snprintf(buf, sizeof(buf), "%8.4f, %8.4f, %8.4f, %8.4f", x, y, z,
             square_mag);
    Serial.println(buf);

    last_print = millis();
  }
}
