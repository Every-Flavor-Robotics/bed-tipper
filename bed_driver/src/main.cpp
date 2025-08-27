#include <Adafruit_AHRS.h>
#include <Adafruit_LSM6DS33.h>
#include <Arduino.h>
#include <joybridge_receiver.h>
#include <motorgo_plink.h>

#include "engagement_estimator.cpp"

#define RAD_TO_DEG (57.2957795131)

float zero = 0.0f;

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
unsigned long last_imu_read = micros();
Adafruit_Madgwick filter;

// Current tilt

enum motion_state_t
{
  MOTION_STATE_GO_TO_ZERO,
  MOTION_STATE_GO_TO_ANGLE,
  MOTION_STATE_COMPLETE
};

// Since we don't have sensors to detect the positions of the linear actuators,
// we have to store that state based on previous motions. In right tilt state,
// the left actuators are contacting the ground, while the right ones are
// fully retracted. In level, all actuators are fully retracted.
// This affects the extension and retraction - we need to switch operation modes
enum tilt_state_t
{
  TILT_STATE_UNKNOWN,
  TILT_STATE_LEVEL,
  TILT_STATE_RIGHT,
  TILT_STATE_LEFT
};

struct motion_data_t
{
  float target_angle;
  tilt_state_t cur_tilt_state;
  tilt_state_t target_tilt_state;
  unsigned long start_time;
  motion_state_t state;
};

struct execute_motion_input_t
{
  motion_data_t motion;
  float zero;
  float roll;
  float angle_tolerance;
  unsigned long cur_time;
};

struct motor_commands_t
{
  float m1_command;
  float m2_command;
  float m3_command;
  float m4_command;
};

struct execute_motion_output_t
{
  motion_data_t motion;
  motor_commands_t commands;
};

// ROBOT STATE
tilt_state_t cur_tilt_state = TILT_STATE_UNKNOWN;
const float ANGLE_TOLERANCE = 0.005f;  // Radians

// Hard coded target angles
float angle_sweep[] = {5.0f,  10.0f, 3.0f,
                       -5.0f, -7.5f, 0.0f};  // 10.0f, 15.0f};
int cur_index = 0;
float angle_sweep_len = 6;

void imu_loop()
{
  lsm6ds.getEvent(&accel_event, &gyro_event, &temp);

  filter.updateIMU(RAD_TO_DEG * gyro_event.gyro.x,
                   RAD_TO_DEG * gyro_event.gyro.y,
                   RAD_TO_DEG * gyro_event.gyro.z, accel_event.acceleration.x,
                   accel_event.acceleration.y, accel_event.acceleration.z,
                   (micros() - last_imu_read) / 1000000.0f);

  last_imu_read = micros();
}

float dot_product(float x1, float y1, float x2, float y2)
{
  return (x1 * x2 + y1 * y2);
}

float cross_product(float x1, float y1, float x2, float y2)
{
  return (x1 * y2 - y1 * x2);
}

float get_roll()
{
  float x, y, z;
  filter.getGravityVector(&x, &y, &z);

  float dot = dot_product(x, z, 0.0f, 1.0f);
  float acos_val = acos(dot);

  // this constraint saves us from acos() returning nans
  if (dot >= 1.0)
  {
    acos_val = 0.0;
  }
  else if (dot <= -1)
  {
    acos_val = _PI;
  }

  return copysign(acos_val, cross_product(x, z, 0.0f, 1.0f));
}

execute_motion_output_t execute_go_to_zero(const execute_motion_input_t& input)
{
  execute_motion_output_t output;
  motion_data_t data = input.motion;

  // 15 seconds to retract ALL actuators
  if (input.cur_time - data.start_time > 15000)
  {
    output.commands.m1_command = 0;
    output.commands.m2_command = 0;
    output.commands.m3_command = 0;
    output.commands.m4_command = 0;

    data.cur_tilt_state = TILT_STATE_LEVEL;

    // Once all actuators are retracted, check if we need to go to the target
    // angle Or if the target was to level out.
    if (data.cur_tilt_state != data.target_tilt_state)
    {
      data.state = MOTION_STATE_GO_TO_ANGLE;
    }
    else
    {
      // Motion complete, robot is level
      data.state = MOTION_STATE_COMPLETE;
    }
  }
  else
  {
    // Retract all motors
    output.commands.m1_command = -1.0;
    output.commands.m2_command = -1.0;
    output.commands.m3_command = -1.0;
    output.commands.m4_command = -1.0;
  }

  output.motion = data;
  return output;
}

execute_motion_output_t execute_go_to_angle(const execute_motion_input_t& input)
{
  execute_motion_output_t output;
  motion_data_t motion = input.motion;
  float corrected_roll = input.roll - input.zero;
  float error = motion.target_angle - corrected_roll;
  motion.cur_tilt_state = motion.target_tilt_state;

  Serial.print("Error: ");
  Serial.print(error, 4);
  Serial.print(" | ");
  Serial.print("Angle tolerance: ");
  Serial.println(input.angle_tolerance, 4);

  // If we are within the angle tolerance
  if (fabs(error) < input.angle_tolerance)
  {
    // Set all motors to 0
    output.commands.m1_command = 0;
    output.commands.m2_command = 0;
    output.commands.m3_command = 0;
    output.commands.m4_command = 0;

    Serial.println("Target angle reached!");

    motion.state = MOTION_STATE_COMPLETE;
  }

  else
  {
    motion.state = MOTION_STATE_GO_TO_ANGLE;

    if (motion.target_tilt_state == TILT_STATE_RIGHT)
    {
      if (error > 0)
      {
        output.commands.m1_command = 0.0;
        output.commands.m2_command = 0.0;
        output.commands.m3_command = 1.0;
        output.commands.m4_command = 1.0;
      }
      else
      {
        output.commands.m1_command = 0.0;
        output.commands.m2_command = 0.0;
        output.commands.m3_command = -1.0;
        output.commands.m4_command = -1.0;
      }
    }
    else if (motion.target_tilt_state == TILT_STATE_LEFT)
    {
      if (error < 0)
      {
        output.commands.m1_command = 1.0;
        output.commands.m2_command = 1.0;
        output.commands.m3_command = 0.0;
        output.commands.m4_command = 0.0;
      }
      else
      {
        output.commands.m1_command = -1.0;
        output.commands.m2_command = -1.0;
        output.commands.m3_command = 0.0;
        output.commands.m4_command = 0.0;
      }
    }
    else
    {
      Serial.println("Unknown tilt state, stopping motors");
      output.commands.m1_command = 0.0;
      output.commands.m2_command = 0.0;
      output.commands.m3_command = 0.0;
      output.commands.m4_command = 0.0;
    }
  }

  output.motion = motion;
  return output;
}

execute_motion_output_t execute_motion(const execute_motion_input_t& input)
{
  motion_data_t data = input.motion;

  // If we are in the GO TO ZERO state, we need to retract all motors
  if (data.state == MOTION_STATE_GO_TO_ZERO)
  {
    return execute_go_to_zero(input);
  }
  else if (data.state == MOTION_STATE_GO_TO_ANGLE)
  {
    return execute_go_to_angle(input);
  }
  else
  {
    execute_motion_output_t output;

    output.motion = data;
    output.commands.m1_command = 0.0;
    output.commands.m2_command = 0.0;
    output.commands.m3_command = 0.0;
    output.commands.m4_command = 0.0;

    Serial.println("Motion complete!");
    return output;
  }
}

execute_motion_output_t start_motion(float cur_angle, float zero,
                                     float target_angle, float angle_tolerance,
                                     tilt_state_t cur_tilt_state,
                                     unsigned long cur_time)
{
  Serial.println("Starting new motion");
  execute_motion_input_t input;
  input.angle_tolerance = angle_tolerance;
  input.roll = cur_angle;
  input.zero = zero;
  input.cur_time = cur_time;

  motion_data_t data;
  data.target_angle = target_angle;
  data.cur_tilt_state = cur_tilt_state;
  data.start_time = cur_time;
  if (target_angle == 0)
  {
    data.target_tilt_state = TILT_STATE_LEVEL;  // Level state
  }
  else if (target_angle > 0.0f)
  {
    data.target_tilt_state = TILT_STATE_RIGHT;  // Tilt up
  }
  else
  {
    data.target_tilt_state = TILT_STATE_LEFT;  // Tilt down
  }

  //   Return to zero if we need to change tilt states
  // This avoids continuously increasing the actuator lengths
  if (data.target_tilt_state != data.cur_tilt_state &&
      data.cur_tilt_state != TILT_STATE_LEVEL)
  {
    data.state = MOTION_STATE_GO_TO_ZERO;
  }
  else
  {
    data.state = MOTION_STATE_GO_TO_ANGLE;
  }

  input.motion = data;
  return execute_motion(input);
}

execute_motion_output_t result;
unsigned long start;
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

  //   Warm up IMU for 3 seconds
  long start = millis();
  while (millis() - start < 3000)
  {
    imu_loop();
  }

  result = start_motion(get_roll(), zero, 0, ANGLE_TOLERANCE,
                        TILT_STATE_UNKNOWN, millis());

  //   Retract ALL motors all the way
  motor1.set_power(result.commands.m1_command);
  motor2.set_power(result.commands.m2_command);
  motor3.set_power(result.commands.m3_command);
  motor4.set_power(result.commands.m4_command);

  while (result.motion.state != MOTION_STATE_COMPLETE)
  {
    imu_loop();

    // Create a new input
    execute_motion_input_t input;
    input.angle_tolerance = ANGLE_TOLERANCE;
    input.cur_time = millis();
    input.roll = get_roll();
    input.zero = 0.0f;
    input.motion = result.motion;
    result = execute_motion(input);

    motor1.set_power(result.commands.m1_command);
    motor2.set_power(result.commands.m2_command);
    motor3.set_power(result.commands.m3_command);
    motor4.set_power(result.commands.m4_command);
  }

  // Stop all motors
  motor1.set_power(0);
  motor2.set_power(0);
  motor3.set_power(0);
  motor4.set_power(0);

  delay(100);
  zero = get_roll();

  result =
      start_motion(get_roll(), zero, angle_sweep[cur_index] / RAD_TO_DEG,
                   ANGLE_TOLERANCE, result.motion.cur_tilt_state, millis());
}

void loop()
{
  imu_loop();

  execute_motion_input_t input;
  input.angle_tolerance = ANGLE_TOLERANCE;
  input.cur_time = millis();
  input.roll = get_roll();
  input.zero = zero;
  input.motion = result.motion;
  result = execute_motion(input);

  motor1.set_power(result.commands.m1_command);
  motor2.set_power(result.commands.m2_command);
  motor3.set_power(result.commands.m3_command);
  motor4.set_power(result.commands.m4_command);

  if (result.motion.state == MOTION_STATE_COMPLETE)
  {
    Serial.println("Motion complete!");
    cur_index++;
    if (cur_index >= angle_sweep_len)
    {
      cur_index = 0;
    }

    //
    result =
        start_motion(get_roll(), zero, angle_sweep[cur_index] / RAD_TO_DEG,
                     ANGLE_TOLERANCE, result.motion.cur_tilt_state, millis());
  }
}
