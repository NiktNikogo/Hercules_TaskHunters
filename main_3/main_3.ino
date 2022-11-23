#include <SoftwareSerial.h>
#include "enums.hpp"
#include <Servo.h>
#include <Arduino.h>
#include "MyServo.hpp"

#define IS_DEBUG false

#if IS_DEBUG
#define DEBUG_PRINT(x) Serial.print(x);
#define DEBUG_PRINTLN(x) Serial.println(x);
#else
#define DEBUG_PRINT(x) ;
#define DEBUG_PRINTLN(x) ;
#endif

#define dirPin 31
#define stepPin 29

#define dir_11 5
#define dir_21 9
#define pwm_1 7

#define dir_12 27
#define dir_22 25
#define pwm_2_1 23
#define pwm_2_2 10

#define LOAD_SIZE 6

static char HEX_LUT[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
char load[6] = {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000};
int potents[2] = {127, 127};     // 0 - L, 1 - R
int joy_l[3] = {0, 0, 0};        // 0 - h, 1 - v, 2 - b3;
int joy_r[3] = {0, 0, 0};        // 0 - h, 1 - v, 2 - b3;
bool triggers[2] = {true, true}; // 0 - L, 1 - R
// 0 - square, 1 - cross, 2 - circle, 3 - triangle
bool buttons[4] = {false, false, false, false};

MotorsState motors_state = MotorsState::NONE;
DriveDir drive_dir = DriveDir::NONE;
int _step = 0;
bool stopped = false;
bool locked = false;
int which = 2;
bool got_data = false;
int test = 0;

DrivingDir driving_dir = DrivingDir::NONE;
float drive_speed = 0;
float prev_driving_speed = 0;
float drive_detla = 0.005f;
float stopping_delta = 0.001f;
float breaking_delta = 0.02f;
uint64_t drive_current;
uint64_t drive_prev;
//const uint64_t drive_icc_delta =R

uint64_t stepper_current;
uint64_t stepper_prev;
bool stepper_state = false;
bool can_stepper = true;
StepperOffset stepper_offset_state = StepperOffset::NOT_READ;
int stepper_offset_start = 128;
int stepper_offset_end = 128;
int steps_buffer = 0; 
uint64_t stepper_delta = 1500;
 

uint8_t rchash;
uint8_t rclen;
uint8_t rcid;
bool data_mode = false;
int min_speed = 1000;
int start_speed = min_speed;

uint64_t servo_current;
uint64_t servo_prev;
uint64_t servo_delta = 30;
const int32_t servo_closed = 108;
const int32_t servo_opened = 80;

MyServo s0(90, 0, 180);
MyServo s1(90, 45, 135);
MyServo s2(servo_opened, servo_opened -1, servo_closed + 1);

int32_t servo_angles[3] = {90, 87, servo_opened};
bool can_servo = false;

uint64_t delta_time = 0;
uint64_t prev_time = 0;

bool read_data_safe()
{
  static uint8_t data[256];

  if (data_mode)
  {
    if (Serial1.available() < rclen)
    {
      return false;
    }

    uint8_t hash = 0;
    for (size_t i = 0; i < rclen; i++)
    {
      uint8_t tmp = Serial1.read();
      data[i] = tmp;
      hash ^= tmp;
    }
    if (rchash != hash)
    {
      return false;
    }

    for (size_t i = 0; i < rclen; i++)
    {
      load[i] = data[i]; // remember about it
    }

    Serial1.write(0x56);
    Serial1.write(0x78);
    Serial1.write(rcid);
    Serial1.write(rchash);

    data_mode = false;
    return true;
  }
  else
  {
    if (Serial1.available() < 5)
    {
      return false;
    }
    uint8_t rc0 = Serial1.read();
    uint8_t rc1 = Serial1.read();
    rcid = Serial1.read();
    rchash = Serial1.read();
    rclen = Serial1.read();

    if (rc0 != 0x12 || rc1 != 0x34)
    {
      return false;
    }
    data_mode = true;
    return false;
  }
}
bool read_data()
{
  return read_data_safe();
}

void print_data()
{
  DEBUG_PRINT("Load");
  for (int i = 0; i < LOAD_SIZE; i++)
  {
    DEBUG_PRINT(HEX_LUT[((load[i] >> 4) & 0xF)]);
    DEBUG_PRINT(HEX_LUT[(load[i] & 0xF)]);
  }
  DEBUG_PRINT(" Potents: ");
  for (int i = 0; i < 2; i++)
    DEBUG_PRINT(String(potents[i]) + " ");
  DEBUG_PRINT("L-joy: ");
  for (int i = 0; i < 3; i++)
    DEBUG_PRINT(String(joy_l[i]) + " ");
  DEBUG_PRINT("R-joy: ");
  for (int i = 0; i < 3; i++)
    DEBUG_PRINT(String(joy_r[i]) + " ");
  DEBUG_PRINT("Triggers: ");
  for (int i = 0; i < 2; i++)
    DEBUG_PRINT(String(triggers[i]) + " ");
  DEBUG_PRINT("Buttons: ");
  for (int i = 0; i < 4; i++)
    DEBUG_PRINT(String(buttons[i]) + " ");
  DEBUG_PRINTLN("");
}
void digest_data()
{
  /*
  load_new[0] <- pierwsze 8 bitów potencjometra 0
  load_new[1] <- drugie 8 bitów potencjometra 0
  load_new[2] <- pierwsze 8 bitów potencjometra 0
  load_new[3] <- drugie 8 bitów potencjometra 0
  load_new[4] <- dwa pierwsze lewy X, dwa drugie lewy Y, dwa trzecie prawy X, dwa czwarte prawy Y
  load_new[5] <- dwa pierwsze L3, R3, dwa drugie TL, TR, 4 ostatnie guziki
  */

  potents[0] = load[0] << 8;
  potents[0] += load[1];
  if (potents[0] < 0)
    potents[0] = potents[0] + 256;
  potents[1] = load[2] << 8;
  potents[1] += load[3];
  if (potents[1] < 0)
    potents[1] = potents[1] + 256;

  joy_r[1] = (load[4] >> 6) & 0b00000011;
  joy_r[0] = (load[4] >> 4) & 0b00000011;
  joy_l[1] = (load[4] >> 2) & 0b00000011;
  joy_l[0] = (load[4] >> 0) & 0b00000011;

  joy_r[2] = (load[5] >> 7) & 1;
  joy_l[2] = (load[5] >> 6) & 1;

  triggers[0] = (load[5] >> 5) & 1;
  triggers[1] = (load[5] >> 4) & 1;

  buttons[0] = (load[5] >> 3) & 1;
  buttons[1] = (load[5] >> 2) & 1;
  buttons[2] = (load[5] >> 1) & 1;
  buttons[3] = (load[5] >> 0) & 1;
}

int calc_stepper_offset(int p)
{
  return -p + 128;
}

void move_stepper()
{
  //    // 400 to obrót o 180 stopni
  const int maxRot = 400 * 43 / 10;
  int potent_corrected_value = constrain(abs(256 - potents[0]), 0, 256);
  potent_corrected_value =
      constrain(potent_corrected_value, 0, 256);
  //int steps = int(map(potent_corrected_value, 0, 256, -maxRot, maxRot));
  int offset_steps = int(map(stepper_offset_start - stepper_offset_end, -256, 256, -maxRot, maxRot));
  //Serial.println(offset_steps);
  if (offset_steps != 0)
  {
    stepper_offset_start = stepper_offset_end;
    steps_buffer += offset_steps;
  }
  int delta = steps_buffer >= 0 ? 1 : -1;
  //Serial.println("offset: " + String(stepper_offset) + "| calculated: " + String(potent_corrected_value) + "| steps: " + String(steps) + "| _step: " + String(_step));
  digitalWrite(dirPin, delta >= 0 ? LOW : HIGH);
  if (steps_buffer)
  {
    stepper_current = micros();
    if (stepper_current - stepper_prev > stepper_delta)
    {
      digitalWrite(stepPin, stepper_state ? LOW : HIGH);
      stepper_state = !stepper_state;
      stepper_prev = stepper_current;
      steps_buffer -= delta;
    }
  }
  else
  {
    start_speed = min_speed;
  }
  //DEBUG_PRINTLN(String(_step) + " " + String(delta) + " " + String(steps));
}

void set_speed(int i)
{
  if (i == 0)
  {
    analogWrite(pwm_1, 0);
  }
  else
  {
    int vel = map(constrain(256 - potents[1], 0, 256), 0, 256, 0, 256);
    analogWrite(pwm_1, vel);
  }
}

float get_potents_speed() {
  return constrain(256 - potents[1], 0, 256)/256.0f;
}

void set_soft_speed(float t)
{
  //int vel = ilerp(t, 0, constrain(256 - potents[1], 0, 256));
  //Serial.println(vel);
  analogWrite(pwm_1, t * 255);
}

int32_t ilerp(float t, int _min, int _max)
{
  if (t >= 1)
    t = 1;
  if (t <= 0)
    t = 0;
  return int32_t(float(_max) * t + float(_min) * (1.0 - t));
}

void halt()
{
}
void drive(int dir)
{
  if (!triggers[1])
  {
    motors_state = MotorsState::DRIVING;
    drive_dir = DriveDir::FORWARD;
    driving_dir = DrivingDir::FORWARD;
  }
  if (!triggers[0])
  {
    motors_state = MotorsState::DRIVING_BACKWARD;
    drive_dir = DriveDir::BACKWARD;
    driving_dir = DrivingDir::BACKWARDS;
  }
  if (triggers[1] && triggers[0])
  {
    driving_dir = DrivingDir::NONE;
    motors_state = MotorsState::NONE;
    drive_dir = DriveDir::NONE;
  }
}

void move_wheels()
{
  drive_current = millis();
  uint64_t delta = drive_current - drive_prev;
  drive_prev = drive_current;

  bool is_forward = false;
  bool is_backwards = false;
  switch (motors_state)
  {
  case MotorsState::DRIVING:
    if (drive_dir == DriveDir::FORWARD)
    {
      is_forward = true;
    }
    break;
  case MotorsState::ROTATING:
    if ((drive_dir == DriveDir::RIGHT && driving_dir == DrivingDir::FORWARD) ||
        (drive_dir == DriveDir::LEFT && driving_dir == DrivingDir::BACKWARDS))
    {
      digitalWrite(dir_11, LOW);
      digitalWrite(dir_21, LOW);
    }
    else if ((drive_dir == DriveDir::LEFT && driving_dir == DrivingDir::FORWARD) ||
             (drive_dir == DriveDir::RIGHT && driving_dir == DrivingDir::BACKWARDS))
    {

      digitalWrite(dir_11, HIGH);
      digitalWrite(dir_21, HIGH);
    }
    if (driving_dir != DrivingDir::BACKWARDS && driving_dir != DrivingDir::FORWARD)
    {
      set_speed(0);
    }
    break;
  case MotorsState::DRIVING_BACKWARD:
    if (drive_dir == DriveDir::BACKWARD)
    {
      is_backwards = true;
    }
    break;
  case MotorsState::NONE:
    break;
  }

  float max_speed = get_potents_speed();
  float scale_drive_delta = max_speed * drive_detla;
  
  if (driving_dir == DrivingDir::FORWARD)
  {
    if (prev_driving_speed < 0)
    {
      if (drive_speed < 0)
      {
        drive_speed += min(-drive_speed, breaking_delta * delta);
      }
      else
      {
        drive_speed += min(-drive_speed, stopping_delta * delta);
      }
    }
    else
    {
      drive_speed += scale_drive_delta * delta;
    }
    if(drive_speed > 0) {
      prev_driving_speed = drive_speed;
    }
  }
  else if (driving_dir == DrivingDir::BACKWARDS)
  {
    if (prev_driving_speed > 0)
    {
      if (drive_speed > 0)
      {
        drive_speed -= min(drive_speed, breaking_delta * delta);
      }
      else
      {
        drive_speed -= min(drive_speed, stopping_delta * delta);
      }
    }
    else
    {
      drive_speed -= scale_drive_delta * delta;
    }
    if(drive_speed < 0) {
      prev_driving_speed = drive_speed;
    }
  }
  else
  {
    if (drive_speed > 0)
    {
      drive_speed -= min(drive_speed, stopping_delta * delta);
    }
    else
    {
      drive_speed += min(-drive_speed, stopping_delta * delta);
    }
    prev_driving_speed = drive_speed;
  }
  if (drive_speed >= max_speed)
    drive_speed = max_speed;
  if (drive_speed <= -max_speed)
    drive_speed = -max_speed;
  if (motors_state != MotorsState::ROTATING)
  {
    if (drive_speed < 0)
    {
      digitalWrite(dir_11, LOW);
      digitalWrite(dir_21, HIGH);
    }
    else
    {
      digitalWrite(dir_11, HIGH);
      digitalWrite(dir_21, LOW);
    }
  }
  set_soft_speed(fabs(drive_speed));
}

void move_acctuator(Acctuar acctuar, AcctuarDir dir)
{
  static float angles_coef[2] = {0.005, 0.0025};
  static AcctuarPin accLut[2] = {
      AcctuarPin::UPPER,
      AcctuarPin::LOWER};
  static AcctuarDirPin dirLut[2] = {
      AcctuarDirPin::UPPER,
      AcctuarDirPin::LOWER};
  static uint8_t valLut[3] = {
      0, //default
      LOW,
      HIGH,
  };
  if (dir != AcctuarDir::NONE)
  {
    if (acctuar != Acctuar::UPPER)
    {
      if (dir == AcctuarDir::BACKWARDS)
        dir = AcctuarDir::FORWARD;
      else if (dir == AcctuarDir::FORWARD)
        dir = AcctuarDir::BACKWARDS;
    }
  }
  digitalWrite(
      static_cast<uint8_t>(accLut[static_cast<size_t>(acctuar)]),
      dir != AcctuarDir::NONE ? HIGH : LOW);
  digitalWrite(
      static_cast<uint8_t>(dirLut[static_cast<size_t>(acctuar)]),
      valLut[static_cast<size_t>(dir)]);
  switch (dir)
  {
  case AcctuarDir::FORWARD:
    //servo_angles[0] += angles_coef[static_cast<size_t>(acctuar)] * delta_time;
    s0.change(+angles_coef[static_cast<size_t>(acctuar)]);
    break;
  case AcctuarDir::BACKWARDS:
    //servo_angles[0] -= angles_coef[static_cast<size_t>(acctuar)] * delta_time;
    s0.change(-angles_coef[static_cast<size_t>(acctuar)]);
    break;
  default:
    break;
  }
}

void stop_acctuators()
{
  digitalWrite(static_cast<uint8_t>(AcctuarPin::UPPER), LOW);
  digitalWrite(static_cast<uint8_t>(AcctuarPin::LOWER), LOW);
}
void move_servo(ServoPins which, int32_t angle)
{
  /*angle = constrain(angle, 0, 180);
  switch (which)
  {
  case ServoPins::FIRST:
    s0.write(angle);
    servo_angles[static_cast<size_t>(Servos::FIRST)] = angle;
    break;
  case ServoPins::SECOND:
    s1.write(angle);
    servo_angles[static_cast<size_t>(Servos::SECOND)] = angle;
    break;
  case ServoPins::THRID:
    s2.write(angle);
    servo_angles[static_cast<size_t>(Servos::THRID)] = angle;
  default:
    break;
  }*/
}

MyServo &get_servo(ServoPins which)
{
  // s0 s1 s2 are globals
  switch (which)
  {
  case ServoPins::FIRST:
    return s0;
  case ServoPins::SECOND:
    return s1;
  case ServoPins::THRID:
    return s2;
  }
}
void change_servo(ServoPins which, int32_t delta)
{
  get_servo(which).change(delta);
}
void set_servo(ServoPins which, int32_t value)
{
  get_servo(which).set(value);
}

void servo()
{
  if (can_servo)
  {
    servo_current = millis();
    if (abs(servo_current - servo_prev) >= servo_delta)
    {
      if (joy_r[1] == 1)
      {
        change_servo(ServoPins::FIRST, 1);
        //move_servo(ServoPins::FIRST, servo_angles[static_cast<size_t>(Servos::FIRST)] + 1);
      }
      else if (joy_r[1] == 2)
      {
        change_servo(ServoPins::FIRST, -1);
        //move_servo(ServoPins::FIRST, servo_angles[static_cast<size_t>(Servos::FIRST)] - 1);
      }
      if (joy_r[0] == 1)
      {
        change_servo(ServoPins::SECOND, -1);
        //move_servo(ServoPins::SECOND, servo_angles[static_cast<size_t>(Servos::SECOND)] + 1);
      }
      else if (joy_r[0] == 2)
      {
        change_servo(ServoPins::SECOND, 1);
        //move_servo(ServoPins::SECOND, servo_angles[static_cast<size_t>(Servos::SECOND)] - 1);
      }
      servo_prev = servo_current;
    }
    if (!joy_r[2])
    {
      //Serial.println("closed");
      //move_servo(ServoPins::THRID, servo_closed);
      set_servo(ServoPins::THRID, servo_closed);
    }
    else
    {
      //Serial.println("opened");
      //move_servo(ServoPins::THRID, servo_opened);
      set_servo(ServoPins::THRID, servo_opened);
    }
  }

  //move_servo(ServoPins::FIRST, servo_angles[static_cast<size_t>(Servos::FIRST)]);
  //move_servo(ServoPins::SECOND, servo_angles[static_cast<size_t>(Servos::SECOND)]);
  //move_servo(ServoPins::THRID, servo_angles[static_cast<size_t>(Servos::THRID)]);
  //DEBUG_PRINTLN(String(servo_angles[0]) + " " + String(servo_angles[1]) + String(servo_angles[2]));
}
void acctuators()
{
  //AcctuarController _which = static_cast<AcctuarController>(which);
  //AcctuarControllerDir dir = static_cast<AcctuarControllerDir>(joy_r[1]);
  AcctuarDir upper_dir = static_cast<AcctuarDir>(joy_r[1]);
  AcctuarDir lower_dir = static_cast<AcctuarDir>(joy_l[1]);
  if (can_servo)
  {
    move_acctuator(Acctuar::LOWER, lower_dir);
  }
  else {
    move_acctuator(Acctuar::LOWER, upper_dir);
    move_acctuator(Acctuar::UPPER, lower_dir);
  }
  
    
}
void rotate()
{
  switch (joy_l[0])
  {
  case 0:
    //if(motors_state == MotorsState::NONE) set_speed(0);
    break;
  case 1:
    motors_state = MotorsState::ROTATING;
    drive_dir = DriveDir::RIGHT;
    //digitalWrite(dir_11, HIGH);
    //digitalWrite(dir_21, HIGH);
    //set_speed(1);
    break;
  case 2:
    motors_state = MotorsState::ROTATING;
    drive_dir = DriveDir::LEFT;
    //digitalWrite(dir_11, LOW);
    //digitalWrite(dir_21, LOW);
    //set_speed(1);
    break;
  }
}
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  pinMode(dir_11, OUTPUT);
  pinMode(dir_12, OUTPUT);
  pinMode(dir_21, OUTPUT);
  pinMode(dir_22, OUTPUT);

  pinMode(pwm_1, OUTPUT);
  pinMode(pwm_2_1, OUTPUT);
  pinMode(pwm_2_2, OUTPUT);
  stepper_current = micros();
  stepper_prev = micros();

  s0.begin(static_cast<size_t>(ServoPins::FIRST));
  s1.begin(static_cast<size_t>(ServoPins::SECOND));
  s2.begin(static_cast<size_t>(ServoPins::THRID));
  servo_current = millis();
  servo_prev = millis();
  //s0.write(90); //gora dol
  //s1.write(90);//lewo prawo
  //s2.write(90);

  s0.set(90);
  s1.set(90);
  s2.set(servo_opened);
}

void loop()
{
  uint64_t current_time = millis();
  delta_time = current_time - prev_time;
  prev_time = current_time;

  if (read_data())
  {
    digest_data();
    print_data();

    if (!buttons[1]) //square
    {
      stepper_offset_start = 256 - constrain(potents[0], 0, 256);
      can_stepper = false;
    }
    else
    {
      stepper_offset_end = 256 - constrain(potents[0], 0, 255);
      can_stepper = true;
      stepper_offset_state = StepperOffset::NOT_READ;
    }
    if (!buttons[0])
    {
      can_servo = true;
    }
    else
    {
      can_servo = false;
    }
  }
  acctuators();
  servo();
  //10pwm 25dir
  //23pwm 27dir
  drive(joy_l[0]);
  rotate();
  move_wheels();
  //Serial.println("can: " + String(can_stepper));
  if (can_stepper)
    move_stepper();
  s0.update();
  s1.update();
  s2.update();
  //Serial.println(String(s0.get()) + " " + String(s1.get()) + " " + String(s2.get()));
}
