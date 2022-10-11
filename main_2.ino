#include <SoftwareSerial.h>
#include "enums.hpp"

#define IS_DEBUG true

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

char load[8] = {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000};
int potents[2] = {127, 127};           // 0 - L, 1 - R
int joy_l[3] = {0, 0, 0};          // 0 - h, 1 - v, 2 - b3;
int joy_r[3] = {0, 0, 0};          // 0 - h, 1 - v, 2 - b3;
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

uint8_t rchash;
uint8_t rclen;
uint8_t rcid;
bool data_mode = false;
int min_speed = 1000;
int start_speed = min_speed;
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
  //return _read_data();
}
bool _read_data()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  got_data = false;
  while (Serial1.available() > 0 && got_data == false)
  {
    rc = Serial1.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        load[ndx] = rc;
        ndx++;
        if (ndx >= 8)
        {
          ndx = 8 - 1;
        }
      }
      else
      {
        recvInProgress = false;
        ndx = 0;
        got_data = true;
        return false;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
  return got_data;
  ;
}

void print_data()
{
  DEBUG_PRINT("Load");
  for (int i = 0; i < 8; i++)
    DEBUG_PRINT(load[i]);
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
  // potent
  for (int i = 0; i < 2; i++)
    potents[i] = 0;
  for (int i = 4; i >= 0; i--)
  {
    potents[0] += (load[0] & (1 << i)) << 5;
    potents[0] += (load[1] & (1 << i));
    potents[1] += (load[2] & (1 << i)) << 5;
    potents[1] += (load[3] & (1 << i));
  }

  // joysticks
  for (int i = 0; i < 3; i++)
  {
    joy_l[i] = 0;
    joy_r[i] = 0;
  }
  for (int i = 1; i >= 0; i--)
  {
    joy_r[0] += (load[4] & (1 << (i + 3))) >> 3;
    joy_r[1] += (load[4] & (1 << (i + 1))) >> 1;
    joy_l[0] += (load[5] & (1 << (i + 3))) >> 3;
    joy_l[1] += (load[5] & (1 << (i + 1))) >> 1;
  }
  joy_r[2] += load[4] & 1;
  joy_l[2] += load[5] & 1;

  // Triggers
  // int tL = 0, tR = 0;
  triggers[0] = (load[6] & (1 << 4)) >> 4;
  triggers[1] = (load[7] & (1 << 4)) >> 4;

  // Buttons
  buttons[0] = (load[6] & (1 << 3)) >> 3;
  buttons[1] = (load[6] & (1 << 2)) >> 2;
  buttons[2] = (load[7] & (1 << 3)) >> 3;
  buttons[3] = (load[7] & (1 << 2)) >> 2;
}
void move_stepper()
{
  //    // 400 to obrót o 180 stopni
  const int maxRot = 400 * 43 / 10;
  int lastPotent = potents[0];
  int steps = int(map(constrain(abs(256 - potents[0]), 0, 256), 0, 256, -maxRot, maxRot));
  int delta = (steps  - _step) >= 0 ? 1 : -1;
  digitalWrite(dirPin, delta >= 0 ? LOW : HIGH);
  
  if (_step != steps)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(650);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(650);
    _step += delta;
    start_speed -= 30;
    start_speed = start_speed < 300 ? 300 : start_speed;
  } else {
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
    int vel = map(constrain(potents[1], 0, 256), 0, 256, 0, 256);
    analogWrite(pwm_1, vel);
  }
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
    //digitalWrite(dir_11, HIGH);
    //digitalWrite(dir_21, LOW);
    //set_speed(1);
  }
  if (!triggers[0])
  {
    motors_state = MotorsState::DRIVING_BACKWARD;
    drive_dir = DriveDir::BACKWARD;
    driving_dir = DrivingDir::BACKWARDS;
    //digitalWrite(dir_11, LOW);
    //digitalWrite(dir_21, HIGH);
    //set_speed(1);
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
  switch (motors_state)
  {
  case MotorsState::DRIVING:
    if(drive_dir == DriveDir::FORWARD) {
      digitalWrite(dir_11, HIGH);
      digitalWrite(dir_21, LOW);
      set_speed(1);
    } 
    break;
  case MotorsState::ROTATING: 
    if((drive_dir == DriveDir::RIGHT && driving_dir == DrivingDir::FORWARD) ||
      (drive_dir == DriveDir::LEFT && driving_dir == DrivingDir::BACKWARDS)) {
      digitalWrite(dir_11, LOW);
      digitalWrite(dir_21, LOW);
      set_speed(1);
    } else if((drive_dir == DriveDir::LEFT && driving_dir == DrivingDir::FORWARD) || 
      (drive_dir == DriveDir::RIGHT && driving_dir == DrivingDir::BACKWARDS)) {
      digitalWrite(dir_11, HIGH);
      digitalWrite(dir_21, HIGH);
      set_speed(1);
    }
    if(driving_dir != DrivingDir::BACKWARDS && driving_dir != DrivingDir::FORWARD) {
      set_speed(0);
    }
    break;
  case MotorsState::DRIVING_BACKWARD:
    if (drive_dir == DriveDir::BACKWARD) {
      digitalWrite(dir_11, LOW);
      digitalWrite(dir_21, HIGH);
      set_speed(1);
    }
    break;
  case MotorsState::NONE:
    set_speed(0);
    break;
  }
}

void move_acctuator(Acctuar acctuar, AcctuarDir dir)
{
  static AcctuarPin accLut[2] = {
      AcctuarPin::UPPER,
      AcctuarPin::LOWER};
  static AcctuarDirPin dirLut[2] = {
      AcctuarDirPin::UPPER,
      AcctuarDirPin::LOWER};
  static uint8_t valLut[3] = {
      0, //default
      LOW,
      HIGH, };
  if(dir != AcctuarDir::NONE) {
    if (acctuar != Acctuar::UPPER) {
      if(dir == AcctuarDir::BACKWARDS) dir = AcctuarDir::FORWARD;
      else if(dir == AcctuarDir::FORWARD) dir = AcctuarDir::BACKWARDS;
    }
  }
  digitalWrite(
      static_cast<uint8_t>(accLut[static_cast<size_t>(acctuar)]),
      dir != AcctuarDir::NONE ? HIGH : LOW);
  digitalWrite(
      static_cast<uint8_t>(dirLut[static_cast<size_t>(acctuar)]),
      valLut[static_cast<size_t>(dir)]);
}

void stop_acctuators()
{
  digitalWrite(static_cast<uint8_t>(AcctuarPin::UPPER), LOW);
  digitalWrite(static_cast<uint8_t>(AcctuarPin::LOWER), LOW);
}

void acctuators()
{
  //AcctuarController _which = static_cast<AcctuarController>(which);
  //AcctuarControllerDir dir = static_cast<AcctuarControllerDir>(joy_r[1]);

  AcctuarDir upper_dir = static_cast<AcctuarDir>(joy_r[1]);
  AcctuarDir lower_dir = static_cast<AcctuarDir>(joy_l[1]);

  move_acctuator(Acctuar::UPPER, upper_dir);
  move_acctuator(Acctuar::LOWER, lower_dir);
  
  // switch (dir)
  // {
  // case AcctuarControllerDir::NONE:
  //   stop_acctuators();
  //   break;
  // case AcctuarControllerDir::FORWARD:
  //   if (_which == AcctuarController::UPPER)
  //   {
  //     move_acctuator(Acctuar::UPPER, AcctuarDir::FORWARD);
  //   }
  //   else if (_which == AcctuarController::LOWER)
  //   {
  //     move_acctuator(Acctuar::LOWER, AcctuarDir::FORWARD);
  //   }
  //   else
  //   {
  //     move_acctuator(Acctuar::UPPER, AcctuarDir::FORWARD);
  //     move_acctuator(Acctuar::LOWER, AcctuarDir::FORWARD);
  //   }
  //   break;
  // case AcctuarControllerDir::BACKWARD:
  //   if (_which == AcctuarController::UPPER)
  //   {
  //     move_acctuator(Acctuar::UPPER, AcctuarDir::BACKWARDS);
  //   }
  //   else if (_which == AcctuarController::LOWER)
  //   {
  //     move_acctuator(Acctuar::LOWER, AcctuarDir::BACKWARDS);
  //   }
  //   else
  //   {
  //     move_acctuator(Acctuar::UPPER, AcctuarDir::BACKWARDS);
  //     move_acctuator(Acctuar::LOWER, AcctuarDir::BACKWARDS);
  //   }
  //   break;
  // }
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
}

void loop()
{

  if (read_data())
  {
    digest_data();
    print_data();

    if (buttons[1]) //square
    {
      which += 1;
      which %= 3;
    }
  }
  acctuators();
  //10pwm 25dir
  //23pwm 27dir
  drive(joy_l[0]);
  rotate();
  move_wheels();
  move_stepper();
}
