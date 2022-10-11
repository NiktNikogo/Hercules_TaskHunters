#include <SoftwareSerial.h>
char load[8] = {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000};
char load_prev[8] = {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000};


// x, square, triangle, circle
// 7, 9, 6, 8
#define x_button 7
#define s_button 9
#define t_button 6
#define c_button 8
const int buttons[] = {x_button, s_button, t_button, c_button};

#define l_trigger 4
#define r_trigger 5
const int triggers[] = {l_trigger, r_trigger};

#define r_button A6
#define r_horizontal A2
#define r_vertical A5
const int right_joy[] = {r_vertical, r_horizontal, r_button};

#define l_button A7
#define l_horizontal A1
#define l_vertical A3
const int left_joy[] = {l_vertical, l_horizontal, l_button};

#define l_potent A4
#define r_potent A0
const int potentiometers[] = {l_potent, r_potent};

SoftwareSerial acct_serial(2, 3); // HC-12 TX Pin, HC-12 RX Pin

int buttons_data[4] = {0}, buttons_prev[4] = {0};
int triggers_data[2] = {0}, triggers_prev[2] = {0};
int left_data[3] = {0}, left_prev[3] = {0};
int right_data[3] = {0}, right_prev[3] = {0};
int potent_data[2] = {0}, potent_prev[2] = {0};

// helper functions
void init_array(int *arr, int len, int val)
{
  for (int i = 0; i < len; i++)
  {
    pinMode(arr[i], val);
  }
}
void print_array(int *arr, int len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.print(arr[i]);
    Serial.print("  ");
  }
}
void gather_joystick(int *to_store, int *joy)
{
  // vertical, horizontal, buttton
  const int minus_threshold = 140;
  const int plus_threshold = 30;
  for (int i = 0; i < 2; i++)
  {
    to_store[i] = analogRead(joy[i]); 
    if (to_store[i] > minus_threshold)
    {
      to_store[i] = -1;
    }
    else if (to_store[i] < plus_threshold)
    {
      to_store[i] = 1;
    }
    else
    {
      to_store[i] = 0;
    }
  }
  to_store[2] = analogRead(joy[2]);
  if (to_store[2] > 0)
    to_store[2] = 1;
}
void print_data()
{
  Serial.print("Buttons: ");
  print_array(buttons_data, 4);
  Serial.print("Triggers: ");
  print_array(triggers_data, 2);
  Serial.print("R-joy: ");
  print_array(right_data, 3);
  Serial.print("L-joy: ");
  print_array(left_data, 3);
  Serial.print("Potent: ");
  print_array(potent_data, 2);
  Serial.print("Load: ");
  for (int i = 0; i < 8; i++)
  {
    Serial.print(load[i]);
  }
  
}
void copy_to_arr(int *dst, int *src, int *len)
{
  for (int i = 0; i < len; i++)
  {
    dst[i] = src[i];
  }
}

void gather_button(int *to_store)
{
  for (int i = 0; i < 4; i++)
  {
    to_store[i] = digitalRead(buttons[i]);
  }
}
void gather_triggers(int *to_store)
{
  for (int i = 0; i < 2; i++)
  {
    to_store[i] = digitalRead(triggers[i]);
  }
}
void gather_joysticks(int *r_store, int *l_store)
{
  gather_joystick(r_store, right_joy);
  gather_joystick(l_store, left_joy);
}
void gather_potent(int *to_store)
{
  for (int i = 0; i < 2; i++)
  {
    to_store[i] = analogRead(potentiometers[i]) / 4;
  }
}
bool make_loads()
{
  char load_new[8] = {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000};
  bool changed = false;
  //potent
  for(int i = 0; i < 2; i++) {
    for (int j = 9; j >= 5; j--) load_new[0 + i * 2] |= ((potent_data[i] >> j) & 1) << (j - 5);
    for (int j = 4; j >= 0; j--) load_new[1 + i * 2] |= ((potent_data[i] >> j) & 1) << j;
    if(potent_data[i] != potent_prev[i]) changed = true;
  }
    
  //joy R 
  for(int i = 0; i < 2; i++) {
    if (abs(right_data[i]) == 1) {
      load_new[4] |= right_data[i] == -1 ? (1 << (2 + 2 * i)) : (1 << (1 + 2 * i));
    }
    if(right_data[i] != right_prev[i]) changed = true;
  }
  if(right_data[2] != right_prev[2] && !right_data[2]) {
    changed = true;
    load_new[4] |= 1 << 0;
  }
  //joy L
  for(int i = 0; i < 2; i++) {
    if (abs(left_data[i]) == 1) {
      load_new[5] |= left_data[i] == -1 ? (1 << (2 + 2 * i)) : (1 << (1 + 2 * i));
    }
    if(left_data[i] != left_prev[i]) changed = true;
  }
  if(left_data[2] != left_prev[2] && !left_data[2]) {
    changed = true;
    load_new[5] |= 1 << 0;
  }

  //trigger
  for(int i = 0; i < 2; i++) {
    //if(triggers_data[i] != triggers_prev[i] && !triggers_data[i]) {
    if(triggers_data[i]) {
      load_new[6 + i] |= 1 << 4;
    } else {
      load_new[6 + i] |= 0 << 4;
    }
    changed = true;
   
    //} else {
    //  bool bit = (load_prev[6 + i] & (1 << 4)) >> 4;
    //  load_new[6 + i] ^= (-(unsigned long)bit ^ load_new[6 + i]) & (1Ul << 4);
    //}
  }
  //buttons
  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 2; j++) {
      if(buttons_data[i * 2 + j] != buttons_prev[i * 2 + j] && !buttons_data[i * 2 + j]) {
        load_new[6 + i] |= 1 << (2 + j);
        changed = true;
      } else {
        int shift = 2 + j;
        bool bit = (load_prev[6 + i] & (1 << shift)) >> shift;
        load_prev[6 + i] ^= (-(unsigned long)bit & load_new[6 + 1]) & (1Ul << shift);
      }
    }
  }
  if (changed) {
    for(int i = 0; i < 8; i++) {
      load[i] = load_new[i];
    }
  }
  return changed;
}
void store_to_prev()
{
  copy_to_arr(buttons_prev, buttons_data, 4);
  copy_to_arr(triggers_prev, triggers_data, 2);
  copy_to_arr(left_prev, left_data, 3);
  copy_to_arr(right_prev, right_data, 3);
  copy_to_arr(potent_prev, potent_data, 2);
  for (int i = 0; i < 8; i++)
  {
    load[i] = load_prev[i];
  }
}
void print_changed_data()
{
  for(int i = 0; i < 8; i++) {
    Serial.print(load[i]);
  }
  Serial.println(" ");
}
void send_data(){
  send_data_safe();
  //_send_data();
}
void _send_data()
{
  //Serial.print("start - ");
  acct_serial.write('<');
  for(int i = 0; i < 8; i++) {
    acct_serial.write(load[i]);
    //Serial.print(load[i]);
  }
  acct_serial.write('>');
  //Serial.println(" - end");
}

uint8_t old_hash;
uint8_t old_id;

void send_data_safe(){  
  uint8_t* data = load;
  size_t len = 8;

  
  static uint8_t id = 0;
  static uint8_t old_data[256];

  bool changed = false;
  for(size_t i = 0; i < len; i++){
    changed |= (old_data[i] != data[i]);
    old_data[i] = data[i];
  }
  id += changed;

  uint8_t hash = 0;
  for(size_t i = 0; i < len; i++){
    hash ^= data[i];
  }

  if(old_hash == hash && old_id == id){
    Serial.println("IGNORE");
    return;
  }

  acct_serial.write(0x12);
  acct_serial.write(0x34);
  acct_serial.write(id);
  acct_serial.write(hash);
  acct_serial.write(len);
  acct_serial.write(data, len);
}

void read_data_safe()
{

    if (acct_serial.available() < 4)
    {
      return;
    }

    uint8_t rc0 = acct_serial.read();
    uint8_t rc1 = acct_serial.read();
    uint8_t rcid = acct_serial.read();
    uint8_t rchash = acct_serial.read();

    if (rc0 != 0x56 || rc1 != 0x78)
    {
      return;
    }

    old_id = rcid;
    old_hash = rchash;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  acct_serial.begin(9600);
  init_array(buttons, 4, INPUT_PULLUP);
  init_array(triggers, 2, INPUT_PULLUP);
  init_array(right_joy, 3, INPUT_PULLUP);
  init_array(left_joy, 3, INPUT_PULLUP);
  init_array(potentiometers, 2, INPUT_PULLUP);
}

void loop()
{
  gather_button(buttons_data);
  gather_triggers(triggers_data);
  gather_joysticks(right_data, left_data);
  gather_potent(potent_data);
  bool changed = make_loads();
  if(changed) {
    print_data();
  }
  send_data();
  
  store_to_prev();

  read_data_safe();
}
