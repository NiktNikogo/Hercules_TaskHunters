#include <SoftwareSerial.h>
#include "Button.hpp"
#define IS_DEBUG true

#if IS_DEBUG
#define DEBUG_PRINT(x) Serial.print(x);
#define DEBUG_PRINTLN(x) Serial.println(x);
#else
#define DEBUG_PRINT(x) ;
#define DEBUG_PRINTLN(x) ;
#endif

#define LOAD_SIZE 6

static char HEX_LUT[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

char load[LOAD_SIZE] = {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000};
char load_prev[LOAD_SIZE] = {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000};

// x, square, triangle, circle
// 7, 9, 6, 8
#define x_button 7
#define s_button 9
#define t_button 6
#define c_button 8
const int buttons_pins[] = {x_button, s_button, t_button, c_button};

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

#define led_pin 12
#define servo_pin 11
int led_state = LOW;
int servo_state = LOW;
int catch_state = true;
SoftwareSerial acct_serial(2, 3); // HC-12 TX Pin, HC-12 RX Pin

int buttons_data[4] = {0}, buttons_prev[4] = {0};
int triggers_data[2] = {0}, triggers_prev[2] = {0};
int left_data[3] = {0}, left_prev[3] = {0};
int right_data[3] = {0}, right_prev[3] = {0};
int potent_data[2] = {0}, potent_prev[2] = {0};
Digital_button buttons[] = {Digital_button(x_button), Digital_button(s_button), 
	Digital_button(t_button), Digital_button(c_button)};
Analog_button button_joysticks[] = {Analog_button(l_button), Analog_button(r_button)};
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
		DEBUG_PRINT(arr[i]);
		DEBUG_PRINT("  ");
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
}
void print_data()
{
	DEBUG_PRINT("Buttons: ");
	print_array(buttons_data, 4);
	DEBUG_PRINT("Triggers: ");
	print_array(triggers_data, 2);
	DEBUG_PRINT("R-joy: ");
	print_array(right_data, 3);
	DEBUG_PRINT("L-joy: ");
	print_array(left_data, 3);
	DEBUG_PRINT("Potent: ");
	print_array(potent_data, 2);
	DEBUG_PRINT("Load: ");
	for (int i = 0; i < LOAD_SIZE; i++)
	{
		DEBUG_PRINT(HEX_LUT[((load[i] >> 4) & 0xF)]);
		DEBUG_PRINT(HEX_LUT[(load[i] & 0xF)]);
	}
	DEBUG_PRINTLN("");
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
		to_store[i] = buttons[i].get_state();
	}
	led_state = to_store[1] ? HIGH : LOW;
	digitalWrite(led_pin, led_state);
	servo_state = to_store[0] ? HIGH : LOW;
	digitalWrite(servo_pin, !servo_state);

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
	r_store[2] = button_joysticks[1].get_state();
	gather_joystick(l_store, left_joy);
	l_store[2] = button_joysticks[0].get_state();

	//Serial.println(String([2]) + " " + String(r_store[2]));
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

	/*
  load_new[0] <- pierwsze 8 bitów potencjometra 0
  load_new[1] <- drugie 8 bitów potencjometra 0
  load_new[2] <- pierwsze 8 bitów potencjometra 0
  load_new[3] <- drugie 8 bitów potencjometra 0
  load_new[4] <- dwa pierwsze lewy X, dwa drugie lewy Y, dwa trzecie prawy X, dwa czwarte prawy Y
  load_new[5] <- dwa pierwsze L3, R3 dwa drugie, TL, TR, 4 ostatnie guziki
  */

	char load_new[LOAD_SIZE] = {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000};

	load_new[0] |= (potent_data[0] >> 8) & 0xFF;
	load_new[1] |= (potent_data[0]) & 0xFF;
	load_new[2] |= (potent_data[1] >> 8) & 0xFF;
	load_new[3] |= (potent_data[1]) & 0xFF;

	//joy R
	switch (right_data[0])
	{
	case -1:
		load_new[4] |= 0b10000000;
		break;
	case 1:
		load_new[4] |= 0b01000000;
		break;
	default:
		break;
	}
	switch (right_data[1])
	{
	case -1:
		load_new[4] |= 0b00100000;
		break;
	case 1:
		load_new[4] |= 0b00010000;
		break;
	default:
		break;
	}
	switch (left_data[0])
	{
	case -1:
		load_new[4] |= 0b00001000;
		break;
	case 1:
		load_new[4] |= 0b00000100;
		break;
	default:
		break;
	}
	switch (left_data[1])
	{
	case -1:
		load_new[4] |= 0b00000010;
		break;
	case 1:
		load_new[4] |= 0b00000001;
		break;
	default:
		break;
	}

	if (right_data[2])
		load_new[5] |= 0b10000000;
	if (left_data[2])
		load_new[5] |= 0b01000000;
	if (triggers_data[0])
		load_new[5] |= 0b00100000;
	if (triggers_data[1])
		load_new[5] |= 0b00010000;
	if (buttons_data[0])
		load_new[5] |= 0b00001000;
	if (buttons_data[1])
		load_new[5] |= 0b00000100;
	if (buttons_data[2])
		load_new[5] |= 0b00000010;
	if (buttons_data[3])
		load_new[5] |= 0b00000001;

	memcpy(load, load_new, LOAD_SIZE);
}
void store_to_prev()
{
	copy_to_arr(buttons_prev, buttons_data, 4);
	copy_to_arr(triggers_prev, triggers_data, 2);
	copy_to_arr(left_prev, left_data, 3);
	copy_to_arr(right_prev, right_data, 3);
	copy_to_arr(potent_prev, potent_data, 2);
	for (int i = 0; i < LOAD_SIZE; i++)
	{
		load[i] = load_prev[i];
	}
}
void print_changed_data()
{
	for (int i = 0; i < LOAD_SIZE; i++)
	{
		DEBUG_PRINT(load[i]);
	}
	DEBUG_PRINTLN(" ");
}
void send_data()
{
	send_data_safe();
	//_send_data();
}
void _send_data()
{
	//Serial.print("start - ");
	acct_serial.write('<');
	for (int i = 0; i < LOAD_SIZE; i++)
	{
		acct_serial.write(load[i]);
		//Serial.print(load[i]);
	}
	acct_serial.write('>');
	//Serial.println(" - end");
}

uint8_t old_hash;
uint8_t old_id;

void send_data_safe()
{
	uint8_t *data = load;
	size_t len = LOAD_SIZE;

	static uint8_t id = 0;
	static uint8_t old_data[256];

	bool changed = false;
	for (size_t i = 0; i < len; i++)
	{
		changed |= (old_data[i] != data[i]);
		old_data[i] = data[i];
	}
	id += changed;

	uint8_t hash = 0;
	for (size_t i = 0; i < len; i++)
	{
		hash ^= data[i];
	}

	if (old_hash == hash && old_id == id)
	{
		DEBUG_PRINT("IGNORE");
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
	//init_array(buttons_pins, 4, INPUT_PULLUP);

	init_array(triggers, 2, INPUT_PULLUP);
	init_array(right_joy, 3, INPUT_PULLUP);
	init_array(left_joy, 3, INPUT_PULLUP);
	init_array(potentiometers, 2, INPUT_PULLUP);
	pinMode(led_pin, led_state);
	pinMode(servo_pin, servo_state);
}

void loop()
{
	for (int i = 0; i < 4; i++)
	{
		buttons[i].update(millis());
	}
	for (int i = 0; i < 2; i++) {
		button_joysticks[i].update(millis());
	}
	gather_button(buttons_data);
	gather_triggers(triggers_data);
	gather_joysticks(right_data, left_data);
	gather_potent(potent_data);
	make_loads();
	print_data();
	send_data();
	store_to_prev();
	read_data_safe();
}
