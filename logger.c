#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

//#include "logger.h"

// ==============================================================
// ======================== SETTING =============================
// ==============================================================

#define F_CFU 16000000
#define BAUD  115200
#define BAUD_PRESCALE ((F_CPU / (BAUD * 8UL)) - 1)

#define clockCyclesPerMicrosecond() (F_CPU / 1000000L)
#define clockCyclesToMicroseconds(a) ((a) / clockCyclesPerMicrosecond())
#define microsecondsToClockCycles(a) ((a) * clockCyclesPerMicrosecond())

#define bit_value(bit) (1 << (bit))

#define HIGH 0x1
#define LOW  0x0

// ==============================================================
// ========================= STRUCT =============================
// ==============================================================

typedef struct DistSensor {
	char* desc;
	float data;
	uint8_t trig_bit;
	uint8_t echo_bit;
} DistSensor;

typedef struct WaterSensor {
	char* desc;
	float data;
	uint8_t analog_bit;
} WaterSensor;

typedef struct TempSensor {
	char* desc;
	float temp_data;
	float hum_data;
	uint8_t digital_bit;
} TempSensor;

typedef struct LogStruct {
	DistSensor* ds;
	WaterSensor* ws;
	TempSensor* ts;
	int write_count;
	int read_count;
} LogStruct;

void struct_init(DistSensor* dist_sensor, WaterSensor* water_sensor, TempSensor* temp_sensor, LogStruct* log_struct) {
	dist_sensor->desc  = (char*) malloc(32);
	water_sensor->desc = (char*) malloc(32);
	temp_sensor->desc  = (char*) malloc(32);
	
	dist_sensor->desc = "Distanza";
	water_sensor->desc = "Profondità";
	temp_sensor->desc = "Temperatura e Umidità";

	dist_sensor->data = 0.0;
	water_sensor->data = 0.0;
	temp_sensor->temp_data = 0.0;
	temp_sensor->hum_data = 0.0;
	
	dist_sensor->echo_bit = PORTB4;
	dist_sensor->trig_bit = PORTB3;
	water_sensor->analog_bit = 0;
	temp_sensor->digital_bit = PORTD6;
	
	log_struct->ds = dist_sensor;
	log_struct->ws = water_sensor;
	log_struct->ts = temp_sensor;
	log_struct->read_count = 0;
	log_struct->write_count = 0;
}

DistSensor*  dist_sensor;
WaterSensor* water_sensor;
TempSensor*  temp_sensor;
LogStruct*   log_struct;

// ==============================================================
// ========================== UART ==============================
// ==============================================================

void UART_init() {
	UBRR0H = (BAUD_PRESCALE >> 8);
	UBRR0L = BAUD_PRESCALE;
	
	UCSR0A |= bit_value(U2X0);
	//UCSR0B |= bit_value(RXCIE0);
	UCSR0B |= bit_value(RXEN0)  | bit_value(TXEN0);
	UCSR0C |= bit_value(UCSZ00) | bit_value(UCSZ01) | bit_value(USBS0);
}

static int UART_receive(FILE *f) {
	loop_until_bit_is_set(UCSR0A, RXC0);
	while(!(UCSR0A & bit_value(UDRE0)));
	return UDR0;
}

static int UART_send(char c, FILE *f) {
	if(c == '\n') UART_send('\r', f);
	while(!(UCSR0A & bit_value(UDRE0)));
	UDR0 = c;
	return 0;
}

static FILE OUTFP = FDEV_SETUP_STREAM(UART_send, NULL, _FDEV_SETUP_WRITE);
static FILE INFP  = FDEV_SETUP_STREAM(NULL, UART_receive, _FDEV_SETUP_READ);

// ==============================================================
// ========================= EEPROM =============================
// ==============================================================

void EEPROM_read(char* str, int offset, int size){
	uint8_t* start = (uint8_t*) offset;
	uint8_t* end = (uint8_t*) (offset + size);
	while(start < end) {
		eeprom_busy_wait();
		*str = eeprom_read_byte(start);
		start++;
		str++;
	}
}

void EEPROM_write(char* str, int offset, int size) {
	uint8_t* start = (uint8_t*) offset;
	uint8_t* end = (uint8_t*) (offset + size);
	while(start < end) {
		eeprom_busy_wait();
		eeprom_write_byte(start, *str);
		start++;
		str++;
	}
}

void EEPROM_read_int(int* data, int offset) {
	eeprom_busy_wait();
	*data = eeprom_read_byte((uint8_t*) offset);
}

void EEPROM_write_int(int data, int offset) {
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*) offset, data);
}

void EEPROM_erase(int n) {
	for(int i = 0; i < n; i++)
		eeprom_write_byte((uint8_t*) i, 0);
}

int EEPROM_init() {
	int data;
	cli();
	
	if(!eeprom_is_ready()) {
		printf("Waiting for EEPROM to become ready...\n");
		eeprom_busy_wait();
	}
	printf("EEPROM ready\n");
	
	printf("Checking EEPROM...\n");
	EEPROM_read_int(&data, 0);
	int eeprom_empty = !data;
	
	if(eeprom_empty)
		printf("EEPROM empty\n");
	return eeprom_empty;
}

// ==============================================================
// ======================= DIST SENSOR ==========================
// ==============================================================

void dist_sensor_init() {
	DDRB |= bit_value(dist_sensor->trig_bit);
	DDRB &= ~bit_value(dist_sensor->echo_bit);
}

void begin_trig() {
	PORTB &= ~bit_value(dist_sensor->trig_bit);
	_delay_us(5);
	PORTB |= bit_value(dist_sensor->trig_bit);
	_delay_us(10);
	PORTB &= ~bit_value(dist_sensor->trig_bit);
}

unsigned long pulseIn(uint8_t state, unsigned long timeout) {
	uint8_t bit = bit_value(dist_sensor->echo_bit);
	uint8_t stateMask = (state ? bit : 0);
	
	unsigned long width = 0;
	unsigned long numloops = 0;
	
	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes 16 clock cycles per iteration
	unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;

	// wait for any previous pulse to end
	while ((PINB & bit) == stateMask)
		if (numloops++ == maxloops)
			return 0;
	
	// wait for the pulse to start
	while ((PINB & bit) != stateMask)
		if (numloops++ == maxloops)
			return 0;

	// wait for the pulse to stop
	while ((PINB & bit) == stateMask) {
		if (numloops++ == maxloops)
			return 0;
		width++;
	}

	// convert the reading to microseconds. The loop has been determined
	// to be 20 clock cycles long and have about 16 clocks between the edge
	// and the start of the loop
	return clockCyclesToMicroseconds(width * 16 + 16);
}

void dist_printf(float f_val) {
	int i_val = (int) (f_val * 10.0);
	printf("Distanza: %d.%d cm\n", i_val / 10, i_val % 10);
}

void dist_sensor_active(char* str) {
	char res[16];
	float max = 0.0;
	
	begin_trig();
	
	unsigned long duration = pulseIn(HIGH, 10000);
	float cm = (duration/2) / 29.1;
	
	max = (cm > max) ? cm : max;
	
	dist_printf(cm);
	_delay_ms(500);
	
	dtostrf(max, 4, 2, res);
	//sprintf(res, "%u", (uint32_t) max);
	strcpy(str, "Distanza: ");
	strcat(str, res);
	strcat(str, " cm");
}

// ==============================================================
// ======================= ANALOG SENSOR ========================
// ==============================================================

void analog_init() {
	ADMUX  = 0x00;
	ADCSRA = 0x00;
	ADCSRB = 0x00;
	
	ADMUX  |= bit_value(REFS0) | bit_value(MUX3)  | bit_value(MUX2)  | bit_value(MUX1)  | bit_value(MUX0);
	ADCSRA |= bit_value(ADEN)  | bit_value(ADPS2) | bit_value(ADPS1) | bit_value(ADPS0) | bit_value(ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);
}

void analog_sensor_init() {
	analog_init();
	DDRC  &= ~bit_value(water_sensor->analog_bit);
	PORTC &= ~bit_value(water_sensor->analog_bit);
	DIDR0 |= bit_value(water_sensor->analog_bit);
}

uint16_t analog_read() {
	ADMUX = (ADMUX & 0xf0) | (water_sensor->analog_bit & 0x0f);

	ADCSRA |= bit_value(ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);

	uint8_t lb = ADCL;
	uint8_t hb = ADCH;

	return (((uint16_t) hb) << 8) | lb;
}

void analog_sensor_active(char* str) {
	uint16_t val = analog_read();
	if(0 <= val && val < 130) {
		strcpy(str, "Profondità: 0.0-0.5 cm");
		printf("%s\n", str);
	}
	if(130 <= val && val < 260)	{
		strcpy(str, "Profondità: 0.5-1.0 cm");
		printf("%s\n", str);
	}
	if(260 <= val && val < 320)	{
		strcpy(str, "Profondità: 1.0-2.0 cm");
		printf("%s\n", str);
	}
	if(320 <= val && val < 390)	{
		strcpy(str, "Profondità: 2.0-3.0 cm");
		printf("%s\n", str);
	}
	if(390 <= val && val < 460)	{
		strcpy(str, "Profondità: 3.0-4.0 cm");
		printf("%s\n", str);
	}
	if(val >= 460)	{
		strcpy(str, "Profondità: >4.0 cm");
		printf("%s\n", str);
	}
	_delay_ms(500);
}

// ==============================================================
// ======================= TEMP SENSOR ==========================
// ==============================================================

void temp_request() {
	DDRD |= bit_value(temp_sensor->digital_bit);
	
	PORTD &= ~bit_value(temp_sensor->digital_bit);
	_delay_ms(20);
	
	PORTD |= bit_value(temp_sensor->digital_bit);
}

void temp_response() {
	uint8_t bit = bit_value(temp_sensor->digital_bit);
	DDRD &= ~bit;
	
	while (PIND & bit);
	while (!(PIND & bit));
	while (PIND & bit);
}

uint8_t temp_data() {
	uint8_t bit = bit_value(temp_sensor->digital_bit);
	uint8_t c = 0;
	
	for(int i = 0; i < 8; i++) {
		while (!(PIND & bit));
		_delay_us(30);
		
		if(PIND & bit)	c = (c << 1) | HIGH;
		else 			c = (c << 1);
		
		while (PIND & bit);
	}
	
	return c;
}

void temp_sensor_active(char* str) {
	char res1[16];
	char res2[16];
	
	float hum, temp, checksum;
	float max_t, max_h;
	
	temp_request();
	temp_response();
	
	hum = ((float) temp_data()) + ((float) (temp_data() / 100));
	temp = ((float) temp_data()) + ((float) (temp_data() / 100));
	checksum = (float) temp_data();
	
	printf("T: %u °C\n", (uint32_t) temp);
	printf("H: %u %%\n", (uint32_t) hum);
	
	max_t = (temp > max_t) ? temp : max_t;
	max_h = (hum  > max_h) ? hum  : max_h;
	
	_delay_ms(500);
	
	dtostrf(max_t, 4, 2, res1);
	dtostrf(max_h, 4, 2, res2);
	//sprintf(res1, "%u", (uint32_t) temp);
	//sprintf(res2, "%u", (uint32_t) hum);
	strcpy(str, "Temp: ");
	strcat(str, res1);
	strcat(str, " °C; Um: ");
	strcat(str, res2);
	strcat(str, " %");
}

// ==============================================================
// ========================== MAIN ==============================
// ==============================================================

void sensor_init() {
	dist_sensor_init();
	analog_sensor_init();
}

int main() {
	dist_sensor  = (DistSensor*)  malloc(sizeof(DistSensor));
	water_sensor = (WaterSensor*) malloc(sizeof(WaterSensor));
	temp_sensor  = (TempSensor*)  malloc(sizeof(TempSensor));
	log_struct   = (LogStruct*)   malloc(sizeof(LogStruct));
	
	char res1[32], res2[32], res3[32];
	int len1, len2, len3;
	
	UART_init();
	stdout = &OUTFP;
	stdin  = &INFP;
	
	int eeprom_empty = EEPROM_init();
	struct_init(dist_sensor, water_sensor, temp_sensor, log_struct);
	sensor_init();
	
	while(1) {
		printf("Mode [w, r, e]: ");
		char mode = UART_receive(&INFP);
		printf("%c\n", mode);
		
		switch(mode) {
			case 'W':
			case 'w':
				printf("Store results in EEPROM\n");
				
				dist_sensor_active(res1);
				analog_sensor_active(res2);
				temp_sensor_active(res3);

				len1 = strlen(res1) + 1;
				len2 = strlen(res2) + 1;
				len3 = strlen(res3) + 1;
								
				EEPROM_write_int(len1, 0);
				EEPROM_write(res1, 1, len1);
				
				EEPROM_write_int(len2, len1+1);
				EEPROM_write(res2, len1+2, len2);
				
				EEPROM_write_int(len3, len1+len2+2);
				EEPROM_write(res3, len1+len2+3, len3);
				
				eeprom_empty = 0;
				printf("EEPROM written\n");
				break;

			case 'R':
			case 'r':
				if(eeprom_empty) {
					printf("EEPROM empty!\n");
					break;
				}
				
				EEPROM_read_int(&len1, 0);
				EEPROM_read(res1, 1, len1);
				
				EEPROM_read_int(&len2, len1+1);
				EEPROM_read(res2, len1+2, len2);
				
				EEPROM_read_int(&len3, len1+len2+2);
				EEPROM_read(res3, len1+len2+3, len3);
								
				printf("Contents of string in EEPROM:\n");
				printf("\t %s\n", res1);
				printf("\t %s\n", res2);
				printf("\t %s\n", res3);
				break;
			
			case 'E':
			case 'e':
				EEPROM_erase(1000);
				eeprom_empty = 1;
				
				printf("EEPROM erased\n");
				break;
			
			default:
				printf("Invalid operation!\n");
				break;
		}
	}
}
