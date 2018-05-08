#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "logger.h"

// ==============================================================
// ========================= STRUCT =============================
// ==============================================================

DistSensor*  dist_sensor;
WaterSensor* water_sensor;
TempSensor*  temp_sensor;
LogStruct*   log_struct;
Buffer* 	 buffer;

void struct_init(DistSensor* dist_sensor, WaterSensor* water_sensor, TempSensor* temp_sensor, LogStruct* log_struct, Buffer* buffer) {
	dist_sensor->desc  = (char*) malloc(16);
	water_sensor->desc = (char*) malloc(16);
	temp_sensor->temp_desc = (char*) malloc(16);
	temp_sensor->hum_desc  = (char*) malloc(16);
	
	dist_sensor->desc = "Distanza";
	water_sensor->desc = "Profondità";
	temp_sensor->temp_desc = "Temperatura";
	temp_sensor->hum_desc = "Umidità";
	
	dist_sensor->data = 0.0;
	water_sensor->from = 0.0;
	water_sensor->to = 0.0;
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
	log_struct->type = GetAll;
	
	buffer->data = malloc(4);
	buffer->size = 4;
	buffer->next = 0;
}

void reset_buffer(Buffer* b) {
	memset(((char*) buffer->data), 0, strlen(buffer->data)+1);
	buffer->next = 0;
}

void reserve_space(Buffer* buffer, int bytes) {
    if((buffer->next + bytes) > buffer->size) {
        buffer->data = realloc(buffer->data, buffer->size * 2);
        buffer->size *= 2;
    }
}

void serialize_string(char* str, Buffer* b) {
	int len = strlen(str);
	reserve_space(b, len+2);
	memcpy(((char*) b->data) + b->next, str, len);
	memcpy(((char*) b->data) + b->next + len, ": ", 2);
	b->next += len+2;
}

void serialize_float(float x, FloatType float_type, Buffer* b) {
	char str[8];
	dtostrf(x, 4, 2, str);
	
	int len = strlen(str);
	reserve_space(b, len+5);
	memcpy(((char*) b->data) + b->next, str, len);
    
	switch(float_type) {
		case DistFloat:
		case WaterTo:
			memcpy(((char*) b->data) + b->next + len, " cm\n", 4);
			b->next += len+4;
			break;
			
		case WaterFrom:
			memcpy(((char*) b->data) + b->next + len, "-", 1);
			b->next += len+1;
			break;
		
		case TempFloat:
			memcpy(((char*) b->data) + b->next + len, " °C\n", 5);
			b->next += len+5;
			break;
			
		case HumFloat:
			memcpy(((char*) b->data) + b->next + len, " %\n", 3);
			b->next += len+3;
			break;
		
		default:
			memcpy(((char*) b->data) + b->next + len, "\n", 1);
			b->next += len+1;
			break;
	}
}

void packet_serialize(LogStruct* ls, Buffer* b) {
	switch(ls->type) {
		case GetDist:
			serialize_string(ls->ds->desc, b);
			serialize_float(ls->ds->data, DistFloat, b);
			break;
		
		case GetWater:
			serialize_string(ls->ws->desc, b);
			serialize_float(ls->ws->from, WaterFrom, b);
			serialize_float(ls->ws->to, WaterTo, b);
			break;
		
		case GetTemp:
			serialize_string(ls->ts->temp_desc, b);
			serialize_float(ls->ts->temp_data, TempFloat, b);
			serialize_string(ls->ts->hum_desc, b);
			serialize_float(ls->ts->hum_data, HumFloat, b);
			break;
		
		case GetAll:
			ls->type = GetDist;
			packet_serialize(ls, b);
			ls->type = GetWater;
			packet_serialize(ls, b);
			ls->type = GetTemp;
			packet_serialize(ls, b);
			break;
		
		default:
			printf("Operazione non valida!\n");
			break;			
	}
}

// ==============================================================
// ========================== UART ==============================
// ==============================================================

void UART_init() {
	UBRR0H = (UBRR0_SET >> 8);
	UBRR0L = UBRR0_SET;
	
	UCSR0A |= bit_value(U2X0);
	UCSR0B |= bit_value(RXCIE0);
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

void EEPROM_read(char* str, int offset, int size) {
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

void EEPROM_init() {
	int data;
	cli();
	
	if(!eeprom_is_ready()) {
		printf("EEPROM non ancora pronta...\n");
		eeprom_busy_wait();
	}
	printf("EEPROM pronta\n");
	
	printf("Controllo del contenuto della EEPROM...\n");
	EEPROM_read_int(&data, 0);
	int eeprom_empty = !data;
	
	if(eeprom_empty)
		printf("EEPROM vuota\n");
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

int pulseIn(uint8_t state, int timeout) {
	uint8_t bit = bit_value(dist_sensor->echo_bit);
	uint8_t cmp = (state ? bit : 0);
	
	int res = 0;
	int cnt = 0;

	while ((PINB & bit) == cmp)
		if (cnt++ == timeout)
			return 0;
	
	while ((PINB & bit) != cmp)
		if (cnt++ == timeout)
			return 0;

	while ((PINB & bit) == cmp) {
		if (cnt++ == timeout)
			return 0;
		res++;
	}

	return res + 1;
}

void dist_sensor_active() {
	char res[16];

	begin_trig();
	
	int duration = pulseIn(HIGH, 10000);
	float cm = (duration/2) / 29.1;
	
	dist_sensor->data = cm;
	
	_delay_ms(500);
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

void analog_sensor_active() {
	uint16_t val = analog_read();
	if(0 <= val && val < 130) {
		water_sensor->from = 0.0;
		water_sensor->to = 0.5;
	}
	if(130 <= val && val < 260)	{
		water_sensor->from = 0.5;
		water_sensor->to = 1.0;
	}
	if(260 <= val && val < 320)	{
		water_sensor->from = 1.0;
		water_sensor->to = 2.0;
	}
	if(320 <= val && val < 390)	{
		water_sensor->from = 2.0;
		water_sensor->to = 3.0;
	}
	if(390 <= val && val < 460)	{
		water_sensor->from = 3.0;
		water_sensor->to = 4.0;
	}
	if(val >= 460) {
		water_sensor->from = 4.0;
		water_sensor->to = 5.0;
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

void temp_sensor_active() {
	char res1[16];
	char res2[16];
	
	float hum = 52.71, temp = 25.64, checksum;

	temp_request();
	temp_response();

	hum = ((float) temp_data()) + ((float) (temp_data() / 100));
	temp = ((float) temp_data()) + ((float) (temp_data() / 100));
	checksum = (float) temp_data();
	
	temp_sensor->temp_data = temp;
	temp_sensor->hum_data = hum;
	
	_delay_ms(500);
}

// ==============================================================
// ========================== MAIN ==============================
// ==============================================================

void sensor_init() {
	dist_sensor_init();
	analog_sensor_init();
}

ISR(USART_RX_vect) {
	printf("Comando [p, w, r, e]: ");
	char mode = UART_receive(&INFP);
	printf("%c\n", mode);
	
	cli();
	reset_buffer(buffer);
	
	int data;
	EEPROM_read_int(&data, 0);
	int eeprom_empty = !data;
	
	char str[256];
	int len;
			
	switch(mode) {
		case 'P':
		case 'p':
			log_struct->type = GetAll;
			packet_serialize(log_struct, buffer);
			
			printf("%s", (char*) buffer->data);
			break;
		
		case 'W':
		case 'w':
			printf("Scrittura dei risultati nella EEPROM\n");
			
			log_struct->write_count++;
			printf("write_count: %d\n", log_struct->write_count);
			
			log_struct->type = GetAll;
			packet_serialize(log_struct, buffer);
			
			len = strlen(buffer->data)+1;
			EEPROM_write_int(len, 0);
			EEPROM_write(buffer->data, 1, len);
			
			eeprom_empty = 0;
			printf("EEPROM aggiornata\n");
			break;

		case 'R':
		case 'r':
			if(eeprom_empty) {
				printf("EEPROM vuota!\n");
				break;
			}
			
			log_struct->read_count++;
			printf("read_count: %d\n", log_struct->read_count);
			
			EEPROM_read_int(&len, 0);
			EEPROM_read(str, 1, len);
			
			printf("Contenuto della EEPROM:\n");
			printf("%s", str);
			break;
		
		case 'E':
		case 'e':
			EEPROM_erase(1000);
			eeprom_empty = 1;
			
			log_struct->read_count = 0;
			log_struct->write_count = 0;
			
			printf("EEPROM svuotata\n");
			break;
		
		default:
			printf("Operazione non valida!\n");
			break;
	}
	
	printf("----------------------------\n");
	sei();
}

int main() {
	dist_sensor = (DistSensor*) malloc(sizeof(DistSensor));
	water_sensor = (WaterSensor*) malloc(sizeof(WaterSensor));
	temp_sensor = (TempSensor*) malloc(sizeof(TempSensor));
	log_struct = (LogStruct*) malloc(sizeof(LogStruct));
	buffer = (Buffer*) malloc(sizeof(Buffer));
	
	UART_init();
	stdout = &OUTFP;
	stdin  = &INFP;
	
	EEPROM_init();
	sensor_init();
	
	struct_init(dist_sensor, water_sensor, temp_sensor, log_struct, buffer);
	
	sei();
	
	while(1) {
		dist_sensor_active();
		analog_sensor_active();
		temp_sensor_active();
		_delay_ms(1000);
	}
}
