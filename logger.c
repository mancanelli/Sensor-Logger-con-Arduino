#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <avr/eeprom.h>
//#include <avr/interrupt.h>

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

static int USART_send(char c, FILE *f);
static int USART_receive(FILE *f);
static FILE OUTFP = FDEV_SETUP_STREAM(USART_send, NULL, _FDEV_SETUP_WRITE);
static FILE INFP  = FDEV_SETUP_STREAM(NULL, USART_receive, _FDEV_SETUP_READ);

// ==============================================================
// ========================= USART ==============================
// ==============================================================

void USART_init() {
	UBRR0H = (BAUD_PRESCALE >> 8);
	UBRR0L = BAUD_PRESCALE;
	
	UCSR0A |= bit_value(U2X0);
	UCSR0B |= bit_value(RXCIE0);
	UCSR0B |= bit_value(RXEN0)  | bit_value(TXEN0);
	UCSR0C |= bit_value(UCSZ00) | bit_value(UCSZ01) | bit_value(USBS0);
}

static int USART_receive(FILE *f) {
	loop_until_bit_is_set(UCSR0A, RXC0);
	while (!(UCSR0A & bit_value(UDRE0))) {}
	return UDR0;
}

static int USART_send(char c, FILE *f) {
	if(c == '\n') USART_send('\r', f);
	while (!(UCSR0A & bit_value(UDRE0))) {}
	UDR0 = c;
	return 0;
}

// ==============================================================
// ========================= EEPROM =============================
// ==============================================================

int EEPROM_init() {
	char buffer;
	//cli();
	
	if(!eeprom_is_ready()) {
		printf("Waiting for EEPROM to become ready...\n");
		eeprom_busy_wait();
	}
	printf("EEPROM ready\n");
	
	printf("Checking EEPROM...\n");
	int eeprom_empty = eeprom_read_byte(0) ? 0 : 1;
	if (eeprom_empty)
		printf("EEPROM empty\n");
	return eeprom_empty;
}

// ==============================================================
// ======================= DIST SENSOR ==========================
// ==============================================================

void dist_sensor_init() {
	DDRB |= bit_value(DDB3);
	DDRB &= ~bit_value(DDB4);
}

void begin_trig() {
	PORTB &= ~bit_value(PORTB3);
	_delay_us(5);
	PORTB |= bit_value(PORTB3);
	_delay_us(10);
	PORTB &= ~bit_value(PORTB3);
}

unsigned long pulseIn(uint8_t state, unsigned long timeout) {
	uint8_t bit = bit_value(PORTB4);
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
	char res[8];
	float max = 0.0;
	
	for(int i = 0; i < 30; i++) {
		begin_trig();
		
		unsigned long duration = pulseIn(HIGH, 10000);
		float cm = (duration/2) / 29.1;
		
		max = (cm > max) ? cm : max;
		
		dist_printf(cm);
		_delay_ms(500);
	}
	
	sprintf(res, "%f", max);
	strcpy(str, "Distanza: ");
	strcat(str, res);
	strcat(str, " cm");
}

// ==============================================================
// ======================= ANALOG SENSOR ========================
// ==============================================================

void analog_init() {
	ADMUX = 0x00;
	ADCSRA = 0x00;
	ADCSRB = 0x00;
	
	ADMUX  |= bit_value(REFS0) | bit_value(MUX3)  | bit_value(MUX2)  | bit_value(MUX1)  | bit_value(MUX0);
	ADCSRA |= bit_value(ADEN)  | bit_value(ADPS2) | bit_value(ADPS1) | bit_value(ADPS0) | bit_value(ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);
}

void analog_sensor_init() {
	DDRC  &= ~bit_value(0);
	PORTC &= ~bit_value(0);
	DIDR0 |= bit_value(0);
}

uint16_t analog_read() {
	ADMUX = (ADMUX & 0xf0) | (0 & 0x0f);

	ADCSRA |= bit_value(ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);

	uint8_t lb = ADCL;
	uint8_t hb = ADCH;

	return (((uint16_t) hb) << 8) | lb;
}

void analog_printf(uint16_t val, char* str) {
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
}

void analog_sensor_active(char* str) {
	for(int i = 0; i < 30; i++) {
		uint16_t raw = analog_read();
		analog_printf(raw, str);
		_delay_ms(500);
	}
}

// ==============================================================
// ======================= TEMP SENSOR ==========================
// ==============================================================

void temp_request(uint8_t bit) {
	DDRD |= bit;
	
	PORTD &= ~bit;
	_delay_ms(20);
	
	PORTD |= bit;
}

void temp_response(uint8_t bit) {
	DDRD &= ~bit;
	while (PIND & bit);
	while (!(PIND & bit));
	while (PIND & bit);
}

uint8_t temp_data(uint8_t bit) {
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
	char res1[8];
	char res2[8];
	
	float hum, temp, checksum;
	float max_t, max_h;
	
	uint8_t bit = bit_value(6); 
	
	for(int i = 0; i < 30; i++) {
		temp_request(bit);
		temp_response(bit);
		
		hum = (float) temp_data(bit) + ((float) temp_data(bit)) / 100;
		temp = (float) temp_data(bit) + ((float) temp_data(bit)) / 100;
		checksum = (float) temp_data(bit);
		
		printf("H: %f %%\n", hum);
		printf("T: %f °C\n", temp);
		
		max_t = (temp > max_t) ? temp : max_t;
		max_h = (hum  > max_h) ? hum  : max_h;
		
		_delay_ms(500);
	}
	
	sprintf(res1, "%f", temp);
	sprintf(res2, "%f", hum);
	strcpy(str, "Temperatura: ");
	strcat(str, res1);
	strcat(str, " °C\nUmidità: ");
	strcat(str, res2);
	strcat(str, " %%");
}

// ==============================================================
// ========================== MAIN ==============================
// ==============================================================

void sensor_init() {
	dist_sensor_init();
	analog_init();
	analog_sensor_init();
}

int main() {
	char res1[32], res2[32], res3[32];
	int len1, len2, len3;
	
	USART_init();
	stdout = &OUTFP;
	stdin  = &INFP;
	
	int eeprom_empty = EEPROM_init();
	sensor_init();
	
	while(1) {
		printf("Mode: ");
		char mode = USART_receive(&INFP);
		printf("%c\n", mode);
		
		switch(mode) {
			case 'R':
			case 'r':
				if(eeprom_empty) {
					printf("EEPROM empty!\n");
					break;
				}
				
				len1 = eeprom_read_byte((uint8_t*) 0);
				eeprom_read_block(&res1, (uint8_t*) 1, len1);
				
				len2 = eeprom_read_byte((uint8_t*) len1+1);
				eeprom_read_block(&res2, (uint8_t*) len1+2, len1+len2+1);
				
				len3 = eeprom_read_byte((uint8_t*) len1+len2+2);
				eeprom_read_block(&res3, (uint8_t*) len1+len2+3, len1+len2+len3+2);
				
				printf("Contents of string in EEPROM:\n");
				printf("\t %s\n", res1);
				printf("\t %s\n", res2);
				printf("\t %s\n", res3);
				break;
			/*
			case 'E':
			case 'e':
				eeprom_write_dword(0,  0x0000);
				printf("EEPROM erased\n");
				break;
			*/
			case 'A':
			case 'a':
				dist_sensor_active(res1);
				analog_sensor_active(res2);
				temp_sensor_active(res3);
				
				len1 = strlen(res1) + 1;
				len2 = strlen(res2) + 1;
				len3 = strlen(res3) + 1;
				
				printf("Store results in EEPROM\n");
				
				eeprom_write_byte((uint8_t*) 0, len1);
				eeprom_write_block(&res1, (uint8_t*) 1, len1);
				
				eeprom_write_byte((uint8_t*) len1+1, len2);
				eeprom_write_block(&res2, (uint8_t*) len1+2, len2);
				
				eeprom_write_byte((uint8_t*) len1+len2+2, len3);
				eeprom_write_block(&res3, (uint8_t*) len1+len2+3, len3);		
				
				printf("EEPROM written\n");
				eeprom_empty = 0;
				break;
			
			default:
				printf("Invalid operation!\n");
				break;
		}
	}
}
