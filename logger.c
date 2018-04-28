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

#define OFF_SIG ((uint32_t*) 0)
#define OFF_LEN ((uint8_t*)  6)
#define OFF_TXT ((uint8_t*)  7)

#define bit_value(bit) (1 << (bit))
#define SIG_LEN 6

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
	while (!(UCSR0A & bit_value(UDRE0))) {}
	UDR0 = c;
	return 0;
}

// ==============================================================
// ========================= EEPROM =============================
// ==============================================================

//# define cli()  __asm__ __volatile__ ("cli" ::: "memory")

void EEPROM_init(char* signature) {
	char buffer[SIG_LEN];
	//cli();
	
	if(!eeprom_is_ready()) {
		printf("Waiting for EEPROM to become ready...\n");
		eeprom_busy_wait();
	}
	printf("EEPROM ready\n");
	
	printf("Checking EEPROM format...\n");
	eeprom_read_block(&buffer, OFF_SIG, SIG_LEN);
	if (memcmp(buffer, signature, SIG_LEN) == 0)
		printf("EEPROM already formatted\n");
	else {
		printf("EEPROM is blank, formatting...\n");
		eeprom_write_block(signature, OFF_SIG, SIG_LEN);
		eeprom_write_byte(OFF_LEN, (int8_t) 0);
		eeprom_write_byte(OFF_TXT, (int8_t) 0);
		printf("EEPROM formatted\n");
	}
}

// ==============================================================
// ========================= SENSOR =============================
// ==============================================================

void dist_sensor_init() {
	DDRB |= bit_value(DDB3);
	DDRB &= ~bit_value(DDB4);
}

void sensor_init() {
	dist_sensor_init();
}

void begin_trig() {
	PORTB &= ~bit_value(PORTB3);
	_delay_ms(0.005);
	PORTB |= bit_value(PORTB3);
	_delay_ms(0.010);
	PORTB &= ~bit_value(PORTB3);
}

unsigned long pulseIn(uint8_t state, unsigned long timeout) {
	uint8_t bit = bit_value(4);
	uint8_t stateMask = (state ? bit : 0);
	
	unsigned long width = 0;
	unsigned long numloops = 0;

	// wait for any previous pulse to end
	while ((PINB & bit) == stateMask)
		if (numloops++ == timeout)
			return 0;
	
	// wait for the pulse to start
	while ((PINB & bit) != stateMask)
		if (numloops++ == timeout)
			return 0;

	// wait for the pulse to stop
	while ((PINB & bit) == stateMask) {
		if (numloops++ == timeout)
			return 0;
		width++;
	}

	return width + 1;
}

void myPrintf(float f_val) {
	int i_val = (int) (f_val * 10.0);
	printf("Distanza: %d.%d cm\n", i_val / 10, i_val % 10);
}

void dist_sensor_active() {
	for(int i = 0; i < 30; i++) {
		begin_trig();
		
		unsigned long duration = pulseIn(HIGH, 10000);
		float cm = (duration/2) / 29.1;
		
		myPrintf(cm);		
		_delay_ms(500);
	}
}

// ==============================================================
// ========================== MAIN ==============================
// ==============================================================

int main() {
	char* signature = "Matteo";
	char buffer[64];
	uint8_t str_length;
	
	USART_init();
	stdout = &OUTFP;
	stdin  = &INFP;
	
	EEPROM_init(signature);
	sensor_init();
	
	while(1) {
		//printf("[A]ctivate sensor, [R]ead memory, [W]rite memory, [E]rase memory: ");
		printf("Mode: ");
		char mode = USART_receive(&INFP);
		printf("%c\n", mode);
		
		switch(mode) {
			case 'W':
			case 'w':
				printf("Enter a string to store in EEPROM: ");
				scanf("%s", buffer);
				str_length = strlen(buffer);
				printf("EEPROM written\n");
				
				eeprom_write_byte(OFF_LEN, str_length);
				eeprom_write_block(&buffer, OFF_TXT, str_length + 1);
				break;
			
			case 'R':
			case 'r':
				str_length = eeprom_read_byte(OFF_LEN);
				eeprom_read_block(&buffer, OFF_TXT, str_length + 1);
				
				printf("Contents of string in EEPROM: %s\n", buffer);
				break;
				
			case 'E':
			case 'e':
				//eeprom_write_dword(OFF_SIG, 0x0000);
				eeprom_write_byte(OFF_LEN, (int8_t) 0);
				eeprom_write_byte(OFF_TXT, (int8_t) 0);
				
				printf("EEPROM erased\n");
				break;
				
			case 'A':
			case 'a':
				dist_sensor_active();
				break;
			
			default:
				printf("Invalid operation!\n");
				break;
		}
	}
}
