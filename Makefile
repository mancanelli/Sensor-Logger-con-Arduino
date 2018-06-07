CC = avr-gcc
AVRDUDE_FLAGS = -F -V -c $(protocol) -p $(part) -P $(port) -b $(baud)

mcu = -mmcu=atmega328p
port = /dev/ttyACM0
baud = 115200
freq = -DF_CPU=16000000UL
protocol = arduino
part = ATMEGA328P


logger.hex logger.bin logger.lst : logger
	avr-objcopy -O ihex -R .eeprom $< logger.hex
	avr-objcopy -O binary -R .eeprom $< logger.bin
	avr-objdump -d $< >logger.lst

logger : logger.o
	$(CC) $(mcu) $< -o $@

logger.o : logger.c logger.h
	$(CC) -Os $(freq) $(mcu) -c $<


.PHONY : clean upload download

clean:
	rm -f logger $(addprefix logger, .o .hex .bin .lst)

upload:
	avrdude $(AVRDUDE_FLAGS) -U flash:w:logger.hex

download:
	avrdude $(AVRDUDE_FLAGS) -U flash:r:logger_backup.hex:i
