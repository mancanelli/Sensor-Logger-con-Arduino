CC = avr-gcc
AVRDUDE_FLAGS = -F -V -c $(protocol) -p $(part) -P $(port) -b $(baud)

mcu = -mmcu=atmega328p
port = /dev/ttyACM0
baud = 115200
freq = -DF_CPU=16000000UL
protocol = arduino
part = ATMEGA328P


$(prog).hex $(prog).bin $(prog).lst : $(prog)
	avr-objcopy -O ihex -R .eeprom $< $(prog).hex
	avr-objcopy -O binary -R .eeprom $< $(prog).bin
	avr-objdump -d $< >$(prog).lst

$(prog) : $(prog).o
	$(CC) $(mcu) $< -o $@

$(prog).o : $(prog).c
	$(CC) -Os $(freq) $(mcu) -c $<


.PHONY : clean upload download

clean:
	rm -f $(prog) $(addprefix $(prog), .o .hex .bin .lst)

upload:
	avrdude $(AVRDUDE_FLAGS) -U flash:w:$(prog).hex

download:
	avrdude $(AVRDUDE_FLAGS) -U flash:r:$(prog)_backup.hex:i
