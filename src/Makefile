
FILENAME   = main
PORT       = usb
DEVICE     = atmega328p
PROGRAMMER = xplainedmini
BAUD       = 115200
COMPILE    = avr-gcc -Wall -Os -DF_CPU=16000000UL -mmcu=$(DEVICE)


default: compile upload clean

compile:
	$(COMPILE) -c $(FILENAME).cpp -o $(FILENAME).o
	$(COMPILE) -o $(FILENAME).elf $(FILENAME).o
	avr-objcopy -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex 
	avr-size --format=avr --mcu=$(DEVICE) $(FILENAME).elf

upload:
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(FILENAME).hex:i

clean:
	rm $(FILENAME).o
	rm $(FILENAME).elf
	rm $(FILENAME).hex
