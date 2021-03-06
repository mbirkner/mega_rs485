DEVICE_CC = atmega328p
DEVICE_DUDE = m328p

PROGRAMMER_DUDE = -Pusb -c avrisp2
# PROGRAMMER_DUDE = -P/dev/ttyUSB0 -c stk500v1 -b 57600

# orig: E:FF, H:DF, L:64
# CLKDIV8 not set, --> 8 MHz
# external resonator quarz > 10 MHz --> 16 MHz
FUSE_L=0xCE
FUSE_H=0xd9
FUSE_E=0xff

AVRDUDE=avrdude
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
CC=avr-gcc
LD=avr-gcc
SIZE=avr-size

LDFLAGS=-Wall -mmcu=$(DEVICE_CC)
CPPFLAGS=-mmcu=$(DEVICE_CC)
CFLAGS=-Os -Wall -g3 -ggdb -DF_CPU=16000000UL

MYNAME=avr-stepper-iface

OBJS=$(MYNAME).o rs485.o tiny485_syscfg.o stepper.o

all : $(MYNAME).hex $(MYNAME).lst

$(MYNAME).bin : $(OBJS)

%.hex : %.bin
	$(OBJCOPY) -j .text -j .data -O ihex $^ $@ || (rm -f $@ ; false )

%.lst : %.bin
	$(OBJDUMP) -S $^ >$@ || (rm -f $@ ; false )

%.bin : %.o
	$(LD) $(LDFLAGS) -o $@ $^
	$(SIZE) -C --mcu=$(DEVICE_CC) $@

include $(OBJS:.o=.d)

%.d : %.c
	$(CC) $(CPPFLAGS) -o $@ -MM $^

.PHONY : clean burn fuse read_eeprom write_eeprom
burn : $(MYNAME).hex
	$(AVRDUDE) $(PROGRAMMER_DUDE) -p $(DEVICE_DUDE) -U flash:w:$^
fuse :
	$(AVRDUDE) $(PROGRAMMER_DUDE) -p $(DEVICE_DUDE) \
		-U lfuse:w:$(FUSE_L):m -U hfuse:w:$(FUSE_H):m \
		-U efuse:w:$(FUSE_E):m
read_eeprom :
	$(AVRDUDE) $(PROGRAMMER_DUDE) -p $(DEVICE_DUDE) \
		-U eeprom:r:eeprom.txt:h
write_eeprom :
	$(AVRDUDE) $(PROGRAMMER_DUDE) -p $(DEVICE_DUDE) \
		-U eeprom:w:eeprom.txt:h

clean :
	rm -f *.bak *~ *.bin *.hex *.lst *.o *.d eeprom.txt
