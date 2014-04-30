SRCDIR=src
INCDIR=include
OBJDIR=build
BINDIR=bin

F_CPU=8000000UL
CFLAGS=-mmcu=atmega32u4 -I$(INCDIR) -Wall -DF_CPU=$(F_CPU) -Os
LFLAGS=$(CFLAGS)
# AVRDUDE_FLAGS=-p atmega32u4 -c usbtiny
AVRDUDE_FLAGS=-p atmega32u4 -c avr109 -P /dev/ttyACM0

CROSS_COMPILE=avr-
CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
AVRDUDE=avrdude

all:	$(BINDIR)/smd_rf22.hex

$(BINDIR)/smd_rf22.hex: $(BINDIR)/smd_rf22.elf | $(BINDIR)
	$(OBJCOPY) -R .eeprom -O ihex $< $@

$(BINDIR)/smd_rf22.eep: $(BINDIR)/smd_rf22.elf | $(BINDIR)
	$(OBJCOPY) -j .eeprom -O ihex $< $@

$(BINDIR)/smd_rf22.elf:	$(OBJDIR)/main.o $(OBJDIR)/pll.o $(OBJDIR)/usb.o $(OBJDIR)/spi.o $(OBJDIR)/rf22.o $(OBJDIR)/eeprom.o $(OBJDIR)/eeprom_defaults.o $(OBJDIR)/debug.o | $(BINDIR)
	$(CC) $(LFLAGS) -o $@ $^

$(OBJDIR)/main.o:	$(SRCDIR)/main.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJDIR)/pll.o:	$(SRCDIR)/pll.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJDIR)/usb.o:	$(SRCDIR)/usb.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJDIR)/spi.o:	$(SRCDIR)/spi.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJDIR)/rf22.o:	$(SRCDIR)/rf22.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJDIR)/eeprom.o:	$(SRCDIR)/eeprom.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJDIR)/eeprom_defaults.o:	$(SRCDIR)/eeprom_defaults.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $^

$(OBJDIR)/debug.o:	$(SRCDIR)/debug.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $^

clean:
	@rm -rf $(OBJDIR) $(BINDIR)

program:	$(BINDIR)/smd_rf22.hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$^

initialize:	$(BINDIR)/smd_rf22.eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U eeprom:w:$^

$(OBJDIR):
	@mkdir $@

$(BINDIR):
	@mkdir $@
