# commands
CC := gcc
CFLAG := -O3 -Wall
RM := rm -rf

# target and source
TARGET := stm32isp
OBJS := main.o hexfile.o memmap.o serial.o stm32.o term.o

# All Target
all: $(TARGET)

# deps
hexfile.o : hexfile.c hexfile.h memmap.h
main.o : main.c stm32.h hexfile.h memmap.h term.h
memmap.o : memmap.h
serial.o : serial.h 
stm32.o : stm32.h serial.h memmap.h
term.o : term.h serial.h stm32.h

# Tool invocations
stm32isp: $(OBJS)
	$(CC)  -o $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAG) -c -o $@ $<

# test
test: $(TARGET)
	./$(TARGET) -term 9600 -verbose -bootp RTS -reset DTR blink.hex /dev/tty.usbserial-AE01DHMV 115200

# install
install: $(TARGET)
	sudo cp $(TARGET) /usr/local/bin

# Other Targets
clean:
	-$(RM) $(OBJS) $(TARGET)

.PHONY: all clean dependents
.SECONDARY:
