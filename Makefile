
TARGET := stm32isp
OBJS := main.o hexfile.o memmap.o serial.o stm32.o term.o

RM := rm -rf
CC := gcc
CFLAG := -O3 -Wall

# All Target
all: $(TARGET)

# Tool invocations
stm32isp: $(OBJS)
	$(CC)  -o $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAG) -c -o $@ $<

# deps
hexfile.o : hexfile.c hexfile.h memmap.h
main.o : main.c stm32.h hexfile.h memmap.h term.h
memmap.o : memmap.h
serial.o : serial.h 
stm32.o : stm32.h serial.h memmap.h
term.o : term.h serial.h stm32.h

# install
install: $(TARGET)
	sudo cp $(TARGET) /usr/local/bin

# Other Targets
clean:
	-$(RM) $(OBJS) $(TARGET)

.PHONY: all clean dependents
.SECONDARY: