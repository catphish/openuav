CFLAGS=-O2 -Wall -Wextra -Warray-bounds -Wno-unused-parameter -Werror
CFLAGS+=`pkg-config --libs --cflags libusb-1.0`

SRC_FILES=$(wildcard *.c)

all: configurator
configurator: $(SRC_FILES)
	gcc -o configurator $(SRC_FILES) $(CFLAGS)
clean:
	rm -f configurator
