CC ?= gcc
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic -Werror -Ivendor
LDFLAGS += -Lvendor -Wl,-rpath,'$$ORIGIN/vendor'
LDLIBS += -lECanVci -lusb-0.1 -lpthread

.PHONY: all clean check

all: gcan_socketcan_bridge

gcan_socketcan_bridge: src/gcan_socketcan_bridge.c vendor/ECanVci.h vendor/libECanVci.so vendor/libusb-0.1.so
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

check: gcan_socketcan_bridge
	./gcan_socketcan_bridge --help
	ldd ./gcan_socketcan_bridge | grep 'not found' && exit 1 || true

clean:
	$(RM) gcan_socketcan_bridge
