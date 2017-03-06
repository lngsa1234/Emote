VERSION = diagnosis
PREFIX = /usr/local
CFLAGS += -Wall -g

INSTALL = install

SOURCE =serial_posix.c	\
	serial.c		\
	ymodem.c	\
	main.c	
OBJS =	serial_posix.o	\
	serial.o		\
	ymodem.o	\
	main.o		

DEPS = serial.h ymodem.h serial_posix.h
TARGET = serial-$(VERSION)
all: $(TARGET)

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
$(TARGET): $(OBJS) 
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) $(TARGET)

install: all
	$(INSTALL) -d $(DESTDIR)$(PREFIX)/bin
	$(INSTALL) -m 755 $(TARGET) $(DESTDIR)$(PREFIX)/bin
	

.PHONY: all clean install
