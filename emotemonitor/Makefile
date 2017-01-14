VERSION = 2.0.0

PREFIX = /usr/local
CFLAGS += -Wall -g
INSTALL = install

SOURCE =serial_posix.c	\
	serial.c		\
	main.c	
OBJS =	serial_posix.o	\
	serial.o		\
	main.o		

DEPS = serial.h serial_posix.h
all: emotemonitor-$(VERSION)

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
emotemonitor-$(VERSION): $(OBJS) 
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) emotemonitor

install: all
	$(INSTALL) -d $(DESTDIR)$(PREFIX)/bin
	$(INSTALL) -m 755 emotemonitor $(DESTDIR)$(PREFIX)/bin
	

.PHONY: all clean install
