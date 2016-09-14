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
all: emotemonitor

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
emotemonitor: $(OBJS) 
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) emotemonitor

install: all
	$(INSTALL) -d $(DESTDIR)$(PREFIX)/bin
	$(INSTALL) -m 755 emotemonitor $(DESTDIR)$(PREFIX)/bin
	

.PHONY: all clean install
