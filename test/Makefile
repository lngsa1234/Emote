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
all: test

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
test: $(OBJS) 
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) test

install: all
	$(INSTALL) -d $(DESTDIR)$(PREFIX)/bin
	$(INSTALL) -m 755 test $(DESTDIR)$(PREFIX)/bin
	

.PHONY: all clean install
