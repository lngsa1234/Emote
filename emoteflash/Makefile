PREFIX = /usr/local
CFLAGS += -Wall -g

INSTALL = install

OBJS =	serial_posix.o	\
	Serial.c		\
	Ymodem.c	\
	main.o		


all: emoteflash


emoteflash: $(OBJS) 
	$(CC) $(LDFLAGS) -o $@ $(OBJS)

clean:
	rm -f $(OBJS) emoteflash

install: all
	$(INSTALL) -d $(DESTDIR)$(PREFIX)/bin
	$(INSTALL) -m 755 emoteflash $(DESTDIR)$(PREFIX)/bin
	

.PHONY: all clean install
