# setup environment

#import DEBUG
#import MESSAGES

TARGET_ARCH = -mcpu=cortex-m3 -mthumb

COMPILE_OPTS = -Os -g $(WARNINGS) $(TARGET_OPTS) $(MESSAGES) $(F_NO_STRICT_ALIASING) $(INCLUDE_DIRS)
WARNINGS = -Wall -W -Wshadow -Wwrite-strings -Winline
INCLUDE_DIRS = -I ../fwlib -I ../fwlib/inc

# needed to safely build the library (until proven otherwise)
F_NO_STRICT_ALIASING = -fno-strict-aliasing


TARGET_OPTS = $(F_INLINE) $(F_INLINE_ONCE) $(F_UNROLL_LOOPS)
F_INLINE = -finline
F_INLINE_ONCE = -finline-functions-called-once


CPPFLAGS = $(DEBUG_MACRO)

CC = arm-none-eabi-gcc
CFLAGS = $(COMPILE_OPTS)

AS = $(CC) -x assembler-with-cpp -c $(TARGET_ARCH)
ASFLAGS = $(COMPILE_OPTS)

AR = arm-none-eabi-ar
ARFLAGS = cr


LIB_OUT = libstm32-uartserial.a

LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard *.c)) \
 $(patsubst %.s,%.o,$(wildcard src/*.s)))

LIB_OBJS+=stm32f10x_usart.o
# all

.PHONY: all
all: $(LIB_OUT)


# lib

$(LIB_OUT): $(LIB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJS)

$(LIB_OBJS): $(wildcard *.h) $(wildcard inc/*.h)


# clean

.PHONY: clean
clean:
	-rm -f $(LIB_OBJS) $(LIB_OUT)
