INCLUDE_DIRS =
LIB_DIRS =
CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt

HFILES=
CFILES= negative.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:    negative

clean:
        -rm -f *.o *.d
        -rm -f negative
        -rm -f output.ppm       

distclean:
        -rm -f *.o *.d
