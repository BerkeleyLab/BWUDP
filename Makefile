CFLAGS = -std=c89 -O2 -pedantic
CFLAGS += -Wall -Wextra -Wundef -Wshadow
CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wredundant-decls
CFLAGS += -Wpointer-arith -Wcast-align -Wcast-qual -Wunreachable-code
CFLAGS += -Wnested-externs -Winline -Wwrite-strings -Waggregate-return

all: badger.o bwudp.o

clean:
	rm -f *.o
