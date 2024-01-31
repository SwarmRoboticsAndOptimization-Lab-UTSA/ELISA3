TARGET = main
CC = gcc
LDFLAGS = -lelisa3 -lm -lusb-1.0 -lpthread -lncurses
CFLAGS = -O2 -I../../pc-side-elisa3-library

LIBS = -L/usr/lib/i386-linux-gnu/ -L.


SRC = ../main.c
OBJS = $(SRC:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LIBS) $(LDFLAGS) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)

