TARGET = main
CC = gcc
CFLAGS = -g -Wall
OBJS = main.o
LIBDIR = -L../../lib/
LIBS = -lmatrix -lm
SRC = main.c
INCLUDE = -I../../include

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(INCLUDE) -o $(TARGET) $(SRC) $(LIBDIR) $(LIBS)

clean:
	rm -f $(TARGET) $(OBJS)
	
