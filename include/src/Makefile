TARGET=libmatrix.a
LIBDIR=../../lib/
CC=gcc
CFLAGS=-g -Wall
SRC=matrix.c
ARC=ar
ARCFLAGS=-cvq
OBJS=matrix.o

$(OBJS): $(SRC)
	$(CC) $(CFLAGS) -c $(SRC)
        
lib: $(OBJS)
	$(ARC) $(ARCFLAGS) $(LIBDIR)$(TARGET) $^

clean:
	rm -rf $(OBJS) $(LIBDIR)$(TARGET)
