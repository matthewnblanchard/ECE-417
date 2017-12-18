TARGET=main
OBJS=${TARGET}.o

CXX=g++
CC=gcc
CXXFLAGS=-g -Wall -Wextra
CFLAGS=-g -Wall -Wextra

LIBLABVOLT=${LIBDIR}/liblabvolt.a

# Include the labvolt and usb libraries
INC=-I/usr/local/include/labvolt/inc -I../../include
LIBSDIR=-L/usr/local/include/labvolt/lib -L../../lib/
LIBS=${LIBSDIR} -llabvolt -ludev -lmatrix -lm

all: ${TARGET}

# Compile lab
${TARGET}: ${OBJS}
	${CC} ${CFLAGS} -o ${TARGET} ${OBJS} ${INC} ${LIBS}

${OBJS}: ${TARGET}.c
	${CC} ${CFLAGS} -c -o ${TARGET}.o ${TARGET}.c ${INC} ${LIBS}


# If the compiler complains about a missing -llabvolt library type
# make labvolt
# Or it will auto generate it if the library isn't in LIBDIR


clean:
	rm -f *.o core* ${TARGET}
