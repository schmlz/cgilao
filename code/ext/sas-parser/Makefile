OBJ = axiom.o helper_functions.o mutex_group.o operator.o state.o variable.o parser.o

CFLAGS	+= --std=c++11 --pedantic -Wall
C++	=	g++

all: parser

parser: $(OBJ)
	$(C++) -o parser $(OBJ) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(OBJ) parser

.cc.o:
		$(C++) $(CFLAGS) $(INCLUDE) -c $<

