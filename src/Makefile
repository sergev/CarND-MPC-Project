CXXFLAGS        = -Wall -std=c++11 -O
INCLUDES        = -I/usr/local/include
LIBS            = -luWS -luv -lssl -lz
OBJS            = mpc.o main.o compass.o direct.o

all:            mpc

clean:
		rm -f mpc *.o

mpc:            $(OBJS)
		$(CXX) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

cdirect.o: cdirect.cpp cdirect.h redblack.h
compass.o: compass.cpp compass.h
direct.o: direct.cpp direct.h
main.o: main.cpp mpc.h
mpc.o: mpc.cpp mpc.h direct.h compass.h
redblack.o: redblack.cpp redblack.h
