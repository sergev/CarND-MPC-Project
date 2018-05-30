CXXFLAGS        = -Wall -std=c++11 -O
INCLUDES        = -I/usr/local/include
#LIBS            = -lipopt -lz -lssl -luv -luWS
LIBS            = -lnlopt -lz -lssl -luv -luWS
OBJS            = mpc.o main.o

all:            mpc

clean:
		rm -f mpc *.o

mpc:            $(OBJS)
		$(CXX) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

main.o: main.cpp mpc.h
mpc.o: mpc.cpp mpc.h
