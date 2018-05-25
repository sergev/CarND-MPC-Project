CXXFLAGS        = -Wall -std=c++11 -O
INCLUDES        = -I/usr/local/include
LIBS            = -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lipopt -lz -lssl -luv -luWS
OBJS            = MPC.o main.o

all:            mpc

clean:
		rm -f mpc *.o

mpc:            $(OBJS)
		$(CXX) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

MPC.o:          MPC.cpp
main.o:         main.cpp
