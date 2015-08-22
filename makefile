CXX= clang++ 
INSTALL_PATH?= /usr/local
CCFLAGS= -c -ffast-math -Wall -std=c++11
LDFLAGS= -dynamiclib
SRC= collisions.cpp
OBJ= $(SRC:.cpp=.o)
EXE= libcollisions.dylib

all: $(SRC) $(EXE)

$(EXE): $(OBJ)
	$(CXX) $(LDFLAGS) $(OBJ) -o $@
	make install

.cpp.o:
	$(CXX) $(CCFLAGS) $< -o $@

tidy:
	rm *.o
	
clean:
	rm $(EXE)
	rm *.o

install:
	cp ./*.h $(INSTALL_PATH)/include
	cp ./*.dylib $(INSTALL_PATH)/lib