BINARY=colordetector

all:
	PKG_CONFIG_PATH=$(OPENCV_PATH)/lib/pkgconfig g++ -Wall -pedantic -O2 -o $(BINARY) `pkg-config --cflags opencv` *.cpp `pkg-config --libs opencv`

clean:
	rm -f $(BINARY)

run:
	LD_LIBRARY_PATH=$(OPENCV_PATH)/lib ./$(BINARY)
