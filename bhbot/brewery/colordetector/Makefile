BINARY=colordetector

all:
	g++ -Wall -pedantic -O2 -o $(BINARY) `pkg-config --cflags opencv` $(BINARY).cpp `pkg-config --libs opencv`

clean:
	rm -f $(BINARY)

run:
	./$(BINARY)
