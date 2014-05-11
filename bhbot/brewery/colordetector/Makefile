BINARY=colordetector
CPPFLAGS+=
SOURCES=$(BINARY).cpp

all: release

$(BINARY): $(SOURCES)
	g++ -Wall -pedantic -o $(BINARY) $(SOURCES) -lopencv_core -lopencv_highgui -lopencv_imgproc $(CPPFLAGS)

debug: CPPFLAGS += -g
debug: CFLAGS += -g
debug: $(BINARY)

release: CPPFLAGS += -O2
release: CFLAGS += -O2
release: $(BINARY)

clean:
	rm -f $(BINARY)

run:
	./$(BINARY)
