BINARY=colordetector
CPPFLAGS+=-Wno-deprecated-writable-strings

all: release

$(BINARY):
	g++ -Wall -pedantic -o $(BINARY) $(BINARY).cpp -lopencv_core -lopencv_highgui -lopencv_imgproc $(CPPFLAGS)

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
