import serial

port = serial.PosixSerial('/dev/ttyUSB0', 115200)

while True:
    c = port.read(1)
    print(ord(c))
