#!/usr/bin/env python3
import serial, sys

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/pts/7"
ser = serial.Serial(port, 115200, timeout=0.01)
print("Fake device on", port)
while True:
    data = ser.read(256)
    if data:
        # For now just print and send a 1-byte ACK
        print("RX", data.hex())
        ser.write(b'\x06')  # ACK
