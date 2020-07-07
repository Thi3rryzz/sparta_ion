#!/usr/bin/python3
# Little script to sniff and test communication to CU3 Display, Far from perfect, but pretty functional.
import sys
import serial
import msvcrt

FRAME_HEADER = 0x10
ADDR_CU3_Display = 0xC1
wakePacket = bytearray([0x10,0xC1,0x2D,0x28,0x00,0x00,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2B])
string = []

if(len(sys.argv) < 2):
    print("Please run using: python3 SerialSniff.py *COMPORT*")
    exit()

print("Sparta ION Serial Sniffer V1.0")
print("Made for testing CU3 Display")
print("Press w to wake up display")
print("Press q to quit")

ser = serial.Serial(sys.argv[1], 19200, timeout=None)
while True:
    if msvcrt.kbhit():
        keyPressed = msvcrt.getch()
        if(keyPressed == b'q'):
            ser.close()
            exit()
        elif(keyPressed == b'w'):
            ser.write(wakePacket)

    bytesToRead = ser.inWaiting()
    if(bytesToRead > 0):
        data = ser.read(bytesToRead)
        for byte in data:
            # Separate strings correctly by Frame header.
            if(byte == FRAME_HEADER):
                # Only print strings bigger than 4 bytes. Don't know what the smaller ones do.
                if(len(string)>4):
                    print(string)
                string=[]
            string.append("{}".format(format(byte, 'x')))
