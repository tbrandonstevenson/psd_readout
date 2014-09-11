#!/usr/bin/env python3

import socket
import logging
import struct
import time

TCP_IP      = '169.232.153.99'
TCP_PORT    = 23
BUFFER_SIZE = 128

# Number of measurements to take for each channel
NUM_MEAS = 1

# setup socket

PACKET_LENGTH = 10

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def main():
    s.connect((TCP_IP, TCP_PORT))
    for i in range(NUM_MEAS):
        ReadSensor()
    s.close()

def ReadSensor(): 
    #sent init command
    cmd = bytearray(struct.pack("B", 0xFF))
    #s.send(cmd)

    #remaining = PACKET_LENGTH
    #data = bytearray()
    #while remaining > 0:
    #    chunk = s.recv(remaining)    # Get available data
    #    data.extend(chunk)            # Add to message
    #    remaining -= len(chunk)

    #print(data)

    #header   =  data[0]
    #x1       = (data[1] << 4) | data[2]
    #x2       = (data[3] << 4) | data[4]
    #y1       = (data[5] << 4) | data[6]
    #y2       = (data[7] << 4) | data[8]
    #trailer  =  data[9]

    ## write to file 
    #fileName = "data.txt"
    #try:
    #    with open(fileName,'w') as f:
    #        f.write("%04i %04i %04i %04i\n" % (x1, x2, y1, y2))
    #except IOError:
    #    logger.error("Error opening file %s for writing" % fileName)
        
    # fini

if __name__ == "__main__":
    main()
