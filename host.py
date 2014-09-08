#!/usr/bin/env python3

import socket
import logging

TCP_IP      = '169.232.153.99'
TCP_PORT    = 23
BUFFER_SIZE = 128

# Settings to Write
THRESH      = 255
BIAS        = 255
EN_A        = 1
EN_B        = 1
EN_C        = 1
EN_D        = 1

# Construct Command
COMMAND     = (BIAS   << 16)
COMMAND    |= (THRESH << 8)
COMMAND    |= (EN_A   << 0)
COMMAND    |= (EN_B   << 1)
COMMAND    |= (EN_C   << 2)
COMMAND    |= (EN_D   << 3)

# Number of measurements to take for each channel
NUM_MEAS_A = 100
NUM_MEAS_B = 0
NUM_MEAS_C = 0
NUM_MEAS_D = 0

# setup socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

#sent init command
s.send(COMMAND)

def main()
    try: 
        ReadData()
    except MisalignedFrame: 
        logger.error("Misaligned data frame")

def ReadData(): 
    while (true): 
        # nothing left to read
        if (COMMAND & 0x3) == 0x0:
            break

        data     =  s.recv(BUFFER_SIZE)
        header0  =  data[0]
        header1  =  data[1]
        rsvd     =  data[2]
        event    = (data[3]  << 16) | (data[4] <<  8) | (data[5] <<  0)
        x1       = (data[6]  << 4)  | data[7]
        x2       = (data[8]  << 4)  | data[9]
        y1       = (data[10] << 4)  | data[11]
        y2       = (data[12] << 4)  | data[13])
        crc      =  data[14]
        trailer1 =  data[15]


        if (header0  != 0x55): 
            except MisalignedFrame("header")
        if (trailer1 != 0xAA): 
            except MisalignedFrame("trailer")


        data[14] = 0
        crc_calc = crc8(data)

        if (crc_calc != crc)
            except CRCError("crc8 checksum failed")

        sensor  = header1 & 0x3

        # break if this sensor is not selected for whatever reason
        if ((sensor & (COMMAND&0x3)) == 0):
            break

        if (sensor = 0): 
            fileName = "A.log"
            countA += 1
        if (sensor = 1):
            fileName = "B.log"
            countB += 1
        if (sensor = 2): 
            fileName = "C.log"
            countC += 1
        if (sensor = 3): 
            fileName = "D.log"
            countD += 1

        if (countA > NUM_MEAS_A):
            COMMAND &= ~(0x1 << 0)
            s.send(COMMAND)
        if (countB > NUM_MEAS_B):
            COMMAND &= ~(0x1 << 1)
            s.send(COMMAND)
        if (countC > NUM_MEAS_C):
            COMMAND &= ~(0x1 << 2)
            s.send(COMMAND)
        if (countD > NUM_MEAS_D):
            COMMAND &= ~(0x1 << 3)
            s.send(COMMAND)

        error_crc = 0

        # write to file 
        try:
            with open(fileName,'w') as f:
                f.write("%08i: %04i %04i %04i %i\n", % (event, x1, x2, y1, y2, error_crc))
        except IOError:
            logger.error("Error opening file %s for writing" % fileName)
        
    # fini
    s.close()

class MisalignedFrame(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self)L

    return repr(self.value)

def crc8(b_array):
    crc = 0
    for m in range(len(b_array)):
        x = ord(b_array[m:m+1])
        for k in range(8):
            j = 1&(x ^ crc)
            crc = (crc // 2) & 255
            x = (x // 2) & 255
            if j != 0:
                crc = crc ^ 140
    return crc

if __name__ == "__main__":
    main()
