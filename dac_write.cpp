#include <SPI.h>
#include "spi_peripherals.h"

/*
 * DAC Write Functions for AD5623R
 *
 * Datasheet: 
 * http://www.analog.com/static/imported-files/data_sheets/AD5623R_43R_63R.pdf
 *
 * 24 bit data format: 
 * | DC | DC | CMD2 | CMD1 | CMD0 | A2 | A1 | A0 | D13 ...0 | X | X| 
 *
 * Command Bits: 
 * C2 | C1 | C0 | Command
 * ---+----+----+--------------------------------------------------------
 *  0 |  0 |  0 | Write to Input Register n
 *  0 |  0 |  1 | Update DAC Register n
 *  0 |  1 |  0 | Write to Input Register n , update all (software LDAC)
 *  0 |  1 |  1 | Write to and update DAC Channel n
 *  1 |  0 |  0 | Power down DAC (power up)
 *  1 |  0 |  1 | Reset
 *  1 |  1 |  0 | LD AC register setup
 *  1 |  1 |  1 | Internal reference setup (on/off )
 *
 * Address bits: 
 * A2 | A1 | A0 | ADDRESS (n)
 * ---+----+----+------------
 * 0  | 0  | 0  | DAC A
 * 0  | 0  | 1  | DAC B
 * 0  | 1  | 0  | Reserved
 * 0  | 1  | 1  | Reserved
 * 1  | 1  | 1  | All DACs
 */

#define WRITE_AND_UPDATE 0x3
#define RESET            0x5
#define IREF_CTL         0x7

int setDAC(unsigned int dac_counts, unsigned int channel)
{
    // Check for invalid dac value
    if ((dac_counts < 0) | (dac_counts > 16383)) {
        return (-1); 
    }

    // Check for invalid channel
    if ((channel < 0) | (channel > 1)) {
        return (-1); 
    }

    // construct packet of 24-bit data format
    unsigned int data = build_packet (WRITE_AND_UPDATE, channel, dac_counts); 

    // decompose packet into 3 bytes and write. 
    digitalWrite(52, LOW); 
    SPI.transfer((data >> 16) & 0xFF);
    SPI.transfer((data >>  8) & 0xFF);
    SPI.transfer((data >>  0) & 0xFF);
    digitalWrite(52, HIGH); 
    return (1);
}

unsigned int build_packet (unsigned int command, unsigned int adr, unsigned int data) {
    unsigned int packet = 0;
    packet |= (0x7    & command) << 19; //CMD Bits
    packet |= (0x7    & adr)     << 16; //ADR Bits
    packet |= (0x3FFF & data)    <<  2; //Value
    return (packet); 
}
