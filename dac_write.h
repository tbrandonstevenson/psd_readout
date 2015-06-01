#ifndef SPI_PERIPHERALS_H
#define SPI_PERIPHERALS_H

// SPI CS Channels
#define DAC_CS1      22U
#define DAC_CS2      23U

// DAC Channels
#define OAVCC        0U // todo: fix these
#define VR3          1U
#define VTHRESH      1U

/* DAC */
int setDAC(int idac, int channel, unsigned int dac_counts);
unsigned int build_packet (unsigned int command, unsigned int adr, unsigned int data);

#endif
