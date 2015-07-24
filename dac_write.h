#ifndef DAC_WRITE_H
#define DAC_WRITE_H

// SPI CS Channels
#define DAC_CS       43U

/* DAC */
void setDAC (int channel, unsigned int dac_counts);

#endif
