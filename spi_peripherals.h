#ifndef SPI_PERIPHERALS_H
#define SPI_PERIPHERALS_H

/* ADC  */
uint16_t ADCWriteRead(uint16_t data); 
void initializeADC(); 
uint16_t measureADC(unsigned int ichan); 

/* DAC */
int setDAC(unsigned int dac_counts, unsigned int channel); 
unsigned int build_packet (unsigned int command, unsigned int adr, unsigned int data); 

#endif
