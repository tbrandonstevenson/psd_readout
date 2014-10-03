#include <SPI.h>
#include "spi_peripherals.h"
#include "TLC3548_ADC.h"

//------------------------------------------------------------------------------
// ADCs
//------------------------------------------------------------------------------

uint16_t ADCWriteRead(uint16_t data) {
    digitalWrite(4,LOW); 
    delayMicroseconds(5);
    byte byte1 = SPI.transfer((data >> 8) & 0xFF); 
    byte byte2 = SPI.transfer((data >> 0) & 0xFF);
    delayMicroseconds(5);
    digitalWrite(4,HIGH); 

    uint16_t out = 0; 
    out |= byte1 << 8; 
    out |= byte2; 

    return (out);
}

void initializeADC() {
    //ADCWriteRead(codeInitialize());
    ADCWriteRead(codeConfigDefault()); 
    ADCWriteRead(codeConfig());
}

uint16_t measureADC(unsigned int ichan) {

    //initializeADC(); 
    //ADCWriteRead(codeConfig());

    //Poll ADC Channel
    uint16_t code = codeSelect(ichan);
    ADCWriteRead(code);

    // Read out Data
    uint16_t datum = ADCWriteRead(codeReadFIFO());

    return(datum >> 2); 
    //return (decodeUSB(datum));
}
