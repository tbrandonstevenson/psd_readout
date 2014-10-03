#include <stdint.h>
#include "TLC3548_ADC.h"

uint16_t codeCommand(uint16_t cmd, uint16_t data)
{
    uint16_t out = 0; 
    out |= (cmd  & 0xF) << 12;
    out |= (data & 0xFFF); 
    return (out);
}

uint16_t codeSelectChannel(unsigned ichan)
{
    return codeCommand(ichan&0x7);
}

uint16_t codeSWPowerDown()
{
    return codeCommand(0x8);
}

uint16_t codeInitialize()
{
    return codeCommand(0xA);
}

uint16_t codeConfig(SamplePeriod sp, ReferenceSelect rs,
        ConversionClock cc, ConversionMode cm, SweepSequence ss, InputMode im,
        OutputFormat of, PinFunction pf, TriggerLevel tl) 
{
    uint16_t cfr = 0;
    if (rs == RS_EXTERNAL)               
        cfr |= 0x800;
    if (of == OF_BTC)                    
        cfr |= 0x400;
    if (sp == SP_SHORT)                  
        cfr |= 0x200;
    if (cc == CC_SCLK)                   
        cfr |= 0x100;
    if (im == IM_PSEUDO_DIFFERENTIAL)    
        cfr |= 0x080;
    switch(cm)
    {
        case CM_ONE_SHOT:
            break;
        case CM_REPEAT:
            cfr |= 0x020;
            break;
        case CM_SWEEP:
            cfr |= 0x040;
            break;
        case CM_REPEAT_SWEEP:
            cfr |= 0x060;
            break;
    };

    switch(ss)
    {
        case SS_01234567:
            break;
        case SS_02460246:
            cfr |= 0x008;
            break;
        case SS_00224466:
            cfr |= 0x010;
            break;
        case SS_02020202:
            cfr |= 0x018;
            break;
    };

    if(pf == PF_EOC)
        cfr |= 0x004;

    switch(tl)
    {
        case TL_FULL:
            break;
        case TL_75PC:
            cfr |= 0x001;
            break;
        case TL_50PC:
            cfr |= 0x002;
            break;
        case TL_25PC:
            cfr |= 0x003;
            break;
    }
    return codeCommand(0xA,cfr);
}

uint16_t codeSelectRefMid()
{
    return codeCommand(0xB);
}

uint16_t codeSelectRefM()
{
    return codeCommand(0xC);
}

uint16_t codeSelectRefP()
{
    return codeCommand(0xD);
}

uint16_t codeSelect(unsigned ichan)
{
    if(ichan<8)
        return codeSelectChannel(ichan);
    else if (ichan==8)
        return codeSelectRefP();
    else if (ichan==9)
        return codeSelectRefMid();
    else if (ichan==10)
        return codeSelectRefM();
    else
        return (0); 
}

uint16_t codeReadFIFO()
{
    return codeCommand(0xE);
}

uint16_t codeConfigDefault()
{
    return codeCommand(0xF);
}

uint16_t fullScaleUSB()
{
    return (0x1<<NBIT)-1;
}

uint16_t decodeUSB(uint16_t data)
{
    data >>= (16-NBIT);
    data  &= (0x3FFF);
    data  &= ((0x1<<NBIT)-1);
    return data;
}

int16_t decodeBOB(uint16_t data)
{
    data >>= (16-NBIT);
    data &= ((0x1<<NBIT)-1);
    return static_cast<int16_t>(data)-(0x1<<(NBIT-1));
}

int16_t decodeBTC(uint16_t data)
{
    uint16_t udata = data;
    udata >>= (16-NBIT);
    udata &= ((0x1<<(NBIT-1))-1);
    if (data & 0x00008000)
        udata |= 0xFFFF8000;
    return static_cast<int16_t>(udata);
}

float fracData(const uint16_t data)
{
    return float(data)/float(fullScaleUSB());
}

float voltData(const uint16_t data, const float full_volt)
{
    return fracData(data)*full_volt;
}

float fracUSB(const uint16_t data)
{
    return float(decodeUSB(data))/float(fullScaleUSB());
}

float voltUSB(const uint16_t data, const float full_volt)
{
    return fracUSB(data)*full_volt;
}
