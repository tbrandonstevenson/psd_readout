/*
 * Compose and decode messages for TLCx5xx 4/8-channel ADCs
 */

#ifndef TLC3548_ADC_HPP
#define TLC3548_ADC_HPP

#include <stdint.h>
#define NBIT 14


enum OutputFormat { OF_BOB, OF_USB, OF_BTC };
enum SamplePeriod { SP_LONG, SP_SHORT };
enum ReferenceSelect { RS_INTERNAL, RS_EXTERNAL };
enum ConversionClock { CC_INTERNAL, CC_SCLK };
enum InputMode { IM_SINGLE_ENDED, IM_PSEUDO_DIFFERENTIAL };
enum ConversionMode { CM_ONE_SHOT, CM_REPEAT, CM_SWEEP, CM_REPEAT_SWEEP };
enum SweepSequence { SS_01234567, SS_02460246, SS_00224466, SS_02020202 };
enum PinFunction { PF_INT_BAR, PF_EOC };
enum TriggerLevel { TL_FULL, TL_75PC, TL_50PC, TL_25PC };

uint16_t codeCommand(uint16_t cmd, uint16_t data = 0);
uint16_t codeSelectChannel(unsigned ichan);
float voltData(const uint16_t data, const float full_volt = 5.0);

uint16_t codeSWPowerDown();
uint16_t codeSelectRefMid();
uint16_t codeSelectRefM();
uint16_t codeSelectRefP();
uint16_t codeSelect(unsigned ichan);
uint16_t codeReadFIFO();
uint16_t codeConfigDefault();

uint16_t fullScaleUSB();
uint16_t decodeUSB(uint16_t data);
int16_t decodeBOB(uint16_t data);
int16_t decodeBTC(uint16_t data);
float fracData(const uint16_t data);
float fracUSB(const uint16_t data);
float voltUSB(const uint16_t data, const float full_volt = 5.0);

uint16_t codeInitialize();
uint16_t codeConfig(
  SamplePeriod    sp = SP_SHORT,
  ReferenceSelect rs = RS_EXTERNAL,
  ConversionClock cc = CC_INTERNAL,
  ConversionMode  cm = CM_ONE_SHOT,
  SweepSequence   ss = SS_01234567,
  InputMode       im = IM_SINGLE_ENDED,
  OutputFormat    of = OF_BOB,
  PinFunction     pf = PF_EOC,
  TriggerLevel    tl = TL_50PC);

#endif // ndef TLC3548_ADC_HPP
