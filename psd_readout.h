#ifndef PSD_READOUT_H
#define PSD_READOUT_H 

volatile bool triggered = false;
bool emulate=false; 

bool print_stddev          = false;

bool pinstate              = false;
bool debug                 = false;
bool dynamic_recalibration = true;
bool abort_reading         = false;
bool continuous_readout    = true;

int beat                   = 0;
int meas_cnt               = 0;

char msg [110];

static const int OPTICAL_TABLE = 0; 
static const int CAMERA = 1; 
static const int PSD_ID = CAMERA; 

static const int    recalibration_threshold = 1; 

static const int    LED     [2]    = { 24, 25 };
static const int    PSD_PIN [2][4] = {{ 0,  1,  2,  3}, { 4,  5,  6,  7}};
static const int    TRIG    [2][4] = {{41, 50, 34, 39}, {36, 37, 38, 40}}; 
static const int    TRIGf   [8]    = { 41, 50, 34, 39 ,  36, 37, 38, 40 }; 

static const double  GAIN[2][2]            = {{2,2},{1, 4}};  // per channel gain of each PSD
static const double  DAC_ATTENUATION[2][2] = {{1.0,1.0},{0.325,0.325}}; // attenuation value applied to the threshold comparator DACs

/* This is the ANALOG threshold. It is NOT the one that is used to generate
* trigger pulses, but rather is used to determine whether the PSD is in
* high or low state. 
*/ 

double MIN_THRESHOLD = 0.13; 
double analog_threshold [2] = {0.15, 0.15}; 
double   trig_threshold [2] = {MIN_THRESHOLD, MIN_THRESHOLD}; 

static const int TIMEOUT_COUNT = 1000; 

                                     //A1, A2, A3, A4, B1, B2, B3, B4
static const int    TRIG_ENABLE [8] = {1,  1,  1,  1,  1,  1,  1,  1}; 

static const int    NSAMPLES = 500;         // Total number of samples to accumulate before printing measurement 
static const int    NSAMPLES_PER_CYCLE = 3; // Number of samples to take in a duty cycle 

static const double  VREF = 2.5f;  // Analog Voltage Reference

int analogWriteVoltage (double voltage, int ipsd); 
double readTemperature(); 
double voltage (double adc_counts, int ipsd); 
double voltageNoCal (double adc_counts); 

void debugPrint (); 
void readPosition (); 
void readPositionDebug(); 
void calibrateThresholds (bool silent = true); 
void printPedestal (); 

struct dualPSDMeasurement readPSDs(); 
struct dualPSDMeasurement readAmplitude (); 
struct dualPSDMeasurement measurePedestal (bool silent=true); 
struct dualPSDMeasurement measurePeak (bool silent=true); 
struct dualPSDMeasurement measurePeakPedestal (bool target_state, bool silent=true); 

void configureBoard (); 

#endif /* PSD_READOUT_H */
