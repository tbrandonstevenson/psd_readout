#ifndef PSD_READOUT_H
#define PSD_READOUT_H 

static const enum psd_id_list = {OPTICAL_TABLE, CAMERA}; 
static const      psd_id_list PSD_ID = CAMERA; 

static const int    recalibration_threshold = 5; 

static const int    LED     [2]    = { 24, 25 };
static const int    PSD_PIN [2][4] = {{ 0,  1,  2,  3}, { 4,  5,  6,  7}};
static const int    TRIG    [2][4] = {{41, 50, 34, 39}, {36, 37, 38, 40}}; 
static const int    TRIGf   [8]    = { 41, 50, 34, 39 ,  36, 37, 38, 40 }; 

static const int    GAIN[2][2]            = {{2,2},{1, 4}};  // per channel gain of each PSD
static const float  DAC_ATTENUATION[2][2] = {{1.0,1.0},{0.325,0.325}}; // attenuation value applied to the threshold comparator DACs

/* This is the ANALOG threshold. It is NOT the one that is used to generate
* trigger pulses, but rather is used to determine whether the PSD is in
* high or low state. 
*/ 
float analog_threshold [2] = {0.10,0.10}; 

float MIN_THRESHOLD = 0.05; 

static const int TIMEOUT_COUNT = 500000; 

                                     //A1, A2, A3, A4, B1, B2, B3, B4
static const int    TRIG_ENABLE [8] = {1,  1,  1,  1,  1,  1,  1,  1}; 

static const int    NSAMPLES = 500;         // Total number of samples to accumulate before printing measurement 
static const int    NSAMPLES_PER_CYCLE = 3; // Number of samples to take in a duty cycle 

static const float  VREF = 2.5f;  // Analog Voltage Reference

void debugPrint (); 
void sanitizePosition(struct position_t &position); 
void printPositions(position_t position0, position_t position1); 
void measurePosition (); 
struct dualPSDMeasurement measureAmplitude (); 
struct dualPSDMeasurement measurePositionOnce(); 
void measurePositionDebug(); 
float voltage (float adc_counts, int ipsd); 
float voltageNoCal (float adc_counts); 
struct psdMeasurement readSensor(int ipsd); 
float readTemperature(); 
int analogWriteVoltage (float voltage, int ipsd); 
void calibrateThresholds (); 
void printPedestal (); 
struct dualPSDMeasurement measurePedestal (); 
void configureBoard (); 

struct position_t {
    float x; 
    float x_sq; 
    float y; 
    float y_sq; 

    float x_err; 
    float y_err; 

    void reset () {
        x      =  0;
        y      =  0;
        x_sq   =  0;
        y_sq   =  0;
        x_err  =  0;
        y_err  =  0;
    }; 
}; 

struct dualPSDMeasurement {
    psdMeasurement psd0; 
    psdMeasurement psd1; 

    double x1 (int ipsd) {(ipsd==0) ? return(psd0.x1) : return(psd1.x1) }
    double x2 (int ipsd) {(ipsd==0) ? return(psd0.x2) : return(psd1.x2) }
    double y1 (int ipsd) {(ipsd==0) ? return(psd0.y1) : return(psd1.y1) }
    double y2 (int ipsd) {(ipsd==0) ? return(psd0.y2) : return(psd1.y2) }

    double x  (int ipsd) {(ipsd==0) ? return(psd0.x()) : return(psd1.x()) }
    double y  (int ipsd) {(ipsd==0) ? return(psd0.y()) : return(psd1.y()) }
}; 

#endif /* PSD_READOUT_H */
