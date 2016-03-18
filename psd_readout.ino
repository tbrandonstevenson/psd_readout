#include "position_measurement.h"

static const enum psd_id_list = {OPTICAL_TABLE, CAMERA}; 
static const      psd_id_list PSD_ID = CAMERA; 

static const int    LED     [2]    = { 24, 25 };
static const int    PSD_PIN [2][4] = {{ 0,  1,  2,  3}, { 4,  5,  6,  7}};
static const int    TRIG    [2][4] = {{41, 50, 34, 39}, {36, 37, 38, 40}}; 
static const int    TRIGf   [8]    = { 41, 50, 34, 39 ,  36, 37, 38, 40 }; 

static const int    GAIN[2][2]            = {{2,2},{1, 4}};  // per channel gain of each PSD
static const float  DAC_ATTENUATION[2][2] = {{1.0,1.0},{0.325,0.325}}; // attenuation value applied to the threshold comparator DACs

                                     //A1, A2, A3, A4, B1, B2, B3, B4
static const int    TRIG_ENABLE [8] = {1,  1,  1,  1,  1,  1,  1,  1}; 

static const int    NSAMPLES = 500;         // Total number of samples to accumulate before printing measurement 
static const int    NSAMPLES_PER_CYCLE = 3; // Number of samples to take in a duty cycle 

static const float  VREF = 2.5f;  // Analog Voltage Reference

char msg [110];

volatile bool triggered = false;

psdMeasurement readSensor(int ipsd);

bool pinstate = 0;

bool debug = 0;
bool print_stddev = 0;



void debugPrint () {
    psdMeasurement debug_data [2];

    debug_data[0] = readSensor(0); 
    debug_data[1] = readSensor(1); 

    sprintf(msg, "0: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f temp:%4.1f", 
        voltage(debug_data[0].x1, 0), 
        voltage(debug_data[0].x2, 0), 
        voltage(debug_data[0].y1, 0), 
        voltage(debug_data[0].y2, 0), 
        readTemperature());
    Serial.println(msg);

    sprintf(msg, "1: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f temp:%4.1f", 
        voltage(debug_data[1].x1, 1), 
        voltage(debug_data[1].x2, 1), 
        voltage(debug_data[1].y1, 1), 
        voltage(debug_data[1].y2, 1), 
        readTemperature());
    Serial.println(msg);
}

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
}; 

void sanitizePosition(struct position_t &position) {

    // Handle Cases of NAN
    if (position.x!=position.x || position.y!=position.y)  {
        position.x    =999.99999; 
        position.y    =999.99999;
        position.x_err=  9.99999; 
        position.y_err=  9.99999; 
    }
    if (position.x_err!=position.x_err || position.y_err!=position.y_err)  {
        position.x_err=  9.99999; 
        position.y_err=  9.99999; 
    }

}

void measurePosition () {
    struct position_t sum [2]; 
    sum[0].reset(); 
    sum[1].reset(); 

    int cnt=0; 
    while (cnt<NSAMPLES) {
        struct dualPSDMeasurement data = measureAmplitude(); 

        // psd0
        sum[0].x = sum[0].x + data.psd0.x(); 
        sum[0].y = sum[0].y + data.psd0.y(); 

        sum[0].x_sq = sum[0].x_sq + sum[0].x*sum[0].x; 
        sum[0].y_sq = sum[0].y_sq + sum[0].y*sum[0].y;

        // psd1
        sum[1].x = sum[1].x + data.psd1.x(); 
        sum[1].y = sum[1].y + data.psd1.y(); 

        sum[1].x_sq = sum[1].x_sq + sum[1].x*sum[1].x; 
        sum[1].y_sq = sum[1].y_sq + sum[1].y*sum[1].y;

        // increment
        cnt=cnt+1; 
    }

    struct position_t position [2];

    for (int ipsd=0; ipsd<2; ipsd++) {
        position[ipsd].x     = sum[ipsd].x   /(NSAMPLES*NSAMPLES_PER_CYCLE); 
        position[ipsd].y     = sum[ipsd].y   /(NSAMPLES*NSAMPLES_PER_CYCLE); 
        position[ipsd].x_sq  = sum[ipsd].x_sq/(NSAMPLES*NSAMPLES_PER_CYCLE); 
        position[ipsd].y_sq  = sum[ipsd].y_sq/(NSAMPLES*NSAMPLES_PER_CYCLE); 

        position[ipsd].x_err = sqrt(position[ipsd].x_sq - position[ipsd].x*position[ipsd].x); 
        position[ipsd].y_err = sqrt(position[ipsd].y_sq - position[ipsd].y*position[ipsd].y); 

        // Handle Cases of NAN
        sanitizePosition(position[ipsd]); 
    }

    if (print_stddev)   {
        sprintf(msg, "% 9.5f % 9.5f % 9.5f % 9.5f % 7.5f % 7.5f % 7.5f % 7.5f % 5.2f", 
                position[0].x, 
                position[0].y, 
                position[1].x, 
                position[1].y, 
                position[0].x_err, 
                position[0].y_err, 
                position[1].x_err, 
                position[1].y_err, 
                readTemperature());
    }
    else  {
        sprintf(msg, "% 9.5f % 9.5f % 9.5f % 9.5f % 5.2f", 
                position[0].x,
                position[0].y,
                position[1].x,
                position[1].y, 
                readTemperature());
    }

    Serial.println(msg);
}

struct dualPSDMeasurement measureAmplitude ()
{ 
    bool finished_reading=0; 
    bool last_read_state=0; 

    struct dualPSDMeasurement pos_high; 
    struct dualPSDMeasurement amplitude; 

    int timeout = 0; 
    while (!finished_reading)  {

        // let's not get stuck here forever..
        timeout++; 
        if (timeout > 500000) {
            amplitude.psd0.reset(); 
            amplitude.psd1.reset(); 
            return amplitude; 
        }; 

        if (triggered) { 

            triggered = false; 

            struct dualPSDMeasurement data = measurePositionOnce(); 

            bool state = (data.psd0.state==1 || data.psd1.state==1); 

            /* Check that Currently Reading State is opposite of Last Read State (HIGH vs. LOW) */
            if (state == last_read_state) { break; }
            else                          { last_read_state = state;}

            if (state==1) {
                pos_high = data; 
            }
            else { // if state==0 then we have a high state and low state accumulated
                amplitude.psd0 = pos_high.psd0 - data.psd0; 
                amplitude.psd1 = pos_high.psd1 - data.psd1; 
            }

            finished_reading = true; 
        }
    }

    return amplitude; 
}

struct dualPSDMeasurement measurePositionOnce() { 

    /* Position measurements used for this duty cycle's measurements*/ 
    struct dualPSDMeasurement data; 
    struct dualPSDMeasurement sum; 

    data.psd0.reset(); 
    data.psd1.reset(); 
     sum.psd0.reset(); 
     sum.psd1.reset(); 

    // wait for signals to rise
    delayMicroseconds(125);

    /* Take some number of samples in an individual duty cycle */
    for (int i=0; i<(2*NSAMPLES_PER_CYCLE); i++) {
      int ipsd = i % 2; // read PSD 0 on even, PSD1 on odd measurements

      if (ipsd==0) sum.psd0 = sum.psd0 + data.psd0; 
      if (ipsd==1) sum.psd1 = sum.psd1 + data.psd1; 
    }

    return sum; 
}

void measurePositionDebug() { 

    bool last_read_state=0; 

    psdMeasurement voltage_high;
    psdMeasurement voltage_low ;

    int count_high [2] = {0, 0};
    int count_low  [2] = {0, 0};

    double x1_min [2] = {4095, 4095}; 
    double x2_min [2] = {4095, 4095}; 
    double y1_min [2] = {4095, 4095}; 
    double y2_min [2] = {4095, 4095}; 

    double x1_max [2] = {0,0}; 
    double x2_max [2] = {0,0}; 
    double y1_max [2] = {0,0}; 
    double y2_max [2] = {0,0}; 

    psdMeasurement sum_low [2];
    psdMeasurement sum_high [2];

    psdMeasurement sum_sq_high[2];
    psdMeasurement sum_sq_low [2];

    bool finished_reading = 0; 

    while (!finished_reading) { 
        /* Position measurements used for this duty cycle's measurements*/ 
        psdMeasurement data [2];
        psdMeasurement sum  [2];

        memset (data, 0, sizeof(data[0]) * 2);
        memset (sum,  0, sizeof(sum [0]) * 2);

        // wait for signals to rise
        delayMicroseconds(200);

        /* Take some number of samples in an individual duty cycle */
        for (int i=0; i<(2*NSAMPLES_PER_CYCLE); i++) {
        int ipsd = i % 2; // read PSD 0 on even, PSD1 on odd measurements
        data[ipsd] = readSensor(ipsd); 
        sum[ipsd] = sum[ipsd] + data[ipsd];
        }

        /* This is the ANALOG threshold. It is NOT the one that is used to generate
        * trigger pulses, but rather is used to determine whether the PSD is in
        * high or low state. 
        */ 
        float analog_threshold = 0.10; 
        bool state = 0; 
        for (int ipsd = 0; ipsd < 2; ipsd++) {
        for (int i=0; i<2; i++) {
            state |= voltage(data[ipsd].x1, ipsd) > analog_threshold; 
            state |= voltage(data[ipsd].x2, ipsd) > analog_threshold;
            state |= voltage(data[ipsd].y1, ipsd) > analog_threshold; 
            state |= voltage(data[ipsd].y2, ipsd) > analog_threshold; 
        }
        }

        /* Check that Currently Reading State is opposite of Last Read State (HIGH vs. LOW) */
        if (state == last_read_state) { return; }
        else                          { last_read_state = state;}


        for (int ipsd = 0; ipsd < 2; ipsd++) {

        /* Accumulate values for high state in this duty cycle */ 
        if (state == HIGH && count_high[ipsd] < NSAMPLES) {
            count_high[ipsd] += 1;
            sum_high   [ipsd] = sum_high   [ipsd] + data[ipsd];
            sum_sq_high[ipsd] = sum_sq_high[ipsd] + data[ipsd]*data[ipsd];

            x1_max[ipsd] = max(data[ipsd].x1, x1_max[ipsd]); 
            x2_max[ipsd] = max(data[ipsd].x2, x2_max[ipsd]); 
            y1_max[ipsd] = max(data[ipsd].y1, y1_max[ipsd]); 
            y2_max[ipsd] = max(data[ipsd].y2, y2_max[ipsd]); 

            x1_min[ipsd] = min(data[ipsd].x1, x1_min[ipsd]); 
            x2_min[ipsd] = min(data[ipsd].x2, x2_min[ipsd]); 
            y1_min[ipsd] = min(data[ipsd].y1, y1_min[ipsd]); 
            y2_min[ipsd] = min(data[ipsd].y2, y2_min[ipsd]); 
        }

        /* Accumulate values for low state in this duty cycle */ 
        else if (state == LOW && count_low[ipsd] < NSAMPLES) {
            count_low[ipsd] += 1;
            sum_low   [ipsd] = sum_low   [ipsd] + data[ipsd];
            sum_sq_low[ipsd] = sum_sq_low[ipsd] + data[ipsd]*data[ipsd];
        }
        }

        /* Check if we have enough total samples for both the high and low states*/ 
        bool finished_measuring = false;
        for (int ipsd = 0; ipsd < 2; ipsd++) {
        finished_measuring |= ((count_low[ipsd] == NSAMPLES) && (count_high[ipsd] == NSAMPLES));
        }

        if (finished_measuring) {

        psdMeasurement amplitude [2] ;
        psdMeasurement mean_high [2];
        psdMeasurement mean_low  [2];
        psdMeasurement var_high  [2];
        psdMeasurement var_low   [2];
        psdMeasurement var       [2];
        psdMeasurement stddev    [2];


        double diffx1_0 = x1_max[0] - x1_min[0]; 
        double diffx2_0 = x2_max[0] - x2_min[0]; 
        double diffy1_0 = y1_max[0] - y1_min[0]; 
        double diffy2_0 = y2_max[0] - y2_min[0]; 

        double diffx1_1 = x1_max[1] - x1_min[1]; 
        double diffx2_1 = x2_max[1] - x2_min[1]; 
        double diffy1_1 = y1_max[1] - y1_min[1]; 
        double diffy2_1 = y2_max[1] - y2_min[1]; 

        double diffmax = 0; 

        // update the maximum difference if the current is the largest
        // we scale this by the GAIN of the channel to equalize ADC values
        diffmax = max(diffx1_0/GAIN[PSD_ID][0],diffmax); 
        diffmax = max(diffx2_0/GAIN[PSD_ID][0],diffmax); 
        diffmax = max(diffy1_0/GAIN[PSD_ID][0],diffmax); 
        diffmax = max(diffy2_0/GAIN[PSD_ID][0],diffmax); 
        diffmax = max(diffx1_1/GAIN[PSD_ID][1],diffmax); 
        diffmax = max(diffx2_1/GAIN[PSD_ID][1],diffmax); 
        diffmax = max(diffy1_1/GAIN[PSD_ID][1],diffmax); 
        diffmax = max(diffy2_1/GAIN[PSD_ID][1],diffmax); 

        for (int ipsd = 0; ipsd < 2; ipsd++) {
            mean_high[ipsd] = sum_high[ipsd]/NSAMPLES; 
            mean_low [ipsd] = sum_low [ipsd]/NSAMPLES; 
            
            amplitude [ipsd] = (mean_high[ipsd] - mean_low[ipsd]); 

            var_high[ipsd] = (sum_sq_high[ipsd]/NSAMPLES-mean_high[ipsd]*mean_high[ipsd]); 
            var_low[ipsd]  = ( sum_sq_low[ipsd]/NSAMPLES-mean_low [ipsd]*mean_low [ipsd]);

            var[ipsd] = var_high[ipsd] + var_low[ipsd]; 

            stddev[ipsd].x1 = sqrt(var[ipsd].x1); 
            stddev[ipsd].x2 = sqrt(var[ipsd].x2); 
            stddev[ipsd].y1 = sqrt(var[ipsd].y1); 
            stddev[ipsd].y2 = sqrt(var[ipsd].y2); 
        }

        for (int ipsd = 0; ipsd < 2; ipsd++) {
            // sprintf(msg, "%f %f %f %f", x1_max[ipsd], x2_max[ipsd], y1_max[ipsd], y2_max[ipsd]); 
            // Serial.println(msg);
            // sprintf(msg, "%f %f %f %f", x1_min[ipsd], x2_min[ipsd], y1_min[ipsd], y2_min[ipsd]); 
            // Serial.println(msg);

            /* Verbose Output */
            sprintf(msg, "%1i high: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f temp:%4.1f", ipsd, 
                voltage(mean_high[ipsd].x1, ipsd),
                voltage(mean_high[ipsd].x2, ipsd),
                voltage(mean_high[ipsd].y1, ipsd),
                voltage(mean_high[ipsd].y2, ipsd),
                readTemperature());
            Serial.println(msg);

            sprintf(msg, "           %6.4f    %6.4f    %6.4f    %6.4f",
                voltage(sqrt(var_high[ipsd].x1), ipsd),
                voltage(sqrt(var_high[ipsd].x2), ipsd),
                voltage(sqrt(var_high[ipsd].y1), ipsd),
                voltage(sqrt(var_high[ipsd].y2), ipsd)); 
            Serial.println(msg);

            sprintf(msg, "   low: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f temp:%4.1f",
                voltage(mean_low[ipsd].x1, ipsd),
                voltage(mean_low[ipsd].x2, ipsd),
                voltage(mean_low[ipsd].y1, ipsd),
                voltage(mean_low[ipsd].y2, ipsd), 
                readTemperature());
            Serial.println(msg);

            sprintf(msg, "           %6.4f    %6.4f    %6.4f    %6.4f",
                voltage(sqrt(var_low[ipsd].x1), ipsd),
                voltage(sqrt(var_low[ipsd].x2), ipsd),
                voltage(sqrt(var_low[ipsd].y1), ipsd),
                voltage(sqrt(var_low[ipsd].y2), ipsd)); 
            Serial.println(msg);

            sprintf(msg, "  stddev : %6.4f    %6.4f    %6.4f    %6.4f",
                voltage(stddev[ipsd].x1, ipsd),
                voltage(stddev[ipsd].x2, ipsd),
                voltage(stddev[ipsd].y1, ipsd),
                voltage(stddev[ipsd].y2, ipsd)); 
            Serial.println(msg);

            /* Calculated Output */
            sprintf(msg, "%1i x: % 6.4f y: % 6.4f", ipsd, 
                ((amplitude[ipsd].x2-amplitude[ipsd].x1))/((amplitude[ipsd].x2+amplitude[ipsd].x1)), 
                ((amplitude[ipsd].y2-amplitude[ipsd].y1))/((amplitude[ipsd].y2+amplitude[ipsd].y1)));
            Serial.println(msg);
        }  // psd loop

        finished_reading = true; 
        // reset position measurements
        for (int ipsd=0; ipsd<2; ipsd++) {
            x1_max[ipsd] = 0; 
            y1_max[ipsd] = 0; 
            x2_max[ipsd] = 0; 
            y2_max[ipsd] = 0; 

            x1_min[ipsd] = 4095; 
            y1_min[ipsd] = 4095; 
            x2_min[ipsd] = 4095; 
            y2_min[ipsd] = 4095; 

            count_low[ipsd] = 0; 
            count_high[ipsd] = 0; 

            sum_high[ipsd]     =  sum_high    [ipsd]*0;
            sum_low[ipsd]      =  sum_low     [ipsd]*0;
            sum_sq_high[ipsd]  =  sum_sq_high [ipsd]*0;
            sum_sq_low[ipsd]   =  sum_sq_low  [ipsd]*0;
        }
        }
    }
}

int beat=0; 

float voltage (float adc_counts, int ipsd) {
    return ((VREF*adc_counts)/(GAIN[PSD_ID][ipsd]*4095)); 
}

float voltageNoCal (float adc_counts) {
    return ((VREF*adc_counts)/(4095)); // factor of 2 is because the amplifier has gain of 2 !!
}

psdMeasurement readSensor(int ipsd)
{
    psdMeasurement data; 

    data.x1 = analogRead(PSD_PIN[ipsd][0]);
    data.x2 = analogRead(PSD_PIN[ipsd][1]);
    data.y1 = analogRead(PSD_PIN[ipsd][2]);
    data.y2 = analogRead(PSD_PIN[ipsd][3]);

    data.x1_sq = sq(data.x1); 
    data.x2_sq = sq(data.x2); 
    data.y1_sq = sq(data.y1); 
    data.y2_sq = sq(data.y2); 

    /* This is the ANALOG threshold. It is NOT the one that is used to generate
     * trigger pulses, but rather is used to determine whether the PSD is in
     * high or low state. 
     */ 
    float analog_threshold = 0.10; 

    data.state = 0; 

    data.state |= voltage(data.x1, ipsd) > analog_threshold; 
    data.state |= voltage(data.x2, ipsd) > analog_threshold;
    data.state |= voltage(data.y1, ipsd) > analog_threshold; 
    data.state |= voltage(data.y2, ipsd) > analog_threshold; 

    return data; 
}

/* Pin Change Interrupt Routines */
void interruptA1() { if (TRIG_ENABLE[0]) {interrupt();}}
void interruptA2() { if (TRIG_ENABLE[1]) {interrupt();}}
void interruptA3() { if (TRIG_ENABLE[2]) {interrupt();}}
void interruptA4() { if (TRIG_ENABLE[3]) {interrupt();}}
void interruptB1() { if (TRIG_ENABLE[4]) {interrupt();}}
void interruptB2() { if (TRIG_ENABLE[5]) {interrupt();}}
void interruptB3() { if (TRIG_ENABLE[6]) {interrupt();}}
void interruptB4() { if (TRIG_ENABLE[7]) {interrupt();}}

void interrupt()   { if (!triggered) {triggered = true;}};

float readTemperature() {
    int temp_measurements = 25; 
    double temperature_sum=0; 
    for (int imeas=0; imeas<temp_measurements; imeas++) {
      int   temp        = analogRead(9); 
      float voltage     = (temp * VREF) /((1<<12)-1); 
      float temperature = (voltage-0.424)/0.00625;  // conferre the LM60 data sheet for the origin of these magic numbers
      temperature_sum += temperature; 
    }
    return (temperature_sum/temp_measurements); 
}

int analogWriteVoltage (float voltage, int ipsd) { 
    // TODO: voltage should range between ~0.13 and 0.6875  

    float vref = VREF;  // 2.5V
    float attenuation = DAC_ATTENUATION[PSD_ID][ipsd]; 

    // the atsam3x8e processor has a DAC output voltage range that varies between 1/6 and 5/6 of VREF; 
    // on one PSD, at least, that voltage is decreased by an additional factor of ~1/3 (or more closely 0.325)
    int dac_counts = (voltage-1.0/6.0*vref*attenuation)*((1<<12)-1)/(4.0/6.0*vref*attenuation); 

    uint32_t dac_pin [2] = {DAC0, DAC1}; 

    analogWrite (dac_pin[ipsd], dac_counts); 

    return dac_counts; 
}

void calibrateThresholds () {
    struct dualPSDMeasurement mean = measurePedestal(); 

    float pedestal0 = 2.5; 
    pedestal0 = min(pedestal0, voltageNoCal(mean.psd0.x1)); 
    pedestal0 = min(pedestal0, voltageNoCal(mean.psd0.x2)); 
    pedestal0 = min(pedestal0, voltageNoCal(mean.psd0.y1)); 
    pedestal0 = min(pedestal0, voltageNoCal(mean.psd0.y2)); 

    float pedestal1 = 2.5; 
    pedestal1 = min(pedestal1, voltageNoCal(mean.psd1.x1)); 
    pedestal1 = min(pedestal1, voltageNoCal(mean.psd1.x2)); 
    pedestal1 = min(pedestal1, voltageNoCal(mean.psd1.y1)); 
    pedestal1 = min(pedestal1, voltageNoCal(mean.psd1.y2)); 

    struct dualPSDMeasurement amplitude = measureAmplitude(); 

    float amplitude0 = 0; 
    amplitude0 = max(amplitude0, voltageNoCal(mean.psd0.x1)); 
    amplitude0 = max(amplitude0, voltageNoCal(mean.psd0.x2)); 
    amplitude0 = max(amplitude0, voltageNoCal(mean.psd0.y1)); 
    amplitude0 = max(amplitude0, voltageNoCal(mean.psd0.y2)); 

    float amplitude1 = 0; 
    amplitude1 = max(amplitude1, voltageNoCal(mean.psd1.x1)); 
    amplitude1 = max(amplitude1, voltageNoCal(mean.psd1.x2)); 
    amplitude1 = max(amplitude1, voltageNoCal(mean.psd1.y1)); 
    amplitude1 = max(amplitude1, voltageNoCal(mean.psd1.y2)); 

    float threshold0 = (amplitude0-pedestal0) / 2; 
    float threshold1 = (amplitude1-pedestal1) / 2; 

    sprintf(msg, "Calibrating DAC thresholds to: %fV, %fV", threshold0, threshold1); 
    Serial.println(msg); 

    analogWriteVoltage (threshold0, 0); 
    analogWriteVoltage (threshold1, 1); 
}

void printPedestal () {

    struct dualPSDMeasurement dual_mean = measurePedestal(); 

    psdMeasurement mean [2]; 
    mean[0] = dual_mean.psd0; 
    mean[1] = dual_mean.psd1; 

    float x1_err[2]; 
    float x2_err[2]; 
    float y1_err[2]; 
    float y2_err[2]; 

    for (int ipsd=0; ipsd<2; ipsd++) {

        x1_err[ipsd] = sqrt(mean[ipsd].x1_sq - mean[ipsd].x1*mean[ipsd].x1); 
        x2_err[ipsd] = sqrt(mean[ipsd].x2_sq - mean[ipsd].x2*mean[ipsd].x2); 
        y1_err[ipsd] = sqrt(mean[ipsd].y1_sq - mean[ipsd].y1*mean[ipsd].y1); 
        y2_err[ipsd] = sqrt(mean[ipsd].y2_sq - mean[ipsd].y2*mean[ipsd].y2); 

        sprintf(msg, "psd%i pedestal: x1=% 9.5f x2=% 9.5f y1=% 9.5f y2=% 9.5f x1_err=% 7.5f x2_err=% 7.5f y1_err=% 7.5f y2_err=% 7.5f", 

                ipsd,

                voltage(mean[ipsd].x1,ipsd),
                voltage(mean[ipsd].x2,ipsd),
                voltage(mean[ipsd].y1,ipsd),
                voltage(mean[ipsd].y2,ipsd),

                voltage(x1_err[ipsd],ipsd),
                voltage(x2_err[ipsd],ipsd),
                voltage(y1_err[ipsd],ipsd),
                voltage(y2_err[ipsd],ipsd)
        );
        Serial.println(msg);
    }
}

struct dualPSDMeasurement measurePedestal () {
    psdMeasurement sum [2];
    psdMeasurement mean [2];
    sum[0].reset(); 
    sum[1].reset(); 

    int cnt=0; 
    while (cnt<NSAMPLES) {
        psdMeasurement data [2];

        data[0] = readSensor(0); 
        data[1] = readSensor(1); 

        bool state = (data[0].state==1 || data[1].state==1); 

        if (state==0) { 
            sum[0]=sum[0]+data[0]; 
            sum[1]=sum[1]+data[1]; 
            cnt=cnt+1; 
        }
    }

    for (int ipsd=0; ipsd<2; ipsd++) {
        mean[ipsd] = sum[ipsd]/NSAMPLES; 
    }

    struct dualPSDMeasurement dualpsd; 
    dualpsd.psd0 = mean[0]; 
    dualpsd.psd1 = mean[1]; 

    return dualpsd; 
}; 

void configureBoard () {

    analogReadResolution(12);
    analogWriteResolution(12);

    const char *psd_ascii[2] = {"OPTICAL_TABLE", "CAMERA       "}; 
    sprintf(msg, "PSD Firmware Compiled for PSD Station: %s", psd_ascii[PSD_ID]); 
    Serial.println(msg);

    // Set Default Threshold Voltages
    for (int ipsd=0; ipsd<2; ipsd++) { 
        int dac_counts = analogWriteVoltage(0.15, ipsd);  
        sprintf(msg, "Writing analog threshold to DAC%i: %i, %f", ipsd, dac_counts, voltage); 
        Serial.println(msg);
    }

    // Configure Trigger Interrupt
    for (int i=0; i<2; i++) {
        for (int j=0; j<4; j++) {
            pinMode     (TRIG[i][j], INPUT); 
            digitalWrite(TRIG[i][j],  HIGH); 
        }
    }

    printPedestal(); 

    attachInterrupt(TRIG[0][0], interruptA1, CHANGE);
    attachInterrupt(TRIG[0][1], interruptA2, CHANGE);
    attachInterrupt(TRIG[0][2], interruptA3, CHANGE);
    attachInterrupt(TRIG[0][3], interruptA4, CHANGE);
    attachInterrupt(TRIG[1][0], interruptB1, CHANGE);
    attachInterrupt(TRIG[1][1], interruptB2, CHANGE);
    attachInterrupt(TRIG[1][2], interruptB3, CHANGE);
    attachInterrupt(TRIG[1][3], interruptB4, CHANGE);

    // Configure DAC Chip Select
    pinMode(43, OUTPUT);
    digitalWrite(43, HIGH);

    /* Turn off LED */
    pinMode(LED[0], OUTPUT);
    digitalWrite(LED[0], LOW);

    pinMode(LED[1], OUTPUT);
    digitalWrite(LED[1], LOW);
}

void setup() {
  // Setup Serial Bus
  Serial.begin(115200);

  //SPI.begin();
  //SPI.setDataMode(SPI_MODE1);
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setClockDivider(4);

  Serial.println("configuring board...");

  configureBoard();

  Serial.println("board configured...");
}

void loop()
{
  if (debug) {
    beat++; 
    if (beat%1000000==0) {
	    triggered = 0; 
	    Serial.println("heartbeat..."); 
        debugPrint(); 
    }
  }

  if (Serial.available() > 0) {
      char byteIn = Serial.read(); 
      if      (byteIn == 'd') debug = !debug; 
      if      (byteIn == 's') print_stddev = !print_stddev; 
      if      (byteIn == 'c') calibrateThresholds; 
  }
  else if (triggered) {
    beat=0;
    if   (!debug) measurePosition(); 
    else          measurePositionDebug(); 
  }
}
