//TODO min/max checking
#include "position_measurement.h"
#include "psd_readout.h"


char msg [110];

volatile bool triggered = false;

bool pinstate              = false;
bool debug                 = false;
bool print_stddev          = false;
bool dynamic_recalibration = false;
bool abort_reading         = false;
int beat                   = 0;
int meas_cnt               = 0;

void setup() {
  // Setup Serial Bus
  Serial.begin(115200);
  Serial.println("Configuring board...");
  configureBoard();
}

void loop()
{
    beat++; 
    if (beat%1000000==0) {
        triggered = 0; 
        diagnosticPrint(); 
    }

    if (Serial.available() > 0) {
        char byteIn = Serial.read(); 
        if      (byteIn == 'd') debug = !debug; 
        if      (byteIn == 's') print_stddev = !print_stddev; 
        if      (byteIn == 'c') calibrateThresholds; 
        if      (byteIn == 'r') dynamic_recalibration = !dynamic_recalibration; 
    }
    else if (meas_cnt==recalibration_threshold && dynamic_recalibration) {
        meas_cnt=0; 
        calibrateThresholds(); 
    }
    else if (triggered) {
        beat=0;
        if   (!debug) measurePosition(); 
        else          measurePositionDebug(); 
        meas_cnt++; 
    }
}

void diagnosticPrint () {

    struct psdMeasurement debug_data [2];

    debug_data[0] = readSensor(0); 
    debug_data[1] = readSensor(1); 

    for (int ipsd=0; ipsd<2; ipsd++) { 
        sprintf(msg, "heartbeat: psd%i: x1=%6.4f x2=%6.4f y1=%6.4f y2=%6.4f temp=%4.1f", 
            ipsd, 
            voltage(debug_data[ipsd].x1, ipsd), 
            voltage(debug_data[ipsd].x2, ipsd), 
            voltage(debug_data[ipsd].y1, ipsd), 
            voltage(debug_data[ipsd].y2, ipsd), 
            readTemperature()
        );
        Serial.println(msg);
    }
}


void sanitizePosition(struct position_t &position) {

    // Handle Cases of NAN
    bool data_err = (position.x!=position.x || position.y!=position.y ||
            position.x_err!=position.x_err || position.y_err!=position.y_err ); 

    if (data_err)  {
        position.x    =999.99999; 
        position.y    =999.99999;
        position.x_err=  9.99999; 
        position.y_err=  9.99999; 
    }

}

void printPositions(position_t position0, position_t position1) {

    if (print_stddev)   {
        sprintf(msg, "% 9.5f % 9.5f % 9.5f % 9.5f % 7.5f % 7.5f % 7.5f % 7.5f % 5.2f", 
                position0.x, 
                position0.y, 
                position0.x, 
                position0.y, 
                position1.x_err, 
                position1.y_err, 
                position1.x_err, 
                position1.y_err, 
                readTemperature());
    }
    else  {
        sprintf(msg, "% 9.5f % 9.5f % 9.5f % 9.5f % 5.2f", 
                position0.x,
                position0.y,
                position1.x,
                position1.y, 
                readTemperature());
    }

    Serial.println(msg);
}


void measurePosition () {
    struct position_t sum [2]; 
    sum[0].reset(); 
    sum[1].reset(); 

    int cnt=0; 
    while (cnt<NSAMPLES) {

        // allow the sub-processes to abort_reading the reading.. for timeout, or bad data, etc.. 
        if (abort_reading) {
            abort_reading = !abort_reading; 
            return; 
        }

        struct dualPSDMeasurement data = measureAmplitude(); 
        for (int ipsd=0; ipsd<2; ipsd++) {
            sum[ipsd].x = sum[ipsd].x + data.x(ipsd); 
            sum[ipsd].y = sum[ipsd].y + data.y(ipsd); 

            sum[ipsd].x_sq = sum[ipsd].x_sq + sum[ipsd].x*sum[ipsd].x; 
            sum[ipsd].y_sq = sum[ipsd].y_sq + sum[ipsd].y*sum[ipsd].y;
        }

        // increment
        cnt=cnt+1; 
    }

    struct position_t position [2];

    for (int ipsd=0; ipsd<2; ipsd++) {
        position[ipsd].x     = sum[ipsd].x    / (NSAMPLES*NSAMPLES_PER_CYCLE);
        position[ipsd].y     = sum[ipsd].y    / (NSAMPLES*NSAMPLES_PER_CYCLE);
        position[ipsd].x_sq  = sum[ipsd].x_sq / (NSAMPLES*NSAMPLES_PER_CYCLE);
        position[ipsd].y_sq  = sum[ipsd].y_sq / (NSAMPLES*NSAMPLES_PER_CYCLE);

        position[ipsd].x_err = sqrt(position[ipsd].x_sq - position[ipsd].x*position[ipsd].x); 
        position[ipsd].y_err = sqrt(position[ipsd].y_sq - position[ipsd].y*position[ipsd].y); 

        // Handle Cases of NAN
        sanitizePosition(position[ipsd]); 
    }

    printPositions(position[0], position[1]); 

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
        if (timeout > TIMEOUT_COUNT) {
            abort_reading = true; 
            amplitude.reset(); 
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

    data.reset(); 
    sum.reset(); 

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

    struct psdMeasurement voltage_high;
    struct psdMeasurement voltage_low ;

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

    struct psdMeasurement sum_low [2];
    struct psdMeasurement sum_high [2];

    struct psdMeasurement sum_sq_high[2];
    struct psdMeasurement sum_sq_low [2];

    bool finished_reading = 0; 

    while (!finished_reading) { 
        /* Position measurements used for this duty cycle's measurements*/ 
        struct psdMeasurement data [2];
        struct psdMeasurement sum  [2];

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
        bool state = 0; 
        for (int ipsd = 0; ipsd < 2; ipsd++) {
        for (int i=0; i<2; i++) {
            state |= (voltage(data[ipsd].x1, ipsd) > analog_threshold[ipsd]); 
            state |= (voltage(data[ipsd].x2, ipsd) > analog_threshold[ipsd]);
            state |= (voltage(data[ipsd].y1, ipsd) > analog_threshold[ipsd]); 
            state |= (voltage(data[ipsd].y2, ipsd) > analog_threshold[ipsd]); 
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

        struct psdMeasurement amplitude [2] ;
        struct psdMeasurement mean_high [2];
        struct psdMeasurement mean_low  [2];
        struct psdMeasurement var_high  [2];
        struct psdMeasurement var_low   [2];
        struct psdMeasurement var       [2];
        struct psdMeasurement stddev    [2];


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


float voltage (float adc_counts, int ipsd) {
    return ((VREF*adc_counts)/(GAIN[PSD_ID][ipsd]*4095)); 
}

float voltageNoCal (float adc_counts) {
    return ((VREF*adc_counts)/(4095)); // factor of 2 is because the amplifier has gain of 2 !!
}

struct psdMeasurement readSensor(int ipsd)
{
    struct psdMeasurement data; 

    data.x1 = analogRead(PSD_PIN[ipsd][0]);
    data.x2 = analogRead(PSD_PIN[ipsd][1]);
    data.y1 = analogRead(PSD_PIN[ipsd][2]);
    data.y2 = analogRead(PSD_PIN[ipsd][3]);

    data.x1_sq = data.x1*data.x1; 
    data.x2_sq = data.x2*data.x2; 
    data.y1_sq = data.y1*data.y1; 
    data.y2_sq = data.y2*data.y2; 

    data.state = 0; 
    data.state |= voltage(data.x1, ipsd) > analog_threshold[ipsd]; 
    data.state |= voltage(data.x2, ipsd) > analog_threshold[ipsd];
    data.state |= voltage(data.y1, ipsd) > analog_threshold[ipsd]; 
    data.state |= voltage(data.y2, ipsd) > analog_threshold[ipsd]; 

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
    int num_meas = 25; 
    double temperature_sum=0; 
    for (int imeas=0; imeas<num_meas; imeas++) {
      int   temp        = analogRead(9); 
      float voltage     = (temp * VREF) /((1<<12)-1); 
      float temperature = (voltage-0.424)/0.00625;  // conferre the LM60 data sheet for the origin of these magic numbers
      temperature_sum += temperature; 
    }
    return (temperature_sum/num_meas); 
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

//TODO need to make this work sanely with no-laser 
void calibrateThresholds () {
    struct dualPSDMeasurement pedestal_reading  = measurePedestal(); 
    struct dualPSDMeasurement amplitude_reading = measureAmplitude(); 

    float max_amplitude [2] = {0,0}; 
    float min_pedestal  [2] = {2.5,2.5}; 
    float threshold [2]; 

    for (int ipsd=0; ipsd<2; ipsd++) { 
        min_pedestal[ipsd] = min(min_pedestal[ipsd], voltageNoCal(pedestal_reading.x1(ipsd))); 
        min_pedestal[ipsd] = min(min_pedestal[ipsd], voltageNoCal(pedestal_reading.x2(ipsd))); 
        min_pedestal[ipsd] = min(min_pedestal[ipsd], voltageNoCal(pedestal_reading.y1(ipsd))); 
        min_pedestal[ipsd] = min(min_pedestal[ipsd], voltageNoCal(pedestal_reading.y2(ipsd))); 

        max_amplitude [ipsd] = max(max_amplitude[ipsd], voltageNoCal(amplitude_reading.x1(ipsd))); 
        max_amplitude [ipsd] = max(max_amplitude[ipsd], voltageNoCal(amplitude_reading.x2(ipsd))); 
        max_amplitude [ipsd] = max(max_amplitude[ipsd], voltageNoCal(amplitude_reading.y1(ipsd))); 
        max_amplitude [ipsd] = max(max_amplitude[ipsd], voltageNoCal(amplitude_reading.y2(ipsd))); 

        threshold[ipsd] = max(MIN_THRESHOLD, (max_amplitude[ipsd]-min_pedestal[ipsd]) / 3.0); 

        analogWriteVoltage (threshold[ipsd], ipsd); 
        analog_threshold[ipsd] = threshold[ipsd];  
    }

    sprintf(msg, "Calibrating DAC thresholds to: %fV, %fV", threshold[0], threshold[1]); 
    Serial.println(msg); 
}

void printPedestal () {

    struct dualPSDMeasurement dual_mean = measurePedestal(); 

    struct psdMeasurement mean [2]; 
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
    struct psdMeasurement sum [2];
    struct psdMeasurement mean [2];
    sum[0].reset(); 
    sum[1].reset(); 

    int cnt=0; 
    int start_time = millis(); 
    bool no_trig_mode = false; 

    while (cnt<NSAMPLES) {
        if (triggered || no_trig_mode) {
            triggered = false; 

            struct psdMeasurement data [2];

            data[0] = readSensor(0); 
            data[1] = readSensor(1); 

            bool state = (data[0].state==1 || data[1].state==1); 

            if (state==0) { 
                sum[0]=sum[0]+data[0]; 
                sum[1]=sum[1]+data[1]; 
                cnt=cnt+1; 
            }
        } 
        else {
            int ellapsed_time = millis() - start_time; 
            if (ellapsed_time > 100) {
                Serial.println("Entering no_trig mode for pedestal calibration. Laser should be OFF!"); 
                no_trig_mode = true; 
            }
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

    sprintf(msg, "Gain: Ch0=%4.3f Ch1=%4.3f", GAIN[PSD_ID][0], GAIN[PSD_ID][1]); 
    Serial.println(msg);

    sprintf(msg, "DAC Attenuation: Ch0=%4.3f Ch1=%4.3f", DAC_ATTENUATION[PSD_ID][0], DAC_ATTENUATION[PSD_ID][1]); 
    Serial.println(msg);

    // Set Default Threshold Voltages
    for (int ipsd=0; ipsd<2; ipsd++) { 
        int dac_counts = analogWriteVoltage(0.15, ipsd);  
        sprintf(msg, "Writing threshold to DAC%i: %i, %f", ipsd, dac_counts, voltage); 
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

    Serial.println ("d to toggle debug mode");
    Serial.println ("s to toggle stddev output"); 
    Serial.println ("c to recalibrate thresholds"); 
    Serial.println ("r to toggle dynamic recalibration"); 

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
