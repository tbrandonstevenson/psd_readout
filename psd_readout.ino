/* TODO min/max checking */

#include "position_measurement.h"
#include "psd_readout.h"

/* Pin Change Interrupt Routines */
void interruptA1() { if (TRIG_ENABLE[0]) {interrupt();}}
void interruptA2() { if (TRIG_ENABLE[1]) {interrupt();}}
void interruptA3() { if (TRIG_ENABLE[2]) {interrupt();}}
void interruptA4() { if (TRIG_ENABLE[3]) {interrupt();}}
void interruptB1() { if (TRIG_ENABLE[4]) {interrupt();}}
void interruptB2() { if (TRIG_ENABLE[5]) {interrupt();}}
void interruptB3() { if (TRIG_ENABLE[6]) {interrupt();}}
void interruptB4() { if (TRIG_ENABLE[7]) {interrupt();}}

void interrupt()   {triggered = true;};

void setup() {
    /* Setup Serial Port */
    Serial.begin(115200);

    /* Configure board properties */
    configureBoard();
    Serial.println("");
    Serial.println("Finished configuring board.");
    Serial.println("");
}

void loop()
{
    /* if we go a long time without a trigger, try recalibrating thresholds and produce a diagnostic printout  */
    beat++;
    if (beat%1000000==0) {
        triggered = false;
        if (debug) diagnosticPrint();
        //calibrateThresholds();
    }

    /* parse serial commands */
    if (Serial.available() > 0) {
        char byteIn = Serial.read();
        if      (byteIn == 'd') debug = !debug;
        if      (byteIn == 's') print_stddev = !print_stddev;
        if      (byteIn == 'c') calibrateThresholds();
        if      (byteIn == 'r') dynamic_recalibration = !dynamic_recalibration;
        if      (byteIn == 'e') emulate = !emulate;
    }

    /* continuous triggering mode */
    else if (triggered) {
        beat=0;
        if   (!debug) readPosition(NSAMPLES);
        else          readPositionDebug();
        meas_cnt++;
    }

    /* recalibrate automatically if we have dynamic calibration enabled.. and some time has elapsed since the last reading */
    else if (meas_cnt>=recalibration_threshold && dynamic_recalibration) {
        meas_cnt=0;
        calibrateThresholds();
    }


}

/* print an UNTRIGGERED readout of the current voltage reading (no averaging) */
void diagnosticPrint () {

    struct psdMeasurement debug_data [2];

    debug_data[0].read(0); 
    debug_data[1].read(1); 

    for (int ipsd=0; ipsd<2; ipsd++) {
        sprintf(msg, "     psd%i: x1=%6.4f x2=%6.4f y1=%6.4f y2=%6.4f temp=%4.1f",
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

void readPosition (int nsamples) {
    struct dualPosition positions; 
    positions.reset(); 

    double x [2]={0,0}; 
    double y [2]={0,0}; 

    double x_sq [2]={0,0}; 
    double y_sq [2]={0,0}; 

    int cnt=0;
    while (cnt<nsamples) {
        if (triggered) {
            /* allow the sub-processes to abort_reading the reading.. for timeout, or bad data, etc..  */
            if (abort_reading) {
                abort_reading = !abort_reading;
                Serial.println ("READING WAS ABORTED");
                return;
            }

            /* measure signal amplitude on both PSDs */
            struct dualPSDMeasurement data = readAmplitude();
            for (int ipsd=0; ipsd<2; ipsd++) {
                positions.set_x (ipsd, positions.x(ipsd) + data.x(ipsd));
                positions.set_y (ipsd, positions.y(ipsd) + data.y(ipsd));

                positions.set_x_sq(ipsd, positions.x_sq(ipsd) + data.x(ipsd)*data.x(ipsd));
                positions.set_y_sq(ipsd, positions.y_sq(ipsd) + data.y(ipsd)*data.y(ipsd));
            }

            /* increment measurement counter */
            cnt=cnt+1;
            triggered = false;
        }

        //TODO: probably want to add a timeout monitor here..? 
    }

    for (int ipsd=0; ipsd<2; ipsd++) {
        positions.set_x(ipsd, positions.x(ipsd)    / (nsamples));
        positions.set_y(ipsd, positions.y(ipsd)    / (nsamples));

        positions.set_x_sq(ipsd, positions.x_sq(ipsd) / (nsamples));
        positions.set_y_sq(ipsd, positions.y_sq(ipsd) / (nsamples));

        positions.set_x_err (ipsd, sqrt(positions.x_sq(ipsd) - positions.x(ipsd)*positions.x(ipsd)));
        positions.set_y_err (ipsd, sqrt(positions.y_sq(ipsd) - positions.y(ipsd)*positions.y(ipsd)));

        /* Handle Cases of NAN */
        positions.sanitize();
    }

    /* print to serial port  */
    positions.print();

}

/* measures the mean amplitude (over NSAMPLES_PER_CYCLE) of both PSDs */
struct dualPSDMeasurement readAmplitude ()
{
    bool finished_reading=0;
    bool last_read_state=0;

    struct dualPSDMeasurement pos_high;
    struct dualPSDMeasurement amplitude;

    bool read_high = false;

    int last_read = millis();

    while (!finished_reading)  {
        if (triggered) {
            last_read==millis();

            /* take nsamples_per_cycle readings of the amplitude */
            struct dualPSDMeasurement data = readPSDs();

            /* take the OR allowing either PSD to generate the trigger.  */
            bool state = data.state();

            /* if we are in the high state, save a copy of the data (to use when reading the low state)  */
            if (state) {
                pos_high = data;
                read_high = true;
            }
            else if (read_high)  {
                amplitude.psd0 = pos_high.psd0 - data.psd0;
                amplitude.psd1 = pos_high.psd1 - data.psd1;
                // sprintf(msg, "psd%i: x1=%6.4f x2=%6.4f y1=%6.4f y2=%6.4f temp=%4.1f", 1, voltage(amplitude.psd1.x1, 1), voltage(amplitude.psd1.x2, 1), voltage(amplitude.psd1.y1, 1), voltage(amplitude.psd1.y2, 1), readTemperature());
                // Serial.println(msg);
                finished_reading = true;
            }
            triggered = false;
        }

        /* let's not get stuck here forever.. so monitor for a timeout to abort the reading if there are no triggers */
        else {
            if (millis()-last_read > TIMEOUT_COUNT) {
                abort_reading = true;
                Serial.println("TIMEOUT");
                triggered=false;
                amplitude.reset();
                return amplitude;
            };
        }
    }

    if (emulate) {

        int sign; 

        sign = random(0,1)*2-1; 

        amplitude.psd0.x1=1 + sign * random(100)/10000.0; 
        amplitude.psd0.x2=3 - sign * random(100)/10000.0; 
        amplitude.psd0.y1=1 + sign * random(100)/10000.0; 
        amplitude.psd0.y2=3 - sign * random(100)/10000.0; 

        sign = random(0,1)*2-1; 

        amplitude.psd1.x1=1 + sign * random(100)/10000.0; 
        amplitude.psd1.x2=3 - sign * random(100)/10000.0; 
        amplitude.psd1.y1=1 + sign * random(100)/10000.0; 
        amplitude.psd1.y2=3 - sign * random(100)/10000.0; 
    }

    return amplitude;
}

/* takes NSAMPLES_PER_CYCLE measurements on the PSD. This is UNTRIGGERED... so
 * you should wait for a trigger before calling this function */
struct dualPSDMeasurement readPSDs() {

    /* Position measurements used for this duty cycle's measurements*/
    struct dualPSDMeasurement data;
    struct dualPSDMeasurement sum;

    data.reset();
    sum.reset();

    /* wait for signals to rise */
    delayMicroseconds(125);

    /* Take some number of samples in an individual duty cycle */
    for (int i=0; i<(2*NSAMPLES_PER_CYCLE); i++) {

        /* read PSD 0 on even, PSD1 on odd measurements */
        int ipsd = i % 2;

        if (ipsd==0) {
            data.psd0.read(ipsd); 
            sum.psd0  = sum.psd0 + data.psd0;
            sum.psd0.state |= data.psd0.state;
        }
        if (ipsd==1)  {
            data.psd1.read(ipsd); 
            sum.psd1 = sum.psd1 + data.psd1;
            sum.psd1.state |= data.psd1.state;
        }
    }

    sum.psd0 = sum.psd0/NSAMPLES_PER_CYCLE; 
    sum.psd1 = sum.psd1/NSAMPLES_PER_CYCLE; 

    return sum;
}

void readPositionDebug() {

    bool last_read_state=0;

    struct psdMeasurement voltage_high [2];
    struct psdMeasurement voltage_low  [2];

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
    struct psdMeasurement sum_high[2]; 

    struct psdMeasurement sum_sq_low [2];
    struct psdMeasurement sum_sq_high[2]; 

    bool finished_reading = 0;

    while (!finished_reading) {
        if (triggered)  {
            /* Position measurements used for this duty cycle's measurements*/
            struct psdMeasurement data [2];
            struct psdMeasurement sum [2]; 

            data[0].reset(); 
            data[1].reset(); 
             sum[0].reset(); 
             sum[1].reset(); 

            // wait for signals to rise
            delayMicroseconds(125);

            bool state=false; 

            /* Take some number of samples in an individual duty cycle */
            for (int i=0; i<(2*NSAMPLES_PER_CYCLE); i++) {

                /* read PSD 0 on even, PSD1 on odd measurements */
                int ipsd = i % 2;

                data[ipsd].read(ipsd); 
                sum[ipsd] = sum[ipsd] + data[ipsd];
                state |= data[ipsd].state;
            }

            /* Check that Currently Reading State is opposite of Last Read State (HIGH vs. LOW) */
            if (state == last_read_state) { continue; }
            else                          { last_read_state = state;}


            for (int ipsd=0; ipsd<2; ipsd++) {
                /* Accumulate values for high state in this duty cycle */
                if (state==1 && count_high[ipsd]<NSAMPLES) {
                    count_high [ipsd] += 1;
                    sum_high   [ipsd]  = sum_high   [ipsd] + data[ipsd];
                    sum_sq_high[ipsd]  = sum_sq_high[ipsd] + data[ipsd]*data[ipsd];

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
                else if (state==0 && count_low[ipsd] < NSAMPLES) {
                    count_low [ipsd] += 1;
                    sum_low   [ipsd]  = sum_low   [ipsd] + data[ipsd];
                    sum_sq_low[ipsd]  = sum_sq_low[ipsd] + data[ipsd]*data[ipsd];
                }
            }

            /* Check if we have enough total samples for both the high and low states*/
            for (int ipsd = 0; ipsd < 2; ipsd++) {
                finished_reading |= ((count_low[ipsd] == NSAMPLES) && (count_high[ipsd] == NSAMPLES));
            }
        }
    }

    struct psdMeasurement amplitude [2] ;
    struct psdMeasurement mean_high [2];
    struct psdMeasurement mean_low  [2];
    struct psdMeasurement var_high  [2];
    struct psdMeasurement var_low   [2];
    struct psdMeasurement var       [2];
    struct psdMeasurement stddev    [2];

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
    }  /* psd loop */

    /* reset position measurements */
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

        sum_high[ipsd].reset();
        sum_low[ipsd].reset();
        sum_sq_high[ipsd].reset();
        sum_sq_low[ipsd].reset();
    }
}


double voltage (double adc_counts, int ipsd) {
    return ((VREF*adc_counts)/(GAIN[PSD_ID][ipsd]*4095));
}

double voltageNoCal (double adc_counts) {
    return ((VREF*adc_counts)/(4095));
}

double readTemperature() {
    int num_meas = 25;
    double temperature_sum=0;
    for (int imeas=0; imeas<num_meas; imeas++) {
        int   temp        = analogRead(9);
        double voltage     = (temp * VREF) /((1<<12)-1);
        double temperature = (voltage-0.424)/0.00625;  /* conferre the LM60 data sheet for the origin of these magic numbers */
        temperature_sum += temperature;
    }
    return (temperature_sum/num_meas);
}

int analogWriteVoltage (double voltage, int ipsd) {
    /* TODO: voltage should range between ~0.13 and 0.6875 */
    /* need to figure this out exactly.. */

    voltage = max(0.14,   voltage);
    voltage = min(0.68, voltage);

    double vref = VREF;  /* 2.5V */
    double attenuation = DAC_ATTENUATION[PSD_ID][ipsd];

    /* the atsam3x8e processor has a DAC output voltage range that varies between 1/6 and 5/6 of VREF;
     * on one PSD, at least, that voltage is decreased by an additional factor of ~1/3 (or more closely 0.325)
     */
    int dac_counts = (voltage-1.0/6.0*vref*attenuation)*((1<<12)-1)/(4.0/6.0*vref*attenuation);

    uint32_t dac_pin [2] = {DAC0, DAC1};

    analogWrite (dac_pin[ipsd], dac_counts);

    return dac_counts;
}

/* TODO need to make this work sanely with no-laser */
void calibrateThresholds () {
    Serial.println ("Recalibrating PSD Thresholds:"); 
    struct dualPSDMeasurement pedestal_reading  = measurePedestal();
    struct dualPSDMeasurement      peak_reading = measurePeak();

    double max_peak      [2] = {0,0};
    double min_pedestal  [2] = {2.5,2.5};
    double threshold [2];

    for (int ipsd=0; ipsd<2; ipsd++) {
        min_pedestal[ipsd] = min(min_pedestal[ipsd], voltageNoCal(pedestal_reading.x1(ipsd)));
        min_pedestal[ipsd] = min(min_pedestal[ipsd], voltageNoCal(pedestal_reading.x2(ipsd)));
        min_pedestal[ipsd] = min(min_pedestal[ipsd], voltageNoCal(pedestal_reading.y1(ipsd)));
        min_pedestal[ipsd] = min(min_pedestal[ipsd], voltageNoCal(pedestal_reading.y2(ipsd)));

        max_peak [ipsd] = max(max_peak[ipsd], voltageNoCal(peak_reading.x1(ipsd)));
        max_peak [ipsd] = max(max_peak[ipsd], voltageNoCal(peak_reading.x2(ipsd)));
        max_peak [ipsd] = max(max_peak[ipsd], voltageNoCal(peak_reading.y1(ipsd)));
        max_peak [ipsd] = max(max_peak[ipsd], voltageNoCal(peak_reading.y2(ipsd)));

        sprintf(msg, "    PSD%i pedestal=%5.3fV, peak=%5.3fV", ipsd, min_pedestal[ipsd], max_peak[ipsd]); 
        Serial.println(msg);

        threshold[ipsd] = max(MIN_THRESHOLD, (max_peak[ipsd]-min_pedestal[ipsd]) / 4.0);

        analogWriteVoltage (threshold[ipsd], ipsd);
        analog_threshold[ipsd] = threshold[ipsd];
        trig_threshold[ipsd] = threshold[ipsd];
    }

    sprintf(msg, "    Calibrating DAC thresholds to: %fV, %fV", threshold[0], threshold[1]);
    Serial.println(msg);
}

void printDualPSDMeasurement (struct dualPSDMeasurement dual_mean ) {

    struct psdMeasurement mean [2];
    mean[0] = dual_mean.psd0;
    mean[1] = dual_mean.psd1;

    double x1_err[2];
    double x2_err[2];
    double y1_err[2];
    double y2_err[2];

    for (int ipsd=0; ipsd<2; ipsd++) {

        // TODO: need to fix this! 
        x1_err[ipsd] = sqrt(mean[ipsd].x1_sq - mean[ipsd].x1*mean[ipsd].x1);
        x2_err[ipsd] = sqrt(mean[ipsd].x2_sq - mean[ipsd].x2*mean[ipsd].x2);
        y1_err[ipsd] = sqrt(mean[ipsd].y1_sq - mean[ipsd].y1*mean[ipsd].y1);
        y2_err[ipsd] = sqrt(mean[ipsd].y2_sq - mean[ipsd].y2*mean[ipsd].y2);

        sprintf(msg, "    psd%i: x1=%6.4f x2=%6.4f y1=%6.4f y2=%6.4f \(x1_err=%6.4f x2_err=%6.4f y1_err=%6.4f y2_err=%6.4f\)",

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

struct dualPSDMeasurement measurePeak () {
    return (measurePeakPedestal(1));
}

struct dualPSDMeasurement measurePedestal () {
    return (measurePeakPedestal(0));
}

struct dualPSDMeasurement measurePeakPedestal (bool target_state) {
    struct psdMeasurement sum [2];
    struct psdMeasurement mean [2];
    sum[0].reset();
    sum[1].reset();


    for (int ipsd=0; ipsd<2; ipsd++) {

        int cnt = 0;
        bool no_trig_mode = false;
        int last_trig_time = millis();

        while (cnt<NSAMPLES) {
            if (!no_trig_mode && (millis()-last_trig_time > 1000)) {
                sprintf(msg, "    PSD0: n.b. no_trig mode for peak/pedestal measurement of PSD. Laser is off or low amplitude for this PSD", ipsd);
                Serial.println(msg);
                no_trig_mode = true;
            }
            else if (triggered || no_trig_mode) {

                struct psdMeasurement data;

                data.read(ipsd);

                if (data.state==target_state || no_trig_mode) {
                    last_trig_time=millis();
                    sum[ipsd]=sum[ipsd]+data;
                    sum[ipsd].x1_sq = sum[ipsd].x1_sq + data.x1*data.x1; 
                    sum[ipsd].x2_sq = sum[ipsd].x2_sq + data.x2*data.x2; 
                    sum[ipsd].y1_sq = sum[ipsd].y1_sq + data.y1*data.y1; 
                    sum[ipsd].y2_sq = sum[ipsd].y2_sq + data.y2*data.y2; 
                    cnt=cnt+1;
                }
            }
        }
        triggered = false;
    }

    struct dualPSDMeasurement dualpsd;
    dualpsd.psd0 = sum [0]/NSAMPLES;
    dualpsd.psd1 = sum [1]/NSAMPLES;

    return dualpsd;
}

void configureBoard () {

    analogReadResolution(12);
    analogWriteResolution(12);

    /* Configure Trigger Interrupt */
    for (int i=0; i<2; i++) {
        for (int j=0; j<4; j++) {
            pinMode     (TRIG[i][j], INPUT);
            digitalWrite(TRIG[i][j],  HIGH);
        }
    }

    attachInterrupt(TRIG[0][0], interruptA1, CHANGE);
    attachInterrupt(TRIG[0][1], interruptA2, CHANGE);
    attachInterrupt(TRIG[0][2], interruptA3, CHANGE);
    attachInterrupt(TRIG[0][3], interruptA4, CHANGE);
    attachInterrupt(TRIG[1][0], interruptB1, CHANGE);
    attachInterrupt(TRIG[1][1], interruptB2, CHANGE);
    attachInterrupt(TRIG[1][2], interruptB3, CHANGE);
    attachInterrupt(TRIG[1][3], interruptB4, CHANGE);

    /* Turn off LED */
    pinMode(LED[0], OUTPUT);
    digitalWrite(LED[0], LOW);

    pinMode(LED[1], OUTPUT);
    digitalWrite(LED[1], LOW);

    Serial.println("PSD Settings:");
    const char *psd_ascii[2] = {"OPTICAL_TABLE", "CAMERA       "};
    sprintf(msg, "    PSD Firmware Compiled for PSD Station: %s", psd_ascii[PSD_ID]);
    Serial.println(msg);

    sprintf(msg, "    Gain: Ch0=%4.3f (PSD122C) Ch1=%4.3f (PSD122D)", GAIN[PSD_ID][0], GAIN[PSD_ID][1]);
    Serial.println(msg);

    sprintf(msg, "    DAC Attenuation: Ch0=%4.3f Ch1=%4.3f", DAC_ATTENUATION[PSD_ID][0], DAC_ATTENUATION[PSD_ID][1]);
    Serial.println(msg);

    /* Set Default Threshold Voltages */
    //analogWrite(DAC0, 100);
    //analogWrite(DAC1, 100);

    for (int ipsd=0; ipsd<2; ipsd++) {
        int dac_counts = analogWriteVoltage(trig_threshold[ipsd], ipsd);
        sprintf(msg, "    Writing default threshold to DAC%i: %f \(%i counts\)", ipsd, trig_threshold[ipsd], dac_counts);
        Serial.println(msg);
    }


    Serial.println ("Measuring Pedestals:");
    printDualPSDMeasurement(measurePedestal());
    Serial.println ("Measuring Peak Voltages:");
    printDualPSDMeasurement(measurePeak());

    // Serial.println ("d to toggle debug mode");
    // Serial.println ("s to toggle stddev output");
    // Serial.println ("c to recalibrate thresholds");
    // Serial.println ("r to toggle dynamic recalibration");

}
