#include <SPI.h>
#include "dac_write.h"
#include "position_measurement.h"

//static const int SERIAL_NO = 0; 
static const int SERIAL_NO = 1; 

static const int    LED     [2]    = { 24, 25 };
static const int    PSD_PIN [2][4] = {{ 0,  1,  2,  3}, { 4,  5,  6,  7}};
static const int    TRIG    [2][4] = {{41, 50, 34, 39}, {36, 37, 38, 40}}; 
static const int    TRIGf   [8]    = { 41, 50, 34, 39 ,  36, 37, 38, 40 }; 

static const int    GAIN[2][2]         = {{2,2},{1, 4}};  // per channel gain of each PSD
static const float  DAC_ATTENUATION[2] = {1.0,0.325}; // attenuation value applied to the threshold comparator DACs

                                     //A1, A2, A3, A4, B1, B2, B3, B4
static const int    TRIG_ENABLE [8] = {1,  1,  1,  1,  1,  1,  1,  1}; 

static const int    NSAMPLES = 500;         // Total number of samples to accumulate before printing measurement 
static const int    NSAMPLES_PER_CYCLE = 3; // Number of samples to take in a duty cycle 

static const float  VREF = 2.5f;  // Analog Voltage Reference


char msg [110];
bool last_read_state;

volatile bool is_reading = false;
volatile bool enableA = false; 
volatile bool enableB = false; 
volatile char trig_source = 0;

void readSensor(int ipsd, positionMeasurement &data);


bool  pinstate = 0;
bool debug = 0;

positionMeasurement voltage_high;
positionMeasurement voltage_low ;


void setup()
{
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

positionMeasurement sum_low [2];
positionMeasurement sum_high [2];

positionMeasurement sum_sq_high[2];
positionMeasurement sum_sq_low [2];

int beat=0; 
void loop()
{
  if (debug) {
    beat++; 
    if (beat%1000000==0) {
	    is_reading = 0; 
	    Serial.println("heartbeat..."); 

	    positionMeasurement debug_data [2];

	    readSensor(0, debug_data[0]); 
	    readSensor(1, debug_data[1]); 

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
  }

  if (Serial.available() > 0) {
      char byteIn = Serial.read(); 
      if      (byteIn == 'd') debug = !debug; 
  }
  else if (is_reading) {
    beat=0;
    is_reading = false;
    
    positionMeasurement data [2];
    positionMeasurement sum  [2];

    memset (data, 0, sizeof(data[0]) * 2);
    memset (sum,  0, sizeof(sum [0]) * 2);

    // wait for signals to rise
    delayMicroseconds(200);

    /* Take some number of samples in an individual cycle */
    for (int i=0; i<(2*NSAMPLES_PER_CYCLE); i++) {
      int ipsd = i % 2; // read PSD 0 on even, PSD1 on odd measurements
      readSensor(ipsd, data[ipsd]);
      sum[ipsd] = sum[ipsd] + data[ipsd];
    }

    float analog_threshold = 0.3; 
    bool state = 0; 
    for (int ipsd = 0; ipsd < 2; ipsd++) {
      for (int i=0; i<2; i++) {
        state |= voltage(data[ipsd].x1, ipsd) > analog_threshold; 
        state |= voltage(data[ipsd].x2, ipsd) > analog_threshold;
        state |= voltage(data[ipsd].y1, ipsd) > analog_threshold; 
        state |= voltage(data[ipsd].y2, ipsd) > analog_threshold; 
      }
    }

    // Check that Currently Reading State is opposite of Last Read State (HIGH vs. LOW)
    if (state == last_read_state) { return; }
    else                          { last_read_state = state;}



    for (int ipsd = 0; ipsd < 2; ipsd++) {
      // Accumulate values for high state
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

      // Accumulate values for low state
      else if (state == LOW && count_low[ipsd] < NSAMPLES) {
        count_low[ipsd] += 1;
        sum_low   [ipsd] = sum_low   [ipsd] + data[ipsd];
        sum_sq_low[ipsd] = sum_sq_low[ipsd] + data[ipsd]*data[ipsd];
      }
    }

    bool finished_measuring = false;
    for (int ipsd = 0; ipsd < 2; ipsd++) {
      finished_measuring |= ((count_low[ipsd] == NSAMPLES) && (count_high[ipsd] == NSAMPLES));
    }

    if (finished_measuring) {
      positionMeasurement difference [2] ;
      positionMeasurement mean_high[2];
      positionMeasurement mean_low[2];
      positionMeasurement var_high[2];
      positionMeasurement var_low[2];
      positionMeasurement var[2];
      positionMeasurement stddev[2];


      double diffx1_0 = x1_max[0] - x1_min[0]; 
      double diffx2_0 = x2_max[0] - x2_min[0]; 
      double diffy1_0 = y1_max[0] - y1_min[0]; 
      double diffy2_0 = y2_max[0] - y2_min[0]; 

      double diffx1_1 = x1_max[1] - x1_min[1]; 
      double diffx2_1 = x2_max[1] - x2_min[1]; 
      double diffy1_1 = y1_max[1] - y1_min[1]; 
      double diffy2_1 = y2_max[1] - y2_min[1]; 

      double diffmax = 0; 
      diffmax = max(diffx1_0,diffmax); 
      diffmax = max(diffx2_0,diffmax); 
      diffmax = max(diffy1_0,diffmax); 
      diffmax = max(diffy2_0,diffmax); 
      diffmax = max(diffx1_1,diffmax); 
      diffmax = max(diffx2_1,diffmax); 
      diffmax = max(diffy1_1,diffmax); 
      diffmax = max(diffy2_1,diffmax); 

      double diffmax_limit = 100; 

      if (diffmax < diffmax_limit)  {
          if (debug) {
              for (int ipsd = 0; ipsd < 2; ipsd++) {
                  sprintf(msg, "%f %f %f %f", x1_max[ipsd], x2_max[ipsd], y1_max[ipsd], y2_max[ipsd]); 
                  Serial.println(msg);
                  sprintf(msg, "%f %f %f %f", x1_min[ipsd], x2_min[ipsd], y1_min[ipsd], y2_min[ipsd]); 
                  Serial.println(msg);

                  // Take average of last NSAMPLES values
                  mean_high[ipsd] = sum_high[ipsd]/NSAMPLES; 
                  mean_low [ipsd] = sum_low [ipsd]/NSAMPLES; 
                  
                  difference [ipsd] = (mean_high[ipsd] - mean_low[ipsd]); 

                  var_high[ipsd] = (sum_sq_high[ipsd]/NSAMPLES-mean_high[ipsd]*mean_high[ipsd]); 
                  var_low[ipsd]  = ( sum_sq_low[ipsd]/NSAMPLES-mean_low [ipsd]*mean_low [ipsd]);

                  var[ipsd] = var_high[ipsd] + var_low[ipsd]; 

                  stddev[ipsd].x1 = sqrt(var[ipsd].x1); 
                  stddev[ipsd].x2 = sqrt(var[ipsd].x2); 
                  stddev[ipsd].y1 = sqrt(var[ipsd].y1); 
                  stddev[ipsd].y2 = sqrt(var[ipsd].y2); 
             
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
                    ((difference[ipsd].x2-difference[ipsd].x1))/((difference[ipsd].x2+difference[ipsd].x1)), 
                    ((difference[ipsd].y2-difference[ipsd].y1))/((difference[ipsd].y2+difference[ipsd].y1)));
                  Serial.println(msg);
              } 
            }

            else {
              
              difference [0] = (sum_high[0] - sum_low[0]);
              difference [1] = (sum_high[1] - sum_low[1]);

              float xa = (difference[0].x2-difference[0].x1)/(difference[0].x2+difference[0].x1);
              float ya = (difference[0].y2-difference[0].y1)/(difference[0].y2+difference[0].y1);
              float xb = (difference[1].x2-difference[1].x1)/(difference[1].x2+difference[1].x1);
              float yb = (difference[1].y2-difference[1].y1)/(difference[1].y2+difference[1].y1);

              float min_diff = 0.10; 
              if (    voltage(difference[0].x1, 0) < min_diff 
                   || voltage(difference[0].x2, 0) < min_diff
                   || voltage(difference[0].y1, 0) < min_diff
                   || voltage(difference[0].y2, 0) < min_diff )  {
            	xa = 999.; 
            	ya = 999.; 
              }

              if (    voltage(difference[1].x1, 1) < min_diff 
                   || voltage(difference[1].x2, 1) < min_diff
                   || voltage(difference[1].y1, 1) < min_diff
                   || voltage(difference[1].y2, 1) < min_diff )  {
            	xb = 999.; 
            	yb = 999.; 
              }

              // Handle Cases of NAN
              if (xa != xa) xa = 999.;
              if (ya != ya) ya = 999.;
              if (xb != xb) xb = 999.;
              if (yb != yb) yb = 999.; 

              sprintf(msg, "% 9.5f % 9.5f % 9.5f % 9.5f % 5.2f", xa,ya,xb,yb, readTemperature());
              Serial.println(msg);
          }
      }

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
    return ((VREF*adc_counts)/(GAIN[SERIAL_NO][ipsd]*4095)); // factor of 2 is because the amplifier has gain of 2 !!
 }

void readSensor(int ipsd, positionMeasurement &data)
{
  data.x1 = analogRead(PSD_PIN[ipsd][0]);
  data.x2 = analogRead(PSD_PIN[ipsd][1]);
  data.y1 = analogRead(PSD_PIN[ipsd][2]);
  data.y2 = analogRead(PSD_PIN[ipsd][3]);
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

void interrupt()   { if (!is_reading) {is_reading = true;}};

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

void analogWriteVoltage (float voltage, int ipsd) { 
    float vref = VREF;  // 2.5V
    float attenuation = DAC_ATTENUATION[SERIAL_NO]; 

    // the atsam3x8e processor has a DAC output voltage range that varies between 1/6 and 5/6 of VREF; 
    // on one PSD, at least, that voltage is decreased by an additional factor of ~1/3 (or more closely 0.325)
    int dac_counts = (voltage-1/6*vref*attenuation)*((1<<12)-1)/(4/6*vref*attenuation); 

    uint32_t dac_pin [2] = {DAC0, DAC1}; 
    analogWrite (dac_pin[ipsd], dac_counts); 
}

void configureBoard () {

    analogReadResolution(12);
    analogWriteResolution(12);

    // Set Threshold Voltages
    analogWriteVoltage(0.25, 0); 
    analogWriteVoltage(0.25, 1); 

    // Configure Trigger Interrupt
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


  // Configure DAC Chip Select
  pinMode(43, OUTPUT);
  digitalWrite(43, HIGH);

  /* Turn off LED */
  pinMode(LED[0], OUTPUT);
  digitalWrite(LED[0], LOW);

  pinMode(LED[1], OUTPUT);
  digitalWrite(LED[1], LOW);
}
