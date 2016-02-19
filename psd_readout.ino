#include <SPI.h>
#include "dac_write.h"
#include "position_measurement.h"

static const int    LED     [2]    = { 24, 25 };
static const int    PSD_PIN [2][4] = {{ 0,  1,  2,  3}, { 4,  5,  6,  7}};
static const int    TRIG    [2][4] = {{41, 50, 34, 39}, {36, 37, 38, 40}}; 
static const int    TRIGf   [8]    = { 41, 50, 34, 39 ,  36, 37, 38, 40 }; 

                                     //A1, A2, A3, A4, B1, B2, B3, B4
static const int    TRIG_ENABLE [8] = {1,  1,  1,  1,  1,  1,  1,  1}; 

static const int    NSAMPLES = 500;         // Total number of samples to accumulate before printing measurement 
static const int    NSAMPLES_PER_CYCLE = 5; // Number of samples to take in a duty cycle 

static const float  VREF = 2.5f;  // Analog Voltage Reference

char msg [110];
bool last_read_state;

volatile bool is_reading = false;
volatile bool enableA = false; 
volatile bool enableB = false; 
volatile char trig_source = 0;

void readSensor(int ipsd, positionMeasurement &data);

int count_high [2] = {0, 0};
int count_low  [2] = {0, 0};

positionMeasurement sum_high[2];
positionMeasurement sum_low [2];

bool  pinstate = 0;
bool debug = 0;
int threshold = 820; 

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
	    /* Verbose Output */
	    sprintf(msg, "0: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f temp:%4.1f", voltage(debug_data[0].x1), voltage(debug_data[0].x2), voltage(debug_data[0].y1), voltage(debug_data[0].y2), readTemperature());
	    Serial.println(msg);
	    sprintf(msg, "1: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f temp:%4.1f", voltage(debug_data[1].x1), voltage(debug_data[1].x2), voltage(debug_data[1].y1), voltage(debug_data[1].y2), readTemperature());
	    Serial.println(msg);
    }
  }

  if (Serial.available() > 0) {
      char byteIn = Serial.read(); 
      if      (byteIn == 'd') debug = !debug; 
  }
  //else if (is_reading) {
  else {
    beat=0;
    is_reading = false;
    // wait for signals to rise
    delayMicroseconds(300);
    
    positionMeasurement data [2];
    positionMeasurement sum  [2];
    memset (data, 0, sizeof(data[0]) * 2);
    memset (sum,  0, sizeof(sum [0]) * 2);

    /* Take some number of samples in an individual cycle */
    for (int i=0; i<(2*NSAMPLES_PER_CYCLE); i++) {
      int ipsd = i % 2; // read PSD 0 on even, PSD1 on odd measurements
      readSensor(ipsd, data[ipsd]);
      sum[ipsd] = sum[ipsd] + data[ipsd];
    }

    bool state=0; 
    for (int i=0; i<8; i++) {
     state |= digitalRead(TRIGf[i]);
    }
   
    for (int ipsd = 0; ipsd < 2; ipsd++) {
      data[ipsd] = sum[ipsd] / NSAMPLES_PER_CYCLE;

      bool state=0; 
      for (int i=0; i<2; i++) {
        state |= data[ipsd].x1 > threshold; 
        state |= data[ipsd].x2 > threshold;
        state |= data[ipsd].y1 > threshold; 
        state |= data[ipsd].y2 > threshold; 
      }
    }
    
    //Check that Currently Reading State is opposite of Last Read State (HIGH vs. LOW)
    if (state == last_read_state) { is_reading=false; return; }
    else                          { last_read_state = state;}
    
    for (int ipsd = 0; ipsd < 2; ipsd++) {
      // Accumulate values for high state
      if (state == HIGH && count_high[ipsd] < NSAMPLES) {
        count_high[ipsd] += 1;
        sum_high[ipsd] = sum_high[ipsd] + data[ipsd];
      }

      // Accumulate values for low state
      else if (state == LOW && count_low[ipsd] < NSAMPLES) {
        count_low[ipsd] += 1;
        sum_low[ipsd]    = sum_low[ipsd] + data[ipsd];
      }
    }

    bool finished_measuring = false;
    for (int ipsd = 0; ipsd < 2; ipsd++) {
      finished_measuring |= ((count_low[ipsd] == NSAMPLES) && (count_high[ipsd] == NSAMPLES));
    }

    if (finished_measuring) {
      // Take average of last NSAMPLES values
      positionMeasurement difference [2] ;

      if (debug) {
          for (int ipsd = 0; ipsd < 2; ipsd++) {
          sum_high[ipsd] = sum_high[ipsd]/NSAMPLES; 
          sum_low [ipsd] = sum_low [ipsd]/NSAMPLES; 
          
          difference [ipsd] = (sum_high[ipsd] - sum_low[ipsd]); 
        
  
         
          /* Verbose Output */
          sprintf(msg, "%1i high: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f temp:%4.1f", ipsd, voltage(sum_high[ipsd].x1), voltage(sum_high[ipsd].x2), voltage(sum_high[ipsd].y1), voltage(sum_high[ipsd].y2), readTemperature());
          Serial.println(msg);
          sprintf(msg, "%1i  low: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f temp:%4.1f", ipsd, voltage(sum_low[ipsd].x1), voltage(sum_low[ipsd].x2), voltage(sum_low[ipsd].y1), voltage(sum_low[ipsd].y2), readTemperature());
          Serial.println(msg);
  
          /* Calculated Output */
          sprintf(msg, "%1i x: % 6.4f y: % 6.4f", ipsd, voltage(double(difference[ipsd].x2-difference[ipsd].x1))/voltage(double(difference[ipsd].x2+difference[ipsd].x1)), 
                                                   voltage(double(difference[ipsd].y2-difference[ipsd].y1))/voltage(double(difference[ipsd].y2+difference[ipsd].y1)));
          Serial.println(msg);
        
          } 
        }

        
        else {
          
          difference [0] = (sum_high[0] - sum_low[0]);
          difference [1] = (sum_high[1] - sum_low[1]);
          float xa = voltage(difference[0].x2-difference[0].x1)/voltage(difference[0].x2+difference[0].x1);
          float ya = voltage(difference[0].y2-difference[0].y1)/voltage(difference[0].y2+difference[0].y1);
          float xb = voltage(difference[1].x2-difference[1].x1)/voltage(difference[1].x2+difference[1].x1);
          float yb = voltage(difference[1].y2-difference[1].y1)/voltage(difference[1].y2+difference[1].y1);

          // Handle Cases of NAN
          if (xa != xa) xa = 999.;
          if (ya != ya) ya = 999.;
          if (xb != xb) xb = 999.;
          if (yb != yb) yb = 999.; 

          // Handle Cases Where Only One of the PSDs was triggered
          if (!enableA) {
            xa = 999.;
            ya = 999.;}
          if (!enableB) {
            xb = 999.;
            yb = 999.;
          }
        
          sprintf(msg, "% 9.5f % 9.5f % 9.5f % 9.5f % 5.2f", xa,ya,xb,yb, readTemperature());
          Serial.println(msg);
      }

      /* reset */
      memset (count_low,  0, sizeof(count_low [0]) * 2);
      memset (count_high, 0, sizeof(count_high[0]) * 2);
      memset (sum_high,   0, sizeof(sum_high  [0]) * 2);
      memset (sum_low,    0, sizeof(sum_low   [0]) * 2);
      
      enableA=false; 
      enableB=false; 
    }
  }
}

float voltage (float dac_counts) {
    return ((VREF*dac_counts)/(2*4095));
 }

void readSensor(int ipsd, positionMeasurement &data)
{
  data.x1 = analogRead(PSD_PIN[ipsd][0]);
  data.x2 = analogRead(PSD_PIN[ipsd][1]);
  data.y1 = analogRead(PSD_PIN[ipsd][2]);
  data.y2 = analogRead(PSD_PIN[ipsd][3]);
}

/* Pin Change Interrupt Routines */
void interruptA1() { if (TRIG_ENABLE[0]) {trig_source = 0; enableA=true; interrupt();}}
void interruptA2() { if (TRIG_ENABLE[1]) {trig_source = 1; enableA=true; interrupt();}}
void interruptA3() { if (TRIG_ENABLE[2]) {trig_source = 2; enableA=true; interrupt();}}
void interruptA4() { if (TRIG_ENABLE[3]) {trig_source = 3; enableA=true; interrupt();}}
void interruptB1() { if (TRIG_ENABLE[4]) {trig_source = 4; enableB=true; interrupt();}}
void interruptB2() { if (TRIG_ENABLE[5]) {trig_source = 5; enableB=true; interrupt();}}
void interruptB3() { if (TRIG_ENABLE[6]) {trig_source = 6; enableB=true; interrupt();}}
void interruptB4() { if (TRIG_ENABLE[7]) {trig_source = 7; enableB=true; interrupt();}}

void interrupt()   { if (!is_reading)    {is_reading = true;}};

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

void configureBoard () {
    analogReadResolution(12);
    analogWriteResolution(12);
    float VTHRESH = 0.5;  
    analogWrite(DAC0, VTHRESH/VREF * ((1<<12)-1)); 
    analogWrite(DAC1, VTHRESH/VREF * ((1<<12)-1)); 

    /* Set OAVCC by DAC; Currently, OAVCC is hardwired. Programmable voltage 
     * requires hardware modification! 
     * setDAC(0, 0.5/VREF * ((1<<14)-1) );  
     * setDAC(1, 0.5/VREF * ((1<<14)-1) );
     */

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

