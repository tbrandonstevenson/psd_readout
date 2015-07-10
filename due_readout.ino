#include <SPI.h>
#include "dac_write.h"
#include "position_measurement.h"

#define LED1        24U
#define LED2        25U
#define TRIG1       26U
#define TRIG2       27U
#define OAVCC   0
#define VTHRESH 1

static const int TRIG [2] = {TRIG1, TRIG2};
static const int LED  [2] = {LED1,  LED2 };


char msg [110];
int NSAMPLES = 500;  // Total number of samples to accumulate before printing measurement 
int NSAMPLES_PER_CYCLE = 2; // Number of samples to take in a duty cycle 
bool last_read_state;

volatile bool is_reading = false;
volatile char trig_source = 0;

void readSensor(int ipsd, positionMeasurement &data);

int count_high[2] = {0, 0};
int count_low [2] = {0, 0};

positionMeasurement sum_high[2];
positionMeasurement sum_low [2];
bool  pinstate = 0;

positionMeasurement voltage_high;
positionMeasurement voltage_low ;

void setup()
{
  // Setup Serial Bus
  Serial.begin(115200);

  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(4);

  Serial.println("configuring board...");

  configurePinModes();
  analogReadResolution(12);
  initialize_board();

  Serial.println("board configured...");
}

void loop()
{
  if (is_reading) {
    is_reading = false;
    // wait for signals to rise
    delayMicroseconds(400);

    //Check that Currently Reading State is opposite of Last Read State (HIGH vs. LOW)
    bool state = digitalRead(TRIG[trig_source]);


    if (state == last_read_state) { return; }

    last_read_state = state;
    digitalWrite(LED[0], state);
    digitalWrite(LED[1], state);

    positionMeasurement data [2];
    positionMeasurement sum  [2];
    memset (data, 0, sizeof(data[0]) * 2);
    memset (sum,  0, sizeof(sum [0]) * 2);

    // Take some number of samples in an individual cycle
    for (int i = 0; i < 2 * NSAMPLES_PER_CYCLE; i++) {
      // read PSD 0 on even measurements, and PSD1 on odd measurements
      int ipsd = i % 2;
      readSensor(ipsd, data[ipsd]);

      sum[ipsd] = sum[ipsd] + data[ipsd];
//      Serial.println(data[ipsd].x1+sum[ipsd].x1); 
//      Serial.println(data[ipsd].x2)+sum[ipsd].y2;
//      Serial.println(data[ipsd].y1+sum[ipsd].y1); 
//      Serial.println(data[ipsd].x1+sum[ipsd].y2); 
    }

    for (int ipsd = 0; ipsd < 2; ipsd++) {
      data[ipsd] = sum[ipsd] / NSAMPLES_PER_CYCLE;

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
      positionMeasurement difference;

      for (int ipsd = 0; ipsd < 2; ipsd++) {
        difference = (sum_high[ipsd] - sum_low[ipsd]); 
      
        sprintf(msg, "%1i: % 6.4f % 6.4f", ipsd, voltage(double(difference.x2-difference.x1))/voltage(double(difference.x2+difference.x1)), voltage(double(difference.y2-difference.y1))/voltage(double(difference.y2+difference.y1)));
        Serial.println(msg);

        //sprintf(msg, "%1i high: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f", ipsd, voltage(sum_high[ipsd].x1), voltage(sum_high[ipsd].x2), voltage(sum_high[ipsd].y1), voltage(sum_high[ipsd].y2));
        //Serial.println(msg);
        //sprintf(msg, "%1i  low: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f", ipsd, voltage(sum_low[ipsd].x1), voltage(sum_low[ipsd].x2), voltage(sum_low[ipsd].y1), voltage(sum_low[ipsd].y2));
        //Serial.println(msg);
      }

      // reset
      memset (count_low,  0, sizeof(count_low[0])  * 2);
      memset (count_high, 0, sizeof(count_high[0]) * 2);
      memset (sum_high,   0, sizeof(sum_high[0])   * 2);
      memset (sum_low,    0, sizeof(sum_low [0])   * 2);
    }
  }
}

static const int PSD_PIN [2][4] = {
  {0, 1, 2, 3},
  {4, 5, 6, 7}
};

double voltage (int dac_counts) {
    return (dac_counts * (3.3/4096)/NSAMPLES); 
}

/* Routine to take a reading from a single sensor and transmit results through TCP/IP */
void readSensor(int ipsd, positionMeasurement &data)
{
  data.x1     = analogRead(PSD_PIN[ipsd][0]);
  data.x2     = analogRead(PSD_PIN[ipsd][1]);
  data.y1     = analogRead(PSD_PIN[ipsd][2]);
  data.y2     = analogRead(PSD_PIN[ipsd][3]);
  //Serial.println(data.y1);
}

/* Pin Change Interrupt Routines */

void interrupt1()
{
  trig_source = 0;
  interrupt();
}

void interrupt2()
{
  trig_source = 1;
  interrupt();
}

void interrupt()
{
  if (!is_reading) {
    is_reading = true;
  }
};

void initialize_board ()
{
  for (int i = 0; i < 2; i++) {
    setDAC(i, OAVCC,   16011); // 16011 Set OAVCC to 4.867V for 14-bits 16383 at 5.0V Vref
    setDAC(i, VTHRESH, 1638);  // Set Comparator Threshold for 14-bits 16383 at 5.0V Vref
  }
}

void configurePinModes () {
  // Configure Trigger Interrupt
  pinMode(TRIG1, INPUT);                    // configure as input
  pinMode(TRIG2, INPUT);                    // configure as input
  attachInterrupt(TRIG1, interrupt1, CHANGE); // enable interrupts on this pin
  attachInterrupt(TRIG2, interrupt2, CHANGE); // enable interrupts on this pin
  digitalWrite(TRIG1, LOW);                  // turn on a pulldown resistor
  digitalWrite(TRIG2, LOW);                  // turn on a pulldown resistor

  // Configure DAC Chip Select
  pinMode(DAC_CS1, OUTPUT);
  digitalWrite(DAC_CS1, HIGH);

  // Configure DAC Chip Select
  pinMode(DAC_CS2, OUTPUT);
  digitalWrite(DAC_CS2, HIGH);

  /* Turn off LED */
  pinMode(LED1, OUTPUT);
  digitalWrite(LED2, LOW);

  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
}
