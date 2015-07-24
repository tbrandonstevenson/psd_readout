#include <SPI.h>
#include "dac_write.h"
#include "position_measurement.h"

#define LED1        24U
#define LED2        25U

#define TRIG_A1     41U
#define TRIG_A2     50U
#define TRIG_A3     34U
#define TRIG_A4     39U

#define TRIG_B1     36U
#define TRIG_B2     37U
#define TRIG_B3     38U
#define TRIG_B4     40U

static const int TRIG [8] = {TRIG_A1, TRIG_A2, TRIG_A3, TRIG_A4, TRIG_B1, TRIG_B2, TRIG_B3, TRIG_B4};
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
  analogWriteResolution(12);
  configureDACs();

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

    /* Take some number of samples in an individual cycle */
    for (int i=0; i<(2*NSAMPLES_PER_CYCLE); i++) {
      int ipsd = i % 2; // read PSD 0 on even, PSD1 on odd measurements
      readSensor(ipsd, data[ipsd]);

      sum[ipsd] = sum[ipsd] + data[ipsd];
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

        //sprintf(msg, "%1i: % 6.4f % 6.4f", ipsd, voltage(difference.x()), voltage(difference.y());
        //Serial.println(msg);

        //sprintf(msg, "%1i high: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f", ipsd, voltage(sum_high[ipsd].x1), voltage(sum_high[ipsd].x2), voltage(sum_high[ipsd].y1), voltage(sum_high[ipsd].y2));
        //Serial.println(msg);
        //sprintf(msg, "%1i  low: x1:%6.4f x2:%6.4f y1:%6.4f y2:%6.4f", ipsd, voltage(sum_low[ipsd].x1), voltage(sum_low[ipsd].x2), voltage(sum_low[ipsd].y1), voltage(sum_low[ipsd].y2));
        //Serial.println(msg);
      }

      /* reset */
      memset (count_low,  0, sizeof(count_low [0]) * 2);
      memset (count_high, 0, sizeof(count_high[0]) * 2);
      memset (sum_high,   0, sizeof(sum_high  [0]) * 2);
      memset (sum_low,    0, sizeof(sum_low   [0]) * 2);
    }
  }
}

static const int PSD_PIN [2][4] = {
  {0, 1, 2, 3},
  {4, 5, 6, 7}
};

double voltage (int dac_counts) {
    return (dac_counts * (3.3/4095)/NSAMPLES); 
}

void readSensor(int ipsd, positionMeasurement &data)
{
  data.x1     = analogRead(PSD_PIN[ipsd][0]);
  data.x2     = analogRead(PSD_PIN[ipsd][1]);
  data.y1     = analogRead(PSD_PIN[ipsd][2]);
  data.y2     = analogRead(PSD_PIN[ipsd][3]);
}

/* Pin Change Interrupt Routines */
void interrupt0() { if (!is_reading) { trig_source = 0; interrupt(); }}
void interrupt1() { if (!is_reading) { trig_source = 1; interrupt(); }}
void interrupt2() { if (!is_reading) { trig_source = 2; interrupt(); }}
void interrupt3() { if (!is_reading) { trig_source = 3; interrupt(); }}
void interrupt4() { if (!is_reading) { trig_source = 4; interrupt(); }}
void interrupt5() { if (!is_reading) { trig_source = 5; interrupt(); }}
void interrupt6() { if (!is_reading) { trig_source = 6; interrupt(); }}
void interrupt7() { if (!is_reading) { trig_source = 7; interrupt(); }}
void interrupt()  { if (!is_reading) { is_reading = true; }};

float readTemperature() {
    int temp = analogRead(9); 
    float voltage = temp / 3.3 * (4095); // 3.3V ref, 12 bits
    float temperature = (voltage-0.424)/0.00625;  // conferre the LM60 data sheet
    return temperature; 
}

void configureDACs ()
{
  for (int i=0; i<2; i++) {
    setDAC(i, 7780);  // 4.867V. Output voltage = 3.3+3.3*(7780/16383)
    analogWrite(i, 612);     // Set Comparator Threshold for 12-bits at 3.3VREF
  }
}

void configurePinModes () {
  /* Configure Trigger Interrupts */
  for (int i=0; i<8; i++) {
      pinMode(TRIG[i], INPUT);                    
      digitalWrite(TRIG[0], HIGH);
  }

  attachInterrupt(TRIG[0], interrupt0, CHANGE);
  attachInterrupt(TRIG[1], interrupt1, CHANGE); 
  attachInterrupt(TRIG[2], interrupt2, CHANGE); 
  attachInterrupt(TRIG[3], interrupt3, CHANGE); 
  attachInterrupt(TRIG[4], interrupt4, CHANGE); 
  attachInterrupt(TRIG[5], interrupt5, CHANGE);
  attachInterrupt(TRIG[6], interrupt6, CHANGE); 
  attachInterrupt(TRIG[7], interrupt7, CHANGE); 

  /* DAC Chip Select */
  pinMode(DAC_CS, OUTPUT);
  digitalWrite(DAC_CS, HIGH);

  /* LEDs */
  pinMode(LED1, OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
}
