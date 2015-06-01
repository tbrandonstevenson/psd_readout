#include <SPI.h>
#include "dac_write.h"

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
int NSAMPLES_PER_CYCLE = 5; // Number of samples to take in a duty cycle 
bool last_read_state;

volatile bool is_reading = false; 
volatile char trig_source; 

template <typename T>
struct position_measurement {
    T x1=0; 
    T x2=0; 
    T y1=0; 
    T y2=0;
};

void readSensor(int ipsd, position_measurement<short> &data); 

int count_high = 0;
int count_low = 0;

position_measurement<long> sum_high[2]; 
position_measurement<long> sum_low [2]; 

void initializeSerial () {
    // Setup Serial Bus
    Serial.begin(115200);

    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(4);

}

void setup()
{
    initializeSerial (); 
    configurePinModes(); 
    analogReadResolution(12);
    initialize_board();
}

void loop()
{
    if (is_reading) {
        // wait for signals to rise
        delayMicroseconds(400);

        //Check that Currently Reading State is opposite of Last Read State (HIGH vs. LOW)
        bool state = digitalRead(TRIG[trig_source]);

        digitalWrite(LED[0],state);
        digitalWrite(LED[1],state);

        if (state==last_read_state) {
            return;
        }

        else {
            last_read_state = state;
        }

        position_measurement<short>    data [2]; 
        position_measurement<long>     sum  [2]; 

        // Take some number of samples in an individual cycle
        for (int i = 0; i < 2*NSAMPLES_PER_CYCLE; i++) {
            // read PSD 0 on even measurements, and PSD1 on odd measurements
            int ipsd = i%2; 
            readSensor(ipsd, data[ipsd]); 
            sum[ipsd].x1     += data[ipsd].x1;
            sum[ipsd].x2     += data[ipsd].x2;
            sum[ipsd].y1     += data[ipsd].y1;
            sum[ipsd].y2     += data[ipsd].y2;
        }

        for (int ipsd=0; ipsd<2; ipsd++) {
            data[ipsd].x1     = sum[ipsd].x1 / NSAMPLES_PER_CYCLE;
            data[ipsd].x2     = sum[ipsd].x2 / NSAMPLES_PER_CYCLE;
            data[ipsd].y1     = sum[ipsd].y1 / NSAMPLES_PER_CYCLE;
            data[ipsd].y2     = sum[ipsd].y2 / NSAMPLES_PER_CYCLE;

            // Accumulate values for high state
            if (state==HIGH && count_high < NSAMPLES) {
                count_high += 1;

                sum_high[ipsd].x1     += data[ipsd].x1;
                sum_high[ipsd].x2     += data[ipsd].x2;
                sum_high[ipsd].y1     += data[ipsd].y1;
                sum_high[ipsd].y2     += data[ipsd].y2;
            }

            // Accumulate values for low state
            else if (state==LOW && count_low < NSAMPLES) {
                count_low += 1;

                sum_low[ipsd].x1 += data[ipsd].x1;
                sum_low[ipsd].x2 += data[ipsd].x2;
                sum_low[ipsd].y1 += data[ipsd].y1;
                sum_low[ipsd].y2 += data[ipsd].y2;
            }
        }

        double x[2] = {0}; 
        double y[2] = {0};

        if ((count_low == NSAMPLES) && (count_high == NSAMPLES)) {

            // Take average of last NSAMPLES values
            position_measurement<double> corrected[2]; 
            double x[2]; 
            double y[2]; 

            for (int ipsd=0; ipsd<2; ipsd++) {
                corrected[ipsd].x1     = (sum_high[ipsd].x1     - sum_low[ipsd].x1    ) / NSAMPLES;
                corrected[ipsd].x2     = (sum_high[ipsd].x2     - sum_low[ipsd].x2    ) / NSAMPLES;
                corrected[ipsd].y1     = (sum_high[ipsd].y1     - sum_low[ipsd].y1    ) / NSAMPLES;
                corrected[ipsd].y2     = (sum_high[ipsd].y2     - sum_low[ipsd].y2    ) / NSAMPLES;

                x[ipsd] = (corrected[ipsd].x2-corrected[ipsd].x1)/(corrected[ipsd].x2+corrected[ipsd].x1);
                y[ipsd] = (corrected[ipsd].y2-corrected[ipsd].y1)/(corrected[ipsd].y2+corrected[ipsd].y1);

                sprintf(msg, "%1i: % 8.6f % 8.6f", ipsd, x[ipsd], y[ipsd]);
                Serial.println(msg);
            }

            // reset 
            count_low = 0;
            count_high = 0;
            memset (sum_high, 0, sizeof(sum_high[0])*2); 
            memset (sum_low,  0, sizeof(sum_low [0])*2); 
        }
    is_reading = false; 
    }
}

static const int PSD_PIN [2][4] = {
    {0,1,2,3}, 
    {4,5,6,7}
};

/* Routine to take a reading from a single sensor and transmit results through TCP/IP */
void readSensor(int ipsd, position_measurement<short> &data)
{
    data.x1     = analogRead(PSD_PIN[ipsd][0]);
    data.x2     = analogRead(PSD_PIN[ipsd][1]);
    data.y1     = analogRead(PSD_PIN[ipsd][2]);
    data.y2     = analogRead(PSD_PIN[ipsd][3]);
}

/* Pin Change Interrupt Routines */

void interrupt1()
{
    trig_source=0; 
    interrupt(); 
}

void interrupt2()
{
    trig_source=1; 
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
    delay(2000); 
    for (int i=0; i<2; i++) {
        setDAC(i, OAVCC,   16011); // Set OAVCC to 4.867V for 14-bits 16383 at 5.0V Vref
        setDAC(i, VTHRESH, 1638);  // Set Comparator Threshold for 14-bits 16383 at 5.0V Vref
    }
}

void configurePinModes () {
    // Configure Trigger Interrupt
    pinMode(TRIG1, INPUT);                    // configure as input
    pinMode(TRIG2, INPUT);                    // configure as input
    attachInterrupt(TRIG1, interrupt, CHANGE); // enable interrupts on this pin
    attachInterrupt(TRIG2, interrupt, CHANGE); // enable interrupts on this pin
    digitalWrite(TRIG1, LOW);                  // turn on a pulldown resistor
    digitalWrite(TRIG2, LOW);                  // turn on a pulldown resistor

    // Configure DAC Chip Select
    pinMode(DAC_CS1, OUTPUT);
    digitalWrite(DAC_CS1,HIGH);

    // Configure DAC Chip Select
    pinMode(DAC_CS2, OUTPUT);
    digitalWrite(DAC_CS2,HIGH);

    /* Turn off LED */
    pinMode(LED1, OUTPUT);
    digitalWrite(LED2,LOW);

    pinMode(LED2, OUTPUT);
    digitalWrite(LED2,LOW);
}
