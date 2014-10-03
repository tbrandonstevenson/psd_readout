#include <SPI.h>
#include "spi_peripherals.h"

#define LED         22U
#define TRIG        50U
#define CPU_CLOCK   51U

// DAC Channels     
#define OAVCC        0U
#define VR3          1U
#define VTHRESH      1U

// SPI CS Channels
#define DAC_CS      52U
#define ADC_CS       4U

#define SENSE_3V3   24U

#define NATIVEADC

//#define DECOUTPUT
//#define HEXOUTPUT
//#define CALIBRATED
//#define NATIVEUSB

char msg [110];
int nsamples = 500; 
int nsamples_per_cycle = 1;
uint8_t last_read_state; 

int count_high = 0; 
unsigned long     sum_x1_high=0; 
unsigned long     sum_x2_high=0; 
unsigned long     sum_y1_high=0; 
unsigned long     sum_y2_high=0; 
unsigned long sum_deltax_high=0; 
unsigned long sum_deltay_high=0; 

int count_low = 0; 
unsigned long     sum_x1_low=0; 
unsigned long     sum_x2_low=0; 
unsigned long     sum_y1_low=0; 
unsigned long     sum_y2_low=0; 
unsigned long sum_deltax_low=0; 
unsigned long sum_deltay_low=0; 

void setup()
{
    // Setup Serial Bus
#ifdef NATIVEUSB
    SerialUSB.begin(115200);
#else
    Serial.begin(115200);
#endif

    SPI.begin(); 
    SPI.setDataMode(SPI_MODE1); 
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(4); 

    // Configure Trigger Interrupt
    pinMode(TRIG, INPUT);                     // configure as input
    attachInterrupt(TRIG, interrupt, CHANGE); // enable interrupts on this pin
    digitalWrite(TRIG, LOW);                  // turn on a pulldown resistor

    // Configure DAC Chip Select
    pinMode(DAC_CS, OUTPUT); 
    digitalWrite(DAC_CS,HIGH); 

    // Configure ADC Chip Select
    pinMode(ADC_CS, OUTPUT); 
    digitalWrite(ADC_CS,HIGH); 

    // Configure CPU Clock INput
    pinMode(CPU_CLOCK, INPUT); 

    // 3.3V SENSE Interrupt 
    // Loops for rising edge of a 3.3V line from the ADC Host Board
    // Ensures that ADC
    pinMode(SENSE_3V3, INPUT); 
    attachInterrupt(SENSE_3V3, initialize_board, RISING); // enable interrupts on this pin
    digitalWrite(TRIG, LOW);                  // turn on a pulldown resistor

    /* Turn off LED */
    pinMode(LED, OUTPUT); 
    digitalWrite(LED,LOW); 

    // Set Resolution for analog Read and Write for Arduino Built-in DAC/ADC
    analogReadResolution(12);

    initialize_board(); 
}

void loop()
{
}


/* Routine to take a reading from a single sensor and transmit results through TCP/IP */
int ReadSensor(uint16_t &x1, uint16_t &x2, uint16_t &y1, uint16_t &y2, uint16_t &deltax, uint16_t &deltay)
{
    int fail=0; 
#ifdef NATIVEADC
    // Read from Onboard ADC
    fail |= digitalRead(CPU_CLOCK); 
    deltay = analogRead(0x0);
    fail |= digitalRead(CPU_CLOCK); 
    deltax = analogRead(0x1);
    y2     = analogRead(0x2);
    fail |= digitalRead(CPU_CLOCK); 
    y1     = analogRead(0x3);
    fail |= digitalRead(CPU_CLOCK); 
    x2     = analogRead(0x4);
    fail |= digitalRead(CPU_CLOCK); 
    x1     = analogRead(0x5);
    fail |= digitalRead(CPU_CLOCK); 
#else
    // Read from External ADC
    fail |= digitalRead(CPU_CLOCK); 
    x1     = measureADC(0x0);
    fail |= digitalRead(CPU_CLOCK); 
    x2     = measureADC(0x1);
    fail |= digitalRead(CPU_CLOCK); 
    y1     = measureADC(0x2);
    fail |= digitalRead(CPU_CLOCK); 
    y2     = measureADC(0x3);
    fail |= digitalRead(CPU_CLOCK); 
    deltay = measureADC(0x4);
    fail |= digitalRead(CPU_CLOCK); 
    deltax = measureADC(0x5);
    fail |= digitalRead(CPU_CLOCK); 
#endif
    return (fail); 
}

/* Pin Change Interrupt Routines */
void interrupt() {
    int fail = 0; 

    //Check that Currently Reading State is opposite of Last Read State (HIGH vs. LOW)
    uint8_t state = digitalRead(TRIG); 
    if (state==last_read_state) {
        return; 
    }
    else {
        last_read_state = state; 
    }

    unsigned int then = micros(); 

    uint16_t x1, x2, y1, y2, deltax, deltay; 

    // Reset Sums
    long sum_x1 = 0;
    long sum_x2 = 0;
    long sum_y1 = 0;
    long sum_y2 = 0;
    long sum_deltax = 0;
    long sum_deltay = 0;

    // wait for signals to rise 
    delayMicroseconds(400);

    // some primitive debouncing
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = micros();
    if (interrupt_time - last_interrupt_time < 900)
        return; 
    last_interrupt_time = interrupt_time;  //the rest of the debounce code


    // Take some number of samples in an individual cycle
    for (int i = 0; i < nsamples_per_cycle; i++) {
        fail |= ReadSensor(x1, x2, y1, y2, deltax, deltay);
        sum_x1 += x1;
        sum_x2 += x2;
        sum_y1 += y1;
        sum_y2 += y2;
        sum_deltax += deltax;
        sum_deltay += deltay;
    }

    if (fail) {
//#ifdef NATIVEUSB
//        SerialUSB.println("reject"); 
//#else
//        Serial.println("reject"); 
//#endif
        return;
    }

    // Accumulate samples for individual sample
    x1     = sum_x1 / nsamples_per_cycle;
    x2     = sum_x2 / nsamples_per_cycle;
    y1     = sum_y1 / nsamples_per_cycle;
    y2     = sum_y2 / nsamples_per_cycle;
    deltax = sum_deltax / nsamples_per_cycle;
    deltay = sum_deltay / nsamples_per_cycle;



    // Accumulate values for high state
    if (state == 1 && count_high < nsamples) {
        count_high += 1; 
        sum_x1_high += x1; 
        sum_x2_high += x2; 
        sum_y1_high += y1; 
        sum_y2_high += y2; 
        sum_deltax_high += deltax; 
        sum_deltay_high += deltay; 
    }

    // Accumulate values for low state
    else if (state == 0 && count_low < nsamples) {
        count_low += 1; 
        sum_x1_low += x1; 
        sum_x2_low += x2; 
        sum_y1_low += y1; 
        sum_y2_low += y2; 
        sum_deltax_low += deltax; 
        sum_deltay_low += deltay; 
    }


    char str; 

    if ((count_low == nsamples) && (count_high == nsamples)) {
        // reset count
        count_low = 0; 
        // reset count
        count_high = 0; 

        // Take average of last n values
        double x1_avg     = (sum_x1_high     - sum_x1_low    ) / nsamples;
        double x2_avg     = (sum_x2_high     - sum_x2_low    ) / nsamples;
        double y1_avg     = (sum_y1_high     - sum_y1_low    ) / nsamples;
        double y2_avg     = (sum_y2_high     - sum_y2_low    ) / nsamples;
        double deltax_avg = (sum_deltax_high - sum_deltax_low) / nsamples;
        double deltay_avg = (sum_deltay_high - sum_deltax_low) / nsamples;

        double x = (x2_avg-x1_avg)/(x2_avg+x1_avg); 
        double y = (y2_avg-y1_avg)/(y2_avg+y1_avg); 

#ifdef HEXOUTPUT
        //Print output in Hex Format
        sprintf(msg, "%04X,%04X,%04X,%04X,%04X,%04X\n", str, x1, x2, y1, y2, deltax, deltay);
#elif defined DECOUTPUT
#ifdef NATIVEADC
        //Print output in Decimal format, scaled for 3.3V internal ADC
        sprintf(msg, "% 7.5f % 7.5f % 7.5f % 7.5f % 7.5f % 7.5f % 7.5f % 7.5f\n", x1_avg*3.3/4095, x2_avg*3.3/4095, y1_avg*3.3/4095, y2_avg*3.3/4095, deltax_avg*3.3/4095-2.5, (x1_avg-x2_avg)*3.3/4095, deltay_avg*3.3/4095-2.5, -(y1_avg-y2_avg)*3.3/4095); 
#else
        //Print output in Decimal format, scaled for 5.0V external ADC
        sprintf(msg, "% 8.6f % 8.6f % 8.6f % 8.6f % 8.6f % 8.6f % 8.6f % 8.6f\n", x1_avg*5.0/16383, x2_avg*5.0/16383, y1_avg*5.0/16383, y2_avg*5.0/16383, deltax_avg*5.0/16383-2.5, (x1_avg-x2_avg)*5.0/16383, deltay_avg*5.0/16383-2.5, -(y1_avg-y2_avg)*5.0/16383); 
#endif
#else 
        sprintf(msg, "% 8.6f % 8.6f", x, y);
#endif

#ifdef NATIVEUSB
        SerialUSB.println(msg); 
#else
        Serial.println(msg); 
#endif

        // Reset sum to zero
        sum_x1_high     = 0;
        sum_x2_high     = 0;
        sum_y1_high     = 0;
        sum_y2_high     = 0;
        sum_deltax_high = 0;
        sum_deltay_high = 0;

        // Reset sum to zero
        sum_x1_low     = 0;
        sum_x2_low     = 0;
        sum_y1_low     = 0;
        sum_y2_low     = 0;
        sum_deltax_low = 0;
        sum_deltay_low = 0;
    }

    unsigned int now = micros(); 
    //Serial.println((now-then)); 
}

void initialize_board () {
    initializeADC(); 
    setDAC(16011, OAVCC);    // Set OAVCC to 4.867V for 14-bits 16383 at 5.0V Vref
    setDAC(1638,  VTHRESH);  // Set Comparator Threshold for 14-bits 16383 at 5.0V Vref
}
