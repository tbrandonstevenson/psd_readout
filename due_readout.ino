#include <Ethernet.h>
#include <SPI.h>

#define X1_A    0U
#define X2_A    1U
#define Y1_A    2U
#define Y2_A    3U

#define X_1     0U
#define X_2     1U
#define Y_1     2U
#define Y_2     4U

#define CLKA    22U

/* Setup Ethernet */
byte mac[]      = {
  0xF8, 0xB1, 0x56, 0xA8, 0xFB, 0x4D
};
byte ip []      = {
  169, 232, 153, 99
};
byte gateway [] = {
  169, 232, 153, 254
};
byte subnet  [] = {
  255, 255, 255, 0
};

EthernetServer server = EthernetServer(23);

/* Variable Initialization, for later..*/
uint32_t command;

/*uint8_t testdata[]      = {
  0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0xFF
};*/

void setup()
{
  /* Start Ethernet Server */
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();

  //pinMode(DAC0, OUTPUT);
  //pinMode(DAC1, OUTPUT);

  /* Configure Interrupt Pins*/
  pinMode(CLKA, INPUT);                     // configure as input
  attachInterrupt(CLKA, interrupt, CHANGE); // enable interrupts on this pin
  digitalWrite(CLKA, HIGH);                 // turn on a pullup resistor (could just as well be LOW)

  // Set Resolution for analog Read and Write
  analogReadResolution(12);
  analogWriteResolution(12);

  // Setup Serial Bus
  SerialUSB.begin(115200);
  //while (!SerialUSB.available());
  //SerialUSB.print("Starting...\n");
}

void loop()
{

}

/* Routine to take a reading from a single sensor and transmit results through TCP/IP */
void ReadSensor(uint16_t &x1, uint16_t &x2, uint16_t &y1, uint16_t &y2)
{
  x1 = analogRead(X1_A);
  x2 = analogRead(X2_A);
  y1 = analogRead(Y1_A);
  y2 = analogRead(Y2_A);

  uint8_t header0;
  uint8_t trailer0;

  header0  = 0xFF;
  trailer0 = 0xFF;

}

/* Pin Change Interrupt Routines */
void interrupt() {
  uint16_t x1;
  uint16_t x2;
  uint16_t y1;
  uint16_t y2;
  int sum_x1 = 0;
  int sum_x2 = 0;
  int sum_y1 = 0;
  int sum_y2 = 0;
  int start_time; 
  int stop_time; 
  int nsamples = 40;
  
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = micros();
  if (interrupt_time - last_interrupt_time > 100)
  {
    delayMicroseconds(50);
    //start_time = micros();
    for (int i = 0; i < nsamples; i++) {
      ReadSensor(x1, x2, y1, y2);
      sum_x1 += x1;
      sum_x2 += x2;
      sum_y1 += y1;
      sum_y2 += y2;
    //stop_time = micros(); 
    }
    
    char msg [50];
    x1 = sum_x1 / nsamples;
    x2 = sum_x2 / nsamples;
    y1 = sum_y1 / nsamples;
    y2 = sum_y2 / nsamples;

    //sprintf(msg, "%04i", stop_time-start_time);
    sprintf(msg, "%04i,%04i,%04i,%04i", x1, x2, y1, y2);
    SerialUSB.println(msg);
  }
}

/*
  uint8_t data [] = {
    header0,          //0
    (x1 >> 8) & 0xFF, //1
    (x1 >> 0) & 0xFF, //2
    (x2 >> 8) & 0xFF, //3
    (x2 >> 0) & 0xFF, //4
    (y1 >> 8) & 0xFF, //5
    (y1 >> 0) & 0xFF, //6
    (y2 >> 8) & 0xFF, //7
    (y2 >> 0) & 0xFF, //8
    trailer0,         //9
  };
*/
//server.write(testdata, sizeof(testdata));
