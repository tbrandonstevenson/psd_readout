#include <SPI.h>
#include <Ethernet.h>

#define X1_A    0U
#define X2_A    1U
#define Y1_A    2U
#define Y2_A    3U

#define X1_B    4U
#define X2_B    5U
#define Y1_B    6U
#define Y2_B    7U

#define X1_C    8U
#define X2_C    9U
#define Y1_C    10U
#define Y2_C    11U

#define X1_D    8U
#define X2_D    9U
#define Y1_D    10U
#define Y2_D    11U

#define X_1     0U
#define X_2     1U
#define Y_1     2U
#define Y_2     4U

#define SENSOR_A 0U
#define SENSOR_B 1U
#define SENSOR_C 2U
#define SENSOR_D 2U

#define CLKA    50U
#define CLKB    51U
#define CLKC    52U
#define CLKD    52U

#define WAIT_TIME 50U

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

int client_connected = 0;

uint32_t evt_A = 0;
uint32_t evt_B = 0;
uint32_t evt_C = 0;
uint32_t evt_D = 0;

boolean trigA;
boolean trigB;
boolean trigC;
boolean trigD;

boolean measureA;
boolean measureB;
boolean measureC;
boolean measureD;

void setup()
{
  /* Start Ethernet Server */
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();

  /* Configure Interrupt Pins*/
  pinMode(CLKA, INPUT);
  pinMode(CLKB, INPUT);
  pinMode(CLKC, INPUT);
  pinMode(CLKD, INPUT);

  /* Attach Interrupt Routines to pcint pins */
  attachInterrupt(CLKA, pcintA, CHANGE);
  attachInterrupt(CLKB, pcintB, CHANGE);
  attachInterrupt(CLKC, pcintC, CHANGE);
  attachInterrupt(CLKD, pcintD, CHANGE);
}

void loop()
{
  /* Check if there's some data to read */
  EthernetClient client = server.available();
  if (client) {
    client_connected = 1;
    uint32_t tmp = client.read();
    if (tmp >= 0)
      command = tmp;
  }
  else
    client_connected = 0;

  /* Control of Channel A */
  if (command & 0x1)
    measureA = true;
  else
    measureA = false;

  /* Control of Channel B */
  if (command & 0x2)
    measureB = true;
  else
    measureB = false;

  /* Control of Channel C */
  if (command & 0x4)
    measureC = true;
  else
    measureC = false;

  /* Control of Channel D */
  if (command & 0x8)
    measureD = true;
  else
    measureD = false;

  /* Read Bias and Threshold Voltage Settings */
  byte bias   = (command >> 16) & 0xFF;
  byte thresh = (command >>  8) & 0xFF;

  /* If Ch.A is enabled and a trigger was seen, take a measurement*/
  if (trigA && measureA) {
    if (evt_A > 0xFFFFFF) evt_A = 0;
    ReadSensor(SENSOR_A, evt_A);
    evt_A += 1;
    trigA = false;
  }

  /* If Ch.B is enabled and a trigger was seen, take a measurement */
  if (trigB && measureB) {
    if (evt_B > 0xFFFFFF) evt_B = 0;
    ReadSensor(SENSOR_B, evt_B);
    evt_B += 1;
    trigB = false;
  }

  /* If Ch.C is enabled and a trigger was seen, take a measurement */
  if (trigC && measureC) {
    if (evt_C > 0xFFFFFF) evt_C = 0;
    ReadSensor(SENSOR_C, evt_C);
    evt_C += 1;
    trigC = false;
  }

  /* If Ch.D is enabled and a trigger was seen */
  if (trigD && measureD) {
    if (evt_D > 0xFFFFFF) evt_D = 0;
    ReadSensor(SENSOR_D, evt_D);
    evt_D += 1;
    trigD = false;
  }

  /* Write threshold and bias settings to DAC */
  analogWrite(DAC0, thresh);
  analogWrite(DAC1, bias);
}

/* Routine to take a reading from a single sensor and transmit results through TCP/IP */
void ReadSensor(byte sensor, uint32_t event)
{
  if (client_connected == 1) {
    int x1 = analogRead(whichPin(sensor, X_1));
    int x2 = analogRead(whichPin(sensor, X_2));
    int y1 = analogRead(whichPin(sensor, Y_1));
    int y2 = analogRead(whichPin(sensor, Y_2));

    byte header0;
    byte header1;
    byte rsvd;
    byte evtbyte0;
    byte evtbyte1;
    byte evtbyte2;
    byte trailer0;
    byte trailer1;

    header0  = 0x55;
    header1  = sensor & 0x3;
    rsvd     = 0;
    evtbyte0 = (event >> 16) & 0xFF;
    evtbyte1 = (event >>  8) & 0xFF;
    evtbyte2 = (event >>  0) & 0xFF;
    trailer0 = 0;
    trailer1 = 0xAA;

    byte data [] = {
      header0,          //0
      header1,          //1
      rsvd,             //2
      evtbyte0,         //3
      evtbyte1,         //4
      evtbyte2,         //5
      (x1 >> 4) & 0xFF, //6
      (x1 >> 0) & 0xFF, //7
      (x2 >> 4) & 0xFF, //8
      (x2 >> 0) & 0xFF, //9
      (y1 >> 4) & 0xFF, //10
      (y1 >> 0) & 0xFF, //11
      (y2 >> 4) & 0xFF, //12
      (y2 >> 0) & 0xFF, //13
      trailer0,         //14
      trailer1          //15
    };

    byte crc = CRC8(data, 15);
    data[14] = crc;

    server.write(data, sizeof(data));
  }
}

/* Pin Change Interrupt Routines */
void pcintA() {
  read_adc_interrupt(trigA);
}

void pcintB() {
  read_adc_interrupt(trigB);
}

void pcintC() {
  read_adc_interrupt(trigC);
}

void pcintD() {
  read_adc_interrupt(trigD);
}

void read_adc_interrupt(boolean trig) {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 300)
    delayMicroseconds(WAIT_TIME);
  trig = true;
}

/* Returns the Pin ID for a given sensor and coordinate (e.g. sensor A X1 => X1_A)*/
unsigned int whichPin(unsigned int sensor, unsigned int coordinate) {
  if (sensor == SENSOR_A) {
    if (coordinate == X_1) return X1_A;
    if (coordinate == X_2) return X2_A;
    if (coordinate == Y_1) return Y2_A;
    if (coordinate == Y_2) return Y2_A;
  }

  if (sensor == SENSOR_B) {
    if (coordinate == X_1) return X1_B;
    if (coordinate == X_2) return X2_B;
    if (coordinate == Y_1) return Y2_B;
    if (coordinate == Y_2) return Y2_B;
  }

  if (sensor == SENSOR_C) {
    if (coordinate == X_1) return X1_C;
    if (coordinate == X_2) return X2_C;
    if (coordinate == Y_1) return Y2_C;
    if (coordinate == Y_2) return Y2_C;
  }

  if (sensor == SENSOR_D) {
    if (coordinate == X_1) return X1_D;
    if (coordinate == X_2) return X2_D;
    if (coordinate == Y_1) return Y2_D;
    if (coordinate == Y_2) return Y2_D;
  }
}

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}
