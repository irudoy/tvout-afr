#include <TVout.h>
#include <fontALL.h>
#include <CAN.h>

/**
 * ### Microchip MCP2515 wiring
 *
 * 
 * | MCP2515 | Arduino |
 * | :-----: | :-----: |
 * |   VCC   |   5V    |
 * |   GND   |   GND . |
 * |   SCK   |   SCK   | D52
 * |   SO    |   MISO  | D50
 * |   SI    |   MOSI  | D51
 * |   CS    |   10    | D53
 * |   INT   |   2     | n/c
 *
 * `CS` and `INT` pins can be changed by using `CAN.setPins(cs, irq)`.
 * `INT` pin is optional, it is only needed for receive callback mode.
 * If `INT` pin is used, it **must** be interrupt capable via [`attachInterrupt(...)`](https://www.arduino.cc/en/Reference/AttachInterrupt).
 * **NOTE**: Logic level converters must be used for boards which operate at 3.3V.
 *
 * ### TVOut
 * 
 * MCU         | SYNC  | VIDEO | AUDIO   | Arduino         | SYNC | VIDEO    | AUDIO
 * ------------|-------|-------|---------|-----------------|------|----------|-------
 * m168,m328   | B1    | D7    | B3      | NG,Decimila,UNO | 9    | 7        | 11
 * m1280,m2560 | B5    | A7    | B4      | Mega            | 11   | A7 (D29) | 10
 * m644,m1284p | D5    | A7    | D7      | sanguino        | 13   | A7 (D24) | 8
 * m32u4       | B5    | B4    | B7      | Leonardo        | 9    | 8        | 11
 * AT90USB1286 | B5    | F7    | B4      | --              | --   | --       | --
 * 
 * SYNC is on OCR1A and AUDIO is on OCR2A (except on the Arduino Leonardo, where AUDIO is on OCR0A)
 * There are some timing issues with the m1284p, may be related to sanguino core.
 * On NG, Decimila, UNO and Nano the sync is pin 9, video on 7 and audio on 11.
 * On Mega2560 sync is pin 11, video is on A7(D29) and audio is on pin 10.
 * 
 * MEGA2560
 * SYNC - Pin 11
 * VIDEO - A7 (D29)
 */

#define CAN_SPEED 500E3 // 1000E3 or 500E3 
#define CAN_SPI_CS_PIN 53
#define DELAY_FRAMES 3
#define GRAPH_MIN 5
#define GRAPH_MAX 25

#define DEBUG false
#define DEBUG_GRAPH true

TVout TV;
double minAFR = 100, maxAFR = 0;
byte graph[117] = { 0 };
double currTemp = 1.0;

int DEBUG_direction = 1;

void setup()  {
  TV.begin(PAL, 128, 96);
  renderInitialScreen("AFR", GRAPH_MIN, GRAPH_MAX);
  Serial.begin(9600);
  while (!Serial);
  Serial.println("CAN Init");
  CAN.setPins(CAN_SPI_CS_PIN); // defaults to 10
  if (!CAN.begin(CAN_SPEED)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  delay(1000);
}

void loop() {
  double prevTemp = currTemp;
  double val = 0.0;

  if (!DEBUG_GRAPH) {
    val = readAFR();
  } else {
    double step = 0.1;
    val = DEBUG_direction == 1 ? prevTemp + step : prevTemp - step;
    if (prevTemp <= 4.0 && DEBUG_direction == 0) {
      DEBUG_direction = 1;
    } else if (prevTemp >= 26.0 && DEBUG_direction == 1) {
      DEBUG_direction = 0;
    }
  }

  double currAFR = prevTemp;

  if (val) {
    currAFR = val;
    currTemp = val;
  }
  
  display_current(currAFR);
  renderGraph(currAFR, GRAPH_MIN, GRAPH_MAX);

  if (currAFR < minAFR && currAFR != -1) {
    minAFR = currAFR;
    display_min(minAFR);
  }

  if (currAFR > maxAFR) {
    maxAFR = currAFR;
    display_max(maxAFR);
  }

  TV.delay_frame(DELAY_FRAMES);
}

void display_min(double mt) {
  TV.set_cursor(0, 33);
  TV.select_font(font6x8);
  TV.print(mt, 1);
}

void display_max(double mt) {
  TV.set_cursor(103, 33);
  TV.select_font(font6x8);
  if (mt < 10) TV.print(" ");
  TV.print(mt, 1);
}

void display_current(double t) {
  TV.set_cursor(42, 32);
  TV.select_font(font8x8);
  TV.print(t, 2);
  TV.draw_rect(40, 30, 44, 12, 1, -1);
}

double readAFR() {
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    Serial.print("Received packet with id 0x");
    Serial.print(CAN.packetId(), HEX);
    Serial.print(" and length ");
    Serial.println(packetSize);

    unsigned short int resTemp = (CAN.read() << 8) | CAN.read();
    
    return (double)resTemp / 100;
  }
}

double scaleBetween(double num, double newMin, double newMax, double oldMin, double oldMax) {
  return (oldMax - oldMin) * (num - newMin) / (newMax - newMin) + oldMin;
}

void renderGraph(double val, int min, int max) {
  if (val > max) val = max;
  if (val < min) val = min;

  int offsetLeft = 0;
  const int dcount = digitsCount(max);

  if (dcount > 4) {
    offsetLeft += 22;
  } else if (dcount > 3) {
    offsetLeft += 18;
  } else if (dcount > 2) {
    offsetLeft += 14;
  } else if (dcount > 1) {
    offsetLeft += 12;
  } else {
    offsetLeft += 8;
  }

  const int offsetTop = 48;
  const int offsetBottom = TV.vres() - 8;
  const int graphWidth = TV.hres() - offsetLeft;
  const int graphHeight = offsetBottom - offsetTop;

  for (int i = 0; i < graphWidth; i++) {
    graph[i] = graph[i + 1];
    TV.set_pixel(i + offsetLeft, offsetBottom - graph[i], 1);
    if (graph[i + 1] > 0) {
      TV.set_pixel(i + offsetLeft + 1, offsetBottom - graph[i + 1], 0);
    }
  }

  graph[graphWidth] = (byte) scaleBetween(val, min, max, 0, graphHeight);

  if (graph[graphWidth] > graphHeight) {
    graph[graphWidth] = graphHeight;
  }
}

int digitsCount(long long n) { 
    return floor(log10(n) + 1); 
} 

void renderInitialScreen(const char* label, int lower, int upper) {
  const int labelGapX = upper > 10 ? 4 : 0;

  const int ticks = 5;
  int steps [ticks];
  const int span = upper - lower;
  const double step = span / (ticks - 1);

  for (int i = 0; i < ticks; i++) {
    steps[i] = lower + (int)floor(step * i);
    if (i == 0) steps[i] = lower;
    if (i == ticks - 1) steps[i] = upper;
  }

  TV.clear_screen();

  TV.set_cursor(labelGapX, 1);
  TV.select_font(font8x8);
  TV.print(label);
  TV.draw_rect(0, 0, (strlen(label) * 8) + labelGapX * 2, 10, 1, 2);

  TV.select_font(font6x8);
  TV.set_cursor(0, 20);
  TV.print("MIN");

  TV.set_cursor(43, 20);
  TV.print("Current");

  TV.set_cursor(109, 20);
  TV.print("MAX");

  TV.select_font(font4x6);

  int j = 86;
  for (int i = 0; i < ticks; i++) {
    int curr = steps[i];
    TV.set_cursor(0, j);
    TV.print(curr);
    j -= 10;
  }

  int offsetLeft = labelGapX;
  const int dcount = digitsCount(upper);

  if (dcount > 4) {
    offsetLeft += 20;
  } else if (dcount > 3) {
    offsetLeft += 16;
  } else if (dcount > 2) {
    offsetLeft += 12;
  } else if (dcount > 1) {
    offsetLeft += 8;
  } else {
    offsetLeft += 8;
  }

  TV.draw_line(offsetLeft, 46, offsetLeft, 88, 1);
  TV.draw_line(offsetLeft, 88, 126, 88, 1);
  TV.set_pixel(offsetLeft - 1, 48, 1);
  TV.set_pixel(offsetLeft - 1, 58, 1);
  TV.set_pixel(offsetLeft - 1, 68, 1);
  TV.set_pixel(offsetLeft - 1, 78, 1);
}
