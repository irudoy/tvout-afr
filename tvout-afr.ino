#include <TVout.h>
#include <fontALL.h>

#define AFR_ANALOG_PIN A0
#define DELAY_FRAMES 3
#define GRAPH_MIN 0
#define GRAPH_MAX 20

#define DEBUG false

TVout TV;
float minAFR = 100, maxAFR = 0;
byte graph[117] = { 0 };

void setup()  {
  TV.begin(NTSC, 128, 96);
  draw_initial_screen();
  delay(1000);
}

void loop() {
  double currAFR = readAFR();
  
  display_current(currAFR);
  display_graph(currAFR);

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

void display_graph(float val) {
  if (val > GRAPH_MAX) {
    val = GRAPH_MAX;
  }
  if (val < GRAPH_MIN) {
    val = GRAPH_MIN;
  }
  for (int i = 0; i < 116; i++) {
    graph[i] = graph[i + 1];
    TV.set_pixel(i + 10, 88 - graph[i], 1);
    if (graph[i + 1] > GRAPH_MIN)
      TV.set_pixel(i + 11, 88 - graph[i + 1], 0);
  }
  graph[116] = (byte)val * 2;
}

void display_min(float mt) {
  TV.set_cursor(0, 33);
  TV.select_font(font6x8);
  if (mt < 10) TV.print(" ");
  TV.print(mt, 1);
}

void display_max(float mt) {
  TV.set_cursor(103, 33);
  TV.select_font(font6x8);
  if (mt < 10) TV.print(" ");
  TV.print(mt, 1);
}

void display_current(float t) {
  TV.set_cursor(42, 32);
  TV.select_font(font8x8);
  TV.print(t, 2);
  TV.draw_rect(40, 30, 44, 12, 1, -1);
}

double readAFR() {
  int rawVoltage = analogRead(AFR_ANALOG_PIN);

  if (rawVoltage < 5) {
    return -1.0;
  }

  double voltage = rawVoltage * (5.0 / 1023.0);
  double result = (voltage * 2.375) + 7.3125;

  if (DEBUG) {
    TV.clear_screen();
    TV.set_cursor(0, 0);
    TV.select_font(font4x6);
    TV.print("Raw: ");
    TV.print(rawVoltage);
    TV.set_cursor(40, 0);
    TV.print(voltage);
    TV.print("v");
    TV.set_cursor(80, 0);
    TV.print("AFR: ");
    TV.print(result);
  }

  return result;
}

void draw_initial_screen() {
  TV.clear_screen();

  TV.set_cursor(4, 1);
  TV.select_font(font8x8);
  TV.print("AFR");
  TV.draw_rect(0, 0, 31, 10, 1, 2);

  TV.select_font(font6x8);
  TV.set_cursor(0, 20);
  TV.print("MIN");

  TV.set_cursor(43, 20);
  TV.print("Current");

  TV.set_cursor(109, 20);
  TV.print("MAX");

  TV.select_font(font4x6);
  TV.set_cursor(4, 86);
  TV.print("0");
  TV.set_cursor(4, 76);
  TV.print("5");
  TV.set_cursor(0, 66);
  TV.print("10");
  TV.set_cursor(0, 56);
  TV.print("15");
  TV.set_cursor(0, 46);
  TV.print("20");

  TV.draw_line(10, 46, 10, 88, 1);
  TV.draw_line(10, 88, 126, 88, 1);
  TV.set_pixel(9, 48, 1);
  TV.set_pixel(11, 48, 1);
  TV.set_pixel(9, 58, 1);
  TV.set_pixel(11, 58, 1);
  TV.set_pixel(9, 68, 1);
  TV.set_pixel(11, 68, 1);
  TV.set_pixel(9, 78, 1);
  TV.set_pixel(11, 78, 1);
}
