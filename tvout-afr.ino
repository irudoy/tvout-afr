#include <TVout.h>
#include <fontALL.h>
#include <EMUcan.h>
#include <EEPROM.h>
#include "settings.h"
#include "aemafr.h"
#include "pitches.h"
#include "TVOLFont.h"

// SCREENS
#define SCREEN_AFR 0
#define SCREEN_OXYGEN 1
#define SCREEN_SYS_VOLTS 2
#define SCREEN_HTR_VOLTS 3
#define SCREEN_SUMMARY 4
#define MAX_SCREEN 4
// SCREENS

int currentScreen = -1;
int nextScreen = 0;

EMUcan emucan(0x600, CAN_SPI_CS_PIN);
MCP2515 mcp = *emucan.getMcp2515();
TVout TV;

AemAFRData aemAFRData;

// Graph State
const char *graphStateLabel;
double graphStateMaxValue;
int graphStateLower;
int graphStateUpper;
byte graphStatePoints[117];
// Graph State

bool shouldResetPeak = false;
bool shouldClearWarmupMessage = true;

uint8_t canRXerrors;
uint8_t canTXerrors;
uint8_t canErrorFlags;

uint8_t prevBtnValue = 0;

int lastDebounceTime = 0;
int lastDebounceTime2 = 0;

uint8_t DEBUG_direction = 1;
double DEBUG_prevValue = 0.0;
bool DEBUG_d1 = true;
bool DEBUG_d2 = true;
bool DEBUG_d3 = true;
bool DEBUG_d4 = true;
uint8_t DEBUG_d5 = 0;
uint8_t DEBUG_d6 = 0;

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void setup()  {
  // TV setup
  TV.begin(NTSC, 128, 96);
  TV.tone(NOTE_F4, 150); delay(150);
  TV.tone(NOTE_F4, 150); delay(150);
  TV.tone(NOTE_AS4, 250);

  // Serial setup
  if (DEBUG_SERIAL_ENABLE) {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("CAN Init");
  }

  // CAN Setup
  emucan.begin(CAN_SPEED, MCP_8MHZ);
  ReturnAllFramesFunction frameProcessFn = handleCANFrame;
  emucan.ReturnAllFrames(frameProcessFn);
  if (DEBUG_SERIAL_ENABLE) {
    Serial.print("EMUCAN_LIB_VERSION: ");
    Serial.println(EMUCAN_LIB_VERSION);
    Serial.println("------- CAN Read ----------");
  }

  pinMode(BTN_PIN, INPUT); // INPUT_PULLUP

  nextScreen = EEPROM.read(MEM_ADDR_CURR_SCREEN);
  if (nextScreen == 255) nextScreen = 0;

  delay(1000);
}

void handleCANFrame(const struct can_frame *frame) {
  if (frame->can_id == AEM_CAN_AFR_ID) {
    aemAFRData.lambda = ((frame->data[0] << 8) + frame->data[1]) * 0.001465; // .001465 AFR/bit ; range 0 to 96.0088 AFR
    aemAFRData.oxygen = ((frame->data[2] << 8) + frame->data[3]) * 0.001; // 0.001%/bit ; -32.768% to 32.767%
    aemAFRData.sysVolts = frame->data[4] * 0.1; // System Volts
    aemAFRData.htrVolts = frame->data[5] * 0.1; // Heater Volts
    aemAFRData.isLSU42 = frame->data[6] & 0; // Bosch LSU4.2 Sensor Detected
    aemAFRData.isLSU49 = frame->data[6] & 2; // Bosch LSU4.9 Sensor Detected
    aemAFRData.isNTKLH = frame->data[6] & 4; // NTK L#H# Sensor Detected
    aemAFRData.isNTKLHA = frame->data[6] & 8; // NTK LHA Sensor Detected
    aemAFRData.htrPIDLocked = frame->data[6] & 16; // Heater PID locked
    aemAFRData.usingFreeAirCal = frame->data[6] & 32; // Using Free-Air Cal
    aemAFRData.freeAirCalRequired = frame->data[6] & 64; // Free-Air cal required
    aemAFRData.lambdaDataValid = frame->data[6] & 128; // Lambda Data Valid
    aemAFRData.sensorState = frame->data[7] & 0x1F; // Sensor State ; 5 bit unsigned ; enum AFR_SENSOR_STATE
    aemAFRData.sensorFault = frame->data[7] & 64; // Sensor Fault
    aemAFRData.fatalError = frame->data[7] & 128; // Fatal Error
  }

  const unsigned long tm = millis();
  if ((tm - lastDebounceTime) > DEBOUNCE_DELAY || !USE_DEBOUNCE) {
    if (DEBUG_CAN) {
      Serial.print("CAN ID: ");
      Serial.print(frame->can_id, HEX); // print ID
      Serial.print("; CAN DLC: ");
      Serial.print(frame->can_dlc, HEX); // print DLC
      Serial.print("; CAN DATA: ");
      for (int i = 0; i < frame->can_dlc; i++)  { // print the data
        Serial.print(frame->data[i], HEX);
        Serial.print(", ");
      }
      Serial.println();
      Serial.print("rx errs: ");
      Serial.print(canRXerrors);
      Serial.print("; tx errs: ");
      Serial.print(canTXerrors);
      Serial.print("; eflg register: ");
      Serial.println(canErrorFlags);
    }
  
    if (DEBUG_CAN_AEMAFR && frame->can_id == AEM_CAN_AFR_ID) {
      Serial.print("AFR: "); Serial.print(aemAFRData.lambda); Serial.print("; ");
      Serial.print("Oxygen: "); Serial.print(aemAFRData.oxygen); Serial.print("%; ");
      Serial.print("System Volts: "); Serial.print(aemAFRData.sysVolts); Serial.print("V; ");
      Serial.print("Heater Volts: "); Serial.print(aemAFRData.htrVolts); Serial.print("V; ");
      Serial.print("is LSU4.2: "); Serial.print(aemAFRData.isLSU42); Serial.print("; ");
      Serial.print("is LSU4.9: "); Serial.print(aemAFRData.isLSU49); Serial.print("; ");
      Serial.print("is NTK L#H#: "); Serial.print(aemAFRData.isNTKLH); Serial.print("; ");
      Serial.print("is NTK LHA: "); Serial.print(aemAFRData.isNTKLHA); Serial.print("; ");
      Serial.print("Heater PID locked: "); Serial.print(aemAFRData.htrPIDLocked); Serial.print("; ");
      Serial.print("Using Free-Air Cal: "); Serial.print(aemAFRData.usingFreeAirCal); Serial.print("; ");
      Serial.print("Free-Air cal required: "); Serial.print(aemAFRData.freeAirCalRequired); Serial.print("; ");
      Serial.print("Lambda Data Valid: "); Serial.print(aemAFRData.lambdaDataValid); Serial.print("; ");
      Serial.print("Sensor State: "); Serial.print(AEM_AFR_STATE[aemAFRData.sensorState]); Serial.print("; ");
      Serial.print("Sensor Fault: "); Serial.print(aemAFRData.sensorFault); Serial.print("; ");
      Serial.print("Fatal Error: "); Serial.print(aemAFRData.fatalError); Serial.print("; ");
      Serial.println();
    }

    lastDebounceTime = tm;
  }
}

void loop() {
  emucan.checkEMUcan();

  if (DEBUG_SUMMARY) {
    if (!shouldClearWarmupMessage && aemAFRData.sensorState == AemAFRData::SensorState::WarmUp) {
      shouldClearWarmupMessage = true;
    }
    DEBUG_loopSummary();
  }

  if (DEBUG_CAN) {
    canRXerrors = emucan.CanErrorCounter(false);
    canTXerrors = emucan.CanErrorCounter(true);
    canErrorFlags = mcp.getErrorFlags();
  }

  // buttons press logic
  uint8_t currBtnValue = analogRead(BTN_PIN);
  bool isBtnOff = btnInRange(currBtnValue, 0);
  bool isBtnPrevOff = btnInRange(prevBtnValue, 0);
  if (isBtnOff && !isBtnPrevOff) {
    if (btnInRange(prevBtnValue, BTN_VAL_1)) {
      TV.tone(NOTE_F5, 50);
      nextScreen++;
      if (nextScreen > MAX_SCREEN) nextScreen = 0;
    } else if (btnInRange(prevBtnValue, BTN_VAL_2)) {
      TV.tone(NOTE_C5, 50);
      nextScreen--;
      if (nextScreen < 0) nextScreen = MAX_SCREEN;
    } else if (btnInRange(prevBtnValue, BTN_VAL_3)) {
      TV.tone(NOTE_C2, 50);
      shouldResetPeak = true; // Peak reset
    } else if (btnInRange(prevBtnValue, BTN_VAL_4)) {
      TV.tone(NOTE_C3, 50);
    }
  }
  prevBtnValue = currBtnValue;
  // buttons press logic

  /**
   * Setup static UI elements only once, after screen change
   */
  if (currentScreen != nextScreen) {
    currentScreen = nextScreen;
    EEPROM.update(MEM_ADDR_CURR_SCREEN, currentScreen);
    switch (nextScreen) {
      case SCREEN_AFR:
        resetGraphState("AFR", aemAFRData.lambda, 5, 25);
        renderGraphUI();
        break;
      case SCREEN_OXYGEN:
        resetGraphState("OXYGEN", aemAFRData.oxygen, -33, 33);
        renderGraphUI();
        break;
      case SCREEN_SYS_VOLTS:
        resetGraphState("SYS VOLTS", aemAFRData.sysVolts, 0, 12);
        renderGraphUI();
        break;
      case SCREEN_HTR_VOLTS:
        resetGraphState("HTR VOLTS", aemAFRData.htrVolts, 0, 12);
        renderGraphUI();
        break;
      case SCREEN_SUMMARY:
        renderSummaryUI();
        break;
      default:
        break;
    }
  }

  /**
   * Update dynamic screen data
   */
  switch (currentScreen) {
    case SCREEN_AFR:
      renderAFRGraphView(aemAFRData.lambda);
      break;
    case SCREEN_OXYGEN:
      renderGraphView(aemAFRData.oxygen);
      break;
    case SCREEN_SYS_VOLTS:
      renderGraphView(aemAFRData.sysVolts);
      break;
    case SCREEN_HTR_VOLTS:
      renderGraphView(aemAFRData.htrVolts);
      break;
    case SCREEN_SUMMARY:
      renderSummaryData();
      break;
    default:
      break;
  }

  TV.delay_frame(DELAY_FRAMES);
}

void renderSummaryUI() {
  TV.clear_screen();

  TV.select_font(font4x6);

  const int vgap = 3;
  const int fvsz = 6;

  TV.set_cursor(0, 0);
  TV.print("AFR/OXYGEN ");
  TV.set_cursor(0, (vgap + fvsz) * 1);
  TV.print("SYSTEM VOLTAGE");
  TV.set_cursor(0, (vgap + fvsz) * 2);
  TV.print("HEATER VOLTAGE");
  TV.set_cursor(0, (vgap + fvsz) * 3);
  TV.print("SENSOR TYPE");
  TV.set_cursor(0, (vgap + fvsz) * 4);
  TV.print("HEATER PID LOCKED");
  TV.set_cursor(0, (vgap + fvsz) * 5);
  TV.print("USING FREE-AIR CAL");
  TV.set_cursor(0, (vgap + fvsz) * 6);
  TV.print("FREE-AIR CAL REQ ");
  TV.set_cursor(0, (vgap + fvsz) * 7);
  TV.print("LAMBDA DATA VALID");
  TV.set_cursor(0, (vgap + fvsz) * 8);
  TV.print("SENSOR STATE");
  TV.set_cursor(0, (vgap + fvsz) * 9);
  TV.print("SENSOR FAULT");
  TV.set_cursor(0, (vgap + fvsz) * 10);
  TV.print("FATAL ERROR ");
}

void renderSummaryData() {
  TV.select_font(font4x6);

  const int xpos = 100;
  const int vgap = 3;
  const int fvsz = 6;

  TV.draw_rect(xpos - 31, 0, 60, fvsz * 3 + vgap * 2, 0, 0); // clear
  TV.set_cursor(xpos - 31, 0);
  TV.print(aemAFRData.lambda, 1);
  TV.set_cursor(xpos - 13, 0);
  TV.print(" / ");
  TV.set_cursor(xpos, 0);
  TV.print(aemAFRData.oxygen, 1);

  TV.set_cursor(xpos, (vgap + fvsz) * 1);
  TV.print(aemAFRData.sysVolts, 1);
  TV.set_cursor(xpos, (vgap + fvsz) * 2);
  TV.print(aemAFRData.htrVolts, 1);

  TV.set_cursor(xpos, (vgap + fvsz) * 3);
  if (aemAFRData.isLSU42) {
    TV.print("LSU4.2");
  } else if (aemAFRData.isLSU49) {
    TV.print("LSU4.9");
  } else if (aemAFRData.isNTKLH) {
    TV.print("NTKLH ");
  } else if (aemAFRData.isNTKLHA) {
    TV.print("NTKLHA");
  } else  {
    TV.print("N/C    ");
  }

  TV.set_cursor(xpos, (vgap + fvsz) * 4);
  TV.print(aemAFRData.htrPIDLocked ? "YES" : "NO ");
  TV.set_cursor(xpos, (vgap + fvsz) * 5);
  TV.print(aemAFRData.usingFreeAirCal ? "YES" : "NO ");
  TV.set_cursor(xpos, (vgap + fvsz) * 6);
  TV.print(aemAFRData.freeAirCalRequired ? "YES" : "NO ");
  TV.set_cursor(xpos, (vgap + fvsz) * 7);
  TV.print(aemAFRData.lambdaDataValid ? "YES" : "NO ");
  TV.set_cursor(xpos, (vgap + fvsz) * 8);
  TV.print(AEM_AFR_STATE[aemAFRData.sensorState]);
  TV.set_cursor(xpos, (vgap + fvsz) * 9);
  TV.print(aemAFRData.sensorFault ? "YES" : "NO ");
  TV.set_cursor(xpos, (vgap + fvsz) * 10);
  TV.print(aemAFRData.fatalError ? "YES" : "NO ");
}

void renderAFRGraphView(double value) {
  if (aemAFRData.sensorState == AemAFRData::SensorState::WarmUp) {
    TV.set_cursor(30, 60);
    TV.select_font(font8x8);
    TV.print("WARMING UP");
    TV.draw_rect(28, 58, 83, 12, 1, -1);
    renderGraphView(0);
  } else {
    renderGraphView(value);
    if (shouldClearWarmupMessage) {
      TV.draw_rect(28, 58, 83, 12, 0, 0);
      shouldClearWarmupMessage = false;
      TV.tone(NOTE_C6, 500);
    }
  }
}

void renderGraphView(double value) {
  if (DEBUG_GRAPH) {
    double step = 0.8;
    value = DEBUG_direction == 1 ? DEBUG_prevValue + step : DEBUG_prevValue - step;
    if (DEBUG_prevValue <= graphStateLower - 1 && DEBUG_direction == 0) {
      DEBUG_direction = 1;
    } else if (DEBUG_prevValue >= graphStateUpper + 1 && DEBUG_direction == 1) {
      DEBUG_direction = 0;
    }
    DEBUG_prevValue = value;
  }
  
  renderGraph(value, graphStateLower, graphStateUpper);

  printLNum(0, 19, value);

  if (value > graphStateMaxValue || shouldResetPeak) {
    graphStateMaxValue = value;
    shouldResetPeak = false;
  }
  TV.set_cursor(100, 29);
  TV.select_font(font6x8);
  TV.print(graphStateMaxValue, 1);
}

void renderGraph(double val, int min, int max) {
  if (val > max) val = max;
  if (val < min) val = min;

  int offsetLeft = getOffsetLeft(min, max);

  const int offsetTop = 48;
  const int offsetBottom = TV.vres() - 8;
  const int graphWidth = TV.hres() - offsetLeft;
  const int graphHeight = offsetBottom - offsetTop;

  for (int i = 0; i < graphWidth; i++) {
    graphStatePoints[i] = graphStatePoints[i + 1];
    TV.set_pixel(i + offsetLeft, offsetBottom - graphStatePoints[i], 1);
    if (graphStatePoints[i + 1] > 0) {
      TV.set_pixel(i + offsetLeft + 1, offsetBottom - graphStatePoints[i + 1], 0);
    }
  }

  graphStatePoints[graphWidth] = (byte) scaleBetween(val, min, max, 0, graphHeight);

  if (graphStatePoints[graphWidth] > graphHeight) {
    graphStatePoints[graphWidth] = graphHeight;
  }
}

void renderGraphUI() {
  TV.clear_screen();

  const int labelGapX = graphStateUpper > 10 ? 4 : 0;

  const int ticks = 5;
  int steps [ticks];
  const int span = graphStateUpper - graphStateLower;
  const double step = span / (ticks - 1);

  for (int i = 0; i < ticks; i++) {
    steps[i] = graphStateLower + (int)floor(step * i);
    if (i == 0) steps[i] = graphStateLower;
    if (i == ticks - 1) steps[i] = graphStateUpper;
  }

  TV.set_cursor(labelGapX, 1);
  TV.select_font(font8x8);
  TV.print(graphStateLabel);
  TV.draw_rect(0, 0, (strlen(graphStateLabel) * 8) + labelGapX * 2, 10, 1, 2);

  TV.select_font(font4x6);
  TV.set_cursor(100, 21);
  TV.print("PEAK");

  TV.select_font(font4x6);

  // TODO: align right
  int j = 86;
  for (int i = 0; i < ticks; i++) {
    int curr = steps[i];
    TV.set_cursor(0, j);
    TV.print(curr);
    j -= 10;
  }

  int offsetLeft = getOffsetLeft(graphStateLower, graphStateUpper);

  TV.draw_line(offsetLeft, 46, offsetLeft, 88, 1);
  TV.draw_line(offsetLeft, 88, 126, 88, 1);
  TV.set_pixel(offsetLeft - 1, 48, 1);
  TV.set_pixel(offsetLeft - 1, 58, 1);
  TV.set_pixel(offsetLeft - 1, 68, 1);
  TV.set_pixel(offsetLeft - 1, 78, 1);
}

void resetGraphState(const char *label, double value, int lower, int upper) {
  graphStateLabel = label;
  graphStateMaxValue = value;
  graphStateLower = lower;
  graphStateUpper = upper;
  memset(graphStatePoints, 0, sizeof(graphStatePoints));
}

/**
 * Helpers
 */

double scaleBetween(double num, double newMin, double newMax, double oldMin, double oldMax) {
  return (oldMax - oldMin) * (num - newMin) / (newMax - newMin) + oldMin;
}

int digitsCount(long n) { 
    const int val = floor(log10(abs(n)) + 1);
    return n < 0 ? val + 1 : val;
}

int getOffsetLeft(int lower, int upper) {
  int val = 0;
  const int dcountUpper = digitsCount(upper);
  const int dcountLower = digitsCount(lower);
  const int dcount = max(dcountUpper, dcountLower);
  if (dcount > 4) {
    val = 22;
  } else if (dcount > 3) {
    val = 18;
  } else if (dcount > 2) {
    val = 14;
  } else if (dcount > 1) {
    val = 12;
  } else {
    val = 8;
  }
  return val;
}

void DEBUG_loopSummary() {
  if (DEBUG_d1) {
    aemAFRData.lambda = aemAFRData.lambda + 0.1;
    if (aemAFRData.lambda > 30) DEBUG_d1 = false;
  } else {
    if (aemAFRData.lambda <= 0) {
      DEBUG_d1 = true;
      aemAFRData.lambda = 0;
    } else {
      aemAFRData.lambda = aemAFRData.lambda - 0.1;
    }
  }
  if (DEBUG_d2) {
    aemAFRData.oxygen = aemAFRData.oxygen + 0.33;
    if (aemAFRData.oxygen > 30) DEBUG_d2 = false;
  } else {
    aemAFRData.oxygen = aemAFRData.oxygen - 0.33;
    if (aemAFRData.oxygen <= -30) DEBUG_d2 = true;
  }
  if (DEBUG_d3) {
    aemAFRData.sysVolts = aemAFRData.sysVolts + 0.05;
    if (aemAFRData.sysVolts > 14) DEBUG_d3 = false;
  } else {
    aemAFRData.sysVolts = aemAFRData.sysVolts - 0.05;
    if (aemAFRData.sysVolts <= 5) DEBUG_d3 = true;
  }
  if (DEBUG_d4) {
    aemAFRData.htrVolts = aemAFRData.htrVolts + 0.05;
    if (aemAFRData.htrVolts > 9) DEBUG_d4 = false;
  } else {
    aemAFRData.htrVolts = aemAFRData.htrVolts - 0.05;
    if (aemAFRData.htrVolts <= 6) DEBUG_d4 = true;
  }

  const unsigned long tm = millis();
  if ((tm - lastDebounceTime) < 1000) return;

  aemAFRData.isLSU42 = 0;
  aemAFRData.isLSU49 = 0;
  aemAFRData.isNTKLH = 0;
  aemAFRData.isNTKLHA = 0;
  switch (DEBUG_d5) {
    case 0:
      aemAFRData.isLSU42 = 1;
      break;
    case 1:
      aemAFRData.isLSU49 = 1;
      break;
    case 2:
      aemAFRData.isNTKLH = 1;
      break;
    case 3:
      aemAFRData.isNTKLHA = 1;
      break;
    default:
      break;
  }
  DEBUG_d5 = DEBUG_d5 >= 3 ? 0 : DEBUG_d5 + 1;
  aemAFRData.htrPIDLocked = !aemAFRData.htrPIDLocked;
  aemAFRData.usingFreeAirCal = !aemAFRData.usingFreeAirCal;
  aemAFRData.freeAirCalRequired = !aemAFRData.freeAirCalRequired;
  aemAFRData.lambdaDataValid = !aemAFRData.lambdaDataValid;
  aemAFRData.sensorFault = !aemAFRData.sensorFault;
  aemAFRData.fatalError = !aemAFRData.fatalError;
  aemAFRData.sensorState = DEBUG_d6;
  DEBUG_d6 = DEBUG_d6 >= 20 ? 0 : DEBUG_d6 + 1;

  lastDebounceTime = tm;
}

bool btnInRange(uint8_t val, uint8_t target) {
  return (val <= (target + 8) && val >= (target - 8));
}

void printLNum(uint8_t posx, uint8_t posy, double value) {
  char buf[5];
  dtostrf(value, 0, 1, buf); // todo: use min_width for padding maybe?
  const uint8_t pad = 32;
  const uint8_t w = 14;
  for (uint8_t i = 0; i < 5; i++) {
    char c = buf[i];
    const uint8_t x = posx + w * i;
    const uint8_t y = posy;
    switch (c) {
      case 48:
        TV.bitmap(x, y, TVOLFont, pad * 0, w, w);
        break;
      case 49:
        TV.bitmap(x, y, TVOLFont, pad * 1, w, w);
        break;
      case 50:
        TV.bitmap(x, y, TVOLFont, pad * 2, w, w);
        break;
      case 51:
        TV.bitmap(x, y, TVOLFont, pad * 3, w, w);
        break;
      case 52:
        TV.bitmap(x, y, TVOLFont, pad * 4, w, w);
        break;
      case 53:
        TV.bitmap(x, y, TVOLFont, pad * 5, w, w);
        break;
      case 54:
        TV.bitmap(x, y, TVOLFont, pad * 6, w, w);
        break;
      case 55:
        TV.bitmap(x, y, TVOLFont, pad * 7, w, w);
        break;
      case 56:
        TV.bitmap(x, y, TVOLFont, pad * 8, w, w);
        break;
      case 57:
        TV.bitmap(x, y, TVOLFont, pad * 9, w, w);
        break;
      case 45:
        TV.bitmap(x, y, TVOLFont, pad * 10, w, w);
        break;
      case 46:
        TV.bitmap(x, y, TVOLFont, pad * 11, w, w);
        break;
      default:
        TV.bitmap(x, y, TVOLFont, pad * 12, w, w);
        break;
    }
  }
}
