#include <TVout.h>
#include <fontALL.h>
#include <EMUcan.h>
#include "./src/AemAFRData.h"

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

#define CAN_SPEED CAN_500KBPS
#define CAN_SPI_CS_PIN 53
#define AEM_CAN_AFR_ID 0x80000180
#define DELAY_FRAMES 3

#define USE_DEBOUNCE false
#define DEBOUNCE_DELAY 1000

#define DEBUG_GRAPH false
#define DEBUG_CAN false
#define DEBUG_CAN_AEMAFR false

// SCREENS
#define SCREEN_AFR 0
#define SCREEN_OXYGEN 1
#define SCREEN_SYS_VOLTS 2
#define SCREEN_HTR_VOLTS 3
#define SCREEN_SUMMARY 4
#define MAX_SCREEN 4
// SCREENS

int currentScreen = -1;
int nextScreen = SCREEN_SUMMARY; // Default Screen

const char *AEM_AFR_STATE[21] = {
  "RESET",
  "WARM_UP",
  "STABILIZE",
  "READ_NERNST_PUMP",
  "EQUALIZE",
  "READ_RCAL",
  "RUN",
  "OVERHEAT",
  "OVERCOOL",
  "HEATER_SHORT",
  "HEATER_OPEN",
  "START_FAC",
  "FAC",
  "DETECT_SENSOR",
  "READ_JUNCT",
  "EVAP_STARTUP",
  "SENSOR_TYPE",
  "PREPARE_TO_RUN",
  "SENSOR_SAVE",
  "NEED_FAC",
  "ERROR"
};

EMUcan emucan(0x600, CAN_SPI_CS_PIN);
MCP2515 mcp = *emucan.getMcp2515();
TVout TV;

struct AemAFRData *aemAFRData;

// Graph State
const char *graphStateLabel;
double graphStateMinValue;
double graphStateMaxValue;
int graphStateLower;
int graphStateUpper;
byte graphStatePoints[117];
// Graph State

uint8_t canRXerrors;
uint8_t canTXerrors;
uint8_t canErrorFlags;

int lastDebounceTime = 0;

int DEBUG_direction = 1;
double DEBUG_prevValue = 0.0;

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void setup()  {
  // TV setup
  TV.begin(NTSC, 128, 96);

  // Serial setup
  Serial.begin(9600);
  while (!Serial);
  Serial.println("CAN Init");

  // CAN Setup
  emucan.begin(CAN_SPEED, MCP_8MHZ);
  Serial.print("EMUCAN_LIB_VERSION: ");
  Serial.println(EMUCAN_LIB_VERSION);
  ReturnAllFramesFunction frameProcessFn = handleCANFrame;
  emucan.ReturnAllFrames(frameProcessFn);
  Serial.println("------- CAN Read ----------");

  delay(1000);
}

void handleCANFrame(const struct can_frame *frame) {
  if (frame->can_id == AEM_CAN_AFR_ID) {
    aemAFRData->lambda = ((frame->data[0] << 8) + frame->data[1]) * 0.001465; // .001465 AFR/bit ; range 0 to 96.0088 AFR
    aemAFRData->oxygen = ((frame->data[2] << 8) + frame->data[3]) * 0.001; // 0.001%/bit ; -32.768% to 32.767%
    aemAFRData->sysVolts = frame->data[4] * 0.1; // System Volts
    aemAFRData->htrVolts = frame->data[5] * 0.1; // Heater Volts
    aemAFRData->isLSU42 = frame->data[6] & 0; // Bosch LSU4.2 Sensor Detected
    aemAFRData->isLSU49 = frame->data[6] & 2; // Bosch LSU4.9 Sensor Detected
    aemAFRData->isNTKLH = frame->data[6] & 4; // NTK L#H# Sensor Detected
    aemAFRData->isNTKLHA = frame->data[6] & 8; // NTK LHA Sensor Detected
    aemAFRData->htrPIDLocked = frame->data[6] & 16; // Heater PID locked
    aemAFRData->usingFreeAirCal = frame->data[6] & 32; // Using Free-Air Cal
    aemAFRData->freeAirCalRequired = frame->data[6] & 64; // Free-Air cal required
    aemAFRData->lambdaDataValid = frame->data[6] & 128; // Lambda Data Valid
    aemAFRData->sensorState = frame->data[7] & 0x1F; // Sensor State ; 5 bit unsigned ; enum AFR_SENSOR_STATE
    aemAFRData->sensorFault = frame->data[7] & 64; // Sensor Fault
    aemAFRData->fatalError = frame->data[7] & 128; // Fatal Error
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
      Serial.print("AFR: "); Serial.print(aemAFRData->lambda); Serial.print("; ");
      Serial.print("Oxygen: "); Serial.print(aemAFRData->oxygen); Serial.print("%; ");
      Serial.print("System Volts: "); Serial.print(aemAFRData->sysVolts); Serial.print("V; ");
      Serial.print("Heater Volts: "); Serial.print(aemAFRData->htrVolts); Serial.print("V; ");
      Serial.print("is LSU4.2: "); Serial.print(aemAFRData->isLSU42); Serial.print("; ");
      Serial.print("is LSU4.9: "); Serial.print(aemAFRData->isLSU49); Serial.print("; ");
      Serial.print("is NTK L#H#: "); Serial.print(aemAFRData->isNTKLH); Serial.print("; ");
      Serial.print("is NTK LHA: "); Serial.print(aemAFRData->isNTKLHA); Serial.print("; ");
      Serial.print("Heater PID locked: "); Serial.print(aemAFRData->htrPIDLocked); Serial.print("; ");
      Serial.print("Using Free-Air Cal: "); Serial.print(aemAFRData->usingFreeAirCal); Serial.print("; ");
      Serial.print("Free-Air cal required: "); Serial.print(aemAFRData->freeAirCalRequired); Serial.print("; ");
      Serial.print("Lambda Data Valid: "); Serial.print(aemAFRData->lambdaDataValid); Serial.print("; ");
      Serial.print("Sensor State: "); Serial.print(AEM_AFR_STATE[aemAFRData->sensorState]); Serial.print("; ");
      Serial.print("Sensor Fault: "); Serial.print(aemAFRData->sensorFault); Serial.print("; ");
      Serial.print("Fatal Error: "); Serial.print(aemAFRData->fatalError); Serial.print("; ");
      Serial.println();
    }

    lastDebounceTime = tm;
  }
}

void loop() {
  emucan.checkEMUcan();

  if (DEBUG_CAN) {
    canRXerrors = emucan.CanErrorCounter(false);
    canTXerrors = emucan.CanErrorCounter(true);
    canErrorFlags = mcp.getErrorFlags();
  }

  if (Serial.available()) {
    Serial.read();
    nextScreen++;
    if (nextScreen > MAX_SCREEN) nextScreen = 0;
  }

  /**
   * Setup static UI elements only once, after screen change
   */
  if (currentScreen != nextScreen) {
    currentScreen = nextScreen;
    switch (nextScreen) {
      case SCREEN_AFR:
        resetGraphState("AFR", aemAFRData->lambda, 5, 25);
        renderGraphUI();
        break;
      case SCREEN_OXYGEN:
        resetGraphState("OXYGEN", aemAFRData->oxygen, -33, 33);
        renderGraphUI();
        break;
      case SCREEN_SYS_VOLTS:
        resetGraphState("SYS VOLTS", aemAFRData->sysVolts, 0, 12);
        renderGraphUI();
        break;
      case SCREEN_HTR_VOLTS:
        resetGraphState("HTR VOLTS", aemAFRData->htrVolts, 0, 12);
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
      renderGraphView(aemAFRData->lambda);
      break;
    case SCREEN_OXYGEN:
      renderGraphView(aemAFRData->oxygen);
      break;
    case SCREEN_SYS_VOLTS:
      renderGraphView(aemAFRData->sysVolts);
      break;
    case SCREEN_HTR_VOLTS:
      renderGraphView(aemAFRData->htrVolts);
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

  TV.set_cursor(xpos - 35, 0);
  TV.print(aemAFRData->lambda);
  TV.set_cursor(xpos - 15, 0);
  TV.print(" / ");
  TV.set_cursor(xpos, 0);
  TV.print(aemAFRData->oxygen);

  TV.set_cursor(xpos, (vgap + fvsz) * 1);
  TV.print(aemAFRData->sysVolts);
  TV.set_cursor(xpos, (vgap + fvsz) * 2);
  TV.print(aemAFRData->htrVolts);

  TV.set_cursor(xpos, (vgap + fvsz) * 3);
  if (aemAFRData->isLSU42) {
    TV.print("LSU4.2");
  } else if (aemAFRData->isLSU49) {
    TV.print("LSU4.9");
  } else if (aemAFRData->isNTKLH) {
    TV.print("NTKLH");
  } else if (aemAFRData->isNTKLHA) {
    TV.print("NTKLHA");
  } else  {
    TV.print("N/C");
  }

  TV.set_cursor(xpos, (vgap + fvsz) * 4);
  TV.print(aemAFRData->htrPIDLocked ? "YES" : "NO");
  TV.set_cursor(xpos, (vgap + fvsz) * 5);
  TV.print(aemAFRData->usingFreeAirCal ? "YES" : "NO");
  TV.set_cursor(xpos, (vgap + fvsz) * 6);
  TV.print(aemAFRData->freeAirCalRequired ? "YES" : "NO");
  TV.set_cursor(xpos, (vgap + fvsz) * 7);
  TV.print(aemAFRData->lambdaDataValid ? "YES" : "NO");
  TV.set_cursor(xpos, (vgap + fvsz) * 8);
  TV.print(AEM_AFR_STATE[aemAFRData->sensorState]);
  TV.set_cursor(xpos, (vgap + fvsz) * 9);
  TV.print(aemAFRData->sensorFault ? "YES" : "NO");
  TV.set_cursor(xpos, (vgap + fvsz) * 10);
  TV.print(aemAFRData->fatalError ? "YES" : "NO");
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
  
  TV.set_cursor(42, 32);
  TV.select_font(font8x8);
  TV.print(value, 2);
  TV.draw_rect(40, 30, 44, 12, 1, -1);

  if (value < graphStateMinValue) graphStateMinValue = value;
  TV.set_cursor(0, 33);
  TV.select_font(font6x8);
  TV.print(graphStateMinValue, 1);

  if (value > graphStateMaxValue) graphStateMaxValue = value;
  TV.set_cursor(103, 33);
  TV.select_font(font6x8);
  if (graphStateMaxValue < 10) TV.print(" ");
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

  TV.select_font(font6x8);
  TV.set_cursor(0, 20);
  TV.print("MIN");

  TV.set_cursor(43, 20);
  TV.print("Current");

  TV.set_cursor(109, 20);
  TV.print("MAX");

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
  graphStateMinValue = value;
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
