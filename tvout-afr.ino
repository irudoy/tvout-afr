#include <TVout.h>
#include <fontALL.h>
#include <EMUcan.h>

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
#define DELAY_FRAMES 3
#define GRAPH_MIN 5
#define GRAPH_MAX 25

#define DEBOUNCE_DELAY 1000

#define AEM_CAN_AFR_ID 0x00000180

#define DEBUG_GRAPH true

int lastDebounceTime = 0;

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

// DUMMY MSG FOR TEST
struct can_frame dummyCanMsg;
// DUMMY MSG FOR TEST

EMUcan emucan(0x600, CAN_SPI_CS_PIN);

TVout TV;
double minAFR = 100.0;
double maxAFR = 0.0;
double prevAFR = 0.0; // FOR DEBUG
double AFRValue = 0.0;
byte graph[117] = { 0 };

int DEBUG_direction = 1;

void setup()  {
  // TV setup
  TV.begin(PAL, 128, 96);
  renderInitialScreen("AFR", GRAPH_MIN, GRAPH_MAX);

  // Serial setup
  Serial.begin(9600);
  while (!Serial);
  Serial.println("CAN Init");

  // CAN Setup
  emucan.begin(CAN_SPEED);
  Serial.print("EMUCAN_LIB_VERSION: ");
  Serial.println(EMUCAN_LIB_VERSION);
  ReturnAllFramesFunction frameProcessFn = handleCANFrame;
  emucan.ReturnAllFrames(frameProcessFn);
  Serial.println("------- CAN Read ----------");




  // DUMMY MSG FOR TEST
  dummyCanMsg.can_id = 0x00000180;
  dummyCanMsg.can_dlc = 8;
  dummyCanMsg.data[0] = 0;
  dummyCanMsg.data[1] = 0;
  dummyCanMsg.data[2] = 0;
  dummyCanMsg.data[3] = 0;
  dummyCanMsg.data[4] = 0;
  dummyCanMsg.data[5] = 0;
  dummyCanMsg.data[6] = 0;
  dummyCanMsg.data[7] = 0;
  // DUMMY MSG FOR TEST


  

  delay(1000);
}

void handleCANFrame(const struct can_frame *frame) {
//  Serial.print("CAN ID: ");
//  Serial.print(frame->can_id, HEX); // print ID
//  Serial.print("; CAN DLC: ");
//  Serial.print(frame->can_dlc, HEX); // print DLC
//  Serial.print("; CAN DATA: ");
//  for (int i = 0; i < frame->can_dlc; i++)  { // print the data
//    Serial.print(frame->data[i], HEX);
//    Serial.print(", ");
//  }
//  Serial.println();

  if (frame->can_id == AEM_CAN_AFR_ID) {
    double lambda = ((frame->data[1] << 8) + frame->data[0]) * 0.001465; // .001465 AFR/bit ; range 0 to 96.0088 AFR
    double oxygen = ((frame->data[3] << 8) + frame->data[2]) * 0.001; // 0.001%/bit ; -32.768% to 32.767%
    double sysVolts = frame->data[4] * 0.1; // System Volts
    double htrVolts = frame->data[5] * 0.1; // Heater Volts
    bool isLSU42 = frame->data[6] & 0; // Bosch LSU4.2 Sensor Detected
    bool isLSU49 = frame->data[6] & 2; // Bosch LSU4.9 Sensor Detected
    bool isNTKLH = frame->data[6] & 4; // NTK L#H# Sensor Detected
    bool isNTKLHA = frame->data[6] & 8; // NTK LHA Sensor Detected
    bool htrPIDLocked = frame->data[6] & 16; // Heater PID locked
    bool usingFreeAirCal = frame->data[6] & 32; // Using Free-Air Cal
    bool freeAirCalRequired = frame->data[6] & 64; // Free-Air cal required
    bool lambdaDataValid = frame->data[6] & 128; // Lambda Data Valid
    uint8_t sensorState = (frame->data[7] >> 3) & ((1 << 5) - 1); // Sensor State ; 5 bit unsigned ; enum AFR_SENSOR_STATE
    bool sensorFault = frame->data[7] & 64; // Sensor Fault
    bool fatalError = frame->data[7] & 128; // Fatal Error

    if (false) {
      Serial.print("AFR: "); Serial.print(lambda); Serial.print("; ");
      Serial.print("Oxygen: "); Serial.print(oxygen); Serial.print("%; ");
      Serial.print("System Volts: "); Serial.print(lambda); Serial.print("V; ");
      Serial.print("Heater Volts: "); Serial.print(lambda); Serial.print("V; ");
      Serial.print("is LSU4.2: "); Serial.print(isLSU42); Serial.print("; ");
      Serial.print("is LSU4.9: "); Serial.print(isLSU49); Serial.print("; ");
      Serial.print("is NTK L#H#: "); Serial.print(isNTKLH); Serial.print("; ");
      Serial.print("is NTK LHA: "); Serial.print(isNTKLHA); Serial.print("; ");
      Serial.print("Heater PID locked: "); Serial.print(htrPIDLocked); Serial.print("; ");
      Serial.print("Using Free-Air Cal: "); Serial.print(usingFreeAirCal); Serial.print("; ");
      Serial.print("Free-Air cal required: "); Serial.print(freeAirCalRequired); Serial.print("; ");
      Serial.print("Lambda Data Valid: "); Serial.print(lambdaDataValid); Serial.print("; ");
      Serial.print("Sensor State: "); Serial.print(AEM_AFR_STATE[sensorState]); Serial.print("; ");
      Serial.print("Sensor Fault: "); Serial.print(sensorFault); Serial.print("; ");
      Serial.print("Fatal Error: "); Serial.print(fatalError); Serial.print("; ");
      Serial.println();
    }
  }
}

void loop() {
  emucan.checkEMUcan();

  const unsigned long tm = millis();
  if ((tm - lastDebounceTime) > DEBOUNCE_DELAY) {
    handleCANFrame(&dummyCanMsg);

    // CAN DEBUG
    uint8_t rxerrors = emucan.CanErrorCounter(false);
    uint8_t txerrors = emucan.CanErrorCounter(true);
    Serial.print("rx errs: ");
    Serial.print(rxerrors);
    Serial.print("; tx errs: ");
    Serial.print(txerrors);
    //retreive the mcp2515 object for direct access
    MCP2515 mcp = *emucan.getMcp2515();
    //call the getErrorFlags function from the mcp2515 lib:
    uint8_t eflg = mcp.getErrorFlags();
    Serial.print("; eflg register: ");
    Serial.println(eflg);
    // CAN DEBUG
    
    lastDebounceTime = tm;
  }


  if (DEBUG_GRAPH){
    double step = 0.1;
    AFRValue = DEBUG_direction == 1 ? prevAFR + step : prevAFR - step;
    if (prevAFR <= 4.0 && DEBUG_direction == 0) {
      DEBUG_direction = 1;
    } else if (prevAFR >= 26.0 && DEBUG_direction == 1) {
      DEBUG_direction = 0;
    }
    prevAFR = AFRValue;
  }
  
  display_current(AFRValue);
  renderGraph(AFRValue, GRAPH_MIN, GRAPH_MAX);

  if (AFRValue < minAFR && AFRValue != -1) {
    minAFR = AFRValue;
    display_min(minAFR);
  }

  if (AFRValue >= maxAFR) {
    maxAFR = AFRValue;
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
