typedef struct aemafrdata {
  double lambda; // 0 to 96.0088 AFR
  double oxygen; // -32.768% to 32.767%
  double sysVolts; // System Volts
  double htrVolts; // Heater Volts
  bool isLSU42; // Bosch LSU4.2 Sensor Detected
  bool isLSU49; // Bosch LSU4.9 Sensor Detected
  bool isNTKLH; // NTK L#H# Sensor Detected
  bool isNTKLHA; // NTK LHA Sensor Detected
  bool htrPIDLocked; // Heater PID locked
  bool usingFreeAirCal; // Using Free-Air Cal
  bool freeAirCalRequired; // Free-Air cal required
  bool lambdaDataValid; // Lambda Data Valid
  uint8_t sensorState; // Sensor State ; 5 bit unsigned
  bool sensorFault; // Sensor Fault
  bool fatalError; // Fatal Error
  enum SensorState {
    Reset = 0,
    WarmUp = 1,
    Stabilize = 2,
    ReadNernstPump = 3,
    Equalize = 4,
    ReadRcal = 5,
    Run = 6,
    Overheat = 7,
    Overcool = 8,
    HeaterShort = 9,
    HeaterOpen = 10,
    StartFAC = 11,
    FAC = 12,
    DetectSensor = 13,
    ReadJunct = 14,
    EvapStartup = 15,
    SensorType = 16,
    PrepareToRun = 17,
    SensorSave = 18,
    NeedFAC = 19,
    Error = 20,
  };
} AemAFRData;

const char *AEM_AFR_STATE[21] = {
  "RESET ", // 0
  "WRMUP ", // 1
  "STBLZ ", // 2
  "RDNPM ", // 3
  "EQLZE ", // 4
  "RDRCL ", // 5
  "RUN   ", // 6
  "OVRHT ", // 7
  "OVRCL ", // 8
  "HTRSH ", // 9
  "HTROP ", // 10
  "STFAC ", // 11
  "FAC   ", // 12
  "DTSEN ", // 13
  "RDJNC ", // 14
  "EVSTR ", // 15
  "SNSTP ", // 16
  "PRTRN ", // 17
  "SENSV ", // 18
  "NDFAC ", // 19
  "ERROR "  // 20
};
