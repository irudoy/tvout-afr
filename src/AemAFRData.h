struct AemAFRData {
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
};