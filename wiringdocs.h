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
