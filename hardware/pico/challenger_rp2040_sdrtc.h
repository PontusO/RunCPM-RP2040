#ifndef CHALLENGER_RP2040_SDRTC_H
#define CHALLENGER_RP2040_SDRTC_H

// =========================================================================================
// Tell the system that we have a hw port emualation layer
// =========================================================================================
#define HW_PORTS_EMULATION_ENABLED      1
// =========================================================================================
// Define SdFat as alias for SD
// =========================================================================================
SdFat SD;

// =========================================================================================
// Define Board-Data
// GP25 green onboard LED
// =========================================================================================
#define LED         PIN_LED  // GPIO25
#define LEDinv 0
#define board_pico
#define board_analog_io
#define board_digital_io
#define BOARD "Challenger RP2040 SD/RTC"

#define HW_PORTS_EMU_VERSION    "0.2.1"

// =========================================================================================
// Pin Documentation
// =========================================================================================
// Challenger RP2040 SD/RTC (SD Card slot, default config from bsp)
// MISO - GPIO 12
// MOSI - GPIO 11
// CS   - GPIO 9
// SCK  - GPIO 10

#include <Arduino.h>
#include <Wire.h>

/**
 * GPIO Interface
 *
 * The GPIO interface registers are used to map the RP2040 pins and functions
 * to a Z80 / CP/M register space. A number of I/O locations in the Z80 IO space
 * are mapped to specific C-functions that implements the required function.
 *
 * This is the CP/M pin number map (CP/M pin numbers in paranthesis):
 *
 *                      +-------------+
 *          RST         |     USB     |    (20) LED
 *          3.3V        |             |
 *          Aref        |             |
 *          GND         |             |
 *          A0  (0#)    |             |         BAT
 *          A1  (1#)    |             |         En
 *          A2  (2#)    |             |         USB
 *          A3  (3#)    |             |    (19) D13
 *          A4  (4)     |             |    (18) D12
 *          A5  (5)     |             |    (17) D11
 *          SCK (6)     |             |    (16) D10
 *          SDO (7)     |             |    (15)  D9
 *          SDI (8)     |             |    (14)  D6
 *          RX  (9)     |             |    (13)  D5
 *          TX  (10)    |             |    (12)  SCL
 *          GND         |             |    (11)  SDA
 *                      +-------------+
 *
 * Pin numbers marked with # are analog/digital pins, all other pins are purely
 * digital.
 *
 * D I G I T A L   G P I O
 *
 * The digital gpio hw interface consists of 3 registers, the pin number
 * register (v_pin_no), the pin function register (v_pin_func) and the pin value
 * register (v_pin_val).
 *
 * All GPIO operations begin with the user first setting up on what pin they
 * want to do an operation on. This is done by writing the pin number, according
 * to the pin map diagran above, to the v_pin_no register.
 *
 * To define the function of the pin, the register v_pin_func must also be
 * set to the desired mode of operation. The number written into this register
 * must follow the following scheme:
 *   INPUT           = 0x0,
 *   OUTPUT          = 0x1,
 *   INPUT_PULLUP    = 0x2,
 *   INPUT_PULLDOWN  = 0x3,
 *   OUTPUT_2MA      = 0x4,
 *   OUTPUT_4MA      = 0x5,
 *   OUTPUT_8MA      = 0x6,
 *   OUTPUT_12MA     = 0x7,
 * Any value outside of the above mentioned will cause an undefined behaviour.
 * Reading this register will return an identifier (0xda) for this block. This
 * can be used to identify that the GPIO functions are available to the device.
 *
 * Lastly, the user can now read or write to the pin through the v_pin_val
 * register.
 *
 * Multiple read and/or write operations can be performed on the same pin by
 * simply reading and/or writing to the v_pin_val register.
 *
 * A N A L O G   I N P U T
 *
 * The RP2040 has a 12-bit SAR A/D converter that can be accessed through the
 * analog input interface.
 *
 * The analog input interface is built arround 3 different registers. These
 * registers controls what input to read from  as well as two registers that
 * holds the value of the ADC.
 *
 * When reading a value from the ADC the user must first set the desired channel
 * that he/she wants to read from. This is done by writing the channel to
 * register v_analog_chan. When this is done the analog value can the be read
 * directly from the data registers.
 *
 * Since the ADC can represent up to 12 bits, two registers are used to hold the
 * data. The high 4 bits (nibble) are read from the register v_analog_hi and the
 * 8 low bits (byte) are read from register v_analog_lo. These two value can
 * then be combined into an integer value in MSBASIC as follows.
 *
 *   350 OUT 195,1 : REM SET INPUT A1
 *   360 LET AN = INP(196) * 256 + INP(197)
 *
 * The resolution of the ADC can be adjusted by writing the desired number of
 * bits into the v_analog_hi_ctrl register. For example, if you write the value
 * 8 to this register the resolution of the adc is effectively reduce to 8 bits.
 * The default setting for the adc resolution is 10 bits.
 *
 */
#define v_GPIO_ID           0xda
#define v_IOBASE            0xc0
#define v_pin_no            (v_IOBASE + 0x00)   // 192
#define v_pin_func          (v_IOBASE + 0x01)   // 193
#define v_pin_val           (v_IOBASE + 0x02)   // 194
#define v_analog_chan       (v_IOBASE + 0x03)   // 195
#define v_analog_hi_ctrl    (v_IOBASE + 0x04)   // 196
#define v_analog_lo         (v_IOBASE + 0x05)   // 197

static const uint32_t cpmPin2RP2040[] = {
  A0,  // CPM GPIO 0 / Analog input 0
  A1,  // CPM GPIO 1 / Analog input 1
  A2,  // CPM GPIO 2 / Analog input 2
  A3,  // CPM GPIO 3 / Analog input 3
  A4,  // CPM GPIO 4
  A5,  // CPM GPIO 5
  D4,  // CPM GPIO 6
  D3,  // CPM GPIO 7
  D2,  // CPM GPIO 8
  D5,  // CPM GPIO 9
  D6,  // CPM GPIO 10
  D7,  // CPM GPIO 11
  D8,  // CPM GPIO 12
  D9,  // CPM GPIO 13
  D10, // CPM GPIO 14
  D11, // CPM GPIO 15
  D12, // CPM GPIO 16
  D13, // CPM GPIO 17
  D1,  // CPM GPIO 18
  D0,  // CPM GPIO 19
  20   // CPM GPIO 20
};
#define NUMBER_OF_CPM_PINS    21
#define INVALID_CPM_PIN       0xff

/**
 * Definition of the standard CP/M real time clock chip and mapping to the
 * challenger_rp2040_sdrtc on board MCP79410.
 * We must assume that any register can be accessed in any order, therefore
 * we must read the associated data everytime.
 *
 * RTC registers
 *
 * rtc1sec    .db	RTCBASE+00h	; 1 second digit
 * rtc10sec   .db	RTCBASE+01h	; 10 second digit
 * rtc1min	  .db	RTCBASE+02h	; 1 minute digit
 * rtc10min	  .db	RTCBASE+03h	; 10 minute digit
 * rtc1hour	  .db	RTCBASE+04h	; 1 hour digit
 * rtc10hour  .db	RTCBASE+05h	; 10 hour digit (also AM/PM indicator)
 * rtc1day		.db	RTCBASE+06h	; 1 day digit
 * rtc10day	  .db	RTCBASE+07h	; 10 day digit
 * rtc1month	.db	RTCBASE+08h	; 1 month digit
 * rtc10month	.db	RTCBASE+09h	; 10 month digit
 * rtc1year	  .db	RTCBASE+0Ah	; 1 year digit
 * rtc10year	.db	RTCBASE+0Bh	; 10 year digit
 * rtcweek		.db	RTCBASE+0Ch	; day of week
 * rtccrtlD	  .db	RTCBASE+0Dh	; control register D
 * rtcctrlE	  .db	RTCBASE+0Eh	; control register E
 * rtcctrlF	  .db	RTCBASE+0Fh	; control register F
 *
 */
#define v_RTCBASE     0xb0
#define v_rtc1sec     (v_RTCBASE + 0x00)
#define v_rtc10sec    (v_RTCBASE + 0x01)
#define v_rtc1min     (v_RTCBASE + 0x02)
#define v_rtc10min    (v_RTCBASE + 0x03)
#define v_rtc1hour    (v_RTCBASE + 0x04)
#define v_rtc10hour   (v_RTCBASE + 0x05)
#define v_rtc1day     (v_RTCBASE + 0x06)
#define v_rtc10day    (v_RTCBASE + 0x07)
#define v_rtc1month   (v_RTCBASE + 0x08)
#define v_rtc10month  (v_RTCBASE + 0x09)
#define v_rtc1year    (v_RTCBASE + 0x0a)
#define v_rtc10year   (v_RTCBASE + 0x0b)
#define v_rtcweek     (v_RTCBASE + 0x0c)
#define v_rtccrtlD    (v_RTCBASE + 0x0d)
#define v_rtccrtlE    (v_RTCBASE + 0x0e)
#define v_rtccrtlF    (v_RTCBASE + 0x0f)

/*
 * Address Register
 * Name           Bit 7   Bit 6    Bit 5    Bit 4    Bit 3    Bit 2    Bit 1    Bit 0
 * 00h RTCSEC   |    ST SECTEN2  SECTEN1  SECTEN0  SECONE3  SECONE2  SECONE1  SECONE0
 * 01h RTCMIN   |   —   MINTEN2  MINTEN1  MINTEN0  MINONE3  MINONE2  MINONE1  MINONE0
 * 02h RTCHOUR  | 12/24   AM/PM   HRTEN1   HRTEN0   HRONE3   HRONE2   HRONE1   HRONE0
 * 03h RTCWKDAY |   —       —     OSCRUN  PWRFAIL   VBATEN   WKDAY2   WKDAY1   WKDAY0
 * 04h RTCDATE  |   —       —   DATETEN1 DATETEN0 DATEONE3 DATEONE2 DATEONE1 DATEONE0
 * 05h RTCMNTH  |   —       —       LPYR  MTHTEN0  MTHONE3  MTHONE2  MTHONE1  MTHONE0
 * 06h RTCYEAR  |YRTEN3  YRTEN2   YRTEN1   YRTEN0   YRONE3   YRONE2   YRONE1   YRONE0
 * 07h RTCCTRL  |  OUT    SQWEN   ALM1EN   ALM0EN   EXTOSC  CRSTRIM   SQWFS1   SQWFS0
 * 08h ORTCOSC  | SIGN TRIMVAL6 TRIMVAL5 TRIMVAL4 TRIMVAL3 TRIMVAL2 TRIMVAL1 TRIMVAL0
 *
 *
 */
#define RTC_ADDR  0x6F
#define RTCSEC    0x00
#define RTCMIN    0x01
#define RTCHOUR   0x02
#define RTCWKDAY  0x03
#define RTCDAY    0x04
#define RTCMNTH   0x05
#define RTCYEAR   0x06
#define RTCCTRL   0x07
#define RTCOSC    0x08

// Control Register bits
#define OUT       7   // sets logic level on MFP when not used as square wave output
#define SQWE      6   // set to enable square wave output
#define ALM1      5   // alarm 1 is active
#define ALM0      4   // alarm 0 is active
#define EXTOSC    3   // set to drive the RTC registers from an external oscillator instead of a crystal
#define RS2       2   // RS2:0 set square wave output frequency: 0==1Hz, 1==4096Hz, 2==8192Hz, 3=32768Hz
#define RS1       1
#define RS0       0

// Other Control Bits
#define ST        7   // Seconds register (TIME_REG) oscillator start/stop bit, 1==Start, 0==Stop
#define HR1224    6   // Hours register (TIME_REG+2) 12 or 24 hour mode (24 hour mode==0)
#define AMPM      5   // Hours register (TIME_REG+2) AM/PM bit for 12 hour mode
#define OSCON     5   // Day register (TIME_REG+3) oscillator running (set and cleared by hardware)
#define VBAT      4   // Day register (TIME_REG+3) set by hardware when Vcc fails and RTC runs on battery.
                      // VBAT is cleared by software, clearing VBAT also clears the timestamp registers
#define VBATEN    3   // Day register (TIME_REG+3) VBATEN==1 enables backup battery, VBATEN==0 disconnects the VBAT pin (e.g. to save battery)
#define LP        5   // Month register (TIME_REG+5) leap year bit

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

static bool mcp79410_initialized = false;

/**
 * Reads one byte (register) from the RTC
 */
static uint8_t i2cRead(uint8_t reg) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  Wire.requestFrom(RTC_ADDR, 1);
  return Wire.read();
}

/**
 * Writes one byte (register) to the RTC
 */
static bool i2cWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

/**
 * Initialization of the MCP79410.
 */
static void mcp_initialize() {
  if (!mcp79410_initialized) {
    mcp79410_initialized = true;
    Wire.begin();
    delay(100);
    uint8_t tv = i2cRead(RTCSEC);
    if (!(tv & _BV(ST)))
      i2cWrite(RTCSEC, tv | _BV(ST));
  }
}

/**
 *
/**
 * Getter and setters for the hardware registers of the RTC
 */
uint8_t get_rtc1sec() {
  return i2cRead(RTCSEC) & 0x0f;
}

uint8_t get_rtc10sec() {
  return (i2cRead(RTCSEC) >> 4) & 0x07;
}

uint8_t get_rtc1min() {
  return i2cRead(RTCMIN) & 0x0f;
}

uint8_t get_rtc10min() {
  return (i2cRead(RTCMIN) >> 4) & 0x07;
}

uint8_t get_rtc1hour() {
  return i2cRead(RTCHOUR) & 0x0f;
}

uint8_t get_rtc10hour() {
  return (i2cRead(RTCHOUR) >> 4) & 0x03;
}

uint8_t get_rtc1day() {
  return i2cRead(RTCDAY) & 0x0f;
}

uint8_t get_rtc10day() {
  return (i2cRead(RTCDAY) >> 4) & 0x03;
}

uint8_t get_rtc1month() {
  return i2cRead(RTCMNTH) & 0x0f;
}

uint8_t get_rtc10month() {
  return (i2cRead(RTCMNTH) >> 4) & 0x01;
}

uint8_t get_rtc1year() {
  return i2cRead(RTCYEAR) & 0x0f;
}

uint8_t get_rtc10year() {
  return (i2cRead(RTCYEAR) >> 4);
}

uint8_t get_rtcweek() {
  return i2cRead(RTCWKDAY) & 0x07 - 1;
}

uint8_t get_rtccrtlD() {
  return 0;
}

uint8_t get_rtccrtlE() {
  return 0;
}

uint8_t get_rtccrtlF() {
  return 0;
}

//
// Setters
//
void set_rtc1sec(const uint32 Value) {
  uint8_t tv = i2cRead(RTCSEC) & 0xf0 | (Value & 0x0f);
  i2cWrite(RTCSEC, tv);
}

void set_rtc10sec(const uint32 Value) {
  uint8_t tv = i2cRead(RTCSEC) & 0x8f | (Value << 4) & 0x70;
  i2cWrite(RTCSEC, tv);
}

void set_rtc1min(const uint32 Value) {
  uint8_t tv = i2cRead(RTCMIN) & 0xf0 | (Value & 0x0f);
  i2cWrite(RTCMIN, tv);
}

void set_rtc10min(const uint32 Value) {
  uint8_t tv = i2cRead(RTCMIN) & 0x0f | (Value << 4) & 0xf0;
  i2cWrite(RTCMIN, tv);
}

void set_rtc1hour(const uint32 Value) {
  uint8_t tv = i2cRead(RTCHOUR) & 0xf0 | (Value & 0x0f);
  i2cWrite(RTCHOUR, tv);
}

void set_rtc10hour(const uint32 Value) {
  uint8_t tv = i2cRead(RTCHOUR) & 0xcf | (Value << 4) & 0x30;
  i2cWrite(RTCHOUR, tv);
}

void set_rtc1day(const uint32 Value) {
  uint8_t tv = i2cRead(RTCDAY) & 0xf0 | (Value & 0x0f);
  i2cWrite(RTCDAY, tv);
}

void set_rtc10day(const uint32 Value) {
  uint8_t tv = i2cRead(RTCDAY) & 0x0f | (Value << 4) & 0xf0;
  i2cWrite(RTCDAY, tv);
}

void set_rtc1month(const uint32 Value) {
  uint8_t tv = i2cRead(RTCMNTH) & 0xf0 | (Value & 0x0f);
  i2cWrite(RTCMNTH, tv);
}

void set_rtc10month(const uint32 Value) {
  uint8_t tv = i2cRead(RTCMNTH) & 0x2f | (Value << 4) & 0xf0;
  i2cWrite(RTCMNTH, tv);
}

void set_rtc1year(const uint32 Value) {
  uint8_t tv = i2cRead(RTCYEAR) & 0xf0 | (Value & 0x0f);
  i2cWrite(RTCYEAR, tv);
}

void set_rtc10year(const uint32 Value) {
  uint8_t tv = i2cRead(RTCYEAR) & 0x0f | (Value << 4) & 0xf0;
  i2cWrite(RTCYEAR, tv);
}

void set_rtcweek(const uint32 Value) {
  uint8_t tv = i2cRead(RTCWKDAY) & 0xf8 | ((Value + 1) & 0x07);
  i2cWrite(RTCWKDAY, tv);
}

void set_rtccrtlD(const uint32 Value) {
}

void set_rtccrtlE(const uint32 Value) {
}

void set_rtccrtlF(const uint32 Value) {
}

static uint32_t pin_no;
static uint32_t pin_val;
static uint32_t an_chan;
static uint32_t an_val;

/**
 * RTC register selector
 */
uint8_t hw_port_read(uint8_t port) {
  switch(port) {
    // RTC handling
    case v_rtc1sec:
      return get_rtc1sec();
    case v_rtc10sec:
      return get_rtc10sec();
    case v_rtc1min:
      return get_rtc1min();
    case v_rtc10min:
      return get_rtc10min();
    case v_rtc1hour:
      return get_rtc1hour();
    case v_rtc10hour:
      return get_rtc10hour();
    case v_rtc1day:
      return get_rtc1day();
    case v_rtc10day:
      return get_rtc10day();
    case v_rtc1month:
      return get_rtc1month();
    case v_rtc10month:
      return get_rtc10month();
    case v_rtc1year:
      return get_rtc1year();
    case v_rtc10year:
      return get_rtc10year();
    case v_rtcweek:
      return get_rtcweek();
    // GPIO handling
    case v_pin_no:
      return pin_no;
    case v_pin_func:
      return v_GPIO_ID;
    case v_pin_val:
      if (pin_no < NUMBER_OF_CPM_PINS)
        return digitalRead(cpmPin2RP2040[pin_no]);
      else
        return 0;
    case v_analog_chan:
      return an_chan;
    case v_analog_hi_ctrl:
      an_val = analogRead(cpmPin2RP2040[an_chan]);
      return (an_val >> 8) & 0x0f;
    case v_analog_lo:
      return an_val & 0xff;
    default:
      return 0;
  }
}

void hw_port_write(const uint32 Port, const uint32 Value) {
  switch(Port) {
    case v_rtc1sec:
      set_rtc1sec(Value);
      break;
    case v_rtc10sec:
      set_rtc10sec(Value);
      break;
    case v_rtc1min:
      set_rtc1min(Value);
      break;
    case v_rtc10min:
      set_rtc10min(Value);
      break;
    case v_rtc1hour:
      set_rtc1hour(Value);
      break;
    case v_rtc10hour:
      set_rtc10hour(Value);
      break;
    case v_rtc1day:
      set_rtc1day(Value);
      break;
    case v_rtc10day:
      set_rtc10day(Value);
      break;
    case v_rtc1month:
      set_rtc1month(Value);
      break;
    case v_rtc10month:
      set_rtc10month(Value);
      break;
    case v_rtc1year:
      set_rtc1year(Value);
      break;
    case v_rtc10year:
      set_rtc10year(Value);
      break;
    case v_rtcweek:
      set_rtcweek(Value);
      break;
    case v_pin_no:
      if (Value < NUMBER_OF_CPM_PINS)
        pin_no = Value;
      else
        pin_no = INVALID_CPM_PIN;
      break;
    case v_pin_func:
      if (pin_no < NUMBER_OF_CPM_PINS)
        pinMode(cpmPin2RP2040[pin_no], Value);
      break;
    case v_pin_val:
      if (pin_no < NUMBER_OF_CPM_PINS)
        digitalWrite(cpmPin2RP2040[pin_no], Value);
      break;
    case v_analog_chan:
      an_chan = Value;
      break;
    case v_analog_hi_ctrl:
      analogReadResolution(Value);
      break;
  }
}

void setup_hw_ports() {
  mcp_initialize();
   analogReadResolution(10);
}

#endif
