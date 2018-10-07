//=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
//
// ATTiny85 Anniversary Alarm
//
// Plays a melody to celebrate our wedding on 2009-08-28
//
// With power consumption and battery capacity, this
// device should be able to run for more then 10 years.
//
// Created: 2018-08-15
// By: Chris Capobianco
// For: Mina Rohanizadegan
//
//=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=

// Uncomment when *setting* the RTC date, time and alarm
// DO NOT FORGET: >>> Set Jumper To *Connect* VDD to VBAT <<<
// Comment when *not* setting the RTC
// DO NOT FORGET: >>> Set Jumper To *Disconnect* VDD to VBAT <<<
//#define SETTIME

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <TinyWireM.h>

// Pin definitions
#define ALARM_PIN           (3)
#define STATUS_LED          (4)
#define SPEAKER_PIN         (1) // Can be 1 or 4

// I2C Address of RTC
#define RTC_ADDRESS         (0x6F)
// Time Seconds (SC) Register Address
#define TIME_SEC_REG        (0x00)
// Alarm Seconds (SCA) Register Address
#define ALARM_SEC_REG       (0x0C)
// Status Register (SR) Address
#define STATUS_REG          (0x07)
// SR Value (ARST = 1 , WRTC = 1)
#define SR_VAL              (0x90)
// Interrupt Register (INT) Address
#define INTERRUPT_REG       (0x08)
// INT Value (IM = 1, ALME = 1, LPMOD = 0, F03 = F02 = F01 = 0)
#define INT_VAL             (0xC0)

// EEPROM Address
#define EEPROM_ADDRESS      (0x80)

// Number of melodies
#define NMELODIES           (3)

// Anniversary Year
volatile byte anniversaryYear;

// Notes
#define NOTE_C              (0)
#define NOTE_CS             (1)
#define NOTE_D              (2)
#define NOTE_DS             (3)
#define NOTE_E              (4)
#define NOTE_F              (5)
#define NOTE_FS             (6)
#define NOTE_G              (7)
#define NOTE_GS             (8)
#define NOTE_A              (9)
#define NOTE_AS             (10)
#define NOTE_B              (11)

// Cater for 16 MHz, 8 MHz, or 1 MHz clock
const int Clock = ((F_CPU/1000000UL) == 16) ? 4 : ((F_CPU/1000000UL) == 8) ? 3 : 0;
const uint8_t scale[] PROGMEM = {239,226,213,201,190,179,169,160,151,142,134,127};

// Poor man's GUI
void blinkLED(byte led, byte times) {
  for (byte i = 0; i < times; i++) {
    digitalWrite(led, HIGH);
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
  }
}

void setup() {
  // Set all pins as INPUT and set to LOW
  for (byte i = 0; i <= 8; i++) {
    pinMode(i, INPUT); 
    digitalWrite(i, LOW);
  }

  // Set alarm interrupt pin and speaker pin
  pinMode(ALARM_PIN, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);

  // Blink LED so we know setup has completed
  blinkLED(SPEAKER_PIN, 3);

#ifdef SETTIME
  // Enable LED for debugging purposes
  //pinMode(SPEAKER_PIN, OUTPUT);

  // Initialize I2C lib
  TinyWireM.begin();
  delay(100);

  // Communicate with RTC via I2C
  TinyWireM.beginTransmission(RTC_ADDRESS);
  TinyWireM.send(STATUS_REG); // Status Register (SR) Address
  TinyWireM.send(SR_VAL);     // SR value (ARST = 1, WRTC = 1)
  TinyWireM.send(INT_VAL);    // INT value (IM = 1, ALME = 1, LPMOD = 0, F03 = F02 = F01 = 0)
  byte err = TinyWireM.endTransmission();
  if (err != 0x0) blinkLED(SPEAKER_PIN, 10);

  // Set Current Date and Time
  // NB: Allow for time to flash and move the IC (e.g. 2 - 3 minutes)
  byte yearValue = 18;
  byte monthValue = 9;
  byte dateValue = 2;
  byte hourValue = 2;
  byte minuteValue = 31;
  byte secondValue = 0;
  byte periodValue = 0; // AM = 0, PM = 1

  // Write to Time Register
  TinyWireM.beginTransmission(RTC_ADDRESS);
  TinyWireM.send(TIME_SEC_REG);          // Starting address of time register
  TinyWireM.send(decToBcd(secondValue)); // Convert the dec value to BCD and send
  TinyWireM.send(decToBcd(minuteValue));

  // Convert to BCD
  hourValue = decToBcd(hourValue);
  if(periodValue == 1) {
    hourValue |= B00100000;
  } else {
    hourValue &= B00011111;
  }

  // Write the modified hour value with AM/PM
  TinyWireM.send(hourValue);
  TinyWireM.send(decToBcd(dateValue));
  TinyWireM.send(decToBcd(monthValue));
  TinyWireM.send(decToBcd(yearValue));
  TinyWireM.endTransmission();

  // Set Alarm Date and Time
  byte monthEnabled = 1;
  monthValue = 8; // August = 8
  byte dateEnabled = 1;
  dateValue = 28;
  byte hourEnabled = 1;
  hourValue = 9;
  byte minuteEnabled = 1;
  minuteValue = 0;
  byte secondEnabled = 0;
  secondValue = 0;
  periodValue = 0; // AM = 0, PM = 1

  // Write to Alarm Register
  // NB: The OR operation is required to enable
  // the individual alarm registers
  TinyWireM.beginTransmission(RTC_ADDRESS);
  TinyWireM.send(ALARM_SEC_REG);
  if (secondEnabled == 1) {
    TinyWireM.send((B10000000 | decToBcd(secondValue)));
  } else {
    TinyWireM.send(decToBcd(secondValue));
  }
  if (minuteEnabled == 1) {
    TinyWireM.send((B10000000 | decToBcd(minuteValue)));
  } else {
    TinyWireM.send(decToBcd(minuteValue));
  }

  // Convert to BCD and correct for AM/PM
  hourValue = decToBcd(hourValue);
  if(periodValue == 1) {
    hourValue |= B00100000;
  } else {
    hourValue &= B00011111;
  }

  // Write the modified hour value
  if (hourEnabled == 1) {
    TinyWireM.send((B10000000 | hourValue));
  } else {
    TinyWireM.send(hourValue);
  }
  if (dateEnabled == 1) {
    TinyWireM.send((B10000000 | decToBcd(dateValue)));
  } else {
    TinyWireM.send(decToBcd(dateValue));
  }
  if (monthEnabled == 1) {
    TinyWireM.send((B10000000 | decToBcd(monthValue)));
  } else {
    TinyWireM.send(decToBcd(monthValue));
  }
  TinyWireM.endTransmission();

  // Blink LED so we know setup has completed
  blinkLED(SPEAKER_PIN, 5);
  digitalWrite(SPEAKER_PIN, LOW);
#endif
}

// Put into deep sleep, and only wake when ALARM_PIN is brought LOW
void deep_sleep() {
  // Use PB3 (Pin 3) as interrupt pin
  PCMSK |= bit(PCINT3);

  // Clear any outstanding interrupts
  GIFR |= bit(PCIF);
  
  // Enable Pin Change Interrupts
  GIMSK |= bit(PCIE);

  // Turn off ADC
  ADCSRA &= ~bit(ADEN);

  // Disable all modules
  power_all_disable();

  // Choose our preferred sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Do not interrupt before we go to sleep, otherwise
  // ISR will detach interrupts and we won't wake.
  noInterrupts();

  // Clear various reset flags
  MCUSR = 0;
  
  // Disable watchdog timer
  wdt_disable();

  // Set sleep enable (SE) bit
  sleep_enable();

  // Turn off brown-out enable in software
  //MCUCR &= ~(bit(BODS) | bit(BODSE));
  //MCUCR = bit(BODS);

  // Turn off brown-out enable in software
  // If the MCU does not have BOD disable capability,
  // this code will not have an effect
  byte mcucr1 = MCUCR | bit(BODS) | bit(BODSE);
  byte mcucr2 = mcucr1 & ~bit(BODSE);
  MCUCR = mcucr1;
  MCUCR = mcucr2;

  // Re-enable interrupts and timers
  interrupts();

  // Go to sleep
  sleep_mode();

  //--------------------------------------------------//
  // Upon waking up, sketch continues from this point //
  //--------------------------------------------------//

  // Disable sleep mode
  sleep_disable();

  // Enable all modules
  power_all_enable();

  // Disable interrupts and timers
  noInterrupts();

  // Turn off PB3 as interrupt pin
  PCMSK &= ~bit(PCINT3);

  // Turn on ADC
  ADCSRA |= bit(ADEN);

  // Re-enable interrupts and timers
  interrupts();
}

// Converts the BCD read from RTC register
// to DEC for transmission
byte bcdToDec(byte val) {
  return ((val/16*10) + (val%16));
}

// Converts the DEC input values to BCD in order
// to update the RTC register
byte decToBcd(byte val) {
  return ((val/10*16) + (val%10));
}

// This is called when the interrupt occurs,
// and we clear the GIMSK register to diable
// all the pin change interrupts.
ISR(PCINT0_vect) {
  GIMSK = 0;
}

// Play note in given octave
void note(int n, int octave) {
  int prescaler = 8 + Clock - (octave + n/12);
  if (prescaler < 1 || prescaler > 15 || octave == 0) prescaler = 0;
  DDRB = (DDRB & ~(1 << SPEAKER_PIN)) | (prescaler != 0) << SPEAKER_PIN;
  OCR1C = pgm_read_byte(&scale[n % 12]) - 1;
  GTCCR = (SPEAKER_PIN == 4) << COM1B0;
  TCCR1 = 1 << CTC1 | (SPEAKER_PIN == 1) << COM1A0 | prescaler << CS10;
}

// Compute delay duration based on BPM, note and whether staccato or not
int calculateDuration(int beatEvery, int noteDuration, bool isDotted) {
  int duration = (beatEvery * 4) / noteDuration;
  int prolonged = isDotted ? (duration / 2) : 0;
  return duration + prolonged;
}

// MusicBox Dancer
void melody1() {
  int bpm = 140;
  int beat_every = 60000 / bpm;
  int dur2 = calculateDuration(beat_every, 2, false);
  int dur4 = calculateDuration(beat_every, 4, false);
  int dur8 = calculateDuration(beat_every, 8, false);
  int dur16 = calculateDuration(beat_every, 16, false);

  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_G, 5); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_E, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_E, 6); delay(dur8); note(0, 0);
  note(NOTE_G, 5); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 7); delay(dur8); note(0, 0);
  note(NOTE_B, 6); delay(dur8); note(0, 0);
  note(NOTE_A, 6); delay(dur8); note(0, 0);
  note(NOTE_G, 5); delay(dur4); note(0, 0);
  delay(dur8);
  note(NOTE_G, 6); delay(dur8); note(0, 0);
  note(NOTE_F, 6); delay(dur8); note(0, 0);
  note(NOTE_D, 6); delay(dur8); note(0, 0);
  note(NOTE_B, 5); delay(dur8); note(0, 0);
  note(NOTE_G, 5); delay(dur8); note(0, 0);
  note(NOTE_B, 5); delay(dur8); note(0, 0);
  note(NOTE_D, 6); delay(dur8); note(0, 0);
  note(NOTE_F, 6); delay(dur8); note(0, 0);
  note(NOTE_E, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_A, 6); delay(dur8); note(0, 0);
  note(NOTE_G, 6); delay(dur4); note(0, 0);
  delay(dur8);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_G, 5); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_E, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_E, 6); delay(dur8); note(0, 0);
  note(NOTE_G, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 7); delay(dur8); note(0, 0);
  note(NOTE_B, 6); delay(dur8); note(0, 0);
  note(NOTE_A, 6); delay(dur8); note(0, 0);
  note(NOTE_G, 6); delay(dur4); note(0, 0);
  delay(dur8);
  note(NOTE_G, 6); delay(dur8); note(0, 0);
  note(NOTE_F, 6); delay(dur8); note(0, 0);
  note(NOTE_D, 6); delay(dur8); note(0, 0);
  note(NOTE_B, 5); delay(dur8); note(0, 0);
  note(NOTE_G, 5); delay(dur8); note(0, 0);
  note(NOTE_B, 5); delay(dur8); note(0, 0);
  note(NOTE_D, 6); delay(dur8); note(0, 0);
  note(NOTE_B, 5); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_G, 5); delay(dur8); note(0, 0);
  note(NOTE_E, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur2); note(0, 0);
}

// Ravel's Bolero
void melody2() {
  int bpm = 80;
  int beat_every = 60000 / bpm;
  int dur2 = calculateDuration(beat_every, 2, false);
  int dur4 = calculateDuration(beat_every, 4, false);
  int dur8 = calculateDuration(beat_every, 8, false);
  int dur16 = calculateDuration(beat_every, 16, false);

  note(NOTE_C, 6); delay(dur4); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_B, 5); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_D, 6); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_B, 5); delay(dur16); note(0, 0);
  note(NOTE_A, 5); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_A, 5); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur4); note(0, 0);
  note(NOTE_C, 6); delay(dur8); note(0, 0);
  note(NOTE_B, 5); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_A, 5); delay(dur16); note(0, 0);
  note(NOTE_G, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 5); delay(dur16); note(0, 0);
  note(NOTE_F, 5); delay(dur16); note(0, 0);
  note(NOTE_G, 5); delay(dur2); note(0, 0);
  note(NOTE_G, 5); delay(dur16); note(0, 0);
  note(NOTE_F, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 5); delay(dur16); note(0, 0);
  note(NOTE_D, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 5); delay(dur16); note(0, 0);
  note(NOTE_F, 5); delay(dur16); note(0, 0);
  note(NOTE_G, 5); delay(dur16); note(0, 0);
  note(NOTE_A, 5); delay(dur16); note(0, 0);
  note(NOTE_G, 5); delay(dur4); note(0, 0);
  note(NOTE_G, 5); delay(dur4); note(0, 0);
  note(NOTE_G, 5); delay(dur16); note(0, 0);
  note(NOTE_A, 5); delay(dur16); note(0, 0);
  note(NOTE_B, 5); delay(dur16); note(0, 0);
  note(NOTE_A, 5); delay(dur16); note(0, 0);
  note(NOTE_G, 5); delay(dur16); note(0, 0);
  note(NOTE_F, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 5); delay(dur16); note(0, 0);
  note(NOTE_D, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 5); delay(dur16); note(0, 0);
  note(NOTE_D, 5); delay(dur16); note(0, 0);
  note(NOTE_C, 5); delay(dur8); note(0, 0);
  note(NOTE_C, 5); delay(dur8); note(0, 0);
  note(NOTE_C, 5); delay(dur16); note(0, 0);
  note(NOTE_D, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 5); delay(dur8); note(0, 0);
  note(NOTE_F, 5); delay(dur8); note(0, 0);
  note(NOTE_D, 5); delay(dur4); note(0, 0);
  note(NOTE_G, 5); delay(dur2); note(0, 0);
}

// Tchaikovsky's Nutcracker Suite
void melody3() {
  int bpm = 70;
  int beat_every = 60000 / bpm;
  int dur2 = calculateDuration(beat_every, 2, false);
  int dur4 = calculateDuration(beat_every, 4, false);
  int dur4dot = calculateDuration(beat_every, 4, true);
  int dur8 = calculateDuration(beat_every, 8, false);
  int dur8dot = calculateDuration(beat_every, 8, true);
  int dur16 = calculateDuration(beat_every, 16, false);
  int dur16dot = calculateDuration(beat_every, 16, true);
  int dur32 = calculateDuration(beat_every, 32, false);

  note(NOTE_G, 6); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_G, 6); delay(dur8); note(0, 0);
  delay(dur16);
  note(NOTE_FS, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_DS, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_E, 6); delay(dur16dot); note(0, 0);
  delay(dur16);
  note(NOTE_D, 6); delay(dur16); note(0, 0);
  note(NOTE_D, 6); delay(dur16); note(0, 0);
  note(NOTE_D, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_CS, 6); delay(dur16); note(0, 0);
  note(NOTE_CS, 6); delay(dur16); note(0, 0);
  note(NOTE_CS, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_B, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_B, 5); delay(dur16); note(0, 0);
  delay(dur8dot);
  note(NOTE_G, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 5); delay(dur16); note(0, 0);
  note(NOTE_G, 5); delay(dur8); note(0, 0);
  delay(dur16);
  note(NOTE_FS, 5); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_B, 5); delay(dur8); note(0, 0);
  delay(dur16);
  note(NOTE_G, 6); delay(dur16); note(0, 0);
  note(NOTE_G, 6); delay(dur16); note(0, 0);
  note(NOTE_G, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_FS, 6); delay(dur16); note(0, 0);
  note(NOTE_FS, 6); delay(dur16); note(0, 0);
  note(NOTE_FS, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_DS, 6); delay(dur16); note(0, 0);
  note(NOTE_FS, 6); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_FS, 6); delay(dur16); note(0, 0);
  note(NOTE_DS, 6); delay(dur16); note(0, 0);
  delay(dur4dot);
  note(NOTE_G, 6); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_G, 6); delay(dur16dot); note(0, 0);
  delay(dur32);
  note(NOTE_FS, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_DS, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_D, 6); delay(dur16); note(0, 0);
  note(NOTE_D, 6); delay(dur16); note(0, 0);
  note(NOTE_D, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_CS, 6); delay(dur16); note(0, 0);
  note(NOTE_CS, 6); delay(dur16); note(0, 0);
  note(NOTE_CS, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  delay(dur16);
  note(NOTE_B, 5); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_C, 6); delay(dur16); note(0, 0);
  note(NOTE_E, 6); delay(dur16); note(0, 0);
  note(NOTE_B, 5); delay(dur16); note(0, 0);
  delay(dur2);
}

void loop() {
#ifdef SETTIME
  digitalWrite(STATUS_LED, LOW);

  // Set anniversaryYear in EEPROM
  EEPROM.update(EEPROM_ADDRESS, 10);
  delay(10);
#endif

  // Put ATTiny85 into deep sleep
  deep_sleep();

  // Retrieve anniversaryYear from EEPROM
  anniversaryYear = EEPROM.read(EEPROM_ADDRESS);
  if (anniversaryYear <= 0 || anniversaryYear >= 255) anniversaryYear = 10;

  if (anniversaryYear % NMELODIES == 0) {
    // Play Melody #1
    melody1();
  } else if (anniversaryYear % NMELODIES == 1) {
    // Play Melody #2
    melody2();
  } else {
    // Play Melody #3
    melody3();
  }

  // Count out number of anniversaries
  delay(250);
  for (byte i = 0; i < anniversaryYear; i++) {
    note(NOTE_C, 6); delay(250); note(0, 0);
    delay(400);
  }

  // Update anniversaryYear in EEPROM
  EEPROM.update(EEPROM_ADDRESS, ++anniversaryYear);
  delay(10);
}
