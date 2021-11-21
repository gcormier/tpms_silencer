#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include "tpms_silencer.h"

#define PIN_EN A0  // PA0, Pin 13, Arduino 10
#define PIN_FSK A1 // PA1, Pin 12, Arduino 9
#define PIN_ASK A2 // PA2, Pin 11, Arduino 8
#define PIN_BUTTON A7 // PA7, Pin 6, PCINT7
#define INTERRUPT_PIN PCINT7

#define ENABLE_WDT // disable for tx on button press only
#define BACKOFF // double transmit period everytime

#define ENHIGH (bitSet(PORTA, 0))
#define ENLOW (bitClear(PORTA, 0))

// Tie ASK and FSK signals together to drive high stronger/faster?
// cut ASK link between attiny and MICRF112.
// bodge ASK to EN on MICRF112. and ASK/FSK together on the attiny
//#define FSK_MOD

#ifdef FSK_MOD
// ASK is tied high already
#define ASKHIGH {}
#define ASKLOW {}

// reversed for some reason
#define FSKHIGH (PORTA &= ~(0x6))
#define FSKLOW (PORTA |= 0x6)
#define FSKTOGGLE (PORTA = PORTA ^ _BV(0x6))

#else

#define ASKHIGH (bitSet(PORTA, 2))
#define ASKLOW (bitClear(PORTA, 2))

#define FSKHIGH (bitSet(PORTA, 1))
#define FSKLOW (bitClear(PORTA, 1))
#define FSKTOGGLE (PORTA = PORTA ^ _BV(1))

#endif

#define PACKETSIZE 144

#define PACKET_DELAY 65 // Milliseconds period for packet tx
#ifdef BACKOFF
#define LIMIT_START   1 // (re-transmit intervals: 14s, 28s, 56s, ..., MAX_LIMIT/8)
#define MAX_LIMIT     160 // 80 max limit = 10mins
#else
#define LIMIT_START   8 // ~60s = 8*(7s) wakeups before transmit
#endif

/*
Each symbol 0/1 is 100us long. sent at a rate of 10kHz
144 symbols sent = 14.4ms long packets (+ 300us warmup)
Each packet sent 65ms appart

time      : 2021-10-22 18:46:04
model     : PMV-107J     type      : TPMS          id        : 07698722
status    : 16           battery_ok: 1             counter   : 2             failed    : OK            pressure_kPa: 228.160     temperature_C: 13.000     mic       : CRC
pulse_demod_pcm(): PMV-107J (Toyota) TPMS
bitbuffer:: Number of rows: 1 
[00] {143} fc b3 4d 2b 2a b3 52 ad 4a ad 54 ab 32 ca cb 4d 2c ac 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
time      : 2021-10-22 18:46:13
model     : PMV-107J     type      : TPMS          id        : 0805a722
status    : 24           battery_ok: 1             counter   : 3             failed    : OK            pressure_kPa: 220.720     temperature_C: 13.000     mic       : CRC
pulse_demod_pcm(): PMV-107J (Toyota) TPMS
bitbuffer:: Number of rows: 1 
[00] {143} fc d5 55 4b 2d 4c ad 52 b2 ad 55 4b 33 2a cb 4c cd 4c 
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
time      : 2021-10-22 18:46:14
model     : PMV-107J     type      : TPMS          id        : 0f536702
status    : 16           battery_ok: 1             counter   : 2             failed    : OK            pressure_kPa: 218.240     temperature_C: 13.000     mic       : CRC
pulse_demod_pcm(): PMV-107J (Toyota) TPMS
bitbuffer:: Number of rows: 1 
[00] {144} fc cc b4 ac b2 b3 55 52 b5 52 aa ab 33 35 34 b3 53 2c 
13C = 53 = 0x35 = 0 0 1 1 0 1 0 1
1111110 01100110010110100101011001011001010110011010101010101001010110101010100101010101010101011001100110011010100110100101100110101001100101100
        0 0 0 0 0 1 0 1 0 1 1 0 0 1 0 0 1 1 0 0 0 1 1 1 1 1 1 0 1 1 0 1 1 1 1 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 0 0 1 0 1 0 0 0 1 1 0 0 0 1 0 0
                 |       |       |       |       |       |       |       |       |       |       |       |       |       |       |       |       |
                0       a       c       4       c       7       e       d       e       f       f       8       1       9       4       6        
_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
time      : 2021-10-22 18:46:14
model     : PMV-107J     type      : TPMS          id        : 00a89722
status    : 8            battery_ok: 1             counter   : 1             failed    : OK            pressure_kPa: 230.640     temperature_C: 14.000     mic       : CRC
pulse_demod_pcm(): PMV-107J (Toyota) TPMS
bitbuffer:: Number of rows: 1 
[00] {144} fc aa d2 d5 2b 4c ad 52 ad 52 ab 4b 32 d5 34 d5 2b 2c
*/
/* check diff manchester:
apparently:
          FC CD 55 55 4A B4 D4 B2 AD 52 AB 54 CD 34 AB 54 CD 2C
"1 111110 01 10 011010101010101010101010 0101010110100110101001011001010101101010100101010101101010100110011010011010010101011010101001100110100101100";
          0  0  0 0 1 1 1 1 1 1 1 1 1 1  0 1 1 1 0 1 0 0 1 1 0 1 0 0 1 1 1 0 1 1 1 0 1 1 1 1 0 1 1 1 0 0 0 0 1 0 0 1 0 1 1 1 0 1 1 1 0 0 0 0 1 0 1 0 0

*/


// use the compact encoding reported by rtl_433 -vv  <-f 315M -R 110> 
// Could also store the Diff Manchester decoded values.
// Or generate whole packets on the fly!
const unsigned char packets[4][18] = { { 0xfc, 0xb3, 0x4d, 0x2b, 0x2a, 0xb3, 0x52, 0xad, 0x4a, 0xad, 0x54, 0xab, 0x32, 0xca, 0xcb, 0x4d, 0x2c, 0xac},
                              { 0xfc, 0xd5, 0x55, 0x4b, 0x2d, 0x4c, 0xad, 0x52, 0xb2, 0xad, 0x55, 0x4b, 0x33, 0x2a, 0xcb, 0x4c, 0xcd, 0x4c},
                              { 0xfc, 0xcc, 0xb4, 0xac, 0xb2, 0xb3, 0x55, 0x52, 0xb5, 0x52, 0xaa, 0xab, 0x33, 0x35, 0x34, 0xb3, 0x53, 0x2c},
                              { 0xfc, 0xaa, 0xd2, 0xd5, 0x2b, 0x4c, 0xad, 0x52, 0xad, 0x52, 0xab, 0x4b, 0x32, 0xd5, 0x34, 0xd5, 0x2b, 0x2c} };
volatile unsigned char packet=0;
volatile unsigned int currentBit;

volatile bool transmitting = false;

volatile unsigned int wakeupCounter = 0;
volatile unsigned int wakeuplimit = LIMIT_START;  // How many 8-second wakeups before transmit


// 10khz Interrupt for an 8MHz clock
void setupInterrupt8()
{
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 10000 Hz (8000000/((99+1)*8))
  OCR1A = 99;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 8
  TCCR1B |= (1 << CS11);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  // button interrupt
  PCMSK0 |= (1 << INTERRUPT_PIN);
  GIMSK |= (1 << PCIE0 );

  interrupts();
}

// 10khz Interrupt for a 16MHz clock
void setupInterrupt16()
{
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 10000 Hz (16000000/((24+1)*64))
  OCR1A = 24;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  // button interrupt
  PCMSK0 |= (1 << INTERRUPT_PIN);
  GIMSK |= (1 << PCIE0 );

  interrupts();
}

void setup()
{
  for (byte i = 0; i < 13; i++)
    pinMode(i, INPUT);

  // disable ADC + other stuff that uses power
  ADCSRA = 0;
  power_spi_disable();
  power_usart0_disable();
  power_timer2_disable();
  power_twi_disable();

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FSK, OUTPUT);
  pinMode(PIN_ASK, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  
#ifdef ENABLE_WDT
  /* Clear the reset flags. */
  MCUSR = 0;

  /* set watchdog interrupt with prescalers */
  WDTCSR = 1<<WDP0 | 1<<WDP3 | 1<<WDIE | 1<<WDE; /* 8.0 seconds */ 
#endif


#if F_CPU == 16000000L
  setupInterrupt16();
#elif F_CPU == 8000000L
  setupInterrupt8();
#else
#error CPU is not set to 16MHz or 8MHz!
#endif
  disableTX();

  wakeupCounter = wakeuplimit; // force an immediate transmit
}

// 100 usec timer for bit tx
ISR(TIMER1_COMPA_vect)
{
  if (transmitting)
  {
    if (currentBit >= PACKETSIZE)
    {
      disableTX();
      return;
    }

    // read off bits in bigendian order
    if ( ( packets[packet][currentBit/8] & (0x80>>(currentBit % 8)) ) ) 
      FSKHIGH;
    else
      FSKLOW;
    currentBit++;
  }
}

#ifdef ENABLE_WDT
// watchdog timer is setup by setupInterrupt8(). roughly 7-8s at 3v.
ISR(WDT_vect)
{
  // ensure next watchdog uses the interrupt too
  WDTCSR |= bit(WDIE);
  wakeupCounter++;
}
#endif

// button interrupt is setup by setupInterrupt8()
ISR(PCINT0_vect)
{
  if( digitalRead(PIN_BUTTON) == LOW ) {
    // button press. force wakeup and initial retransmit period
    wakeuplimit = LIMIT_START;
    wakeupCounter = wakeuplimit;
  //} else {
    // button release
  }
}

void sleepyTime()
{
  // Just in case
  disableTX();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();
  sleep_mode();
  sleep_disable();
}

void sendPacket(unsigned int which)
{
  transmitting = false; // just in case...
  packet = which;
  currentBit = 0;

  enableTX();
}

void loop()
{
  if (wakeupCounter >= wakeuplimit)
  {
    for (int i=0;i<4;i++) {
      sendPacket(i);
      delay(PACKET_DELAY);
    }

    // reset to wait for next tx
    wakeupCounter = 0;

#ifdef BACKOFF
    // double the wakeuplimit each time to back-off and save battery
    wakeuplimit *= 2;
#ifdef MAX_LIMIT
    // up to this limit
    wakeuplimit = min(wakeuplimit, MAX_LIMIT);
#endif
#endif
  }

  sleepyTime();
}

void enableTX()
{
  FSKLOW;
  ASKHIGH;
  ENHIGH;

  delayMicroseconds(300); // 300uS to wake up (similar to captured tpms)
  transmitting = true;
  power_timer1_enable();
}

void disableTX()
{
  ENLOW;
  FSKLOW;
  transmitting = false;
  power_timer1_disable();
}