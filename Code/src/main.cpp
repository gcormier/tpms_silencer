#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include "tpms_silencer.h"
/*
 * MAKE SURE to check for over 100% memory usage after compiling!
 */

#define PIN_EN A0  // PA0, Pin 13, Arduino 10
#define PIN_FSK A1 // PA1, Pin 12, Arduino 9
#define PIN_ASK A2 // PA2, Pin 11, Arduino 8
#define PIN_BUTTON A7 // PA7, Pin 6, PCINT7
#define INTERRUPT_PIN PCINT7

#define ENABLE_WDT // continually resets? if disabled!

// Tie ASK and FSK signals together to drive high stronger/faster?
// cut ASK link between attiny and MICRF112.
// bodge ASK to EN on MICRF112. and ASK/FSK together on the attiny
#define FSK_MOD

#define ENHIGH (bitSet(PORTA, 0))
#define ENLOW (bitClear(PORTA, 0))

#ifdef FSK_MOD
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

#define PACKET_DELAY 50 // Milliseconds period for packet tx
#define LIMIT_START   1 // How many 8-second wakeups before transmit

/*
Each symbol 0/1 is 100us long. sent at a rate of 10kHz
144 symbols sent = 14.4ms long packets
Each packet sent 40.85/40.9ms appart
Notes. currently retransmits every 2:26s (when wakeupCounter > wakeuplimit)

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
//test const char PROGMEM packetOne[] = "111111111111110000000000000111111100000000111100001100111011001010101101010100101010101101010100110011010011010010101011010101001100110100101100";
//const char PROGMEM packetOne[] = "111111001100110101010101010101010100101010110100110101001011001010101101010100101010101101010100110011010011010010101011010101001100110100101100";
//const char PROGMEM packetTwo[] = "111111001011001100101011001100101010101101001011001011010101010101001010101011010101010100110100110011001010101101010101001100101010101010101100";
//const char PROGMEM packetFour[] = "111111001100110010101010101011010010101010110100110101001011001101010010101011010101010101010100110011001100101101010101010011001100110101001100";
//const char PROGMEM packetThree[] = "111111001011001010110011010010101100110101001011001010110100101101001101010100101010101011010100110011001011010010101011010101001101010010101100";
// mine:
// 07698722
const char PROGMEM packetOne[] = "111111001011001101001101001010110010101010110011010100101010110101001010101011010101010010101011001100101100101011001011010011010010110010101100";
const char PROGMEM packetTwo[] = "111111001101010101010101010010110010110101001100101011010101001010110010101011010101010101001011001100110010101011001011010011001100110101001100";
const char PROGMEM packetThree[] = "111111001010101011010010110101010010101101001100101011010101001010101101010100101010101101001011001100101101010100110100110101010010101100101100";
const char PROGMEM packetFour[] = "111111001100110010110100101011001011001010110011010101010101001010110101010100101010101010101011001100110011010100110100101100110101001100101100";

// Allocate the memory
// volatile
char currentPacket[] = "111111001011001101001101001010110010101010110011010100101010110101001010101011010101010010101011001100101100101011001011010011010010110010101100";

volatile unsigned int currentPos;
volatile bool transmitting = false;
volatile unsigned int wakeupCounter = 0;
volatile unsigned int wakeuplimit = LIMIT_START;  // How many 8-second wakeups before transmit

// static code to execute earlier that setup/loop
class initcode {
public:
  initcode();
};

initcode::initcode()
{
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_ASK, OUTPUT);
  ENLOW;
  ASKHIGH;
}
initcode soft;

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
  pinMode(PIN_BUTTON, INPUT_PULLUP);
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
  interrupts();
}

void setup()
{
  // TODO: Unsure if INPUT or INPUT_PULLUP gives lowest power consumption.
  for (byte i = 0; i < 13; i++)
    pinMode(i, INPUT);

  // disable ADC
  ADCSRA = 0;

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FSK, OUTPUT);
  pinMode(PIN_ASK, OUTPUT);

  disableTX();

#if F_CPU == 16000000L
  setupInterrupt16();
#elif F_CPU == 8000000L
  setupInterrupt8();
#else
#error CPU is not set to 16MHz or 8MHz!
#endif
  wakeupCounter = wakeuplimit; // This will force a transmit soonish after powered on, helps with debug
}

ISR(TIMER1_COMPA_vect)
{
  if (transmitting)
  {
    if (currentPos >= PACKETSIZE)
    {
      disableTX();
      return;
    }

    if (currentPacket[currentPos] == '1')
      FSKHIGH;
    else
      FSKLOW;
    currentPos++;
  }
}

#ifdef ENABLE_WDT
// watchdog timer is setup by sleepytime(). should be every 8s.
ISR(WDT_vect)
{
  wakeupCounter++;
  wdt_disable(); // disable watchdog
}
#endif

ISR(PCINT0_vect)
{
  if( digitalRead(PIN_BUTTON) == LOW ) {
    // button press. force wakeup
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

  // disable ADC
  ADCSRA = 0;

  power_spi_disable();
  power_usart0_disable();
  power_timer2_disable();
  power_twi_disable();

#ifdef ENABLE_WDT
  // clear various "reset" flags
  MCUSR = 0;

  // allow changes, disable reset
  WDTCSR = bit(WDE);

  // set interrupt mode and an interval
  WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0); // set WDIE, and 8 seconds delay
  wdt_reset();                                // pat the dog
#endif

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts(); // timed sequence follows
  sleep_enable();
  interrupts(); // guarantees next instruction executed
  sleep_cpu();

  //sleep_disable();
}

void sendPacket(const char *thePacket)
{
  transmitting = false; // just in case...
  memcpy_P(currentPacket, thePacket, PACKETSIZE);
  //currentPacket = thePacket;
  currentPos = 0;

  enableTX();
}

void loop()
{
  if (wakeupCounter >= wakeuplimit)
  {
    sendPacket(packetOne);
    //while (transmitting)
    //  __asm__("nop\n\t"); // why not empty {}
    delay(PACKET_DELAY); // presumably delay will mess with ISR/tx??

    //sendPacket(packetTwo);
    //while (transmitting)
    //  __asm__("nop\n\t");
    delay(PACKET_DELAY);

    //sendPacket(packetThree);
    //while (transmitting)
    //  __asm__("nop\n\t");
    delay(PACKET_DELAY);

    //sendPacket(packetFour);
    //while (transmitting)
    //  __asm__("nop\n\t");
    delay(PACKET_DELAY);

    wakeupCounter = 0;
    // double the wakeuplimit each time to back-off and save battery
    wakeuplimit *= 2;
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
}

void disableTX()
{
  ENLOW;
  FSKLOW;
  transmitting = false;
}