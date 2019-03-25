#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>

/*
 * Note to self:
 * DID YOU BURN THE BOOTLOADER?!?!
 * DID YOU SET THE FUSES PROPERLY?
 * Fuses OK (E:FE, H:D6, L:EE)
 * Low should be set to EE to reduce wakeup time
 * -U lfuse:w:0xEE:m -U hfuse:w:0xD6:m -U efuse:w:0xFE:m
 * C:\Users\Greg\AppData\Local\arduino15\packages\arduino\tools\avrdude\6.3.0-arduino14\bin\avrdude "-CC:\Users\Greg\AppData\Local\arduino15\packages\ATTinyCore\hardware\avr\1.2.1/avrdude.conf" -v -pattiny841 -cusbtiny -U lfuse:w:0xEE:m -U hfuse:w:0xD6:m -U efuse:w:0xFE:m
 * 
 * MAKE SURE to check for over 100% memory usage after compiling!
 */


#define PIN_EN        A0   // PA0, Pin 13, Arduino 10
#define PIN_FSK       A1   // PA1, Pin 12, Arduino 9
#define PIN_ASK       A2   // PA2, Pin 11, Arduino 8

#define FSKLOW        (bitSet(PORTA, 1))
#define FSKHIGH       (bitClear(PORTA, 1))
#define FSKTOGGLE     (PORTA = PORTA ^ _BV(1))

#define PACKETSIZE    144

#define PACKET_DELAY  25   // Milliseconds between packets
#define WAKEUPCOUNT   16     // How make 8-second wakeups before transmit

const char PROGMEM packetOne[] = "111111001100110101010101010101010100101010110100110101001011001010101101010100101010101101010100110011010011010010101011010101001100110100101100";
const char PROGMEM packetTwo[] = "111111001011001100101011001100101010101101001011001011010101010101001010101011010101010100110100110011001010101101010101001100101010101010101100";
const char PROGMEM packetFour[] = "111111001100110010101010101011010010101010110100110101001011001101010010101011010101010101010100110011001100101101010101010011001100110101001100";
const char PROGMEM packetThree[] = "111111001011001010110011010010101100110101001011001010110100101101001101010100101010101011010100110011001011010010101011010101001101010010101100";

// Allocate the memory
char currentPacket[] = "111111001100110010101010101011010010101010110100110101001011001101010010101011010101010101010100110011001100101101010101010011001100110101001100";

volatile unsigned int currentPos;
volatile bool transmitting = false;
volatile int wakeupCounter = 0;


// Interrupt for 10khz for an 8MHz clock
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
  interrupts();
}

// Interrupt for 10khz for a 16MHz clock
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
  wakeupCounter = 254;  // This will force a transmit when powered on, helps with debug
}


ISR(TIMER1_COMPA_vect)
{
  if (transmitting)
  {
    if (currentPacket[currentPos] == '1')
      FSKHIGH;
    else
      FSKLOW;

    if (++currentPos > PACKETSIZE)
    {
      transmitting = false;
      disableTX();
    }
  }

}

ISR(WDT_vect)
{
  wakeupCounter++;
  wdt_disable();  // disable watchdog
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
  
  // clear various "reset" flags
  MCUSR = 0;
  
  // allow changes, disable reset
  WDTCSR = bit(WDE);
  
  // set interrupt mode and an interval 
  WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0);    // set WDIE, and 8 seconds delay
  wdt_reset();  // pat the dog

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();           // timed sequence follows
  sleep_enable();
  interrupts();             // guarantees next instruction executed
  sleep_cpu();
}

void sendPacket(const char *thePacket)
{
  memcpy_P(currentPacket, thePacket, PACKETSIZE);

  currentPos = 0;

  enableTX();
  transmitting = true;
}

void loop()
{
  if (wakeupCounter > WAKEUPCOUNT)
  {
    wakeupCounter = 0;

    sendPacket((char *)packetOne);
    while (transmitting)
      __asm__("nop\n\t");
    delay(PACKET_DELAY);

    sendPacket(packetTwo);
    while (transmitting)
      __asm__("nop\n\t");
    delay(PACKET_DELAY);

    sendPacket(packetThree);
    while (transmitting)
      __asm__("nop\n\t");
    delay(PACKET_DELAY);

    sendPacket(packetFour);
    while (transmitting)
      __asm__("nop\n\t");
    delay(PACKET_DELAY);
  }

  sleepyTime();

}

void enableTX()
{
  FSKLOW;
  digitalWrite(PIN_ASK, HIGH);
  digitalWrite(PIN_EN, HIGH);
  delay(1); //1mS to wake up
}

void disableTX()
{
  FSKLOW;
  digitalWrite(PIN_ASK, LOW);
  digitalWrite(PIN_EN, LOW);

}