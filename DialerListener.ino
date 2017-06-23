#include <TinyWireS.h>


#define PIN_DIALER_INPUT                               1  // PB1
#define PIN_DIALER_READY                               3  // PB3
#define PIN_DIALER_ACTIVE                              4  // PB4

#define I2C_SLAVE_ADDR                              0x01 // i2c slave address (1)

#define DEBOUNCE_DELAY                                10  // 0.01 seconds, debounce time
#define T_TURNING_TIMEDOUT                           200  //  0.2 seconds
#define T_DIALING_TIMEDOUT                          2000  // 2   seconds

#define N_DIGIT                                       20  // # of digits (or char's)

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

volatile long lastDebounceTime  =                      0; // the last time the interrupt was triggered

volatile boolean dialingReady   =                  false; // Flag to indicate a dialed number is available
volatile boolean pinDialerHigh  =                  false;
volatile boolean isSleeping     =                   true; // Flag to indicate phone is in sleeping modus

volatile unsigned int tCount    =                      0;
volatile byte dialTicks         =                      0; // counted # of ticks of rotary dialer
volatile unsigned long digits   =                      0; // Should only contain digits

volatile uint8_t response[N_DIGIT+1] = "00000000000000000000"; // May contain aplha-numeric char's

// Tracks the current register pointer position
volatile byte response_position = 0;

/**
 * This is called for each read request we receive, never put more than one byte of data
 * (with TinyWireS.send) to the send-buffer when using this callback
 */
void requestEvent()
{  
  TinyWireS.send(response[response_position]);
  // Increment the reg position on each read, and loop back to zero
  response_position++;
  if (response_position >= N_DIGIT+1)
  {
    response_position = 0;
  }
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop)
 * on the bus does so be quick, set flags for long running tasks to be called from the
 * mainloop instead of running them directly,
 */
void receiveEvent(uint8_t howMany)
{
  byte command = '?';
  response_position = 0;
  memcpy(response, "                    ", N_DIGIT+1);
  
  if (howMany == 1)
  { // Only one byte commands
    command = TinyWireS.receive();
  }
  else
  {
    // Only one byte commands, so command is default
    command = '?';
  }

  if (command == 'R') // READ NUMBER
  {
    itoa(digits, response, 10);
    if (dialingReady)
      setDialingInitial();
  }
  else if (command == 'V') // READ VOLTAGE PIN
  {
    itoa(analogRead(PIN_DIALER_INPUT), response, 10);
  }
  else if (command == 'S') // READ STATUS
  {
    if (dialingReady)
    {
      memcpy(response, "dialingReady........", N_DIGIT+1);
    }
    else if (isSleeping)
    {
      memcpy(response, "isSleeping..........", N_DIGIT+1);
    }
    else
    {
      memcpy(response, "!dialingReady.......", N_DIGIT+1);
    }
  }
  else
  {
    memcpy(response, "DialerListener......", N_DIGIT+1);
  }
  
}

void setup() {

  TinyWireS.begin(I2C_SLAVE_ADDR);       // init I2C Slave mode, pins PB0 (SDA) and PB2 (SCL) are used
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);

  
  pinMode(PIN_DIALER_READY, OUTPUT);
  pinMode(PIN_DIALER_ACTIVE, OUTPUT);
  pinMode(PIN_DIALER_INPUT, INPUT);
  
  cli();
  initChangeInterrupt();                 // Init change interrupt for sensing dialer
  initTimer();                           // Init timer 0
  sei();

  setDialingInitial();
  
}

void loop()
{
  TinyWireS_stop_check();   
} // End loop ()


// Handle pin change event
ISR(PCINT0_vect)
{
  unsigned long currentTime = tCount;
    
  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY)
  { // Change event has occurred after a long time (> DEBOUNCE_DELAY),
    // so let's take this seriously and do some things ...
  
    lastDebounceTime = currentTime;
   
    // Reset timer 0
    resetTimerCounter();

    pinDialerHigh = (digitalRead(PIN_DIALER_INPUT) == HIGH);
    
    if (pinDialerHigh)
    { // change event from low to high
      if (isSleeping)
      {
        setSleepingMode(false);
      }
      else
      {
        dialTicks++;
      }
    }
  }
}

// Handle timer interrupts
ISR(TIMER0_COMPA_vect)
{
  if (!isSleeping)
  {
    if (!dialingReady)
    {
      if (tCount == T_TURNING_TIMEDOUT)
      { // Dial has stopped turning for some (short) time now
        if (pinDialerHigh)
        { // Dial stopped turning, dialed digit can be appended to number
          addDigitToNumber(dialTicks);
          dialTicks = 0;
        }
        else
        { // phone switch off, return to initial status
          setDialingInitial();
        }

      }
      else if (tCount == T_DIALING_TIMEDOUT)
      { // Dial has stopped turning and no new attempt has occurred up to now
        if (pinDialerHigh)
        {
	        if (digits != 0)
	        { // We say  dialing is ready, set PIN_DIALER_READY to signal that number can be read
            setDialingReady(true);
	        }
        }
      }
    } // end if (!dialingReady)
    else if (tCount == T_TURNING_TIMEDOUT)
    {
      setDialingInitial();
    }
  } // if (!isSleeping)

  tCount++;
}

void addDigitToNumber (byte digit)
{
  digits *= 10;
  digits += (digit%10);
}


void setSleepingMode(boolean setASleep)
{
  isSleeping = setASleep;
  digitalWrite(PIN_DIALER_ACTIVE, isSleeping? LOW: HIGH);
}

void setDialingReady(boolean setReady)
{
  dialingReady = setReady;
  digitalWrite(PIN_DIALER_READY, setReady? HIGH : LOW);
}

void setDialingInitial()
{
  dialTicks = 0;
  digits = 0;
  lastDebounceTime = 0;

  pinDialerHigh = (digitalRead(PIN_DIALER_INPUT) == HIGH);
  
  setSleepingMode(!pinDialerHigh);

  setDialingReady(false);
}

void initChangeInterrupt()
{
  GIMSK |= (1<<PCIE);    // turns on pin change interrupts
  PCMSK |= (1<<PCINT1);  // turn on interrupts on pins PB1 PIN_DIALER_INPUT
}

void initTimer ()
{
  TCCR0A = (1 << WGM01);             // Clear Timer on Compare (CTC) mode
  TCCR0B = (1 << CS01);              // div8
  OCR0A  = F_CPU/8 * 0.001 - 1;       // 1000us compare value
  TIMSK |= (1<<OCIE0A);              // if you want interrupt
}

void resetTimerCounter()
{
  // Reset counter
  tCount = 0;
}
