#include <dht.h>
#include <Stepper.h>
#include <LiquidCrystal.h>
#include <DS3231.h>

#define RDA 0x80
#define TBE 0x20 

//GLOBAL CONSTANTS
const int TEMP_THRESHOLD = 72;
const int WATER_THRESHOLD = 5;


// PINS FOR WATER SENSOR -- 1 Analog pin
const int WaterSensorPin = 1;

 
     

// PINS FOR STEPPER MOTOR -- 4 digital pins
const int StepperPin1 = 22;
const int StepperPin2 = 23;
const int StepperPin3 = 24;
const int StepperPin4 = 25;
const int StepsPerRevolution  = 2048;
const int VentSpeed = 15;

// PINS FOR LCD DISPLAY -- 6 digital pins
const int LCDRS = 26;
const int LCDENABLE = 27;
const int LCDD4 = 28;
const int LCDD5 = 29;
const int LCDD6 = 30;
const int LCDD7  = 31;

// PINS FOR REALTIME CLOCK -- 2 digital pins
const int RTCPin1 = 32;
const int RTCPin2 = 33;
bool h12Flag;
bool pmFlag;

// PINS FOR TEMPERATURE SENSOR -- 1 digital pin
const int DHTPin = 34;

// PINS FOR FAN -- 3 digital pins
volatile unsigned char *fan_portB     = (unsigned char *) 0x25;
volatile unsigned char *fan_DDRB  = (unsigned char *) 0x24;      // PB7, 6, 5 Pins 13, 12, 11 , ENABLE, DIRA, DIRB
 
// PINS FOR LEDs -- 4 digital pins
volatile unsigned char *LED_DDRH  = (unsigned char *) 0x101;   
volatile unsigned char *LED_PORTH     = (unsigned char *) 0x102;  // PH 6, 5, 4, 3 , Pins, 9, 8, 7, 6

// BUTTON PINS -- 3 Start/Stop 
volatile unsigned char *BUTTON_PINE  = (unsigned char *) 0x2C;  
volatile unsigned char *BUTTON_DDRE  = (unsigned char *) 0x2D; // PE5, PE4, PE1 Pins 3, 2,1 -- RESET, START, STOP
//BUTTON PINS -- Vent Controls
volatile unsigned char *BUTTON_PINJ  = (unsigned char *) 0x103;  
volatile unsigned char *BUTTON_DDRJ  = (unsigned char *) 0x104; // PJ1, PJ0, Pins 14, 15

//UART Registers
 volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
 volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
 volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
 volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
 volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

 //Timer Registers
volatile unsigned char *myTCCR1A  = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B  = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C  = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1  = (unsigned char *) 0x6F;
volatile unsigned char *myTIFR1   = (unsigned char *) 0x36;
volatile unsigned int  *myTCNT1   = (unsigned  int *) 0x84;

//ADC Registers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

dht DHT; // temperature sensor
Stepper ventMotor(StepsPerRevolution,StepperPin1,StepperPin2,StepperPin3,StepperPin4); // stepper motor for vent
LiquidCrystal LCD(LCDRS,LCDENABLE,LCDD4,LCDD5,LCDD6,LCDD7);  //LCD display
DS3231 RTClock; // Real Time Clock

enum cooler_state {DISABLED, IDLE, RUNNING, ERROR};

volatile cooler_state state = DISABLED;
cooler_state previousState = DISABLED; // to check for change in state
char previousMinute = '\0';

void setup() {
  // put your setup code here, to run once:
  U0Init(9600);

    // reset the TOV flag
  *myTIFR1 |= 0x01;
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0x01;

  LCD.begin(16,2);
  ventMotor.setSpeed(VentSpeed);

  adc_init();

  // initialize pins
  *fan_DDRB |= 0b11100000; // set PB7,6,5 to output
  *LED_DDRH |= 0b01111000; // set PH6, 5, 4, 3 to output
  *BUTTON_DDRE &= 0b11001101;  // set PE5, 4, 1 to input
  *BUTTON_DDRJ &= 0b11111100; // set PJ1, 0 to input

  PCICR |= 0b00000100;		//Enable interrupt vector PCIE2
  PCMSK2 |= 0b00000100;		//  set digital pin 2 to be monitored by ISR
}

void loop() 
{
  // Check for change in state
  if (state != previousState)
  {
    // REPORT TIME WITH RT CLOCK
    ReportStateChange(state, previousState);
  }
  previousState = state; //set state for next loop

  if(state != DISABLED)
  {
    //RECORD HUMIDITY/TEMPERATURE
    int chk = DHT.read11(DHTPin);
    
    //IF A MINUTE HAS PASSED
    if(RTClock.getMinute() != previousMinute && previousMinute != '\0')
    {
      writeTemperature();// UPDATE LCD
    }
    previousMinute = RTClock.getMinute();
    
    // IF LEVELS ARE TOO LOW
    if(adc_read(WaterSensorPin) < WATER_THRESHOLD)
    {    
     state = ERROR; // SET STATE TO ERROR
    }
    //HANDLE CHANGES TO VENT CONTROLS // PJ1, PJ0, Pins 14, 15
    if(*BUTTON_PINJ & 0x01)
    {
      ventMotor.step(StepsPerRevolution);//TURN VENT LEFT
    }
    else if(*BUTTON_PINJ & 0x02)
    {
      ventMotor.step(-StepsPerRevolution);//TURN VENT RIGHT
    } 
    //IF STOP BUTTON IS PUSHED
    else if(*BUTTON_PINE & 0x02)
    {
      state = DISABLED;//SET STATE TO DISABLED
    } 
      
  }
  if(state != RUNNING)
  {
    //TURN OFF FAN MOTOR  // PB7, 6, 5
    *fan_portB &= 0b00011111;
  }
  
  switch(state)
  {
    case DISABLED: 
      //TURN ON YELLOW LED
      *LED_PORTH  |= 0b01000000;       // PH6
      // TURN OFF REMAINING LEDs
      *LED_PORTH  &= 0b11000111;
      //ISR WILL TRIGGER WHEN START BUTTON IS PRESSED TO TURN SYSTEM ON
      break;
    case IDLE:
      //TURN ON GREEN LED
      *LED_PORTH  |= 0b001000000;       // PH5
      // TURN OFF REMAINING LEDs
      *LED_PORTH  &= 0b10100111;
      //IF TEMP IS ABOVE THRESHOLD
      if(DHT.temperature > TEMP_THRESHOLD)
      {      
       state = RUNNING; //SET STATE TO RUNNING
      }
      break;
    case ERROR:
      //TURN ON RED LED
      *LED_PORTH  |= 0b00010000;       // PH4
      // TURN OFF REMAINING LEDs
      *LED_PORTH  &= 0b10010111;
      //WRITE ERROR MESSAGE TO LCD
      LCD.print("ERROR: Water Low");  
      //IF RESET BUTTON IS PRESSED
      if(*BUTTON_PINE & 0b00100000)
      {
       state = IDLE; //CHANGE STATE TO IDLE
      }
      break;
    case RUNNING:
      //TURN ON BLUE LED
      *LED_PORTH  |= 0b00001000;       // PH3
      // TURN OFF REMAINING LEDs
      *LED_PORTH  &= 0b10001000;
      //TURN ON FAN MOTOR
      *fan_portB |= 0b11000000;
      //IF TEMPERATURE IS BELOW THRESHOLD
      if(DHT.temperature < TEMP_THRESHOLD)
      {
       state = IDLE; //SET STATE TO IDLE
      }
      break;
  }
}


void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char getChar()
{
  return *myUDR0;
}
void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
      *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void ReportStateChange(cooler_state newstate, cooler_state oldstate)
{
    putChar("State change from ");
    printState(oldstate);
    putChar(" to ");
    printState(newstate);    
    putChar(" occurred at ");      
    putChar(RTClock.getHour(h12Flag, pmFlag));
    putChar(":");
    putChar(RTClock.getMinute());
    putChar(":");
    putChar(RTClock.getSecond());
    putChar(" on ");
    putChar(RTClock.getDate());
    putChar('\n');
}

void printState(cooler_state cstate)
{
      switch(cstate)
    {
      case IDLE:
        putChar("IDLE");
        break;
      case DISABLED:
        putChar("DISABLED");
        break;
      case ERROR:
        putChar("ERROR");
        break;
      case RUNNING:
        putChar("RUNNING");
        break;       
    } 
}

void writeTemperature()
{
    LCD.print("\n\nHumidity: ");
    LCD.print(DHT.humidity);
    LCD.print('\n');
    LCD.print("Temperature: ");
    LCD.print(DHT.temperature);
    LCD.print('\n');    
}

ISR (PCINT2_vect) 
{
  if(*BUTTON_PINE & 0b00010000)
  {
    state = IDLE;
  }     
} 