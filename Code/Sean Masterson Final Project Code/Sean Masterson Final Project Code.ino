#include <uRTCLib.h>
#include <dht.h>
#include <Stepper.h>
#include <LiquidCrystal.h>





#define RDA 0x80
#define TBE 0x20 



//GLOBAL CONSTANTS
const int TEMP_THRESHOLD = 25;
const int WATER_THRESHOLD = 150;


// PINS FOR WATER SENSOR -- 1 Analog pin
const int WaterSensorPin = 1;

 
     

// PINS FOR STEPPER MOTOR -- 4 digital pins
const int StepperPin1 = 22;
const int StepperPin2 = 23;
const int StepperPin3 = 24;
const int StepperPin4 = 25;
const int StepsPerRevolution  = 200;
const int VentSpeed = 60;

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

//BUTTON PINS RESET BUTTON
volatile unsigned char *BUTTON_PING  = (unsigned char *) 0x32;  
volatile unsigned char *BUTTON_DDRG  = (unsigned char *) 0x33; // PG5

// BUTTON PINS -- 2 Start/Stop 
volatile unsigned char *BUTTON_PINE  = (unsigned char *) 0x2C;  
volatile unsigned char *BUTTON_DDRE  = (unsigned char *) 0x2D; // PE5, PE4, PE1 Pins  2,1 -- RESET, START, STOP
//BUTTON PINS -- Vent Controls
volatile unsigned char *BUTTON_PINL  = (unsigned char *) 0x109;  
volatile unsigned char *BUTTON_DDRL  = (unsigned char *) 0x10A; // PL7, PL6, Pins 42, 43



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

volatile int* secondCounter;
volatile bool* minutePassed;
volatile bool* checkTime;

dht DHT; // temperature sensor
Stepper ventMotor(StepsPerRevolution,StepperPin1,StepperPin2,StepperPin3,StepperPin4); // stepper motor for vent
LiquidCrystal LCD(LCDRS,LCDENABLE,LCDD4,LCDD5,LCDD6,LCDD7);  //LCD display
uRTCLib RTClock(0x68); // Real Time Clock

enum cooler_state {DISABLED, IDLE, RUNNING, ERROR};

volatile cooler_state state = DISABLED;
cooler_state previousState = DISABLED; // to check for change in state
char previousMinute = '\0';

//UART Registers
 volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
 volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
 volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
 volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
 volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

void setup()
{
  *secondCounter = 0;
 *minutePassed = false;
  *checkTime = true;
   U0init(9600);
//Serial.begin(9600);
       // reset the TOV flag
  *myTIFR1 |= 0x01;
  
  // enable the TOV interrupt
  //*myTIMSK1 |= 0x01;
  *myTIMSK1 |= 0x01;
    // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) 3036); // 65535-3036 = 1 second w/ prescaler of 256
    // Start the Timer
  *myTCCR1B |=   0b00000100;
  LCD.begin(16,2);
  ventMotor.setSpeed(VentSpeed);

  adc_init();
  // initialize pins
  *fan_DDRB |= 0b11100000; // set PB7,6,5 to output

  *LED_DDRH |= 0b01111000; // set PH6, 5, 4, 3 to output
  *BUTTON_DDRG &= 0b00100000;// set PG5 to input
  *BUTTON_DDRE &= 0b11001100;  // set PE5, 4, to input
  *BUTTON_DDRL &= 0b11000000; // set PL7, PL6, to input


 PCICR |= 0b00000100;		//Enable interrupt vector PCIE2
 PCMSK2 |= 0b00000100;		//  set digital pin 2 to be monitored by ISR
 	//RTClock.set(0, 45, 17, 3, 9, 5, 23);
}

void loop() 
{
  if(*checkTime)
  {
   RTClock.refresh();
  }
  // Check for change in state

  if (state != previousState)
  {
    // REPORT TIME WITH RT CLOCK
    ReportStateChange(state, previousState);
  }
  previousState = state; //set state for next loop

    if(*BUTTON_PINE & 0b00010000) // IF STOP BUTTON IS EVER PRESSED
    {
      state = DISABLED;//SET STATE TO DISABLED
    } 

  if(state != DISABLED && state!= ERROR)
  {
    //RECORD HUMIDITY/TEMPERATURE
    int chk = DHT.read11(DHTPin);
    writeTemperature();// UPDATE LCD
    //IF A MINUTE HAS PASSED
   if(*minutePassed)
    {
      
    }

    
    // IF LEVELS ARE TOO LOW
    if(adc_read(WaterSensorPin) < WATER_THRESHOLD)
    {    
     state = ERROR; // SET STATE TO ERROR
    }
    //HANDLE CHANGES TO VENT CONTROLS // PL7, PL6
    if(*BUTTON_PINL & 0b10000000)
    {

    //  ventMotor.step(StepsPerRevolution);//TURN VENT LEFT
    }
    else if(*BUTTON_PINL & 0b01000000)
    {
   //  ventMotor.step(-StepsPerRevolution);//TURN VENT RIGHT
    }   
  }
  if(state != ERROR)
  {
          //HANDLE CHANGES TO VENT CONTROLS // PL7, PL6
    if(*BUTTON_PINL & 0b10000000)
    {
      Serial.print("LEFT");
      ventMotor.step(StepsPerRevolution);//TURN VENT LEFT
    }
    else if(*BUTTON_PINL & 0b01000000)
    {
      Serial.print("RIGHT");
      ventMotor.step(-StepsPerRevolution);//TURN VENT RIGHT
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
      LCD.clear();
      //TURN ON YELLOW LED
      *LED_PORTH  |= 0b01000000;       // PH6
      // TURN OFF REMAINING LEDs
      *LED_PORTH  &= 0b11000111;
      //ISR WILL TRIGGER WHEN START BUTTON IS PRESSED TO TURN SYSTEM ON
      if(*BUTTON_PINE & 0b00100000)
      {     
       state = IDLE;        
      }
      break;
    case IDLE:
      //TURN ON GREEN LED
      *LED_PORTH  |= 0b00100000;       // PH5
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
      *LED_PORTH  |= 0b00001000;       // PH4
      // TURN OFF REMAINING LEDs
      *LED_PORTH  &= 0b10001111;
      //WRITE ERROR MESSAGE TO LCD
      LCD.clear();
      LCD.setCursor(0,0);
      LCD.print("ERROR: Water Low");  
      //IF RESET BUTTON IS PRESSED
      if(*BUTTON_PING & 0b00100000)
      {
       state = IDLE; //CHANGE STATE TO IDLE
      }
      break;
    case RUNNING:
      //TURN ON BLUE LED
      *LED_PORTH  |= 0b00010000;       // PH3
      // TURN OFF REMAINING LEDs
      *LED_PORTH  &= 0b10010000;
      //TURN ON FAN MOTOR
      *fan_portB |= 0b10100000;
      //IF TEMPERATURE IS BELOW THRESHOLD
      if(DHT.temperature < TEMP_THRESHOLD)
      {
       state = IDLE; //SET STATE TO IDLE
      }
      break;
  }
}

void U0init(int U0baud)
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

void ReportStateChange(cooler_state newstate, cooler_state oldstate)//, DateTime DTime)
{
    myprint("State change from ");
    printState(oldstate);
    myprint(" to ");
    printState(newstate); 
    myprint("\n");       
    myprint(" occurred at ");    
    myprint((String)RTClock.hour());
    myprint(":");
    myprint((String)RTClock.minute());
    myprint(":");
    myprint((String)RTClock.second());
    myprint(" on ");
    myprint((String)RTClock.month());
    myprint("/");
    myprint((String)RTClock.day());
    myprint("/");
    myprint((String)RTClock.year());    
    myprint("\n");
}

void printState(cooler_state cstate)
{
      switch(cstate)
    {
      case IDLE:
        myprint("IDLE");
        break;
      case DISABLED:
        myprint("DISABLED");
        break;
      case ERROR:
        myprint("ERROR");
        break;
      case RUNNING:
        myprint("RUNNING");
        break;       
    } 
}

void writeTemperature()
{
    LCD.clear();
    LCD.setCursor(0,0);
    LCD.print("Humidity: ");
    LCD.print(DHT.humidity);
    LCD.setCursor(0,1);
    LCD.print("Temperature: ");
    LCD.print(DHT.temperature);
}



unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void myprint(String string)
{
for (auto c : string)
  {
    U0putchar(c);
  }  
}



void myprintln(String string)
{
  myprint(string);
  U0putchar('\n');  
}

ISR (PCINT2_vect) 
{
    state = IDLE;

}

ISR(TIMER1_OVF_vect)
{
  // Stop the Timer
  *myTCCR1B &=0xF8;

  *checkTime = !checkTime;
  *secondCounter ++;
  if(*secondCounter == 60)
  {
    *secondCounter = 0;
    *minutePassed = true;
  }
  else
  {
    *minutePassed = false;
  }
  // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) 3036); // 65535-3036 = 1 second w/ prescaler of 256
    // Start the Timer
  *myTCCR1B |=   0b00000100;


}