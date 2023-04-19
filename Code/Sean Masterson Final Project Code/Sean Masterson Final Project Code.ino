enum cooler_state {DISABLED, IDLE, RUNNING, ERROR};

cooler_state state = DISABLED;
cooler_state previousState = DISABLED; // to check for change in state

void setup() {
  // put your setup code here, to run once:

}

void loop() 
{
  // Check for change in state
  if (state != previousState)
  {
    // REPORT TIME WITH RT CLOCK
  }
  previousState = state; //set state for next loop

  if(state != DISABLED)
  {
    //RECORD HUMIDITY/TEMPERATURE
    //IF A MINUTE HAS PASSED
      // UPDATE LCD
    // IF LEVELS ARE TOO LOW
      // SET STATE TO ERROR
    //HANDLE CHANGES TO VENT CONTROLS
    //IF STOP BUTTON IS PUSHED
      //SET STATE TO DISABLED
  }
  switch(state)
  {
    case DISABLED: 
      //TURN ON YELLOW LED
      // TURN OFF REMAINING LEDs
      //TURN OFF MOTOR
      //ISR WILL TRIGGER WHEN START BUTTON IS PRESSED TO TURN SYSTEM ON
      break;
    case IDLE:
      //TURN ON GREEN LED
      //TURN OFF REMAINING LEDs
      //IF TEMP IS ABOVE THRESHOLD
        //SET STATE TO RUNNING
      break;
    case ERROR:
      //TURN ON RED LED
      // TURN OFF REMAINING LEDs
      //TURN OFF MOTOR
      //WRITE ERROR MESSAGE TO LCD
      //IF RESET BUTTON IS PRESSED
        //CHANGE STATE TO IDLE
      break
    case RUNNING:
      //TURN ON BLUE LED
      // TURN OFF REMAINING LEDS
      //TURN ON FAN MOTOR
      //IF TEMPERATURE IS BELOW THRESHOLD
        //SET STATE TO IDLE
      break;
  }
}
