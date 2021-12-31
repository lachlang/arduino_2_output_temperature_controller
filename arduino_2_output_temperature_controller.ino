#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <Time.h>

/*--------------------------------------------------------------------------------------
  Constants
--------------------------------------------------------------------------------------*/
// internal state constants
const int DELAY = 100; // milli-seconds (0.1 second)
const float TEMP_INCREMENTS = 0.5;
const float MINIMUM_TARGET = 2.0; // this is enough to get into trouble
const float MAXIMUM_TARGET = 40.0; // increased to allow for a VOSS IPA :D
const float TEMP_THRESHOLD_INCREMENTS = 0.25;
const float MINIMUM_THREASHOLD = 0.5;
const float MAXIMUM_THREASHOLD = 5.0;
const int DISPLAY_TIMER_INCREMENT = 1000 / DELAY; // make sure our display timer is in seconds
const int REDIRECT_TIMEOUT = 1000; // redirect after 10 seconds

// Pins in use
# define BUTTON_ADC_PIN            A2  // A0 is the button ADC input
const int COOLING_PIN              = 12; // can't seem to get pin 1 to work
const int HEATING_PIN              = 2;
const int LCD_BACKLIGHT_PIN        = 3;  // D3 controls LCD backlight
const int TEMP_SENSOR_PIN          = 10;

// ADC readings expected for the 5 buttons on the ADC input
const int RIGHT_10BIT_ADC        =   0;  // right
const int UP_10BIT_ADC           = 145;  // up
const int DOWN_10BIT_ADC         = 329;  // down
const int LEFT_10BIT_ADC         = 505;  // left
const int SELECT_10BIT_ADC       = 741;  // right
const int NO_BUTTON_ADC          = 1023;  // no button
const int BUTTONHYSTERESIS       =  10;  // hysteresis for valid button sensing window

//return values for ReadButtons()
const int BUTTON_NONE            =   0;  // 
const int BUTTON_RIGHT           =   1;  // 
const int BUTTON_UP              =   2;  // 
const int BUTTON_DOWN            =   3;  // 
const int BUTTON_LEFT            =   4;  // 
const int BUTTON_SELECT          =   5;  // 

// temperature states
const int STATE_COOLING          =   0;
const int STATE_INACTIVE         =   1;
const int STATE_HEATING          =   2;

// LCD States
const int DISPLAY_SUMMARY        =   0;
const int DISPLAY_SET_TARGET     =   1;
const int DISPLAY_SET_MAX_TEMP   =   2;
const int DISPLAY_SET_MIN_TEMP   =   3;
const int DISPLAY_COOLER_STATUS  =   4;
const int DISPLAY_WARMER_STATUS  =   5;
const int DISPLAY_TEMP_SUMMARY   =   6;
const int DISPLAY_NEXT_SETPOINT  =   7;
const int DISPLAY_RUN_TIME       =   8;
const int DISPLAY_TEMP_HISTORY   =   9;
const int NO_OF_LCD_STATES       =  10;

// EEPROM Memory Addresses
const int TARGET_TEMP_ADDRESS       = 0x00;
const int COOLING_THRESHOLD_ADDRESS = 0x04; 
const int HEATING_THRESHOLD_ADDRESS = 0x08;

/**
 * state variables
 */
 // controller state
int currentControllerState = 1; // 0 = cooling, 1 = inactive, 2 = heating
boolean controllerSettingsChanged = false;
float currentTemp, targetTemp = 18.5, coolingThreshold = 0.5, heatingThreshold = 0.5;
OneWire ds(TEMP_SENSOR_PIN); // pin 10
byte addr[8];

// LCD state
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );
int currentLCDState = 0; // valid state id
int timeInLCDState = 0; // in seconds
int backlightTimeout = 0;  // in seconds
boolean buttonIsPressed = false;
boolean isBacklightActive = true;

// history stuff
float maxTemp = -100.0, minTemp = 200.0;
unsigned long timeInCurrentControllerState_ms = 0; // milli-seconds
unsigned long timeEnteredCurrentControllerState_ms = 0; // milli-seconds
unsigned long maxTimeHeating_ms = 0; // in milli-seconds
unsigned long maxTimeCooling_ms = 0; // in milli-seconds

// Fermentation Tracking and Target Temperature Scheduling
const int SCHEDULE_ARRAY_SIZE = 10;

unsigned long timeToNextChange[SCHEDULE_ARRAY_SIZE] = { 0 }; // init all elements to 0
float nextSetpoint[SCHEDULE_ARRAY_SIZE] = { 0 };     // init all elements to 0
// saisson settings
//unsigned long timeToNextChange[SCHEDULE_ARRAY_SIZE] = { 3600, 86400, 172800, 259200, 345600, 432000, 518400}; // after 1 hour perform first change, and every day thereafter
//float nextSetpoint[SCHEDULE_ARRAY_SIZE] = { 22, 23, 24, 25, 26, 27, 28 };     // stabilise at 22 degrees then incrememt by 1 degree per day until 28 degrees
// test variables
//unsigned long timeToNextChange[SCHEDULE_ARRAY_SIZE] = { 691210, 30, 45, 60, 86400, 86400, 86400}; // after 1 hour perform first change, and every day thereafter
//float nextSetpoint[SCHEDULE_ARRAY_SIZE] = { 22, 23, 24, 25, 26, 27, 28 };     // stabilise at 22 degrees then incrememt by 1 degree per day until 28 degrees

/**
 * Arduino setup method
 */
void setup(void) {
  lcd.begin(16, 2);  
  lcd.print("Hi Lachlan...");
  lcd.setCursor(0,1);
  lcd.print("Let's make beer!");
  Serial.begin(9800);
  Serial.println("Hi Lachlan...");
  Serial.println("Let's make beer!");
  
  if ( !initialiseTemperatureSensor() ) {
    lcd.print("Sensor init failed!");
    return;
  }
  
  maxTemp = getCurrentTemperature();
  minTemp = maxTemp;
  
  // initialise the pins
  pinMode( BUTTON_ADC_PIN, INPUT );         //ensure button read pin is an input
  digitalWrite( BUTTON_ADC_PIN, LOW );      //ensure pullup is off on button read pin
  pinMode( TEMP_SENSOR_PIN, INPUT );
  pinMode( LCD_BACKLIGHT_PIN, OUTPUT );
  digitalWrite( LCD_BACKLIGHT_PIN, HIGH );
  pinMode( COOLING_PIN, OUTPUT);
  pinMode( HEATING_PIN, OUTPUT );
  digitalWrite( COOLING_PIN, LOW );      // ensure initial state is inactive
  digitalWrite( HEATING_PIN, LOW );      // ensure initial state is inactive

  pinMode( 13, OUTPUT );
  digitalWrite( 13, LOW );      // ensure initial state is inactive

  
//  // initialise the time to 01-Jan-2013 07:00:000
//  setTime(0,0,0,1,1,2013); // hour, min, sec, day month, year

  // read variables from flash memory
  EEPROMReadFloat(TARGET_TEMP_ADDRESS, targetTemp);
  EEPROMReadFloat(COOLING_THRESHOLD_ADDRESS, coolingThreshold);
  EEPROMReadFloat(HEATING_THRESHOLD_ADDRESS, heatingThreshold);
}

/**
 * main arduino control loop
 * temperature readings are taken every second (1 second) and LCD buttons 
 * and display are checked over 0.1 seconds 
 */
void loop(void) {

  // obtain the temperature reading and update the heating/cooling state
  updateTemperature();
  controlTemperatureState();
  
  // check for button input and process it
  for (int i = 0; i< 10; i ++) {
    controlDisplayState();  
    delay(DELAY); // NOTE: the only delay() in the program
  }
  
  // update the display
  displayState(); 

  // check to see if a target temperature change has been scheduled
  checkForScheduledTargetTemperatureChange();

  // for extra debuggering
  // outputStateToSerial();
}

/**
 * calculate the current temperature and update the current, min and max temp state variables
 */
void updateTemperature() {

  currentTemp = getCurrentTemperature();

  if (currentTemp > maxTemp) {
    maxTemp = currentTemp;
  }
  else if (currentTemp < minTemp) {
    minTemp = currentTemp;
  }
}

/**
 * returns the current temperature
 */
float getCurrentTemperature() {
  int highByteData, lowByteData, sensorReading;
  byte data[12];
  float temperature;
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);
  
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);

  for (int i = 0; i < 12; i++) {
    data[i] = ds.read();
  }
  
  lowByteData = data[0];
  highByteData = data[1];
  sensorReading = (highByteData << 8) + lowByteData;
  
  if (sensorReading & 0x8000) { // test the most significant bit for sign
    sensorReading = -sensorReading;
  }
  temperature = sensorReading * 0.0625; // minimum quantised reading from temp sensor DS18B20
  
  return temperature;  
}

/**
 * controls state changes and timers for each state
 * (HEATIING) <=> (INACTIVE) <=> (COOLING)
 */
void controlTemperatureState() {

  timeInCurrentControllerState_ms = millis() - timeEnteredCurrentControllerState_ms;
  
  switch ( currentControllerState ) {
    case STATE_COOLING:
      if (currentTemp < targetTemp + coolingThreshold) {
        //stop cooling
        currentControllerState = STATE_INACTIVE;
        if (timeInCurrentControllerState_ms > maxTimeCooling_ms) {
          maxTimeCooling_ms = timeInCurrentControllerState_ms;
        }
        timeEnteredCurrentControllerState_ms = millis();
        digitalWrite(COOLING_PIN, LOW);
        digitalWrite( 13, LOW );

      }
      break;
    case STATE_INACTIVE:
      if (currentTemp < targetTemp - heatingThreshold) {
        //start heating
        currentControllerState = STATE_HEATING;
        timeEnteredCurrentControllerState_ms = millis();
        digitalWrite(HEATING_PIN, HIGH);
        digitalWrite( 13, HIGH );
      }
      else if (currentTemp > targetTemp + coolingThreshold) {
        //start cooling
        currentControllerState = STATE_COOLING;
        timeEnteredCurrentControllerState_ms = millis();
        digitalWrite(COOLING_PIN, HIGH);
        digitalWrite( 13, HIGH );
      }
      break;
    case STATE_HEATING:
      if (currentTemp > targetTemp - heatingThreshold) {
        //stop heating
        currentControllerState = STATE_INACTIVE;
        if (timeInCurrentControllerState_ms > maxTimeHeating_ms) {
          maxTimeHeating_ms = timeInCurrentControllerState_ms;
        }
        timeInCurrentControllerState_ms = millis();
        digitalWrite(HEATING_PIN, LOW);
        digitalWrite( 13, LOW );
      } // if
      break;
  } // switch
}

/**
 * checks to see if there is a scheduled temperature change and performs
 * the change if it is time
 */
void checkForScheduledTargetTemperatureChange() {

  // no scheduled target temperature changes
  if (nextSetpoint[0] < MINIMUM_TARGET) {
    return;
  }

  // decrement countdown timer
  timeToNextChange[0]--;

  //
  if (timeToNextChange[0] <= 0) {
    performTargetTemperatureChange();
  }
}

/**
 * performs the target temperature change by cycling the array values and
 * updating the target temperature variable
 */
void performTargetTemperatureChange() {
  
  targetTemp = nextSetpoint[0];
  
  // shift the array elements down (pop queue)
  for (int i = 0; i < SCHEDULE_ARRAY_SIZE -1; i++) {
    nextSetpoint[i] = nextSetpoint[i+1];
    timeToNextChange[i] = timeToNextChange[i+1];
  }
}

/**
 * handles button presses and state transitions
 */
void controlDisplayState() {
  
  // update the backlight display
  checkBacklightTimeout();
  
  // obtain input from the buttons
  timeInLCDState += DISPLAY_TIMER_INCREMENT;
  byte whichButtonPressed = ReadButtons();
  
  // if no button and NOT summary screen increment home redirect counter
  if ( !whichButtonPressed && currentLCDState != DISPLAY_SUMMARY) {
    
    // return to summary screen after timeout
    if ( timeInLCDState >= REDIRECT_TIMEOUT ) {
      if (controllerSettingsChanged) {
        controllerSettingsChanged = false;
        flashUpdatedControllerState();
      }
      timeInLCDState = 0;
      currentLCDState = DISPLAY_SUMMARY;
      displaySummary();
    }
    return;
  }
  
  // select state
  else if ( whichButtonPressed == BUTTON_RIGHT || whichButtonPressed == BUTTON_LEFT ) {
    handleLeftRight( whichButtonPressed );
    enableBacklight();
    displayState();
  }
  
  // up - down - all around
  else if ( whichButtonPressed == BUTTON_UP   || 
            whichButtonPressed == BUTTON_DOWN ) {
    // handle up and down and seledct based upon state
    handleUpDown( whichButtonPressed );
    enableBacklight();
    displayState(); 
  }
  else if ( whichButtonPressed == BUTTON_SELECT ) {
    handleSelect();
  }
}

/**
 * switch the backlight off if the timeout reached
 * time out set to 5 x the lcd state transition timeout
 */
void checkBacklightTimeout() {
  if (isBacklightActive && (backlightTimeout += DISPLAY_TIMER_INCREMENT) > (REDIRECT_TIMEOUT * 5) ) {
    disableBacklight();
  }
}

/**
 * handle display state transitions
 */
void handleLeftRight( int whichButtonPressed ) {
    // reset state timeout
    timeInLCDState = 0;

    // flash changed variables
    if (controllerSettingsChanged) {
      flashUpdatedControllerState();
      controllerSettingsChanged = false;
    }
    
    if ( whichButtonPressed == BUTTON_RIGHT ) {
      currentLCDState++;
    } else {
      currentLCDState--;
    }
    currentLCDState += NO_OF_LCD_STATES;
    currentLCDState %= NO_OF_LCD_STATES;
}

/**
 * change the internal state variables of the controller for up, down and select inputs
 * based on the current view state
 */
void handleUpDown( int whichButtonPressed ) {

   switch ( currentLCDState ) {
    case DISPLAY_SUMMARY:
    case DISPLAY_COOLER_STATUS:
    case DISPLAY_WARMER_STATUS:
    case DISPLAY_TEMP_SUMMARY:
      // no action
      break;
    case DISPLAY_TEMP_HISTORY:
      // TODO: reset if held
      break;
    case DISPLAY_SET_MAX_TEMP:
      // raise or lower the maximum temperature via the fridge 
      // cut-in temperature (cooling threshold)
      if ( whichButtonPressed == BUTTON_UP ) {
        // reset state timeout
        timeInLCDState = 0;
        if ( coolingThreshold < MAXIMUM_THREASHOLD ) {
          coolingThreshold += TEMP_THRESHOLD_INCREMENTS;
          controllerSettingsChanged = true;
        } 
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        // reset state timeout
        timeInLCDState = 0;
        
        if ( coolingThreshold > MINIMUM_THREASHOLD ) {
          coolingThreshold -= TEMP_THRESHOLD_INCREMENTS; 
          controllerSettingsChanged = true;
        }
      }
      break;
    case DISPLAY_SET_MIN_TEMP:
      // raise or lower the minimum temperature via the heater 
      // cut-in temperature (heating threshold)
      if ( whichButtonPressed == BUTTON_UP ) {
        // reset state timeout
        timeInLCDState = 0;

        if ( heatingThreshold < MAXIMUM_THREASHOLD ) {
          heatingThreshold += TEMP_THRESHOLD_INCREMENTS; 
          controllerSettingsChanged = true;
        }
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        // reset state timeout
        timeInLCDState = 0;
        if ( heatingThreshold > MINIMUM_THREASHOLD ) {
          heatingThreshold -= TEMP_THRESHOLD_INCREMENTS; 
          controllerSettingsChanged = true;
        }
      }
      break;
    case DISPLAY_SET_TARGET:
      // raise or lower the target temperature
      if ( whichButtonPressed == BUTTON_UP ) {
        // reset state timeout
        timeInLCDState = 0;

        if ( targetTemp < MAXIMUM_TARGET ) {
          targetTemp += TEMP_INCREMENTS; 
          controllerSettingsChanged = true;
        }
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        // reset state timeout
        timeInLCDState = 0;
        if ( targetTemp > MINIMUM_TARGET ) {
          targetTemp -= TEMP_INCREMENTS; 
          controllerSettingsChanged = true;
        }
      }
      break;
  }
}

/**
 * switch the backlight on or off
 */
void handleSelect() {
  if (isBacklightActive) {
    disableBacklight();
  }
  else {
    enableBacklight();
  }
}

/**
 * disable the backlight
 */
void disableBacklight() {
  digitalWrite( LCD_BACKLIGHT_PIN, LOW );
  isBacklightActive = false;
  backlightTimeout = 0;
}

/**
 * enable the backlight
 */
void enableBacklight() {
  digitalWrite( LCD_BACKLIGHT_PIN, HIGH );
  backlightTimeout = 0;
  isBacklightActive = true;
}

/**
 * write updated controller state variables to flash memory
 */
void flashUpdatedControllerState() {

  switch (currentLCDState) {
    case DISPLAY_SET_TARGET:
      EEPROMWriteFloat(TARGET_TEMP_ADDRESS, targetTemp);
      break;
    case DISPLAY_SET_MAX_TEMP:
      EEPROMWriteFloat(COOLING_THRESHOLD_ADDRESS, coolingThreshold);
      break;  
    case DISPLAY_SET_MIN_TEMP:
      EEPROMWriteFloat(HEATING_THRESHOLD_ADDRESS, heatingThreshold);
      break;
  } 
}


/**
 * output the data for the current display state to the LCD screen
 */
void displayState()  {
  switch ( currentLCDState ) {
    case DISPLAY_SUMMARY:
      displaySummary();
      break;
    case DISPLAY_TEMP_HISTORY:
      displayMinMaxHistory();
      break;
    case DISPLAY_SET_MAX_TEMP:
      displayMaxTempThreshold();
      break;
    case DISPLAY_SET_MIN_TEMP:
      displayMinTempThreshold();
      break;
    case DISPLAY_COOLER_STATUS:
      displayCoolerStatus();
      break;
    case DISPLAY_WARMER_STATUS:
      displayHeaterStatus();
      break;
    case DISPLAY_SET_TARGET:
      displayTargetTemp();
      break;
    case DISPLAY_TEMP_SUMMARY:
      displayCurrentTempSummary();
      break;
    case DISPLAY_NEXT_SETPOINT:
      displayNextSetpoint();
      break;
    case DISPLAY_RUN_TIME:
      displayRunTime();
      break;
    default:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Current State:");
      lcd.setCursor(15,0);
      lcd.print( currentLCDState );
      lcd.setCursor(0,1);
      lcd.print("Invalid State...");
  }
}

/**
 * summary display
 */ 
void displaySummary() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Current Temp:");
  lcd.setCursor(0,1);
  lcd.print(currentTemp);
  lcd.setCursor(6,1);
  lcd.print("-");
  lcd.setCursor(8,1);
  switch ( currentControllerState ) {
    case STATE_COOLING:
      lcd.print("Cooling");
      break;
    case STATE_INACTIVE:
      lcd.print("Inactive");
      break;
    case STATE_HEATING:
      lcd.print("Heating");
      break;
  }
}

/**
 * display temp history
 */ 
void displayMinMaxHistory() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp History:");
  lcd.setCursor(0,1);
  lcd.print("Min");
  lcd.setCursor(3,1);
  lcd.print(minTemp);
  lcd.setCursor(8,1);
  lcd.print("Max");
  lcd.setCursor(11,1);
  lcd.print(maxTemp);
}

/**
 * display max temp threshold
 */ 
void displayMaxTempThreshold() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Fridge Threshold");
  lcd.setCursor(0,1);
  lcd.print(coolingThreshold);
  lcd.setCursor(7,1);
  lcd.print("[up/down]");
}

/**
 * display min temp threshold
 */ 
void displayMinTempThreshold() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Heater Threshold");
  lcd.setCursor(0,1);
  lcd.print(heatingThreshold);
  lcd.setCursor(7,1);
  lcd.print("[up/down]");
}

/**
 * display fridge status
 */ 
void displayCoolerStatus() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Fridge is");
  lcd.setCursor(10,0);
  if ( currentControllerState == STATE_COOLING ) {
    lcd.print("active");
    lcd.setCursor(0,1);
    lcd.print("Running:");
    lcd.setCursor(8,1);
    lcd.print(getPrintableRunTime(timeInCurrentControllerState_ms / 1000));
  }
  else {
    lcd.print("off");
    lcd.setCursor(0,1);
    lcd.print("Max Run:");
    lcd.setCursor(8,1);
    lcd.print(getPrintableRunTime(maxTimeCooling_ms / 1000));
  }
}

/**
 * display heater status
 */ 
void displayHeaterStatus() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Heater is");
  lcd.setCursor(10,0);
  if ( currentControllerState == STATE_HEATING ) {
    lcd.print("active");
    lcd.setCursor(0,1);
    lcd.print("Running:");
    lcd.setCursor(8,1);
    lcd.print(getPrintableRunTime(timeInCurrentControllerState_ms / 1000));
  }
  else {
    lcd.print("off");
    lcd.setCursor(0,1);
    lcd.print("Max Run:");
    lcd.setCursor(8,1);
    lcd.print(getPrintableRunTime(maxTimeHeating_ms / 1000));
  }
}

/**
 * display target temp
 */ 
void displayTargetTemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Target Temp:");
  lcd.setCursor(0,1);
  lcd.print(targetTemp);
  lcd.setCursor(7,1);
  lcd.print("[up/down]");
}

/**
 * display current temp summary
 */ 
void displayCurrentTempSummary() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Current:");
  lcd.setCursor(8,0);
  lcd.print(currentTemp);
  lcd.setCursor(0,1);
  lcd.print("Target:");
  lcd.setCursor(8,1);
  lcd.print(targetTemp);
}

/**
 * display run time
 */ 
void displayRunTime() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Total Run Time:");
  lcd.setCursor(4,1);
  lcd.print(getPrintableRunTimeInDays(millis()/1000));
}

/**
 * display next set point
 * count down to change and new target
 */ 
void displayNextSetpoint() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Next Target Temp");
  if (nextSetpoint[0] != 0) {
    lcd.setCursor(0,1);
    lcd.print(getPrintableRunTimeInDays(timeToNextChange[0]));
    lcd.setCursor(12,1);
    lcd.print(nextSetpoint[0]);
  } else {
    lcd.setCursor(2,1);
    lcd.print("none");
  }
}

/**
 * return a time in days "dd hh:mm:ss"
 */
String getPrintableRunTimeInDays (unsigned long timeInSeconds) {

  String days = "   ";
  if (timeInSeconds / 86400 > 0) {
    days = zeroPad(timeInSeconds / 86400);
    days += "d";
  }
  days += getPrintableRunTime(timeInSeconds % 86400);
  return days;
}

/**
 * return an int time in seconds as hh:mm:dd
 */
String getPrintableRunTime(unsigned long timeInSeconds) {
  
  unsigned int temp = timeInSeconds / 3600;
  if ( temp > 99 ) {
    return "99:99:99";
  }
  
  String time = zeroPad(temp);
  temp = (timeInSeconds % 3600 ) / 60;
  time += ":";
  time += zeroPad(temp);
  time += ":";
  time += zeroPad( timeInSeconds % 60 );
  return time;
}

String zeroPad( int value ) {
  String valueString = String(value);
  if (value < 10) {
    valueString = String("0") + valueString;
  }
  return valueString;
}

/**
 * initialisation code copied from
 * http://www.arduino.cc/playground/Learning/OneWire/
 */
boolean initialiseTemperatureSensor() {
  if ( !ds.search(addr)) {
      Serial.print("Search failure.\n");
      ds.reset_search();
      return 0;
  }

  Serial.print("R=");
  for(int i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      return 0;
  }

  if ( addr[0] == 0x10) {
      Serial.print("Device is a DS18S20 family device.\n");
  }
  else if ( addr[0] == 0x28) {
      Serial.print("Device is a DS18B20 family device.\n");
  }
  else {
      Serial.print("Device family is not recognized: 0x");
      Serial.println(addr[0],HEX);
      return 0;
  }
  return true;
}

/*--------------------------------------------------------------------------------------
  ReadButtons()
  Detect the button pressed and return the value
  Uses global values buttonWas, buttonJustPressed, buttonJustReleased.
--------------------------------------------------------------------------------------*/
byte ReadButtons()
{
   unsigned int buttonVoltage;
   byte button = BUTTON_NONE;   // return no button pressed if the below checks don't write to btn
   
   buttonVoltage = analogRead( BUTTON_ADC_PIN );
   
   // check if this is a new button press
   if (buttonIsPressed && buttonVoltage > ( NO_BUTTON_ADC - BUTTONHYSTERESIS )) {
     // button has been released
     buttonIsPressed = false;
     return button;
   } else if ( buttonIsPressed ) {
     // button is currently pressed but transition has already been handled
     return button;
   }
   
   //sense if the voltage falls within valid voltage windows
   if( buttonVoltage < ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_RIGHT;
      buttonIsPressed = true;
   }
   else if(   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_UP;
      buttonIsPressed = true;
   }
   else if(   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_DOWN;
      buttonIsPressed = true;
   }
   else if(   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_LEFT;
      buttonIsPressed = true;
   }
   else if(   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_SELECT;
      buttonIsPressed = true;
   }
   
   return( button );
}

/** 
 * read and write float values to/from EEPROM
 based on http://forum.arduino.cc/index.php/topic,41497.0.html
 */
int EEPROMReadFloat(int ee, float& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return i;
}

int EEPROMWriteFloat(int ee, float& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
    return i;
}

int EEPROMReadLong(int ee, long& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return i;
}

int EEPROMWriteLong(int ee, long& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
    return i;
}

/**
 * debug function to serial port
 */
void outputStateToSerial() {
  Serial.print("controller state: ");
  switch (currentControllerState ) {
    case STATE_COOLING:
      Serial.println("cooling");
      break;
    case STATE_INACTIVE:
      Serial.println("inactive");
      break;
    case STATE_HEATING:
      Serial.println("warming");
      break;
  }
  Serial.print("time in controller state: '");
  Serial.print(timeInCurrentControllerState_ms);
  Serial.println("' (s)");

  Serial.print("current temp: '");
  Serial.print(currentTemp);
  Serial.println("'");
  Serial.print("target temp: '");
  Serial.print(targetTemp);
  Serial.println("'");
  Serial.print("cooling threashold: '");
  Serial.print(coolingThreshold);
  Serial.println("'");
  Serial.print("warming threashold: '");
  Serial.print(heatingThreshold);
  Serial.println("'");

  Serial.print("LCD State: '");
  Serial.print(currentLCDState);
  Serial.println("' (s)");
  Serial.print("time in LCD state: '");
  Serial.print(timeInLCDState);
  Serial.println("' (s)");

  Serial.print("max temp: '");
  Serial.print(maxTemp);
  Serial.println("'");
  Serial.print("min temp: '");
  Serial.print(minTemp);
  Serial.println("'");
  Serial.print("time heating: '");
  Serial.print(maxTimeHeating_ms);
  Serial.println("' (s)");
  Serial.print("time cooling: '");
  Serial.print(maxTimeCooling_ms);
  Serial.println("' (s)");
}
