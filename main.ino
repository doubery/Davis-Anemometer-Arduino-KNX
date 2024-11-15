/*
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
This code is an extension of the following great projects:

https://github.com/wrybread/weewx-ArduinoWeatherStation
https://github.com/doubery/Davis-Anemometer-Arduino/tree/main

With the awsome work of:
https://github.com/thorsten-gehrig/arduino-tpuart-knx-user-forum


This is a sketch to connect a davis wind-sensor to a Arduino (Pro Mini 5V), for use on a knx network with a BUS-Coupling-Unit

Davis - Arduino Pro Mini (5V)
- - - yellow  -> VCC (3.3 V)
- - - green   -> Pin A0
- - - red     -> GND
- - - black   -> Pin 2 

Arduino - BUS-Coupling-Unit
- - RX    -> TX
- - TX    -> RX
- - RAW   -> VCC
- - GND   -> GND

Arduino - TTL
- - Pin 12  -> TX
- - Pin 13  -> RX
- - RAW     -> 5V (VCC)
- - GND     -> GND

How the scetch works:
The wind speed is measured over 2.5 seconds.
If a measurement was successful, the wind direction is also determined.
This values are then evaluated in order to transmit this values and alarm messages to the bus using reference values.
Attention was paid to not overload the bus unnecessarily: 
Values are only send, if there was a change to previous measurements, 
or if a time of approx. 30 seconds has elapsed (12 times with waiting 2.5 sek for Davis speed meassurement).

The high_speed value is updated every 30 seconds at the latest.
This is required for shutter actuator to detect a failure of the weather station, so that the shutters are opened as a precaution. 
This actuator expects (depending on the paramater settings in the ETS) at least one update of the value within one minute (if used)

SPEED ALARM:
Speed at which the shutters should be opened automatically for safety reasons.
Can be set by the ETS / or via the group address (via homeassistant...)

DIFF ALARM:
Speed difference between two measurements (2x/min) at which the blinds should be opened automatically for safety reasons.
Can be set by the ETS / or via the group address (via homeassistant...)

VANE OFFSET:
This is the value of how much the anemometer turns away from the northern point.
Can be set by the ETS / or via the group address (via homeassistant...)

© doubery
copy, share and change this code by specifying the original code writer is allowed thx.
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

// Include librarys
#include "TimerOne.h" // Timer Interrupt set to 2 second for read sensors 
#include <KnxTpUart.h> // knx communication
#include <EEPROM.h> // used to read and write eeprom
#include <SoftwareSerial.h> // used to setup a second serial port for debugging

// Define pin connections
#define WindSensorPin 2 // pin location of the anemometer sensor 
#define WindVanePin A0 // pin the wind vane sensor is connected to 

// Define KNX Group-Addresses
#define SPEED_GROUP_ADDRESS             "10/1/10" // Groupaddress for the windspeed value
#define DIRECTION_NAME_GROUP_ADDRESS    "10/1/11" // Groupaddress for the direction name
#define DIRECTION_GROUP_ADDRESS         "10/1/12" // Groupaddress for the direction in degree
#define ERROR_GROUP_ADDRESS             "10/1/13" // Groupaddress to transmit error messeges (ASCII)

#define ALARM_GROUP_ADDRESS             "10/1/14" // Groupaddress to set the alarm (bool)  
	//(True, if difference between actual speed and last speed is over diff_alerm value)
	
#define HIGH_ALARM_GROUP_ADDRESS        "10/1/15" // Groupaddress to set high alarm (bool) 
	//(True, if speed is over speed_alarm value, also it is updated every 30sec, to trigger wind alarm function in Jal actor, so if the sensor doesn't work the jal's are opened for safety)

#define DIFF_ALARM_GROUP_ADDRESS        "10/1/16" // Groupaddress to recieve a diff value (km/h)
#define SPEED_ALARM_GROUP_ADDRESS       "10/1/17" // Groupaddress to recieve a speed value (km/h)
#define DIFF_ALARM_STATE_GROUP_ADDRESS  "10/1/18" // Groupaddress to request the diff value (km/h)
#define SPEED_ALARM_STATE_GROUP_ADDRESS "10/1/19" // Groupaddress to request the speed value (km/h)
#define VANE_OFFSET_GROUP_ADDRESS       "10/1/20" // Groupaddress to recieve a float offset value of the vane
#define VANE_OFFSET_STATE_GROUP_ADDRESS "10/1/21" // Groupaddress to request the offset value of the vane

// Define "virtual" Physical-Address !!!ATTANTION!!! this address has to be free in the ETS project
KnxTpUart knx(&Serial, "10.1.1");

// Set up SoftwareSerial pins
const byte rxPin = 12;
const byte txPin = 13;

// Set up a new SoftwareSerial object to show data on seperate serial
SoftwareSerial mySerial (rxPin, txPin);

bool high_speed; // bool if speed is high or low
bool alarm; // bool if alarm is high or low
bool success; // bool if data was send to bus

int counter; // counter 0 - 15 meassure loop-number  
int vaneValue; // raw analog value from wind vane 
int windDirection; // translated 0 - 360 direction 
int windCalDirection; // converted value with offset applied 
int windCalDirection_old; // converted value with offset applied 
int vane_offset; // define the anemometer offset from magnetic north 
int vane_offset_state_alarm; // define the anemometer state offset from magnetic north 

String windCompassDirection; // wind direction as compass points
String dirTable[]= {"N","NNO","NO","ONO","O","OSO","SO","SSO","S","SSW","SW","WSW","W","WNW","NW","NNW"}; // wind direction compass names
String lastWindDirectionValue; // last direction value 

volatile bool IsSampleRequired; // this is set true every 2.5s. Get wind speed 
volatile unsigned int TimerCount; // used to determine 2.5sec timer count 
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine 
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in isr 

float WindSpeed; // speed miles per hour 
float speedkmh; // speed in km/h
float speedkmh_old; //speed in km/h of last measurement
float speedkmh_last; //speed in km/h of last measurement and check if this value is higher than diff_alarm
float diff_alarm; // diff alarm in km/h (this value ist used to check if the windspeed is rised over in between 15 measurements)
float speed_alarm; // speed alarm in km/h (this value os used to check if the windspeed is over this value)
float diff_state_alarm; // diff alarm in km/h (this value ist used to check if the windspeed is rised over in between 15 measurements)
float speed_state_alarm; // speed alarm in km/h (this value os used to check if the windspeed is over this value)



void setup() {

  // Start the serial connections (knx and the debug-serial-port on myserial)
  Serial.begin(19200, SERIAL_8E1);
  mySerial.begin(9600);


  mySerial.println("Windsensor starting");

  // set values for first time
  lastWindDirectionValue = ""; 
  IsSampleRequired = false; 

  speedkmh = 0.0;
  speedkmh_old = 999.0;
  speedkmh_last = 0.0;

  counter = 0;
  high_speed = false;
  alarm = false;
  
  TimerCount = 0; 
  Rotations = 0; // Set Rotations to 0 ready for calculations 
  
  pinMode(WindSensorPin, INPUT_PULLUP); // with pullup, no external pullup needed

  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING); // check if the WindSensorPin is detected by falling edge
  
  // Setup the timer interupt 
  Timer1.initialize(500000);// Timer interrupt every 2.5 seconds 
  Timer1.attachInterrupt(isr_timer);

  // reset knx communication (importend)
  knx.uartReset();

  delay(250);

  // define listening addresses
  knx.addListenGroupAddress(DIFF_ALARM_GROUP_ADDRESS);
  knx.addListenGroupAddress(SPEED_ALARM_GROUP_ADDRESS);
  knx.addListenGroupAddress(DIFF_ALARM_STATE_GROUP_ADDRESS);
  knx.addListenGroupAddress(SPEED_ALARM_STATE_GROUP_ADDRESS);
  knx.addListenGroupAddress(VANE_OFFSET_GROUP_ADDRESS);
  knx.addListenGroupAddress(VANE_OFFSET_STATE_GROUP_ADDRESS);
  delay(250);

  // check if there is a value on bus
  knx.groupRead(DIFF_ALARM_GROUP_ADDRESS);
  knx.groupRead(SPEED_ALARM_GROUP_ADDRESS);
  knx.groupRead(VANE_OFFSET_GROUP_ADDRESS);

  delay(250);

  // check the eeprom for diff_alarm value
  if (isnan(EEPROM.get(0, diff_alarm)) || !EEPROM.get(0, diff_alarm)) {

    mySerial.println("diff-alarm not found in eeprom!");

    EEPROM.put(0, 15.0);

    mySerial.println("written float data type to eeprom!");

    knx.groupWrite2ByteFloat(DIFF_ALARM_GROUP_ADDRESS, 15.0);

    diff_alarm = EEPROM.get(0, diff_alarm);
    mySerial.print("Diff Alarm Value: ");
    mySerial.println(diff_alarm);
  
  }

  // if there is no value in eeprom, set default
  else {

    diff_alarm = EEPROM.get(0, diff_alarm);
    mySerial.print("Diff Alarm Value: ");
    mySerial.println(diff_alarm);

    knx.groupWrite2ByteFloat(DIFF_ALARM_GROUP_ADDRESS, diff_alarm);

  }

  delay(250);

  // check the eeprom for speed_alarm value
  if (isnan(EEPROM.get(64, speed_alarm)) || !EEPROM.get(64, speed_alarm)) {

    mySerial.println("speed-alarm not found in eeprom!");

    EEPROM.put(64, 15.0);

    mySerial.println("written float data type to eeprom!");

    knx.groupWrite2ByteFloat(SPEED_ALARM_GROUP_ADDRESS, 15.0);

    speed_alarm = EEPROM.get(64, speed_alarm);
    mySerial.print("Speed Alarm Value: ");
    mySerial.println(speed_alarm);

  }

  // if there is a value in eeprom, print
  else {

    speed_alarm = EEPROM.get(64, speed_alarm);
    mySerial.print("Speed Alarm Value: ");
    mySerial.println(speed_alarm);

    knx.groupWrite2ByteFloat(SPEED_ALARM_GROUP_ADDRESS, speed_alarm);

  }

  // check the eeprom for vane_offset value
  if (isnan(EEPROM.get(128, vane_offset))) {

    mySerial.println("vane-offset not found in eeprom!");

    EEPROM.put(128, 0);

    mySerial.println("written float data type to eeprom!");

    knx.groupWrite2ByteInt(VANE_OFFSET_GROUP_ADDRESS, 0);

    vane_offset = EEPROM.get(64, vane_offset);
    mySerial.print("Vane Offset Value: ");
    mySerial.println(vane_offset);

  }

  // if there is a value in eeprom, print
  else {

    vane_offset = EEPROM.get(64, vane_offset);
    mySerial.print("Vane Offset Value: ");
    mySerial.println(vane_offset);

    knx.groupWrite2ByteInt(VANE_OFFSET_GROUP_ADDRESS, vane_offset);

  }

  mySerial.println("Windsensor ready");

}


void loop() {

  // start reading the windsensor
  readWind();

}


// function to read the values
void readWind() {  

    // check if sample is required
    if(IsSampleRequired) { 

      // get the wind direction of function
      getWindDirection(); 

      // get the values of eeprom
      diff_alarm = EEPROM.get(0, diff_alarm);
      speed_alarm = EEPROM.get(64, speed_alarm);

      // convert to mp/h using the formula V=P(2.25/T) 
      // V = P(2.25/2.5) = P * 0.9 
      WindSpeed = Rotations * 0.9; 
      Rotations = 0; // Reset count for next sample 
      
      // set this to start over
      IsSampleRequired = false; 

      // calculate the windspeed to km/h
      speedkmh = getKmh(WindSpeed);

      // Reset for next value 
      WindSpeed = 0.0; 

      // check if the actual windspeed minus the last known windspeed is over the diff_alarm value
      // to get the diff value of windspeed after measurent
      if ((speedkmh - speedkmh_last) > diff_alarm) {

        mySerial.print("Speed km/h ");
        mySerial.println(speedkmh);

        // check if the alarm is false to update this value if not
        if (alarm == false) {

          mySerial.print("Diff-Alarm over ");
          mySerial.println(diff_alarm);

          // check if the transmission was successfully
          success = knx.groupWrite2ByteFloat(SPEED_GROUP_ADDRESS, speedkmh);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("Speed successfully send to KNX-BUS");

          }

          else {

            mySerial.println("Speed not successfully send to KNX-BUS");

          }

          // set alaerm true, if the next meassurement is also over there is noting to change in bus-information
          alarm = true;

          // check if the transmission was successfully
          success = knx.groupWriteBool(ALARM_GROUP_ADDRESS, alarm);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("Alarm successfully send to KNX-BUS");

          }

          else {

            mySerial.println("Alarm not successfully send to KNX-BUS");

          }

        }

      }

      // reset the alarm at the bus
      else {

        if (alarm == true) {

          // set alarm false
          alarm = false;

          // check if the transmission was successfully
          success = knx.groupWriteBool(ALARM_GROUP_ADDRESS, alarm);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("Alarm successfully send to KNX-BUS");

          }

          else {

            mySerial.println("Alarm not successfully send to KNX-BUS");

          }       

        }

      }

      // set speedkmh_last for the next measurement
      speedkmh_last = speedkmh;

      // check if the actual speed is over the speed_alarm value
      if (speedkmh >= speed_alarm){

        mySerial.print("Windspeed over Speedlimit ");
        mySerial.println(speed_alarm);
        mySerial.print("Counter: ");
        mySerial.println(counter);

        // check if hihg_speed is low
        if (high_speed == false) {

          mySerial.print("Max Speed over ");
          mySerial.println(speedkmh);
          mySerial.println("Set highspeed = true");

          // check if the transmission was successfully
          success = knx.groupWrite2ByteFloat(SPEED_GROUP_ADDRESS, speedkmh);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("Speed successfully send to KNX-BUS");

          }

          else {

            mySerial.println("Speed not successfully send to KNX-BUS");

          }

          // reset high speed, if the next meassurement is also over there is noting to change in bus-information
          high_speed = true;

          // check if the transmission was successfully
          success = knx.groupWriteBool(HIGH_ALARM_GROUP_ADDRESS, high_speed);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("High Speed successfully send to KNX-BUS");

          }

          else {

            mySerial.println("High Speed not successfully send to KNX-BUS");
          
          }

        }

        // if ca. 30sek over 
        if (counter == 12) {

          mySerial.print("Windspeed over ");
          mySerial.print(speed_alarm);
          mySerial.println("km/h and counter = 12");

          // check if actual speed isn't the same like the old one
          if (speedkmh != speedkmh_old) {

            mySerial.print("Speed km/h ");
            mySerial.println(speedkmh);

            // check if the transmission was successfully
            success = knx.groupWrite2ByteFloat(SPEED_GROUP_ADDRESS, speedkmh);

            // if the transmission was successfull
            if (success == 1) {

              mySerial.println("Speed successfully send to KNX-BUS");

            }

            else {

              mySerial.println("Speed not successfully send to KNX-BUS");

            }

            // set the speedkmh_old value
            speedkmh_old = speedkmh;

          }

          // check if the wind compass direction has changed
          if (windCompassDirection != lastWindDirectionValue) {

            mySerial.print("Direction Compass ");
            mySerial.println(windCompassDirection);

            // check if the transmission was successfully
            success = knx.groupWrite14ByteText(DIRECTION_NAME_GROUP_ADDRESS, windCompassDirection);

            // if the transmission was successfull
            if (success == 1) {

              mySerial.println("Direction Compass successfully send to KNX-BUS");

            }

            else {

              mySerial.println("Direction Compass not successfully send to KNX-BUS");

            }

            // set the lastWindDirectionValue 
            lastWindDirectionValue = windCompassDirection;

          }
          
          // check if the direction value has changed
          if (windCalDirection != windCalDirection_old) {

            mySerial.print("Direction Value ");
            mySerial.println(windCalDirection);

            // check if the transmission was successfully
            success = knx.groupWrite2ByteInt(DIRECTION_GROUP_ADDRESS, windCalDirection);

            // if the transmission was successfull
            if (success == 1) {

              mySerial.println("Direction Value successfully send to KNX-BUS");

            }

            else {

              mySerial.println("Direction Value not successfully send to KNX-BUS");

            }

            // set the windCalDirection_old value
            windCalDirection_old = windCalDirection;

          }

          //Simply send an update for the wind alarm function in Jal, as this must be updated cyclically, 
          //so that the Jal can safely close the external venetian blind in an emergency (without a weather station) (then the blind is blocked!)
          // check if the transmission was successfully
          success = knx.groupWriteBool(HIGH_ALARM_GROUP_ADDRESS, high_speed);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("High Speed successfully send to KNX-BUS");

          }

          else {

            mySerial.println("High Speed not successfully send to KNX-BUS");

          }
          
          // set counter 0 for the next loop round
          counter = 0;

        }

      }

      else {

        mySerial.print("Windspeed under Speedlimit "); 
        mySerial.println(speed_alarm);
        mySerial.print("Counter: ");
        mySerial.println(counter);

        // check if high_speed is true
        if (high_speed == true) {

          mySerial.println("Set high_speed = false");
          high_speed = false;

          // check if the transmission was successfully
          success = knx.groupWriteBool(HIGH_ALARM_GROUP_ADDRESS, high_speed);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("High Speed successfully send to KNX-BUS");

          }

          else {

            mySerial.println("High Speed not successfully send to KNX-BUS");

          }

        }

        // check if counter has reached 12
        if (counter == 12) {

          mySerial.print("Windspeed under ");
          mySerial.print(speed_alarm);
          mySerial.println("km/h and counter = 12");

          // check if the actual speed is different of the old speed value
          if (speedkmh != speedkmh_old) {

            mySerial.print("speedkmh: ");
            mySerial.println(speedkmh);
            mySerial.print("speedkmh_old: ");
            mySerial.println(speedkmh_old);

            // check if the transmission was successfully
            success = knx.groupWrite2ByteFloat(SPEED_GROUP_ADDRESS, speedkmh);

            // if the transmission was successfull
            if (success == 1) {

              mySerial.println("Wind Speed successfully send to KNX-BUS");

              }

              else {

              mySerial.println("Wind Speed not successfully send to KNX-BUS");

              }

            speedkmh_old = speedkmh;

          }

          // check if the Direction Compass has changed
          if (windCompassDirection != lastWindDirectionValue) {

            mySerial.print("Winddirection Compass: ");
            mySerial.println(windCompassDirection);

            // check if the transmission was successfully
            success = knx.groupWrite14ByteText(DIRECTION_NAME_GROUP_ADDRESS, windCompassDirection);

            // if the transmission was successfull
            if (success == 1) {

              mySerial.println("Direction Compass successfully send to KNX-BUS");

              }

              else {

              mySerial.println("Direction Compass not successfully send to KNX-BUS");

              }

            // set the lastWindDirectionValue
            lastWindDirectionValue = windCompassDirection;

          }
          
          // check if the Direction Value has changed
          if (windCalDirection != windCalDirection_old) {

            mySerial.print("Winddirection Value: ");
            mySerial.println(windCalDirection);

            // check if the transmission was successfully
            success = knx.groupWrite2ByteInt(DIRECTION_GROUP_ADDRESS, windCalDirection);

            // if the transmission was successfull
            if (success == 1) {

              mySerial.println("Direction Value successfully send to KNX-BUS");

              }

            else {

              mySerial.println("Direction Value not successfully send to KNX-BUS");
            
            }

            // set the windCalDirectionValue_old
            windCalDirection_old = windCalDirection;

          }

          //Simply send an update for the wind alarm function in Jal, as this must be updated cyclically, 
          //so that the Jal can safely close the external venetian blind in an emergency (without a weather station) (then the blind is blocked!)
          success = knx.groupWriteBool(HIGH_ALARM_GROUP_ADDRESS, high_speed);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("High Speed successfully send to KNX-BUS");

          }

          else {

            mySerial.println("High Speed not successfully send to KNX-BUS");
            
          }

          // set counter 0 for the next loop round
          counter = 0;

        }

      }
    
    // set counter + 1 for the next loop round
    counter++;

    }

}


// function to respond to a request via serial interface
void serialEvent() {

  // check if there is traffic on the serial port
  if (Serial.available() > 0) {

    mySerial.println("Data recieved");

    KnxTpUartSerialEventType eType = knx.serialEvent();

    // check if the recieved data is a knx tegram
    if (eType == KNX_TELEGRAM) {

      KnxTelegram* telegram = knx.getReceivedTelegram();

      // strip the recieved groupaddress
      String target =
      String(0 + telegram->getTargetMainGroup()) + "/" +
      String(0 + telegram->getTargetMiddleGroup()) + "/" +
      String(0 + telegram->getTargetSubGroup());

      mySerial.print("Destinationaddress: ");
      mySerial.println(target);

      // check if the telegram is a request to send information
      if (telegram->getCommand() == KNX_COMMAND_READ) {

        // check if the groupaddress is the DIFF_ALARM_STATE_GROUP_ADDRESS
        if (target == DIFF_ALARM_STATE_GROUP_ADDRESS) {

          // read value of eeprom
          diff_state_alarm = EEPROM.get(0, diff_alarm);

          // check if the request was successfully
          success = knx.groupWrite2ByteFloat(DIFF_ALARM_STATE_GROUP_ADDRESS, diff_state_alarm);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("Diff Alarm State Value successfully send to KNX-BUS");

          }

          else {

            mySerial.println("Diff Alarm State Value not successfully send to KNX-BUS");
            
          }

        } 
    
        // check if the groupaddress is the SPEED_ALARM_STATE_GROUP_ADDRESS
        else if (target == SPEED_ALARM_STATE_GROUP_ADDRESS) {

          // read value of eeprom
          speed_state_alarm = EEPROM.get(64, speed_alarm);

          // check if the request was successfully
          success = knx.groupWrite2ByteFloat(SPEED_ALARM_STATE_GROUP_ADDRESS, speed_state_alarm);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("Speed Alarm State Value successfully send to KNX-BUS");

          }

          else {

            mySerial.println("Speed Alarm State Value not successfully send to KNX-BUS");
            
          }

        } 

        // check if the groupaddress is the VANE_OFFSET_STATE_GROUP_ADDRESS
        else if (target == VANE_OFFSET_STATE_GROUP_ADDRESS) {

          // read value of eeprom
          vane_offset_state_alarm = EEPROM.get(128, vane_offset);

          // check if the request was successfully
          success = knx.groupWrite2ByteInt(VANE_OFFSET_STATE_GROUP_ADDRESS, vane_offset_state_alarm);

          // if the transmission was successfull
          if (success == 1) {

            mySerial.println("Vane Offset State Value successfully send to KNX-BUS");

          }

          else {

            mySerial.println("Vane Offset State Value not successfully send to KNX-BUS");
            
          }

        } 

      } 
      
      // check if the telegram is a request to get some value
      else if (telegram->getCommand() == KNX_COMMAND_WRITE) {

        // check if the groupaddress is the DIFF_ALARM_GROUP_ADDRESS
        if (target == DIFF_ALARM_GROUP_ADDRESS) {

          mySerial.print("Recieved Value: ");

          float value1 = telegram->get2ByteFloatValue();
          mySerial.println(value1);

          knx.groupAnswer2ByteFloat(target, value1);

          // check if the value exist in the eeprom
          if (value1 != EEPROM.get(0, diff_alarm)) {

              // if not, save this value to eeprom
              EEPROM.put(0, value1);

          }

        } 
    
        // check if the groupaddress is the SPEED_ALARM_GROUP_ADDRESS
        else if (target == SPEED_ALARM_GROUP_ADDRESS) {

          mySerial.print("Recieved Value: ");

          float value2 = telegram->get2ByteFloatValue();
          mySerial.println(value2);

          knx.groupAnswer2ByteFloat(target, value2);

          // check if the value exist in the eeprom
          if (value2 != EEPROM.get(64, speed_alarm)) {

              // if not, save this value to eeprom
              EEPROM.put(64, value2);

          }

        }

        // check if the groupaddress is the VANE_OFFSET_GROUP_ADDRESS
        else if (target == VANE_OFFSET_GROUP_ADDRESS) {

          mySerial.print("Recieved Value: ");

          int value3 = telegram->get2ByteIntValue();
          mySerial.println(value3);

          knx.groupAnswer2ByteInt(target, value3);

          // check if the value exist in the eeprom
          if (value3 != EEPROM.get(128, vane_offset)) {

              // if not, save this value to eeprom
              EEPROM.put(128, value3);

          }

        }

      }

    }

  }

}


// isr handler for timer interrupt 
void isr_timer() { 

  TimerCount++; 
  
  if (TimerCount == 6) { 

    IsSampleRequired = true; 
    TimerCount = 0; 

  } 

} 


// this is the function that the interrupt calls to increment the rotation count 
void isr_rotation() { 

  if((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact. 

    Rotations++; 
    ContactBounceTime = millis(); 

  } 

}

// get Wind Direction 
void getWindDirection() { 

  // read the analog value and transorm this 1024 bit to 360 degree
  vaneValue = analogRead(WindVanePin); 
  windDirection = map(vaneValue, 0, 1023, 0, 360); 

  vane_offset = EEPROM.get(128, vane_offset);
  windCalDirection = windDirection + vane_offset; 

  // check if anemometer offset is over 360 from magnetic north
  if(windCalDirection > 360) {
    
    // calculate new value
    windCalDirection = windCalDirection - 360; 

  }

  // check if anemometer offset is under 0 from magnetic north
  if(windCalDirection < 0) {
    
    // calculate new value
    windCalDirection = windCalDirection + 360; 

  }

  windCompassDirection = dirTable[int(windCalDirection/22.5)];

} 

// convert MPH to Knots 
float getKnots(float speed) { 

  return speed * 0.868976; 

} 

// convert MPH to km/h
float getKmh(float speed) { 

  return speed * 1.609; 

} 
