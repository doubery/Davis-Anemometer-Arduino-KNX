# Davis-Anemometer-Arduino-KNX
Connect Davis Anemometer to Arduino (Pro Mini 5V) send Data to KNX

This code is an extension of the following great projects:

https://github.com/wrybread/weewx-ArduinoWeatherStation

https://github.com/doubery/Davis-Anemometer-Arduino/tree/main

With the awsome work of:

https://github.com/thorsten-gehrig/arduino-tpuart-knx-user-forum


This is a sketch to connect a davis wind-sensor to a Arduino (Pro Mini 5V), for use on a knx network with a BUS-Coupling-Unit

# Wiring and Setup
Davis - Arduino Pro Mini (5V)
- yellow  -> VCC (3.3 V)
- green   -> Pin A0
- red     -> GND
- black   -> Pin 2 

Arduino - BUS-Coupling-Unit
- RX    -> TX
- TX    -> RX
- RAW   -> VCC
- GND   -> GND

Arduino - TTL
- Pin 12  -> TX
- Pin 13  -> RX
- RAW     -> 5V (VCC)
- GND     -> GND

#How the scetch works
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

# Installation / prepare for use
1. Load the latest library of "timerone"
2. Load the latest library of "KnxTpUart" 
3. Copy the code to your arduino ide
4. Flash the Arduino 
5. Open serial monitor with baudrate 9600
