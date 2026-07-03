#include <Arduino.h>
#include <IBusBM.h>
#include "fsRCcontrol.h"
#include "constants.h"

// Credit to DroneBot Workshop for this code: https://dronebotworkshop.com/radio-control-arduino-car/

// Create iBus Object
IBusBM ibus;
 
HardwareSerial Serial1(PB7, PB6); 

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
float readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  float ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 
// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
 
void setupFSRCcontrol() {
  // Start serial monitor
  Serial1.setTx(PB6);
  Serial1.setRx(PB7);
  // Attach iBus object to serial port
  ibus.begin(Serial1, IBUSBM_NOTIMER);
}
 
float getFSRCData(int channel) {
   ibus.loop(); 

  // Cycle through first 5 channels and determine values
  // Print values to serial monitor
  // Note IBusBM library labels channels starting with "0"
 
  float receivedVal = readChannel(channel - 1, -300, 300, 0);
  if (receivedVal >= Constants::deadzone || receivedVal <= -Constants::deadzone){
    return receivedVal;
  } else {
    return 0;
  }
 
  delay(10);
}