/*
* A class for interfacing the Melexis 90620 Sensor from a Teensy 3.0++
* Uses the 2c_t3 library for communication with the sensor
* 2013 by Felix Bonowski
* Based on a forum post by maxbot: http://forum.arduino.cc/index.php/topic,126244.msg949212.html#msg949212
* This code is in the public domain.
*
* Connection Instructions:
* Connect the Anode of a Silicon Diode to 3V Pin of Teensy. The Diode will drop ~0.7V, so the Cathode will be at ~2.7V. These 2.7V will be the supply voltage "VDD" for the sensor.
* Plug in the USB and measure the supply voltage with a multimeter! - it should be somewhere between 2.5V and 2.75V, else it will fry your precious sensor...
* ...disconnect USB again...
* Connect Teensy Pin 18 to 2.7V with a 4.7kOhm Resistor (Pullup)
* Connect Teensy Pin 19 to 2.7V with a 4.7kOhm Resistor (Pullup)
* Connect Teensy Pin 18 to I2C Data (SDA) Pin of Sensor
* Connect Teensy Pin 19 to I2C clock (SCL) Pin of Sensor
* Connect GND and 2.7V with a 100nF ceramic Capacitor.
* Connect the VSS Pin of the Sensor to GND.
* Connect the VDD Pin of the Sensor to 2.7V

*---> You are ready to go!
*/

#include <Arduino.h>
#include <i2c_t3.h>
#include "teensy3_Melexis90620.h"

Melexis90620 sensor; // create an instance of the Sensor class
void setup(){ 
  Serial.begin(115200);
  Serial.println("trying to initialize sensor...");
  sensor.init (16); // start the thermo cam with 16 frames per second
  Serial.println("sensor initialized!");
}

void loop(){
  sensor.readTemperatures(); //get new readings from the sensor
  Serial.println(sensor.getPackageTemperature());
  //sensor.sendTemperaturesToSerial(); //a convenient way to just print the data
  
  // this is how you can get the data if you want to do further processing:
  double* pixelTemps= sensor.getPixelTemperatures(); //get pixel temperatures for doing something with them
  for(int y=0;y<4;y++){ //go through all the rows
    for(int x=0;x<16;x++){ //go through all the columns
      double tempAtXY=pixelTemps[y+x*4]; // extract the temperature at position x/y
      Serial.print(tempAtXY);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
  delay(10);
};



