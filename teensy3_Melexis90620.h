/*
* A class for interfacing the Melexis 90620 Sensor from a Teensy 3.0++
* Uses the 2c_t3 Library for Communication. You have to install the library and put "#include <i2c_t3.h>" into your sketch...
*
* Connection Instructions:
* Connect the Anode of a Silicon Diode to 3V Pin of Teensy. The Diode will drop ~0.7V, so the Cathode will be at ~2.7V. These 2.7V will be the supply Voltage for the sensor.
* Connect Teensy Pin 18 to 2.7V with a 4.7kOhm Resistor (Pullup)
* Connect Teensy Pin 19 to 2.7V with a 4.7kOhm Resistor (Pullup)
* Connect Teensy Pin 18 to I2C Data (SDA) Pin of Sensor
* Connect Teensy Pin 19 to I2C clock (SCL) Pin of Sensor
* Connect the VSS Pin of the Sensor to GND.
* Connect the VDD Pin of the Sensor to 2.7V
* Connect GND and 2.7V with a 100nF ceramic Capacitor.
*---> You are ready to go!
*
* Some of the communication was taken from the Thread http://forum.arduino.cc/index.php?topic=126244.0
* Most was taken from the Datasheet...
* Everything else was written 2013 by Felix Bonowski
*  
* This code is in the public domain.
*
*/

#pragma once
#include <Arduino.h>
#include <i2c_t3.h>

// I2C Addresses of the sensor and calibration data 
const uint8_t MLX90620SensorAddr=0x60;
const uint8_t MLX90620EepromAddr=0x50;

class Melexis90620{
public:
  /// set up the sensor and set the framerate. You have to do this before the sensor will do anything useful
  void init(int newFrameRate){ 
    frameRate=newFrameRate; //remember framerate in case we have to write the configuration register again.
    readEeprom();
    
    //!!!FIXME!!! the calibration offset values of the first 8 pixel in my sensor seem to be broken, so I just copy those from the last two. You don't want that!
    for(int i=0;i<8;i++){
      EEPROM_DATA[i]=EEPROM_DATA[63-i];
      //EEPROM_DATA[i+64]=EEPROM_DATA[63-i+64];   
    }
    
    writeTrimmingValue(EEPROM_DATA[0xF7]);
    writeSensorConfiguration(newFrameRate);
    initializeCalibrationConstants(EEPROM_DATA);
  };
  
  /// goes through all steps necessary for getting fresh temperature values.
  void readTemperatures(){
    resetIfPowerWasLost();
    readRawPackageTemperature(); //read ambient temperature
    readRawPixelData();  // read pixel data
    readRawCorrectionPixel(); //read correction pixel data
    calculateTA();
    calculateTO();
  };
  
  /// returns an array with the last measuerments. Pixel at 3rd row, 16th column can be found at index (16-1)*4=63
  double* getPixelTemperatures(){return pixelTemperatures;}; 
  
  /// returns last measured package temperature
  double getPackageTemperature(){return ta;};  
  
  /// print the temperature readings as a tab separated 4x16 table
  void sendTemperaturesToSerial(){
    for(int x=0;x<4;x++){
      for(int y=0;y<16;y++){
        Serial.print(pixelTemperatures[x+y*4]);
        //Serial.print(rawPixelData[x+y*4]);
        Serial.print("\t");
      }
      Serial.print("\n");
    }
  }

private:
  // the IR sensor readout framerate. Valid values are 1,2,4,8,16 and 32. The slower you read, the less noise you get.
  int frameRate;

  // calibrated sensor readings - this is what is probably most interesting for you!
  double pixelTemperatures[64]; // the calculated pixelTemperatures
  double ta; //the temperature of the sensor package

  //raw sensor readings
  uint16_t correctionPixelRaw; 
  int16_t rawPixelData[64];    // the last raw pixel data read from the sensor
  uint16_t PTAT;               // the last raw data read from the ambient temperature sensor

  // configuration and calibration data as read from the sensor
  uint8_t EEPROM_DATA[256]; // the raw calibration data read from the sensor
  uint16_t cfgRegister; // last Data read from the configuration register of the sensor

    // all following values are calibration constants derived from the raw EEPROM_DATA data
  int  v_th;   // package temperature sensor offset calibration constant (see datasheet section 7.3.1)
  double k_t1; // package temperature sensor - some other calibration constant (see datasheet section 7.3.1)
  double k_t2; // package temperature sensor - some other calibration constant (see datasheet section 7.3.1)

  double  emissivity; // emmissivity of the material we look at (this can be changed to get corrrect temperature readings when looking at different surface materials)

  int a_cp, b_cp, tgc ; //compensation pixel calibration constants: offset , ambient temperature dependence, influence

  int b_i_scale; //scaling factor for pixel ambient temperature dependence
  int a_ij[64]; //pixel offsets 
  int b_ij[64]; // pixel ambient temperature dependence
  double alpha_ij[64]; //pixel scaling

  ///////////////////////////////////
  // functions for converting raw measurements into calibrated temperature readings
  ////////////////////////////////////

  //calculate temperature of the sensor package (Melexis calls it "absolute" temperature) from the measured raw data (PTAT)
  void calculateTA(){ 
    // this formula can be found on page 14 (Section 7.3.1) of the Datasheet.
    ta = (-k_t1 + sqrt(square(k_t1) - (4 * k_t2 * (v_th - (double)PTAT))))/(2*k_t2) + 25.0;
  }

  //calculate the pixel pixelTemperatures from the raw-data.
  void calculateTO(){
    // some offset correction value derived from data from the "thermal gradient compensation pixel" raw data (correctionPixelRaw)
    double v_cp_off_comp = (double) correctionPixelRaw - (a_cp + (b_cp/pow(2, b_i_scale)) * (ta - 25)); // See 7.3.3.1 Part 2. of the Datasheet

    // perform calibration computation for all the pixels
    for (int i=0; i<64; i++){
      //first, we calculate some value that seems to be corrected for thermal radiation inside the package (Formula from 7.3.3.1 in the Datasheet)
      double v_ir_tgc_comp = rawPixelData[i] - (a_ij[i] + (double)(b_ij[i]/pow(2, b_i_scale)) * (ta - 25)) - (((double)tgc/32)*v_cp_off_comp);
      // them we do some more calibration magic (First formula in Section 7.3.3.1)
      pixelTemperatures[i] = sqrt(sqrt((v_ir_tgc_comp/alpha_ij[i]) + pow((ta + 273.15),4))) - 273.15;	//edited to work with v_ir_tgc_comp instead of v_ir_comp
    }
  }

  /////////////////////////////////////////////////////////////////  
  // functions for calculating calibration constants
  /////////////////////////////////////////////////////////////////
  double square(double value){
    return value*value;
  };
  
  // initialize all kinds of calibration factors from EEPROM-data
  void initializeCalibrationConstants(byte EEPROM_DATA[]){

    //some calibration values of the package temperature sensor Datasheet section 7.3.1
    v_th = (EEPROM_DATA[219] <<8) + EEPROM_DATA[218];
    k_t1 = ((EEPROM_DATA[221] <<8) + EEPROM_DATA[220])/1024.0;
    k_t2 =((EEPROM_DATA[223] <<8) + EEPROM_DATA[222])/1048576.0;

    // the emissivity of the object we look at (1.0 for an ideal black body). actually, there is no reason to read this from EEPROM
    emissivity = (((unsigned int)EEPROM_DATA[229] << 8) + EEPROM_DATA[228])/32768.0;

    //calibration values of the "thermal gradient compensation pixel" (Datasheet section 7.3.3.1)
    a_cp = EEPROM_DATA[212]; 
    if(a_cp > 127)a_cp = a_cp - 256; //the values are stored as "two complement" in the EEPROM. this calculation takes care of the correct sign
    b_cp = EEPROM_DATA[213];
    if(b_cp > 127) b_cp = b_cp - 256;//the values are stored as "two complement" in the EEPROM. this calculation takes care of the correct sign

    //scaling of the thermal gradient compensation
    tgc = EEPROM_DATA[216];
    if(tgc > 127)tgc = tgc - 256;//the values are stored as "two complement" in the EEPROM. this calculation takes care of the correct sign


    //he pixel dependent sensitivities are termed "alpha" in the datasheet Section7.3.3
    //calculate global offset of the pixel dependent "alpha" values (first part of the formula in Datasheet Section 7.3.3.2)
    double alpha_offset=EEPROM_DATA[0xE0]+256*EEPROM_DATA[0xE1]; //this corresponds to alpha_0 in the datasheet
    alpha_offset=alpha_offset*pow(2,-EEPROM_DATA[0xE2]);//this corresponds to dividing alpha_0 by 2^alpha_0_scale

    //calculate a scaling coefficient for the pixel dependent part of the alpha-values
    double delta_alpha_mult=pow(2,-EEPROM_DATA[0xE3]);

    //  "scaling coefficient for slope of IR pixels offset"
    b_i_scale = EEPROM_DATA[217];

    //initialize Aij and Bij, the offsets and ambient tempereature dependence of the pixels
    for(int i=0;i<=63;i++){

      //a_ij is a pixel dependent offset value
      a_ij[i] = EEPROM_DATA[i];
      if(a_ij[i] > 127){
        a_ij[i] = a_ij[i] - 256;//the values are stored as "two complement" in the EEPROM. this calculation takes care of the correct sign
      }

      //b_ij is a value that describes how much the ambient temperature influences each pixel
      b_ij[i] = EEPROM_DATA[64+i];
      if(b_ij[i] > 127){
        b_ij[i] = b_ij[i] - 256;//the values are stored as "two complement" in the EEPROM. this calculation takes care of the correct sign
      }

      //calculate alphaij, some pixel dependent scaling coefficients
      alpha_ij[i]=alpha_offset+(double)(EEPROM_DATA[128+i])*delta_alpha_mult;

    }
  };

  ////////////////////////////////////
  // Functions for low level communication with the sensor
  ////////////////////////////////////

  //read EEPROM raw data from the sensor. It contains the calibration data.
  void readEeprom(){
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    //read all the data from eeprom

    // tell the EEPROM to send us all the data
    Wire.beginTransmission(MLX90620EepromAddr); // I2C slave address of the eeprom in the melexis 
    Wire.write(0x00); // start address of eeprom read 
    Wire.endTransmission(I2C_NOSTOP);
    Wire.requestFrom(MLX90620EepromAddr,255,I2C_STOP);
    //now we let the data flow...
    for(int i=0;i<255;i++){
      EEPROM_DATA[i]=Wire.readByte();
      //Serial.println(EEPROM_DATA[i]);
    }
  };

  //writes an Oscilator trimming value to the sensor that makes it more precise. The value is stored in EEPROM Adress 0xF7 (EEPROM_DATA[0xF7])
  void writeTrimmingValue(uint8_t val){
    Wire.beginTransmission(0x60); // I2C slave address of the sensor in the melexis 
    Wire.write(0x04); // command
    Wire.write((uint8_t)((uint8_t)val-(uint8_t)0xAA)); 
    Wire.write(val);  
    //high byte is assumed to be 0
    Wire.write((uint8_t)0x56);  
    Wire.write((uint8_t)0x00);
    Wire.endTransmission(I2C_STOP);
  };

  void writeSensorConfiguration(int Hz){
    uint8_t Hz_LSB;
    switch(Hz){
    case 0:
      Hz_LSB = B00001111;
      break;
    case 1:
      Hz_LSB = B00001110;
      break;
    case 2:
      Hz_LSB = B00001101;
      break;
    case 4:
      Hz_LSB = B00001100;
      break;
    case 8:
      Hz_LSB = B00001011;
      break;
    case 16:
      Hz_LSB = B00001010;
      break;
    case 32:
      Hz_LSB = B00001001;
      break;
    default:
      Hz_LSB = B00001110;
    }
    Wire.beginTransmission(0x60); // I2C slave address of the sensor in the melexis 
    Wire.write(0x03);

    //this take the refresh rate into account:
    Wire.write((uint8_t)((uint8_t)Hz_LSB-(uint8_t)0x55));  // checksum of LSB is calculated by substracting 0x55
    Wire.write(Hz_LSB);   

    // this uses some default values from the datasheet instead
    //Wire.write(0xB9); 
    //Wire.write(0x0E);   

    //what follows is some standard value for the MSB of the configuration register:
    Wire.write(0x1F);  
    Wire.write(0x74);  
    Wire.endTransmission();
  };

  void readRawPackageTemperature(){
    Wire.beginTransmission(0x60); // I2C slave address of the thermocam status register 
    Wire.write(0x02); // kommando gib mit die temperatur
    Wire.write(0x90); // erste adresse, die gelesen werden soll
    Wire.write(0x00); // "adress step" nach 
    Wire.write(0x01);
    Wire.endTransmission(I2C_NOSTOP);

    //now we get the requested data...
    Wire.requestFrom(0x60,2);
    uint8_t PTAT_LSB = Wire.readByte();
    uint8_t PTAT_MSB = Wire.readByte();
    PTAT = ((uint16_t)PTAT_MSB << 8) + PTAT_LSB;
  }


  void readRawPixelData(){
    // send request for pixel data read
    Wire.beginTransmission(MLX90620SensorAddr); // I2C slave address of the thermocam
    Wire.write(0x02);      
    Wire.write(0x00);     
    Wire.write(0x01);       
    Wire.write(0x40);
    Wire.endTransmission(I2C_NOSTOP);

    //now we get the requested data...
    Wire.requestFrom(MLX90620SensorAddr,64*2,I2C_NOSTOP);
    for(int i=0;i<=63;i++){
      uint8_t PIX_LSB = Wire.readByte(); 
      uint8_t PIX_MSB = Wire.readByte(); 
      rawPixelData[i] = (PIX_MSB << 8) + PIX_LSB;
    }

  };

  void readRawCorrectionPixel(){
    Wire.beginTransmission(MLX90620SensorAddr); // I2C slave address of the thermocam
    Wire.write(0x02);
    Wire.write(0x91);
    Wire.write(0x00);
    Wire.write(0x01);
    Wire.endTransmission(I2C_NOSTOP);

    //now we get the requested data...
    Wire.requestFrom(MLX90620SensorAddr,2,I2C_NOSTOP);
    uint8_t correctionPixelRaw_LSB = Wire.readByte();
    uint8_t correctionPixelRaw_MSB = Wire.readByte();
    correctionPixelRaw = (correctionPixelRaw_MSB << 8) + correctionPixelRaw_LSB;
  };

  void readConfigRegister(){
    Wire.beginTransmission(0x60); // I2C slave address of the thermocam
    Wire.write(0x02);
    Wire.write(0x92);
    Wire.write(0x00);
    Wire.write(0x01);
    Wire.endTransmission(I2C_NOSTOP);

    Wire.requestFrom(0x60,2);
    uint8_t CFG_LSB, CFG_MSB;
    CFG_LSB = Wire.readByte();
    CFG_MSB = Wire.readByte();
    cfgRegister= (CFG_MSB << 8) + CFG_LSB;

  };

  void resetIfPowerWasLost(){
    readConfigRegister();
    // rewrite configuration values to the sensor if the "brown-out flag" indicates that it has been reset.
    if ((!cfgRegister & 0x04<< 8) == 0x04<< 8){
      writeSensorConfiguration(frameRate);
    }
  }

};

