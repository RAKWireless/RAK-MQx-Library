/**
 * @file ADC121C021.cpp
 * @author rakwireless.com
 * @brief  This code is designed to config ADC121C021 ADC device and handle the data get from MQ-X Sensors
 * @version 1.0
 * @date 2021-05-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _ADC121C021_H_
#define _ADC121C021_H_

#include "Arduino.h"
#include <Wire.h>

#define DEBUG_LEVEL 0
#define LOG Serial

/**************************************************************************
    I2C ADDRESS/BITS
**************************************************************************/
/*    ADC121C021021  Slave (Hardware) Address   
(0x50)      // 1010000      -----       Floating    Floating
(0x51)      // 1010001      -----       Floating    GND
(0x52)      // 1010010      -----       Floating    VA
(0x54)      // 1010100  Single Address  GND         Floating
(0x55)      // 1010101      -----       GND         GND
(0x56)      // 1010110      -----       GND         VA
(0x58)      // 1011000      -----       VA          Floating
(0x59)      // 1011001      -----       VA          GND
(0x5A)      // 1011010      -----       VA          VA
*/
/**************************************************************************
    POINTER REGISTER
**************************************************************************/
#define ADC121C021_REG_POINTER_MASK (0xFF)
#define ADC121C021_REG_POINTER_CONVERSION (0x00)
#define ADC121C021_REG_POINTER_ALERT_STATUS (0x01)
#define ADC121C021_REG_POINTER_CONFIG (0x02)
#define ADC121C021_REG_POINTER_LOW_LIMIT (0x03)
#define ADC121C021_REG_POINTER_HIGH_LIMIT (0x04)
#define ADC121C021_REG_POINTER_HYSTERESIS (0x05)
#define ADC121C021_REG_POINTER_LOWCONV (0x06)
#define ADC121C021_REG_POINTER_HIGHCONV (0x07)

/**************************************************************************
    CONFIG REGISTER
**************************************************************************/

#define ADC121C021_REG_CONFIG_ALERT_HOLD_MASK (0x10)	// Alert Hold
#define ADC121C021_REG_CONFIG_ALERT_HOLD_CLEAR (0xef)	// Alerts will self-clear when the measured voltage moves within the limits by more than the hysteresis register value
#define ADC121C021_REG_CONFIG_ALERT_FLAG_NOCLEAR (0x10) // Alerts will not self-clear and are only cleared when a one is written to the alert high flag or the alert low flag in the Alert Status register

#define ADC121C021_REG_CONFIG_ALERT_FLAG_MASK (0x08) // Alert Flag Enable
#define ADC121C021_REG_CONFIG_ALERT_FLAG_DIS (0xf7)	 // Disables alert status bit in the Conversion Result register
#define ADC121C021_REG_CONFIG_ALERT_FLAG_EN (0x08)	 // Enables alert status bit in the Conversion Result register

#define ADC121C021_REG_CONFIG_ALERT_PIN_MASK (0x04) // Alert Pin Enable
#define ADC121C021_REG_CONFIG_ALERT_PIN_DIS (0xfb)	// Disables the ALERT output pin. The ALERT output will TRI-STATE when the pin is disabled
#define ADC121C021_REG_CONFIG_ALERT_PIN_EN (0x04)	// Enables the ALERT output pin.

#define ADC121C021_REG_CONFIG_POLARITY_MASK (0x01) // Polarity
#define ADC121C021_REG_CONFIG_POLARITY_LOW (0xfe)  // Sets the ALERT pin to active low
#define ADC121C021_REG_CONFIG_POLARITY_HIGH (0x01) // Sets the ALERT pin to active high

#define CONFIG_CYCLE_TIME_MASK (0xE0)  // Configures Automatic Conversion Mode
#define AUTOMATIC_MODE_DISABLED (0x00) // Automatic Mode Disabled, 0 ksps
#define CYCLE_TIME_32 (0x20)		   // Tconvert x 32, 27 ksps
#define CYCLE_TIME_64 (0x40)		   // Tconvert x 64, 13.5 ksps
#define CYCLE_TIME_128 (0x60)		   // Tconvert x 128, 6.7 ksps
#define CYCLE_TIME_256 (0x80)		   // Tconvert x 256, 3.4 ksps
#define CYCLE_TIME_512 (0xA0)		   // Tconvert x 512, 1.7 ksps
#define CYCLE_TIME_1024 (0xC0)		   // Tconvert x 1024, 0.9 ksps
#define CYCLE_TIME_2048 (0xE0)		   // Tconvert x 2048, 0.4 ksps

typedef enum
{
	Enable = true,
	Disable = false
} configType;

typedef enum
{
	High = 1,
	Low = 0
} polarityType;

class ADC121C021
{
protected:
	// Instance-specific properties
	uint8_t adc_alertLow;  //status when the value under the low threshold
	uint8_t adc_alertHigh; //status when the value uper the high threshold
	bool adc_alert;		   //When came Alert this bit will be high Otherwise, this bit is a zero.

public:
	bool begin(uint8_t addr, TwoWire &theWire); //Sets up the Hardware
	uint8_t readConfigRegister();				//reads 8-bits data from the config register
	uint8_t readAlertStatus();					//Get the current status of the alert flag
	uint8_t readAlertLowStatus();				//Get the current status of the Under Range Alert/adc_alertLow
	uint8_t readAlertHighStatus();				//Get the current status of the Over Range Alert/adc_alertHigh
	void clearAlertStatus();					//Clears the alert low and alert high flags.

	void configCycleTime(uint8_t cycletime);	//Sets the Cycle Time
	uint8_t readCycleTime();					//Gets the Conversion Cycle Time
	void configAlertHold(configType alertHold); //Sets the Alert Hold Status
	uint8_t readAlertHold();					//Gets the Alert Hold Status
	void configAlertFlag(configType alertflag); //Sets the Alert Flag Enable Status
	uint8_t readAlertFlag();					//Gets the Alert Flag Enable Status
	void configAlertPin(configType alertpin);	//Sets the Alert Pin Enable Status
	uint8_t readAlertPin();						//Gets the Alert Pin Enable Status

	void configPolarity(polarityType polarity);		//Sets the Polarity
	uint8_t readPolarity();							//Gets the Polarity
	void setAlertLowThreshold(uint16_t threshold);	// Sets the lower limit threshold used to determine the alert condition
	uint16_t readAlertLowThreshold();				//read the lower limit threshold from register
	void setAlertHighThreshold(uint16_t threshold); //Sets the hysteresis value used to determine the alert condition
	uint16_t readAlertHighThreshold();				//read the upper limit threshold from register
	void setHysteresis(uint16_t hysteresis);		//Sets the upper limit threshold used to determine the alert condition
	uint16_t readHysteresis();						////read the hysteresis from register
	uint16_t readLowestConversion();				//Gets the Lowest Conversion result recorded so far
	void clearLowestConversion();					//Clears the Lowest Conversion value recorded so far
	uint16_t readHighestConversion();				//Gets the Highest Conversion result recorded so far
	void clearHighestConversion();					//Clears the Highest Conversion value recorded so far
	uint16_t getAdcValue();							//get adc transmission result
	//----------------------------MQx ----------------------------------------------------------
	//Functions to set and get values
	void setVoltageResolution(float value); //the _VOLT_RESOLUTION default is 5.0V if3.3V use 3.3
	float getVoltageResolution();			//readback the _VOLT_RESOLUTION

	void setR0(float R0 = 10); //set R0 (resister of sensor in the fresh air )
	float getR0();
	void setRL(float RL = 10); //set RL Value in KiloOhms ,Reference circuit
	float getRL();			   //get RL Value
	void setA(float K);
	float getA();
	void setB(float b);
	float getB();

	//user functions
	float getSensorVoltage();				  //get the voltage out of sensor
	float calibrateR0(float ratioInCleanAir); //calibrate R0 ()
	void setRegressionMethod(uint8_t method); //set regression method
	uint8_t getRegressionMethod();			  //get regression method
	float readSensor();						  //read sensor data return PPM

private:
	uint8_t i2c_addr; //the  deviceaddress
	TwoWire *_wire;	  //the i2c port interface

	void writeRegister8(uint8_t reg, uint8_t value);   //Writes 8-bits to the specified destination register
	void writeRegister16(uint8_t reg, uint16_t value); //Writes 16-bits to the specified destination register
	uint8_t readRegister8(uint8_t reg);				   //Reads 8-bits from the specified destination register
	uint16_t readRegister16(uint8_t reg);			   //Reads 16-bits from the specified destination register
													   //-----------------------MQx -----------------------------------------------------------------------
	float _VOLT_RESOLUTION = 5.0;					   // if 3.3v use 3.3
	uint8_t _regressionMethod = 1;					   // 1  Exponential  else Linear

	float _constantA; //Slope
	float _constantB; //Y-Intercept
	uint8_t _RL = 10; //Value in KiloOhms
	float _R0;		  //Sensor Resistance in fresh air
};
#endif
