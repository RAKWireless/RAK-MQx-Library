/**
 * @file RAK12004_MQ2_Sampling.ino
 * @author rakwireless.com
 * @brief use MQ2 gas sensor read sensor data.
 * @version 0.1
 * @date 2021-05-08
 * @copyright Copyright (c) 2021
 */
#include <Wire.h>
#include "ADC121C021.h"

#define EN_PIN WB_IO6	 //Logic high enables the device. Logic low disables the device
#define ALERT_PIN WB_IO5 //a high indicates that the respective limit has been violated.
#define MQ2_ADDRESS 0x51 //the device i2c address

#define RatioMQ2CleanAir (1.0) //RS / R0 = 1.0 ppm
#define MQ2_RL (10.0)		   //the board RL = 10KΩ  can adjust

ADC121C021 MQ2;

void setup()
{
	pinMode(ALERT_PIN, INPUT);
	pinMode(EN_PIN, OUTPUT);
	digitalWrite(EN_PIN, HIGH); //power on RAK12004
	delay(500);
	time_t timeout = millis();
	Serial.begin(115200);
	while (!Serial)
	{
		if ((millis() - timeout) < 5000)
		{
			delay(100);
		}
		else
		{
			break;
		}
	}

	//********ADC121C021 ADC convert init ********************************
	while (!(MQ2.begin(MQ2_ADDRESS, Wire)))
	{
		Serial.println("please check device!!!");
		delay(200);
	}
	Serial.println("RAK12004 test Example");

	//*******config ADC121C021 *******************************************
	MQ2.configCycleTime(CYCLE_TIME_32); //set ADC121C021 Conversion Cycle Time
	MQ2.configAlertHold(Disable);		//set ADC121C021 alerts  self-clear
	MQ2.configAlertFlag(Disable);		//Disables ADC121C021 alert status bit [D15] in the Conversion Result register.
	MQ2.configAlertPin(Enable);			//Enables the ALERT output pin of ADC121C021
	MQ2.configPolarity(High);			//Sets ADC121C021 ALERT pin to active high
	MQ2.setAlertLowThreshold(1);		//set ADC121C021 Alert low Limit the register default value is 0
	MQ2.setAlertHighThreshold(2500);	//set ADC121C021 Alert high Limit when adc value over 3000 alert PIN output level to high

	//**************init MQ2********************************************
	MQ2.setRL(MQ2_RL);
	MQ2.setA(-0.890);			//A -> Slope, -0.685
	MQ2.setB(1.125);			//B -> Intersect with X - Axis  1.019
								//Set math model to calculate the PPM concentration and the value of constants
	MQ2.setRegressionMethod(0); //PPM =  pow(10, (log10(ratio)-B)/A)

	float calcR0 = 0;
	for (int i = 1; i <= 100; i++)
	{
		calcR0 += MQ2.calibrateR0(RatioMQ2CleanAir);
	}
	MQ2.setR0(calcR0 / 10);
	if (isinf(calcR0))
	{
		Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply");
		while (1)
			;
	}
	if (calcR0 == 0)
	{
		Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");
		while (1)
			;
	}
	Serial.printf("R0 Value is:%3.2f\r\n", MQ2.getR0());
}
void loop()
{
	float sensorPPM;
	float PPMpercentage;

	Serial.println("Getting Conversion Readings from ADC121C021");
	Serial.println(" ");
	sensorPPM = MQ2.readSensor();
	Serial.printf("sensor PPM Value is: %3.2f\r\n", sensorPPM);
	PPMpercentage = sensorPPM / 10000;
	Serial.printf("PPM percentage Value is:%3.2f%%\r\n", PPMpercentage);
	Serial.println(" ");
	Serial.println("        ***************************        ");
	Serial.println(" ");

	uint8_t PINstatus = digitalRead(ALERT_PIN);
	if (PINstatus)
	{
		Serial.println("value over AlertHighThreshold !!!");
	}

	delay(1000);
}
