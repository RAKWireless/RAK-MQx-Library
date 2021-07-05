/**
 * @file ADC121C021.h
 * @author rakwireless.com
 * @brief This code is designed to config ADC121C021 ADC device and handle the data get from MQ-X Sensors
 * @version 1.0
 * @date 2021-05-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ADC121C021.h"
#include <Wire.h>

/**
 * @brief Init the ADC121C021 device
 * @param addr device i2c address
 * @param &theWire i2c port
 * @return true: init succeed  false: init failed
 */
bool ADC121C021::begin(uint8_t addr,TwoWire &theWire) //Sets up the Hardware
{
  i2c_addr = addr;
  _wire = &theWire;
  _wire->begin();
  // A basic scanner, see if it ACK's
  _wire->beginTransmission(i2c_addr);
  if (_wire->endTransmission() == 0) {
    return true;
  }
  return false;
}
/**
 * @brief i2c write byte data to specified 8-bit register
 * @param reg specified register address
 * @param value write byte data
 * @return void
 */
void ADC121C021::writeRegister8(uint8_t reg, uint8_t value)  
{  
  _wire->beginTransmission(i2c_addr);
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)value);  
  _wire->endTransmission(); 
}
/**
 * @brief i2c write word data to specified 8-bit register
 * @param reg specified register address
 * @param value write word data
 * @return void
 */
void ADC121C021::writeRegister16(uint8_t reg, uint16_t value) 
{ 
  _wire->beginTransmission(i2c_addr);
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)(value>>8));
  _wire->write((uint8_t)(value & 0xFF));
  _wire->endTransmission();
}
/**
 * @brief i2c read 8-bits data from specified register
 * @param reg specified register address
 * @return uint8_t data read from register
 */
uint8_t ADC121C021::readRegister8(uint8_t reg) 
{ 
    uint8_t data=0;
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom(i2c_addr,1); 
    data = (uint8_t)_wire->read();      
    return data;
}
/**
 * @brief i2c read 16-bits data from specified register
 * @param reg specified register address
 * @return uint16_t value read from the register
 */
uint16_t ADC121C021::readRegister16(uint8_t reg) 
{
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom(i2c_addr, 2);
    return (uint16_t)((_wire->read() << 8) | _wire->read());
}
/**
 * @brief Get the current status of the alert flag If the flag is set, 
 * the low or high alert indicators are set as appropriate.
 * @param none
 * @return alert status.
 */
uint8_t ADC121C021::readAlertStatus()
{
  uint16_t temp;
  uint8_t alertStatus;
  // high order bit is the alert flag, mask off the rest
  _wire->beginTransmission(i2c_addr);
  _wire->write((uint8_t)ADC121C021_REG_POINTER_CONVERSION);
  _wire->endTransmission();
  _wire->requestFrom(i2c_addr, 2);
  temp = (uint16_t)((_wire->read() << 8) | _wire->read());
  if(temp&0x8000) 
  {
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)ADC121C021_REG_POINTER_ALERT_STATUS);
    _wire->endTransmission();
    _wire->requestFrom(i2c_addr,1); 
    alertStatus = (uint8_t)_wire->read();  

     // Under Range Alert Flag
    if (alertStatus & 0x01)
      {
       adc_alertLow = 1; 
      }        
    else
      {
        adc_alertLow = 0;
      }
    
    // Over Range Alert Flag
    if (alertStatus & 0x02)
      {
        adc_alertHigh = 1;
      }
    else
      {
        adc_alertHigh = 0;
      }
  }     
  return alertStatus;
}
/**
 * @brief Get the current status of the Under Range Alert Flag 
 * readAlertStatus() must be called to update this value.
 * @param none
 * @return Under Range Alert Flag status.
 */
uint8_t ADC121C021::readAlertLowStatus()
{
    return adc_alertLow;
}
/**
 * @brief Get the current status of the Over Range Alert Flag 
 * readAlertStatus() must be called to update this value.
 * @param none
 * @return Over Range Alert Flag
 */
uint8_t ADC121C021::readAlertHighStatus()
{
    return adc_alertHigh;
}
/**
 * @brief Clears the alert low and alert high flags. 
 * @param none
 * @return void
 */
void ADC121C021::clearAlertStatus()
{
    // Zero out both the low and high alert flags
    uint8_t alertStatus = 0;
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)ADC121C021_REG_POINTER_ALERT_STATUS);
    _wire->write((uint8_t)alertStatus);
    _wire->endTransmission();    
    adc_alertHigh = 0;
    adc_alertLow = 0;
}
/**
 * @brief Sets the Configuration Register of Cycle Time bits value,when 
 * the value set to zeros, the automatic conversion mode is disabled. 
 * This is the case at power-up.
 * @param cycletime the conversion intervals of convert time can be 0-7
 * value       Conversion Interval            Typical fconvert (ksps)
 *  0           Automatic Mode Disabled         0
 *  1           Tconvert x 32                   27
 *  2           Tconvert x 64                   13.5
 *  3           Tconvert x 128                  6.7
 *  4           Tconvert x 256                  3.4
 *  5           Tconvert x 512                  1.7
 *  6           Tconvert x 1024                 0.9
 *  7           Tconvert x 2048                 0.4
 * @return
 */
void ADC121C021::configCycleTime(uint8_t cycletime)
{
  uint8_t temp=0;
  temp = readConfigRegister()&0x1f; //&00011111   Retain the original data
	temp = temp|cycletime;
  _wire->beginTransmission(i2c_addr);
	_wire->write((uint8_t)ADC121C021_REG_POINTER_CONFIG);
	_wire->write(cycletime);
	_wire->endTransmission(); 
}
/**
 * @brief Read the Configuration Register of Cycle Time bits value
 * @param none
 * @return uint8_t the Cycle Time bits value from the Configuration Register
 */
uint8_t ADC121C021::readCycleTime()
{
  uint8_t temp;
  temp = readConfigRegister()&CONFIG_CYCLE_TIME_MASK;
  return temp;
}
/**
 * @brief Read the Configuration Register data, the register address is 0x02
 * @param none
 * @return uint8_t Configuration Register data
 */
uint8_t ADC121C021::readConfigRegister()
{
  uint8_t temp;
  _wire->beginTransmission(i2c_addr);
  _wire->write((uint8_t)ADC121C021_REG_POINTER_CONFIG);
  _wire->endTransmission();  
  _wire->requestFrom(i2c_addr, 1);    // request 6 bytes from secondary device #2 
  temp = _wire->read(); // receive a byte as character
  return temp;
}
/**************************************************************************/
/*
        Sets the Alert Hold Status ,if set Enable to enable alert hold,
        else disable alert hold.
*/
/**************************************************************************/
/**
 * @brief Sets the Alert Hold bit enable or disable
 * @param alertHold Enable:enable alert hold, Disable:disable alert hold.
 * @return void
 */
void ADC121C021::configAlertHold(configType alertHold)
{
  uint8_t temp;
  if(alertHold == Enable)
  {
    temp = readConfigRegister();
    temp = temp|ADC121C021_REG_CONFIG_ALERT_FLAG_NOCLEAR;
    _wire->beginTransmission(i2c_addr);
    _wire->write(ADC121C021_REG_POINTER_CONFIG);
    _wire->write(temp);
    _wire->endTransmission();      
  }
  else
  {
    temp = readConfigRegister();
    temp = temp&ADC121C021_REG_CONFIG_ALERT_HOLD_CLEAR;   //0xef=0b:1110 1111
    _wire->beginTransmission(i2c_addr);
    _wire->write(ADC121C021_REG_POINTER_CONFIG);
    _wire->write(temp);
    _wire->endTransmission();  
  }  
}
/**
 * @brief get the Alert Hold bit status from the Configuration Register
 * @param none
 * @return uint8_t the value of the Alert Hold bit
 */
uint8_t ADC121C021::readAlertHold()
{
  uint8_t temp;
  temp = readConfigRegister()&ADC121C021_REG_CONFIG_ALERT_HOLD_MASK;
  return temp;    
}
/**
 * @brief Config the Alert Flag Enable bit,This controls the alert status 
 * bit [D15] in the Conversion Result register.
 * @param alertflag Enable or Disable.
 * @return void
 */
void ADC121C021::configAlertFlag(configType alertflag)
{
  uint8_t temp;
  /*  
   * 1: Alerts will not self-clear and are only cleared when a one is written to 
   * the alert high flag or the alert low flag in the Alert Status register.
   */
  if(alertflag == Enable)          
  {                                 
    temp = readConfigRegister();
    temp = temp|ADC121C021_REG_CONFIG_ALERT_FLAG_EN;
    _wire->beginTransmission(i2c_addr);
    _wire->write(ADC121C021_REG_POINTER_CONFIG);
    _wire->write(temp);
    _wire->endTransmission();      
  }
  else 
  {
    /*
     * 0: Alerts will self-clear when the measured voltage moves within 
     * the limits by more than the hysteresis register value.
     */
    temp = readConfigRegister();
    temp = temp&ADC121C021_REG_CONFIG_ALERT_FLAG_DIS;  //0xf7=0b:1111 0111
    _wire->beginTransmission(i2c_addr);
    _wire->write(ADC121C021_REG_POINTER_CONFIG);
    _wire->write(temp);
    _wire->endTransmission();  
  }  
}
/**
 * @brief get the bit value of Alert Flag Enable Status from the Configuration Register
 * @param none
 * @return uint8_t the bit value of Alert Flag Enable
 */
uint8_t ADC121C021::readAlertFlag()
{
  uint8_t temp;
  temp = readConfigRegister()&ADC121C021_REG_CONFIG_ALERT_FLAG_MASK;
  return temp;
}
/**
 * @brief Config the alert pin output enable or disable when ALERT come.
 * @param alertpin Enable or Disable
 * @return void
 */
void ADC121C021::configAlertPin(configType alertpin)
{
  uint8_t temp;
  if(alertpin == Enable)  //1: Enables the ALERT output pin.
  {
    temp = readConfigRegister();
    temp = temp|ADC121C021_REG_CONFIG_ALERT_PIN_EN;
    _wire->beginTransmission(i2c_addr);
    _wire->write(ADC121C021_REG_POINTER_CONFIG);
    _wire->write(temp);
    _wire->endTransmission();      
  }
  else  //0: Disables the ALERT output pin. The ALERT output will be high impedance when the pin is disabled.
  {
    temp = readConfigRegister();
    temp = temp&ADC121C021_REG_CONFIG_ALERT_PIN_DIS;  //0xfb=0b:1111 1011
    _wire->beginTransmission(i2c_addr);
    _wire->write(ADC121C021_REG_POINTER_CONFIG);
    _wire->write(temp);
    _wire->endTransmission();  
  }  
}
/**
 * @brief Get the Alert Pin Enable Status from the Configuration Register
 * @param none
 * @return uint8_t the Alert Pin Enable Status register value
 */
uint8_t ADC121C021::readAlertPin()
{
  uint8_t temp;
  temp = readConfigRegister()&ADC121C021_REG_CONFIG_ALERT_PIN_MASK;
  return temp;
}
/**
 * @brief Config the alert pin output level polarity when ALERT come.
 * @param polarity High or Low
 * @return void
 */
void ADC121C021::configPolarity(polarityType polarity)
{
  uint8_t temp;
  if(polarity == 0)   //Sets the ALERT pin to active low
  {
    temp = readConfigRegister();
    temp = temp&ADC121C021_REG_CONFIG_POLARITY_LOW;   //0xfe=0b:1111 1110
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)ADC121C021_REG_POINTER_CONFIG);
    _wire->write(temp);
    _wire->endTransmission();  
  } 
  else  //Sets the ALERT pin to active high.
  {
    temp = readConfigRegister();
    temp = temp|ADC121C021_REG_CONFIG_POLARITY_HIGH;
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)ADC121C021_REG_POINTER_CONFIG);
    _wire->write(temp);
    _wire->endTransmission();      
  } 
}
/**
 * @brief get the alert pin output level polarity configuration.
 * @param none
 * @return uint8_t the polarity configuration
 */
uint8_t ADC121C021::readPolarity()
{
  uint8_t temp;
  temp = readConfigRegister()&ADC121C021_REG_CONFIG_POLARITY_MASK;
  return temp;
}
/**
 * @brief Sets the lower limit threshold value
 * @param uint16_t threshold value
 * @return void
 */
void ADC121C021::setAlertLowThreshold(uint16_t threshold)
{
    // mask off the invalid bits in case they were set
  threshold &= 0x0FFF;
  _wire->beginTransmission(i2c_addr);
  _wire->write((uint8_t)ADC121C021_REG_POINTER_LOW_LIMIT);
  _wire->write((uint8_t)(threshold>>8));
  _wire->write((uint8_t)(threshold & 0xFF));
  _wire->endTransmission();     
}
/**
 * @brief get the value of the lower limit threshold value configuration
 * @param none
 * @return uint16_t the lower limit threshold value
 */
uint16_t ADC121C021::readAlertLowThreshold()
{
    // mask off the invalid bits in case they were set
    return readRegister16(ADC121C021_REG_POINTER_LOW_LIMIT)&0x0FFF;
}
/**
 * @brief Sets the upper limit threshold value
 * @param uint16_t threshold value
 * @return void
 */
void ADC121C021::setAlertHighThreshold(uint16_t threshold)
{
    // mask off the invalid bits in case they were set
    threshold &= 0x0FFF;
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)ADC121C021_REG_POINTER_HIGH_LIMIT);
    _wire->write((uint8_t)(threshold>>8));
    _wire->write((uint8_t)(threshold & 0xFF));
    _wire->endTransmission(); 
}
/**
 * @brief get the value of the upper limit threshold value configuration
 * @param none
 * @return uint16_t the upper limit threshold value
 */
uint16_t ADC121C021::readAlertHighThreshold()
{
    // mask off the invalid bits in case they were set
    return readRegister16(ADC121C021_REG_POINTER_HIGH_LIMIT)&0x0FFF;
}
/**
 * @brief config the hysteresis value
 * @param uint16_t hysteresis value
 * @return void
 */
void ADC121C021::setHysteresis(uint16_t hysteresis)
{
    // mask off the invalid bits in case they were set
    hysteresis &= 0x0FFF;    
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)ADC121C021_REG_POINTER_HYSTERESIS);
    _wire->write((uint8_t)(hysteresis>>8));
    _wire->write((uint8_t)(hysteresis & 0xFF));
    _wire->endTransmission(); 
}
/**
 * @brief get the hysteresis value configuration
 * @param none
 * @return uint16_t the hysteresis value configuration
 */
uint16_t ADC121C021::readHysteresis()
{
    // mask off the invalid bits in case they were set
    return readRegister16(ADC121C021_REG_POINTER_HYSTERESIS)&0x0FFF;
}
/**
 * @brief Gets the Lowest Conversion result recorded from the Lowest Conversion Register
 * @param none
 * @return uint16_t the lowest conversion result value
 */
uint16_t ADC121C021::readLowestConversion()
{
    return readRegister16(ADC121C021_REG_POINTER_LOWCONV);
}
/**
 * @brief Clears the Lowest Conversion register value 
 * @param none
 * @return void
 */
void ADC121C021::clearLowestConversion()
{    
  uint16_t lowClear = 0x0FFF; 
  _wire->beginTransmission(i2c_addr);
  _wire->write((uint8_t)ADC121C021_REG_POINTER_LOWCONV);
  _wire->write((uint8_t)(lowClear>>8));
  _wire->write((uint8_t)(lowClear & 0xFF));
  _wire->endTransmission(); 
}
/**
 * @brief Gets the Highest Conversion result recorded from the Highest Conversion Register
 * @param none
 * @return uint16_t the highest conversion result value
 */
uint16_t ADC121C021::readHighestConversion()
{
    return readRegister16(ADC121C021_REG_POINTER_HIGHCONV);
}
/**
 * @brief Clears the Highest Conversion register value 
 * @param none
 * @return void
 */
void ADC121C021::clearHighestConversion()
{
  uint16_t highClear = 0x0000; 
  _wire->beginTransmission(i2c_addr);
  _wire->write((uint8_t)ADC121C021_REG_POINTER_HIGHCONV);
  _wire->write((uint8_t)(highClear>>8));
  _wire->write((uint8_t)(highClear & 0xFF));
  _wire->endTransmission(); 
}
/**
 * @brief Read the ADC conversation value from the Conversion Register
 * @param none
 * @return uint16_t ADC conversation value
 */
uint16_t ADC121C021::getAdcValue()
{
  uint16_t result;
  uint8_t alertStatus;
  // Read the conversion result
  // 12-bit unsigned result for the ADC121C021
  result = readRegister16(ADC121C021_REG_POINTER_CONVERSION);
  if(result&0x8000) 
  {
    _wire->beginTransmission(i2c_addr);
    _wire->write((uint8_t)ADC121C021_REG_POINTER_ALERT_STATUS);
    _wire->endTransmission();
    _wire->requestFrom(i2c_addr,1); 
    alertStatus = (uint8_t)_wire->read();  

    // Under Range Alert Flag
    if (alertStatus & 0x01)
      {
       adc_alertLow = 1; 
      }        
    else
      {
        adc_alertLow = 0;
      }
    
    // Over Range Alert Flag
    if (alertStatus & 0x02)
      {
        adc_alertHigh = 1;
      }
    else
      {
        adc_alertHigh = 0;
      }

     adc_alert = true;   //alert has occur
  }
  else
  {
    adc_alert = false;
  }   

  result = result&0xfff;    
  return  result;
}
//-------------------------MQx sensor ---------------------------------------------------------------
/*
 *MQX sensor can reference https://components101.com/sensors/mq2-gas-sensor
 */
/**
 * @brief Gets the voltage of sensor output 
 * @param none
 * @return float voltage V
 */
float ADC121C021::getSensorVoltage() 
{
  float voltage;
  voltage = getAdcValue();
  #if(DEBUG_LEVEL >1)
  {
   LOG.printf("DEBUG ADC VALUE: %.0f\r\n",voltage);
  }
  #endif  
  voltage = voltage*_VOLT_RESOLUTION/4095.0;  //calculate sensor voltage，ADC 12bit(0-4095),Vref = 5.0V
  #if(DEBUG_LEVEL >1)
  {
   LOG.printf("DEBUG VALTAGE VALUE: %f\r\n",voltage);
  }
  #endif 
  return voltage;
}
/**
 * @brief set the resolution voltage 
 * @param float value， if use 5V supply for MQx sensor to set value 5.0
 * if use 3.3V supply for MQx sensor to set value 3.3
 * @return void
 */
void ADC121C021::setVoltageResolution(float value) 
{
  _VOLT_RESOLUTION = value;
}
/**
 * @brief Gets the resolution voltage configuration
 * @param none
 * @return float the value of the resolution voltage configuration
 */
float ADC121C021::getVoltageResolution() 
{
  return _VOLT_RESOLUTION;
}
/**
 * @brief set the slope of the line 
 * @param float K
 * @return void
 */
void ADC121C021::setA(float K) 
{
  _constantA = K;
}
/**
 * @brief get the slope of the line parameter
 * @param none
 * @return float the slope of the line configuration
 */
float ADC121C021::getA() 
{
  return _constantA;
}
/**
 * @brief config Y intercept parameter
 * @param float b Y intercept
 * @return none
 */
void ADC121C021::setB(float b) 
{
  _constantB = b;
}
/**
 * @brief get the Y intercept parameter configuration
 * @param none
 * @return float Y intercept parameter
 */
float ADC121C021::getB() 
{
  return _constantB;
}
/**
 * @brief set the resistance of the sensor at a known concentration 
 * without the presence of other gases, or in fresh air.
 * @param float R0 the resistance of the sensor at fresh air.
 * @return void
 */
void ADC121C021::setR0(float R0) 
{
  _R0 = R0;
}
/**
 * @brief Gets the R0 parameter
 * @param none
 * @return float R0
 */
float ADC121C021::getR0()
{
  return _R0;
}
/**
 * @brief config RL value
 * @param float RL the load resistance value
 * @return none
 */
void ADC121C021::setRL(float RL) 
{
  _RL = RL;
}
/**
 * @brief Gets the RL parameter
 * @param none
 * @return float RL the load resistance value
 */
float ADC121C021::getRL() 
{
  return _RL;
}
/**
 * @brief set the regression method
 * @param uint8_t method
 * set the regression method if=1 use Exponential else Linear
        Exponential:  PPM =  A*(RS/R0)^B
        Linear:       PPM =  pow(10, (log10(RS/R0)-B)/A) 
 * @return
 */
void ADC121C021::setRegressionMethod(uint8_t method)
{
  _regressionMethod = method;
}
/**
 * @brief Gets the regression method
 * @param none
 * @return uint8_t the regression method
 */
uint8_t ADC121C021::getRegressionMethod()
{
  return _regressionMethod;
}
/**
 * @brief set the ratio of RS/R0 in clean air
 * @param float ratioInCleanAir = RS/R0
 * @return float R0 MQx sensor resistance in the clean air
 */
float ADC121C021::calibrateR0(float ratioInCleanAir) 
{
  /*
  reference linking https://jayconsystems.com/blog/understanding-a-gas-sensor
  V = I x R 
  VRL = [VC / (RS + RL)] x RL 
  VRL = (VC x RL) / (RS + RL) 
  Así que ahora resolvemos para RS: 
  VRL x (RS + RL) = VC x RL
  (VRL x RS) + (VRL x RL) = VC x RL 
  (VRL x RS) = (VC x RL) - (VRL x RL)
  RS = [(VC x RL) - (VRL x RL)] / VRL
  RS = [(VC x RL) / VRL] - RL
  */
  float RS_air;   //Define variable for sensor resistance
  float R0;      //Define variable for R0
  float sensorVoltage;
  sensorVoltage = getSensorVoltage(); //calculate VRL voltage
  RS_air = ((_VOLT_RESOLUTION*_RL)/sensorVoltage)-_RL; //Calculate RS in fresh air
  if(RS_air < 0)  RS_air = 0; //No negative values accepted.
  R0 = RS_air/ratioInCleanAir; //Calculate R0 
  if(R0 < 0)  R0 = 0; //No negative values accepted.
  return R0;
}
/**
 * @brief read the MQx sensor data to PPM
 * @param none
 * @return float MQx sensor data PPM
 */
float ADC121C021::readSensor()
{
  float RS_air;
  float sensorVoltage;
  float ratio;
  float ppm;
  sensorVoltage = getSensorVoltage(); //calculate VRL voltage
  RS_air = ((_VOLT_RESOLUTION*_RL)/sensorVoltage)-_RL; //Get value of RS in a gas
  #if(DEBUG_LEVEL >1)
  {
   LOG.printf("DEBUG RS_air VALUE: %3.3f \r\n",RS_air);
  }
  #endif
  if(RS_air < 0)  RS_air = 0; //No negative values accepted.
  ratio = RS_air / _R0;   // Get ratio RS_gas/RS_air
  if(ratio <= 0)  ratio = 0; //No negative values accepted or upper datasheet recomendation.  
  
  if(_regressionMethod == 1) 
  {
    ppm= _constantA*pow(ratio, _constantB); // 
  }
  else 
  {    
    double ppm_log = (log10(ratio)-_constantB)/_constantA; //Get ppm value in linear scale according to the the ratio value  
    ppm = pow(10, ppm_log); //Convert ppm value to log scale  
  }
  if(ppm < 0)  ppm = 0; //No negative values accepted or upper datasheet recomendation.
  //if(ppm > 10000) ppm = 99999999; //No negative values accepted or upper datasheet recomendation.
  return ppm;
}
