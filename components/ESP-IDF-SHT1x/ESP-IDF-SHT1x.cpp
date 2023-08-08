
#include <stdio.h>
#include <string.h>
#include "SHT1x-ESP.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <rom/ets_sys.h> //for having the delay in the microseconds. 
#include <cmath>
#include <ctgmath>

SHT1x::SHT1x(gpio_num_t dataPin, gpio_num_t clockPin, Voltage voltage) : _dataPin(dataPin), _clockPin(clockPin), _voltage(voltage), _tempResolution(TemperatureMeasurementResolution::Temperature_14bit), _humidityResolution(HumidityMeasurementResolution::Humidity_12bit)
{
}

float SHT1x::readTemperatureC() const
{
	// Conversion coefficients from datasheet
	const double D1 = getD1ForC(_voltage);
	const double D2 = getD2ForC(_tempResolution);

	// Fetch raw value
	uint16_t rawData = readRawData(ShtCommand::MeasureTemperature, _dataPin, _clockPin);
	if (rawData == UINT16_MAX)
		return NAN;
	// Convert raw value to degrees Celsius
	float temperature = D1 + D2 * rawData;

	return temperature;
}

float SHT1x::readTemperatureF() const
{
	// Conversion coefficients from datasheet
	const double D1 = getD1ForF(_voltage);
	const double D2 = getD2ForF(_tempResolution);

	// Fetch raw value
	uint16_t rawData = readRawData(ShtCommand::MeasureTemperature, _dataPin, _clockPin);
	if (rawData == UINT16_MAX)
		return NAN;

	// Convert raw value to degrees Fahrenheit
	float temperature = D1 + D2 * rawData;

	return temperature;
}

float SHT1x::readHumidity() const
{
	// Conversion coefficients from datasheet
	const double C1 = getC1(_humidityResolution);
	const double C2 = getC2(_humidityResolution);
	const double C3 = getC3(_humidityResolution);
	const double T1 = getT1(_humidityResolution);
	const double T2 = getT2(_humidityResolution);

	// Fetch the value from the sensor
	uint16_t rawData = readRawData(ShtCommand::MeasureRelativeHumidity, _dataPin, _clockPin);
	if (rawData == UINT16_MAX)
		return NAN;
		
	// Apply linear conversion to raw value
	double linearHumidity = C1 + C2 * rawData + C3 * rawData * rawData;

	// Get current temperature for humidity correction
	float temperature = readTemperatureC();

	// Correct humidity value for current temperature
	float correctedHumidity = (temperature - 25.0) * (T1 + T2 * rawData) + linearHumidity;

	return correctedHumidity;
}


uint16_t SHT1x::readRawData(ShtCommand command, gpio_num_t dataPin, gpio_num_t clockPin) const
{
#ifdef ESP32
	// ESP32 is a multi core / multi processing chip
	// It is necessary to disable task switches during the readings
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);
#endif

	if (!sendCommandSHT(command, dataPin, clockPin))
	{
#ifdef ESP32
	    portEXIT_CRITICAL(&mux);
#endif 
		return UINT16_MAX;
	}
	if (!waitForResultSHT(dataPin))
	{
#ifdef ESP32
	    portEXIT_CRITICAL(&mux);
#endif 
		return UINT16_MAX;
	}
	uint32_t result = getData16SHT(dataPin, clockPin);
	skipCrcSHT(dataPin, clockPin);

#ifdef ESP32
	portEXIT_CRITICAL(&mux);
#endif 

	return result;
}




bool SHT1x::sendCommandSHT(ShtCommand command, gpio_num_t dataPin, gpio_num_t clockPin) const
{
	// Transmission Start
    gpio_set_direction(clockPin,  GPIO_MODE_OUTPUT);
	controlDataPin(dataPin, 1);
    ets_delay_us(1);			//Stalls execution for #uS
	gpio_set_level(clockPin, 1);
    controlDataPin(dataPin, 0);
    ets_delay_us(1);			
	gpio_set_level(clockPin, 0);
	ets_delay_us(1);			
	gpio_set_level(clockPin, 1);
	controlDataPin(dataPin, 1);
	ets_delay_us(1);			
	gpio_set_level(clockPin, 0);
	ets_delay_us(1);			
	// The command (3 msb are address and must be 000, and last 5 bits are command)
	for (uint8_t i = 0; i < 8; i++)
	{
		controlDataPin(dataPin, !!(static_cast<uint8_t>(command) & (1 << (7 - i))));
		gpio_set_level(clockPin, 1);
		ets_delay_us(1);			
		gpio_set_level(clockPin, 0);
		ets_delay_us(1);			
	}

	// Verify we get the correct ack
	gpio_set_direction(dataPin, GPIO_MODE_INPUT);
	gpio_set_level(clockPin, 1);
	ets_delay_us(1);			
	int ack = gpio_get_level(dataPin);
	if (ack != 0)
	{
		// Serial.println("Ack Error 0");
		return false;
	}
	gpio_set_level(clockPin, 0);
	ets_delay_us(10);			
	ack = gpio_get_level(dataPin);
	if (ack != 1)
	{
		// Serial.println("Ack Error 1");
		return false;
	}

	return true;
}





bool SHT1x::waitForResultSHT(gpio_num_t dataPin) const
{
	gpio_set_direction(dataPin, GPIO_MODE_INPUT);

	// wait for at most 1000ms for the result
	// from the datasheet,
	// "This takes a maximum of 20/80/320 ms for a 8/12/14bit measurement.
	// The time varies with the speed of the internal oscillator and can be lower by up to 30%."
	int ack = 1;
	time_t startWait = esp_timer_get_time();
	

	while (ack == 1)
	{
		ets_delay_us(10000);
		ack = gpio_get_level(dataPin);
		if (ack == 0)
		{
			break;
		}
		if (esp_timer_get_time() - startWait > 1000 * 1000)
		{
			break;
		}
	}

	if (ack == 1)
	{
		// Serial.println("Ack Error 2"); // Can't do serial stuff here, need another way of reporting errors
		return false;
	}

	return true;
}


uint16_t SHT1x::getData16SHT(gpio_num_t dataPin, gpio_num_t clockPin) const
{
	uint8_t rawData[2];
	memset(rawData, 0, 2);

	// Get the most significant bits
	gpio_set_direction(dataPin, GPIO_MODE_INPUT);

	rawData[0] = 0;
	for (uint8_t i = 0; i < 8; ++i)
	{
		gpio_set_level(clockPin, 1);
		ets_delay_us(1);
		rawData[0] |= gpio_get_level(dataPin) << (7 - i);
		ets_delay_us(1);
		gpio_set_level(clockPin, 0);
		ets_delay_us(1);
	}
	// Send the required ack
	gpio_set_direction(dataPin, GPIO_MODE_OUTPUT);
	controlDataPin(dataPin, 0);

	gpio_set_direction(clockPin, GPIO_MODE_OUTPUT);
	gpio_set_level(clockPin, 1);
	ets_delay_us(1);
	gpio_set_level(clockPin, 0);
	ets_delay_us(1);

	// Get the least significant bits
	gpio_set_direction(dataPin, GPIO_MODE_INPUT);

	rawData[1] = 0;
	for (uint8_t i = 0; i < 8; ++i)
	{
		gpio_set_level(clockPin, 1);
		ets_delay_us(1);
		rawData[1] |= gpio_get_level(dataPin) << (7 - i);
		ets_delay_us(1);
		gpio_set_level(clockPin, 0);
		ets_delay_us(1);
	}

	// extract value from the big-endian raw data
	uint16_t result = (rawData[0] << 8) | (rawData[1] << 0);

	return result;
}

void SHT1x::skipCrcSHT(gpio_num_t dataPin, gpio_num_t clockPin) const
{
	// Skip acknowledge to end trans (no CRC)
	//pinMode(dataPin, OUTPUT);
	gpio_set_direction(clockPin,GPIO_MODE_OUTPUT);

	controlDataPin (dataPin, 1);
	gpio_set_level(clockPin, 1);
	gpio_set_level(clockPin, 0);
}







double SHT1x::getC1(SHT1x::HumidityMeasurementResolution resolution) const
{
	const double VALUES[] = {
		-2.0468 // 12 bit
		,
		-2.0468 // 8 bit
	};

	return VALUES[static_cast<uint8_t>(resolution)];
}

double SHT1x::getC2(SHT1x::HumidityMeasurementResolution resolution) const
{
	const double VALUES[] = {
		0.0367 // 12 bit
		,
		0.5872 // 8 bit
	};

	return VALUES[static_cast<uint8_t>(resolution)];
}

double SHT1x::getC3(SHT1x::HumidityMeasurementResolution resolution) const
{
	const double VALUES[] = {
		-1.5955e-6 // 12 bit
		,
		-4.0845e-4 // 8 bit
	};

	return VALUES[static_cast<uint8_t>(resolution)];
}

double SHT1x::getT1(SHT1x::HumidityMeasurementResolution resolution) const
{
	const double VALUES[] = {
		0.01 // 12 bit
		,
		0.01 // 8 bit
	};

	return VALUES[static_cast<uint8_t>(resolution)];
}

double SHT1x::getT2(SHT1x::HumidityMeasurementResolution resolution) const
{
	const double VALUES[] = {
		0.00008 // 12 bit
		,
		0.00128 // 8 bit
	};

	return VALUES[static_cast<uint8_t>(resolution)];
}

double SHT1x::getD1ForC(SHT1x::Voltage voltage) const
{
	const double VALUES[] = {
		-40.1 // 5.0v
		,
		-39.8 // 4.0v
		,
		-39.7 // 3.5v
		,
		-39.66 // 3.3v, linear interpolation
		,
		-39.6 // 3.0v
		,
		-39.4 // 2.5v
	};

	return VALUES[static_cast<uint8_t>(voltage)];
}

double SHT1x::getD1ForF(SHT1x::Voltage voltage) const
{
	const double VALUES[] = {
		-40.2 // 5.0v
		,
		-39.6 // 4.0v
		,
		-39.5 // 3.5v
		,
		-39.42 // 3.3v, linear interpolation
		,
		-39.3 // 3.0v
		,
		-38.9 // 2.5v
	};

	return VALUES[static_cast<uint8_t>(voltage)];
}

double SHT1x::getD2ForC(SHT1x::TemperatureMeasurementResolution resolution) const
{
	const double VALUES[] = {
		0.01 // 14 bit
		,
		0.04 // 12 bit
	};

	return VALUES[static_cast<uint8_t>(resolution)];
}

double SHT1x::getD2ForF(SHT1x::TemperatureMeasurementResolution resolution) const
{
	const double VALUES[] = {
		0.018 // 14 bit
		,
		0.072 // 12 bit
	};

	return VALUES[static_cast<uint8_t>(resolution)];
}



void SHT1x::controlDataPin(gpio_num_t dataPin, uint8_t val)const {
  if (val) {
    gpio_set_direction(dataPin, GPIO_MODE_INPUT);
  } else {
    gpio_set_level(dataPin, 0);
    gpio_set_direction(dataPin, GPIO_MODE_OUTPUT);
  }
}

