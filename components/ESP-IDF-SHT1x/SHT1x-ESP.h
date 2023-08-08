
#include <stdint.h>
#include "driver/gpio.h"

#ifndef SHT1x_h
#define SHT1x_h

class SHT1x
{
  public:
    enum class Voltage : int8_t
    {
      DC_5_0v = 0
      , DC_4_0v
      , DC_3_5v
      , DC_3_3v
      , DC_3_0v
      , DC_2_5v
    };

    enum class TemperatureMeasurementResolution : int8_t
    {
      Temperature_14bit = 0
      , Temperature_12bit
    };

    enum class HumidityMeasurementResolution : int8_t
    {
      Humidity_12bit = 0
      , Humidity_8bit
    };

    enum class ShtCommand : int8_t
    {
      MeasureTemperature        = 0b00000011
      , MeasureRelativeHumidity = 0b00000101
      , ReadStatusRegister      = 0b00000111
      , WriteStatusRegister     = 0b00000110
      , SoftReset               = 0b00011110
    };

    SHT1x(gpio_num_t dataPin, gpio_num_t clockPin, Voltage voltage = Voltage::DC_5_0v);

    float readHumidity() const;
    float readTemperatureC() const;
    float readTemperatureF() const;

  private:
    uint16_t readRawData(ShtCommand command, gpio_num_t dataPin, gpio_num_t clockPin) const;

    bool sendCommandSHT(ShtCommand command, gpio_num_t dataPin, gpio_num_t clockPin) const;
    bool waitForResultSHT(gpio_num_t dataPin) const;
    uint16_t getData16SHT(gpio_num_t dataPin, gpio_num_t clockPin) const;
    void skipCrcSHT(gpio_num_t dataPin, gpio_num_t clockPin) const;

    double getC1(HumidityMeasurementResolution resolution) const;
    double getC2(HumidityMeasurementResolution resolution) const;
    double getC3(HumidityMeasurementResolution resolution) const;

    double getT1(HumidityMeasurementResolution resolution) const;
    double getT2(HumidityMeasurementResolution resolution) const;

    double getD1ForC(Voltage voltage) const;
    double getD1ForF(Voltage voltage) const;

    double getD2ForC(TemperatureMeasurementResolution resolution) const;
    double getD2ForF(TemperatureMeasurementResolution resolution) const;

    void   controlDataPin(gpio_num_t dataPin, uint8_t val) const;
    
    const gpio_num_t _dataPin;
    const gpio_num_t _clockPin;

    const Voltage _voltage;
    const TemperatureMeasurementResolution _tempResolution;
    const HumidityMeasurementResolution _humidityResolution;
};

#endif
