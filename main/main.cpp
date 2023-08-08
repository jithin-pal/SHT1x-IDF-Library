#include <stdio.h>
#include "SHT1x-ESP.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define dataPin  27
#define clockPin 26

SHT1x sht1x((gpio_num_t)dataPin, (gpio_num_t)clockPin, SHT1x::Voltage::DC_3_3v);

void task_read(void *args)
{
    while(1) {
        float temp_c;
        float temp_f;
        float humidity;

        // Read values from the sensor
        temp_c = sht1x.readTemperatureC();
        temp_f = sht1x.readTemperatureF();
        humidity = sht1x.readHumidity();

        printf("Temp C: %f, Temp :%f  Humidity: %f\n", temp_c, temp_f, humidity);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Enabling C++ compile
extern "C" { void app_main(); }

void app_main(void)
{
    printf("starting the sensor..");
    xTaskCreate(task_read, "read_data", 4096, NULL, 0, NULL);
}