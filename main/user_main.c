#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "dht22.h"

/*
 *   Program: dht22 
 *   Date   : 9/22/2018
 *   Author : hcurajr@hotmail.com
 *
 *   Description
 *   This program will use dht22 temperature & humidity sensor to periodically
 *   print the temperature & humidity. The following pins will be used:
 *       gpio4: toggle green LED
 *       gpio5: communicate with dht22 DATA pin
 */

#define MAX_QUEUE_SIZE    10
#define DHT_READ_INTERVAL 15000                     // Poll sensor every ~15 seconds
#define DHT_SENSOR_NAME   "Daniel's Greenhouse    fffdsa jkl; abc"   

 typedef struct _dht_queue_entry_t {
    dht_data_t dhtData;
    dht_result_t result;
}dht_queue_entry_t; 

static const char *TAG = DHT_SENSOR_NAME;
static bool gQUIT = false;
static QueueHandle_t gQueueHandle;

/*
 * WriteSensorTask function reads DHT sensor data from Queue and publishes data to cloud.
 */
void 
WriteSensorTask(void *input) {
    dht_queue_entry_t dhtQEntry;

    while (!gQUIT) {
        if (xQueueReceive(gQueueHandle, &dhtQEntry, pdMS_TO_TICKS(DHT_READ_INTERVAL)) == pdTRUE) {
            if (dhtQEntry.result == DHT_OK) {
                ESP_LOGI(TAG, "Temperature %d.%d F (%d.%d C), Relative Humidity %d.%d%%", 
                            dhtQEntry.dhtData.faTempWhole, 
                            dhtQEntry.dhtData.faTempFraction, 
                            dhtQEntry.dhtData.csTempWhole, 
                            dhtQEntry.dhtData.csTempFraction,
                            dhtQEntry.dhtData.rhWhole, 
                            dhtQEntry.dhtData.rhFraction);
            }
        } else {
            ESP_LOGE(TAG, "WriteSensorTask: Failed to read from Queue.");
        }
    } 
    
    ESP_LOGE(TAG, "WriteSensorTask: QUIT signal is TRUE. Task exiting loop. (%d)", __LINE__);
    vTaskDelete(NULL);
}

/* 
 * ReadSensorTask reads Temperature and Relative Humidity data from 
 * DHT22 Sensor and writes it to queue for processing.
 */
void 
ReadSensorTask(void *input) {
    dht_t *pDht = NULL;
    dht_queue_entry_t dhtQEntry;
 
    if ((dhtQEntry.result = dhtInitialize(GPIO_NUM_5, DHT_SENSOR_NAME, &pDht)) != DHT_OK) {
        ESP_LOGE(TAG, "ReadSensorTask: Failed to initialize DHT22 Sensor! Exiting (Error=%d).", dhtQEntry.result);
        gQUIT = true;
        return;
    }

    // wait a full 2sec cycle before reading from sensor for first time
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (!gQUIT) {
        dhtQEntry.result = dhtRead(pDht, &dhtQEntry.dhtData);
        if (xQueueSendToBack(gQueueHandle, &dhtQEntry, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGE(TAG, "ReadSensorTask: Failed to add entry to queue.");      
        }

        vTaskDelay(pdMS_TO_TICKS(DHT_READ_INTERVAL));
    }

    ESP_LOGE(TAG, "ReadSensorTask: QUIT signal is TRUE. Task exitng loop. (%d)", __LINE__);
    vTaskDelete(NULL);
}

void 
app_main(void)
{
    gQueueHandle = xQueueCreate(MAX_QUEUE_SIZE, sizeof(dht_queue_entry_t));
    if (gQueueHandle == NULL) {
        ESP_LOGE(TAG, "Insufficient heap-memory to create Queue Handle. Program Exiting. (%d)", __LINE__);
        return;
    }

    if (xTaskCreate(&ReadSensorTask,  "ReadSensorTask",  configMINIMAL_STACK_SIZE<<2, NULL, 5, NULL) == pdFAIL) {
        // free queue
        gQUIT = true;
        vQueueDelete(gQueueHandle);
        ESP_LOGE(TAG, "Failed to create ReadSensorTask. Program exiting. (%d)", __LINE__);
        return;
    }

    if (xTaskCreate(&WriteSensorTask, "WriteSensorTask", configMINIMAL_STACK_SIZE<<2, NULL, 4, NULL) == pdFAIL) {
        gQUIT = true;
        vTaskDelay(pdMS_TO_TICKS(2000));
        // free queue
        vQueueDelete(gQueueHandle);
        ESP_LOGE(TAG, "Failed to create WriteSensorTask. Program exiting. (%d)", __LINE__);
    }
}

