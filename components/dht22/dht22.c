#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp8266/rom_functions.h"
#include "task.h"

#include "esp_log.h"
#include "esp_system.h"

#include "dht22.h"

#define DHT_LOW  0
#define DHT_HIGH 1

#define BEGIN_READ_CYCLE_LOW          10 // in milli-seconds
#define BEGIN_READ_CYCLE_HIGH         40 // in micro-seconds
#define BEGIN_READ_CYCLE_DHT_LOW      80 // in micro-seconds
#define BEGIN_READ_CYCLE_DHT_HIGH     80 // in micro-seconds
#define BEGIN_DATA_READ_DHT_ATTENTION 50 // in micro-seconds 
#define BEGIN_DATA_RECEIVE_DHT_DATA   70 // in micro-seconds

// Forward references
dht_result_t dhtReadRawData(dht_t *, uint8_t *);
dht_result_t dhtProcessRawData(uint8_t *, dht_data_t *);

static const char *DHT_TAG = "DHT22";

// private struct to capture context
typedef struct _dhtpvt {
    void *   pDhtAddress;   // address of DHT pointer
    uint32_t successCount;
    uint32_t errorCount;
}dhtpvt_t;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 * dhtInitalize- Public method to initialize structs to read from DHT Sensor.
 * 
 * Inputs
 *      pinId - pin identifier.
 *      name  - DHT name identifier - used for output messages.
 *      ppDht - pointer-to-pointer to dht_t struct. Creates a context (dhtpvt_t)
 *              used to store address for allocated dht_t struct. 
 *
 * Returns dht_result_t.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
dht_result_t
dhtInitialize(uint8_t pinId, char *name, dht_t **ppDht) {
    if (!GPIO_IS_VALID_GPIO(pinId)) {
        ESP_LOGE(DHT_TAG, "DHT::initialize: Pin '%d' is not valid!", pinId);
        return DHT_INVALID_INPUT;
    }

    if (name == NULL || name[0] == 0) {
        ESP_LOGE(DHT_TAG, "DHT::initialize: 'name' invalid, cannot be NULL or empty!");
        return DHT_INVALID_INPUT;
    }

    bool nullTerminated = false;
    for (int x=0; x<DHT_MAX_SENSOR_NAME; x++) {
        if (name[x] == 0) {
            nullTerminated = true;
            break;
        }
    }

    if (nullTerminated == false) {
        name[DHT_MAX_SENSOR_NAME-1] = 0;
        ESP_LOGW(DHT_TAG, "DHT::initialize: 'name' invalid, not null-terminated. \  
                           Clipping to max allowed. name = '%s'. (%d)", name, __LINE__);
    }
 
    if (ppDht == NULL) {
        ESP_LOGE(DHT_TAG, "DHT::initialize: Pointer to return-value cannot be null!");    
        return DHT_INVALID_INPUT;
    }

    dhtpvt_t *pDhtpvt = malloc(sizeof(dhtpvt_t));
    if (pDhtpvt == NULL) {
        ESP_LOGE(DHT_TAG, "DHT::initialzie: Failed to allocate memory for dht opaque data!");
        return DHT_MALLOC_FAILED;
    }

    dht_t *pDht = malloc(sizeof(dht_t));
    if (pDht == NULL) {
        ESP_LOGE(DHT_TAG, "DHT::initialize: Failed to allocate memory for dht_data!");
        free(pDhtpvt);
        return DHT_MALLOC_FAILED;    
    }

    if (gpio_set_pull_mode(pinId, GPIO_PULLUP_ONLY) != ESP_OK) {
        ESP_LOGE(DHT_TAG, "DHT::initialize: Failed to set pin '%d' to PULLUP.", pinId);
        free(pDhtpvt);
        free(pDht);
        return DHT_FAILED_TO_SET_PIN_MODE;
    } 

    #if DEBUG == 1
    ESP_LOGI(DHT_TAG, "DHT::initialize: dhtpvt ptr=0x%x, pdht ptr=0x%x (%d)", (uint32_t)pDhtpvt, (uint32_t)pDht, __LINE__);
    #endif

    pDhtpvt->pDhtAddress = (void *)pDht;
    pDht->pin = pinId;
    pDht->pc = 0;
    pDht->opaque = (void *)pDhtpvt;
    *ppDht = pDht;
    return DHT_OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 * dhtCleanup- Public method to cleanup DHT pointer.
 * 
 * Inputs
 *      ppDht - pointer-to-pointer to dht_t
 *
 * Returns dht_result_t, sets *ppDht to NULL.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
dht_result_t
dhtCleanup(dht_t** ppDht) {
    if (ppDht == NULL || *ppDht == NULL) {
        ESP_LOGE(DHT_TAG, "DHT::cleanup: Invalid dht pointer provided, will not free memory! (%d)", __LINE__);
        return DHT_INVALID_INPUT;
    }

    dhtpvt_t *pvt = (dhtpvt_t *)((*ppDht)->opaque);

    #if DEBUG == 1
    ESP_LOGI(DHT_TAG, "DHT::cleanup: Freeing dhtpvt-ptr=0x%x and pdht-ptr=0x%x (%d)", (uint32_t)pvt, (uint32_t)*ppDht, __LINE__);
    #endif

    if (pvt->pDhtAddress != (void *)*ppDht) {
        ESP_LOGE(DHT_TAG, "DHT::cleanup: Invalid pDht pointer provided, will not free memory! (%d)", __LINE__);
        return DHT_INVALID_INPUT;
    } 
        
    free(*ppDht);
    *ppDht = NULL;
    return DHT_OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 * dhtRead - Public method to initiate read from DHT and return readable results.
 *
 * Returns dht_result_t.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
dht_result_t 
dhtRead(dht_t *pDht, dht_data_t *outdata) {
    if (pDht == NULL) {
        ESP_LOGE(DHT_TAG, "DHT::read: input 'dht' cannot be NULL.");
        return DHT_INVALID_INPUT;
    }

    if (outdata == NULL) {
        ESP_LOGE(DHT_TAG, "DHT::read: input 'outdata' cannot be NULL.");
        return DHT_INVALID_INPUT;
    }

    TickType_t ticks = xTaskGetTickCount(); 
    if ((ticks - pDht->pc) < pdMS_TO_TICKS(2000)) {
        ESP_LOGE(DHT_TAG, "DHT::read: call frequency cannot be less than 2 seconds. ticks=%ld, pc=%ld", ticks, pDht->pc);
        return DHT_READ_QUERY_TOO_FREQUENT;
    }
 
    // buffer to capture DHT22 sensor input
    uint8_t b[40] = {0};

    dht_result_t result = dhtReadRawData(pDht, b);
    if (result == DHT_OK) {
        result = dhtProcessRawData(b, outdata);
    }

    return result;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 *  dhtReadRawData - Private method to read data from DHT bus.
 *  
 *  Inputs
 *      pDht:   pointer to pin info
 *      b   :   pointer to raw data buffer (40 bytes)
 *
 *  Returns dht_result_t  
 * 
 *  Read bus line to get Temperature & Humidity data.
 *
 *  Notes 
 *      Algo from DHT22 spec sheet:
 *          1. Read interval should be greater than 2 seconds.
 *          2. To begin read cycle:
 *              a. MCU Pulls DATA LOW for 1-10ms   (start signal)
 *              b. MCU Pulls DATA HIGH for 20-40us (wait for response)
 *              c. DHT responds with 80us LOW
 *              d. DHT responds with 80us HIGH
 *              e. Response cycle
 *                  i.   50us LOW
 *                  ii.  1-bit DATA --> 26-28us = 0, 70us = 1 
 *                  iii. repeat 40 times 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
dht_result_t
dhtReadRawData(dht_t *pDht, uint8_t *b) {
   if (gpio_set_direction(pDht->pin, GPIO_MODE_OUTPUT) != ESP_OK) {
        ESP_LOGE(DHT_TAG, "DHT::read: failed to set pin:%d direction to OUTPUT. (%d)", pDht->pin, __LINE__);
        return DHT_FAILED_TO_SET_PIN_DIRECTION;;
    } 
    
    // START time-sensitive code.
    // send LOW signal to get DHT sensor's attention 
    if (gpio_set_level(pDht->pin, DHT_LOW) != ESP_OK) {
        ESP_LOGE(DHT_TAG, "DHT::read: failed to set pin:%d to LOW. (%d)", pDht->pin, __LINE__);
        return DHT_FAILED_TO_SET_PIN_LEVEL;
    }

    vTaskDelay(pdMS_TO_TICKS(BEGIN_READ_CYCLE_LOW));

    // send HIGH signal to tell DHT sensor that MCU is ready to receive data
    if (gpio_set_level(pDht->pin, DHT_HIGH) != ESP_OK) {
        ESP_LOGE(DHT_TAG, "DHT::read: failed to set pin:%d to HIGH. (%d)", pDht->pin, __LINE__);
        return DHT_FAILED_TO_SET_PIN_LEVEL;
    }

    ets_delay_us(BEGIN_READ_CYCLE_HIGH);

    // DHT sensor should be in LOW state after this delay
    // set port direction to input
    if (gpio_set_direction(pDht->pin, GPIO_MODE_INPUT) != ESP_OK) {
        ESP_LOGE(DHT_TAG, "DHT::read: failed to set pin:%d direction to INPUT. (%d)", pDht->pin, __LINE__);
        return DHT_FAILED_TO_SET_PIN_DIRECTION;
    }

    int prevState;
    int counter=0;
    // Wait for DHT sensor to ready itself to send info: 
    // DHT sensor will stay LOW for 80us
    while (counter++ < BEGIN_READ_CYCLE_DHT_LOW && (prevState = gpio_get_level(pDht->pin)) == DHT_LOW) {
        ets_delay_us(1);
    }

    if(prevState == DHT_LOW) {
        ESP_LOGE(DHT_TAG, "DHT::read: DHT22 sensor did not switch to HIGH in 80us. counter=%d. (%d)", counter, __LINE__);
        return DHT_SENSOR_DID_NOT_SWITCH_TO_HIGH;
    }

    counter = 0;
    // DHT sensor will stay HIGH for 80us
    while (counter++ < BEGIN_READ_CYCLE_DHT_HIGH && (prevState = gpio_get_level(pDht->pin)) == DHT_HIGH) {
        ets_delay_us(1);
    } 

    if(prevState == DHT_HIGH) {
        ESP_LOGE(DHT_TAG, "DHT::read: DHT22 sensor did not switch to LOW in 80us. counter=%d (%d)", counter,  __LINE__);
        return DHT_SENSOR_DID_NOT_SWITCH_TO_LOW;
    }

    // DHT sensor in LOW state, begin reading bits
    for (int x=0;x<40;x++) {
        // Now begin receiving data, DHT sensor is in HIGH state
        // 50us LOW followed by variable signal:
        //      ~28us = HIGH 
        //      ~70us = HIGH 
        counter = 0;
        while (counter++ < BEGIN_DATA_READ_DHT_ATTENTION && (prevState = gpio_get_level(pDht->pin)) == DHT_LOW) {
            ets_delay_us(1);
        } 

        if(prevState == DHT_LOW) {
            ESP_LOGE(DHT_TAG, "DHT::read: DHT22 sensor did not switch to HIGH in 50us. counter=%d (%d)", counter, __LINE__);
            return DHT_SENSOR_DID_NOT_SWITCH_TO_HIGH;
        }
 
        counter = 0;
        while (counter++ < BEGIN_DATA_RECEIVE_DHT_DATA && (prevState = gpio_get_level(pDht->pin)) == DHT_HIGH) {
            ets_delay_us(1);
        }

        if (prevState == DHT_HIGH) {
            ESP_LOGE(DHT_TAG, "DHT::read: DHT22 sensor did not set bus to LOW. counter=%d (%d)", counter, __LINE__);
            return DHT_SENSOR_DID_NOT_SWITCH_TO_LOW;
        }

        // store the counter value to analyze bit-value later
        b[40-(x+1)] = counter;
    }
    
    // END time-sensitive code. 

    #if DEBUG == 1
    ESP_LOGI(DHT_TAG, "DHT::read:RH: [%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d]", b[39], b[38], b[37], b[36], b[35], b[34], b[33], b[32]);
    ESP_LOGI(DHT_TAG, "DHT::read:RH: [%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d]", b[31], b[30], b[29], b[28], b[27], b[26], b[25], b[24]);
    ESP_LOGI(DHT_TAG, "DHT::read:TP: [%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d]", b[23], b[22], b[21], b[20], b[19], b[18], b[17], b[16]);
    ESP_LOGI(DHT_TAG, "DHT::read:TP: [%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d]", b[15], b[14], b[13], b[12], b[11], b[10], b[9], b[8]);
    ESP_LOGI(DHT_TAG, "DHT::read:CS: [%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d]", b[7], b[6], b[5], b[4], b[3], b[2], b[1], b[0]);
    #endif

    return DHT_OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 *  dhtProcessRawData - Private Method to convert raw DHT sensor data to Temp, 
 *                      RH, and Checksum 
 *  
 *  Inputs
 *      b      : pointer to raw data buffer (40 bytes) 
 *      outdata: pointer to dht_data_t struct
 8
 *  Returns - dht_result_t
 *
 *  Notes 
 *           i.  16 bits RH       - Convert to decimal and divide by 10. 
 *                                  Range 0-100%
 *           ii. 16 bits T        - Convert to decimal and divide by 10. 
 *                                  Range -40-80C, -40-176F
 *                                  High order bit=1 means negative T
 *           iii. 8 bits CHECKSUM - Lower Order 8 bits of SUM(i + ii) 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
dht_result_t
dhtProcessRawData(uint8_t *b, dht_data_t *outdata) {
    /*
     *  Buffer Map:
     *      Relative Humidity = buffer[39]...buffer[24] 
     *      Temperature       = buffer[23]...buffer[8]
     *      Checsum           = buffer[7]...buffer[0]
     */

    uint32_t rh=0;
    // Get RH
    for (int x=39;x>23;x--) {
        rh |= b[x] > 19 ? (uint8_t)0x1 : (uint8_t)0x0; 
        rh <<= x>24 ? 1 : 0;        // don't shift last bit
    }

    // Get TEMP
    // highest order bit if 1 means negative temperature
    bool isNegative = b[23] > 19 ? true : false;
    uint32_t temp=0;
    for (int x=22;x>7;x--) {
        temp |= b[x] > 19 ? (uint8_t)0x1 : (uint8_t)0x0; 
        temp <<= x>8 ? 1 : 0;       // don't shift last bit
    }

    if (isNegative) {
        temp *= -1;
    }

    // Get CHECKSUM
    uint32_t checksum=0;
    for (int x=7;x>-1;x--) {
        checksum |= b[x] > 19 ? (uint8_t)0x1 : (uint8_t)0x0;
        checksum <<= x>0 ? 1 : 0;   // don't shift last bit
    }

    // validate sensor reading against checksum
    // sum up the 4 bytes that make up the RH+TEMP 
    // readings individually and keep the lower 8 bits.
    uint32_t calculated_checksum = ((rh & 0x0000ff00) >> 8) + (rh & 0x000000ff) + ((temp & 0x0000ff00) >> 8) + (temp & 0x000000ff);
    calculated_checksum &= 0x0000ff;

    if (checksum != calculated_checksum) {
        ESP_LOGE(DHT_TAG, "DHT::read: Checksum failure! CS=0x%x, Calculated-CS=0x%x. (%d)", checksum, calculated_checksum, __LINE__);
        return DHT_INVALID_CHECKSUM;
    }

    #if DEBUG == 1
    ESP_LOGI(DHT_TAG, "DHT::read: RH = %d (0x%x)", rh, rh);
    ESP_LOGI(DHT_TAG, "DHT::read: TEMP = %d C (0x%x)", temp, temp);
    ESP_LOGI(DHT_TAG, "DHT::read: checksum = 0x%x", checksum);
    #endif

    outdata->csTempWhole    = temp/10;
    outdata->csTempFraction = (temp*10)%100;    // math for fahrenheit conversion divides by 100 showing extra zero decimal place, do same here for consistency in display.
    outdata->rhWhole        = rh/10;
    outdata->rhFraction     = rh%10;
    temp *= 100;    // get rid of fraction altogether
    temp /= 50;     // convert to fahrenheit
    temp *= 9;
    outdata->faTempWhole    = temp/100 + 32;
    outdata->faTempFraction = temp%100;

    return DHT_OK;
}
