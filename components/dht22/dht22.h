/*
 *   DHT22 Support File
 *   Author: Hector Cura
 *   Date:   9/22/2018
 */

#ifndef _dht22_h_
#define _dht22_h_

// set to 1 to display debugging info 
#ifndef DEBUG
#define DEBUG 1  
#endif

#define DHT_MAX_SENSOR_NAME 32

typedef enum _dht_result_t {
    DHT_OK,
    DHT_INVALID_INPUT,
    DHT_MALLOC_FAILED,
    DHT_FAILED_TO_SET_PIN_MODE,
    DHT_READ_QUERY_TOO_FREQUENT,
    DHT_FAILED_TO_SET_PIN_DIRECTION,
    DHT_FAILED_TO_SET_PIN_LEVEL,
    DHT_SENSOR_DID_NOT_SWITCH_TO_HIGH,
    DHT_SENSOR_DID_NOT_SWITCH_TO_LOW,
    DHT_INVALID_CHECKSUM  
}dht_result_t;

typedef struct _dht_t {
    char       name[DHT_MAX_SENSOR_NAME];    // Room for 31 characters + null terminator.
    uint8_t    pin;
    void *     opaque;      // Contains pointer to private struct to capture context
    TickType_t pc;
}dht_t;

typedef struct _dht_data_t {
    uint16_t faTempWhole;       // fahrenheit result
    uint16_t faTempFraction;
    uint16_t csTempWhole;       // celsius result - sensor returns celsius by default range: [-40,80]
    uint16_t csTempFraction;
    uint16_t rhWhole;
    uint16_t rhFraction;        // range: [0,100] percent
}dht_data_t;

/*
 *  Initialize dht for reading.
 *  Input: dataPinId - DATA line
 */
dht_result_t 
dhtInitialize(uint8_t pinId, char *name, dht_t **ppDht);

/*
 *  Cleanup timer structure.
 *  Input: pointer to pointer to DHT created in initialize call.
 */
dht_result_t
dhtCleanup(dht_t** ppDht);

/*
 *  Read bus line to get Temperature & Humidity data.
 *  Inputs:
 *      pinId - GPIO pin to read/write to
 *      name  - sensor name
 *  Returns:
 *      TRUE  - success, data pointer will contain output.
 *      FALSE - input data pointer is null, 
 *              error reading DATA pin, or 
 *              consecutive call less than 2 seconds in frequency.
 */
dht_result_t 
dhtRead(dht_t *dht, dht_data_t *outdata);

#endif //_dht22_h_
