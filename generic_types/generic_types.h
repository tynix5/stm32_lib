#ifndef GENERIC_TYPES_H_
#define GENERIC_TYPES_H_

// get rid of this
#define SPI_TIMEOUT_MS  5
#define SPI_MAX_RETRIES 5

/***************** Generic Types *****************/
typedef enum
{
    STATUS_OK = 0,     // success
    STATUS_ERR,        // general error case, try to avoid using this
    STATUS_BUSY,       // peripheral or controller is busy
    STATUS_TIMEOUT,    // timed out
    STATUS_INVALID,    // bad configuration
    STATUS_DISCREPANCY // written value != read value
} status_t;

typedef enum
{
    FEAT_DISABLE = 0,
    FEAT_ENABLE
} enable_t;
/*************************************************/

#endif