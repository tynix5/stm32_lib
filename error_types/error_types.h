#ifndef ERROR_TYPES_H_
#define ERROR_TYPES_H_

/************ Generic Global Defines *************/
#define SPI_TIMEOUT_MS                  5
#define SPI_MAX_RETRIES                 5
/*************************************************/

/************** Generic Error Types **************/
typedef enum {
    
    RET_OK = 0,                         // success
    RET_BUSY,
    RET_TIMEOUT,                        // timed out
    RET_INVALID_CFG,                    // bad configuration
    RET_DISCREPANCY                     // written value != read value
    
} ErrorType;

typedef enum {
    
    FEAT_DISABLE = 0,
    FEAT_ENABLE
    
} EnableType;
/*************************************************/

#endif