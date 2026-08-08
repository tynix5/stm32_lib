#ifndef ERROR_TYPES_H_
#define ERROR_TYPES_H_

// get rid of this
#define SPI_TIMEOUT_MS                      5


/************** Generic Error Types **************/
typedef enum {
    
    STATUS_OK = 0,                         // success
    STATUS_ERR,
    STATUS_BUSY,
    STATUS_TIMEOUT,                        // timed out
    STATUS_INVALID_CFG,                    // bad configuration
    STATUS_DISCREPANCY                     // written value != read value
    
} status_t;

// keep for other libraries at the moment
typedef enum {
    
    RET_OK = 0,                         // success
    RET_ERR,
    RET_BUSY,
    RET_TIMEOUT,                        // timed out
    RET_INVALID_CFG,                    // bad configuration
    RET_DISCREPANCY                     // written value != read value
    
} ErrorType;

// typedef enum {
    
//     FEAT_DISABLE = 0,
//     FEAT_ENABLE
    
// } enable_t;

// get rid
typedef enum {
    
    FEAT_DISABLE = 0,
    FEAT_ENABLE
    
} EnableType;
/*************************************************/

#endif