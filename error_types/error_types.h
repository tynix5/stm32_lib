
/************ Generic Global Defines *************/
#define SPI_TIMEOUT_MS                  5
#define SPI_MAX_RETRIES                 5

#define ENABLE                          1
#define DISABLE                         0

/************** Generic Error Types **************/
typedef enum {

    SUCCESS = 0,
    TIMEOUT,
    INVALID_CFG

} ErrorType;