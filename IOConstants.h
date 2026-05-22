#ifndef IO_CONSTANTS_H    /* Guard against multiple inclusion */
#define IO_CONSTANTS_H

#include <xc.h>
#include "IO_Ports.h"
#include "AD.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_1_PORT       AD_PORTV5
#define BEACON_2_PORT       AD_PORTV6

#define NUM_TAPE_SENSORS 5

typedef struct tapeConsts {
    uint8_t port;
    uint16_t emitter;
    uint16_t detector;
} TapeIOConstants;

static TapeIOConstants TAPE_SENSOR_CONSTANTS[NUM_TAPE_SENSORS] = { 
    { PORTV, PIN3, AD_PORTV4 },
    { PORTV, PIN5, AD_PORTV6 },
    { PORTV, PIN7, AD_PORTV8 },
    { PORTW, PIN3, AD_PORTW4 },
    { PORTW, PIN5, AD_PORTW6 },
};


#define NUM_MOTORS 5

typedef struct motorConsts {
    uint8_t port;
    uint16_t en;
    uint16_t in1;
    uint16_t in2;
} MotorIOConstants;

static MotorIOConstants MOTOR_SENSOR_CONSTANTS[NUM_MOTORS] = { 
    { PORTV, PIN3, AD_PORTV4 },
    { PORTV, PIN5, AD_PORTV6 },
    { PORTV, PIN7, AD_PORTV8 },
    { PORTW, PIN3, AD_PORTW4 },
    { PORTW, PIN5, AD_PORTW6 },
};

#define TRACKWIRE_PORT      PORTX
#define TRACKWIRE_1_PORT    PORTX03_BIT
#define TRACKWIRE_2_PORT    PORTX04_BIT

#ifdef __cplusplus
}
#endif

#endif /* IO_CONSTANTS_H */

/* *****************************************************************************
 End of File
 */
