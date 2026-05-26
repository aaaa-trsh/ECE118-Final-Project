#ifndef IO_CONSTANTS_H    /* Guard against multiple inclusion */
#define IO_CONSTANTS_H

#include <xc.h>
#include "IO_Ports.h"
#include "pwm.h"
#include "AD.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#define NUM_TAPE_SENSORS 5
#define TAPE_PORT_V_PINS (PIN3 | PIN5 | PIN7)
#define TAPE_PORT_W_PINS (PIN3 | PIN5)

#define DRIVE_RL        0
#define DRIVE_FR        1
#define DRIVE_RR        2
#define DRIVE_FL        3

typedef struct driveMotorConsts {
    uint8_t port;
    uint16_t pwm;
    uint16_t in1;
    uint16_t in2;
    uint8_t invert;
} DriveMotorIOConstants;

DriveMotorIOConstants DRIVE_REAR_LEFT   = { PORTY, PWM_PORTY10, PIN8, PIN9, 0 };
DriveMotorIOConstants DRIVE_FRONT_RIGHT = { PORTY, PWM_PORTY04, PIN3, PIN5, 1 };
DriveMotorIOConstants DRIVE_REAR_RIGHT  = { PORTY, PWM_PORTY12, PIN6, PIN7, 0 };
DriveMotorIOConstants DRIVE_FRONT_LEFT  = { PORTZ, PWM_PORTZ06, PIN4, PIN5, 0 };

#define SHOOTER_MOTOR_PWM   PWM_PORTX11
#define INDEXER_MOTOR_PORT  PORTX
#define INDEXER_MOTOR_EN    PIN3

#define TRACKWIRE_1_PORT    PORTZ
#define TRACKWIRE_1_PIN     PIN3
#define TRACKWIRE_2_PORT    PORTY
#define TRACKWIRE_2_PIN     PIN11

#define AD_BEACON_1         AD_PORTW8
#define AD_BEACON_2         AD_PORTW7

#ifdef __cplusplus
}
#endif

#endif /* IO_CONSTANTS_H */

/* *****************************************************************************
 End of File
 */
