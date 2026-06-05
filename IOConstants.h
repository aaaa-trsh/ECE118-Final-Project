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
#define TAPE_PORT_V_PINS (PIN3 | PIN5 | PIN7 | PIN8)
#define TAPE_PORT_W_PINS (PIN5)

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
    double multiplier;
} DriveMotorIOConstants;

extern DriveMotorIOConstants DRIVE_REAR_LEFT;
extern DriveMotorIOConstants DRIVE_FRONT_RIGHT;
extern DriveMotorIOConstants DRIVE_REAR_RIGHT;
extern DriveMotorIOConstants DRIVE_FRONT_LEFT;

#define SHOOTER_MOTOR_PWM   PWM_PORTX11
#define INDEXER_MOTOR_PORT  PORTX
#define INDEXER_MOTOR_EN    PIN3

#define OBSTACLE_1_PORT    PORTV
#define OBSTACLE_1_PIN     PIN4
#define OBSTACLE_2_PORT    PORTV
#define OBSTACLE_2_PIN     PIN6

#define AD_BEACON_1         AD_PORTW7
#define AD_BEACON_2         AD_PORTW8

#ifdef __cplusplus
}
#endif

#endif /* IO_CONSTANTS_H */

/* *****************************************************************************
 End of File
 */
