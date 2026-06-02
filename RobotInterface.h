#include <xc.h>
#include "IOConstants.h"

#ifndef _ROBOT_INTERFACE_H    /* Guard against multiple inclusion */
#define _ROBOT_INTERFACE_H

    void InitRobot(void);
    
    /**
      @Parameters
        @param fwd      -128: back     127: fwd
        @param strafe   -128: left     127: right
     */
    void TankDrive(int16_t left, int16_t right);
    void MecanumDrive(int16_t fwd, int16_t strafe, int16_t rot);
    uint16_t SetDriveMotor(DriveMotorIOConstants driveConsts, int16_t speed);
    
    /**
      @Parameters
        @param enabled -- bool is on?
        @param distance 0-255
     */
    void SetShooter(uint8_t enabled, uint8_t distance);
    void SetIndexer(uint8_t enabled);
    
    #define TAPE_WHITE 0
    #define TAPE_BLACK 1
    uint8_t ReadTapeSensorFL(void);
    uint8_t ReadTapeSensorFR(void);
    uint8_t ReadTapeSensorSL(void);
    uint8_t ReadTapeSensorSR(void);
    uint8_t ReadTapeSensorR(void);

    double ReadBeaconSensor1(void);
    double ReadBeaconSensor2(void);
    
    uint16_t ReadObstacleSensor1(void);
    uint16_t ReadObstacleSensor2(void);

#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_INTERFACE_H */

