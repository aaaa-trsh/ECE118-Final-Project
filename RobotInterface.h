#include <xc.h>
#include "IOConstants.h"

#ifndef _ROBOT_INTERFACE_H    /* Guard against multiple inclusion */
#define _ROBOT_INTERFACE_H

    void InitRobot(void);
    
    void TankDrive(int16_t left, int16_t right);

    /**
      @Parameters
        @param fwd      -1000: back     1000: fwd
        @param strafe   1000: left     -1000: right
     */
    void MecanumDrive(int16_t fwd, int16_t strafe, int16_t rot);
    void MecanumDriveRescale(int16_t fwd, int16_t strafe, int16_t rot, int8_t rescale);
    uint16_t SetDriveMotor(DriveMotorIOConstants driveConsts, int16_t speed);
    uint16_t SetDriveMotorRescale(DriveMotorIOConstants driveConsts, int16_t speed, int8_t rescale);

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
    unsigned int ReadBatteryVoltage(void);


#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_INTERFACE_H */

