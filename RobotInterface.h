#include <xc.h>

#ifndef _ROBOT_INTERFACE_H    /* Guard against multiple inclusion */
#define _ROBOT_INTERFACE_H

    void InitRobot(void);
    
    /**
      @Parameters
        @param fwd      -128: back     127: fwd
        @param strafe   -128: left     127: right
     */
    void MecanumDrive(int8_t fwd, int8_t strafe, int8_t rot);
    
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

    uint16_t ReadBeaconSensor1(void);
    uint16_t ReadBeaconSensor2(void);
    
    uint16_t ReadTrackwireSensor1(void);
    uint16_t ReadTrackwireSensor2(void);

#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_INTERFACE_H */

