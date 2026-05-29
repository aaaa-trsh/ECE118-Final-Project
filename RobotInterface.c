#include <xc.h>
#include "IO_Ports.h"
#include "pwm.h"
#include "AD.h"
#include "IOConstants.h"
#include "RobotInterface.h"
#include <stdio.h>

//uint16_t _SetDriveMotor(DriveMotorIOConstants driveConsts, int8_t speed);

void InitRobot(void) {
    PWM_Init();
    PWM_SetFrequency(PWM_2KHZ);

    PWM_AddPins(SHOOTER_MOTOR_PWM);
    
    // Initialize beacons
    AD_Init();
    AD_AddPins(AD_BEACON_1);
    AD_AddPins(AD_BEACON_2);
    
    // Initialize indexer motor
    IO_PortsSetPortOutputs(INDEXER_MOTOR_PORT, INDEXER_MOTOR_EN);
    
    // Initialize drive motors
    IO_PortsSetPortOutputs(DRIVE_REAR_LEFT.port,     DRIVE_REAR_LEFT.in1   );
    IO_PortsSetPortOutputs(DRIVE_REAR_LEFT.port,     DRIVE_REAR_LEFT.in2   );
    PWM_AddPins(DRIVE_REAR_LEFT.pwm);
    
    IO_PortsSetPortOutputs(DRIVE_FRONT_RIGHT.port,   DRIVE_FRONT_RIGHT.in1 );
    IO_PortsSetPortOutputs(DRIVE_FRONT_RIGHT.port,   DRIVE_FRONT_RIGHT.in2 );
    PWM_AddPins(DRIVE_REAR_RIGHT.pwm);
    
    IO_PortsSetPortOutputs(DRIVE_REAR_RIGHT.port,    DRIVE_REAR_RIGHT.in1  );
    IO_PortsSetPortOutputs(DRIVE_REAR_RIGHT.port,    DRIVE_REAR_RIGHT.in2  );
    PWM_AddPins(DRIVE_FRONT_LEFT.pwm);
    
    IO_PortsSetPortOutputs(DRIVE_FRONT_LEFT.port,    DRIVE_FRONT_LEFT.in1  );
    IO_PortsSetPortOutputs(DRIVE_FRONT_LEFT.port,    DRIVE_FRONT_LEFT.in2  );
    PWM_AddPins(DRIVE_FRONT_RIGHT.pwm);
    
    // Initialize tapes
    IO_PortsSetPortInputs(PORTV, TAPE_PORT_V_PINS);
    IO_PortsSetPortInputs(PORTW, TAPE_PORT_W_PINS);
    
    // Initialize trackwires
    IO_PortsSetPortInputs(OBSTACLE_1_PORT, OBSTACLE_1_PIN);
    IO_PortsSetPortInputs(OBSTACLE_2_PORT, OBSTACLE_2_PIN);
    
    SetShooter(0, 0);
    SetIndexer(0);
}

uint16_t SetDriveMotor(DriveMotorIOConstants driveConsts, int8_t speed) {
    if (speed == 0) {
        IO_PortsSetPortBits(driveConsts.port, driveConsts.in1);
        IO_PortsSetPortBits(driveConsts.port, driveConsts.in2);
        PWM_SetDutyCycle(driveConsts.pwm, MIN_PWM);
//        printf("zero", driveConsts.pwm);
        return 0;
    }
    
    uint8_t direction = speed > 0;
    if (driveConsts.invert) {
        direction = !direction;
    }
    
    // TODO: Finish this :)
    if (direction) {
        IO_PortsSetPortBits(driveConsts.port, driveConsts.in1);
        IO_PortsClearPortBits(driveConsts.port, driveConsts.in2);
    } else {
        IO_PortsClearPortBits(driveConsts.port, driveConsts.in1);
        IO_PortsSetPortBits(driveConsts.port, driveConsts.in2);
    }
    
    // translate speed to be unsigned 0 - 127
    uint16_t absspd = speed < 0 ? -speed : speed;
    uint16_t unispd = (absspd > 127) ? 127 : absspd;
    uint16_t retval = (MAX_PWM * unispd) / 127;
    
    PWM_SetDutyCycle(driveConsts.pwm, retval);
    
    return retval;
}

void MecanumDrive(int8_t fwd, int8_t strafe, int8_t rot)
{
    int16_t afwd    = (fwd    < 0) ? -(int16_t)fwd    : fwd;
    int16_t astrafe = (strafe < 0) ? -(int16_t)strafe : strafe;
    int16_t arot    = (rot    < 0) ? -(int16_t)rot    : rot;

    int16_t sum  = afwd + astrafe + arot;
    int16_t norm = (sum > 127) ? sum : 127;

    int16_t fl = ((int16_t)(fwd + strafe + rot) * 127) / norm;
    int16_t fr = ((int16_t)(fwd - strafe - rot) * 127) / norm;
    int16_t rl = ((int16_t)(fwd - strafe + rot) * 127) / norm;
    int16_t rr = ((int16_t)(fwd + strafe - rot) * 127) / norm;
    
//    printf("Set motor speeds -- FL:%5d FR:%5d RL:%5d RR:%5d\n", 
//        SetDriveMotor(DRIVE_FRONT_LEFT,  (int8_t)fl),
//        SetDriveMotor(DRIVE_FRONT_RIGHT, (int8_t)fr), 
//        SetDriveMotor(DRIVE_REAR_LEFT,   (int8_t)rl), 
//        SetDriveMotor(DRIVE_REAR_RIGHT,  (int8_t)rr)
//    );
}

void SetShooter(uint8_t enabled, uint8_t distance) {
    if (enabled) {
        if (PWM_SetDutyCycle(SHOOTER_MOTOR_PWM, 1000) != SUCCESS) {
            printf("Shooter enabled error\n");
        }
    } else {
        if (PWM_SetDutyCycle(SHOOTER_MOTOR_PWM, MIN_PWM) != SUCCESS) {
            printf("Shooter disable error\n");
        }
    }
}

void SetIndexer(uint8_t enabled) {
    if (enabled) {
        if (IO_PortsSetPortBits(INDEXER_MOTOR_PORT, INDEXER_MOTOR_EN) != SUCCESS) {
            printf("Indexer enable error\n");
        }
    } else {
        if (IO_PortsClearPortBits(INDEXER_MOTOR_PORT, INDEXER_MOTOR_EN)) {
            printf("Indexer disable error\n");
        }
    }
}

uint8_t ReadTapeSensorFR(void) { return (IO_PortsReadPort(PORTV) & PIN3) == 0; }
uint8_t ReadTapeSensorR(void)  { return (IO_PortsReadPort(PORTV) & PIN5) == 0; }
uint8_t ReadTapeSensorFL(void) { return (IO_PortsReadPort(PORTV) & PIN7) == 0; }
uint8_t ReadTapeSensorSL(void) { return (IO_PortsReadPort(PORTW) & PIN3) == 0; }
uint8_t ReadTapeSensorSR(void) { return (IO_PortsReadPort(PORTW) & PIN5) == 0; }

uint16_t ReadBeaconSensor1(void) { return AD_ReadADPin(AD_BEACON_1); }
uint16_t ReadBeaconSensor2(void) { return AD_ReadADPin(AD_BEACON_2); }

uint16_t ReadObstacleSensor1(void) { return (IO_PortsReadPort(OBSTACLE_1_PORT) & OBSTACLE_1_PIN) == 0; }
uint16_t ReadObstacleSensor2(void) { return (IO_PortsReadPort(OBSTACLE_2_PORT) & OBSTACLE_2_PIN) == 0; }