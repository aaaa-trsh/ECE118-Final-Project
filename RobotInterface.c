#include <xc.h>
#include "IO_Ports.h"
#include "pwm.h"
#include "AD.h"
#include "IOConstants.h"
#include "RobotInterface.h"
#include <stdio.h>

//uint16_t _SetDriveMotor(DriveMotorIOConstants driveConsts, int8_t speed);
int Rescale(int signal);

double scale_ratio = 0;
int Rescale(int signal) {
    return signal;// / scale_ratio;
}

static int robot_initialized = 0;

void InitRobot(void) {
    if (robot_initialized) {
        return;
    }
    robot_initialized = 1;
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
    scale_ratio = ((double)AD_GetBatteryVoltage()) / ((double)275);
}

uint16_t SetDriveMotorRescale(DriveMotorIOConstants driveConsts, int16_t speed, int8_t rescale) {
    if (speed == 0) {
        IO_PortsSetPortBits(driveConsts.port, driveConsts.in1);
        IO_PortsSetPortBits(driveConsts.port, driveConsts.in2);
        PWM_SetDutyCycle(driveConsts.pwm, MIN_PWM);
        return 0;
    }
    
    uint8_t direction = speed > 0;
    if (driveConsts.invert) {
        direction = !direction;
    }
    
    if (direction) {
        IO_PortsClearPortBits(driveConsts.port, driveConsts.in1);
        IO_PortsSetPortBits(driveConsts.port, driveConsts.in2);
    } else {
        IO_PortsSetPortBits(driveConsts.port, driveConsts.in1);
        IO_PortsClearPortBits(driveConsts.port, driveConsts.in2);
    }
    
    // translate speed to be unsigned 0 - 127
    uint16_t absspd = (uint16_t)(((double)(speed < 0 ? -speed : speed)) * driveConsts.multiplier);
    uint16_t unispd = (absspd > 1000) ? 1000 : absspd;
    
    if (rescale) {
        PWM_SetDutyCycle(driveConsts.pwm, Rescale(unispd));
    } else {
        PWM_SetDutyCycle(driveConsts.pwm, unispd);
    }
    return unispd;
}
uint16_t SetDriveMotor(DriveMotorIOConstants driveConsts, int16_t speed) {
    return SetDriveMotorRescale(driveConsts, speed, 1);
}

void MecanumDriveRescale(int16_t fwd, int16_t strafe, int16_t rot, int8_t rescale) {
    int16_t afwd    = (fwd    < 0) ? -(int16_t)fwd    : fwd;
    int16_t astrafe = (strafe < 0) ? -(int16_t)strafe : strafe;
    int16_t arot    = (rot    < 0) ? -(int16_t)rot    : rot;

    int16_t sum  = afwd + astrafe + arot;
    int16_t norm = (sum > 1000) ? sum : 1000;

    int16_t fl = ((int16_t)(fwd + strafe + rot) * 1000) / norm;
    int16_t fr = ((int16_t)(fwd - strafe - rot) * 1000) / norm;
    int16_t rl = ((int16_t)(fwd - strafe + rot) * 1000) / norm;
    int16_t rr = ((int16_t)(fwd + strafe - rot) * 1000) / norm;

    SetDriveMotorRescale(DRIVE_FRONT_LEFT,  fl, rescale);
    SetDriveMotorRescale(DRIVE_FRONT_RIGHT, fr, rescale); 
    SetDriveMotorRescale(DRIVE_REAR_LEFT,   rl, rescale);
    SetDriveMotorRescale(DRIVE_REAR_RIGHT,  rr, rescale);
}

void MecanumDrive(int16_t fwd, int16_t strafe, int16_t rot) {
    MecanumDriveRescale(fwd, strafe, rot, 1);
}

void DriveForward(int16_t spd)  { TankDrive(spd, spd);      }
void DriveBackward(int16_t spd) { TankDrive(-spd, -spd);    }
void DriveLeft(int16_t spd)     {
    SetDriveMotorRescale(DRIVE_FRONT_LEFT,  -spd * 0.97, 1);
    SetDriveMotorRescale(DRIVE_FRONT_RIGHT,  spd * 0.91, 1); 
    SetDriveMotorRescale(DRIVE_REAR_LEFT,    spd * 0.94, 1);
    SetDriveMotorRescale(DRIVE_REAR_RIGHT,  -spd, 1);
}
void DriveRight(int16_t spd)    {
    MecanumDrive(0, spd, 30);
//    SetDriveMotorRescale(DRIVE_FRONT_LEFT,   spd * 0.94, 1);
//    SetDriveMotorRescale(DRIVE_FRONT_RIGHT, -spd * 0.94, 1); 
//    SetDriveMotorRescale(DRIVE_REAR_LEFT,   -spd, 1);
//    SetDriveMotorRescale(DRIVE_REAR_RIGHT,   spd, 1);
}
void DriveStop()                { MecanumDrive(0, 0, 0);    }

void TankDrive(int16_t left, int16_t right) {
    SetDriveMotor(DRIVE_FRONT_LEFT,  left);
    SetDriveMotor(DRIVE_REAR_LEFT,   left);
    SetDriveMotor(DRIVE_FRONT_RIGHT, right); 
    SetDriveMotor(DRIVE_REAR_RIGHT,  right);
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
uint8_t ReadTapeSensorSL(void) { return (IO_PortsReadPort(PORTV) & PIN8) == 0; }
uint8_t ReadTapeSensorSR(void) { return (IO_PortsReadPort(PORTW) & PIN5) == 0; }

double ReadBeaconSensor1(void) { return ((double)AD_ReadADPin(AD_BEACON_1)) * 0.00322265625; }
double ReadBeaconSensor2(void) { return ((double)AD_ReadADPin(AD_BEACON_2)) * 0.00322265625; }
unsigned int ReadBatteryVoltage(void) { return AD_ReadADPin(BAT_VOLTAGE); }

uint16_t ReadObstacleSensor1(void) { return (IO_PortsReadPort(OBSTACLE_1_PORT) & OBSTACLE_1_PIN) == 0; }
uint16_t ReadObstacleSensor2(void) { return (IO_PortsReadPort(OBSTACLE_2_PORT) & OBSTACLE_2_PIN) == 0; }
