#include "BOARD.h"
#include <xc.h>
#include <stdio.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "pwm.h"
#include "timers.h"

#include "RobotInterface.h"

uint8_t RunStateAlign(void);
uint8_t RunStateNav(void);
    
#define STATE_ALIGN     0
#define STATE_NAV_ISZ   1

static int state = STATE_ALIGN;

#define SLOW 750
#define CORNER_ALIGN_RATE_PER_MS 0.6 // counts per millisecond

#define DO_STATE_MACHINE

void main(void)
{
    
    ES_Return_t ErrorType;
    
    BOARD_Init();
    InitRobot();
    TIMERS_Init();
    
    printf("Starting Team 5 Robot!!\r\n");
    printf("using the 2nd Generation Events & Services Framework\r\n");
    
//    ErrorType = ES_Initialize();
//    if (ErrorType == Success) {
//        ErrorType = ES_Run();
//    }
//    
//    //if we got to here, there was an error
//    switch (ErrorType) {
//    case FailedPointer:
//        printf("Failed on NULL pointer");
//        break;
//    case FailedInit:
//        printf("Failed Initialization");
//        break;
//    default:
//        printf("Other Failure: %d", ErrorType);
//        break;
//    }
//    for (;;)
//        ;
    
#ifdef DO_STATE_MACHINE
    uint8_t fin = 0;
    while (1) {
        switch (state) {
            case STATE_ALIGN:
                fin = RunStateAlign();
                if (fin) { state = STATE_NAV_ISZ; }
                break;
           case STATE_NAV_ISZ:
               fin = RunStateNav();
               if (fin) { state = -1; }
            default:
                break;
        }
    }
#endif
#ifndef DO_STATE_MACHINE
    while (1) {
//        SetShooter(1, 0);
//        SetIndexer(1);
    }
#endif
}

#define STATE_ALIGN_SCAN                0
#define STATE_ALIGN_FIND_SOUTH          1
#define STATE_ALIGN_FIND_SOUTH_BOUND    2
#define STATE_ALIGN_BACK_OFF            3
#define STATE_ALIGN_STRAIGHTEN          4
#define STATE_ALIGN_CORNER              5
#define STATE_ALIGN_180_PT1             6
#define STATE_ALIGN_180_PT2             7

uint8_t RunStateAlign(void) {
    // Initialize state mgmt variables
    static int substate = STATE_ALIGN_SCAN;
    static uint8_t scanned_beacon = 0;
    static uint8_t scanned_tape = 0;
    static uint8_t initialized = 0;

    static uint8_t corner_last_trip_time = 0;
    static uint8_t tapes_tripped = 0;

    // Read sensors + state variables
    uint8_t tape_fr = ReadTapeSensorFR() == 0 ? 1 : 0;
    uint8_t tape_fl = ReadTapeSensorFL() == 0 ? 1 : 0;
    uint8_t tape_sr = ReadTapeSensorSR() == 0 ? 1 : 0;
    uint8_t tape_sl = ReadTapeSensorSL() == 0 ? 1 : 0;
    double beacon = ReadBeaconSensor1();

    uint8_t timer_expired = TIMERS_IsTimerExpired(1) == TIMER_EXPIRED;


    if (!initialized) {
        printf("INITIALIZE!");
        TIMERS_InitTimer(1, 6000);
        initialized = 1;
    }

    switch (substate) {
        // first spin around and keep track of landmarkss
        case STATE_ALIGN_SCAN:
            printf("ALIGN_SCAN: beacon?=%d, tape?=%d\n", scanned_beacon, scanned_tape);
            MecanumDrive(0, 0, -800);

            if (beacon > 2.4)       { scanned_beacon = 1; }
            if (tape_fr || tape_fl) { scanned_tape = 1; }
            if (timer_expired)      { substate = STATE_ALIGN_FIND_SOUTH; }
            break;
        
        // then turn to south
        case STATE_ALIGN_FIND_SOUTH:
            printf("ALIGN_FIND_SOUTH: beacon=%5f\n", beacon);
            MecanumDrive(0, 0, scanned_beacon ? SLOW : -SLOW);

            if (beacon > 2.4) {
                MecanumDrive(0, 0, 0);
                
                // attempt to only get off the edge
                if (scanned_tape) { // if were on the tape, back up from the edge and align
                    MecanumDrive(-SLOW, 0, 0);
                    substate = STATE_ALIGN_BACK_OFF;
                    TIMERS_InitTimer(1, 1500);
                    printf("Beacon Detected! backing off..\n");
                } else {            // if were not on the tape, approach the south bound, then back up + align
                    substate = STATE_ALIGN_FIND_SOUTH_BOUND;
                    printf("Beacon Detected! finding bound..\n");
                }
            }
            break;
        
        case STATE_ALIGN_FIND_SOUTH_BOUND:
            printf("ALIGN_FIND_BOUND: FR: %d, FL: %d\n", tape_fr, tape_fl);
            MecanumDrive(SLOW, 0, 0);
            
            if (tape_fr || tape_fl) { // after hitting tape, back off
                MecanumDrive(-SLOW, 0, 0);
                substate = STATE_ALIGN_BACK_OFF;
                TIMERS_InitTimer(1, 1000);
            }
            break;

        case STATE_ALIGN_BACK_OFF:
            printf("Back up...\n");
            if (timer_expired) {
                substate = STATE_ALIGN_STRAIGHTEN;
            }
            break;

        // approach the bound, sometimes backing up to align to the tape
        // (like how a car aligns to a parking spot)
        case STATE_ALIGN_STRAIGHTEN:
            printf("Align...\n");
            
            if (!timer_expired) {
                break;
            }
            
            if (tape_fr) {
                SetDriveMotor(DRIVE_FRONT_RIGHT, -750);
                SetDriveMotor(DRIVE_REAR_RIGHT,  -750);
                SetDriveMotor(DRIVE_FRONT_LEFT,  -000);
                SetDriveMotor(DRIVE_REAR_LEFT,   -000);
                
                if (tape_fr && tape_fl) {
                    TIMERS_InitTimer(1, 500);
                    substate = STATE_ALIGN_CORNER;
                    break;
                }

                substate = STATE_ALIGN_BACK_OFF;
                TIMERS_InitTimer(1, 300);
                break;
            } else {
                SetDriveMotor(DRIVE_FRONT_RIGHT,  750);
                SetDriveMotor(DRIVE_REAR_RIGHT,   750);
                TIMERS_InitTimer(1, 100);
            }
            
            if (tape_fl) {
                SetDriveMotor(DRIVE_FRONT_LEFT,  -750);
                SetDriveMotor(DRIVE_REAR_LEFT,   -750);
                SetDriveMotor(DRIVE_FRONT_RIGHT, -000);
                SetDriveMotor(DRIVE_REAR_RIGHT,  -000);

                if (tape_fr && tape_fl) {
                    TIMERS_InitTimer(1, 500);
                    substate = STATE_ALIGN_CORNER;
                    break;
                }
                
                substate = STATE_ALIGN_BACK_OFF;
                TIMERS_InitTimer(1, 300);
                break;
            } else {
                SetDriveMotor(DRIVE_FRONT_LEFT,  750);
                SetDriveMotor(DRIVE_REAR_LEFT,   750);
                TIMERS_InitTimer(1, 100);
            }
            break;
        case STATE_ALIGN_CORNER:
            if (!timer_expired) { // wait till delay to prevent jerk
                printf("Move to corner... (wait)\n");
                break;
            }

            printf("Move to corner...\n");
            
            uint8_t next_tripped = tape_fl || tape_fr;
            if (tapes_tripped != next_tripped) {
                corner_last_trip_time = TIMERS_GetTime();
                tapes_tripped = next_tripped;
            }

            int align_speed = (TIMERS_GetTime() - corner_last_trip_time) * CORNER_ALIGN_RATE_PER_MS; 
            int align_speed_capped = align_speed > SLOW ? SLOW : align_speed;
            int align_dir = next_tripped ? -1 : 1;

            MecanumDrive(align_dir * align_speed_capped, 1000, 0);

            if (tape_sl || tape_sr) {
                substate = STATE_ALIGN_180_PT1;
            }

            break;
        case STATE_ALIGN_180_PT1:
            // turn right till front left is white
            printf("ALIGN_180_PT1...\n");
            TankDrive(SLOW, -SLOW);
            if (!tape_fl) {
                substate = STATE_ALIGN_180_PT2;
            }
            break;
        case STATE_ALIGN_180_PT2:
            printf("ALIGN_180_PT2...\n");
            // turn right till front right is black
            TankDrive(SLOW, -SLOW);
            if (tape_fr) {
                return 1;
                // substate = STATE_ALIGN_180_PT2;
            }
            break;
        default:
            break;
    }

    return 0;
}


uint8_t RunStateNav(void) {
    static int substate = STATE_ALIGN_SCAN;
    static uint8_t initialized = 0;

    // Read sensors + state variables
    uint8_t tape_fr = ReadTapeSensorFR() == 0 ? 1 : 0;
    uint8_t tape_fl = ReadTapeSensorFL() == 0 ? 1 : 0;
    uint8_t tape_sr = ReadTapeSensorSR() == 0 ? 1 : 0;
    uint8_t tape_sl = ReadTapeSensorSL() == 0 ? 1 : 0;
    double beacon = ReadBeaconSensor1();

    uint8_t timer_expired = TIMERS_IsTimerExpired(1) == TIMER_EXPIRED;

    if (!initialized) {
        printf("INITIALIZE NAV!");
        initialized = 1;
    }

    TankDrive(
         tape_fl ? 0 : SLOW,        // if LEFT  is BLACK, turn LEFT  drive OFF
        !tape_fr ? 0 : SLOW         // if RIGHT is WHITE, turn RIGHT drive OFF
    );

    return 0;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
 

//#include "BOARD.h"
//#include <xc.h>
//#include <stdio.h>
//#include "pwm.h"
//#include "IO_Ports.h"
//int main(void) {
//    BOARD_Init();
//    PWM_Init();
//    PWM_SetFrequency(2000);
//    PWM_AddPins(PWM_PORTX11);
//    IO_PortsSetPortOutputs(PORTX, PIN3);
//    while (1) {
//        IO_PortsSetPortBits(PORTX, PIN3);
//        PWM_SetDutyCycle(PWM_PORTX11, 1000);
//    }
//}
