#include "BOARD.h"
#include <xc.h>
#include <stdio.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "pwm.h"
#include "AD.h"
#include "timers.h"

#include "RobotInterface.h"

uint8_t RunStateAlign(void);
uint8_t RunStateNav(void);
uint8_t RunStateShoot(void);
void UpdateDebouncedValues(void);
int GetTailExitEvent(int tape_r);
void UpdateBeaconFilter(void);

#define STATE_ALIGN     0
#define STATE_NAV_ISZ   1
#define STATE_SHOOT     2

#define MEDIUM      1000
#define SLOW        970
#define SUPASLOW    700
#define CORNER_ALIGN_RATE_PER_MS 0.6 // counts per millisecond

#define TURN_180_DUR 1500

#define DO_STATE_MACHINE

static uint8_t tape_fr = 0;
static uint8_t tape_fl = 0;
static uint8_t tape_sr = 0;
static uint8_t tape_sl = 0;
static uint8_t tape_r = 0;
static uint8_t debounce_timer_expired = 0;
static uint8_t obs1_detected = 0;
static uint8_t obs2_detected = 0;
static double beacon = 0;

void main(void)
{
    
//    ES_Return_t ErrorType;
    
    BOARD_Init();
    AD_Init();
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
    
    TIMERS_InitTimer(10, 50); // start debounce timer
#ifdef DO_STATE_MACHINE
    static int state = STATE_NAV_ISZ;
    uint8_t fin = 0;
    while (1) {
        debounce_timer_expired = TIMERS_IsTimerExpired(10) == TIMER_EXPIRED;
        switch (state) {
            case STATE_ALIGN:
                fin = RunStateAlign();
                if (fin) { state = STATE_NAV_ISZ; }
                break;
             case STATE_NAV_ISZ:
                fin = RunStateNav();
                if (fin) { state = STATE_SHOOT; }
            case STATE_SHOOT:
                fin = RunStateShoot();
                if (fin) { state = -1; }
            default:
                break;
        }
        
        UpdateDebouncedValues();
        UpdateBeaconFilter();
    }
#endif
    while (1) {
        debounce_timer_expired = TIMERS_IsTimerExpired(10) == TIMER_EXPIRED;
        printf(
            "Beacon: %2f, FR: %d, FL: %d, SR: %d, SL: %d, R: %d\n", 
            beacon,
            tape_fr,
            tape_fl,
            tape_sr,
            tape_sl,
            tape_r
        );
        
        UpdateDebouncedValues();
        UpdateBeaconFilter();
    }
}

void UpdateDebouncedValues(void) {
    if (debounce_timer_expired) {
        tape_fr = ReadTapeSensorFR() == 0 ? 1 : 0; // is the front right black?
        tape_fl = ReadTapeSensorFL() == 0 ? 1 : 0;
        tape_sr = ReadTapeSensorSR() == 0 ? 1 : 0;
        tape_sl = ReadTapeSensorSL() == 0 ? 1 : 0;
        tape_r = ReadTapeSensorR() == 0 ? 1 : 0;
        obs1_detected = ReadObstacleSensor1();
        obs2_detected = ReadObstacleSensor2();
        TIMERS_InitTimer(10, 50);
    }
}

void UpdateBeaconFilter(void) {
    static double buf[5] = { 0, 0, 0, 0, 0 };
    static int b_i = 0;
    
    double reading = ReadBeaconSensor1();
    buf[b_i] = reading;
    b_i = (b_i + 1) % 5;
    
    beacon = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) / 5;
}

#define STATE_ALIGN_HOME                0
#define STATE_ALIGN_FIND_SOUTH          1
#define STATE_ALIGN_FIND_SOUTH_BOUND    2
#define STATE_ALIGN_BACK_OFF            3
#define STATE_ALIGN_FIND_CORNER         4
#define STATE_ALIGN_MOVE_TO_CORNER      5
#define STATE_ALIGN_CORNER_ROT          6
#define STATE_ALIGN_CORNER_PAN          7

uint8_t RunStateAlign(void) {
    // Initialize state mgmt variables
    static int substate = STATE_ALIGN_HOME;
    static uint8_t scanned_beacon = 0;
    static uint8_t scanned_tape = 0;
    static uint8_t initialized = 0;

    static uint8_t corner_last_trip_time = 0;
    static uint8_t tapes_tripped = 0;

    uint8_t timer_expired = TIMERS_IsTimerExpired(1) == TIMER_EXPIRED;

    if (!initialized) {
        if (beacon > 2) { // if we are facing the beacon, do a spin
            TIMERS_InitTimer(1, TURN_180_DUR);
            substate = STATE_ALIGN_HOME;
        } else { // if we arent facing the beacon, skip to seeking the beacon
            substate = STATE_ALIGN_FIND_SOUTH;
            TIMERS_InitTimer(1, 1000);
        }
        initialized = 1;
    }
    
    // 1. turn till beacon
    // 2. turn 90 deg
    // 3. sweep to left
    switch (substate) {
        case STATE_ALIGN_HOME:
            MecanumDrive(0, 0, MEDIUM);

            if (timer_expired) { // once we do spin, seek da beacon
                substate = STATE_ALIGN_FIND_SOUTH;
                TIMERS_InitTimer(1, 1000);
            }
            break;
        
        // then turn to south
        case STATE_ALIGN_FIND_SOUTH:
            printf("ALIGN_FIND_SOUTH: beacon=%5f\n", beacon);
            MecanumDrive(0, 0, 800);

            if (timer_expired && beacon > 2.4) {
                DriveBackward(SLOW);
                substate = STATE_ALIGN_BACK_OFF;
                TIMERS_InitTimer(1, 1000);
                printf("Beacon Detected! backing off..\n");
            }
            break;

        case STATE_ALIGN_BACK_OFF:
            printf("Back up...\n");
            if (timer_expired) {
                substate = STATE_ALIGN_FIND_CORNER;
                TIMERS_InitTimer(1, 700);
            }
            break;

        // approach the bound, sometimes backing up to align to the tape
        // (like how a car aligns to a parking spot)
        case STATE_ALIGN_FIND_CORNER:
            /*
            Comment away!
            advance forward straight
            */
            DriveForward(SLOW);
            printf("Seek towards bound line!\n");
            
            if (timer_expired && (tape_fl || tape_fr)) {
//                substate = STATE_ALIGN_MOVE_TO_CORNER;
//                TIMERS_InitTimer(1, 20);
                substate = STATE_ALIGN_CORNER_ROT;
                TIMERS_InitTimer(1, TURN_180_DUR * 0.9);
            }

            /*
                if frlont left is triggered
                    back up, turn discrete angle right
                    try again - go forwards
                if right hit, begin first follow of the tape
                    if the turns are calibrated correctly, the tape will be followed even at the 90 degree turn, so this behavior
                    should be continued until a obstacle is detected
            */
            break;
            
         case STATE_ALIGN_CORNER_ROT: // pan left to re follow tape
            MecanumDrive(0, 0, -MEDIUM);
            if (timer_expired) {
                DriveRight(SLOW);
                substate = STATE_ALIGN_CORNER_PAN;
                TIMERS_InitTimer(1, 500);
            }
            break;
        case STATE_ALIGN_CORNER_PAN:
            DriveRight(MEDIUM);
            if (timer_expired && tape_sr) {
                MecanumDrive(0, 0, 0);
                return 1;
            }
            break;
        default:
            break;
    }
                
    return 0;
}


int GetTailExitEvent(int tape_r) {
    static unsigned int tail_in_tape = 0;
    static unsigned int tail_enter_time = 0;

    static unsigned int was_tail_in_tape = 0;
    static unsigned int tail_exited_tape = 0;
    static unsigned int prev_r = 0;
    
    if (prev_r != tape_r && tape_r) {
        tail_enter_time = TIMERS_GetTime();
    } else if (!tape_r) {
        tail_in_tape = 0;
    }

    prev_r = tape_r;

    if (TIMERS_GetTime() - tail_enter_time > 1000 && !tail_in_tape && tape_r) {
        tail_in_tape = 1;
    }

    if (tail_exited_tape) { tail_exited_tape = 0; }
    if (was_tail_in_tape != tail_in_tape && !tail_in_tape) { tail_exited_tape = 1; }

    was_tail_in_tape = tail_in_tape;
    return tail_exited_tape;
}

#define STATE_NAV_LINE_RIGHT        0
#define STATE_NAV_LINE_LEFT         1
#define STATE_NAV_JUMP_RIGHT        2
#define STATE_NAV_JUMP_LEFT         3
#define STATE_NAV_DELAY             4
#define STATE_NAV_FIN               5

#define JUMP_ACCEL 1
#define MAX_JUMP_SPEED MEDIUM

uint8_t RunStateNav(void) {
    static int substate = STATE_NAV_LINE_RIGHT;
    static uint8_t initialized = 0;
    static unsigned int accel_start_time = 0;
    
    static uint8_t tape_r_prev = 0;
    static int queued_state = 0;

    uint8_t timer_expired = TIMERS_IsTimerExpired(1) == TIMER_EXPIRED;

    if (!initialized) {
        TIMERS_InitTimer(1, 1000); // such that the right path can see corners given no obstacles detected
        printf("INITIALIZE NAV!");
        initialized = 1;
    }
    
    int js = JUMP_ACCEL * (TIMERS_GetTime() - accel_start_time);
    js = (js > MAX_JUMP_SPEED ? MAX_JUMP_SPEED : js);
    int jump_speed = js <= 800 ? 800 : js;
    
    // tail tape events
//    int tail_exited_tape = GetTailExitEvent(tape_r);
    
    switch (substate) {
        case STATE_NAV_LINE_RIGHT:
            printf("LINE_RIGHT\n");
            TankDrive(
                !tape_fl ? SLOW : 0,         // if LEFT  is BLACK, turn LEFT  drive OFF
                tape_fr ? SLOW : 0         // if RIGHT is WHITE, turn RIGHT drive OFF
            );
            
            if (timer_expired && tape_sl) { 
                MecanumDrive(0, 0, 0);
                substate = STATE_NAV_FIN;
                TIMERS_InitTimer(1, 4000);
                break;
            }
            
            if (obs1_detected || obs2_detected) {
                printf("Hit object. Jumping left..\n");
                substate = STATE_NAV_DELAY;
                TIMERS_InitTimer(1, 300);
                MecanumDrive(0, 0, SLOW * 0.8);
                queued_state = STATE_NAV_JUMP_LEFT;
                break;
            }
            
            tape_r_prev = tape_r;
            break;
        case STATE_NAV_LINE_LEFT:
            printf("LINE_LEFT\n");            
            TankDrive(
                tape_fl ? SLOW : 0,         // if LEFT  is WHITE, turn LEFT  drive OFF
                !tape_fr ? SLOW : 0           // if RIGHT is BLACK, turn RIGHT drive OFF
            );
            
            if (timer_expired && tape_sr) {
                MecanumDrive(0, 0, 0);
                substate = STATE_NAV_FIN;
                TIMERS_InitTimer(1, 4000);
                break;
            }
            
            if (obs1_detected || obs2_detected) {
                printf("Hit object. Jumping right..\n");
                substate = STATE_NAV_DELAY;
                TIMERS_InitTimer(1, 300);
                MecanumDrive(0, 0, -SLOW * 0.8);
                queued_state = STATE_NAV_JUMP_RIGHT;
                break;
            }
            
            tape_r_prev = tape_r;
            break;
        case STATE_NAV_JUMP_RIGHT:
            DriveRight(timer_expired ? SLOW : MEDIUM);
            printf("Jumping right: %d\n", jump_speed);
            if (timer_expired && (tape_fr || tape_r)) {
                queued_state = STATE_NAV_LINE_RIGHT;
                substate = STATE_NAV_DELAY;
                MecanumDrive(0, 0, 0);
                TIMERS_InitTimer(1, 300);
            }
            break;
        case STATE_NAV_JUMP_LEFT:
            DriveLeft(timer_expired ? SLOW : MEDIUM);
            printf("Jumping left: %d\n", jump_speed);
            if (timer_expired && (tape_fl || tape_r)) {
                queued_state = STATE_NAV_LINE_LEFT;
                substate = STATE_NAV_DELAY;
                MecanumDrive(0, 0, 0);
                TIMERS_InitTimer(1, 300);
            }
            break;
        case STATE_NAV_DELAY: // after waiting, set up the "pass middle line" timer
            if (timer_expired) {
                printf("Running queued state %d\n", queued_state);
                substate = queued_state;

                if (
                    queued_state == STATE_NAV_JUMP_LEFT ||
                    queued_state == STATE_NAV_JUMP_RIGHT
                ) {
                    accel_start_time = TIMERS_GetTime() - 300;
                    TIMERS_InitTimer(1, queued_state == STATE_NAV_JUMP_LEFT ? 3500 : 4000);
                } else {
                    TIMERS_InitTimer(1, 1000);
                }
            }
            break;
        case STATE_NAV_FIN:
            if (timer_expired) {
                printf("FIN !!!\n");
                MecanumDrive(0, 0, 0);
                return 1;
            }
            break;
        default:
            break;
    }

    tape_r_prev = tape_r;

    return 0;
}

#define STATE_SHOOT_SEEK        0
#define STATE_SHOOT_SPRAY       1

uint8_t RunStateShoot(void) {
    static int substate = STATE_SHOOT_SEEK;
    static int direction = 1;
    static int indexer_on = 0;
    static int initialized = 0;
    static double max_thresh = 1.5;
    static int seek_speed = 800;
    static int seek_steps = 0;
    

    if (!initialized) {
        TIMERS_InitTimer(7, 5000);
        printf("set shooter on!");
        initialized = 1;
        TIMERS_InitTimer(4, 50);
        TIMERS_InitTimer(5, 4000); // seek for atleast 4s
    }
    
//    printf("beacon=%2f thresh=%2f\n", beacon, max_thresh);
    switch (substate) {
        case STATE_SHOOT_SEEK:
            MecanumDrive(0, 0, seek_speed);
            if (beacon > max_thresh && TIMERS_IsTimerExpired(4) == TIMER_EXPIRED) {
                printf("Seek step... reached %2f... speed %d\n", max_thresh, seek_speed);
                max_thresh += 0.05;
                seek_speed = 800 - (300 * ((max_thresh - 1.5) / .6));
                
                // far scalefactor   = 0.6 ----- max measurement = 2.15
                // mid scalefactor   = 1.2 ----- max measurement = 2.25
                // close scalefactor = 1.5 ----- max measurement = 2.85
                TIMERS_InitTimer(4, 200);
                
                if (TIMERS_IsTimerExpired(5) == TIMER_EXPIRED) {
                    printf("Resetting timer\n");
                    TIMERS_InitTimer(6, 2000); // wait for stillness for 2s, then transition
                }
            }
            
            if (TIMERS_IsTimerExpired(5) == TIMER_EXPIRED && TIMERS_IsTimerExpired(6) == TIMER_EXPIRED) {
                printf("TRANSITION\n");
                substate = STATE_SHOOT_SPRAY;
                TankDrive(0, 0);
                SetShooter(1, 0);
                TIMERS_InitTimer(1, 5000); // wait for 5s before indexing
                TIMERS_InitTimer(3, 400); // rambo 1s
                TIMERS_InitTimer(2, 20);
            }

            break;
        case STATE_SHOOT_SPRAY:
//            printf("Seek spray dir=%d\n", direction);
             MecanumDrive(0, 0, 800 * direction);
             if (TIMERS_IsTimerExpired(3) == TIMER_EXPIRED) {
//                direction *= -1;
                 MecanumDrive(0, 0, 0);
//                TIMERS_InitTimer(3, 1000);
             }

            // turn indexer on only after 5 seconds elapsed since entering shooting
            if (TIMERS_IsTimerExpired(7) == TIMER_EXPIRED) {
//                if (TIMERS_IsTimerExpired(2) == TIMER_EXPIRED) {
//                    indexer_on = !indexer_on;
//                    TIMERS_InitTimer(2, 20);
//                }
                SetIndexer(1);
            }

            break;
        default:
            break;
    }
    
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
