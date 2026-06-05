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
uint8_t SeekBeacon(int initialize, double target_thresh);

#define STATE_ALIGN     0
#define STATE_NAV_ISZ   1
#define STATE_SHOOT     2

#define MEDIUM      1000
#define SLOW        970
#define SUPASLOW    700
#define CORNER_ALIGN_RATE_PER_MS 0.6 // counts per millisecond

#define TURN_180_DUR 1700

#define DO_STATE_MACHINE

#define INITIAL_THRESH          1.5

// !!!!!!!!!!! BEACON DEFINES !!!!!!!!!!!
#define TARGET_THRESH           3.6
#define WIDE_ALIGN_THRESH       2.1
#define NARROW_ALIGN_THRESH     2.4
#define FILTER_SIZE             40

static uint8_t tape_fr = 0;
static uint8_t tape_fl = 0;
static uint8_t tape_sr = 0;
static uint8_t tape_sl = 0;
static uint8_t prev_tape_sr = 0;
static uint8_t prev_tape_sl = 0;
static uint8_t tape_r = 0;
static uint8_t debounce_timer_expired = 0;
static uint8_t obs1_detected = 0;
static uint8_t obs2_detected = 0;
static double aim_beacon = 0;
static double shoot_beacon = 0;

void main(void) {
    BOARD_Init();
    AD_Init();
    InitRobot();
    TIMERS_Init();
    
    printf("Starting Team 5 Robot!!\r\n");
    printf("using the 2nd Generation Events & Services Framework\r\n");
    
    TIMERS_InitTimer(10, 50); // start debounce timer
    
#ifdef DO_STATE_MACHINE
    static int state = STATE_ALIGN;
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
                break;
            case STATE_SHOOT:
                fin = RunStateShoot();
                if (fin) { state = -1; }
                break;
            default:
                break;
        }
        
        UpdateDebouncedValues();
        UpdateBeaconFilter();
    }
#endif
    double max_beacon = INITIAL_THRESH;
    int seeking = 0;
    while (1) {
        debounce_timer_expired = TIMERS_IsTimerExpired(10) == TIMER_EXPIRED;
        uint8_t hold_timer_expired = TIMERS_IsTimerExpired(2) == TIMER_EXPIRED;
        
        if (debounce_timer_expired) {
            printf(
                "aim=%2f, sht=%2f/max=%2f, FR: %d, FL: %d, SR: %d, SL: %d, R: %d\n", 
                aim_beacon,
                shoot_beacon,
                max_beacon,
                tape_fr,
                tape_fl,
                tape_sr,
                tape_sl,
                tape_r
            );
        }
        
        if (shoot_beacon > max_beacon) {
            max_beacon += 0.05;
        }
        
        UpdateDebouncedValues();
        UpdateBeaconFilter();
    }
}

void UpdateDebouncedValues(void) {
    if (debounce_timer_expired) {
        uint8_t prev_sr = tape_sr;
        uint8_t prev_sl = tape_sl;
        
        tape_fr = ReadTapeSensorFR() == 0 ? 1 : 0; // is the front right black?
        tape_fl = ReadTapeSensorFL() == 0 ? 1 : 0;
        tape_sr = ReadTapeSensorSR() == 0 ? 1 : 0;
        tape_sl = ReadTapeSensorSL() == 0 ? 1 : 0;
        tape_r = ReadTapeSensorR() == 0 ? 1 : 0;
        obs1_detected = ReadObstacleSensor1();
        obs2_detected = ReadObstacleSensor2();
        
        prev_tape_sr = prev_sr;
        prev_tape_sl = prev_sl;
        
        TIMERS_InitTimer(10, 50);
    }
}

static uint8_t filter_init = 0;
void UpdateBeaconFilter(void) {
    static double aim_buf[FILTER_SIZE];
    static double sht_buf[FILTER_SIZE];
    static int b_i = 0;
    
    if (!filter_init) {
        for (int i = 0; i < FILTER_SIZE; i++) {
            aim_buf[i] = 0;
            sht_buf[i] = 0;
        }
        filter_init = 1;
    }
    
    aim_buf[b_i] = ReadBeaconSensor1();
    sht_buf[b_i] = ReadBeaconSensor2();

    b_i = (b_i + 1) % FILTER_SIZE;
    
    double aim_avg = 0, sht_avg = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        aim_avg += aim_buf[i];
        sht_avg += sht_buf[i];
    }
    
    aim_beacon = aim_avg / FILTER_SIZE;
    shoot_beacon = sht_avg / FILTER_SIZE;
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
    static uint8_t align_initialized = 0;

    uint8_t timer_expired = TIMERS_IsTimerExpired(1) == TIMER_EXPIRED;

    if (!align_initialized) {
        if (aim_beacon > WIDE_ALIGN_THRESH) { // if we are facing the beacon, do a spin
            TIMERS_InitTimer(1, TURN_180_DUR);
            substate = STATE_ALIGN_HOME;
        } else { // if we arent facing the beacon, skip to seeking the beacon
            substate = STATE_ALIGN_FIND_SOUTH;
            TIMERS_InitTimer(1, 1000);
        }
        align_initialized = 1;
    }
    
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
            printf("ALIGN_FIND_SOUTH: beacon=%5f\n", aim_beacon);
            MecanumDrive(0, 0, 800);

            if (timer_expired && aim_beacon > NARROW_ALIGN_THRESH) {
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

        case STATE_ALIGN_FIND_CORNER:
            DriveForward(SLOW);
            printf("Seek towards bound line!\n");
            
            if (timer_expired && (tape_fl || tape_fr)) {
                substate = STATE_ALIGN_CORNER_ROT;
                TIMERS_InitTimer(1, TURN_180_DUR * 0.9);
            }
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
    static uint8_t nav_initialized = 0;
    static unsigned int accel_start_time = 0;
    
    static int queued_state = 0;
    static uint8_t tape_on_left = 0;

    uint8_t timer_expired = TIMERS_IsTimerExpired(1) == TIMER_EXPIRED;
    
    if (!nav_initialized) {
        TIMERS_InitTimer(1, 1000); // such that the right path can see corners given no obstacles detected
        printf("INITIALIZE NAV!");
        nav_initialized = 1;
    }
    
    int js = JUMP_ACCEL * (TIMERS_GetTime() - accel_start_time);
    js = (js > MAX_JUMP_SPEED ? MAX_JUMP_SPEED : js);
    int jump_speed = js <= 800 ? 800 : js;
    
    // tail tape events
    int tail_exited_tape = GetTailExitEvent(tape_r);
    
    switch (substate) {
        case STATE_NAV_LINE_RIGHT:
            tape_on_left = 0;
            printf("LINE_RIGHT\n");
            TankDrive(
                !tape_fl ? SLOW : -670,         // if LEFT  is BLACK, turn LEFT  drive OFF
                tape_fr ? SLOW : -670         // if RIGHT is WHITE, turn RIGHT drive OFF
            );
            
            if (timer_expired && (tape_sl && tape_sl == prev_tape_sl)) { 
                TankDrive(-SLOW, -SLOW);
                substate = STATE_NAV_DELAY;
                queued_state = STATE_NAV_FIN;
                TIMERS_InitTimer(1, 1000);
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
            
            break;
        case STATE_NAV_LINE_LEFT:
            tape_on_left = 1;
            printf("LINE_LEFT\n");            
            TankDrive(
                tape_fl ? SLOW : -670,         // if LEFT  is WHITE, turn LEFT  drive OFF
                !tape_fr ? SLOW : -670           // if RIGHT is BLACK, turn RIGHT drive OFF
            );
            
            if (timer_expired && (tape_sr && tape_sr == prev_tape_sr)) {
                TankDrive(-SLOW, -SLOW);
                substate = STATE_NAV_DELAY;
                queued_state = STATE_NAV_FIN;
                TIMERS_InitTimer(1, 1000);
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
            
            break;
        case STATE_NAV_JUMP_RIGHT:
            DriveRight(timer_expired ? SLOW * 1.1 : MEDIUM);
            printf("Jumping right: %d\n", jump_speed);
            if (timer_expired && (tape_fr || tape_r)) {
                queued_state = STATE_NAV_LINE_RIGHT;
                substate = STATE_NAV_DELAY;
                MecanumDrive(0, 0, 0);
                TIMERS_InitTimer(1, 300);
            }
            break;
        case STATE_NAV_JUMP_LEFT:
            DriveLeft(timer_expired ? SLOW * 1.1 : MEDIUM);
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
                } else if (queued_state == STATE_NAV_FIN) {
                    TIMERS_InitTimer(1, 2000);
                } else {
                    TIMERS_InitTimer(1, 1000);
                }
            }
            break;
        case STATE_NAV_FIN:
            if (!timer_expired && tape_on_left) {
                DriveRight(MEDIUM);
            } else {
                DriveLeft(MEDIUM);
            }
            
            if (timer_expired) {
                printf("FIN !!!\n");
                MecanumDrive(0, 0, 0);
                return 1;
            }
            break;
        default:
            break;
    }

    return 0;
}

#define STATE_SHOOT_SEEK        0
#define STATE_SHOOT_SPRAY       1

uint8_t SeekBeacon(int initialize, double target_thresh) {
    return 0;
}

uint8_t RunStateShoot(void) {
    static int substate = STATE_SHOOT_SEEK;
    static int shoot_initialized = 0;
    static int seek_speed = 900;
    static double max_thresh = INITIAL_THRESH;
    
    double target_thresh = TARGET_THRESH;
    
    uint8_t ignore_timer_done = TIMERS_IsTimerExpired(5) == TIMER_EXPIRED;
    uint8_t still_timer_done = TIMERS_IsTimerExpired(6) == TIMER_EXPIRED;

    if (!shoot_initialized) {
        TIMERS_InitTimer(7, 5000);
        printf("set shooter on!");
        shoot_initialized = 1;
        SeekBeacon(1, TARGET_THRESH);
        seek_speed = 800;
        max_thresh = INITIAL_THRESH;
        TIMERS_InitTimer(4, 20); // update timer
        TIMERS_InitTimer(5, 1500); // seek for atleast 1.5s
        TIMERS_InitTimer(6, 20); // direction change timer
    }
    
    uint8_t do_update = TIMERS_IsTimerExpired(4) == TIMER_EXPIRED;

    switch (substate) {
        case STATE_SHOOT_SEEK:
            printf("Seeking b=%.2f max=%.2f spd=%d\n", shoot_beacon, max_thresh, seek_speed);
            MecanumDrive(0, 0, seek_speed);
            if (shoot_beacon > max_thresh && do_update) {
                max_thresh += 0.2;
                seek_speed = 900 - (300 * ((shoot_beacon - INITIAL_THRESH) / (target_thresh - INITIAL_THRESH)));

                TIMERS_InitTimer(4, 20);
                
                if (ignore_timer_done && max_thresh > INITIAL_THRESH) {
                    printf("Resetting timer\n");
                    TIMERS_InitTimer(6, 1000); // wait for stillness for 2s, then transition
                }
            }
            
            if (ignore_timer_done && still_timer_done && shoot_beacon > INITIAL_THRESH) {
                printf("TRANSITION\n");
                substate = STATE_SHOOT_SPRAY;
                TankDrive(0, 0);
                SetShooter(1, 0);
                TIMERS_InitTimer(7, 4000); // wait for 5s before indexing
                TIMERS_InitTimer(3, 300); // rambo 1s
                TIMERS_InitTimer(2, 20);
            }

            break;
        case STATE_SHOOT_SPRAY:
//            printf("Seek spray dir=%d\n", direction);
             MecanumDrive(0, 0, 800);
             if (TIMERS_IsTimerExpired(3) == TIMER_EXPIRED) {
                 MecanumDrive(0, 0, 0);
             }

            // turn indexer on only after 5 seconds elapsed since entering shooting
            if (TIMERS_IsTimerExpired(7) == TIMER_EXPIRED) {
//                if (TIMERS_IsTimerExpired(2) == TIMER_EXPIRED) {
//                    indexer_on = !indexer_on;
//                    TIMERS_InitTimer(2, 20);
//                }
                printf("INDEX");
                SetIndexer(1);
            }

            break;
        default:
            break;
    }
    
    return 0;
}