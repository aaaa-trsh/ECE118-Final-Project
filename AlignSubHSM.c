/*
* File: TemplateSubHSM.c
* Author: J. Edward Carryer
* Modified: Gabriel H Elkaim
*
* Template file to set up a Heirarchical State Machine to work with the Events and
* Services Framework (ES_Framework) on the Uno32 for the CMPE-118/L class. Note that
* this file will need to be modified to fit your exact needs, and most of the names
* will have to be changed to match your code.
*
* There is for a substate machine. Make sure it has a unique name
*
* This is provided as an example and a good place to start.
*
* History
* When           Who     What/Why
* -------------- ---     --------
* 09/13/13 15:17 ghe      added tattletail functionality and recursive calls
* 01/15/12 11:12 jec      revisions for Gen2 framework
* 11/07/11 11:26 jec      made the queue static
* 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
* 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
*/


/*******************************************************************************
* MODULE #INCLUDE                                                             *
******************************************************************************/

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "BOARD.h"
#include "RobotInterface.h"
#include "IOConstants.h"
#include "AlignSubHSM.h"
#include "TopHSM.h"
#include <stdio.h>

/*******************************************************************************
* MODULE #DEFINES                                                             *
******************************************************************************/

// FIND BEACON parameters
#define FIND_BEACON_THRESH 2
#define FIND_BEACON_SPEED 800

// FIND BOUND state tracking
int8_t bound_dir = -1;
uint8_t hit_bound_tape = 0;
uint16_t find_bound_dur = 1000;
// FIND BOUND parameters
#define FIND_BOUND_SPEED 750
#define FIND_BOUND_DUR_INC 500

// ALIGN TAPES state tracking
uint8_t align_tapes_l_hit = 0;
uint8_t align_tapes_r_hit = 0;
// ALIGN TAPES parameters
#define ALIGN_NUDGE_DUR 500

typedef enum {
    Init,
    FIND_SOUTH,
    FIND_BOUND,
    ALIGN_BOUND,
    FIND_START_TAPE,
} AlignSubHSMState_t;

static const char *StateNames[] = {
	"Init",
    "FIND_SOUTH",
    "FIND_BOUND",
    "ALIGN_BOUND",
    "FIND_START_TAPE",
};



/*******************************************************************************
* PRIVATE FUNCTION PROTOTYPES                                                 *
******************************************************************************/
/* Prototypes for private functions for this machine. They should be functions
  relevant to the behavior of this state machine */

void InitFindBoundTape(void);
void FindStartTape(ES_Event ThisEvent);

/*******************************************************************************
* PRIVATE MODULE VARIABLES                                                            *
******************************************************************************/
/* You will need MyPriority and the state variable; you may need others as well.
* The type of state variable should match that of enum in header file. */

static AlignSubHSMState_t CurrentState = Init; // <- change name to match ENUM
static uint8_t MyPriority;


/*******************************************************************************
* PUBLIC FUNCTIONS                                                            *
******************************************************************************/

/**
* @Function InitTemplateSubHSM(uint8_t Priority)
* @param Priority - internal variable to track which event queue to use
* @return TRUE or FALSE
* @brief This will get called by the framework at the beginning of the code
*        execution. It will post an ES_INIT event to the appropriate event
*        queue, which will be handled inside RunTemplateFSM function. Remember
*        to rename this to something appropriate.
*        Returns TRUE if successful, FALSE otherwise
* @author J. Edward Carryer, 2011.10.23 19:25 */
uint8_t InitAlignSubHSM(void)
{
    ES_Event returnEvent;

    CurrentState = Init;
    returnEvent = RunAlignSubHSM(INIT_EVENT);
    if (returnEvent.EventType == ES_NO_EVENT) {
        return TRUE;
    }
    return FALSE;
}

/**
* @Function RunTemplateSubHSM(ES_Event ThisEvent)
* @param ThisEvent - the event (type and param) to be responded.
* @return Event - return event (type and param), in general should be ES_NO_EVENT
* @brief This function is where you implement the whole of the heirarchical state
*        machine, as this is called any time a new event is passed to the event
*        queue. This function will be called recursively to implement the correct
*        order for a state transition to be: exit current state -> enter next state
*        using the ES_EXIT and ES_ENTRY events.
* @note Remember to rename to something appropriate.
*       The lower level state machines are run first, to see if the event is dealt
*       with there rather than at the current level. ES_EXIT and ES_ENTRY events are
*       not consumed as these need to pass pack to the higher level state machine.
* @author J. Edward Carryer, 2011.10.23 19:25
* @author Gabriel H Elkaim, 2011.10.23 19:25 */
ES_Event RunAlignSubHSM(ES_Event ThisEvent)
{
    uint8_t makeTransition = FALSE; // use to flag transition
    AlignSubHSMState_t nextState; // <- change type to correct enum

//    ES_Tattle(); // trace call stack
//    printf(".\n");
//    printf("Find Beacon.. B1=%5f, B2=%5f\n", ReadBeaconSensor1(), ReadBeaconSensor2());

    switch (CurrentState) {
    case Init: // If current state is initial Psedudo State
        if (ThisEvent.EventType == ES_INIT)// only respond to ES_Init
        {
            // this is where you would put any actions associated with the
            // transition from the initial pseudo-state into the actual
            // initial state

            // now put the machine into the actual initial state
            nextState = FIND_SOUTH;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            printf("ALIGN INIT\n");
        }
        break;

    case FIND_SOUTH: // spin robot until beacon found
        if (ReadBeaconSensor1() > FIND_BEACON_THRESH) {
            MecanumDrive(0, 0, 0);
            nextState = FIND_BOUND;
            makeTransition = TRUE;
            ThisEvent.EventType = ES_NO_EVENT;
            InitFindBoundTape();
            printf("Beacon Detected!\n");
        } else {
            MecanumDrive(0, 0, -FIND_BEACON_SPEED);
        }
        break;

    case FIND_BOUND: //move back and forth for increasingly long amounts of time to find boundary line
//        MecanumDrive(0, 0, 0);
        MecanumDrive(bound_dir * FIND_BOUND_SPEED, 0, 0);
        printf("FIND_BOUND");
        switch (ThisEvent.EventType) {
            case TAPE_ENTER_FR:
            case TAPE_ENTER_FL:
                MecanumDrive(0, 0, 0);
                nextState = ALIGN_BOUND;
                makeTransition = TRUE;
                ThisEvent.EventType = ES_NO_EVENT;
                ES_Timer_InitTimer(1, ALIGN_NUDGE_DUR);
                MecanumDrive(-bound_dir * FIND_BOUND_SPEED, 0, 0);
                break;
            case ES_TIMEOUT:
                bound_dir *= -1;
                find_bound_dur += FIND_BOUND_DUR_INC;
                ES_Timer_InitTimer(1, find_bound_dur);
            default: // all unhandled events pass the event back up to the next level
                break;  
        }
        break;
    case ALIGN_BOUND: // align front sensors with out of bounds line 
//        // first, the robot steps back, then approaches the tape
//        // if a tape sensor is tripped, its corresponding side stops moving
//        MecanumDrive(0, 0, 0);

        switch (ThisEvent.EventType) {
            case ES_TIMEOUT:
                MecanumDrive(0, 0, 0);
                SetDriveMotor(DRIVE_FRONT_RIGHT, bound_dir * FIND_BOUND_SPEED);
                SetDriveMotor(DRIVE_REAR_RIGHT,  bound_dir * FIND_BOUND_SPEED);
                SetDriveMotor(DRIVE_FRONT_LEFT,  bound_dir * FIND_BOUND_SPEED);
                SetDriveMotor(DRIVE_REAR_LEFT,   bound_dir * FIND_BOUND_SPEED);
            case TAPE_ENTER_FL:
                align_tapes_l_hit = 1;
                SetDriveMotor(DRIVE_FRONT_LEFT,  0);
                SetDriveMotor(DRIVE_REAR_LEFT,   0);
                
                if (align_tapes_l_hit && align_tapes_r_hit) {
                    nextState = FIND_START_TAPE;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            case TAPE_ENTER_FR:
                align_tapes_r_hit = 1;
                SetDriveMotor(DRIVE_FRONT_RIGHT, 0);
                SetDriveMotor(DRIVE_REAR_RIGHT,  0);
                
                if (align_tapes_l_hit && align_tapes_r_hit) {
                    nextState = FIND_START_TAPE;
                    makeTransition = TRUE;
                    ThisEvent.EventType = ES_NO_EVENT;
                }
                break;
            default: // all unhandled events pass the event back up to the next level
                break;
        }
        break;
        case FIND_START_TAPE:
            MecanumDrive(0, 0, 0);
            printf("Made it !!\n");
            break;
        //    
    default: // all unhandled states fall into here
        break;
    } // end switch on Current State

    if (makeTransition == TRUE) { // making a state transition, send EXIT and ENTRY
        // recursively call the current state with an exit event
        RunAlignSubHSM(EXIT_EVENT); // <- rename to your own Run function
        CurrentState = nextState;
        RunAlignSubHSM(ENTRY_EVENT); // <- rename to your own Run function
    }

//    ES_Tail(); // trace call stack end
    return ThisEvent;
}


/*******************************************************************************
* PRIVATE FUNCTIONS                                                           *
******************************************************************************/
void InitFindBoundTape(void) {
    ES_Timer_InitTimer(1, find_bound_dur / 2);
    hit_bound_tape = 0;
    bound_dir = -1;
}
void FindStartTape(ES_Event ThisEvent) {
    ThisEvent.EventType = START_TAPE_FOUND;
}
