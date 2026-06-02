#include "ES_Configure.h"
#include "ObstacleEventChecker.h"
#include "ES_Events.h"
#include "serial.h"
#include "AD.h"
#include "IO_Ports.h"
#include "roach.h"
#include "RobotInterface.h"
#include "TopHSM.h"
#include <xc.h>
#include <stdio.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/
//#define TRACK_VERBOSE
/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/
/* Prototypes for private functions for this EventChecker. They should be functions
   relevant to the behavior of this particular event checker */

/*******************************************************************************
 * PRIVATE MODULE VARIABLES                                                    *
 ******************************************************************************/

/* Any private module level variable that you might need for keeping track of
   events would be placed here. Private variables should be STATIC so that they
   are limited in scope to this module. */

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                            *
 ******************************************************************************/

/**
 * @Function CheckWireEvents(void)
 * @param none
 * @return TRUE or FALSE
 * @brief This function is a prototype event checker that checks the track wire sensors
 *        The function will post either TW_1_DETECTED or TW_2_DETECTED.
 *        Returns TRUE if there was an event, FALSE otherwise.
 */
;uint8_t CheckObstacleEvents(void) {
    // Initialize track wire ports
    static uint8_t obs1_ref = 0;
    static uint8_t obs2_ref = 0;
    static uint8_t fl_ref = 0;
    static uint8_t fr_ref = 0;
    static uint8_t sl_ref = 0;
    static uint8_t sr_ref = 0;
    static uint8_t r_ref = 0;
    
    uint8_t obs1 = ReadObstacleSensor1();
    uint8_t obs2 = ReadObstacleSensor2();
    uint8_t fl = ReadTapeSensorFL();
    uint8_t fr = ReadTapeSensorFR();
    uint8_t sl = ReadTapeSensorSL();
    uint8_t sr = ReadTapeSensorSR();
    uint8_t r  = ReadTapeSensorR();
    
    uint8_t returnVal = FALSE;

    ES_Event obs1Event;
    if (obs1 != obs1_ref && obs1) {
        obs1Event.EventType = OBS_1_DETECTED;        
#ifdef TRACK_VERBOSE
        printf("OBSTACLE 1 TRIPPED\n");
#endif
        PostTopHSM(obs1Event);
        returnVal = TRUE;
    } else if (obs1 != obs1_ref && !obs1) {
        obs1Event.EventType = OBS_1_UNDETECTED;
#ifdef TRACK_VERBOSE
        printf("OBSTACLE 1 UNTRIPPED\n");
#endif
        PostTopHSM(obs1Event);
        returnVal = TRUE;
    }
    
    ES_Event obs2Event;
    if (obs2 != obs2_ref && obs2) {
        obs2Event.EventType = OBS_2_DETECTED;        
#ifdef TRACK_VERBOSE
        printf("OBSTACLE 2 TRIPPED\n");
#endif
        PostTopHSM(obs2Event);
        returnVal = TRUE;
    } else if (obs2 != obs2_ref && !obs2) {
        obs2Event.EventType = OBS_2_UNDETECTED;
#ifdef TRACK_VERBOSE
        printf("OBSTACLE 2 UNTRIPPED\n");
#endif
        PostTopHSM(obs2Event);
        returnVal = TRUE;
    }
    
    ES_Event flEvent;
    if (fl != fl_ref && fl) {
        flEvent.EventType = TAPE_ENTER_FL;
#ifdef TRACK_VERBOSE
        printf("FL TRIPPED\n");
#endif
        PostTopHSM(flEvent);
        returnVal = TRUE;
    } else if (fl != fl_ref && !fl) {
        flEvent.EventType = TAPE_EXIT_FL;
#ifdef TRACK_VERBOSE
        printf("FL UNTRIPPED\n");
#endif
        PostTopHSM(flEvent);
        returnVal = TRUE;
    }

    ES_Event frEvent;
    if (fr != fr_ref && fr) {
        frEvent.EventType = TAPE_ENTER_FR;
#ifdef TRACK_VERBOSE
        printf("FR TRIPPED\n");
#endif
        PostTopHSM(frEvent);
        returnVal = TRUE;
    } else if (fr != fr_ref && !fr) {
        frEvent.EventType = TAPE_EXIT_FR;
#ifdef TRACK_VERBOSE
        printf("FR UNTRIPPED\n");
#endif
        PostTopHSM(frEvent);
        returnVal = TRUE;
    }

    ES_Event slEvent;
    if (sl != sl_ref && sl) {
        slEvent.EventType = TAPE_ENTER_SL;
#ifdef TRACK_VERBOSE
        printf("SL TRIPPED\n");
#endif
        PostTopHSM(slEvent);
        returnVal = TRUE;
    } else if (sl != sl_ref && !sl) {
        slEvent.EventType = TAPE_EXIT_SL;
#ifdef TRACK_VERBOSE
        printf("SL UNTRIPPED\n");
#endif
        PostTopHSM(slEvent);
        returnVal = TRUE;
    }

    ES_Event srEvent;
    if (sr != sr_ref && sr) {
        srEvent.EventType = TAPE_ENTER_SR;
#ifdef TRACK_VERBOSE
        printf("SR TRIPPED\n");
#endif
        PostTopHSM(srEvent);
        returnVal = TRUE;
    } else if (sr != sr_ref && !sr) {
        srEvent.EventType = TAPE_EXIT_SR;
#ifdef TRACK_VERBOSE
        printf("SR UNTRIPPED\n");
#endif
        PostTopHSM(srEvent);
        returnVal = TRUE;
    }

    ES_Event rEvent;
    if (r != r_ref && r) {
        srEvent.EventType = TAPE_ENTER_R;
#ifdef TRACK_VERBOSE
        printf("R TRIPPED\n");
#endif
        PostTopHSM(rEvent);
        returnVal = TRUE;
    } else if (r != r_ref && !r) {
        srEvent.EventType = TAPE_EXIT_R;
#ifdef TRACK_VERBOSE
        printf("R UNTRIPPED\n");
#endif
        PostTopHSM(rEvent);
        returnVal = TRUE;
    }

    fl_ref = fl;
    fr_ref = fr;
    sl_ref = sl;
    sr_ref = sr;
    r_ref  = r;
    obs1_ref = obs1;
    obs2_ref = obs2;
    
    return (returnVal);
}