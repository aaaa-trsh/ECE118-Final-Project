#include "ES_Configure.h"
#include "ObstacleEventChecker.h"
#include "ES_Events.h"
#include "serial.h"
#include "AD.h"
#include "IO_Ports.h"
#include "roach.h"
#include "RobotInterface.h"
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
    
    uint8_t obs1 = ReadObstacleSensor1();
    uint8_t obs2 = ReadObstacleSensor2();
    
    uint8_t returnVal = FALSE;

    if (obs1 != obs1_ref && obs1) {
        ES_Event obs1Event;
        obs1Event.EventType = TW_1_DETECTED;        
#ifdef TRACK_VERBOSE
        printf("OBSTACLE 1 TRIPPED\n");
#endif
        returnVal = TRUE;
    } else if (obs1 != obs1_ref && !obs1) {
        printf("OBSTACLE 1 UNTRIPPED\n");
    }
    
    if (obs2 != obs2_ref && obs2) {
        ES_Event obs2Event;
        obs2Event.EventType = TW_2_DETECTED;        
#ifdef TRACK_VERBOSE
        printf("OBSTACLE 2 TRIPPED\n");
#endif
        returnVal = TRUE;
    } else if (obs2 != obs2_ref && !obs2) {
        printf("OBSTACLE 2 UNTRIPPED\n");
    }
    
    obs1_ref = obs1;
    obs2_ref = obs2;
    
    return (returnVal);
}