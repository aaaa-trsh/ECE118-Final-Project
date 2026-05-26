#include "ES_Configure.h"
#include "TrackwireEventChecker.h"
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
#define TRACK_VERBOSE
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
;uint8_t CheckWireEvents(void) {
    // Initialize track wire ports
    static uint8_t wire1_ref = 0;
    static uint8_t wire2_ref = 0;
    
    uint8_t wire1 = ReadTrackwireSensor1();
    uint8_t wire2 = ReadTrackwireSensor2();
    
    uint8_t returnVal = FALSE;

    if (wire1 != wire1_ref && wire1) {
        ES_Event wire1Event;
        wire1Event.EventType = TW_1_DETECTED;        
#ifdef TRACK_VERBOSE
        printf("TRACKWIRE 1 TRIPPED");
#endif
        returnVal = TRUE;
    }
    
    if (wire2 != wire2_ref && wire2) {
        ES_Event wire2Event;
        wire2Event.EventType = TW_2_DETECTED;        
#ifdef TRACK_VERBOSE
        printf("TRACKWIRE 2 TRIPPED");
#endif
        returnVal = TRUE;
    }
    
    wire1_ref = wire1;
    wire2_ref = wire2;
    
    return (returnVal);
}