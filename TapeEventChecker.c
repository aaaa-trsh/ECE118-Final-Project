#include "ES_Configure.h"
#include "TrackwireEventChecker.h"
#include "ES_Events.h"
#include "serial.h"
#include "AD.h"
#include "IO_Ports.h"
#include "RobotInterface.h"
//#include "IOConstants.h"
#include <xc.h>

/*******************************************************************************
 * MODULE #DEFINES                                                             *
 ******************************************************************************/

uint8_t CheckTapeEvents(void) {
    uint8_t returnVal = FALSE;

    static uint8_t refFL = 0;
    static uint8_t refFR = 0;
    static uint8_t refSL = 0;
    static uint8_t refSR = 0;
    static uint8_t refR  = 0;
    
    static uint8_t refsInitialized = 0;
    
    uint8_t readingFL = ReadTapeSensorFL();
    uint8_t readingFR = ReadTapeSensorFR();
    uint8_t readingSL = ReadTapeSensorSL();
    uint8_t readingSR = ReadTapeSensorSR();
    uint8_t readingR  = ReadTapeSensorR();
    
    if (!refsInitialized) {
        refFL = readingFL;
        refFR = readingFR;
        refSL = readingSL;
        refSR = readingSR;
        refR  = readingR;
        refsInitialized = 1;
        return (returnVal);
    }

    ES_Event eventFL = NO_EVENT;
    if (refFL != 255 && refFL != readingFL) {
        returnVal = TRUE;
        eventFL.EventType = readingFL ? TAPE_ENTER_FL : TAPE_EXIT_FL;
    }
    
    ES_Event eventFR = NO_EVENT;
    if (refFR != readingFR) {
        returnVal = TRUE;
        eventFR.EventType = readingFR ? TAPE_ENTER_FR : TAPE_EXIT_FR;
    }
    
    ES_Event eventR = NO_EVENT;
    if (refR != readingR) {
        returnVal = TRUE;
        eventR.EventType = readingR ? TAPE_ENTER_R : TAPE_EXIT_R;
    }
    
    ES_Event eventSL = NO_EVENT;
    if (refSL != readingSL) {
        returnVal = TRUE;
        eventSL.EventType = readingSL ? TAPE_ENTER_SL : TAPE_EXIT_SL;
    }
    
    ES_Event eventSR = NO_EVENT;
    if (refSR != readingSR) {
        returnVal = TRUE;
        eventSR.EventType = readingSR ? TAPE_ENTER_SR : TAPE_EXIT_SR;
    }
    
    refFL = readingFL;
    refFR = readingFR;
    refSL = readingSL;
    refSR = readingSR;
    refR  = readingR;
    
    return (returnVal);
}