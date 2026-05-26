#include "BOARD.h"
#include "AD.h"
#include "IO_Ports.h"
//#include "IOConstants.h"
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "TapeService.h"
#include "ES_Timers.h"
#include <stdio.h>
#include <xc.h>

//#define TAPE_VERBOSE

static uint8_t MyPriority;

uint8_t InitTapeService(uint8_t Priority) {
    MyPriority = Priority;
    
    ES_Event ThisEvent;
    ThisEvent.EventType = ES_INIT;
    return ES_PostToService(MyPriority, ThisEvent);
}

uint8_t PostTapeService(ES_Event ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

ES_Event RunTapeService(ES_Event ThisEvent) {
    ES_Event ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT;
    
//    static uint8_t refFL = ReadTapeSensorFL();
//    static uint8_t refFR = ReadTapeSensorFR();
//    static uint8_t refSL = ReadTapeSensorSL();
//    static uint8_t refSR = ReadTapeSensorSR();
//    static uint8_t refR  = ReadTapeSensorR();
//    
//    uint8_t readingFL = ReadTapeSensorFL();
//    uint8_t readingFR = ReadTapeSensorFR();
//    uint8_t readingSL = ReadTapeSensorSL();
//    uint8_t readingSR = ReadTapeSensorSR();
//    uint8_t readingR  = ReadTapeSensorR();
//    
//    if (reading !=)
//    
//    

    return ReturnEvent;
}