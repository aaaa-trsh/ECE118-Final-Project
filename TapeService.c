#include "BOARD.h"
#include "AD.h"
#include "IO_Ports.h"
#include "IOConstants.h"
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "TapeService.h"
#include "ES_Timers.h"
#include <stdio.h>
#include <xc.h>

#define TAPE_TIMER 2

#define STATE_BASELINE_WAIT 0
#define STATE_BASELINE_MEASURE 1
#define STATE_SAMPLE_WAIT 2
#define STATE_SAMPLE_MEASURE 3

#define TAPE_WHITE 0
#define TAPE_BLACK 1

//#define TAPE_VERBOSE

static uint8_t MyPriority;
static uint8_t state;
static uint8_t last_baseline;


uint8_t InitTapeService(uint8_t Priority) {
    MyPriority = Priority;
    state = STATE_BASELINE_WAIT;
    
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
    
    ES_EventTyp_t curEvent;
    static unsigned int baselines[NUM_TAPE_SENSORS];
    static unsigned int samples[NUM_TAPE_SENSORS];
    static uint16_t outputs[NUM_TAPE_SENSORS];
    
    switch (ThisEvent.EventType) {
    case ES_INIT:
        printf("INIT TAPE SERVICE");
        for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
            AD_AddPins(TAPE_SENSOR_CONSTANTS[i].detector);
            IO_PortsSetPortOutputs(
                TAPE_SENSOR_CONSTANTS[i].port, 
                TAPE_SENSOR_CONSTANTS[i].emitter
            );
        }
        
        ES_Timer_InitTimer(TAPE_TIMER, 2);
        break;

    case ES_TIMEOUT:
        if (!AD_IsNewDataReady) {
            break;
        } else {
            state = (state + 1) % 4;
        }
        
        switch (state) {
            case STATE_BASELINE_WAIT: // turn emitters off, then wait 20ms
#ifdef TAPE_VERBOSE
                printf("LED OFF\n");
#endif
                for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
                    TapeIOConstants current = TAPE_SENSOR_CONSTANTS[i];
                    IO_PortsClearPortBits(current.port, current.emitter);
                }
                ES_Timer_InitTimer(TAPE_TIMER, 2);
                break;
            case STATE_BASELINE_MEASURE: // take baseline measurement, wait 50ms
#ifdef TAPE_VERBOSE
                printf("Measure baselines (LED OFF): ");
#endif
                for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
                    baselines[i] = AD_ReadADPin(
                        TAPE_SENSOR_CONSTANTS[i].detector
                    );
                    
#ifdef TAPE_VERBOSE
                    printf("%d, ", baselines[i]);
#endif
                }
#ifdef TAPE_VERBOSE
                printf("\n");
#endif
                ES_Timer_InitTimer(TAPE_TIMER, 5);
                break;
            case STATE_SAMPLE_WAIT: // turn emitters on, then wait 20ms
#ifdef TAPE_VERBOSE
                printf("LED ON\n");
#endif
                for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
                    TapeIOConstants current = TAPE_SENSOR_CONSTANTS[i];
                    IO_PortsSetPortBits(current.port, current.emitter);
                }
                ES_Timer_InitTimer(TAPE_TIMER, 2);
                break;
            case STATE_SAMPLE_MEASURE: // take sample measurement, then wait 20ms
#ifdef TAPE_VERBOSE
                printf("Measure samples   (LED ON) : ");
#endif
                for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
                    samples[i] = AD_ReadADPin(
                        TAPE_SENSOR_CONSTANTS[i].detector
                    );
#ifdef TAPE_VERBOSE
                    printf("%d, ", samples[i]);
#endif
                }
#ifdef TAPE_VERBOSE
                printf("\n");
#endif
                ES_Timer_InitTimer(TAPE_TIMER, 5);
                break;
        }
        PostTapeService(ReturnEvent);
        break;
    }
    
    return ReturnEvent;
}