#include "BOARD.h"
#include <xc.h>
#include <stdio.h>
#include "ES_Configure.h"
#include "ES_Framework.h"

#include "pwm.h"

#include "RobotInterface.h"
void main(void)
{
    
    ES_Return_t ErrorType;
    
    BOARD_Init();
    InitRobot();
    
    printf("Starting Team 5 Robot!!\r\n");
    printf("using the 2nd Generation Events & Services Framework\r\n");
    
    while (1) {
//        MecanumDrive(0, 128);
//        SetShooter(1, 0);
//        SetIndexer(0);
//        printf("FL %d, FR %d, R %d, SL %d, SR %d\n", 
//            ReadTapeSensorFL() > 0,
//            ReadTapeSensorFR()>0,
//            ReadTapeSensorR()>0,
//            ReadTapeSensorSL()>0,
//            ReadTapeSensorSR()>0
//        );
        
        printf("TL: %d, TR: %d\n", // BEACON1: %5d, BEACON2: %5d
            ReadTrackwireSensor1() > 0,
            ReadTrackwireSensor2() > 0
//            ReadBeaconSensor1(),
//            ReadBeaconSensor2()
        );
    }

    
//    printf("Tape: %d, Trackwire: %d", ReadTapeSensorFL(), ReadTrackwireSensor1());

    // Your hardware initialization function calls go here

    // now initialize the Events and Services Framework and start it running
//    ErrorType = ES_Initialize();
//    if (ErrorType == Success) {
//        ErrorType = ES_Run();
//
//    }
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
