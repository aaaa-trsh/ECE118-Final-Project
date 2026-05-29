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
    ES_Timer_Init();
    
    printf("Starting Team 5 Robot!!\r\n");
    printf("using the 2nd Generation Events & Services Framework\r\n");
    
//    while (1) {
//        MecanumDrive(0, 128, 0);
//        SetShooter(1, 0);
//        SetIndexer(1);
//        
//        printf("Obstacle L: %d, Obstacle R: %d BEACON1: %5d, BEACON2: %5d, FL %d, FR %d, R %d, SL %d, SR %d\n", 
//            ReadObstacleSensor1(),
//            ReadObstacleSensor2(),
//            ReadBeaconSensor1(),
//            ReadBeaconSensor2(),
//            ReadTapeSensorFL(),
//            ReadTapeSensorFR(),
//            ReadTapeSensorR(),
//            ReadTapeSensorSL(),
//            ReadTapeSensorSR()
//        );
//        printf("BEACON1: %5d, BEACON2: %5d\n", 
//            ((int16_t)ReadBeaconSensor1()) - 512,
//            ((int16_t)ReadBeaconSensor2()) - 512
//        );
//    }

    
    ErrorType = ES_Initialize();
    if (ErrorType == Success) {
        ErrorType = ES_Run();

    }
    //if we got to here, there was an error
    switch (ErrorType) {
    case FailedPointer:
        printf("Failed on NULL pointer");
        break;
    case FailedInit:
        printf("Failed Initialization");
        break;
    default:
        printf("Other Failure: %d", ErrorType);
        break;
    }
    for (;;)
        ;
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
