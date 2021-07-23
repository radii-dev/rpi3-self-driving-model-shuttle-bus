#include "system_management.h"
#include <sys/time.h>
#include "car_control.h"

Driver::Driver()
{
    driveState = {1,0,};
    globalDelay = 0;
}
void Driver::drive(CVinfo cvInfo, uint8_t& buffer)
{
    printf("\r\n--- Drive state\r\n");
    printf("\tGoing %d Left %d Right %d EnteringCurve %d\r\n", driveState.isGoing, driveState.isTurningLeft, driveState.isTurningRight, driveState.isEnteringCurve);
    /**
     */
    // Code Cleanup Required.
    // if(cvInfo.isPathStraight) buffer += NORMAL_SPEED;
    // else buffer += SLOW_SPEED;
    
    

    if(cvInfo.isDepartedLeft) {
        driveState.isGoing = false;
        driveState.isTurningRight = true;
        buffer = 50;
        SpeedControl(cvInfo, buffer);
        printf("\tDepartedLeft\r\n");
        return;
    }
    else if(cvInfo.isDepartedRight) {
        driveState.isGoing = false;
        driveState.isTurningLeft = true;
        buffer = 100;
        SpeedControl(cvInfo, buffer);
        printf("\tDepartedRight\r\n");
        return;
    }
    
    if(driveState.isGoing) {
        if(TurnDetected(cvInfo)) {
            StateisEnteringCurve(&driveState);
            return;
        }
        else if( !Turning(driveState) ) {
            buffer = (cvInfo.direction / 20);
            SpeedControl(cvInfo, buffer);
            printf("\tNotTurning\r\n");
            return;
        }
    }
    if(driveState.isTurningRight) {
        if( !LineDetected(cvInfo) ) {
            StateisGoing(&driveState);
            return;
        }
        else if(cvInfo.isDownHillDetected && cvInfo.isDownHillDetected2) {
        buffer = (cvInfo.direction / 20);
        SpeedControl(cvInfo, buffer);
        printf("\tDownHillDetected\r\n");
        return;
        }
        else {
            if(!(buffer & 0b10000000)) buffer += 0b10000000;
            printf("\tIt is Not DownHill!\r\n");
        }
    }
    if(driveState.isTurningLeft) {
        if( !LineDetected(cvInfo) ) {
            StateisGoing(&driveState);
            return;
        }
        else return;
    }
    if(driveState.isEnteringCurve) {
        if(cvInfo.isRoadClose) {
            if(cvInfo.direction < 1500) {
                driveState.isEnteringCurve = false;
                driveState.isTurningRight = true;
                buffer = 50;
                SpeedControl(cvInfo, buffer);
                printf("\tTurningRight\r\n");
                return;
            }
            else if(cvInfo.direction > 1500) {
                driveState.isEnteringCurve = false;
                driveState.isTurningLeft = true;
                buffer = 100;
                SpeedControl(cvInfo, buffer);
                printf("\tTurningLeft\r\n");
                return;
            }
            else {
                buffer = (cvInfo.direction / 20);
                SpeedControl(cvInfo, buffer);
                printf("\tGo\r\n");
                return;
            }
        }
        else {
            //buffer += (cvInfo.direction / 20)-75;
            buffer = 75;
            SpeedControl(cvInfo, buffer);
            printf("\tStraight\r\n");
            return;
        }
        
    }
}
void Driver::SpeedControl(CVinfo cvInfo, uint8_t& buffer)
{
    /*
    if(cvInfo.isPathStraight) buffer += NORMAL_SPEED;
    else if(cvInfo.isDownHillDetected) buffer += SLOW_SPEED;
    else buffer += SLOW_SPEED;
    */
   if(cvInfo.isDownHillDetected && driveState.isTurningRight) buffer += SLOW_SPEED;
   else buffer += NORMAL_SPEED;
}
bool Driver::TurnDetected(CVinfo cvInfo)
{
    if(cvInfo.isForwadPathExist) return false;
    if(cvInfo.isRightTurnDetected | cvInfo.isLeftTurnDetected) return true;
    else return false;
}
bool Driver::LineDetected(CVinfo cvInfo)
{
    if(cvInfo.isLeftDetected | cvInfo.isRightDetected) return true;
    else return false;
}
bool Driver::Turning(DriveState driveState)
{
    if(driveState.isTurningRight | driveState.isTurningLeft) return true;
    else return false;
}
void Driver::StateisEnteringCurve(struct DriveState *driveState)
{
    driveState->isGoing = false;
    driveState->isEnteringCurve = true;
}
void Driver::StateisGoing(struct DriveState *driveState)
{
    driveState->isTurningLeft = false;
    driveState->isTurningRight = false;
    driveState->isGoing = true;
}
bool Driver::msDelay(uint16_t mstime)
{
    globalDelay++;
    if(globalDelay == mstime*3/100) {
        globalDelay = 0;
        return true;
    }
    else return false;
}
