#include <stdint.h>
#include <unistd.h>

extern "C" {
	#include "car_lib.h"
}
class Driver
{
    public:
    	Driver();
        void drive(CVinfo cvInfo, uint8_t& buffer);
    private:
        struct DriveState {
        	bool isGoing;
        	bool isTurningRight;
        	bool isTurningLeft;
        	bool isEnteringCurve;

            bool isWhiteLineDetected;
        };
        DriveState driveState;

        // global delay
        uint16_t globalDelay;
        // Normal Drive
        bool Turning(DriveState driveState);
        void StateisEnteringCurve(struct DriveState *driveState);
        void StateisGoing(struct DriveState *driveState);
        bool TurnDetected(CVinfo cvInfo);
        bool LineDetected(CVinfo cvInfo);

        bool msDelay(uint16_t mstime);
};