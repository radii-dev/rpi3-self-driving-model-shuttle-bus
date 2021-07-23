/**
  * @brief
  */
#include <stdint.h>

class Navigator
{
	public:
		Navigator();

        /* for drawPath */
        void drawPath(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint8_t (des)[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        /* Result info */
        CVinfo getInfo(uint8_t display_buf[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);

	private:
        /* Structures */
        struct Point {
            uint16_t x;
            uint16_t y;
            bool detected;
            bool isCenterPoint;
            bool isRightPoint;
            bool isLeftPoint;
        };
        struct DepartedFlag {
            bool isDepartedLeft;
            bool isDepartedRight;
        };
        /* Valuables */
        Point lastPoint;
        Point startingPoint;
        DepartedFlag departedFlag;

        /*  */
        bool isDownHillDetected(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isBothSideDetected(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y);

        /**
         * Normal Drive
         */
        Point getStartingPoint(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3]); // for loop control
        Point getRoadCenter(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y);
        Point getRoadPoint(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y);
        Point getRightPosition(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y);
        Point getLeftPosition(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y);
        float getRoadDiff(Point current, Point last);
        uint16_t getDirection(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        // before using isRoadEndDetected() function, confirm the last.roadCenter has been updated.
        bool isRoadEndDetected(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y);
        bool isRightDetected(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isLeftDetected(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isPathRight(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isPathLeft(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isPathStraight(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isRoadClose(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t distance);
        bool isDepartedRight(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isDepartedLeft(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isLeftReinstation(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isRightReinstation(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        bool isForwadPathExist(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3]);
        /* Functions for CV class */
        bool isDifferentType(Point first, Point second);

        /* Draw functions */
        void drawDot(uint8_t (des)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], Point point);
        void drawBigdot(uint8_t (des)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], Point point);
};
