/**
  * @brief
  */
#include <stdint.h>
#include <stdio.h>
#include "system_management.h"
#include "cv.h"
Navigator::Navigator()
{
    departedFlag = {0,};
    lastPoint.x = VPE_OUTPUT_W/2;
    lastPoint.y = VPE_OUTPUT_H;
    startingPoint = {(VPE_OUTPUT_W/2), VPE_OUTPUT_H, 0,};
}
CVinfo Navigator::getInfo(uint8_t display_buf[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    startingPoint = getStartingPoint(yellow);
    CVinfo cvInfo = {1500, 0,};
    /**
     *  Normal Drive
     */
    cvInfo.direction = getDirection(yellow);
    if(cvInfo.direction == MAX_DIRECTION)         cvInfo.isLeftTurnDetected = true;
    else if(cvInfo.direction == MIN_DIRECTION)   cvInfo.isRightTurnDetected = true;

    cvInfo.isLeftDetected = isLeftDetected(yellow);
    cvInfo.isRightDetected = isRightDetected(yellow);
    if( !cvInfo.isLeftDetected && !cvInfo.isRightDetected )
        cvInfo.direction = CEN_DIRECTION;

    cvInfo.isDepartedLeft = isDepartedLeft(yellow);
    cvInfo.isDepartedRight = isDepartedRight(yellow);

    cvInfo.isLeftReinstation = isLeftReinstation(yellow);
    cvInfo.isRightReinstation = isRightReinstation(yellow);
    cvInfo.isRoadClose = isRoadClose(yellow, ISROADCLOSE_DISTANCE);
    cvInfo.isPathStraight = isPathStraight(yellow);
    cvInfo.isForwadPathExist = isForwadPathExist(yellow);
    /************************************************************************************/
    /**
     *  DownHill
     */
    cvInfo.isDownHillDetected = isDownHillDetected(yellow);
    cvInfo.isDownHillDetected2 = isDownHillDetected2(yellow);

    // Depart handling
    if(cvInfo.isDepartedLeft)           departedFlag.isDepartedLeft = true;
    else if(cvInfo.isDepartedRight)     departedFlag.isDepartedRight = true;

    if(cvInfo.isRightReinstation)       departedFlag.isDepartedLeft = false;
    else if(cvInfo.isLeftReinstation)   departedFlag.isDepartedRight = false;

    if(departedFlag.isDepartedLeft) {
        cvInfo.isDepartedLeft = true;
        startingPoint = {319, VPE_OUTPUT_H, 0,};
    }
    else if(departedFlag.isDepartedRight) {
        cvInfo.isDepartedRight = true;
        startingPoint = {0, VPE_OUTPUT_H, 0,};
    }
    return cvInfo;
}
/**
 *  Draw functions 
 */
void Navigator::drawPath(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint8_t (des)[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint16_t y;
    Point roadCenter = {0,};
    for(y=VPE_OUTPUT_H-1; y > 0; y--) {
        roadCenter = getRoadCenter(src, y);
        if(roadCenter.detected) {
            drawDot(des, roadCenter);
            drawDot(des, getRightPosition(src, y));
            drawDot(des, getLeftPosition(src, y));
            lastPoint = roadCenter;
            if(isRoadEndDetected(src, y)) break;
        }
    }
    lastPoint = startingPoint;
}
void Navigator::drawDot(uint8_t (des)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], Point point)
{
    uint16_t x,y;
    x = point.x;
    y = point.y;
    #ifdef bgr24
        des[y][x][2] = 255;
        des[y][x][0] = des[y][x][1] = 0;
    #endif
}
void Navigator::drawBigdot(uint8_t (des)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], Point point)
{
    int32_t i,j;
    uint16_t x,y;
    x = point.x;
    y = point.y;
    #ifdef bgr24
        for(i=-1; i<2; i++) {
            for(j=-1; j<2; j++) {
                des[y+j][x+i][1] = 255;
                des[y+j][x+i][0] = des[y+j][x+i][2] = 0;
            }
        }
    #endif
}
bool Navigator::isRoadEndDetected(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y)
{
    Point right_point = getRightPosition(src,y);
    Point left_point = getLeftPosition(src,y);
    if(right_point.detected & left_point.detected) {
        if( (right_point.x - left_point.x) < 6 ) return true;
    }
    if(right_point.detected) {
        if( right_point.x < 3 ) return true;
    }
    else if(left_point.detected) {
        if( left_point.x > 315 ) return true;
    }
    return false;
}
/**
 *  Normal Drive
 */
uint16_t Navigator::getDirection(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint8_t y;
    uint8_t i,j;
    i = j = 0;
    uint8_t threshold = ( (double)(SIDE_DOWN - SIDE_UP)*((double)GET_DIRECTION_THRESHOLD/100) );
    float totalRoadDiff = 0;
    uint16_t direction = 1500;
    float slope;
    Point lastRoadPoint = {0,};
    Point roadPoint = {0,};
    Point roadCenter = {0,};
    for(y=SIDE_DOWN; y > SIDE_UP; y--) {
        roadCenter = getRoadCenter(src, y);
        roadPoint = getRoadPoint(src, y);
        if(roadPoint.detected) {
            i++;
            lastRoadPoint = roadPoint;
            lastPoint = roadCenter;
            break;
        }
        j++;
    }
    for(y--; y > SIDE_UP; y--) {
        roadCenter = getRoadCenter(src, y);
        roadPoint = getRoadPoint(src, y);
        if(roadPoint.detected) {
            if( !isDifferentType(roadPoint, lastRoadPoint) ) {
                totalRoadDiff += getRoadDiff(roadPoint, lastRoadPoint);
                lastRoadPoint = roadPoint;
                lastPoint = roadCenter;
            }
            i++;
            if(isRoadEndDetected(src, y)) break;
        }
        j++;
    }
    lastPoint = startingPoint;

    if(((float)i/j)*100 > threshold) slope = (totalRoadDiff / i)/SLOPE_DIVIDE_FACTOR;
    else slope = 0;
    // if(slope == 0)              direction = 1500;
    if(slope > 1.11)      direction = MAX_DIRECTION;    // Left
    // else if(slope > 1.11)      direction = MAX_DIRECTION;    // Left
    else if (slope < -1.11)   direction = MIN_DIRECTION;     // Right
    /*
    else if(slope > 1.11)      direction = 2000;    // Left
    else if (slope < -1.11)   direction = 1000;     // Right
    */
    else                        direction = (uint16_t)(1500 + 450 * slope);
    return direction;
}
Navigator::Point Navigator::getRoadCenter(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y)
{
    Point roadPoint = {0,};
    Point right_point = getRightPosition(src,y);
    Point left_point = getLeftPosition(src,y);
    if(right_point.detected & left_point.detected) {
        roadPoint.x = (right_point.x + left_point.x)/2;
        roadPoint.y = (right_point.y + left_point.y)/2;
        roadPoint.detected = true;
    }
    else if( right_point.detected ) {
        roadPoint.x = right_point.x / 2;
        roadPoint.y = right_point.y;
        roadPoint.detected = true;
    }
    else if( left_point.detected ) {
        roadPoint.x = left_point.x + (VPE_OUTPUT_W - left_point.x)/ 2;
        roadPoint.y = left_point.y;
        roadPoint.detected = true;
    }
    else roadPoint.detected = false;
    return roadPoint;
}
Navigator::Point Navigator::getRoadPoint(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y)
{
    Point roadPoint = {0,};
    Point right_point = getRightPosition(src,y);
    Point left_point = getLeftPosition(src,y);
    
    if(right_point.detected & left_point.detected) {
        roadPoint.x = (right_point.x + left_point.x)/2;
        roadPoint.y = (right_point.y + left_point.y)/2;
        roadPoint.detected = true;
        roadPoint.isCenterPoint = true;
    }
    else if( right_point.detected ) {
        roadPoint.x = right_point.x;
        roadPoint.y = right_point.y;
        roadPoint.detected = true;
        roadPoint.isRightPoint = true;
    }
    else if( left_point.detected ) {
        roadPoint.x = left_point.x;
        roadPoint.y = left_point.y;
        roadPoint.detected = true;
        roadPoint.isLeftPoint = true;
    }
    else roadPoint.detected = false;
    return roadPoint;
}
bool Navigator::isDownHillDetected(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    int y;
    for(y = DOWNHILL_CHECK_DOWN; y > DOWNHILL_CHECK_UP; y--) {
        if(!isBothSideDetected(yellow, y)) break;
    }
    Point right_point = getRightPosition(yellow,y);
    Point left_point = getLeftPosition(yellow,y);

    if( ((left_point.x - right_point.x) < DOWNHILL_DETECT_THRESHOLD) && (right_point.x - left_point.x) != 0 ) return true;
    else return false;
}
bool Navigator::isDownHillDetected2(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    Point left_point = getLeftPosition(yellow, DOWNHILL_CHECK_UP);
    if( left_point.x < 160 && left_point.x > 0 ) return false;
    else return true;
}
bool Navigator::isBothSideDetected(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y)
{
    Point right_point = getRightPosition(yellow,y);
    Point left_point = getLeftPosition(yellow,y);
    if(right_point.detected & left_point.detected) return true;
    else return false;
}
Navigator::Point Navigator::getRightPosition(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y)
{
    uint16_t temp,i,j;
    // detect direction from Right
    temp = 0;
    Point point = {0,};
    for(i = lastPoint.x; i < VPE_OUTPUT_W; i++) {
        if( src[y][i][0] )
        {
            for(j=1; j<11; j++) {
                if( src[y][i+j][0] )    temp++;
            }
            if(temp > LINE_DETECT_THRESHOLD) {
                point = {i, y, true};
                return point;
            }
        }
    }
    return point;
}
Navigator::Point Navigator::getLeftPosition(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t y)
{
    uint16_t temp,i,j;
    // detect direction from Left
    temp = 0;
    Point point = {0,};
    for(i = lastPoint.x; i > 0; i--) {
        if( src[y][i][0] )
        {
            for(j=1; j<11; j++) {
                if( src[y][i-j][0] )    temp++;
            }
            if(temp > LINE_DETECT_THRESHOLD) {
                point = {i, y, true};
                return point;
            }
        }
    }
    return point;
}
Navigator::Point Navigator::getStartingPoint(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint8_t y;
    Point roadCenter;
    for(y=VPE_OUTPUT_H-1; y > 0; y--) {
        roadCenter = getRoadCenter(src, y);
        if(roadCenter.detected) {
            return roadCenter;
        }
    }
    return {(VPE_OUTPUT_W/2), VPE_OUTPUT_H, 0,};
}
float Navigator::getRoadDiff(Point current, Point last)
{
    int x_Variation, y_Variation;
    x_Variation = current.x - last.x;
    y_Variation = current.y - last.y;
    if( !(x_Variation && y_Variation) ) return 0;
    else return (float)x_Variation / y_Variation;
}
bool Navigator::isDepartedRight(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    Point point = getRoadPoint(yellow, 179);
    if(point.y && point.isRightPoint) {
        if(point.x < 2) return true;
    }
    return false;
}
bool Navigator::isDepartedLeft(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    Point point = getRoadPoint(yellow, 179);
    if(point.y && point.isLeftPoint) {
        if(point.x > 317) return true;
    }
    return false;
}
bool Navigator::isLeftReinstation(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint8_t i;
    uint8_t temp = 0;
    Point point = getRoadPoint(yellow, 179);
    if(point.y) {
        for(i=0; i<5; i++) {
            if(!yellow[179][i][0]) temp++;
        }
        if(temp == 5) {
            if(point.x < REINSTATION_WIDTH) return true;
        }
    }
    return false;
}
bool Navigator::isRightReinstation(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint8_t i;
    uint8_t temp = 0;
    Point point = getRoadPoint(yellow, 179);
    if(point.y) {
        for(i=0; i<5; i++) {
            if(!yellow[179][319-i][0]) temp++;
        }
        if(temp == 5) {
            if(point.x > 319 - REINSTATION_WIDTH) return true;
        }
    }
    return false;
}
bool Navigator::isRoadClose(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3], uint16_t distance)
{
    uint8_t y,i,temp;
    temp = 0;
    for( y = 179; y > 0; y--) {
        if( src[y][159][0] ) {
            for(i=1; i<11; i++) {
                if( src[y-i][159][0] ) temp++;
            }
            if( temp >  ISROADCLOSE_THRESHOLD ) {
                if( y > (179 - distance) ) return true;
            }
        }
    }
    return false;
}
bool Navigator::isForwadPathExist(uint8_t yellow[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint8_t y,i,temp;
    temp = 0;
    for( y = 179; y > 179 - FOWARD_PATH_EXIST_DISTANCE; y--) {
        if( yellow[y][159][0] ) {
            for(i=1; i<11; i++) {
                if( yellow[y-i][159][0] ) temp++;
            }
            if( temp >  FOWARD_PATH_EXIST_THRESHOLD )   return false;
        }
    }
    return true;
}
bool Navigator::isRightDetected(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint8_t y;
    uint8_t i = 0;
    Point lastRoadPoint = {0,};
    Point roadPoint = {0,};
    Point roadCenter = {0,};
    for(y=SIDE_DOWN; y > SIDE_UP; y--) {
        roadCenter = getRoadCenter(src, y);
        roadPoint = getRoadPoint(src, y);
        if(roadPoint.isRightPoint | roadPoint.isCenterPoint) {
            i++;
            lastRoadPoint = roadPoint;
            lastPoint = roadCenter;
            break;
        }
    }
    for(y--; y > SIDE_UP; y--) {
        roadCenter = getRoadCenter(src, y);
        roadPoint = getRoadPoint(src, y);
        if(roadPoint.isRightPoint | roadPoint.isCenterPoint) {
            lastRoadPoint = roadPoint;
            lastPoint = roadCenter;
            i++;
            if(isRoadEndDetected(src, y)) return true;
        }
    }
    lastPoint = startingPoint;

    if(i > SIDE_DIRECTION_THRESHOLD) return true;
    else return false;
}
bool Navigator::isLeftDetected(uint8_t (src)[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint8_t y;
    uint8_t i = 0;
    Point lastRoadPoint = {0,};
    Point roadPoint = {0,};
    Point roadCenter = {0,};
    for(y=SIDE_DOWN; y > SIDE_UP; y--) {
        roadCenter = getRoadCenter(src, y);
        roadPoint = getRoadPoint(src, y);
        if(roadPoint.isLeftPoint | roadPoint.isCenterPoint) {
            i++;
            lastRoadPoint = roadPoint;
            lastPoint = roadCenter;
            break;
        }
    }
    for(y--; y > SIDE_UP; y--) {
        roadCenter = getRoadCenter(src, y);
        roadPoint = getRoadPoint(src, y);
        if(roadPoint.isLeftPoint | roadPoint.isCenterPoint) {
            lastRoadPoint = roadPoint;
            lastPoint = roadCenter;
            i++;
            if(isRoadEndDetected(src, y)) return true;
        }
    }
    lastPoint = startingPoint;

    if(i > SIDE_DIRECTION_THRESHOLD) return true;
    else return false;
}
bool Navigator::isPathStraight(uint8_t src[VPE_OUTPUT_H][VPE_OUTPUT_W][3])
{
    uint8_t y;
    uint8_t i,j;
    i = j = 0;
    uint8_t threshold = ( (double)(FRONT_DOWN - FRONT_UP)*((double)GET_DIRECTION_THRESHOLD/100) );
    float totalRoadDiff = 0;
    float slope;
    Point lastRoadPoint = {0,};
    Point roadPoint = {0,};
    Point roadCenter = {0,};
    for(y=FRONT_DOWN; y > FRONT_UP; y--) {
        roadCenter = getRoadCenter(src, y);
        roadPoint = getRoadPoint(src, y);
        if(roadPoint.detected) {
            i++;
            lastRoadPoint = roadPoint;
            lastPoint = roadCenter;
            break;
        }
        j++;
    }
    for(y--; y > FRONT_UP; y--) {
        roadCenter = getRoadCenter(src, y);
        roadPoint = getRoadPoint(src, y);
        if(roadPoint.detected) {
            if( !isDifferentType(roadPoint, lastRoadPoint) ) {
                totalRoadDiff += getRoadDiff(roadPoint, lastRoadPoint);
                lastRoadPoint = roadPoint;
                lastPoint = roadCenter;
            }
            i++;
            if(isRoadEndDetected(src, y)) break;
        }
        j++;
    }
    lastPoint = startingPoint;

    if(((float)i/j)*100 > threshold) slope = (totalRoadDiff / i)/2;
    else return false;

    if(abs(slope) < 0.3)  return true;
    else                   return false;
}
bool Navigator::isDifferentType(Point first, Point second)
{
    if(first.isCenterPoint & second.isCenterPoint) return false;
    else if(first.isRightPoint & second.isRightPoint) return false;
    else if(first.isLeftPoint & second.isLeftPoint) return false;
    else return true;
}

