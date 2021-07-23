/*
 * @File   project_config.h
 * @Brief  Configurations for main.c
 *
 */
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <sys/sysinfo.h>

#define NORMAL_SPEED	0b10000000 // 130
#define SLOW_SPEED		0b00000000 // 100
// #define PARKING_SPEED	90 // 100

/**
 *  Normal Drive
 */
#define FRONT_UP			79
#define FRONT_DOWN		148
#define SIDE_UP			149
#define SIDE_DOWN			179

#define MAX_DIRECTION 1997
#define MIN_DIRECTION 1003
#define CEN_DIRECTION 1500

#define SIDE_DIRECTION_THRESHOLD		8	//90%

#define GET_DIRECTION_THRESHOLD			90 	//90%
#define LINE_DETECT_THRESHOLD 			7
#define SLOPE_DIVIDE_FACTOR	  			2

#define ISROADCLOSE_DISTANCE			40
#define ISROADCLOSE_THRESHOLD			5

#define FOWARD_PATH_EXIST_DISTANCE		80
#define FOWARD_PATH_EXIST_THRESHOLD	ISROADCLOSE_THRESHOLD

#define REINSTATION_WIDTH				80

#define WHITELINE_DETECT_THRESHOLD_CV	10
#define WHITELINE_DETECT_THRESHOLD		6 // Low -> sensetive
/**
 *  Down Hill
 */
#define DOWNHILL_CHECK_UP			49
#define DOWNHILL_CHECK_DOWN			179
#define DOWNHILL_DITECT_THRESHOLD	125
/**
*  HSV
*/
#define YELLOW 0
#define yellow_HUE_MAX    55
#define yellow_HUE_MIN    5
#define yellow_SAT_MAX    255
#define yellow_SAT_MIN    50
#define yellow_VAL_MAX    255
#define yellow_VAL_MIN    50

#define RED 1
#define red_HUE_MAX    180
#define red_HUE_MIN    140
#define red_SAT_MAX    255
#define red_SAT_MIN    150
#define red_VAL_MAX    255
#define red_VAL_MIN    50

#define GREEN 2
#define green_HUE_MAX    90
#define green_HUE_MIN    45
#define green_SAT_MAX    255
#define green_SAT_MIN    60
#define green_VAL_MAX    255
#define green_VAL_MIN    10

#define WHITE 3
#define white_HUE_MAX    255
#define white_HUE_MIN    90
#define white_SAT_MAX    70
#define white_SAT_MIN    0
#define white_VAL_MAX    255
#define white_VAL_MIN    190

/**
  * @Brief
  */
#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"
#define VPE_OUTPUT_W            320
#define VPE_OUTPUT_H            180

// display output & dump  format: bgr24, w:320, h:180
#define bgr24
#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3) // bgr24 : 24bpp
#define VPE_OUTPUT_FORMAT       "bgr24"
#define VPE_OUTPUT_RESOLUTION  VPE_OUTPUT_W*VPE_OUTPUT_H

class System_resource
{
	public:
		System_resource();
		uint64_t getVirtualMemUsed();
		uint64_t getTotalVirtualMem();
		uint64_t getTotalPhysMem();
		uint64_t getPhysMemUsed();
	private:
		struct sysinfo memInfo;
		long long totalVirtualMem;
		long long virtualMemUsed;
		long long totalPhysMem;
		long long physMemUsed;
};
struct CVinfo {
	/**
	 *  Normal Drive
	 */
	uint16_t direction;
	bool isLeftTurnDetected;
	bool isRightTurnDetected;
	bool isLeftDetected;
	bool isRightDetected;
	bool isPathStraight;
	bool isPathLeft;
	bool isPathRight;
	bool isRoadClose;
	bool isDepartedLeft;
	bool isDepartedRight;
	bool isLeftReinstation;
	bool isRightReinstation;
	bool isForwadPathExist;
	/**
	 *  Down Hill
	 */
	bool isDownHillDetected;
};
struct Mission {
	bool isDownHillEnd;
};