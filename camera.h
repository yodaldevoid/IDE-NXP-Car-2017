#ifndef __CAMERA_H_
#define __CAMERA_H_

// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk
// (camera clk is the mod value set in FTM1)
#define INTEGRATION_TIME (0.0375f)

// Enable streaming camera data over UART
//#define DEBUG_CAM_DATA

#ifdef DEBUG_CAM_DATA
extern int capCnt;
#endif

extern int camReadDone;
extern uint16_t line[128];

void init_camera(void);

#endif /* __CAMERA_H_ */
