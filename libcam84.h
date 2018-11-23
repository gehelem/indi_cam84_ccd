#ifndef __LIBCAM84_H__
#define __LIBCAM84_H__

#define CAM84_BAUDRATE  1000
#define CAM84_GAIN      0
#define CAM84_OFFSET    0
#define CAM84_LATENCYA  1
#define CAM84_LATENCYB  1
#define CAM84_TIMERA    2400
#define CAM84_TIMERB    2400
//#define D2XX


#ifdef __cplusplus
extern "C" {
#endif

bool cameraConnect(void);
bool cameraDisconnect(void); 
bool cameraIsConnected(void);
int  cameraStartExposure(int Bin,int StartX,int StartY,int NumX,int NumY, double Duration, bool light);
bool cameraStopExposure(void);
int  cameraGetCameraState(void);
bool cameraGetImageReady(void);
bool cameraSetGain (int val);
bool cameraSetOffset (int val);
int  cameraGetError(void);
bool cameraSetBaudrate(int val);  
uint16_t cameraGetImage(int i, int j);
void cameraGetImage2(void *buff);
bool cameraSetLibftdiTimers(int latA,int latB,int timerA,int timerB);
bool cameraSetContinuousADToggle(bool continuousADToggleValue);

#ifdef __cplusplus
}
#endif

#endif
