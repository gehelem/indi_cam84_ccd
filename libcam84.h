#ifndef __LIBCAM84_H__
#define __LIBCAM84_H__

#define CAM84_BAUDRATE_DIVISOR  2
#define CAM84_GAIN      50
#define CAM84_OFFSET    0
#define CAM84_LATENCYA  1
#define CAM84_LATENCYB  1
#define CAM84_TIMERA    2400
#define CAM84_TIMERB    2400
//#define D2XX


#ifdef __cplusplus
//extern "C" {
#endif

//#include <libusb-1.0/libusb.h>

#include <mutex>

bool cameraConnect(void);
bool cameraDisconnect(void); 
bool cameraIsConnected(void);
int  cameraStartExposure(int Bin,int StartX,int StartY,int NumX,int NumY, double Duration, int theFrameType);
bool cameraStopExposure(void);
int  cameraGetCameraState(void);
bool cameraGetImageReady(void);
bool cameraSetGain (int val);
bool cameraSetOffset (int val);
int  cameraGetError(void);
bool cameraSetBaudrateDivisor(int val);
uint8_t * cameraGetImagePointer();
//void cameraGetImage2(void *buff);
bool cameraSetLibftdiTimers(int latA,int latB,int timerA,int timerB);
bool cameraSetContinuousADToggle(bool continuousADToggleValue);

#ifdef __cplusplus
//}
#endif

#endif

class LibCam84
{
public:
    LibCam84();
protected:
//    const char *getDeviceName();
};


