#ifndef __LIBCAM84_H__
#define __LIBCAM84_H__

bool cameraConnect(void);
bool cameraDisconnect(void); 
bool cameraIsConnected(void);
int  cameraStartExposure(int Bin,int StartX,int StartY,int NumX,int NumY, double Duration, bool light);
bool cameraStopExposure(void);
int  cameraGetCameraState(void);
bool cameraGetImageReady(void);
//unsigned short  * cameraGetImage(void);
void cameraGetImage(void *buff);
bool cameraSetGain (int val);
bool cameraSetOffset (int val);
int  cameraGetError(void);
bool cameraSetBaudrate(int val);  
static unsigned short bufim[3000][2000];

#endif
