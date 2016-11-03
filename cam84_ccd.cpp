/*
   INDI Developers Manual
   Tutorial #3

   "Cam84 Driver"

   We develop a Cam84 driver.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simpleccd.cpp
    \brief Construct a basic INDI CCD device that simulates exposure & temperature settings. It also generates a random pattern and uploads it as a FITS file.
    \author Jasem Mutlaq

    \example simpleccd.cpp
    A Cam84 device that can capture images and control temperature. It returns a FITS image to the client. To build drivers for complex CCDs, please
    refer to the INDI Generic CCD driver template in INDI SVN (under 3rdparty).
*/

#include <sys/time.h>
#include <memory>
#include <math.h>
#include "cam84_ccd.h"
#include "libcam84.h"

const int POLLMS           = 500;       /* Polling interval 500 ms */
const int MAX_CCD_TEMP     = 45;		/* Max CCD temperature */
const int MIN_CCD_TEMP	   = -55;		/* Min CCD temperature */
const float TEMP_THRESHOLD = .25;		/* Differential temperature threshold (C)*/

/* Macro shortcut to CCD temperature value */
#define currentCCDTemperature   TemperatureN[0].value

std::unique_ptr<Cam84CCD> simpleCCD(new Cam84CCD());

void ISGetProperties(const char *dev)
{
         simpleCCD->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
         simpleCCD->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
         simpleCCD->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
         simpleCCD->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
   INDI_UNUSED(dev);
   INDI_UNUSED(name);
   INDI_UNUSED(sizes);
   INDI_UNUSED(blobsizes);
   INDI_UNUSED(blobs);
   INDI_UNUSED(formats);
   INDI_UNUSED(names);
   INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
     simpleCCD->ISSnoopDevice(root);
}

Cam84CCD::Cam84CCD()
{
    InExposure = false;
}

/*******************************************************************************
** Client is asking us to set a new number
*******************************************************************************/
bool Cam84CCD::ISNewNumber(const char *dev, const char *name,
                         double values[], char *names[], int n)
{
    if (!strcmp(dev, getDeviceName()))
    {
        if (!strcmp(name, GainNP.name))
        {
            IUUpdateNumber(&GainNP, values, names, n);
            GainNP.s = IPS_OK;
            IDSetNumber(&GainNP, NULL);
            cameraSetGain   (GainN[0].value);
            IDMessage(getDeviceName(), "Cam84 set gain = %d",(int) GainN[0].value);
            return true;
        }

        if (!strcmp(name, OffsetNP.name))
        {
            IUUpdateNumber(&OffsetNP, values, names, n);
            OffsetNP.s = IPS_OK;
            IDSetNumber(&OffsetNP, NULL);
            cameraSetOffset (OffsetN[0].value);
            IDMessage(getDeviceName(), "Cam84 set offset = %d",(int) OffsetN[0].value);
            return true;
        }

        if (!strcmp(name, BaudrateNP.name))
        {
            IUUpdateNumber(&BaudrateNP, values, names, n);
            BaudrateNP.s = IPS_OK;
            IDSetNumber(&BaudrateNP, NULL);
            cameraSetBaudrate(BaudrateN[0].value);
            IDMessage(getDeviceName(), "Cam84 set baudrate = %d",(int) BaudrateN[0].value);
            return true;
        }

        if (!strcmp(name, LibftditimersNP.name))
        {
            IUUpdateNumber(&LibftditimersNP, values, names, n);
            LibftditimersNP.s = IPS_OK;
            IDSetNumber(&LibftditimersNP, NULL);
            cameraSetLibftdiTimers(LibftdilatencyN[0].value,LibftditimersN[0].value);
            IDMessage(getDeviceName(), "Cam84 set libftdi timers = %d",(int) LibftditimersN[0].value);
            return true;
        }

       if (!strcmp(name, LibftdilatencyNP.name))
        {
            IUUpdateNumber(&LibftdilatencyNP, values, names, n);
            LibftdilatencyNP.s = IPS_OK;
            IDSetNumber(&LibftdilatencyNP, NULL);
            cameraSetLibftdiTimers(LibftdilatencyN[0].value,LibftditimersN[0].value);
            IDMessage(getDeviceName(), "Cam84 set libftdi latency = %d",(int) LibftdilatencyN[0].value);
            return true;
        }

    }

    // If we didn't process anything above, let the parent handle it.
    return INDI::CCD::ISNewNumber(dev,name,values,names,n);
}

/*******************************************************************************
** Client is asking us to set a new switch
*******************************************************************************/

bool Cam84CCD::ISNewSwitch (const char *dev, const char *name,
                          ISState *states, char *names[], int n)
{
    return INDI::CCD::ISNewSwitch (dev, name, states, names,  n);
}


/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool Cam84CCD::Connect()
{


    // Let's set a timer that checks teleCCDs status every POLLMS milliseconds.
    SetTimer(POLLMS);
    //cameraSetBaudrate(80);
    cameraConnect();
    cameraSetBaudrate(80);
//    cameraSetOffset(100);
    IDMessage(getDeviceName(), "Cam84 connected successfully!");

    return true;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool Cam84CCD::Disconnect()
{

    cameraDisconnect();
    return true;
    IDMessage(getDeviceName(), "Cam84 disconnected successfully!");
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char * Cam84CCD::getDefaultName()
{
    return "Cam84";
}

/**************************************************************************************
** INDI is asking us to init our properties.
***************************************************************************************/
bool Cam84CCD::initProperties()
{
    // Must init parent properties first!
    INDI::CCD::initProperties();

    /*const short minGain = 0;
    const short maxGain = 63;
    const short minOffset = -127;
    const short maxOffset = 127;
    const short minBaudrate = 80;
    const short maxBaudrate = 240;*/

    /* Add Gain number property (gs) */
    IUFillNumber(GainN, "GAIN", "Gain", "%g", 0, 63, 1, 32);
    IUFillNumberVector(&GainNP, GainN, 1, getDeviceName(),"GAIN",
                       "Gain", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    /* Add Offset number property (gs) */
    IUFillNumber(OffsetN, "OFFSET", "Offset", "%g", -127, 127, 1, 0);
    IUFillNumberVector(&OffsetNP, OffsetN, 1, getDeviceName(),"OFFSET",
                       "Offset", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    /* Add Baudrate number property (gs) */
    IUFillNumber(BaudrateN, "BAUDRATE", "Baudrate", "%g", 80, 150, 10, 150);
    IUFillNumberVector(&BaudrateNP, BaudrateN, 1, getDeviceName(),"BAUDRATE",
                       "Baudrate", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    /* Add Latency number property (gs) */
    IUFillNumber(LibftdilatencyN, "LATENCY", "Latency", "%g", 2, 100, 2, 2);
    IUFillNumberVector(&LibftdilatencyNP, LibftdilatencyN, 1, getDeviceName(),"LATENCY",
                       "Latency", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    /* Add Tiemrs number property (gs) */
    IUFillNumber(LibftditimersN, "TIMERS", "Timers", "%g", 1000, 50000, 1000, 2000);
    IUFillNumberVector(&LibftditimersNP, LibftditimersN, 1, getDeviceName(),"TIMERS",
                       "Timers", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);




    // We set the CCD capabilities
    uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_BAYER;
    SetCCDCapability(cap);

    // Add Debug, Simulator, and Configuration controls
    addAuxControls();

    return true;

}

/********************************************************************************************
** INDI is asking us to update the properties because there is a change in CONNECTION status
** This fucntion is called whenever the device is connected or disconnected.
*********************************************************************************************/
bool Cam84CCD::updateProperties()
{                //cameraSetBaudrate(80);
    // Call parent update properties first
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        // Let's get parameters now from CCD
        setupParams();

        // Start the timer
        SetTimer(POLLMS);
        defineNumber(&GainNP);
        defineNumber(&OffsetNP);
        defineNumber(&BaudrateNP);
        defineNumber(&LibftditimersNP);
        defineNumber(&LibftdilatencyNP);
    }
    else
    {
        deleteProperty(GainNP.name);
        deleteProperty(OffsetNP.name);
        deleteProperty(BaudrateNP.name);
        deleteProperty(LibftditimersNP.name);
        deleteProperty(LibftdilatencyNP.name);
    }

    return true;
}

/**************************************************************************************
** Setting up CCD parameters
***************************************************************************************/
void Cam84CCD::setupParams()
{
    // Our CCD is an 8 bit CCD, 1280x1024 resolution, with 5.4um square pixels.
    SetCCDParams(3000, 2000, 16, 7.8, 7.8);

    // Let's calculate how much memory we need for the primary CCD buffer
    int nbuf;
    nbuf=PrimaryCCD.getXRes()*PrimaryCCD.getYRes() * PrimaryCCD.getBPP()/8;
    nbuf+=512;                      //  leave a little extra at the end
    PrimaryCCD.setFrameBufferSize(nbuf);
}

/**************************************************************************************
** Client is asking us to start an exposure
***************************************************************************************/
bool Cam84CCD::StartExposure(float duration)
{


    ExposureRequest=duration;
    IDMessage(getDeviceName(), "Start exposure %g",duration);
    // Since we have only have one CCD with one chip, we set the exposure duration of the primary CCD
    PrimaryCCD.setExposureDuration(duration);
    //cameraStartExposure(1,0,0,3000,2000, duration,true);
    int r = cameraStartExposure(1,0,0,3000,2000, duration, false);
    //int r = cameraStartExposure(1,0,0,3000,2000, 0.4, true);
    gettimeofday(&ExpStart,NULL);

    InExposure=true;

    // We're done
    return true;
}

/**************************************************************************************
** Client is asking us to abort an exposure
***************************************************************************************/
bool Cam84CCD::AbortExposure()
{
    InExposure = false;
    return true;
}

/**************************************************************************************
** Client is asking us to set a new temperature
***************************************************************************************/
int Cam84CCD::SetTemperature(double temperature)
{
    TemperatureRequest = temperature;

    // 0 means it will take a while to change the temperature
    return 0;
}

/**************************************************************************************
** How much longer until exposure is done?
***************************************************************************************/
float Cam84CCD::CalcTimeLeft()
{
    double timesince;
    double timeleft;
    struct timeval now;
    gettimeofday(&now,NULL);

    timesince=(double)(now.tv_sec * 1000.0 + now.tv_usec/1000) - (double)(ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec/1000);
    timesince=timesince/1000;

    timeleft=ExposureRequest-timesince;
    return timeleft;
}

/**************************************************************************************
** Main device loop. We check for exposure and temperature progress here
***************************************************************************************/
void Cam84CCD::TimerHit()
{
    long timeleft;

    if(isConnected() == false)
        return;  //  No need to reset timer if we are not connected anymore

    if (InExposure)
    {
        timeleft=CalcTimeLeft();

        // Less than a 0.1 second away from exposure completion
        // This is an over simplified timing method, check CCDSimulator and simpleCCD for better timing checks
        if(timeleft < 0.1)
        {
          /* We're done exposing */
           IDMessage(getDeviceName(), "Exposure done, downloading image...");

          // Set exposure left to zero
          PrimaryCCD.setExposureLeft(0);

          // We're no longer exposing...
          InExposure = false;

          /* grab and save image */
          grabImage();

        }
        else
         // Just update time left in client
         PrimaryCCD.setExposureLeft(timeleft);

    }

    // TemperatureNP is defined in INDI::CCD
    switch (TemperatureNP.s)
    {
      case IPS_IDLE:
      case IPS_OK:
        break;

      case IPS_BUSY:
        /* If target temperature is higher, then increase current CCD temperature */
        if (currentCCDTemperature < TemperatureRequest)
           currentCCDTemperature++;
        /* If target temperature is lower, then decrese current CCD temperature */
        else if (currentCCDTemperature > TemperatureRequest)
          currentCCDTemperature--;
        /* If they're equal, stop updating */
        else
        {
          TemperatureNP.s = IPS_OK;
          IDSetNumber(&TemperatureNP, "Target temperature reached.");

          break;
        }

        IDSetNumber(&TemperatureNP, NULL);

        break;

      case IPS_ALERT:
        break;
    }

    SetTimer(POLLMS);
    return;
}

/**************************************************************************************
** Create a random image and return it to client
***************************************************************************************/
void Cam84CCD::grabImage()
{
   // Let's get a pointer to the frame buffer
    uint8_t * image = PrimaryCCD.getFrameBuffer();
    int width = PrimaryCCD.getSubW() / PrimaryCCD.getBinX()   * (PrimaryCCD.getBPP() / 8);
    int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();
    //int width = PrimaryCCD.getSubW() / PrimaryCCD.getBinX() * (PrimaryCCD.getBPP() / 8);
    //int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY() * (PrimaryCCD.getBPP() / 8);


    IDMessage(getDeviceName(), "grabimage width=%d height=%d BPP=%d\n", width, height, PrimaryCCD.getBPP() );

   // Fill buffer with random pattern
   while (!cameraGetImageReady() ); // waiting image

   for (int j=0; j < height ; j++)
     for (int i=0; i < width/2; i++)
         {
            uint16_t pix = cameraGetImage(i,j);
            uint8_t hibyte = (pix & 0xff00) >> 8;
            uint8_t lobyte = (pix & 0xff);
            image[2*i+  j*width] = hibyte;
            image[2*i+1+j*width] = lobyte;
         };

   /*for (int j=0; j < height ; j++)
     for (int i=0; i < width; i++)
         {
            image[i+  j*width] = cameraGetImage(i,j)/256;
         };*/
   //cameraGetImage2(image);
   //*bufim = (unsigned short *) image;
   //image = *bufim;
   //PrimaryCCD.setFrameBuffer(bufim);
   //cameraGetImage(image);
     IDMessage(getDeviceName(), "Download complete.");

   // Let INDI::CCD know we're done filling the image buffer
   ExposureComplete(&PrimaryCCD);
}


/*

int main(void)
{
    cameraConnect();
    cameraSetBaudrate(80);
    cameraSetOffset(0);
    cameraSetGain(0);
    int r = cameraStartExposure(1,0,0,3000,2000, 1, false);
    while (!cameraGetImageReady())
//        fprintf(stdout,"wait\n");
      ; // waiting image
    //for (int i=0; i < 10 ; i++)
    //  for (int j=0; j < 10; j++)
    //      fprintf(stdout,"img %d/%d:%d\n",i,j,cameraGetImage(i,j));
    cameraDisconnect();

}

*/

