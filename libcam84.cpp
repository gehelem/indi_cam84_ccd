// define which ftdi library to use
// if LIBFTDI is defined using ifdef LIBFTDI

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include "libcam84.h"
#include "config.h"
#include <unistd.h>
#include <indiccd.h>
#include <cam84_ccd.h>
#include <indilogger.h>
#include <indiccd.h>
#include <atomic>

//#include <libusb-1.0/libusb.h>

#include <mutex>


std::mutex m;

#ifdef LIBFTDI
#include <ftdi.h>
#else
#include "ftd2xx.h"
#endif


struct timeval  curTime;



// /*?????? ???????????*/
const int CameraWidth = 3000;
/*?????? ???????????*/
const int CameraHeight = 2000;
/*?????????????? ???????? ?? ??????? ????? BDBUS*/
const int xccd = 1500;
const int yccd = 1000;
const int dx = 44;//3044-2*xccd;
const int dx2 = 86;//1586-xccd;
const int dy = 12;//512-(yccd / 2);
//const int apolosa = 50;

//const short minBaudrate = 10;
//const short maxBaudrate = 3000;

/*camera state consts*/
const int cameraIdle = 0;
const int cameraWaiting = 1;
const int cameraExposing = 2;
const int cameraReading = 3;
const int cameraDownload = 4;
const int cameraError = 5;

#ifdef LIBFTDI
struct ftdi_context *CAM8A, *CAM8B;
#else
FT_HANDLE CAM8A, CAM8B;
FT_STATUS ftStatus = FT_OK;
uint32_t   bytesWritten = 0;
uint32_t   dwBytesRead  = 0;
uint32_t   numDevs = 0;
char  *PORTA = (char *) "cam8 A";
char  *PORTB = (char *) "cam8 B";
char * BufPtrs[3];
char Buffer1[64];
char Buffer2[64];
char Buffer3[64];
#endif

int ftdi_result;
int rfstatus =0;
double durat;
//int x;
//struct ftdi_context *ftdi, *ftdi2bool;
//int f,i;
unsigned char buf[1];
//??????????-????, ?????????? ????????? ?????????? ? ???????
bool isConnected  = false;
//??????????-????, ??????????, ????? ????? ?????????? ??????????
bool canStopExposureNow = true;
//bool canRead = false;

static std::atomic_bool canWrite = {false};  //Should this be protected by

//????????? ???????? ?????? ? ???????? ?????? FT2232HL
int adress;
//???????,
int mBin;
//??????????-????, ?????????? ?????????? ? ?????????? ?????
bool imageReady = false;
//??????????-????????? ??????
int cameraState;
//?????? ?????????? ? 15? ??????
//?????????? ???????? ??????????-????? CanStopExposureCount
int checkCanStopExposureCount;
//?????????? ??? ??????? ?????? (?????? ???????????)
// thread co;//co: posl;
//std::thread co;
//void *posExecute(void *arg);
//void posExecute(void);
//???????? ??????-??????????? ??? ????????
static uint16_t bufim[3000][2000];
//static unsigned short bufim[CameraWidth][CameraHeight];
//?????? ?????? ? ?????????? ?? ???????
int mYn,mdeltY;
//?????? ?????? ? ?????????? ?? ????????
int mXn,mdeltX;
uint8_t zatv;
//error Flag
bool errorReadFlag;
bool errorWriteFlag;
//speed
int spusb;
int ms1;

//exposure start time
struct timespec exposureStartTime, currentTime, readStartTime;
double elapsedTime;

//struct timespec rdmStartTime, rdmCurrentTime;
//double rdmElapsedTime, rdmPreElapsedTime;


void AD9822 ( uint8_t adr , uint16_t val );
void HC595 ( uint8_t va );
//static uint16_t FT_In_Buffer[6000];
static uint8_t FT_In_Buffer[13000];
static uint8_t FT_Out_Buffer[500000];
int  FT_Current_Baud;
bool FT_OP_flag;

void clearline2 ( void );
void shift ( void );
void shift0 ( void );
void shift2 ( void );
void shift3 ( void );
void clearframe ( void );
void readframe ( int bin,int expoz );
void coexecute ( void );
void ComRead ( void );
#ifdef LIBFTDI
int ftdi_read_data_modified ( struct ftdi_context *ftdi, unsigned char *buf,  int size );
#else
int ftdi_read_data_modified (FT_HANDLE ftdi, unsigned char *buf,  int size );
#endif

//inline uint16_t  swap ( void * x );

//int ftStatus = 0;
//uint32_t   bytesWritten = 0;
//int32_t   dwBytesRead  = 0;

//int  getImageImax;
//int  getImageImin;
//int  getImageJmax;
//int  getImageJmin;

bool continougsADToggle = false;

const char * getDeviceName()
{
    return("libcam84");
}

bool cameraSetContinuousADToggle(bool continuousADToggleValue)
{
    continougsADToggle = continuousADToggleValue;
    fprintf(stderr,"continuousADToggle set to %d\n", continougsADToggle);
}




/* ????????? ????????? ?????? ? FT2232LH.*/
/*?????? ???????????? ????? ?????:*/
/*1. ??????? ??????????? ????? ? ????????? ??????? (??????????? ?????????????????? ????????? ?? ??????? ????? BDBUS).*/
/*??? ???? ???????????????? ????????? adress.*/
/*2. ????? ???? ???? ?????? ?????????? ?? ????? ????????: n:=Write_USB_Device_Buffer(FT_CAM8B,adress);*/
/*????????????? ?????????? FT2232HL ?????? ??? ???????? ??? ??? ???????? ?? ???? ???? BDBUS. ???????? 1 ????? ??? ???? ???????? 65 ??.*/
/*????? ????????? ????????? ??????? n:=Write_USB_Device_Buffer(FT_CAM8B,adress) ??????? ?? ????????????? ??????????? ? ?? ??????????????*/
/*????. ??????? ??????????? ?????????????????? ????????? ????? ????????? ???, ? ?? ?????????? ?? ???????.*/
/*????? ?????????? ????? ???????? ??? ????????? (? ???? ????????? ?? 24 ?????!) ??? ????? ????? ???????? ????? D2XX.pas, ? ?????? ??? MyD2XX.pas*/

/*?????????? ????????? ?????? ???????? ??? ???????? ? ?????????? ????? val ?? ?????? adr ? ?????????? AD9822.*/
/*???????? ???? ? ???????????????? ????.*/

/*AD9822 uses FT2232H USB interface to write commands to the serial port of the AD9826 by writing a */
/*parallel to serial conversion.  0x01 corresponds to SLOAD line, 0x02 corresponds to SCLK line, 0x04 corresponds to SDATA line */
/*SCLK and SDATA also used by HC595 but with SL2 */
void AD9822 ( uint8_t adr,uint16_t val )
{
    //    fprintf(stderr,"AD9822 %d %d\n",adr,val);
    int kol = 64;
    uint8_t dan[kol];
    int i;
    memset ( dan,0xE9,kol );		    //set all 64 elements of dan to 0xE9			1110 1001
    for ( i = 1; i <= 32; i++ )
    {								//element 0, still 0xE9
        dan[i]=dan[i] & 0xFE;			//sets elements 1 to 32 to 0xE8 = 0xE9 & 0xFE	1110 1000
        //brings SL (SLOAD) low, i.e., active
    };								//elements 33 to 63 and 0 are still 0xE9
    for ( i = 0; i <= 15; i++ )
    {
        dan[2*i+2]=dan[2*i+2] + 2 ;	//sets elements 2, 4, ... 32 to 0xEA			1110 1010
    };								//toggles SCK (SCLK) 16 times

    //convert adr to a 3 bit serial stream
    if ( ( adr & 4 ) ==4 )
    {
        dan[3] =dan[3]+4;				//A2 bit										1110 1100
        dan[4] =dan[4]+4;				//												1110 1110
    };
    if ( ( adr & 2 ) ==2 )
    {
        dan[5] =dan[5]+4;				//A1 bit
        dan[6] =dan[6]+4;
    };
    if ( ( adr & 1 ) ==1 )
    {
        dan[7] =dan[7]+4;				//A0 bit
        dan[8] =dan[8]+4;
    };

    //convert value to serial 9 bit stream
    if ( ( val & 256 ) ==256 )
    {
        dan[15]=dan[15]+4;
        dan[16]=dan[16]+4;
    };
    if ( ( val & 128 ) ==128 )
    {
        dan[17]=dan[17]+4;
        dan[18]=dan[18]+4;
    };
    if ( ( val &  64 ) ==64 )
    {
        dan[19]=dan[19]+4;
        dan[20]=dan[20]+4;
    };
    if ( ( val &  32 ) ==32 )
    {
        dan[21]=dan[21]+4;
        dan[22]=dan[22]+4;
    };
    if ( ( val &  16 ) ==16 )
    {
        dan[23]=dan[23]+4;
        dan[24]=dan[24]+4;
    };
    if ( ( val &   8 ) ==8 )
    {
        dan[25]=dan[25]+4;
        dan[26]=dan[26]+4;
    };
    if ( ( val &   4 ) ==4 )
    {
        dan[27]=dan[27]+4;
        dan[28]=dan[28]+4;
    };
    if ( ( val &   2 ) ==2 )
    {
        dan[29]=dan[29]+4;
        dan[30]=dan[30]+4;
    };
    if ( ( val &   1 ) ==1 )
    {
        dan[31]=dan[31]+4;
        dan[32]=dan[32]+4;
    };

    //for (int xx=0;xx<sizeof(dan);xx++) fprintf(stderr,"%X ",dan[xx]);
    //fprintf(stderr,"\n ");
    fprintf(stderr,"0) ftdi_write_data with AD command size of %lu\n", sizeof ( dan ));
#ifdef LIBFTDI
    if ( ftdi_write_data ( CAM8B, dan, sizeof ( dan ) ) < 0 )  //Send command to serial port of AD9822
#else
    if ( FT_Write( CAM8B, dan, sizeof ( dan ), &bytesWritten ) != FT_OK)
#endif
    {
        fprintf ( stderr,"write failed on channel 2)\n" );
    }
}

/*?????????? ????????? ?????? ???????? ??? ???????? ????? val ?? ?????? ?????????? HC595.*/
/*???????? ???? ? ???????????????? ????.*/

/*HC595 stores commands in FT_OutBuffer for use with */
/*FT2232H USB interface to write commands to the serial port of the 74HC595 by a */
/*parallel to serial conversion.  74HC595 outputs as a parallel byte*/
/*0x80 corresponds to SL2 line, 0x02 corresponds to SCLK line, 0x04 corresponds to SDATA line */
/*SCLK and SDATA also used by HC595 but with SLOAD */
/*74HC595 drives CXD1267 */
void HC595 ( uint8_t va )
{
    //fprintf(stderr,"HC595 %d \n",adress);
    int kol = 20;
    uint8_t dan[kol];
    int i;
    uint16_t val;

    memset ( dan,0xE9,kol );		//set all 20 elements of dan to 0xE9			1110 1001
    //for (i = 0;i < kol ;i++) dan[i]=0xE9;
    if ( zatv == 1 )
    {
        val=va+0x0100;
    }
    else
    {
        val=va ;
    }
    for ( i = 0; i <= 8; i++ )
    {
        dan[2*i+1]=dan[2*i+1] +2 ;
        if ( ( val & 0x100 ) ==0x100 )		//High order bit is sent out first
        {
            dan[2*i  ]=dan[2*i]   + 4;
            dan[2*i+1]=dan[2*i+1] + 4;
        }
        val=val*2;
    };
    dan[18]=dan[18]+ 0x80;					//Toggle SL2 in dan18, latch data on rising edge at dan[19]
    for ( i = 0; i <= kol-1; i++ )
    {
        FT_Out_Buffer[2*i+adress]  =dan[i];	//adress is pointer into FT_Out_Buffer
        FT_Out_Buffer[2*i+adress+1]=dan[i];	//two copies of each dan so data goes out half as fast??
    }
    adress=adress+2*kol;						//increment adress to point to next open space in FT_Out_Buffer
}

// /* Fast version of shift() but without overlap, not used? */
// void shift0()
// {
// // printf("shift0\n");
// HC595 ( 0xDB );  // 1101 1011
// HC595 ( 0xFA );  // 1111 1010
// HC595 ( 0xEE );  // 1110 1110
// HC595 ( 0xCF );  // 1100 1111
// }

/*?????????? ????????? ?????? ???????? ??? ?????? ???? ????????????? ??????*/
void shift()		//Shifts vertically one line
{
    //printf("shift\n");
    HC595 ( 0xCB );  // 1100 1011
    HC595 ( 0xDB );  // 1101 1011
    HC595 ( 0xDA );  // 1101 1010
    HC595 ( 0xFA );  // 1111 1010
    HC595 ( 0xEA );  // 1110 1010
    HC595 ( 0xEE );  // 1110 1110
    HC595 ( 0xCE );  // 1100 1110
    HC595 ( 0xCF );  // 1100 1111
}

/*?????????? ????????? ?????? ???????? ??? "?????" ???????????? ??????????? ? ????????? ???????*/
void shift2()		//Toggles XSG1 and XSG2 portions of vertical shift logic
{
    //printf("shift2\n");
    shift();
    HC595 ( 0xC7 );  // 1100 0111
    HC595 ( 0xC7 );  // 1100 0111
    HC595 ( 0xC7 );  // 1100 0111
    HC595 ( 0xC7 );  // 1100 0111
    HC595 ( 0xCB );  // 1100 1011
    HC595 ( 0xD9 );  // 1101 1001
    HC595 ( 0xD9 );  // 1101 1001
    HC595 ( 0xD9 );  // 1101 1001
    HC595 ( 0xD9 );  // 1101 1001
    HC595 ( 0xDB );  // 1101 1011
    HC595 ( 0xFA );  // 1111 1010
    HC595 ( 0xEA );  // 1110 1010
    HC595 ( 0xEE );  // 1110 1110
    HC595 ( 0xCE );  // 1100 1110
    HC595 ( 0xCF );  // 1100 1111
}

/*?????????? ????????? ?????? ???????? ??? ?????? ???? ????????????? ?????? + ?????? SUB ??? ?????? ??????? ???????????*/
void shift3()			//Similar to shift() but also toggles XSHT -> SUB -> VSUB on ICX453 CCD
{
    HC595 ( 0xCB );  // 1100 1011
    HC595 ( 0xDB );  // 1101 1011
    /*SUB*/
    HC595 ( 0x9A );  // 1001 1010
    HC595 ( 0xBA );  // 1011 1010
    HC595 ( 0xAA );  // 1010 1010
    HC595 ( 0xEE );  // 1110 1110
    HC595 ( 0xCE );  // 1100 1110
    HC595 ( 0xCF );  // 1100 1111
}

void clearline2()
{
    int x;
    //  for ( x=0; x <= 79*xccd; x ++ )  		//xccd = 1500 constant
    for ( x=0; x <= 8*xccd; x ++ )  		//xccd = 1500 constant
    {									//appends 4*(79*1500)+1 elements to FT_Out_Buffer
        FT_Out_Buffer[adress+0]=0xD9;		//0xD9  1101 1001
        FT_Out_Buffer[adress+1]=0x99;		//0x99	1001 1001
        FT_Out_Buffer[adress+2]=0xA9;		//0xA9	1010 1001
        FT_Out_Buffer[adress+3]=0xE9;		//0xE9	1110 1001
        adress += 4;
    }
}

//Tickles clocks for AD without causing any shifts
void preCharge(int nCycles)
{
    int x;
    for ( x=0; x <= nCycles; x ++ )  				//xccd = 1500 constant
    {											//appends ncycles elements to FT_Out_Buffer
        FT_Out_Buffer[adress+0]=0xD9;		        //0xD9  1101 1001
        FT_Out_Buffer[adress+1]=0xD9;			    //0xD9	1101 1001
        FT_Out_Buffer[adress+2]=0xD9;			    //0xD9	1101 1001
        FT_Out_Buffer[adress+3]=0xE9;		        //0xE9	1110 1001
        FT_Out_Buffer[adress+4]=0xE9;			    //0xE9	1110 1001
        FT_Out_Buffer[adress+5]=0xE9;			    //0xE9	1110 1001
        adress += 6;
    }
}


/*?????????? ????????? ?????? ???????? ???:*/
/*??????? ?????????? ????????. ???? ??? ?? ????????,*/
/*?? ??????????? ? ??? ????? ????? ???????? ? ???????????.*/
/*????????? ???? ??????? ?????? ? "???????" ? ??????????????? ????????.*/
/*???????? ?????????? ????? "??????" ??????????? ? ????????? ???????*/
void clearframe()
{
    fprintf(stderr,"clearframe started\n");

    uint16_t y;
    uint16_t x;

    // fprintf(stderr,"Previous imin = %d, imax = %d, jmin = %d, jmax = %d for getImage(i,j)\n",getImageImin, getImageImax, getImageJmin, getImageJmax);

    //getImageImax = -1;
    //getImageImin = 99999;
    //getImageJmax = -1;
    //getImageJmin = 999999;

    // for ( x=0; x < 3000; x ++) {	  //clear image buffer (or set to test value)
    //   for ( y=0; y < 2000; y ++ ){
    //		bufim[x][y] = 0;
    //	}
    // }



    clearline2();
    for ( x=0; x < 1; x ++) {  //See if multiple clears help
        for ( y=0; y < 1012; y ++ )  //calls shift 1012 times followed by clearline2
        {
            shift();
        }
        clearline2();
    }

    fprintf(stderr,"clearframe complete\n");
}

/* ?????? ?????????????? ?????????? ?????? FT2232HL ? ???????? ?????? ???????????*/
/*??-?? ???????????? AD9822 ????????? ??????? ??????? ????, ????? ???????, ? ? delphi ????????.*/
/*?????????? ????? ??? int32, ? ?? word16 ??-?? ???????????? ??? ??????????? ?????????*/

/*?????????? ???? ?????? ??????? ????? ?? ADBUS*/
void *posExecute ( void *arg )	//Thread assembles data read from AD and places it in the image buffer
{
    //printf("poseExecute\n");
    uint16_t x,y,x1,y1,byteCnt,x_int,y_int;
    uint32_t bytesRead;
    uint8_t * FT_In_BufferP;
    bool readFailed;
    fprintf(stderr,"poseexecute mYn %d mdeltY %d mXn %d mdeltX %d \n",mYn,mdeltY,mXn,mdeltX);
    clock_gettime(CLOCK_MONOTONIC, &readStartTime);

    fprintf(stderr,"poseexecute y : %d < %d x : %d>%d\n",mYn,mYn+mdeltY,0,mdeltX );
    //for (x=0;x<13000;x++) FT_In_Buffer[x]=0;
    //canWrite=true;
    readFailed=false;
    byteCnt=0;
    for ( y= 0; y < mdeltY; y ++ )
    {
        if ( mBin == 1 )
        {
            //if (y<5 | y> 995)   fprintf(stderr,"reading : read before %d\n",y);
            //while (!canRead);
            /*{
          gettimeofday(&curTime, NULL);
          fprintf(stderr,"wait ... %d %d\n",curTime.tv_sec,curTime.tv_usec);
               };*/
            //canWrite=false;

            //gettimeofday(&curTime, NULL);
            //fprintf(stderr,"Time pre-read  %ld %ld\n",curTime.tv_sec,curTime.tv_usec);

            bytesRead=ftdi_read_data_modified ( CAM8A, FT_In_Buffer, 8*mdeltX );

            //gettimeofday(&curTime, NULL);
            //fprintf(stderr,"Time pos-tread %ld %ld\n",curTime.tv_sec,curTime.tv_usec);

            //canWrite=true;
            //if (y<5 | y> 995)   fprintf(stderr,"reading : read after  %d (%d)\n",y,8*mdeltX);
            if ( bytesRead < 0 )
            {
                fprintf ( stderr,"FT_Read failed (%d)\n",bytesRead );
            }

            if ( bytesRead!=8*mdeltX )
            {
                readFailed=true;
                fprintf ( stderr,"poseExecute bin==1 readfailed %d<>%d - %d / %d \n",bytesRead,8*mdeltX,y,mYn+mdeltY-1 );

                //break;
            }

            for ( x=0; x < mdeltX; x ++ )
            {
                x1=x+mXn;

                x_int=x1+x1;
                y1 = y + mYn;
                y_int=y1 + y1;
                FT_In_BufferP = FT_In_Buffer + 8*x;
                bufim[x_int][y_int]    = * (uint16_t *) (FT_In_BufferP);
                bufim[x_int][y_int+1]  = * (uint16_t *) (FT_In_BufferP + 2) ;
                x_int++;
                bufim[x_int][y_int+1]  = * (uint16_t *) (FT_In_BufferP + 4) ;
                bufim[x_int][y_int]    = * (uint16_t *) (FT_In_BufferP + 6) ;

            }
        }
        else
        {
            fprintf(stderr, "ERROR--Binning is not supported in this version of the driver \n");
        }
    }
    /*if (readFailed)
  {
    errorReadFlag = true;
    if (~ errorWriteFlag)  ftdi_usb_purge_rx_buffer(CAM8A);
    if (~ errorWriteFlag)  ftdi_usb_purge_tx_buffer(CAM8B);
  }*/

    //TODO NEEDED???????????????????
#ifdef LIBFTDI
    ftdi_usb_purge_rx_buffer ( CAM8A );
#else
    if ( FT_Purge( CAM8A, FT_PURGE_RX ) != FT_OK )
    {
        fprintf ( stderr,"ftd2xx error purge interface A\n" );
    }

#endif

    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    elapsedTime = (currentTime.tv_sec - readStartTime.tv_sec) + (currentTime.tv_nsec - readStartTime.tv_nsec)*1.0e-9;
    fprintf(stderr,"Read Elapsed Time %f\n",elapsedTime);

    //ftdi_usb_purge_tx_buffer(CAM8B);
    //ftdi_usb_purge_rx_buffer(CAM8B);
    //ftdi_usb_purge_tx_buffer(CAM8A);
    ( void ) arg;
    pthread_exit ( 0 );
}


/*???????????? 2 ??????:*/
/*1.??????? ??? ???????.*/
/*2.?/? ? ???????? 2*2.*/
/*???????????? ??????? ICX453 ???????? ??, ??? ?????????????? ??????? ????? ????????? ??????? ?*/
/*??? ????? ???? ????????????? ?????? ? ?????????????? ??????? "??????" ????? ???? ?????,*/
/*??????? ?????????? ????? ??? ???? ???? ???????? ??????????.*/
/*???????????? ????????:*/
/*readframe, display, display2 - ??? 1 ??????,*/
/*readframe2, display3, display4 - ??? 2 ??????*/
/*?????????? ????????? ?????? ???????? ? ?????????? ???? ???????? ?????? ????? ? 1 ??????*/
void readframe ( int bin,int expoz )
{
    rfstatus = 0;
    fprintf ( stderr,"readframe(bin: %d, expoz: %d) : begin\n", bin, expoz );
    int x,y;
    cameraState = cameraReading;
    //if (~ errorWriteFlag)  ftdi_usb_purge_rx_buffer(CAM8A);
    //if (~ errorWriteFlag)  ftdi_usb_purge_tx_buffer(CAM8B);

    // fprintf ( stderr,"Pre-charging : begin\n" );
    // adress=0;
    // preCharge(100000);
    // for ( x=0; x<100; x++) {
    // if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
    // {
    // fprintf ( stderr,"write failed on channel 2)\n" );
    // }
    // }
    // fprintf ( stderr,"Pre-charging : done\n" );

    adress=0;

    if ( expoz > 52 )
    {

        if ( expoz < 500 )
        {
            clearframe();
            shift3();
            for ( y=0; y <= expoz-52; y ++ )
                for ( x=0; x <= ms1-1; x ++ )
                {
                    HC595 ( 0xCF );
                }
        }

        clearframe();
    }
    else
    {
        //     fprintf(stderr,"ExpozElse\n");

        clearframe();
        shift3();
        if ( expoz > 0 )
        {
            //        fprintf(stderr,"ExpozElse\n");
            for ( y=0; y <= expoz; y ++ )
                for ( x=0; x <= ms1-1; x ++ )
                {
                    HC595 ( 0xCF );
                }
        }
    }


    shift2();
    fprintf(stderr,"1) ftdi_write_data with adress size of %d\n", adress);
    //  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#ifdef LIBFTDI
    if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#else
    if ( FT_Write( CAM8B, FT_Out_Buffer, adress, &bytesWritten ) != FT_OK)
#endif
    {
        fprintf ( stderr,"write failed on channel 2)\n" );
    }
    adress=0;
    fprintf(stderr,"shifting %d + %d times\n", dy, mYn);
    for ( y=0; y < dy+mYn; y ++ )  //Shift by dy (some constant and by number of lines to skip
    {
        shift();
    }
    clearline2();
    fprintf(stderr,"2) ftdi_write_data with adress size of %d\n", adress);
    //  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#ifdef LIBFTDI
    if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#else
    if ( FT_Write( CAM8B, FT_Out_Buffer, adress, &bytesWritten ) != FT_OK)
#endif
    {
        fprintf ( stderr,"write failed on channel 2)\n" );
    }
    adress=0;
    //canRead=false;
    pthread_t t1;
    pthread_create ( &t1, NULL, posExecute, NULL );
    //pthread_detach(t1);

    fprintf( stderr,"mdeltX: %6d, mdeltaY: %6d, mXn %6d, mYn %6d, dx %6d, dy %6d, dx2 %6d  \n", mdeltX, mdeltY, mXn, mYn, dx, dy, dx2);
    fprintf( stderr,"L1: %6d, L3: %6d, L4 %6d \n\n", dx+4*mXn, 4*mdeltX-2, dx2+6000-4*mdeltX-4*mXn);

    //  for ( y=0; y < mdeltY ; y ++ )					//Loops number of ypixels/2 in requested image
    //    {
    adress=0;
//    clearline2();  //See if this helps  NO MUCH WORSE
    shift();										//Shift one vertical line
    for ( x=0; x < dx+4*mXn; x ++ )			//L1
    {
        FT_Out_Buffer[adress+0]=0xD9;       // 1101 1001
        FT_Out_Buffer[adress+1]=0x99;       // 1001 1001    // Horizontal Shift
        FT_Out_Buffer[adress+2]=0xA9;       // 1010 1001
        FT_Out_Buffer[adress+3]=0xE9;       // 1110 1001
        adress += 4;
    }
    if ( bin == 1 )
    {
        for ( x=0; x <= 4; x ++ )
        {
            FT_Out_Buffer[adress+0]=0xD9;       // 1101 1001
            FT_Out_Buffer[adress+1]=0x99;       // 1001 1001    // Horizontal Shift
            FT_Out_Buffer[adress+2]=0xA9;       // 1010 1001
            FT_Out_Buffer[adress+3]=0xE9;       // 1110 1001
            adress += 4;
        }

        FT_Out_Buffer[adress+0]=0xD9;           // 1101 1001
        FT_Out_Buffer[adress+1]=0x99;           // 1001 1001    // Horizontal Shift
        FT_Out_Buffer[adress+2]=0xA9;           // 1010 1001
        FT_Out_Buffer[adress+3]=0xE1;           // 1110 0001	//Toggle WR#1--Read A/D output
        adress += 4;
        for ( x=0; x <= 4*mdeltX-2; x ++ )				//L3
        {
            FT_Out_Buffer[adress+0]=0xD9;       // 1101 1001
            FT_Out_Buffer[adress+1]=0x99;       // 1001 1001    // Horizontal Shift
            FT_Out_Buffer[adress+2]=0x91;       // 1001 0001	//Toggle WR#1
            FT_Out_Buffer[adress+3]=0xA9;       // 1010 1001
            FT_Out_Buffer[adress+4]=0xE9;       // 1110 1001
            FT_Out_Buffer[adress+5]=0xE1;       // 1110 0001	//Toggle WR#1
            adress += 6;
        }
    }
    else
    {
        for ( x=0; x <= 4; x ++ )
        {
            FT_Out_Buffer[adress+0]=0xD9;
            FT_Out_Buffer[adress+1]=0x99;
            FT_Out_Buffer[adress+2]=0xD9;
            FT_Out_Buffer[adress+3]=0x99;
            FT_Out_Buffer[adress+4]=0xD9;
            FT_Out_Buffer[adress+5]=0x99;
            FT_Out_Buffer[adress+6]=0xD9;
            FT_Out_Buffer[adress+7]=0x99;
            FT_Out_Buffer[adress+8]=0xE9;
            FT_Out_Buffer[adress+9]=0xE9;
            adress += 10;
        }
        FT_Out_Buffer[adress+0]=0xD9;
        FT_Out_Buffer[adress+1]=0x99;
        FT_Out_Buffer[adress+2]=0xD9;
        FT_Out_Buffer[adress+3]=0x99;
        FT_Out_Buffer[adress+4]=0xD9;
        FT_Out_Buffer[adress+5]=0x99;
        FT_Out_Buffer[adress+6]=0xD9;
        FT_Out_Buffer[adress+7]=0x99;
        FT_Out_Buffer[adress+8]=0xE9;
        FT_Out_Buffer[adress+9]=0xE1;				//Toggle WR#1
        adress += 10;
        for ( x=0; x <= mdeltX-2; x ++ )
        {
            FT_Out_Buffer[adress+0]=0xD9;
            FT_Out_Buffer[adress+1]=0x91;			//Toggle WR#1
            FT_Out_Buffer[adress+2]=0xD9;
            FT_Out_Buffer[adress+3]=0x99;
            FT_Out_Buffer[adress+4]=0xD9;
            FT_Out_Buffer[adress+5]=0x99;
            FT_Out_Buffer[adress+6]=0xD9;
            FT_Out_Buffer[adress+7]=0x99;
            FT_Out_Buffer[adress+8]=0xE9;
            FT_Out_Buffer[adress+9]=0xE1;			//Toggle WR#1
            adress += 10;
        }
    }
    FT_Out_Buffer[adress+0]=0xD9;
    FT_Out_Buffer[adress+1]=0x91;					//Toggle WR#1
    FT_Out_Buffer[adress+2]=0xA9;
    FT_Out_Buffer[adress+3]=0xE9;
    adress += 4;
    for ( x=0; x <= dx2-1+6000-4*mdeltX-4*mXn; x ++ ) //L4   ??Shift through rest of line?
    {
        FT_Out_Buffer[adress+0]=0xD9;       // 1101 1001
        FT_Out_Buffer[adress+1]=0x99;       // 1001 1001    // Horizontal Shift
        FT_Out_Buffer[adress+2]=0xA9;       // 1010 1001
        FT_Out_Buffer[adress+3]=0xE9;       // 1110 1001
        adress += 4;
    }

    //if (y<5 | y> 995)   fprintf(stderr,"reading : write before %d\n",y);
    //while (!canWrite);
    //canRead=false;
    //gettimeofday(&curTime, NULL);
    //fprintf(stderr,"Time writ %d %d\n",curTime.tv_sec,curTime.tv_usec);
    //      if(y==0)
    fprintf(stderr,"3) ftdi_write_data with adress size of %d for %d times\n", adress, mdeltY);
    for ( y=0; y < mdeltY ; y ++ )					//Loops number of ypixels/2 in requested image
    {
        //      if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )

//        while (!canWrite)
        {
//            usleep(0);
        }
//        usleep(10000);

#ifdef LIBFTDI
        if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#else
        if ( FT_Write( CAM8B, FT_Out_Buffer, adress, &bytesWritten ) != FT_OK)
#endif
        {
            fprintf ( stderr,"write failed on channel 2)\n" );
        }
        canWrite = false;
        //     fprintf(stderr,"Prepare yLine %d\n",y);
        //canRead=true;
        //usleep(10);
        //canRead=trcdue;
        //if (y<5 | y> 995)   fprintf(stderr,"reading : write after  %d (%d) \n",y,adress);

    }
    pthread_join ( t1,NULL );
    imageReady = true;
    cameraState=cameraIdle;

}

void *ExposureTimerTick ( void *arg )  /*stdcall;*/
{
    uint32_t dd;
    dd = ( durat*1000-52 ) *1000;
    //  usleep ( dd );
    elapsedTime = -1.0;
    while(elapsedTime < durat)  //may need to subtract an offset
    {
        //   fprintf(stderr,">>>ExposureTimerTick\n");
        if(continougsADToggle)
        {
            adress = 0;
            preCharge(10000);
            for (int x=0; x<1; x++) {
                fprintf(stderr,"4) ftdi_write_data with adress size of %d\n", adress);
                //            if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#ifdef LIBFTDI
                if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#else
                if ( FT_Write( CAM8B, FT_Out_Buffer, adress, &bytesWritten ) != FT_OK)
#endif
                {
                    fprintf ( stderr,"write failed on channel 2)\n" );
                }
            }
        }
        else
        {
            usleep(10000);
        }
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        elapsedTime = (currentTime.tv_sec - exposureStartTime.tv_sec) + (currentTime.tv_nsec - exposureStartTime.tv_nsec)*1.0e-9;

        //fprintf(stderr,"Elapsed Time: %f, Duration: %f \n", elapsedTime, durat);
    }

    clock_gettime(CLOCK_MONOTONIC, &currentTime);
    elapsedTime = (currentTime.tv_sec - exposureStartTime.tv_sec) + (currentTime.tv_nsec - exposureStartTime.tv_nsec)*1.0e-9;
    fprintf(stderr,"Elapsed Time: %f, Duration: %f \n", elapsedTime, durat);


    canStopExposureNow = false;
    adress=0;
    //on +15V
    HC595 ( 0xCF );
    fprintf ( stderr,"write exp tick, durat: %f, dd %d\n",durat,dd );
    fprintf(stderr,"5) ftdi_write_data with adress size of %d\n", adress);
    //  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#ifdef LIBFTDI
    if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#else
    if ( FT_Write( CAM8B, FT_Out_Buffer, adress, &bytesWritten ) != FT_OK)
#endif
    {
        fprintf ( stderr,"write failed on channel 2)\n" );
    }
    readframe ( mBin, 1000 ); //read frame after an additional second of exposure???
    canStopExposureNow = true;
    ( void ) arg;
    pthread_exit ( 0 );
}

//off +15V
void *Timer15VTick ( void *arg )  /*stdcall;*/
{
    usleep ( 1000*1000 );
    adress=0;
    HC595 ( 0x4F );
    fprintf ( stderr,"write 15V tick\n" );

    fprintf(stderr,"6) ftdi_write_data with adress size of %d\n", adress);
    //  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#ifdef LIBFTDI
    if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#else
    if ( FT_Write( CAM8B, FT_Out_Buffer, adress, &bytesWritten ) != FT_OK)
#endif
    {
        fprintf ( stderr,"write failed on channel 2)\n" );
    }
    canStopExposureNow = true;
    ( void ) arg;
    pthread_exit ( 0 );
}

/*Stop Exposure Timer, purge FTDI FT2232HL buffers and frame bufeer*/
void StopExposure()
{
    if ( ~ errorWriteFlag )
    {
#ifdef LIBFTDI
        ftdi_usb_purge_rx_buffer ( CAM8A );
#else
        FT_Purge( CAM8A, FT_PURGE_RX);
#endif
    }
    cameraState = cameraIdle;
    imageReady = true;
}

/*???????? ?????? 0,1 ??? ?? ?????????? 15 ???*/
void StopExposureTimerTick()     /*stdcall;*/
{
    if ( canStopExposureNow )
    {
        StopExposure();
    }
    checkCanStopExposureCount=checkCanStopExposureCount+1;
    if ( checkCanStopExposureCount==150 )  ;

}

/*Connect camera, return bool result*/
/*????? ???????????? ????????? ? ????????????? AD9822*/
bool cameraConnect()               /*stdcall; export;*/
{
    FT_OP_flag = true;

//    DEBUG(INDI::Logger::DBG_SESSION,"TEST DEBUG MESSAGE\n");

#ifdef LIBFTDI
    fprintf (stderr, "Using libftdi \n");
    CAM8A = ftdi_new();
    CAM8B = ftdi_new();
    int errorCode;
    if ( ftdi_set_interface ( CAM8A, INTERFACE_A ) <0 )
    {
        fprintf ( stderr,"libftdi error set interface A\n" );
    }
    if ( ftdi_set_interface ( CAM8B, INTERFACE_B ) <0 )
    {
        fprintf ( stderr,"libftdi error set interface B\n" );
    }
    errorCode = ftdi_usb_open ( CAM8A, 0x0403, 0x6010 );
    if (errorCode  <0 )
    {
        fprintf ( stderr,"libftdi error open interface A (%d:%s)\n", errorCode, ftdi_get_error_string ( CAM8A ));
    }
    errorCode = ftdi_usb_open ( CAM8B, 0x0403, 0x6010 );
    if ( errorCode <0 )
    {
        fprintf ( stderr,"libftdi error open interface B (%d:%s)\n", errorCode, ftdi_get_error_string ( CAM8B ));
    }

    // BitBang channel 2
    if ( ftdi_set_bitmode ( CAM8B, 0xFF, BITMODE_BITBANG ) <0 )
    {
        fprintf ( stderr,"libftdi error setting asychronous bitbang mode interface B\n" );
    }
//    fprintf(stderr,"BITMODE_BITBANG const %d \n",BITMODE_BITBANG);

    // Baudrate
    cameraSetBaudrateDivisor ( CAM84_BAUDRATE_DIVISOR );

    //timeouts - latency
    cameraSetLibftdiTimers ( CAM84_LATENCYA,CAM84_LATENCYB,CAM84_TIMERA,CAM84_TIMERB );

    fprintf ( stderr,"libftdi interface A read chunksize %u\n",CAM8A->readbuffer_chunksize );
    fprintf ( stderr,"libftdi interface B read chunksize %u\n",CAM8B->readbuffer_chunksize );
    fprintf ( stderr,"libftdi interface A write chunksize %u\n",CAM8A->writebuffer_chunksize );
    fprintf ( stderr,"libftdi interface B write chunksize %u\n",CAM8B->writebuffer_chunksize );

    if (ftdi_read_data_set_chunksize (CAM8A,1024*4)<0 )
    {
        fprintf(stderr,"libftdi error set chunksize A\n");
    }

    if ( ftdi_write_data_set_chunksize ( CAM8B,1024*4 ) <0 )
    {
        fprintf ( stderr,"libftdi error set chunksize B\n" );
    }

    //Purge
    if ( ftdi_usb_purge_rx_buffer ( CAM8A ) <0 )
    {
        fprintf ( stderr,"libftdi error purge RX interface A\n" );
    }
    if ( ftdi_usb_purge_tx_buffer ( CAM8A ) <0 )
    {
        fprintf ( stderr,"libftdi error purge TX interface A\n" );
    }
    if ( ftdi_usb_purge_rx_buffer ( CAM8B ) <0 )
    {
        fprintf ( stderr,"libftdi error purge RX interface B\n" );
    }
    if ( ftdi_usb_purge_tx_buffer ( CAM8B ) <0 )
    {
        fprintf ( stderr,"libftdi error purge TX interface B\n" );
    }
#else
    fprintf (stderr, "Using d2xx \n");
    //Set up using FTD2xx
    BufPtrs[0] = Buffer1;
    BufPtrs[1] = Buffer2;
    BufPtrs[2] = NULL;     // last entry should be null

    if(FT_ListDevices(BufPtrs, &numDevs, FT_LIST_ALL | FT_OPEN_BY_DESCRIPTION ) != FT_OK)
    {
        fprintf ( stderr,"FTD2xx FT_ListDevices error set interface A %d %s %s \n" , numDevs, BufPtrs[0], BufPtrs[1]);
    }
    fprintf ( stderr,"FT_ListDevices worked %d \"%s\" \"%s\" \n" , numDevs, BufPtrs[0], BufPtrs[1]);

    ftStatus = FT_OpenEx(PORTA, FT_OPEN_BY_DESCRIPTION, &CAM8A);
    if ( ftStatus != FT_OK)
    {
        fprintf ( stderr,"FTD2xx error opening %s (%d)\n", PORTA, ftStatus );
    }
    ftStatus = FT_OpenEx(PORTB, FT_OPEN_BY_DESCRIPTION, &CAM8B);
    if ( ftStatus != FT_OK)
    {
        fprintf ( stderr,"FTD2xx error opening %s (%d)\n", PORTB, ftStatus );
    }


    // BitBang channel 2
    if (FT_SetBitMode( CAM8B, 0xFF, FT_BITMODE_ASYNC_BITBANG ) != FT_OK )
    {
        fprintf ( stderr,"FTD2xx error setting asynchronous bitbang mode interface B\n" );
    }


    // Baudrate
    cameraSetBaudrateDivisor ( CAM84_BAUDRATE_DIVISOR );

    //timeouts - latency
    cameraSetLibftdiTimers ( CAM84_LATENCYA,CAM84_LATENCYB,CAM84_TIMERA,CAM84_TIMERB );

   DWORD transferSize = 4096; //16384;
    if(FT_SetUSBParameters(CAM8A, transferSize , 0)!=FT_OK)
        fprintf(stderr,"Error setting USB transfer size\n");
    else
        fprintf(stderr,"Setting USB transfer size to %d\n", transferSize);


    //    fprintf ( stderr,"libftdi interface A read chunksize %u\n",CAM8A->readbuffer_chunksize );
    //    fprintf ( stderr,"libftdi interface B read chunksize %u\n",CAM8B->readbuffer_chunksize );
    //    fprintf ( stderr,"libftdi interface A write chunksize %u\n",CAM8A->writebuffer_chunksize );
    //    fprintf ( stderr,"libftdi interface B write chunksize %u\n",CAM8B->writebuffer_chunksize );

    //if (ftdi_read_data_set_chunksize (CAM8A,256*1)<0 ) fprintf(stderr,"libftdi error set chunksize A\n");
    //    if ( ftdi_write_data_set_chunksize ( CAM8B,256 ) <0 )
    //    {
    //        fprintf ( stderr,"libftdi error set chunksize B\n" );
    //    }

    if ( FT_Purge( CAM8A, FT_PURGE_RX | FT_PURGE_TX ) != FT_OK )
    {
        fprintf ( stderr,"ftd2xx error purge interface A\n" );
    }
    if ( FT_Purge( CAM8B, FT_PURGE_RX | FT_PURGE_TX ) != FT_OK )
    {
        fprintf ( stderr,"ftd2xx error purge interface B\n" );
    }
#endif

    AD9822 ( 0,0x58 );
    AD9822 ( 1,0xA0 );
    AD9822 ( 6,0 );
    //???????? ??????????????? ?????. ??? ?? ????????????? ???
    AD9822 ( 3,34 );
    adress=0;
    HC595 ( 0xCF );
    fprintf(stderr,"7) ftdi_write_data with adress size of %d\n", adress);
    //  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#ifdef LIBFTDI
    if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#else
    if ( FT_Write( CAM8B, FT_Out_Buffer, adress, &bytesWritten ) != FT_OK)
#endif
    {
        fprintf ( stderr,"write failed on channel 2)\n" );
    }

    isConnected = FT_OP_flag;
    cameraSetGain ( CAM84_GAIN );
    cameraSetOffset ( CAM84_OFFSET );
    errorReadFlag = false;
    cameraState = cameraIdle;
    if ( FT_OP_flag==false )
    {
        cameraState = cameraError;
    }
    return isConnected;
}




/*Disconnect camera, return bool result*/
bool cameraDisconnect ( void )            /*stdcall; export;*/
{
    //    bool FT_OP_flag;
#ifdef LIBFTDI
    ftdi_disable_bitbang ( CAM8B );
    ftdi_usb_close ( CAM8B );
    ftdi_free ( CAM8B );
    ftdi_usb_close ( CAM8A );
    ftdi_free ( CAM8A );
#else
    FT_Close(CAM8A);
    FT_Close(CAM8B);
#endif
    return true;

}

/*Check camera connection, return bool result*/
bool cameraIsConnected()               /*stdcall; export;*/
{
    return isConnected;
}

int cameraStartExposure ( int Bin,int StartX,int StartY,int NumX,int NumY, double Duration, int theFrameType ) /*stdcall; export;*/
{
    bool light = false;
    if(theFrameType == 0)
    {
        light = true;
    }

    fprintf ( stderr,"Start exposure bin %d x %d y %d w %d h %d s %f l %d\n",Bin,StartX,StartY,NumX,NumY,Duration,light );
    durat=Duration;
#ifdef LIBFTDI
    ftdi_usb_purge_tx_buffer ( CAM8B );
    ftdi_usb_purge_rx_buffer ( CAM8B );
    ftdi_usb_purge_tx_buffer ( CAM8A );
    ftdi_usb_purge_rx_buffer ( CAM8A );
#else
    if ( FT_Purge( CAM8A, FT_PURGE_RX | FT_PURGE_TX ) != FT_OK )
    {
        fprintf ( stderr,"ftd2xx error purge interface A\n" );
    }
    if ( FT_Purge( CAM8B, FT_PURGE_RX | FT_PURGE_TX ) != FT_OK )
    {
        fprintf ( stderr,"ftd2xx error purge interface B\n" );
    }
#endif
    canStopExposureNow = false;
    errorReadFlag = false;
    mBin = Bin;
    if ( light )
    {
        zatv=0;
    }
    else
    {
        zatv=1;
    }
    if ( ( NumY+StartY > CameraHeight ) || ( StartY < 0 ) || ( NumY <= 0 ) )
    {
        mYn=0;
        mdeltY=yccd;
        fprintf ( stderr,"NumY KO)\n" );
    }
    else
    {
        mYn=StartY / 2;
        mdeltY=NumY / 2;
    }
    if ( ( NumX+StartX > CameraWidth ) || ( StartX < 0 ) || ( NumX <= 0 ) )
    {
        mXn=0;
        mdeltX=xccd;
        fprintf ( stderr,"NumX KO)\n" );

    }
    else
    {
        mXn=StartX / 2;
        mdeltX=NumX / 2;
    }
    imageReady = false;
    cameraState = cameraExposing;
    if ( Duration > 0.499 )
    {
        adress=0;

        clearframe();
        shift3();

        //	  fprintf(stderr, "adress = %d\n", adress);
        fprintf(stderr,"8) ftdi_write_data with adress size of %d\n", adress);
#ifdef LIBFTDI
        if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
#else
        if ( FT_Write( CAM8B, FT_Out_Buffer, adress, &bytesWritten ) != FT_OK)
#endif
        {
            fprintf ( stderr,"write failed on channel 2)\n" );
        }

        clock_gettime(CLOCK_MONOTONIC, &exposureStartTime);

        pthread_t te,tt;
        pthread_create ( &te, NULL, ExposureTimerTick, NULL );
        pthread_detach ( te );
        pthread_create ( &tt, NULL, Timer15VTick, NULL );
        pthread_detach ( tt );

    }
    else
    {
        uint32_t dd;
        dd = Duration*1000;
        readframe ( mBin,dd );
    }
    return rfstatus;

}

/*Stop camera exposure when it is possible*/
bool cameraStopExposure()               /*stdcall; export;*/
{
    //printf("cameraStopExposure\n");
    if ( canStopExposureNow )
    {
        StopExposure();
    }
    else
    {
        checkCanStopExposureCount=0;
        //stopExposureTimer = settimer (0,0,100,&StopExposureTimerTick);
        void *p;
        //UtilTimer_AddTimer (*(100,0,0,*StopExposureTimerTick));
    }
    return true;

}

/*Get camera state, return int result*/
int cameraGetCameraState()           /*stdcall; export;*/
{
    int Result;
    if ( ! errorWriteFlag )
    {
        Result = cameraState;
    }
    else
    {
        Result = cameraError;
    }
    return Result;
}

/*Check ImageReady flag, is image ready for transfer - transfer image to driver and return bool ImageReady flag*/
bool cameraGetImageReady()               /*stdcall; export;*/
{
    return imageReady;
}



uint16_t cameraGetImage ( int i,int j )
{
    //	fprintf(stderr,"cameraGetImage %d %d\n",i,j);
    /*  if(i<getImageImin) getImageImin=i;
  if(i>getImageImax) getImageImax=i;
  if(j<getImageJmin) getImageJmin=j;
  if(j>getImageJmax) getImageJmax=j; */
    cameraState=cameraDownload;
    cameraState=cameraIdle;
    return bufim[i][j];
}

void cameraGetImage2 ( void *buff )
{
    fprintf(stderr,"cameraGetImage2\n");
    buff = bufim;
}
/*Set camera gain, return bool result*/
bool cameraSetGain ( int val )                 /*stdcall; export;*/
{
    AD9822 ( 3,val );
    return true;
}

/*Set camera offset, return bool result*/
bool cameraSetOffset ( int val )                 /*stdcall; export;*/
{
    int x;
    x=abs ( 2*val );
    if ( val < 0 )
    {
        x=x+256;
    }
    AD9822 ( 6,x );
    return true;
}

/*Get camera error state, return bool result*/
int cameraGetError()           /*stdcall; export;*/
{
    int res;

    res=0;
    if ( errorWriteFlag )
    {
        res =res+2;
    }
    if ( errorReadFlag )
    {
        res =res+1;
    }
    return res;
}

/*Set camera baudrate, return bool result*/
bool cameraSetBaudrateDivisor ( int theDivisor )                /*stdcall; export;*/
{
    int theBaudRatekbps;
    if (theDivisor < 2)
    {
        if (theDivisor <= 0)
        {
            theBaudRatekbps = 3000;
        }
        else
        {
            theBaudRatekbps = 2000;
        }
    }
    else
    {
        theBaudRatekbps = 3000 / theDivisor;
    }

    bool Result = false;
    /*setup FT2232 baud rate*/
//    if ( ( theBaudRatekbps>=minBaudrate ) & ( theBaudRatekbps<=maxBaudrate ) )
    {
        Result = true;      //valid baud rate requested
        spusb = theBaudRatekbps*1000;
        ms1 = spusb / 5000;  //used to generate a millisecond of delay via output clock rate
                             // represents number of bit bang pulses in 1 millisecond
                             // Note: Output clock rate for asynchrounous bit bang is 5x the
                             // requested baud rate (see note regarding libftdi 4x scaling issue
                             // in setBaudRate code
        FT_Current_Baud = spusb;
        fprintf( stderr, " FT_Current_Baud %d   ms1 %d\n", FT_Current_Baud, ms1);
        fprintf( stderr, "***Setting Baud Rate A to %d\n", FT_Current_Baud);
        fprintf( stderr, "***Setting Baud Rate B to %d\n", FT_Current_Baud);
// Started adding INDI style debug statements but kstars posts them as popups, need further review before completing
//        DEBUGF(INDI::Logger::DBG_SESSION,"***Setting Baud Rate to %d\n", FT_Current_Baud);
#ifdef LIBFTDI
        if(ftdi_result=ftdi_set_baudrate ( CAM8A, FT_Current_Baud ) < 0)
        {
            fprintf ( stderr,"libftdi error set baud interface A (%d:%s)\n",ftdi_result,ftdi_get_error_string ( CAM8A ) );
            Result = false;
        }
        if ( ftdi_result=ftdi_set_baudrate ( CAM8B,FT_Current_Baud / 4 ) < 0)  //Baud rate for bit bang is 4x baud rate, WHY?
        {
            fprintf ( stderr,"libftdi error set baud interface B (%d:%s)\n",ftdi_result,ftdi_get_error_string ( CAM8B ) );
            Result = false;
        }
#else
        if( (FT_SetBaudRate( CAM8A, FT_Current_Baud ) != FT_OK) )
        {
            fprintf ( stderr,"D2xx error set baud interface A\n");
            Result = false;
        }
        if( FT_SetBaudRate( CAM8B,  FT_Current_Baud ) != FT_OK)     //D2xx lib does not uprate by 4x
        {
            fprintf ( stderr,"D2xx error set baud interface B \n");
            Result = false;
        }
#endif


#ifdef LIBFTDI
        fprintf ( stderr,"libftdi  BRA=%d BRB=%d TA=%d TB=%d\n",CAM8A->baudrate,CAM8B->baudrate,CAM8A->usb_read_timeout,CAM8B->usb_write_timeout );
#endif

        return Result;
    }
//    else
//    {
//        fprintf( stderr, "Requested Baud rate of %d kbps is outside of allowed limits.\n",theBaudRatekbps);
//        return false;
//    }
}

bool cameraSetLibftdiTimers ( int latA,int latB,int timerA,int timerB )
{
    //    int wlatA,wlatB;
#ifdef LIBFTDI
    unsigned char latA_char = latA;
    if ( ftdi_set_latency_timer ( CAM8A,latA_char ) <0 )
    {
        fprintf ( stderr,"libftdi error set latency interface A\n" );
    }
    unsigned char latB_char = latB;
    if ( ftdi_set_latency_timer ( CAM8B,latB_char ) <0 )
    {
        fprintf ( stderr,"libftdi error set latency interface B\n" );
    }

    fprintf( stderr, "       latA %d latB %d \n", latA_char, latB_char);

    ftdi_get_latency_timer ( CAM8A, &latA_char);
    ftdi_get_latency_timer ( CAM8B, &latB_char);

//   fprintf(stderr, "libusb %d \n", libusb_control_transfer (CAM8A-> usb_dev, FTDI_DEVICE_OUT_REQTYPE, SIO_SET_LATENCY_TIMER_REQUEST, latA_char, CAM8A-> usb_dev -> index, NULL, 0, CAM8A-> usb_dev -> usb_write_timeout)) ;

//    ftdi_get_latency_timer ( CAM8A, &latA_char);
//    ftdi_get_latency_timer ( CAM8B, &latB_char);
    fprintf( stderr, "actual latA %d latB %d \n", latA_char, latB_char);

    CAM8A->usb_read_timeout=timerA;
    CAM8B->usb_read_timeout=timerB;
    CAM8A->usb_write_timeout=timerA;
    CAM8B->usb_write_timeout=timerB;
    fprintf ( stderr,"libftdi BRA=%d BRB=%d TA=%d TB=%d\n",CAM8A->baudrate,CAM8B->baudrate,CAM8A->usb_read_timeout,CAM8B->usb_write_timeout );
#else
    unsigned char latA_char, latB_char;
    FT_GetLatencyTimer(CAM8A, &latA_char);
    FT_GetLatencyTimer(CAM8A, &latB_char);
    fprintf( stderr, "old   latA %d latB %d \n", latA_char, latB_char);

    if(FT_SetLatencyTimer(CAM8A, latA) != FT_OK)
        fprintf(stderr, "D2xx error setting latency A to %d\n", latA);
    if(FT_SetLatencyTimer(CAM8B, latB)!= FT_OK)
        fprintf(stderr, "D2xx error setting latency B to %d\n", latB);
    if(FT_SetTimeouts(CAM8A,timerA, timerA)!= FT_OK)
            fprintf(stderr, "D2xx error setting timer A to %d\n", timerA);
    if(FT_SetTimeouts(CAM8B, timerB, timerB)!= FT_OK)
            fprintf(stderr, "D2xx error setting timerB to %d\n", timerB);

    FT_GetLatencyTimer(CAM8A, &latA_char);
    FT_GetLatencyTimer(CAM8A, &latB_char);
    fprintf( stderr, "actual latA %d latB %d \n", latA_char, latB_char);

#endif
    return true;
}

//uint16_t  swap ( void * x )  //NO NEED TO SWAP JUST NEED RESIZE OF POINTER
//{
//    uint16_t * xp = (uint16_t *) x;
//    return * xp;
//}

#ifdef LIBFTDI
int ftdi_read_data_modified ( struct  ftdi_context * ftdi, unsigned char * buf, int size )
{
//    clock_gettime(CLOCK_MONOTONIC, &rdmStartTime);

//    rdmPreElapsedTime = (-rdmCurrentTime.tv_sec + rdmStartTime.tv_sec) + (- rdmCurrentTime.tv_nsec + rdmStartTime.tv_nsec)*1.0e-9;

    canWrite = true;    //Really needs to be inside of read command after command is sent out USB bus
    int rsize = ftdi_read_data ( ftdi, buf, size );
    int nsize=size-rsize;
    int retry=0;
    while ( ( nsize>0 ) & ( retry<20 ) )
    {
        retry++;
        usleep ( 10 );
        //     fprintf ( stderr,"Try %d since %d<>%d \n",retry, rsize,size );
        rsize = rsize+ftdi_read_data ( ftdi, buf+rsize, nsize );
        nsize = size - rsize;
    }
    if(retry>=20)
    {
        fprintf ( stderr,"Read Error: Too many retries stopped at %d \n",retry );
    } else if(retry>0) {
        fprintf ( stderr,"Read: Retries needed: %d \n",retry );
    }

//    clock_gettime(CLOCK_MONOTONIC, &rdmCurrentTime);
//    rdmElapsedTime = (rdmCurrentTime.tv_sec - rdmStartTime.tv_sec) + (rdmCurrentTime.tv_nsec - rdmStartTime.tv_nsec)*1.0e-9;
//    fprintf(stderr,"RDM: %f %f \n", rdmPreElapsedTime, rdmElapsedTime );

    return rsize;
}
#else
int ftdi_read_data_modified ( FT_HANDLE ftdi, unsigned char * buf, int size )
{
//    clock_gettime(CLOCK_MONOTONIC, &rdmStartTime);

//    rdmPreElapsedTime = (-rdmCurrentTime.tv_sec + rdmStartTime.tv_sec) + (- rdmCurrentTime.tv_nsec + rdmStartTime.tv_nsec)*1.0e-9;

    DWORD rsize, rsize_new;
    DWORD sizeLPD = (DWORD) size;
    canWrite = true;    //Really needs to be inside of read command after command is sent out USB bus
    if(FT_Read ( ftdi, buf, sizeLPD, &rsize)!=FT_OK)
            fprintf(stderr, "FT_READ failed\n");
    int nsize=size-rsize;
    int retry=0;
    while ( ( nsize>0 ) & ( retry<20 ) )
    {
        retry++;
        usleep ( 10 );
        //     fprintf ( stderr,"Try %d since %d<>%d \n",retry, rsize,size );
        if(FT_Read ( ftdi, buf, sizeLPD, &rsize_new)!=FT_OK)
                fprintf(stderr, "FT_READ failed\n");
        rsize = rsize+rsize_new;
        nsize = sizeLPD - rsize;
    }
    if(retry>=20)
    {
        fprintf ( stderr,"Read Error: Too many retries stopped at %d \n",retry );
    } else if(retry>0) {
        fprintf ( stderr,"Read: Retries needed: %d \n",retry );
    }

//    clock_gettime(CLOCK_MONOTONIC, &rdmCurrentTime);
//    rdmElapsedTime = (rdmCurrentTime.tv_sec - rdmStartTime.tv_sec) + (rdmCurrentTime.tv_nsec - rdmStartTime.tv_nsec)*1.0e-9;
//    fprintf(stderr,"RDM: %f %f \n", rdmPreElapsedTime, rdmElapsedTime );

    return rsize;
}

#endif

