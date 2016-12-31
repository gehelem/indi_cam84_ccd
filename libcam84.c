#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/time.h>
#include "libcam84.h"
#include "config.h"
#include <unistd.h>
#include <ftdi.h>

struct timeval  curTime;


// /*?????? ???????????*/
const int CameraWidth = 3000;
/*?????? ???????????*/
const int CameraHeight = 2000;
/*?????????????? ???????? ?? ??????? ????? BDBUS*/
const uint8_t portfirst = 0xE1+8;
const uint8_t portsecond = 0x91+8;
const int xccd = 1500;
const int yccd = 1000;
const int dx = 44;//3044-2*xccd;
const int dx2 = 86;//1586-xccd;
const int dy = 12;//512-(yccd / 2);
//const int apolosa = 50;

/*camera state consts*/
const int cameraIdle = 0;
const int cameraWaiting = 1;
const int cameraExposing = 2;
const int cameraReading = 3;
const int cameraDownload = 4;
const int cameraError = 5;
struct ftdi_context *CAM8A, *CAM8B;

int ftdi_result;
int rfstatus =0;
double durat;
//int x;
//struct ftdi_context *ftdi, *ftdi2;
//int f,i;
unsigned char buf[1];
//??????????-????, ?????????? ????????? ?????????? ? ???????
bool isConnected  = false;
//??????????-????, ??????????, ????? ????? ?????????? ??????????
bool canStopExposureNow = true;
//bool canRead = false;
//bool canWrite = false;
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
double spusb;
double ms1;

void AD9822 ( uint8_t adr , uint16_t val );
void HC595 ( uint8_t va );
//static uint16_t FT_In_Buffer[6000];
static uint8_t FT_In_Buffer[13000];
static uint8_t FT_Out_Buffer[26000000];
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
int ftdi_read_data_modified ( struct ftdi_context *ftdi, unsigned char *buf,  int size );

uint16_t swap ( uint16_t x );

//int ftStatus = 0;
uint32_t   bytesWritten = 0;
int32_t   dwBytesRead  = 0;
char *PORTA;
char *PORTB;


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

void AD9822 ( uint8_t adr,uint16_t val )
{
//    fprintf(stderr,"AD9822 %d %d\n",adr,val);
  int kol = 64;
  uint8_t dan[kol];
  int i;
  memset ( dan,portfirst,kol );
  for ( i = 1; i <= 32; i++ )
    {
      dan[i]=dan[i] & 0xFE;
    };
  for ( i = 0; i <= 15; i++ )
    {
      dan[2*i+2]=dan[2*i+2] +2 ;
    };
  if ( ( adr & 4 ) ==4 )
    {
      dan[3] =dan[3]+4;
      dan[4] =dan[4]+4;
    };
  if ( ( adr & 2 ) ==2 )
    {
      dan[5] =dan[5]+4;
      dan[6] =dan[6]+4;
    };
  if ( ( adr & 1 ) ==1 )
    {
      dan[7] =dan[7]+4;
      dan[8] =dan[8]+4;
    };
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
  if ( ftdi_write_data ( CAM8B, dan, sizeof ( dan ) ) < 0 )
    {
      fprintf ( stderr,"write failed on channel 2)\n" );
    }
}

/*?????????? ????????? ?????? ???????? ??? ???????? ????? val ?? ?????? ?????????? HC595.*/
/*???????? ???? ? ???????????????? ????.*/
void HC595 ( uint8_t va )
{
//fprintf(stderr,"HC595 %d \n",adress);
  int kol = 20;
  uint8_t dan[kol];
  int i;
  uint16_t val;

  memset ( dan,portfirst,kol );
//for (i = 0;i < kol ;i++) dan[i]=portfirst;
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
      if ( ( val & 0x100 ) ==0x100 )
        {
          dan[2*i  ]=dan[2*i]   + 4;
          dan[2*i+1]=dan[2*i+1] + 4;
        }
      val=val*2;
    };
  dan[18]=dan[18]+ 0x80;
  for ( i = 0; i <= kol-1; i++ )
    {
      FT_Out_Buffer[2*i+adress]  =dan[i];
      FT_Out_Buffer[2*i+adress+1]=dan[i];
    }
  adress=adress+2*kol;
}

void shift0()
{
//printf("shift0\n");
  HC595 ( 0xDB );
  HC595 ( 0xFA );
  HC595 ( 0xEE );
  HC595 ( 0xCF );
}

/*?????????? ????????? ?????? ???????? ??? ?????? ???? ????????????? ??????*/
void shift()
{
//printf("shift\n");
  HC595 ( 0xCB );
  HC595 ( 0xDB );
  HC595 ( 0xDA );
  HC595 ( 0xFA );
  HC595 ( 0xEA );
  HC595 ( 0xEE );
  HC595 ( 0xCE );
  HC595 ( 0xCF );
}

/*?????????? ????????? ?????? ???????? ??? "?????" ???????????? ??????????? ? ????????? ???????*/
void shift2()
{
//printf("shift2\n");
  shift();
  HC595 ( 0xC7 );
  HC595 ( 0xC7 );
  HC595 ( 0xC7 );
  HC595 ( 0xC7 );
  HC595 ( 0xCB );
  HC595 ( 0xD9 );
  HC595 ( 0xD9 );
  HC595 ( 0xD9 );
  HC595 ( 0xD9 );
  HC595 ( 0xDB );
  HC595 ( 0xFA );
  HC595 ( 0xEA );
  HC595 ( 0xEE );
  HC595 ( 0xCE );
  HC595 ( 0xCF );
}

/*?????????? ????????? ?????? ???????? ??? ?????? ???? ????????????? ?????? + ?????? SUB ??? ?????? ??????? ???????????*/
void shift3()
{
  HC595 ( 0xCB );
  HC595 ( 0xDB );
  /*SUB*/
  HC595 ( 0x9A );
  HC595 ( 0xBA );
  HC595 ( 0xAA );
  HC595 ( 0xEE );
  HC595 ( 0xCE );
  HC595 ( 0xCF );
}

void clearline2()
{
  uint8_t dout[4];
  int x;
  dout[0]=portsecond;
  dout[1]=portsecond+8;
  dout[2]=portfirst+8;
  dout[3]=portfirst;
  for ( x=0; x <= 79*xccd; x ++ )
    {
      FT_Out_Buffer[adress+0]=dout[0]+0x40;
      FT_Out_Buffer[adress+1]=dout[0];
      FT_Out_Buffer[adress+2]=dout[3]-0x40;
      FT_Out_Buffer[adress+3]=dout[3];
      adress += 4;
    }
}

/*?????????? ????????? ?????? ???????? ???:*/
/*??????? ?????????? ????????. ???? ??? ?? ????????,*/
/*?? ??????????? ? ??? ????? ????? ???????? ? ???????????.*/
/*????????? ???? ??????? ?????? ? "???????" ? ??????????????? ????????.*/
/*???????? ?????????? ????? "??????" ??????????? ? ????????? ???????*/
void clearframe()
{
  uint16_t y;
  for ( y=0; y <= 1012-1; y ++ )
    {
      shift();
    }
  clearline2();
}

/* ?????? ?????????????? ?????????? ?????? FT2232HL ? ???????? ?????? ???????????*/
/*??-?? ???????????? AD9822 ????????? ??????? ??????? ????, ????? ???????, ? ? delphi ????????.*/
/*?????????? ????? ??? int32, ? ?? word16 ??-?? ???????????? ??? ??????????? ?????????*/

/*?????????? ???? ?????? ??????? ????? ???? ADBUS*/
void *posExecute ( void *arg )
{
//printf("poseExecute\n");
  uint16_t x,y,x1,byteCnt;
  bool readFailed;
//fprintf(stderr,"poseexecute mYn %d mdeltY %d mXn %d mdeltX %d \n",mYn,mdeltY,mXn,mdeltX);
//fprintf(stderr,"poseexecute y : %d>%d x : %d>%d\n",mYn,mYn+mdeltY-1,0,mdeltX - 1);
//for (x=0;x<13000;x++) FT_In_Buffer[x]=0;
  //canWrite=true;
  readFailed=false;
  byteCnt=0;
  for ( y= mYn; y <= mYn+mdeltY-1; y ++ )
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
//fprintf(stderr,"Time read %d %d\n",curTime.tv_sec,curTime.tv_usec);
          dwBytesRead=ftdi_read_data_modified ( CAM8A,FT_In_Buffer, 8*mdeltX );

          //canWrite=true;
          //if (y<5 | y> 995)   fprintf(stderr,"reading : read after  %d (%d)\n",y,8*mdeltX);
          if ( dwBytesRead < 0 )
            {
              fprintf ( stderr,"FT_Read failed (%d)\n",dwBytesRead );
            }

          if ( dwBytesRead!=8*mdeltX )
            {
              readFailed=true;
              fprintf ( stderr,"poseExecute bin==1 readfailed %d<>%d - %d / %d \n",dwBytesRead,8*mdeltX,y,mYn+mdeltY-1 );

              //break;
            }

          for ( x=0; x <= mdeltX - 1; x ++ )
            {
              x1=x+mXn;
              bufim[2*x1][2*y]    =1*FT_In_Buffer[ ( 4*x ) *2]+256*FT_In_Buffer[ ( 4*x ) *2+1];
              bufim[2*x1][2*y+1]  =1*FT_In_Buffer[ ( 4*x+1 ) *2]+256*FT_In_Buffer[ ( 4*x+1 ) *2+1];
              bufim[2*x1+1][2*y+1]=1*FT_In_Buffer[ ( 4*x+2 ) *2]+256*FT_In_Buffer[ ( 4*x+2 ) *2+1];
              bufim[2*x1+1][2*y]  =1*FT_In_Buffer[ ( 4*x+3 ) *2]+256*FT_In_Buffer[ ( 4*x+3 ) *2+1];
            }
        }
      else
        {
          dwBytesRead=ftdi_read_data_modified ( CAM8A,FT_In_Buffer, 2*mdeltX );

          if ( dwBytesRead < 0 )
            {
              fprintf ( stderr,"FT_Read failed (%d)\n",dwBytesRead );
            }

          if ( dwBytesRead!=2*mdeltX )
            {
              readFailed=true;
              fprintf ( stderr,"poseExecute bin<>1 readfailed %d<>%d - %d / %d \n",dwBytesRead,2*mdeltX,y,mYn+mdeltY-1 );
              break;
            }
          for ( x=0; x <= mdeltX - 1; x ++ )
            {
              x1=x+mXn;
              /*bufim[2*x1][2*y]=swap(FT_In_Buffer[x]);
              bufim[2*x1+1][2*y]=swap(FT_In_Buffer[x]);
              bufim[2*x1+1][2*y+1]=swap(FT_In_Buffer[x]);
              bufim[2*x1][2*y+1]=swap(FT_In_Buffer[x]);*/
              bufim[2*x1][2*y]    =1*FT_In_Buffer[2*x]+256*FT_In_Buffer[2*x+1];
              bufim[2*x1][2*y+1]  =1*FT_In_Buffer[2*x]+256*FT_In_Buffer[2*x+1];
              bufim[2*x1+1][2*y+1]=1*FT_In_Buffer[2*x]+256*FT_In_Buffer[2*x+1];
              bufim[2*x1+1][2*y]  =1*FT_In_Buffer[2*x]+256*FT_In_Buffer[2*x+1];
            }
        }
    }
  /*if (readFailed)
  {
    errorReadFlag = true;
    if (~ errorWriteFlag)  ftdi_usb_purge_rx_buffer(CAM8A);
    if (~ errorWriteFlag)  ftdi_usb_purge_tx_buffer(CAM8B);
  }*/
  ftdi_usb_purge_rx_buffer ( CAM8A );

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
  fprintf ( stderr,"reading : begin\n" );
  uint8_t dout[5] = {portsecond,portsecond+8,portfirst+8,portfirst,portsecond+0x28};
  int x,y;
  cameraState = cameraReading;
  //if (~ errorWriteFlag)  ftdi_usb_purge_rx_buffer(CAM8A);
  //if (~ errorWriteFlag)  ftdi_usb_purge_tx_buffer(CAM8B);
  adress=0;

  if ( expoz > 52 )
    {

      if ( expoz < 500 )
        {
          shift3();
          for ( y=0; y <= expoz-52; y ++ )
            for ( x=0; x <= ms1-1; x ++ )
              {
                HC595 ( 0xCF );
              }
        }
      clearline2();
      clearframe();
    }
  else
    {
      clearline2();
      clearframe();
      shift3();
      if ( expoz > 0 )
        for ( y=0; y <= expoz; y ++ )
          for ( x=0; x <= ms1-1; x ++ )
            {
              HC595 ( 0xCF );
            }
    }


  shift2();
  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
    {
      fprintf ( stderr,"write failed on channel 2)\n" );
    }
  adress=0;
  for ( y=0; y <= dy-1+mYn; y ++ )
    {
      shift();
    }
  clearline2();
  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
    {
      fprintf ( stderr,"write failed on channel 2)\n" );
    }
  adress=0;
  //canRead=false;
  pthread_t t1;
  pthread_create ( &t1, NULL, posExecute, NULL );
  //pthread_detach(t1);

  for ( y=0; y <= mdeltY -1; y ++ )
    {
      adress=0;
      shift();
      for ( x=0; x <= dx-1+4*mXn; x ++ )
        {
          FT_Out_Buffer[adress+0]=dout[0]+0x40;
          FT_Out_Buffer[adress+1]=dout[0];
          FT_Out_Buffer[adress+2]=dout[3]-0x40;
          FT_Out_Buffer[adress+3]=dout[3];
          adress += 4;
        }
      if ( bin == 1 )
        {
          for ( x=0; x <= 4; x ++ )
            {
              FT_Out_Buffer[adress+0]=dout[0]+0x40;
              FT_Out_Buffer[adress+1]=dout[0];
              FT_Out_Buffer[adress+2]=dout[3]-0x40;
              FT_Out_Buffer[adress+3]=dout[3];
              adress += 4;
            }

          FT_Out_Buffer[adress+0]=dout[0]+0x40;
          FT_Out_Buffer[adress+1]=dout[0];
          FT_Out_Buffer[adress+2]=dout[3]-0x40;
          FT_Out_Buffer[adress+3]=dout[3]-8;
          adress += 4;
          for ( x=0; x <= 4*mdeltX-2; x ++ )
            {
              FT_Out_Buffer[adress+0]=dout[0]+0x40;
              FT_Out_Buffer[adress+1]=dout[0];
              FT_Out_Buffer[adress+2]=dout[0]-8;
              FT_Out_Buffer[adress+3]=dout[3]-0x40;
              FT_Out_Buffer[adress+4]=dout[3];
              FT_Out_Buffer[adress+5]=dout[3]-8;
              adress += 6;
            }
        }
      else
        {
          for ( x=0; x <= 4; x ++ )
            {
              FT_Out_Buffer[adress+0]=dout[0]+0x40;
              FT_Out_Buffer[adress+1]=dout[0];
              FT_Out_Buffer[adress+2]=dout[0]+0x40;
              FT_Out_Buffer[adress+3]=dout[0];
              FT_Out_Buffer[adress+4]=dout[0]+0x40;
              FT_Out_Buffer[adress+5]=dout[0];
              FT_Out_Buffer[adress+6]=dout[0]+0x40;
              FT_Out_Buffer[adress+7]=dout[0];
              FT_Out_Buffer[adress+8]=dout[3];
              FT_Out_Buffer[adress+9]=dout[3];
              adress += 10;
            }
          FT_Out_Buffer[adress+0]=dout[0]+0x40;
          FT_Out_Buffer[adress+1]=dout[0];
          FT_Out_Buffer[adress+2]=dout[0]+0x40;
          FT_Out_Buffer[adress+3]=dout[0];
          FT_Out_Buffer[adress+4]=dout[0]+0x40;
          FT_Out_Buffer[adress+5]=dout[0];
          FT_Out_Buffer[adress+6]=dout[0]+0x40;
          FT_Out_Buffer[adress+7]=dout[0];
          FT_Out_Buffer[adress+8]=dout[3];
          FT_Out_Buffer[adress+9]=dout[3]-8;
          adress += 10;
          for ( x=0; x <= mdeltX-2; x ++ )
            {
              FT_Out_Buffer[adress+0]=dout[0]+0x40;
              FT_Out_Buffer[adress+1]=dout[0]-8;
              FT_Out_Buffer[adress+2]=dout[0]+0x40;
              FT_Out_Buffer[adress+3]=dout[0];
              FT_Out_Buffer[adress+4]=dout[0]+0x40;
              FT_Out_Buffer[adress+5]=dout[0];
              FT_Out_Buffer[adress+6]=dout[0]+0x40;
              FT_Out_Buffer[adress+7]=dout[0];
              FT_Out_Buffer[adress+8]=dout[3];
              FT_Out_Buffer[adress+9]=dout[3]-8;
              adress += 10;
            }
        }
      FT_Out_Buffer[adress+0]=dout[0]+0x40;
      FT_Out_Buffer[adress+1]=dout[0]-8;
      FT_Out_Buffer[adress+2]=dout[3]-0x40;
      FT_Out_Buffer[adress+3]=dout[3];
      adress += 4;
      for ( x=0; x <= dx2-1+6000-4*mdeltX-4*mXn; x ++ )
        {
          FT_Out_Buffer[adress+0]=dout[0]+0x40;
          FT_Out_Buffer[adress+1]=dout[0];
          FT_Out_Buffer[adress+2]=dout[3]-0x40;
          FT_Out_Buffer[adress+3]=dout[3];
          adress += 4;
        }

      //if (y<5 | y> 995)   fprintf(stderr,"reading : write before %d\n",y);
      //while (!canWrite);
      //canRead=false;
//gettimeofday(&curTime, NULL);
//fprintf(stderr,"Time writ %d %d\n",curTime.tv_sec,curTime.tv_usec);
      if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
        {
          fprintf ( stderr,"write failed on channel 2)\n" );
        }
      //canRead=true;
      //usleep(1000*2);
      //canRead=true;
      //if (y<5 | y> 995)   fprintf(stderr,"reading : write after  %d (%d) \n",y,adress);

    }
  pthread_join ( t1,NULL );
  imageReady = true;
  cameraState=cameraIdle;
  fprintf ( stderr,"reading : end\n" );

}

void *ExposureTimerTick ( void *arg )  /*stdcall;*/
{
  uint32_t dd;
  dd = ( durat*1000-52 ) *1000;
  usleep ( dd );
  canStopExposureNow = false;
  adress=0;
  //on +15V
  HC595 ( 0xCF );
  fprintf ( stderr,"write exp tick\n" );
  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
    {
      fprintf ( stderr,"write failed on channel 2)\n" );
    }
  readframe ( mBin, 1000 );
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
  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
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
      ftdi_usb_purge_rx_buffer ( CAM8A );
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
  CAM8A = ftdi_new();
  CAM8B = ftdi_new();
  if ( ftdi_set_interface ( CAM8A, INTERFACE_A ) <0 )
    {
      fprintf ( stderr,"libftdi error set interface A\n" );
    }
  if ( ftdi_set_interface ( CAM8B, INTERFACE_B ) <0 )
    {
      fprintf ( stderr,"libftdi error set interface B\n" );
    }
  if ( ftdi_usb_open ( CAM8A, 0x0403, 0x6010 ) <0 )
    {
      fprintf ( stderr,"libftdi error open interface A\n" );
    }
  if ( ftdi_usb_open ( CAM8B, 0x0403, 0x6010 ) <0 )
    {
      fprintf ( stderr,"libftdi error open interface B\n" );
    }

// BitBang channel 2
  if ( ftdi_set_bitmode ( CAM8B, 0xFF, BITMODE_BITBANG ) <0 )
    {
      fprintf ( stderr,"libftdi error set bitbang mode interface B\n" );
    }

// Baudrate
  cameraSetBaudrate ( CAM84_BAUDRATE );

//timeouts - latency
  cameraSetLibftdiTimers ( CAM84_LATENCYA,CAM84_LATENCYB,CAM84_TIMERA,CAM84_TIMERB );

  fprintf ( stderr,"libftdi interface A read chunksize %u\n",CAM8A->readbuffer_chunksize );
  fprintf ( stderr,"libftdi interface B read chunksize %u\n",CAM8B->readbuffer_chunksize );
  fprintf ( stderr,"libftdi interface A write chunksize %u\n",CAM8A->writebuffer_chunksize );
  fprintf ( stderr,"libftdi interface B write chunksize %u\n",CAM8B->writebuffer_chunksize );

  //if (ftdi_read_data_set_chunksize (CAM8A,256*1)<0 ) fprintf(stderr,"libftdi error set chunksize A\n");
  if ( ftdi_write_data_set_chunksize ( CAM8B,256 ) <0 )
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

  AD9822 ( 0,0x58 );
  AD9822 ( 1,0xA0 );
  AD9822 ( 6,0 );
  //???????? ??????????????? ?????. ??? ?? ????????????? ???
  AD9822 ( 3,34 );
  adress=0;
  HC595 ( 0xCF );
  if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
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
  bool FT_OP_flag;
  ftdi_disable_bitbang ( CAM8B );
  ftdi_usb_close ( CAM8B );
  ftdi_free ( CAM8B );
  ftdi_usb_close ( CAM8A );
  ftdi_free ( CAM8A );
  return true;

}

/*Check camera connection, return bool result*/
bool cameraIsConnected()               /*stdcall; export;*/
{
  return isConnected;
}

int cameraStartExposure ( int Bin,int StartX,int StartY,int NumX,int NumY, double Duration, bool light ) /*stdcall; export;*/
{
  fprintf ( stderr,"Start exposure bin %d x %d y %d w %d h %d s %f l %d\n",Bin,StartX,StartY,NumX,NumY,Duration,light );
  durat=Duration;
  ftdi_usb_purge_tx_buffer ( CAM8B );
  ftdi_usb_purge_rx_buffer ( CAM8B );
  ftdi_usb_purge_tx_buffer ( CAM8A );
  ftdi_usb_purge_rx_buffer ( CAM8A );
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
      shift3();
      if ( ftdi_write_data ( CAM8B, FT_Out_Buffer, adress ) < 0 )
        {
          fprintf ( stderr,"write failed on channel 2)\n" );
        }

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
  cameraState=cameraDownload;
  cameraState=cameraIdle;
  return bufim[i][j];
}

void cameraGetImage2 ( void *buff )
{
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
bool cameraSetBaudrate ( int val )                /*stdcall; export;*/
{
  bool Result;
  /*setup FT2232 baud rate*/
  if ( ( val>=10 ) & ( val<=150 ) )
    {
      spusb = val*10000;
      ms1 = spusb / 8000;
      FT_Current_Baud = spusb;
      ftdi_result=ftdi_set_baudrate ( CAM8A,FT_Current_Baud );
      if ( ftdi_result<0 )
        {
          fprintf ( stderr,"libftdi error set baud interface A (%d:%s)\n",ftdi_result,ftdi_get_error_string ( CAM8A ) );
        }
      ftdi_result=ftdi_set_baudrate ( CAM8B,FT_Current_Baud );
      if ( ftdi_result<0 )
        {
          fprintf ( stderr,"libftdi error set baud interface B (%d:%s)\n",ftdi_result,ftdi_get_error_string ( CAM8B ) );
        }
      fprintf ( stderr,"libftdi BRA=%d BRB=%d TA=%d TB=%d\n",CAM8A->baudrate,CAM8B->baudrate,CAM8A->usb_read_timeout,CAM8B->usb_write_timeout );
      Result = true;
    }
  else
    {
      Result = false;
    }
  return Result;
}

bool cameraSetLibftdiTimers ( int latA,int latB,int timerA,int timerB )
{
  int wlatA,wlatB;
  if ( ftdi_set_latency_timer ( CAM8A,latA ) <0 )
    {
      fprintf ( stderr,"libftdi error set latency interface A\n" );
    }
  if ( ftdi_set_latency_timer ( CAM8B,latB ) <0 )
    {
      fprintf ( stderr,"libftdi error set latency interface B\n" );
    }
  CAM8A->usb_read_timeout=timerA;
  CAM8B->usb_read_timeout=timerB;
  CAM8A->usb_write_timeout=timerA;
  CAM8B->usb_write_timeout=timerB;
  fprintf ( stderr,"libftdi BRA=%d BRB=%d TA=%d TB=%d\n",CAM8A->baudrate,CAM8B->baudrate,CAM8A->usb_read_timeout,CAM8B->usb_write_timeout );
  return true;
}

uint16_t swap ( uint16_t x )
{
  uint8_t hibyte = ( x & 0xFF00 ) >> 8;
  uint8_t lobyte = ( x & 0x00FF );
  uint16_t ret;
  ret= ( lobyte << 8 ) | hibyte; // originale ?
  //ret = hibyte << 8 | lobyte;
  return ret;
}

int ftdi_read_data_modified ( struct  ftdi_context * ftdi, unsigned char * buf, int size )
{
  int rsize = ftdi_read_data ( ftdi, buf, size );
  int nsize=size-rsize;
  int retry=0;
  while ( ( nsize>0 ) & ( retry<20 ) )
    {
      retry++;
      usleep ( 1 );
      fprintf ( stderr,"Try %d since %d<>%d \n",retry, rsize,size );
      rsize = rsize+ftdi_read_data ( ftdi, buf+rsize, nsize );
      nsize = size - rsize;
    }
  return rsize;
}

