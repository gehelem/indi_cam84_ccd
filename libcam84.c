#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
//#include <ftdi.h>
#include "ftd2xx.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
//#include <thread.h>
#include <time.h>
#include <sys/time.h>
#include "libcam84.h"
#include "config.h"
#include <unistd.h>

//using namespace std;

typedef uint8_t  Byte;
typedef uint16_t Word;
//typedef uint8_t Word;


/*?????? ???????????*/
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
const int apolosa = 50;

/*camera state consts*/
const int cameraIdle = 0;
const int cameraWaiting = 1;
const int cameraExposing = 2;
const int cameraReading = 3;
const int cameraDownload = 4;
const int cameraError = 5;


/*Class for reading thread
posl = class(TThread)
private
 Private declarations
protected
procedure Execute; override;
end;*/
int kkk;
int rfstatus =0;
double durat;
  //int x;
  struct ftdi_context *ftdi, *ftdi2;
  int f,i;
  unsigned char buf[1];
//??????????-????, ?????????? ????????? ?????????? ? ???????
  bool isConnected  = false;
//??????????-????, ??????????, ????? ????? ?????????? ??????????
 bool canStopExposureNow = true;
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

void AD9822(uint8_t adr , uint16_t val);
void HC595(uint8_t va);
//static uint16_t FT_In_Buffer[6000];
static uint8_t FT_In_Buffer[13000];
static uint8_t FT_Out_Buffer[26000000];
int  FT_Current_Baud;
bool FT_OP_flag;

void clearline2(void);
void shift(void);
void shift0(void);
void shift2(void);
void shift3(void);
void clearframe(void);
void readframe(int bin,int expoz);
void coexecute(void);
void ComRead(void);

uint16_t swap(uint16_t x);

FT_HANDLE CAM8A;
FT_HANDLE CAM8B;
FT_STATUS ftStatus = FT_OK;
//int ftStatus = 0;
DWORD   bytesWritten = 0;
DWORD   dwBytesRead  = 0;
char *PORTA;
char *PORTB;
DWORD numDevs;
char *BufPtrs[3];   // pointer to array of 3 pointers
char Buffer1[64];      // buffer for description of first device
char Buffer2[64];      // buffer for description of second device
char Buffer[64]; // more than enough room!
//thread tt,t1,te;


/*int FT_Write_wfile(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD dwBytesToWrite, LPDWORD lpBytesWritten)
{
ftStatus = FT_Write(ftHandle, &lpBuffer, sizeof(lpBuffer), &lpbytesWritten);
        if (ftStatus != FT_OK)
        {
                printf("FT_Write failed (error %d).\n", (int)ftStatus);
                exit(-1);
        }

return ftStatus;
}*/


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
void prtbuf(uint8_t *buff,DWORD l)
{
int kkk;
int max = 1000;
if ( l <= 1000) max=l;
for (kkk=1;kkk<=max;kkk++) {fprintf(stdout,"%X ",buff[kkk]);};
fprintf(stdout,"\n");
}

void AD9822(uint8_t adr,uint16_t val)
{
printf("AD9822 %d %d\n",adr,val);
int kol = 64;
uint8_t dan[kol];
int i;
//memset(array,fill_this_character,sizeof(array));
memset(dan,portfirst,kol);
    for (i = 1; i <= 32; i++)  { dan[i]=dan[i] & 0xFE;          };
    for (i = 0; i <= 15; i++)  { dan[2*i+2]=dan[2*i+2] +2 ;      };
    if ((adr & 4)==4)       { dan[3] =dan[3]+4;     dan[4] =dan[4]+4;  };
    if ((adr & 2)==2)       { dan[5] =dan[5]+4;     dan[6] =dan[6]+4;  };
    if ((adr & 1)==1)       { dan[7] =dan[7]+4;     dan[8] =dan[8]+4;  };
    if ((val & 256)==256)   { dan[15]=dan[15]+4;    dan[16]=dan[16]+4; };
    if ((val & 128)==128)   { dan[17]=dan[17]+4;    dan[18]=dan[18]+4; };
    if ((val &  64)==64)    { dan[19]=dan[19]+4;    dan[20]=dan[20]+4; };
    if ((val &  32)==32)    { dan[21]=dan[21]+4;    dan[22]=dan[22]+4; };
    if ((val &  16)==16)    { dan[23]=dan[23]+4;    dan[24]=dan[24]+4; };
    if ((val &   8)==8)     { dan[25]=dan[25]+4;    dan[26]=dan[26]+4; };
    if ((val &   4)==4)     { dan[27]=dan[27]+4;    dan[28]=dan[28]+4; };
    if ((val &   2)==2)     { dan[29]=dan[29]+4;    dan[30]=dan[30]+4; };
    if ((val &   1)==1)     { dan[31]=dan[31]+4;    dan[32]=dan[32]+4; };
    if (FT_Write(CAM8B, dan, sizeof(dan),&bytesWritten) != 0) fprintf(stderr,"write failed on channel 2)\n");
    fprintf(stdout,"AD9822 W\n");
    prtbuf(dan,bytesWritten);
}

/*?????????? ????????? ?????? ???????? ??? ???????? ????? val ?? ?????? ?????????? HC595.*/
/*???????? ???? ? ???????????????? ????.*/
void HC595(uint8_t va)
{
//fprintf(stdout,"HC595 %d \n",adress);
int kol = 20;
uint8_t dan[kol];
int i;
uint16_t val;

memset(dan,portfirst,kol);
//for (i = 0;i < kol ;i++) dan[i]=portfirst;
 if (zatv == 1 ) val=va+0x0100; else val=va ;
 for (i = 0; i <= 8; i++)  
 { 
	dan[2*i+1]=dan[2*i+1] +2 ;
        if ((val & 0x100)==0x100)
	{
		dan[2*i  ]=dan[2*i]   + 4;
		dan[2*i+1]=dan[2*i+1] + 4;
	}
	val=val*2;  
 };
 dan[18]=dan[18]+ 0x80; 
 for (i = 0; i <= kol-1; i++)   
 {
	FT_Out_Buffer[2*i+adress]  =dan[i];
	FT_Out_Buffer[2*i+adress+1]=dan[i];
 }
 adress=adress+2*kol;
 }

void shift0()
{
//printf("shift0\n");
    HC595(0xDB);
  HC595(0xFA);
  HC595(0xEE);
  HC595(0xCF);
}

/*?????????? ????????? ?????? ???????? ??? ?????? ???? ????????????? ??????*/
void shift()
{
//printf("shift\n");
  HC595(0xCB);
  HC595(0xDB);
  HC595(0xDA);
  HC595(0xFA);
  HC595(0xEA);
  HC595(0xEE);
  HC595(0xCE);
  HC595(0xCF);
}

/*?????????? ????????? ?????? ???????? ??? "?????" ???????????? ??????????? ? ????????? ???????*/
void shift2()
{
//printf("shift2\n");
  shift();
  HC595(0xC7);
  HC595(0xC7);
  HC595(0xC7);
  HC595(0xC7);
  HC595(0xCB);
  HC595(0xD9);
  HC595(0xD9);
  HC595(0xD9);
  HC595(0xD9);
  HC595(0xDB);
  HC595(0xFA);
  HC595(0xEA);
  HC595(0xEE);
  HC595(0xCE);
  HC595(0xCF);
}

/*?????????? ????????? ?????? ???????? ??? ?????? ???? ????????????? ?????? + ?????? SUB ??? ?????? ??????? ???????????*/
void shift3()
{
//printf("shift3\n");
  HC595(0xCB);
  HC595(0xDB);
  /*SUB*/
  HC595(0x9A);
  HC595(0xBA);
  HC595(0xAA);
  HC595(0xEE);
  HC595(0xCE);
  HC595(0xCF);
}

void clearline2()
{
//printf("clearline2\n");
uint8_t dout[4];
int x;
dout[0]=portsecond;
dout[1]=portsecond+8;
dout[2]=portfirst+8;
dout[3]=portfirst;
//printf("clearline2-1\n");
  for( x=0; x <= 79*xccd; x ++)
  {
//fprintf(stdout,"clearline2-2 %d \n",adress);
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
//fprintf(stdout,"clearframe %d\n",adress );
        uint16_t y;
        for( y=0; y <= 1012-1; y ++)
            {
                //fprintf(stdout,"clearframe %d-%d\n",y,adress );
                shift();
            }
	clearline2();
}

/* ?????? ?????????????? ?????????? ?????? FT2232HL ? ???????? ?????? ???????????*/
/*??-?? ???????????? AD9822 ????????? ??????? ??????? ????, ????? ???????, ? ? delphi ????????.*/
/*?????????? ????? ??? int32, ? ?? word16 ??-?? ???????????? ??? ??????????? ?????????*/

/*?????????? ???? ?????? ??????? ????? ???? ADBUS*/
void *posExecute(void *arg)
{
//printf("poseExecute\n");
uint16_t x,y,x1,byteCnt;
bool readFailed;
//fprintf(stdout,"poseexecute mYn %d mdeltY %d mXn %d mdeltX %d \n",mYn,mdeltY,mXn,mdeltX);
//fprintf(stdout,"poseexecute y : %d>%d x : %d>%d\n",mYn,mYn+mdeltY-1,0,mdeltX - 1);
readFailed=false;
  byteCnt=0;
  for( y= mYn; y <= mYn+mdeltY-1; y ++)
  {
    if (mBin == 1) 
    {
		
      //if (~ errorWriteFlag)  byteCnt=ftdi_read_data(ftdi,FT_In_Buffer,8*mdeltX);
      //byteCnt=ftdi_read_data(ftdi,FT_In_Buffer, 8*mdeltX);

      ftStatus=FT_Read(CAM8A, FT_In_Buffer, 8*mdeltX, &dwBytesRead);
      if (ftStatus != FT_OK) fprintf(stdout,"FT_Read failed (%d)\n",ftStatus);
      //byteCnt=&dwBytesRead;
      //fprintf(stdout,"FT_READ y=%d B0=%d B1/2=%d Bmax=%d\n",y,FT_In_Buffer[0],FT_In_Buffer[4*mdeltX-1],FT_In_Buffer[8*mdeltX-1]);

      if (dwBytesRead!=8*mdeltX)
      {
        readFailed=true;
        fprintf(stdout,"poseExecute bin==1 readfailed %d<>%d - %d / %d \n",dwBytesRead,8*mdeltX,y,mYn+mdeltY-1);
        break;
      }

      for( x=0; x <= mdeltX - 1; x ++)
      {
//fprintf(stdout,"poseExecute x=%d y=%d\n",x,y);
          x1=x+mXn;
		
        /*bufim[2*x1][2*y]=swap(FT_In_Buffer[4*x]);
        bufim[2*x1][2*y+1]=swap(FT_In_Buffer[4*x+1]);
        bufim[2*x1+1][2*y+1]=swap(FT_In_Buffer[4*x+2]);
        bufim[2*x1+1][2*y]=swap(FT_In_Buffer[4*x+3]);*/
        bufim[2*x1][2*y]    =1*FT_In_Buffer[(4*x  )*2]+256*FT_In_Buffer[(4*x  )*2+1];
        bufim[2*x1][2*y+1]  =1*FT_In_Buffer[(4*x+1)*2]+256*FT_In_Buffer[(4*x+1)*2+1];
        bufim[2*x1+1][2*y+1]=1*FT_In_Buffer[(4*x+2)*2]+256*FT_In_Buffer[(4*x+2)*2+1];
        bufim[2*x1+1][2*y]  =1*FT_In_Buffer[(4*x+3)*2]+256*FT_In_Buffer[(4*x+3)*2+1];
        //fprintf(stdout,"bufmax x=%d y=%d\n",2*x1+1,2*y);
        /*if (2*x1+1<100)  bufim[2*x1+1][2*y+1]=65535;
        if (2*x1+1>2900) bufim[2*x1+1][2*y+1]=65535;
        if (2*y+1 <100)  bufim[2*x1+1][2*y+1]=65535;
        if (2*y+1 >2900) bufim[2*x1+1][2*y+1]=65535;*/

      }
    }
    else {
      //if (~ errorWriteFlag)
      //byteCnt=ftdi_read_data(ftdi,FT_In_Buffer,2*mdeltX);
        ftStatus=FT_Read(CAM8A, FT_In_Buffer, 2*mdeltX, &dwBytesRead);
        if (ftStatus != FT_OK) fprintf(stdout,"FT_Read failed (%d)\n",ftStatus);
        //byteCnt=&dwBytesRead;

        if (dwBytesRead!=2*mdeltX)
        {
          readFailed=true;
          fprintf(stdout,"poseExecute bin<>1 readfailed %d<>%d\n",dwBytesRead,2*mdeltX);
          break;
        }
      for( x=0; x <= mdeltX - 1; x ++)
      {
//fprintf(stdout,"poseExecute x=%d y=%d\n",x,y);
        x1=x+mXn;
		
        bufim[2*x1][2*y]=swap(FT_In_Buffer[x]);
        bufim[2*x1+1][2*y]=swap(FT_In_Buffer[x]);
        bufim[2*x1+1][2*y+1]=swap(FT_In_Buffer[x]);
        bufim[2*x1][2*y+1]=swap(FT_In_Buffer[x]);
      }
    }
  }
  if (readFailed) 
  {
    errorReadFlag = true;
    if (~ errorWriteFlag)  FT_Purge(CAM8A,FT_PURGE_RX);
    if (~ errorWriteFlag)  FT_Purge(CAM8B,FT_PURGE_TX);
  }
  //fprintf(stdout,"end of poseExecute x %d y %d \n",x,y);
  (void) arg;
  pthread_exit(NULL);
}

/*???????? ?????? ? ???????????????*/
void ComRead()
{/*
//printf("ComRead\n");
        if (pthread_create(&co, NULL, posExecute, NULL)) {
        //perror("pthread_create");
	//return EXIT_FAILURE;
    }

    if (pthread_join(co, NULL)) {
	perror("pthread_join");
	//return EXIT_FAILURE;
    }*/
  /*co:=posl.Create(true);
  co.FreeOnTerminate:=true;
  co.Priority:=tpNormal;
  co.Resume;*/
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
void readframe(int bin,int expoz)
{
//fprintf(stdout,"readframe %d %d \n",bin,expoz);
rfstatus = 0;
uint8_t dout[5] = {portsecond,portsecond+8,portfirst+8,portfirst,portsecond+0x28};
    int x,y;

  cameraState = cameraReading;
  if (~ errorWriteFlag)  FT_Purge(CAM8A,FT_PURGE_RX);
  if (~ errorWriteFlag)  FT_Purge(CAM8B,FT_PURGE_TX);
  FT_Purge(CAM8A,FT_PURGE_RX);
  FT_Purge(CAM8B,FT_PURGE_TX);
  adress=0;
  fprintf(stdout,"expoz %d\n",expoz);

  if (expoz > 52)
  {

    if (expoz < 500)
    {
      shift3();
      for( y=0; y <= expoz-52; y ++)
      for( x=0; x <= ms1-1; x ++)
        HC595(0xCF);
    }
    clearline2();
    clearframe();
  }
  else {
    clearline2();
    clearframe();
    shift3();
    if (expoz > 0) 
      for( y=0; y <= expoz; y ++)
      for( x=0; x <= ms1-1; x ++)
          HC595(0xCF);
  }

  //fprintf(stdout,"readframe  11111\n");
  shift2();
  fprintf(stdout,"readframe  22222 - %d\n",FT_Out_Buffer[0]);
  if (FT_Write(CAM8B, FT_Out_Buffer, adress,&bytesWritten) != FT_OK) fprintf(stderr,"write failed on channel 2)\n");
  fprintf(stdout,"FT_Write %d/%d\n",adress,bytesWritten);
  adress=0;
  //fprintf(stdout,"readframe  33333\n");
  for( y=0; y <= dy-1+mYn; y ++) shift();
  //fprintf(stdout,"readframe  44444\n");
  clearline2();
  //fprintf(stdout,"readframe  55555\n");
  if (FT_Write(CAM8B, FT_Out_Buffer, adress,&bytesWritten) != FT_OK) fprintf(stderr,"write failed on channel 2)\n");
  fprintf(stdout,"FT_Write %d/%d\n",adress,bytesWritten);

  //fprintf(stdout,"readframe  66666\n");
  adress=0;
  //fprintf(stdout,"readframe before comRead  : %d \n",adress);
  //ComRead();
  pthread_t t1;
  pthread_create(&t1, NULL, posExecute, NULL);

  //posExecute();
  //fprintf(stdout,"readframe before for  : %d \n",adress);
  for( y=0; y <= mdeltY -1; y ++)
  {
    //fprintf(stdout,"readframe for y=%d \n",y);
    adress=0;
    shift();
    for( x=0; x <= dx-1+4*mXn; x ++)
    {
      FT_Out_Buffer[adress+0]=dout[0]+0x40;
      FT_Out_Buffer[adress+1]=dout[0];
      FT_Out_Buffer[adress+2]=dout[3]-0x40;
      FT_Out_Buffer[adress+3]=dout[3];
      adress += 4;
    }
    if (bin == 1) 
    {
      for( x=0; x <= 4; x ++)
      {
        FT_Out_Buffer[adress+0]=dout[0]+0x40;
        FT_Out_Buffer[adress+1]=dout[0];
        FT_Out_Buffer[adress+2]=dout[3]-0x40;
        FT_Out_Buffer[adress+3]=dout[3];
        adress += 4;
      }    //fprintf(stdout,"readframe for y=%d \n",y);

      FT_Out_Buffer[adress+0]=dout[0]+0x40;
      FT_Out_Buffer[adress+1]=dout[0];
      FT_Out_Buffer[adress+2]=dout[3]-0x40;
      FT_Out_Buffer[adress+3]=dout[3]-8;
      adress += 4;
      for( x=0; x <= 4*mdeltX-2; x ++)
      {
        //fprintf(stdout,"readframe for2 bin==1 y=%d x=%d \n",y,x);
        FT_Out_Buffer[adress+0]=dout[0]+0x40;
        FT_Out_Buffer[adress+1]=dout[0];
        FT_Out_Buffer[adress+2]=dout[0]-8;
        FT_Out_Buffer[adress+3]=dout[3]-0x40;
        FT_Out_Buffer[adress+4]=dout[3];
        FT_Out_Buffer[adress+5]=dout[3]-8;
        adress += 6;
      }
    }
    else {
      for( x=0; x <= 4; x ++)
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
      for( x=0; x <= mdeltX-2; x ++)
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
    for( x=0; x <= dx2-1+6000-4*mdeltX-4*mXn; x ++)
    {
      FT_Out_Buffer[adress+0]=dout[0]+0x40;
      FT_Out_Buffer[adress+1]=dout[0];
      FT_Out_Buffer[adress+2]=dout[3]-0x40;
      FT_Out_Buffer[adress+3]=dout[3];
      adress += 4;
    }
    if (FT_Write(CAM8B, FT_Out_Buffer, adress,&bytesWritten) != 0) fprintf(stderr,"write failed on channel 2)\n");
    //fprintf(stdout,"FT_Write readframe %d/%d\n",adress,bytesWritten);

  }
  imageReady = true;
  cameraState=cameraIdle;
}

void *ExposureTimerTick(void *arg)     /*stdcall;*/
{
  uint32_t dd;
  dd = (durat*1000-52)*1000;
  usleep(dd);
  printf("ExposureTimerTick %d\n",dd);
  canStopExposureNow = false;
  adress=0;
  //on +15V
  HC595(0xCF);
    if (FT_Write(CAM8B, FT_Out_Buffer, adress,&bytesWritten) != 0) fprintf(stderr,"write failed on channel 2)\n");
    fprintf(stdout,"FT_Write ExposureTimerTick %d/%d\n",adress,bytesWritten);
  readframe (mBin, 1000);
  canStopExposureNow = true;
  printf("ExposureTimerTick end\n");
  (void) arg;
  pthread_exit(NULL);
}

//off +15V
void *Timer15VTick(void *arg)     /*stdcall;*/
{
  usleep(1000*1000);
  printf("Timer15VTick\n");
  adress=0;
  HC595(0x4F);
  //if (! errorWriteFlag)  errorWriteFlag = Write_USB_Device_Buffer_wErr(FT_CAM8B,FT_Out_Buffer,adress);
  if (FT_Write(CAM8B, FT_Out_Buffer, adress,&bytesWritten) != 0) fprintf(stderr,"write failed on channel 2)\n");
    fprintf(stdout,"FT_Write  Timer15VTick %d/%d\n",adress,bytesWritten);
  //if (f < 0) fprintf(stderr,"write failed on channel 2 for 0x%x, error %d (%s)\n", FT_Out_Buffer[0], f, ftdi_get_error_string(ftdi2));

  canStopExposureNow = true;
  printf("Timer15VTick end\n");
  (void) arg;
  pthread_exit(NULL);
}

/*Stop Exposure Timer, purge FTDI FT2232HL buffers and frame bufeer*/
void StopExposure()
{
//printf("StopExposure\n");
//  UtilTimer_RemoveTimer(ExposureTimerHandler);
  if (~ errorWriteFlag)  FT_Purge(CAM8A,FT_PURGE_RX);

  cameraState = cameraIdle;
  imageReady = true;

}

/*???????? ?????? 0,1 ??? ?? ?????????? 15 ???*/
void StopExposureTimerTick()     /*stdcall;*/
{
//printf("StopExposureTimerTick\n");
  if (canStopExposureNow) 
    {
      //printf("ftdi open succeeded(channel 2): %d\n",f);
      //KillTimer (0,stopExposureTimer);
      StopExposure();printf("ftdi open succeeded(channel 2): %d\n",f);
    }
  checkCanStopExposureCount=checkCanStopExposureCount+1;
  //if (checkCanStopExposureCount==150)  KillTimer (0,stopExposureTimer);
  if (checkCanStopExposureCount==150)  /*UtilTimer_RemoveTimer (stopExposureTimer)*/;

}

/*Connect camera, return bool result*/
/*????? ???????????? ????????? ? ????????????? AD9822*/
bool cameraConnect()               /*stdcall; export;*/
{
printf("cameraConnect\n");


FT_OP_flag = true;


    DWORD version;
    FT_GetLibraryVersion(&version);
    //printf("ftdi version: %x\n",version);

    // initialize the array of pointers
    BufPtrs[0] = Buffer1;
    BufPtrs[1] = Buffer2;
    BufPtrs[2] = NULL;      // last entry should be NULL

    ftStatus = FT_ListDevices(BufPtrs,&numDevs,FT_LIST_ALL|FT_OPEN_BY_DESCRIPTION);
    if (ftStatus == FT_OK) {
        // FT_ListDevices OK, product descriptions are in Buffer1 and Buffer2, and
        // numDevs contains the number of devices connected
    }
    else {
        // FT_ListDevices failed
    }

    //printf("first  device : %s\n",Buffer1);
    //printf("second device : %s\n",Buffer2);
/*
    DWORD devIndex = 0; // first device
    char Buffer[64]; // more than enough room!

    ftStatus = FT_ListDevices((PVOID)devIndex,Buffer,FT_LIST_BY_INDEX|FT_OPEN_BY_SERIAL_NUMBER);
    if (ftStatus == FT_OK) {
        // FT_ListDevices OK, serial number is in Buffer
    }
    else {
        // FT_ListDevices failed
    }

    //printf("first  device S/N : %s\n",Buffer);

    devIndex = 1; // second device
    ftStatus = FT_ListDevices((PVOID)devIndex,Buffer,FT_LIST_BY_INDEX|FT_OPEN_BY_SERIAL_NUMBER);
    if (ftStatus == FT_OK) {
        // FT_ListDevices OK, serial number is in Buffer
    }
    else {
        // FT_ListDevices failed
    }
    //printf("second device S/N : %s\n",Buffer);/*
;


/*    if ((ftdi  = ftdi_new()) == 0) {fprintf(stderr, "ftdi_new 1 failed\n");exit(-1);}
    if ((ftdi2 = ftdi_new()) == 0) {fprintf(stderr, "ftdi_new 2 failed\n");exit(-1);}

    if (ftdi_set_interface(ftdi , INTERFACE_A) != 0) {fprintf(stderr, "ftdi_set_interface 1 failed\n");exit(-1);}
    if (ftdi_set_interface(ftdi2, INTERFACE_B) != 0) {fprintf(stderr, "ftdi_set_interface 2 failed\n");exit(-1);}
    //ftdi_set_interface(ftdi , INTERFACE_A);
    //ftdi_set_interface(ftdi2, INTERFACE_B);

    f = ftdi_usb_open(ftdi, 0x0403, 0x6010);
    if (f < 0 && f != -5)
    {
        fprintf(stdout, "unable to open ftdi device: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        exit(-1);
    }
    printf("ftdi open succeeded(channel 1): %d\n",f);

    f = ftdi_usb_open(ftdi2, 0x0403, 0x6010);
    if (f < 0 && f != -5)
    {
        fprintf(stdout, "unable to open ftdi device: %d (%s)\n", f, ftdi_get_error_string(ftdi2));
        ftdi_free(ftdi2);
        exit(-1);
    }
    printf("ftdi open succeeded(channel 2): %d\n",f);*/
    PORTA="CAM8A";
    PORTB="CAM8B";

    ftStatus=FT_OpenEx(PORTA,FT_OPEN_BY_SERIAL_NUMBER,&CAM8A);
    if (ftStatus!=FT_OK) { fprintf(stdout, "unable to open ftdi device: 1 (%d)\n", ftStatus); exit(-1);}
    //printf("ftdi open succeeded(channel 1)\n");
    ftStatus=FT_OpenEx(PORTB,FT_OPEN_BY_SERIAL_NUMBER,&CAM8B);
    if (ftStatus!=FT_OK) { fprintf(stdout, "unable to open ftdi device: 2 (%d)\n", ftStatus); exit(-1);}
    //printf("ftdi open succeeded(channel 2)\n");


// BitBang channel 2
    if (FT_SetBitMode(CAM8B, 0xFF, 0x01) != 0) {fprintf(stderr, "ftdi_set_bitmode 2 failed\n");exit(-1);}

// Baudrate
    //if (FT_SetBaudRate(CAM8A ,115200) != 0) {fprintf(stderr, "ftdi_set_baudrate 1 failed\n");exit(-1);}
    spusb = 1600000;
    ms1 = spusb / 8000;
    FT_Current_Baud = spusb;
    if (FT_SetBaudRate(CAM8B ,FT_Current_Baud) != 0) {fprintf(stderr, "ftdi_set_baudrate 2 failed\n");exit(-1);}

    FT_SetLatencyTimer(CAM8A,20);
    FT_SetLatencyTimer(CAM8B,20);

//timeouts
    FT_SetTimeouts(CAM8A,12000,12000);
    FT_SetTimeouts(CAM8B,12000,12000);

//Purge
    FT_Purge(CAM8A,FT_PURGE_RX);
    FT_Purge(CAM8B,FT_PURGE_RX);
    FT_Purge(CAM8A,FT_PURGE_TX);
    FT_Purge(CAM8B,FT_PURGE_TX);


    //fprintf(stdout, "End of init\n");

    AD9822(0,0x58);
    AD9822(1,0xA0);
    AD9822(6,0);
    //???????? ??????????????? ?????. ??? ?? ????????????? ???
    AD9822(3,34);
    adress=0;
    HC595(0xCF);
    ftStatus=FT_Write(CAM8B, FT_Out_Buffer, adress,&bytesWritten);
    if ( ftStatus!= FT_OK) fprintf(stderr,"write failed on channel 2 (%d)\n",ftStatus);
    fprintf(stdout,"cameraConnect W\n");
    prtbuf(FT_Out_Buffer,adress);

  isConnected = FT_OP_flag;
  errorReadFlag = false;
  cameraState = cameraIdle;
  if(FT_OP_flag==false)  cameraState = cameraError;
  printf("cameraConnect END\n");

  return isConnected;
}



/*Disconnect camera, return bool result*/
bool cameraDisconnect(void)               /*stdcall; export;*/
{
printf("cameraDisconnect\n");
    bool FT_OP_flag;
  /*void cameraDisconnect_result;
  FT_OP_flag = true;
  if (Close_USB_Device(FT_CAM8A) != FT_OK)  FT_OP_flag = false;
  if (Close_USB_Device(FT_CAM8B) != FT_OK)  FT_OP_flag = false;
  isConnected = ! FT_OP_flag;
  Result= FT_OP_flag;
  return cameraDisconnect_result;*/

    //FT_SetBitMode(CAM8B,0,FT_BITMODE_RESET);
    FT_Close(CAM8A);
    FT_Close(CAM8B);

    printf("cameraDisconnect END\n");

	return true;
   
}

/*Check camera connection, return bool result*/
bool cameraIsConnected()               /*stdcall; export;*/
{
  return isConnected;
}

int cameraStartExposure (int Bin,int StartX,int StartY,int NumX,int NumY, double Duration, bool light) /*stdcall; export;*/
{
//printf("cameraStartExposure\n");
  durat=Duration;
  printf("cameraStartExposure bin=%d dur=%f light=%d\n",Bin,durat,light);
  canStopExposureNow = false;
  errorReadFlag = false;
  mBin = Bin;
  if (light)  zatv=0;
  else zatv=1;
  if ((NumY+StartY > CameraHeight)||(StartY < 0)||(NumY <= 0)) 
  {
    mYn=0;
    mdeltY=yccd;
  }
  else {
    mYn=StartY / 2;
    mdeltY=NumY / 2;
  }
  if ((NumX+StartX > CameraWidth)||(StartX < 0)||(NumX <= 0)) 
  {
    mXn=0;
    mdeltX=xccd;
  }
  else {
    mXn=StartX / 2;
    mdeltX=NumX / 2;
  }
  imageReady = false;
  cameraState = cameraExposing;
  if (Duration > 0.499) 
  {
    adress=0;
    shift3();
    //if (! errorWriteFlag)  errorWriteFlag = Write_USB_Device_Buffer_wErr(FT_CAM8B,FT_Out_Buffer,adress);
    if (FT_Write(CAM8B, FT_Out_Buffer, adress,&bytesWritten) != 0) fprintf(stderr,"write failed on channel 2)\n");
    //fprintf(stdout,"FT_Write cameraStartExposure %d/%d\n",adress,bytesWritten);
    fprintf(stdout,"cameraStartExposure W\n");
    prtbuf(FT_Out_Buffer,adress);

    //ExposureTimer = addTimer (0,0,round(Duration*1000-52),&ExposureTimerTick);
    //Timer15V = addTimer (0,0,1000,&Timer15VTick);
    //ExposureTimer = UtilTimer_AddTimer((void *) (1000-52,0,0,*ExposureTimerTick));
    //Timer15V = UtilTimer_AddTimer((void *) (1000,0,0,*Timer15VTick));
    pthread_t te,tt;
    pthread_create(&te, NULL, ExposureTimerTick, NULL);
    pthread_create(&tt, NULL, Timer15VTick, NULL);

  }
  else {
    //readframe (mBin,round(Duration*1000));
//printf("cameraStartExposure pas timers\n");
     uint32_t dd;
     dd = Duration*1000;
    readframe (mBin,dd);
  }
  printf("cameraStartExposure END\n");

  return rfstatus;

}

/*Stop camera exposure when it is possible*/
bool cameraStopExposure()               /*stdcall; export;*/
{
//printf("cameraStopExposure\n");
  if (canStopExposureNow)  StopExposure();
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
//printf("cameraGetCameraState\n");
  int Result;
  if (! errorWriteFlag)  Result = cameraState;
  else Result = cameraError;
  return Result;
}

/*Check ImageReady flag, is image ready for transfer - transfer image to driver and return bool ImageReady flag*/
bool cameraGetImageReady()               /*stdcall; export;*/
{
//printf("cameraGetImageReady\n");
  return imageReady;
}


uint16_t cameraGetImage(int i,int j)
{
//printf("cameraGetImage\n");
/*for ( int i = 0 ; i < 100 ;  i ++)
      for( int j = 0 ; j<100 ; j ++)
          bb[i][j]=bufim[i][j];*/

  cameraState=cameraDownload;
  cameraState=cameraIdle;
  //return bufim[i][j];
  //return rand() % 65535;
  //return 0;
  //if ((i<100) &(j<100 )) return 0;
  //if ((i>2900)&(j>1900)) return 65535;
  //if ((i<100) &(j>1900)) return 65535;
  //if ((i>2900)&(j<100 )) return 65535;
  return bufim[i][j];
}

void cameraGetImage2(void *buff)
{
buff = bufim;
}
/*Set camera gain, return bool result*/
bool cameraSetGain (int val)                   /*stdcall; export;*/
{
printf("cameraSetGain\n");
  AD9822(3,val);
  return true;
  printf("cameraSetGain END\n");
}

/*Set camera offset, return bool result*/
bool cameraSetOffset (int val)                   /*stdcall; export;*/
{
printf("cameraSetOffset\n");
  int x;
  x=abs(2*val);
  if (val < 0)  x=x+256;
  AD9822(6,x);
  //AD9822(6,val);
  printf("cameraSetOffset END\n");

  return true;
}

/*Get camera error state, return bool result*/
int cameraGetError()           /*stdcall; export;*/
{
//printf("cameraGetError\n");
  int res;

  res=0;
  if (errorWriteFlag)  res =res+2;
  if (errorReadFlag)  res =res+1;
  return res;
}

/*Set camera baudrate, return bool result*/
bool cameraSetBaudrate(int val)                   /*stdcall; export;*/
{
printf("cameraSetBaudrate\n");
bool Result;
  /*setup FT2232 baud rate*/
  if ((val>=80) & (val<=240))
  {
    spusb = val*10000;
    ms1 = spusb / 8000;
    FT_Current_Baud = spusb;
    if (FT_SetBaudRate(CAM8B ,FT_Current_Baud) != 0) {fprintf(stderr, "ftdi_set_baudrate 2 failed\n");exit(-1);}
    Result = true;
  }
  else Result = false;
printf("cameraSetBaudrate END\n");
  return Result;
}

uint16_t swap(uint16_t x)
{
        uint16_t ret;
        //fprintf(stdout,"swap %d\n",x);
        uint8_t hibyte = (x & 0xFF00) >> 8;
        uint8_t lobyte = (x & 0x00FF);
        ret=  (lobyte << 8) | hibyte; // originale ?
        //ret = hibyte << 8 | lobyte;
        return ret;
}
