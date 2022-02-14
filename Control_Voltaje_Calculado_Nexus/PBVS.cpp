//******************************************************************************************************
//Author: Jose Armando Saenz Esqueda
//Date: Novembre 26, 2019
//OpencvVersion: 4.1.6
//Description:  This program is a test for take a photo with a camera Grasshopper from Flir.
//              The image is adquire with Spinnaker library and it is showed with OpenCV.
//******************************************************************************************************
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp> //Requiered for function "moments(Mat,bool)"
#include <opencv2/core/types.hpp>//Required for class "Point"
#include <chrono>
#include <time.h>
#include <fcntl.h>
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <math.h>



using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;
int posX1, posY1, posX2, posY2;
Rect roi1; //Tne CV namespace is required
Rect roi2; //Tne CV namespace is required
Point p; //Tne CV namespace is required
int nFeature=1;
float ox,oy;
float xi[] = {0,0,0}, xia[] = {0,0,0}, xip[] = {0,0,0};
float u[] = {0,0,0,0}, upd[]={0,0,0};
float dt;
float xt, xtp, yt, ytp, tht, thtp;
float t0=0, t1=0;
float PI=3.14159265;
int revol = 0;
bool fLoop = true;
chrono::duration<double> elapsed;
char write_buffer[] = "LMCZDZRZRZD"; //This word is sended to stop the robot
//Sistema de Vision
float alpha=125000;
float lambda=0.008;//"0.008
float u0=1080;
float v0=413;
float OCR3=2.24;

float vol[246] = {1.4505,1.9949,2.4965,2.9592,3.3866,3.7817,4.1473,4.4858,4.7995,5.0904,5.3603,5.6108,5.8436,6.06,6.2613,6.4487,6.6232,6.7858,6.9374,7.0789,7.211,7.3345,7.4499,7.558,7.6592,7.754,7.8429,7.9264,8.0049,8.0787,8.1481,8.2136,8.2753,8.3335,8.3886,8.4406,8.4899,8.5366,8.581,8.6231,8.6631,8.7012,8.7375,8.7722,8.8053,8.8369,8.8672,8.8962,8.9241,8.9508,8.9765,9.0013,9.0251,9.0482,9.0704,9.0918,9.1126,9.1328,9.1523,9.1712,9.1896,9.2075,9.2249,9.2418,9.2583,9.2744,9.2902,9.3055,9.3205,9.3352,9.3496,9.3637,9.3775,9.391,9.4043,9.4173,9.4302,9.4427,9.4551,9.4673,9.4793,9.4911,9.5027,9.5141,9.5254,9.5365,9.5475,9.5583,9.569,9.5796,9.59,9.6003,9.6104,9.6205,9.6304,9.6402,9.6499,9.6595,9.669,9.6784,9.6877,9.6969,9.706,9.715,9.7239,9.7328,9.7416,9.7502,9.7588,9.7673,9.7758,9.7842,9.7924,9.8007,9.8088,9.8169,9.8249,9.8329,9.8407,9.8486,9.8563,9.864,9.8716,9.8792,9.8867,9.8942,9.9016,9.9089,9.9162,9.9235,9.9306,9.9378,9.9448,9.9519,9.9589,9.9658,9.9727,9.9795,9.9863,9.993,9.9997,10.0064,10.013,10.0195,10.0261,10.0325,10.039,10.0454,10.0517,10.058,10.0643,10.0705,10.0767,10.0829,10.089,10.0951,10.1011,10.1071,10.1131,10.1191,10.125,10.1308,10.1367,10.1425,10.1482,10.1539,10.1596,10.1653,10.1709,10.1766,10.1821,10.1877,10.1932,10.1987,10.2041,10.2095,10.2149,10.2203,10.2256,10.2309,10.2362,10.2414,10.2467,10.2519,10.257,10.2622,10.2673,10.2724,10.2774,10.2825,10.2875,10.2925,10.2974,10.3024,10.3073,10.3122,10.317,10.3219,10.3267,10.3315,10.3363,10.341,10.3458,10.3505,10.3551,10.3598,10.3645,10.3691,10.3737,10.3782,10.3828,10.3873,10.3919,10.3963,10.4008,10.4053,10.4097,10.4141,10.4185,10.4229,10.4273,10.4316,10.4359,10.4402,10.4445,10.4488,10.453,10.4572,10.4614,10.4656,10.4698,10.474,10.4781,10.4822,10.4863,10.4904,10.4945,10.4986,10.5026,10.5066,10.5106,10.5146,10.5186,10.5226,10.5265,10.5304};

int pwm[246] = {10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255};

float mR = 2.8;
float mR1 = 0.38;
float IRz = 0.060848;
float IRy1 = 0.000324;
float IRz1 = 0.000469;
float l1 = 0.1524;
float l2 = 0.1505;
float r = 0.42;
float Jm = 0.00000057;
float kb = 0.01336;
float Ka = 0.0134;
float Ra = 1.9;
float kv = 0.001;
float re = 64;
float l;
float uvx, uvy, uvth, taux, tauy, tauth;
float lambdapx = 20.0;
float lambdapy = 20.0;
float lambdapth =15.0;
float lambdavx = 5.0;
float lambdavy = 5.0;
float lambdavth = 5.0;
float xd, xpd, xppd, yd, ypd, yppd, thd, thpd, thppd;

//Differentiator variables
float xig[3] = {0.0,0.0,0.0};
float xigt[3] = {0.0,0.0,0.0};
float xigp[3] = {0.0,0.0,0.0};

float ipsilong[3] = {0.0,0.0,0.0};
float ipsilongp[3] = {0.0,0.0,0.0};

//Differentiator gains
float kxg = 8;
float kyg = 8;
float kthg = 5;
float kxgp = 20;
float kygp = 20;
float kthgp = 20;

float sign (float dato)
{
	if(dato > 0.0)
		return 1.0;
		else
		{
			if(dato < 0.0)
				return -1.0;
			else
			 	return 0.0;
		}
}
char vol2pwm(float voltaje)
{
	int j;
	voltaje = fabs(voltaje);
	if(voltaje<=vol[0])
		return 1;
	if(voltaje>=vol[246])
		return 255;
	for(j=1;j<244;j++)
	{
		if(voltaje<=vol[j])
			return (char)pwm[j];
	}
  return 0;
}
//METHOD TO DETECT THE LEFT CLICK
void ClickEvent(int event, int x, int y, int flags, void* userdata)
{
  //This rutine saves the mouse clicked position
  if(event == EVENT_LBUTTONDOWN)
  {
    switch(nFeature)
    {
      case 1:
        posX1 = x;
        posY1 = y;
        break;
      case 2:
        posX2 = x;
        posY2 = y;
        break;
    }
    nFeature++;
  }
}

int main()
{
  // VARIABLES
  SystemPtr system = System::GetInstance();
  FeatureList_t features;
  CameraPtr pCam = nullptr;
  int numCamera;
  bool fin = true;
  int min_val = 0;
  unsigned int XPadding, YPadding, rowsize, colsize;
  ImagePtr frame;
  ImagePtr convertedImage;
  // Print out current library version
  const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
  cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
       << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
       << endl;

  // Retrieve list of cameras from the system
  CameraList camList = system->GetCameras();
  const unsigned int numCameras = camList.GetSize();

  cout << "Number of cameras detected: " << numCameras << endl << endl;

  // Finish if there are no cameras
  if (numCameras == 0)
  {
      // Clear camera list before releasing system
      camList.Clear();

      // Release system
      system->ReleaseInstance();

      cout << "Not enough cameras!" << endl;
      cout << "Done! Press Enter to exit..." << endl;
      getchar();

      return -1;
  }

  printf("|%5s|%30s|%30s|%15s|\n\r","#","ID","CAMERA","SERIAL NUMBER");
  for(int i = 0; i < numCameras; i++)
  {
    pCam = camList.GetByIndex(i);
    CStringPtr serialcam = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
    CStringPtr namecam = pCam->GetTLDeviceNodeMap().GetNode("DeviceModelName");
    CStringPtr idcam = pCam->GetTLDeviceNodeMap().GetNode("DeviceModelName");
    printf("|%5d|%30s|%30s|%15s|\n\r",i,idcam->GetValue().c_str(),namecam->GetValue().c_str(),serialcam->GetValue().c_str());
  }
  //Selecting camera
  printf("Type the number of camera tu use: ");
  scanf("%d",&numCamera);
  pCam = camList.GetByIndex(numCamera);
  getchar();
  printf("Image acquisition is initializing.\n\r");
  pCam->Init();
  INodeMap& nodeMap = pCam->GetNodeMap();
  pCam->Width.SetValue(640);
  pCam->Height.SetValue(600);
  pCam->OffsetX.SetValue(800);
  pCam->OffsetY.SetValue(200);
  printf("|%10s|%10s|%10s|%10s|%10s|\n\r","WIDTH","HEIGHT","OFFSETX","OFFSETY","FPS");
  printf("|%10d|%10d|%10d|%10d|%10.2f|\n\r",(int)pCam->Width.GetValue(),(int)pCam->Height.GetValue(),(int)pCam->OffsetX.GetValue(),(int)pCam->OffsetY.GetValue(),pCam->AcquisitionFrameRate.GetValue());
  ox = (int)pCam->Width.GetValue()/2;
  oy = (int)pCam->Height.GetValue()/2;
  getchar();
  CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
  if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
  {
      cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
      return -1;
  }
  else
  {
    cout << "Acquisition mode is enable." << endl;
  }

  // Retrieve entry node from enumeration node
  CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
  if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
  {
      cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
      return -1;
  }
  else
  {
    cout << "Contiuos acquisition mode is enable." << endl;
  }

  // Retrieve integer value from entry node
  const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

  // Set integer value from entry node as new value of enumeration node
  ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

  cout << "Acquisition mode set to continuous..." << endl;
  pCam->BeginAcquisition();
  //printf("Image acquisition is intialized\n\n\r");
  //printf("The image is being retrieved\n\r");
  //OPEN SERIAL PORT
  printf("The serial port is being opened\r\n");
  int fd = open("/dev/ttyS0",O_RDWR | O_NOCTTY | O_NDELAY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
            /* O_RDWR Read/Write access to serial port           */
          /* O_NOCTTY - No terminal will control the process   */
          /* O_NDELAY -Non Blocking Mode,Does not care about-  */
          /* -the status of DCD line,Open() returns immediatly */
  if(fd == -1)						/* Error Checking */
  {
       printf("\n  Error! in Opening ttyS0  ");
  }
  else
  {
         printf("\n  Serial port opened successfully ");
  }

  //The Baudrate is set to 115200
  printf("The baudrate is set to 115200 bps");
  struct termios SerialPortSettings;	/* Create the structure                          */
  cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
  cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */

	time_t t = std::time(0);   // get time now
  tm* now = std::localtime(&t);
	printf("Create file to save values");
  char title[100];
	sprintf(title, "experimentos/Experiment_%4d_%2d_%2d_%2d_%2d_%2d.txt",now->tm_year + 1900,now->tm_mon + 1,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
	FILE* datos;
	datos = fopen(title,"w+");
	namedWindow("current Image 1");
	auto start = chrono::high_resolution_clock::now();
	//Vision
	float gamma = (alpha*lambda)/(OCR3-lambda);
	//Parametros dinámicos
	l = l1+l2;
	float m11 = mR + 4*mR1 + (IRy1 + Jm*re*re)*4/(r*r);
	float m33 = 4*mR1*(l1*l1 + l2*l2) + IRz + 4*IRz1 + (IRy1+Jm*re*re)*4*l*l/(r*r);

	float c12 = 4*(IRy1 + Jm*re*re)/(r*r);

	float d11 = re*re*(Ka*kb/Ra +kv)*4/(r*r);
	float d33 = re*re*(Ka*kb/Ra +kv)*4*l*l/(r*r);

	//fprintf(datos,"%% lambdapx=%5.3f;lambdapy=%5.3f;lambdapth=%5.3f;lambdavx=%5.3f;lambdavth=%5.3f;kxg=%6.3f;kyg=%6.3f;kthg=%6.3f;kxgp=%4.3f;kygp=%4.3f;kthgp=%4.3f;\r\n",lambdapx,lambdapy, lambdapth,lambdavx,lambdavy,lambdavth,kxg,kyg,kthg,kxgp,kygp,kthgp);
	//fprintf(datos,"datos=[");
	for(int i;fin;i++)
  {
  //Obtiene la imagen
	frame = pCam->GetNextImage();
	frame = pCam->GetNextImage();
	/*
	do {
		auto finish = chrono::high_resolution_clock::now();
	  //Calcula el tiempo tomado
	  elapsed = finish - start;
	  dt =  elapsed.count()-t0;
  } while(dt<0.01);//Hora de termino*/
	auto finish = chrono::high_resolution_clock::now();
	//Calcula el tiempo tomado
	elapsed = finish - start;
	dt =  elapsed.count()-t0;
	t0 = elapsed.count();
	//Espacio sin imagen
  XPadding = frame->GetXPadding();
  YPadding = frame->GetYPadding();
  //Obtiene el resolución de la imagen
  rowsize = frame->GetWidth();
  colsize = frame->GetHeight();
  Mat cvimg = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, frame->GetData(), frame->GetStride());
	frame->Release();
  Mat resultado;
  threshold( cvimg, resultado, min_val, 255,THRESH_BINARY_INV);
  setMouseCallback("current Image 1", ClickEvent, NULL);
  if(nFeature==3)
	{
  t1 = t1 + dt;
  roi1.x = posX1 - 15;
  roi1.y = posY1 - 15;
  roi1.width =  30;
  roi1.height = 30;
  roi2.x = posX2 - 15;
  roi2.y = posY2 - 15;
  roi2.width =  30;
  roi2.height = 30;
  Mat crop1 = resultado(roi1);
  Mat imgcrop1 = cvimg(roi1);
  Mat crop2 = resultado(roi2);
  Mat imgcrop2 = cvimg(roi2);

	if(i%20==0)
	{
	  imshow("current Image 1", imgcrop1);
	  imshow("current Image 2", imgcrop2);
	}

  Moments m1 = moments(crop1,true);
  Moments m2 = moments(crop2,true);
  posX1 = roi1.x + (int)(m1.m10/m1.m00);
  posY1 = roi1.y + (int)(m1.m01/m1.m00);
  posX2 = roi2.x + (int)(m2.m10/m2.m00);
  posY2 = roi2.y + (int)(m2.m01/m2.m00);
  //PREVIOUS POSTURE IS SAVED
  xia[0] = xi[0];
  xia[1] = xi[1];
  xia[2] = xi[2];
  //ACTUAL POSTURE IS ACQUIRED
  xi[0] = ((float)posX1-u0)/gamma;
  xi[1] = -((float)posY1-v0)/gamma;
  xi[2] = revol*2*PI + atan2((float)posY1-(float)posY2, (float)posX2-(float)posX1);
	if(xia[2]-xi[2] > PI)
		revol++;
	if(xia[2]-xi[2] < -PI)
		revol--;
	xi[2] = revol*2*PI+atan2((float)posY1-(float)posY2, (float)posX2-(float)posX1);
  //VELOCITY VECTOR IS COMPUTED
  if(!fLoop)
  {
    xip[0] = (xi[0]-xia[0])/dt;
    xip[1] = (xi[1]-xia[1])/dt;
    xip[2] = (xi[2]-xia[2])/dt;
  }
  else
  {
    fLoop = false;
  }
	//THE TRAYECTORY IS COMPUTED
	xd = 0.5*cos(2.0*PI*t1/10.0);
	yd = 0.5*sin(2.0*PI*t1/10.0);
	thd = 2.0*PI*t1/10.0+PI/2;

	xpd = -PI/10.0*sin(2.0*PI*t1/10.0);
	ypd = PI/10.0*cos(2.0*PI*t1/10.0);
	thpd = 2.0*PI/10.0;

	xppd = -PI*PI/100.0*cos(2.0*PI*t1/10.0);
	yppd = -PI*PI/100.0*sin(2.0*PI*t1/10.0);
	thppd = 0;

	//Differentiator errors are COMPUTED
	xigt[0] = xi[0] - xig[0];
	xigt[1] = xi[1] - xig[1];
	xigt[2] = xi[2] - xig[2];

	//Differentiator dynamics
	xigp[0] = ipsilong[0] + kxg * ( pow(fabs(xigt[0]),1.0/2.0) + 1.0 * pow(fabs(xigt[0]),3.0/2.0)) * sign(xigt[0]);
	xigp[1] = ipsilong[1] + kyg * ( pow(fabs(xigt[1]),1.0/2.0) + 1.0 * pow(fabs(xigt[1]),3.0/2.0)) * sign(xigt[1]);
	xigp[2] = ipsilong[2] + kthg * ( pow(fabs(xigt[2]),1.0/2.0) + 1.0 * pow(fabs(xigt[2]),3.0/2.0)) * sign(xigt[2]);

	ipsilongp[0] = kxgp * (1.0/2.0 + 2.0 * 1.0 * fabs(xigt[0]) + 3.0/2.0 * pow(1.0,2.0) * pow(fabs(xigt[0]),2.0)) * sign(xigt[0]);
	ipsilongp[1] = kygp * (1.0/2.0 + 2.0 * 1.0 * fabs(xigt[1]) + 3.0/2.0 * pow(1.0,2.0) * pow(fabs(xigt[1]),2.0)) * sign(xigt[1]);
	ipsilongp[2] = kthgp * (1.0/2.0 + 2.0 * 1.0 * fabs(xigt[2]) + 3.0/2.0 * pow(1.0,2.0) * pow(fabs(xigt[2]),2.0)) * sign(xigt[2]);

	//The observer states are integrated

	xig[0] = xig[0] + dt*xigp[0];
	xig[1] = xig[1] + dt*xigp[1];
	xig[2] = xig[2] + dt*xigp[2];

	ipsilong[0] = ipsilong[0] + dt*ipsilongp[0];
	ipsilong[1] = ipsilong[1] + dt*ipsilongp[1];
	ipsilong[2] = ipsilong[2] + dt*ipsilongp[2];

	//THEW CONTROL LAW IS COMPUTED
  uvx = xppd + lambdavx * xtp + lambdapx * xt;
	uvy = yppd + lambdavy * ytp + lambdapy * yt;
	uvth = thppd + lambdavth * thtp + lambdapth * tht;

	//THE ERRORS ARE COMPUTED
	xt = xd - xi[0];
	yt = yd - xi[1];
	tht = thd - xi[2];

	//velocity errores are computed with velocity acquired by Euler
  xtp = xpd - xip[0];
	ytp = ypd - xip[1];
	thtp = thpd - xip[2];

	//velocity errores are computed with velocity acquired by HOSM
	/*xtp = xpd - ipsilong[0];
	ytp = ypd - ipsilong[1];
	thtp = thpd - ipsilong[2];*/

  //Control law with velocity computed by Euler
	/*
	taux = m11*uvx - c12*xip[1]*xip[2] + d11 * xip[0];
	tauy = m11*uvy + c12*xip[0]*xip[2] + d11 * xip[1];
	tauth = m33*uvth + d33 * xip[2];*/

	//Control law with velocity computed by HOSM

	taux = m11*uvx;// + d11 * ipsilong[0];// - c12*ipsilong[1]*ipsilong[2];
	tauy = m11*uvy;// + d11 * ipsilong[1];// + c12*ipsilong[0]*ipsilong[2] + d11 * ipsilong[1];
	tauth = m33*uvth; + d33 * xip[2];

	float zeta1 = cos(xi[2]) + sin(xi[2]);
	float zeta2 = cos(xi[2]) - sin(xi[2]);

	u[0] = Ra/(Ka*re)*r*(zeta2*taux + zeta1*tauy + tauth/l)/4;
	u[1] = Ra/(Ka*re)*r*(zeta1*taux - zeta2*tauy - tauth/l)/4;
	u[2] = Ra/(Ka*re)*r*(zeta2*taux + zeta1*tauy - tauth/l)/4;
	u[3] = Ra/(Ka*re)*r*(zeta1*taux - zeta2*tauy + tauth/l)/4;

  /*
	u[0] = 8*r*PI/8 + 8*r*l*PI/4;
	u[1] = 8*r*PI/8 - 8*r*l*PI/4;
	u[2] = 8*r*PI/8 - 8*r*l*PI/4;
	u[3] = 8*r*PI/8 + 8*r*l*PI/4;
  */
  //PWM VALUES|
  write_buffer[3] = vol2pwm(u[0]);
  write_buffer[5] = vol2pwm(u[1]);
  write_buffer[7] = vol2pwm(u[2]);
  write_buffer[9] = vol2pwm(u[3]);
  //DIRECTION
  write_buffer[4] = u[0]<0?'R':'D';
  write_buffer[6] = u[1]<0?'R':'D';
  write_buffer[8] = u[2]<0?'R':'D';
  write_buffer[10] = u[3]<0?'R':'D';
  write(fd,write_buffer,11);
}
else
{
	if(i%20==0)
  imshow("current Image 1", resultado);
}
  //resizeWindow("current Image", rowsize / 2, colsize / 2);
  //resizeWindow("current Image", 200,200);
  switch((char)waitKey(1))
  {
    case 27:
      fin = false;
      break;
      case 'u':
      min_val += 10;
      break;
      case 'd':
      min_val -= 10;
      break;
  }
  min_val = min_val < 0 ? 0:min_val;
  //Muestra el pantalla el tiempo tomado
  min_val = min_val > 255 ? 255:min_val;
  if(nFeature==3)
	{
  	printf("%5.3f;u1=%5.3f; u2=%5.3f; u3=%5.3f;u4=%5.3f;X=%6.2f; Y=%6.2f; Theta=%5.3f\r\n",t1,u[0],u[1],u[2],u[3],xt,yt,tht);
		fprintf(datos,"%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%6.3f,%6.3f,%6.3f,%4.3f,%4.3f,%4.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f;\r\n",t1,u[0],u[1],u[2],u[3],xi[0],xi[1],xi[2],xip[0],xip[1],xip[2],xt,yt,tht,xtp,ytp,thtp,ipsilong[0],ipsilong[1],ipsilong[2],xd,yd,thd,xpd,ypd,thpd,xppd,yppd,thppd);
	}
}
	//fprintf(datos,"];");
	fclose(datos);
  ending:;
  write_buffer[4] = 'P';
  write_buffer[6] = 'P';
  write_buffer[8] = 'P';
  write_buffer[10] = 'P';
  write(fd,write_buffer,11);
  write(fd,write_buffer,11);
  printf("EL ROBOT SE HA DETENIDO. PRESIONAR ENTER PARA SEGUIR");
  getchar();
  close(fd);
  pCam->EndAcquisition();
  pCam = nullptr;
  camList.Clear();
  system->ReleaseInstance();
  printf("Program ended");
  return 0;
}
