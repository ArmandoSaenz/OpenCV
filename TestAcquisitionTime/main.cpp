//******************************************************************************************************
//Author: Jose Armando Saenz Esqueda
//Date: March 8, 2021
//OpencvVersion: 4.1.6
//Description:  This program is a test for take a photo with a camera Grasshopper from Flir.
//              The image is adquire with Spinnaker library and it is showed with OpenCV.
//******************************************************************************************************
//Headers of Spinnaker SDK
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
//Headers of OPENCV4
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp> //Requiered for function "moments(Mat,bool)"
#include <opencv2/core/types.hpp>//Required for class "Point"
//***************************************************************************
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <time.h>
#include <fcntl.h>
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <math.h>

//***************************************************************************
//Namespace for Spinnaker
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
//Namespace for OpenCV4
using namespace cv;
//Namepace for cout and cin
using namespace std;

//***************************************************************************
//constants
//***************************************************************************
LibraryVersion spinnakerLibraryVersion; //This command obtains the version of the used SDk
const gcstring acquisitionMode = "Continuous";
//***************************************************************************
//Public variables
//***************************************************************************
int iWidth = 640; //Image width
int iHeight = 600;  //Image height
int cOffsetx = 800; //Camera x-axis offset
int cOffsety = 200; //Camera y-axis offset
uint64_t idImage;
unsigned int XPadding; 
unsigned int YPadding; 
unsigned int rowsize;
unsigned int colsize;
CameraList camList; //This object stores the list of detected cameras
CameraPtr pCam = nullptr; //This object stores the camera direction
FeatureList_t features;  //This object stores the camera features
double dt;
chrono::duration<double> elapsed;
auto start = chrono::high_resolution_clock::now();
auto finish = chrono::high_resolution_clock::now();
int frameNum = 1;
int frameSkip = 20;
//***************************************************************************
//Methods
//***************************************************************************
SystemPtr spinnakerSDK;
int main()//Main method
{
    //***************************************************************************
    // Local variables
    //***************************************************************************
    int numCamera; //This variable stores the number of detected cameras
    ImagePtr frame; //This object store the image converted to OpenCV format

    //***************************************************************************
    //Process
    //***************************************************************************
    spinnakerSDK = System::GetInstance();  //Create the instance for SDK      
    spinnakerLibraryVersion = spinnakerSDK->GetLibraryVersion();
    //The SDK version is printed on screen
    cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
        << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
        << endl;

    camList = spinnakerSDK->GetCameras(); //Get the cameras list
    const unsigned int numCameras = camList.GetSize(); //Get the list length
    cout << "Number of cameras detected: " << numCameras << endl << endl; //Print on screen the number of cameras list
    
    // Finish if there are no cameras
    if (numCameras == 0)
    {
        camList.Clear(); //Clear camera list before releasing spinnakerSDK
        spinnakerSDK->ReleaseInstance(); //Release spinnakerSDK
        cout << "Not enough cameras!. " << "Press Enter to exit..." << endl << endl; //Message to finish the app
        getchar(); //Wait a press key
        return -1; //Finish the app
    }

    //--------------------------------------------------------------------
    //Algorithm to print on screen the cameras list
    //--------------------------------------------------------------------
    printf("|%5s|%30s|%30s|%15s|\n\r","#","ID","CAMERA","SERIAL NUMBER"); 
    for(int i = 0; i < numCameras; i++)
    {
        pCam = camList.GetByIndex(i); //Get the camera instance
        CStringPtr serialcam = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber"); //Get the camera serial number
        CStringPtr namecam = pCam->GetTLDeviceNodeMap().GetNode("DeviceModelName"); //Get the camera model name
        CStringPtr idcam = pCam->GetTLDeviceNodeMap().GetNode("DeviceID"); //Get the camera ID
        printf("|%5d|%30s|%30s|%15s|\n\r",i,idcam->GetValue().c_str(),namecam->GetValue().c_str(),serialcam->GetValue().c_str()); //Print on screen the camera information
    }
    //--------------------------------------------------------------------
    //Algorithm to select camera
    //--------------------------------------------------------------------
    printf("Type the number of camera tu use: "); //Print indication
    scanf("%d",&numCamera); //Read the option
    pCam = camList.GetByIndex(numCamera); //Get the instance of selected camera
    printf("Image acquisition is initializing.\n\r"); //Print on screeen message
    //INodeMap& nodeMap = pCam->GetNodeMap();

    //--------------------------------------------------------------------
    //Algorithm to stablish the camera feratures
    //--------------------------------------------------------------------
    pCam->Init(); //Stablish the camera connection
    pCam->Width.SetValue(640); //Set the image width
    pCam->Height.SetValue(600); //Set the image height
    pCam->OffsetX.SetValue(800); //Set the offset on x-axis
    pCam->OffsetY.SetValue(200); //Set the offset on y-axis
    //Print on screen ths camera features
    printf("|%10s|%10s|%10s|%10s|%10s|\n\r","WIDTH","HEIGHT","OFFSETX","OFFSETY","FPS"); 
    printf("|%10d|%10d|%10d|%10d|%10.2f|\n\r",(int)pCam->Width.GetValue(),(int)pCam->Height.GetValue(),(int)pCam->OffsetX.GetValue(),(int)pCam->OffsetY.GetValue(),pCam->AcquisitionFrameRate.GetValue());

    //---------------------------------------------------------------------
    //Algorithm to check if the camera can read and write the camera features
    //---------------------------------------------------------------------
    INodeMap& nodeMap = pCam->GetNodeMap();
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
        return -1;
    }
    else
    {
        cout << "Acquisition mode is possible." << endl;
    }

    //---------------------------------------------------------------------
    //Algorithm to check if the camera acquires video
    //---------------------------------------------------------------------
    CEnumEntryPtr ptrAcquisitionModeSet = ptrAcquisitionMode->GetEntryByName(acquisitionMode);
    if (!IsAvailable(ptrAcquisitionModeSet) || !IsReadable(ptrAcquisitionModeSet))
    {
        cout << "Unable to set acquisition mode to " << acquisitionMode <<"(entry retrieval). Aborting..." << endl << endl;
        return -1;
    }
    else
    {
        cout << "Aquisition mode "<< acquisitionMode  << " is enable." << endl;
    }
    
    const int64_t idAcquisitionMode = ptrAcquisitionModeSet->GetValue(); // Retrieve integer value from entry node
    ptrAcquisitionMode->SetIntValue(idAcquisitionMode);// Set integer value from entry node as new value of enumeration node
    cout << "Acquisition mode set to "<< acquisitionMode <<"..." << endl;
    cout << "Key press to star image acquisition" << endl;
    pCam->BeginAcquisition();
    cout << "Image acquisition started" << endl;
    namedWindow("Camera");//Open OpenCv window
    time_t t = std::time(0);   // get time now
    tm* now = std::localtime(&t);
    char title[100];
	sprintf(title, "Experiment_%4d_%2d_%2d_%2d_%2d_%2d.txt",now->tm_year + 1900,now->tm_mon + 1,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
	FILE* datos;
	datos = fopen(title,"w+");
    fprintf(datos,"dt=[",dt);
    for(int i = 0;i<2000000000;i++)
    {
        start = chrono::high_resolution_clock::now();
        frame = pCam->GetNextImage(); //Get the actual frame
        XPadding = frame->GetXPadding(); //Horizontal padding
        YPadding = frame->GetYPadding(); //Vertical padding
        rowsize = frame->GetWidth(); //Image width
        colsize = frame->GetHeight(); //Image height
        Mat cvimg = Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, frame->GetData(), frame->GetStride()); //Convert monocromatic image from spinnaker to OpenCV
        frame->Release(); //Release Spinnaker image
        if(i%frameSkip==0)
            imshow("Camera",cvimg);
        if(waitKey(1)>=0)
            break;
        elapsed = finish - start;
        dt = elapsed.count()*1000;
        finish = chrono::high_resolution_clock::now();
        printf("Delay %d:%f \r\n",i,dt);
        fprintf(datos,"%f \r\n",dt);
    }
    fprintf(datos,"];",dt);
    //---------------------------------------------------------------------
    //Algorithm to close connection with the camera
    //---------------------------------------------------------------------
    fclose(datos);
    pCam->EndAcquisition(); //Finish acquisition
    pCam = nullptr; //Release pointer
    camList.Clear(); //Clear camera list before releasing system
    spinnakerSDK->ReleaseInstance(); //Release system
    return 0;
}