// SPDX-License-Identifier: CC-BY-NC-ND-4.0
// Copyright (c) Javadian. All rights reserved.

#include "mex.h"
#include <stdio.h>
#include <Windows.h>
#include <ole2.h>
#include <NuiApi.h>
#include <matrix.h>
#include <iostream>
#include <math.h>

#define ColorImageHeight 480
#define ColorImageWidth  640
#define DepthImageHeight 240
#define DepthImageWidth  320

//bool __stdcall Nui_GotDepthAlert(double  Depthes[][imageWidth]);
bool __stdcall Nui_GotDepthAlert(double Depthes[][DepthImageWidth],double Players[][DepthImageWidth]);
bool __stdcall Nui_GotVideoAlert(double  RGBinfo[][ColorImageWidth][3]);
bool __stdcall Nui_GotSkeletonAlert(double JointX[], double JointY[], double JointZ[]);
void __stdcall KinectInit();
void __stdcall KinectShutdown();

HRESULT	hr;
HANDLE m_pDepthStreamHandle = NULL;
HANDLE m_pVideoStreamHandle;
HANDLE m_hNextSkeletonEvent = NULL;
const NUI_IMAGE_FRAME * pImageFrame = NULL;

int main(){}

void __stdcall KinectInit()
{
	//hr=NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	hr=NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON |NUI_INITIALIZE_FLAG_USES_COLOR);
	

	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,
         NUI_IMAGE_RESOLUTION_640x480, 0, 2, NULL, &m_pVideoStreamHandle );

	//hr = NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH,
	//	 NUI_IMAGE_RESOLUTION_640x480, 0, 2, NULL, &m_pDepthStreamHandle);

	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
		 NUI_IMAGE_RESOLUTION_320x240, 0, 2, NULL, &m_pDepthStreamHandle);

	hr = NuiSkeletonTrackingEnable( m_hNextSkeletonEvent, 0 );

	//for now we've ignored all the errors here, needs finishing later
	
}

void __stdcall KinectShutdown()
{
	NuiShutdown();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////  DEPTH  ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
bool __stdcall Nui_GotDepthAlert(double Depthes[][DepthImageWidth],double Players[][DepthImageWidth])
{
	//public float NominalHorizontalFieldOfView { get; }
	//public float NominalVerticalFieldOfView { get; }
	//balad nistam in 2ta ro kar bendazam!

    const NUI_IMAGE_FRAME * pImageFrame = NULL;

    HRESULT hr = NuiImageStreamGetNextFrame(
        m_pDepthStreamHandle,	
        0,///////////////////////////was 66, I changed it to 0
        &pImageFrame );

    if( FAILED( hr ) )
		return 1;/////////////difference

    //NuiImageBuffer * pTexture = pImageFrame->pFrameTexture;
	INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;

    //KINECT_LOCKED_RECT LockedRect;
	NUI_LOCKED_RECT LockedRect;

    pTexture->LockRect( 0, &LockedRect, NULL, 0 );

    if( LockedRect.Pitch != 0 )//////////////////here's the equivalent of the DRAW [function] from the other one
    {
        //BYTE * pBuffer = (BYTE*) LockedRect.pBits;
        //USHORT * pBufferRun = (USHORT*) pBuffer;
		USHORT * pBufferRun = (USHORT *)LockedRect.pBits; ///////////added

		//double RealDepth;
		USHORT RealDepth;
		USHORT Player;

		for( int y = 0 ; y < DepthImageHeight ; y++ )
        {	
            for( int x = 0 ; x < DepthImageWidth ; x++ )
            {
                pBufferRun++;
				//RealDepth = (*pBufferRun  & 0x0fff);
				RealDepth = NuiDepthPixelToDepth(*pBufferRun);
				//Player = NuiDepthPixelToPlayerIndex(*pBufferRun);
				char c = *(pBufferRun+1);
				Player = c & 0x07;
				Depthes[y][x] = RealDepth;
				Players[y][x] = Player;
            }
        }	
    }
    else
    {
	  //OutputDebugString( L"Buffer length of received texture is bogus\r\n" );
    }

	pTexture->UnlockRect( 0 );/////////////////added

    NuiImageStreamReleaseFrame( m_pDepthStreamHandle, pImageFrame );

	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////  COLOR  ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
bool __stdcall Nui_GotVideoAlert(double RGBinfo[][ColorImageWidth][3])
{
    const NUI_IMAGE_FRAME * pImageFrame = NULL;

    HRESULT hr = NuiImageStreamGetNextFrame(
        m_pVideoStreamHandle,
        66,
        &pImageFrame );

    if( FAILED( hr ) )
        return 1;/////////////difference

    //NuiImageBuffer * pTexture = pImageFrame->pFrameTexture;
	INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;

    //KINECT_LOCKED_RECT LockedRect;
	NUI_LOCKED_RECT LockedRect;

    pTexture->LockRect( 0, &LockedRect, NULL, 0 );

    if( LockedRect.Pitch != 0 )
    {
        BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		int base, b, a= 0;
		for (int j= 0;j< ColorImageHeight*4;j+= 4){
			b= 0;
			base = j*ColorImageWidth;
			for(int i= 0;i< (ColorImageWidth*4);i+= 4)
			{
				RGBinfo[a][b][0]= (double)pBuffer[base+i+2]; //R
				RGBinfo[a][b][1]= (double)pBuffer[base+i+1]; //G
				RGBinfo[a][b][2]= (double)pBuffer[base+i+0]; //B
				b++;
			}
			a++;
		}
    }
	else
    {
		//OutputDebugString( L"Buffer length of received texture is bogus\r\n" ); 
    }

    NuiImageStreamReleaseFrame( m_pVideoStreamHandle, pImageFrame );
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////  SKELETON  ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
bool __stdcall Nui_GotSkeletonAlert(double JointX[], double JointY[], double JointZ[])
{
	NUI_SKELETON_FRAME SkeletonFrame = {0};

	HRESULT hr = NuiSkeletonGetNextFrame( 0, &SkeletonFrame );
	if( FAILED( hr ) )
    {
		//if (hr == E_POINTER)
		//	{ Joints[0][0]=19;
		//	}
		//else {
		//	Joints[0][0]=20;
		//	}
		
		printf("No Skeleton Initialization!",(char)hr); 
        return 1;
    }

	//find out whether they are tracking 
	bool bFoundSkeleton = false;
    for ( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
    {
		// Found atleast one tracked skeleton
		if( (SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED) || 
			(SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_POSITION_ONLY))
        {
             bFoundSkeleton = true;
        }
    }

    // no skeletons!
    if( !bFoundSkeleton )
    {
		printf("No skeletons\n"); 
		return 1;
    }

	// smooth out the skeleton data
    hr = NuiTransformSmooth( &SkeletonFrame, NULL );
    if ( FAILED(hr) )
    {
		return 1;
    }

	//Sleep(5*100); //wait for 5 seconds

  //  // we found a skeleton, re-start the skeletal timer
  //   m_bScreenBlanked = false;
  //   m_LastSkeletonFoundTime = timeGetTime( );

	// extract tracked skeletons
	int jointindex = 0;
	//bool bSkeletonIdsChanged = false;
    for ( int i = 0 ; i < NUI_SKELETON_COUNT; i++ )
    {
		for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
		{
			long DepthX, DepthY;
			USHORT DepthZ;
			//Joints[0][jointindex] = (double) SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].x; 
			//Joints[1][jointindex] = (double) SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].y;
			//Joints[2][jointindex] = (double) SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].z;

		    NuiTransformSkeletonToDepthImage( SkeletonFrame.SkeletonData[i].SkeletonPositions[j], 
											  &DepthX,	/*x position in 320 pixel scale*/
											  &DepthY,  /*y position in 240 pixel scale*/  
											  &DepthZ); /*z position or depth of the joint*/ 
			JointX[jointindex] = (double) DepthX; 
			JointY[jointindex] = (double) DepthY;
			JointZ[jointindex] = (double) DepthZ;
			jointindex++;
		}

    }

	return 0;
}


////////////////////////////////////////// MEX FUNCTION //////////////////////////////
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{	
	    if(nlhs!=6)
		{
			mexErrMsgIdAndTxt("Getimagedata:nlhs","One output required.");
		}
		double *Dep, *Rgb, *Ply, *JnX, *JnY, *JnZ, *Option;
		int dims[3] = {ColorImageHeight, ColorImageWidth, 3};

		/* Create MATLAB input matrices */
		plhs[0] = mxCreateDoubleMatrix(DepthImageHeight, DepthImageWidth, mxREAL); 
		plhs[1] = mxCreateDoubleMatrix(DepthImageHeight, DepthImageWidth, mxREAL);
		plhs[2] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
		plhs[3] = mxCreateDoubleMatrix(1, NUI_SKELETON_COUNT * 20, mxREAL); 
		plhs[4] = mxCreateDoubleMatrix(1, NUI_SKELETON_COUNT * 20, mxREAL);
		plhs[5] = mxCreateDoubleMatrix(1, NUI_SKELETON_COUNT * 20, mxREAL);

		Dep = mxGetPr(plhs[0]);
		Ply = mxGetPr(plhs[1]);
		Rgb = mxGetPr(plhs[2]);
		JnX = mxGetPr(plhs[3]);
		JnY = mxGetPr(plhs[4]);
		JnZ = mxGetPr(plhs[5]);

		Option = mxGetPr(prhs[0]);

		double (*Depxy)[DepthImageWidth] = (double (*)[DepthImageWidth])Dep;
		double (*Plyxy)[DepthImageWidth] = (double (*)[DepthImageWidth])Ply;
		double (*Rgbxyz)[ColorImageWidth][3] = (double (*)[ColorImageWidth][3])Rgb;

		double (*JntXx) = (double (*))JnX;
		double (*JntYy) = (double (*))JnY;
		double (*JntZz) = (double (*))JnZ;

		switch ((int)*Option)
		{
			case 1 :
				KinectInit();			
				mexLock();
				break;
			case 2 :
				Nui_GotDepthAlert(Depxy,Plyxy);
				Nui_GotVideoAlert(Rgbxyz);
				Nui_GotSkeletonAlert(JntXx,JntYy,JntZz);
				break;
			case 3:
				KinectShutdown();
				mexUnlock();

				break;
			default:
				break;
		}
}